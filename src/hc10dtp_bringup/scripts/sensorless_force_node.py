#!/usr/bin/env python3
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

# Thử import PyKDL và kdl_parser_py
try:
    import PyKDL
    from kdl_parser_py.urdf import treeFromString
except ImportError:
    print("LỖI: Không tìm thấy thư viện PyKDL hoặc kdl_parser_py.")
    print("Vui lòng chạy lệnh: sudo apt install ros-humble-kdl-parser-python python3-pykdl")
    sys.exit(1)


class SensorlessForceNode(Node):
    def __init__(self):
        super().__init__('sensorless_force_node')
        
        # Khai báo các tham số
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool0')
        self.declare_parameter('gravity_z', -9.81)
        
        self.base_link = self.get_parameter('base_link').value
        self.tip_link = self.get_parameter('tip_link').value
        gravity_z = self.get_parameter('gravity_z').value
        
        self.kdl_tree = None
        self.kdl_chain = None
        self.jac_solver = None
        self.dyn_param = None
        self.gravity = PyKDL.Vector(0, 0, gravity_z)
        
        # Subscribe vào robot_description để lấy URDF
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_urdf = self.create_subscription(
            String, '/robot_description', self.urdf_callback, qos)
            
        # Publisher cho Wrench ảo
        self.pub_wrench = self.create_publisher(WrenchStamped, '/sensorless_force', 10)
        
        # Sẽ subscribe vào joint_states sau khi có URDF
        self.sub_joint_states = None
        
        # Tên các khớp để map đúng thứ tự của KDL
        self.joint_names = []
        
        self.get_logger().info("Đang chờ /robot_description...")

    def urdf_callback(self, msg: String):
        if self.kdl_tree is not None:
            return  # Đã load xong
            
        urdf_str = msg.data
        success, self.kdl_tree = treeFromString(urdf_str)
        if not success:
            self.get_logger().error("Không thể parse URDF bằng KDL!")
            return
            
        self.kdl_chain = self.kdl_tree.getChain(self.base_link, self.tip_link)
        self.num_joints = self.kdl_chain.getNrOfJoints()
        self.get_logger().info(f"Đã tạo KDL chain từ {self.base_link} đến {self.tip_link} với {self.num_joints} khớp.")
        
        # Lấy danh sách tên các khớp chủ động (revolute/prismatic) từ chuỗi KDL
        for i in range(self.kdl_chain.getNrOfSegments()):
            segment = self.kdl_chain.getSegment(i)
            joint = segment.getJoint()
            if joint.getType() != PyKDL.Joint.None:
                self.joint_names.append(joint.getName())
                
        self.get_logger().info(f"Khớp KDL: {self.joint_names}")
        
        # Tạo Solvers
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)
        self.dyn_param = PyKDL.ChainDynParam(self.kdl_chain, self.gravity)
        
        # Bắt đầu subscribe joint_states
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
            
        self.get_logger().info("Sensorless Force Node đã sẵn sàng!")

    def joint_state_callback(self, msg: JointState):
        if self.jac_solver is None or self.dyn_param is None:
            return
            
        # Đảm bảo msg.effort có dữ liệu
        if not msg.effort or len(msg.effort) == 0:
            self.get_logger().warning("Không có dữ liệu effort trong /joint_states!", throttle_duration_sec=2.0)
            return
            
        # Lấy positions và efforts theo đúng thứ tự của KDL
        q_kdl = PyKDL.JntArray(self.num_joints)
        tau_measured = np.zeros(self.num_joints)
        
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                q_kdl[i] = msg.position[idx]
                tau_measured[i] = msg.effort[idx]
            else:
                self.get_logger().warning(f"Thiếu khớp {name} trong joint_states!", throttle_duration_sec=1.0)
                return
                
        # Với Yaskawa HC10DTP (và MotoROS2), trường 'effort' trong joint_states
        # thường ĐÃ ĐƯỢC bộ điều khiển YRC1000 bù trọng lực và ma sát bên trong.
        # Nghĩa là tau_measured chính là ngoại lực (tau_ext) tác dụng lên robot!
        tau_ext = tau_measured
        
        # 2. Tính toán ma trận Jacobian
        jacobian = PyKDL.Jacobian(self.num_joints)
        self.jac_solver.JntToJac(q_kdl, jacobian)
        
        # Chuyển Jacobian sang numpy array (6 x N)
        J = np.zeros((6, self.num_joints))
        for i in range(6):
            for j in range(self.num_joints):
                J[i, j] = jacobian[i, j]
                
        # 3. Tính Lực Tương tác (F_ext) thông qua Nghịch đảo giả (Pseudo-inverse) của Jacobian Chuyển vị
        # F_ext = (J^T)^+ * tau_ext
        J_T = J.T  # (N x 6)
        J_T_pinv = np.linalg.pinv(J_T)  # (6 x N)
        
        f_ext = np.dot(J_T_pinv, tau_ext)  # Vector 6 phần tử: [Fx, Fy, Fz, Tx, Ty, Tz]
        
        # 4. Publish lên WrenchStamped
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = self.base_link
        
        # Khử nhiễu: Lọc các giá trị quá nhỏ (Deadband)
        deadband = 1.0 # 1 Newton
        f_ext_filtered = np.where(np.abs(f_ext) < deadband, 0.0, f_ext)
        
        wrench_msg.wrench.force.x = f_ext_filtered[0]
        wrench_msg.wrench.force.y = f_ext_filtered[1]
        wrench_msg.wrench.force.z = f_ext_filtered[2]
        wrench_msg.wrench.torque.x = f_ext_filtered[3]
        wrench_msg.wrench.torque.y = f_ext_filtered[4]
        wrench_msg.wrench.torque.z = f_ext_filtered[5]
        
        self.pub_wrench.publish(wrench_msg)
        
        # In ra log để dễ debug nghiệm thu
        f_str = f"Fx:{f_ext_filtered[0]:5.1f} Fy:{f_ext_filtered[1]:5.1f} Fz:{f_ext_filtered[2]:5.1f}"
        self.get_logger().info(f"F_ext: {f_str}", throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    node = SensorlessForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
