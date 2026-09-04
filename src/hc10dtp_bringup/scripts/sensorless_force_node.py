#!/usr/bin/env python3
"""
SensorlessForceNode — Ước tính lực tương tác tại EE từ dữ liệu dòng điện khớp.

Công thức:  F_ext = (J^T)^{-1} * tau_ext
            với tau_ext = msg.effort (YRC1000 đã bù trọng lực + ma sát nội)

Tối ưu hóa (v2):
  - Pre-allocate mảng NumPy (q, tau, J) để tránh GC mỗi callback
  - Cache index map tên khớp → index trong joint_states (O(1) thay vì O(n))
  - Thay np.linalg.pinv bằng np.linalg.solve (nhanh ~5x với ma trận vuông 6x6)
  - Dùng copy trực tiếp từ PyKDL vào mảng đã pre-allocate
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

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

        # Tham số
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool0')
        self.declare_parameter('gravity_z', -9.81)
        self.declare_parameter('deadband_n', 1.0)

        self.base_link  = self.get_parameter('base_link').value
        self.tip_link   = self.get_parameter('tip_link').value
        gravity_z       = self.get_parameter('gravity_z').value
        self._deadband  = self.get_parameter('deadband_n').value

        # KDL objects (khởi tạo sau khi nhận URDF)
        self.kdl_tree   = None
        self.kdl_chain  = None
        self.jac_solver = None
        self.num_joints = 0

        self.gravity = PyKDL.Vector(0, 0, gravity_z)

        # Mảng pre-allocated (khởi tạo sau khi biết num_joints)
        self._q_kdl   : PyKDL.JntArray | None = None   # góc khớp
        self._jac_kdl : PyKDL.Jacobian  | None = None   # Jacobian KDL
        self._J       : np.ndarray | None = None         # Jacobian NumPy (6 x N)
        self._tau     : np.ndarray | None = None         # Torque vector (N,)

        # Cache: tên khớp → index trong joint_states (tránh msg.name.index mỗi callback)
        self._joint_idx_map: dict[str, int] = {}

        # Tên khớp theo thứ tự KDL
        self._kdl_joint_names: list[str] = []

        # Subscribe URDF với QoS TRANSIENT_LOCAL
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_urdf = self.create_subscription(
            String, '/robot_description', self._cb_urdf, qos)

        # Publisher
        self.pub_wrench = self.create_publisher(WrenchStamped, '/sensorless_force', 10)

        # Joint states (đăng ký sau khi có URDF)
        self._sub_js = None

        self.get_logger().info("SensorlessForceNode v2 — Đang chờ /robot_description...")

    # ── URDF callback ──────────────────────────────────────────────────────────

    def _cb_urdf(self, msg: String):
        if self.kdl_tree is not None:
            return  # Đã init rồi

        success, self.kdl_tree = treeFromString(msg.data)
        if not success:
            self.get_logger().error("Không thể parse URDF bằng KDL!")
            return

        self.kdl_chain  = self.kdl_tree.getChain(self.base_link, self.tip_link)
        self.num_joints = self.kdl_chain.getNrOfJoints()

        # Thu thập tên các khớp chủ động
        for i in range(self.kdl_chain.getNrOfSegments()):
            seg   = self.kdl_chain.getSegment(i)
            joint = seg.getJoint()
            if joint.getType() != PyKDL.Joint.Fixed:
                self._kdl_joint_names.append(joint.getName())

        self.get_logger().info(
            f"KDL chain: {self.base_link} → {self.tip_link} | "
            f"{self.num_joints} khớp: {self._kdl_joint_names}")

        # Tạo KDL solver
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)

        # ── Pre-allocate tất cả mảng một lần duy nhất ──
        self._q_kdl   = PyKDL.JntArray(self.num_joints)
        self._jac_kdl = PyKDL.Jacobian(self.num_joints)
        self._J       = np.zeros((6, self.num_joints), dtype=np.float64)
        self._tau     = np.zeros(self.num_joints, dtype=np.float64)

        # Đăng ký joint_states SAU khi đã sẵn sàng
        self._sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 10)

        self.get_logger().info("SensorlessForceNode v2 sẵn sàng!")

    # ── Joint states callback (hot path — tối ưu tối đa) ──────────────────────

    def _cb_joint_states(self, msg: JointState):
        if self.jac_solver is None:
            return

        # Đảm bảo có dữ liệu effort
        if not msg.effort or len(msg.effort) < self.num_joints:
            self.get_logger().warning(
                "Không đủ dữ liệu effort trong /joint_states!",
                throttle_duration_sec=2.0)
            return

        # ── Build index cache lần đầu tiên (hoặc khi tên khớp thay đổi) ──
        if not self._joint_idx_map:
            for name in self._kdl_joint_names:
                try:
                    self._joint_idx_map[name] = list(msg.name).index(name)
                except ValueError:
                    self.get_logger().warning(
                        f"Không tìm thấy khớp {name} trong joint_states!",
                        throttle_duration_sec=1.0)
                    return

        # ── Đọc q và tau vào mảng pre-allocated (O(n) đơn giản, không cấp phát) ──
        try:
            for i, name in enumerate(self._kdl_joint_names):
                idx = self._joint_idx_map[name]
                self._q_kdl[i]  = msg.position[idx]
                self._tau[i]    = msg.effort[idx]
        except (IndexError, KeyError):
            self.get_logger().warning(
                "Lỗi đọc joint_states — index cache có thể lỗi thời.",
                throttle_duration_sec=1.0)
            self._joint_idx_map.clear()  # Reset cache để build lại lần sau
            return

        # ── Tính Jacobian vào đối tượng KDL đã pre-allocate ──
        self.jac_solver.JntToJac(self._q_kdl, self._jac_kdl)

        # ── Copy Jacobian KDL → NumPy (in-place, không cấp phát bộ nhớ mới) ──
        for i in range(6):
            for j in range(self.num_joints):
                self._J[i, j] = self._jac_kdl[i, j]

        # ── Giải hệ phương trình J^T * F_ext = tau_ext ──
        # Với robot 6 bậc tự do: J^T là ma trận vuông 6x6 → dùng solve thay vì pinv
        J_T = self._J.T  # (6 x 6) — view, không copy
        try:
            f_ext = np.linalg.solve(J_T, self._tau)   # ~5x nhanh hơn pinv
        except np.linalg.LinAlgError:
            # Gần điểm kỳ dị: fall back về lstsq (an toàn hơn)
            f_ext, _, _, _ = np.linalg.lstsq(J_T, self._tau, rcond=None)

        # ── Deadband ──
        mask  = np.abs(f_ext) >= self._deadband
        f_out = np.where(mask, f_ext, 0.0)

        # ── Publish ──
        w = WrenchStamped()
        w.header.stamp    = msg.header.stamp  # Dùng timestamp từ joint_states (chuẩn xác hơn)
        w.header.frame_id = self.base_link
        w.wrench.force.x  = float(f_out[0])
        w.wrench.force.y  = float(f_out[1])
        w.wrench.force.z  = float(f_out[2])
        w.wrench.torque.x = float(f_out[3])
        w.wrench.torque.y = float(f_out[4])
        w.wrench.torque.z = float(f_out[5])
        self.pub_wrench.publish(w)

        self.get_logger().info(
            f"F_ext: Fx={f_out[0]:5.1f}  Fy={f_out[1]:5.1f}  Fz={f_out[2]:5.1f} N",
            throttle_duration_sec=0.5)


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
