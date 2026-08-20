#!/usr/bin/env python3
"""
gravity_compensator.py
══════════════════════
ROS2 Node bù trọng trường (Gravity Compensation) cho cảm biến lực/mô-men
ATI Axia80-M20 trong bài toán Co-carrying + Admittance Control.

■ Thuật toán:
  1. Đọc Orientation của cảm biến (frame 'axia_sensor_link') từ TF2
     tương đối với frame 'base_link' (world-fixed).
  2. Tính vectơ trọng lực g = [0, 0, -9.81] m/s² trong hệ tọa độ cảm biến
     bằng cách áp dụng ma trận xoay ngược: g_sensor = R_ws @ g_world
  3. Tính lực trọng trường trong hệ tọa độ cảm biến:
       F_gravity = mass * g_sensor
  4. Tính mô-men xoắn do trọng lực gây ra (đòn bẩy):
       T_gravity = r_com x F_gravity     (r_com: vectơ từ tâm cảm biến đến CoM)
  5. Trừ đi các giá trị trọng trường:
       F_human = F_raw - F_gravity
       T_human = T_raw - T_gravity

■ Subscriptions:
  /axia/raw_wrench         (geometry_msgs/WrenchStamped)
  /tf                      (TF2, dùng qua tf2_ros)

■ Publications:
  /axia/compensated_wrench (geometry_msgs/WrenchStamped)
  /axia/human_force        (geometry_msgs/Vector3Stamped)

■ Services:
  /axia/set_bias           (std_srvs/Trigger)

■ Tham số:
  payload.mass_kg          : Khối lượng tổng (gạch + gá) tính bằng kg
  payload.com_x/y/z        : Tọa độ Trọng tâm CoM (m) trong frame cảm biến
  sensor_frame_id          : TF frame của cảm biến (mặc định: 'axia_sensor_link')
  base_frame_id            : TF frame gốc world-fixed (mặc định: 'base_link')
  bias_samples             : Số mẫu để tính bias (mặc định: 100)
  deadband_force_n         : Ngưỡng deadband lực đầu ra (N)
  deadband_torque_nm       : Ngưỡng deadband mô-men đầu ra (Nm)

■ Cách chạy:
  sudo -E ros2 run coord_transform gravity_compensator --ros-args \
    -p payload.mass_kg:=5.0 \
    -p payload.com_x:=0.05 \
    -p payload.com_y:=0.0  \
    -p payload.com_z:=-0.10
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from std_srvs.srv import Trigger

import tf2_ros


class GravityCompensatorNode(Node):

    GRAVITY = 9.80665  # m/s^2

    def __init__(self):
        super().__init__('gravity_compensator')

        # ── Khai báo tham số ─────────────────────────────────────────────────
        self.declare_parameter('payload.mass_kg',    1.0)
        self.declare_parameter('payload.com_x',      0.0)
        self.declare_parameter('payload.com_y',      0.0)
        self.declare_parameter('payload.com_z',     -0.05)
        self.declare_parameter('sensor_frame_id',   'axia_sensor_link')
        self.declare_parameter('base_frame_id',     'base_link')
        self.declare_parameter('bias_samples',       100)
        self.declare_parameter('deadband_force_n',   1.0)
        self.declare_parameter('deadband_torque_nm', 0.05)

        self._load_params()

        # ── TF2 ──────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── State nội bộ ─────────────────────────────────────────────────────
        self._bias_F = np.zeros(3)
        self._bias_T = np.zeros(3)
        self._collecting_bias   = False
        self._bias_samples_F   = []
        self._bias_samples_T   = []

        # ── Subscribers ──────────────────────────────────────────────────────
        self._sub_raw = self.create_subscription(
            WrenchStamped,
            '/axia/raw_wrench',
            self._cb_raw_wrench,
            10)

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_comp = self.create_publisher(
            WrenchStamped,
            '/axia/compensated_wrench',
            10)

        self._pub_Fh = self.create_publisher(
            Vector3Stamped,
            '/axia/human_force',
            10)

        # ── Service đặt bias tại tư thế hiện tại ─────────────────────────
        self._srv_bias = self.create_service(
            Trigger,
            '/axia/set_bias',
            self._cb_set_bias)

        self.get_logger().info(
            f'[GravityCompensator] Khởi động.\n'
            f'  payload mass = {self._mass:.3f} kg\n'
            f'  CoM (sensor) = [{self._com[0]:.3f}, {self._com[1]:.3f}, {self._com[2]:.3f}] m\n'
            f'  sensor_frame = {self._sensor_frame}\n'
            f'  base_frame   = {self._base_frame}\n'
            f'  deadband F   = +-{self._db_F:.3f} N,  T = +-{self._db_T:.4f} Nm'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _load_params(self):
        self._mass  = self.get_parameter('payload.mass_kg').value
        com_x       = self.get_parameter('payload.com_x').value
        com_y       = self.get_parameter('payload.com_y').value
        com_z       = self.get_parameter('payload.com_z').value
        self._com   = np.array([com_x, com_y, com_z])       # (m)
        self._sensor_frame = self.get_parameter('sensor_frame_id').value
        self._base_frame   = self.get_parameter('base_frame_id').value
        self._bias_n       = int(self.get_parameter('bias_samples').value)
        self._db_F         = self.get_parameter('deadband_force_n').value
        self._db_T         = self.get_parameter('deadband_torque_nm').value

    def _get_rotation_world_to_sensor(self):
        """
        Trả về ma trận xoay R (3x3) chuyển vectơ từ hệ world (base_link)
        sang hệ tọa độ cảm biến (axia_sensor_link).
        Trả về None nếu TF chưa sẵn sàng.

        R được dùng để biến đổi vectơ trọng trường g từ world frame
        sang sensor frame: g_sensor = R @ g_world
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._sensor_frame,      # target (đích): hệ tọa độ cảm biến
                self._base_frame,        # source (nguồn): hệ tọa độ world
                rclpy.time.Time(),       # lấy TF mới nhất có sẵn
                timeout=Duration(seconds=0.05)
            )
            q = tf.transform.rotation
            # scipy Rotation từ quaternion (x, y, z, w)
            R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # shape (3, 3)
            return R
        except Exception:
            return None

    def _compute_gravity_wrench(self, R_ws: np.ndarray):
        """
        Tính lực và mô-men xoắn trọng trường trong hệ tọa độ cảm biến.

        ■ Toán học chi tiết:

          Bước 1 — Biến đổi vectơ trọng trường vào sensor frame:
            g_world  = [0, 0, -g]            (Z-up convention, g = 9.80665)
            g_sensor = R_ws @ g_world

          Bước 2 — Lực trọng trường tác động lên vật trong sensor frame:
            F_grav = m * g_sensor             (N)

          Bước 3 — Mô-men xoắn do đòn bẩy trọng lực:
            Tâm cảm biến là gốc tọa độ (origin).
            r_com là vectơ vị trí Trọng tâm (CoM) trong sensor frame.
            T_grav = r_com x F_grav           (cross product, Nm)

            Công thức mở rộng của cross product:
              T_x = com_y * F_gz - com_z * F_gy
              T_y = com_z * F_gx - com_x * F_gz
              T_z = com_x * F_gy - com_y * F_gx

          Bước 4 — Bù vào dữ liệu cảm biến thô:
            F_human = F_raw - F_grav
            T_human = T_raw - T_grav
        """
        g_world  = np.array([0.0, 0.0, -self.GRAVITY])    # (m/s^2)
        g_sensor = R_ws @ g_world                          # (m/s^2) trong sensor frame

        F_grav = self._mass * g_sensor                     # (N)
        T_grav = np.cross(self._com, F_grav)               # (Nm)

        return F_grav, T_grav

    @staticmethod
    def _apply_deadband(val: np.ndarray, threshold: float) -> np.ndarray:
        """
        Áp dụng Deadband theo từng thành phần:
          |val_i| < threshold  =>  output_i = 0.0
          |val_i| >= threshold =>  output_i = val_i - sign(val_i)*threshold  (soft deadband)
        """
        result = np.zeros_like(val)
        for i in range(len(val)):
            if abs(val[i]) >= threshold:
                result[i] = val[i] - np.sign(val[i]) * threshold
        return result

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_raw_wrench(self, msg: WrenchStamped):
        """
        Callback chính: nhận dữ liệu thô, bù trọng trường, publish kết quả.

        Pipeline:
          raw_wrench
            -> trừ bias tĩnh (offset tại tư thế calib)
            -> trừ lực trọng trường động (tính từ TF2 + toán học ma trận)
            -> áp dụng deadband
            -> publish compensated_wrench + human_force
        """
        F_raw = np.array([msg.wrench.force.x,
                          msg.wrench.force.y,
                          msg.wrench.force.z])
        T_raw = np.array([msg.wrench.torque.x,
                          msg.wrench.torque.y,
                          msg.wrench.torque.z])

        # ── Thu thập mẫu bias ────────────────────────────────────────────
        if self._collecting_bias:
            R_ws = self._get_rotation_world_to_sensor()
            if R_ws is not None:
                # Tính trọng lượng lý thuyết tại tư thế này
                F_grav_expected, T_grav_expected = self._compute_gravity_wrench(R_ws)
                
                # Bias nội tại (chỉ do cảm biến) = Đo đạc thô - Trọng lượng lý thuyết
                pure_bias_F = F_raw - F_grav_expected
                pure_bias_T = T_raw - T_grav_expected
                
                self._bias_samples_F.append(pure_bias_F)
                self._bias_samples_T.append(pure_bias_T)
                
                if len(self._bias_samples_F) >= self._bias_n:
                    self._bias_F = np.mean(self._bias_samples_F, axis=0)
                    self._bias_T = np.mean(self._bias_samples_T, axis=0)
                    self._collecting_bias = False
                    self.get_logger().info(
                        f'[GravityCompensator] Lấy mẫu Calib xong. Bias NỘI TẠI:\n'
                        f'  F_bias = {self._bias_F}\n'
                        f'  T_bias = {self._bias_T}'
                    )
            else:
                self.get_logger().warn('Đang chờ TF để Calib...', throttle_duration_sec=2.0)
            return  # Không publish khi đang calibrating

        # ── Lấy ma trận xoay từ TF2 ───────────────────────────────────────
        R_ws = self._get_rotation_world_to_sensor()

        if R_ws is None:
            self.get_logger().warn(
                '[GravityCompensator] TF chưa sẵn sàng, bù bằng bias tĩnh.',
                throttle_duration_sec=5.0
            )
            F_comp = F_raw - self._bias_F
            T_comp = T_raw - self._bias_T
        else:
            # ── Bước 1: Trừ bias tĩnh ────────────────────────────────────
            F_debiased = F_raw - self._bias_F
            T_debiased = T_raw - self._bias_T

            # ── Bước 2: Tính và trừ lực/mô-men trọng trường động ────────
            F_grav, T_grav = self._compute_gravity_wrench(R_ws)
            F_comp = F_debiased - F_grav
            T_comp = T_debiased - T_grav

        # ── Bước 3: Áp dụng Deadband ─────────────────────────────────────
        F_out = self._apply_deadband(F_comp, self._db_F)
        T_out = self._apply_deadband(T_comp, self._db_T)

        # ── Publish: compensated_wrench ───────────────────────────────────
        comp_msg = WrenchStamped()
        comp_msg.header.stamp    = msg.header.stamp
        comp_msg.header.frame_id = self._sensor_frame
        comp_msg.wrench.force.x  = float(F_out[0])
        comp_msg.wrench.force.y  = float(F_out[1])
        comp_msg.wrench.force.z  = float(F_out[2])
        comp_msg.wrench.torque.x = float(T_out[0])
        comp_msg.wrench.torque.y = float(T_out[1])
        comp_msg.wrench.torque.z = float(T_out[2])
        self._pub_comp.publish(comp_msg)

        # ── Publish: human_force (chỉ Fx, Fy, Fz) ────────────────────────
        Fh_msg = Vector3Stamped()
        Fh_msg.header.stamp    = msg.header.stamp
        Fh_msg.header.frame_id = self._sensor_frame
        Fh_msg.vector.x = float(F_out[0])
        Fh_msg.vector.y = float(F_out[1])
        Fh_msg.vector.z = float(F_out[2])
        self._pub_Fh.publish(Fh_msg)

    def _cb_set_bias(self, request, response):
        """
        Service callback: bắt đầu thu thập mẫu để tính bias mới.

        Cách gọi qua CLI:
          ros2 service call /axia/set_bias std_srvs/srv/Trigger {}
        """
        self._bias_samples_F.clear()
        self._bias_samples_T.clear()
        self._collecting_bias = True
        msg = f'Đang thu thập {self._bias_n} mẫu để cập nhật bias...'
        self.get_logger().info(f'[GravityCompensator] {msg}')
        response.success = True
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
