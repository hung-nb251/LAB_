#!/usr/bin/env python3
"""
identify_payload.py
═══════════════════
Script nhận dạng tham số tải trọng (Payload Identification) cho cảm biến
ATI Axia80-M20 sử dụng phương pháp Bình phương Tối thiểu (Least Squares).

■ Lý thuyết:
  Tại mỗi tư thế i, cảm biến đo tổng:
    F_raw = m * R_i^T * g + F_bias
    T_raw = r_com x (m * R_i^T * g) + T_bias

  Đây là hệ phương trình TUYẾN TÍNH với bộ nghiệm:
    p = [m, m*rx, m*ry, m*rz, bFx, bFy, bFz, bTx, bTy, bTz]

  Mỗi tư thế đóng góp 6 phương trình (3 lực + 3 mô-men).
  Với >= 2 tư thế khác nhau => 12 phương trình > 10 ẩn => Least Squares giải được.
  Càng nhiều tư thế, kết quả càng chính xác.

■ YÊU CẦU VỀ TƯ THẾ:
  - Phải XOAY cổ tay robot (thay đổi góc nghiêng Rotation).
  - KHÔNG dùng tịnh tiến thuần túy (XYZ không đổi hướng).
  - Nên xoay ít nhất 45 độ mỗi lần, theo các chiều KHÁC NHAU.
  - Tối thiểu 6 tư thế, khuyến nghị 8-12 tư thế.

■ Cách chạy:
  sudo -E ros2 run coord_transform identify_payload \
    --ros-args \
    -p sensor_frame_id:='axia_sensor_link' \
    -p base_frame_id:='base_link' \
    -p samples_per_pose:=200 \
    -p output_yaml:='/home/hungnb/cocarry_ws/src/hrc_bringup/config/all_params.yaml'
"""

import sys
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import WrenchStamped
import tf2_ros
import yaml


class PayloadIdentifierNode(Node):

    GRAVITY = 9.80665  # m/s^2

    def __init__(self):
        super().__init__('payload_identifier')

        # ── Tham số ──────────────────────────────────────────────────────────
        self.declare_parameter('sensor_frame_id',   'axia_sensor_link')
        self.declare_parameter('base_frame_id',     'base_link')
        self.declare_parameter('samples_per_pose',   200)
        self.declare_parameter('output_yaml',        '')

        self._sensor_frame  = self.get_parameter('sensor_frame_id').value
        self._base_frame    = self.get_parameter('base_frame_id').value
        self._samples_per_pose = int(self.get_parameter('samples_per_pose').value)
        self._output_yaml   = self.get_parameter('output_yaml').value

        # ── TF2 ──────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── State ─────────────────────────────────────────────────────────────
        # Mỗi pose lưu: (rotation_matrix 3x3, F_mean 3, T_mean 3)
        self._all_poses: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self._current_samples_F: list[np.ndarray] = []
        self._current_samples_T: list[np.ndarray] = []
        self._current_R: np.ndarray | None = None
        self._collecting = False
        self._done = False

        # ── Subscriber ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            WrenchStamped,
            '/axia/raw_wrench',
            self._cb_wrench,
            20)

        # ── Bắt đầu tương tác ────────────────────────────────────────────────
        self._print_banner()
        self._prompt_next_pose()

    # ─────────────────────────────────────────────────────────────────────────

    def _print_banner(self):
        print('\n' + '═'*60)
        print('    NHẬN DẠNG THAM SỐ TẢI TRỌNG — Least Squares')
        print('═'*60)
        print(f'  Sensor frame : {self._sensor_frame}')
        print(f'  Base frame   : {self._base_frame}')
        print(f'  Mẫu/tư thế  : {self._samples_per_pose}')
        print('═'*60)
        print()
        print('  HƯỚNG DẪN TƯ THẾ:')
        print('  Bạn sẽ lần lượt di chuyển robot đến nhiều tư thế khác nhau.')
        print('  Quan trọng: phải XOAY cổ tay (thay đổi hướng nghiêng),')
        print('  không chỉ tịnh tiến XYZ thuần túy.')
        print()
        print('  GỢI Ý 8 TƯ THẾ CHUẨN:')
        print('  [1] Cổ tay thẳng đứng, cảm biến chĩa xuống đất (Pose Home)')
        print('  [2] Xoay quanh trục Y của robot: nghiêng cổ tay về phía trước 45°')
        print('  [3] Xoay quanh trục Y của robot: nghiêng cổ tay về phía sau 45°')
        print('  [4] Xoay quanh trục X của robot: nghiêng cổ tay sang trái 45°')
        print('  [5] Xoay quanh trục X của robot: nghiêng cổ tay sang phải 45°')
        print('  [6] Xoay quanh trục Y: nghiêng 90° (cảm biến chĩa ngang)')
        print('  [7] Xoay quanh trục X: nghiêng 90° (cảm biến chĩa ngang chiều kia)')
        print('  [8] Xoay cổ tay 180°: cảm biến chĩa lên trên (ngược đất)')
        print()

    def _prompt_next_pose(self):
        pose_num = len(self._all_poses) + 1
        print(f'─'*60)
        print(f'  TƯ THẾ #{pose_num}')
        print(f'  Hãy di chuyển robot đến tư thế muốn lấy mẫu,')
        print(f'  giữ robot ĐỨNG YÊN hoàn toàn, rồi nhấn ENTER.')
        print(f'  (Gõ "q" + ENTER để dừng và giải phương trình với {len(self._all_poses)} tư thế đã có.)')

        import threading
        t = threading.Thread(target=self._wait_input, daemon=True)
        t.start()

    def _wait_input(self):
        user_input = input('  > ').strip().lower()
        if user_input == 'q':
            if len(self._all_poses) < 2:
                print('\n  [LỖI] Cần ít nhất 2 tư thế để giải phương trình!')
                self._prompt_next_pose()
                return
            print('\n  => Kết thúc thu thập. Đang giải phương trình ...')
            self._solve_least_squares()
        else:
            self._start_collecting()

    def _start_collecting(self):
        # Lấy R tại tư thế hiện tại
        R = self._get_R()
        if R is None:
            print('  [LỖI] TF2 chưa sẵn sàng! Thử lại sau 2 giây...')
            import time; time.sleep(2.0)
            self._prompt_next_pose()
            return

        self._current_R = R
        self._current_samples_F.clear()
        self._current_samples_T.clear()
        self._collecting = True

        n = self._samples_per_pose
        print(f'  Đang thu thập {n} mẫu ... ', end='', flush=True)

    def _cb_wrench(self, msg: WrenchStamped):
        if not self._collecting:
            return

        F = np.array([msg.wrench.force.x,
                      msg.wrench.force.y,
                      msg.wrench.force.z])
        T = np.array([msg.wrench.torque.x,
                      msg.wrench.torque.y,
                      msg.wrench.torque.z])

        self._current_samples_F.append(F)
        self._current_samples_T.append(T)

        if len(self._current_samples_F) >= self._samples_per_pose:
            self._collecting = False
            F_mean = np.mean(self._current_samples_F, axis=0)
            T_mean = np.mean(self._current_samples_T, axis=0)
            self._all_poses.append((self._current_R.copy(), F_mean, T_mean))

            print('XONG!')
            print(f'  F_mean = [{F_mean[0]:+.4f}, {F_mean[1]:+.4f}, {F_mean[2]:+.4f}] N')
            print(f'  T_mean = [{T_mean[0]:+.4f}, {T_mean[1]:+.4f}, {T_mean[2]:+.4f}] Nm')
            print(f'  Đã có {len(self._all_poses)} tư thế.')
            print()
            self._prompt_next_pose()

    def _get_R(self) -> np.ndarray | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._sensor_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            q = tf.transform.rotation
            return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return None

    # ─────────────────────────────────────────────────────────────────────────
    # Least Squares Solver
    # ─────────────────────────────────────────────────────────────────────────

    def _solve_least_squares(self):
        """
        Xây dựng hệ phương trình Ax = b và giải bằng Least Squares.

        Ẩn số x = [m, m*rx, m*ry, m*rz, bFx, bFy, bFz, bTx, bTy, bTz]
                    [0]  [1]   [2]   [3]   [4]   [5]   [6]   [7]   [8]   [9]

        Mỗi pose i đóng góp 6 hàng vào ma trận A và vectơ b:

        Lực (3 phương trình):
          Fx_raw = m*gx  + bFx   =>  hàng A: [gx,  0,   0,   0,  1, 0, 0, 0, 0, 0]
          Fy_raw = m*gy  + bFy   =>  hàng A: [gy,  0,   0,   0,  0, 1, 0, 0, 0, 0]
          Fz_raw = m*gz  + bFz   =>  hàng A: [gz,  0,   0,   0,  0, 0, 1, 0, 0, 0]

        Mô-men xoắn (3 phương trình) từ: T = r x F_grav = r x (m*g_s)
          Tx = ry*Fgz - rz*Fgy = (m*ry)*gz - (m*rz)*gy
            => hàng A: [0,  0,  gz, -gy, 0, 0, 0, 1, 0, 0]
          Ty = rz*Fgx - rx*Fgz = (m*rz)*gx - (m*rx)*gz
            => hàng A: [0, -gz,  0,  gx, 0, 0, 0, 0, 1, 0]
          Tz = rx*Fgy - ry*Fgx = (m*rx)*gy - (m*ry)*gx
            => hàng A: [0,  gy, -gx,  0, 0, 0, 0, 0, 0, 1]
        """
        g_world = np.array([0.0, 0.0, -self.GRAVITY])

        A_rows = []
        b_rows = []

        for R, F_mean, T_mean in self._all_poses:
            # Vectơ trọng trường trong frame cảm biến tại pose này
            g_s = R @ g_world   # [gx, gy, gz]
            gx, gy, gz = g_s

            # ── Hàng lực (Force) ──────────────────────────────────────────
            # m, m*rx, m*ry, m*rz, bFx, bFy, bFz, bTx, bTy, bTz
            A_rows.append([gx,  0,    0,   0,  1, 0, 0, 0, 0, 0])  # Fx
            A_rows.append([gy,  0,    0,   0,  0, 1, 0, 0, 0, 0])  # Fy
            A_rows.append([gz,  0,    0,   0,  0, 0, 1, 0, 0, 0])  # Fz
            b_rows.extend([F_mean[0], F_mean[1], F_mean[2]])

            # ── Hàng mô-men xoắn (Torque) ────────────────────────────────
            # Tx = (m*ry)*gz - (m*rz)*gy + bTx
            A_rows.append([0,   0,   gz, -gy,  0, 0, 0, 1, 0, 0])  # Tx
            # Ty = (m*rz)*gx - (m*rx)*gz + bTy
            A_rows.append([0,  -gz,  0,   gx,  0, 0, 0, 0, 1, 0])  # Ty
            # Tz = (m*rx)*gy - (m*ry)*gx + bTz
            A_rows.append([0,   gy, -gx,  0,   0, 0, 0, 0, 0, 1])  # Tz
            b_rows.extend([T_mean[0], T_mean[1], T_mean[2]])

        A = np.array(A_rows)  # shape (6*N, 10)
        b = np.array(b_rows)  # shape (6*N,)

        # ── Giải Least Squares ──────────────────────────────────────────
        # Tìm x tối ưu hóa ||Ax - b||^2
        x, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)

        # ── Trích xuất kết quả ──────────────────────────────────────────
        m    = x[0]
        m_rx = x[1]
        m_ry = x[2]
        m_rz = x[3]
        bFx  = x[4]; bFy = x[5]; bFz = x[6]
        bTx  = x[7]; bTy = x[8]; bTz = x[9]

        rx = m_rx / m if abs(m) > 1e-6 else 0.0
        ry = m_ry / m if abs(m) > 1e-6 else 0.0
        rz = m_rz / m if abs(m) > 1e-6 else 0.0

        # Residual (sai số khớp): càng nhỏ càng tốt
        F_pred = A @ x
        residual_rms = np.sqrt(np.mean((F_pred - b)**2))

        # ── Kiểm tra chất lượng ─────────────────────────────────────────
        cond_number = np.linalg.cond(A)

        print('\n' + '═'*60)
        print('  KẾT QUẢ NHẬN DẠNG THAM SỐ TẢI TRỌNG')
        print('═'*60)
        print(f'  Số tư thế thu thập  : {len(self._all_poses)}')
        print(f'  Rank ma trận A      : {rank} / 10 (cần = 10 để đủ thông tin)')
        print(f'  Condition number    : {cond_number:.1f} (< 100 là tốt)')
        print(f'  Residual RMS        : {residual_rms:.6f} N/Nm (càng nhỏ càng tốt)')
        print()
        print(f'  ┌─────────────────────────────────────────┐')
        print(f'  │  mass_kg : {m:+.4f} kg                   │')
        print(f'  │  com_x   : {rx:+.4f} m                  │')
        print(f'  │  com_y   : {ry:+.4f} m                  │')
        print(f'  │  com_z   : {rz:+.4f} m                  │')
        print(f'  │                                         │')
        print(f'  │  F_bias  : [{bFx:+.4f}, {bFy:+.4f}, {bFz:+.4f}] N  │')
        print(f'  │  T_bias  : [{bTx:+.4f}, {bTy:+.4f}, {bTz:+.4f}] Nm│')
        print(f'  └─────────────────────────────────────────┘')

        if rank < 10:
            print()
            print('  [CẢNH BÁO] Rank < 10: Các tư thế chưa đa dạng đủ!')
            print('  Hãy thêm tư thế nghiêng cổ tay theo hướng khác nhau hơn.')

        if cond_number > 1000:
            print()
            print('  [CẢNH BÁO] Condition number cao: kết quả có thể kém ổn định.')
            print('  Nên thêm các tư thế xoay vuông góc với nhau hơn.')

        if abs(m) < 0.05:
            print()
            print('  [CẢNH BÁO] Khối lượng xác định được < 0.05kg.')
            print('  Kiểm tra lại: cảm biến có kết nối TF2 đúng không?')

        # ── Ghi file YAML ───────────────────────────────────────────────
        if self._output_yaml:
            self._write_yaml(m, rx, ry, rz, bFx, bFy, bFz, bTx, bTy, bTz)

        print()
        print('  XONG! Bạn có thể copy các giá trị trên vào all_params.yaml.')
        print('═'*60)
        rclpy.shutdown()

    def _write_yaml(self, m, rx, ry, rz, bFx, bFy, bFz, bTx, bTy, bTz):
        """Ghi kết quả vào file YAML dưới dạng comment ở cuối file."""
        result_block = f"""
# ── Kết quả Nhận dạng Tải trọng (Payload Identification) ─────────────────────
# Chạy bởi: identify_payload.py với {len(self._all_poses)} tư thế
# Hãy copy các giá trị dưới đây vào phần gravity_compensator:
#
#   gravity_compensator:
#     ros__parameters:
#       payload:
#         mass_kg: {m:.4f}
#         com_x:   {rx:.4f}
#         com_y:   {ry:.4f}
#         com_z:   {rz:.4f}
#
#   (F_bias và T_bias được xử lý tự động bởi service /axia/set_bias)
"""
        try:
            with open(self._output_yaml, 'a') as f:
                f.write(result_block)
            print(f'\n  Kết quả đã được ghi vào: {self._output_yaml}')
        except Exception as e:
            print(f'\n  [LỖI] Không ghi được file: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PayloadIdentifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
