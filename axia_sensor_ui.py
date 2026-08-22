#!/usr/bin/env python3
"""
axia_sensor_ui.py  (Phiên bản tách biệt — chạy bằng USER THƯỜNG)
──────────────────────────────────────────────────────────────────
Node giao diện + tính toán bù trọng lực. KHÔNG cần Sudo, KHÔNG cần
quyền đặc biệt. Subscribe /axia/raw_wrench từ axia_sensor_driver
(chạy ngầm bằng sudo) rồi tính toán gravity compensation dùng TF
(hoạt động 100% vì cùng quyền user với hc10dtp_start).

■ Subscribes:
    /axia/raw_wrench      (geometry_msgs/WrenchStamped)  — từ driver

■ Publishes:
    /axia/human_force     (geometry_msgs/Vector3Stamped) — sau bù trọng lực

■ Services:
    /axia/set_bias        (std_srvs/Trigger)             — calib từ ngoài

■ Cách chạy (KHÔNG CẦN SUDO):
    python3 axia_sensor_ui.py
    # (hoặc được nhúng trong cocarry_real_gui.launch.py)
"""

import sys
import time
import socket
import struct
import threading
import numpy as np
from collections import deque

import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

# ── ROS 2 ───────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Trigger
import tf2_ros
from scipy.spatial.transform import Rotation

# ── Tham số vật lý ───────────────────────────────────────────────────────────
PAYLOAD_MASS_KG = 0.33         # Khối lượng thanh sắt + gá (kg)
GRAVITY         = 9.80665      # m/s²
SENSOR_FRAME    = 'axia_sensor_link'
BASE_FRAME      = 'base_link'
BIAS_SAMPLES    = 100          # Số mẫu để tính bias
DEADBAND_N      = 4.0          # Ngưỡng deadband (N) - Tăng lên 4.0 để khử độ căng cáp


# ════════════════════════════════════════════════════════════════════════════
#  Bộ lọc phần mềm (Median + EMA)
# ════════════════════════════════════════════════════════════════════════════

class ForceFilter:
    def __init__(self, median_size=5, ema_alpha=0.1):
        self.buffer   = deque(maxlen=median_size)
        self.smoothed = None
        self.alpha    = ema_alpha

    def apply(self, val):
        self.buffer.append(val)
        med = np.median(self.buffer, axis=0) if len(self.buffer) >= 3 else val
        if self.smoothed is None:
            self.smoothed = med.copy()
        else:
            self.smoothed = self.alpha * med + (1.0 - self.alpha) * self.smoothed
        return self.smoothed

    def reset(self):
        self.buffer.clear()
        self.smoothed = None


# ════════════════════════════════════════════════════════════════════════════
#  ROS 2 Node (Subscribe + TF2 + Publisher + Service)
# ════════════════════════════════════════════════════════════════════════════

class AxiaROS2Node(Node):
    """
    ROS 2 node tích hợp:
      - Subscribe /axia/raw_wrench (từ driver chạy sudo)
      - Publish /axia/human_force  (sau bù trọng lực)
      - Service /axia/set_bias     (calib từ ngoài)
      - TF2 listener để lấy orientation của cảm biến
    """

    def __init__(self, on_data_cb=None):
        super().__init__('axia_sensor_ui_node')

        # Callback để gửi data lên UI thread
        self._on_data_cb = on_data_cb

        self._pub_fh = self.create_publisher(Vector3Stamped, '/axia/human_force', 10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_service(Trigger, '/axia/set_bias', self._cb_set_bias)

        self._bias_F        = np.zeros(3)
        self._collecting    = False
        self._calib_samples = []
        self._on_calib_done = None

        self._filter = ForceFilter(median_size=5, ema_alpha=0.1)

        # Mở UDP socket nhận data từ axia_sensor_driver.py (sudo)
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.bind(('127.0.0.1', 50000))
        self._udp_thread = threading.Thread(target=self._udp_listener_loop, daemon=True)
        self._udp_thread.start()

        self.get_logger().info(
            f'[AxiaUI] Khởi động (user mode — UDP Listener).\n'
            f'  payload.mass = {PAYLOAD_MASS_KG} kg | sensor = {SENSOR_FRAME}\n'
            f'  Deadband = +/-{DEADBAND_N} N | Calib samples = {BIAS_SAMPLES}'
        )

    # ── UDP Listener (thay thế ROS 2 Subscriber) ────────────────────────────

    def _udp_listener_loop(self):
        self.get_logger().info("[AxiaUI] Bắt đầu lắng nghe UDP port 50000...")
        while True:
            try:
                data, _ = self._udp_sock.recvfrom(1024)
                if len(data) == 24:
                    # Gói 24 bytes = 6 floats (Fx, Fy, Fz, Tx, Ty, Tz)
                    fx, fy, fz, tx, ty, tz = struct.unpack('<6f', data)
                    F3 = np.array([fx, fy, fz])
                    self._process_raw_data(F3)
            except Exception as e:
                self.get_logger().error(f"[AxiaUI] Lỗi UDP: {e}")
                time.sleep(1)

    # ── Xử lý dữ liệu thô ───────────────────────────────────────────────────

    def _process_raw_data(self, F3: np.ndarray):
        F_filt = self._filter.apply(F3)

        if self._collecting:
            self._feed_calib_sample(F_filt)
            return

        F_human = self.compensate(F_filt)
        self.publish_human_force(F_human)

        if self._on_data_cb is not None:
            self._on_data_cb(
                float(F_filt[0]),  float(F_filt[1]),  float(F_filt[2]),
                float(F_human[0]), float(F_human[1]), float(F_human[2]),
            )

    # ── TF ──────────────────────────────────────────────────────────────────

    def get_rotation_world_to_sensor(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                SENSOR_FRAME, BASE_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0)
            )
            q = tf.transform.rotation
            return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        except Exception:
            return None

    def compute_gravity_force(self, R_ws):
        g_sensor = R_ws @ np.array([0.0, 0.0, -GRAVITY])
        return PAYLOAD_MASS_KG * g_sensor

    # ── Calib ────────────────────────────────────────────────────────────────

    def start_bias_collection(self, on_done=None):
        # Kiểm tra an toàn: Bắt buộc phải có TF trước khi Calib
        R_ws = self.get_rotation_world_to_sensor()
        if R_ws is None:
            self.get_logger().error(
                '[AxiaUI] LỖI: Chưa nhận được TF (joint_states) từ Robot. Từ chối Calib!')
            return False

        self._calib_samples.clear()
        self._on_calib_done = on_done
        self._collecting    = True
        self._filter.reset()
        self.get_logger().info(f'[AxiaUI] Calib: thu {BIAS_SAMPLES} mau...')
        return True

    def _feed_calib_sample(self, F_raw_3):
        R_ws = self.get_rotation_world_to_sensor()
        if R_ws is not None:
            self._calib_samples.append(F_raw_3 - self.compute_gravity_force(R_ws))
        else:
            self._calib_samples.append(F_raw_3.copy())
            self.get_logger().warn(
                'Calib: TF chua san sang, dung bias tinh.',
                throttle_duration_sec=2.0)

        if len(self._calib_samples) >= BIAS_SAMPLES:
            self._bias_F     = np.mean(self._calib_samples, axis=0)
            self._collecting = False
            self.get_logger().info(
                f'[AxiaUI] Calib xong! F_bias = {np.round(self._bias_F, 3)}')
            if self._on_calib_done:
                self._on_calib_done()

    def compensate(self, F_raw_3):
        F_comp = F_raw_3 - self._bias_F
        R_ws = self.get_rotation_world_to_sensor()
        if R_ws is not None:
            F_comp = F_comp - self.compute_gravity_force(R_ws)
        F_out = np.zeros(3)
        for i in range(3):
            if abs(F_comp[i]) >= DEADBAND_N:
                F_out[i] = F_comp[i] - np.sign(F_comp[i]) * DEADBAND_N
                
        # Transform Human Force from SENSOR_FRAME back to WORLD_FRAME
        if R_ws is not None:
            # R_ws rotates World -> Sensor. Inverse is Transpose.
            F_out = R_ws.T @ F_out
            
        return F_out

    # ── Publisher ────────────────────────────────────────────────────────────

    def publish_human_force(self, Fh):
        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.vector.x = float(Fh[0])
        msg.vector.y = float(Fh[1])
        msg.vector.z = float(Fh[2])
        self._pub_fh.publish(msg)

    def set_filter_alpha(self, alpha):
        self._filter.alpha = alpha

    def _cb_set_bias(self, request, response):
        success = self.start_bias_collection()
        if not success:
            response.success = False
            response.message = 'Chưa nhận được TF/joint_states. Từ chối Calib!'
            return response
        response.success = True
        response.message = f'Dang thu {BIAS_SAMPLES} mau de cap nhat bias...'
        return response


# ════════════════════════════════════════════════════════════════════════════
#  Worker thread kích hoạt calib + emit signal lên UI
# ════════════════════════════════════════════════════════════════════════════

class AxiaUIWorker(QtCore.QObject):
    """Cầu nối giữa ROS2 callbacks (background thread) và PyQt5 UI (main thread)."""
    data_ready   = QtCore.pyqtSignal(float, float, float, float, float, float)
    error_signal = QtCore.pyqtSignal(str)
    calib_done   = QtCore.pyqtSignal()

    def __init__(self, ros_node: AxiaROS2Node):
        super().__init__()
        self.ros_node = ros_node
        # Nối callback data từ ROS node sang Qt signal
        ros_node._on_data_cb = self._on_ros_data

    def _on_ros_data(self, rx, ry, rz, hx, hy, hz):
        self.data_ready.emit(rx, ry, rz, hx, hy, hz)

    def request_tare(self):
        def _on_done():
            self.calib_done.emit()

        success = self.ros_node.start_bias_collection(on_done=_on_done)
        if not success:
            self.error_signal.emit(
                "Chưa nhận được TF/joint_states từ Robot.\n"
                "Vui lòng kiểm tra kết nối (micro-ROS) hoặc Robot chưa bật!")

    def set_filter(self, alpha):
        self.ros_node.set_filter_alpha(alpha)


# ════════════════════════════════════════════════════════════════════════════
#  Giao diện chính (PyQt5)
# ════════════════════════════════════════════════════════════════════════════

class AxiaSensorUI(QtWidgets.QMainWindow):

    def __init__(self, ros_node: AxiaROS2Node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Force Dashboard (Axia80-M20)')
        self.resize(1400, 750)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        # ── Control panel ────────────────────────────────────────────────
        ctrl = QtWidgets.QHBoxLayout()

        self.btn_calib = QtWidgets.QPushButton('🎯 Calibrate F/T Sensor')
        self.btn_calib.setMinimumHeight(45)
        self.btn_calib.setStyleSheet(
            'font-weight:bold;font-size:14px;'
            'background-color:#2e8b57;color:white;border-radius:6px;')
        self.btn_calib.clicked.connect(self._on_calib)
        ctrl.addWidget(self.btn_calib)

        ctrl.addSpacing(15)
        btn_rst = QtWidgets.QPushButton('🔄 Reset Draw')
        btn_rst.setMinimumHeight(45)
        btn_rst.setStyleSheet(
            'font-weight:bold;font-size:13px;'
            'background-color:#d2691e;color:white;border-radius:6px;')
        btn_rst.clicked.connect(self._on_reset)
        ctrl.addWidget(btn_rst)

        ctrl.addSpacing(30)
        lbl = QtWidgets.QLabel('Low-pass Filter:')
        lbl.setStyleSheet('font-weight:bold;')
        ctrl.addWidget(lbl)

        self.combo = QtWidgets.QComboBox()
        self.combo.setMinimumHeight(40)
        self.combo.addItems([
            '0: Raw (Alpha=1.0)',
            '1: Nhe (Alpha=0.5)',
            '2: Trung binh (Alpha=0.2)',
            '3: Manh (Alpha=0.1) — Mac dinh',
            '4: Rat manh (Alpha=0.05)',
            '5: Sieu muot (Alpha=0.01)',
        ])
        self.combo.setCurrentIndex(3)
        self.combo.currentIndexChanged.connect(self._on_filter)
        ctrl.addWidget(self.combo)

        ctrl.addStretch()
        self.lbl_status = QtWidgets.QLabel('⚪ Chua Calib')
        self.lbl_status.setStyleSheet('font-size:13px;color:#aaaaaa;')
        ctrl.addWidget(self.lbl_status)

        vbox.addLayout(ctrl)

        # ── Đồ thị 2 hàng × 3 cột ───────────────────────────────────────
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', '#0f0f23')
        pg.setConfigOption('foreground', 'w')

        self.graph = pg.GraphicsLayoutWidget()
        vbox.addWidget(self.graph)

        N = 500
        self._N = N
        self._raw  = {a: [0]*N for a in 'xyz'}
        self._comp = {a: [0]*N for a in 'xyz'}

        RAW_COLORS  = {'x': (255,120,50),  'y': (255,200,50),  'z': (50,200,255)}
        COMP_COLORS = {'x': (100,255,100), 'y': (50,255,200),  'z': (200,100,255)}

        self._cr = {}; self._cc = {}

        for col, ax in enumerate('xyz'):
            pr = self.graph.addPlot(row=0, col=col, title=f'Raw Force {ax.upper()} (N)')
            pr.showGrid(x=True, y=True)
            pr.setLabel('bottom', 'Time steps')
            self._cr[ax] = pr.plot(pen=pg.mkPen(RAW_COLORS[ax], width=2))

            pc = self.graph.addPlot(row=1, col=col, title=f'Human Force {ax.upper()} (N)')
            pc.showGrid(x=True, y=True)
            pc.setLabel('bottom', 'Time steps')
            self._cc[ax] = pc.plot(pen=pg.mkPen(COMP_COLORS[ax], width=2))

        # ── Worker (Qt signal bridge) ────────────────────────────────────
        self.worker = AxiaUIWorker(ros_node)
        self.worker.data_ready.connect(self._update)
        self.worker.error_signal.connect(self._err)
        self.worker.calib_done.connect(self._calib_done)

    # ── Slots ────────────────────────────────────────────────────────────────

    def _on_calib(self):
        self.btn_calib.setEnabled(False)
        self.btn_calib.setText('⏳ Dang Calib...')
        self.lbl_status.setText('🟡 Dang thu mau...')
        self.lbl_status.setStyleSheet('font-size:13px;color:#ffc107;')
        self.worker.request_tare()

    def _calib_done(self):
        self.btn_calib.setEnabled(True)
        self.btn_calib.setText('🎯 Calibrate F/T Sensor')
        self.lbl_status.setText('🟢 Calib OK!')
        self.lbl_status.setStyleSheet(
            'font-size:13px;color:#4caf50;font-weight:bold;')

    def _on_reset(self):
        for a in 'xyz':
            self._raw[a]  = [0]*self._N
            self._comp[a] = [0]*self._N
            self._cr[a].setData(self._raw[a])
            self._cc[a].setData(self._comp[a])

    def _on_filter(self, idx):
        alphas = [1.0, 0.5, 0.2, 0.1, 0.05, 0.01]
        self.worker.set_filter(alphas[idx] if idx < len(alphas) else 0.1)

    def _update(self, rx, ry, rz, hx, hy, hz):
        for a, rv, hv in (('x',rx,hx), ('y',ry,hy), ('z',rz,hz)):
            self._raw[a].pop(0);  self._raw[a].append(rv)
            self._comp[a].pop(0); self._comp[a].append(hv)
            self._cr[a].setData(self._raw[a])
            self._cc[a].setData(self._comp[a])

    def _err(self, msg):
        # Nếu Calib thất bại (TF chưa sẵn sàng), reset lại nút
        self.btn_calib.setEnabled(True)
        self.btn_calib.setText('🎯 Calibrate F/T Sensor')
        self.lbl_status.setText('⚪ Chua Calib')
        self.lbl_status.setStyleSheet('font-size:13px;color:#aaaaaa;')
        QtWidgets.QMessageBox.critical(self, 'Loi ket noi Cam bien', msg)

    def closeEvent(self, event):
        event.accept()


# ════════════════════════════════════════════════════════════════════════════
#  Điểm vào
# ════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    ros_node = AxiaROS2Node()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = AxiaSensorUI(ros_node)
    win.show()

    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
