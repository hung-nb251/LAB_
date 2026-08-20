import sys
import time
import struct
import numpy as np
from collections import deque
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import pysoem

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import threading

class ForceFilter:
    def __init__(self, median_size=5, ema_alpha=0.1):
        self.median_buffer = deque(maxlen=median_size)
        self.smoothed_val = None
        self.ema_alpha = ema_alpha

    def apply(self, val):
        self.median_buffer.append(val)
        if len(self.median_buffer) < 3:
            med_val = val
        else:
            med_val = np.median(self.median_buffer, axis=0)

        if self.smoothed_val is None:
            self.smoothed_val = med_val.copy()
        else:
            self.smoothed_val = self.ema_alpha * med_val + (1.0 - self.ema_alpha) * self.smoothed_val
        return self.smoothed_val

    def reset(self):
        self.median_buffer.clear()
        self.smoothed_val = None

# Cấu hình hằng số cho Axia80-M20
RAW_FMT = '<6iII'
SDO_CALIB_INDEX = 0x2021
SUBIDX_COUNTS_PER_FORCE = 0x37
SUBIDX_COUNTS_PER_TORQUE = 0x38
CONTROL_INDEX = 0x7010
CONTROL_SUBIDX_1 = 0x01

class ATISensorWorker(QtCore.QThread):
    # Tín hiệu phát dữ liệu mới: (Fx, Fy, Fz, Tx, Ty, Tz)
    data_ready = QtCore.pyqtSignal(float, float, float, float, float, float)
    error_signal = QtCore.pyqtSignal(str)
    
    def __init__(self, iface):
        super().__init__()
        self.iface = iface
        self.running = True
        self.master = None
        self.slave = None
        
        self._request_tare = False
        self._is_calibrating = False
        self._calibration_samples = []
        self.bias = np.zeros(6)
        
        self.filter = ForceFilter(median_size=5, ema_alpha=0.1)
        
    def request_tare(self):
        self._request_tare = True
        
    def set_filter(self, alpha):
        self.filter.ema_alpha = alpha
        
    def run(self):
        try:
            self.master = pysoem.Master()
            self.master.open(self.iface)
            
            if self.master.config_init() <= 0:
                raise RuntimeError("Không tìm thấy cảm biến. Vui lòng kiểm tra quyền sudo và cổng mạng.")
            
            self.slave = self.master.slaves[0]
            self.master.config_map()
            
            # Sau config_map(), slave tự động vào SAFEOP — kiểm tra trực tiếp
            self.master.state = pysoem.SAFEOP_STATE
            self.master.write_state()
            self.master.state_check(pysoem.SAFEOP_STATE, 2000000)  # 2s timeout
            if self.master.state != pysoem.SAFEOP_STATE:
                raise RuntimeError("Slave không vào được SAFEOP. Kiểm tra dây EtherCAT và nguồn cảm biến.")

            self.master.state = pysoem.OP_STATE
            self.master.write_state()

            reached_op = False
            for _ in range(400):  # 400 × 50ms = 20 giây timeout
                self.master.send_processdata()
                self.master.receive_processdata(50000)
                self.master.state_check(pysoem.OP_STATE, 50000)
                if self.master.state == pysoem.OP_STATE:
                    reached_op = True
                    break

            if not reached_op:
                raise RuntimeError("Cảm biến không vào được trạng thái OP.")
                
            # Đọc Counts per Force / Torque
            counts_per_force = int.from_bytes(
                self.slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_FORCE), 'little')
            counts_per_torque = int.from_bytes(
                self.slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_TORQUE), 'little')
                
            # Khởi tạo hardware tare (1 lần duy nhất lúc khởi động)
            self.slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (1).to_bytes(4, 'little'))
            for _ in range(5):
                self.master.send_processdata()
                self.master.receive_processdata(2000)
                time.sleep(0.01)
            self.slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (0).to_bytes(4, 'little'))
            
            # Vòng lặp đọc dữ liệu liên tục 100Hz
            while self.running:
                if self._request_tare:
                    self._is_calibrating = True
                    self._calibration_samples = []
                    self.filter.reset()
                    self._request_tare = False

                self.master.send_processdata()
                wkc = self.master.receive_processdata(2000)
                
                if wkc >= self.master.expected_wkc:
                    data = self.slave.input
                    if len(data) == struct.calcsize(RAW_FMT):
                        Fx, Fy, Fz, Tx, Ty, Tz, status, counter = struct.unpack(RAW_FMT, data)
                        
                        raw_val = np.array([
                            Fx / counts_per_force, Fy / counts_per_force, Fz / counts_per_force,
                            Tx / counts_per_torque, Ty / counts_per_torque, Tz / counts_per_torque
                        ])
                        
                        if self._is_calibrating:
                            self._calibration_samples.append(raw_val)
                            # Thu thập mẫu trong 0.5s (50 mẫu ở 100Hz)
                            if len(self._calibration_samples) >= 50:
                                self.bias = np.mean(self._calibration_samples, axis=0)
                                self._is_calibrating = False
                            continue
                            
                        # Khử bias và lọc bằng phần mềm
                        biased_val = raw_val - self.bias
                        filtered_val = self.filter.apply(biased_val)
                        
                        self.data_ready.emit(
                            filtered_val[0], filtered_val[1], filtered_val[2],
                            filtered_val[3], filtered_val[4], filtered_val[5]
                        )
                
                # Chờ một lát cho chu kỳ tiếp theo (10ms)
                time.sleep(0.01)
                
        except Exception as e:
            self.error_signal.emit(str(e))
        finally:
            if self.master:
                self.master.state = pysoem.INIT_STATE
                self.master.write_state()
                self.master.close()
                
    def stop(self):
        self.running = False
        self.wait()


class ROS2PublisherNode(Node):
    def __init__(self):
        super().__init__('axia_sensor_publisher')
        self.pub = self.create_publisher(WrenchStamped, '/axia/raw_wrench', 10)

    def publish_wrench(self, fx, fy, fz, tx, ty, tz):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'axia_sensor_link'
        msg.wrench.force.x = float(fx)
        msg.wrench.force.y = float(fy)
        msg.wrench.force.z = float(fz)
        msg.wrench.torque.x = float(tx)
        msg.wrench.torque.y = float(ty)
        msg.wrench.torque.z = float(tz)
        self.pub.publish(msg)


class AxiaSensorUI(QtWidgets.QMainWindow):
    def __init__(self, iface, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle(f"Force & Torque Dashboard (Axia80-M20) - Port: {iface}")
        self.resize(1400, 850)
        
        # Cài đặt giao diện chính
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)
        
        # Control Panel (Top)
        self.control_panel = QtWidgets.QHBoxLayout()
        
        self.btn_calibrate = QtWidgets.QPushButton("🎯 Calibrate F/T Sensor")
        self.btn_calibrate.setMinimumHeight(40)
        self.btn_calibrate.setStyleSheet("font-weight: bold; background-color: #2e8b57; color: white;")
        self.btn_calibrate.clicked.connect(self.on_calibrate_clicked)
        self.control_panel.addWidget(self.btn_calibrate)
        
        self.control_panel.addSpacing(15)
        
        self.btn_reset = QtWidgets.QPushButton("🔄 Reset Draw")
        self.btn_reset.setMinimumHeight(40)
        self.btn_reset.setStyleSheet("font-weight: bold; background-color: #d2691e; color: white;")
        self.btn_reset.clicked.connect(self.on_reset_clicked)
        self.control_panel.addWidget(self.btn_reset)
        
        self.control_panel.addSpacing(30)
        
        self.lbl_filter = QtWidgets.QLabel("Low-pass Filter Level:")
        self.lbl_filter.setStyleSheet("font-weight: bold;")
        self.control_panel.addWidget(self.lbl_filter)
        
        self.combo_filter = QtWidgets.QComboBox()
        self.combo_filter.setMinimumHeight(40)
        self.combo_filter.addItems([
            "0: No filter (Raw)",
            "1: Nhẹ (Alpha = 0.5)",
            "2: Trung bình (Alpha = 0.2)",
            "3: Mạnh (Alpha = 0.1) - Mặc định",
            "4: Rất mạnh (Alpha = 0.05)",
            "5: Siêu mượt (Alpha = 0.01)"
        ])
        self.combo_filter.setCurrentIndex(3) # Mặc định là 3 (Mạnh)
        self.combo_filter.currentIndexChanged.connect(self.on_filter_changed)
        self.control_panel.addWidget(self.combo_filter)
        
        self.control_panel.addStretch()
        
        self.layout.addLayout(self.control_panel)
        
        # Tạo Widget GraphicsLayout của PyQtGraph
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', 'k')
        pg.setConfigOption('foreground', 'w')
        
        self.graph_layout = pg.GraphicsLayoutWidget()
        self.layout.addWidget(self.graph_layout)
        
        # Khởi tạo kích thước dữ liệu lịch sử trên đồ thị (ví dụ 500 điểm)
        self.history_size = 500
        
        self.fx_data = [0] * self.history_size
        self.fy_data = [0] * self.history_size
        self.fz_data = [0] * self.history_size
        self.tx_data = [0] * self.history_size
        self.ty_data = [0] * self.history_size
        self.tz_data = [0] * self.history_size
        
        # --- HÀNG 1: FORCE (Fx, Fy, Fz) ---
        self.p_fx = self.graph_layout.addPlot(title="Force X (N)")
        self.p_fy = self.graph_layout.addPlot(title="Force Y (N)")
        self.p_fz = self.graph_layout.addPlot(title="Force Z (N)")
        
        self.curve_fx = self.p_fx.plot(pen=pg.mkPen((255, 150, 50), width=2))
        self.curve_fy = self.p_fy.plot(pen=pg.mkPen((255, 150, 50), width=2))
        self.curve_fz = self.p_fz.plot(pen=pg.mkPen((255, 150, 50), width=2))
        
        self.graph_layout.nextRow()
        
        # --- HÀNG 2: TORQUE (Tx, Ty, Tz) ---
        self.p_tx = self.graph_layout.addPlot(title="Torque X (Nm)")
        self.p_ty = self.graph_layout.addPlot(title="Torque Y (Nm)")
        self.p_tz = self.graph_layout.addPlot(title="Torque Z (Nm)")
        
        self.curve_tx = self.p_tx.plot(pen=pg.mkPen((100, 200, 255), width=2)) 
        self.curve_ty = self.p_ty.plot(pen=pg.mkPen((100, 200, 255), width=2)) 
        self.curve_tz = self.p_tz.plot(pen=pg.mkPen((100, 200, 255), width=2))
        
        # Bật lưới grid cho tất cả các đồ thị
        for p in [self.p_fx, self.p_fy, self.p_fz, self.p_tx, self.p_ty, self.p_tz]:
            p.showGrid(x=True, y=True)
            p.setLabel('bottom', 'Time steps')
        
        # --- Khởi động luồng đọc dữ liệu ---
        self.worker = ATISensorWorker(iface)
        self.worker.data_ready.connect(self.update_data)
        self.worker.error_signal.connect(self.show_error)
        self.worker.start()

    def on_calibrate_clicked(self):
        print("Yêu cầu Calibrate (Tare)...")
        self.worker.request_tare()

    def on_reset_clicked(self):
        print("Reset Draw...")
        self.fx_data = [0] * self.history_size
        self.fy_data = [0] * self.history_size
        self.fz_data = [0] * self.history_size
        self.tx_data = [0] * self.history_size
        self.ty_data = [0] * self.history_size
        self.tz_data = [0] * self.history_size
        
        self.curve_fx.setData(self.fx_data)
        self.curve_fy.setData(self.fy_data)
        self.curve_fz.setData(self.fz_data)
        self.curve_tx.setData(self.tx_data)
        self.curve_ty.setData(self.ty_data)
        self.curve_tz.setData(self.tz_data)

    def on_filter_changed(self, index):
        alphas = [1.0, 0.5, 0.2, 0.1, 0.05, 0.01]
        alpha = alphas[index] if index < len(alphas) else 0.1
        print(f"Thay đổi Filter Software Alpha: {alpha}")
        self.worker.set_filter(alpha)

    def update_data(self, fx, fy, fz, tx, ty, tz):
        # Cập nhật mảng lưu trữ theo cơ chế trượt (FIFO)
        self.fx_data.pop(0); self.fx_data.append(fx)
        self.fy_data.pop(0); self.fy_data.append(fy)
        self.fz_data.pop(0); self.fz_data.append(fz)
        
        self.tx_data.pop(0); self.tx_data.append(tx)
        self.ty_data.pop(0); self.ty_data.append(ty)
        self.tz_data.pop(0); self.tz_data.append(tz)
        
        # Cập nhật đồ thị
        self.curve_fx.setData(self.fx_data)
        self.curve_fy.setData(self.fy_data)
        self.curve_fz.setData(self.fz_data)
        
        self.curve_tx.setData(self.tx_data)
        self.curve_ty.setData(self.ty_data)
        self.curve_tz.setData(self.tz_data)
        
        # Publish ra mạng ROS2
        if self.ros_node:
            self.ros_node.publish_wrench(fx, fy, fz, tx, ty, tz)
        
    def show_error(self, err_msg):
        QtWidgets.QMessageBox.critical(self, "Lỗi kết nối Cảm biến", err_msg)
        
    def closeEvent(self, event):
        print("Đang ngắt kết nối với cảm biến...")
        self.worker.stop()
        event.accept()

def main():
    if len(sys.argv) < 2:
        print("Vui lòng cung cấp giao diện mạng LAN!")
        print("Ví dụ: sudo -E python3 axia_sensor_ui.py enxec9a0c1fc063")
        sys.exit(1)
        
    rclpy.init()
    ros_node = ROS2PublisherNode()
    
    # Chạy ROS2 spin ở một thread riêng để không block UI
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
        
    app = QtWidgets.QApplication(sys.argv)
    window = AxiaSensorUI(sys.argv[1], ros_node)
    window.show()
    
    # Khi tắt cửa sổ
    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == "__main__":
    main()
