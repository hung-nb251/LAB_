#!/usr/bin/env python3
"""
UI Node — PyQtGraph live dashboard so sánh tọa độ thực tế vs dự đoán.

- Subscribe /hand_position        → tọa độ thực tế (HandState)
- Subscribe /ml/predicted_position → tọa độ dự đoán (HandPrediction)
- 3 đồ thị cuộn real-time: X, Y, Z
- Hiển thị: inference time, FPS, model name, buffer size
- Nút Start/Stop logging (gọi service /logger/toggle)
- Nút đổi model (publish tới /predictor/model_cmd)
"""

import sys
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from human_hand_msgs.msg import HandState, HandPrediction
from geometry_msgs.msg import Point, PointStamped, PoseStamped

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore
    HAS_PYQTGRAPH = True
except ImportError:
    HAS_PYQTGRAPH = False


MAX_POINTS = 300   # Số điểm hiển thị trên mỗi trục
UPDATE_HZ = 20     # FPS cập nhật plot


class PredictorUiNode(Node):
    """ROS 2 node + PyQtGraph dashboard."""

    def __init__(self):
        super().__init__('predictor_ui')

        self.declare_parameter('max_points', MAX_POINTS)
        self.declare_parameter('update_hz', float(UPDATE_HZ))
        n_pts = self.get_parameter('max_points').value
        upd_hz = self.get_parameter('update_hz').value

        # ── Data buffers ────────────────────────────────────────────────────
        self._meas = {'x': deque(maxlen=n_pts),     # Filtered (chính)
                      'y': deque(maxlen=n_pts),
                      'z': deque(maxlen=n_pts)}
        self._pred = {'x': deque(maxlen=n_pts),
                      'y': deque(maxlen=n_pts),
                      'z': deque(maxlen=n_pts)}
        # Robot frame: target_base (lệnh gửi robot) và robot_ee (vị trí thực)
        self._target_base = {'x': deque(maxlen=n_pts),
                             'y': deque(maxlen=n_pts),
                             'z': deque(maxlen=n_pts)}
        self._robot_ee = {'x': deque(maxlen=n_pts),
                          'y': deque(maxlen=n_pts),
                          'z': deque(maxlen=n_pts)}
        self._t_meas: deque = deque(maxlen=n_pts)
        self._t_pred: deque = deque(maxlen=n_pts)
        self._t_target: deque = deque(maxlen=n_pts)
        self._t_ee: deque = deque(maxlen=n_pts)

        # ── Stats display ────────────────────────────────────────────────────
        self._inf_ms = 0.0
        self._model_name = 'N/A'
        self._buf_size = 0
        self._fps_meas = 0.0
        self._fps_pred = 0.0
        self._fps_counter_m = 0
        self._fps_counter_p = 0
        self._fps_t = time.time()
        self._fps_t = time.time()
        self._logging = False
        self._is_drawing_ui = True
        self._is_predicting = False
        self._is_calibrated = False
        self._is_init_pose_captured = False
        self._backend_mode = 'UNKNOWN'
        self._hybrid_state = 'OFF'
        self._trajectory_mode = 'ground_truth'  # 'ground_truth' or 'prediction'
        self._is_running = False
        self._external_stop_requested = False
        self._lock = threading.Lock()

        # ── Subscribers ──────────────────────────────────────────────────────
        # Filtered: đã qua toàn bộ bộ lọc của transform_node — đường chính trên UI
        self.create_subscription(
            PointStamped, '/coord_transform/filtered_hand_position', self._cb_meas, 10)
        self.create_subscription(
            HandPrediction, '/ml/predicted_position', self._cb_pred, 10)
        self.create_subscription(
            PointStamped, '/coord_transform/target_base', self._cb_target_base, 10)
        self.create_subscription(
            PoseStamped, '/cartesian_streamer/current_pose', self._cb_robot_ee, 10)
        self.create_subscription(
            Bool, '/run_status', self._cb_run_status, 10)
        self.create_subscription(
            String, '/predictor/hybrid_state', self._cb_hybrid_state, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self._model_pub = self.create_publisher(String, '/predictor/model_cmd', 5)
        self._run_status_pub = self.create_publisher(Bool, '/run_status', 5)
        self._hybrid_cmd_pub = self.create_publisher(String, '/predictor/hybrid_cmd', 5)
        self._goal_pub = self.create_publisher(Point, '/predictor/goal_cmd', 5)

        # ── Service clients ──────────────────────────────────────────────────
        self._logger_cli = self.create_client(SetBool, '/logger/toggle')
        self._predictor_cli = self.create_client(SetBool, '/predictor/toggle')
        self._calib_cli = self.create_client(Trigger, '/realsense/calibrate_origin')
        self._capture_init_cli = self.create_client(Trigger, '/coord_transform/capture_init_pose')
        self._stop_traj_cli = self.create_client(Trigger, '/stop_traj_mode')
        self._servo_on_cli = self.create_client(Trigger, '/servo_on')
        self._reset_error_cli = self.create_client(Trigger, '/reset_error')
        self._streamer_enable_cli = self.create_client(SetBool, '/cartesian_streamer/enable')

        self._traj_mode_pub = self.create_publisher(String, '/trajectory_mode', 5)

        # ── Action clients ───────────────────────────────────────────────────
        self._go_home_sim_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/hc10dtp_arm_controller/follow_joint_trajectory',
        )
        self._go_home_real_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory',
        )

        # ── FPS timer ───────────────────────────────────────────────────────
        self.create_timer(1.0, self._update_fps)

        # ── Spin in background thread ────────────────────────────────────────
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

        self.get_logger().info('[UI] Node started')

    # ── ROS Callbacks ────────────────────────────────────────────────────────

    # def _cb_raw(self, msg: HandState):
    #     """Lưu tọa độ RAW để vẽ đườ tham khảo mờ."""
    #     if not msg.is_tracked or not self._is_drawing_ui:
    #         return
    #     t = time.time()
    #     with self._lock:
    #         self._t_raw.append(t)
    #         self._raw['x'].append(msg.x)
    #         self._raw['y'].append(msg.y)
    #         self._raw['z'].append(msg.z)

    def _cb_meas(self, msg: PointStamped):
        """Lưu tọa độ ĐÃ LỌC từ /coord_transform/filtered_hand_position."""
        if not self._is_drawing_ui:
            return
        t = time.time()
        with self._lock:
            self._t_meas.append(t)
            self._meas['x'].append(msg.point.x)
            self._meas['y'].append(msg.point.y)
            self._meas['z'].append(msg.point.z)
            self._fps_counter_m += 1

    def _cb_pred(self, msg: HandPrediction):
        if not self._is_drawing_ui or not self._is_predicting:
            return
        t = time.time()
        with self._lock:
            self._t_pred.append(t)
            self._pred['x'].append(msg.x)
            self._pred['y'].append(msg.y)
            self._pred['z'].append(msg.z)
            self._inf_ms = msg.inference_time_ms
            self._model_name = msg.model_name or 'N/A'
            self._buf_size = msg.buffer_size
            self._fps_counter_p += 1

    def _cb_target_base(self, msg: PointStamped):
        """Nhận target đã transform sang base_link (lệnh gửi robot)."""
        if not self._is_drawing_ui:
            return
        t = time.time()
        with self._lock:
            self._t_target.append(t)
            self._target_base['x'].append(msg.point.x)
            self._target_base['y'].append(msg.point.y)
            self._target_base['z'].append(msg.point.z)

    def _cb_robot_ee(self, msg: PoseStamped):
        """Nhận vị trí thực tế của robot EE (base_link frame)."""
        if not self._is_drawing_ui:
            return
        t = time.time()
        with self._lock:
            self._t_ee.append(t)
            self._robot_ee['x'].append(msg.pose.position.x)
            self._robot_ee['y'].append(msg.pose.position.y)
            self._robot_ee['z'].append(msg.pose.position.z)

    def _update_fps(self):
        now = time.time()
        dt = now - self._fps_t
        if dt > 0:
            with self._lock:
                self._fps_meas = self._fps_counter_m / dt
                self._fps_pred = self._fps_counter_p / dt
                self._fps_counter_m = 0
                self._fps_counter_p = 0
        self._fps_t = now

    def _cb_run_status(self, msg: Bool):
        """Lắng nghe lệnh Stop Run từ các node khác (vd: Target Snap từ transform_node)."""
        if not msg.data and self._is_running:
            self.get_logger().info('[UI] Received external Stop Run command (e.g. Target Snap)')
            self._external_stop_requested = True

    def _cb_hybrid_state(self, msg: String):
        with self._lock:
            self._hybrid_state = msg.data

    def get_buffers(self):
        with self._lock:
            return (
                list(self._meas['x']), list(self._meas['y']), list(self._meas['z']),
                list(self._pred['x']), list(self._pred['y']), list(self._pred['z']),
                list(self._target_base['x']), list(self._target_base['y']), list(self._target_base['z']),
                list(self._robot_ee['x']), list(self._robot_ee['y']), list(self._robot_ee['z']),
                list(self._t_meas), list(self._t_pred), list(self._t_target), list(self._t_ee),
                self._inf_ms, self._model_name, self._buf_size,
                self._fps_meas, self._fps_pred, self._hybrid_state
            )

    # ── Service calls ────────────────────────────────────────────────────────

    def call_logger_toggle(self, enable: bool):
        if not self._logger_cli.service_is_ready():
            self.get_logger().warn('[UI] /logger/toggle not ready')
            try:
                QtWidgets.QMessageBox.critical(None, "Service Error", "Logger service (/logger/toggle) is not ready. The logger node might have crashed or not started.")
            except Exception:
                pass
            return
        req = SetBool.Request()
        req.data = enable
        self._logger_cli.call_async(req)
        self._logging = enable

    def call_predictor_toggle(self, enable: bool):
        if not self._predictor_cli.service_is_ready():
            self.get_logger().warn('[UI] /predictor/toggle not ready')
            return
        req = SetBool.Request()
        req.data = enable
        self._predictor_cli.call_async(req)
        self._is_predicting = enable

    def set_trajectory_mode(self, mode: str):
        """Set trajectory mode: 'ground_truth' or 'prediction'"""
        if mode not in ['ground_truth', 'prediction']:
            self.get_logger().warn(f'[UI] Invalid trajectory mode: {mode}')
            return
        self._trajectory_mode = mode
        msg = String()
        msg.data = mode
        self._traj_mode_pub.publish(msg)
        
        # If currently running, we must enable/disable predictor accordingly
        if self._is_running:
            if mode == 'prediction':
                self.call_predictor_toggle(True)
            else:
                self.call_predictor_toggle(False)
                self._is_predicting = False
                
        self.get_logger().info(f'[UI] Trajectory mode set to: {mode}')

    def send_model_cmd(self, model_name: str):
        msg = String()
        msg.data = model_name
        self._model_pub.publish(msg)
        self.get_logger().info(f'[UI] Model cmd → {model_name}')

    def send_hybrid_cmd(self, cmd: str):
        """Publish lệnh bật/tắt Hybrid GRU+MJM mode."""
        msg = String()
        msg.data = cmd
        self._hybrid_cmd_pub.publish(msg)
        self.get_logger().info(f'[UI] Hybrid cmd → {cmd}')

    def call_calibrate(self):
        if not self._calib_cli.service_is_ready():
            self.get_logger().warn('[UI] /realsense/calibrate_origin not ready')
            return
        req = Trigger.Request()
        self._calib_cli.call_async(req)
        self._is_calibrated = True

    def call_capture_init_pose(self):
        if not self._capture_init_cli.service_is_ready():
            self.get_logger().warn('[UI] /coord_transform/capture_init_pose not ready')
            return False
        req = Trigger.Request()
        future = self._capture_init_cli.call_async(req)
        future.add_done_callback(self._on_capture_init_done)
        # Chưa set _is_init_pose_captured — chờ callback xác nhận
        return True

    def _on_capture_init_done(self, future):
        """Callback khi capture_init_pose service trả về kết quả."""
        try:
            result = future.result()
            if result.success:
                self._is_init_pose_captured = True
                self.get_logger().info(
                    f'[UI] Capture Init Pose thành công: {result.message}')
                # Cập nhật status trên GUI (phải gọi cẩn thận nếu khác luồng, 
                # nhưng PyQtGraph xử lý setText() từ thread ngoài trong một số trường hợp)
            else:
                self._is_init_pose_captured = False
                self.get_logger().error(
                    f'[UI] Capture Init Pose THẤT BẠI: {result.message}')
        except Exception as e:
            self._is_init_pose_captured = False
            self.get_logger().error(f'[UI] Capture Init Pose exception: {e}')

    def call_trigger_service(self, client, name: str):
        if not client.service_is_ready():
            self.get_logger().warn(f'[UI] {name} not ready')
            return False
        req = Trigger.Request()
        client.call_async(req)
        return True

    def detect_backend_mode(self) -> str:
        sim_ready = self._go_home_sim_action.wait_for_server(timeout_sec=0.05)
        real_ready = self._go_home_real_action.wait_for_server(timeout_sec=0.05)
        if real_ready:
            self._backend_mode = 'REAL'
        elif sim_ready:
            self._backend_mode = 'SIM'
        elif self._stop_traj_cli.wait_for_service(timeout_sec=0.01):
            # Có MotoROS2 service nhưng chưa có action ready => backend trung gian.
            self._backend_mode = 'HYBRID'
        else:
            self._backend_mode = 'UNKNOWN'
        return self._backend_mode

    def go_home(self):
        import subprocess
        # Dùng script trực tiếp cho real robot/hybrid để đảm bảo an toàn với MotoROS2
        if self._backend_mode in ['REAL', 'HYBRID'] or not (self._go_home_real_action.wait_for_server(timeout_sec=0.1) or self._go_home_sim_action.wait_for_server(timeout_sec=0.1)):
            self.get_logger().info('[UI] Using standalone go_home.py script for real robot')
            import os
            script_path = os.path.expanduser('~/cocarry_ws/src/hc10dtp_bringup/scripts/go_home.py')
            try:
                subprocess.Popen(['python3', script_path])
                return True
            except Exception as e:
                self.get_logger().error(f'[UI] Failed to run go_home.py: {e}')
                return False

        action_client = None
        if self._go_home_real_action.wait_for_server(timeout_sec=0.1):
            action_client = self._go_home_real_action
        elif self._go_home_sim_action.wait_for_server(timeout_sec=0.1):
            action_client = self._go_home_sim_action

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'joint_1_s', 'joint_2_l', 'joint_3_u',
            'joint_4_r', 'joint_5_b', 'joint_6_t'
        ]
        point = JointTrajectoryPoint()
        point.positions = [1.570774, 0.124230, -1.049406, 0.000000, -0.397843, -1.443567]
        point.time_from_start = Duration(sec=3, nanosec=0)
        goal_msg.trajectory.points = [point]
        action_client.send_goal_async(goal_msg)
        return True

    def call_clear_draw(self):
        with self._lock:
            for k in ['x', 'y', 'z']:
                self._meas[k].clear()
                self._pred[k].clear()
                self._target_base[k].clear()
                self._robot_ee[k].clear()
            self._t_meas.clear()
            self._t_pred.clear()
            self._t_target.clear()
            self._t_ee.clear()


# ── PyQtGraph Window ─────────────────────────────────────────────────────────

class DashboardWindow:
    """PyQtGraph dashboard window."""

    COLOR_MEAS = (100, 200, 255)   # blue — thực tế
    COLOR_PRED = (255, 150, 50)    # orange — dự đoán

    def __init__(self, node: PredictorUiNode):
        self.node = node
        pg.setConfigOption('antialias', True)
        pg.setConfigOption('background', '#1a1a2e')
        pg.setConfigOption('foreground', '#e0e0e0')

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle('HRC Trajectory Dashboard — Ubuntu')
        self.win.resize(1600, 1100)   # Mở rộng chiều cao để chứa 6 đồ thị
        self.win.setStyleSheet('background: #1a1a2e; color: #e0e0e0;')

        main_layout = QtWidgets.QVBoxLayout(self.win)

        # ── Top bar: stats ────────────────────────────────────────────────
        top = QtWidgets.QHBoxLayout()
        self._lbl_model = QtWidgets.QLabel('Model: N/A')
        self._lbl_inf = QtWidgets.QLabel('Inf: 0.0 ms')
        self._lbl_fps = QtWidgets.QLabel('FPS: 0 | 0')
        self._lbl_buf = QtWidgets.QLabel('Buf: 0')
        self._lbl_backend = QtWidgets.QLabel('Backend: UNKNOWN')
        self._lbl_hybrid = QtWidgets.QLabel('Mode: OFF')
        for lbl in [self._lbl_model, self._lbl_inf, self._lbl_fps, self._lbl_buf, self._lbl_backend]:
            lbl.setStyleSheet('color: #a0f0a0; font-size: 13px; font-weight: bold;')
            top.addWidget(lbl)
        self._lbl_hybrid.setStyleSheet('color: #a0f0a0; font-size: 13px; font-weight: bold; background: transparent; padding: 2px;')
        top.addWidget(self._lbl_hybrid)
        top.addStretch()
        main_layout.addLayout(top)

        # ── Plots (2 hàng) ────────────────────────────────────────
        plots_area = QtWidgets.QVBoxLayout()

        # ── Hàng 1: Camera Frame (Actual tay người vs Predicted AI) ──────
        lbl_cam = QtWidgets.QLabel('■  Camera Frame — Ý định tay người')
        lbl_cam.setStyleSheet('color: #64c8ff; font-size: 12px; font-weight: bold; padding: 2px 4px;')
        plots_area.addWidget(lbl_cam)

        mid_layout = QtWidgets.QHBoxLayout()
        self.gw = pg.GraphicsLayoutWidget()
        # Removed setFixedHeight to allow auto-stretching
        plots_data_cam = [
            ('X (m) — Camera', -1.0,  1.0),
            ('Y (m) — Camera', -0.1,  1.5),
            ('Z (m) — Camera', -0.2,  0.8),
        ]

        self.plots = {}
        self.curves_m = {}   # Filtered actual (xanh)
        self.curves_p = {}   # Predicted (cam)

        for i, (title, y_lo, y_hi) in enumerate(plots_data_cam):
            p = self.gw.addPlot(row=0, col=i, title=title)
            p.setLabel('left', title)
            p.setLabel('bottom', 'Time (s)')
            p.addLegend(offset=(5, 5))
            p.showGrid(x=True, y=True, alpha=0.3)
            p.getAxis('left').enableAutoSIPrefix(False)
            p.getAxis('bottom').enableAutoSIPrefix(False)
            p.setXRange(-10, 0, padding=0)
            p.enableAutoRange(axis='y', enable=False)
            p.setYRange(y_lo, y_hi, padding=0)
            self.curves_m[i] = p.plot(pen=pg.mkPen(self.COLOR_MEAS, width=2), name='Actual')
            self.curves_p[i] = p.plot(pen=pg.mkPen(self.COLOR_PRED, width=2), name='Predicted')
            self.plots[i] = p
            self.gw.ci.layout.setColumnStretchFactor(i, 1)

        mid_layout.addWidget(self.gw, stretch=6)

        self.right_panel = QtWidgets.QVBoxLayout()
        self.btn_set_goal = QtWidgets.QPushButton('Set Goal')
        self.btn_set_goal.setStyleSheet(self._btn_style('#b9770e', '#f39c12'))
        self.btn_set_goal.setToolTip('Lưu lại vị trí hiện tại làm Goal')
        self.btn_set_goal.clicked.connect(self._do_set_goal)
        self.right_panel.addWidget(self.btn_set_goal)

        lbl_style = "color: #e0e0e0; font-size: 24px; font-weight: bold; margin-top: 15px; margin-left: 10px;"
        self.lbl_goal_x = QtWidgets.QLabel('X: -----')
        self.lbl_goal_x.setStyleSheet(lbl_style)
        self.right_panel.addWidget(self.lbl_goal_x)
        self.lbl_goal_y = QtWidgets.QLabel('Y: -----')
        self.lbl_goal_y.setStyleSheet(lbl_style)
        self.right_panel.addWidget(self.lbl_goal_y)
        self.lbl_goal_z = QtWidgets.QLabel('Z: -----')
        self.lbl_goal_z.setStyleSheet(lbl_style)
        self.right_panel.addWidget(self.lbl_goal_z)
        self.right_panel.addStretch()
        mid_layout.addLayout(self.right_panel, stretch=1)
        plots_area.addLayout(mid_layout)

        # ── Hàng 2: Robot Frame (target_base vs Robot EE) ─────────────────
        lbl_robot = QtWidgets.QLabel('■  Robot Frame (base_link) — Lệnh target vs Vị trí thực robot')
        lbl_robot.setStyleSheet('color: #90ee90; font-size: 12px; font-weight: bold; padding: 2px 4px;')
        plots_area.addWidget(lbl_robot)

        self.gw2 = pg.GraphicsLayoutWidget()
        # Removed setFixedHeight to allow auto-stretching
        plots_data_robot = [
            ('X (m) — Robot', -0.8, 0.8),
            ('Y (m) — Robot',  0.0, 1.5),
            ('Z (m) — Robot',  0.0, 1.2),
        ]

        self.plots_r = {}
        # Target: nét liền cam — lệnh gửi robot (đã transform sang base_link)
        self.curves_tgt = {}
        # Robot EE: nét đứt xanh lá — vị trí thực tế robot (bám sau target)
        self.curves_ee = {}

        COLOR_TARGET = (255, 165, 50)    # cam — target (lệnh)
        COLOR_EE     = (100, 220, 120)   # xanh lá — robot EE

        for i, (title, y_lo, y_hi) in enumerate(plots_data_robot):
            p = self.gw2.addPlot(row=0, col=i, title=title)
            p.setLabel('left', title)
            p.setLabel('bottom', 'Time (s)')
            p.addLegend(offset=(5, 5))
            p.showGrid(x=True, y=True, alpha=0.3)
            p.getAxis('left').enableAutoSIPrefix(False)
            p.getAxis('bottom').enableAutoSIPrefix(False)
            p.setXRange(-10, 0, padding=0)
            p.enableAutoRange(axis='y', enable=False)
            p.setYRange(y_lo, y_hi, padding=0)
            self.curves_tgt[i] = p.plot(
                pen=pg.mkPen(COLOR_TARGET, width=2),
                name='Predicted')
            self.curves_ee[i] = p.plot(
                pen=pg.mkPen(COLOR_EE, width=2, style=QtCore.Qt.PenStyle.DashLine),
                name='Actual')
            self.plots_r[i] = p
            self.gw2.ci.layout.setColumnStretchFactor(i, 1)

        plots_area.addWidget(self.gw2)
        main_layout.addLayout(plots_area)

        # ── Bottom bar: controls ──────────────────────────────────────────
        ctrl = QtWidgets.QVBoxLayout()

        row_1 = QtWidgets.QHBoxLayout()

        # Trajectory mode selector
        traj_grp = QtWidgets.QGroupBox('Trajectory Mode')
        traj_grp.setStyleSheet(
            'QGroupBox { color: #e0e0e0; border: 1px solid #555; border-radius: 4px; margin-top: 15px; padding-top: 4px; }'
            'QGroupBox::title { subcontrol-origin: margin; left: 8px; top: 0px; }'
        )
        traj_l = QtWidgets.QHBoxLayout(traj_grp)
        self.btn_traj_gt = QtWidgets.QPushButton('Ground Truth')
        self.btn_traj_gt.setCheckable(True)
        self.btn_traj_gt.setChecked(True)
        self.btn_traj_gt.setStyleSheet(self._btn_style('#1a6b2e', '#2ba347'))
        self.btn_traj_gt.clicked.connect(lambda: self._set_trajectory_mode('ground_truth'))
        traj_l.addWidget(self.btn_traj_gt)
        
        self.btn_traj_svgp = QtWidgets.QPushButton('SVGP')
        self.btn_traj_svgp.setCheckable(True)
        self.btn_traj_svgp.setStyleSheet(self._btn_style('#16213e', '#0f3460'))
        self.btn_traj_svgp.clicked.connect(lambda: self._set_trajectory_mode('svgp'))
        traj_l.addWidget(self.btn_traj_svgp)

        self.btn_traj_svgpmjm = QtWidgets.QPushButton('SVGP+MJM')
        self.btn_traj_svgpmjm.setCheckable(True)
        self.btn_traj_svgpmjm.setStyleSheet(self._btn_style('#7d4e00', '#ffaa00'))
        self.btn_traj_svgpmjm.clicked.connect(lambda: self._set_trajectory_mode('svgp_mjm'))
        traj_l.addWidget(self.btn_traj_svgpmjm)

        row_1.addWidget(traj_grp)
        row_1.addStretch()

        # Run toggle
        self.btn_pred = QtWidgets.QPushButton('Start Run')
        self.btn_pred.setCheckable(True)
        self.btn_pred.setStyleSheet(self._btn_style('#1a6b2e', '#2ba347'))
        self.btn_pred.clicked.connect(self._toggle_run)
        row_1.addWidget(self.btn_pred)

        # Calibrate Button
        self.btn_calib = QtWidgets.QPushButton('Calibrate Camera')
        self.btn_calib.setStyleSheet(self._btn_style('#2980b9', '#3498db'))
        self.btn_calib.clicked.connect(self._do_calibrate)
        row_1.addWidget(self.btn_calib)

        self.btn_capture = QtWidgets.QPushButton('Capture Init Pose')
        self.btn_capture.setStyleSheet(self._btn_style('#145a86', '#1f78b4'))
        self.btn_capture.clicked.connect(self._do_capture_init_pose)
        row_1.addWidget(self.btn_capture)

        ctrl.addLayout(row_1)

        row_2 = QtWidgets.QHBoxLayout()

        self.btn_enable_robot = QtWidgets.QPushButton('Enable Robot')
        self.btn_enable_robot.setStyleSheet(self._btn_style('#125c2b', '#1f8a3a'))
        self.btn_enable_robot.clicked.connect(self._enable_robot)
        row_2.addWidget(self.btn_enable_robot)

        self.btn_disable_robot = QtWidgets.QPushButton('Disable Robot')
        self.btn_disable_robot.setStyleSheet(self._btn_style('#7b241c', '#922b21'))
        self.btn_disable_robot.clicked.connect(self._disable_robot)
        row_2.addWidget(self.btn_disable_robot)

        self.btn_soft_stop = QtWidgets.QPushButton('Soft Stop')
        self.btn_soft_stop.setStyleSheet(self._btn_style('#8e0000', '#c0392b'))
        self.btn_soft_stop.clicked.connect(self._soft_stop)
        row_2.addWidget(self.btn_soft_stop)

        self.btn_go_home = QtWidgets.QPushButton('Go Home')
        self.btn_go_home.setStyleSheet(self._btn_style('#6c3483', '#8e44ad'))
        self.btn_go_home.clicked.connect(self._go_home)
        row_2.addWidget(self.btn_go_home)

        # Draw control toggle
        self.btn_draw = QtWidgets.QPushButton('Stop Draw')
        self.btn_draw.setCheckable(True)
        self.btn_draw.setChecked(True)
        self.btn_draw.setStyleSheet(self._btn_style('#8e44ad', '#9b59b6'))
        self.btn_draw.clicked.connect(self._toggle_draw)
        row_2.addWidget(self.btn_draw)

        # Draw clear
        self.btn_clear = QtWidgets.QPushButton('Reset Draw')
        self.btn_clear.setStyleSheet(self._btn_style('#f39c12', '#f1c40f'))
        self.btn_clear.clicked.connect(self._do_clear_draw)
        row_2.addWidget(self.btn_clear)

        row_2.addStretch()
        ctrl.addLayout(row_2)

        self._lbl_status = QtWidgets.QLabel('State: PREPARE | Ready checks pending')
        self._lbl_status.setStyleSheet('color: #ffd479; font-size: 12px; font-weight: bold;')
        ctrl.addWidget(self._lbl_status)

        main_layout.addLayout(ctrl)

        # ── Timer ─────────────────────────────────────────────────────────
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._refresh)
        self.timer.start(int(1000 / UPDATE_HZ))

        self.win.show()

    @staticmethod
    def _btn_style(bg_off, bg_on):
        return (
            f'QPushButton {{ background: {bg_off}; color: #e0e0e0; border: 1px solid {bg_off}; '
            f'border-radius: 4px; padding: 6px 10px; font-weight: bold; }}'
            f'QPushButton:checked {{ background: {bg_on}; border: 1px solid {bg_on}; }}'
            f'QPushButton:hover {{ background: {bg_on}; border: 1px solid #fff; color: #fff; }}'
        )

    def _toggle_run(self, checked):
        if checked:
            if not self.node._is_calibrated:
                self._set_status('State: ALIGN | Calibrate camera first')
                self.node.get_logger().warn('[UI] Cannot start: Calibrate camera first!')
                QtWidgets.QMessageBox.warning(self.win, "Action Required", "Please click 'Calibrate Camera' before starting the run.")
                self.btn_pred.setChecked(False)
                return
            if not self.node._is_init_pose_captured:
                self._set_status('State: ALIGN | Capture init pose first')
                self.node.get_logger().warn('[UI] Cannot start: Capture init pose first!')
                QtWidgets.QMessageBox.warning(self.win, "Action Required", "Please click 'Capture Init Pose' before starting the run.")
                self.btn_pred.setChecked(False)
                return
        
        # Only enable prediction if in prediction mode
        if checked:
            if self.node._trajectory_mode == 'prediction':
                self.node.call_predictor_toggle(True)
            else:
                self.node.call_predictor_toggle(False)
                # Ensure the UI knows prediction is off
                self.node._is_predicting = False
        else:
            self.node.call_predictor_toggle(False)
        
        self.node.call_logger_toggle(checked)
        self.node._is_running = checked
        self.btn_pred.setText('Stop Run' if checked else 'Start Run')
        
        # Notify transform_node whether we are running
        run_msg = Bool()
        run_msg.data = checked
        self.node._run_status_pub.publish(run_msg)
        
        # Khi Stop Run → disable robot luôn để đảm bảo an toàn
        if not checked:
            if self.node._streamer_enable_cli.service_is_ready():
                req = SetBool.Request()
                req.data = False
                self.node._streamer_enable_cli.call_async(req)
                self.node.get_logger().info('[UI] Auto-disabled robot on Stop Run')
        
        mode_str = f'({self.node._trajectory_mode.replace("_", " ").upper()})'
        self._set_status('State: RUN | Streaming enabled ' + mode_str if checked else 'State: STOPPED | Robot disabled')

    def _do_calibrate(self):
        self.node.call_calibrate()
        self._set_status('State: ALIGN | Camera origin calibrated')

    def _do_capture_init_pose(self):
        # Ensure prediction is NOT enabled during capture
        self.node.call_predictor_toggle(False)
        ok = self.node.call_capture_init_pose()
        if ok:
            self._set_status('State: ALIGN | Capturing init pose... (chờ kết quả)')
        else:
            self._set_status('State: ALIGN | Capture init pose failed — service not ready')

    def _enable_robot(self):
        # Guard: phải calibrate và capture init pose trước
        if not self.node._is_calibrated:
            self._set_status('State: ALIGN | Calibrate camera first!')
            return
        if not self.node._is_init_pose_captured:
            self._set_status('State: ALIGN | Capture init pose first!')
            return
        if not self.node._streamer_enable_cli.service_is_ready():
            self._set_status('State: PREPARE | /cartesian_streamer/enable not ready')
            return
        req = SetBool.Request()
        req.data = True
        self.node._streamer_enable_cli.call_async(req)
        # Không publish run_status ở đây — chỉ bật Point Queue Mode.
        # Robot sẽ gửi hold-points và chờ người dùng bấm Start Run.
        self._set_status('State: ENABLED | Robot enabled — press ▶ Start Run to begin')

    def _disable_robot(self):
        # Tắt streamer
        if self.node._streamer_enable_cli.service_is_ready():
            req = SetBool.Request()
            req.data = False
            self.node._streamer_enable_cli.call_async(req)
        # Dừng data flow
        run_msg = Bool()
        run_msg.data = False
        self.node._run_status_pub.publish(run_msg)
        self.node._is_running = False
        # Tắt predictor + logger
        self.node.call_predictor_toggle(False)
        self.node.call_logger_toggle(False)
        self.btn_pred.setChecked(False)
        self.btn_pred.setText('▶ Start Run')
        self._set_status('State: RECOVER | Robot disabled')

    def _soft_stop(self):
        ok = self.node.call_trigger_service(self.node._stop_traj_cli, '/stop_traj_mode')
        if ok:
            self._set_status('State: RECOVER | Soft stop sent')
        else:
            self._set_status('State: RECOVER | Soft stop unavailable')

    def _go_home(self):
        ok = self.node.go_home()
        if ok:
            self._set_status('State: RECOVER | Go-home command sent')
        else:
            self._set_status('State: RECOVER | Go-home action server unavailable')

    def _set_trajectory_mode(self, mode: str):
        # Update button states
        if mode == 'ground_truth':
            self.node.set_trajectory_mode('ground_truth')
            self.btn_traj_gt.setChecked(True)
            self.btn_traj_svgp.setChecked(False)
            self.btn_traj_svgpmjm.setChecked(False)
            self._set_status('State: PREPARE | Mode: GROUND TRUTH')
        elif mode == 'svgp':
            self.node.set_trajectory_mode('prediction')
            self.node.send_model_cmd('svgp')
            self.node.send_hybrid_cmd('hybrid_off')
            self.btn_traj_gt.setChecked(False)
            self.btn_traj_svgp.setChecked(True)
            self.btn_traj_svgpmjm.setChecked(False)
            self._set_status('State: PREPARE | Mode: SVGP')
        elif mode == 'svgp_mjm':
            self.node.set_trajectory_mode('prediction')
            self.node.send_model_cmd('svgp')
            self.node.send_hybrid_cmd('hybrid_on')
            self.btn_traj_gt.setChecked(False)
            self.btn_traj_svgp.setChecked(False)
            self.btn_traj_svgpmjm.setChecked(True)
            self._set_status('State: PREPARE | Mode: SVGP+MJM')

    def _do_set_goal(self):
        with self.node._lock:
            if len(self.node._meas['x']) > 0:
                gx = self.node._meas['x'][-1]
                gy = self.node._meas['y'][-1]
                gz = self.node._meas['z'][-1]
            else:
                self.node.get_logger().warn("[UI] Cannot Set Goal: No tracking data yet!")
                return
                
        # Update UI Labels
        self.lbl_goal_x.setText(f"X: {gx:.4f}")
        self.lbl_goal_y.setText(f"Y: {gy:.4f}")
        self.lbl_goal_z.setText(f"Z: {gz:.4f}")
        
        # Publish to Predictor Node
        msg = Point()
        msg.x = gx
        msg.y = gy
        msg.z = gz
        self.node._goal_pub.publish(msg)
        self.node.get_logger().info(f"[UI] Set Goal → ({gx:.4f}, {gy:.4f}, {gz:.4f})")

    def _toggle_draw(self, checked):
        self.node._is_drawing_ui = checked
        self.btn_draw.setText('⏹ Stop Draw' if checked else '🖍 Start Draw')

    def _do_clear_draw(self):
        self.node.call_clear_draw()

    def _set_status(self, text: str):
        self._lbl_status.setText(text)

    def _refresh(self):
        if getattr(self.node, '_external_stop_requested', False):
            self.node._external_stop_requested = False
            self.node.get_logger().info(
                '[UI] Processing external stop request (Auto-Snap)')
            if self.btn_pred.isChecked():
                self.btn_pred.setChecked(False)
                self._toggle_run(False)
                self._set_status('State: STOPPED | Auto-Snap → Robot disabled')

        (mx, my, mz,
         px, py, pz,
         tx, ty, tz,
         rex, rey, rez,
         t_meas, t_pred, t_target, t_ee,
         inf_ms, model, buf, fps_m, fps_p, hybrid_state) = self.node.get_buffers()

        now = time.time()
        tm = [t - now for t in t_meas]
        tp = [t - now for t in t_pred]
        tt = [t - now for t in t_target]
        te = [t - now for t in t_ee]

        axes_m = [mx, my, mz]
        axes_p = [px, py, pz]
        axes_t = [tx, ty, tz]
        axes_e = [rex, rey, rez]

        for i in range(3):
            ym = axes_m[i]
            yp = axes_p[i]
            yt = axes_t[i]
            ye = axes_e[i]

            self.curves_m[i].setData(tm, ym)   # Filtered (chính)
            self.curves_p[i].setData(tp, yp)   # Predicted
            
            # Robot Frame
            self.curves_tgt[i].setData(tt, yt)
            self.curves_ee[i].setData(te, ye)


        self._lbl_model.setText(f'Model: {model}')
        self._lbl_inf.setText(f'Inf: {inf_ms:.1f} ms')
        self._lbl_fps.setText(f'Actual: {fps_m:.1f} Hz | Predicted: {fps_p:.1f} Hz')
        self._lbl_buf.setText(f'Buf: {buf}')
        mode = self.node.detect_backend_mode()      
        self._lbl_backend.setText(f'Backend: {mode}')

        # Update hybrid state label
        self._lbl_hybrid.setText(f'Mode: {hybrid_state}')
        if hybrid_state == 'LEADER':
            self._lbl_hybrid.setStyleSheet('color: #ffffff; font-size: 14px; font-weight: bold; background: #c0392b; border-radius: 4px; padding: 2px 6px;')
        elif hybrid_state == 'FOLLOWER':
            self._lbl_hybrid.setStyleSheet('color: #ffffff; font-size: 14px; font-weight: bold; background: #2ba347; border-radius: 4px; padding: 2px 6px;')
        else:
            self._lbl_hybrid.setStyleSheet('color: #a0f0a0; font-size: 13px; font-weight: bold; background: transparent; padding: 2px;')

    def exec(self):
        return self.app.exec()


# ── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    if not HAS_PYQTGRAPH:
        print('[ui_node] ERROR: pyqtgraph not installed. Run: pip install pyqtgraph PyQt5')
        sys.exit(1)

    rclpy.init(args=args)
    node = PredictorUiNode()

    try:
        dashboard = DashboardWindow(node)
        sys.exit(dashboard.exec())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
