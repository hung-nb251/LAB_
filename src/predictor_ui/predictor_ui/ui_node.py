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
import math
from collections import deque
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import SetBool, Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from human_hand_msgs.msg import HandState, HandPrediction
from geometry_msgs.msg import Point, PointStamped, PoseStamped

from .plot_history import append_time_window_xyz
from .target_math import relative_goal, requires_robot_ee_target

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore
    HAS_PYQTGRAPH = True
except ImportError:
    HAS_PYQTGRAPH = False


MAX_POINTS = 300   # Số điểm hiển thị trên mỗi trục
UPDATE_HZ = 20     # FPS cập nhật plot
JOINT_NAMES = [
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
]
# Chỉ dùng khi package cấu hình chưa được cài. Giá trị này phải khớp với
# hc10dtp_moveit_config/config/initial_positions.yaml tại thời điểm phát hành.
FALLBACK_HOME_JOINTS = [
    1.570742, 0.027735, -0.710960,
    0.000032, -0.847848, -0.000658,
]


class PredictorUiNode(Node):
    """ROS 2 node + PyQtGraph dashboard."""

    def __init__(self):
        super().__init__('predictor_ui')

        self.declare_parameter('max_points', MAX_POINTS)
        self.declare_parameter('update_hz', float(UPDATE_HZ))
        self.declare_parameter('require_alignment_steps', True)
        self.declare_parameter('show_camera_plots', True)
        self.declare_parameter('prediction_model', 'svgp')
        n_pts = self.get_parameter('max_points').value
        upd_hz = self.get_parameter('update_hz').value
        self._update_hz = max(1.0, float(upd_hz))
        self._require_alignment_steps = bool(
            self.get_parameter('require_alignment_steps').value)
        self._show_camera_plots = bool(
            self.get_parameter('show_camera_plots').value)
        self._prediction_model = str(
            self.get_parameter('prediction_model').value).strip().lower()
        if self._prediction_model not in ('svgp', 'gru'):
            raise ValueError('prediction_model must be svgp or gru')
        self._home_joint_positions = self._load_home_joint_positions()

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
        # Actual EE arrives at 50--100 Hz while Limited x_d is 15 Hz. A shared
        # sample-count limit kept only 3--6 seconds of Actual EE. Keep this
        # trace by elapsed time instead so both curves cover the same window.
        self._robot_ee = {'x': deque(), 'y': deque(), 'z': deque()}
        self._nominal = {'x': deque(maxlen=n_pts),
                         'y': deque(maxlen=n_pts),
                         'z': deque(maxlen=n_pts)}
        self._t_meas: deque = deque(maxlen=n_pts)
        self._t_pred: deque = deque(maxlen=n_pts)
        self._t_target: deque = deque(maxlen=n_pts)
        self._t_ee: deque = deque()
        self._t_nominal: deque = deque(maxlen=n_pts)
        # Co-carry plots use elapsed time from the beginning of each trial.
        # monotonic() avoids wall-clock jumps and keeps the x-axis non-negative.
        self._robot_plot_epoch = time.monotonic()
        self._robot_plot_window_sec = max(1.0, n_pts / self._update_hz)

        # ── Stats display ────────────────────────────────────────────────────
        self._inf_ms = 0.0
        self._model_name = 'N/A'
        self._buf_size = 0
        self._force_age_ms = float('nan')
        self._prediction_age_ms = float('nan')
        self._controller_status = 'UNKNOWN'
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
        self._trajectory_profile = 'ground_truth'
        self._is_running = False
        self._external_stop_requested = False
        self._enable_response = None
        self._latest_robot_ee = None
        self._latest_robot_ee_time = 0.0
        self._captured_target_ee = None
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
            PointStamped, '/cocarry/nominal_position', self._cb_nominal, 10)
        self.create_subscription(
            Float32, '/cocarry/force_age_ms', self._cb_force_age, 10)
        self.create_subscription(
            Float32, '/cocarry/prediction_age_ms', self._cb_prediction_age, 10)
        self.create_subscription(
            String, '/cocarry/status', self._cb_controller_status, 10)
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
            target=self._spin_ros, daemon=True)
        self._spin_thread.start()

        self.get_logger().info('[UI] Node started')

    def _spin_ros(self):
        """Exit quietly when launch shuts the shared ROS context down."""
        try:
            rclpy.spin(self)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    def _load_home_joint_positions(self):
        """Đọc một nguồn home pose chung với MoveIt và robot mô phỏng."""
        try:
            config_path = (
                Path(get_package_share_directory('hc10dtp_moveit_config'))
                / 'config' / 'initial_positions.yaml'
            )
            config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
            initial_positions = config['initial_positions']
            positions = [float(initial_positions[name]) for name in JOINT_NAMES]
            self.get_logger().info(
                f'[UI] Loaded Go Home pose from {config_path}: '
                f'{[round(value, 6) for value in positions]}')
            return positions
        except Exception as exc:
            self.get_logger().warn(
                '[UI] Could not load initial_positions.yaml; using the '
                f'packaged fallback home pose: {exc}')
            return list(FALLBACK_HOME_JOINTS)

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
            if self._show_camera_plots:
                self._t_meas.append(t)
                self._meas['x'].append(-msg.point.x)  # Đảo dấu X để thuận mắt trên UI
                self._meas['y'].append(msg.point.y)
                self._meas['z'].append(msg.point.z)
            self._fps_counter_m += 1

    def _cb_pred(self, msg: HandPrediction):
        if not self._is_drawing_ui or not self._is_predicting:
            return
        t = time.time()
        with self._lock:
            if self._show_camera_plots:
                self._t_pred.append(t)
                self._pred['x'].append(-msg.x)  # Đảo dấu X để đồng bộ với Actual
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
        wall_time = time.time()
        plot_time = time.monotonic()
        with self._lock:
            point = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            )
            self._latest_robot_ee = point
            self._latest_robot_ee_time = wall_time
            # In co-carry mode the nominal trace stops with the trial. Freeze
            # Actual EE at the same point instead of allowing idle feedback to
            # overwrite the completed trajectory after Stop Run.
            record_history = self._is_drawing_ui and (
                self._show_camera_plots or self._is_running)
            if record_history:
                append_time_window_xyz(
                    self._robot_ee,
                    self._t_ee,
                    plot_time,
                    point,
                    self._robot_plot_window_sec,
                )

    def _append_point(self, buffers, timestamps, msg):
        if not self._is_drawing_ui:
            return
        with self._lock:
            timestamps.append(time.monotonic())
            buffers['x'].append(msg.point.x)
            buffers['y'].append(msg.point.y)
            buffers['z'].append(msg.point.z)

    def _cb_nominal(self, msg: PointStamped):
        self._append_point(self._nominal, self._t_nominal, msg)

    def _cb_force_age(self, msg: Float32):
        with self._lock:
            self._force_age_ms = float(msg.data)

    def _cb_prediction_age(self, msg: Float32):
        with self._lock:
            self._prediction_age_ms = float(msg.data)

    def _cb_controller_status(self, msg: String):
        with self._lock:
            self._controller_status = msg.data

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

    def capture_target_from_robot_ee(self):
        """Store an absolute base_link EE target while the robot is stopped."""
        with self._lock:
            if (self._latest_robot_ee is None or
                    time.time() - self._latest_robot_ee_time > 0.5):
                return None
            self._captured_target_ee = tuple(self._latest_robot_ee)
            return self._captured_target_ee

    def publish_captured_goal_relative(self):
        """Publish goal displacement relative to the EE pose at Start Run."""
        with self._lock:
            if (self._captured_target_ee is None or
                    self._latest_robot_ee is None or
                    time.time() - self._latest_robot_ee_time > 0.5):
                return None
            relative = relative_goal(
                self._captured_target_ee, self._latest_robot_ee)
        msg = Point()
        msg.x, msg.y, msg.z = relative
        self._goal_pub.publish(msg)
        self.get_logger().info(
            '[UI] MJM goal relative to Start EE → '
            f'({relative[0]:.4f}, {relative[1]:.4f}, {relative[2]:.4f})')
        return relative

    def get_buffers(self):
        with self._lock:
            return (
                list(self._meas['x']), list(self._meas['y']), list(self._meas['z']),
                list(self._pred['x']), list(self._pred['y']), list(self._pred['z']),
                list(self._target_base['x']), list(self._target_base['y']), list(self._target_base['z']),
                list(self._robot_ee['x']), list(self._robot_ee['y']), list(self._robot_ee['z']),
                list(self._nominal['x']), list(self._nominal['y']), list(self._nominal['z']),
                list(self._t_meas), list(self._t_pred), list(self._t_target), list(self._t_ee),
                list(self._t_nominal), self._robot_plot_epoch,
                self._inf_ms, self._model_name, self._buf_size,
                self._fps_meas, self._fps_pred, self._hybrid_state,
                self._force_age_ms, self._prediction_age_ms,
                self._controller_status,
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
        """Publish lệnh bật/tắt Prediction+MJM mode."""
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
        goal_msg.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(self._home_joint_positions)
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
                self._nominal[k].clear()
            self._t_meas.clear()
            self._t_pred.clear()
            self._t_target.clear()
            self._t_ee.clear()
            self._t_nominal.clear()
            self._robot_plot_epoch = time.monotonic()

    def reset_robot_plot_for_trial(self):
        """Start a clean, positive elapsed-time axis for one co-carry trial."""
        with self._lock:
            for k in ['x', 'y', 'z']:
                self._target_base[k].clear()
                self._robot_ee[k].clear()
                self._nominal[k].clear()
            self._t_target.clear()
            self._t_ee.clear()
            self._t_nominal.clear()
            self._robot_plot_epoch = time.monotonic()


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
        title = ('HRC Trajectory Dashboard — Ubuntu'
                 if node._show_camera_plots
                 else 'Co-carry Admittance Dashboard — Robot Frame')
        self.win.setWindowTitle(title)
        self.win.resize(1600, 1100 if node._show_camera_plots else 700)
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
        self._lbl_force_age = QtWidgets.QLabel('Force age: N/A')
        self._lbl_prediction_age = QtWidgets.QLabel('Pred age: N/A')
        self._lbl_controller = QtWidgets.QLabel('Controller: UNKNOWN')
        for lbl in [self._lbl_model, self._lbl_inf, self._lbl_fps, self._lbl_buf, self._lbl_backend]:
            lbl.setStyleSheet('color: #a0f0a0; font-size: 13px; font-weight: bold;')
            top.addWidget(lbl)
        self._lbl_hybrid.setStyleSheet('color: #a0f0a0; font-size: 13px; font-weight: bold; background: transparent; padding: 2px;')
        top.addWidget(self._lbl_hybrid)
        if not node._show_camera_plots:
            for lbl in [self._lbl_force_age, self._lbl_prediction_age,
                        self._lbl_controller]:
                lbl.setStyleSheet(
                    'color: #a0f0a0; font-size: 13px; font-weight: bold;')
                top.addWidget(lbl)
        top.addStretch()
        main_layout.addLayout(top)

        # ── Plots (2 hàng) ────────────────────────────────────────
        plots_area = QtWidgets.QVBoxLayout()

        self.plots = {}
        self.curves_m = {}   # Filtered actual (xanh)
        self.curves_p = {}   # Predicted (cam)

        if node._show_camera_plots:
            # Camera pipeline cũ giữ nguyên hàng Actual/Predicted và Set Goal.
            lbl_cam = QtWidgets.QLabel('■  Camera Frame — Ý định tay người')
            lbl_cam.setStyleSheet('color: #64c8ff; font-size: 12px; font-weight: bold; padding: 2px 4px;')
            plots_area.addWidget(lbl_cam)

            mid_layout = QtWidgets.QHBoxLayout()
            self.gw = pg.GraphicsLayoutWidget()
            plots_data_cam = [
                ('X (m) — Camera', -1.0,  1.0),
                ('Y (m) — Camera', -0.1,  1.5),
                ('Z (m) — Camera', -0.2,  0.8),
            ]

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
                self.curves_m[i] = p.plot(
                    pen=pg.mkPen(self.COLOR_MEAS, width=2), name='Actual')
                self.curves_p[i] = p.plot(
                    pen=pg.mkPen(self.COLOR_PRED, width=2), name='Predicted')
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
        lbl_robot = QtWidgets.QLabel(
            '■  Robot Frame (base_link) — Reference vs vị trí thực robot EE')
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
        # Nominal x_d sau safety limit: nét liền xanh dương.
        self.curves_nominal = {}
        # Actual EE: nét đứt xanh lá — vị trí thực tế robot EE (feedback)
        self.curves_ee = {}

        COLOR_NOMINAL = (90, 210, 255)   # xanh dương — nominal x_d (x̂)
        COLOR_EE      = (100, 220, 120)  # xanh lá — actual EE (x)

        for i, (title, y_lo, y_hi) in enumerate(plots_data_robot):
            p = self.gw2.addPlot(row=0, col=i, title=title)
            p.setLabel('left', title)
            p.setLabel('bottom', 'Elapsed time', units='s')
            p.addLegend(offset=(5, 5))
            p.showGrid(x=True, y=True, alpha=0.3)
            p.getAxis('left').enableAutoSIPrefix(False)
            p.getAxis('bottom').enableAutoSIPrefix(False)
            p.setXRange(0, node._robot_plot_window_sec, padding=0)
            p.enableAutoRange(axis='y', enable=False)
            p.setYRange(y_lo, y_hi, padding=0)
            # Nominal x_d vẽ trước để nằm dưới Actual EE
            if not node._show_camera_plots:
                self.curves_nominal[i] = p.plot(
                    pen=pg.mkPen(COLOR_NOMINAL, width=2),
                    name='Limited x_d')
            self.curves_ee[i] = p.plot(
                pen=pg.mkPen(COLOR_EE, width=2, style=QtCore.Qt.PenStyle.DashLine),
                name='Actual EE (x)')
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
        
        model_label = node._prediction_model.upper()
        self.btn_traj_svgp = QtWidgets.QPushButton(model_label)
        self.btn_traj_svgp.setCheckable(True)
        self.btn_traj_svgp.setStyleSheet(self._btn_style('#16213e', '#0f3460'))
        self.btn_traj_svgp.clicked.connect(lambda: self._set_trajectory_mode('svgp'))
        traj_l.addWidget(self.btn_traj_svgp)

        self.btn_traj_svgpmjm = QtWidgets.QPushButton(f'{model_label}+MJM')
        self.btn_traj_svgpmjm.setCheckable(True)
        self.btn_traj_svgpmjm.setStyleSheet(self._btn_style('#7d4e00', '#ffaa00'))
        self.btn_traj_svgpmjm.clicked.connect(lambda: self._set_trajectory_mode('svgp_mjm'))
        traj_l.addWidget(self.btn_traj_svgpmjm)

        self.btn_traj_gt.setToolTip(
            'Giữ x_d tại pose Start Run; admittance tạo x_r từ lực người')
        self.btn_traj_svgp.setToolTip(
            f'Dùng dự đoán {model_label} từ chuỗi robot EE làm x_d')
        self.btn_traj_svgpmjm.setToolTip(
            f'FOLLOWER: {model_label} + Admittance; '
            'LEADER: MJM gửi điểm trực tiếp')

        row_1.addWidget(traj_grp)

        if not node._show_camera_plots:
            target_grp = QtWidgets.QGroupBox('MJM Target — robot EE')
            target_grp.setStyleSheet(
                'QGroupBox { color: #e0e0e0; border: 1px solid #555; '
                'border-radius: 4px; margin-top: 15px; padding-top: 4px; }'
                'QGroupBox::title { subcontrol-origin: margin; left: 8px; top: 0px; }')
            target_layout = QtWidgets.QHBoxLayout(target_grp)
            self.btn_capture_target = QtWidgets.QPushButton(
                'Set Goal / Capture Target')
            self.btn_capture_target.setStyleSheet(
                self._btn_style('#704214', '#a86820'))
            self.btn_capture_target.setToolTip(
                'Lưu vị trí robot EE hiện tại trong base_link làm đích MJM')
            self.btn_capture_target.clicked.connect(self._do_capture_target)
            target_layout.addWidget(self.btn_capture_target)
            self.lbl_captured_target = QtWidgets.QLabel('Target: chưa capture')
            self.lbl_captured_target.setStyleSheet(
                'color: #ffd479; font-size: 12px; font-weight: bold;')
            target_layout.addWidget(self.lbl_captured_target)
            row_1.addWidget(target_grp)

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
        self.btn_calib.setVisible(node._show_camera_plots)
        row_1.addWidget(self.btn_calib)

        self.btn_capture = QtWidgets.QPushButton('Capture Init Pose')
        self.btn_capture.setStyleSheet(self._btn_style('#145a86', '#1f78b4'))
        self.btn_capture.clicked.connect(self._do_capture_init_pose)
        self.btn_capture.setVisible(node._show_camera_plots)
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
        self.timer.start(int(1000 / node._update_hz))

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
        # Pipeline camera cũ vẫn yêu cầu hai bước alignment (mặc định True).
        # Pipeline robot_ee đặt False vì controller tự capture/calibrate tại Start.
        if checked and self.node._require_alignment_steps:
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

        if checked and requires_robot_ee_target(
                self.node._show_camera_plots,
                self.node._trajectory_profile):
            relative_goal = self.node.publish_captured_goal_relative()
            if relative_goal is None:
                self._set_status(
                    'State: PREPARE | Capture a fresh robot-EE target first')
                self.node.get_logger().warn(
                    '[UI] Cannot start Prediction+MJM without a captured '
                    'target and fresh EE pose')
                QtWidgets.QMessageBox.warning(
                    self.win, 'Action Required',
                    'Move the robot to the desired goal and click '
                    'Set Goal / Capture Target before Start Run.')
                self.btn_pred.setChecked(False)
                return

        if checked and not self.node._show_camera_plots:
            # Each co-carry trial gets one shared positive time origin. This
            # also prevents samples from a previous trial appearing together
            # with the new Actual EE trace.
            self.node.reset_robot_plot_for_trial()
        
        # Camera mode keeps the historical direct predictor control.  In the
        # robot-EE co-carry profile the admittance controller owns the ordered
        # Stop -> calibrate -> Start sequence; issuing another UI request here
        # caused the observed STARTED/STOPPED/STARTED race.
        if self.node._show_camera_plots:
            if checked and self.node._trajectory_mode == 'prediction':
                self.node.call_predictor_toggle(True)
            else:
                self.node.call_predictor_toggle(False)
        else:
            self.node._is_predicting = bool(
                checked and self.node._trajectory_mode == 'prediction')
        
        self.node.call_logger_toggle(checked)
        self.node._is_running = checked
        if hasattr(self, 'btn_capture_target'):
            self.btn_capture_target.setEnabled(not checked)
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
        
        mode_name = self.node._trajectory_profile.replace('_', ' ').upper()
        mode_str = f'({mode_name})'
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
        if self.node._require_alignment_steps and not self.node._is_calibrated:
            self._set_status('State: ALIGN | Calibrate camera first!')
            return
        if self.node._require_alignment_steps and not self.node._is_init_pose_captured:
            self._set_status('State: ALIGN | Capture init pose first!')
            return
        if not self.node._streamer_enable_cli.service_is_ready():
            self._set_status('State: PREPARE | /cartesian_streamer/enable not ready')
            return
        req = SetBool.Request()
        req.data = True
        future = self.node._streamer_enable_cli.call_async(req)

        def _enable_done(done_future):
            try:
                result = done_future.result()
                response = (bool(result.success), result.message)
            except Exception as exc:
                response = (False, f'Enable service exception: {exc}')
            with self.node._lock:
                self.node._enable_response = response

        future.add_done_callback(_enable_done)
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
        if hasattr(self, 'btn_capture_target'):
            self.btn_capture_target.setEnabled(True)
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
            self.node._trajectory_profile = 'ground_truth'
            self.node.set_trajectory_mode('ground_truth')
            self.btn_traj_gt.setChecked(True)
            self.btn_traj_svgp.setChecked(False)
            self.btn_traj_svgpmjm.setChecked(False)
            self._set_status('State: PREPARE | Mode: GROUND TRUTH')
        elif mode == 'svgp':
            self.node._trajectory_profile = 'svgp'
            self.node.set_trajectory_mode('prediction')
            self.node.send_model_cmd(self.node._prediction_model)
            self.node.send_hybrid_cmd('hybrid_off')
            self.btn_traj_gt.setChecked(False)
            self.btn_traj_svgp.setChecked(True)
            self.btn_traj_svgpmjm.setChecked(False)
            model_label = self.node._prediction_model.upper()
            self._set_status(f'State: PREPARE | Mode: {model_label}')
        elif mode == 'svgp_mjm':
            self.node._trajectory_profile = 'svgp_mjm'
            self.node.set_trajectory_mode('prediction')
            self.node.send_model_cmd(self.node._prediction_model)
            self.node.send_hybrid_cmd('hybrid_on')
            self.btn_traj_gt.setChecked(False)
            self.btn_traj_svgp.setChecked(False)
            self.btn_traj_svgpmjm.setChecked(True)
            model_label = self.node._prediction_model.upper()
            self._set_status(f'State: PREPARE | Mode: {model_label}+MJM')
        # Cập nhật visibility nominal curve ngay khi đổi mode
        if not self.node._show_camera_plots:
            self._update_nominal_visibility()

    def _update_nominal_visibility(self):
        """Ẩn/hiện nominal x_d (x̂) tuỳ trajectory mode.

        Ground Truth: chỉ hiện Actual EE — không có x̂ nào để so sánh.
        Prediction / Prediction+MJM: hiện cả hai để so sánh x̂ vs x.
        """
        show = self.node._trajectory_profile in ('svgp', 'svgp_mjm')
        for curve in self.curves_nominal.values():
            curve.setVisible(show)

    def _do_capture_target(self):
        if self.node._is_running:
            self._set_status('State: RUN | Stop Run before capturing a target')
            return
        target = self.node.capture_target_from_robot_ee()
        if target is None:
            self._set_status('State: PREPARE | No fresh robot EE pose')
            QtWidgets.QMessageBox.warning(
                self.win, 'No robot pose',
                'No fresh /cartesian_streamer/current_pose was received.')
            return
        self.lbl_captured_target.setText(
            f'Target abs: X={target[0]:.4f}, Y={target[1]:.4f}, Z={target[2]:.4f} m')
        self._set_status('State: PREPARE | MJM target captured from robot EE')
        self.node.get_logger().info(
            '[UI] Captured absolute robot-EE target → '
            f'({target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f})')

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
        # rclpy's SIGINT handler can invalidate the ROS context before the Qt
        # event loop exits. Stop this timer immediately so shutdown cannot
        # keep calling ROS action/service clients with an invalid context.
        if not rclpy.ok():
            self._stop_for_ros_shutdown()
            return

        with self.node._lock:
            enable_response = self.node._enable_response
            self.node._enable_response = None
        if enable_response is not None:
            success, message = enable_response
            prefix = 'State: ENABLED' if success else 'State: ENABLE REJECTED'
            self._set_status(f'{prefix} | {message}')

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
         nx, ny, nz,
         t_meas, t_pred, t_target, t_ee, t_nominal, robot_plot_epoch,
         inf_ms, model, buf, fps_m, fps_p, hybrid_state,
         force_age_ms, prediction_age_ms,
         controller_status) = self.node.get_buffers()

        now = time.time()
        tm = [t - now for t in t_meas]
        tp = [t - now for t in t_pred]
        tt = [t - now for t in t_target]
        # Robot curves share a trial-relative, non-negative time axis. Unlike
        # sample indices, this keeps 100 Hz Actual EE aligned with 15 Hz x_d.
        te = [max(0.0, t - robot_plot_epoch) for t in t_ee]
        tn = [max(0.0, t - robot_plot_epoch) for t in t_nominal]

        axes_m = [mx, my, mz]
        axes_p = [px, py, pz]
        axes_t = [tx, ty, tz]
        axes_e = [rex, rey, rez]
        axes_n = [nx, ny, nz]

        if self.node._show_camera_plots:
            for i in range(3):
                self.curves_m[i].setData(tm, axes_m[i])
                self.curves_p[i].setData(tp, axes_p[i])

        show_nominal = self.node._trajectory_profile in ('svgp', 'svgp_mjm')
        elapsed = max(te[-1] if te else 0.0, tn[-1] if tn else 0.0)
        window = self.node._robot_plot_window_sec
        x_right = max(window, elapsed)
        x_left = max(0.0, x_right - window)
        for i in range(3):
            if not self.node._show_camera_plots:
                self.curves_nominal[i].setData(tn, axes_n[i])
                self.curves_nominal[i].setVisible(show_nominal)
            self.curves_ee[i].setData(te, axes_e[i])
            self.plots_r[i].setXRange(x_left, x_right, padding=0)


        self._lbl_model.setText(f'Model: {model}')
        self._lbl_inf.setText(f'Inf: {inf_ms:.1f} ms')
        self._lbl_fps.setText(f'Actual: {fps_m:.1f} Hz | Predicted: {fps_p:.1f} Hz')
        self._lbl_buf.setText(f'Buf: {buf}')
        if not self.node._show_camera_plots:
            force_age = (
                f'{force_age_ms:.0f} ms' if math.isfinite(force_age_ms)
                else 'N/A')
            prediction_age = (
                f'{prediction_age_ms:.0f} ms'
                if math.isfinite(prediction_age_ms) else 'N/A')
            self._lbl_force_age.setText(f'Force age: {force_age}')
            self._lbl_prediction_age.setText(f'Pred age: {prediction_age}')
            self._lbl_controller.setText(f'Controller: {controller_status}')
            stale_or_fault = (
                'STALE' in controller_status or 'FAULT' in controller_status)
            self._lbl_force_age.setStyleSheet(
                'color: #ff7070; font-size: 13px; font-weight: bold;'
                if stale_or_fault else
                'color: #a0f0a0; font-size: 13px; font-weight: bold;')
        try:
            mode = self.node.detect_backend_mode()
        except (KeyboardInterrupt, RuntimeError):
            self._stop_for_ros_shutdown()
            return
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

    def _stop_for_ros_shutdown(self):
        self.timer.stop()
        self.win.close()
        self.app.quit()


# ── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    if not HAS_PYQTGRAPH:
        print('[ui_node] ERROR: pyqtgraph not installed. Run: pip install pyqtgraph PyQt5')
        sys.exit(1)

    rclpy.init(args=args)
    node = PredictorUiNode()

    exit_code = 0
    try:
        dashboard = DashboardWindow(node)
        exit_code = dashboard.exec()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    main()
