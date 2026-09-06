#!/usr/bin/env python3
"""
Predictor Node — ROS 2 node quản lý inference worker subprocess.

- Subscribe /hand_position (HandState) — tọa độ thô từ bridge.
- Duy trì sliding window 20 frames.
- Spawn inference_worker.py trong venv Python qua stdin/stdout JSON.
- Publish kết quả lên /ml/predicted_position (HandPrediction).
- Service /predictor/set_model để đổi model, /predictor/toggle để bật/tắt.

Hybrid prediction+MJM:
- Pha FOLLOWER (0→T_SWITCH s): model dự đoán bình thường.
- Pha LEADER  (T_SWITCH trở đi): MJM thay thế model, inference worker
  nhàn rỗi (không gửi lệnh predict).
"""

import json
import os
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point, PointStamped
from human_hand_msgs.msg import HandState, HandPrediction, SystemStatus

from .feature_math import per_sample_displacement
from .prediction_hold import TimeBasedPredictionHold
from .inference_schedule import InferenceSchedule

# Hybrid math utilities (Fitts' Law + Minimum Jerk Model)
try:
    from .hybrid_math import (
        fitts_law_duration,
        leader_start_position,
        minimum_jerk_positions,
    )
except ImportError:
    from hybrid_math import (
        fitts_law_duration,
        leader_start_position,
        minimum_jerk_positions,
    )


class PredictorNode(Node):
    """ROS 2 Predictor Node với inference subprocess."""

    def __init__(self):
        super().__init__('trajectory_predictor')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('model_dir', '')
        self.declare_parameter('default_model', 'gru')
        self.declare_parameter('scaler_x_file', 'scaler_x_Ts3.pkl')
        self.declare_parameter('scaler_y_file', 'scaler_y_Ts3.pkl')
        self.declare_parameter('window_size', 20)
        self.declare_parameter('num_features', 6)
        self.declare_parameter('velocity_feature_mode', 'legacy_16hz_ema')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('clear_on_tracking_lost', 1.0)
        self.declare_parameter('model_files.rnn', 'rnn_model_Ts3.h5')
        self.declare_parameter('model_files.gru', 'gru_model_Ts3.h5')
        self.declare_parameter('model_files.lstm', 'lstm_model_Ts3.h5')
        self.declare_parameter('model_files.svgp', 'svgp_model.pkl')
        # Path tới Python venv (nếu có), nếu không dùng sys.executable
        self.declare_parameter('venv_python', '')

        # ── Hybrid prediction+MJM parameters ─────────────────────────────────
        self.declare_parameter('mjm.enabled', False)
        # Camera pipeline keeps its historical 30 Hz default.  Co-carry YAML
        # overrides this to 15 Hz to match the HC10DTP Cartesian streamer.
        self.declare_parameter('mjm.publish_rate_hz', 30.0)
        self.declare_parameter('mjm.reference_timeout_sec', 0.25)
        # Preserve the historical camera pipeline fallback. The robot-EE
        # co-carry YAML overrides this to true and requires UI Capture Target.
        self.declare_parameter('mjm.require_goal_cmd', False)
        self.declare_parameter('mjm.t_switch', 5.0)
        self.declare_parameter('mjm.goal_x', -0.0578)
        self.declare_parameter('mjm.goal_y',  0.7138)
        self.declare_parameter('mjm.goal_z',  0.0)
        self.declare_parameter('mjm.fitts_a', 4.4455)
        self.declare_parameter('mjm.fitts_b', 1.4248)
        self.declare_parameter('mjm.fitts_w', 0.3)

        # ── Output filter parameters ─────────────────────────────────────────
        # Proximity clamp: max deviation from last measured position (m)
        self.declare_parameter('filter.max_deviation', 0.15)
        # Rate limiter: max change per frame (m). At 30Hz, 0.04m ≈ 1.2 m/s
        self.declare_parameter('filter.max_rate', 0.04)
        # EMA smoothing factor (0 = no smoothing, 1 = raw prediction)
        self.declare_parameter('filter.ema_alpha', 0.4)
        # Enable/disable output filter
        self.declare_parameter('filter.enabled', True)
        # Co-carry uses controller-owned Start/Stop sequencing.  Keep the
        # historical camera behaviour as the default and override it in the
        # co-carry parameter file.
        self.declare_parameter('trajectory_mode_auto_toggle', True)
        # Zero keeps the historical unlimited request rate.  Robot-EE co-carry
        # overrides this to the 15 Hz model/stream rate.
        self.declare_parameter('inference_rate_hz', 0.0)
        self.declare_parameter('hold.enabled', True)
        self.declare_parameter('hold.time_based', False)
        self.declare_parameter('hold.window_sec', 0.40)
        self.declare_parameter('hold.max_motion_m', 0.003)
        self.declare_parameter('hold.release_distance_m', 0.010)
        self.declare_parameter('hold.publish_rate_hz', 15.0)

        self.model_dir = self.get_parameter('model_dir').value
        self.default_model = self.get_parameter('default_model').value
        self.scaler_x_file = self.get_parameter('scaler_x_file').value
        self.scaler_y_file = self.get_parameter('scaler_y_file').value
        self.window_size = self.get_parameter('window_size').value
        self.num_features = self.get_parameter('num_features').value
        self._velocity_feature_mode = str(
            self.get_parameter('velocity_feature_mode').value).strip().lower()
        if self._velocity_feature_mode not in (
                'legacy_16hz_ema', 'delta_position'):
            raise ValueError(
                'velocity_feature_mode must be legacy_16hz_ema or '
                'delta_position')
        self.auto_start = self.get_parameter('auto_start').value
        self.clear_timeout = self.get_parameter('clear_on_tracking_lost').value
        venv_python = self.get_parameter('venv_python').value
        self.python_exe = venv_python if venv_python else sys.executable

        # Filter config
        self._filter_max_dev = self.get_parameter('filter.max_deviation').value
        self._filter_max_rate = self.get_parameter('filter.max_rate').value
        self._filter_ema_alpha = self.get_parameter('filter.ema_alpha').value
        self._filter_enabled = self.get_parameter('filter.enabled').value
        self._trajectory_mode_auto_toggle = bool(
            self.get_parameter('trajectory_mode_auto_toggle').value)
        inference_rate_hz = float(self.get_parameter('inference_rate_hz').value)
        self._inference_schedule = InferenceSchedule(inference_rate_hz)

        self.model_files = {
            'rnn': self.get_parameter('model_files.rnn').value,
            'gru': self.get_parameter('model_files.gru').value,
            'lstm': self.get_parameter('model_files.lstm').value,
            'svgp': self.get_parameter('model_files.svgp').value,
        }

        # ── Hybrid prediction+MJM config ─────────────────────────────────────
        self._hybrid_enabled  = self.get_parameter('mjm.enabled').value
        mjm_publish_rate = float(self.get_parameter('mjm.publish_rate_hz').value)
        if mjm_publish_rate <= 0.0:
            raise ValueError('mjm.publish_rate_hz must be positive')
        self._mjm_reference_timeout = float(
            self.get_parameter('mjm.reference_timeout_sec').value)
        if self._mjm_reference_timeout <= 0.0:
            raise ValueError('mjm.reference_timeout_sec must be positive')
        self._require_goal_cmd = bool(
            self.get_parameter('mjm.require_goal_cmd').value)
        self._t_switch        = self.get_parameter('mjm.t_switch').value
        self._goal            = np.array([
            self.get_parameter('mjm.goal_x').value,
            self.get_parameter('mjm.goal_y').value,
            self.get_parameter('mjm.goal_z').value,
        ])
        self._fitts_a         = self.get_parameter('mjm.fitts_a').value
        self._fitts_b         = self.get_parameter('mjm.fitts_b').value
        self._fitts_w         = self.get_parameter('mjm.fitts_w').value
        # Sampling period for the generated MJM points.  Simulation may
        # override this independently from the conservative real profile.
        self._mjm_dt          = 1.0 / mjm_publish_rate

        # ── State ────────────────────────────────────────────────────────────
        self._buffer: deque = deque(maxlen=self.window_size)
        self._last_data_time = 0.0
        self._predicting = self.auto_start
        self._current_model = self.default_model
        self._worker_ready = False
        self._worker_proc: subprocess.Popen | None = None
        self._worker_lock = threading.Lock()
        self._pending_model_switch: str | None = None

        # ── Output filter state ──────────────────────────────────────────────
        self._last_meas = [0.0, 0.0, 0.0]      # last measured position
        self._input_source = ''
        self._last_filtered = None              # last filtered prediction [x,y,z]
        self._filter_reject_count = 0
        
        # ── EMA velocity smoothing state ──
        self._smoothed_vel = [0.0, 0.0, 0.0]
        self._vel_ema_alpha = 0.3

        # ── Hybrid prediction+MJM state ───────────────────────────────────────
        # FOLLOWER: model predict normally and remembers its last output.
        # LEADER: stop model requests and publish direct MJM position points.
        self._hybrid_phase       = 'FOLLOWER'  # 'FOLLOWER' | 'LEADER'
        self._hybrid_start_time  = 0.0         # time.time() khi _predicting -> True
        self._x_switch           = None        # [x,y,z] — GRU output cuối pha FOLLOWER
        self._mjm_trajectory     = []          # list of [x,y,z]
        self._mjm_step_idx       = 0
        self._predict_epoch      = 0           # tăng khi chuyển LEADER → invalidate stale responses
        self._goal_valid         = not self._require_goal_cmd
        self._external_control_hold = False
        self._limited_reference = None          # last command, capture-relative XYZ
        self._limited_reference_time = 0.0

        # ── Prediction Hold state ─────────────────────────────────────────────
        # Phát hiện khi tay đứng yên → khóa output prediction = vị trí tay
        # thay vì chạy GRU (loại bỏ hoàn toàn nhiễu dự đoán khi tay dừng)
        self._hold_recent = deque(maxlen=10)     # buffer vị trí gần nhất
        self._hold_stationary_count = 0          # đếm frame liên tiếp tĩnh
        self._hold_active = False                # đang khóa?
        self._hold_target = None                 # vị trí robot sẽ được giữ
        self._hold_reference = None              # vị trí tay người lúc bắt đầu giữ
        self._HOLD_STD_THRESH = 0.006            # ngưỡng std (15mm) — tăng để tránh false-positive khi mang vật
        self._HOLD_ENTER_FRAMES = 5             # cần 8 frame tĩnh liên tiếp (~266ms ở 30Hz)
        self._HOLD_RELEASE_THRESH = 0.030        # tay rời > 50mm thì thả hold — tăng để chắc chắn hơn
        self._hold_enabled = bool(self.get_parameter('hold.enabled').value)
        self._time_based_hold = bool(self.get_parameter('hold.time_based').value)
        self._hold_detector = TimeBasedPredictionHold(
            self.get_parameter('hold.window_sec').value,
            self.get_parameter('hold.max_motion_m').value,
            self.get_parameter('hold.release_distance_m').value,
        )
        hold_publish_rate = float(self.get_parameter('hold.publish_rate_hz').value)
        if hold_publish_rate <= 0.0:
            raise ValueError('hold.publish_rate_hz must be positive')
        self._hold_publish_period = 1.0 / hold_publish_rate
        self._last_hold_publish_time = 0.0

        # ── Publishers ───────────────────────────────────────────────────────
        self.pred_pub = self.create_publisher(
            HandPrediction, '/ml/predicted_position', 10)
        self.raw_pred_pub = self.create_publisher(
            HandPrediction, '/ml/raw_predicted_position', 10)
        self.status_pub = self.create_publisher(
            SystemStatus, '/ml/predictor_status', 10)
        self.hybrid_state_pub = self.create_publisher(
            String, '/predictor/hybrid_state', 5)

        # ── Subscribers ──────────────────────────────────────────────────────
        # Nhận dữ liệu thô (meas) từ /hand_position để giảm độ trễ
        self.create_subscription(HandState, '/hand_position', self._on_hand, 10)
        # Nếu bridge publish tọa độ thô (trước khi predict), cũng lắng nghe
        self.create_subscription(
            HandPrediction, '/predicted_position', self._on_bridge_data, 10)

        # Model switch command từ UI hay bridge
        self.create_subscription(String, '/predictor/model_cmd', self._on_model_cmd, 5)

        # Cập nhật Goal động từ UI
        self.create_subscription(Point, '/predictor/goal_cmd', self._on_goal_cmd, 10)

        # Hybrid mode toggle from the UI (predictor+MJM in co-carry mode).
        self.create_subscription(String, '/predictor/hybrid_cmd', self._on_hybrid_cmd, 5)

        # Trajectory mode to automatically sync predicting state
        self.create_subscription(String, '/trajectory_mode', self._on_trajectory_mode, 10)
        self.create_subscription(
            Bool, '/cocarry/control_hold', self._on_control_hold, 5)
        self.create_subscription(
            PointStamped, '/cocarry/reference_relative',
            self._on_limited_reference, 10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(SetBool, '/predictor/toggle', self._srv_toggle)
        self.create_service(SetBool, '/predictor/set_model_srv', self._srv_toggle)

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_status)
        self.create_timer(self.clear_timeout, self._check_stale)
        # MJM position points use the same rate as the Cartesian streamer.
        self.create_timer(self._mjm_dt, self._mjm_tick)

        # ── Launch inference worker ──────────────────────────────────────────
        self._launch_worker()

        self.get_logger().info(
            f'[Predictor] Started | model={self._current_model} | '
            f'window={self.window_size} | features={self.num_features} | '
            f'velocity={self._velocity_feature_mode} | '
            f'auto_start={self.auto_start}\n'
            f'  Stationary HOLD: enabled={self._hold_enabled}\n'
            f'  Output Filter: enabled={self._filter_enabled}, '
            f'ema_alpha={self._filter_ema_alpha}, '
            f'max_dev={self._filter_max_dev}, '
            f'max_rate={self._filter_max_rate}'
        )
        self._filter_log_counter = 0

    # ── Worker lifecycle ─────────────────────────────────────────────────────

    def _get_worker_script(self) -> str:
        """Tìm đường dẫn tới inference_worker.py."""
        # Thử cùng thư mục với file này
        here = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.join(here, 'inference_worker.py')
        if os.path.exists(candidate):
            return candidate
        # Thử share/ install
        share_candidate = os.path.join(
            os.path.expanduser('~'), 'hrc_ws', 'install',
            'trajectory_predictor', 'lib', 'trajectory_predictor',
            'inference_worker.py'
        )
        if os.path.exists(share_candidate):
            return share_candidate
        raise FileNotFoundError('inference_worker.py not found')

    def _launch_worker(self):
        """Spawn inference_worker.py subprocess."""
        if not self.model_dir:
            self.get_logger().warn('[Predictor] model_dir empty — worker not launched')
            return

        script = self._get_worker_script()
        config = {
            'model_dir': self.model_dir,
            'model_files': self.model_files,
            'scaler_x_file': self.scaler_x_file,
            'scaler_y_file': self.scaler_y_file,
            'default_model': self.default_model,
            'num_features': self.num_features,
            'window_size': self.window_size,
        }

        env = os.environ.copy()
        env['TF_CPP_MIN_LOG_LEVEL'] = '3'
        env['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

        try:
            proc = subprocess.Popen(
                [self.python_exe, script],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env,
            )
            # Gửi config ban đầu
            proc.stdin.write(json.dumps(config) + '\n')
            proc.stdin.flush()

            with self._worker_lock:
                self._worker_proc = proc

            # Thread đọc stdout từ worker
            t = threading.Thread(target=self._reader_loop, daemon=True)
            t.start()

            # Thread đọc stderr
            t2 = threading.Thread(target=self._stderr_loop, daemon=True)
            t2.start()

            self.get_logger().info(f'[Predictor] Worker launched (pid={proc.pid})')
        except Exception as e:
            self.get_logger().error(f'[Predictor] Failed to launch worker: {e}')

    def _reader_loop(self):
        """Đọc JSON response từ worker stdout trong background thread."""
        proc = self._worker_proc
        if not proc:
            return
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                resp = json.loads(line)
            except json.JSONDecodeError:
                continue

            rtype = resp.get('type', '')
            if rtype == 'ready':
                self._worker_ready = resp.get('success', False)
                model = resp.get('model_name', self._current_model)
                if self._worker_ready:
                    self.get_logger().info(
                        f'[Predictor] Worker READY | model={model}')
                    self._current_model = model
                else:
                    self.get_logger().error(
                        f'[Predictor] Worker NOT READY: {resp.get("message")}')

            elif rtype == 'model_loaded':
                ok = resp.get('success', False)
                model = resp.get('model_name', '')
                if ok:
                    self._current_model = model
                    self.get_logger().info(f'[Predictor] Model switched to {model}')
                else:
                    self.get_logger().warn(
                        f'[Predictor] Model switch failed: {resp.get("message")}')

            elif rtype == 'predict':
                pred = resp.get('prediction')
                if pred and len(pred) == 3:
                    # Dùng epoch để loại bỏ stale response khi đã chuyển LEADER.
                    # Worker chạy async → response cũ có thể đến sau khi phase đã đổi.
                    resp_epoch = resp.get('epoch', -1)
                    if (self._predicting and resp_epoch == self._predict_epoch
                            and self._hybrid_phase != 'LEADER'):
                        self._publish_raw_prediction(
                            pred, resp.get('inference_ms', 0.0))
                        self._publish_prediction(pred, resp.get('inference_ms', 0.0))
                        # Lưu _last_filtered (output đã qua EMA/rate-limiter) làm điểm nối
                        # cho MJM, KHÔNG dùng raw pred để tránh cú giật tại điểm chuyển pha.
                        if self._last_filtered is not None:
                            self._x_switch = self._last_filtered[:]
                        else:
                            self._x_switch = pred[:]

            elif rtype == 'info':
                self.get_logger().info(f'[Worker] {resp.get("message", "")}')

    def _stderr_loop(self):
        proc = self._worker_proc
        if not proc:
            return
        for line in proc.stderr:
            line = line.strip()
            if line:
                self.get_logger().warn(f'[Worker STDERR] {line}')

    def _send_to_worker(self, cmd: dict):
        with self._worker_lock:
            proc = self._worker_proc
        if proc and proc.poll() is None and self._worker_ready:
            try:
                proc.stdin.write(json.dumps(cmd) + '\n')
                proc.stdin.flush()
            except BrokenPipeError:
                self.get_logger().error('[Predictor] Worker pipe broken')

    # ── ROS Callbacks ────────────────────────────────────────────────────────

    def _on_hand(self, msg: HandState):
        """Nhận tọa độ thô từ /hand_position (HandState)."""
        if not msg.is_tracked:
            return
        self._input_source = msg.source
        self._ingest_point(msg.x, msg.y, msg.z)

    def _on_bridge_data(self, msg: HandPrediction):
        """
        Nhận từ /predicted_position (output của bridge_node).
        Bridge node publish raw XYZ ở đây (trong pipeline mới, bridge
        publish HandPrediction với inference_time_ms=0 để forward tọa độ gốc).
        """
        # Chỉ dùng nếu /hand_position không hoạt động
        # (bridge gửi raw coords qua HandPrediction với model_name='raw')
        if msg.model_name == 'raw' or msg.model_name == '':
            self._ingest_point(msg.x, msg.y, msg.z)

    def _on_trajectory_mode(self, msg: String):
        """Tự động đồng bộ trạng thái predicting khi mode thay đổi."""
        if not self._trajectory_mode_auto_toggle:
            return
        mode = msg.data
        if mode == 'prediction':
            if not self._predicting:
                self._predicting = True
                self._reset_prediction_runtime()
                # Chờ frame data đầu tiên mới bắt đầu đếm T_SWITCH
                self._hybrid_start_time = 0.0
                self._hybrid_phase = 'FOLLOWER'
                self._x_switch = None
                self._mjm_trajectory = []
                self._mjm_step_idx = 0
                self._publish_hybrid_state()
                self.get_logger().info('[Predictor] Auto-enabled prediction mode via /trajectory_mode')
        else:
            if self._predicting:
                self._predicting = False
                self._reset_prediction_runtime()
                self._publish_hybrid_state()
                self.get_logger().info('[Predictor] Auto-disabled prediction mode via /trajectory_mode')

    def _on_control_hold(self, msg: Bool):
        self._external_control_hold = bool(msg.data)

    def _on_limited_reference(self, msg: PointStamped):
        """Remember the final FOLLOWER command after all controller limits."""
        if (not self._predicting or self._hybrid_phase != 'FOLLOWER'
                or self._input_source != 'robot_ee'):
            return
        reference = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
        if not np.all(np.isfinite(reference)):
            self.get_logger().warn(
                '[Hybrid] Ignoring non-finite limited reference')
            return
        self._limited_reference = reference
        self._limited_reference_time = time.monotonic()

    def _ingest_point(self, x: float, y: float, z: float):
        now = time.time()

        # Keep the latest measurement available for a clean next Start, but do
        # not build model/HOLD state while prediction is stopped.
        if not self._predicting:
            self._last_data_time = now
            self._last_meas = [x, y, z]
            return
        
        # Bắt đầu đếm thời gian FOLLOWER từ frame data đầu tiên thay vì từ lúc bấm nút
        if self._predicting and self._hybrid_enabled and self._hybrid_phase == 'FOLLOWER':
            if getattr(self, '_hybrid_start_time', 0.0) == 0.0:
                self._hybrid_start_time = now

        vx, vy, vz = 0.0, 0.0, 0.0
        if self.num_features == 6:
            if self._velocity_feature_mode == 'delta_position':
                previous = self._last_meas if self._last_data_time > 0 else None
                delta = per_sample_displacement((x, y, z), previous)
                vx, vy, vz = map(float, delta)
                # Keep reset/debug state coherent even though this profile does
                # not apply velocity EMA.
                self._smoothed_vel = [vx, vy, vz]
            else:
                # Historical camera-GRU contract from 7d7700ff/1356a83a:
                # calculate physical velocity, convert it to a nominal 16 Hz
                # per-frame displacement, then apply EMA.  Do not change this
                # path when selecting the robot-EE GRU profile.
                if self._last_data_time > 0:
                    dt = now - self._last_data_time
                    if dt > 0.001:
                        if hasattr(self, '_smoothed_dt'):
                            self._smoothed_dt = (
                                0.1 * dt + 0.9 * self._smoothed_dt)
                        else:
                            self._smoothed_dt = dt
                        raw_velocity = per_sample_displacement(
                            (x, y, z), self._last_meas)
                        raw_velocity = raw_velocity / self._smoothed_dt / 16.0
                    else:
                        raw_velocity = np.zeros(3)
                else:
                    raw_velocity = np.zeros(3)

                smoothed = np.asarray(self._smoothed_vel, dtype=float)
                smoothed = (
                    self._vel_ema_alpha * raw_velocity
                    + (1.0 - self._vel_ema_alpha) * smoothed)
                self._smoothed_vel = smoothed.tolist()
                vx, vy, vz = map(float, smoothed)
        
        self._last_data_time = now
        self._last_meas = [x, y, z]

        if self.num_features == 6:
            self._buffer.append([x, y, z, vx, vy, vz])
        else:
            self._buffer.append([x, y, z])

        # The temporary timer trigger must not be hidden behind HOLD's early
        # return. Goal Classification will replace this trigger later.
        if (self._hybrid_enabled and self._predicting and
                self._hybrid_phase == 'FOLLOWER'):
            elapsed = time.time() - self._hybrid_start_time
            if elapsed >= self._t_switch:
                self._trigger_leader_phase()
                return

        # ── Prediction Hold: phát hiện robot/tay đứng yên ────────────────
        if self._hold_enabled and self._time_based_hold:
            hold_state, hold_target = self._hold_detector.update([x, y, z], now)
            if hold_state == 'released':
                self._buffer.clear()
                self._buffer.append(
                    [x, y, z, vx, vy, vz] if self.num_features == 6
                    else [x, y, z])
                self.get_logger().info(
                    '[Pred HOLD OFF] Robot EE moved beyond the release distance; '
                    'reset model buffer.')
            elif hold_state in ('entered', 'hold'):
                if hold_state == 'entered':
                    self.get_logger().info(
                        '[Pred HOLD ON] Robot EE stationary; hold the measured '
                        f'position ({x:.4f}, {y:.4f}, {z:.4f}).')
                if now - self._last_hold_publish_time >= self._hold_publish_period:
                    self._last_hold_publish_time = now
                    self._publish_prediction(
                        list(hold_target), 0.0, output_source='hold')
                return
        elif self._hold_enabled:
            self._legacy_prediction_hold(x, y, z)
            if self._hold_active:
                return

        # Pha LEADER không gửi lệnh predict; MJM timer publish trực tiếp.
        if not self._worker_ready:
            return
        if self._hybrid_enabled and self._hybrid_phase == 'LEADER':
            return  # Inference worker nhàn rỗi; MJM timer lo phần còn lại
        if not self._inference_schedule.ready(time.monotonic()):
            return

        if 0 < len(self._buffer) < self.window_size:
            first_point = list(self._buffer[0])
            pad_count = self.window_size - len(self._buffer)
            padded = [first_point] * pad_count + list(self._buffer)
        elif len(self._buffer) >= self.window_size:
            padded = list(self._buffer)
        else:
            return

        self._send_to_worker({'cmd': 'predict', 'data': padded, 'epoch': self._predict_epoch})

    def _legacy_prediction_hold(self, x, y, z):
        """Preserve the historical callback-count HOLD for camera profiles."""
        self._hold_recent.append([x, y, z])
        if len(self._hold_recent) >= 5:
            import numpy as _np
            recent = _np.array(self._hold_recent)
            max_std = float(_np.max(_np.std(recent, axis=0)))

            if self._hold_active:
                # Đang HOLD → kiểm tra xem tay đã bắt đầu di chuyển chưa
                mean_pos = _np.mean(recent, axis=0)
                dev = float(_np.linalg.norm(mean_pos - _np.array(self._hold_reference)))
                if dev > self._HOLD_RELEASE_THRESH:
                    self._hold_active = False
                    self._hold_stationary_count = 0
                    self._buffer.clear()
                    self.get_logger().info(
                        f'[Pred HOLD OFF] Tay di chuyển (dev={dev*1000:.1f}mm). Thả hold, reset buffer.')
                else:
                    self._last_filtered = list(self._hold_target)
                    self._publish_prediction(
                        list(self._hold_target), 0.0, output_source='hold')
                    return
            else:
                if max_std < self._HOLD_STD_THRESH:
                    self._hold_stationary_count += 1
                else:
                    self._hold_stationary_count = 0

                if self._hold_stationary_count >= self._HOLD_ENTER_FRAMES:
                    self._hold_active = True
                    self._hold_reference = list(_np.mean(recent, axis=0))
                    if self._last_filtered is not None:
                        self._hold_target = list(self._last_filtered)
                    else:
                        self._hold_target = list(self._hold_reference)
                        
                    self.get_logger().info(
                        f'[Pred HOLD ON] Tay đứng yên (std={max_std*1000:.1f}mm). '
                        f'Target robot Z: {self._hold_target[2]:.4f} | Tay thật Z: {self._hold_reference[2]:.4f}')
                    self._publish_prediction(
                        list(self._hold_target), 0.0, output_source='hold')
                    return

    # ── Hybrid prediction+MJM methods ────────────────────────────────────────

    def _on_hybrid_cmd(self, msg: String):
        """Receive hybrid enable/disable commands from the dashboard."""
        if msg.data == 'hybrid_on':
            self._hybrid_enabled = True
            self._publish_hybrid_state()
            self.get_logger().info('[Hybrid] Mode ENABLED (prediction+MJM). T_SWITCH starts at Start Run.')
        elif msg.data == 'hybrid_off':
            self._hybrid_enabled = False
            self._hybrid_phase = 'FOLLOWER'
            self._mjm_trajectory = []
            self._mjm_step_idx = 0
            self._publish_hybrid_state()
            self.get_logger().info('[Hybrid] Mode DISABLED (prediction only)')

    def _trigger_leader_phase(self):
        """Switch to direct-MJM LEADER without a position-reference jump."""
        if self._require_goal_cmd and not self._goal_valid:
            self.get_logger().warn(
                '[Hybrid] LEADER blocked: capture a robot-EE target before Start Run',
                throttle_duration_sec=2.0)
            return
        if self._input_source == 'robot_ee':
            reference_age = time.monotonic() - self._limited_reference_time
            if (self._limited_reference is None
                    or reference_age > self._mjm_reference_timeout):
                self.get_logger().warn(
                    '[Hybrid] LEADER waiting for a fresh limited FOLLOWER '
                    'reference', throttle_duration_sec=1.0)
                return
            x_0 = leader_start_position(self._limited_reference)
        else:
            # Historical camera path: the last filtered model output was the
            # command-continuous switch point in camera-relative coordinates.
            x_0 = leader_start_position(
                self._x_switch if self._x_switch is not None else self._last_meas)
        # Invalidate learned-model responses only after a valid handoff reference is
        # available.  While waiting, FOLLOWER inference must continue.
        self._predict_epoch += 1
        t_f = fitts_law_duration(
            x_0, self._goal,
            a=self._fitts_a, b=self._fitts_b, w=self._fitts_w,
        )
        positions = minimum_jerk_positions(x_0, self._goal, t_f, self._mjm_dt)
        self._mjm_trajectory = positions.tolist()
        self._mjm_step_idx = 0
        self._hybrid_phase = 'LEADER'

        # Reset HOLD state để tránh chen vào MJM khi tay đang đứng yên tại điểm chuyển pha
        self._hold_active = False
        self._hold_stationary_count = 0
        self._hold_target = None
        self._hold_reference = None
        self._hold_recent.clear()

        self._publish_hybrid_state()
        # Publish x_MJM(0)=last limited FOLLOWER reference immediately.  The
        # controller accepts LEADER only together with an explicit MJM sample.
        self._mjm_tick()

        self.get_logger().info(
            f'[Hybrid] FOLLOWER -> LEADER | '
            f'x_0=({x_0[0]:.4f},{x_0[1]:.4f},{x_0[2]:.4f}) | '
            f'GOAL=({self._goal[0]:.4f},{self._goal[1]:.4f},{self._goal[2]:.4f}) | '
            f't_f={t_f:.2f}s | {len(positions)} steps @ {1.0/self._mjm_dt:.0f}Hz | '
            f'{str(self._current_model).upper()} worker dừng gửi lệnh predict '
            f'(tiết kiệm tài nguyên)'
        )

    def _mjm_tick(self):
        """Publish the next direct MJM point while in LEADER."""
        if (self._hybrid_phase != 'LEADER' or not self._predicting
                or self._external_control_hold):
            return
        if not self._mjm_trajectory:
            return
        if self._mjm_step_idx < len(self._mjm_trajectory):
            pos = self._mjm_trajectory[self._mjm_step_idx]
            self._publish_prediction(pos, 0.0, output_source='mjm')
            self._mjm_step_idx += 1
            
            # In ra log 1 lần duy nhất khi vừa chạm tới step cuối cùng
            if self._mjm_step_idx == len(self._mjm_trajectory):
                self.get_logger().info('✅ [MJM] ĐÃ CHẠM ĐÍCH (GOAL REACHED)!')
        else:
            # Hết trajectory → giữ robot tại GOAL
            self._publish_prediction(list(self._goal), 0.0, output_source='mjm')

    def _on_model_cmd(self, msg: String):
        """Nhận lệnh đổi model từ /predictor/model_cmd."""
        model_name = msg.data.strip().lower()
        if model_name not in self.model_files:
            self.get_logger().warn(
                f'[Predictor] Unknown model: {model_name}')
            return
        self.get_logger().info(f'[Predictor] Switching model → {model_name}')
        self._send_to_worker({'cmd': 'load_model', 'model_name': model_name})

    def _on_goal_cmd(self, msg: Point):
        """Cập nhật tọa độ Goal động từ UI."""
        if self._hybrid_phase == 'LEADER':
            self.get_logger().warn(
                '[Hybrid] Ignoring goal update during LEADER; stop the run first')
            return
        self._goal = np.array([msg.x, msg.y, msg.z])
        self._goal_valid = True
        self.get_logger().info(f'[Predictor] Dynamic Goal set to → ({msg.x:.4f}, {msg.y:.4f}, {msg.z:.4f})')

    def _srv_toggle(self, request, response):
        requested = bool(request.data)
        # Every explicit Start/Stop begins a fresh prediction epoch.  This
        # invalidates worker responses and HOLD targets from the previous run.
        self._reset_prediction_runtime()
        self._predicting = requested
        response.success = True
        response.message = 'Predicting STARTED' if self._predicting else 'Predicting STOPPED'
        if self._predicting:
            # Chờ frame data đầu tiên mới đếm T_SWITCH
            self._hybrid_start_time = 0.0
            self._hybrid_phase = 'FOLLOWER'
            self._x_switch = None
            self._mjm_trajectory = []
            self._mjm_step_idx = 0
        else:
            self._hybrid_phase = 'FOLLOWER'
        self._publish_hybrid_state()
        self.get_logger().info(f'[Predictor] {response.message}')
        return response

    def _reset_prediction_runtime(self):
        """Clear every run-scoped predictor, filter and HOLD state."""
        self._predict_epoch += 1
        self._buffer.clear()
        self._last_filtered = None
        self._filter_reject_count = 0
        self._inference_schedule.reset()
        self._last_hold_publish_time = 0.0
        self._last_data_time = 0.0
        self._smoothed_vel = [0.0, 0.0, 0.0]
        if hasattr(self, '_smoothed_dt'):
            del self._smoothed_dt
        self._hold_active = False
        self._hold_stationary_count = 0
        self._hold_target = None
        self._hold_reference = None
        self._hold_recent.clear()
        self._hold_detector.reset()
        self._external_control_hold = False
        self._limited_reference = None
        self._limited_reference_time = 0.0

    # ── Output Filter ────────────────────────────────────────────────────────

    def _filter_prediction(self, raw_pred: list) -> list:
        """3-layer output filter: proximity clamp → rate limiter → EMA.

        Layer 1 — Proximity Clamp:
            Giới hạn khoảng cách tối đa giữa dự đoán và vị trí thực tế.
            Nếu sai lệch trên bất kỳ trục nào vượt quá max_deviation,
            clamp giá trị đó lại gần vị trí đo được.

        Layer 2 — Rate Limiter:
            Giới hạn tốc độ thay đổi tối đa giữa 2 frame dự đoán liên tiếp.
            Ngăn hiện tượng nhảy đột ngột (jerk) khi model bị spike.

        Layer 3 — EMA Smoothing:
            Exponential Moving Average để làm mượt tín hiệu đầu ra.
            alpha nhỏ → mượt hơn nhưng trễ hơn.
        """
        filtered = [0.0, 0.0, 0.0]
        meas = self._last_meas
        max_dev = self._filter_max_dev
        max_rate = self._filter_max_rate
        alpha = self._filter_ema_alpha

        # ── Layer 1: Proximity clamp ─────────────────────────────────────
        for i in range(3):
            deviation = raw_pred[i] - meas[i]
            if abs(deviation) > max_dev:
                # Clamp: giữ hướng nhưng giới hạn biên độ
                clamped = meas[i] + max_dev * (1.0 if deviation > 0 else -1.0)
                filtered[i] = clamped
            else:
                filtered[i] = raw_pred[i]

        # ── Layer 2: Rate limiter ────────────────────────────────────────
        if self._last_filtered is not None:
            for i in range(3):
                delta = filtered[i] - self._last_filtered[i]
                if abs(delta) > max_rate:
                    filtered[i] = self._last_filtered[i] + max_rate * (1.0 if delta > 0 else -1.0)

        # ── Layer 3: EMA smoothing ───────────────────────────────────────
        if self._last_filtered is not None:
            for i in range(3):
                filtered[i] = alpha * filtered[i] + (1.0 - alpha) * self._last_filtered[i]

        self._last_filtered = filtered[:]
        return filtered

    # ── Publishing ───────────────────────────────────────────────────────────

    def _publish_prediction(
            self, pred: list, inf_ms: float, output_source: str | None = None):
        # Pha LEADER: bypass toàn bộ output filter — quỹ đạo MJM là pure math,
        # không bị ảnh hưởng bởi proximity clamp hay EMA từ camera.
        if output_source == 'hold':
            # HOLD is the measured position and must not inherit the previous
            # model EMA value; otherwise the robot can keep chasing stale x_d.
            filtered = pred
            self._last_filtered = list(pred)
        elif self._hybrid_enabled and self._hybrid_phase == 'LEADER':
            filtered = pred
        elif self._filter_enabled:
            filtered = self._filter_prediction(pred)
            # Periodic debug log (every 100 predictions)
            self._filter_log_counter += 1
            if self._filter_log_counter % 100 == 1:
                dx = abs(pred[0] - filtered[0])
                dy = abs(pred[1] - filtered[1])
                dz = abs(pred[2] - filtered[2])
                self.get_logger().info(
                    f'[Filter] raw=({pred[0]:.4f},{pred[1]:.4f},{pred[2]:.4f}) '
                    f'→ filt=({filtered[0]:.4f},{filtered[1]:.4f},{filtered[2]:.4f}) '
                    f'Δ=({dx:.4f},{dy:.4f},{dz:.4f})',
                    throttle_duration_sec=5.0
                )
        else:
            filtered = pred

        msg = HandPrediction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (
            'base_link' if self._input_source == 'robot_ee' else 'world')
        msg.x = float(filtered[0])
        msg.y = float(filtered[1])
        msg.z = float(filtered[2])
        msg.inference_time_ms = float(inf_ms)
        msg.model_name = output_source or self._current_model
        msg.prediction_confidence = 1.0
        msg.buffer_size = len(self._buffer)
        self.pred_pub.publish(msg)

    def _publish_raw_prediction(self, pred: list, inf_ms: float):
        """Publish the unfiltered worker result for diagnostics only."""
        msg = HandPrediction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (
            'base_link' if self._input_source == 'robot_ee' else 'world')
        msg.x, msg.y, msg.z = map(float, pred)
        msg.inference_time_ms = float(inf_ms)
        msg.model_name = f'{self._current_model}_raw'
        msg.prediction_confidence = 1.0
        msg.buffer_size = len(self._buffer)
        self.raw_pred_pub.publish(msg)

    def _publish_hybrid_state(self):
        if self._hybrid_enabled and self._predicting:
            state = self._hybrid_phase
        elif self._hybrid_enabled:
            state = 'READY'
        else:
            state = 'OFF'
        self.hybrid_state_pub.publish(String(data=state))

    def _publish_status(self):
        if hasattr(self, '_hybrid_enabled'):
            self._publish_hybrid_state()

        status = SystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.node_name = 'trajectory_predictor'
        status.fps = 0.0
        if self._worker_ready and self._predicting:
            status.status = 'ok'
            status.message = f'Predicting | model={self._current_model} | buf={len(self._buffer)}'
        elif self._worker_ready:
            status.status = 'idle'
            status.message = f'Ready | model={self._current_model} | NOT predicting'
        else:
            status.status = 'warning'
            status.message = 'Worker not ready'
        self.status_pub.publish(status)

    def _check_stale(self):
        """Xóa buffer nếu không nhận dữ liệu trong thời gian dài."""
        if self._last_data_time > 0:
            dt = time.time() - self._last_data_time
            if dt > self.clear_timeout and self._buffer:
                self._buffer.clear()
                # The first sample after a gap must use a zero displacement
                # feature, exactly like the first sample of a training trial.
                self._last_data_time = 0.0
                self._smoothed_vel = [0.0, 0.0, 0.0]
                if hasattr(self, '_smoothed_dt'):
                    del self._smoothed_dt
                self.get_logger().info(
                    f'[Predictor] Buffer cleared (no data for {dt:.1f}s)')

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        with self._worker_lock:
            proc = self._worker_proc
        if proc and proc.poll() is None:
            try:
                proc.stdin.write(json.dumps({'cmd': 'shutdown'}) + '\n')
                proc.stdin.flush()
                proc.wait(timeout=3.0)
            except Exception:
                proc.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
