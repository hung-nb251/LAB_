#!/usr/bin/env python3
"""
Predictor Node — ROS 2 node quản lý inference worker subprocess.

- Subscribe /hand_position (HandState) — tọa độ thô từ bridge.
- Duy trì sliding window 20 frames.
- Spawn inference_worker.py trong venv Python qua stdin/stdout JSON.
- Publish kết quả lên /ml/predicted_position (HandPrediction).
- Service /predictor/set_model để đổi model, /predictor/toggle để bật/tắt.

Hybrid GRU+MJM:
- Pha FOLLOWER (0→T_SWITCH s): GRU dự đoán bình thường.
- Pha LEADER  (T_SWITCH trở đi): MJM thay thế GRU, GRU worker nhàn rỗi
  (không gửi lệnh predict → tiết kiệm tài nguyên TF inference).
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
from std_msgs.msg import String
from std_srvs.srv import SetBool

from human_hand_msgs.msg import HandState, HandPrediction, SystemStatus

# Hybrid math utilities (Fitts' Law + Minimum Jerk Model)
try:
    from .hybrid_math import fitts_law_duration, minimum_jerk_positions
except ImportError:
    from hybrid_math import fitts_law_duration, minimum_jerk_positions


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
        self.declare_parameter('auto_start', False)
        self.declare_parameter('clear_on_tracking_lost', 1.0)
        self.declare_parameter('model_files.rnn', 'rnn_model_Ts3.h5')
        self.declare_parameter('model_files.gru', 'gru_model_Ts3.h5')
        self.declare_parameter('model_files.lstm', 'lstm_model_Ts3.h5')
        self.declare_parameter('model_files.svgp', 'svgp_model.pkl')
        # Path tới Python venv (nếu có), nếu không dùng sys.executable
        self.declare_parameter('venv_python', '')

        # ── Hybrid GRU+MJM parameters ────────────────────────────────────────
        self.declare_parameter('mjm.enabled', False)
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

        self.model_dir = self.get_parameter('model_dir').value
        self.default_model = self.get_parameter('default_model').value
        self.scaler_x_file = self.get_parameter('scaler_x_file').value
        self.scaler_y_file = self.get_parameter('scaler_y_file').value
        self.window_size = self.get_parameter('window_size').value
        self.num_features = self.get_parameter('num_features').value
        self.auto_start = self.get_parameter('auto_start').value
        self.clear_timeout = self.get_parameter('clear_on_tracking_lost').value
        venv_python = self.get_parameter('venv_python').value
        self.python_exe = venv_python if venv_python else sys.executable

        # Filter config
        self._filter_max_dev = self.get_parameter('filter.max_deviation').value
        self._filter_max_rate = self.get_parameter('filter.max_rate').value
        self._filter_ema_alpha = self.get_parameter('filter.ema_alpha').value
        self._filter_enabled = self.get_parameter('filter.enabled').value

        self.model_files = {
            'rnn': self.get_parameter('model_files.rnn').value,
            'gru': self.get_parameter('model_files.gru').value,
            'lstm': self.get_parameter('model_files.lstm').value,
            'svgp': self.get_parameter('model_files.svgp').value,
        }

        # ── Hybrid GRU+MJM config ────────────────────────────────────────────
        self._hybrid_enabled  = self.get_parameter('mjm.enabled').value
        self._t_switch        = self.get_parameter('mjm.t_switch').value
        self._goal            = np.array([
            self.get_parameter('mjm.goal_x').value,
            self.get_parameter('mjm.goal_y').value,
            self.get_parameter('mjm.goal_z').value,
        ])
        self._fitts_a         = self.get_parameter('mjm.fitts_a').value
        self._fitts_b         = self.get_parameter('mjm.fitts_b').value
        self._fitts_w         = self.get_parameter('mjm.fitts_w').value
        # dt = 1/30 s (nhất quán với Kinect ~30Hz)
        self._mjm_dt          = 1.0 / 30.0

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
        self._last_filtered = None              # last filtered prediction [x,y,z]
        self._filter_reject_count = 0
        
        # ── EMA velocity smoothing state ──
        self._smoothed_vel = [0.0, 0.0, 0.0]
        self._vel_ema_alpha = 0.3

        # ── Hybrid GRU+MJM state ─────────────────────────────────────────────
        # Pha FOLLOWER: GRU predict bình thường, ghi nhớ output cuối làm x_switch.
        # Pha LEADER:   Ngừng gửi lệnh predict cho GRU (tiết kiệm TF inference),
        #               thay thế bằng MJM timer 30Hz.
        self._hybrid_phase       = 'FOLLOWER'  # 'FOLLOWER' | 'LEADER'
        self._hybrid_start_time  = 0.0         # time.time() khi _predicting -> True
        self._x_switch           = None        # [x,y,z] — GRU output cuối pha FOLLOWER
        self._mjm_trajectory     = []          # list of [x,y,z]
        self._mjm_step_idx       = 0
        self._predict_epoch      = 0           # tăng khi chuyển LEADER → invalidate stale responses

        # ── Prediction Hold state ─────────────────────────────────────────────
        # Phát hiện khi tay đứng yên → khóa output prediction = vị trí tay
        # thay vì chạy GRU (loại bỏ hoàn toàn nhiễu dự đoán khi tay dừng)
        self._hold_recent = deque(maxlen=10)     # buffer vị trí gần nhất
        self._hold_stationary_count = 0          # đếm frame liên tiếp tĩnh
        self._hold_active = False                # đang khóa?
        self._hold_position = None               # vị trí khóa
        self._HOLD_STD_THRESH = 0.006            # ngưỡng std (6mm)
        self._HOLD_ENTER_FRAMES = 5              # cần 5 frame tĩnh liên tiếp
        self._HOLD_RELEASE_THRESH = 0.030        # tay rời > 30mm thì thả hold (chống nhiễu)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pred_pub = self.create_publisher(
            HandPrediction, '/ml/predicted_position', 10)
        self.status_pub = self.create_publisher(
            SystemStatus, '/ml/predictor_status', 10)
        self.hybrid_state_pub = self.create_publisher(
            String, '/predictor/hybrid_state', 5)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(HandState, '/hand_position', self._on_hand, 10)
        # Nếu bridge publish tọa độ thô (trước khi predict), cũng lắng nghe
        self.create_subscription(
            HandPrediction, '/predicted_position', self._on_bridge_data, 10)

        # Model switch command từ UI hay bridge
        self.create_subscription(String, '/predictor/model_cmd', self._on_model_cmd, 5)

        # Hybrid mode toggle từ UI (GRU+MJM button)
        self.create_subscription(String, '/predictor/hybrid_cmd', self._on_hybrid_cmd, 5)

        # Trajectory mode to automatically sync predicting state
        self.create_subscription(String, '/trajectory_mode', self._on_trajectory_mode, 10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(SetBool, '/predictor/toggle', self._srv_toggle)
        self.create_service(SetBool, '/predictor/set_model_srv', self._srv_toggle)

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_status)
        self.create_timer(self.clear_timeout, self._check_stale)
        # MJM tick timer: 30Hz (1/30s) — nhất quán với Kinect camera rate
        self.create_timer(self._mjm_dt, self._mjm_tick)

        # ── Launch inference worker ──────────────────────────────────────────
        self._launch_worker()

        self.get_logger().info(
            f'[Predictor] Started | model={self._current_model} | '
            f'window={self.window_size} | auto_start={self.auto_start}\n'
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
                    if resp_epoch == self._predict_epoch and self._hybrid_phase != 'LEADER':
                        self._x_switch = pred[:]
                        self._publish_prediction(pred, resp.get('inference_ms', 0.0))

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
        """Nhận tọa độ từ /hand_position (HandState)."""
        if not msg.is_tracked:
            return
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
        mode = msg.data
        if mode == 'prediction':
            if not self._predicting:
                self._predicting = True
                # Bắt đầu đếm T_SWITCH từ đây
                self._hybrid_start_time = time.time()
                self._hybrid_phase = 'FOLLOWER'
                self._x_switch = None
                self._mjm_trajectory = []
                self._mjm_step_idx = 0
                self._predict_epoch += 1  # invalidate stale responses từ run trước
                self.get_logger().info('[Predictor] Auto-enabled prediction mode via /trajectory_mode')
        else:
            if self._predicting:
                self._predicting = False
                self._hybrid_phase = 'FOLLOWER'
                self._buffer.clear()
                self._last_filtered = None
                self._filter_reject_count = 0
                self.get_logger().info('[Predictor] Auto-disabled prediction mode via /trajectory_mode')

    def _ingest_point(self, x: float, y: float, z: float):
        now = time.time()
        
        # Calculate velocity if we have a previous data point
        if self._last_data_time > 0:
            dt = now - self._last_data_time
            if dt > 0.001:
                if hasattr(self, '_smoothed_dt'):
                    self._smoothed_dt = 0.1 * dt + 0.9 * self._smoothed_dt
                else:
                    self._smoothed_dt = dt
                    
                true_vx = (x - self._last_meas[0]) / self._smoothed_dt
                true_vy = (y - self._last_meas[1]) / self._smoothed_dt
                true_vz = (z - self._last_meas[2]) / self._smoothed_dt
                
                raw_vx = true_vx / 16.0
                raw_vy = true_vy / 16.0
                raw_vz = true_vz / 16.0
            else:
                raw_vx, raw_vy, raw_vz = 0.0, 0.0, 0.0
        else:
            raw_vx, raw_vy, raw_vz = 0.0, 0.0, 0.0
            
        # Apply EMA smoothing to velocity
        self._smoothed_vel[0] = self._vel_ema_alpha * raw_vx + (1.0 - self._vel_ema_alpha) * self._smoothed_vel[0]
        self._smoothed_vel[1] = self._vel_ema_alpha * raw_vy + (1.0 - self._vel_ema_alpha) * self._smoothed_vel[1]
        self._smoothed_vel[2] = self._vel_ema_alpha * raw_vz + (1.0 - self._vel_ema_alpha) * self._smoothed_vel[2]
        vx, vy, vz = self._smoothed_vel
        
        self._last_data_time = now
        self._last_meas = [x, y, z]

        if self.num_features == 6:
            self._buffer.append([x, y, z, vx, vy, vz])
        else:
            self._buffer.append([x, y, z])

        # ── Prediction Hold: Disabled ──────────────────────────────────────

        # ── Kiểm tra chuyển pha FOLLOWER → LEADER ──────────────────────────
        # Chỉ kiểm tra khi đang FOLLOWER — guard chống gọi lại nhiều lần
        if self._hybrid_enabled and self._predicting and self._hybrid_phase == 'FOLLOWER':
            elapsed = time.time() - self._hybrid_start_time
            if elapsed >= self._t_switch:
                self._trigger_leader_phase()
                return  # MJM timer sẽ xử lý từ đây

        # ── Gửi dữ liệu cho GRU (chỉ khi KHÔNG hold và FOLLOWER) ──────────
        # Pha LEADER: KHÔNG gửi lệnh predict cho GRU → worker nhàn rỗi,
        # tiết kiệm tài nguyên TF inference. MJM timer đảm nhận việc publish.
        if not self._predicting or not self._worker_ready:
            return
        if self._hybrid_enabled and self._hybrid_phase == 'LEADER':
            return  # GRU đã nhàn rỗi — MJM timer lo phần còn lại

        if 0 < len(self._buffer) < self.window_size:
            first_point = list(self._buffer[0])
            pad_count = self.window_size - len(self._buffer)
            padded = [first_point] * pad_count + list(self._buffer)
        elif len(self._buffer) >= self.window_size:
            padded = list(self._buffer)
        else:
            return
        self._send_to_worker({'cmd': 'predict', 'data': padded, 'epoch': self._predict_epoch})

    # ── Hybrid GRU+MJM methods ───────────────────────────────────────────────

    def _on_hybrid_cmd(self, msg: String):
        """Nhận lệnh bật/tắt Hybrid mode từ /predictor/hybrid_cmd (UI button GRU+MJM)."""
        if msg.data == 'hybrid_on':
            self._hybrid_enabled = True
            self.get_logger().info('[Hybrid] Mode ENABLED (GRU+MJM). Timer T_SWITCH sẽ bắt đầu khi Start Run.')
        elif msg.data == 'hybrid_off':
            self._hybrid_enabled = False
            self._hybrid_phase = 'FOLLOWER'
            self._mjm_trajectory = []
            self._mjm_step_idx = 0
            self.get_logger().info('[Hybrid] Mode DISABLED (GRU only)')

    def _trigger_leader_phase(self):
        """Chuyển sang pha LEADER: tính MJM từ x_switch về GOAL."""
        # Tăng epoch → invalidate toàn bộ SVGP response đang trong pipe
        self._predict_epoch += 1
        self._hybrid_phase = 'LEADER'  # set sớm trước khi tính toán
        x_0 = np.array(self._x_switch if self._x_switch is not None else self._last_meas,
                        dtype=np.float64)
        t_f = fitts_law_duration(
            x_0, self._goal,
            a=self._fitts_a, b=self._fitts_b, w=self._fitts_w,
        )
        positions = minimum_jerk_positions(x_0, self._goal, t_f, self._mjm_dt)
        self._mjm_trajectory = positions.tolist()
        self._mjm_step_idx = 0

        # Reset HOLD state để tránh chen vào MJM khi tay đang đứng yên tại điểm chuyển pha
        self._hold_active = False
        self._hold_stationary_count = 0
        self._hold_position = None
        self._hold_recent.clear()

        self.get_logger().info(
            f'[Hybrid] FOLLOWER -> LEADER | '
            f'x_0=({x_0[0]:.4f},{x_0[1]:.4f},{x_0[2]:.4f}) | '
            f'GOAL=({self._goal[0]:.4f},{self._goal[1]:.4f},{self._goal[2]:.4f}) | '
            f't_f={t_f:.2f}s | {len(positions)} steps @ 30Hz | '
            f'GRU worker dừng gửi lệnh predict (tiết kiệm tài nguyên)'
        )

    def _mjm_tick(self):
        """Timer 30Hz: publish điểm MJM khi pha LEADER."""
        if self._hybrid_phase != 'LEADER' or not self._predicting:
            return
        if not self._mjm_trajectory:
            return
        if self._mjm_step_idx < len(self._mjm_trajectory):
            pos = self._mjm_trajectory[self._mjm_step_idx]
            self._publish_prediction(pos, 0.0)
            self._mjm_step_idx += 1
        else:
            # Hết trajectory → giữ robot tại GOAL
            self._publish_prediction(list(self._goal), 0.0)

    def _on_model_cmd(self, msg: String):
        """Nhận lệnh đổi model từ /predictor/model_cmd."""
        model_name = msg.data.strip().lower()
        if model_name not in self.model_files:
            self.get_logger().warn(
                f'[Predictor] Unknown model: {model_name}')
            return
        self.get_logger().info(f'[Predictor] Switching model → {model_name}')
        self._send_to_worker({'cmd': 'load_model', 'model_name': model_name})

    def _srv_toggle(self, request, response):
        self._predicting = request.data
        response.success = True
        response.message = 'Predicting STARTED' if self._predicting else 'Predicting STOPPED'
        if self._predicting:
            # Bắt đầu đếm T_SWITCH từ đây (khi Start Run)
            self._hybrid_start_time = time.time()
            self._hybrid_phase = 'FOLLOWER'
            self._x_switch = None
            self._mjm_trajectory = []
            self._mjm_step_idx = 0
            self._predict_epoch += 1  # invalidate stale responses từ run trước
        else:
            self._hybrid_phase = 'FOLLOWER'
            self._buffer.clear()
            self._last_filtered = None
            self._filter_reject_count = 0
        self.get_logger().info(f'[Predictor] {response.message}')
        return response

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

    def _publish_prediction(self, pred: list, inf_ms: float):
        # Pha LEADER: bypass toàn bộ output filter — quỹ đạo MJM là pure math,
        # không bị ảnh hưởng bởi proximity clamp hay EMA từ camera.
        if self._hybrid_enabled and self._hybrid_phase == 'LEADER':
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
        # Pha LEADER: tọa độ MJM đã ở base_link frame, báo cho transform_node
        # biết để bypass camera filter và TARGET SNAP, đi thẳng tới robot.
        if self._hybrid_enabled and self._hybrid_phase == 'LEADER':
            msg.header.frame_id = 'base_link'
        else:
            msg.header.frame_id = 'world'
        msg.x = float(filtered[0])
        msg.y = float(filtered[1])
        msg.z = float(filtered[2])
        msg.inference_time_ms = float(inf_ms)
        msg.model_name = self._current_model
        msg.prediction_confidence = 1.0
        msg.buffer_size = len(self._buffer)
        self.pred_pub.publish(msg)

    def _publish_status(self):
        if hasattr(self, '_hybrid_enabled'):
            if self._hybrid_enabled and self._predicting:
                self.hybrid_state_pub.publish(String(data=self._hybrid_phase))
            elif self._hybrid_enabled:
                self.hybrid_state_pub.publish(String(data='READY'))
            else:
                self.hybrid_state_pub.publish(String(data='OFF'))

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
