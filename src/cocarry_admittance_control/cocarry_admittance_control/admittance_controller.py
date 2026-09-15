#!/usr/bin/env python3
"""3D admittance controller using robot-EE history and an Axia force sensor."""

from __future__ import annotations

import time
import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from human_hand_msgs.msg import HandPrediction, HandState
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger
from .prediction_reference import PredictionReference
from .manual_hybrid import ManualHybrid

from .admittance import (
    CartesianAdmittance,
    commanded_position,
    critical_damping,
    fresh_mjm_sample,
    force_watchdog_action,
    limit_position_lead,
    nominal_reference,
    soft_axis_deadzone,
    soft_radial_deadzone,
)


class AdmittanceController3D(Node):
    def __init__(self):
        super().__init__('cocarry_admittance_controller')

        defaults = {
            'control_rate_hz': 15.0,
            'virtual_mass_x': 1.0, 'virtual_mass_y': 1.0, 'virtual_mass_z': 1.0,
            'stiffness_x': 5.0, 'stiffness_y': 5.0, 'stiffness_z': 5.0,
            'critical_damping': True,
            'damping_x': 4.47213595, 'damping_y': 4.47213595,
            'damping_z': 4.47213595,
            # Axia UI already applies the 4 N radial deadband.
            'intent_threshold_n': 0.0,
            # Extra Z-only soft deadzone after Axia's 4 N radial deadband.
            'additional_z_deadzone_n': 2.0,
            'xminus_z_deadzone_force_enter_n': 0.50,
            'xminus_z_deadzone_force_exit_n': 0.20,
            'xminus_z_deadzone_velocity_enter_mps': 0.020,
            'xminus_z_deadzone_velocity_exit_mps': 0.010,
            'xminus_z_deadzone_ramp_sec': 0.25,
            'xminus_z_deadzone_release_dwell_sec': 0.30,
            'force_sign_x': 1.0, 'force_sign_y': 1.0, 'force_sign_z': 1.0,
            'max_virtual_velocity_mps': 0.15,
            'max_virtual_acceleration_mps2': 0.50,
            # Chặn reference chạy quá xa feedback thật khi IK/joint limiter
            # bám chậm. Đây là anti-windup, không phải workspace limit.
            'max_command_lead_m': 0.03,
            'safe_workspace_x_min_m': -1.4,
            'safe_workspace_x_max_m': 1.4,
            'safe_workspace_y_min_m': -0.5,
            'safe_workspace_y_max_m': 1.3,
            # Explicit EE workspace bound in base_link.  The current tool is
            # horizontal, so do not derive this from the old downward rod.
            'safe_workspace_z_min_m': 0.0314,
            'safe_workspace_z_max_m': 1.5,
            'max_force_per_axis_n': 20.0,
            'max_force_norm_n': 30.0,
            'force_stale_hold_sec': 0.20,
            'force_timeout_sec': 0.50,
            'current_pose_stale_hold_sec': 0.25,
            'current_pose_timeout_sec': 0.50,
            'prediction_timeout_sec': 0.50,
            # Keep the AI nominal close enough to measured EE for IK/tracking
            # safety.  This is a geometric bound, not a force/intention gate.
            'prediction_max_nominal_lead_m': 0.05,
            # Co-carry launch/YAML opts in; standalone default is passthrough.
            'prediction_reference_tau_sec': 0.0,
            'prediction_reference_lead_sec': 0.0,
            'prepare_timeout_sec': 8.0,
            'min_prediction_buffer_size': 10,
            'require_axia_calibrated': True,
            'hybrid_target_file': '',
            'hybrid_arrival_tolerance_m': 0.01,
            'hybrid_arrival_speed_mps': 0.02,
            'hybrid_arrival_dwell_sec': 0.5,
            'hybrid_arrival_grace_sec': 5.0,
            'hybrid_fitts_a': 3.0,
            'hybrid_fitts_b': 0.7492,
            'hybrid_fitts_w': 0.3,
            'hybrid_reentry_blend_sec': 0.6,
            'hybrid_reentry_warmup_samples': 10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        gp = lambda name: self.get_parameter(name).value

        self._rate = float(gp('control_rate_hz'))
        mass = np.array([gp('virtual_mass_x'), gp('virtual_mass_y'), gp('virtual_mass_z')], dtype=float)
        stiffness = np.array([gp('stiffness_x'), gp('stiffness_y'), gp('stiffness_z')], dtype=float)
        manual_damping = np.array([gp('damping_x'), gp('damping_y'), gp('damping_z')], dtype=float)
        self._critical_damping = bool(gp('critical_damping'))
        damping = critical_damping(mass, stiffness) if self._critical_damping else manual_damping
        if np.any(damping <= 0.0):
            raise ValueError(
                'Every admittance axis needs positive damping. For K=0, '
                'critical damping is undefined; set critical_damping=false '
                'and provide damping_x/y/z > 0.')
        self._admittance = CartesianAdmittance(
            mass, damping, stiffness,
            gp('max_virtual_velocity_mps'), gp('max_virtual_acceleration_mps2'))
        # Release FOLLOWER: keep inertia/damping, but no spring to a GRU/home anchor.
        self._free_admittance = CartesianAdmittance(
            mass, damping, np.zeros(3),
            gp('max_virtual_velocity_mps'), gp('max_virtual_acceleration_mps2'))
        self._free_origin = np.zeros(3)
        self._emitted_position = None
        self._emitted_velocity = np.zeros(3)
        self._emitted_acceleration = np.zeros(3)
        self._emitted_time = None
        self._reentry_token = self._last_reentry_token = 0
        self._reentry_samples = self._reentry_input_stamp = 0
        self._reentry_phase = 'OFF'
        self._reentry_elapsed = 0.
        self._reentry_blend_sec = float(gp('hybrid_reentry_blend_sec'))
        if not np.isfinite(self._reentry_blend_sec) or self._reentry_blend_sec <= 0:
            raise ValueError('hybrid_reentry_blend_sec must be positive')
        self._reentry_warmup_samples = int(gp('hybrid_reentry_warmup_samples'))
        if self._reentry_warmup_samples <= 0:
            raise ValueError('hybrid_reentry_warmup_samples must be positive')
        self._reentry_align = False
        self._force_sign = np.array([
            1.0 if float(gp('force_sign_x')) >= 0.0 else -1.0,
            1.0 if float(gp('force_sign_y')) >= 0.0 else -1.0,
            1.0 if float(gp('force_sign_z')) >= 0.0 else -1.0,
        ])
        self._intent_threshold = float(gp('intent_threshold_n'))
        self._additional_z_deadzone = float(gp('additional_z_deadzone_n'))
        if (not np.isfinite(self._additional_z_deadzone)
                or self._additional_z_deadzone < 0.0):
            raise ValueError('additional_z_deadzone_n must be finite and non-negative')
        self._xminus_force_enter = float(gp('xminus_z_deadzone_force_enter_n'))
        self._xminus_force_exit = float(gp('xminus_z_deadzone_force_exit_n'))
        self._xminus_velocity_enter = float(
            gp('xminus_z_deadzone_velocity_enter_mps'))
        self._xminus_velocity_exit = float(
            gp('xminus_z_deadzone_velocity_exit_mps'))
        self._xminus_deadzone_ramp = float(gp('xminus_z_deadzone_ramp_sec'))
        self._xminus_release_dwell = float(
            gp('xminus_z_deadzone_release_dwell_sec'))
        if (not 0.0 <= self._xminus_force_exit < self._xminus_force_enter
                or not 0.0 <= self._xminus_velocity_exit < self._xminus_velocity_enter
                or self._xminus_deadzone_ramp <= 0.0
                or self._xminus_release_dwell < 0.0):
            raise ValueError('Invalid adaptive X- Z-deadzone thresholds')
        self._max_command_lead = float(gp('max_command_lead_m'))
        if self._max_command_lead <= 0.0:
            raise ValueError('max_command_lead_m must be positive')
        self._workspace_min = np.array([
            gp('safe_workspace_x_min_m'), gp('safe_workspace_y_min_m'),
            gp('safe_workspace_z_min_m')], dtype=float)
        self._workspace_max = np.array([
            gp('safe_workspace_x_max_m'), gp('safe_workspace_y_max_m'),
            gp('safe_workspace_z_max_m')], dtype=float)
        if np.any(self._workspace_min >= self._workspace_max):
            raise ValueError('Invalid Cartesian workspace bounds')
        self._max_force_axis = float(gp('max_force_per_axis_n'))
        self._max_force_norm = float(gp('max_force_norm_n'))
        self._force_stale_hold = float(gp('force_stale_hold_sec'))
        self._force_timeout = float(gp('force_timeout_sec'))
        if not 0.0 < self._force_stale_hold < self._force_timeout:
            raise ValueError(
                'force_stale_hold_sec must be positive and less than force_timeout_sec')
        self._pose_stale_hold = float(gp('current_pose_stale_hold_sec'))
        self._pose_timeout = float(gp('current_pose_timeout_sec'))
        if not 0.0 < self._pose_stale_hold < self._pose_timeout:
            raise ValueError(
                'current_pose_stale_hold_sec must be positive and less than '
                'current_pose_timeout_sec')
        self._prediction_timeout = float(gp('prediction_timeout_sec'))
        self._prediction_reference = PredictionReference(
            gp('prediction_reference_tau_sec'),
            lead_sec=gp('prediction_reference_lead_sec'))
        self._prediction_max_nominal_lead = float(
            gp('prediction_max_nominal_lead_m'))
        if self._prediction_max_nominal_lead <= 0.0:
            raise ValueError('prediction_max_nominal_lead_m must be positive')
        self._prepare_timeout = float(gp('prepare_timeout_sec'))
        self._min_prediction_buffer = int(gp('min_prediction_buffer_size'))
        self._require_calibrated = bool(gp('require_axia_calibrated'))

        self._state = 'STOPPED'
        self._fault = ''
        self._mode = 'ground_truth'
        self._axia_calibrated = False
        self._axia_connected = False
        self._streamer_ready = False
        self._current_pose = None
        self._current_pose_time = 0.0
        self._capture_pose = None
        self._capture_ee = None
        self._force = np.zeros(3)
        self._force_time = 0.0
        self._robot_ee = None
        self._robot_ee_time = 0.0
        self._prediction = None
        self._prediction_time = 0.0
        self._prediction_buffer_size = 0
        self._prediction_source = ''
        self._raw_prediction = None
        self._raw_prediction_time = 0.0
        self._hybrid_state = 'OFF'
        self._requested_hybrid_state = 'OFF'
        self._leader_pending = False
        # UI callbacks only latch a manual LEADER request.  The control timer
        # consumes it from the same state snapshot used by automatic Test mode,
        # avoiding a one-sample handoff between two controller ticks.
        self._manual_leader_pending = False
        self._pending_mjm = None
        self._role_change_time = 0.0
        self._follower_realign = False
        self._calibration_epoch = 0.0
        self._prepare_deadline = 0.0
        self._aligned = False
        self._workspace_clamped = False
        self._command_lead_limited = False
        self._force_stale_active = False
        self._pose_stale_active = False
        self._force_recovery_pending = False
        self._force_stale_nominal = None
        self._xminus_z_suppression = False
        self._xminus_release_since = None
        self._z_deadzone_weight = 0.0
        self._last_tick = time.monotonic()
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        target_file = gp('hybrid_target_file') or (
            f'/home/hungnb/cocarry_ws/config/hybrid_targets_domain_{domain}.json')
        self._manual = ManualHybrid(
            target_file, self._workspace_min, self._workspace_max,
            gp('hybrid_arrival_tolerance_m'), gp('hybrid_arrival_speed_mps'),
            gp('hybrid_arrival_dwell_sec'), gp('hybrid_arrival_grace_sec'),
            gp('max_virtual_velocity_mps'), gp('max_virtual_acceleration_mps2'),
            gp('hybrid_fitts_a'), gp('hybrid_fitts_b'), gp('hybrid_fitts_w'))
        self._ee_speed = float('inf')
        self._follower_prediction_after_ns = 0
        self._manual_pub = self.create_publisher(String, '/cocarry/hybrid_status', 10)
        self.create_subscription(String, '/cocarry/hybrid_command', self._manual_command, 10)
        self.create_subscription(String, '/ml/reentry_prediction', self._on_reentry_prediction, 10)

        self._target_pub = self.create_publisher(PoseStamped, '/cartesian_streamer/target_pose', 10)
        self._target_base_pub = self.create_publisher(PointStamped, '/coord_transform/target_base', 10)
        self._filtered_pub = self.create_publisher(PointStamped, '/coord_transform/filtered_hand_position', 10)
        self._nominal_pub = self.create_publisher(PointStamped, '/cocarry/nominal_position', 10)
        self._reference_pub = self.create_publisher(PointStamped, '/cocarry/reference_position', 10)
        self._relative_reference_pub = self.create_publisher(
            PointStamped, '/cocarry/reference_relative', 10)
        self._error_pub = self.create_publisher(Vector3Stamped, '/cocarry/admittance_error', 10)
        self._effective_force_pub = self.create_publisher(
            Vector3Stamped, '/cocarry/effective_force', 10)
        self._z_deadzone_weight_pub = self.create_publisher(
            Float32, '/cocarry/z_deadzone_weight', 10)
        self._status_pub = self.create_publisher(String, '/cocarry/status', 10)
        self._force_age_pub = self.create_publisher(Float32, '/cocarry/force_age_ms', 10)
        self._pose_age_pub = self.create_publisher(Float32, '/cocarry/pose_age_ms', 10)
        self._prediction_age_pub = self.create_publisher(
            Float32, '/cocarry/prediction_age_ms', 10)
        self._raw_nominal_pub = self.create_publisher(
            PointStamped, '/cocarry/raw_prediction_nominal', 10)
        self._control_hold_pub = self.create_publisher(
            Bool, '/cocarry/control_hold', 5)
        self._run_status_pub = self.create_publisher(Bool, '/run_status', 5)

        self.create_subscription(PoseStamped, '/cartesian_streamer/current_pose', self._on_current_pose, 10)
        self.create_subscription(Bool, '/cartesian_streamer/ready', self._on_streamer_ready, 5)
        self.create_subscription(HandState, '/hand_position', self._on_robot_ee, 20)
        self.create_subscription(HandPrediction, '/ml/predicted_position', self._on_prediction, 10)
        self.create_subscription(
            HandPrediction, '/ml/raw_predicted_position', self._on_raw_prediction, 10)
        self.create_subscription(Vector3Stamped, '/axia/human_force', self._on_force, 20)
        self.create_subscription(Bool, '/axia/calibrated', self._on_calibrated, 5)
        self.create_subscription(Bool, '/axia/connected', self._on_connected, 5)
        self.create_subscription(Bool, '/run_status', self._on_run_status, 5)
        self.create_subscription(String, '/trajectory_mode', self._on_mode, 5)
        self.create_subscription(String, '/predictor/hybrid_state', self._on_hybrid_state, 10)

        self._ee_calibrate_client = self.create_client(Trigger, '/coord_transform/calibrate')
        self._predictor_toggle_client = self.create_client(SetBool, '/predictor/toggle')
        self._streamer_enable_client = self.create_client(SetBool, '/cartesian_streamer/enable')
        self.create_service(Trigger, '/realsense/calibrate_origin', self._on_ui_calibrate)

        self.create_timer(1.0 / self._rate, self._control_tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            'Independent 3D co-carrying admittance ready (robot_ee, no camera). '
            'Law: M*e_ddot + D*e_dot + K*e = F_h. '
            'FOLLOWER uses the selected predictor + admittance; '
            'LEADER uses direct MJM points. '
            f'M={mass.tolist()}, D={damping.round(6).tolist()}, K={stiffness.tolist()}, '
            f'vmax={self._admittance.max_velocity:.3f} m/s, EE Z workspace='
            f'[{self._workspace_min[2]:.4f}, {self._workspace_max[2]:.2f}] m '
            f'(explicit EE Z bound in base_link), '
            f'force stale/hard={self._force_stale_hold:.2f}/{self._force_timeout:.2f} s, '
            f'Predictor nominal lead={self._prediction_max_nominal_lead:.3f} m '
            '(no force gate), '
            f'nominal reference tau={self._prediction_reference.time_constant:.3f} s, '
            f'lead compensation={self._prediction_reference.lead_sec:.3f} s '
            f'(cap={self._prediction_reference.max_lead_m:.3f} m)')

    def _on_current_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        if not np.all(np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w])):
            self._current_pose_time = 0.
            if self._state in ('PREPARING', 'RUNNING'):
                self._set_fault('Non-finite EE pose')
            return
        now = time.monotonic()
        if self._current_pose is not None and now > self._current_pose_time:
            p, prev = msg.pose.position, self._current_pose.position
            self._ee_speed = float(np.linalg.norm(
                [p.x-prev.x, p.y-prev.y, p.z-prev.z]) / (now-self._current_pose_time))
        self._current_pose = msg.pose
        self._current_pose_time = now

    def _manual_status(self):
        data = self._manual.snapshot()
        data['controller_state'] = self._state
        # Derived diagnostic phase: do not introduce another mutable state machine.
        phase = self._state
        if self._state == 'RUNNING':
            if self._force_stale_active:
                phase = 'FORCE_HOLD'
            elif self._manual_leader_pending:
                phase = 'FOLLOWER_TO_LEADER_PENDING'
            elif self._mode != 'prediction':
                phase = 'GROUND_TRUTH'
            elif self._manual.enabled and self._manual.role == 'LEADER':
                phase = 'LEADER_BRIDGE' if data['transition'] else 'LEADER_MJM'
            elif self._manual.enabled and self._reentry_phase == 'WAIT':
                phase = 'FOLLOWER_FORCE_WARMUP'
            elif self._manual.enabled and self._reentry_phase == 'BLEND':
                phase = 'FOLLOWER_REENTRY_BLEND'
            else:
                phase = 'FOLLOWER_PREDICTION'
        data['control_phase'] = phase
        data['leader_request_pending'] = self._manual_leader_pending
        data['stamp_ns'] = self.get_clock().now().nanoseconds
        data.update(reentry_token=self._reentry_token, reentry_samples=self._reentry_samples,
                    reentry_required_samples=self._reentry_warmup_samples, reentry_phase=self._reentry_phase,
                    reentry_elapsed=self._reentry_elapsed, reentry_blend_sec=self._reentry_blend_sec)
        if self._manual.enabled and self._manual.role == 'FOLLOWER':
            if self._reentry_phase == 'WAIT':
                data['control_source'] = 'force_admittance_warmup'
            elif self._reentry_phase == 'BLEND':
                data['control_source'] = 'prediction_reentry_blend'
                data['transition'] = 'FORCE_TO_PREDICTION'
        self._manual_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def _manual_follow(self, reason):
        self._manual_leader_pending = False
        self._reset_adaptive_z_deadzone()
        if self._manual.follow(reason):
            self._hybrid_state = self._requested_hybrid_state = 'FOLLOWER'
            self._prediction = None
            self._prediction_source = ''
            self._role_change_time = time.monotonic()
            self._follower_prediction_after_ns = self.get_clock().now().nanoseconds
            self._follower_realign = False
            p = self._current_pose.position
            self._free_origin = (np.array([p.x, p.y, p.z]) if self._emitted_position is None
                                 else self._emitted_position.copy())
            self._free_admittance.reset()
            self._free_admittance.error_velocity[:] = self._emitted_velocity
            if reason in ('reached', 'skipped_by_user'):
                self._begin_reentry_wait()
            else:
                self._reentry_token, self._reentry_phase = 0, 'OFF'

    def _begin_reentry_wait(self):
        self._reentry_token = max(self.get_clock().now().nanoseconds, self._last_reentry_token+1)
        self._last_reentry_token = self._reentry_token
        self._reentry_samples = self._reentry_input_stamp = 0
        self._reentry_phase, self._reentry_elapsed = 'WAIT', 0.
        self._prediction, self._prediction_source = None, ''
        self._manual.force_follower = True

    def _on_reentry_prediction(self, msg):
        if (not self._manual.enabled or self._manual.role != 'FOLLOWER'
                or self._state != 'RUNNING' or not self._reentry_token):
            return
        try:
            data = json.loads(msg.data)
            token, samples, stamp = int(data['token']), int(data['samples']), int(data['input_stamp_ns'])
            prediction = np.asarray(data['prediction'], dtype=float)
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            source = str(data['model']).lower()
            if (token != self._reentry_token or samples < 1
                    or stamp <= max(token, self._reentry_input_stamp)
                    or not 0 <= age <= self._prediction_timeout
                    or prediction.shape != (3,) or not np.all(np.isfinite(prediction))
                    or source not in ('gru', 'svgp')):
                return
        except (KeyError, TypeError, ValueError):
            return
        self._reentry_samples = samples
        self._reentry_input_stamp = stamp
        if samples < self._reentry_warmup_samples:
            return
        self._prediction, self._prediction_source = prediction, source
        # Age refers to the model INPUT, not the late worker's publication time.
        self._prediction_time = time.monotonic()-age
        self._prediction_buffer_size = self._reentry_warmup_samples

    def _manual_command(self, msg):
        try:
            command = json.loads(msg.data)
            action = command['action']
            busy = self._state in ('PREPARING', 'RUNNING')
            now = time.monotonic()
            if action == 'mode':
                if busy:
                    raise ValueError('Stop Run before changing mode')
                if type(command['enabled']) is not bool:
                    raise ValueError('enabled must be boolean')
                if type(command.get('test_mode', False)) is not bool:
                    raise ValueError('test_mode must be boolean')
                self._manual.enabled = command['enabled']
                self._manual.test_mode = self._manual.enabled and command.get('test_mode', False)
                self._manual.reset_run('mode_changed')
                self._reentry_token, self._reentry_phase = 0, 'OFF'
            elif action == 'reset':
                self._manual.reset_targets(busy)
            elif action == 'save':
                if self._current_pose is None or now-self._current_pose_time > self._pose_timeout:
                    raise ValueError('No fresh EE pose')
                if self._ee_speed > self._manual.speed:
                    raise ValueError('Wait for robot to stop before saving')
                p = self._current_pose.position
                self._manual.save(command['target'], [p.x, p.y, p.z], busy)
            elif action == 'select':
                if self._state != 'RUNNING' and not (self._manual.test_mode and not busy):
                    raise ValueError('Select Target after Start Run is RUNNING')
                if self._manual.test_mode and busy:
                    raise ValueError('Select test target before Start Run')
                self._manual.select(command['target'])
            elif action == 'follower':
                if not self._manual.enabled or self._state != 'RUNNING':
                    raise ValueError('Hybrid must be running')
                self._manual_follow('skipped_by_user')
            elif action == 'leader':
                if (not self._manual.enabled or self._state != 'RUNNING'
                        or self._mode != 'prediction'):
                    raise ValueError('Hybrid must be running')
                if (len(self._manual.targets) != 2
                        or self._manual.selected is None):
                    raise ValueError(
                        'Hybrid requires two saved targets and an explicit selection')
                self._manual_leader_pending = True
                self._manual.reason = 'leader_queued'
            else:
                raise ValueError('Unknown Hybrid action')
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self._manual.reason = f'rejected: {exc}'
            self.get_logger().warn(self._manual.reason)
        self._manual_status()

    def _activate_manual_leader_on_tick(self, now, actual_ee):
        """Consume a UI/Test LEADER request on the control timer boundary."""
        if not self._manual_leader_pending:
            return
        self._manual_leader_pending = False
        try:
            if (now-self._current_pose_time > self._pose_stale_hold
                    or now-self._force_time > self._force_stale_hold
                    or not self._axia_connected or not self._axia_calibrated
                    or not self._streamer_ready or self._force_stale_active
                    or self._pose_stale_active
                    or np.any(np.abs(self._force) > self._max_force_axis)
                    or np.linalg.norm(self._force) > self._max_force_norm):
                raise ValueError('Fresh pose/force and robot readiness required')
            if self._manual.lead(actual_ee, self._emitted_position,
                                 self._emitted_velocity,
                                 self._emitted_acceleration):
                self._hybrid_state = self._requested_hybrid_state = 'LEADER'
                self._admittance.reset()
                self._follower_realign = False
                self._manual.test_fired = True
                self._reentry_token, self._reentry_phase = 0, 'OFF'
        except (ValueError, TypeError) as exc:
            self._manual.reason = f'rejected: {exc}'
            self.get_logger().warn(self._manual.reason)

    def _on_streamer_ready(self, msg):
        was_ready = self._streamer_ready
        self._streamer_ready = bool(msg.data)
        if was_ready and not self._streamer_ready and self._state in ('PREPARING', 'RUNNING'):
            self._set_fault('Cartesian streamer lost readiness')

    def _on_robot_ee(self, msg):
        if not msg.is_tracked or msg.source != 'robot_ee':
            return
        if not np.all(np.isfinite([msg.x, msg.y, msg.z])):
            self._robot_ee_time = 0.
            if self._state in ('PREPARING', 'RUNNING'):
                self._set_fault('Non-finite robot EE input')
            return
        self._robot_ee = np.array([msg.x, msg.y, msg.z], dtype=float)
        self._robot_ee_time = time.monotonic()
        out = PointStamped()
        out.header = msg.header
        out.header.frame_id = 'base_link'
        out.point.x, out.point.y, out.point.z = map(float, self._robot_ee)
        self._filtered_pub.publish(out)

    def _on_prediction(self, msg):
        prediction = np.array([msg.x, msg.y, msg.z], dtype=float)
        received_at = time.monotonic()
        buffer_size = int(msg.buffer_size)
        source = str(msg.model_name).strip().lower()
        if self._manual.enabled:
            # Manual MJM is controller-owned; predictor remains learned-only.
            if source == 'mjm' or self._manual.role == 'LEADER':
                return
            if self._reentry_token:
                return  # Only window-qualified results are accepted after release.
            if self._follower_realign:
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if stamp <= self._follower_prediction_after_ns:
                    return

        # DDS does not guarantee ordering across hybrid_state and prediction
        # topics.  Keep an early MJM point separate so FOLLOWER can continue
        # using its last learned-predictor sample until the LEADER request arrives.
        if source == 'mjm' and self._requested_hybrid_state != 'LEADER':
            if np.all(np.isfinite(prediction)):
                self._pending_mjm = (prediction, received_at, buffer_size)
            return
        if self._requested_hybrid_state == 'LEADER' and source != 'mjm':
            return

        if not np.all(np.isfinite(prediction)):
            self._prediction, self._prediction_time = None, 0.
            if self._state in ('PREPARING', 'RUNNING'):
                self._set_fault('Non-finite prediction')
            return
        self._prediction = prediction
        self._prediction_time = received_at
        self._prediction_buffer_size = buffer_size
        self._prediction_source = source
        if self._leader_pending and source == 'mjm':
            self._activate_leader()

    def _on_raw_prediction(self, msg):
        self._raw_prediction = np.array([msg.x, msg.y, msg.z], dtype=float)
        self._raw_prediction_time = time.monotonic()
        if self._capture_ee is None or self._mode != 'prediction':
            return
        raw_nominal = self._capture_ee + self._raw_prediction
        out = PointStamped()
        out.header = msg.header
        out.header.frame_id = 'base_link'
        out.point.x, out.point.y, out.point.z = map(float, raw_nominal)
        self._raw_nominal_pub.publish(out)

    def _on_hybrid_state(self, msg):
        if self._manual.enabled:
            return
        new_state = msg.data.strip().upper()
        if new_state not in ('OFF', 'READY', 'FOLLOWER', 'LEADER'):
            self.get_logger().warn(f'Ignoring unsupported hybrid state: {new_state}')
            return
        if new_state == self._requested_hybrid_state:
            return
        self._requested_hybrid_state = new_state
        self._role_change_time = time.monotonic()
        if self._state != 'RUNNING':
            self._hybrid_state = new_state
            self._leader_pending = False
            self._pending_mjm = None
            return
        if new_state == 'LEADER':
            if self._mode != 'prediction':
                self._set_fault('LEADER requested outside Prediction mode')
                return
            if self._pending_mjm is not None:
                prediction, received_at, buffer_size = self._pending_mjm
                if fresh_mjm_sample(
                        'mjm', received_at, self._role_change_time,
                        self._prediction_timeout):
                    self._prediction = prediction
                    self._prediction_time = received_at
                    self._prediction_buffer_size = buffer_size
                    self._prediction_source = 'mjm'
                    self._pending_mjm = None
                    self._activate_leader()
                    return
            self._leader_pending = True
            self.get_logger().info(
                'LEADER requested; continuing FOLLOWER until the first MJM '
                'sample arrives')
            return

        old_state = self._hybrid_state
        self._leader_pending = False
        self._manual_leader_pending = False
        self._pending_mjm = None
        self._hybrid_state = new_state
        if old_state == 'LEADER':
            # Wait for a fresh learned nominal, then align e so FOLLOWER restarts
            # exactly at the measured EE pose.
            self._prediction = None
            self._prediction_source = ''
            self._follower_realign = True
            self.get_logger().info('Role switched to FOLLOWER: waiting to realign admittance')

    def _activate_leader(self):
        """Atomically promote a fresh MJM sample to the active controller role."""
        self._hybrid_state = 'LEADER'
        self._leader_pending = False
        self._pending_mjm = None
        self._role_change_time = time.monotonic()
        self._admittance.reset()
        self._follower_realign = False
        self.get_logger().info(
            'Role switched to LEADER with a fresh MJM sample: direct position control')

    def _on_force(self, msg):
        if not np.all(np.isfinite([msg.vector.x, msg.vector.y, msg.vector.z])):
            self._force_time = 0.
            if self._state in ('PREPARING', 'RUNNING'):
                self._set_fault('Non-finite force')
            return
        self._force[:] = [msg.vector.x, msg.vector.y, msg.vector.z]
        self._force_time = time.monotonic()
        if self._force_stale_active:
            self._force_recovery_pending = True

    def _effective_force_input(self, now, dt):
        """Return and publish the force that the admittance actually consumes."""
        force = self._force_sign * self._force
        enter = (force[0] < -self._xminus_force_enter
                 or self._emitted_velocity[0] < -self._xminus_velocity_enter)
        release = (force[0] > -self._xminus_force_exit
                   and self._emitted_velocity[0] > -self._xminus_velocity_exit)
        if enter:
            self._xminus_z_suppression = True
            self._xminus_release_since = None
        elif self._xminus_z_suppression:
            if release:
                if self._xminus_release_since is None:
                    self._xminus_release_since = now
                elif now-self._xminus_release_since >= self._xminus_release_dwell:
                    self._xminus_z_suppression = False
                    self._xminus_release_since = None
            else:
                self._xminus_release_since = None
        target_weight = 1.0 if self._xminus_z_suppression else 0.0
        max_change = max(0.0, min(float(dt), 0.1)) / self._xminus_deadzone_ramp
        self._z_deadzone_weight += float(np.clip(
            target_weight-self._z_deadzone_weight, -max_change, max_change))
        z_threshold = self._additional_z_deadzone * self._z_deadzone_weight
        force = soft_axis_deadzone(force, np.array([0.0, 0.0, z_threshold]))
        force = soft_radial_deadzone(force, self._intent_threshold)
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.vector.x, msg.vector.y, msg.vector.z = map(float, force)
        self._effective_force_pub.publish(msg)
        self._z_deadzone_weight_pub.publish(
            Float32(data=float(self._z_deadzone_weight)))
        return force

    def _reset_adaptive_z_deadzone(self):
        self._xminus_z_suppression = False
        self._xminus_release_since = None
        self._z_deadzone_weight = 0.0

    def _on_calibrated(self, msg):
        self._axia_calibrated = bool(msg.data)
        if not self._axia_calibrated and self._state in ('PREPARING', 'RUNNING'):
            self._set_fault('Axia calibration became invalid')

    def _on_connected(self, msg):
        self._axia_connected = bool(msg.data)
        if not self._axia_connected and self._state in ('PREPARING', 'RUNNING'):
            self._set_fault('Axia sensor disconnected')

    def _on_mode(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode not in ('ground_truth', 'prediction'):
            self.get_logger().warn(f'Ignoring unsupported trajectory mode: {new_mode}')
            return
        if new_mode != self._mode and self._state in ('PREPARING', 'RUNNING'):
            self._set_fault('Trajectory mode changed during a run')
            return
        self._mode = new_mode

    def _on_ui_calibrate(self, request, response):
        if not self._ee_calibrate_client.service_is_ready():
            response.success = False
            response.message = 'Robot-EE calibration service is not ready'
            return response
        self._ee_calibrate_client.call_async(Trigger.Request())
        response.success = True
        response.message = 'Forwarded calibration to robot-EE tracker (no camera used)'
        return response

    def _on_run_status(self, msg):
        if msg.data and self._state not in ('PREPARING', 'RUNNING'):
            self._start_prepare()
        elif not msg.data and self._state in ('PREPARING', 'RUNNING'):
            self._stop_run()

    def _start_prepare(self):
        if self._manual.enabled and self._manual.test_mode and (
                len(self._manual.targets) != 2 or self._manual.selected is None):
            self._reject_start('Save both targets and select a test target before Start Run')
            return
        self._manual.reset_run()
        self._reentry_token, self._reentry_phase = 0, 'OFF'
        self._reentry_samples = 0
        self._emitted_position = self._emitted_time = None
        self._emitted_velocity.fill(0.)
        self._emitted_acceleration.fill(0.)
        self._free_admittance.reset()
        if self._manual.enabled:
            self._hybrid_state = self._requested_hybrid_state = 'FOLLOWER'
        self._fault = ''
        now = time.monotonic()
        if self._current_pose is None or now - self._current_pose_time > self._pose_timeout:
            self._reject_start('No fresh current EE pose')
            return
        if not self._streamer_ready:
            self._reject_start('Cartesian streamer is not ready')
            return
        if not self._axia_connected or now - self._force_time > self._force_stale_hold:
            self._reject_start('No fresh Axia force data')
            return
        if self._require_calibrated and not self._axia_calibrated:
            self._reject_start('Calibrate the Axia sensor before Start Run')
            return
        if not self._ee_calibrate_client.service_is_ready():
            self._reject_start('Robot-EE calibration service is not ready')
            return

        p = self._current_pose.position
        self._capture_pose = self._current_pose
        capture_ee = np.array([p.x, p.y, p.z], dtype=float)
        if np.any(capture_ee < self._workspace_min) or np.any(capture_ee > self._workspace_max):
            self._reject_start(f'Current EE pose {capture_ee.round(4).tolist()} is outside safe workspace')
            return
        self._capture_ee = capture_ee
        self._prediction_reference.reset(capture_ee, now)
        self._admittance.reset()
        self._prediction = None
        self._prediction_time = 0.0
        self._prediction_buffer_size = 0
        self._prediction_source = ''
        self._leader_pending = False
        self._pending_mjm = None
        self._raw_prediction = None
        self._raw_prediction_time = 0.0
        self._robot_ee = None
        self._robot_ee_time = 0.0
        # Không cho dữ liệu absolute/stale đến trong lúc service calibrate đang
        # xử lý thỏa điều kiện PREPARING của lần chạy trước.
        self._calibration_epoch = 0.0
        self._aligned = False
        self._workspace_clamped = False
        self._command_lead_limited = False
        self._follower_realign = False
        self._force_stale_active = False
        self._pose_stale_active = False
        self._force_recovery_pending = False
        self._force_stale_nominal = None
        self._reset_adaptive_z_deadzone()
        self._control_hold_pub.publish(Bool(data=False))
        self._state = 'PREPARING'
        self._prepare_deadline = now + self._prepare_timeout
        future = self._ee_calibrate_client.call_async(Trigger.Request())
        future.add_done_callback(self._after_ee_calibration)
        self.get_logger().info(
            f'PREPARING 3D run; captured EE={self._capture_ee.round(4).tolist()}')

    def _after_ee_calibration(self, future):
        if self._state != 'PREPARING':
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'Robot-EE calibration exception: {exc}')
            return
        if not result.success:
            self._set_fault(f'Robot-EE calibration failed: {result.message}')
            return
        self._calibration_epoch = time.monotonic()
        self._robot_ee = None
        self._prediction = None
        self._prediction_source = ''
        if not self._predictor_toggle_client.service_is_ready():
            self._set_fault('Predictor toggle service is not ready')
            return
        stop = SetBool.Request()
        stop.data = False
        toggle_future = self._predictor_toggle_client.call_async(stop)
        toggle_future.add_done_callback(self._after_predictor_stopped)

    def _after_predictor_stopped(self, future):
        if self._state != 'PREPARING':
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'Predictor reset exception: {exc}')
            return
        if not result.success:
            self._set_fault(f'Predictor reset failed: {result.message}')
            return
        if self._mode == 'prediction':
            start = SetBool.Request()
            start.data = True
            self._predictor_toggle_client.call_async(start)

    def _try_finish_prepare(self, now):
        if now > self._prepare_deadline:
            self._set_fault('Timed out waiting for post-calibration robot-EE/prediction data')
            return
        if self._calibration_epoch <= 0.0 or self._robot_ee is None:
            return
        if self._robot_ee_time <= self._calibration_epoch:
            return
        if self._mode == 'prediction':
            if self._prediction is None or self._prediction_time <= self._calibration_epoch:
                return
            if self._prediction_buffer_size < self._min_prediction_buffer:
                return
        nominal = self._nominal_position(now)
        if nominal is None:
            return
        actual_ee = np.array([
            self._current_pose.position.x,
            self._current_pose.position.y,
            self._current_pose.position.z])
        self._admittance.reset(actual_ee - nominal)
        self._aligned = True
        self._last_tick = now
        self._state = 'RUNNING'
        self.get_logger().info(
            f'3D co-carrying RUNNING '
            f'({"learned nominal" if self._mode == "prediction" else "fixed captured nominal"}); '
            'orientation fixed, XYZ enabled')

    def _nominal_position(self, now, apply_prediction_limit=True):
        if self._mode == 'prediction':
            if self._prediction is None or now - self._prediction_time > self._prediction_timeout:
                return None
        desired = nominal_reference(
            self._capture_ee,
            self._mode,
            self._prediction,
        )
        if self._mode != 'prediction' or not apply_prediction_limit:
            return desired

        desired = self._prediction_reference.step(desired, now)

        actual = np.array([
            self._current_pose.position.x,
            self._current_pose.position.y,
            self._current_pose.position.z], dtype=float)
        desired, limited, distance = limit_position_lead(
            desired, actual, self._prediction_max_nominal_lead)
        if limited:
            # Keep the conditioner at the accepted nominal too. An extreme
            # prediction must not leave a hidden, far-away filter state.
            self._prediction_reference.reset(desired, now)
            self.get_logger().warn(
                'Predictor nominal limited to '
                f'{self._prediction_max_nominal_lead:.3f} m from actual EE '
                f'(requested {distance:.3f} m)',
                throttle_duration_sec=1.0)
        return desired

    def _stop_run(self):
        self._manual_leader_pending = False
        self._reset_adaptive_z_deadzone()
        self._manual.reset_run('stopped')
        self._reentry_token, self._reentry_phase = 0, 'OFF'
        self._state = 'STOPPED'
        self._admittance.reset()
        self._aligned = False
        self._pose_stale_active = False
        self._force_stale_nominal = None
        self._control_hold_pub.publish(Bool(data=False))
        self._request_predictor_stop()
        self.get_logger().info('3D co-carrying stopped; streamer will hold until UI disables it')

    def _reject_start(self, reason):
        self._state = 'STOPPED'
        self._request_predictor_stop()
        self.get_logger().error(f'3D co-carrying start rejected: {reason}')
        self._run_status_pub.publish(Bool(data=False))

    def _set_fault(self, reason):
        if self._fault:
            return
        self._manual_leader_pending = False
        self._reset_adaptive_z_deadzone()
        self._manual.reset_run('fault: ' + reason)
        self._reentry_token, self._reentry_phase = 0, 'OFF'
        self._fault = reason
        self._state = 'FAULT'
        self._control_hold_pub.publish(Bool(data=False))
        self._request_predictor_stop()
        self.get_logger().error(f'3D co-carrying FAULT: {reason}')
        if self._streamer_enable_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self._streamer_enable_client.call_async(request)
        self._run_status_pub.publish(Bool(data=False))

    def _request_predictor_stop(self):
        if self._predictor_toggle_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self._predictor_toggle_client.call_async(request)

    def _publish_ages(self, now):
        force_age_ms = (
            float('nan') if self._force_time <= 0.0
            else max(0.0, (now - self._force_time) * 1000.0))
        prediction_age_ms = (
            float('nan') if self._prediction_time <= 0.0
            else max(0.0, (now - self._prediction_time) * 1000.0))
        self._force_age_pub.publish(Float32(data=force_age_ms))
        self._prediction_age_pub.publish(Float32(data=prediction_age_ms))
        pose_age_ms = (
            float('nan') if self._current_pose_time <= 0.0
            else max(0.0, (now - self._current_pose_time) * 1000.0))
        self._pose_age_pub.publish(Float32(data=pose_age_ms))

    def _control_tick(self):
        now = time.monotonic()
        self._publish_ages(now)
        if self._state == 'PREPARING':
            self._try_finish_prepare(now)
            return
        if self._state != 'RUNNING':
            return
        force_age = now - self._force_time
        force_action = force_watchdog_action(
            force_age, self._force_stale_hold, self._force_timeout)
        if force_action == 'fault':
            self._set_fault(
                f'Force data timeout ({force_age * 1000.0:.0f} ms)')
            return
        pose_age = now - self._current_pose_time
        pose_action = force_watchdog_action(
            pose_age, self._pose_stale_hold, self._pose_timeout)
        if pose_action == 'fault':
            self._set_fault(
                f'Current EE pose timeout ({pose_age * 1000.0:.0f} ms)')
            return
        if not self._streamer_ready:
            self._set_fault('Cartesian streamer is not ready')
            return
        if np.any(np.abs(self._force) > self._max_force_axis):
            self._set_fault(f'Per-axis force limit exceeded: {self._force.round(2).tolist()} N')
            return
        force_norm = float(np.linalg.norm(self._force))
        if force_norm > self._max_force_norm:
            self._set_fault(f'Resultant force limit exceeded: {force_norm:.2f} N')
            return

        actual_ee = np.array([
            self._current_pose.position.x,
            self._current_pose.position.y,
            self._current_pose.position.z], dtype=float)
        leader = self._mode == 'prediction' and self._hybrid_state == 'LEADER'
        if pose_action == 'hold':
            if not self._pose_stale_active:
                self._pose_stale_active = True
                self._control_hold_pub.publish(Bool(data=True))
                self.get_logger().warn(
                    f'Current EE pose stale for {pose_age * 1000.0:.0f} ms; '
                    'holding last measured EE until feedback recovers')
            self._last_tick = now
            self._publish_reference(actual_ee, np.zeros(3), actual_ee)
            return
        if self._pose_stale_active:
            self._pose_stale_active = False
            self._follower_realign = True
            if not self._force_stale_active:
                self._control_hold_pub.publish(Bool(data=False))
            self.get_logger().info('Current EE pose stream recovered; resume from measured EE')
        if force_action == 'hold':
            self._manual._inside_since = None
            if not self._force_stale_active:
                self._force_stale_active = True
                self._force_recovery_pending = False
                self._force_stale_nominal = (
                    actual_ee - self._admittance.error)
                self._control_hold_pub.publish(Bool(data=True))
                self.get_logger().warn(
                    f'Force stream stale for {force_age * 1000.0:.0f} ms; '
                    'holding actual EE until data recovers')
                if self._manual.enabled and not leader and self._reentry_phase != 'OFF':
                    self._begin_reentry_wait()
            # Never advance a predictor/MJM nominal from the last non-zero force
            # while force data is stale.  Keep diagnostics alive but freeze
            # the control reference at measured EE.
            nominal = (
                self._force_stale_nominal.copy()
                if self._force_stale_nominal is not None
                else actual_ee.copy())
            error = actual_ee - nominal
            # Freeze nominal state/time too: no hidden advance during HOLD.
            self._prediction_reference.reset(nominal, now)
            self._admittance.reset(error)
            self._free_origin = actual_ee.copy()
            self._free_admittance.reset()
            self._last_tick = now
            self._publish_reference(nominal, error, actual_ee)
            return

        if self._manual.enabled and leader and self._force_stale_active and self._force_recovery_pending:
            try:
                self._manual.resume_after_hold(actual_ee)
            except ValueError as exc:
                self._set_fault(f'Cannot resume Hybrid after force HOLD: {exc}')
                return
        if self._manual.enabled and self._manual.test_mode and not self._manual.test_fired:
            self._manual.test_elapsed += max(0., min(now-self._last_tick, .1))
            if self._manual.test_elapsed >= 5.:
                # One attempt only, including a rejected bridge. No surprise retry.
                self._manual.test_fired = True
                self._manual_command(String(data='{"action":"leader"}'))
                if self._manual.role == 'LEADER':
                    self._manual.role_source = 'test_timer'
                    self._manual.reason = 'test_timer_leader'
        self._activate_manual_leader_on_tick(now, actual_ee)
        if self._manual.test_mode and self._manual.role == 'LEADER':
            self._manual.role_source = 'test_timer'
            if self._manual.reason == 'manual_leader':
                self._manual.reason = 'test_timer_leader'
        leader = self._hybrid_state == 'LEADER'
        manual_leader = self._manual.enabled and leader
        if self._manual.enabled:
            was_leader = self._manual.role == 'LEADER'
            reached = self._manual.observe(actual_ee, self._ee_speed, now)
            if reached and was_leader:
                self._manual_follow('reached')
                leader = manual_leader = False
            if manual_leader and self._manual.elapsed > self._manual.duration + self._manual.grace:
                self._set_fault('Hybrid target_not_reached after MJM grace period')
                return
        if (self._manual.enabled and not leader and self._reentry_phase == 'WAIT'
                and self._reentry_samples >= self._reentry_warmup_samples and self._prediction is not None
                and now-self._prediction_time <= self._prediction_timeout):
            self._reentry_phase, self._reentry_elapsed, self._reentry_align = 'BLEND', 0., True
            start = actual_ee if self._emitted_position is None else self._emitted_position
            self._prediction_reference.reset(start, now)
        blending = self._manual.enabled and not leader and self._reentry_phase == 'BLEND'
        free_follower = self._manual.enabled and self._manual.force_follower and not leader and not blending
        if self._mode == 'prediction' and not manual_leader and not free_follower:
            correct_source = (
                self._prediction_source == 'mjm'
                if leader else self._prediction_source not in ('', 'mjm'))
            if not correct_source:
                if now - self._role_change_time <= self._prediction_timeout:
                    if self._follower_realign:
                        self._last_tick = now
                        self._publish_reference(actual_ee, np.zeros(3), actual_ee)
                    return
                expected = 'MJM' if leader else 'predictor'
                self._set_fault(f'Timed out waiting for a fresh {expected} position')
                return
        if self._follower_realign and not leader:
            self._prediction_reference.reset(actual_ee, now)
        nominal = (self._free_origin.copy() if free_follower else
                   self._manual.sample(now-self._last_tick) if manual_leader else
                   self._nominal_position(now, apply_prediction_limit=not leader))
        if nominal is None:
            self._set_fault(f'{self._mode} position timeout')
            return

        if self._force_stale_active and self._force_recovery_pending:
            self._force_stale_active = False
            self._force_recovery_pending = False
            self._control_hold_pub.publish(Bool(data=False))
            self._admittance.reset(actual_ee - nominal)
            self._force_stale_nominal = None
            self.get_logger().info('Force stream recovered; resume from measured EE')
        dt = now - self._last_tick
        self._last_tick = now
        if leader:
            # MJM already represents the Cartesian robot trajectory.  Force is
            # still watched above for timeout/limits, but is not applied here.
            error = np.zeros(3, dtype=float)
            reference_ee = commanded_position(nominal, error, leader=True)
        elif free_follower:
            effective_force = self._effective_force_input(now, dt)
            error, _, _ = self._free_admittance.step(effective_force, dt)
            reference_ee = nominal + error
        elif blending:
            force = self._effective_force_input(now, dt)
            if self._reentry_align:
                start = actual_ee if self._emitted_position is None else self._emitted_position
                self._admittance.reset(start-nominal)
                self._admittance.error_velocity[:] = self._emitted_velocity
                self._reentry_align = False
            s = min(self._reentry_elapsed/self._reentry_blend_sec, 1.)
            weight = 10*s**3-15*s**4+6*s**5
            free_error, _, _ = self._free_admittance.step(force, dt)
            learned_error, _, _ = self._admittance.step(force, dt, stiffness_scale=weight)
            reference_ee = ((1-weight)*(self._free_origin+free_error)
                            + weight*(nominal+learned_error))
            error = reference_ee-nominal
            self._reentry_elapsed += max(0., min(dt, .1))
            if s >= 1.:
                self._reentry_phase = 'ACTIVE'
                self._manual.force_follower = False
        else:
            if self._follower_realign:
                self._admittance.reset(actual_ee - nominal)
                self._follower_realign = False
                error = self._admittance.error.copy()
            else:
                effective_force = self._effective_force_input(now, dt)
                error, _, _ = self._admittance.step(effective_force, dt)
            reference_ee = commanded_position(nominal, error, leader=False)

        integrator = self._free_admittance if free_follower else self._admittance
        unconstrained_reference = reference_ee.copy()
        if self._manual.enabled and not leader and self._reentry_phase in ('BLEND', 'ACTIVE'):
            # Smoothstep alone does not bound velocity for an arbitrary GRU
            # jump. Bound the TOTAL output and retain this guard after blending.
            if self._emitted_position is not None:
                step_dt = float(np.clip(dt, 1e-4, .1))
                velocity = (reference_ee-self._emitted_position)/step_dt
                change = CartesianAdmittance._limit_norm(
                    velocity-self._emitted_velocity, integrator.max_acceleration*step_dt)
                velocity = CartesianAdmittance._limit_norm(
                    self._emitted_velocity+change, integrator.max_velocity)
                limited = self._emitted_position+velocity*step_dt
                if not np.allclose(limited, reference_ee, atol=1e-10, rtol=0):
                    reference_ee = limited
                    if not blending:
                        integrator.error = reference_ee-nominal
                    error = reference_ee-nominal
        clamped_ee = np.clip(reference_ee, self._workspace_min, self._workspace_max)
        self._workspace_clamped = not np.allclose(clamped_ee, reference_ee)
        if self._workspace_clamped:
            unclamped_ee = reference_ee.copy()
            reference_ee = clamped_ee
            if not leader and not blending:
                for axis in range(3):
                    if reference_ee[axis] != unclamped_ee[axis]:
                        integrator.error[axis] = reference_ee[axis] - nominal[axis]
                        outward = unclamped_ee[axis] - reference_ee[axis]
                        if np.sign(integrator.error_velocity[axis]) == np.sign(outward):
                            integrator.error_velocity[axis] = 0.0
                error = integrator.error.copy()
            self.get_logger().warn(
                f'Workspace clamped at EE={reference_ee.round(4).tolist()}',
                throttle_duration_sec=1.0)

        # Anti-windup theo feedback thật: workspace hình hộp có thể chứa pose
        # không đạt được với orientation/joint limits hiện tại. Không cho
        # reference chạy xa trước robot rồi tích lũy thành IK failure giả.
        lead = reference_ee - actual_ee
        lead_norm = float(np.linalg.norm(lead))
        self._command_lead_limited = lead_norm > self._max_command_lead
        if self._command_lead_limited:
            lead_direction = lead / lead_norm
            reference_ee = actual_ee + lead_direction * self._max_command_lead
            if not leader and not blending:
                integrator.error = reference_ee - nominal
                outward_velocity = float(np.dot(
                    integrator.error_velocity, lead_direction))
                if outward_velocity > 0.0:
                    integrator.error_velocity -= (
                        outward_velocity * lead_direction)
                error = integrator.error.copy()
            self.get_logger().warn(
                f'Command lead limited to {self._max_command_lead:.3f} m '
                f'(requested {lead_norm:.3f} m)',
                throttle_duration_sec=1.0)
        if blending:
            # A common correction preserves the weighted sum for EVERY weight,
            # including zero. Never assign a blended error to only one branch.
            correction = reference_ee-unconstrained_reference
            for branch in (self._free_admittance, self._admittance):
                branch.error += correction
                norm = float(np.linalg.norm(correction))
                if norm > 1e-10:
                    outward = -correction/norm
                    speed = float(np.dot(branch.error_velocity, outward))
                    if speed > 0.:
                        branch.error_velocity -= speed*outward
            error = reference_ee-nominal
        self._publish_reference(nominal, error, reference_ee)

    def _publish_reference(self, nominal, error, reference_ee):
        if not np.all(np.isfinite([nominal, error, reference_ee])):
            self._set_fault('Non-finite Cartesian reference')
            return
        now = time.monotonic()
        if (self._emitted_time is not None and now > self._emitted_time
                and not self._force_stale_active and not self._pose_stale_active):
            dt = now-self._emitted_time
            velocity = (reference_ee-self._emitted_position)/dt
            self._emitted_acceleration = (velocity-self._emitted_velocity)/dt
            self._emitted_velocity = velocity
        else:
            self._emitted_velocity.fill(0.)
            self._emitted_acceleration.fill(0.)
        self._emitted_position, self._emitted_time = reference_ee.copy(), now
        self._manual_status()
        stamp = self.get_clock().now().to_msg()
        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = 'base_link'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, reference_ee)
        target.pose.orientation = self._capture_pose.orientation
        self._target_pub.publish(target)

        def point_message(values):
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            msg.point.x, msg.point.y, msg.point.z = map(float, values)
            return msg

        nominal_msg = point_message(nominal)
        reference_msg = point_message(reference_ee)
        self._nominal_pub.publish(nominal_msg)
        self._reference_pub.publish(reference_msg)
        if self._capture_ee is not None:
            relative_msg = point_message(reference_ee - self._capture_ee)
            relative_msg.header.frame_id = 'capture_relative'
            self._relative_reference_pub.publish(relative_msg)
        self._target_base_pub.publish(reference_msg)
        err = Vector3Stamped()
        err.header = nominal_msg.header
        err.vector.x, err.vector.y, err.vector.z = map(float, error)
        self._error_pub.publish(err)

    def _publish_status(self):
        self._manual_status()
        if self._fault:
            state = f'FAULT: {self._fault}'
        elif self._state == 'RUNNING' and self._pose_stale_active:
            pose_age_ms = max(
                0.0, (time.monotonic() - self._current_pose_time) * 1000.0)
            state = f'RUNNING: POSE_STALE_HOLD ({pose_age_ms:.0f} ms)'
        elif self._state == 'RUNNING' and self._force_stale_active:
            force_age_ms = max(
                0.0, (time.monotonic() - self._force_time) * 1000.0)
            state = f'RUNNING: FORCE_STALE_HOLD ({force_age_ms:.0f} ms)'
        elif self._state == 'RUNNING' and self._workspace_clamped:
            state = 'RUNNING: WORKSPACE_CLAMPED'
        elif self._state == 'RUNNING' and self._command_lead_limited:
            state = 'RUNNING: COMMAND_LEAD_LIMITED'
        elif self._state in ('PREPARING', 'RUNNING'):
            state = self._state
        elif self._axia_calibrated and self._axia_connected and self._streamer_ready:
            state = 'READY'
        else:
            state = 'WAIT_AXIA_CALIBRATION'
        self._status_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
