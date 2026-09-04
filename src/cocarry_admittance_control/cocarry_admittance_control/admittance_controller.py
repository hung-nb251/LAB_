#!/usr/bin/env python3
"""3D admittance controller using robot-EE history and an Axia force sensor."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from human_hand_msgs.msg import HandPrediction, HandState
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger

from .admittance import (
    CartesianAdmittance,
    commanded_position,
    critical_damping,
    fresh_mjm_sample,
    force_watchdog_action,
    limit_position_lead,
    minimum_safe_ee_z,
    nominal_reference,
    soft_radial_deadzone,
)


class AdmittanceController3D(Node):
    def __init__(self):
        super().__init__('cocarry_admittance_controller')

        defaults = {
            'control_rate_hz': 15.0,
            'virtual_mass_x': 1.0, 'virtual_mass_y': 1.0, 'virtual_mass_z': 1.0,
            'stiffness_x': 10.0, 'stiffness_y': 10.0, 'stiffness_z': 10.0,
            'critical_damping': True,
            'damping_x': 6.32455532, 'damping_y': 6.32455532, 'damping_z': 6.32455532,
            # Axia UI already applies the 4 N radial deadband.
            'intent_threshold_n': 0.0,
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
            # The controller uses EE coordinates.  Keep the downward rod above
            # the floor by converting tip clearance into an EE lower bound.
            'safe_tool_tip_z_min_m': 0.05,
            'downward_tool_length_m': 0.1814,
            'safe_workspace_z_max_m': 1.5,
            'max_force_per_axis_n': 15.0,
            'max_force_norm_n': 20.0,
            'force_stale_hold_sec': 0.20,
            'force_timeout_sec': 0.50,
            'current_pose_timeout_sec': 0.25,
            'prediction_timeout_sec': 0.50,
            # Keep the AI nominal close enough to measured EE for IK/tracking
            # safety.  This is a geometric bound, not a force/intention gate.
            'prediction_max_nominal_lead_m': 0.05,
            'prepare_timeout_sec': 8.0,
            'min_prediction_buffer_size': 10,
            'require_axia_calibrated': True,
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
        self._force_sign = np.array([
            1.0 if float(gp('force_sign_x')) >= 0.0 else -1.0,
            1.0 if float(gp('force_sign_y')) >= 0.0 else -1.0,
            1.0 if float(gp('force_sign_z')) >= 0.0 else -1.0,
        ])
        self._intent_threshold = float(gp('intent_threshold_n'))
        self._max_command_lead = float(gp('max_command_lead_m'))
        if self._max_command_lead <= 0.0:
            raise ValueError('max_command_lead_m must be positive')
        self._safe_tip_z_min = float(gp('safe_tool_tip_z_min_m'))
        self._downward_tool_length = float(gp('downward_tool_length_m'))
        self._workspace_min = np.array([
            gp('safe_workspace_x_min_m'), gp('safe_workspace_y_min_m'),
            minimum_safe_ee_z(
                self._safe_tip_z_min, self._downward_tool_length)], dtype=float)
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
        self._pose_timeout = float(gp('current_pose_timeout_sec'))
        self._prediction_timeout = float(gp('prediction_timeout_sec'))
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
        self._pending_mjm = None
        self._role_change_time = 0.0
        self._follower_realign = False
        self._calibration_epoch = 0.0
        self._prepare_deadline = 0.0
        self._aligned = False
        self._workspace_clamped = False
        self._command_lead_limited = False
        self._force_stale_active = False
        self._force_recovery_pending = False
        self._force_stale_nominal = None
        self._last_tick = time.monotonic()

        self._target_pub = self.create_publisher(PoseStamped, '/cartesian_streamer/target_pose', 10)
        self._target_base_pub = self.create_publisher(PointStamped, '/coord_transform/target_base', 10)
        self._filtered_pub = self.create_publisher(PointStamped, '/coord_transform/filtered_hand_position', 10)
        self._nominal_pub = self.create_publisher(PointStamped, '/cocarry/nominal_position', 10)
        self._reference_pub = self.create_publisher(PointStamped, '/cocarry/reference_position', 10)
        self._relative_reference_pub = self.create_publisher(
            PointStamped, '/cocarry/reference_relative', 10)
        self._error_pub = self.create_publisher(Vector3Stamped, '/cocarry/admittance_error', 10)
        self._status_pub = self.create_publisher(String, '/cocarry/status', 10)
        self._force_age_pub = self.create_publisher(Float32, '/cocarry/force_age_ms', 10)
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
            f'(tip floor={self._safe_tip_z_min:.2f} m, rod={self._downward_tool_length:.4f} m), '
            f'force stale/hard={self._force_stale_hold:.2f}/{self._force_timeout:.2f} s, '
            f'Predictor nominal lead={self._prediction_max_nominal_lead:.3f} m '
            '(no force gate)')

    def _on_current_pose(self, msg):
        self._current_pose = msg.pose
        self._current_pose_time = time.monotonic()

    def _on_streamer_ready(self, msg):
        was_ready = self._streamer_ready
        self._streamer_ready = bool(msg.data)
        if was_ready and not self._streamer_ready and self._state in ('PREPARING', 'RUNNING'):
            self._set_fault('Cartesian streamer lost readiness')

    def _on_robot_ee(self, msg):
        if not msg.is_tracked or msg.source != 'robot_ee':
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

        # DDS does not guarantee ordering across hybrid_state and prediction
        # topics.  Keep an early MJM point separate so FOLLOWER can continue
        # using its last learned-predictor sample until the LEADER request arrives.
        if source == 'mjm' and self._requested_hybrid_state != 'LEADER':
            self._pending_mjm = (prediction, received_at, buffer_size)
            return
        if self._requested_hybrid_state == 'LEADER' and source != 'mjm':
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
        self._force[:] = [msg.vector.x, msg.vector.y, msg.vector.z]
        self._force_time = time.monotonic()
        if self._force_stale_active:
            self._force_recovery_pending = True

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
        self._force_recovery_pending = False
        self._force_stale_nominal = None
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

        actual = np.array([
            self._current_pose.position.x,
            self._current_pose.position.y,
            self._current_pose.position.z], dtype=float)
        desired, limited, distance = limit_position_lead(
            desired, actual, self._prediction_max_nominal_lead)
        if limited:
            self.get_logger().warn(
                'Predictor nominal limited to '
                f'{self._prediction_max_nominal_lead:.3f} m from actual EE '
                f'(requested {distance:.3f} m)',
                throttle_duration_sec=1.0)
        return desired

    def _stop_run(self):
        self._state = 'STOPPED'
        self._admittance.reset()
        self._aligned = False
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
        if now - self._current_pose_time > self._pose_timeout:
            self._set_fault('Current EE pose timeout')
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
        if force_action == 'hold':
            if not self._force_stale_active:
                self._force_stale_active = True
                self._force_recovery_pending = False
                self._force_stale_nominal = (
                    actual_ee - self._admittance.error)
                self._control_hold_pub.publish(Bool(data=True))
                self.get_logger().warn(
                    f'Force stream stale for {force_age * 1000.0:.0f} ms; '
                    'holding actual EE until data recovers')
            # Never advance a predictor/MJM nominal from the last non-zero force
            # while force data is stale.  Keep diagnostics alive but freeze
            # the control reference at measured EE.
            nominal = (
                self._force_stale_nominal.copy()
                if self._force_stale_nominal is not None
                else actual_ee.copy())
            error = actual_ee - nominal
            self._admittance.reset(error)
            self._last_tick = now
            self._publish_reference(nominal, error, actual_ee)
            return

        if self._mode == 'prediction':
            correct_source = (
                self._prediction_source == 'mjm'
                if leader else self._prediction_source not in ('', 'mjm'))
            if not correct_source:
                if now - self._role_change_time <= self._prediction_timeout:
                    return
                expected = 'MJM' if leader else 'predictor'
                self._set_fault(f'Timed out waiting for a fresh {expected} position')
                return
        nominal = self._nominal_position(now, apply_prediction_limit=not leader)
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
        else:
            if self._follower_realign:
                self._admittance.reset(actual_ee - nominal)
                self._follower_realign = False
            effective_force = soft_radial_deadzone(
                self._force_sign * self._force, self._intent_threshold)
            error, _, _ = self._admittance.step(effective_force, dt)
            reference_ee = commanded_position(nominal, error, leader=False)

        clamped_ee = np.clip(reference_ee, self._workspace_min, self._workspace_max)
        self._workspace_clamped = not np.allclose(clamped_ee, reference_ee)
        if self._workspace_clamped:
            unclamped_ee = reference_ee.copy()
            reference_ee = clamped_ee
            if not leader:
                for axis in range(3):
                    if reference_ee[axis] != unclamped_ee[axis]:
                        self._admittance.error[axis] = reference_ee[axis] - nominal[axis]
                        outward = unclamped_ee[axis] - reference_ee[axis]
                        if np.sign(self._admittance.error_velocity[axis]) == np.sign(outward):
                            self._admittance.error_velocity[axis] = 0.0
                error = self._admittance.error.copy()
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
            if not leader:
                self._admittance.error = reference_ee - nominal
                outward_velocity = float(np.dot(
                    self._admittance.error_velocity, lead_direction))
                if outward_velocity > 0.0:
                    self._admittance.error_velocity -= (
                        outward_velocity * lead_direction)
                error = self._admittance.error.copy()
            self.get_logger().warn(
                f'Command lead limited to {self._max_command_lead:.3f} m '
                f'(requested {lead_norm:.3f} m)',
                throttle_duration_sec=1.0)
        self._publish_reference(nominal, error, reference_ee)

    def _publish_reference(self, nominal, error, reference_ee):
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
        if self._fault:
            state = f'FAULT: {self._fault}'
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
