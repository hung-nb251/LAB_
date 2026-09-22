#!/usr/bin/env python3
"""Shadow-only F_robot estimator from YRC M310-M315 external joint torque.

The node never commands the robot. It freezes a no-contact Home baseline on
the rising edge of /run_status, applies the validated local torque calibration,
and publishes the existing /sensorless_force diagnostic interface for CSV logs.
"""
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import WrenchStamped
from motoros2_interfaces.srv import ReadMRegister
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from local_ik_solver import LocalIKSolver
from sensorless_force_math import apply_mregister_calibration, recover_calibrated_wrench


JOINT_NAMES = (
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
)
ADDRESSES = tuple(range(310, 316))


class MRegisterForceNode(Node):
    def __init__(self):
        super().__init__('sensorless_force_node')
        default_model = str(Path(get_package_share_directory('hc10dtp_bringup')) /
                            'config/f_robot_m310_candidate_20260918.json')
        self.declare_parameter('calibration_file', default_model)
        self.declare_parameter('baseline_min_scans', 12)
        self.declare_parameter('baseline_window_scans', 30)
        self.declare_parameter('baseline_max_joint_speed_rad_s', 0.02)
        self.declare_parameter('scan_gap_sec', 0.05)
        self.declare_parameter('request_timeout_sec', 1.0)
        self.declare_parameter('request_recovery_sec', 2.0)
        self.declare_parameter('joint_state_timeout_sec', 0.25)
        self.declare_parameter('deadband_n', 1.0)
        self.declare_parameter('max_force_norm_n', 100.0)

        model_path = Path(str(self.get_parameter('calibration_file').value)).expanduser()
        model = json.loads(model_path.read_text())
        self._scale = np.asarray(model['scale'], dtype=float)
        self._coef = np.asarray(model['coef'], dtype=float)
        if self._scale.shape != (12,) or self._coef.shape != (12, 6):
            raise ValueError('M310 calibration must have scale[12] and coef[12,6]')
        if not np.all(np.isfinite(self._scale)) or not np.all(np.isfinite(self._coef)):
            raise ValueError('M310 calibration contains non-finite values')
        self._damping = float(model['damping'])
        self._characteristic_length = float(model['characteristic_length_m'])
        self._model_path = str(model_path)
        self._baseline_min = int(self.get_parameter('baseline_min_scans').value)
        baseline_window = int(self.get_parameter('baseline_window_scans').value)
        if self._baseline_min < 3 or baseline_window < self._baseline_min:
            raise ValueError('baseline window must contain at least baseline_min_scans >= 3')
        self._baseline_max_speed = float(
            self.get_parameter('baseline_max_joint_speed_rad_s').value)
        self._scan_gap = float(self.get_parameter('scan_gap_sec').value)
        self._request_timeout = float(self.get_parameter('request_timeout_sec').value)
        self._request_recovery = float(self.get_parameter('request_recovery_sec').value)
        self._joint_timeout = float(self.get_parameter('joint_state_timeout_sec').value)
        self._deadband = float(self.get_parameter('deadband_n').value)
        self._max_force = float(self.get_parameter('max_force_norm_n').value)

        self._ik = LocalIKSolver()
        self._latest_q = self._latest_velocity = None
        self._latest_joint_monotonic = 0.0
        self._last_names = self._joint_indices = None
        self._running = False
        self._baseline_samples = deque(maxlen=baseline_window)
        self._baseline = self._q0 = None
        self._client = self.create_client(ReadMRegister, '/read_mregister')
        self._address_index = 0
        self._scan_values = np.zeros(6)
        self._scan_raw = np.zeros(6, dtype=int)
        self._scan_first_ns = 0
        self._pending = None
        self._next_scan = 0.0
        self._last_status = ''
        self._scan_id = 0
        self._scan_registers = []
        self._scan_restarts = 0
        self._timeout_active = False
        self._errors = {'service_error': 0, 'not_success': 0,
                        'request_timeout': 0, 'abandoned_request': 0}

        self._wrench_pub = self.create_publisher(WrenchStamped, '/sensorless_force', 10)
        self._torque_pub = self.create_publisher(
            JointState, '/sensorless_force/joint_torque', 10)
        self._valid_pub = self.create_publisher(Bool, '/sensorless_force/valid', 10)
        self._status_pub = self.create_publisher(String, '/sensorless_force/status', 10)
        self._diagnostics_pub = self.create_publisher(
            Float64MultiArray, '/sensorless_force/diagnostics', 10)
        self._sample_pub = self.create_publisher(String, '/sensorless_force/sample', 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state, qos_profile_sensor_data)
        self.create_subscription(Bool, '/run_status', self._run_status, 10)
        self.create_timer(0.005, self._tick)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.get_logger().warning(
            'M310 F_robot estimator is SHADOW LOGGING ONLY; it is not connected '
            'to control or role selection. Candidate is local to Home-T1/T2.')

    def _on_set_parameters(self, params):
        """Timing parameters were previously latched at init only."""
        tunable = {'scan_gap_sec': '_scan_gap',
                   'request_timeout_sec': '_request_timeout',
                   'request_recovery_sec': '_request_recovery'}
        for param in params:
            if param.name not in tunable:
                return SetParametersResult(
                    successful=False,
                    reason=f'{param.name} is not runtime-tunable on this node')
            try:
                value = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason=f'{param.name} must be a number')
            if not math.isfinite(value) or value < 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{param.name} must be finite and >= 0')
            if param.name != 'scan_gap_sec' and value <= 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{param.name} must be > 0')
        for param in params:
            setattr(self, tunable[param.name], float(param.value))
            self.get_logger().info(f'{param.name} set to {float(param.value):.4f}')
        return SetParametersResult(successful=True)

    def _indices_for(self, msg):
        names = tuple(msg.name)
        if names == self._last_names and self._joint_indices is not None:
            return self._joint_indices
        lookup = {name: i for i, name in enumerate(names)}
        if any(name not in lookup for name in JOINT_NAMES):
            raise ValueError('missing required robot joints')
        self._last_names = names
        self._joint_indices = tuple(lookup[name] for name in JOINT_NAMES)
        return self._joint_indices

    def _joint_state(self, msg):
        try:
            indices = self._indices_for(msg)
            if max(indices) >= len(msg.position):
                return
            q = np.asarray([msg.position[i] for i in indices], dtype=float)
            velocity = (np.asarray([msg.velocity[i] for i in indices], dtype=float)
                        if msg.velocity and max(indices) < len(msg.velocity)
                        else np.full(6, np.nan))
            if np.all(np.isfinite(q)):
                self._latest_q = q
                self._latest_velocity = velocity
                self._latest_joint_monotonic = time.monotonic()
        except (IndexError, ValueError):
            return

    def _run_status(self, msg):
        running = bool(msg.data)
        if running and not self._running:
            if len(self._baseline_samples) >= self._baseline_min:
                values = np.array([x[0] for x in self._baseline_samples])
                positions = np.array([x[1] for x in self._baseline_samples])
                self._baseline = np.median(values, axis=0)
                self._q0 = np.median(positions, axis=0)
                self.get_logger().info(
                    f'Frozen M310 baseline from {len(values)} scans: '
                    f'{self._baseline.round(3).tolist()} Nm')
            else:
                self._baseline = self._q0 = None
                self.get_logger().error(
                    f'No M310 baseline: only {len(self._baseline_samples)}/'
                    f'{self._baseline_min} stationary scans before Start Run')
        elif not running and self._running:
            self._baseline = self._q0 = None
            self._baseline_samples.clear()
        self._running = running

    def _sample_template(self, stamp_ns, status):
        return {
            'schema': 1, 'stamp_ns': int(stamp_ns), 'frame': 'base_link',
            'mode': 'mregister_calibrated_pose', 'calibration_confirmed': False,
            'role_valid': False, 'status': status, 'force': None,
            'force_unfiltered': None, 'torque_nm': None, 'position': None,
            'effort_raw': None, 'diagnostics': None,
            'scale_nm_per_effort': self._scale[:6].tolist(),
            'bias_nm': self._baseline.tolist() if self._baseline is not None else None,
            'mregister_addresses': list(ADDRESSES),
            'mregister_values_nm': None,
            'mregister_delta_nm': None,
            'baseline_q0_rad': self._q0.tolist() if self._q0 is not None else None,
            'calibration_file': self._model_path,
            'register_scan_span_ms': None,
            'acquisition': None,
        }

    def _acquisition_block(self, span_ms=None, end_ns=None):
        """Per-register provenance for latency/jitter audits. Additive only."""
        joint_age_ms = None
        if self._latest_q is not None:
            joint_age_ms = (time.monotonic() - self._latest_joint_monotonic) * 1e3
        return {
            'acquisition_schema': 1,
            'reader_backend': 'scalar_sequential',
            'reader_version': '20260919_recover_v1',
            'service': '/read_mregister',
            'scan_id': self._scan_id,
            'scan_first_ns': int(self._scan_first_ns) or None,
            'scan_end_ns': int(end_ns) if end_ns else None,
            'span_ms': span_ms,
            'scan_gap_target_ms': self._scan_gap * 1e3,
            'request_timeout_ms': self._request_timeout * 1e3,
            'registers': list(self._scan_registers),
            'restarts_this_scan': self._scan_restarts,
            'errors_cumulative': dict(self._errors),
            'joint_age_ms': joint_age_ms,
        }

    def _publish_sample(self, sample):
        self._sample_pub.publish(String(data=json.dumps(sample, allow_nan=False)))
        self._valid_pub.publish(Bool(data=False))
        self._status_pub.publish(String(data=sample['status']))
        if sample['status'] != self._last_status:
            self.get_logger().info(sample['status'])
            self._last_status = sample['status']

    def _complete_scan(self, stamp_ns, span_ms, end_ns):
        sample = self._sample_template(stamp_ns, 'INVALID:baseline_not_frozen')
        sample['mregister_values_nm'] = self._scan_values.tolist()
        sample['register_scan_span_ms'] = span_ms
        sample['acquisition'] = self._acquisition_block(span_ms, end_ns)
        joint_fresh = (self._latest_q is not None and
                       time.monotonic() - self._latest_joint_monotonic <= self._joint_timeout)
        if not joint_fresh:
            sample['status'] = 'INVALID:stale_joint_state'
            self._publish_sample(sample)
            return
        q = self._latest_q.copy()
        stationary = (self._latest_velocity is not None and
                      np.all(np.isfinite(self._latest_velocity)) and
                      np.max(np.abs(self._latest_velocity)) <= self._baseline_max_speed)
        if not self._running:
            if stationary:
                self._baseline_samples.append((self._scan_values.copy(), q))
            sample['status'] = (
                f'BASELINE:{len(self._baseline_samples)}/{self._baseline_min}:stationary={stationary}')
            sample['position'] = q.tolist()
            self._publish_sample(sample)
            return
        if self._baseline is None or self._q0 is None:
            self._publish_sample(sample)
            return

        delta = self._scan_values - self._baseline
        calibrated_tau = apply_mregister_calibration(
            self._scan_values, self._baseline, q, self._q0,
            self._scale, self._coef)
        try:
            estimate = recover_calibrated_wrench(
                self._ik.compute_jacobian(q), calibrated_tau,
                damping=self._damping,
                characteristic_length_m=self._characteristic_length)
        except (ValueError, np.linalg.LinAlgError) as exc:
            sample['status'] = f'INVALID:wrench_recovery:{exc}'
            self._publish_sample(sample)
            return
        force_unfiltered = estimate.wrench[:3].copy()
        force = force_unfiltered.copy()
        force[np.abs(force) < self._deadband] = 0.0
        if not np.all(np.isfinite(force)) or np.linalg.norm(force) > self._max_force:
            sample['status'] = 'INVALID:force_limit'
            self._publish_sample(sample)
            return

        diagnostics = [estimate.sigma_min,
                       estimate.condition_number if math.isfinite(estimate.condition_number) else -1.0,
                       estimate.damping, estimate.relative_residual]
        sample.update(
            status='SHADOW_VALID:mregister_calibrated_pose',
            force=force.tolist(), force_unfiltered=force_unfiltered.tolist(),
            torque_nm=calibrated_tau.tolist(), position=q.tolist(),
            effort_raw=delta.tolist(), diagnostics=diagnostics,
            mregister_delta_nm=delta.tolist())

        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = 'base_link'
        wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = map(float, force)
        self._wrench_pub.publish(wrench)
        torque = JointState()
        torque.header = wrench.header
        torque.name = list(JOINT_NAMES)
        torque.position = q.tolist()
        torque.effort = calibrated_tau.tolist()
        self._torque_pub.publish(torque)
        self._diagnostics_pub.publish(Float64MultiArray(data=diagnostics))
        self._publish_sample(sample)

    def _restart_scan(self, now, reason):
        self._pending = None
        self._address_index = 0
        self._scan_registers = []
        self._scan_restarts += 1
        self._next_scan = now + self._scan_gap
        self._timeout_active = False
        if reason:
            self._errors[reason] = self._errors.get(reason, 0) + 1

    def _publish_stall(self, reason):
        """Make a stalled reader observable instead of silently emitting nothing."""
        status = f'INVALID:mregister_{reason}'
        sample = self._sample_template(self.get_clock().now().nanoseconds, status)
        sample['acquisition'] = self._acquisition_block()
        self._publish_sample(sample)

    def _tick(self):
        now = time.monotonic()
        if self._pending is not None:
            future, sent_monotonic, sent_ros_ns = self._pending
            if future.done():
                self._timeout_active = False
                try:
                    response = future.result()
                except Exception as exc:
                    self._restart_scan(now, 'service_error')
                    self.get_logger().error(f'M310 service error: {exc}')
                    return
                recv_ns = self.get_clock().now().nanoseconds
                self._pending = None
                if response is None or not response.success:
                    self._restart_scan(now, 'not_success')
                    return
                self._scan_registers.append({
                    'address': int(ADDRESSES[self._address_index]),
                    'raw': int(response.value),
                    'value_nm': (int(response.value) - 10000) * 0.1,
                    'request_ns': int(sent_ros_ns),
                    'response_ns': int(recv_ns),
                    'rtt_ms': (recv_ns - sent_ros_ns) / 1e6,
                    'request_monotonic': sent_monotonic,
                    'response_monotonic': now,
                })
                self._scan_raw[self._address_index] = int(response.value)
                self._scan_values[self._address_index] = (int(response.value) - 10000) * 0.1
                self._address_index += 1
                if self._address_index == len(ADDRESSES):
                    span_ms = (recv_ns - self._scan_first_ns) / 1e6
                    self._scan_id += 1
                    self._complete_scan((self._scan_first_ns + recv_ns) // 2, span_ms, recv_ns)
                    self._address_index = 0
                    self._scan_registers = []
                    self._scan_restarts = 0
                    self._next_scan = now + self._scan_gap
                    return
                # Send the next register in this tick; waiting for the following
                # tick added up to one timer period per register.
                self._send_request(now)
                return
            if now - sent_monotonic > self._request_timeout:
                if not self._timeout_active:
                    self._timeout_active = True
                    self._errors['request_timeout'] += 1
                    self.get_logger().error(
                        f'M310 request timeout on address {ADDRESSES[self._address_index]} '
                        f'after {self._request_timeout:.2f} s; recovering in '
                        f'{self._request_recovery:.2f} s')
                self._publish_stall('request_timeout')
                if now - sent_monotonic > self._request_timeout + self._request_recovery:
                    # Drop the unanswered request before re-arming so a lost
                    # response cannot leave two requests outstanding.
                    self._client.remove_pending_request(future)
                    self._restart_scan(now, 'abandoned_request')
                    self.get_logger().error(
                        'M310 request abandoned; restarting scan from address '
                        f'{ADDRESSES[0]}')
            return
        if now < self._next_scan or not self._client.service_is_ready():
            return
        if self._address_index == 0:
            self._scan_first_ns = self.get_clock().now().nanoseconds
            self._scan_registers = []
        self._send_request(now)

    def _send_request(self, now):
        request = ReadMRegister.Request(address=ADDRESSES[self._address_index])
        self._pending = (self._client.call_async(request), now,
                         self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    node = MRegisterForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
