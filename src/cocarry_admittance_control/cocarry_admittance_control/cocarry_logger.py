#!/usr/bin/env python3
"""CSV logger dedicated to the independent 3D co-carrying pipeline."""

import csv
from datetime import datetime
import os
import json
import time
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped, WrenchStamped
from human_hand_msgs.msg import HandPrediction
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from std_srvs.srv import SetBool
from .force_log_sample import validate_force_sample, force_log_record


ROBOT_JOINT_NAMES = (
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
)


class CocarryAdmittanceLogger(Node):
    def __init__(self):
        super().__init__('cocarry_admittance_logger')
        self.declare_parameter('log_dir', '/home/hungnb/cocarry_ws/cocarry_logs')
        self.declare_parameter('file_prefix', 'cocarry_admittance_3d')
        self.declare_parameter('robot_force_sample_timeout_sec', 0.25)
        self.declare_parameter('robot_force_log_unfiltered', True)
        self._force_log_unfiltered = bool(self.get_parameter('robot_force_log_unfiltered').value)
        self._force_sample_timeout = float(self.get_parameter('robot_force_sample_timeout_sec').value)
        if not math.isfinite(self._force_sample_timeout) or self._force_sample_timeout <= 0:
            raise ValueError('robot_force_sample_timeout_sec must be finite and positive')
        self._log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        self._file_prefix = str(self.get_parameter('file_prefix').value).strip()
        if not self._file_prefix or os.path.basename(self._file_prefix) != self._file_prefix:
            raise ValueError('file_prefix must be a non-empty filename prefix')
        os.makedirs(self._log_dir, exist_ok=True)
        self._logging = False
        self._rows = []
        self._path = ''
        self._latest = {}
        self._hybrid_events = []
        self._hybrid_event_key = None
        self._force_sample = None
        self._force_sample_received = 0.0

        self.create_subscription(PoseStamped, '/cartesian_streamer/current_pose', self._ee, 10)
        self.create_subscription(PointStamped, '/cocarry/nominal_position', self._nominal, 10)
        self.create_subscription(PointStamped, '/cocarry/reference_position', self._reference, 20)
        self.create_subscription(Vector3Stamped, '/cocarry/admittance_error', self._error, 10)
        self.create_subscription(Vector3Stamped, '/axia/human_force', self._human_force, 20)
        self.create_subscription(
            Vector3Stamped, '/cocarry/effective_force', self._effective_force, 20)
        self.create_subscription(
            Float32, '/cocarry/z_deadzone_weight', self._z_deadzone_weight, 10)
        self.create_subscription(WrenchStamped, '/sensorless_force', self._robot_force, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state, qos_profile_sensor_data)
        self.create_subscription(String, '/sensorless_force/sample', self._robot_force_sample, 20)
        self.create_subscription(
            JointState, '/sensorless_force/joint_torque', self._joint_torque, 10)
        self.create_subscription(
            Bool, '/sensorless_force/valid', self._robot_force_valid, 10)
        self.create_subscription(
            String, '/sensorless_force/status', self._robot_force_status, 10)
        self.create_subscription(
            Float64MultiArray, '/sensorless_force/diagnostics',
            self._robot_force_diagnostics, 10)
        self.create_subscription(HandPrediction, '/ml/predicted_position', self._prediction, 10)
        self.create_subscription(
            HandPrediction, '/ml/raw_predicted_position', self._raw_prediction, 10)
        self.create_subscription(String, '/predictor/hybrid_state', self._role, 5)
        self.create_subscription(String, '/cocarry/hybrid_status', self._manual_status, 10)
        self.create_subscription(String, '/cartesian_streamer/motion_diagnostics',
                                 self._motion_diagnostics, 10)
        self.create_subscription(Float32, '/cocarry/force_age_ms', self._force_age, 10)
        self.create_subscription(Float32, '/cocarry/pose_age_ms', self._pose_age, 10)
        self.create_subscription(
            Float32, '/cocarry/prediction_age_ms', self._prediction_age, 10)
        self.create_subscription(Float32, '/axia/udp_gap_ms', self._udp_gap, 10)
        self.create_service(SetBool, '/logger/toggle', self._toggle)
        self.get_logger().info(f'3D co-carrying logger ready: {self._log_dir}')

    def _motion_diagnostics(self, msg):
        self._latest['motion_diagnostics'] = msg.data

    def _ee(self, msg):
        self._latest['ee'] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _nominal(self, msg):
        self._latest['nominal'] = (msg.point.x, msg.point.y, msg.point.z)

    def _error(self, msg):
        self._latest['error'] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _human_force(self, msg):
        self._latest['fh'] = (msg.vector.x, msg.vector.y, msg.vector.z)
        self._latest['fh_ts'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _effective_force(self, msg):
        self._latest['effective_force'] = (
            msg.vector.x, msg.vector.y, msg.vector.z)

    def _z_deadzone_weight(self, msg):
        self._latest['z_deadzone_weight'] = msg.data

    def _robot_force(self, msg):
        self._latest['fr'] = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
        self._latest['fr_ts'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _robot_force_sample(self, msg):
        try:
            self._force_sample = validate_force_sample(json.loads(msg.data))
            self._force_sample_received = time.monotonic()
        except (ValueError, TypeError, KeyError):
            self._force_sample = None
            self.get_logger().warn('Invalid atomic robot-force sample', throttle_duration_sec=1.0)

    @staticmethod
    def _ordered_joint_field(msg, field):
        values = getattr(msg, field)
        if not values:
            return None
        lookup = {name: index for index, name in enumerate(msg.name)}
        try:
            indices = [lookup[name] for name in ROBOT_JOINT_NAMES]
            if max(indices) >= len(values):
                return None
            return tuple(values[index] for index in indices)
        except (KeyError, IndexError):
            return None

    def _joint_state(self, msg):
        self._latest['joint_position'] = self._ordered_joint_field(msg, 'position')
        self._latest['joint_velocity'] = self._ordered_joint_field(msg, 'velocity')
        self._latest['joint_effort_raw'] = self._ordered_joint_field(msg, 'effort')
        self._latest['joint_ts'] = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

    def _joint_torque(self, msg):
        self._latest['joint_torque_nm'] = self._ordered_joint_field(msg, 'effort')

    def _robot_force_valid(self, msg):
        # False means the estimate must not select LEADER/FOLLOWER. Keep the
        # diagnostic values in the CSV so they can still be calibrated offline.
        self._latest['fr_valid'] = bool(msg.data)

    def _robot_force_status(self, msg):
        self._latest['fr_status'] = msg.data

    def _robot_force_diagnostics(self, msg):
        if len(msg.data) >= 4:
            self._latest['fr_diagnostics'] = tuple(msg.data[:4])

    def _prediction(self, msg):
        self._latest['pred'] = (msg.x, msg.y, msg.z)
        self._latest['inference_ms'] = msg.inference_time_ms
        self._latest['model'] = msg.model_name

    def _raw_prediction(self, msg):
        self._latest['raw_pred'] = (msg.x, msg.y, msg.z)

    def _force_age(self, msg):
        self._latest['force_age_ms'] = msg.data

    def _pose_age(self, msg):
        self._latest['pose_age_ms'] = msg.data

    def _prediction_age(self, msg):
        self._latest['prediction_age_ms'] = msg.data

    def _udp_gap(self, msg):
        self._latest['udp_gap_ms'] = msg.data

    def _role(self, msg):
        self._latest['role'] = msg.data

    def _manual_status(self, msg):
        self._latest['hybrid_status'] = msg.data
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self._latest['manual_role'] = data.get('role') if data.get('enabled') else None
        key = tuple(data.get(k) for k in
                    ('enabled', 'role', 'selected', 'active', 'leg', 'reason', 'controller_state',
                     'test_mode', 'test_fired', 'control_source', 'control_phase',
                     'bridge_continuity', 'transition', 'reentry_phase',
                     'reentry_token'))
        if self._logging and key != self._hybrid_event_key:
            self._hybrid_events.append(data)
        self._hybrid_event_key = key

    @staticmethod
    def _values(value, size):
        return list(value) if value is not None else [''] * size

    def _reference(self, msg):
        if not self._logging:
            return
        now_ns = self.get_clock().now().nanoseconds
        fr = force_log_record(self._force_sample, now_ns,
                              time.monotonic() - self._force_sample_received,
                              self._force_sample_timeout)
        # Calibration acquisition must not erase small signals with deadband.
        # Keep the standard estimator topic and its safety rejection unchanged.
        force_values = fr.get('force')
        if self._force_log_unfiltered and force_values is not None:
            force_values = fr.get('force_unfiltered')
        self._rows.append([
            now_ns,
            *self._values(self._latest.get('ee'), 3),
            *self._values(self._latest.get('pred'), 3),
            *self._values(self._latest.get('nominal'), 3),
            *self._values(self._latest.get('error'), 3),
            msg.point.x, msg.point.y, msg.point.z,
            *self._values(self._latest.get('fh'), 3),
            *self._values(self._latest.get('effective_force'), 3),
            self._latest.get('z_deadzone_weight', ''),
            *self._values(force_values, 3),
            self._latest.get('inference_ms', ''),
            self._latest.get('model', ''),
            self._latest.get('manual_role') or self._latest.get('role', ''),
            self._latest.get('fh_ts', ''),
            fr.get('stamp_ns', ''),
            *self._values(self._latest.get('raw_pred'), 3),
            self._latest.get('force_age_ms', ''),
            self._latest.get('pose_age_ms', ''),
            self._latest.get('prediction_age_ms', ''),
            self._latest.get('udp_gap_ms', ''),
            *self._values(self._latest.get('joint_position'), 6),
            *self._values(self._latest.get('joint_velocity'), 6),
            *self._values(self._latest.get('joint_effort_raw'), 6),
            *self._values(fr.get('torque_nm'), 6),
            self._latest.get('joint_ts', ''),
            fr.get('role_valid', False),
            fr['status'],
            *self._values(fr.get('diagnostics'), 4),
            fr.get('age_ms', ''), fr.get('mode', ''), fr.get('frame', ''),
            fr.get('calibration_confirmed', False),
            *self._values(fr.get('force_unfiltered'), 3),
            *self._values(fr.get('position'), 6),
            *self._values(fr.get('effort_raw'), 6),
            *self._values(fr.get('scale_nm_per_effort'), 6),
            *self._values(fr.get('bias_nm'), 6),
            'pre_deadband' if self._force_log_unfiltered else 'post_deadband',
            self._latest.get('hybrid_status', ''),
            self._latest.get('motion_diagnostics', ''),
        ])

    def _toggle(self, request, response):
        if request.data and not self._logging:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._path = os.path.join(
                self._log_dir, f'{self._file_prefix}_{stamp}.csv')
            self._rows = []
            self._hybrid_events = []
            self._hybrid_event_key = None
            self._logging = True
            response.success = True
            response.message = f'Logging to {self._path}'
        elif not request.data and self._logging:
            self._logging = False
            self._write()
            response.success = True
            response.message = f'Saved {self._path}'
        else:
            response.success = True
            response.message = 'Logger state unchanged'
        return response

    def _write(self):
        if self._hybrid_events:
            with open(self._path + '.hybrid_events.json', 'w') as stream:
                json.dump(self._hybrid_events, stream, indent=2)
        if not self._rows:
            self.get_logger().warn('No 3D co-carrying samples to write')
            return
        header = [
            'ros_timestamp_ns',
            'actual_ee_x', 'actual_ee_y', 'actual_ee_z',
            'predicted_xd_relative', 'predicted_yd_relative', 'predicted_zd_relative',
            'nominal_xd', 'nominal_yd', 'nominal_zd',
            'admittance_error_x', 'admittance_error_y', 'admittance_error_z',
            'reference_xr', 'reference_yr', 'reference_zr',
            'f_human_x', 'f_human_y', 'f_human_z',
            'f_effective_x', 'f_effective_y', 'f_effective_z',
            'z_deadzone_weight',
            'f_robot_x', 'f_robot_y', 'f_robot_z',
            'inference_ms', 'model', 'role', 'fh_timestamp_ns', 'fr_timestamp_ns',
            'raw_predicted_xd_relative', 'raw_predicted_yd_relative',
            'raw_predicted_zd_relative', 'force_age_ms', 'pose_age_ms',
            'prediction_age_ms', 'udp_gap_ms',
            'joint_position_j1', 'joint_position_j2', 'joint_position_j3',
            'joint_position_j4', 'joint_position_j5', 'joint_position_j6',
            'joint_velocity_j1', 'joint_velocity_j2', 'joint_velocity_j3',
            'joint_velocity_j4', 'joint_velocity_j5', 'joint_velocity_j6',
            'joint_effort_raw_j1', 'joint_effort_raw_j2', 'joint_effort_raw_j3',
            'joint_effort_raw_j4', 'joint_effort_raw_j5', 'joint_effort_raw_j6',
            'joint_torque_est_nm_j1', 'joint_torque_est_nm_j2',
            'joint_torque_est_nm_j3', 'joint_torque_est_nm_j4',
            'joint_torque_est_nm_j5', 'joint_torque_est_nm_j6',
            'joint_timestamp_ns', 'f_robot_ready_for_role_selection',
            'f_robot_status', 'f_robot_sigma_min', 'f_robot_condition_number',
            'f_robot_damping', 'f_robot_relative_residual',
            'f_robot_age_ms', 'f_robot_effort_unit_mode', 'f_robot_frame',
            'f_robot_calibration_confirmed',
            'f_robot_unfiltered_x', 'f_robot_unfiltered_y', 'f_robot_unfiltered_z',
            *[f'f_robot_source_joint_position_j{i}' for i in range(1, 7)],
            *[f'f_robot_source_effort_raw_j{i}' for i in range(1, 7)],
            *[f'f_robot_scale_nm_per_effort_j{i}' for i in range(1, 7)],
            *[f'f_robot_bias_nm_j{i}' for i in range(1, 7)],
            'f_robot_output_stage',
            'hybrid_status_json',
            'motion_diagnostics_json',
        ]
        with open(self._path, 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self._rows)
        self.get_logger().info(f'Wrote {len(self._rows)} rows to {self._path}')


def main(args=None):
    rclpy.init(args=args)
    node = CocarryAdmittanceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._logging:
            node._logging = False
            node._write()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
