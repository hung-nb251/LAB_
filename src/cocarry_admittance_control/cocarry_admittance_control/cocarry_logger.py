#!/usr/bin/env python3
"""CSV logger dedicated to the independent 3D co-carrying pipeline."""

import csv
from datetime import datetime
import os

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped, WrenchStamped
from human_hand_msgs.msg import HandPrediction
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from std_srvs.srv import SetBool


ROBOT_JOINT_NAMES = (
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
)


class CocarryAdmittanceLogger(Node):
    def __init__(self):
        super().__init__('cocarry_admittance_logger')
        self.declare_parameter('log_dir', '/home/hungnb/cocarry_ws/cocarry_logs')
        self.declare_parameter('file_prefix', 'cocarry_admittance_3d')
        self._log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        self._file_prefix = str(self.get_parameter('file_prefix').value).strip()
        if not self._file_prefix or os.path.basename(self._file_prefix) != self._file_prefix:
            raise ValueError('file_prefix must be a non-empty filename prefix')
        os.makedirs(self._log_dir, exist_ok=True)
        self._logging = False
        self._rows = []
        self._path = ''
        self._latest = {}

        self.create_subscription(PoseStamped, '/cartesian_streamer/current_pose', self._ee, 10)
        self.create_subscription(PointStamped, '/cocarry/nominal_position', self._nominal, 10)
        self.create_subscription(PointStamped, '/cocarry/reference_position', self._reference, 20)
        self.create_subscription(Vector3Stamped, '/cocarry/admittance_error', self._error, 10)
        self.create_subscription(Vector3Stamped, '/axia/human_force', self._human_force, 20)
        self.create_subscription(WrenchStamped, '/sensorless_force', self._robot_force, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state, 20)
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
        self.create_subscription(Float32, '/cocarry/force_age_ms', self._force_age, 10)
        self.create_subscription(
            Float32, '/cocarry/prediction_age_ms', self._prediction_age, 10)
        self.create_subscription(Float32, '/axia/udp_gap_ms', self._udp_gap, 10)
        self.create_service(SetBool, '/logger/toggle', self._toggle)
        self.get_logger().info(f'3D co-carrying logger ready: {self._log_dir}')

    def _ee(self, msg):
        self._latest['ee'] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _nominal(self, msg):
        self._latest['nominal'] = (msg.point.x, msg.point.y, msg.point.z)

    def _error(self, msg):
        self._latest['error'] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _human_force(self, msg):
        self._latest['fh'] = (msg.vector.x, msg.vector.y, msg.vector.z)
        self._latest['fh_ts'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _robot_force(self, msg):
        self._latest['fr'] = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
        self._latest['fr_ts'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

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

    def _prediction_age(self, msg):
        self._latest['prediction_age_ms'] = msg.data

    def _udp_gap(self, msg):
        self._latest['udp_gap_ms'] = msg.data

    def _role(self, msg):
        self._latest['role'] = msg.data

    @staticmethod
    def _values(value, size):
        return list(value) if value is not None else [''] * size

    def _reference(self, msg):
        if not self._logging:
            return
        self._rows.append([
            self.get_clock().now().nanoseconds,
            *self._values(self._latest.get('ee'), 3),
            *self._values(self._latest.get('pred'), 3),
            *self._values(self._latest.get('nominal'), 3),
            *self._values(self._latest.get('error'), 3),
            msg.point.x, msg.point.y, msg.point.z,
            *self._values(self._latest.get('fh'), 3),
            *self._values(self._latest.get('fr'), 3),
            self._latest.get('inference_ms', ''),
            self._latest.get('model', ''),
            self._latest.get('role', ''),
            self._latest.get('fh_ts', ''),
            self._latest.get('fr_ts', ''),
            *self._values(self._latest.get('raw_pred'), 3),
            self._latest.get('force_age_ms', ''),
            self._latest.get('prediction_age_ms', ''),
            self._latest.get('udp_gap_ms', ''),
            *self._values(self._latest.get('joint_position'), 6),
            *self._values(self._latest.get('joint_velocity'), 6),
            *self._values(self._latest.get('joint_effort_raw'), 6),
            *self._values(self._latest.get('joint_torque_nm'), 6),
            self._latest.get('joint_ts', ''),
            self._latest.get('fr_valid', False),
            self._latest.get('fr_status', ''),
            *self._values(self._latest.get('fr_diagnostics'), 4),
        ])

    def _toggle(self, request, response):
        if request.data and not self._logging:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._path = os.path.join(
                self._log_dir, f'{self._file_prefix}_{stamp}.csv')
            self._rows = []
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
            'f_robot_x', 'f_robot_y', 'f_robot_z',
            'inference_ms', 'model', 'role', 'fh_timestamp_ns', 'fr_timestamp_ns',
            'raw_predicted_xd_relative', 'raw_predicted_yd_relative',
            'raw_predicted_zd_relative', 'force_age_ms',
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
