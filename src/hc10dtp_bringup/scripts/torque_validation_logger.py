#!/usr/bin/env python3
"""Passive logger for validating joint-current torque against ATI Axia data.

It never publishes motion commands and does not alter admittance, robot-force
estimation, Axia calibration, or controller parameters.  Rows are triggered by
the actual /joint_states stream so one robot state is kept intact per row.
"""

import csv
from datetime import datetime
import math
import os
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped, WrenchStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import tf2_ros


JOINT_NAMES = (
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
)


class TorqueValidationLogger(Node):
    def __init__(self):
        super().__init__('torque_validation_logger')
        self.declare_parameter(
            'log_dir', '/home/hungnb/cocarry_ws/cocarry_logs/torque_validation')
        self.declare_parameter('file_prefix', 'torque_validation')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_frame', 'axia_sensor_link')
        self.declare_parameter('max_axia_age_sec', 0.25)
        self._log_dir = os.path.expanduser(str(self.get_parameter('log_dir').value))
        self._prefix = str(self.get_parameter('file_prefix').value).strip()
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._sensor_frame = str(self.get_parameter('sensor_frame').value)
        self._max_axia_age_sec = float(self.get_parameter('max_axia_age_sec').value)
        if not self._prefix or os.path.basename(self._prefix) != self._prefix:
            raise ValueError('file_prefix must be a filename prefix')
        if not math.isfinite(self._max_axia_age_sec) or self._max_axia_age_sec <= 0.0:
            raise ValueError('max_axia_age_sec must be finite and positive')
        os.makedirs(self._log_dir, exist_ok=True)

        self._logging = False
        self._rows = []
        self._path = ''
        self._indices = None
        self._names = None
        self._raw_wrench = None
        self._raw_wrench_received = 0.0
        self._human_force = None
        self._axia_connected = False
        self._axia_calibrated = False
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            JointState, '/joint_states', self._joint_state, qos_profile_sensor_data)
        self.create_subscription(
            WrenchStamped, '/axia/raw_wrench', self._raw_axia, 50)
        self.create_subscription(
            Vector3Stamped, '/axia/human_force', self._human_axia, 50)
        self.create_subscription(Bool, '/axia/connected', self._connected, 5)
        self.create_subscription(Bool, '/axia/calibrated', self._calibrated, 5)
        self.create_service(SetBool, '/torque_validation_logger/toggle', self._toggle)
        self.get_logger().info(
            'Torque validation logger ready. Passive service: '
            '/torque_validation_logger/toggle')

    @staticmethod
    def _stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _raw_axia(self, msg):
        self._raw_wrench = msg
        self._raw_wrench_received = time.monotonic()

    def _human_axia(self, msg):
        self._human_force = msg

    def _connected(self, msg):
        self._axia_connected = bool(msg.data)

    def _calibrated(self, msg):
        self._axia_calibrated = bool(msg.data)

    def _joint_indices(self, msg):
        names = tuple(msg.name)
        if names == self._names and self._indices is not None:
            return self._indices
        lookup = {name: index for index, name in enumerate(names)}
        missing = [name for name in JOINT_NAMES if name not in lookup]
        if missing:
            raise ValueError(f'missing HC10DTP joints: {missing}')
        self._names = names
        self._indices = tuple(lookup[name] for name in JOINT_NAMES)
        return self._indices

    @staticmethod
    def _ordered(values, indices):
        if not values or max(indices) >= len(values):
            return [''] * 6
        return [values[index] for index in indices]

    def _base_to_sensor(self):
        try:
            return self._tf_buffer.lookup_transform(
                self._sensor_frame, self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except Exception:
            return None

    @staticmethod
    def _transform_values(transform):
        if transform is None:
            return [''] * 7
        t = transform.transform.translation
        q = transform.transform.rotation
        return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

    def _joint_state(self, msg):
        if not self._logging:
            return
        try:
            indices = self._joint_indices(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        now_ns = self.get_clock().now().nanoseconds
        raw = self._raw_wrench
        raw_age_ms = ''
        raw_values = [''] * 6
        raw_stamp_ns = ''
        if raw is not None:
            raw_age_ms = (time.monotonic() - self._raw_wrench_received) * 1000.0
            raw_stamp_ns = self._stamp_ns(raw.header)
            raw_values = [
                raw.wrench.force.x, raw.wrench.force.y, raw.wrench.force.z,
                raw.wrench.torque.x, raw.wrench.torque.y, raw.wrench.torque.z,
            ]
        human_values = [''] * 3
        human_stamp_ns = ''
        if self._human_force is not None:
            human_stamp_ns = self._stamp_ns(self._human_force.header)
            human_values = [
                self._human_force.vector.x, self._human_force.vector.y,
                self._human_force.vector.z,
            ]
        transform = self._base_to_sensor()
        fresh = bool(raw is not None and raw_age_ms <= self._max_axia_age_sec * 1000.0)
        self._rows.append([
            now_ns, self._stamp_ns(msg.header),
            *self._ordered(msg.position, indices),
            *self._ordered(msg.velocity, indices),
            *self._ordered(msg.effort, indices),
            raw_stamp_ns, raw_age_ms, fresh,
            *raw_values, human_stamp_ns, *human_values,
            self._axia_connected, self._axia_calibrated,
            *self._transform_values(transform),
        ])

    def _toggle(self, request, response):
        if request.data and not self._logging:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._path = os.path.join(self._log_dir, f'{self._prefix}_{stamp}.csv')
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
            self.get_logger().warn('No joint-state rows were received')
            return
        header = [
            'ros_timestamp_ns', 'joint_timestamp_ns',
            *[f'joint_position_j{i}' for i in range(1, 7)],
            *[f'joint_velocity_j{i}' for i in range(1, 7)],
            *[f'joint_effort_j{i}' for i in range(1, 7)],
            'axia_raw_timestamp_ns', 'axia_raw_age_ms', 'axia_raw_fresh',
            'axia_raw_fx', 'axia_raw_fy', 'axia_raw_fz',
            'axia_raw_tx', 'axia_raw_ty', 'axia_raw_tz',
            'axia_human_timestamp_ns',
            'axia_human_fx', 'axia_human_fy', 'axia_human_fz',
            'axia_connected', 'axia_calibrated',
            'base_to_sensor_x', 'base_to_sensor_y', 'base_to_sensor_z',
            'base_to_sensor_qx', 'base_to_sensor_qy', 'base_to_sensor_qz',
            'base_to_sensor_qw',
        ]
        with open(self._path, 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self._rows)
        self.get_logger().info(f'Wrote {len(self._rows)} rows to {self._path}')


def main(args=None):
    rclpy.init(args=args)
    node = TorqueValidationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._logging:
            node._logging = False
            node._write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
