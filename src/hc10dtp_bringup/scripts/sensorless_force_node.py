#!/usr/bin/env python3
"""Estimate the robot actuator-equivalent Cartesian force from joint effort.

This node is diagnostic only. It does not send any command to the robot and
its output is not connected to the admittance controller. The convention is
``tau_robot ~= J.T @ W_robot``; no implicit sign reversal is applied.
"""

import math
import json

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from local_ik_solver import LocalIKSolver
from sensorless_force_math import (
    EFFORT_MODES,
    HC10DTP_RATED_TORQUE_NM,
    effort_to_joint_torque,
    estimate_robot_wrench,
)


JOINT_NAMES = (
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
)


class SensorlessForceNode(Node):
    def __init__(self):
        super().__init__('sensorless_force_node')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool0')
        self.declare_parameter('deadband_n', 1.0)
        self.declare_parameter('max_force_norm_n', 500.0)
        self.declare_parameter('effort_unit_mode', 'raw_only')
        self.declare_parameter('calibration_confirmed', False)
        self.declare_parameter(
            'rated_joint_torques_nm', HC10DTP_RATED_TORQUE_NM.tolist())
        self.declare_parameter('custom_effort_scale_nm', [1.0] * 6)
        self.declare_parameter('joint_torque_bias_nm', [0.0] * 6)
        self.declare_parameter('damping_min', 0.002)
        self.declare_parameter('damping_max', 0.08)
        self.declare_parameter('singularity_threshold', 0.05)
        self.declare_parameter('publish_rate_hz', 15.0)

        self._base_link = str(self.get_parameter('base_link').value)
        tip_link = str(self.get_parameter('tip_link').value)
        if self._base_link != 'base_link' or tip_link != 'tool0':
            raise ValueError(
                'The shared LocalIKSolver Jacobian is verified only for '
                'base_link -> tool0')
        self._deadband = float(self.get_parameter('deadband_n').value)
        self._max_force_norm = float(self.get_parameter('max_force_norm_n').value)
        self._mode = str(self.get_parameter('effort_unit_mode').value)
        self._calibration_confirmed = bool(
            self.get_parameter('calibration_confirmed').value)
        if self._mode not in EFFORT_MODES:
            raise ValueError(
                f'effort_unit_mode must be one of {EFFORT_MODES}, got {self._mode!r}')
        self._rated_torque = self._six_vector('rated_joint_torques_nm')
        self._custom_scale = self._six_vector('custom_effort_scale_nm')
        self._torque_bias = self._six_vector('joint_torque_bias_nm')
        self._damping_min = float(self.get_parameter('damping_min').value)
        self._damping_max = float(self.get_parameter('damping_max').value)
        self._singularity_threshold = float(
            self.get_parameter('singularity_threshold').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._minimum_period_ns = int(1e9 / publish_rate_hz)
        self._last_process_ns = 0

        self._ik = LocalIKSolver()
        self._joint_indices = None
        self._last_names = None
        self._last_status = ''

        self._wrench_pub = self.create_publisher(
            WrenchStamped, '/sensorless_force', 10)
        self._torque_pub = self.create_publisher(
            JointState, '/sensorless_force/joint_torque', 10)
        self._valid_pub = self.create_publisher(
            Bool, '/sensorless_force/valid', 10)
        self._status_pub = self.create_publisher(
            String, '/sensorless_force/status', 10)
        self._diagnostics_pub = self.create_publisher(
            Float64MultiArray, '/sensorless_force/diagnostics', 10)
        # One diagnostic record binds force, units, health and source joints.
        self._sample_pub = self.create_publisher(String, '/sensorless_force/sample', 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state, qos_profile_sensor_data)

        if self._mode == 'raw_only':
            self.get_logger().warning(
                'Sensorless force is in raw_only mode: raw joint efforts can be '
                'logged, but no Cartesian force is published until the effort '
                'unit/conversion is explicitly selected.')
        elif self._mode == 'normalized_rated_torque':
            self.get_logger().warning(
                'Using provisional URDF rated-torque scaling. This output is '
                'UNCALIBRATED and must not drive LEADER/FOLLOWER selection yet.')
        else:
            self.get_logger().info(
                f'Sensorless force estimator ready in {self._mode} mode')

    def _six_vector(self, name):
        values = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError(f'{name} must contain six finite values')
        return values

    def _publish_health(self, valid, status):
        self._sample.update(role_valid=bool(valid), status=status)
        self._sample_pub.publish(String(data=json.dumps(self._sample, allow_nan=False)))
        valid_msg = Bool()
        valid_msg.data = bool(valid)
        self._valid_pub.publish(valid_msg)
        # Repeat status so a late-starting logger still receives it.
        status_msg = String()
        status_msg.data = status
        self._status_pub.publish(status_msg)
        if status != self._last_status:
            if valid:
                self.get_logger().info(status)
            else:
                self.get_logger().warning(status)
            self._last_status = status

    def _indices_for(self, msg):
        names = tuple(msg.name)
        if names == self._last_names and self._joint_indices is not None:
            return self._joint_indices
        lookup = {name: index for index, name in enumerate(names)}
        if any(name not in lookup for name in JOINT_NAMES):
            missing = [name for name in JOINT_NAMES if name not in lookup]
            raise ValueError(f'missing joints in /joint_states: {missing}')
        self._last_names = names
        self._joint_indices = tuple(lookup[name] for name in JOINT_NAMES)
        return self._joint_indices

    def _joint_state(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_process_ns < self._minimum_period_ns:
            return
        self._last_process_ns = now_ns
        self._sample = {
            'schema': 1,
            'stamp_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            'frame': self._base_link, 'mode': self._mode,
            'calibration_confirmed': self._calibration_confirmed,
            'force': None, 'force_unfiltered': None, 'torque_nm': None,
            'position': None, 'effort_raw': None, 'diagnostics': None,
            'scale_nm_per_effort': (self._rated_torque.tolist()
                if self._mode == 'normalized_rated_torque' else
                self._custom_scale.tolist() if self._mode == 'custom_scale' else
                [1.0] * 6 if self._mode == 'torque_nm' else None),
            'bias_nm': self._torque_bias.tolist(),
        }
        try:
            indices = self._indices_for(msg)
            if not msg.effort or max(indices) >= len(msg.effort):
                self._publish_health(False, 'INVALID:no_joint_effort')
                return
            if max(indices) >= len(msg.position):
                self._publish_health(False, 'INVALID:no_joint_position')
                return
            q = np.asarray([msg.position[i] for i in indices], dtype=np.float64)
            raw_effort = np.asarray([msg.effort[i] for i in indices], dtype=np.float64)
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(raw_effort)):
                self._publish_health(False, 'INVALID:non_finite_joint_state')
                return
        except (IndexError, ValueError) as exc:
            self._publish_health(False, f'INVALID:{exc}')
            return

        self._sample.update(position=q.tolist(), effort_raw=raw_effort.tolist())
        if self._mode == 'raw_only':
            self._publish_health(False, 'RAW_ONLY:conversion_not_confirmed')
            return

        try:
            torque_nm = effort_to_joint_torque(
                raw_effort,
                self._mode,
                rated_torque_nm=self._rated_torque,
                custom_scale_nm=self._custom_scale,
                bias_nm=self._torque_bias,
            )
            jacobian = self._ik.compute_jacobian(q)
            estimate = estimate_robot_wrench(
                jacobian,
                torque_nm,
                damping_min=self._damping_min,
                damping_max=self._damping_max,
                singularity_threshold=self._singularity_threshold,
            )
        except (ValueError, np.linalg.LinAlgError) as exc:
            self._publish_health(False, f'INVALID:estimation_failed:{exc}')
            return

        force = estimate.wrench[:3].copy()
        # Preserve pre-deadband data even if the standard output is rejected.
        if np.all(np.isfinite(force)):
            self._sample.update(
                force_unfiltered=force.tolist(), torque_nm=torque_nm.tolist(),
                diagnostics=[estimate.sigma_min,
                    estimate.condition_number if math.isfinite(estimate.condition_number) else -1.0,
                    estimate.damping, estimate.relative_residual])
        force[np.abs(force) < self._deadband] = 0.0
        force_norm = float(np.linalg.norm(force))
        if not np.all(np.isfinite(force)) or force_norm > self._max_force_norm:
            self._publish_health(
                False, f'INVALID:force_norm={force_norm:.3f}_N')
            return

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = msg.header.stamp
        wrench_msg.header.frame_id = self._base_link
        wrench_msg.wrench.force.x = float(force[0])
        wrench_msg.wrench.force.y = float(force[1])
        wrench_msg.wrench.force.z = float(force[2])
        # Moments are solved internally, but intentionally not exposed/used.
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0
        self._wrench_pub.publish(wrench_msg)

        torque_msg = JointState()
        torque_msg.header = msg.header
        torque_msg.name = list(JOINT_NAMES)
        torque_msg.position = q.tolist()
        torque_msg.effort = torque_nm.tolist()
        self._torque_pub.publish(torque_msg)

        diagnostics = Float64MultiArray()
        diagnostics.data = [
            estimate.sigma_min,
            estimate.condition_number if math.isfinite(estimate.condition_number) else -1.0,
            estimate.damping,
            estimate.relative_residual,
        ]
        self._diagnostics_pub.publish(diagnostics)

        quality = 'VALID' if self._calibration_confirmed else 'UNCALIBRATED'
        self._sample['force'] = force.tolist()
        self._publish_health(
            self._calibration_confirmed, f'{quality}:{self._mode}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorlessForceNode()
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
