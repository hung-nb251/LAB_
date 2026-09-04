#!/usr/bin/env python3
"""Fuse robot-EE prediction and ATI force into a planar pose reference."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from human_hand_msgs.msg import HandPrediction, HandState

from .admittance import PlanarAdmittance, soft_radial_deadzone


def quaternion_matrix(x, y, z, w):
    """Return a 3x3 rotation matrix for a normalized quaternion."""
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=float)


class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')

        defaults = {
            'control_rate_hz': 15.0,
            'virtual_mass_x': 2.0, 'virtual_mass_y': 2.0,
            'damping_x': 15.0, 'damping_y': 15.0,
            'stiffness_x': 20.0, 'stiffness_y': 20.0,
            # Axia UI đã áp deadband 4 N; không deadzone lần hai.
            'intent_threshold_n': 0.0,
            'force_sign_x': 1.0,
            'force_sign_y': 1.0,
            'max_virtual_velocity_mps': 0.05,
            'max_virtual_acceleration_mps2': 0.20,
            'workspace_half_width_x_m': 0.5,
            'workspace_forward_y_m': 1.0,
            'forward_sign_y': 1.0,
            'safe_workspace_x_min_m': -1.4,
            'safe_workspace_x_max_m': 1.4,
            'safe_workspace_y_min_m': -0.5,
            'safe_workspace_y_max_m': 1.3,
            'safe_workspace_z_min_m': 0.05,
            'safe_workspace_z_max_m': 1.5,
            'tool_tip_offset_x_m': 0.0,
            'tool_tip_offset_y_m': 0.0,
            'tool_tip_offset_z_m': -0.18,
            'max_planar_force_n': 15.0,
            'max_normal_force_n': 15.0,
            'force_timeout_sec': 0.20,
            'current_pose_timeout_sec': 0.25,
            'actual_z_tolerance_m': 0.010,
            'prediction_timeout_sec': 0.50,
            'auto_capture_on_start': True,
            'require_axia_calibrated': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        gp = lambda name: self.get_parameter(name).value
        self._rate = float(gp('control_rate_hz'))
        self._intent_threshold = float(gp('intent_threshold_n'))
        self._force_sign = np.array([
            1.0 if float(gp('force_sign_x')) >= 0.0 else -1.0,
            1.0 if float(gp('force_sign_y')) >= 0.0 else -1.0,
        ])
        self._workspace_half_x = float(gp('workspace_half_width_x_m'))
        self._workspace_forward_y = float(gp('workspace_forward_y_m'))
        self._forward_sign = 1.0 if float(gp('forward_sign_y')) >= 0.0 else -1.0
        self._safe_x = np.array([
            gp('safe_workspace_x_min_m'), gp('safe_workspace_x_max_m')
        ], dtype=float)
        self._safe_y = np.array([
            gp('safe_workspace_y_min_m'), gp('safe_workspace_y_max_m')
        ], dtype=float)
        self._safe_z = np.array([
            gp('safe_workspace_z_min_m'), gp('safe_workspace_z_max_m')
        ], dtype=float)
        self._tip_offset_local = np.array([
            gp('tool_tip_offset_x_m'),
            gp('tool_tip_offset_y_m'),
            gp('tool_tip_offset_z_m'),
        ], dtype=float)
        self._max_planar_force = float(gp('max_planar_force_n'))
        self._max_normal_force = float(gp('max_normal_force_n'))
        self._force_timeout = float(gp('force_timeout_sec'))
        self._current_pose_timeout = float(gp('current_pose_timeout_sec'))
        self._actual_z_tolerance = float(gp('actual_z_tolerance_m'))
        self._prediction_timeout = float(gp('prediction_timeout_sec'))
        self._auto_capture = bool(gp('auto_capture_on_start'))
        self._require_calibrated = bool(gp('require_axia_calibrated'))

        self._admittance = PlanarAdmittance(
            mass=[gp('virtual_mass_x'), gp('virtual_mass_y')],
            damping=[gp('damping_x'), gp('damping_y')],
            stiffness=[gp('stiffness_x'), gp('stiffness_y')],
            max_velocity=gp('max_virtual_velocity_mps'),
            max_acceleration=gp('max_virtual_acceleration_mps2'),
        )

        self._running = False
        self._fault = ''
        self._mode = 'ground_truth'
        self._axia_calibrated = False
        self._axia_connected = False
        self._current_pose = None
        self._current_pose_time = 0.0
        self._streamer_ready = False
        self._locked_ee_z = None
        self._capture_pose = None
        self._capture_tip = None
        self._tip_offset_base = np.zeros(3)
        self._force = np.zeros(3)
        self._force_time = 0.0
        self._prediction = None
        self._prediction_time = 0.0
        self._prediction_aligned = False
        self._last_tick = time.monotonic()

        self._target_pub = self.create_publisher(
            PoseStamped, '/cartesian_streamer/target_pose', 10)
        self._target_base_pub = self.create_publisher(
            PointStamped, '/coord_transform/target_base', 10)
        self._filtered_pub = self.create_publisher(
            PointStamped, '/coord_transform/filtered_hand_position', 10)
        self._nominal_pub = self.create_publisher(
            PointStamped, '/codrawing/nominal_position', 10)
        self._reference_pub = self.create_publisher(
            PointStamped, '/codrawing/reference_position', 10)
        self._tip_pub = self.create_publisher(
            PointStamped, '/codrawing/tip_position', 10)
        self._error_pub = self.create_publisher(
            Vector3Stamped, '/codrawing/admittance_error', 10)
        self._status_pub = self.create_publisher(String, '/codrawing/status', 10)
        self._run_status_pub = self.create_publisher(Bool, '/run_status', 5)

        self.create_subscription(
            PoseStamped, '/cartesian_streamer/current_pose', self._on_current_pose, 10)
        self.create_subscription(
            Bool, '/cartesian_streamer/ready', self._on_streamer_ready, 5)
        self.create_subscription(
            HandState, '/hand_position', self._on_robot_ee, 10)
        self.create_subscription(
            HandPrediction, '/ml/predicted_position', self._on_prediction, 10)
        self.create_subscription(
            Vector3Stamped, '/axia/human_force', self._on_force, 20)
        self.create_subscription(Bool, '/axia/calibrated', self._on_calibrated, 5)
        self.create_subscription(Bool, '/axia/connected', self._on_connected, 5)
        self.create_subscription(Bool, '/run_status', self._on_run_status, 5)
        self.create_subscription(String, '/trajectory_mode', self._on_mode, 5)

        # Compatibility with the existing UI. The camera launch is untouched;
        # this alias exists only while the independent co-drawing launch runs.
        self._ee_calibrate_client = self.create_client(
            Trigger, '/coord_transform/calibrate')
        self._predictor_toggle_client = self.create_client(
            SetBool, '/predictor/toggle')
        self._streamer_enable_client = self.create_client(
            SetBool, '/cartesian_streamer/enable')
        self.create_service(
            Trigger, '/realsense/calibrate_origin', self._on_ui_calibrate)

        self.create_timer(1.0 / self._rate, self._control_tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            'Co-drawing admittance ready. Camera transform is not used. '
            f'Tip offset={self._tip_offset_local.tolist()} m')

    def _on_current_pose(self, msg):
        self._current_pose = msg.pose
        self._current_pose_time = time.monotonic()
        q = msg.pose.orientation
        rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
        actual_offset = rotation @ self._tip_offset_local
        actual_tip = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ]) + actual_offset
        tip = PointStamped()
        tip.header = msg.header
        tip.header.frame_id = 'base_link'
        tip.point.x, tip.point.y, tip.point.z = map(float, actual_tip)
        self._tip_pub.publish(tip)

    def _on_streamer_ready(self, msg):
        was_ready = self._streamer_ready
        self._streamer_ready = bool(msg.data)
        if was_ready and not self._streamer_ready and self._running:
            self._set_fault('Cartesian streamer lost readiness')

    def _on_robot_ee(self, msg):
        if not msg.is_tracked or msg.source != 'robot_ee':
            return
        out = PointStamped()
        out.header = msg.header
        out.point.x, out.point.y, out.point.z = msg.x, msg.y, msg.z
        self._filtered_pub.publish(out)

    def _on_prediction(self, msg):
        self._prediction = np.array([msg.x, msg.y], dtype=float)
        self._prediction_time = time.monotonic()

    def _on_force(self, msg):
        self._force[:] = [msg.vector.x, msg.vector.y, msg.vector.z]
        self._force_time = time.monotonic()

    def _on_calibrated(self, msg):
        self._axia_calibrated = bool(msg.data)
        if not self._axia_calibrated and self._running:
            self._set_fault('Axia calibration became invalid')

    def _on_connected(self, msg):
        self._axia_connected = bool(msg.data)
        if not self._axia_connected and self._running:
            self._set_fault('Axia sensor disconnected')

    def _on_mode(self, msg):
        self._mode = msg.data.strip().lower()
        self._prediction_aligned = False

    def _on_ui_calibrate(self, request, response):
        if not self._ee_calibrate_client.service_is_ready():
            response.success = False
            response.message = 'EE tracker calibration service is not ready'
            return response
        self._ee_calibrate_client.call_async(Trigger.Request())
        response.success = True
        response.message = 'Forwarded calibration to robot-EE tracker'
        return response

    def _on_run_status(self, msg):
        if msg.data and not self._running:
            self._start_run()
        elif not msg.data and self._running:
            self._running = False
            self._admittance.reset()
            self.get_logger().info('Co-drawing stopped; holding current robot pose')

    def _start_run(self):
        self._fault = ''
        now = time.monotonic()
        if self._current_pose is None:
            self._reject_start('No current EE pose')
            return
        if now - self._current_pose_time > self._current_pose_timeout:
            self._reject_start('Current EE pose is stale')
            return
        if not self._streamer_ready:
            self._reject_start('Cartesian streamer is not ready')
            return
        if self._require_calibrated and not self._axia_calibrated:
            self._reject_start('Calibrate the Axia sensor before Start Run')
            return
        if not self._axia_connected:
            self._reject_start('No live Axia data')
            return
        if now - self._force_time > self._force_timeout:
            self._reject_start('Axia force data is stale')
            return
        if self._auto_capture or self._capture_pose is None:
            self._capture_current_pose()
        ee_z = self._capture_tip[2] - self._tip_offset_base[2]
        if not self._safe_z[0] <= ee_z <= self._safe_z[1]:
            self._reject_start(
                f'Captured EE Z={ee_z:.3f} m is outside safe workspace')
            return
        self._admittance.reset()
        self._prediction = None
        self._prediction_aligned = False
        self._force.fill(0.0)
        self._running = True
        self._locked_ee_z = float(ee_z)
        self._last_tick = time.monotonic()
        self._reset_robot_ee_origin_and_predictor()
        self.get_logger().info(
            f'Co-drawing RUN. Tip origin={self._capture_tip.round(4).tolist()}')

    def _capture_current_pose(self):
        self._capture_pose = self._current_pose
        p = self._capture_pose.position
        q = self._capture_pose.orientation
        rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
        self._tip_offset_base = rotation @ self._tip_offset_local
        ee = np.array([p.x, p.y, p.z], dtype=float)
        self._capture_tip = ee + self._tip_offset_base

    def _reset_robot_ee_origin_and_predictor(self):
        if not self._ee_calibrate_client.service_is_ready():
            self.get_logger().warn('EE calibration service unavailable; using existing origin')
            return
        future = self._ee_calibrate_client.call_async(Trigger.Request())

        def after_calibration(_future):
            if not self._predictor_toggle_client.service_is_ready():
                return
            stop = SetBool.Request()
            stop.data = False
            self._predictor_toggle_client.call_async(stop)
            if self._mode == 'prediction':
                start = SetBool.Request()
                start.data = True
                self._predictor_toggle_client.call_async(start)

        future.add_done_callback(after_calibration)

    def _reject_start(self, reason):
        self.get_logger().error(f'Co-drawing start rejected: {reason}')
        self._run_status_pub.publish(Bool(data=False))

    def _set_fault(self, reason):
        if self._fault:
            return
        self._fault = reason
        self._running = False
        self.get_logger().error(f'Co-drawing FAULT: {reason}')
        if self._streamer_enable_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self._streamer_enable_client.call_async(request)
        self._run_status_pub.publish(Bool(data=False))

    def _control_tick(self):
        if not self._running or self._capture_pose is None:
            return
        now = time.monotonic()
        if now - self._force_time > self._force_timeout:
            self._set_fault('Force data timeout')
            return
        if now - self._current_pose_time > self._current_pose_timeout:
            self._set_fault('Current EE pose timeout')
            return
        actual_z = float(self._current_pose.position.z)
        if (self._locked_ee_z is None
                or abs(actual_z - self._locked_ee_z) > self._actual_z_tolerance):
            self._set_fault(
                f'Actual EE Z deviation: actual={actual_z:.4f} m, '
                f'locked={self._locked_ee_z}')
            return
        if not self._streamer_ready:
            self._set_fault('Cartesian streamer is not ready')
            return

        planar_norm = float(np.linalg.norm(self._force[:2]))
        if planar_norm > self._max_planar_force:
            self._set_fault(f'Planar force limit exceeded: {planar_norm:.2f} N')
            return
        if abs(self._force[2]) > self._max_normal_force:
            self._set_fault(f'Normal force limit exceeded: {self._force[2]:.2f} N')
            return

        if self._mode == 'prediction':
            if self._prediction is None:
                return
            if now - self._prediction_time > self._prediction_timeout:
                self._set_fault('Prediction timeout')
                return
            nominal_xy = self._capture_tip[:2] + self._prediction
        else:
            # Ground Truth is an admittance-only baseline around the start pose.
            nominal_xy = self._capture_tip[:2].copy()

        if not self._prediction_aligned:
            actual_tip_xy = np.array([
                self._current_pose.position.x,
                self._current_pose.position.y,
            ]) + self._tip_offset_base[:2]
            self._admittance.reset(actual_tip_xy - nominal_xy)
            self._prediction_aligned = True

        dt = now - self._last_tick
        self._last_tick = now
        effective_force = soft_radial_deadzone(
            self._force_sign * self._force[:2], self._intent_threshold)
        error, _, _ = self._admittance.step(effective_force, dt)
        reference_tip_xy = nominal_xy + error

        x_min = self._capture_tip[0] - self._workspace_half_x
        x_max = self._capture_tip[0] + self._workspace_half_x
        if self._forward_sign > 0.0:
            y_min, y_max = self._capture_tip[1], self._capture_tip[1] + self._workspace_forward_y
        else:
            y_min, y_max = self._capture_tip[1] - self._workspace_forward_y, self._capture_tip[1]
        # Convert global tool0 workspace to equivalent drawing-tip bounds.
        x_min = max(x_min, self._safe_x[0] + self._tip_offset_base[0])
        x_max = min(x_max, self._safe_x[1] + self._tip_offset_base[0])
        y_min = max(y_min, self._safe_y[0] + self._tip_offset_base[1])
        y_max = min(y_max, self._safe_y[1] + self._tip_offset_base[1])
        if x_min > x_max or y_min > y_max:
            self._set_fault('Captured drawing workspace has no safe intersection')
            return
        unclamped = reference_tip_xy.copy()
        reference_tip_xy[0] = float(np.clip(reference_tip_xy[0], x_min, x_max))
        reference_tip_xy[1] = float(np.clip(reference_tip_xy[1], y_min, y_max))
        # Anti-windup: do not accumulate virtual displacement outside the
        # drawing rectangle while the user keeps pushing against a boundary.
        for axis in range(2):
            if reference_tip_xy[axis] != unclamped[axis]:
                self._admittance.error[axis] = reference_tip_xy[axis] - nominal_xy[axis]
                if np.sign(self._admittance.error_velocity[axis]) == np.sign(
                        unclamped[axis] - reference_tip_xy[axis]):
                    self._admittance.error_velocity[axis] = 0.0
        error = self._admittance.error.copy()

        # Orientation and drawing-tip Z stay fixed. Convert tip target back to tool0.
        reference_tip = np.array([
            reference_tip_xy[0], reference_tip_xy[1], self._capture_tip[2]
        ])
        reference_ee = reference_tip - self._tip_offset_base
        self._publish_reference(nominal_xy, error, reference_tip, reference_ee)

    def _publish_reference(self, nominal_xy, error, reference_tip, reference_ee):
        stamp = self.get_clock().now().to_msg()
        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = 'base_link'
        target.pose.position.x = float(reference_ee[0])
        target.pose.position.y = float(reference_ee[1])
        target.pose.position.z = float(reference_ee[2])
        target.pose.orientation = self._capture_pose.orientation
        self._target_pub.publish(target)

        def point_message(x, y, z):
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
            return msg

        nominal = point_message(nominal_xy[0], nominal_xy[1], self._capture_tip[2])
        reference = point_message(reference_tip[0], reference_tip[1], reference_tip[2])
        self._nominal_pub.publish(nominal)
        self._reference_pub.publish(reference)
        self._target_base_pub.publish(reference)

        err = Vector3Stamped()
        err.header = nominal.header
        err.vector.x, err.vector.y, err.vector.z = float(error[0]), float(error[1]), 0.0
        self._error_pub.publish(err)

    def _publish_status(self):
        if self._fault:
            state = f'FAULT: {self._fault}'
        elif self._running:
            state = 'RUNNING'
        elif (self._axia_calibrated and self._axia_connected
              and self._streamer_ready):
            state = 'READY'
        else:
            state = 'WAIT_AXIA_CALIBRATION'
        self._status_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
