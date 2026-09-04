#!/usr/bin/env python3
"""CSV logger dedicated to co-drawing experiments."""

import csv
from datetime import datetime
import os

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped, WrenchStamped
from human_hand_msgs.msg import HandPrediction
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CodrawingLogger(Node):
    def __init__(self):
        super().__init__('codrawing_logger')
        self.declare_parameter(
            'log_dir', '/home/hungnb/cocarry_ws/codrawing_logs')
        self._log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(self._log_dir, exist_ok=True)
        self._logging = False
        self._rows = []
        self._path = ''
        self._latest = {}

        self.create_subscription(PoseStamped, '/cartesian_streamer/current_pose', self._ee, 10)
        self.create_subscription(PointStamped, '/codrawing/tip_position', self._tip, 10)
        self.create_subscription(PointStamped, '/codrawing/nominal_position', self._nominal, 10)
        self.create_subscription(PointStamped, '/codrawing/reference_position', self._reference, 20)
        self.create_subscription(Vector3Stamped, '/codrawing/admittance_error', self._error, 10)
        self.create_subscription(Vector3Stamped, '/axia/human_force', self._human_force, 20)
        self.create_subscription(WrenchStamped, '/sensorless_force', self._robot_force, 10)
        self.create_subscription(HandPrediction, '/ml/predicted_position', self._prediction, 10)
        self.create_subscription(String, '/predictor/hybrid_state', self._role, 5)
        self.create_service(SetBool, '/logger/toggle', self._toggle)
        self.get_logger().info(f'Co-drawing logger ready: {self._log_dir}')

    def _ee(self, msg):
        self._latest['ee'] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _tip(self, msg):
        self._latest['tip'] = (msg.point.x, msg.point.y, msg.point.z)

    def _nominal(self, msg):
        self._latest['nominal'] = (msg.point.x, msg.point.y)

    def _error(self, msg):
        self._latest['error'] = (msg.vector.x, msg.vector.y)

    def _human_force(self, msg):
        self._latest['fh'] = (msg.vector.x, msg.vector.y, msg.vector.z)
        self._latest['fh_ts'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _robot_force(self, msg):
        self._latest['fr'] = (
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
        self._latest['fr_ts'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _prediction(self, msg):
        self._latest['pred'] = (msg.x, msg.y)
        self._latest['inference_ms'] = msg.inference_time_ms
        self._latest['model'] = msg.model_name

    def _role(self, msg):
        self._latest['role'] = msg.data

    @staticmethod
    def _values(value, size):
        return list(value) if value is not None else [''] * size

    def _reference(self, msg):
        if not self._logging:
            return
        now_ns = self.get_clock().now().nanoseconds
        ee = self._values(self._latest.get('ee'), 3)
        tip = self._values(self._latest.get('tip'), 3)
        nominal = self._values(self._latest.get('nominal'), 2)
        error = self._values(self._latest.get('error'), 2)
        pred = self._values(self._latest.get('pred'), 2)
        fh = self._values(self._latest.get('fh'), 3)
        fr = self._values(self._latest.get('fr'), 3)
        self._rows.append([
            now_ns,
            *ee, *tip, *pred, *nominal, *error,
            msg.point.x, msg.point.y,
            *fh, *fr,
            self._latest.get('inference_ms', ''),
            self._latest.get('model', ''),
            self._latest.get('role', ''),
            self._latest.get('fh_ts', ''),
            self._latest.get('fr_ts', ''),
        ])

    def _toggle(self, request, response):
        if request.data and not self._logging:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._path = os.path.join(self._log_dir, f'codrawing_{stamp}.csv')
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
            self.get_logger().warn('No co-drawing samples to write')
            return
        header = [
            'ros_timestamp_ns',
            'actual_ee_x', 'actual_ee_y', 'actual_ee_z',
            'actual_tip_x', 'actual_tip_y', 'actual_tip_z',
            'predicted_xd_relative', 'predicted_yd_relative',
            'nominal_xd', 'nominal_yd',
            'admittance_error_x', 'admittance_error_y',
            'reference_xr', 'reference_yr',
            'f_human_x', 'f_human_y', 'f_human_z',
            'f_robot_x', 'f_robot_y', 'f_robot_z',
            'inference_ms', 'model', 'role', 'fh_timestamp_ns', 'fr_timestamp_ns',
        ]
        with open(self._path, 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self._rows)
        self.get_logger().info(f'Wrote {len(self._rows)} rows to {self._path}')


def main(args=None):
    rclpy.init(args=args)
    node = CodrawingLogger()
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

