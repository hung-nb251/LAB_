#!/usr/bin/env python3
"""
motoros2_mock_node.py
─────────────────────
Giả lập (Mock) các service của MotoROS2 driver để cartesian_streamer
có thể hoạt động mà KHÔNG cần robot thật.

Khi chạy ở chế độ fake hardware (ros2_control mock_components/GenericSystem),
node này đóng vai trò trung gian:

  cartesian_streamer  ──(MotoROS2 services)──►  motoros2_mock_node
                                                      │
                                                      ▼
                                               ros2_control
                                          (joint_trajectory_controller)
                                                      │
                                                      ▼
                                              GenericSystem (fake HW)
                                                      │
                                                      ▼
                                              /joint_states  ──► RViz

Mock node cung cấp (không namespace, giống cấu hình node_namespace="" trên robot thật):
  - /start_point_queue_mode   (StartPointQueueMode)  → trả READY
  - /queue_traj_point         (QueueTrajPoint)       → forward tới JTC
  - /queue_point              (QueueTrajPoint)       → alias
  - /stop_traj_mode           (Trigger)              → trả success
  - /reset_error              (Trigger)              → trả success
  - /servo_on                 (Trigger)              → trả success
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration

from motoros2_interfaces.srv import StartPointQueueMode, QueueTrajPoint, ResetError

JOINT_NAMES = [
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
]

HOME_POSITIONS = [1.5708, 0.1242, -1.0494, 0.0, -0.3978, -1.4436]


class MotoROS2MockNode(Node):
    def __init__(self):
        super().__init__('motoros2_mock')
        self._cb = ReentrantCallbackGroup()
        self._queue_mode_active = False
        self._accepted_count = 0

        # Publisher -> JointTrajectoryController
        self._jtc_pub = self.create_publisher(
            JointTrajectory,
            '/hc10dtp_arm_controller/joint_trajectory',
            10,
        )

        # Mock Services (Dùng đúng type của motoros2_interfaces)
        self.create_service(StartPointQueueMode, '/start_point_queue_mode', self._handle_start_queue, callback_group=self._cb)
        self.create_service(QueueTrajPoint, '/queue_traj_point', self._handle_queue_point, callback_group=self._cb)
        self.create_service(QueueTrajPoint, '/queue_point', self._handle_queue_point, callback_group=self._cb)

        # Các trigger phụ
        self.create_service(Trigger, '/stop_traj_mode', self._handle_trigger_ok, callback_group=self._cb)
        self.create_service(ResetError, '/reset_error', self._handle_reset_error, callback_group=self._cb)
        self.create_service(Trigger, '/servo_on', self._handle_trigger_ok, callback_group=self._cb)

        # Gửi home position ban đầu
        self._init_timer = self.create_timer(0.5, self._publish_home_once)

        self.get_logger().info(
            '\n╔══════════════════════════════════════════════════╗\n'
            '║   MotoROS2 Mock Node — Simulation Mode          ║\n'
            '║   Tất cả /* services MotoROS2 đã sẵn sàng       ║\n'
            '╚══════════════════════════════════════════════════╝'
        )

    def _publish_home_once(self):
        self._send_jtc(HOME_POSITIONS, duration_sec=2.0)
        self._init_count = getattr(self, '_init_count', 0) + 1
        if self._init_count >= 5:
            self.get_logger().info('→ Sent home position to JTC')
            self._init_timer.cancel()

    def _send_jtc(self, positions, velocities=None, duration_sec=0.1):
        msg = JointTrajectory()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        if velocities and len(velocities) == 6:
            pt.velocities = list(velocities)
        else:
            pt.velocities = [0.0] * 6
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        msg.points = [pt]
        self._jtc_pub.publish(msg)

    def _handle_start_queue(self, request, response):
        self._queue_mode_active = True
        self._accepted_count = 0
        self._last_dur = 0.0
        response.result_code.value = 1
        response.message = 'Mock: Queue Mode Started'
        return response

    def _handle_queue_point(self, request, response):
        if not self._queue_mode_active:
            response.result_code.value = 106
            response.message = 'Mock: Not in Queue Mode'
            return response

        response.result_code.value = 1
        response.message = f'Mock: Point accepted ({self._accepted_count})'
        self._accepted_count += 1

        if request.joint_names:
            jpos = request.point.positions
            jvel = request.point.velocities
        else:
            jpos = request.point.positions
            jvel = request.point.velocities

        dur = request.point.time_from_start.sec + request.point.time_from_start.nanosec * 1e-9
        
        if not hasattr(self, '_last_dur'):
            self._last_dur = 0.0
        
        dt = dur - self._last_dur
        if dt <= 0.001:
            dt = 0.066
            
        self._last_dur = dur
        self._send_jtc(jpos, jvel, dt)
        
        return response

    def _handle_trigger_ok(self, request, response):
        response.success = True
        response.message = 'Mock: OK'
        return response

    def _handle_reset_error(self, request, response):
        response.result_code.value = 1  # SUCCESS
        response.message = 'Mock: Error Reset OK'
        return response


def main():
    import traceback
    try:
        rclpy.init()
        node = MotoROS2MockNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
    except Exception as e:
        with open('/tmp/mock_node_crash.log', 'w') as f:
            f.write(traceback.format_exc())
        raise

if __name__ == '__main__':
    main()
