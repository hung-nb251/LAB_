#!/usr/bin/env python3
"""
test_joint_motion.py
─────────────────────
Script đơn giản nhất để kiểm tra simulation hoạt động:
Gửi JointTrajectory trực tiếp tới JTC controller.
KHÔNG cần MoveIt, KHÔNG cần cartesian_streamer.

Cách dùng:
  Terminal 1: ros2 launch hc10dtp_simulation sim_start.launch.py
  Terminal 2: python3 test_joint_motion.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

JOINT_NAMES = ['joint_1_s', 'joint_2_l', 'joint_3_u',
               'joint_4_r', 'joint_5_b', 'joint_6_t']

# Các tư thế cần di chuyển qua lại (rad)
WAYPOINTS = [
    # Tư thế HOME (cánh tay gập xuống, sẵn sàng làm việc)
    [1.5708, 0.30, -1.20, 0.0, -0.50, 0.0],
    # Vươn sang trái
    [0.80,   0.40, -0.90, 0.3, -0.80, 0.2],
    # Vươn thẳng lên
    [1.5708, 0.10, -0.50, 0.0, -0.30, 0.0],
    # Vươn sang phải
    [2.30,   0.40, -0.90, -0.3, -0.80, -0.2],
    # Quay về HOME
    [1.5708, 0.30, -1.20, 0.0, -0.50, 0.0],
]


class JointMotionTester(Node):
    def __init__(self):
        super().__init__('joint_motion_tester')

        self._pub = self.create_publisher(
            JointTrajectory,
            '/hc10dtp_arm_controller/joint_trajectory',
            10
        )

        self.get_logger().info(
            '\n╔══════════════════════════════════════════════════╗\n'
            '║   Joint Motion Test — Direct JTC Control        ║\n'
            '║   Không cần MoveIt, không cần cartesian_streamer ║\n'
            '╚══════════════════════════════════════════════════╝'
        )

        # Đợi publisher sẵn sàng rồi bắt đầu
        self._timer = self.create_timer(1.0, self._start)
        self._wp_index = 0
        self._started = False

    def _start(self):
        if not self._started:
            self._started = True
            self._timer.cancel()
            self.get_logger().info('Bắt đầu gửi lệnh di chuyển...')
            self._send_next()

    def _send_next(self):
        if self._wp_index >= len(WAYPOINTS):
            self.get_logger().info('✓ Đã di chuyển qua hết tất cả tư thế! Lặp lại...')
            self._wp_index = 0

        joints = WAYPOINTS[self._wp_index]
        self._send_trajectory(joints, duration_sec=3.0)

        self.get_logger().info(
            f'Waypoint {self._wp_index + 1}/{len(WAYPOINTS)}: '
            f'[{", ".join(f"{j:.2f}" for j in joints)}] rad'
        )

        self._wp_index += 1

        # Lên lịch di chuyển tư thế tiếp theo sau 3.5s
        self.create_timer(3.5, self._send_next_once)

    def _send_next_once(self):
        # Xóa timer này (one-shot)
        self._send_next()
        raise SystemExit  # tự hủy timer (trick với ROS2 timer)

    def _send_trajectory(self, joint_positions, duration_sec=3.0):
        msg = JointTrajectory()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = list(joint_positions)
        pt.velocities = [0.0] * 6
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        msg.points = [pt]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = JointMotionTester()

    wp_index = [0]
    pub = node.create_publisher(
        JointTrajectory,
        '/hc10dtp_arm_controller/joint_trajectory',
        10
    )

    def send(joints, duration=3.0):
        msg = JointTrajectory()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(joints)
        pt.velocities = [0.0] * 6
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        msg.points = [pt]
        pub.publish(msg)

    node.get_logger().info('Chờ 2 giây để JTC sẵn sàng...')
    rclpy.spin_once(node, timeout_sec=2.0)

    for cycle in range(3):  # Lặp 3 vòng
        for i, wp in enumerate(WAYPOINTS):
            node.get_logger().info(
                f'[Vòng {cycle+1}/3] → Waypoint {i+1}/{len(WAYPOINTS)}: '
                f'[{", ".join(f"{j:.2f}" for j in wp)}] rad'
            )
            send(wp, duration=3.0)

            # Spin để publisher thực sự gửi đi
            end = time.time() + 3.5
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

    node.get_logger().info('✓ Test hoàn tất! Robot đã di chuyển qua 5 tư thế × 3 vòng.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
