#!/usr/bin/env python3
"""
ee_tracker_node.py
──────────────────
Thay thế realsense_tracker bằng đọc vị trí End-Effector từ robot.

Subscribe /joint_states (50Hz từ MotoROS2)
  → Forward Kinematics (LocalIKSolver, ~0.02ms)
  → Publish /hand_position (HandState)

Tương thích hoàn toàn với trajectory_predictor downstream —
cùng topic, cùng message type, chỉ khác field `source = 'robot_ee'`.

Cách chạy:
  ros2 run hc10dtp_bringup ee_tracker_node.py
"""

import os
import sys

import numpy as np

# Import local FK solver (cùng thư mục scripts/)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from local_ik_solver import LocalIKSolver

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from human_hand_msgs.msg import HandState
from std_srvs.srv import Trigger

JOINT_NAMES = [
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
]


class EEPositionTracker(Node):
    """
    Đọc vị trí EE của robot từ joint encoders thông qua FK.

    Trong bài toán Co-Carrying, người và robot nắm cùng vật cứng.
    Khi người di chuyển vật (robot ở Admittance/Compliance mode),
    EE robot dịch chuyển theo → encoder ghi nhận → FK tính ra vị trí mới.
    Vị trí EE này chính là "vị trí tay người" (gián tiếp qua vật thể).
    """

    def __init__(self):
        super().__init__('ee_position_tracker')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 0.0)  # 0 = publish mỗi khi nhận joint_states

        self._publish_rate = self.get_parameter('publish_rate_hz').value

        # ── FK Solver ───────────────────────────────────────────────
        self._fk = LocalIKSolver()
        self._joints = [0.0] * 6
        self._got_joints = False
        self._tracking_id = 0

        # ── Calibration state ───────────────────────────────────────
        # Vị trí EE gốc (khi bấm Calibrate) — dùng làm mốc cho GRU
        self._ee_origin = None

        # A configured rate is a periodic sampler, not callback-count
        # throttling. This gives the predictor the same uniform 15 Hz spacing
        # used to build its training windows, regardless of joint-state rate.
        self._publish_timer = None
        if self._publish_rate > 0:
            self._publish_timer = self.create_timer(
                1.0 / self._publish_rate, self._publish_current_position)

        # ── Publisher ───────────────────────────────────────────────
        # Cùng topic và message type với realsense_tracker
        self._pub = self.create_publisher(HandState, '/hand_position', 10)

        # ── Subscriber ──────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        # ── Services ────────────────────────────────────────────────
        # Calibrate: đặt vị trí EE hiện tại làm gốc tọa độ
        # (tương đương "Calibrate Camera" trong realsense_tracker)
        self.create_service(
            Trigger, '/coord_transform/calibrate', self._on_calibrate)

        # Capture Init Pose: lưu vị trí EE hiện tại làm mốc tương đối
        # cho coord_transform (tương đương nút "📌 Capture Init Pose" trên UI)
        self.create_service(
            Trigger, '/coord_transform/capture_init_pose',
            self._on_capture_init_pose)

        self.get_logger().info(
            'EE Position Tracker khởi động.\n'
            f'  Publish rate: {"unlimited (theo /joint_states)" if self._publish_rate <= 0 else f"{self._publish_rate:.0f} Hz"}\n'
            '  Chờ /joint_states từ MotoROS2...'
        )

    # ── Calibrate Service ────────────────────────────────────────────

    def _on_calibrate(self, request, response):
        """
        Đặt vị trí EE hiện tại làm gốc tọa độ.
        Sau calibrate, /hand_position sẽ publish tọa độ tương đối (relative displacement).
        """
        if not self._got_joints:
            response.success = False
            response.message = 'Chưa nhận được /joint_states từ robot'
            self.get_logger().warn(response.message)
            return response

        pos = self._fk.fk_position(np.array(self._joints))
        self._ee_origin = pos.copy()

        msg = (
            f'✓ Calibrated! EE Origin = '
            f'({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) [base_link]'
        )
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def _on_capture_init_pose(self, request, response):
        """
        Capture Init Pose — lưu vị trí EE hiện tại làm mốc.
        Tương đương nút "📌 Capture Init Pose" trên UI Dashboard.
        Khi dùng robot_ee source, calibrate và capture init pose
        thực chất làm cùng một việc: ghi nhớ EE position làm gốc.
        """
        return self._on_calibrate(request, response)

    # ── Joint State Callback ─────────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        """Nhận joint_states từ MotoROS2, giải FK, publish HandState."""
        # Parse joint positions theo đúng thứ tự HC10DTP
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self._joints[i] = msg.position[idx]

        if not self._got_joints:
            self._got_joints = True
            pos = self._fk.fk_position(np.array(self._joints))
            self.get_logger().info(
                f'✓ /joint_states received! EE position: '
                f'({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) [base_link]\n'
                f'  Joints: {[f"{v:.3f}" for v in self._joints]}'
            )

        # Historical/default mode publishes every joint-state callback. A
        # configured co-carry rate is handled by the uniform timer above.
        if self._publish_timer is None:
            self._publish_current_position()

    def _publish_current_position(self):
        """Sample the latest joint state and publish robot-EE displacement."""
        if not self._got_joints:
            return

        # ── Giải FK → EE position (base_link frame) ─────────────────
        ee_pos = self._fk.fk_position(np.array(self._joints))

        # Nếu đã calibrate: publish tọa độ tương đối
        if self._ee_origin is not None:
            publish_pos = ee_pos - self._ee_origin
        else:
            # Chưa calibrate: publish tọa độ tuyệt đối
            publish_pos = ee_pos

        # ── Publish HandState ────────────────────────────────────────
        self._tracking_id += 1
        state = HandState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'base_link'
        state.x = float(publish_pos[0])
        state.y = float(publish_pos[1])
        state.z = float(publish_pos[2])
        state.tracking_id = self._tracking_id
        state.is_tracked = True
        state.confidence = 1.0
        state.source = 'robot_ee'

        self._pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = EEPositionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
