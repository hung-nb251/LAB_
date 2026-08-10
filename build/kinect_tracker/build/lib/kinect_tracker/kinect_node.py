#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python import BaseOptions
import time
import os
import yaml
import threading
import queue

from human_hand_msgs.msg import HandPrediction, HandState
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters

RIGHT_WRIST_ID = 16

POSE_CONNECTIONS = [
    (11, 12),
    (11, 13), (13, 15),
    (12, 14), (14, 16),
    (11, 23), (12, 24),
    (23, 24),
    (23, 25), (25, 27),
    (24, 26), (26, 28),
]

SKELETON_COLOR = (0, 255, 128)
JOINT_COLOR = (255, 200, 0)
WRIST_COLOR = (0, 0, 255)

def draw_skeleton(image, landmarks, w, h):
    points = {}
    for idx, lm in enumerate(landmarks):
        if lm.visibility > 0.5:
            px = int(lm.x * w)
            py = int(lm.y * h)
            px = max(0, min(px, w - 1))
            py = max(0, min(py, h - 1))
            points[idx] = (px, py)

    for start, end in POSE_CONNECTIONS:
        if start in points and end in points:
            cv2.line(image, points[start], points[end], SKELETON_COLOR, 2, cv2.LINE_AA)

    for idx, pt in points.items():
        color = WRIST_COLOR if idx == RIGHT_WRIST_ID else JOINT_COLOR
        radius = 7 if idx == RIGHT_WRIST_ID else 4
        cv2.circle(image, pt, radius, color, -1, cv2.LINE_AA)
        cv2.circle(image, pt, radius + 2, (255, 255, 255), 1, cv2.LINE_AA)

    return points

class KinectTrackerNode(Node):
    def __init__(self):
        super().__init__('kinect_tracker')

        self.declare_parameter('model_path', '')
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('camera_namespace', '/kinect2/qhd')

        self.model_path = self.get_parameter('model_path').value
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value
        self.camera_ns = self.get_parameter('camera_namespace').value

        self.pred_pub = self.create_publisher(HandState, '/hand_position', 10)
        self.image_pub = self.create_publisher(Image, '/kinect_tracker/annotated_image', 10)
        self.calib_srv = self.create_service(Trigger, '/kinect/calibrate_origin', self.calibrate_cb)

        self.current_raw_x = 0.0
        self.current_raw_y = 0.0
        self.current_raw_z = 0.0
        self._last_valid_depth = 0.0  # Buffer: reuse last valid depth when sensor reads 0

        # --- Display thread: decoupled from AI loop ---
        # Queue holds at most 1 frame; old ones are dropped automatically
        self._display_queue = queue.Queue(maxsize=1)
        self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self._display_thread.start()

        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model not found: {self.model_path}')
            return

        base_options = BaseOptions(model_asset_path=self.model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = vision.PoseLandmarker.create_from_options(options)
        self.timestamp_ms = 0
        self.cv_bridge = CvBridge()
        
        from rclpy.qos import qos_profile_sensor_data
        
        # Subscribe to Kinect topics with BEST EFFORT QoS
        self.image_sub = message_filters.Subscriber(self, Image, f'{self.camera_ns}/image_color_rect', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, f'{self.camera_ns}/image_depth_rect', qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, f'{self.camera_ns}/camera_info', qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.info_sub], queue_size=50, slop=1.5)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info('Kinect Tracker Node Started. Waiting for topics...')

    def calibrate_cb(self, request, response):
        self.offset_x = self.current_raw_x
        self.offset_y = self.current_raw_y
        self.offset_z = self.current_raw_z
        
        self.set_parameters([
            rclpy.Parameter('offset_x', rclpy.Parameter.Type.DOUBLE, float(self.offset_x)),
            rclpy.Parameter('offset_y', rclpy.Parameter.Type.DOUBLE, float(self.offset_y)),
            rclpy.Parameter('offset_z', rclpy.Parameter.Type.DOUBLE, float(self.offset_z)),
        ])

        try:
            yaml_path = '/home/hungnb/cocarry_ws/src/hrc_bringup/config/all_params.yaml'
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                if 'kinect_tracker' in data:
                    data['kinect_tracker']['ros__parameters']['offset_x'] = float(self.offset_x)
                    data['kinect_tracker']['ros__parameters']['offset_y'] = float(self.offset_y)
                    data['kinect_tracker']['ros__parameters']['offset_z'] = float(self.offset_z)
                    with open(yaml_path, 'w') as f:
                        yaml.dump(data, f)
        except Exception as e:
            self.get_logger().error(f"Failed to save calib: {e}")

        msg = f'Calibrated successfully. Origin set to ({self.offset_x:.3f}, {self.offset_y:.3f}, {self.offset_z:.3f})'
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def sync_callback(self, color_msg, depth_msg, info_msg):
        try:
            # Add .copy() to ensure memory safety (cv_bridge shares memory by default)
            color_image = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8").copy()
            
            # Note: kinect depth is sometimes 32FC1 (float m) or 16UC1 (uint16 mm)
            # Safe conversion: Use passthrough to let cv_bridge handle the native format
            raw_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough").copy()
            
            if raw_depth.dtype == np.float32:
                # If it's float32 in meters, convert to mm uint16
                depth_image = (raw_depth * 1000).astype(np.uint16)
            else:
                depth_image = raw_depth

            # Process with MediaPipe
            rgb_image = np.ascontiguousarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            
            result = self.landmarker.detect_for_video(mp_image, self.timestamp_ms)
            self.timestamp_ms += 33
            
            h, w, _ = color_image.shape
        except Exception as e:
            self.get_logger().error(f'CV Bridge or Processing error: {e}')
            return

        wrist_pixel = None
        # Create black canvas for skeleton-only display (lighter, no background)
        skeleton_canvas = np.zeros_like(color_image)

        if result.pose_landmarks and len(result.pose_landmarks) > 0:
            landmarks = result.pose_landmarks[0]
            # Draw skeleton on BOTH images: color_image (for ROS topic) and skeleton_canvas (for display)
            drawn_points = draw_skeleton(color_image, landmarks, w, h)
            draw_skeleton(skeleton_canvas, landmarks, w, h)
            right_wrist = landmarks[RIGHT_WRIST_ID]

            if right_wrist.visibility > 0.5:
                px = int(right_wrist.x * w)
                py = int(right_wrist.y * h)
                px = max(0, min(px, w - 1))
                py = max(0, min(py, h - 1))
                wrist_pixel = (px, py)
                # Highlight wrist on canvas
                cv2.circle(skeleton_canvas, wrist_pixel, 12, (0, 0, 255), -1)
                cv2.circle(skeleton_canvas, wrist_pixel, 14, (255, 255, 255), 2)

        if wrist_pixel is not None:
            px, py = wrist_pixel
            depth_val = depth_image[py, px]

            # --- Depth buffering: use last valid depth if sensor returns 0 ---
            # Search a 5x5 neighborhood when center pixel is invalid
            if depth_val == 0:
                r = 5
                patch = depth_image[max(0, py-r):py+r+1, max(0, px-r):px+r+1]
                valid = patch[patch > 0]
                if len(valid) > 0:
                    depth_val = int(np.median(valid))

            if depth_val > 0:
                self._last_valid_depth = depth_val
            elif self._last_valid_depth > 0:
                # Fall back to last known depth so Y-axis stays continuous
                depth_val = self._last_valid_depth

            if depth_val > 0:
                Z = depth_val / 1000.0
                fx = info_msg.k[0]
                fy = info_msg.k[4]
                cx = info_msg.k[2]
                cy = info_msg.k[5]

                X = (px - cx) * Z / fx
                Y = (py - cy) * Z / fy

                self.current_raw_x = X
                self.current_raw_y = Y
                self.current_raw_z = Z

                msg = HandState()
                msg.is_tracked = True
                x_ws = self.current_raw_x - self.offset_x
                y_ws = -(self.current_raw_z - self.offset_z)
                z_ws = -(self.current_raw_y - self.offset_y)
                msg.x = x_ws
                msg.y = y_ws
                msg.z = z_ws
                self.pred_pub.publish(msg)
            else:
                msg = HandState()
                msg.is_tracked = False
                self.pred_pub.publish(msg)
        else:
            msg = HandState()
            msg.is_tracked = False
            self.pred_pub.publish(msg)

        # Push skeleton canvas (black background) to display thread
        try:
            self._display_queue.put_nowait(skeleton_canvas)
        except queue.Full:
            pass  # Drop frame if display thread is busy



        # Publish the annotated image instead of using cv2.imshow to prevent Segfaults
        try:
            annotated_msg = Image()
            annotated_msg.header = color_msg.header
            annotated_msg.height = color_image.shape[0]
            annotated_msg.width = color_image.shape[1]
            annotated_msg.encoding = "bgr8"
            annotated_msg.is_bigendian = 0
            annotated_msg.step = color_image.shape[1] * 3
            annotated_msg.data = color_image.tobytes()
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            import traceback
            self.get_logger().error(f'Failed to publish image: {e}')
            self.get_logger().error(traceback.format_exc())

    def _display_loop(self):
        """Separate thread: renders camera window at ~15fps without blocking AI."""
        while rclpy.ok():
            try:
                frame = self._display_queue.get(timeout=0.5)
                # Downscale to 50% for lighter GPU load
                small = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow('Kinect AI Tracker', small)
                cv2.waitKey(1)
            except queue.Empty:
                cv2.waitKey(1)
                continue
            except Exception:
                break
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = KinectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
