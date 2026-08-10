

#!/usr/bin/env python3
"""
cocarry_full.launch.py
──────────────────────
Launch file tích hợp toàn bộ pipeline co-carrying.

Kiến trúc 2 terminal:
  Terminal 1 (MoveIt stack):
    ros2 launch hc10dtp_moveit_config hc10dtp_start.launch.py

  Terminal 2 (Co-carry pipeline):
    ros2 launch hrc_bringup cocarry_full.launch.py

Chọn nguồn input:
  # Camera (mặc định — pipeline cũ)
  ros2 launch hrc_bringup cocarry_full.launch.py

  # Robot EE (đọc vị trí EE từ joint encoders — pipeline mới)
  ros2 launch hrc_bringup cocarry_full.launch.py input_source:=robot_ee

Pipeline data flow (camera):
  RealSense → realsense_tracker → /hand_position
    → trajectory_predictor → /ml/predicted_position
    → coord_transform → /cartesian_streamer/target_pose
    → cartesian_streamer_hc10dtp → /queue_traj_point
    → HC10DTP robot

Pipeline data flow (robot_ee):
  /joint_states → ee_tracker_node (FK) → /hand_position
    → trajectory_predictor → /ml/predicted_position
    → coord_transform (bypass cam→base) → /cartesian_streamer/target_pose
    → cartesian_streamer_hc10dtp → /queue_traj_point
    → HC10DTP robot
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Environment fix (TensorFlow + protobuf conflict) ─────────────────
    env_fix = SetEnvironmentVariable(
        'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

    ros_log_fix = SetEnvironmentVariable(
        'ROS_LOG_DIR',
        os.path.expanduser('~/cocarry_ws/cocarry_logs/ros_log'))

    # ── Launch arguments ──────────────────────────────────────────────────
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value=os.path.expanduser('~/cocarry_ws/pHRI_Models/svgp_camera_old'),
        description='Path to directory containing .h5 models and .pkl scalers')

    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value=os.path.expanduser('~/cocarry_ws/cocarry_logs'),
        description='Directory for experiment CSV logs')

    input_source_arg = DeclareLaunchArgument(
        'input_source',
        default_value='camera',
        description='Input source: "camera" (RealSense+MediaPipe) or "robot_ee" (FK from joint encoders)')

    # ── Conditions ────────────────────────────────────────────────────────
    use_camera = UnlessCondition(
        PythonExpression(["'", LaunchConfiguration('input_source'), "' == 'robot_ee'"]))
    use_robot_ee = IfCondition(
        PythonExpression(["'", LaunchConfiguration('input_source'), "' == 'robot_ee'"]))

    # ── Config paths ──────────────────────────────────────────────────────
    rs_model_path = os.path.join(
        get_package_share_directory('realsense_tracker'),
        'models', 'pose_landmarker_full.task')

    transform_params = os.path.join(
        get_package_share_directory('coord_transform'),
        'config', 'transform_params.yaml')

    # Tập trung cấu hình predictor + logger vào 1 file YAML duy nhất
    all_params = os.path.join(
        get_package_share_directory('hrc_bringup'),
        'config', 'all_params.yaml')

    # ── Nodes ─────────────────────────────────────────────────────────────

    # 1a. Camera tracking node (chỉ chạy khi input_source = camera)
    realsense_node = Node(
        package='realsense_tracker',
        executable='realsense_node',
        name='realsense_tracker',
        output='screen',
        condition=use_camera,
        parameters=[{
            'model_path': rs_model_path,
            'offset_x': 0.0,
            'offset_y': 0.0,
            'offset_z': 0.0,
        }])

    # 1b. EE Position Tracker (chỉ chạy khi input_source = robot_ee)
    #     Đọc /joint_states → FK → publish /hand_position (HandState)
    ee_tracker_node = Node(
        package='hc10dtp_bringup',
        executable='ee_tracker_node.py',
        name='ee_position_tracker',
        output='screen',
        condition=use_robot_ee)

    # 2. Trajectory prediction node
    #    Cấu hình chính nằm trong all_params.yaml (filter, model files, scalers)
    #    Launch-specific overrides: model_dir (từ CLI), auto_start
    predictor_node = Node(
        package='trajectory_predictor',
        executable='predictor_node',
        name='trajectory_predictor',
        output='screen',
        parameters=[
            all_params,
            {
                'model_dir': LaunchConfiguration('model_dir'),
                'auto_start': True,
            },
        ])

    # 3. Coordinate transform node (cầu nối giữa 2 repo)
    #    Khi dùng robot_ee: tự động phát hiện source='robot_ee' trong HandState
    #    và bypass camera→base transform
    transform_node = Node(
        package='coord_transform',
        executable='transform_node',
        name='coord_transform',
        output='screen',
        parameters=[transform_params])

    # 4. Cartesian streamer (kết nối robot)
    streamer_node = Node(
        package='hc10dtp_bringup',
        executable='cartesian_streamer_hc10dtp.py',
        name='cartesian_streamer',
        output='screen')

    # 5. Experiment logger
    logger_node = Node(
        package='experiment_logger',
        executable='logger_node',
        name='experiment_logger',
        output='screen',
        parameters=[
            all_params,
            {
                'log_dir': LaunchConfiguration('log_dir'),
            },
        ])

    # 6. UI dashboard
    ui_node = Node(
        package='predictor_ui',
        executable='ui_node',
        name='predictor_ui',
        output='screen')

    return LaunchDescription([
        env_fix,
        ros_log_fix,
        model_dir_arg,
        log_dir_arg,
        input_source_arg,
        realsense_node,
        ee_tracker_node,
        predictor_node,
        transform_node,
        streamer_node,
        logger_node,
        ui_node,
    ])
