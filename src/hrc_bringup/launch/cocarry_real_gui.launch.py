#!/usr/bin/env python3
"""
cocarry_real_gui.launch.py
──────────────────────────
Bộ khởi động TỔNG hợp dành cho thực nghiệm thực tế.

Bao gồm:
  1. hc10dtp_start.launch.py    ← MoveIt stack + TF tree
  2. cocarry_full.launch.py     ← AI pipeline + Cartesian Streamer
  3. axia_sensor_ui.py          ← Giao diện cảm biến + Gravity Compensation
                                   (chạy bằng user thường, TF hoạt động 100%)

⚠ Yêu cầu: axia_sensor_driver.py phải đang chạy ngầm bằng sudo TRƯỚC KHI
   chạy file launch này:
     ./run_sensor_driver.sh

Cách chạy:
  # Mặc định — dùng joint encoder (KHÔNG cần camera)
  ros2 launch hrc_bringup cocarry_real_gui.launch.py

  # Chế độ TEST — tắt cartesian_streamer, dùng Teach Pendant an toàn
  ros2 launch hrc_bringup cocarry_real_gui.launch.py test_mode:=true

  # Nếu muốn dùng camera RealSense
  ros2 launch hrc_bringup cocarry_real_gui.launch.py input_source:=camera
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    # ── Bắt buộc: Tắt Shared Memory của FastDDS để tránh xung đột sudo ──────
    # (axia_sensor_ui.py chạy bằng sudo, các node còn lại chạy user thường)
    fastdds_fix = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.expanduser('~/cocarry_ws/fastdds_no_shm.xml')
    )

    # ── Launch arguments ──────────────────────────────────────────────────────
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value=os.path.expanduser('~/cocarry_ws/pHRI_Models/svgp_camera_old'),
        description='Path to directory containing .pkl models and scalers',
    )
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value=os.path.expanduser('~/cocarry_ws/cocarry_logs'),
        description='Directory for experiment CSV logs',
    )
    # MẶC ĐỊNH là robot_ee (không cần camera), ghi đè bằng input_source:=camera nếu cần
    input_source_arg = DeclareLaunchArgument(
        'input_source',
        default_value='robot_ee',
        description='"robot_ee" (dùng joint encoder, không cần camera) hoặc "camera"',
    )
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='true = tắt cartesian_streamer, dùng Teach Pendant an toàn',
    )

    # ── 1. MoveIt stack (TF + robot_state_publisher + MoveGroup) ─────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hc10dtp_moveit_config'),
                'launch',
                'hc10dtp_start.launch.py',
            )
        ),
        launch_arguments={
            'test_mode': LaunchConfiguration('test_mode'),
        }.items(),
    )

    # ── 2. Co-carry AI pipeline ───────────────────────────────────────────────
    cocarry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hrc_bringup'),
                'launch',
                'cocarry_full.launch.py',
            )
        ),
        launch_arguments={
            'model_dir':    LaunchConfiguration('model_dir'),
            'log_dir':      LaunchConfiguration('log_dir'),
            'input_source': LaunchConfiguration('input_source'),
            'test_mode':    LaunchConfiguration('test_mode'),
        }.items(),
    )

    # ── 3. Giao diện cảm biến lực (chạy user thường, TF hoạt động 100%) ────
    axia_ui_node = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/cocarry_ws/axia_sensor_ui.py')],
        name='axia_sensor_ui',
        output='screen',
    )

    return LaunchDescription([
        fastdds_fix,
        model_dir_arg,
        log_dir_arg,
        input_source_arg,
        test_mode_arg,
        moveit_launch,
        cocarry_launch,
        axia_ui_node,
    ])

