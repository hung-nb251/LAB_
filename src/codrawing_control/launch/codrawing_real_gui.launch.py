#!/usr/bin/env python3
"""Independent real-robot co-drawing pipeline (no camera nodes)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = get_package_share_directory('codrawing_control')
    params = os.path.join(package_share, 'config', 'codrawing_params.yaml')
    log_default = os.path.expanduser('~/cocarry_ws/codrawing_logs')

    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value=os.path.expanduser(
            '~/cocarry_ws/pHRI_Models/svgp_h5_m50_raw'),
        description='SVGP/GRU model directory for robot-EE prediction')
    log_dir_arg = DeclareLaunchArgument(
        'log_dir', default_value=log_default,
        description='Dedicated co-drawing CSV directory')
    test_mode_arg = DeclareLaunchArgument(
        'test_mode', default_value='false',
        description='Disable real Cartesian streaming')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='False')

    not_test_mode = UnlessCondition(
        PythonExpression(["'", LaunchConfiguration('test_mode'), "' == 'true'"]))

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hc10dtp_moveit_config'),
            'launch', 'hc10dtp_start.launch.py')),
        launch_arguments={
            'test_mode': LaunchConfiguration('test_mode'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items())

    ee_tracker = Node(
        package='hc10dtp_bringup', executable='ee_tracker_node.py',
        name='ee_position_tracker', output='screen')

    predictor = Node(
        package='trajectory_predictor', executable='predictor_node',
        name='trajectory_predictor', output='screen',
        parameters=[params, {
            'model_dir': LaunchConfiguration('model_dir'),
            'auto_start': False,
        }])

    moveit_config = MoveItConfigsBuilder(
        'motoman_hc10dtp', package_name='hc10dtp_moveit_config').to_dict()
    streamer = Node(
        package='hc10dtp_bringup',
        executable='cartesian_streamer_hc10dtp.py',
        name='cartesian_streamer', output='screen',
        condition=not_test_mode, parameters=[moveit_config],
        arguments=['--lock-z'])

    admittance = Node(
        package='codrawing_control', executable='admittance_controller',
        name='admittance_controller', output='screen',
        condition=not_test_mode, parameters=[params])

    sensorless_force = Node(
        package='hc10dtp_bringup', executable='sensorless_force_node.py',
        name='sensorless_force_node', output='screen',
        condition=not_test_mode,
        parameters=[{
            'base_link': 'base_link',
            'tip_link': 'tool0',
            'deadband_n': 1.0,
        }])

    logger = Node(
        package='codrawing_control', executable='codrawing_logger',
        name='codrawing_logger', output='screen',
        parameters=[params, {'log_dir': LaunchConfiguration('log_dir')}])

    # Reuse the existing dashboard, but co-drawing auto-captures its plane on Start.
    ui = Node(
        package='predictor_ui', executable='ui_node',
        name='predictor_ui', output='screen',
        parameters=[{'require_alignment_steps': False}])

    # Force dashboard stays a separate process/window, exactly as requested.
    axia_ui = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/cocarry_ws/scripts/axia_sensor_ui.py')],
        name='axia_sensor_ui', output='screen')

    return LaunchDescription([
        SetEnvironmentVariable(
            'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python'),
        SetEnvironmentVariable(
            'ROS_LOG_DIR', log_default),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.expanduser('~/cocarry_ws/fastdds_no_shm.xml')),
        model_dir_arg,
        log_dir_arg,
        test_mode_arg,
        use_rviz_arg,
        moveit_launch,
        ee_tracker,
        predictor,
        streamer,
        admittance,
        sensorless_force,
        logger,
        ui,
        axia_ui,
    ])
