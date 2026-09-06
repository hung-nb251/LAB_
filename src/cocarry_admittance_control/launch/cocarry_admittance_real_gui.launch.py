#!/usr/bin/env python3
"""Independent real-robot 3D co-carrying pipeline (no camera nodes)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from cocarry_admittance_control.predictor_profiles import predictor_profile


def _launch_predictor(context, params):
    backend = LaunchConfiguration('prediction_model').perform(context)
    directories = {
        'svgp': LaunchConfiguration('svgp_model_dir').perform(context),
        'gru': LaunchConfiguration('gru_model_dir').perform(context),
    }
    profile = predictor_profile(
        backend,
        directories,
        LaunchConfiguration('model_dir').perform(context),
    )
    profile['auto_start'] = False
    return [
        LogInfo(msg=(
            f'[CoCarry] predictor={backend} | model_dir={profile["model_dir"]} | '
            f'window={profile["window_size"]} | features={profile["num_features"]}'
        )),
        Node(
            package='trajectory_predictor', executable='predictor_node',
            name='trajectory_predictor', output='screen',
            parameters=[params, profile],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('cocarry_admittance_control')
    params = os.path.join(package_share, 'config', 'cocarry_admittance_params.yaml')
    log_default = os.path.expanduser('~/cocarry_ws/cocarry_logs')

    prediction_model_arg = DeclareLaunchArgument(
        'prediction_model', default_value='gru', choices=['svgp', 'gru'],
        description='Robot-EE prediction backend selected before launch')
    prediction_reference_tau_arg = DeclareLaunchArgument(
        'prediction_reference_tau_sec', default_value='0.4',
        description='Nominal smoothing in seconds, matching validated simulation; 0 disables')
    svgp_model_dir_arg = DeclareLaunchArgument(
        'svgp_model_dir',
        default_value=os.path.expanduser(
            '~/cocarry_ws/pHRI_Models/svgp_robot_ee_h5_m100_relative'),
        description='SVGP robot-EE artifact directory')
    gru_model_dir_arg = DeclareLaunchArgument(
        'gru_model_dir',
        default_value=os.path.expanduser(
            '~/cocarry_ws/pHRI_Models/gru_robot_ee_h5_relative_v1'),
        description='GRU robot-EE artifact directory')
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value='',
        description='Optional custom directory overriding the selected profile')
    log_dir_arg = DeclareLaunchArgument(
        'log_dir', default_value=log_default,
        description='CSV directory (files use cocarry_admittance_3d prefix)')
    test_mode_arg = DeclareLaunchArgument(
        'test_mode', default_value='false',
        description='Disable real Cartesian streaming/controller')
    robot_effort_unit_mode_arg = DeclareLaunchArgument(
        'robot_effort_unit_mode', default_value='raw_only',
        choices=['raw_only', 'torque_nm', 'normalized_rated_torque', 'custom_scale'],
        description=(
            'Joint effort conversion used only by the diagnostic robot-force '
            'estimator; raw_only is the safe default'))
    robot_force_calibrated_arg = DeclareLaunchArgument(
        'robot_force_calibrated', default_value='false',
        description=(
            'Mark f_robot ready for future role selection only after conversion '
            'and bias/dynamics compensation have been validated'))
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='False')

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
        name='ee_position_tracker', output='screen',
        parameters=[params])
    predictor = OpaqueFunction(
        function=_launch_predictor, kwargs={'params': params})

    moveit_config = MoveItConfigsBuilder(
        'motoman_hc10dtp', package_name='hc10dtp_moveit_config').to_dict()
    streamer = Node(
        package='hc10dtp_bringup',
        executable='cartesian_streamer_hc10dtp.py',
        name='cartesian_streamer', output='screen',
        condition=not_test_mode, parameters=[moveit_config],
        # Deliberately no --lock-z: Z is a controlled 3D degree of freedom.
        # --fail-closed keeps workspace/feedback/queue safety active.
        arguments=['--stream-hz', '15', '--max-vel', '0.15',
                   '--max-accel', '0.50', '--continuous-cartesian-smoothing',
                   '--fail-closed'])
    admittance = Node(
        package='cocarry_admittance_control',
        executable='admittance_controller_3d',
        name='cocarry_admittance_controller', output='screen',
        condition=not_test_mode, parameters=[params, {
            'prediction_reference_tau_sec': ParameterValue(
                LaunchConfiguration('prediction_reference_tau_sec'), value_type=float),
        }])
    sensorless_force = Node(
        package='hc10dtp_bringup', executable='sensorless_force_node.py',
        name='sensorless_force_node', output='screen',
        # Read-only diagnostics remain available in test_mode; this node never
        # sends robot commands.
        parameters=[params, {
            'base_link': 'base_link',
            'tip_link': 'tool0',
            'effort_unit_mode': ParameterValue(
                LaunchConfiguration('robot_effort_unit_mode'), value_type=str),
            'calibration_confirmed': ParameterValue(
                LaunchConfiguration('robot_force_calibrated'), value_type=bool),
        }])
    logger = Node(
        package='cocarry_admittance_control', executable='cocarry_logger',
        name='cocarry_admittance_logger', output='screen',
        parameters=[params, {'log_dir': LaunchConfiguration('log_dir')}])
    ui = Node(
        package='predictor_ui', executable='ui_node',
        name='predictor_ui', output='screen',
        parameters=[{
            'require_alignment_steps': False,
            'show_camera_plots': False,
            'update_hz': 15.0,
            'prediction_model': ParameterValue(
                LaunchConfiguration('prediction_model'), value_type=str),
        }])
    axia_ui = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/cocarry_ws/axia_sensor_ui.py')],
        name='axia_sensor_ui', output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python'),
        SetEnvironmentVariable('ROS_LOG_DIR', log_default),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.expanduser('~/cocarry_ws/fastdds_no_shm.xml')),
        prediction_model_arg, svgp_model_dir_arg, gru_model_dir_arg,
        prediction_reference_tau_arg,
        model_dir_arg, log_dir_arg, test_mode_arg, use_rviz_arg,
        robot_effort_unit_mode_arg, robot_force_calibrated_arg,
        moveit_launch, ee_tracker, predictor, streamer, admittance,
        sensorless_force, logger, ui, axia_ui,
    ])
