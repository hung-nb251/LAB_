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


def _launch_sensorless_force(context, params):
    overrides = {
        'calibration_file': ParameterValue(
            LaunchConfiguration('robot_force_calibration_file'), value_type=str),
    }
    return [Node(
        package='hc10dtp_bringup', executable='mregister_force_node.py',
        name='sensorless_force_node', output='screen',
        parameters=[params, overrides])]


def generate_launch_description():
    package_share = get_package_share_directory('cocarry_admittance_control')
    params = os.path.join(package_share, 'config', 'cocarry_admittance_params.yaml')
    log_default = os.path.expanduser('~/cocarry_ws/cocarry_logs')

    prediction_model_arg = DeclareLaunchArgument(
        'prediction_model', default_value='gru', choices=['svgp', 'gru'],
        description='Robot-EE prediction backend selected before launch')
    prediction_reference_tau_arg = DeclareLaunchArgument(
        'prediction_reference_tau_sec', default_value='0.5',
        description='Real-robot nominal smoothing in seconds; 0 disables')
    prediction_reference_lead_arg = DeclareLaunchArgument(
        'prediction_reference_lead_sec', default_value='0.15',
        description='Filtered-velocity lead compensation, capped at 20 mm; 0 disables')
    joint_coordination_arg = DeclareLaunchArgument(
        'joint_coordination', default_value='synchronized',
        choices=['independent', 'synchronized'])
    cart_vel_arg = DeclareLaunchArgument(
        'cartesian_velocity_mps', default_value='0.25',
        description=(
            'Cartesian/virtual velocity ceiling (m/s). Measured 2026-09-22: '
            'this and joint_velocity_limit are matched at 0.25 <-> 0.60 rad/s, '
            'so raising one alone changes nothing.'))
    cart_acc_arg = DeclareLaunchArgument(
        'cartesian_acceleration_mps2', default_value='1.00',
        description='Cartesian/virtual acceleration ceiling (m/s^2)')
    joint_vel_arg = DeclareLaunchArgument(
        'joint_velocity_limit', default_value='0.60',
        description=(
            'Velocity limit for J1/J2/J3/J5/J6 in rad/s; J4/R stays at 0.08. '
            'Hard ceiling is MAX_JOINT_DELTA_PER_AXIS in the streamer: 0.07 '
            'rad/tick at 15 Hz = 1.05 rad/s for J1-J3. That guard triggers a '
            'SAFETY STOP, not a clamp, so leave headroom for IK branch jumps.'))
    tracking_error_arg = DeclareLaunchArgument(
        'max_tracking_error_m', default_value='0.065',
        description=(
            'Streamer safety-stop threshold for actual EE vs the due queue pose '
            '(m). Normal tracking error scales with command lead (~0.75*lead on '
            'the real robot), so this only ever moves together with '
            'command_lead_m. Raising it lowers the sensitivity of the only '
            'check that detects the robot not following its commanded path.'))
    command_lead_arg = DeclareLaunchArgument(
        'command_lead_m', default_value='0.055',
        description=(
            'Maximum nominal-to-actual command lead for the real robot (m). '
            'This sets the speed: the robot chases a carrot this far ahead, '
            'giving v ~ lead / 0.3 s. Measured 2026-09-22 on trial 121935: '
            '55 mm gives 0.130 m/s against 0.095 m/s at the previous 40 mm. '
            'Never move this without moving max_tracking_error_m with it.'))
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
    logging_profile_arg = DeclareLaunchArgument(
        'logging_profile', default_value='compact',
        choices=['compact', 'diagnostic', 'calibration'],
        description='compact=one CSV; diagnostic=wide CSV+events; calibration=raw sidecar')
    test_mode_arg = DeclareLaunchArgument(
        'test_mode', default_value='false',
        description='Disable real Cartesian streaming/controller')
    hybrid_target_file_arg = DeclareLaunchArgument(
        'hybrid_target_file', default_value='',
        description='Persistent base_link targets; empty uses a per-ROS-domain file')
    robot_force_calibration_file_arg = DeclareLaunchArgument(
        'robot_force_calibration_file',
        default_value=os.path.join(
            get_package_share_directory('hc10dtp_bringup'),
            'config', 'f_robot_m310_candidate_20260918.json'),
        description='Shadow-only local M310 F_robot calibration candidate')
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
        arguments=['--stream-hz', '15',
                   '--max-vel', LaunchConfiguration('cartesian_velocity_mps'),
                   '--max-accel',
                   LaunchConfiguration('cartesian_acceleration_mps2'),
                   '--max-jerk', '10.0',
                   # P2 A/B 19/09: prebuffer=2 did not improve median speed or
                   # time-domain queue lag versus the three-point baseline.
                   # Keep the more robust three-point buffer explicitly logged.
                   '--prebuffer', '3',
                   # User-approved speed profile: J1/J2/J3/J5/J6=0.60;
                   # J4 remains conservative at 0.08 rad/s.
                   '--max-joint-vel', LaunchConfiguration('joint_velocity_limit'),
                   '--max-wrist-joint-vel', '0.08',
                   '--max-j3-joint-vel',
                   LaunchConfiguration('joint_velocity_limit'),
                   '--max-j5-joint-vel',
                   LaunchConfiguration('joint_velocity_limit'),
                   '--max-j6-joint-vel',
                   LaunchConfiguration('joint_velocity_limit'),
                   '--continuous-cartesian-smoothing',
                   '--joint-coordination', LaunchConfiguration('joint_coordination'),
                   '--max-tracking-error',
                   LaunchConfiguration('max_tracking_error_m'),
                   '--fail-closed'])
    admittance = Node(
        package='cocarry_admittance_control',
        executable='admittance_controller_3d',
        name='cocarry_admittance_controller', output='screen',
        condition=not_test_mode, parameters=[params, {
            'hybrid_target_file': ParameterValue(LaunchConfiguration('hybrid_target_file'), value_type=str),
            'prediction_reference_tau_sec': ParameterValue(
                LaunchConfiguration('prediction_reference_tau_sec'), value_type=float),
            'prediction_reference_lead_sec': ParameterValue(
                LaunchConfiguration('prediction_reference_lead_sec'), value_type=float),
            'max_virtual_velocity_mps': ParameterValue(
                LaunchConfiguration('cartesian_velocity_mps'), value_type=float),
            'max_virtual_acceleration_mps2': ParameterValue(
                LaunchConfiguration('cartesian_acceleration_mps2'),
                value_type=float),
            'max_command_lead_m': ParameterValue(
                LaunchConfiguration('command_lead_m'), value_type=float),
        }])
    sensorless_force = OpaqueFunction(
        function=_launch_sensorless_force, kwargs={'params': params})
    logger = Node(
        package='cocarry_admittance_control', executable='cocarry_logger',
        name='cocarry_admittance_logger', output='screen',
        parameters=[params, {
            'log_dir': LaunchConfiguration('log_dir'),
            'logging_profile': LaunchConfiguration('logging_profile'),
        }])
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
        cmd=['python3', os.path.expanduser('~/cocarry_ws/scripts/axia_sensor_ui.py')],
        name='axia_sensor_ui', output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python'),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.expanduser('~/cocarry_ws/fastdds_no_shm.xml')),
        prediction_model_arg, svgp_model_dir_arg, gru_model_dir_arg,
        prediction_reference_tau_arg,
        prediction_reference_lead_arg, joint_coordination_arg,
        command_lead_arg, tracking_error_arg,
        cart_vel_arg, cart_acc_arg, joint_vel_arg,
        model_dir_arg, log_dir_arg, logging_profile_arg,
        test_mode_arg, use_rviz_arg, hybrid_target_file_arg,
        robot_force_calibration_file_arg,
        moveit_launch, ee_tracker, predictor, streamer, admittance,
        sensorless_force, logger, ui, axia_ui,
    ])
