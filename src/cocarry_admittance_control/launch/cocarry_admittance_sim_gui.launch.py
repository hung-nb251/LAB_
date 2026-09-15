#!/usr/bin/env python3
"""Force-guided 3D admittance with a fake HC10DTP shown in RViz.

The Axia sensor is real and arrives over UDP, while all robot-side MotoROS2
services and joint motion are provided by hc10dtp_simulation.  No micro-ROS
agent or physical robot connection is launched here.
"""

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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    profile.update({
        'auto_start': False,
        'mjm.publish_rate_hz': float(
            LaunchConfiguration('mjm_publish_rate_hz').perform(context)),
    })
    return [
        LogInfo(msg=(
            f'[CoCarry SIM] predictor={backend} | '
            f'model_dir={profile["model_dir"]} | '
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
    sim_log_default = os.path.expanduser('~/cocarry_ws/cocarry_logs/simulation')
    os.makedirs(sim_log_default, exist_ok=True)

    prediction_model_arg = DeclareLaunchArgument(
        'prediction_model', default_value='gru', choices=['svgp', 'gru'],
        description='Robot-EE prediction backend selected before launch')
    prediction_reference_tau_arg = DeclareLaunchArgument(
        'prediction_reference_tau_sec', default_value='0.4',
        description='Simulation nominal smoothing in seconds; 0 restores raw handoff')
    prediction_reference_lead_arg = DeclareLaunchArgument(
        'prediction_reference_lead_sec', default_value='0.15',
        description='Filtered-velocity lead compensation, capped at 20 mm; 0 disables')
    joint_coordination_arg = DeclareLaunchArgument(
        'joint_coordination', default_value='synchronized',
        choices=['independent', 'synchronized'])
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
        'log_dir', default_value=sim_log_default,
        description='CSV directory for simulated-robot trials')
    simulation_domain_arg = DeclareLaunchArgument(
        'simulation_domain_id', default_value='42',
        description='Isolated ROS domain; keep different from real robot domain 10')
    hybrid_target_file_arg = DeclareLaunchArgument(
        'hybrid_target_file', default_value='',
        description='Persistent base_link targets; empty uses a per-ROS-domain file')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Show the fake HC10DTP in RViz')
    launch_sensor_ui_arg = DeclareLaunchArgument(
        'launch_sensor_ui', default_value='true',
        description='Start axia_sensor_ui.py and receive real UDP force data')
    launch_dashboard_arg = DeclareLaunchArgument(
        'launch_dashboard', default_value='true',
        description='Start the predictor/control dashboard')
    mjm_publish_rate_arg = DeclareLaunchArgument(
        'mjm_publish_rate_hz', default_value='30.0',
        description='Simulation-only MJM replay rate; real co-carry remains 15 Hz')
    wrist_joint_velocity_limit_arg = DeclareLaunchArgument(
        'wrist_joint_velocity_limit', default_value='0.08',
        description=(
            'Simulation-only initial R/B/T velocity limit in rad/s; explicit '
            'B/T override is applied afterwards'))
    j3_joint_velocity_limit_arg = DeclareLaunchArgument(
        'j3_joint_velocity_limit', default_value='0.30',
        description='Simulation-only J3/U velocity limit in rad/s')
    bt_joint_velocity_limit_arg = DeclareLaunchArgument(
        'bt_joint_velocity_limit', default_value='0.15',
        description='Simulation-only J5/B and J6/T velocity limit in rad/s')
    command_lead_arg = DeclareLaunchArgument(
        'command_lead_m', default_value='0.04',
        description='Simulation-only maximum nominal-to-actual command lead (m)')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hc10dtp_simulation'),
            'launch', 'sim_start.launch.py')),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items())

    ee_tracker = Node(
        package='hc10dtp_bringup', executable='ee_tracker_node.py',
        name='ee_position_tracker', output='screen',
        parameters=[params])
    predictor = OpaqueFunction(
        function=_launch_predictor, kwargs={'params': params})

    fake_moveit_config = (
        MoveItConfigsBuilder(
            'motoman_hc10dtp', package_name='hc10dtp_moveit_config')
        .robot_description(
            file_path='config/motoman_hc10dtp.urdf.xacro',
            mappings={'use_fake_hardware': 'true'})
        .robot_description_semantic(file_path='config/motoman_hc10dtp.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_dict())
    streamer = Node(
        package='hc10dtp_bringup',
        executable='cartesian_streamer_hc10dtp.py',
        name='cartesian_streamer', output='screen',
        parameters=[fake_moveit_config],
        arguments=['--stream-hz', '15', '--max-vel', '0.18',
                   '--max-accel', '0.65', '--max-joint-vel', '0.30',
                   '--max-wrist-joint-vel',
                   LaunchConfiguration('wrist_joint_velocity_limit'),
                   '--max-j3-joint-vel',
                   LaunchConfiguration('j3_joint_velocity_limit'),
                   '--max-bt-joint-vel',
                   LaunchConfiguration('bt_joint_velocity_limit'),
                   '--continuous-cartesian-smoothing',
                   '--joint-coordination', LaunchConfiguration('joint_coordination'),
                   '--fail-closed'])
    admittance = Node(
        package='cocarry_admittance_control',
        executable='admittance_controller_3d',
        name='cocarry_admittance_controller', output='screen',
        parameters=[params, {
            'hybrid_target_file': ParameterValue(LaunchConfiguration('hybrid_target_file'), value_type=str),
            'prediction_reference_tau_sec': ParameterValue(
                LaunchConfiguration('prediction_reference_tau_sec'), value_type=float),
            'prediction_reference_lead_sec': ParameterValue(
                LaunchConfiguration('prediction_reference_lead_sec'), value_type=float),
            'max_virtual_velocity_mps': 0.18,
            'max_virtual_acceleration_mps2': 0.65,
            'max_command_lead_m': ParameterValue(
                LaunchConfiguration('command_lead_m'), value_type=float),
        }])
    logger = Node(
        package='cocarry_admittance_control', executable='cocarry_logger',
        name='cocarry_admittance_logger', output='screen',
        parameters=[params, {
            'log_dir': LaunchConfiguration('log_dir'),
            'file_prefix': 'cocarry_admittance_sim_3d',
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
        }],
        condition=IfCondition(LaunchConfiguration('launch_dashboard')))
    axia_ui = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/cocarry_ws/axia_sensor_ui.py')],
        name='axia_sensor_ui', output='screen',
        condition=IfCondition(LaunchConfiguration('launch_sensor_ui')))

    return LaunchDescription([
        simulation_domain_arg,
        hybrid_target_file_arg,
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('simulation_domain_id')),
        SetEnvironmentVariable('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python'),
        SetEnvironmentVariable('ROS_LOG_DIR', sim_log_default),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.expanduser('~/cocarry_ws/fastdds_no_shm.xml')),
        LogInfo(msg=[
            '[SIMULATION ONLY] ROS_DOMAIN_ID=',
            LaunchConfiguration('simulation_domain_id'),
            ' | fake HC10DTP + real Axia UDP; no physical robot connection',
        ]),
        prediction_model_arg,
        prediction_reference_tau_arg,
        prediction_reference_lead_arg, joint_coordination_arg,
        command_lead_arg,
        svgp_model_dir_arg,
        gru_model_dir_arg,
        model_dir_arg,
        log_dir_arg,
        use_rviz_arg,
        launch_sensor_ui_arg,
        launch_dashboard_arg,
        mjm_publish_rate_arg,
        wrist_joint_velocity_limit_arg, j3_joint_velocity_limit_arg,
        bt_joint_velocity_limit_arg,
        simulation,
        ee_tracker,
        predictor,
        streamer,
        admittance,
        logger,
        ui,
        axia_ui,
    ])
