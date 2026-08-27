"""ROS-side production integration launch without an Isaac scene process."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _optional_nav2_include(context):
    enabled = LaunchConfiguration('enable_nav2').perform(context).lower()
    if enabled not in ('1', 'true', 'yes', 'on'):
        return []
    launch_file = LaunchConfiguration('nav2_launch_file').perform(context)
    if not launch_file:
        raise RuntimeError(
            'enable_nav2=true requires an explicit nav2_launch_file. '
            'Main Scene Nav2 integration is a runtime-only contract.'
        )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
        )
    ]


def generate_launch_description():
    """Launch FMS, AMR, optional Vision, and an explicit Nav2 include."""
    enable_fms = LaunchConfiguration('enable_fms')
    enable_scene_endpoints = LaunchConfiguration('enable_scene_endpoints')
    enable_amr = LaunchConfiguration('enable_amr')
    enable_vision = LaunchConfiguration('enable_vision')
    enable_lift = LaunchConfiguration('enable_lift')

    declarations = [
        DeclareLaunchArgument('enable_fms', default_value='true'),
        DeclareLaunchArgument('enable_scene_endpoints', default_value='true'),
        DeclareLaunchArgument('enable_amr', default_value='true'),
        DeclareLaunchArgument('enable_vision', default_value='true'),
        DeclareLaunchArgument('vision_publish_cmd_vel', default_value='true'),
        DeclareLaunchArgument('enable_lift', default_value='true'),
        DeclareLaunchArgument('enable_nav2', default_value='false'),
        DeclareLaunchArgument(
            'nav2_launch_file',
            default_value='',
            description=(
                'Explicit production Nav2 launch path after '
                'Main Scene audit.'
            ),
        ),
        DeclareLaunchArgument('odom_topic', default_value='/amr/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('return_speed_mps', default_value='0.35'),
        DeclareLaunchArgument('return_timeout_s', default_value='30.0'),
        DeclareLaunchArgument(
            'nav2_action_name',
            default_value='/navigate_through_poses',
        ),
        DeclareLaunchArgument('dock_action_name', default_value='/dock_dolly'),
        DeclareLaunchArgument('lift_action_name', default_value='/lift_dolly'),
        DeclareLaunchArgument(
            'lift_joint_state_topic',
            default_value='/joint_states',
        ),
        DeclareLaunchArgument(
            'lift_joint_command_topic',
            default_value='/joint_commands',
        ),
        DeclareLaunchArgument('lift_joint_name', default_value='lift_joint'),
        DeclareLaunchArgument('lift_up_target', default_value='0.04'),
        DeclareLaunchArgument('lift_down_target', default_value='0.0'),
        DeclareLaunchArgument('lift_tolerance', default_value='0.001'),
        DeclareLaunchArgument('lift_timeout_s', default_value='15.0'),
        DeclareLaunchArgument(
            'fms_service_name',
            default_value='/fms/request_task',
        ),
        DeclareLaunchArgument(
            'visualize_route_action_name',
            default_value='/visualize_route',
        ),
    ]

    fms = Node(
        package='fms',
        executable='fleet_management_system',
        name='FleetManagementSystem',
        output='screen',
        condition=IfCondition(enable_fms),
        parameters=[
            {'fms_service_name': LaunchConfiguration('fms_service_name')}
        ],
    )
    scene_endpoints = Node(
        package='cobot3_bringup',
        executable='scene_endpoint_adapter',
        name='scene_endpoint_adapter',
        output='screen',
        condition=IfCondition(enable_scene_endpoints),
    )
    amr = Node(
        package='amr_control',
        executable='amr_node',
        name='amr_node',
        output='screen',
        condition=IfCondition(enable_amr),
        parameters=[
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'return_speed_mps': ParameterValue(
                    LaunchConfiguration('return_speed_mps'),
                    value_type=float,
                ),
                'return_timeout_s': ParameterValue(
                    LaunchConfiguration('return_timeout_s'),
                    value_type=float,
                ),
                'nav2_action_name': LaunchConfiguration('nav2_action_name'),
                'dock_action_name': LaunchConfiguration('dock_action_name'),
                'lift_action_name': LaunchConfiguration('lift_action_name'),
                'fms_service_name': LaunchConfiguration('fms_service_name'),
                'visualize_route_action_name': LaunchConfiguration(
                    'visualize_route_action_name'
                ),
            }
        ],
    )
    lift = Node(
        package='amr_control',
        executable='lift_dolly_server',
        name='lift_dolly_server',
        output='screen',
        condition=IfCondition(enable_lift),
        parameters=[
            {
                'action_name': LaunchConfiguration('lift_action_name'),
                'joint_state_topic': LaunchConfiguration(
                    'lift_joint_state_topic'
                ),
                'joint_command_topic': LaunchConfiguration(
                    'lift_joint_command_topic'
                ),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'joint_name': LaunchConfiguration('lift_joint_name'),
                'up_target': ParameterValue(
                    LaunchConfiguration('lift_up_target'),
                    value_type=float,
                ),
                'down_target': ParameterValue(
                    LaunchConfiguration('lift_down_target'),
                    value_type=float,
                ),
                'tolerance': ParameterValue(
                    LaunchConfiguration('lift_tolerance'),
                    value_type=float,
                ),
                'timeout_s': ParameterValue(
                    LaunchConfiguration('lift_timeout_s'),
                    value_type=float,
                ),
            }
        ],
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('vision_docking'),
                    'launch',
                    'docking.launch.py',
                ]
            )
        ),
        condition=IfCondition(enable_vision),
        launch_arguments={
            'publish_cmd_vel': LaunchConfiguration('vision_publish_cmd_vel'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }.items(),
    )

    integration_entities = declarations + [
        scene_endpoints,
        fms,
        amr,
        vision,
        lift,
        OpaqueFunction(function=_optional_nav2_include),
    ]
    return LaunchDescription(integration_entities)
