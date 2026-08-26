"""Launch one self-judging GPU-free mock mission scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return a launch description for the self-contained mock runner."""
    scenario = LaunchConfiguration('scenario')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'scenario',
                default_value='success',
                description=(
                    'success, dock_failure, lift_up_failure, '
                    'or reverse_timeout'
                ),
            ),
            Node(
                package='mission_mock',
                executable='mission_mock_runner',
                name='mission_mock_runner',
                output='screen',
                arguments=['--scenario', scenario],
            ),
        ]
    )
