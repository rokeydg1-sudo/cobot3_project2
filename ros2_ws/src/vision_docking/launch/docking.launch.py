import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('vision_docking')
    params_file = os.path.join(
        package_share,
        'config',
        'docking.yaml',
    )

    publish_cmd_vel = LaunchConfiguration('publish_cmd_vel')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_cmd_vel',
            default_value='false',
            description='Publish Twist commands to /cmd_vel.',
        ),
        Node(
            package='vision_docking',
            executable='dolly_docking_node',
            name='dolly_docking_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'publish_cmd_vel': ParameterValue(
                        publish_cmd_vel,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
