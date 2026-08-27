#!/usr/bin/env python3
"""Bring up Main Scene static-map localization and required Nav2 servers."""

from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


HERE = Path(__file__).resolve().parent
BASE_PARAMS = str(HERE / "nav2_minimal_params.yaml")
PRODUCTION_PARAMS = str(HERE / "main_scene_production_overlay.yaml")
MAP_FILE = str(
    HERE.parent
    / "ros2_ws"
    / "src"
    / "cobot3_bringup"
    / "maps"
    / "af2_flat_scenario0_map.yaml"
)


def generate_launch_description():
    common_remaps = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("cmd_vel", "/amr1/cmd_vel"),
        ("/cmd_vel", "/amr1/cmd_vel"),
    ]
    parameter_files = [BASE_PARAMS, PRODUCTION_PARAMS]
    servers = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=parameter_files + [{"yaml_filename": MAP_FILE}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=parameter_files,
            remappings=[("scan", "/amr1/scan")],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=parameter_files,
            remappings=common_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=parameter_files,
            remappings=common_remaps,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=parameter_files,
            remappings=common_remaps,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=parameter_files,
            remappings=common_remaps,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=parameter_files,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=parameter_files,
        ),
    ]
    return LaunchDescription(servers)


if __name__ == "__main__":
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    sys.exit(launch_service.run())
