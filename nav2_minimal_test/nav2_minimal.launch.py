#!/usr/bin/env python3

from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


HERE = Path(__file__).resolve().parent
PARAMS_FILE = str(HERE / "nav2_minimal_params.yaml")


def generate_launch_description():

    common_remaps = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("cmd_vel", "/amr1/cmd_vel"),
        ("/cmd_vel", "/amr1/cmd_vel"),
    ]

    return LaunchDescription([

        # =================================================
        # Nav2 Controller Server
        #
        # Path -> /cmd_vel
        # =================================================
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[PARAMS_FILE],
            remappings=common_remaps,
        ),

        # =================================================
        # Nav2 Planner Server
        #
        # 현재 Pose -> Goal까지 Global Path 계산
        # =================================================
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[PARAMS_FILE],
            remappings=common_remaps,
        ),

        # =================================================
        # Recovery / Behavior Server
        # =================================================
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[PARAMS_FILE],
            remappings=common_remaps,
        ),

        # =================================================
        # NavigateToPose / NavigateThroughPoses Action Server
        #
        # BT navigator parameter에 등록된 두 Action을 제공한다.
        # =================================================
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[PARAMS_FILE],
            remappings=common_remaps,
        ),

        # =================================================
        # Nav2 Lifecycle Manager
        #
        # 위 서버들을 configure -> activate 한다.
        # =================================================
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[PARAMS_FILE],
        ),
    ])


if __name__ == "__main__":
    launch_service = LaunchService()
    launch_service.include_launch_description(
        generate_launch_description()
    )
    sys.exit(launch_service.run())
