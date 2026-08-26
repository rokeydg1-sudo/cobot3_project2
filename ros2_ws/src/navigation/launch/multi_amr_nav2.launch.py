#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    bringup_launch = str(Path(nav2_bringup_dir) / 'launch' / 'bringup_launch.py')

    # Resolve paths from this launch file so the source tree works from any clone.
    project_root = Path(__file__).resolve().parents[1]

    map_file = str(project_root / 'maps' / 'factory' / 'AF3.yaml')

    amr1_params = str(project_root / 'config' / 'nav2_amr1.yaml')

    amr2_params = str(project_root / 'config' / 'nav2_amr2.yaml')

    # =========================
    # AMR 1 Nav2
    # =========================

    amr1_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'namespace': 'amr1',
            'use_namespace': 'True',

            'map': map_file,
            'params_file': amr1_params,

            'use_sim_time': 'True',
            'autostart': 'True',

            'slam': 'False',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    # =========================
    # AMR 2 Nav2
    # =========================

    amr2_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'namespace': 'amr2',
            'use_namespace': 'True',

            'map': map_file,
            'params_file': amr2_params,

            'use_sim_time': 'True',
            'autostart': 'True',

            'slam': 'False',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    return LaunchDescription([
        amr1_nav2,
        amr2_nav2,
    ])
