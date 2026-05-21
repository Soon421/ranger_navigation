"""
Isaac Sim Navigation Launch File

Thin wrapper around nav2_launch.py with Isaac Sim defaults
(use_sim_time=True, Isaac Sim params yaml, RTX LiDAR PointCloud2).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    isr_nav_dir = get_package_share_directory('nav2_navigation')
    base_launch = os.path.join(isr_nav_dir, 'launch', 'nav2_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': os.path.join(isr_nav_dir, 'config', 'params_isaac_sim.yaml'),
                'map': os.path.join(isr_nav_dir, 'map_2D', 'isaac_rivermark.yaml'),
            }.items()
        ),
    ])
