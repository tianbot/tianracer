#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar = os.environ.get("TIANRACER_LIDAR", "rplidar_a1")
    model = os.environ.get("TIANRACER_LIDAR_MODEL", "a1")

    if "rplidar" in lidar:
        include_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("tianracer_bringup"),
             'launch','lidar', 'launch_test.launch.py']),
             launch_arguments={'model': LaunchConfiguration(model)}.items(),
        ),

    return LaunchDescription([
        include_file
        ])

