#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar = os.environ.get("TIANRACER_LIDAR", "rplidar_a1")
    model = os.environ.get("TIANRACER_LIDAR_MODEL", "a1")
    ld = LaunchDescription()
    if "rplidar" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes','lidar', 'rplidar.launch.py')),
                launch_arguments={'model': model}.items(),
            ))
    ## TODO
    # if “osight” in lidar
    # if “velodyne” in lidar
    # if “rslidar” in lidar



    return ld
