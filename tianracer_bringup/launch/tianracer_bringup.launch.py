import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

default_namespace = os.environ.get("TIANRACER_NAME", "")

def generate_launch_description():

    return LaunchDescription([
        # chassis driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_core"),
                'launch', 'tianracer_core.launch.py')),
            launch_arguments=[
                ('namespace', default_namespace)
            ]
        ),

        # TF boardcaster
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_description"),
                'launch', 'tianracer_tf.launch.py')),
            launch_arguments=[
                ('namespace', default_namespace)
            ]
        ),

        # lidar driver (2D/3D)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'lidar.launch.py')),
            launch_arguments=[
                ('namespace', default_namespace)
            ]
        ),
        
        # RGBD Camera Driver
        
        # USB Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'usb_cam.launch.py')),
            launch_arguments=[
                ('namespace', default_namespace)
            ]
        ),

        # GPS driver

        # ROS Bridge
    ])