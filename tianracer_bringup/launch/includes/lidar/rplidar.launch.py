#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    serial_port = LaunchConfiguration('serial_port', default=\
        os.environ.get("TIANRACER_LIDAR_PORT", "/dev/ttyUSB0"))
    # serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    model_env = LaunchConfiguration('model', default=\
         os.environ.get("TIANRACER_LIDAR_MODEL", "a1")).perform(context) # for compatibility
    serial_baudrate = 256000 if "a3" in model_env else 115200
    scan_mode = "Sensitivity" if "a3" in model_env else "Boost"

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'model',
            default_value=model_env,
            description='Specifying model of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),


        Node(
            package='rplidar_ros2',
            node_executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen',
            remappings = [('scan', 'scan_raw')]),

        Node(
            package="laser_filters",
            node_executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            parameters=[
                os.path.join(
                    get_package_share_directory("tianracer_bringup"),
                    "param", "tianbot_laser_config.yaml"
                )
            ],
            remappings=[('scan', 'scan_raw'),
                        ('scan_filtered', 'scan'),]
        )
        ])

def generate_launch_description():
        return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])

