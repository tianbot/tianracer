#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_serial_port = os.environ.get("TIANRACER_LIDAR_PORT", "/dev/ttyUSB0")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace
default_laser_frame_id = f"laser" if default_namespace == '' else f"{default_namespace}/laser"

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value=default_namespace, description="Top-level namespace"
    )

    declare_serial_port_cmd = DeclareLaunchArgument(
            'serial_port',
            default_value=default_serial_port,
            description='Specifying usb port to connected lidar')

    declare_frame_id_cmd = DeclareLaunchArgument(
            'frame_id',
            default_value=default_laser_frame_id,
            description='Specifying frame_id of lidar')

    # LiDAR publisher node
    ordlidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        namespace=namespace,
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': frame_id},
            {'scan_topic': 'scan_raw'},
            {'port_name': serial_port},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.05},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10}
        ]
    )

    laser_filters_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        namespace=namespace,
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("tianracer_bringup"),
                "param", "laser_config.yaml",
            ])],
        remappings=[('scan', 'scan_raw'),
                    ('scan_filtered', 'scan'),]
    )

    # Define LaunchDescription variable
    ord = LaunchDescription()

    ord.add_action(declare_namespace_cmd)
    ord.add_action(declare_serial_port_cmd)
    ord.add_action(declare_frame_id_cmd)
    ord.add_action(ordlidar_node)
    ord.add_action(laser_filters_node)

    return ord