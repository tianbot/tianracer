import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=[EnvironmentVariable('TIANRACER_NAME', default_value='tianracer')],
            description='robot name'
        ),

        Node(
            package='tianracer_navigation',
            executable='cmd_vel_to_ackermann_drive.py',
            name='cmd_vel_to_ackermann_drive',
            namespace=robot_name,
            output='screen'
        ),

        Node(
            package='tianracer_teleop',
            executable='teleop_twist_keyboard.py',
            name='teleop_twist_keyboard',
            namespace=robot_name,
            output='screen',
            emulate_tty=True
        )
    ])
