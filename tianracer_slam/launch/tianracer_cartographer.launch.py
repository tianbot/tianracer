import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    lua_path = os.path.join(get_package_share_directory('tianracer_slam'), 'param')
    lua_name = "2d_scan.lua"
    config_dir = LaunchConfiguration("config_dir", default=lua_path)
    config_basename = LaunchConfiguration("config_name", default=lua_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_dir",
            default_value=config_dir,
            description="Full path to config file to load"),

        DeclareLaunchArgument(
            "config_basename",
            default_value=config_basename,
            description="File name of config file for cartographer"),

        Node(
            package='cartographer_ros',
            node='cartographer_node',
            arguments=['-configuration_directory', config_dir,
                    '-configuration_basename', config_basename ],
            remappings = [('/odom', '/odometry/filtered'),
                          ('/imu', '/imu')],
            output='screen',
        ),

        Node(
            package='cartographer_ros',
            node='occupancy_grid_node',
            arguments=['-resolution', '0.05'],
            output='screen',
        )

    ])
