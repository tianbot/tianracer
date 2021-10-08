import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    serial_port = os.environ.get("TIANRACER_BASE_PORT", "/dev/tianbot_racecar")

    tianracer_core = Node(
        package='tianracer_core',
        node_executable='tianracer_core_node',
        name='tianracer',
        output='screen',
        parameters=[{'serial_port': serial_port}])

    return LaunchDescription([

        tianracer_core,

    ])
