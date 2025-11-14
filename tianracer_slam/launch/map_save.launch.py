import os
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace
default_map_topic = f"map" if default_namespace == '' else f"{default_namespace}/map"

def generate_launch_description():

    # map named is map+current time
    current_time = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())
    map_filename = 'map_' + current_time

    # get directory of diablo_navigation
    map_directory = os.path.join(
        get_package_share_directory('tianracer_navigation2'), 'maps')
    path = Path(map_directory)

    # operate path instance ，get the path needed
    map_directory = Path(path.parents[4], "src/tianracer", *path.parts[-2:])

    # set and check save files
    os.makedirs(map_directory, exist_ok=True)

    map_save_config = os.path.join(map_directory, map_filename)

    return LaunchDescription(
        [
            Node(
                package='nav2_map_server',
                executable='map_saver_cli',
                name='map_saver_cli',
                arguments=['-t', default_map_topic,
                           '-f', map_save_config
                        ],
                parameters=[{'save_map_timeout': 10000.0
                        }],
                output='screen'
            )
        ]
    )
