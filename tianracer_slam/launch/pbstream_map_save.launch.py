import os
from pathlib import Path
import time
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace
service_namespace_prefix = f"" if default_namespace == '' else f"/{default_namespace}"

def generate_launch_description():

    # map named is map+current time
    current_time = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())
    map_filename = 'map_' + current_time + '.pbstream'

    # get directory of diablo_navigation
    map_directory = os.path.join(
        get_package_share_directory('tianracer_navigation2'), 'pbstreams')
    path = Path(map_directory)

    # operate path instance ，get the path needed
    map_directory = Path(path.parents[4], "src/tianracer", *path.parts[-2:])

    # set and check save files
    os.makedirs(map_directory, exist_ok=True)

    map_save_config = os.path.join(map_directory, map_filename)

    return LaunchDescription(
        [
            # ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
            ExecuteProcess(
                cmd=[['ros2 service call ', f'{service_namespace_prefix}/finish_trajectory ', 'cartographer_ros_msgs/srv/FinishTrajectory ', "'{trajectory_id: 0}'"]], 
                shell=True
            ),

            # ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${HOME}/diablo_ws/src/hcx_ros2/diablo_navigation2/pbstreams/tianbotoffice-602.pbstream', include_unfinished_submaps: "true"}"
            ExecuteProcess(
                cmd=[['ros2 service call ', f'{service_namespace_prefix}/write_state ', 'cartographer_ros_msgs/srv/WriteState ', "'{filename: '", f'{map_save_config}', "' , include_unfinished_submaps: 'true'}'"]],
                shell=True
            ),
        ]
    )