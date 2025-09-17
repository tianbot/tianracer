import os
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_namespace = f"/" if default_namespace == ' ' or default_namespace =='/' else default_namespace
default_base_frame = f"base_footprint" if default_namespace ==  '/' else f"{default_namespace}/base_footprint"
default_map_frame = f"map" if default_namespace ==  '/' else f"{default_namespace}/map"
default_odom_frame = f"odom" if default_namespace ==  '/' else f"{default_namespace}/odom"

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping', 
            executable = 'slam_gmapping',
            namespace = default_namespace,
            output = 'screen', 
            parameters = [{
                'use_sim_time':use_sim_time,
                'base_frame': default_base_frame,
                'map_frame': default_map_frame,
                'odom_frame': default_odom_frame,
            }])
              
    ])