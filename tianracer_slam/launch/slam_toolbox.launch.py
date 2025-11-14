import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace
default_base_frame = f"base_footprint" if default_namespace == '' else f"{default_namespace}/base_footprint"
default_map_frame = f"map" if default_namespace == '' else f"{default_namespace}/map"
default_odom_frame = f"odom" if default_namespace == '' else f"{default_namespace}/odom"

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value=default_namespace, description="Top-level namespace"
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("tianracer_slam"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time,
           'odom_frame': default_odom_frame,
           'map_frame': default_map_frame,
           'base_frame': default_base_frame,
           'map_name': f"map",
           'scan_topic': f"scan",
          },
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=default_namespace,
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
