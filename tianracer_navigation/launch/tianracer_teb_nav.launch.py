import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition



def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml_file = LaunchConfiguration('map')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    params_file = os.path.join(
        get_package_share_directory('tianracer_navigation'), 'param', 'teb_carlike', 
        'compact_teb_params.yaml')

    if os.environ['TIANRACER_BASE'] == "compact":
        base = 0.255
    elif os.environ['TIANRACER_BASE'] == "standard":
        base = 0.33
    elif os.environ['TIANRACER_BASE'] == "fullsize":
        base = 0.6

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('tianracer_slam'), 'maps', 'test_ros_map.yaml'),
            description='Full path to map yaml file to load'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description="Use rviz if true"
        ),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'tianracer_rviz', 'view_teb_planner.launch.py'],
            condition=IfCondition(use_rviz)
        ),
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'params_file': params_file,
                'map': map_yaml_file
            }.items(),
        ),

        Node(
            package='tianracer_navigation',
            executable='cmd_vel_to_ackermann_drive.py',
            output='screen',
            parameters=[{'wheelbase': base}]
        ),
    ])
