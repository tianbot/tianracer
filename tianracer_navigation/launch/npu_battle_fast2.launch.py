import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')

    # Arguments
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('TIANBOT_NAME', default_value='tianracer'),
        description='Robot name [tianracer_No1, tianracer_No2, tianracer_No3, ...]'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start rviz'
    )

    # Nodes
    wall_following_node = Node(
        package='tianracer_navigation',
        executable='use_to_battle_fast2.py',
        name='wall_following',
        namespace=robot_name,
        output='screen'
    )

    ackermann_convert_node = Node(
        package='tianracer_navigation',
        executable='ackermann_convert_drive.py',
        name='ackermann_convert_drive',
        namespace=robot_name,
        output='screen'
    )

    # RViz2 node (optional)
    rviz_config_path = os.path.join(
        get_package_share_directory('tianracer_rviz'),
        'rviz_cfg',
        'view_lidar.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_robot_name_arg,
        declare_use_rviz_arg,
        wall_following_node,
        ackermann_convert_node,
        rviz_node
    ])
