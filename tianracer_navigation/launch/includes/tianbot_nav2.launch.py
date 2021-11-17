import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_file = os.path.join(
        get_package_share_directory('tianracer_slam'), 'maps', 'turtlebot3_world.yaml')
    param_file = os.path.join(
        get_package_share_directory('tianracer_navigation'), 'param', 'tianracer.yaml')

    map_file = LaunchConfiguration('map', default=map_file)
    param_file = LaunchConfiguration('params_file', default=param_file)

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Set true to open rviz'),

        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_file,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'Vcmd',
            default_value='1.0',
            description='speed of car m/s'),

        DeclareLaunchArgument(
            'base_speed',
            default_value='1.0',
            description='speed of car m/s'),

        DeclareLaunchArgument(
            'base_angle',
            default_value='0.0',
            description='the middle pos of servo if tuning needed'),

        DeclareLaunchArgument(
            'angle_gain',
            default_value='-3.5',
            description='for tt02: >0, for hsp: <0'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(LaunchConfiguration("use_rviz"))),

        # Node(
        #     package='tianracer_navigation',
        #     executable='L1_controller_v2',
        #     name='L1_controller_v2',
        #     parameters=[{
        #         'Vcmd': LaunchConfiguration("Vcmd"),
        #         'base_speed': LaunchConfiguration("base_speed"),
        #         'base_angle': LaunchConfiguration("base_angle"),
        #         'angle_gain': LaunchConfiguration("angle_gain"), 
        #         }],
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration("use_rviz")),
        #     remappings=[
        #         ("/odometry/filtered", "odom")
        #     ]
        # ),


    ])