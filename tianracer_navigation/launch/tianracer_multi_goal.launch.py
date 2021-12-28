from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    goalListX = LaunchConfiguration('goalListX', default='[-2.433,2.902,4.59848]').perform(context)
    goalListY = LaunchConfiguration('goalListY', default='[-2.514,-4.534,-1.5732]').perform(context)
    goalListTheta = LaunchConfiguration('goalListTheta', default='[-1.548,0.008,1.54833]').perform(context)
    map_frame = LaunchConfiguration('map_frame', default='map')
    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'goalListX',
            default_value=goalListX,
            description='The list of x value of goals. [1.0, 1.1, ... ]'
        ),

        DeclareLaunchArgument(
            'goalListY',
            default_value=goalListY,
            description='The list of y value of goals. [2.0, 1.5, ... ]'
        ),
        DeclareLaunchArgument(
            'goalListTheta',
            default_value=goalListTheta,
            description='The list of rotation angles of goals. [1.57, 1.7, ... ]'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value=map_frame,
            description='Name of map frame'
        ),

        Node(
            package='tianracer_navigation',
            executable='multi_goals.py',
            parameters=[{'use_sim_time': use_sim_time,
                        'goalListX': goalListX,
                        'goalListY': goalListY,
                        'goalListTheta': goalListTheta,
                        'map_frame': map_frame}],
            output='screen')
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
