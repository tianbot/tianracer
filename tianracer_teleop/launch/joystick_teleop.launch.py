import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
from launch.conditions import UnlessCondition

def generate_launch_description():
    joy_mode = LaunchConfiguration('joy_mode')
    joy_dev = LaunchConfiguration('joy_dev')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_mode',
            default_value=[EnvironmentVariable('TIANRACER_JOY_MODE', default_value='dji_dt7')],
            description='joy mode'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/tianbot_joystick',
            description='joy device'
        ),

        GroupAction(
            condition=UnlessCondition(PythonExpression(["'dji' in '", joy_mode, "'"])),
            actions=[
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joystick',
                    parameters=[{'dev': joy_dev}]
                ),
                Node(
                    package='tianracer_teleop',
                    executable='tianracer_joy.py',
                    name='tianracer_joy',
                    parameters=[
                        {'joy_mode': joy_mode},
                        {'throttle_scale': 1.0},
                        {'servo_scale': 1.0}
                    ]
                )
            ]
        )
    ])
