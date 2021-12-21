import os
import sys
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.substitutions import  LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory('tianracer_description'), \
        'urdf/tianracer_compact.urdf')
    robot = xacro.process(urdf_file)
    gui = LaunchConfiguration('gui', default=False)
    use_rviz = LaunchConfiguration('use_rviz', default=False)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value="False",
            description="Flag to enable joint_state_publisher_gui"
        ),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value="False",
            description="Flag to enable rviz"
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot}],
            arguments=[urdf_file]
        ),
        Node(
            condition=UnlessCondition(gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            condition=IfCondition(gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        )
    ])