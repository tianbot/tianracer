# https://automaticaddison.com/create-launch-files-to-display-urdf-files-ros-2-jazzy/

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_namespace = f"/" if default_namespace == ' ' or default_namespace =='/' else default_namespace
default_frame_id = f"base_link" if default_namespace ==  '/' else f"{default_namespace}/base_link"

# Define the arguments for the XACRO file
ARGUMENTS = [
    DeclareLaunchArgument('prefix', default_value=default_namespace,
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation')
]

def generate_launch_description():

    # Define filenames
    urdf_package = 'tianracer_description'
    rviz_package = 'tianracer_rviz'
    urdf_filename = 'tianracer_compact.urdf.xacro'
    rviz_config_filename = 'robot_model.rviz'
 
    # Set paths to important files
    urdf_pkg_share_description = FindPackageShare(urdf_package)
    rviz_pkg_share_description = FindPackageShare(rviz_package)
    default_urdf_model_path = PathJoinSubstitution(
        [urdf_pkg_share_description, 'urdf', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [rviz_pkg_share_description, 'rviz', rviz_config_filename])
    
    # Launch configuration variables
    namespace = LaunchConfiguration("namespace")
    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namesapce",
        default_value=default_namespace,
        description="robot name [tianracer_No1, tianracer_No2, tianracer_No3, ...].")
    
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
 
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
 
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Whether to start RVIZ')
 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
    ]), value_type=str)

    return LaunchDescription(
        # Create the launch description and populate
        ARGUMENTS + [

        # Declare the launch options
        declare_namespace_cmd,
        declare_jsp_gui_cmd,
        declare_rviz_config_file_cmd,
        declare_urdf_model_path_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_time_cmd,
        
        # launch action
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=default_namespace,
            output='screen',
            parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}],
        ),
        Node(
            condition=UnlessCondition(jsp_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=default_namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            condition=IfCondition(jsp_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=default_namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=default_namespace,
            arguments=['-d', rviz_config_file,
                       '-f', default_frame_id,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen",
        )
    ])