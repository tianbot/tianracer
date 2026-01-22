import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace

# my_robot_2d.launch.py
def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        namespace = default_namespace,
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('tianracer_slam').find('tianracer_slam') + '/param',
            '-configuration_basename', '3d_points.lua'],
        remappings = [
            ('points2_1', 'horizontal_laser_3d'),
            ('points2_2', 'vertical_laser_3d')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        namespace = default_namespace,
        parameters = [
            {'use_sim_time':  LaunchConfiguration('use_sim_time')},
            {'resolution': 0.025}],
        )
    
    # rviz_node=Node(
    #     package = 'rviz2',
    #     namespace = default_namespace,
    #     executable = 'rviz2',
    #     name = 'rviz2',
    #     output = 'screen' ,
    #     arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz']
    #     )

    return LaunchDescription([
        use_sim_time_arg,
        # robot_state_publisher_node,
        # rviz_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])