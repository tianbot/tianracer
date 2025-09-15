import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# my_robot_2d.launch.py
def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    '''
    去掉了urdf以及robot_state_publisher_node
    '''
    # 修改为my_robot_2d.lua 文件 以及 激光雷达话题remapping(一个激光雷达的话，映射为scan即可；多个激光雷达需要映射为scan1、scan2...)
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('tianracer_slam').find('tianracer_slam') + '/param',
            '-configuration_basename', '2d_scan.lua'],
        remappings = [
            ('scan', '/scan'),
            ('imu', '/imu'),
            ('odom', '/odom')
            ],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )
    
    rviz_node=Node(
        package = 'rviz2',
        namespace = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen' ,
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz']
        )

    return LaunchDescription([
        use_sim_time_arg,
        # robot_state_publisher_node, 注释了这个node
        rviz_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])