import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_share = get_package_share_directory("tianracer_core")

    serial_port = LaunchConfiguration("serial_port_core", \
        default=os.environ.get("TIANRACER_BASE_PORT", "/dev/ttyUSB0"))

    tianracer_core = Node(
        package='tianracer_core',
        executable='tianracer_core_node',
        name='tianracer',
        output='screen',
        parameters=[{'serial_port': serial_port}])

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'param/tianbot_ekf_params.yaml')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port_core',
            default_value=serial_port,
            description='Tianracer base port'),
        tianracer_core,
        robot_localization_node

    ])
