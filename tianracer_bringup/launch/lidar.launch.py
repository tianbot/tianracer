import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    lidar = os.environ.get("TIANRACER_LIDAR", "oradar_lidar")
    model = os.environ.get("TIANRACER_LIDAR_MODEL", "ms200")
    serial_port = os.environ.get("TIANRACER_LIDAR_PORT", "/dev/tianbot_lidar")
    lidar_ip = os.environ.get("TIANRACER_LIDAR_IP", "192.168.198.1")

    namespace = LaunchConfiguration(
        'namespace',
        default = '',
    )

    ld = LaunchDescription()
    # 2D Lidar
    if "rplidar" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes','lidar', 'rplidar.launch.py')),
                launch_arguments={'model': model,
                                  'frame_id': [namespace, "/laser"]
                                  }.items(),
            ))
    elif "richbeam" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes','lidar', 'richbeam_lidar.launch.py')),
                launch_arguments={'namespace': namespace,
                                  'frame_id': [namespace, "/laser"]
                                  }.items(),
            ))
    elif "oradar_lidar" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes','lidar', 'oradar.launch.py')),
                launch_arguments={'namespace': namespace,
                                  'serial_port': serial_port
                                  }.items(),
            ))
    elif "osight" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes','lidar', 'osight_iexxx.launch.py')),
                launch_arguments={'namespace': namespace}.items(),
            ))
        
    # 3D lidar
    elif "velodyne" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes','lidar', 'velodyne.launch.py')),
                launch_arguments={'namespace': namespace}.items(),
            ))
        
    elif "rslidar" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes','lidar', 'rslidar.launch.py')),
                launch_arguments={'namespace': namespace}.items(),
            ))

    # elif ...

    return ld
