import os
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == ' ' or default_namespace =='/' else default_namespace
default_frame_id = f"laser" if default_namespace ==  '' else f"{default_namespace}/laser"

def generate_launch_description():
    model = os.environ.get("TIANRACER_LIDAR_MODEL", "16")
    namespace = LaunchConfiguration(
        'namespace',
        default = '',
    )

    ld = LaunchDescription()
    # 2D Lidar
    if "16" in model:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("velodyne"),
                'launch', 'velodyne-all-nodes-VLP16-launch.py')),
                launch_arguments={'namespace': namespace}.items(),
            ))

    elif "32c" in model:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("velodyne"),
                'launch', 'velodyne-all-nodes-VLP32C-launch.py')),
                launch_arguments={'namespace': namespace}.items(),
            ))
        
    elif "128" in model:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("velodyne"),
                'launch', 'velodyne-all-nodes-VLS128-launch.py')),
                launch_arguments={'namespace': namespace}.items(),
            ))

    # elif ...

    return ld
