import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    base = LaunchConfiguration("base", \
        default=os.environ.get("TIANRACER_BASE", "compact")).perform(context)
    lidar = LaunchConfiguration("lidar", \
        default=os.environ.get("TIANRACER_LIDAR", "rplidar_a1"))
    return_node = [
        DeclareLaunchArgument(
            "base",
            default_value="compact",
            description="compact, standard, fullsize, customized."
        ),
    ]

    if base == "compact":
        return_node.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("tianracer_description"),\
                        'launch', 'tianracer_description.launch.py')
                ),
            )
        )
    return return_node


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])