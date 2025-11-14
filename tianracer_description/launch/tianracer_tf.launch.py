# REF: https://github.com/danzimmerman/dz_launch_examples/blob/rolling/launch/opaque_multi_nodes.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

default_namespace = os.environ.get("TIANRACER_NAME", "/")
default_namespace = f"/" if default_namespace == '' or default_namespace =='/' else default_namespace
default_base = os.environ.get("TIANRACER_BASE", "standard")
default_lidar = os.environ.get("TIANRACER_LIDAR", "richbeam")

print("namespace: {}, base: {}, lidar: {}".format(default_namespace, default_base, default_lidar))

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace", \
        default=default_namespace).perform(context)
    base = LaunchConfiguration("base", \
        default=default_base).perform(context)
    lidar = LaunchConfiguration("lidar", \
        default=default_lidar).perform(context)
    return_node = []

    if base == "compact":
        return_node.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("tianracer_description"),\
                        'launch', 'tianracer_description.launch.py')
                ),
                launch_arguments=[
                    ('namespace', default_namespace),
                    ('prefix', default_namespace)
                ]
            )
        )
    elif base == "standard":
        if 'osight' in lidar:
            return_node.append('')
        elif 'rplidar' in lidar:
            return_node.append('')
        elif 'richbeam' in lidar:
            return_node.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("tianracer_description"),\
                        'launch', 'includes', 'standard_richbeam_tf.launch.py')
                ),
                launch_arguments=[
                    ('namespace', default_namespace)
                ]
            ))
        elif 'velodyne' in lidar:
            return_node.append('')
        elif 'rslidar' in lidar:
            return_node.append('')
    elif base == "fullsize":
        return_node.append('')
            
    return return_node


def generate_launch_description():
    
    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            "namesapce",
            default_value=default_namespace,
            description="robot name [tianracer_No1, tianracer_No2, tianracer_No3, ...]."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "base",
            default_value=default_base,
            description="compact, standard, fullsize, customized."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "lidar",
            default_value=default_lidar,
            description="osight, rplidar, velodyne, rslidar, richbeam."
        )
    )

    # print("namespace: {}, base: {}, lidar: {}".format(default_namespace, default_base, default_lidar))
    return LaunchDescription(
        declared_args + [OpaqueFunction(function=launch_setup)],
    )