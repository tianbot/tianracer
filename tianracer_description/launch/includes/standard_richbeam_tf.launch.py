import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction

default_namespace = os.environ.get("TIANRACER_NAME", "")

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace", \
        default=default_namespace).perform(context)
    return_node = []

    return_node.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint2base_link",
            namespace=namespace,
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.065",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", f"{namespace}/base_footprint",
                "--child-frame-id", f"{namespace}/base_link",
            ],
        )
    )

    return_node.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link2laser",
            namespace=namespace,
            output="screen",
            arguments=[
                "--x", "0.34",
                "--y", "0.0",
                "--z", "0.12",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", f"{namespace}/base_link",
                "--child-frame-id", f"{namespace}/laser",
            ],
        )
    )

    return_node.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link2imu",
            namespace=namespace,
            output="screen",
            arguments=[
                "--x", "0.34",
                "--y", "0.0",
                "--z", "0.12",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", f"{namespace}/base_link",
                "--child-frame-id", f"{namespace}/imu_link",
            ],
        )
    )

    return_node.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link2camera",
            namespace=namespace,
            output="screen",
            arguments=[
                "--x", "0.41",
                "--y", "0.0",
                "--z", "0.075",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", f"{namespace}/base_link",
                "--child-frame-id", f"{namespace}/camera_link",
            ],
        )
    )
            
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

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=launch_setup)],
    )
