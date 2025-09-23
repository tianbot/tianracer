import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

default_namespace = os.environ.get("TIANRACER_NAME", "")

def generate_launch_description():

    namespace = LaunchConfiguration(
        'namespace',
        default = f'{default_namespace}',
    )

    return LaunchDescription([
        # chassis driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_core"),
                'launch', 'tianracer_core.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # TF boardcaster
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_description"),
                'launch', 'tianracer_tf.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # lidar driver (2D/3D)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'lidar.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),
        
        # RGBD Camera Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'rgbd_camera.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),
        
        # USB Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'usb_cam.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # GPS driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'gps.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # ROS Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'bridge.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # Ackermann Mux
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch', 'ackermann_mux.launch.py')),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # Twist to Ackermann
        Node(
            package='twist_to_ackermann',
            executable='twist_to_ackermann_node',
            name='twist_to_ackermann',
            namespace=namespace,
            output='screen',
            parameters=[
                {'cmd_vel_topic': 'cmd_vel'},
                {'drive_topic': 'ackermann_cmd'}
            ]
        )

    ])