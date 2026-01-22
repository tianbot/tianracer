import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace
default_frame_id = f"camera_link" if default_namespace == '' else f"{default_namespace}/camera_link"
default_replacements = '' if default_namespace == '' else '/'

def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('tianracer_rviz'), 'rviz_cfg', 'view_image.rviz')

    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>/': (default_replacements, default_namespace, default_replacements)})
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', namespaced_rviz_config_file,
                       '-f', default_frame_id
                       ],
            output='screen',
        )
    ])
