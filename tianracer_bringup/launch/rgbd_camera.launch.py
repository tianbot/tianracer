import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 启动雷达
def generate_launch_description():
    camera = os.environ.get("TIANRACER_RGBD_CAMERA", "realsense_d415")

    namespace = LaunchConfiguration(
        'namespace',
        default = '',
    )

    ld = LaunchDescription()
   
    # intel realsense series    
    if "realsense_d" in camera:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes', 'rs_launch.py')),
                launch_arguments={'camera_namespace': namespace,
                                  }.items(),
            ))
        
    # orbbec gemini series 
    if "gemini" in camera:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tianracer_bringup"),
                'launch','includes', 'gemini_330_series.launch.py')),
                launch_arguments={'camera_name': namespace,
                                  'cloud_frame_id': [namespace, "/camera_link"]
                                  }.items(),
            ))
        
    # elif ...

    return ld