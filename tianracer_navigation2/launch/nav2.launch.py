import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_namespace = f"/" if default_namespace == ' ' or default_namespace =='/' else default_namespace
default_frame_id = f"map" if default_namespace ==  '/' else f"{default_namespace}/map"

def generate_launch_description():
    
    nav2_dir = os.path.join(get_package_share_directory('tianracer_navigation2'))
    map_share_dir = os.path.join(get_package_share_directory('tianracer_navigation2'), 'maps')
    path = Path(map_share_dir)
    map_src_dir =  Path(path.parents[4], "src/tianracer", *path.parts[-2:])
    nav2_launch_file_dir = os.path.join(get_package_share_directory('tianracer_navigation2'),'launch')
    rviz_config_file = os.path.join(get_package_share_directory('tianracer_rviz'),'rviz_cfg','nav2_namespaced_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_namespace = LaunchConfiguration('use_namespace', default='true')
    use_map = LaunchConfiguration('use_map', default='tianbotoffice_602')
    use_planner = LaunchConfiguration('use_planner', default='navfn_dwb')

    # https://answers.ros.org/question/358655/ros2-concatenate-string-to-launchargument/
    namespace = LaunchConfiguration(
        'namespace',
        default = f'{default_namespace}'
    )

    map_dir = LaunchConfiguration(
        'map',
        default = [map_src_dir, '/', use_map, '.yaml']
    )
    
    param_dir = LaunchConfiguration(
        'params_file',
        default = [nav2_dir, '/params/', use_planner, '_nav2_params.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument('use_rviz', default_value='True', description='Whether to start RVIZ'),
            DeclareLaunchArgument('map',default_value = map_dir,description = 'Full path to map file to load'),
            DeclareLaunchArgument('params_file',default_value = param_dir,description = 'Full path to param file to load'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir,'/bringup_launch.py']),
                launch_arguments = {
                    'namespace': namespace,
                    'use_namespace': use_namespace,
                    'use_composition': "False",
                    'map':map_dir,
                    'use_sim_time':use_sim_time,
                    'params_file':param_dir
                }.items(),
            ),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'rviz_config': rviz_config_file,
                }.items(),
            )
        ])