import os
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
    )
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_prefix("tianracer_description"), 'share')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                      arguments=['-entity', 'tianracer', "-topic", "robot_description"],
                      output='screen')

    urdf_file = os.path.join(get_package_share_directory('tianracer_description'), \
        'urdf/tianracer_compact.urdf')
    robot = xacro.process(urdf_file)
    params = {'robot_description': robot}


    state_pub = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'use_sim_time': use_sim_time},
                    params],
    )

    return LaunchDescription([
        gazebo,
        state_pub,
        spawn_entity,
    ])
