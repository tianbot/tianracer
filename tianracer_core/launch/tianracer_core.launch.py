import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,TextSubstitution
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString

from launch_ros.actions import (
    Node,
    PushRosNamespace, 
    SetRemap
)

from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import (
    IfCondition,
    UnlessCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)

default_namespace = os.environ.get("TIANRACER_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace

default_map_frame_id = f"map" if default_namespace == '' else f"{default_namespace}/map"
default_odom_frame_id = f"odom" if default_namespace == '' else f"{default_namespace}/odom"
default_base_link_frame_id = f"base_footprint" if default_namespace == '' else f"{default_namespace}/base_footprint"
default_base_frame_id = f"base_link" if default_namespace == '' else f"{default_namespace}/base_link"
default_imu_frame_id = f"imu_link" if default_namespace == '' else f"{default_namespace}/imu_link"

def generate_launch_description():
    # os env variable
    default_namespace = os.environ.get("TIANRACER_NAME", "")
    serial_port = os.environ.get("TIANRACER_BASE_PORT", "/dev/tianbot_base")
    serial_baudrate = os.environ.get("TIANRACER_BASE_BAUDRATE", "115200")
    
    pkg_share = get_package_share_directory("tianracer_core")
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    map_frame_id = LaunchConfiguration('map_frame_id')
    base_link_frame_id = LaunchConfiguration('base_link_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    imu_frame_id = LaunchConfiguration('imu_frame_id')

   # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        }

    # Only it applies when `namespace` is not empty.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("")},
        condition=LaunchConfigurationEquals("namespace", ""),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value=default_namespace, description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'param', 'tianbot_ekf_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Serial port and baud rate for LED driver
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value=TextSubstitution(text=serial_port),
        description='Serial port of the Tianracer base'
    )
    declare_serial_baudrate_cmd = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=TextSubstitution(text=serial_baudrate),
        description='Baudrate of the Tianracer base'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_map_frame_id_cmd = DeclareLaunchArgument(
        'map_frame_id',
        default_value=default_map_frame_id,
        description='map frame_id of the Tianracer base'
    )

    declare_base_link_frame_id_cmd = DeclareLaunchArgument(
        'base_link_frame_id',
        default_value=default_base_link_frame_id,
        description='base_link frame_id of the Tianracer base'
    )

    declare_base_frame_id_cmd = DeclareLaunchArgument(
        'base_frame_id',
        default_value=default_base_frame_id,
        description='base frame_id of the Tianracer base'
    )

    declare_odom_frame_id_cmd = DeclareLaunchArgument(
        'odom_frame_id',
        default_value=default_odom_frame_id,
        description='odom frame_id of the Tianracer base'
    )

    declare_imu_frame_id_cmd = DeclareLaunchArgument(
        'imu_frame_id',
        default_value=default_imu_frame_id,
        description='imu frame_id of the Tianracer base'
    )

    tianracer_core_cmd_group = GroupAction(
        [
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=namespace),
            # SetRemap("/tf", "tf"),
            # SetRemap("/tf_static", "tf_static"),

            Node(
                package='tianbot_core',
                executable='tianbot_core',
                name='tianracer_core',
                respawn=use_respawn,
                respawn_delay=2.0,
                output='screen',
                parameters=[{'serial_port': LaunchConfiguration('serial_port'),
                             'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                             'namespace': LaunchConfiguration('namespace'),
                             'autostart': autostart,
                             'type': 'ackermann',
                             'type_verify': False,
                             'publish_tf': False,
                             'base_frame': base_frame_id,
                             'odom_frame': odom_frame_id,
                             'imu_frame': imu_frame_id,
                }],
                arguments=["--ros-args", "--log-level", log_level],
            ),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                respawn=use_respawn,
                respawn_delay=2.0,
                output='screen',
                parameters=[params_file, {
                            'autostart': autostart,
                            'map_frame': map_frame_id,
                            'odom_frame': odom_frame_id,
                            'base_link_frame': base_link_frame_id,
                            'world_frame': odom_frame_id
                }],
                arguments=["--ros-args", "--log-level", log_level],
            )
        ]
    )

    return LaunchDescription([
        # Set environment variables
        stdout_linebuf_envvar,
        colorized_output_envvar,

        # Declare the launch options
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,

        declare_serial_port_cmd,
        declare_serial_baudrate_cmd,
        declare_map_frame_id_cmd,
        declare_odom_frame_id_cmd,
        declare_base_link_frame_id_cmd,
        declare_base_frame_id_cmd,
        declare_imu_frame_id_cmd,

        # Add the actions to launch all of the navigation nodes
        tianracer_core_cmd_group
    ])
