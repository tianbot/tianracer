# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

""" A simple launch file for the nmea_serial_driver node. """

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == ' ' or default_namespace =='/' else default_namespace
default_serial_port = os.environ.get("TIANRACER_GPS", "")
default_frame_id = f"gps" if default_namespace ==  '' else f"{default_namespace}/gps"

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    frame_id = LaunchConfiguration('frame_id')
    serial_port = LaunchConfiguration('serial_port')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=default_namespace,
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value=default_serial_port,
        description='Which serial port to be applied'
    )

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value=default_frame_id,
        description='Whether to apply a namespace to the sensor topic frame_id'
    )

    """Generate a launch description for a single serial driver."""
    config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "nmea_serial_driver.yaml")
    driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        namespace=namespace,
        parameters=[config_file,
                    {
                     'port': serial_port,
                     'baud': 38400,
                     'frame_id': frame_id,
                     'time_ref_source': "gps",
                    }
                    ])

    return LaunchDescription([driver_node, declare_namespace_cmd, declare_serial_port_cmd, declare_frame_id_cmd])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)
