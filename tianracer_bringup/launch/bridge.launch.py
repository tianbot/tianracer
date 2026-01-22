# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Invoke XML launch file from Python Launch file."""

import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node  # noqa: E402
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == ' ' or default_namespace =='/' else default_namespace
default_frame_id = f"camera_link" if default_namespace ==  '' else f"{default_namespace}/camera_link"

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    ld = LaunchDescription()

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=default_namespace,
        description='Whether to apply a namespace to the navigation stack'
    )

    # Rosbridge Server 
    launch_rosbrideg_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch/rosbridge_websocket_launch.xml",
            )
        )
    )

    # tf2_web_republisher
    tf2_web_republisher = Node(
            package='tf2_web_republisher_py',
            executable='tf2_web_republisher',
            name='tf2_web_republisher',
            namespace=namespace,
            output='screen',
            parameters=[]
        )

    ld.add_action(declare_namespace_cmd)
    ld.add_action(launch_rosbrideg_server)
    ld.add_action(tf2_web_republisher)
    return ld
