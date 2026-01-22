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

import argparse
import os
from pathlib import Path  # noqa: E402
import sys
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == ' ' or default_namespace =='/' else default_namespace
default_frame_id = f"camera_link" if default_namespace ==  '' else f"{default_namespace}/camera_link"
camera_name= "camera"

remappings = [
                ('image_raw', f'{camera_name}/image_raw'),
                ('image_raw/compressed', f'{camera_name}/image_compressed'),
                ('image_raw/compressedDepth', f'{camera_name}/compressedDepth'),
                ('image_raw/theora', f'{camera_name}/image_raw/theora'),
                ('camera_info', f'{camera_name}/camera_info'),
            ]

def generate_launch_description():
    pkg_share = get_package_share_directory("tianracer_bringup")
    param_path = os.path.join(pkg_share, 'param', 'camera_params_1.yaml')

    namespace = LaunchConfiguration('namespace')
    frame_id = LaunchConfiguration('frame_id')
    ld = LaunchDescription()

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=default_namespace,
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value=default_frame_id,
        description='Whether to apply a namespace to the sensor topic frame_id'
    )
    
    camera_nodes = [
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name=camera_name,
            namespace=namespace,
            parameters=[
                param_path, {
                'frame_id': frame_id,
            }],
            remappings=remappings
        )
    ]

    camera_group = GroupAction(camera_nodes)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(camera_group)
    return ld