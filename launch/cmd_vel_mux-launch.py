# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
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

"""Launch the cmd_vel_mux node with default configuration."""

import os

from launch.actions import IncludeLaunchDescription
import ament_index_python.packages
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import launch_ros.actions

import yaml


def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('cmd_vel_mux')

    joy_dir = ament_index_python.packages.get_package_share_directory('teleop_twist_joy')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, 'config', 'cmd_vel_mux_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['cmd_vel_mux']['ros__parameters']
    ld = LaunchDescription()


    launch_joy = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share_dir,'launch', 'teleop_twist.launch.py')),
                launch_arguments={'joy_config': 'xbox'}.items()
                )

    launch_goal_cancel = launch_ros.actions.Node(
        package='cmd_vel_mux',
        executable='cancel_goal',
        parameters=[{'cancel_button': 0}]
        )

    cmd_vel_mux_node = launch_ros.actions.Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        output='both',
        parameters=[params]
    )
    ld.add_action(cmd_vel_mux_node)
    ld.add_action(launch_joy)
    ld.add_action(launch_goal_cancel)
    return ld
