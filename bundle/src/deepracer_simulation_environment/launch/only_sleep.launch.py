#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch description for only_sleep - utility launch file
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('deepracer_simulation_environment')
    
    # Path to the shell script
    script_path = os.path.join(pkg_dir, '..', '..', '..', 'lib', 'deepracer_simulation_environment', 'only_sleep.sh')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[script_path],
            name='sleeper',
            output='screen'
        )
    ])
