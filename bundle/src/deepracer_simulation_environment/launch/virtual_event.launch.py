#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch description for virtual_event
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('deepracer_simulation_environment')
    
    # Path to the Python script
    script_path = os.path.join(pkg_dir, '..', '..', '..', 'lib', 'deepracer_simulation_environment', 'download_params_and_roslaunch_agent.py')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'python3', script_path,
                EnvironmentVariable('APP_REGION'),
                EnvironmentVariable('YAML_S3_BUCKET'),
                EnvironmentVariable('YAML_S3_PREFIX'),
                EnvironmentVariable('S3_YAML_NAME'),
                'virtual_event_rl_agent.launch.py'
            ],
            name='download_params_and_roslaunch_agent_node',
            output='screen'
        )
    ])
