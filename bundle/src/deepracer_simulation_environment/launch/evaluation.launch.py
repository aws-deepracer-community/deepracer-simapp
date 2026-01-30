#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch description for evaluation
    Based on original ROS 1 evaluation.launch
    """
    
    # Download params and launch agent node
    download_params_node = Node(
        package='deepracer_simulation_environment',
        executable='download_params_and_roslaunch_agent.py',
        name='download_params_and_roslaunch_agent_node',
        arguments=[
            EnvironmentVariable('APP_REGION'),
            EnvironmentVariable('MODEL_S3_BUCKET'),
            EnvironmentVariable('MODEL_S3_PREFIX'),
            EnvironmentVariable('S3_YAML_NAME'),
            'evaluation_rl_agent.launch.py'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        download_params_node,
    ])
