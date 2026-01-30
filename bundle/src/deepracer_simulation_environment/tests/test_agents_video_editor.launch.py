#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Test launch description for agents_video_editor
    Migrated from ROS 1 XML to ROS 2 Python launch
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('deepracer_simulation_environment')
    
    return LaunchDescription([
        # Set parameters (equivalent to ROS 1 <param> tags)
        SetParameter(name='JOB_TYPE', value='TIME_TRIAL'),
        SetParameter(name='WORLD_NAME', value='reInvent2019_track'),
        SetParameter(name='RACECAR_NAMES', value='Racer1'),
        SetParameter(name='test_module', value='mp4_saving'),
        
        # Set environment variable for test module (for pytest_runner.py)
        SetEnvironmentVariable(name='TEST_MODULE', value='mp4_saving'),
        
        # Test camera topic node
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(pkg_dir, '..', '..', '..', 'lib', 'deepracer_simulation_environment', 'camera_topic_node.py'),
                '1'
            ],
            name='test_camera_topic_node',
            output='screen'
        ),
        
        # Agents video editor node
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(pkg_dir, '..', '..', '..', 'lib', 'deepracer_simulation_environment', 'agents_video_editor.py'),
                '1'
            ],
            name='agent_video_editor',
            output='screen'
        ),
        
        # Pytest runner (test execution)
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(pkg_dir, '..', '..', '..', 'lib', 'deepracer_simulation_environment', 'pytest_runner.py'),
                '--gtest_output=xml:/tmp/test_results.xml'
            ],
            name='test_agents_video_editor',
            output='screen'
        )
    ])
