#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from markov.world_config import WorldConfig


def generate_launch_description():
    """
    Launch description for rollout_rl_agent
    """
    
    # Declare launch arguments
    local_yaml_path_arg = DeclareLaunchArgument('local_yaml_path')
    racecars_with_stereo_cameras_arg = DeclareLaunchArgument('racecars_with_stereo_cameras', default_value='')
    racecars_with_lidars_arg = DeclareLaunchArgument('racecars_with_lidars', default_value='')
    body_shell_types_arg = DeclareLaunchArgument('body_shell_types')
    simapp_versions_arg = DeclareLaunchArgument('simapp_versions')
    kinesis_video_stream_names_arg = DeclareLaunchArgument('kinesis_video_stream_names', 
                                                          default_value=EnvironmentVariable('KINESIS_VIDEO_STREAM_NAME', default_value=''))
    publish_to_kinesis_stream_arg = DeclareLaunchArgument('publish_to_kinesis_stream', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    
    # Get package directory
    pkg_dir = get_package_share_directory('deepracer_simulation_environment')
    
    # Path to the shell script
    script_path = os.path.join(pkg_dir, '..', '..', 'lib', 'deepracer_simulation_environment', 'run_rollout_rl_agent.sh')
    
    return LaunchDescription([
        # Launch arguments
        local_yaml_path_arg,
        racecars_with_stereo_cameras_arg,
        racecars_with_lidars_arg,
        body_shell_types_arg,
        simapp_versions_arg,
        kinesis_video_stream_names_arg,
        publish_to_kinesis_stream_arg,
        gui_arg,
        
        # Load YAML parameters
        WorldConfig.get_launch_parameter(),

        # Include racetrack_with_racecar launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('deepracer_simulation_environment'),
                    'launch',
                    'racetrack_with_racecar.launch.py'
                ])
            ]),
            launch_arguments=[
                ('world_name', EnvironmentVariable('WORLD_NAME')),
                ('kinesis_video_stream_names', LaunchConfiguration('kinesis_video_stream_names')),
                ('kinesis_video_stream_region', EnvironmentVariable('APP_REGION')),
                ('racecars_with_stereo_cameras', LaunchConfiguration('racecars_with_stereo_cameras')),
                ('racecars_with_lidars', LaunchConfiguration('racecars_with_lidars')),
                ('body_shell_types', LaunchConfiguration('body_shell_types')),
                ('simapp_versions', LaunchConfiguration('simapp_versions')),
                ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream')),
                ('gui', LaunchConfiguration('gui'))
            ]
        ),
        
        # Launch the agent
        ExecuteProcess(
            cmd=[
                script_path,
                EnvironmentVariable('ROLLOUT_IDX', default_value='0')
            ],
            name='agent',
            output='screen'
        ),
        
    ])
