#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch description for virtual_event_rl_agent
    """
    
    # Declare launch arguments
    local_yaml_path_arg = DeclareLaunchArgument('local_yaml_path')
    multicar_arg = DeclareLaunchArgument('multicar', default_value='true')
    racecars_with_stereo_cameras_arg = DeclareLaunchArgument('racecars_with_stereo_cameras', default_value='')
    racecars_with_lidars_arg = DeclareLaunchArgument('racecars_with_lidars', default_value='')
    simapp_versions_arg = DeclareLaunchArgument('simapp_versions')
    kinesis_video_stream_names_arg = DeclareLaunchArgument('kinesis_video_stream_names', 
                                                          default_value=EnvironmentVariable('KINESIS_VIDEO_STREAM_NAME', default_value=''))
    kinesis_webrtc_signaling_channel_names_arg = DeclareLaunchArgument('kinesis_webrtc_signaling_channel_names')
    publish_to_kinesis_stream_arg = DeclareLaunchArgument('publish_to_kinesis_stream', default_value='true')
    
    # Get package directory
    pkg_dir = get_package_share_directory('deepracer_simulation_environment')
    
    # Path to the shell script
    script_path = os.path.join(pkg_dir, '..', '..', '..', 'lib', 'deepracer_simulation_environment', 'run_virtual_event_rl_agent.sh')
    
    return LaunchDescription([
        # Launch arguments
        local_yaml_path_arg,
        multicar_arg,
        racecars_with_stereo_cameras_arg,
        racecars_with_lidars_arg,
        simapp_versions_arg,
        kinesis_video_stream_names_arg,
        kinesis_webrtc_signaling_channel_names_arg,
        publish_to_kinesis_stream_arg,
        
        # Include virtual_event_racetrack_with_kvs launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('deepracer_simulation_environment'),
                    'launch',
                    'virtual_event_racetrack_with_kvs.launch.py'
                ])
            ]),
            launch_arguments=[
                ('world_name', EnvironmentVariable('WORLD_NAME')),
                ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream')),
                ('kinesis_video_stream_names', LaunchConfiguration('kinesis_video_stream_names')),
                ('kinesis_video_stream_region', EnvironmentVariable('APP_REGION')),
                ('kinesis_webrtc_signaling_channel_names', LaunchConfiguration('kinesis_webrtc_signaling_channel_names')),
                ('multicar', LaunchConfiguration('multicar'))
            ]
        ),
        
        # Launch the agent
        ExecuteProcess(
            cmd=[script_path],
            name='agent',
            output='screen'
        )
    ])
