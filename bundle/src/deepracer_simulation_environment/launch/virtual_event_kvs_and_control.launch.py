#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch description for virtual_event_kvs_and_control
    """
    
    # Declare launch arguments
    world_name_arg = DeclareLaunchArgument('world_name')
    kinesis_video_stream_names_arg = DeclareLaunchArgument('kinesis_video_stream_names')
    kinesis_video_stream_region_arg = DeclareLaunchArgument('kinesis_video_stream_region')
    kinesis_webrtc_signaling_channel_names_arg = DeclareLaunchArgument('kinesis_webrtc_signaling_channel_names')
    multicar_arg = DeclareLaunchArgument('multicar', default_value='true')
    
    return LaunchDescription([
        # Launch arguments
        world_name_arg,
        kinesis_video_stream_names_arg,
        kinesis_video_stream_region_arg,
        kinesis_webrtc_signaling_channel_names_arg,
        multicar_arg,
        
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
                ('world_name', LaunchConfiguration('world_name')),
                ('kinesis_video_stream_names', LaunchConfiguration('kinesis_video_stream_names')),
                ('kinesis_video_stream_region', LaunchConfiguration('kinesis_video_stream_region')),
                ('kinesis_webrtc_signaling_channel_names', LaunchConfiguration('kinesis_webrtc_signaling_channel_names')),
                ('multicar', LaunchConfiguration('multicar'))
            ]
        )
    ])
