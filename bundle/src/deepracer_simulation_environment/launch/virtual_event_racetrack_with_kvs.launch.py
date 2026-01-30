#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch description for virtual_event_racetrack_with_kvs
    """
    
    # Declare launch arguments
    world_name_arg = DeclareLaunchArgument('world_name')
    kinesis_video_stream_names_arg = DeclareLaunchArgument('kinesis_video_stream_names')
    kinesis_video_stream_region_arg = DeclareLaunchArgument('kinesis_video_stream_region')
    kinesis_webrtc_signaling_channel_names_arg = DeclareLaunchArgument('kinesis_webrtc_signaling_channel_names')
    multicar_arg = DeclareLaunchArgument('multicar', default_value='true')
    publish_to_kinesis_stream_arg = DeclareLaunchArgument('publish_to_kinesis_stream', default_value='true')
    
    # Get package directory
    pkg_dir = get_package_share_directory('deepracer_simulation_environment')
    
    return LaunchDescription([
        # Launch arguments
        world_name_arg,
        kinesis_video_stream_names_arg,
        kinesis_video_stream_region_arg,
        kinesis_webrtc_signaling_channel_names_arg,
        multicar_arg,
        publish_to_kinesis_stream_arg,
        
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
                ('world_name', LaunchConfiguration('world_name')),
                ('kinesis_video_stream_names', LaunchConfiguration('kinesis_video_stream_names')),
                ('kinesis_video_stream_region', LaunchConfiguration('kinesis_video_stream_region')),
                ('multicar', LaunchConfiguration('multicar')),
                ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream'))
            ]
        ),
        
        # Virtual event video editor
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(pkg_dir, '..', '..', '..', 'lib', 'deepracer_simulation_environment', 'virtual_event_video_editor.py')
            ],
            name='virtual_event_video_editor',
            output='screen'
        )
    ])
