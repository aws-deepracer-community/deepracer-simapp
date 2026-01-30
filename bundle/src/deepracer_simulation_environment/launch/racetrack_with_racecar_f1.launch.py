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
    Launch description for racetrack_with_racecar_f1 - F1 racing configuration
    """
    
    # Declare launch arguments (same as racetrack_with_racecar but with F1 defaults)
    world_name_arg = DeclareLaunchArgument('world_name')
    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    kinesis_video_stream_names_arg = DeclareLaunchArgument('kinesis_video_stream_names')
    kinesis_video_stream_region_arg = DeclareLaunchArgument('kinesis_video_stream_region', default_value='us-east-1')
    racecars_with_stereo_cameras_arg = DeclareLaunchArgument('racecars_with_stereo_cameras', default_value='')
    racecars_with_lidars_arg = DeclareLaunchArgument('racecars_with_lidars', default_value='')
    body_shell_types_arg = DeclareLaunchArgument('body_shell_types')
    simapp_versions_arg = DeclareLaunchArgument('simapp_versions')
    multicar_arg = DeclareLaunchArgument('multicar', default_value='true')  # F1 typically multicar
    publish_to_kinesis_stream_arg = DeclareLaunchArgument('publish_to_kinesis_stream', default_value='true')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='true')
    
    return LaunchDescription([
        # Launch arguments
        world_name_arg,
        gui_arg,
        kinesis_video_stream_names_arg,
        kinesis_video_stream_region_arg,
        racecars_with_stereo_cameras_arg,
        racecars_with_lidars_arg,
        body_shell_types_arg,
        simapp_versions_arg,
        multicar_arg,
        publish_to_kinesis_stream_arg,
        make_required_arg,
        
        # Include racetrack_with_racecar launch (F1 is essentially the same with different defaults)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('deepracer_simulation_environment'),
                    'launch',
                    'racetrack_with_racecar.launch.py'
                ])
            ]),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'gui': LaunchConfiguration('gui'),
                'kinesis_video_stream_names': LaunchConfiguration('kinesis_video_stream_names'),
                'kinesis_video_stream_region': LaunchConfiguration('kinesis_video_stream_region'),
                'racecars_with_stereo_cameras': LaunchConfiguration('racecars_with_stereo_cameras'),
                'racecars_with_lidars': LaunchConfiguration('racecars_with_lidars'),
                'body_shell_types': LaunchConfiguration('body_shell_types'),
                'simapp_versions': LaunchConfiguration('simapp_versions'),
                'multicar': LaunchConfiguration('multicar'),
                'publish_to_kinesis_stream': LaunchConfiguration('publish_to_kinesis_stream'),
                'make_required': LaunchConfiguration('make_required')
            }.items()
        )
    ])
