#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch description for kinesis_video_streamer
    """
    
    # Declare launch arguments
    node_config_file_arg = DeclareLaunchArgument('node_config_file', default_value='')
    stream_config_file_arg = DeclareLaunchArgument('stream_config_file', default_value='')
    kinesis_video_stream_name_arg = DeclareLaunchArgument('kinesis_video_stream_name', 
                                                          default_value=EnvironmentVariable('KINESIS_VIDEO_STREAM_NAME', default_value=''))
    kinesis_video_stream_region_arg = DeclareLaunchArgument('kinesis_video_stream_region', default_value='us-east-1')
    racecar_name_arg = DeclareLaunchArgument('racecar_name', default_value='racecar')
    subscription_topic_arg = DeclareLaunchArgument('subscription_topic', default_value='/racecar/deepracer/kvs_stream')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='false')
    
    return LaunchDescription([
        # Launch arguments
        node_config_file_arg,
        stream_config_file_arg,
        kinesis_video_stream_name_arg,
        kinesis_video_stream_region_arg,
        racecar_name_arg,
        subscription_topic_arg,
        make_required_arg,
        
        # Kinesis Video Streamer node
        Node(
            package='kinesis_video_streamer',
            executable='kinesis_video_streamer',
            name='kinesis_video_streamer',
            parameters=[{
                'kinesis_video.stream_count': 1,
                'kinesis_video.stream0.stream_name': LaunchConfiguration('kinesis_video_stream_name'),
                'kinesis_video.stream0.subscription_topic': LaunchConfiguration('subscription_topic'),
                'kinesis_video.stream0.topic_type': 1,
                'kinesis_video.log4cplus_config': PathJoinSubstitution([
                    FindPackageShare('deepracer_simulation_environment'),
                    'config',
                    'kvs_log_configuration'
                ]),
                'aws_client_configuration.region': LaunchConfiguration('kinesis_video_stream_region')
            }],
            output='screen',
            respawn=True
        )
    ])
