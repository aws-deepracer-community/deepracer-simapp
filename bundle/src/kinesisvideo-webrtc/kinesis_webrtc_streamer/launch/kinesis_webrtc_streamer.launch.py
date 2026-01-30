# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""
This is an example launch file to use when you want to launch a Kinesis WebRTC Streamer node.
By default it starts up with a single stream, but the stream_count parameter can be adjusted
and additional stream configuration files can be loaded using the webrtc_config.yaml provided
along with the package.

Parameters:
    config_file: Configuration for the streams. Will load the file into the node's namespace.
    stream_name: Name of the Kinesis Video Stream.
    stream_region: AWS region for the stream. Defaults to us-east-1.
    topic: Topic to subscribe to for video frames.
    connect_timeout_ms: Value that determines the length of time, in milliseconds, to wait before timing out a connect request.
    request_timeout_ms: Value that determines the length of time, in milliseconds, to wait before timing out a request.
    max_retries: The maximum number of allowed connections to a single server for your HTTP communications.
    
Environment Variables:
    AWS_KVS_LOG_LEVEL: Sets the KVS WebRTC C SDK logging level
        LOG_LEVEL_VERBOSE 1
        LOG_LEVEL_DEBUG 2
        LOG_LEVEL_INFO 3
        LOG_LEVEL_WARN 4
        LOG_LEVEL_ERROR 5
        LOG_LEVEL_FATAL 6
        LOG_LEVEL_SILENT 7
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='kinesis_webrtc_streamer',
        description='Name of the node'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to config file'
    )
    
    stream_name_arg = DeclareLaunchArgument(
        'stream_name',
        description='Name of the Kinesis Video Stream'
    )
    
    stream_region_arg = DeclareLaunchArgument(
        'stream_region',
        default_value='us-east-1',
        description='AWS region for the stream'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic',
        description='Topic to subscribe to for video frames'
    )
    
    connect_timeout_ms_arg = DeclareLaunchArgument(
        'connect_timeout_ms',
        default_value='5000',
        description='Connection timeout in milliseconds'
    )
    
    request_timeout_ms_arg = DeclareLaunchArgument(
        'request_timeout_ms',
        default_value='5000',
        description='Request timeout in milliseconds'
    )
    
    max_retries_arg = DeclareLaunchArgument(
        'max_retries',
        default_value='10',
        description='Maximum number of retries'
    )
    
    # Set environment variable for KVS WebRTC C SDK logging level
    set_env_var = SetEnvironmentVariable(
        name='AWS_KVS_LOG_LEVEL',
        value='1'
    )
    
    # Create node
    node = Node(
        package='kinesis_webrtc_streamer',
        executable='kinesis_webrtc_streamer',
        name=LaunchConfiguration('node_name'),
        parameters=[
            # Load config file if specified
            LaunchConfiguration('config_file'),
            {
                'aws_client_configuration.region': LaunchConfiguration('stream_region'),
                'aws_client_configuration.connect_timeout_ms': LaunchConfiguration('connect_timeout_ms'),
                'aws_client_configuration.request_timeout_ms': LaunchConfiguration('request_timeout_ms'),
                'aws_client_configuration.max_retries': LaunchConfiguration('max_retries'),
                'webrtc_streamer_configuration.stream0.signaling_channel_name': LaunchConfiguration('stream_name'),
                'webrtc_streamer_configuration.stream0.video_subscription_topic': LaunchConfiguration('topic') + '/encoded',
            }
        ]
    )
    
    return LaunchDescription([
        node_name_arg,
        config_file_arg,
        stream_name_arg,
        stream_region_arg,
        topic_arg,
        connect_timeout_ms_arg,
        request_timeout_ms_arg,
        max_retries_arg,
        set_env_var,
        node
    ])
