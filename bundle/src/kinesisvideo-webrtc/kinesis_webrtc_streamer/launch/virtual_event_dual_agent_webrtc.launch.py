# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""
Launch file for virtual event dual agent WebRTC streaming.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('kinesis_webrtc_streamer')
    
    # Path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'deepracer_dual_agent_webrtc_config.yaml')
    
    # Declare launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='kinesis_webrtc_streamer',
        description='Name of the node'
    )
    
    racecar_name0_arg = DeclareLaunchArgument(
        'racecar_name0',
        description='Name of the first racecar'
    )
    
    signaling_channel_name0_arg = DeclareLaunchArgument(
        'signaling_channel_name0',
        description='Name of the first signaling channel'
    )
    
    racecar_name1_arg = DeclareLaunchArgument(
        'racecar_name1',
        description='Name of the second racecar'
    )
    
    signaling_channel_name1_arg = DeclareLaunchArgument(
        'signaling_channel_name1',
        description='Name of the second signaling channel'
    )
    
    stream_region_arg = DeclareLaunchArgument(
        'stream_region',
        default_value='us-east-1',
        description='AWS region for the stream'
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
            config_file,
            {
                'aws_client_configuration.region': LaunchConfiguration('stream_region'),
                'aws_client_configuration.connect_timeout_ms': LaunchConfiguration('connect_timeout_ms'),
                'aws_client_configuration.request_timeout_ms': LaunchConfiguration('request_timeout_ms'),
                'aws_client_configuration.max_retries': LaunchConfiguration('max_retries'),
                'webrtc_streamer_configuration.stream0.signaling_channel_name': LaunchConfiguration('signaling_channel_name0'),
                'webrtc_streamer_configuration.stream1.signaling_channel_name': LaunchConfiguration('signaling_channel_name1'),
            }
        ]
    )
    
    return LaunchDescription([
        node_name_arg,
        racecar_name0_arg,
        signaling_channel_name0_arg,
        racecar_name1_arg,
        signaling_channel_name1_arg,
        stream_region_arg,
        connect_timeout_ms_arg,
        request_timeout_ms_arg,
        max_retries_arg,
        set_env_var,
        node
    ])
