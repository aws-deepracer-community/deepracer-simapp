#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch description for h264_video_encoder
    """
    
    # Declare launch arguments
    node_name_arg = DeclareLaunchArgument('node_name', default_value='h264_video_encoder')
    config_file_arg = DeclareLaunchArgument('config_file', 
                                            default_value=PathJoinSubstitution([
                                                FindPackageShare('deepracer_simulation_environment'),
                                                'config', 'racecar', 'deepracer_rpi_h264_encoder_config.yaml'
                                            ]))
    output_arg = DeclareLaunchArgument('output', default_value='log')
    image_transport_arg = DeclareLaunchArgument('image_transport', default_value='compressed')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='false')
    racecar_name_arg = DeclareLaunchArgument('racecar_name', default_value='racecar')
    
    return LaunchDescription([
        # Launch arguments
        node_name_arg,
        config_file_arg,
        output_arg,
        image_transport_arg,
        make_required_arg,
        racecar_name_arg,
        
        # H264 Video Encoder node
        Node(
            package='h264_video_encoder',
            executable='h264_video_encoder',
            name=LaunchConfiguration('node_name'),
            output=LaunchConfiguration('output'),
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'image_transport': LaunchConfiguration('image_transport')
                }
            ],
            respawn=True
        )
    ])
