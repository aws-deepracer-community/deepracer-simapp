#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter, Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch description for local_training
    Based on original ROS 1 local_training.launch
    """
    
    # Declare required launch arguments 
    body_shell_types_arg = DeclareLaunchArgument(
        'body_shell_types',
        description='Comma-separated list of body shell types'
    )
    
    simapp_versions_arg = DeclareLaunchArgument(
        'simapp_versions', 
        description='Comma-separated list of simapp versions'
    )
    
    # Set global parameters with default values
    world_name_param = SetParameter(name='WORLD_NAME', value=EnvironmentVariable('WORLD_NAME', default_value='reInvent2019_track_cw'))
    model_s3_bucket_param = SetParameter(name='MODEL_S3_BUCKET', value=EnvironmentVariable('MODEL_S3_BUCKET', default_value='test-bucket'))
    model_s3_prefix_param = SetParameter(name='MODEL_S3_PREFIX', value=EnvironmentVariable('MODEL_S3_PREFIX', default_value='test-prefix'))
    training_job_arn_param = SetParameter(name='TRAINING_JOB_ARN', value=EnvironmentVariable('TRAINING_JOB_ARN', default_value='test-arn'))
    aws_region_param = SetParameter(name='AWS_REGION', value=EnvironmentVariable('APP_REGION', default_value='us-east-1'))
    job_type_param = SetParameter(name='JOB_TYPE', value='TRAINING')
    
    # Include racetrack_with_racecar.launch
    racetrack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'launch',
                'racetrack_with_racecar.launch.py'
            ])
        ),
        launch_arguments=[
            ('world_name', 'reInvent2019_track_cw'),
            ('kinesis_video_stream_names', 'none'),
            ('body_shell_types', LaunchConfiguration('body_shell_types')),
            ('simapp_versions', LaunchConfiguration('simapp_versions')),
            ('gui', 'true'),
            ('publish_to_kinesis_stream', 'false'),
            ('multicar', 'false'),
            ('racecars_with_lidars', 'racecar'),
            ('racecars_with_stereo_cameras', 'none'),
            ('kinesis_video_stream_region', 'us-east-1')
        ]
    )
    
    # RL Agent node 
    agent_node = Node(
        package='deepracer_simulation_environment',
        executable='run_local_rl_agent.sh',
        name='agent',
        output='screen'
    )
    
    return LaunchDescription([
        # Required launch arguments
        body_shell_types_arg,
        simapp_versions_arg,
        
        # Global parameters
        world_name_param,
        model_s3_bucket_param,
        model_s3_prefix_param,
        training_job_arn_param,
        aws_region_param,
        job_type_param,
        
        # Launch includes
        racetrack_launch,
        
        # Nodes
        agent_node,
    ])
