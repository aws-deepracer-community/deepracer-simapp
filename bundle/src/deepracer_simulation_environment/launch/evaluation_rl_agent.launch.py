#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from markov.world_config import WorldConfig

def generate_launch_description():
    """
    Launch description for evaluation_rl_agent
    Based on original ROS 1 evaluation_rl_agent.launch
    """
    
    # Declare launch arguments
    local_yaml_path_arg = DeclareLaunchArgument('local_yaml_path')
    multicar_arg = DeclareLaunchArgument('multicar', default_value='false')
    racecars_with_stereo_cameras_arg = DeclareLaunchArgument('racecars_with_stereo_cameras', default_value='')
    racecars_with_lidars_arg = DeclareLaunchArgument('racecars_with_lidars', default_value='')
    body_shell_types_arg = DeclareLaunchArgument('body_shell_types')
    simapp_versions_arg = DeclareLaunchArgument('simapp_versions')
    kinesis_video_stream_names_arg = DeclareLaunchArgument('kinesis_video_stream_names', 
                                                          default_value=EnvironmentVariable('KINESIS_VIDEO_STREAM_NAME', default_value=''))
    f1_arg = DeclareLaunchArgument('f1', default_value='false')
    publish_to_kinesis_stream_arg = DeclareLaunchArgument('publish_to_kinesis_stream', default_value='true')
    
    # Include racetrack_with_racecar.launch (unless f1)
    racetrack_launch = IncludeLaunchDescription(
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
            ('multicar', LaunchConfiguration('multicar')),
            ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream'))
        ],
        condition=UnlessCondition(LaunchConfiguration('f1'))
    )
    
    # Include racetrack_with_racecar_f1.launch (if f1)
    racetrack_f1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'launch',
                'racetrack_with_racecar_f1.launch.py'
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
            ('multicar', LaunchConfiguration('multicar')),
            ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream'))
        ],
        condition=IfCondition(LaunchConfiguration('f1'))
    )
    
    # Evaluation RL agent node
    agent_node = Node(
        package='deepracer_simulation_environment',
        executable='run_evaluation_rl_agent.sh',
        name='agent',
        output='screen',
        on_exit=Shutdown(reason='evaluation agent exited')
    )
    
    return LaunchDescription([
        # Launch arguments
        local_yaml_path_arg,
        multicar_arg,
        racecars_with_stereo_cameras_arg,
        racecars_with_lidars_arg,
        body_shell_types_arg,
        simapp_versions_arg,
        kinesis_video_stream_names_arg,
        f1_arg,
        publish_to_kinesis_stream_arg,
        
        # Launch includes
        racetrack_launch,
        racetrack_f1_launch,
        
        # Nodes
        agent_node,
    ])
