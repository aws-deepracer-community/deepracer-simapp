#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Launch file for racecar spawning and control in Gazebo simulation
    """
    
    # Required launch arguments - no defaults
    kinesis_video_stream_name_arg = DeclareLaunchArgument('kinesis_video_stream_name')
    racecar_xacro_file_arg = DeclareLaunchArgument('racecar_xacro_file')
    simapp_version_arg = DeclareLaunchArgument('simapp_version')
    
    # Optional launch arguments with defaults
    kinesis_video_stream_region_arg = DeclareLaunchArgument('kinesis_video_stream_region', default_value='us-east-1')
    body_shell_type_arg = DeclareLaunchArgument('body_shell_type', default_value='deepracer')
    racecar_name_arg = DeclareLaunchArgument('racecar_name', default_value='racecar')
    racecar_bitmask_arg = DeclareLaunchArgument('racecar_bitmask', default_value='0x01')
    publish_to_kinesis_stream_arg = DeclareLaunchArgument('publish_to_kinesis_stream', default_value='false')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='true')
    
    # Camera and sensor configuration
    include_second_camera_arg = DeclareLaunchArgument('include_second_camera', default_value='false')
    include_lidar_sensor_arg = DeclareLaunchArgument('include_lidar_sensor', default_value='false')
    
    # Lidar 360 degree configuration parameters
    lidar_360_degree_sample_arg = DeclareLaunchArgument('lidar_360_degree_sample', default_value='64')
    lidar_360_degree_horizontal_resolution_arg = DeclareLaunchArgument('lidar_360_degree_horizontal_resolution', default_value='1')
    lidar_360_degree_min_angle_arg = DeclareLaunchArgument('lidar_360_degree_min_angle', default_value='-2.61799')
    lidar_360_degree_max_angle_arg = DeclareLaunchArgument('lidar_360_degree_max_angle', default_value='2.61799')
    lidar_360_degree_min_range_arg = DeclareLaunchArgument('lidar_360_degree_min_range', default_value='0.15')
    lidar_360_degree_max_range_arg = DeclareLaunchArgument('lidar_360_degree_max_range', default_value='12.0')
    lidar_360_degree_range_resolution_arg = DeclareLaunchArgument('lidar_360_degree_range_resolution', default_value='0.01')
    lidar_360_degree_noise_mean_arg = DeclareLaunchArgument('lidar_360_degree_noise_mean', default_value='0.0')
    lidar_360_degree_noise_stddev_arg = DeclareLaunchArgument('lidar_360_degree_noise_stddev', default_value='0.01')
    
    # Generate robot description from xacro file with all parameters
    robot_description_content = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('deepracer_simulation_environment'),
            'urdf',
            LaunchConfiguration('racecar_xacro_file'),
            'racecar.xacro'
        ]),
        ' racecar_name:=', LaunchConfiguration('racecar_name'),
        ' racecar_bitmask:=', LaunchConfiguration('racecar_bitmask'),
        ' include_second_camera:=', LaunchConfiguration('include_second_camera'),
        ' include_lidar_sensor:=', LaunchConfiguration('include_lidar_sensor'),
        ' lidar_360_degree_sample:=', LaunchConfiguration('lidar_360_degree_sample'),
        ' lidar_360_degree_horizontal_resolution:=', LaunchConfiguration('lidar_360_degree_horizontal_resolution'),
        ' lidar_360_degree_min_angle:=', LaunchConfiguration('lidar_360_degree_min_angle'),
        ' lidar_360_degree_max_angle:=', LaunchConfiguration('lidar_360_degree_max_angle'),
        ' lidar_360_degree_min_range:=', LaunchConfiguration('lidar_360_degree_min_range'),
        ' lidar_360_degree_max_range:=', LaunchConfiguration('lidar_360_degree_max_range'),
        ' lidar_360_degree_range_resolution:=', LaunchConfiguration('lidar_360_degree_range_resolution'),
        ' lidar_360_degree_noise_mean:=', LaunchConfiguration('lidar_360_degree_noise_mean'),
        ' lidar_360_degree_noise_stddev:=', LaunchConfiguration('lidar_360_degree_noise_stddev'),
        ' body_shell_type:=', LaunchConfiguration('body_shell_type')
    ]), value_type=str)
    
    # Robot state publisher - publishes robot_description and transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('racecar_name'),
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Spawn the racecar model in Gazebo simulation
    # Add delay to ensure robot_state_publisher is ready
    racecar_spawn = TimerAction(
        period=30.0,  # Wait 30 seconds for robot_state_publisher
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', LaunchConfiguration('racecar_name'),
                    '-topic', [LaunchConfiguration('racecar_name'), '/robot_description'],
                    '-z', '0.05'
                ],
                name='racecar_spawn',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Include control system for simapp versions below 5.0
    racecar_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'launch',
                'racecar_control.launch.py'
            ])
        ),
        launch_arguments=[
            ('racecar_name', LaunchConfiguration('racecar_name')),
            ('make_required', LaunchConfiguration('make_required'))
        ],
        condition=IfCondition(PythonExpression([
            'float("', LaunchConfiguration('simapp_version'), '") < 5.0'
        ]))
    )
    
    # Include kinematics control system for simapp versions 5.0 and above
    racecar_control_kinematics_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'launch',
                'racecar_control_kinematics.launch.py'
            ])
        ),
        launch_arguments=[
            ('racecar_name', LaunchConfiguration('racecar_name')),
            ('make_required', LaunchConfiguration('make_required'))
        ],
        condition=IfCondition(PythonExpression([
            'float("', LaunchConfiguration('simapp_version'), '") >= 5.0'
        ]))
    )
    
    # Include H264 video encoder when streaming to Kinesis is enabled AND stream name is provided
    h264_encoder_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'launch',
                'h264_video_encoder.launch.py'
            ])
        ),
        launch_arguments=[
            ('config_file', PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'config', 'racecar', 'deepracer_rpi_h264_encoder_config.yaml'
            ])),
            ('image_transport', 'raw'),
            ('racecar_name', LaunchConfiguration('racecar_name'))
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('publish_to_kinesis_stream'), "'.lower() == 'true' and '",
            LaunchConfiguration('kinesis_video_stream_name'), "' != ''"
        ]))
    )
    
    # Include Kinesis video streamer when streaming is enabled AND stream name is provided
    kinesis_streamer_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'launch',
                'kinesis_video_streamer.launch.py'
            ])
        ),
        launch_arguments=[
            ('node_config_file', PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'config', 'deepracer_node_config.yaml'
            ])),
            ('stream_config_file', PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'config', 'racecar', 'deepracer_stream_config.yaml'
            ])),
            ('kinesis_video_stream_name', LaunchConfiguration('kinesis_video_stream_name')),
            ('kinesis_video_stream_region', LaunchConfiguration('kinesis_video_stream_region')),
            ('racecar_name', LaunchConfiguration('racecar_name')),
            ('subscription_topic', [LaunchConfiguration('racecar_name'), '/b9/kvs/video/encoded'])
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('publish_to_kinesis_stream'), "'.lower() == 'true' and '",
            LaunchConfiguration('kinesis_video_stream_name'), "' != ''"
        ]))
    )
    
    return LaunchDescription([
        # Launch arguments
        kinesis_video_stream_name_arg,
        kinesis_video_stream_region_arg,
        body_shell_type_arg,
        racecar_xacro_file_arg,
        racecar_name_arg,
        racecar_bitmask_arg,
        publish_to_kinesis_stream_arg,
        make_required_arg,
        include_second_camera_arg,
        include_lidar_sensor_arg,
        lidar_360_degree_sample_arg,
        lidar_360_degree_horizontal_resolution_arg,
        lidar_360_degree_min_angle_arg,
        lidar_360_degree_max_angle_arg,
        lidar_360_degree_min_range_arg,
        lidar_360_degree_max_range_arg,
        lidar_360_degree_range_resolution_arg,
        lidar_360_degree_noise_mean_arg,
        lidar_360_degree_noise_stddev_arg,
        simapp_version_arg,
        
        # Debug logging for control system selection
        
        # Robot setup
        robot_state_publisher,
        racecar_spawn,
        
        # Conditional includes
        racecar_control_include,
        racecar_control_kinematics_include,
        h264_encoder_include,
        kinesis_streamer_include,
    ])
