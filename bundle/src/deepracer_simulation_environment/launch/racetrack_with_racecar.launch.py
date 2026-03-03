# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch description for racetrack_with_racecar - complete simulation environment setup
    Based on original ROS 1 racetrack_with_racecar.launch
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('deepracer_simulation_environment')
    
    # Set GZ_SIM_RESOURCE_PATH environment variable for all child processes
    # Include both the share directory (for deepracer_simulation_environment package resources)
    # and the models directory (for model:// URIs like model://Singapore)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                '..'  # Go up one level to get the share directory that contains deepracer_simulation_environment
            ]),
            ':',
            PathJoinSubstitution([
                FindPackageShare('deepracer_simulation_environment'),
                'deepracer_simulation_environment' 
            ]),
            ':',
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
        ]
    )
    
    # Declare launch arguments with proper default values
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        description='Name of the world to load'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui', 
        default_value='false',
        description='Launch GUI'
    )
    
    kinesis_video_stream_names_arg = DeclareLaunchArgument(
        'kinesis_video_stream_names',
        description='Comma-separated list of Kinesis video stream names'
    )
    
    kinesis_video_stream_region_arg = DeclareLaunchArgument(
        'kinesis_video_stream_region', 
        default_value='us-east-1',
        description='AWS region for Kinesis video streams'
    )
    
    racecars_with_stereo_cameras_arg = DeclareLaunchArgument(
        'racecars_with_stereo_cameras', 
        default_value='racecar',
        description='Comma-separated list of racecars with stereo cameras'
    )
    
    racecars_with_lidars_arg = DeclareLaunchArgument(
        'racecars_with_lidars', 
        default_value='racecar',
        description='Comma-separated list of racecars with lidars'
    )
    
    body_shell_types_arg = DeclareLaunchArgument(
        'body_shell_types',
        description='Comma-separated list of body shell types'
    )
    
    car_colors_arg = DeclareLaunchArgument(
        'car_colors',
        default_value='Black',
        description='Comma-separated list of car colors'
    )
    
    simapp_versions_arg = DeclareLaunchArgument(
        'simapp_versions',
        description='Comma-separated list of SimApp versions'
    )
    
    multicar_arg = DeclareLaunchArgument(
        'multicar', 
        default_value='false',
        description='Enable multi-car mode'
    )
    
    publish_to_kinesis_stream_arg = DeclareLaunchArgument(
        'publish_to_kinesis_stream', 
        default_value='true',
        description='Enable publishing to Kinesis stream'
    )
    
    make_required_arg = DeclareLaunchArgument(
        'make_required', 
        default_value='true',
        description='Make nodes required'
    )
    
    # Paths to launch files
    racecar_launch_path = os.path.join(pkg_share, 'launch', 'racecar.launch.py')
    empty_world_launch_path = os.path.join(pkg_share, 'launch', 'empty_world.launch.py')
    
    # Single car racecar launch (unless multicar)
    single_car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(racecar_launch_path),
        launch_arguments=[
            ('racecar_name', 'racecar'),
            ('racecar_bitmask', '0x01'),
            ('simapp_version', PythonExpression([
                '"', LaunchConfiguration('simapp_versions'), '".split(",")[0] if "',
                LaunchConfiguration('simapp_versions'), '" else "3.0"'
            ])),
            ('racecar_xacro_file', PythonExpression([
                '"mit" if ("', LaunchConfiguration('simapp_versions'), '".split(",")[0] if "',
                LaunchConfiguration('simapp_versions'), '" else "3.0") == "1.0" else ',
                '"confetti" if ("', LaunchConfiguration('simapp_versions'), '".split(",")[0] if "',
                LaunchConfiguration('simapp_versions'), '" else "3.0") == "2.0" else ',
                '"deepracer" if ("', LaunchConfiguration('simapp_versions'), '".split(",")[0] if "',
                LaunchConfiguration('simapp_versions'), '" else "3.0") in ["3.0", "4.0"] else ',
                '"deepracer_kinematics" if float("', LaunchConfiguration('simapp_versions'), '".split(",")[0] if "',
                LaunchConfiguration('simapp_versions'), '" else "3.0") >= 5.0 else "deepracer"'
            ])),
            ('kinesis_video_stream_name', PythonExpression([
                '"', LaunchConfiguration('kinesis_video_stream_names'), '".split(",")[0] if "',
                LaunchConfiguration('kinesis_video_stream_names'), '" else ""'
            ])),
            ('kinesis_video_stream_region', LaunchConfiguration('kinesis_video_stream_region')),
            ('include_second_camera', PythonExpression([
                '"true" if "racecar" in "', LaunchConfiguration('racecars_with_stereo_cameras'), '" else "false"'
            ])),
            ('include_lidar_sensor', 'true'),
            ('body_shell_type', PythonExpression([
                '"', LaunchConfiguration('body_shell_types'), '".split(",")[0] if "',
                LaunchConfiguration('body_shell_types'), '" else "deepracer"'
            ])),
            ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream')),
            ('make_required', LaunchConfiguration('make_required'))
        ],
        condition=UnlessCondition(LaunchConfiguration('multicar'))
    )
    
    # Multi-car racecar_0 launch (if multicar)
    multicar_0_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(racecar_launch_path),
        launch_arguments=[
            ('racecar_name', 'racecar_0'),
            ('racecar_bitmask', '0x01'),
            ('simapp_version', PythonExpression([
                '"', LaunchConfiguration('simapp_versions'), '".split(",")[0] if "',
                LaunchConfiguration('simapp_versions'), '" else "3.0"'
            ])),
            ('racecar_xacro_file', 'deepracer'),
            ('kinesis_video_stream_name', PythonExpression([
                '"', LaunchConfiguration('kinesis_video_stream_names'), '".split(",")[0] if "',
                LaunchConfiguration('kinesis_video_stream_names'), '" else ""'
            ])),
            ('kinesis_video_stream_region', LaunchConfiguration('kinesis_video_stream_region')),
            ('include_second_camera', PythonExpression([
                '"true" if "racecar_0" in "', LaunchConfiguration('racecars_with_stereo_cameras'), '" else "false"'
            ])),
            ('include_lidar_sensor', 'true'),
            ('body_shell_type', 'deepracer'),
            ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream')),
            ('make_required', LaunchConfiguration('make_required'))
        ],
        condition=IfCondition(LaunchConfiguration('multicar'))
    )
    
    # Multi-car racecar_1 launch (if multicar)
    multicar_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(racecar_launch_path),
        launch_arguments=[
            ('racecar_name', 'racecar_1'),
            ('racecar_bitmask', '0x02'),
            ('simapp_version', '2.0'),
            ('racecar_xacro_file', 'deepracer'),
            ('kinesis_video_stream_name', ''),
            ('kinesis_video_stream_region', LaunchConfiguration('kinesis_video_stream_region')),
            ('include_second_camera', PythonExpression([
                '"true" if "racecar_1" in "', LaunchConfiguration('racecars_with_stereo_cameras'), '" else "false"'
            ])),
            ('include_lidar_sensor', 'true'),
            ('body_shell_type', 'deepracer'),
            ('publish_to_kinesis_stream', LaunchConfiguration('publish_to_kinesis_stream')),
            ('make_required', LaunchConfiguration('make_required'))
        ],
        condition=IfCondition(LaunchConfiguration('multicar'))
    )
    
    # Empty world launch
    empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(empty_world_launch_path),
        launch_arguments=[
            ('world_name', LaunchConfiguration('world_name')),
            ('gui', LaunchConfiguration('gui')),
            ('make_required', LaunchConfiguration('make_required'))
        ]
    )
    
    # Car reset node
    car_reset_node = Node(
        package='deepracer_simulation_environment',
        executable='car_node.py',
        arguments=[PythonExpression(["'2' if '", LaunchConfiguration('multicar'), "'.lower() == 'true' else '1'"])],
        output='screen',
        parameters=[{
            'car_color': LaunchConfiguration('car_colors'),
        }],
    )
    
    # Agents video editor
    agents_video_editor = Node(
        package='deepracer_simulation_environment',
        executable='agents_video_editor.py',
        arguments=[PythonExpression(["'2' if '", LaunchConfiguration('multicar'), "'.lower() == 'true' else '1'"]),
                   LaunchConfiguration('publish_to_kinesis_stream')],
        output='screen'
    )
    
    # Web video server for browser-based video streaming
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )
    
    return LaunchDescription([
        # Environment variables (must be first to affect all child processes)
        gz_resource_path,
        
        # Launch arguments
        world_name_arg,
        gui_arg,
        kinesis_video_stream_names_arg,
        kinesis_video_stream_region_arg,
        racecars_with_stereo_cameras_arg,
        racecars_with_lidars_arg,
        body_shell_types_arg,
        car_colors_arg,
        simapp_versions_arg,
        multicar_arg,
        publish_to_kinesis_stream_arg,
        make_required_arg,
        
        # Launch includes
        single_car_launch,
        multicar_0_launch,
        multicar_1_launch,
        empty_world_launch,
        
        # Nodes
        car_reset_node,
        agents_video_editor,
        web_video_server,
    ])