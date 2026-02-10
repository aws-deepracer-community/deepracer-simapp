#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    """
    Launch description for empty_world
    """
    
    # Declare launch arguments 
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    extra_gazebo_args_arg = DeclareLaunchArgument('extra_gazebo_args', default_value='')
    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    recording_arg = DeclareLaunchArgument('recording', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    physics_arg = DeclareLaunchArgument('physics', default_value='dartsim') 
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false')
    output_arg = DeclareLaunchArgument('output', default_value='screen')
    world_name_arg = DeclareLaunchArgument('world_name', default_value='empty.sdf')
    respawn_gazebo_arg = DeclareLaunchArgument('respawn_gazebo', default_value='false')
    use_clock_frequency_arg = DeclareLaunchArgument('use_clock_frequency', default_value='false')
    pub_clock_frequency_arg = DeclareLaunchArgument('pub_clock_frequency', default_value='100')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='true')
    
    # Set use_sim_time parameter
    use_sim_time_param = SetParameter(name='use_sim_time', value=(LaunchConfiguration('use_sim_time') == 'true'))
    
    # Get the package share directory
    pkg_share = get_package_share_directory('deepracer_simulation_environment')
    
    # Set GZ_SIM_RESOURCE_PATH environment variable for all child processes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            FindPackageShare('deepracer_simulation_environment'),
            ':',
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
        ]
    )
    
    # Set GZ_SIM_SYSTEM_PLUGIN_PATH for ros2_control Plugin and deepracer_gazebo_system plugin integration
    gz_system_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            PathJoinSubstitution([
                FindPackageShare('deepracer_gazebo_system_plugin'),  # package name
                '..', '..', 'lib'
            ]),
            ':',
            '/opt/ros/jazzy/lib',
            ':',
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')
        ]
    )
    # Dynamic world fetching 
    world_file_path = PathJoinSubstitution([
        FindPackageShare('deepracer_simulation_environment'),
        'worlds',
        PythonExpression(['"', LaunchConfiguration('world_name'), '" + ".world"'])
    ])

    
    # Gazebo server (gz sim) 
    gazebo_server = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            world_file_path,
            # Server mode
            '-s' ,
            PythonExpression([
                '"-r" if "', LaunchConfiguration('paused'), '" == "false" else ""'
            ])
        ],
        output=LaunchConfiguration('output'),
        name='gazebo_server',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Gazebo server (gz sim) 
    gazebo_server_hl = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            world_file_path,
            # Server mode
            '-s' ,
            PythonExpression([
                '"-r" if "', LaunchConfiguration('paused'), '" == "false" else ""'
            ]),
            "--headless-render"
        ],
        output=LaunchConfiguration('output'),
        name='gazebo_server',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Gazebo client (gz sim -g)
    gazebo_client = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-g',
            # Verbose flag (use -v instead of --verbose for compatibility)
            PythonExpression([
                '"-v" if "', LaunchConfiguration('verbose'), '" == "true" else ""'
            ])
        ],
        output=LaunchConfiguration('output'),
        name='gazebo_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Bridge configuration file
    bridge_params = os.path.join(
        get_package_share_directory('deepracer_simulation_environment'),
        'params',
        'deepracer_bridge.yaml'
    )

    # Single bridge for all topics with QoS overrides for performance tuning. The deepracer_bridge.yaml wouldn't apply these effectively so added here
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[{
#            'qos_overrides./racecar/scan.publisher.reliability': 'best_effort',
            'qos_overrides./racecar/scan.publisher.depth': 10,
#            'qos_overrides./racecar/camera/zed/rgb/image_rect_color.publisher.reliability': 'best_effort',
            'qos_overrides./racecar/camera/zed/rgb/image_rect_color.publisher.depth': 10,
#            'qos_overrides./racecar/camera/zed_right/rgb/image_rect_color_right.publisher.reliability': 'best_effort',
            'qos_overrides./racecar/camera/zed_right/rgb/image_rect_color_right.publisher.depth': 10,
#            'qos_overrides./racecar/main_camera/zed/rgb/image_rect_color.publisher.reliability': 'best_effort',
            'qos_overrides./racecar/main_camera/zed/rgb/image_rect_color.publisher.depth': 10,
            'qos_overrides./sub_camera/zed/rgb/image_rect_color.publisher.reliability': 'best_effort',
            'qos_overrides./sub_camera/zed/rgb/image_rect_color.publisher.depth': 10,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        paused_arg,
        use_sim_time_arg,
        extra_gazebo_args_arg,
        gui_arg,
        recording_arg,
        debug_arg,
        physics_arg,
        verbose_arg,
        output_arg,
        world_name_arg,
        respawn_gazebo_arg,
        use_clock_frequency_arg,
        pub_clock_frequency_arg,
        make_required_arg,
        
        # Environment variables
        gz_resource_path,
        gz_system_plugin_path,
        
        # Parameters
        use_sim_time_param,
        
        # Processes
        gazebo_server,
        gazebo_server_hl,
        gazebo_client,

        # Nodes
        gazebo_bridge
    ])
