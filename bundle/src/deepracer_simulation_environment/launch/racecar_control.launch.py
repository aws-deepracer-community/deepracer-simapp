#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    Launch file for racecar control system
    """
    
    # Launch arguments
    racecar_name_arg = DeclareLaunchArgument('racecar_name', default_value='racecar')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='true')
    
    # Controller spawner - spawns controllers defined in YAML configuration
    controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('racecar_name'),
            arguments=[
                # Joint state broadcaster
                'joint_state_broadcaster',
                # Wheel velocity controllers
                'left_rear_wheel_velocity_controller',
                'right_rear_wheel_velocity_controller',
                'left_front_wheel_velocity_controller',
                'right_front_wheel_velocity_controller',
                # Steering position controllers
                'left_steering_hinge_position_controller',
                'right_steering_hinge_position_controller',
                '--controller-manager', 'controller_manager',
                # Lyrical's controller_manager no longer applies the controllers' parameters from the
                # gz_ros2_control parameter file to the controllers it loads; pass the file explicitly.
                '--param-file', PathJoinSubstitution([FindPackageShare('deepracer_simulation_environment'), 'config', 'racecar_control.yaml']),
                '--activate-as-group',
                '--switch-timeout', '20',
            ],
            output='screen'
        )

    return LaunchDescription([
        racecar_name_arg,
        make_required_arg,
        controller_spawner,
    ])
