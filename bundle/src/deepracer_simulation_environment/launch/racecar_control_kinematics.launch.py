#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for racecar control system using kinematics configuration
    """
    
    # Launch arguments
    racecar_name_arg = DeclareLaunchArgument('racecar_name', default_value='racecar')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='true')
    
    # Log start of controller spawning
    
    # Controller spawner with delay for Gazebo initialization
    controller_spawner = TimerAction(
        period=30.0,  # Wait 30 seconds for robot to be spawned and loaded
        actions=[
            Node(
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
                    '--controller-manager-timeout', '30',
                    '--activate-as-group',
                    '--switch-timeout', '20',
                ],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        racecar_name_arg,
        make_required_arg,
        controller_spawner,
    ])