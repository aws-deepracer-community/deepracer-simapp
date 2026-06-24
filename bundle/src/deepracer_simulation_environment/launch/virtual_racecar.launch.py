#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lightweight racecar launch for virtual-event respawn.

Differences from racecar.launch.py:
  - No KVS / H264 / Kinesis streaming
  - No xacro re-processing — accepts a pre-parsed URDF file path
  - Shorter timers (Gazebo is already warm when this is used)
  - Controller-manager referenced by full namespaced path to avoid Jazzy resolution ambiguity
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    racecar_name_arg = DeclareLaunchArgument('racecar_name', default_value='racecar')
    make_required_arg = DeclareLaunchArgument('make_required', default_value='false')
    # Path to a pre-parsed URDF file (written by agent_model._spawn before calling this)
    robot_description_file_arg = DeclareLaunchArgument('robot_description_file')

    robot_description_content = ParameterValue(
        Command(['cat ', LaunchConfiguration('robot_description_file')]),
        value_type=str
    )

    # Publish robot_description and TF transforms immediately
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

    # Spawn into Gazebo after a short delay to let robot_state_publisher publish the topic
    racecar_spawn = TimerAction(
        period=2.0,
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

    # Activate controllers after model is in Gazebo and gz_ros2_control has started.
    # gz_ros2_control starts controller_manager shortly after ros_gz_sim create completes
    # (~2 s timer + ~2 s Gazebo spawn = ~4 s). 5 s is safe; the spawner's own
    # --controller-manager-timeout 30 handles any remaining startup time.
    controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=LaunchConfiguration('racecar_name'),
                arguments=[
                    'joint_state_broadcaster',
                    'left_rear_wheel_velocity_controller',
                    'right_rear_wheel_velocity_controller',
                    'left_front_wheel_velocity_controller',
                    'right_front_wheel_velocity_controller',
                    'left_steering_hinge_position_controller',
                    'right_steering_hinge_position_controller',
                    # Use absolute path to avoid namespace resolution differences in Jazzy
                    '--controller-manager', ['/', LaunchConfiguration('racecar_name'), '/controller_manager'],
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
        robot_description_file_arg,
        robot_state_publisher,
        racecar_spawn,
        controller_spawner,
    ])
