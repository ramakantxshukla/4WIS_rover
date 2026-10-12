#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the joy.yaml config file
    config_file = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'joy.yaml'
    )

    # Spawn joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn wheel_velocity_controller with delay
    spawn_wheel_velocity_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['wheel_velocity_controller'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Spawn steering_position_controller with delay
    spawn_steering_position_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['steering_position_controller'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Start four_wheel_steering_controller with delay
    four_wheel_steering_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='my_robot_bringup',
                executable='four_wheel_steering_controller.py',
                name='four_wheel_steering_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Start joy_node with delay
    joy_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Start teleop_twist_joy with delay
    teleop_node = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                output='screen',
                parameters=[config_file, {'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        spawn_joint_state_broadcaster,
        spawn_wheel_velocity_controller,
        spawn_steering_position_controller,
        four_wheel_steering_node,
        joy_node,
        teleop_node,
    ])
