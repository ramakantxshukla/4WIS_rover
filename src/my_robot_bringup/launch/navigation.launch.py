#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Path to your custom nav2 parameters file
    nav_params_path = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'nav2_params.yaml'
    ])
    
    # Launch Nav2 with collision_monitor enabled
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav_params_path
        }.items()
    )
    
    return LaunchDescription([
        nav2_bringup
    ])