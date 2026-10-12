#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    my_robot_description = get_package_share_directory('my_robot_description')
    my_robot_bringup = get_package_share_directory('my_robot_bringup')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    urdf_path = os.path.join(my_robot_description, 'urdf', 'my_robot.urdf.xacro')
    gazebo_config_path = os.path.join(my_robot_bringup, 'config', 'gazebo_bridge.yaml')
    world_path = os.path.join(my_robot_bringup, 'worlds', 'acker.sdf')
    model_path = os.path.join(my_robot_description, 'meshes')
    
    # Build resource path with all marker directories
    marker_paths = []
    for i in [0, 1, 2, 3, 4]:
        marker_path = os.path.join(my_robot_description, 'meshes', f'marker_{i}')
        if os.path.exists(marker_path):
            marker_paths.append(marker_path)
    
    # Combine all paths with colon separator
    all_resource_paths = ':'.join([model_path] + marker_paths)
    
    # Set Gazebo model path to include our markers
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=all_resource_paths
    )
    
    # Robot description with proper ParameterValue wrapping
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_path} -r'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-x', '-8.8928',
            '-y', '1.8949',
            '-z', '1.35'
        ],
        output='screen'
    )
    
    # Gazebo Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gazebo_config_path
        }],
        output='screen'
    )
    
    return LaunchDescription([
        set_gz_resource_path,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        gz_bridge
    ])
