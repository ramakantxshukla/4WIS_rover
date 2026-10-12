from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    return LaunchDescription([
        # Include teleop_twist_joy launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('teleop_twist_joy'),
                    'launch',
                    'teleop-launch.py'
                ])
            ]),
            launch_arguments={'config_filepath': config_file}.items()
        ),
                
    ])