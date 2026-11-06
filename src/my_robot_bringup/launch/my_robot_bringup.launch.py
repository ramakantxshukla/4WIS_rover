import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node


def generate_launch_description():


    rviz_config_path = os.path.join(get_package_share_path('my_robot_bringup'),
                                    'rviz', 'caterpillar.rviz')


    # ── Package share directories ─────────────────────────────────────────────
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="my_robot_bringup"
    ).find("my_robot_bringup")


    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # ── CHAMP bringup (locomotion controller, state estimator, EKF odometry) ──
    aruco_map_localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_robot_bringup"),
                "launch",
                "aruco_localisation.launch.py",
            )
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_robot_bringup"),
                "launch",
                "my_robot_gazebo.launch.py",
            )
        )
    )


    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_robot_bringup"),
                "launch",
                "navigation.launch.py",
            )
        )
    )


    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )


    # ── Assemble launch description ───────────────────────────────────────────
    return LaunchDescription(
        [
            rviz2_node,
            declare_use_sim_time,
            nav2_launch,
            gazebo_launch,
            aruco_map_localisation,
        ]
    )