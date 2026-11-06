import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["map_server"]


    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )


    map_path = PathJoinSubstitution([
        get_package_share_directory("my_robot_bringup"),
        "maps",
        "arena",
        "map.yaml"
    ])
    
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )



    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'zed_camera_link']
    )


    marker_map_tf_broadcaster = Node(
        package="my_robot_bringup",
        executable="marker_map_tf_broadcaster.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )


    aruco_map_odom_localizer = Node(
        package="my_robot_bringup",
        executable="aruco_map_odom_localizer.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )


    nav2_relay = Node(
        package="my_robot_bringup",
        executable="nav2_relay.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )


    return LaunchDescription([
        marker_map_tf_broadcaster,
        aruco_map_odom_localizer,
        nav2_relay,
        use_sim_time_arg,
        nav2_map_server,
        nav2_lifecycle_manager,
        static_transform_map_to_odom
    ])