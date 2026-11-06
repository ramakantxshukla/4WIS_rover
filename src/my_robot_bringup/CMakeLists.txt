cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds rviz utils maps
  DESTINATION share/${PROJECT_NAME}/
)

# Also install utils to lib so scripts can import it
install(
  DIRECTORY utils
  DESTINATION lib/${PROJECT_NAME}/
)

install(PROGRAMS
  # scripts/imu_odom_fuser.py
  scripts/vel_repub.py
  scripts/four_wheel_steering_controller.py
  scripts/aruco_detector.py
  scripts/marker_map_tf_broadcaster.py
  scripts/aruco_amcl_localizer.py
  scripts/aruco_map_odom_localizer.py
  scripts/zed_aruco_localizer.py
  scripts/zed_aruco_detector.py
  scripts/nav2_relay.py
  # scripts/interactive_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
