# 4WIS Robot - ROS2 Jazzy + Gazebo Harmonic


A **4-Wheel Independent Steering (4WIS)** robot simulation with omnidirectional movement, SLAM, and autonomous navigation using ROS2 Jazzy and Gazebo Sim 8.

## Robot Overview

- **Drive**: 4-wheel independent drive (all wheels actively driven)
- **Steering**: 4-wheel independent steering (±90° on all wheels)
- **Movement**: Full omnidirectional (forward, backward, sideways, diagonal, rotation)
- **Sensors**: RPLidar S2, Intel RealSense D435, IMU
- **Simulation**: Gazebo Harmonic (gz-sim 8) with gz_ros2_control
- **Environment**: Custom Arena model with ArUco markers for visual localization
- **Control**: Keyboard, joystick, or command-line teleop

## Prerequisites

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-joint-state-broadcaster
sudo apt install ros-jazzy-velocity-controllers ros-jazzy-position-controllers
sudo apt install ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard
```

## Quick Start

### TL;DR - Two Terminal Setup

**Terminal 1:**
```bash
cd ~/ackermann_steering_robot-master
colcon build --symlink-install
source install/setup.bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.py
```

**Terminal 2:**
```bash
source install/setup.bash
ros2 launch my_robot_bringup spawn_controllers.launch.py
```

That's it! The robot is now ready. Connect a joystick or use keyboard teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

### Build and Source
```bash
cd ~/ackermann_steering_robot-master
colcon build --symlink-install
source install/setup.bash
```

## Launch Instructions

### Terminal 1 — Launch Gazebo + Robot

**Use Python launch (recommended):**
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.py
```

This launch file:
- Loads the Arena environment with ArUco markers
- Spawns the 4WIS robot
- Sets up Gazebo resource paths for proper marker texture loading
- Starts the ros_gz_bridge for sensor data

**Alternative - XML launch:**
```bash
# Note: May need to manually set GZ_SIM_RESOURCE_PATH for markers to display correctly
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

### Terminal 2 — Spawn Controllers and Teleop

**Option A: All-in-One Launch (Recommended)**

```bash
source install/setup.bash
ros2 launch my_robot_bringup spawn_controllers.launch.py
```

This single launch file will:
1. Spawn joint_state_broadcaster
2. Spawn wheel_velocity_controller
3. Spawn steering_position_controller
4. Start four_wheel_steering_controller node (4WIS kinematics + odometry)
5. Start joy_node (joystick driver)
6. Start teleop_twist_joy node (joystick control)


### Keyboard Control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controls:
- `i/k`: Forward/Backward (linear.x)
- `j/l`: Left/Right strafe (linear.y)
- `u/o`: Diagonal movement
- `,/.`: Rotate left/right (angular.z)
- `q/z`: Increase/decrease max speeds
- `Space`: Stop
- `CTRL+C`: Quit


