# Quick Start Guide - 4WIS Robot

## Build and Source

```bash
cd ~/ackermann_steering_robot-master
colcon build --symlink-install
source install/setup.bash
```

## Launch Gazebo Simulation

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

## Spawn Controllers (in a new terminal)

```bash
source install/setup.bash

# Spawn all controllers
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner wheel_velocity_controller
ros2 run controller_manager spawner steering_position_controller
```

## Start 4WIS Controller (in a new terminal)

```bash
source install/setup.bash
ros2 run my_robot_bringup four_wheel_steering_controller.py
```

## Test Movement

### Forward Movement
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

### Backward Movement
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}}"
```

### Sideways (Crab Mode) - Right
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.5}}"
```

### Sideways (Crab Mode) - Left
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.5}}"
```

### Diagonal Movement
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.3}}"
```

### Rotate in Place (Counter-clockwise)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

### Rotate in Place (Clockwise)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.5}}"
```

### Stop
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## Check System Status

### List Active Controllers
```bash
ros2 control list_controllers
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

### Monitor Steering Angles
```bash
ros2 topic echo /steering_position_controller/commands
```

### Monitor Wheel Velocities
```bash
ros2 topic echo /wheel_velocity_controller/commands
```

## Troubleshooting

### Controllers Not Loading
```bash
# Check controller manager status
ros2 control list_controllers

# Check for errors
ros2 topic echo /rosout
```

### 4WIS Node Not Working
```bash
# Check if node is running
ros2 node list | grep four_wheel_steering

# Check node output
ros2 run my_robot_bringup four_wheel_steering_controller.py --ros-args --log-level debug
```

### Robot Not Moving
1. Verify controllers are active: `ros2 control list_controllers`
2. Verify 4WIS node is running: `ros2 node list`
3. Check if commands are being published: `ros2 topic hz /cmd_vel`
4. Check steering commands: `ros2 topic echo /steering_position_controller/commands`
5. Check wheel commands: `ros2 topic echo /wheel_velocity_controller/commands`

## Key Features

- **Omnidirectional Movement**: Move in any direction (x, y, and rotation)
- **±90° Steering**: Full range steering on all 4 wheels
- **Zero Turning Radius**: Rotate in place without translating
- **Crab Walking**: Move sideways without changing orientation
- **Diagonal Movement**: Move at any angle
