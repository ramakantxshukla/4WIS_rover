# Launch Files Guide

## Available Launch Files

### 1. `my_robot_gazebo.launch.py` (Main Simulation)
**Purpose:** Launch Gazebo with the robot and environment

**What it does:**
- Starts Gazebo Harmonic simulator
- Loads the Arena world with ArUco markers
- Spawns the 4WIS robot
- Starts ros_gz_bridge for sensor data
- Sets up environment variables for marker textures

**Usage:**
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.py
```

---

### 2. `spawn_controllers.launch.py` ⭐ (New - All-in-One)
**Purpose:** Spawn all controllers and start teleop in one command

**What it does:**
1. Spawns `joint_state_broadcaster` (publishes joint states)
2. Spawns `wheel_velocity_controller` (4 wheel velocity control)
3. Spawns `steering_position_controller` (4 steering angle control)
4. Starts `four_wheel_steering_controller.py` (4WIS kinematics + odometry)
5. Starts `joy_node` (joystick driver)
6. Starts `teleop_twist_joy` (joystick to cmd_vel conversion)

**Timing:**
- All nodes/spawners are started with delays to ensure proper initialization
- Total startup time: ~6 seconds

**Usage:**
```bash
ros2 launch my_robot_bringup spawn_controllers.launch.py
```

**Benefits:**
- Single command instead of 5+ manual commands
- Automatic timing/sequencing
- Includes teleop ready to use
- Perfect for quick testing and demos

---

### 3. `rover_teleop.launch.py`
**Purpose:** Start joystick control only

**What it does:**
- Starts `joy_node`
- Starts `teleop_twist_joy` with joy.yaml config

**Usage:**
```bash
ros2 launch my_robot_bringup rover_teleop.launch.py
```

**When to use:**
- If you manually spawned controllers (Option B)
- If you only need to add joystick control to a running system

---

### 4. `slam.launch.xml`
**Purpose:** Start SLAM mapping

**What it does:**
- Launches SLAM Toolbox with async configuration
- Creates a map while driving

**Usage:**
```bash
ros2 launch my_robot_bringup slam.launch.xml
```

---

### 5. `navigation.launch.xml`
**Purpose:** Start Nav2 autonomous navigation

**What it does:**
- Launches Nav2 stack with custom parameters
- Enables autonomous navigation with obstacle avoidance

**Usage:**
```bash
ros2 launch my_robot_bringup navigation.launch.xml
```

**Requires:**
- A saved map (from SLAM)
- AMCL for localization

---

## Recommended Workflow

### Quick Start (2 Terminals)
```bash
# Terminal 1: Simulation
ros2 launch my_robot_bringup my_robot_gazebo.launch.py

# Terminal 2: Controllers + Teleop
ros2 launch my_robot_bringup spawn_controllers.launch.py
```

### With SLAM (3 Terminals)
```bash
# Terminal 1: Simulation
ros2 launch my_robot_bringup my_robot_gazebo.launch.py

# Terminal 2: Controllers + Teleop
ros2 launch my_robot_bringup spawn_controllers.launch.py

# Terminal 3: SLAM
ros2 launch my_robot_bringup slam.launch.xml
```

### With Navigation (4 Terminals)
```bash
# Terminal 1: Simulation
ros2 launch my_robot_bringup my_robot_gazebo.launch.py

# Terminal 2: Controllers + Teleop
ros2 launch my_robot_bringup spawn_controllers.launch.py

# Terminal 3: SLAM (if mapping) OR skip if using existing map
ros2 launch my_robot_bringup slam.launch.xml

# Terminal 4: Navigation
ros2 launch my_robot_bringup navigation.launch.xml
```

---

## Controller Spawning Details

### What Controllers Are Used?

1. **joint_state_broadcaster**
   - Type: `JointStateBroadcaster`
   - Publishes: `/joint_states`
   - Purpose: Broadcasts joint positions/velocities for TF and visualization

2. **wheel_velocity_controller**
   - Type: `JointGroupVelocityController`
   - Controls: All 4 wheel joints (front_left, front_right, rear_left, rear_right)
   - Subscribes: `/wheel_velocity_controller/commands`
   - Purpose: Sets wheel rotation speeds

3. **steering_position_controller**
   - Type: `JointGroupPositionController`
   - Controls: All 4 steering joints
   - Subscribes: `/steering_position_controller/commands`
   - Purpose: Sets steering angles (±90°)

### What four_wheel_steering_controller.py Does

**Subscribes to:**
- `/cmd_vel` (geometry_msgs/Twist) - velocity commands
- `/joint_states` - wheel encoder data

**Publishes to:**
- `/wheel_velocity_controller/commands` - wheel velocities
- `/steering_position_controller/commands` - steering angles
- `/odom` - odometry data
- TF: `odom → base_footprint`

**Kinematics:**
- Converts Twist (vx, vy, wz) → 4 wheel velocities + 4 steering angles
- Handles angle wrapping (flips wheel direction if steering >90°)
- Computes odometry from wheel encoders (dead reckoning)

---

## Troubleshooting

### Controllers fail to spawn
**Solution:** Make sure Gazebo is fully loaded before spawning controllers
- Wait 5 seconds after launching Gazebo
- Or use `spawn_controllers.launch.py` which handles timing automatically

### Joystick not working
**Check:**
```bash
# Test joystick
ros2 run joy joy_enumerate_devices
ros2 topic echo /joy

# Check if joy_node is running
ros2 node list | grep joy
```

### Robot doesn't move
**Check:**
```bash
# Verify controllers are active
ros2 control list_controllers

# Check if four_wheel_steering_controller is running
ros2 node list | grep four_wheel

# Test cmd_vel manually
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Odometry not publishing
**Check:**
```bash
# Verify odom topic
ros2 topic echo /odom

# Check TF tree
ros2 run tf2_tools view_frames

# Ensure use_sim_time is set
ros2 param get /four_wheel_steering_controller use_sim_time
```
