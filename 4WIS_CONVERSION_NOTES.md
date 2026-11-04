# 4WIS (4-Wheel Independent Steering) Conversion

## Overview
This robot has been converted from an Ackermann steering configuration to a 4WIS (4-Wheel Independent Steering) configuration. In 4WIS, all four wheels can steer and drive independently, providing enhanced maneuverability.

## Key Changes Made

### 1. URDF Changes (`mobile_base.xacro`)
- **Added rear steering links**: Created `rear_right_steering_link` and `rear_left_steering_link` with proper inertial properties
- **Added rear steering joints**: Created `base_rear_right_steering_joint` and `base_rear_left_steering_joint` as revolute joints with ±1.5708 rad (±90°) limits
- **Modified rear wheel joints**: Changed rear wheel joints to be children of the steering links instead of directly attached to base_link
- **Joint hierarchy**: All four wheels now follow the pattern: `base_link → steering_link → wheel_link`

### 2. ROS2 Control Configuration (`mobile_base_gazebo.xacro`)
- **All steering joints are now position-controlled**: Added command interfaces for all four steering joints
- **All wheels are now velocity-controlled**: Changed front wheels from passive to active drive
- **Control interfaces**: Each of the 4 steering joints has position command interface, and each of the 4 wheel joints has velocity command interface

### 3. Controller Configuration (`my_robot_controllers.yaml`)
- **Changed controller type**: Using `JointGroupVelocityController` for wheels and `JointGroupPositionController` for steering
- **Wheel controller**: Controls all 4 wheel velocities together
- **Steering controller**: Controls all 4 steering positions together
- **Custom 4WIS kinematics node**: `four_wheel_steering_controller.py` converts `/cmd_vel` to individual wheel commands

### 4. Launch Files (`my_robot_gazebo.launch.xml`)
- Removed TF relay (not needed with new controller setup)

### 5. Velocity Republisher (`vel_repub.py`)
- No longer needed, replaced by `four_wheel_steering_controller.py`

### 6. Four Wheel Steering Controller (`four_wheel_steering_controller.py`)
- New Python node that implements 4WIS kinematics
- Subscribes to `/cmd_vel` (Twist) and `/cmd_vel_stamped` (TwistStamped)
- Calculates individual wheel velocities and steering angles
- Publishes to `/wheel_velocity_controller/commands` and `/steering_position_controller/commands`
- Supports omnidirectional movement (forward, backward, sideways, diagonal, rotation)

### 7. Documentation (`README.md`)
- Updated all references to use the new controller name
- Updated command examples with new topic names

## 4WIS Configuration Details

### Steering Configuration
- **Steering Range**: ±1.5708 radians (±90 degrees) for all wheels
- **Steering Axis**: Z-axis (vertical rotation)
- **Control Type**: Position-controlled via JointGroupPositionController

### Drive Configuration
- **All 4 wheels are driven**: Independent velocity control for each wheel
- **Wheel Radius**: 0.1 m
- **Drive Axis**: Y-axis (wheel rotation)
- **Control Type**: Velocity-controlled via JointGroupVelocityController

### Robot Dimensions
- **Wheelbase**: 0.4 m (front-to-rear distance)
- **Track Width**: 0.45 m (left-to-right distance)
- **Base Length**: 0.8 m
- **Base Width**: 0.4 m

## Advantages of 4WIS

1. **Zero Turning Radius**: Can rotate in place by steering all wheels appropriately
2. **Crab Walking**: Can move sideways by pointing all wheels in the same direction
3. **Diagonal Movement**: Can move at any angle without changing orientation
4. **Enhanced Maneuverability**: Superior control in tight spaces
5. **Complex Trajectories**: Can follow more sophisticated paths

## Usage

### Spawning the Controllers
```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner wheel_velocity_controller
ros2 run controller_manager spawner steering_position_controller
```

### Starting the 4WIS Kinematics Node
```bash
ros2 run my_robot_bringup four_wheel_steering_controller.py
```

### Sending Commands
The controller accepts `Twist` messages on `/cmd_vel`:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Move sideways (crab walking)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Move diagonally
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### For Nav2 Integration
Nav2 can directly publish to `/cmd_vel`, which the 4WIS controller will automatically process.

## Testing Recommendations

1. **Verify Joint States**: Check that all 8 joints (4 steering + 4 wheels) are publishing states
2. **Test Steering Range**: Manually command each steering joint to verify ±90° (±1.5708 rad) limits
3. **Test Individual Wheels**: Command each wheel independently to verify drive capability
4. **Test Coordinated Motion**: Use twist commands to verify proper kinematics calculation
5. **Monitor Odometry**: Ensure odometry is accurate for various motion patterns

## Potential Improvements

1. **Advanced Steering Modes**: Implement mode switching (Ackermann, Crab, Diagonal, Spin)
2. **Coordinated Control**: Add a high-level controller for specific 4WIS maneuvers
3. **Dynamic Steering Limits**: Adjust steering limits based on velocity for stability
4. **Tire Slip Consideration**: Model and compensate for tire slip in complex maneuvers

## Notes

- The robot now uses individual joint controllers (JointGroupVelocityController and JointGroupPositionController) instead of a unified steering controller.
- The `four_wheel_steering_controller.py` node implements the 4WIS kinematics and calculates individual wheel commands.
- All steering angles are calculated to achieve the commanded twist, supporting full omnidirectional movement.
- The Gazebo friction parameters have been set to high values (mu1=5.0, mu2=5.0) to prevent wheel slip.
- With ±90° steering, the robot can move in ANY direction without changing its orientation.
