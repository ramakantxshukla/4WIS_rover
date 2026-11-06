# ArUco Markers in Gazebo World

## Overview
The Gazebo world now includes 4 ArUco markers positioned around the environment for testing camera detection and localization.

## Marker Positions

### Marker 0
- **Position**: (5, 0, 1.0) meters
- **Orientation**: Facing West (toward negative X)
- **Location**: 5 meters East of origin, at 1m height

### Marker 1
- **Position**: (0, 5, 1.0) meters
- **Orientation**: Facing South (toward negative Y)
- **Location**: 5 meters North of origin, at 1m height

### Marker 3
- **Position**: (-5, 0, 1.0) meters
- **Orientation**: Facing East (toward positive X)
- **Location**: 5 meters West of origin, at 1m height

### Marker 4
- **Position**: (0, -5, 1.0) meters
- **Orientation**: Facing North (toward positive Y)
- **Location**: 5 meters South of origin, at 1m height

## Pose Format
```xml
<pose>X Y Z Roll Pitch Yaw</pose>
```

Where:
- **X, Y, Z**: Position in meters
- **Roll, Pitch, Yaw**: Orientation in radians
- Pitch = 1.57 (90°) makes markers vertical (standing upright)

## Visual Layout (Top View)
```
              marker_1 (ID=1)
                   |
                   | (0, 5)
                   |
    (-5, 0) -------+------- (5, 0)
   marker_3        |        marker_0
    (ID=3)         |         (ID=0)
                   |
              (0, -5)
                   |
              marker_4 (ID=4)
```

## Usage

### Testing Camera Detection
Position your robot at origin (0, 0) and rotate to face different markers:
```bash
# Face marker_0 (East)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.0}}"

# Face marker_1 (North) - rotate 90° CCW
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Face marker_3 (West) - rotate 180°
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Face marker_4 (South) - rotate 270° or -90°
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.5}}"
```

### Approaching a Marker
```bash
# Drive toward marker_0
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

## Customizing Marker Positions

To change marker positions, edit:
```
src/my_robot_bringup/worlds/acker.sdf
```

Example marker entry:
```xml
<include>
  <uri>model://marker_0</uri>
  <name>marker_0</name>
  <pose>X Y Z Roll Pitch Yaw</pose>
</include>
```

### Tips:
1. **Height**: Keep Z around 0.5-1.5m for camera visibility
2. **Pitch**: Use 1.57 (90°) to make markers vertical
3. **Yaw**: Adjust to face markers toward the center or desired direction
4. **Scale**: Markers are scaled 4x in the model.sdf files

## Adding More Markers

You also have `marker_2` available. To add it:

```xml
<include>
  <uri>model://marker_2</uri>
  <name>marker_2</name>
  <pose>2.5 2.5 1.0 0 1.57 0.785</pose>  <!-- Diagonal position -->
</include>
```

## Troubleshooting

### Markers Not Appearing
1. **Check model path**: Ensure `GZ_SIM_RESOURCE_PATH` is set correctly in launch file
2. **Verify mesh files exist**: Check `src/my_robot_description/meshes/marker_X/`
3. **Check Gazebo logs**: Look for model loading errors

### Camera Not Detecting Markers
1. Ensure camera plugin is running
2. Check camera field of view and range
3. Verify marker is within camera's line of sight
4. Check lighting conditions in simulation

## Camera Topics
Monitor camera output:
```bash
# List camera topics
ros2 topic list | grep camera

# View camera image
ros2 run rqt_image_view rqt_image_view

# Echo camera info
ros2 topic echo /camera/camera_info
```
