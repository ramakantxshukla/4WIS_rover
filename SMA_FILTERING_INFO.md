# Simple Moving Average (SMA) Filtering for Odometry

## Overview

The four_wheel_steering_controller now includes **Simple Moving Average (SMA)** filtering to improve odometry accuracy by smoothing out encoder noise and measurement jitter.

## What is SMA Filtering?

Simple Moving Average is a digital filter that computes the average of the last N measurements. This helps:
- **Reduce noise** from wheel encoder measurements
- **Smooth out jitter** in velocity calculations
- **Improve map quality** in SLAM by providing more stable odometry
- **Minimize drift** over time

## How It Works

```
Raw Velocity → SMA Filter → Smoothed Velocity → Odometry Integration
```

### Mathematical Formula

For a window size of N samples:

```
v_filtered = (v₁ + v₂ + v₃ + ... + vₙ) / N
```

Each velocity component (vx, vy, omega) is filtered independently.

## Configuration

### Default Settings

```python
sma_window_size = 5  # Number of samples to average (default)
```

### Customizing Window Size

**At launch time:**
```bash
ros2 run my_robot_bringup four_wheel_steering_controller.py \
  --ros-args -p use_sim_time:=true -p sma_window_size:=10
```

**Or with the all-in-one launch file:**
Edit `spawn_controllers.launch.py` and modify:
```python
Node(
    package='my_robot_bringup',
    executable='four_wheel_steering_controller.py',
    name='four_wheel_steering_controller',
    output='screen',
    parameters=[
        {'use_sim_time': True},
        {'sma_window_size': 10}  # Change this value
    ]
)
```

### Window Size Trade-offs

| Window Size | Smoothing | Responsiveness | Best For |
|-------------|-----------|----------------|----------|
| 3-5 | Low | High | Fast movements, dynamic environments |
| 5-10 (default: 5) | Medium | Medium | General use, balanced performance |
| 10-20 | High | Low | Slow movements, maximum map accuracy |
| 1 | None | Instant | Debugging (no filtering) |

## Impact on Performance

### Benefits
✅ Smoother odometry trajectory  
✅ Better SLAM map quality (less jagged walls)  
✅ Reduced oscillations in position estimates  
✅ More stable TF transformations  

### Considerations
⚠️ Adds ~100ms latency (for window_size=5 at 50Hz)  
⚠️ May lag behind very fast movements  
⚠️ Doesn't fix systematic errors (wheel slip, calibration)  

## Implementation Details

### Code Structure

```python
class FourWheelSteeringController(Node):
    def __init__(self):
        # Window size parameter
        self.declare_parameter('sma_window_size', 5)
        self.sma_window_size = self.get_parameter('sma_window_size').value
        
        # Circular buffers for history
        self.vx_history = deque(maxlen=self.sma_window_size)
        self.vy_history = deque(maxlen=self.sma_window_size)
        self.omega_history = deque(maxlen=self.sma_window_size)
    
    def apply_sma_filter(self, vx, vy, omega):
        """Apply SMA to velocity measurements"""
        self.vx_history.append(vx)
        self.vy_history.append(vy)
        self.omega_history.append(omega)
        
        vx_filtered = np.mean(self.vx_history)
        vy_filtered = np.mean(self.vy_history)
        omega_filtered = np.mean(self.omega_history)
        
        return vx_filtered, vy_filtered, omega_filtered
    
    def publish_odometry(self):
        """Odometry with SMA filtering"""
        # Get raw velocities from wheel encoders
        vx_raw, vy_raw, omega_raw = self.compute_odometry_from_wheels()
        
        # Apply SMA filter
        vx, vy, omega = self.apply_sma_filter(vx_raw, vy_raw, omega_raw)
        
        # Integrate filtered velocities to get pose
        # ... (rest of odometry computation)
```

### Memory Usage

Memory per buffer = window_size × 8 bytes (float64)

Example:
- window_size = 5: 3 buffers × 5 × 8 = 120 bytes
- window_size = 20: 3 buffers × 20 × 8 = 480 bytes

**Negligible overhead** for typical configurations.

## Testing SMA Performance

### Visualize Odometry Path

```bash
# Terminal 1: Launch simulation
ros2 launch my_robot_bringup my_robot_gazebo.launch.py

# Terminal 2: Start controllers with SMA
ros2 launch my_robot_bringup spawn_controllers.launch.py

# Terminal 3: Visualize in RViz
rviz2

# Add: Odometry display on /odom topic
# Add: Path display on /odom topic (to see trajectory history)
```

### Compare With/Without Filtering

**Test 1: No filtering (window_size = 1)**
```bash
ros2 run my_robot_bringup four_wheel_steering_controller.py \
  --ros-args -p use_sim_time:=true -p sma_window_size:=1
```

**Test 2: Default filtering (window_size = 5)**
```bash
ros2 run my_robot_bringup four_wheel_steering_controller.py \
  --ros-args -p use_sim_time:=true -p sma_window_size:=5
```

**Test 3: Heavy filtering (window_size = 15)**
```bash
ros2 run my_robot_bringup four_wheel_steering_controller.py \
  --ros-args -p use_sim_time:=true -p sma_window_size:=15
```

Drive the robot in a square pattern and compare:
- Path smoothness in RViz
- SLAM map quality
- Response time to joystick inputs

## Advanced: Comparing Filter Types

SMA is just one type of filter. Other options for future improvement:

| Filter Type | Characteristics | Pros | Cons |
|-------------|----------------|------|------|
| **SMA** (current) | Simple average of last N samples | Easy, predictable, fast | Equal weight to all samples |
| **EMA** (Exponential) | Weighted average, recent samples more important | Responsive, smooth | Requires tuning α parameter |
| **Median** | Middle value of last N samples | Rejects outliers well | Computationally heavier |
| **Kalman** | Optimal estimator with state prediction | Best accuracy | Complex, requires tuning |
| **Low-pass** | Frequency-domain filtering | Good for periodic noise | Requires frequency analysis |

## Tuning Guide

### For Different Scenarios

**1. Racing / Fast Movement**
```python
sma_window_size: 3  # Minimal lag
```

**2. Mapping / SLAM**
```python
sma_window_size: 8-10  # Balance smoothness and response
```

**3. Precise Positioning**
```python
sma_window_size: 15-20  # Maximum smoothing
```

**4. Debugging Odometry Issues**
```python
sma_window_size: 1  # No filtering, raw data
```

### Tuning Process

1. Start with default (5)
2. Drive a known path (e.g., 1m forward)
3. Check actual traveled distance: `ros2 topic echo /odom`
4. If too noisy: increase window size
5. If too laggy: decrease window size
6. Iterate until satisfied

## Monitoring Filter Performance

### Check Current Parameters
```bash
ros2 param get /four_wheel_steering_controller sma_window_size
```

### View Raw vs Filtered Velocity

Add debug logging in the code:
```python
def apply_sma_filter(self, vx, vy, omega):
    # Before filtering
    self.get_logger().info(f'Raw: vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}')
    
    # ... filtering ...
    
    # After filtering
    self.get_logger().info(f'Filtered: vx={vx_filtered:.3f}, vy={vy_filtered:.3f}, omega={omega_filtered:.3f}')
    
    return vx_filtered, vy_filtered, omega_filtered
```

## Limitations

1. **Still subject to wheel slip** - filtering doesn't fix physical issues
2. **Drift accumulates** - dead reckoning inherently drifts over time
3. **No outlier rejection** - large errors are averaged in
4. **Latency** - smoothing introduces delay

## Future Improvements

Potential enhancements:
- [ ] Add Exponential Moving Average (EMA) option
- [ ] Add Median filter for outlier rejection
- [ ] Integrate with IMU for better angular velocity
- [ ] Add robot_localization for sensor fusion
- [ ] Implement adaptive filtering (adjust window based on acceleration)
- [ ] Add ArUco marker corrections for drift compensation

## References

- [Moving Average on Wikipedia](https://en.wikipedia.org/wiki/Moving_average)
- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
- [Odometry Calibration](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)

## See Also

- `four_wheel_steering_controller.py` - Implementation
- `README.md` - General usage
- `4WIS_CONVERSION_NOTES.md` - 4WIS kinematics details
- `LAUNCH_FILES.md` - Launch configuration
