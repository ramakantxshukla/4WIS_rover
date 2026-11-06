#!/usr/bin/env python3
import math

# Robot parameters
wheelbase = 0.4  # m
track_width = 0.45  # m
L = wheelbase / 2.0  # 0.2
W = track_width / 2.0  # 0.225

wheel_positions = [
    ("Front Left", L, W),
    ("Front Right", L, -W),
    ("Rear Left", -L, W),
    ("Rear Right", -L, -W)
]

def test_motion(name, linear_x, linear_y, angular_z):
    print(f"\n{'='*70}")
    print(f"TEST: {name}")
    print(f"Command: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")
    print(f"{'='*70}")
    
    for wheel_name, x, y in wheel_positions:
        # Velocity at wheel position
        vx = linear_x - angular_z * y
        vy = linear_y + angular_z * x
        
        v_mag = math.sqrt(vx**2 + vy**2)
        angle_rad = math.atan2(vy, vx)
        
        # Handle angles outside ±90° by flipping
        max_steer = math.pi / 2.0
        wheel_vel = v_mag
        if angle_rad > max_steer:
            angle_rad -= math.pi
            wheel_vel = -v_mag
        elif angle_rad < -max_steer:
            angle_rad += math.pi
            wheel_vel = -v_mag
        
        angle_deg = math.degrees(angle_rad)
        
        print(f"{wheel_name:12s}: steering={angle_deg:6.1f}°, velocity={wheel_vel:7.4f} m/s")

# Test cases
test_motion("Pure Forward (linear.x)", 0.5, 0.0, 0.0)
test_motion("Pure Sideways Right (linear.y)", 0.0, 0.5, 0.0)
test_motion("Pure Rotation CCW (angular.z)", 0.0, 0.0, 0.5)
test_motion("Forward + Rotation", 0.3, 0.0, 0.3)
test_motion("Sideways + Rotation", 0.0, 0.3, 0.3)
test_motion("Diagonal (Forward+Right)", 0.3, 0.3, 0.0)
test_motion("Combined All Three", 0.3, 0.2, 0.3)
print()
