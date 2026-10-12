#!/usr/bin/env python3
"""
Four Wheel Independent Steering (4WIS) Controller
Converts Twist commands to individual wheel steering angles and velocities
Also computes odometry from joint states
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import math
import numpy as np


class FourWheelSteeringController(Node):
    def __init__(self):
        super().__init__('four_wheel_steering_controller')
        
        # Robot parameters
        self.wheelbase = 0.4  # front to rear distance (m)
        self.track_width = 0.45  # left to right distance (m)
        self.wheel_radius = 0.1  # wheel radius (m)
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        
        # Joint state storage
        self.joint_positions = {}
        self.joint_velocities = {}
        
        # Subscribers
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        self.twist_stamped_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_stamped', self.twist_stamped_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Publishers for wheel velocities
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        
        # Publishers for steering angles
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry timer (50 Hz)
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)
        
        self.get_logger().info('Four Wheel Steering Controller started')
        self.get_logger().info(f'Wheelbase: {self.wheelbase}m, Track width: {self.track_width}m')

    def joint_state_callback(self, msg):
        """Store joint states for odometry calculation"""
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def compute_odometry_from_wheels(self):
        """
        Compute odometry from wheel velocities and steering angles
        Uses forward kinematics of 4WIS
        """
        # Get wheel velocities (rad/s)
        wheel_names = [
            'base_front_left_wheel_joint',
            'base_front_right_wheel_joint',
            'base_rear_left_wheel_joint',
            'base_rear_right_wheel_joint'
        ]
        
        # Get steering angles (rad)
        steering_names = [
            'base_front_left_steering_joint',
            'base_front_right_steering_joint',
            'base_rear_left_steering_joint',
            'base_rear_right_steering_joint'
        ]
        
        # Check if we have all joint data
        if not all(name in self.joint_velocities for name in wheel_names):
            return 0.0, 0.0, 0.0
        if not all(name in self.joint_positions for name in steering_names):
            return 0.0, 0.0, 0.0
        
        # Get current values
        wheel_vels = [self.joint_velocities[name] for name in wheel_names]
        steer_angles = [self.joint_positions[name] for name in steering_names]
        
        # Convert wheel angular velocities to linear velocities
        linear_vels = [v * self.wheel_radius for v in wheel_vels]
        
        # Compute velocity components for each wheel
        vx_wheels = []
        vy_wheels = []
        
        for i in range(4):
            vx_wheels.append(linear_vels[i] * math.cos(steer_angles[i]))
            vy_wheels.append(linear_vels[i] * math.sin(steer_angles[i]))
        
        # Average to get robot velocity (simple approach)
        vx = np.mean(vx_wheels)
        vy = np.mean(vy_wheels)
        
        # Compute angular velocity from wheel positions
        # Using instantaneous center of rotation (ICR) method
        L = self.wheelbase / 2.0
        W = self.track_width / 2.0
        
        # Wheel positions relative to center
        positions = [
            (L, W),    # front_left
            (L, -W),   # front_right
            (-L, W),   # rear_left
            (-L, -W)   # rear_right
        ]
        
        # Compute angular velocity contribution from each wheel
        omega_samples = []
        for i in range(4):
            x, y = positions[i]
            if abs(linear_vels[i]) > 0.001:
                # omega = (vy_wheel - vy_robot) / x - (vx_wheel - vx_robot) / y
                # Simplified: use velocity perpendicular to radius
                omega_i = (vx_wheels[i] - vx) / (-y) if abs(y) > 0.001 else 0.0
                omega_samples.append(omega_i)
        
        omega = np.mean(omega_samples) if omega_samples else 0.0
        
        return vx, vy, omega

    def publish_odometry(self):
        """Compute and publish odometry"""
        current_time = self.get_clock().now()
        
        # Initialize last_time on first call
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Get velocity from wheel encoders
        vx, vy, omega = self.compute_odometry_from_wheels()
        
        # Update pose using velocity (dead reckoning)
        # Transform velocity from robot frame to world frame
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = omega * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        
        # Covariance (rough estimates)
        odom_msg.pose.covariance[0] = 0.001  # x
        odom_msg.pose.covariance[7] = 0.001  # y
        odom_msg.pose.covariance[35] = 0.01  # theta
        
        odom_msg.twist.covariance[0] = 0.001  # vx
        odom_msg.twist.covariance[7] = 0.001  # vy
        odom_msg.twist.covariance[35] = 0.01  # omega
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(transform)
        
        self.last_time = current_time

    def twist_stamped_callback(self, msg):
        """Handle TwistStamped messages"""
        self.process_twist(msg.twist)

    def twist_callback(self, msg):
        """Handle Twist messages"""
        self.process_twist(msg)

    def process_twist(self, twist):
        """
        Convert twist command to 4WIS wheel velocities and steering angles
        Order: [front_left, front_right, rear_left, rear_right]
        """
        linear_x = twist.linear.x
        linear_y = twist.linear.y
        angular_z = twist.angular.z
        
        # Half dimensions
        L = self.wheelbase / 2.0  # half wheelbase
        W = self.track_width / 2.0  # half track width
        
        # Calculate velocities and steering angles for each wheel
        # Using 4WIS kinematics
        
        if abs(linear_x) < 0.001 and abs(linear_y) < 0.001 and abs(angular_z) < 0.001:
            # Robot stopped
            wheel_vels = [0.0, 0.0, 0.0, 0.0]
            steering_angles = [0.0, 0.0, 0.0, 0.0]
        else:
            # Calculate individual wheel velocities based on ICR (Instantaneous Center of Rotation)
            wheel_vels = []
            steering_angles = []
            
            # Wheel positions relative to center: [x, y]
            wheel_positions = [
                [L, W],    # front_left
                [L, -W],   # front_right
                [-L, W],   # rear_left
                [-L, -W]   # rear_right
            ]
            
            for x, y in wheel_positions:
                # Velocity components at wheel position
                # For a point at (x, y) rotating with angular velocity ω:
                # vx = vx_robot - ω * y
                # vy = vy_robot + ω * x
                vx = linear_x - angular_z * y
                vy = linear_y + angular_z * x
                
                # Wheel velocity magnitude
                v_mag = math.sqrt(vx**2 + vy**2)
                
                # Steering angle (direction of velocity vector)
                # atan2(vy, vx) gives the angle the wheel should point
                if abs(vx) > 0.001 or abs(vy) > 0.001:
                    steering_angle = math.atan2(vy, vx)
                else:
                    steering_angle = 0.0
                
                # Handle angles outside ±90° by flipping wheel direction
                # If angle > 90° or < -90°, flip it and reverse wheel velocity
                max_steer = math.pi / 2.0
                if steering_angle > max_steer:
                    steering_angle -= math.pi
                    v_mag = -v_mag
                elif steering_angle < -max_steer:
                    steering_angle += math.pi
                    v_mag = -v_mag
                
                # Convert linear velocity to angular velocity (rad/s)
                wheel_angular_vel = v_mag / self.wheel_radius
                
                wheel_vels.append(wheel_angular_vel)
                steering_angles.append(steering_angle)
        
        # Publish commands
        vel_msg = Float64MultiArray()
        vel_msg.data = wheel_vels
        self.wheel_vel_pub.publish(vel_msg)
        
        steer_msg = Float64MultiArray()
        steer_msg.data = steering_angles
        self.steering_pub.publish(steer_msg)
        
        # Debug logging (throttled)
        if abs(linear_x) > 0.001 or abs(linear_y) > 0.001 or abs(angular_z) > 0.001:
            self.get_logger().info(
                f'Cmd: vx={linear_x:.2f}, vy={linear_y:.2f}, wz={angular_z:.2f}')
            self.get_logger().info(
                f'Steering angles: FL={math.degrees(steering_angles[0]):.1f}°, '
                f'FR={math.degrees(steering_angles[1]):.1f}°, '
                f'RL={math.degrees(steering_angles[2]):.1f}°, '
                f'RR={math.degrees(steering_angles[3]):.1f}°')
            self.get_logger().info(
                f'Wheel velocities: FL={wheel_vels[0]:.2f}, FR={wheel_vels[1]:.2f}, '
                f'RL={wheel_vels[2]:.2f}, RR={wheel_vels[3]:.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelSteeringController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
