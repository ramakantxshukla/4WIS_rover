#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    """
    Subscribes to /cmd_vel as either Twist or TwistStamped
    and republishes to /four_wheel_steering_controller/reference as TwistStamped.
    """

    def __init__(self):
        super().__init__('cmd_vel_relay')

        # Subscribe to plain Twist (teleop_twist_keyboard, etc.)
        self.sub_twist = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)

        # Subscribe to TwistStamped (Nav2 with enable_stamped_cmd_vel: true)
        self.sub_stamped = self.create_subscription(
            TwistStamped, '/cmd_vel_stamped', self.stamped_callback, 10)

        self.pub = self.create_publisher(
            TwistStamped, '/four_wheel_steering_controller/reference', 10)

        self.get_logger().info('cmd_vel relay started')

    def twist_callback(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_footprint'
        out.twist = msg
        self.pub.publish(out)

    def stamped_callback(self, msg: TwistStamped):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
