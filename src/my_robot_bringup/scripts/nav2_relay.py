#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    """
    Subscribes to /cmd_vel_nav (Twist from Nav2)
    and republishes to /cmd_vel (Twist for the controller).
    """

    def __init__(self):
        super().__init__('cmd_vel_relay')

        # Subscribe to Nav2's output
        self.sub_twist = self.create_subscription(
            Twist, '/cmd_vel_nav', self.twist_callback, 10)

        # Publish to controller's input
        self.pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.get_logger().info('cmd_vel relay started: /cmd_vel_nav -> /cmd_vel')

    def twist_callback(self, msg: Twist):
        # Simply republish the message as-is
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
