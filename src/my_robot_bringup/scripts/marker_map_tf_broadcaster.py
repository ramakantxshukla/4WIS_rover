#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations


def make_tf(stamp, parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = str(parent)
    t.child_frame_id = str(child)
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    q = tf_transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


class MarkerMapTFBroadcaster(Node):
    def __init__(self):
        super().__init__('marker_map_tf_broadcaster')
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()

        # Filter out ROS arguments from sys.argv
        args = []
        skip_next = False
        for a in sys.argv[1:]:
            if skip_next:
                skip_next = False
                continue
            if a in ('--ros-args', '-r', '-p'):
                if a in ('-r', '-p'):
                    skip_next = True
                continue
            args.append(a)

        if len(args) >= 6:
            # CLI usage like static_transform_publisher: x y z roll pitch yaw [parent] [child]
            x, y, z, roll, pitch, yaw = [float(v) for v in args[:6]]
            parent = args[6] if len(args) > 6 else 'aruco_0'
            child = args[7] if len(args) > 7 else 'aruco_1'
            transforms = [make_tf(now, parent, child, x, y, z, roll, pitch, yaw)]
            self.get_logger().info(f'Static TF: {parent} -> {child} at ({x}, {y}, {z}) rpy=({roll}, {pitch}, {yaw})')
        else:
            # Default marker map relative to aruco_0 origin
            transforms = [
                #        stamp parent     child        x       y     z  roll pitch yaw
                make_tf(now, 'map',     'aruco_0',  0.01,  0.000, 0.0, 1.57, 0.0,  1.57),
                make_tf(now, 'aruco_0', 'aruco_3',  5.0, 0.0, 0.0, 0.0, 0.0,  0.00),
                make_tf(now, 'aruco_0', 'aruco_2',  5.0,  0.0, 9.0, -3.14, 0.0, 0.0),
                make_tf(now, 'aruco_0', 'aruco_1', 0.0,  0.0, 9.0, 0.0, 3.14,  0.00),
            ]
            self.get_logger().info('Static TF: map -> aruco_0 and aruco_1, aruco_2, aruco_3 published')

        self.broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = MarkerMapTFBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
