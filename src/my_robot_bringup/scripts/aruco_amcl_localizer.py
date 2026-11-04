#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations


class ArucoAmclLocalizer(Node):
    """
    ArUco Detector Node:
    - Detects 4x4 ArUco markers.
    - Overlays marker boundary and 3D TF axes (RGB: Red=X, Green=Y, Blue=Z) on the image.
    - Publishes the annotated image on /camera/aruco_detections.
    - Broadcasts TF frames (camera_optical_frame -> aruco_<id>).
    - Logs detected marker poses: X, Y, Z, Roll, Pitch, Yaw.
    """

    def __init__(self):
        super().__init__('aruco_amcl_localizer')

        # Parameters
        self.declare_parameter('marker_size', 0.5)
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detection_image_topic', '/camera/aruco_detections')

        self.marker_size = self.get_parameter('marker_size').value
        img_topic = self.get_parameter('camera_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        det_img_topic = self.get_parameter('detection_image_topic').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco 4x4 Dictionary and Parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publisher for annotated image with ArUco TF overlay
        self.image_pub = self.create_publisher(Image, det_img_topic, 10)

        # Subscriptions
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, img_topic, self.image_callback, 10)

        self.get_logger().info(
            f'Simple ArUco Detector started | Marker size: {self.marker_size}m | Output image: {det_img_topic}'
        )

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('[CameraInfo] Intrinsics loaded successfully.')

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[Image] Conversion error: {e}')
            return

        annotated_image = cv_image.copy()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            # Draw marker boundary and ID
            cv2.aruco.drawDetectedMarkers(annotated_image, corners, ids)

            # Estimate 3D pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            # Coordinate frame conversion: OpenCV Optical Frame -> ROS Link Frame (REP 103)
            # OpenCV: X = Right,   Y = Down, Z = Forward
            # ROS:    X = Forward, Y = Left, Z = Up
            # Mapping: X_ros = Z_opt, Y_ros = -X_opt, Z_ros = -Y_opt
            R_opt_to_ros = np.array([
                [ 0.0,  0.0,  1.0, 0.0],
                [-1.0,  0.0,  0.0, 0.0],
                [ 0.0, -1.0,  0.0, 0.0],
                [ 0.0,  0.0,  0.0, 1.0]
            ])

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Draw 3D TF frame axes overlapping the marker on image (OpenCV expects optical frame)
                cv2.drawFrameAxes(
                    annotated_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    length=self.marker_size * 0.5,
                    thickness=2
                )

                # Convert pose from OpenCV optical frame to ROS standard link frame (REP 103)
                R_mat, _ = cv2.Rodrigues(rvec)
                T_opt_m = np.eye(4)
                T_opt_m[:3, :3] = R_mat
                T_opt_m[:3, 3] = tvec

                T_ros_m = R_opt_to_ros @ T_opt_m

                pos = tf_transformations.translation_from_matrix(T_ros_m)
                quat = tf_transformations.quaternion_from_matrix(T_ros_m)
                roll, pitch, yaw = tf_transformations.euler_from_matrix(T_ros_m, axes='sxyz')

                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)

                # Broadcast TF transform in ROS link frame (camera_link)
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = 'camera_link'
                t.child_frame_id = f'aruco_{marker_id}'
                t.transform.translation.x = float(pos[0])
                t.transform.translation.y = float(pos[1])
                t.transform.translation.z = float(pos[2])
                t.transform.rotation.x = float(quat[0])
                t.transform.rotation.y = float(quat[1])
                t.transform.rotation.z = float(quat[2])
                t.transform.rotation.w = float(quat[3])
                self.tf_broadcaster.sendTransform(t)

                # Log detected marker pose in ROS frame (X=Forward, Y=Left, Z=Up)
                self.get_logger().info(
                    f'[ARUCO ID={marker_id}] (camera_link) '
                    f'X={pos[0]:.3f}m, Y={pos[1]:.3f}m, Z={pos[2]:.3f}m | '
                    f'Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°',
                    throttle_duration_sec=0.2
                )

        # Publish the detection image with TF overlay
        out_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoAmclLocalizer()
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
