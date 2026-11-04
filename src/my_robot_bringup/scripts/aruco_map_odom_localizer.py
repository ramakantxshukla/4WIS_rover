#!/usr/bin/env python3
"""
ArUco Map-to-Odom AMCL Localizer Node
- Detects 4x4 ArUco markers from camera images.
- Uses static marker poses (from marker_map_tf_broadcaster.py / TF) to compute robot pose in map frame.
- Calculates and continuously broadcasts the 'map -> odom' transform (AMCL behavior).
- Preserves continuous localization even when markers leave camera view.
"""

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import tf2_ros
import tf_transformations


def make_transform_matrix(x, y, z, roll, pitch, yaw):
    T = tf_transformations.euler_matrix(roll, pitch, yaw)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


# ==============================================================================
# Static Marker Offsets (matching marker_map_tf_broadcaster.py)
# Format: (x, y, z, roll, pitch, yaw)
# - Marker 0 is relative to 'map'
# - Markers 1, 2, 3 are relative to 'aruco_0'
# ==============================================================================
MARKER_OFFSETS = {
    0: (1.0, 0.0, 0.0,  1.57,  0.00,  1.57),  # map -> aruco_0
    3: (5.0, 0.0, 0.0,  0.00,  0.00,  0.00),  # aruco_0 -> aruco_3
    2: (5.0, 0.0, 9.0, -3.14,  0.00,  0.00),  # aruco_0 -> aruco_2
    1: (0.0, 0.0, 9.0,  0.00,  3.14,  0.00),  # aruco_0 -> aruco_1
}

# Pre-calculate 4x4 transform matrices in 'map' frame (fallback if TF not yet available)
T_MAP_A0 = make_transform_matrix(*MARKER_OFFSETS[0])
DEFAULT_STATIC_MARKER_MAP = {
    0: T_MAP_A0,
    3: T_MAP_A0 @ make_transform_matrix(*MARKER_OFFSETS[3]),
    2: T_MAP_A0 @ make_transform_matrix(*MARKER_OFFSETS[2]),
    1: T_MAP_A0 @ make_transform_matrix(*MARKER_OFFSETS[1]),
}

# Coordinate conversion: OpenCV camera optical frame -> ROS standard link frame (REP 103)
# OpenCV: X=Right,   Y=Down, Z=Forward
# ROS:    X=Forward, Y=Left, Z=Up
R_OPT_TO_ROS = np.array([
    [ 0.0,  0.0,  1.0, 0.0],  # ROS X = +OpenCV Z (Forward)
    [-1.0,  0.0,  0.0, 0.0],  # ROS Y = -OpenCV X (Left)
    [ 0.0, -1.0,  0.0, 0.0],  # ROS Z = -OpenCV Y (Up)
    [ 0.0,  0.0,  0.0, 1.0]
])


class ArucoMapOdomLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_map_odom_localizer')

        # Parameters ('use_sim_time' is built-in and declared automatically by ROS 2)
        self.declare_parameter('marker_size', 0.5)
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detection_image_topic', '/camera/aruco_detections')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('planar_mode', True)

        self.marker_size = self.get_parameter('marker_size').value
        img_topic = self.get_parameter('camera_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        det_img_topic = self.get_parameter('detection_image_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        self.planar_mode = self.get_parameter('planar_mode').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_odom_matrix = None
        # Initialize map_to_odom to Identity so map -> odom is active from startup
        self.map_to_odom = np.eye(4)

        # ArUco 4x4 detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # TF Broadcaster and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.image_pub = self.create_publisher(Image, det_img_topic, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)

        # Subscriptions
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        # Continuous timer to broadcast map -> odom transform (standard AMCL behavior)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map_to_odom)

        self.get_logger().info(
            f'ArUco AMCL Localizer initialized: {self.map_frame} -> {self.odom_frame} -> {self.base_frame}'
        )

    def odom_callback(self, msg: Odometry):
        """Cache latest odometry matrix as fallback if TF lookup fails."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom_matrix = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix([p.x, p.y, p.z]),
            tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        )

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('[CameraInfo] Intrinsics loaded successfully.')

    @staticmethod
    def transform_to_matrix(t):
        trans = [t.translation.x, t.translation.y, t.translation.z]
        rot = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        return tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(trans),
            tf_transformations.quaternion_matrix(rot)
        )

    def get_marker_pose_in_map(self, marker_id):
        # 1. Lookup live TF tree (from marker_map_tf_broadcaster)
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                f'aruco_{marker_id}',
                rclpy.time.Time()
            )
            return self.transform_to_matrix(t.transform)
        except Exception:
            pass
        # 2. Fallback to predefined marker_map poses
        return DEFAULT_STATIC_MARKER_MAP.get(marker_id, None)

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

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated_image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            # 1. Base to Camera Transform (TF with rigid URDF mounting fallback)
            T_base_cam = None
            try:
                tf_base_cam = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    rclpy.time.Time()
                )
                T_base_cam = self.transform_to_matrix(tf_base_cam.transform)
            except Exception:
                # Chassis mounting offset: x=0.405m, y=0.0m, z=0.10m
                T_base_cam = make_transform_matrix(0.405, 0.0, 0.1, 0.0, 0.0, 0.0)

            # 2. Odom to Base Transform (TF with /odom topic fallback)
            T_odom_base = None
            try:
                tf_odom_base = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
                T_odom_base = self.transform_to_matrix(tf_odom_base.transform)
            except Exception:
                T_odom_base = self.latest_odom_matrix

            if T_odom_base is None:
                self.get_logger().warn(
                    'Waiting for odometry data on /odom topic or TF...',
                    throttle_duration_sec=3.0
                )
                return

            # Select closest detected marker with known map pose
            best_dist = float('inf')
            best_map_to_odom = None
            best_map_base = None
            best_id = None

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Draw 3D axes overlay on detected marker
                cv2.drawFrameAxes(
                    annotated_image, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, length=self.marker_size * 0.5, thickness=2
                )

                T_map_m = self.get_marker_pose_in_map(int(marker_id))
                if T_map_m is None:
                    continue

                # Marker in OpenCV camera optical frame
                R_mat, _ = cv2.Rodrigues(rvec)
                T_opt_m = np.eye(4)
                T_opt_m[:3, :3] = R_mat
                T_opt_m[:3, 3] = tvec

                # Convert to ROS camera link frame, then robot base frame
                T_cam_m = R_OPT_TO_ROS @ T_opt_m
                T_base_m = T_base_cam @ T_cam_m

                # Robot base in map: T_map_base = T_map_m * inv(T_base_m)
                T_map_base = T_map_m @ np.linalg.inv(T_base_m)

                # Enforce planar ground constraint (z=0, roll=0, pitch=0) for 2D mobile rovers
                if self.planar_mode:
                    x, y, _ = tf_transformations.translation_from_matrix(T_map_base)
                    _, _, yaw = tf_transformations.euler_from_matrix(T_map_base)
                    T_map_base = tf_transformations.translation_matrix([x, y, 0.0]) @ \
                                 tf_transformations.euler_matrix(0.0, 0.0, yaw)

                # AMCL correction: T_map_odom = T_map_base * inv(T_odom_base)
                T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

                dist = float(np.linalg.norm(tvec))
                if dist < best_dist:
                    best_dist = dist
                    best_map_to_odom = T_map_odom
                    best_map_base = T_map_base
                    best_id = int(marker_id)

            if best_map_to_odom is not None:
                self.map_to_odom = best_map_to_odom

                # Publish estimated robot pose with covariance on /amcl_pose
                pos = tf_transformations.translation_from_matrix(best_map_base)
                quat = tf_transformations.quaternion_from_matrix(best_map_base)

                p = PoseWithCovarianceStamped()
                p.header.stamp = msg.header.stamp
                p.header.frame_id = self.map_frame
                p.pose.pose.position.x = float(pos[0])
                p.pose.pose.position.y = float(pos[1])
                p.pose.pose.position.z = float(pos[2])
                p.pose.pose.orientation.x = float(quat[0])
                p.pose.pose.orientation.y = float(quat[1])
                p.pose.pose.orientation.z = float(quat[2])
                p.pose.pose.orientation.w = float(quat[3])
                cov = np.zeros(36, dtype=np.float64)
                cov[0] = 0.05
                cov[7] = 0.05
                cov[35] = 0.05
                p.pose.covariance = cov.tolist()
                self.pose_pub.publish(p)

                self.get_logger().info(
                    f'[AMCL] Corrected via ArUco ID={best_id} (dist={best_dist:.2f}m) | '
                    f'Robot in map: X={pos[0]:.2f}m, Y={pos[1]:.2f}m',
                    throttle_duration_sec=0.5
                )

        # Publish detection visualization image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

    def publish_map_to_odom(self):
        """Continuously broadcast map -> odom transform to keep the TF tree connected."""
        if self.map_to_odom is None:
            return

        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        trans = tf_transformations.translation_from_matrix(self.map_to_odom)
        rot = tf_transformations.quaternion_from_matrix(self.map_to_odom)

        t = TransformStamped()
        # Add 100ms tolerance like standard AMCL to prevent TF extrapolation errors
        t.header.stamp = (now + rclpy.duration.Duration(seconds=0.1)).to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])
        t.transform.rotation.x = float(rot[0])
        t.transform.rotation.y = float(rot[1])
        t.transform.rotation.z = float(rot[2])
        t.transform.rotation.w = float(rot[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMapOdomLocalizer()
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
