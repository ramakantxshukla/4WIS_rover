#!/usr/bin/env python3
"""
ZED ArUco Map-to-Odom AMCL Localizer Node:
- Subscribes to ZED camera topics (compressed image & camera info) with SensorData QoS.
- Uses ZED metric depth sensor measurements to guarantee true physical distance.
- Subscribes to robot wheel odometry on /odom (NOT /zed/zed_node/odom).
- Uses static marker map from marker_map_tf_broadcaster.py.
- Accurately converts optical camera coordinates to base_footprint.
- Calculates and continuously broadcasts the 'map -> odom' transform (AMCL behavior).
- Preserves continuous localization even when markers leave camera view.
"""

import sys
import struct
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage, CameraInfo
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
# (Only needed if transform to an optical frame is NOT available in TF)
R_OPT_TO_ROS = np.array([
    [ 0.0,  0.0,  1.0, 0.0],  # ROS X = +OpenCV Z (Forward)
    [-1.0,  0.0,  0.0, 0.0],  # ROS Y = -OpenCV X (Left)
    [ 0.0, -1.0,  0.0, 0.0],  # ROS Z = -OpenCV Y (Up)
    [ 0.0,  0.0,  0.0, 1.0]
])


class ZedArucoLocalizer(Node):
    def __init__(self):
        super().__init__('zed_aruco_localizer')

        # Parameters ('use_sim_time' is declared automatically by ROS 2 Jazzy)
        self.declare_parameter('marker_size', 0.16)
        self.declare_parameter('camera_image_topic', '/zed/zed_node/rgb/color/rect/image/compressed')
        self.declare_parameter('depth_image_topic', '/zed/zed_node/depth/depth_registered/compressedDepth')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/color/rect/camera_info')
        self.declare_parameter('detection_image_topic', '/zed/aruco_detections/compressed')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'zed_left_camera_frame_optical')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('planar_mode', True)
        self.declare_parameter('use_depth', True)

        self.marker_size = float(self.get_parameter('marker_size').value)

        # Allow CLI override if passed directly (e.g. --marker_size 0.16 or 0.16)
        for i, arg in enumerate(sys.argv[1:]):
            if arg in ('--marker_size', '--marker-size') and i + 1 < len(sys.argv[1:]):
                try:
                    self.marker_size = float(sys.argv[1:][i + 1])
                except ValueError:
                    pass
            elif arg.startswith('--marker_size=') or arg.startswith('--marker-size='):
                try:
                    self.marker_size = float(arg.split('=')[1])
                except ValueError:
                    pass

        img_topic = self.get_parameter('camera_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        det_img_topic = self.get_parameter('detection_image_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        self.planar_mode = self.get_parameter('planar_mode').value
        self.use_depth = self.get_parameter('use_depth').value

        # Dynamic parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_depth = None
        self.latest_odom_matrix = None
        self._depth_logged = False
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
        self.image_pub = self.create_publisher(CompressedImage, det_img_topic, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)

        # Subscriptions
        # ZED camera topics publish with BEST_EFFORT reliability -> use qos_profile_sensor_data
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, img_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, depth_topic, self.depth_callback, qos_profile_sensor_data)

        # Odometry subscription (depth 10, compatible with robot controller /odom)
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT  # can accept both RELIABLE and BEST_EFFORT publishers
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, odom_qos)

        # Continuous timer to broadcast map -> odom transform (standard AMCL behavior)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map_to_odom)

        self.get_logger().info(
            f'ZED ArUco Localizer initialized:\n'
            f'  Marker Size       : {self.marker_size}m\n'
            f'  Camera Image Topic: {img_topic}\n'
            f'  Depth Topic       : {depth_topic}\n'
            f'  Camera Info Topic : {info_topic}\n'
            f'  Odom Topic        : {self.odom_topic}\n'
            f'  TF Chain          : {self.map_frame} -> {self.odom_frame} -> {self.base_frame}'
        )

    def parameter_callback(self, params):
        """Allow runtime dynamic parameter updates via ros2 param set."""
        for param in params:
            if param.name == 'marker_size':
                self.marker_size = float(param.value)
                self.get_logger().info(f'[Param] marker_size updated to: {self.marker_size}m')
            elif param.name == 'use_depth':
                self.use_depth = bool(param.value)
                self.get_logger().info(f'[Param] use_depth updated to: {self.use_depth}')
        return SetParametersResult(successful=True)

    def depth_callback(self, msg: CompressedImage):
        """Decode compressedDepth format (12-byte header + PNG) into meters array."""
        try:
            data = np.frombuffer(msg.data, np.uint8)
            if len(data) <= 12:
                return

            # Header: format(4B), depthQuantA(4B float), depthQuantB(4B float)
            png_bytes = data[12:]
            depth_img = cv2.imdecode(png_bytes, cv2.IMREAD_UNCHANGED)
            if depth_img is None:
                depth_img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
                if depth_img is None:
                    return

            if depth_img.dtype == np.uint16:
                depth_quant_a, depth_quant_b = struct.unpack('ff', data[4:12])
                if depth_quant_a > 0.0:
                    depth_f = depth_quant_a / (depth_img.astype(np.float32) - depth_quant_b)
                    depth_f[depth_img == 0] = np.nan
                    self.latest_depth = depth_f
                else:
                    self.latest_depth = depth_img.astype(np.float32) / 1000.0
            elif depth_img.dtype == np.float32:
                self.latest_depth = depth_img
            else:
                self.latest_depth = depth_img.astype(np.float32) / 1000.0

            if not self._depth_logged:
                self.get_logger().info(
                    f'[Depth] ZED compressed depth stream connected (shape: {self.latest_depth.shape})'
                )
                self._depth_logged = True
        except Exception as e:
            self.get_logger().warn(f'[Depth] Decode error: {e}', throttle_duration_sec=5.0)

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
            self.get_logger().info('[CameraInfo] Intrinsics loaded successfully from ZED.')

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

    def image_callback(self, msg: CompressedImage):
        if self.camera_matrix is None:
            return

        # Decode Compressed JPEG image into OpenCV BGR
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                return
        except Exception as e:
            self.get_logger().error(f'[Image] Decode error: {e}')
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

            # Determine camera link frame
            camera_frame = msg.header.frame_id if msg.header.frame_id else self.camera_frame

            # 1. Base to Camera Transform (TF with rigid URDF mounting fallback)
            T_base_cam = None
            is_optical_tf = False
            try:
                tf_base_cam = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    camera_frame,
                    rclpy.time.Time()
                )
                T_base_cam = self.transform_to_matrix(tf_base_cam.transform)
                if 'optical' in camera_frame.lower():
                    is_optical_tf = True
            except Exception:
                # Chassis mounting offset fallback (in ROS standard link coordinates): x=0.405m, y=0.0m, z=0.10m
                T_base_cam = make_transform_matrix(0.405, 0.0, 0.1, 0.0, 0.0, 0.0)
                is_optical_tf = False

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
                    f'Waiting for odometry data on {self.odom_topic} topic or TF ({self.odom_frame} -> {self.base_frame})...',
                    throttle_duration_sec=3.0
                )
                self.publish_annotated_image(annotated_image, msg.header)
                return

            # Select closest detected marker with known map pose
            best_dist = float('inf')
            best_map_to_odom = None
            best_map_base = None
            best_id = None

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0].copy()

                # Metric Depth Refinement from ZED Depth Stream
                c = corners[i][0]
                u_center = int(round(np.mean(c[:, 0])))
                v_center = int(round(np.mean(c[:, 1])))

                measured_z = None
                if self.use_depth and self.latest_depth is not None:
                    h, w = self.latest_depth.shape[:2]
                    if 0 <= v_center < h and 0 <= u_center < w:
                        patch = self.latest_depth[
                            max(0, v_center - 3):min(h, v_center + 4),
                            max(0, u_center - 3):min(w, u_center + 4)
                        ]
                        valid_depths = patch[np.isfinite(patch) & (patch > 0.05)]
                        if len(valid_depths) > 0:
                            measured_z = float(np.median(valid_depths))
                            if tvec[2] > 0.01:
                                scale = measured_z / tvec[2]
                                tvec *= scale

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

                # Convert to robot base frame
                # If T_base_cam is looked up directly to an optical frame, it ALREADY includes optical rotation
                if is_optical_tf:
                    T_base_m = T_base_cam @ T_opt_m
                else:
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

                dist_info = f'{best_dist:.2f}m (depth)' if measured_z is not None else f'{best_dist:.2f}m (solvePnP)'
                self.get_logger().info(
                    f'[AMCL] Corrected via ArUco ID={best_id} (dist={dist_info}) | '
                    f'Robot in map: X={pos[0]:.2f}m, Y={pos[1]:.2f}m',
                    throttle_duration_sec=0.5
                )

        # Publish detection visualization image
        self.publish_annotated_image(annotated_image, msg.header)

    def publish_annotated_image(self, img, header):
        if not rclpy.ok():
            return
        try:
            ret, encoded_img = cv2.imencode('.jpg', img)
            if ret:
                out_msg = CompressedImage()
                out_msg.header = header
                out_msg.format = 'jpeg'
                out_msg.data = encoded_img.tobytes()
                self.image_pub.publish(out_msg)
        except Exception:
            pass

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
    node = ZedArucoLocalizer()
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
