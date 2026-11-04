#!/usr/bin/env python3
"""
ZED ArUco Detector Node:
- Detects 4x4 ArUco markers using ZED compressed RGB & depth streams.
- Same architecture and logic as aruco_detector.py.
- Subscribes to ZED camera topics with SensorData QoS.
- Refines marker 3D distances with metric ZED depth stream if available.
- Transforms and broadcasts marker poses to /tf (relative to target_frame, e.g. base_link).
- Publishes annotated detection image (compressed).
"""

import sys
import struct
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations


class ZedArucoDetectorNode(Node):
    """
    ROS 2 Node for detecting 4x4 ArUco markers from ZED compressed camera streams
    and broadcasting their 3D transforms relative to the target frame (e.g. base_link).
    """

    def __init__(self):
        super().__init__('zed_aruco_detector_node')

        # ----------------------------------------------------------------------
        # 1. ROS 2 Parameters
        # ----------------------------------------------------------------------
        # Physical width/height of the ArUco marker square in meters
        self.declare_parameter('marker_size', 0.16)
        # Topics for ZED compressed RGB image, compressed depth, intrinsics, and detections
        self.declare_parameter('camera_image_topic', '/zed/zed_node/rgb/color/rect/image/compressed')
        self.declare_parameter('depth_image_topic', '/zed/zed_node/depth/depth_registered/compressedDepth')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/color/rect/camera_info')
        self.declare_parameter('detection_image_topic', '/zed/aruco_detections/compressed')
        # Target reference frame for TF (e.g. base_link)
        self.declare_parameter('target_frame', 'base_link')
        # Prefix for published child TF frames (e.g. 'aruco_0', 'aruco_1')
        self.declare_parameter('marker_prefix', 'aruco_')
        # Enable/disable depth refinement
        self.declare_parameter('use_depth', True)
        # Exponential Moving Average (EMA) alpha for translation filtering [0.0 - 1.0]
        self.declare_parameter('ema_alpha', 0.2)

        # Fetch parameter values
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)

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
            elif arg in ('--ema_alpha', '--ema-alpha') and i + 1 < len(sys.argv[1:]):
                try:
                    self.ema_alpha = float(sys.argv[1:][i + 1])
                except ValueError:
                    pass
            elif arg.startswith('--ema_alpha=') or arg.startswith('--ema-alpha='):
                try:
                    self.ema_alpha = float(arg.split('=')[1])
                except ValueError:
                    pass

        img_topic = self.get_parameter('camera_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        det_img_topic = self.get_parameter('detection_image_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.marker_prefix = self.get_parameter('marker_prefix').value
        self.use_depth = self.get_parameter('use_depth').value

        # Dynamic parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Camera intrinsics storage (filled once camera_info arrives)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Latest depth image buffer (float32 meters array)
        self.latest_depth = None
        self._depth_logged = False

        # ----------------------------------------------------------------------
        # 2. ArUco 4x4 Dictionary and Detection Parameters
        # ----------------------------------------------------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Sub-pixel corner refinement parameters (reduces micro-vibrations in rvec/tvec)
        self.subpix_win_size = (5, 5)
        self.subpix_zero_zone = (-1, -1)
        self.subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # EMA filter state for marker translations: {marker_id: np.ndarray([x, y, z])}
        self.filtered_translations = {}
        self.last_marker_time = {}

        # ----------------------------------------------------------------------
        # 3. TF Broadcaster & Transform Listener
        # ----------------------------------------------------------------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------------------------
        # 4. ROS Subscriptions (SensorData QoS for ZED compatibility)
        # ----------------------------------------------------------------------
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, img_topic, self.image_callback, qos_profile_sensor_data)

        # Publisher for annotated camera image (compressed)
        self.detection_image_pub = self.create_publisher(CompressedImage, det_img_topic, 10)

        self.get_logger().info(
            f'ZED ArUco Detector started:\n'
            f'  Marker Size      : {self.marker_size}m\n'
            f'  EMA Alpha        : {self.ema_alpha}\n'
            f'  RGB Topic        : {img_topic}\n'
            f'  Depth Topic      : {depth_topic}\n'
            f'  Camera Info Topic: {info_topic}\n'
            f'  Target Frame     : {self.target_frame}\n'
            f'  Output Image     : {det_img_topic}'
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
            elif param.name == 'ema_alpha':
                self.ema_alpha = float(param.value)
                self.get_logger().info(f'[Param] ema_alpha updated to: {self.ema_alpha}')
        return SetParametersResult(successful=True)

    def camera_info_callback(self, msg: CameraInfo):
        """Receives camera intrinsic matrix K and distortion coefficients D from ZED."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            self.get_logger().info(
                f'[CameraInfo] ZED Intrinsics loaded: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}'
            )

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

    def image_callback(self, msg: CompressedImage):
        """
        Main callback triggered on each incoming compressed RGB frame:
        1. Decode JPEG image to OpenCV BGR
        2. Detect 4x4 ArUco markers (corners and IDs)
        3. Estimate 3D pose (rvec, tvec) using solvePnP
        4. Validate/scale translation with ZED depth if available
        5. Transform coordinates from camera optical frame into target_frame (e.g. base_link)
        6. Broadcast TransformStamped over TF
        7. Publish annotated compressed image
        """
        if self.camera_matrix is None:
            self.get_logger().warn(
                '[Image] Waiting for camera_info intrinsics...',
                throttle_duration_sec=2.0
            )
            return

        # Decode Compressed JPEG to OpenCV BGR
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

        # Detect ArUco markers in grayscale image
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None or len(ids) == 0:
            self.publish_annotated_image(annotated_image, msg.header)
            return

        # Refine detected marker corners using cornerSubPix to reduce micro-vibrations
        for corner in corners:
            cv2.cornerSubPix(
                gray,
                corner,
                self.subpix_win_size,
                self.subpix_zero_zone,
                self.subpix_criteria
            )

        cv2.aruco.drawDetectedMarkers(annotated_image, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        camera_frame = msg.header.frame_id if msg.header.frame_id else 'zed_left_camera_frame_optical'

        # Lookup transform from target_frame to camera optical frame from the TF tree
        T_base_cam = None
        is_optical_tf = False
        try:
            tf_base_to_cam = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                rclpy.time.Time()
            )
            t_base_cam = tf_base_to_cam.transform.translation
            q_base_cam = tf_base_to_cam.transform.rotation

            T_base_cam = tf_transformations.concatenate_matrices(
                tf_transformations.translation_matrix([t_base_cam.x, t_base_cam.y, t_base_cam.z]),
                tf_transformations.quaternion_matrix([q_base_cam.x, q_base_cam.y, q_base_cam.z, q_base_cam.w])
            )
            if 'optical' in camera_frame.lower():
                is_optical_tf = True
        except Exception as ex:
            if self.target_frame == camera_frame:
                T_base_cam = np.eye(4)
                is_optical_tf = True
            else:
                # Fallback to standard chassis mounting offset: x=0.405m, y=0.0m, z=0.10m
                T_base_cam = tf_transformations.translation_matrix([0.405, 0.0, 0.1])
                is_optical_tf = False
                self.get_logger().warn(
                    f'[TF] Lookup failed {self.target_frame} -> {camera_frame}: {ex}. Using fallback mounting offset.',
                    throttle_duration_sec=3.0
                )

        # Coordinate frame conversion: OpenCV camera optical frame -> ROS standard link frame (REP 103)
        R_opt_to_link = np.array([
            [ 0.0,  0.0,  1.0, 0.0],
            [-1.0,  0.0,  0.0, 0.0],
            [ 0.0, -1.0,  0.0, 0.0],
            [ 0.0,  0.0,  0.0, 1.0]
        ])

        # Process each detected marker
        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0].copy()

            # Center of marker
            c = corners[i][0]
            u_center = int(round(np.mean(c[:, 0])))
            v_center = int(round(np.mean(c[:, 1])))

            # Depth refinement if depth stream is active
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

            # Draw 3D coordinate axes overlapping on the image
            cv2.drawFrameAxes(
                annotated_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                length=self.marker_size * 0.5,
                thickness=2
            )

            # Convert Rodrigues rotation vector to 3x3 rotation matrix
            R_mat, _ = cv2.Rodrigues(rvec)

            # Assemble 4x4 pose matrix of marker in camera optical frame
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = R_mat
            T_cam_marker[:3, 3] = tvec

            # If T_base_cam already transforms from the optical frame, do NOT rotate twice
            if is_optical_tf:
                T_base_marker = T_base_cam @ T_cam_marker
            else:
                T_link_marker = R_opt_to_link @ T_cam_marker
                T_base_marker = T_base_cam @ T_link_marker

            # Extract final 3D position [x, y, z] and quaternion [qx, qy, qz, qw]
            pos = tf_transformations.translation_from_matrix(T_base_marker)
            quat = tf_transformations.quaternion_from_matrix(T_base_marker)

            # Low-Pass / Exponential Moving Average (EMA) filter on translation for TF broadcasting:
            # T_filtered = (alpha * T_new) + ((1 - alpha) * T_filtered)
            pos_new = np.array(pos, dtype=np.float64)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if marker_id not in self.filtered_translations or (now_sec - self.last_marker_time.get(marker_id, 0.0)) > 1.0:
                self.filtered_translations[marker_id] = pos_new
            else:
                self.filtered_translations[marker_id] = (
                    self.ema_alpha * pos_new + (1.0 - self.ema_alpha) * self.filtered_translations[marker_id]
                )
            self.last_marker_time[marker_id] = now_sec
            pos_filtered = self.filtered_translations[marker_id]

            child_frame = f'{self.marker_prefix}{marker_id}'

            # Broadcast TF Transform
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.target_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = float(pos_filtered[0])
            t.transform.translation.y = float(pos_filtered[1])
            t.transform.translation.z = float(pos_filtered[2])
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            if not rclpy.ok():
                return
            try:
                self.tf_broadcaster.sendTransform(t)
            except Exception:
                pass

            dist_str = f'{measured_z:.2f}m (depth)' if measured_z is not None else f'{tvec[2]:.2f}m (solvePnP)'
            self.get_logger().info(
                f'[DETECTED] ID={marker_id} ({child_frame}) | Pixel: ({u_center}, {v_center}) | '
                f'Dist: {dist_str} | wrt {self.target_frame} -> X: {pos_filtered[0]:.3f}m, Y: {pos_filtered[1]:.3f}m, Z: {pos_filtered[2]:.3f}m',
                throttle_duration_sec=0.5
            )

        # Publish the detection image
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
                self.detection_image_pub.publish(out_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ZedArucoDetectorNode()
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
