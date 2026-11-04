#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations


class ArucoDetectorNode(Node):
    """
    ROS 2 Node for detecting 4x4 ArUco markers and broadcasting their 3D transforms
    relative to the robot's base frame (base_link).
    """

    def __init__(self):
        super().__init__('aruco_detector_node')

        # ----------------------------------------------------------------------
        # 1. ROS 2 Parameters
        # ----------------------------------------------------------------------
        # Physical width/height of the ArUco marker square in meters
        self.declare_parameter('marker_size', 0.5)
        # Topics for RGB image, depth image, camera intrinsics, and detection image
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detection_image_topic', '/camera/aruco_detections')
        # Target reference frame for TF (e.g. base_link)
        self.declare_parameter('target_frame', 'base_link')
        # Prefix for published child TF frames (e.g. 'aruco_0', 'aruco_1')
        self.declare_parameter('marker_prefix', 'aruco_')

        # Fetch parameter values
        self.marker_size = self.get_parameter('marker_size').value
        img_topic = self.get_parameter('camera_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        det_img_topic = self.get_parameter('detection_image_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.marker_prefix = self.get_parameter('marker_prefix').value

        # Bridge for converting ROS Image messages to OpenCV numpy arrays
        self.bridge = CvBridge()

        # Camera intrinsics storage (filled once /camera/camera_info arrives)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Latest depth image buffer (float32 meters or uint16 mm)
        self.latest_depth = None
        self._depth_logged = False

        # ----------------------------------------------------------------------
        # 2. ArUco 4x4 Dictionary and Detection Parameters
        # ----------------------------------------------------------------------
        # Using 4x4 dictionary (50 markers: IDs 0 to 49)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ----------------------------------------------------------------------
        # 3. TF Broadcaster & Transform Listener
        # ----------------------------------------------------------------------
        # tf_broadcaster: publishes the marker transforms to /tf
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # tf_buffer & tf_listener: listens to existing TF tree (e.g. base_link -> camera_optical_frame)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------------------------
        # 4. ROS Subscriptions
        # ----------------------------------------------------------------------
        # Subscribe to camera info for intrinsic matrix (fx, fy, cx, cy)
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        # Subscribe to depth image for metric distance verification
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        # Subscribe to raw RGB camera stream for tag detection
        self.create_subscription(Image, img_topic, self.image_callback, 10)
        # Publisher for annotated camera image with detected ArUco markers and TF axes
        self.detection_image_pub = self.create_publisher(Image, det_img_topic, 10)

        self.get_logger().info(
            f'Aruco Detector started | Dict: 4x4 | Marker Size: {self.marker_size}m | Target Frame: {self.target_frame} | Output Image: {det_img_topic}'
        )

    def camera_info_callback(self, msg: CameraInfo):
        """
        Receives camera intrinsic matrix K and distortion coefficients D.
        Only needs to be parsed once as intrinsics are static during simulation.
        """
        if self.camera_matrix is None:
            # Reshape 9-element 1D array into 3x3 camera matrix
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            self.get_logger().info(
                f'[CameraInfo] Intrinsics loaded: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}'
            )

    def depth_callback(self, msg: Image):
        """
        Stores the most recent depth image to verify/refine marker distance.
        """
        try:
            # passthrough keeps original encoding (typically 32FC1 in Gazebo simulation)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if not self._depth_logged:
                self.get_logger().info(
                    f'[Depth] Depth image stream connected (encoding: {msg.encoding}, shape: {self.latest_depth.shape})'
                )
                self._depth_logged = True
        except Exception as e:
            self.get_logger().error(f'[Depth] Failed to convert depth image: {e}')

    def image_callback(self, msg: Image):
        """
        Main callback triggered on each incoming RGB camera frame:
        1. Convert image to grayscale OpenCV format
        2. Detect 4x4 ArUco markers (corners and IDs)
        3. Estimate 3D pose (rvec, tvec) using solvePnP
        4. Validate/scale translation with depth image
        5. Transform coordinates from camera optical frame into base_link
        6. Broadcast TransformStamped over TF
        """
        # Ensure camera intrinsics have been received first
        if self.camera_matrix is None:
            self.get_logger().warn(
                '[Image] Waiting for camera_info intrinsics...',
                throttle_duration_sec=2.0
            )
            return

        # Convert ROS Image message to OpenCV BGR image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[Image] Failed to convert image message: {e}')
            return

        # Prepare image copy for visual detection and TF overlay
        annotated_image = cv_image.copy()

        # Convert to grayscale for marker detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the grayscale image
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        # If no markers found in this frame, publish plain frame and exit early
        if ids is None or len(ids) == 0:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            out_msg.header = msg.header
            self.detection_image_pub.publish(out_msg)
            return

        # Draw detected marker borders and IDs on the image
        cv2.aruco.drawDetectedMarkers(annotated_image, corners, ids)

        # ----------------------------------------------------------------------
        # Estimate pose of each detected marker using solvePnP
        # Returns:
        #   rvecs: Rodrigues rotation vectors (marker -> camera optical frame)
        #   tvecs: 3D translation vectors [X, Y, Z] in meters (camera optical frame)
        # ----------------------------------------------------------------------
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        # Lookup transform from base_link to camera optical frame from the TF tree
        camera_frame = msg.header.frame_id if msg.header.frame_id else 'camera_optical_frame'
        try:
            tf_base_to_cam = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                rclpy.time.Time()  # latest available transform
            )
            t_base_cam = tf_base_to_cam.transform.translation
            q_base_cam = tf_base_to_cam.transform.rotation

            # Create 4x4 transformation matrix: T_base_cam
            T_base_cam = tf_transformations.concatenate_matrices(
                tf_transformations.translation_matrix([t_base_cam.x, t_base_cam.y, t_base_cam.z]),
                tf_transformations.quaternion_matrix([q_base_cam.x, q_base_cam.y, q_base_cam.z, q_base_cam.w])
            )
        except Exception as ex:
            self.get_logger().warn(
                f'[TF] Could not lookup {self.target_frame} -> {camera_frame}: {ex}',
                throttle_duration_sec=2.0
            )
            return

        # ----------------------------------------------------------------------
        # Coordinate frame conversion matrix:
        # OpenCV Camera Optical Frame:
        #   X: points Right on image
        #   Y: points Down on image
        #   Z: points Forward (optical axis into the scene)
        # Standard Robot Link Frame:
        #   X: points Forward
        #   Y: points Left
        #   Z: points Up
        # Mapping: X_link = Z_opt, Y_link = -X_opt, Z_link = -Y_opt
        # ----------------------------------------------------------------------
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

            # Calculate 2D pixel center of the marker in the image
            c = corners[i][0]
            u_center = int(round(np.mean(c[:, 0])))
            v_center = int(round(np.mean(c[:, 1])))

            # ------------------------------------------------------------------
            # Optional Depth Refinement:
            # If depth image is available, sample a small window around center
            # to verify and scale metric distance accurately.
            # ------------------------------------------------------------------
            measured_z = None
            if self.latest_depth is not None:
                h, w = self.latest_depth.shape[:2]
                if 0 <= v_center < h and 0 <= u_center < w:
                    patch = self.latest_depth[
                        max(0, v_center - 3):min(h, v_center + 4),
                        max(0, u_center - 3):min(w, u_center + 4)
                    ]
                    valid_depths = patch[np.isfinite(patch) & (patch > 0.05)]
                    if len(valid_depths) > 0:
                        measured_z = float(np.median(valid_depths))
                        # Scale tvec using measured depth from sensor
                        if tvec[2] > 0.01:
                            scale = measured_z / tvec[2]
                            tvec *= scale

            # Draw 3D coordinate axes (TF of the marker) overlapping on the image:
            # X: Red, Y: Green, Z: Blue
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

            # Step 1: Rotate from optical frame convention into link frame convention
            T_link_marker = R_opt_to_link @ T_cam_marker

            # Step 2: Multiply by base_link -> camera transform to get marker in base_link
            T_base_marker = T_base_cam @ T_link_marker

            # Extract final 3D position [x, y, z] and quaternion [qx, qy, qz, qw]
            pos = tf_transformations.translation_from_matrix(T_base_marker)
            quat = tf_transformations.quaternion_from_matrix(T_base_marker)

            # Child frame name (e.g. 'aruco_2')
            child_frame = f'{self.marker_prefix}{marker_id}'

            # ------------------------------------------------------------------
            # Broadcast TF Transform
            # ------------------------------------------------------------------
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.target_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            self.tf_broadcaster.sendTransform(t)

            # Debug log with 3D coordinates and distance (throttled per 0.5s)
            dist_str = f'{measured_z:.2f}m (depth)' if measured_z is not None else f'{tvec[2]:.2f}m (solvePnP)'
            self.get_logger().info(
                f'[DETECTED] ID={marker_id} ({child_frame}) | Pixel: ({u_center}, {v_center}) | '
                f'Dist: {dist_str} | wrt {self.target_frame} -> X: {pos[0]:.3f}m, Y: {pos[1]:.3f}m, Z: {pos[2]:.3f}m',
                throttle_duration_sec=0.5
            )

        # Publish the detection image with markers and TF axes overlapping it
        out_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        out_msg.header = msg.header
        self.detection_image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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
