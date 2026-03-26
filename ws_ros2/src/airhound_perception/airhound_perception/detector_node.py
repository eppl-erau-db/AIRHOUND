"""
AIRHOUND Perception Node with Depth Integration.

Performs object detection on RGB images and optionally fuses depth information
from a RealSense D455 camera. Publishes Detection2DArray with depth embedded
in the pose.position.z field for downstream 3D tracking.

Supports direct V4L2 camera capture (camera_source='v4l2') to bypass
ROS2 DDS serialization overhead for large Image messages.

Author: Perception Lead
"""

import time
import threading
from typing import Optional, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time as RclTime

from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, PointStamped

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from .yolo_detector import YOLODetector


class PerceptionNode(Node):
    """
    ROS2 Perception Node for object detection with depth integration.

    Supports multiple detector backends:
    - yolo: YOLOv8 via Ultralytics (default)
    - rfdetr: RF-DETR transformer-based detector

    Depth Integration:
    - Subscribes to depth image from RealSense D455
    - Extracts depth at bbox center for each detection
    - Publishes depth in Detection2D.results[].pose.pose.position.z
    - Tracking team uses this for 3D position estimation

    Parameters
    ----------
    detector_type : str
        Type of detector to use: "yolo" or "rfdetr". Default: "yolo"
    model_path : str
        Path to model file (.pt, .engine, .onnx, .pth)
    conf : float
        Confidence threshold for detections
    device : str
        Device for inference ("0" for GPU, "cpu" for CPU)
    enable_depth : bool
        Enable depth integration. Default: True
    depth_image_topic : str
        Topic for depth images. Default: "/camera/depth/image_raw"
    depth_scale : float
        Scale factor for depth values (D455: 0.001 for mm->m). Default: 0.001
    """

    def __init__(self):
        super().__init__('perception_node')

        # ==================== PARAMETERS ====================
        # Image/detection params
        self.declare_parameter('input_image_topic', '/camera/color/image_raw')
        self.declare_parameter('output_detections_topic', '/detections')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        # Detector selection
        self.declare_parameter('detector_type', 'yolo')  # "yolo" or "rfdetr"
        self.declare_parameter('model_path', 'yolov8Detector.pt')

        # Common detector params
        self.declare_parameter('conf', 0.25)
        # Use dynamic_typing to accept both string '0' and YAML-parsed integer 0
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('device', '0',
                               ParameterDescriptor(dynamic_typing=True))

        # YOLO-specific params
        self.declare_parameter('imgsz', 1280)
        self.declare_parameter('iou', 0.45)

        # RF-DETR-specific params
        self.declare_parameter('rfdetr_input_size', 560)
        self.declare_parameter('rfdetr_variant', 'base')

        # Depth integration params
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('depth_scale', 0.001)  # D455: mm to meters
        self.declare_parameter('depth_window_size', 5)  # Window for median depth sampling
        
        # Depth quality filtering params (D455 valid range: 0.4m - 6.0m)
        self.declare_parameter('depth_min_range', 0.4)  # Minimum valid depth in meters
        self.declare_parameter('depth_max_range', 6.0)  # Maximum valid depth in meters

        # V4L2 direct capture params (bypasses ROS2 DDS serialization)
        self.declare_parameter('camera_source', 'topic')  # 'topic' or 'v4l2'
        self.declare_parameter('v4l2_device', '/dev/video2')
        from rcl_interfaces.msg import ParameterDescriptor as PD
        dyn = PD(dynamic_typing=True)
        self.declare_parameter('v4l2_width', 640, dyn)
        self.declare_parameter('v4l2_height', 480, dyn)
        self.declare_parameter('v4l2_fps', 15.0, dyn)

        # ==================== GET PARAMETERS ====================
        input_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        det_topic = self.get_parameter('output_detections_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        # Camera source
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value

        # Depth params
        self.enable_depth = self.get_parameter('enable_depth').get_parameter_value().bool_value
        depth_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.depth_window_size = self.get_parameter('depth_window_size').get_parameter_value().integer_value

        # Depth quality filtering ranges
        self.depth_min_range = self.get_parameter('depth_min_range').get_parameter_value().double_value
        self.depth_max_range = self.get_parameter('depth_max_range').get_parameter_value().double_value

        # ==================== QOS PROFILES ====================
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_default = QoSProfile(depth=10)

        # ==================== SUBSCRIBERS / V4L2 CAPTURE ====================
        self.latest_depth: Optional[np.ndarray] = None
        self.depth_stamp: Optional[RclTime] = None
        self._v4l2_cap = None
        self._v4l2_running = False

        if self.camera_source == 'v4l2':
            # Direct V4L2 capture — bypass ROS2 DDS image transport entirely
            self._setup_v4l2_capture(cam_info_topic, qos_default)
        else:
            # Standard ROS2 topic subscription
            if use_compressed:
                self.image_sub = self.create_subscription(
                    CompressedImage,
                    input_topic + '/compressed',
                    self.on_image_compressed,
                    qos_sensor
                )
            else:
                self.image_sub = self.create_subscription(
                    Image,
                    input_topic,
                    self.on_image,
                    qos_sensor
                )

            # Camera info subscriber
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                cam_info_topic,
                self.on_camera_info,
                qos_sensor
            )

        # Depth image subscriber (if enabled and not V4L2)
        if self.enable_depth and self.camera_source != 'v4l2':
            self.depth_sub = self.create_subscription(
                Image,
                depth_topic,
                self.on_depth_image,
                qos_sensor
            )
            self.get_logger().info(f"Depth integration ENABLED, subscribing to {depth_topic}")
            self.get_logger().info(f"Depth range filter: {self.depth_min_range:.2f}m - {self.depth_max_range:.2f}m")
        else:
            self.depth_sub = None
            if self.camera_source != 'v4l2':
                self.get_logger().info("Depth integration DISABLED")

        # ==================== PUBLISHERS ====================
        self.det_pub = self.create_publisher(Detection2DArray, det_topic, qos_default)
        self.target_3d_pub = self.create_publisher(PointStamped, '/perception/target_3d', qos_default)
        self.lat_pub = self.create_publisher(Float32, '/perception/latency_ms', qos_default)
        self.fps_pub = self.create_publisher(Float32, '/perception/fps', qos_default)
        self.depth_status_pub = self.create_publisher(Float32, '/perception/depth_available', qos_default)
        self.depth_quality_pub = self.create_publisher(Float32, '/perception/depth_quality', qos_default)

        # ==================== STATE ====================
        self.last_pub_time: Optional[float] = None
        self.last_image_stamp_ns: Optional[int] = None
        self.bridge = CvBridge()
        if self.camera_source != 'v4l2':
            self.cam_info: Optional[CameraInfo] = None
        self.depth_received_count = 0
        self.depth_valid_count = 0  # Count of detections with valid depth in last frame
        self.depth_total_count = 0  # Total detections in last frame

        # Load detector (must happen before V4L2 thread starts)
        self.detector = self._load_detector()

        # Start V4L2 capture thread now that detector is loaded
        if self.camera_source == 'v4l2' and self._v4l2_cap is not None:
            self._v4l2_running = True
            self._v4l2_thread = threading.Thread(target=self._v4l2_capture_loop, daemon=True)
            self._v4l2_thread.start()

        # FPS timer
        self.fps_timer = self.create_timer(1.0, self.publish_fps)

        self.get_logger().info(
            f"PerceptionNode started. Source: {self.camera_source}, Detections: {det_topic}"
        )

    # ==================== V4L2 DIRECT CAPTURE ====================

    def _setup_v4l2_capture(self, cam_info_topic: str, qos):
        """Open V4L2 camera and start threaded capture loop."""
        import sys

        device = self.get_parameter('v4l2_device').get_parameter_value().string_value
        width = int(self.get_parameter('v4l2_width').value)
        height = int(self.get_parameter('v4l2_height').value)
        fps = float(self.get_parameter('v4l2_fps').value)

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            try:
                cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
            except ValueError:
                pass
        if not cap.isOpened():
            self.get_logger().error(f'V4L2: Failed to open camera: {device}')
            sys.exit(1)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'V4L2 camera opened: {device} @ {actual_w}x{actual_h} {actual_fps:.1f}fps')

        self._v4l2_cap = cap

        # Build and publish static CameraInfo (D455 approximate intrinsics)
        self.cam_info = CameraInfo()
        self.cam_info.header.frame_id = self.frame_id
        self.cam_info.width = actual_w
        self.cam_info.height = actual_h
        self.cam_info.distortion_model = 'plumb_bob'
        fx = 383.0 * (actual_w / 640.0)
        fy = 383.0 * (actual_h / 480.0)
        cx = actual_w / 2.0
        cy = actual_h / 2.0
        self.cam_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.cam_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self._cam_info_pub = self.create_publisher(CameraInfo, cam_info_topic, qos)
        self._v4l2_frame_count = 0
        # Thread started later in __init__ after detector is loaded

    def _v4l2_capture_loop(self):
        """Capture frames from V4L2 and run detection directly — no DDS overhead."""
        while self._v4l2_running:
            ret, frame = self._v4l2_cap.read()
            if not ret:
                continue

            now = self.get_clock().now()

            # Publish CameraInfo
            self.cam_info.header.stamp = now.to_msg()
            self._cam_info_pub.publish(self.cam_info)

            # Run detection directly on the captured frame
            det_array, target_3d_msg = self._process_detections(frame, now)
            self.det_pub.publish(det_array)

            if target_3d_msg is not None:
                self.target_3d_pub.publish(target_3d_msg)

            # Latency = just inference time (no transport delay)
            end = self.get_clock().now()
            now_ns = now.seconds_nanoseconds()
            end_ns = end.seconds_nanoseconds()
            latency_ms = float(
                (end_ns[0] - now_ns[0]) * 1000.0 + (end_ns[1] - now_ns[1]) / 1e6
            )
            self.lat_pub.publish(Float32(data=latency_ms))

            # Depth not available via V4L2 (no depth stream)
            self.depth_status_pub.publish(Float32(data=0.0))
            self.depth_quality_pub.publish(Float32(data=0.0))

            self.last_pub_time = time.time()
            self._v4l2_frame_count += 1
            if self._v4l2_frame_count % 300 == 0:
                self.get_logger().info(f'V4L2 frame {self._v4l2_frame_count}')

    # ==================== DETECTOR LOADING ====================

    def _load_detector(self) -> Optional[Union[YOLODetector, "RFDETRDetector"]]:
        """Load the appropriate detector based on detector_type parameter."""
        detector_type = self.get_parameter('detector_type').get_parameter_value().string_value.lower()
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        conf = float(self.get_parameter('conf').get_parameter_value().double_value or 0.25)
        try:
            val = self.get_parameter('device').value
            device = str(val) if val is not None else '0'
        except Exception:
            device = '0'

        # Resolve model path if relative
        if not os.path.isabs(model_path) and not os.path.exists(model_path):
            try:
                share = get_package_share_directory('airhound_perception')
                candidate = os.path.join(share, 'models', model_path)
                if os.path.exists(candidate):
                    model_path = candidate
            except Exception:
                pass

        if detector_type == 'rfdetr':
            return self._load_rfdetr(model_path, conf, device)
        else:
            return self._load_yolo(model_path, conf, device)

    def _load_yolo(self, model_path: str, conf: float, device: str) -> Optional[YOLODetector]:
        """Load YOLOv8 detector."""
        imgsz = int(self.get_parameter('imgsz').get_parameter_value().integer_value or 1280)
        iou = float(self.get_parameter('iou').get_parameter_value().double_value or 0.45)

        try:
            detector = YOLODetector(
                model_path=model_path,
                imgsz=imgsz,
                conf=conf,
                iou=iou,
                device=device
            )
            self.get_logger().info(
                f"Loaded YOLO detector: {model_path} (imgsz={imgsz}, conf={conf})"
            )
            return detector
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO detector '{model_path}': {e}")
            return None

    def _load_rfdetr(self, model_path: str, conf: float, device: str) -> Optional["RFDETRDetector"]:
        """Load RF-DETR detector."""
        try:
            from .rfdetr_detector import RFDETRDetector
        except ImportError as e:
            self.get_logger().error(f"Failed to import RFDETRDetector: {e}")
            return None

        input_size = int(self.get_parameter('rfdetr_input_size').get_parameter_value().integer_value or 560)
        variant = self.get_parameter('rfdetr_variant').get_parameter_value().string_value or 'base'

        try:
            detector = RFDETRDetector(
                model_path=model_path,
                model_variant=variant,
                conf=conf,
                device=device,
                input_size=input_size,
                verbose=True,
            )
            self.get_logger().info(
                f"Loaded RF-DETR detector: {model_path} (input_size={input_size}, conf={conf})"
            )
            return detector
        except Exception as e:
            self.get_logger().error(f"Failed to load RF-DETR detector '{model_path}': {e}")
            return None

    # ==================== CALLBACKS ====================

    def on_camera_info(self, msg: CameraInfo):
        """Store camera intrinsics for potential 3D projection."""
        self.cam_info = msg

    def on_depth_image(self, msg: Image):
        """
        Store latest depth image for fusion with detections.
        
        D455 publishes 16UC1 depth images in millimeters.
        We store the raw image and apply depth_scale when extracting values.
        """
        try:
            # D455 depth is typically 16UC1 (16-bit unsigned, single channel)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_image
            self.depth_stamp = RclTime.from_msg(msg.header.stamp)
            self.depth_received_count += 1
            
            # Log occasionally
            if self.depth_received_count == 1:
                self.get_logger().info(
                    f"First depth frame received: {depth_image.shape}, dtype={depth_image.dtype}"
                )
            elif self.depth_received_count % 300 == 0:  # Every 10 seconds at 30fps
                self.get_logger().debug(f"Depth frames received: {self.depth_received_count}")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to process depth image: {e}")

    def _extract_depth_at_point(self, u: int, v: int) -> float:
        """
        Extract depth value at pixel (u, v) using median filtering.
        
        Parameters
        ----------
        u : int
            Horizontal pixel coordinate (column)
        v : int
            Vertical pixel coordinate (row)
            
        Returns
        -------
        float
            Depth in meters, or NaN if unavailable
        """
        if self.latest_depth is None:
            return float('nan')
        
        h, w = self.latest_depth.shape[:2]
        
        # Clamp to image bounds
        u = max(0, min(int(u), w - 1))
        v = max(0, min(int(v), h - 1))
        
        # Extract window around point for median filtering (robust to noise)
        half_win = self.depth_window_size // 2
        u_min = max(0, u - half_win)
        u_max = min(w, u + half_win + 1)
        v_min = max(0, v - half_win)
        v_max = min(h, v + half_win + 1)
        
        window = self.latest_depth[v_min:v_max, u_min:u_max]
        
        # Filter out zero/invalid depth values
        valid_depths = window[window > 0]
        
        if len(valid_depths) == 0:
            return float('nan')
        
        # Use median for robustness
        depth_raw = float(np.median(valid_depths))
        
        # Apply scale factor (D455: mm -> m)
        depth_meters = depth_raw * self.depth_scale
        
        # Filter by configured depth range (D455 default: 0.4m - 6.0m)
        if depth_meters < self.depth_min_range or depth_meters > self.depth_max_range:
            return float('nan')
        
        return depth_meters

    def _project_to_3d(self, u: float, v: float, depth_m: float) -> tuple:
        """
        Project 2D pixel + depth to 3D position in camera optical frame.
        
        Uses pinhole camera model:
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            Z = depth
        
        Parameters
        ----------
        u : float
            Horizontal pixel coordinate
        v : float
            Vertical pixel coordinate
        depth_m : float
            Depth in meters
            
        Returns
        -------
        tuple
            (x, y, z) in meters, camera optical frame (+X right, +Y down, +Z forward)
            Returns (nan, nan, nan) if intrinsics unavailable or depth invalid
        """
        if self.cam_info is None or np.isnan(depth_m):
            return (float('nan'), float('nan'), float('nan'))
        
        # Extract intrinsics from K matrix
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]
        
        if fx == 0 or fy == 0:
            return (float('nan'), float('nan'), float('nan'))
        
        # Project to 3D
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        
        return (x, y, z)

    def _process_detections(self, cv_image: np.ndarray, now) -> tuple:
        """
        Process image through detector and return Detection2DArray with depth.
        
        Depth is embedded in each detection's pose.pose.position.z field.
        Also computes 3D position for the highest-confidence detection.
        
        Returns
        -------
        tuple
            (Detection2DArray, PointStamped or None)
        """
        det_array = Detection2DArray()
        det_array.header.stamp = now.to_msg()
        det_array.header.frame_id = self.frame_id
        
        target_3d_msg = None  # Will hold 3D position of best detection

        if self.detector is None or cv_image is None:
            self.depth_valid_count = 0
            self.depth_total_count = 0
            return det_array, target_3d_msg

        detections = self.detector.infer(cv_image)
        
        # Track best detection for 3D publishing
        best_detection = None
        best_conf = 0.0
        best_3d = None
        
        # Track depth quality statistics
        valid_depth_count = 0
        total_depth_attempts = 0
        
        for d in detections:
            x1, y1, x2, y2 = d.xyxy
            
            # Compute bbox center
            center_u = (x1 + x2) / 2.0
            center_v = (y1 + y2) / 2.0
            
            # Create bounding box
            bbox = BoundingBox2D()
            bbox.center.position.x = center_u
            bbox.center.position.y = center_v
            bbox.size_x = max(0.0, x2 - x1)
            bbox.size_y = max(0.0, y2 - y1)

            # Create hypothesis with pose (depth in position.z)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.label
            hyp.hypothesis.score = d.conf
            
            # Extract and embed depth
            if self.enable_depth:
                depth_m = self._extract_depth_at_point(int(center_u), int(center_v))
                hyp.pose.pose.position.z = depth_m
                
                # Track depth quality
                total_depth_attempts += 1
                if not np.isnan(depth_m):
                    valid_depth_count += 1
                
                # Also store pixel coords in x, y for convenience
                # (Tracking team can use these with camera intrinsics for 3D)
                hyp.pose.pose.position.x = center_u
                hyp.pose.pose.position.y = center_v
                
                # Track best detection for 3D publishing
                if d.conf > best_conf and not np.isnan(depth_m):
                    best_conf = d.conf
                    best_3d = self._project_to_3d(center_u, center_v, depth_m)
            else:
                hyp.pose.pose.position.z = float('nan')

            det = Detection2D()
            det.bbox = bbox
            det.results.append(hyp)
            det_array.detections.append(det)
        
        # Create PointStamped for best detection
        if best_3d is not None and not np.isnan(best_3d[2]):
            target_3d_msg = PointStamped()
            target_3d_msg.header.stamp = now.to_msg()
            target_3d_msg.header.frame_id = self.frame_id
            target_3d_msg.point.x = best_3d[0]
            target_3d_msg.point.y = best_3d[1]
            target_3d_msg.point.z = best_3d[2]

        # Store depth quality stats for publishing
        self.depth_valid_count = valid_depth_count
        self.depth_total_count = total_depth_attempts

        return det_array, target_3d_msg

    def on_image(self, msg: Image):
        """Process uncompressed RGB image."""
        now = self.get_clock().now()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            cv_image = None

        det_array, target_3d_msg = self._process_detections(cv_image, now)
        self.det_pub.publish(det_array)
        
        # Publish 3D position if available
        if target_3d_msg is not None:
            self.target_3d_pub.publish(target_3d_msg)

        # Latency measurement
        image_stamp = msg.header.stamp
        img_time = image_stamp.sec + image_stamp.nanosec / 1e9
        now_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        latency_ms = float(max(0.0, (now_sec - img_time) * 1000.0))
        self.lat_pub.publish(Float32(data=latency_ms))

        # Depth status (1.0 if available, 0.0 if not)
        depth_available = 1.0 if self.latest_depth is not None else 0.0
        self.depth_status_pub.publish(Float32(data=depth_available))
        
        # Depth quality (ratio of valid depths in range, 0.0-1.0)
        if self.depth_total_count > 0:
            depth_quality = float(self.depth_valid_count) / float(self.depth_total_count)
        else:
            depth_quality = 0.0
        self.depth_quality_pub.publish(Float32(data=depth_quality))

        self.last_pub_time = time.time()

    def on_image_compressed(self, msg: CompressedImage):
        """Process compressed RGB image."""
        now = self.get_clock().now()

        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"compressed decode failed: {e}")
            cv_image = None

        det_array, target_3d_msg = self._process_detections(cv_image, now)
        self.det_pub.publish(det_array)
        
        # Publish 3D position if available
        if target_3d_msg is not None:
            self.target_3d_pub.publish(target_3d_msg)

        # Latency measurement
        image_stamp = msg.header.stamp
        img_time = image_stamp.sec + image_stamp.nanosec / 1e9
        now_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        latency_ms = float(max(0.0, (now_sec - img_time) * 1000.0))
        self.lat_pub.publish(Float32(data=latency_ms))

        # Depth status
        depth_available = 1.0 if self.latest_depth is not None else 0.0
        self.depth_status_pub.publish(Float32(data=depth_available))
        
        # Depth quality (ratio of valid depths in range, 0.0-1.0)
        if self.depth_total_count > 0:
            depth_quality = float(self.depth_valid_count) / float(self.depth_total_count)
        else:
            depth_quality = 0.0
        self.depth_quality_pub.publish(Float32(data=depth_quality))

        self.last_pub_time = time.time()

    def publish_fps(self):
        """Publish current FPS estimate."""
        if self.last_pub_time is None:
            return
        dt = max(1e-6, time.time() - self.last_pub_time)
        fps = float(1.0 / dt)
        self.fps_pub.publish(Float32(data=fps))

    def destroy_node(self):
        # Stop V4L2 capture thread before destroying node
        self._v4l2_running = False
        if hasattr(self, '_v4l2_thread') and self._v4l2_thread.is_alive():
            self._v4l2_thread.join(timeout=2.0)
        if self._v4l2_cap is not None:
            self._v4l2_cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
