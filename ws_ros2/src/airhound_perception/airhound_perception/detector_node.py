"""
Perception Node for AIRHOUND Object Detection.

Supports multiple detector backends (YOLO, RF-DETR) for modular comparison.
Publishes detections in vision_msgs/Detection2DArray format compatible with
the tracking pipeline.

This module enables switching between detector types via ROS2 parameters,
supporting the SPIE abstract comparison between YOLO and RF-DETR.

Author: EPPL - Embry-Riddle Aeronautical University
Date: 2025
"""

import os
import time
from typing import Optional, Protocol, List, Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


# =============================================================================
# Detector Protocol (Interface)
# =============================================================================


class DetectorProtocol(Protocol):
    """
    Protocol defining the detector interface.

    All detector implementations (YOLO, RF-DETR) must implement this interface
    to be compatible with the perception node.
    """

    names: dict

    def infer(self, image_bgr: np.ndarray) -> List[Any]:
        """
        Run inference on an image.

        Parameters
        ----------
        image_bgr : np.ndarray
            Input image in BGR format (OpenCV default).

        Returns
        -------
        List[Detection]
            List of Detection objects with xyxy, conf, cls, and label attributes.
        """
        ...


# =============================================================================
# Detector Factory
# =============================================================================


class DetectorFactory:
    """
    Factory for creating detector instances based on model type.

    Supports:
        - "yolo": Ultralytics YOLO detector
        - "rfdetr": RF-DETR transformer detector

    Examples
    --------
    >>> detector = DetectorFactory.create(
    ...     model_type="rfdetr",
    ...     model_path="models/drone_rfdetr.engine",
    ...     imgsz=560,
    ...     conf=0.25,
    ...     iou=0.45,
    ...     device="0",
    ... )
    """

    SUPPORTED_TYPES = ("yolo", "rfdetr")

    @staticmethod
    def create(
        model_type: str,
        model_path: str,
        imgsz: int = 1280,
        conf: float = 0.25,
        iou: float = 0.45,
        device: str = "0",
    ) -> DetectorProtocol:
        """
        Create a detector instance.

        Parameters
        ----------
        model_type : str
            Type of detector: "yolo" or "rfdetr".
        model_path : str
            Path to model file.
        imgsz : int
            Input image size.
        conf : float
            Confidence threshold.
        iou : float
            IoU threshold for NMS.
        device : str
            Device for inference.

        Returns
        -------
        DetectorProtocol
            Detector instance implementing the detector interface.

        Raises
        ------
        ValueError
            If model_type is not supported.
        ImportError
            If required dependencies are not installed.
        """
        model_type = model_type.lower().strip()

        if model_type == "yolo":
            from .yolo_detector import YOLODetector

            return YOLODetector(
                model_path=model_path,
                imgsz=imgsz,
                conf=conf,
                iou=iou,
                device=device,
            )

        elif model_type == "rfdetr":
            from .rfdetr_detector import RFDETRDetector

            return RFDETRDetector(
                model_path=model_path,
                imgsz=imgsz,
                conf=conf,
                iou=iou,
                device=device,
            )

        else:
            raise ValueError(
                f"Unknown model_type: '{model_type}'. "
                f"Supported types: {DetectorFactory.SUPPORTED_TYPES}"
            )

    @staticmethod
    def get_default_imgsz(model_type: str) -> int:
        """Get default input size for a model type."""
        defaults = {
            "yolo": 1280,
            "rfdetr": 560,
        }
        return defaults.get(model_type.lower(), 1280)


# =============================================================================
# Perception Node
# =============================================================================


class PerceptionNode(Node):
    """
    ROS2 node for object detection.

    Subscribes to camera images and publishes Detection2DArray messages.
    Supports multiple detector backends via the 'model_type' parameter.

    Parameters (ROS2)
    ------------------
    model_type : str
        Detector type: "yolo" or "rfdetr" (default: "yolo").
    model_path : str
        Path to model file (default: "yolov8Detector.pt").
    imgsz : int
        Input image size (default: auto-detected based on model_type).
    conf : float
        Confidence threshold (default: 0.25).
    iou : float
        IoU threshold for NMS (default: 0.45).
    device : str
        Device for inference: "0", "cuda", or "cpu" (default: "0").
    input_image_topic : str
        Topic for camera images (default: "/camera/color/image_raw").
    output_detections_topic : str
        Topic for detection output (default: "/detections").
    use_compressed : bool
        Subscribe to compressed images (default: True).

    Published Topics
    ----------------
    /detections : vision_msgs/Detection2DArray
        Object detection results.
    /perception/latency_ms : std_msgs/Float32
        End-to-end latency in milliseconds.
    /perception/fps : std_msgs/Float32
        Current inference FPS.
    /perception/model_type : std_msgs/String
        Active model type (for monitoring).
    """

    def __init__(self):
        super().__init__("perception_node")

        # =================================================================
        # Declare Parameters
        # =================================================================

        # Model parameters
        self.declare_parameter("model_type", "yolo")
        self.declare_parameter("model_path", "yolov8Detector.pt")
        self.declare_parameter("imgsz", 0)  # 0 = auto-detect
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("device", "0")

        # Topic parameters
        self.declare_parameter("input_image_topic", "/camera/color/image_raw")
        self.declare_parameter("output_detections_topic", "/detections")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("publish_rate_hz", 30.0)

        # =================================================================
        # Get Parameters
        # =================================================================

        model_type = (
            self.get_parameter("model_type").get_parameter_value().string_value
        )
        model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        imgsz = int(
            self.get_parameter("imgsz").get_parameter_value().integer_value
        )
        conf = float(
            self.get_parameter("conf").get_parameter_value().double_value
        )
        iou = float(
            self.get_parameter("iou").get_parameter_value().double_value
        )
        device = (
            self.get_parameter("device").get_parameter_value().string_value
        )

        input_topic = (
            self.get_parameter("input_image_topic").get_parameter_value().string_value
        )
        det_topic = (
            self.get_parameter("output_detections_topic")
            .get_parameter_value()
            .string_value
        )
        cam_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )
        self.publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )

        # Auto-detect imgsz if not specified
        if imgsz <= 0:
            imgsz = DetectorFactory.get_default_imgsz(model_type)
            self.get_logger().info(
                f"Auto-detected imgsz={imgsz} for model_type='{model_type}'"
            )

        # =================================================================
        # Resolve Model Path
        # =================================================================

        model_path = self._resolve_model_path(model_path)

        # =================================================================
        # Load Detector
        # =================================================================

        self.model_type = model_type
        self.detector: Optional[DetectorProtocol] = None

        try:
            self.detector = DetectorFactory.create(
                model_type=model_type,
                model_path=model_path,
                imgsz=imgsz,
                conf=conf,
                iou=iou,
                device=device,
            )
            self.get_logger().info(
                f"Loaded {model_type.upper()} detector: {model_path} "
                f"(imgsz={imgsz}, conf={conf}, iou={iou}, device={device})"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to load {model_type} detector '{model_path}': {e}"
            )
            self.detector = None

        # =================================================================
        # Setup Subscribers
        # =================================================================

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        if use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                input_topic + "/compressed",
                self.on_image_compressed,
                qos_sensor,
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                input_topic,
                self.on_image,
                qos_sensor,
            )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            cam_info_topic,
            self.on_camera_info,
            qos_sensor,
        )

        # =================================================================
        # Setup Publishers
        # =================================================================

        qos_default = QoSProfile(depth=10)

        self.det_pub = self.create_publisher(Detection2DArray, det_topic, qos_default)
        self.lat_pub = self.create_publisher(
            Float32, "/perception/latency_ms", qos_default
        )
        self.fps_pub = self.create_publisher(Float32, "/perception/fps", qos_default)

        # =================================================================
        # State Variables
        # =================================================================

        self.bridge = CvBridge()
        self.cam_info: Optional[CameraInfo] = None
        self.last_pub_time: Optional[float] = None
        self.frame_count = 0
        self.start_time = time.time()

        # FPS publishing timer
        self.fps_timer = self.create_timer(1.0, self.publish_fps)

        self.get_logger().info(
            f"PerceptionNode started with {model_type.upper()} detector. "
            f"Subscribing to {input_topic}, publishing to {det_topic}"
        )

    def _resolve_model_path(self, model_path: str) -> str:
        """Resolve relative model paths to package share directory."""
        if os.path.isabs(model_path) or os.path.exists(model_path):
            return model_path

        try:
            share = get_package_share_directory("airhound_perception")
            candidate = os.path.join(share, "models", model_path)
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass

        return model_path

    def on_camera_info(self, msg: CameraInfo):
        """Handle camera info messages."""
        self.cam_info = msg

    def _process_detections(self, cv_image: Optional[np.ndarray]) -> List[Detection2D]:
        """
        Run detection and convert to ROS2 messages.

        Parameters
        ----------
        cv_image : Optional[np.ndarray]
            Input image in BGR format, or None if conversion failed.

        Returns
        -------
        List[Detection2D]
            List of Detection2D messages.
        """
        if self.detector is None or cv_image is None:
            return []

        detections = self.detector.infer(cv_image)
        det_msgs = []

        for d in detections:
            x1, y1, x2, y2 = d.xyxy

            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = max(0.0, x2 - x1)
            bbox.size_y = max(0.0, y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.label
            hyp.hypothesis.score = d.conf

            det = Detection2D()
            det.bbox = bbox
            det.results.append(hyp)
            det_msgs.append(det)

        return det_msgs

    def _publish_detection_array(
        self, det_msgs: List[Detection2D], stamp: rclpy.time.Time
    ):
        """Publish detection array message."""
        det_array = Detection2DArray()
        det_array.header.stamp = stamp.to_msg()
        det_array.header.frame_id = self.frame_id
        det_array.detections = det_msgs

        self.det_pub.publish(det_array)

    def _publish_latency(self, image_stamp, now: rclpy.time.Time):
        """Publish end-to-end latency."""
        img_time = image_stamp.sec + image_stamp.nanosec / 1e9
        now_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        latency_ms = float(max(0.0, (now_sec - img_time) * 1000.0))
        self.lat_pub.publish(Float32(data=latency_ms))

    def on_image(self, msg: Image):
        """Handle raw image messages."""
        now = self.get_clock().now()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            cv_image = None

        det_msgs = self._process_detections(cv_image)
        self._publish_detection_array(det_msgs, now)
        self._publish_latency(msg.header.stamp, now)

        self.last_pub_time = time.time()
        self.frame_count += 1

    def on_image_compressed(self, msg: CompressedImage):
        """Handle compressed image messages."""
        now = self.get_clock().now()

        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"compressed decode failed: {e}")
            cv_image = None

        det_msgs = self._process_detections(cv_image)
        self._publish_detection_array(det_msgs, now)
        self._publish_latency(msg.header.stamp, now)

        self.last_pub_time = time.time()
        self.frame_count += 1

    def publish_fps(self):
        """Publish FPS statistics."""
        if self.last_pub_time is None:
            return

        # Calculate average FPS
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            avg_fps = self.frame_count / elapsed
            self.fps_pub.publish(Float32(data=float(avg_fps)))


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


if __name__ == "__main__":
    main()
