import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from .yolo_detector import YOLODetector
##hello

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/camera/color/image_raw')
        self.declare_parameter('output_detections_topic', '/detections')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('model_path', 'yolov8Detector.pt')
        self.declare_parameter('imgsz', 1280)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('device', '0')

        input_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        det_topic = self.get_parameter('output_detections_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        # Subscribers
        if use_compressed:
            # Subscribe to compressed image to be bandwidth-aware
            self.image_sub = self.create_subscription(CompressedImage, input_topic + '/compressed', self.on_image_compressed, qos_sensor)
        else:
            self.image_sub = self.create_subscription(Image, input_topic, self.on_image, qos_sensor)
        self.camera_info_sub = self.create_subscription(CameraInfo, cam_info_topic, self.on_camera_info, qos_sensor)

        qos_default = QoSProfile(depth=10)
        self.det_pub = self.create_publisher(Detection2DArray, det_topic, qos_default)
        self.lat_pub = self.create_publisher(Float32, '/perception/latency_ms', qos_default)
        self.fps_pub = self.create_publisher(Float32, '/perception/fps', qos_default)

        self.last_pub_time: Optional[float] = None
        self.last_image_stamp_ns: Optional[int] = None
        self.bridge = CvBridge()
        self.cam_info: Optional[CameraInfo] = None

        # Load detector
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        imgsz = int(self.get_parameter('imgsz').get_parameter_value().integer_value or 1280)
        conf = float(self.get_parameter('conf').get_parameter_value().double_value or 0.25)
        iou = float(self.get_parameter('iou').get_parameter_value().double_value or 0.45)
        device = self.get_parameter('device').get_parameter_value().string_value or '0'
        # Resolve model path if relative
        if not os.path.isabs(model_path) and not os.path.exists(model_path):
            try:
                share = get_package_share_directory('airhound_perception')
                candidate = os.path.join(share, 'models', model_path)
                if os.path.exists(candidate):
                    model_path = candidate
            except Exception:
                pass
        try:
            self.detector = YOLODetector(model_path=model_path, imgsz=imgsz, conf=conf, iou=iou, device=device)
            self.get_logger().info(f"Loaded detector: {model_path} (imgsz={imgsz}, conf={conf}, iou={iou}, device={device})")
        except Exception as e:
            self.get_logger().error(f"Failed to load detector '{model_path}': {e}")
            self.detector = None

        # Timer to publish FPS periodically
        self.fps_timer = self.create_timer(1.0, self.publish_fps)

        self.get_logger().info(
            f"PerceptionNode started. Subscribing to {input_topic}, publishing detections on {det_topic}"
        )

    def on_camera_info(self, msg: CameraInfo):
        self.cam_info = msg

    def on_image(self, msg: Image):
        now = self.get_clock().now()
        det_array = Detection2DArray()
        det_array.header.stamp = now.to_msg()
        det_array.header.frame_id = self.frame_id

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            cv_image = None

        if self.detector is not None and cv_image is not None:
            detections = self.detector.infer(cv_image)
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
                det_array.detections.append(det)

        self.det_pub.publish(det_array)

        # Latency measurement (image stamp to detection publish now)
        self.last_image_stamp_ns = getattr(msg.header.stamp, 'nanosec', None)
        image_stamp = msg.header.stamp
        img_time = image_stamp.sec + image_stamp.nanosec / 1e9
        now_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        latency_ms = float(max(0.0, (now_sec - img_time) * 1000.0))
        self.lat_pub.publish(Float32(data=latency_ms))

        # Remember publish time for FPS
        self.last_pub_time = time.time()

    def on_image_compressed(self, msg: CompressedImage):
        now = self.get_clock().now()
        det_array = Detection2DArray()
        det_array.header.stamp = now.to_msg()
        det_array.header.frame_id = self.frame_id

        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"compressed decode failed: {e}")
            cv_image = None

        if self.detector is not None and cv_image is not None:
            detections = self.detector.infer(cv_image)
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
                det_array.detections.append(det)

        self.det_pub.publish(det_array)

        # Latency measurement (image stamp to detection publish now)
        image_stamp = msg.header.stamp
        img_time = image_stamp.sec + image_stamp.nanosec / 1e9
        now_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        latency_ms = float(max(0.0, (now_sec - img_time) * 1000.0))
        self.lat_pub.publish(Float32(data=latency_ms))

        self.last_pub_time = time.time()

    def publish_fps(self):
        if self.last_pub_time is None:
            return
        # Simple instantaneous FPS since last publish
        dt = max(1e-6, time.time() - self.last_pub_time)
        fps = float(1.0 / dt)
        self.fps_pub.publish(Float32(data=fps))


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
