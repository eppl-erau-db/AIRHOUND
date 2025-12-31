"""
AIRHOUND Perception Package.

Provides object detection for drone tracking with support for
multiple detector backends (YOLO, RF-DETR).

Modules
-------
detector_node : ROS2 perception node
yolo_detector : YOLO detection wrapper
rfdetr_detector : RF-DETR detection wrapper
postprocessor : Detection postprocessing utilities
mock_detector : Synthetic detector for simulation
"""

__all__ = [
    "yolo_detector",
    "rfdetr_detector",
    "postprocessor",
    "detector_node",
    "mock_detector",
]
