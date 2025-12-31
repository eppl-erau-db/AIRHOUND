"""
Detection Postprocessing Module.

Provides unified postprocessing for different detector backends (YOLO, RF-DETR).
Handles coordinate transformations, confidence filtering, and NMS.

This module enables modular detector comparison for the SPIE abstract by
providing a common interface for all detector outputs.

Author: EPPL - Embry-Riddle Aeronautical University
Date: 2025
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

import numpy as np


class BoxFormat(Enum):
    """Bounding box coordinate formats."""

    XYXY = auto()  # (x1, y1, x2, y2) - absolute pixels
    XYWH = auto()  # (x, y, w, h) - top-left corner + size, absolute pixels
    CXCYWH = auto()  # (cx, cy, w, h) - center + size, absolute pixels
    CXCYWH_NORM = auto()  # (cx, cy, w, h) - center + size, normalized [0,1]


@dataclass(frozen=True)
class Detection:
    """
    Unified detection result.

    Parameters
    ----------
    xyxy : Tuple[float, float, float, float]
        Bounding box in (x1, y1, x2, y2) absolute pixel format.
    conf : float
        Detection confidence score in [0, 1].
    cls : int
        Class index.
    label : str
        Class label string.

    Notes
    -----
    All detections are stored in xyxy format for consistency with
    the existing AIRHOUND pipeline and vision_msgs/Detection2D.
    """

    xyxy: Tuple[float, float, float, float]
    conf: float
    cls: int
    label: str

    @property
    def center(self) -> Tuple[float, float]:
        """Get bounding box center (cx, cy) in pixels."""
        x1, y1, x2, y2 = self.xyxy
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @property
    def size(self) -> Tuple[float, float]:
        """Get bounding box size (width, height) in pixels."""
        x1, y1, x2, y2 = self.xyxy
        return (x2 - x1, y2 - y1)

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        x1, y1, x2, y2 = self.xyxy
        return {
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "conf": self.conf,
            "cls": self.cls,
            "label": self.label,
        }


def convert_boxes(
    boxes: np.ndarray,
    from_format: BoxFormat,
    to_format: BoxFormat,
    image_width: Optional[int] = None,
    image_height: Optional[int] = None,
) -> np.ndarray:
    """
    Convert bounding boxes between coordinate formats.

    Parameters
    ----------
    boxes : np.ndarray
        Bounding boxes with shape (N, 4).
    from_format : BoxFormat
        Input coordinate format.
    to_format : BoxFormat
        Output coordinate format.
    image_width : Optional[int]
        Image width in pixels (required for normalized formats).
    image_height : Optional[int]
        Image height in pixels (required for normalized formats).

    Returns
    -------
    np.ndarray
        Converted bounding boxes with shape (N, 4).

    Raises
    ------
    ValueError
        If image dimensions are required but not provided.

    Examples
    --------
    >>> boxes = np.array([[0.5, 0.5, 0.2, 0.1]])  # cxcywh normalized
    >>> xyxy = convert_boxes(
    ...     boxes,
    ...     BoxFormat.CXCYWH_NORM,
    ...     BoxFormat.XYXY,
    ...     image_width=1280,
    ...     image_height=720,
    ... )
    >>> print(xyxy)  # [[512, 324, 768, 396]]
    """
    if boxes.size == 0:
        return boxes.copy()

    # First convert to XYXY (our canonical format)
    if from_format == BoxFormat.XYXY:
        xyxy = boxes.copy()

    elif from_format == BoxFormat.XYWH:
        # (x, y, w, h) -> (x1, y1, x2, y2)
        xyxy = np.zeros_like(boxes)
        xyxy[:, 0] = boxes[:, 0]  # x1 = x
        xyxy[:, 1] = boxes[:, 1]  # y1 = y
        xyxy[:, 2] = boxes[:, 0] + boxes[:, 2]  # x2 = x + w
        xyxy[:, 3] = boxes[:, 1] + boxes[:, 3]  # y2 = y + h

    elif from_format == BoxFormat.CXCYWH:
        # (cx, cy, w, h) -> (x1, y1, x2, y2)
        xyxy = np.zeros_like(boxes)
        xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2  # x1 = cx - w/2
        xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2  # y1 = cy - h/2
        xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2  # x2 = cx + w/2
        xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2  # y2 = cy + h/2

    elif from_format == BoxFormat.CXCYWH_NORM:
        if image_width is None or image_height is None:
            raise ValueError(
                "image_width and image_height required for CXCYWH_NORM conversion"
            )
        # (cx, cy, w, h) normalized -> (x1, y1, x2, y2) pixels
        xyxy = np.zeros_like(boxes)
        xyxy[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2) * image_width
        xyxy[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2) * image_height
        xyxy[:, 2] = (boxes[:, 0] + boxes[:, 2] / 2) * image_width
        xyxy[:, 3] = (boxes[:, 1] + boxes[:, 3] / 2) * image_height

    else:
        raise ValueError(f"Unknown from_format: {from_format}")

    # Now convert from XYXY to target format
    if to_format == BoxFormat.XYXY:
        return xyxy

    elif to_format == BoxFormat.XYWH:
        result = np.zeros_like(xyxy)
        result[:, 0] = xyxy[:, 0]  # x = x1
        result[:, 1] = xyxy[:, 1]  # y = y1
        result[:, 2] = xyxy[:, 2] - xyxy[:, 0]  # w = x2 - x1
        result[:, 3] = xyxy[:, 3] - xyxy[:, 1]  # h = y2 - y1
        return result

    elif to_format == BoxFormat.CXCYWH:
        result = np.zeros_like(xyxy)
        result[:, 0] = (xyxy[:, 0] + xyxy[:, 2]) / 2  # cx = (x1 + x2) / 2
        result[:, 1] = (xyxy[:, 1] + xyxy[:, 3]) / 2  # cy = (y1 + y2) / 2
        result[:, 2] = xyxy[:, 2] - xyxy[:, 0]  # w = x2 - x1
        result[:, 3] = xyxy[:, 3] - xyxy[:, 1]  # h = y2 - y1
        return result

    elif to_format == BoxFormat.CXCYWH_NORM:
        if image_width is None or image_height is None:
            raise ValueError(
                "image_width and image_height required for CXCYWH_NORM conversion"
            )
        result = np.zeros_like(xyxy)
        result[:, 0] = (xyxy[:, 0] + xyxy[:, 2]) / 2 / image_width
        result[:, 1] = (xyxy[:, 1] + xyxy[:, 3]) / 2 / image_height
        result[:, 2] = (xyxy[:, 2] - xyxy[:, 0]) / image_width
        result[:, 3] = (xyxy[:, 3] - xyxy[:, 1]) / image_height
        return result

    else:
        raise ValueError(f"Unknown to_format: {to_format}")


def scale_boxes(
    boxes: np.ndarray,
    from_shape: Tuple[int, int],
    to_shape: Tuple[int, int],
) -> np.ndarray:
    """
    Scale bounding boxes from one image size to another.

    Parameters
    ----------
    boxes : np.ndarray
        Bounding boxes in xyxy format with shape (N, 4).
    from_shape : Tuple[int, int]
        Source image (height, width).
    to_shape : Tuple[int, int]
        Target image (height, width).

    Returns
    -------
    np.ndarray
        Scaled bounding boxes with shape (N, 4).

    Examples
    --------
    >>> boxes = np.array([[280, 280, 280, 280]])  # center of 560x560
    >>> scaled = scale_boxes(boxes, (560, 560), (720, 1280))
    >>> # Boxes are scaled to match 1280x720 image
    """
    if boxes.size == 0:
        return boxes.copy()

    from_h, from_w = from_shape
    to_h, to_w = to_shape

    scale_x = to_w / from_w
    scale_y = to_h / from_h

    scaled = boxes.copy()
    scaled[:, [0, 2]] *= scale_x  # x coordinates
    scaled[:, [1, 3]] *= scale_y  # y coordinates

    return scaled


def clip_boxes(
    boxes: np.ndarray,
    image_width: int,
    image_height: int,
) -> np.ndarray:
    """
    Clip bounding boxes to image boundaries.

    Parameters
    ----------
    boxes : np.ndarray
        Bounding boxes in xyxy format with shape (N, 4).
    image_width : int
        Image width in pixels.
    image_height : int
        Image height in pixels.

    Returns
    -------
    np.ndarray
        Clipped bounding boxes with shape (N, 4).
    """
    if boxes.size == 0:
        return boxes.copy()

    clipped = boxes.copy()
    clipped[:, [0, 2]] = np.clip(clipped[:, [0, 2]], 0, image_width)
    clipped[:, [1, 3]] = np.clip(clipped[:, [1, 3]], 0, image_height)

    return clipped


def compute_iou(box1: np.ndarray, box2: np.ndarray) -> float:
    """
    Compute IoU between two boxes in xyxy format.

    Parameters
    ----------
    box1 : np.ndarray
        First box (4,) in xyxy format.
    box2 : np.ndarray
        Second box (4,) in xyxy format.

    Returns
    -------
    float
        Intersection over Union score.
    """
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union_area = box1_area + box2_area - inter_area

    if union_area <= 0:
        return 0.0

    return inter_area / union_area


def nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    iou_threshold: float = 0.45,
) -> np.ndarray:
    """
    Non-Maximum Suppression.

    Parameters
    ----------
    boxes : np.ndarray
        Bounding boxes in xyxy format with shape (N, 4).
    scores : np.ndarray
        Confidence scores with shape (N,).
    iou_threshold : float
        IoU threshold for suppression.

    Returns
    -------
    np.ndarray
        Indices of boxes to keep.

    Notes
    -----
    This is a simple Python NMS implementation. For production on Jetson,
    consider using TensorRT's NMS plugin or torchvision.ops.nms.
    """
    if len(boxes) == 0:
        return np.array([], dtype=np.int64)

    # Sort by confidence (descending)
    order = scores.argsort()[::-1]
    keep = []

    while len(order) > 0:
        i = order[0]
        keep.append(i)

        if len(order) == 1:
            break

        # Compute IoU with remaining boxes
        remaining = order[1:]
        ious = np.array([compute_iou(boxes[i], boxes[j]) for j in remaining])

        # Keep boxes with IoU below threshold
        mask = ious < iou_threshold
        order = remaining[mask]

    return np.array(keep, dtype=np.int64)


def postprocess_rfdetr(
    dets: np.ndarray,
    labels: np.ndarray,
    orig_width: int,
    orig_height: int,
    model_input_size: int = 560,
    conf_threshold: float = 0.25,
    iou_threshold: float = 0.45,
    class_names: Optional[List[str]] = None,
) -> List[Detection]:
    """
    Postprocess RF-DETR model outputs.

    Converts RF-DETR's normalized cxcywh boxes and logits to Detection objects
    with absolute pixel coordinates scaled to the original image size.

    Parameters
    ----------
    dets : np.ndarray
        RF-DETR bounding box outputs with shape (1, 300, 4).
        Format: cxcywh normalized to [0, 1].
    labels : np.ndarray
        RF-DETR class logits with shape (1, 300, num_classes).
    orig_width : int
        Original image width in pixels.
    orig_height : int
        Original image height in pixels.
    model_input_size : int
        RF-DETR model input size (default 560).
    conf_threshold : float
        Confidence threshold for filtering detections.
    iou_threshold : float
        IoU threshold for NMS.
    class_names : Optional[List[str]]
        List of class names. Defaults to ["drone"].

    Returns
    -------
    List[Detection]
        Filtered and scaled detections.

    Notes
    -----
    RF-DETR outputs logits, not probabilities. We apply sigmoid for
    single-class detection or softmax for multi-class.
    """
    if class_names is None:
        class_names = ["drone"]

    # Remove batch dimension
    boxes = dets[0]  # (300, 4)
    logits = labels[0]  # (300, num_classes)

    # Handle shape variations
    if logits.ndim == 1:
        logits = logits[:, np.newaxis]

    num_classes = logits.shape[1]

    # Convert logits to probabilities
    if num_classes == 1:
        # Binary classification: sigmoid
        scores = 1.0 / (1.0 + np.exp(-logits[:, 0]))
        class_indices = np.zeros(len(scores), dtype=np.int64)
    else:
        # Multi-class: softmax and argmax
        exp_logits = np.exp(logits - logits.max(axis=1, keepdims=True))
        probs = exp_logits / exp_logits.sum(axis=1, keepdims=True)
        scores = probs.max(axis=1)
        class_indices = probs.argmax(axis=1)

    # Filter by confidence
    mask = scores >= conf_threshold
    boxes = boxes[mask]
    scores = scores[mask]
    class_indices = class_indices[mask]

    if len(boxes) == 0:
        return []

    # Convert cxcywh normalized -> xyxy pixels (at model input size)
    xyxy_model = convert_boxes(
        boxes,
        BoxFormat.CXCYWH_NORM,
        BoxFormat.XYXY,
        image_width=model_input_size,
        image_height=model_input_size,
    )

    # Scale to original image size
    xyxy_orig = scale_boxes(
        xyxy_model,
        from_shape=(model_input_size, model_input_size),
        to_shape=(orig_height, orig_width),
    )

    # Clip to image bounds
    xyxy_orig = clip_boxes(xyxy_orig, orig_width, orig_height)

    # Apply NMS
    keep_indices = nms(xyxy_orig, scores, iou_threshold)

    # Create Detection objects
    detections = []
    for idx in keep_indices:
        box = xyxy_orig[idx]
        cls_idx = class_indices[idx]
        label = class_names[cls_idx] if cls_idx < len(class_names) else str(cls_idx)

        detections.append(
            Detection(
                xyxy=(float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                conf=float(scores[idx]),
                cls=int(cls_idx),
                label=label,
            )
        )

    return detections


def postprocess_yolo(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_indices: np.ndarray,
    class_names: Optional[List[str]] = None,
) -> List[Detection]:
    """
    Postprocess YOLO model outputs (already in xyxy format).

    This is a thin wrapper for consistency with the Detection interface.
    YOLO's Ultralytics wrapper already handles most postprocessing.

    Parameters
    ----------
    boxes : np.ndarray
        Bounding boxes in xyxy format with shape (N, 4).
    scores : np.ndarray
        Confidence scores with shape (N,).
    class_indices : np.ndarray
        Class indices with shape (N,).
    class_names : Optional[List[str]]
        List of class names.

    Returns
    -------
    List[Detection]
        Detection objects.
    """
    if class_names is None:
        class_names = ["drone"]

    detections = []
    for box, score, cls_idx in zip(boxes, scores, class_indices):
        cls_idx = int(cls_idx)
        label = class_names[cls_idx] if cls_idx < len(class_names) else str(cls_idx)

        detections.append(
            Detection(
                xyxy=(float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                conf=float(score),
                cls=cls_idx,
                label=label,
            )
        )

    return detections
