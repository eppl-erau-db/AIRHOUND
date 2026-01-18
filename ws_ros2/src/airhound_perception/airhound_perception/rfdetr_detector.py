# SPDX-License-Identifier: MIT
# Copyright (c) 2025 EPPL Lab, Embry-Riddle Aeronautical University

"""
RF-DETR Detector Wrapper for AIRHOUND.

This module provides a detector wrapper for RF-DETR models, supporting
PyTorch (.pth), ONNX (.onnx), and TensorRT (.engine) model formats.

The interface matches YOLODetector for seamless swapping between models.

Usage:
    >>> from airhound_perception.rfdetr_detector import RFDETRDetector
    >>> detector = RFDETRDetector(model_path="models/drone_rfdetr.engine")
    >>> detections = detector.infer(image_bgr)
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

import numpy as np

# Try importing rfdetr - may not be available in all environments
try:
    from rfdetr import RFDETRBase, RFDETRLarge  # type: ignore[import-not-found]
    RFDETR_AVAILABLE = True
except ImportError:
    RFDETR_AVAILABLE = False
    RFDETRBase = None
    RFDETRLarge = None

# Try importing ONNX Runtime for ONNX inference
try:
    import onnxruntime as ort  # type: ignore[import-not-found]
    ORT_AVAILABLE = True
except ImportError:
    ORT_AVAILABLE = False
    ort = None

# Try importing TensorRT for engine inference
try:
    import tensorrt as trt  # type: ignore[import-not-found]
    import pycuda.driver as cuda  # type: ignore[import-not-found]
    import pycuda.autoinit  # type: ignore[import-not-found]
    TRT_AVAILABLE = True
except ImportError:
    TRT_AVAILABLE = False
    trt = None
    cuda = None


@dataclass
class Detection:
    """
    Detection result container.

    Attributes
    ----------
    xyxy : Tuple[float, float, float, float]
        Bounding box coordinates (x1, y1, x2, y2).
    conf : float
        Detection confidence score.
    cls : int
        Class ID.
    label : str
        Class name/label.
    """

    xyxy: Tuple[float, float, float, float]
    conf: float
    cls: int
    label: str

    def to_dict(self) -> Dict[str, Union[float, int, str]]:
        """Convert detection to dictionary."""
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


class RFDETRDetector:
    """
    RF-DETR detector wrapper with support for multiple model formats.

    Supports:
    - PyTorch checkpoints (.pth) via rfdetr package
    - ONNX models (.onnx) via ONNX Runtime
    - TensorRT engines (.engine) via TensorRT Python API

    Parameters
    ----------
    model_path : str
        Path to model file (.pth, .onnx, or .engine).
    model_variant : str
        Model variant: "base" or "large". Default is "base".
    conf : float
        Confidence threshold for detections.
    device : str
        Device for inference ("0" for GPU, "cpu" for CPU).
    class_names : Optional[Dict[int, str]]
        Mapping of class IDs to names. Defaults to {0: "drone"}.
    input_size : int
        Input image size (square). Default is 560 for RF-DETR.
    verbose : bool
        Enable verbose logging.

    Examples
    --------
    >>> detector = RFDETRDetector(
    ...     model_path="models/drone_rfdetr.engine",
    ...     conf=0.25,
    ... )
    >>> detections = detector.infer(cv2.imread("image.jpg"))
    >>> for det in detections:
    ...     print(f"{det.label}: {det.conf:.2f} at {det.xyxy}")
    """

    def __init__(
        self,
        model_path: str,
        model_variant: str = "base",
        conf: float = 0.25,
        device: str = "0",
        class_names: Optional[Dict[int, str]] = None,
        input_size: int = 560,
        verbose: bool = True,
    ) -> None:
        self.model_path = Path(model_path)
        self.model_variant = model_variant
        self.conf = conf
        self.device = device
        self.class_names = class_names or {0: "drone"}
        self.input_size = input_size
        self.verbose = verbose

        # Determine model format
        self.model_format = self._detect_format()

        # Load model based on format
        self.model = None
        self.ort_session = None
        self.trt_context = None

        self._load_model()

        if self.verbose:
            print(f"[RFDETRDetector] Loaded {self.model_format} model: {self.model_path}")
            print(f"[RFDETRDetector] Input size: {self.input_size}, Conf: {self.conf}")

    def _detect_format(self) -> str:
        """Detect model format from file extension."""
        suffix = self.model_path.suffix.lower()
        if suffix == ".pth":
            return "pytorch"
        elif suffix == ".onnx":
            return "onnx"
        elif suffix == ".engine":
            return "tensorrt"
        else:
            raise ValueError(f"Unsupported model format: {suffix}")

    def _load_model(self) -> None:
        """Load model based on detected format."""
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_path}")

        if self.model_format == "pytorch":
            self._load_pytorch()
        elif self.model_format == "onnx":
            self._load_onnx()
        elif self.model_format == "tensorrt":
            self._load_tensorrt()

    def _load_pytorch(self) -> None:
        """Load PyTorch model via rfdetr package."""
        if not RFDETR_AVAILABLE:
            raise ImportError(
                "rfdetr package not installed. Install with: pip install rfdetr"
            )

        if self.model_variant == "large":
            self.model = RFDETRLarge()
        else:
            self.model = RFDETRBase()

        self.model.load(str(self.model_path))

        if self.verbose:
            print(f"[RFDETRDetector] Loaded PyTorch model ({self.model_variant})")

    def _load_onnx(self) -> None:
        """Load ONNX model via ONNX Runtime."""
        if not ORT_AVAILABLE:
            raise ImportError(
                "onnxruntime not installed. Install with: pip install onnxruntime-gpu"
            )

        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        if self.device == "cpu":
            providers = ["CPUExecutionProvider"]

        self.ort_session = ort.InferenceSession(
            str(self.model_path),
            providers=providers,
        )

        if self.verbose:
            print(f"[RFDETRDetector] Loaded ONNX model with providers: {providers}")

    def _load_tensorrt(self) -> None:
        """Load TensorRT engine."""
        if not TRT_AVAILABLE:
            raise ImportError(
                "TensorRT not available. This must run on a system with TensorRT installed."
            )

        logger = trt.Logger(trt.Logger.WARNING)
        with open(self.model_path, "rb") as f:
            engine_data = f.read()

        runtime = trt.Runtime(logger)
        self.trt_engine = runtime.deserialize_cuda_engine(engine_data)
        self.trt_context = self.trt_engine.create_execution_context()

        # Allocate buffers
        self._allocate_trt_buffers()

        if self.verbose:
            print(f"[RFDETRDetector] Loaded TensorRT engine")

    def _allocate_trt_buffers(self) -> None:
        """Allocate input/output buffers for TensorRT inference."""
        self.trt_inputs = []
        self.trt_outputs = []
        self.trt_bindings = []
        self.trt_stream = cuda.Stream()

        for i in range(self.trt_engine.num_io_tensors):
            name = self.trt_engine.get_tensor_name(i)
            dtype = trt.nptype(self.trt_engine.get_tensor_dtype(name))
            shape = self.trt_engine.get_tensor_shape(name)

            # Handle dynamic shapes
            if -1 in shape:
                shape = tuple(max(1, s) for s in shape)

            size = int(np.prod(shape))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            self.trt_bindings.append(int(device_mem))

            if self.trt_engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.trt_inputs.append({"host": host_mem, "device": device_mem, "shape": shape})
            else:
                self.trt_outputs.append({"host": host_mem, "device": device_mem, "shape": shape})

    def _preprocess(self, image_bgr: np.ndarray) -> np.ndarray:
        """
        Preprocess image for RF-DETR inference.

        Parameters
        ----------
        image_bgr : np.ndarray
            Input BGR image.

        Returns
        -------
        np.ndarray
            Preprocessed image tensor (1, 3, H, W) in RGB, normalized.
        """
        import cv2

        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        # Resize to input size
        image_resized = cv2.resize(image_rgb, (self.input_size, self.input_size))

        # Normalize to [0, 1] and convert to float32
        image_normalized = image_resized.astype(np.float32) / 255.0

        # Transpose to (C, H, W) and add batch dimension
        image_tensor = np.transpose(image_normalized, (2, 0, 1))
        image_tensor = np.expand_dims(image_tensor, axis=0)

        # Make contiguous
        return np.ascontiguousarray(image_tensor)

    def _postprocess(
        self,
        outputs: Dict[str, np.ndarray],
        original_size: Tuple[int, int],
    ) -> List[Detection]:
        """
        Postprocess model outputs to Detection objects.

        Parameters
        ----------
        outputs : Dict[str, np.ndarray]
            Model outputs containing boxes and scores.
        original_size : Tuple[int, int]
            Original image size (height, width) for box scaling.

        Returns
        -------
        List[Detection]
            List of detections above confidence threshold.
        """
        detections = []

        # RF-DETR output format: boxes (N, 4) in cxcywh, scores (N, num_classes)
        # The exact output format depends on the export settings
        # Common output names: boxes/pred_boxes/dets for boxes, scores/pred_logits/labels for scores
        boxes = outputs.get("boxes", outputs.get("pred_boxes", outputs.get("dets", None)))
        scores = outputs.get("scores", outputs.get("pred_logits", outputs.get("labels", None)))

        if boxes is None or scores is None:
            # Try alternate output names via pattern matching
            for key, val in outputs.items():
                key_lower = key.lower()
                if boxes is None and ("box" in key_lower or "det" in key_lower):
                    boxes = val
                elif scores is None and ("score" in key_lower or "logit" in key_lower or "label" in key_lower):
                    scores = val

        if boxes is None or scores is None:
            return detections

        # Handle batched output
        if boxes.ndim == 3:
            boxes = boxes[0]
        if scores.ndim == 3:
            scores = scores[0]

        # Apply sigmoid if logits
        if scores.min() < 0:
            scores = 1 / (1 + np.exp(-scores))

        # Get max score per box
        if scores.ndim == 2:
            max_scores = scores.max(axis=1)
            class_ids = scores.argmax(axis=1)
        else:
            max_scores = scores
            class_ids = np.zeros(len(scores), dtype=int)

        orig_h, orig_w = original_size

        for i, (box, score, cls_id) in enumerate(zip(boxes, max_scores, class_ids)):
            if score < self.conf:
                continue

            # Convert from cxcywh to xyxy if needed
            if len(box) == 4:
                cx, cy, w, h = box
                # Check if normalized (0-1 range)
                if cx <= 1.0 and cy <= 1.0:
                    cx *= orig_w
                    cy *= orig_h
                    w *= orig_w
                    h *= orig_h

                x1 = cx - w / 2
                y1 = cy - h / 2
                x2 = cx + w / 2
                y2 = cy + h / 2
            else:
                x1, y1, x2, y2 = box[:4]

            # Clamp to image bounds
            x1 = max(0, min(x1, orig_w))
            y1 = max(0, min(y1, orig_h))
            x2 = max(0, min(x2, orig_w))
            y2 = max(0, min(y2, orig_h))

            label = self.class_names.get(int(cls_id), str(cls_id))

            detections.append(
                Detection(
                    xyxy=(float(x1), float(y1), float(x2), float(y2)),
                    conf=float(score),
                    cls=int(cls_id),
                    label=label,
                )
            )

        return detections

    def infer(self, image_bgr: np.ndarray) -> List[Detection]:
        """
        Run inference on a BGR image.

        Parameters
        ----------
        image_bgr : np.ndarray
            Input image in BGR format (OpenCV default).

        Returns
        -------
        List[Detection]
            List of detections found in the image.
        """
        original_size = (image_bgr.shape[0], image_bgr.shape[1])

        if self.model_format == "pytorch":
            return self._infer_pytorch(image_bgr, original_size)
        elif self.model_format == "onnx":
            return self._infer_onnx(image_bgr, original_size)
        elif self.model_format == "tensorrt":
            return self._infer_tensorrt(image_bgr, original_size)

        return []

    def _infer_pytorch(
        self, image_bgr: np.ndarray, original_size: Tuple[int, int]
    ) -> List[Detection]:
        """Run inference using PyTorch model."""
        import cv2

        # RF-DETR expects RGB PIL Image or numpy array
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        # Use rfdetr's built-in predict
        results = self.model.predict(image_rgb, threshold=self.conf)

        detections = []
        for result in results:
            # rfdetr returns dict with 'boxes', 'labels', 'scores'
            boxes = result.get("boxes", [])
            labels = result.get("labels", [])
            scores = result.get("scores", [])

            for box, label, score in zip(boxes, labels, scores):
                if score < self.conf:
                    continue

                x1, y1, x2, y2 = box
                class_name = self.class_names.get(int(label), str(label))

                detections.append(
                    Detection(
                        xyxy=(float(x1), float(y1), float(x2), float(y2)),
                        conf=float(score),
                        cls=int(label),
                        label=class_name,
                    )
                )

        return detections

    def _infer_onnx(
        self, image_bgr: np.ndarray, original_size: Tuple[int, int]
    ) -> List[Detection]:
        """Run inference using ONNX Runtime."""
        input_tensor = self._preprocess(image_bgr)

        # Get input name
        input_name = self.ort_session.get_inputs()[0].name

        # Run inference
        outputs = self.ort_session.run(None, {input_name: input_tensor})

        # Build output dict
        output_names = [o.name for o in self.ort_session.get_outputs()]
        output_dict = {name: out for name, out in zip(output_names, outputs)}

        return self._postprocess(output_dict, original_size)

    def _infer_tensorrt(
        self, image_bgr: np.ndarray, original_size: Tuple[int, int]
    ) -> List[Detection]:
        """Run inference using TensorRT engine."""
        input_tensor = self._preprocess(image_bgr)

        # Copy input to device
        np.copyto(self.trt_inputs[0]["host"], input_tensor.ravel())
        cuda.memcpy_htod_async(
            self.trt_inputs[0]["device"],
            self.trt_inputs[0]["host"],
            self.trt_stream,
        )

        # Run inference - TensorRT 10.x uses execute_async_v3, older uses execute_async_v2
        if hasattr(self.trt_context, 'execute_async_v3'):
            # TensorRT 10.x API - set tensor addresses first
            for i, inp in enumerate(self.trt_inputs):
                name = self.trt_engine.get_tensor_name(i)
                self.trt_context.set_tensor_address(name, int(inp["device"]))
            for i, out in enumerate(self.trt_outputs):
                name = self.trt_engine.get_tensor_name(i + len(self.trt_inputs))
                self.trt_context.set_tensor_address(name, int(out["device"]))
            self.trt_context.execute_async_v3(stream_handle=self.trt_stream.handle)
        else:
            # TensorRT 8.x/9.x API
            self.trt_context.execute_async_v2(
                bindings=self.trt_bindings,
                stream_handle=self.trt_stream.handle,
            )

        # Copy outputs to host
        for out in self.trt_outputs:
            cuda.memcpy_dtoh_async(out["host"], out["device"], self.trt_stream)

        self.trt_stream.synchronize()

        # Build output dict
        output_dict = {}
        for i, out in enumerate(self.trt_outputs):
            name = self.trt_engine.get_tensor_name(i + len(self.trt_inputs))
            output_dict[name] = out["host"].reshape(out["shape"])

        return self._postprocess(output_dict, original_size)

    def switch_model(self, new_model_path: str) -> None:
        """
        Switch to a different model at runtime.

        Parameters
        ----------
        new_model_path : str
            Path to new model file.
        """
        self.model_path = Path(new_model_path)
        self.model_format = self._detect_format()
        self._load_model()

        if self.verbose:
            print(f"[RFDETRDetector] Switched to model: {self.model_path}")
