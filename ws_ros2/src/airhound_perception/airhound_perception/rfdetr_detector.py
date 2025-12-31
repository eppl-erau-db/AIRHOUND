"""
RF-DETR Detector Module.

Provides TensorRT and ONNX Runtime inference for RF-DETR drone detection model.
Designed for modular comparison with YOLO in the SPIE abstract.

This module supports:
    - TensorRT engines (.engine) for Jetson deployment
    - ONNX models (.onnx) for development/testing
    - Automatic backend selection based on file extension

Author: EPPL - Embry-Riddle Aeronautical University
Date: 2025

References
----------
- RF-DETR: https://github.com/roboflow/rf-detr
- TensorRT: https://developer.nvidia.com/tensorrt
"""

from __future__ import annotations

import os
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .postprocessor import Detection, postprocess_rfdetr


class InferenceBackend(ABC):
    """Abstract base class for inference backends."""

    @abstractmethod
    def infer(self, input_tensor: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Run inference on preprocessed input tensor.

        Parameters
        ----------
        input_tensor : np.ndarray
            Preprocessed input with shape (1, 3, H, W), dtype float32.

        Returns
        -------
        Tuple[np.ndarray, np.ndarray]
            dets: Bounding boxes with shape (1, 300, 4).
            labels: Class logits with shape (1, 300, num_classes).
        """
        pass


class ONNXBackend(InferenceBackend):
    """ONNX Runtime inference backend."""

    def __init__(self, model_path: str, device: str = "cuda"):
        """
        Initialize ONNX Runtime backend.

        Parameters
        ----------
        model_path : str
            Path to .onnx model file.
        device : str
            Device for inference: "cuda" or "cpu".
        """
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise ImportError(
                "onnxruntime not installed. Install with: "
                "pip install onnxruntime-gpu  # for CUDA"
            ) from e

        # Select providers based on device
        if device.lower() in ("cuda", "gpu", "0"):
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        else:
            providers = ["CPUExecutionProvider"]

        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [out.name for out in self.session.get_outputs()]

        # Log provider info
        actual_provider = self.session.get_providers()[0]
        print(f"[RFDETRDetector] ONNX Runtime using: {actual_provider}")

    def infer(self, input_tensor: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Run ONNX inference."""
        outputs = self.session.run(self.output_names, {self.input_name: input_tensor})

        # RF-DETR outputs: dets (1, 300, 4), labels (1, 300, 1)
        dets = outputs[0]
        labels = outputs[1]

        return dets, labels


class TensorRTBackend(InferenceBackend):
    """TensorRT inference backend for Jetson deployment."""

    def __init__(self, engine_path: str):
        """
        Initialize TensorRT backend.

        Parameters
        ----------
        engine_path : str
            Path to .engine TensorRT serialized engine.
        """
        try:
            import tensorrt as trt
            import pycuda.driver as cuda
            import pycuda.autoinit  # noqa: F401
        except ImportError as e:
            raise ImportError(
                "TensorRT or PyCUDA not installed. "
                "This backend is only available on NVIDIA Jetson or systems with TensorRT."
            ) from e

        self.trt = trt
        self.cuda = cuda

        # Load engine
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(self.logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self._allocate_buffers()

        print(f"[RFDETRDetector] TensorRT engine loaded: {engine_path}")

    def _allocate_buffers(self):
        """Allocate CUDA device buffers for input/output."""
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.stream = self.cuda.Stream()

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = self.engine.get_tensor_shape(name)
            dtype = self.trt.nptype(self.engine.get_tensor_dtype(name))
            size = np.prod(shape) * np.dtype(dtype).itemsize

            # Allocate device memory
            device_mem = self.cuda.mem_alloc(int(size))
            self.bindings.append(int(device_mem))

            if self.engine.get_tensor_mode(name) == self.trt.TensorIOMode.INPUT:
                self.inputs.append(
                    {"name": name, "shape": shape, "dtype": dtype, "device": device_mem}
                )
            else:
                self.outputs.append(
                    {"name": name, "shape": shape, "dtype": dtype, "device": device_mem}
                )

    def infer(self, input_tensor: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Run TensorRT inference."""
        # Copy input to device
        input_tensor = np.ascontiguousarray(input_tensor)
        self.cuda.memcpy_htod_async(
            self.inputs[0]["device"], input_tensor, self.stream
        )

        # Run inference
        self.context.execute_async_v2(
            bindings=self.bindings, stream_handle=self.stream.handle
        )

        # Copy outputs from device
        outputs = []
        for out in self.outputs:
            host_buf = np.empty(out["shape"], dtype=out["dtype"])
            self.cuda.memcpy_dtoh_async(host_buf, out["device"], self.stream)
            outputs.append(host_buf)

        self.stream.synchronize()

        # RF-DETR outputs: dets (1, 300, 4), labels (1, 300, 1)
        return outputs[0], outputs[1]


class RFDETRDetector:
    """
    RF-DETR object detector with TensorRT/ONNX backend.

    This class provides the same interface as YOLODetector for modular
    comparison in the AIRHOUND perception pipeline.

    Parameters
    ----------
    model_path : str
        Path to model file (.engine for TensorRT, .onnx for ONNX Runtime).
    imgsz : int
        Model input size (default 560 for RF-DETR Base).
    conf : float
        Confidence threshold for filtering detections.
    iou : float
        IoU threshold for NMS.
    device : str
        Device for inference: "0" (GPU), "cuda", or "cpu".
    verbose : bool
        Enable verbose logging.

    Attributes
    ----------
    names : dict
        Class name mapping {0: "drone"}.

    Examples
    --------
    >>> detector = RFDETRDetector(
    ...     model_path="models/drone_rfdetr.engine",
    ...     conf=0.25,
    ... )
    >>> detections = detector.infer(image_bgr)
    >>> for det in detections:
    ...     print(f"{det.label}: {det.conf:.2f} at {det.xyxy}")

    See Also
    --------
    YOLODetector : YOLO detector with same interface.
    postprocessor : Postprocessing utilities.
    """

    def __init__(
        self,
        model_path: str,
        imgsz: int = 560,
        conf: float = 0.25,
        iou: float = 0.45,
        device: str = "0",
        verbose: bool = True,
    ):
        """Initialize RF-DETR detector."""
        self.model_path = model_path
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.device = device
        self.verbose = verbose

        # Class names (single-class drone detection)
        self.names = {0: "drone"}

        # Select backend based on file extension
        self.backend = self._load_backend(model_path)

        if self.verbose:
            print(f"[RFDETRDetector] Initialized: {model_path}")
            print(f"[RFDETRDetector] Input size: {imgsz}, conf: {conf}, iou: {iou}")

    def _load_backend(self, model_path: str) -> InferenceBackend:
        """Load appropriate inference backend."""
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")

        ext = os.path.splitext(model_path)[1].lower()

        if ext == ".engine":
            return TensorRTBackend(model_path)
        elif ext == ".onnx":
            return ONNXBackend(model_path, device=self.device)
        else:
            raise ValueError(
                f"Unsupported model format: {ext}. Use .engine or .onnx"
            )

    def preprocess(self, image_bgr: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        """
        Preprocess image for RF-DETR inference.

        Parameters
        ----------
        image_bgr : np.ndarray
            Input image in BGR format (OpenCV default).

        Returns
        -------
        Tuple[np.ndarray, Tuple[int, int]]
            input_tensor: Preprocessed tensor with shape (1, 3, H, W).
            orig_shape: Original image (height, width).
        """
        orig_h, orig_w = image_bgr.shape[:2]

        # Convert BGR -> RGB
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        # Resize to model input size (direct resize, no letterboxing)
        resized = cv2.resize(rgb, (self.imgsz, self.imgsz), interpolation=cv2.INTER_LINEAR)

        # Normalize to [0, 1] and convert to float32
        normalized = resized.astype(np.float32) / 255.0

        # HWC -> CHW -> NCHW
        tensor = normalized.transpose(2, 0, 1)
        tensor = tensor[np.newaxis, ...]

        return tensor, (orig_h, orig_w)

    def infer(self, image_bgr: np.ndarray) -> List[Detection]:
        """
        Run inference on an image.

        Parameters
        ----------
        image_bgr : np.ndarray
            Input image in BGR format (H, W, 3).

        Returns
        -------
        List[Detection]
            List of Detection objects with xyxy coordinates in original
            image space.

        Notes
        -----
        This method matches the YOLODetector.infer() interface for
        drop-in replacement in the perception pipeline.
        """
        # Preprocess
        input_tensor, (orig_h, orig_w) = self.preprocess(image_bgr)

        # Run inference
        dets, labels = self.backend.infer(input_tensor)

        # Postprocess
        detections = postprocess_rfdetr(
            dets=dets,
            labels=labels,
            orig_width=orig_w,
            orig_height=orig_h,
            model_input_size=self.imgsz,
            conf_threshold=self.conf,
            iou_threshold=self.iou,
            class_names=list(self.names.values()),
        )

        return detections

    def switch_model(self, new_model_path: str, prefer_engine: Optional[bool] = None):
        """
        Dynamically load a different model at runtime.

        Parameters
        ----------
        new_model_path : str
            Path to new model file.
        prefer_engine : Optional[bool]
            Ignored (for YOLODetector API compatibility).
        """
        self.model_path = new_model_path
        self.backend = self._load_backend(new_model_path)

        if self.verbose:
            print(f"[RFDETRDetector] Switched model to {new_model_path}")
