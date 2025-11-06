from typing import List, Optional, Tuple, Union, Dict

import os
import time
import numpy as np

try:
    from ultralytics import YOLO  # type: ignore
except Exception:  # pragma: no cover - optional dependency in dev envs
    YOLO = None  # type: ignore


class Detection:
    """
    Simple value object representing a detection.
    """

    __slots__ = ("xyxy", "conf", "cls", "label")

    def __init__(
        self, xyxy: Tuple[float, float, float, float], conf: float, cls: int, label: str
    ):
        self.xyxy = xyxy  # (x1, y1, x2, y2)
        self.conf = conf
        self.cls = cls
        self.label = label

    def to_dict(self) -> Dict[str, Union[float, int, str]]:
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


class YOLODetector:
    """
    Wrapper around Ultralytics YOLO with:
      - Automatic TensorRT engine preference (if .engine present beside .pt)
      - On-demand export to TensorRT
      - Convenience for dynamic model switching at runtime
    """

    def __init__(
        self,
        model_path: str,
        imgsz: int = 1280,
        conf: float = 0.25,
        iou: float = 0.45,
        device: str = "0",
        prefer_engine: bool = True,
        autoload_engine: bool = True,
        export_if_missing: bool = False,
        half: bool = True,
        verbose: bool = True,
    ):
        if YOLO is None:
            raise RuntimeError(
                "Ultralytics not available. Install with: pip install ultralytics"
            )

        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.device = device
        self.prefer_engine = prefer_engine
        self.autoload_engine = autoload_engine
        self.export_if_missing = export_if_missing
        self.half = half
        self.verbose = verbose

        self.original_model_path = model_path
        self.model_path = self._resolve_model_path(model_path)
        self.model = self._load_model(self.model_path)
        self.names = getattr(self.model, "names", {}) or {}
        # Force single-class rename convenience (e.g., drone) if user sets ENV
        custom_single = os.environ.get("YOLO_SINGLE_CLASS_NAME")
        if custom_single and isinstance(self.names, dict) and len(self.names) == 1:
            self.names = {0: custom_single}
            if self.verbose:
                print(f"[YOLODetector] Overriding single class name -> {custom_single}")

    # -------------------------------------------------
    # Internal helpers
    # -------------------------------------------------
    def _resolve_model_path(self, p: str) -> str:
        """
        If prefer_engine and an engine with same stem exists, use it.
        """
        base, ext = os.path.splitext(p)
        engine_candidate = base + ".engine"
        if (
            self.prefer_engine
            and self.autoload_engine
            and os.path.exists(engine_candidate)
        ):
            if self.verbose:
                print(f"[YOLODetector] Found TensorRT engine: {engine_candidate}")
            return engine_candidate
        return p

    def _load_model(self, p: str):
        """
        Load YOLO model or TensorRT engine. Ultralytics YOLO class
        handles .pt and .engine seamlessly (engine when exported with YOLO export).
        Adds a one‑time log clarifying which artifact actually loaded.
        """
        if self.verbose:
            print(f"[YOLODetector] Loading model: {p}")
        try:
            model = YOLO(p)
        except Exception as e:
            # If we attempted engine first and failed, fallback to .pt
            if p.endswith(".engine") and self.prefer_engine:
                pt_candidate = os.path.splitext(self.original_model_path)[0] + ".pt"
                if os.path.exists(pt_candidate):
                    if self.verbose:
                        print(f"[YOLODetector] Engine load failed, retrying .pt: {e}")
                    model = YOLO(pt_candidate)
                elif self.export_if_missing:
                    if self.verbose:
                        print(
                            f"[YOLODetector] Attempting export because engine load failed: {e}"
                        )
                    model = self._export_engine(pt_candidate)
                else:
                    raise
            else:
                raise
        # Post-load explicit artifact clarification
        if self.verbose:
            if p.endswith(".engine"):
                print(f"[YOLODetector] Loaded TensorRT engine: {p}")
            else:
                print(f"[YOLODetector] Loaded PyTorch model (.pt): {p}")
        return model

    def _export_engine(self, pt_path: str):
        """
        Export a TensorRT engine from a .pt if requested. Returns loaded model (engine or .pt fallback).
        """
        if not os.path.exists(pt_path):
            raise FileNotFoundError(f"Cannot export engine; .pt not found: {pt_path}")
        if self.verbose:
            print(
                f"[YOLODetector] Exporting TensorRT engine from {pt_path} (half={self.half}) ..."
            )
        y = YOLO(pt_path)
        try:
            engine_path = y.export(format="engine", device=self.device, half=self.half)
            if self.verbose:
                print(f"[YOLODetector] Engine exported: {engine_path}")
            return YOLO(engine_path)
        except Exception as e:
            print(f"[YOLODetector] Engine export failed, using .pt. Error: {e}")
            return y

    # -------------------------------------------------
    # Public API
    # -------------------------------------------------
    def switch_model(self, new_model_path: str, prefer_engine: Optional[bool] = None):
        """
        Dynamically load a different model / engine at runtime.
        """
        if prefer_engine is not None:
            self.prefer_engine = prefer_engine
        self.original_model_path = new_model_path
        self.model_path = self._resolve_model_path(new_model_path)
        self.model = self._load_model(self.model_path)
        self.names = getattr(self.model, "names", {}) or {}
        if self.verbose:
            print(f"[YOLODetector] Switched model to {self.model_path}")

    def ensure_engine(self) -> str:
        """
        Guarantee an engine exists (export if absent and export_if_missing).
        Returns path to engine if successful (else .pt path).
        """
        base, _ = os.path.splitext(self.original_model_path)
        engine_path = base + ".engine"
        if os.path.exists(engine_path):
            return engine_path
        if not self.export_if_missing:
            return self.original_model_path
        # Export
        self.model = self._export_engine(self.original_model_path)
        # After export, attempt to reload engine explicitly
        if os.path.exists(engine_path):
            self.model_path = engine_path
        return self.model_path

    def infer(self, image_bgr: np.ndarray) -> List[Detection]:
        # Ultralytics expects RGB
        rgb = image_bgr[:, :, ::-1]
        results = self.model.predict(
            source=rgb,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )
        dets: List[Detection] = []
        if not results:
            return dets
        res = results[0]
        if res.boxes is None or len(res.boxes) == 0:
            return dets

        boxes = res.boxes.xyxy.cpu().numpy()
        confs = res.boxes.conf.cpu().numpy()
        clss = res.boxes.cls.cpu().numpy().astype(int)

        for (x1, y1, x2, y2), c, k in zip(boxes, confs, clss):
            label = str(self.names.get(int(k), int(k)))
            dets.append(
                Detection(
                    (float(x1), float(y1), float(x2), float(y2)),
                    float(c),
                    int(k),
                    label,
                )
            )
        return dets
