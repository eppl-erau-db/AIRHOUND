#!/usr/bin/env python3
"""
benchmark_comparison.py - RF-DETR vs YOLOv8 Comprehensive Benchmark

Compares RF-DETR and YOLOv8 detectors on NVIDIA Jetson Orin for the SPIE paper.

Metrics collected:
- FPS / Throughput (frames per second)
- Latency (mean, median, p95 in milliseconds)
- GPU Memory usage (via tegrastats)
- Power consumption (via tegrastats VDD_CPU_GPU_CV)
- Detection accuracy (mAP@0.5, mAP@0.5:0.95)

Usage:
    python scripts/benchmark_comparison.py
    python scripts/benchmark_comparison.py --perf-only  # Skip mAP (faster)
    python scripts/benchmark_comparison.py --accuracy-only  # Skip perf
    python scripts/benchmark_comparison.py --num-images 100  # Limit images for mAP

Output:
    benchmark_results/benchmark_YYYY-MM-DD_HH-MM-SS.json
    benchmark_results/benchmark_YYYY-MM-DD_HH-MM-SS.md
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

# Add the perception package to path
sys.path.insert(0, str(Path(__file__).parent.parent / "ws_ros2" / "src" / "airhound_perception"))

# =============================================================================
# Configuration
# =============================================================================

YOLOV8_ENGINE = Path.home() / "dev_workspace" / "airhoundPerception" / "models" / "yolov8Detector.engine"
RFDETR_ENGINE = Path.home() / "AIRHOUND" / "models" / "drone_rfdetr.engine"

DATASET_PATH = Path.home() / "AIRHOUND" / "data"
VALID_IMAGES = DATASET_PATH / "valid" / "images"
VALID_LABELS = DATASET_PATH / "valid" / "labels"

OUTPUT_DIR = Path.home() / "AIRHOUND" / "benchmark_results"

# Model input sizes (native resolution for fair comparison)
YOLOV8_INPUT_SIZE = 1280
RFDETR_INPUT_SIZE = 560

# Benchmark parameters
WARMUP_ITERATIONS = 10
PERF_ITERATIONS = 100
TEGRASTATS_INTERVAL_MS = 100


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class Detection:
    """Single detection result."""
    xyxy: Tuple[float, float, float, float]  # x1, y1, x2, y2
    conf: float
    cls: int
    label: str


@dataclass
class PerformanceMetrics:
    """Performance benchmark results."""
    model_name: str
    model_path: str
    input_size: int
    warmup_iterations: int
    timed_iterations: int
    latencies_ms: List[float] = field(default_factory=list)
    ram_mb_samples: List[float] = field(default_factory=list)
    power_mw_samples: List[float] = field(default_factory=list)
    gpu_util_samples: List[float] = field(default_factory=list)
    gpu_temp_samples: List[float] = field(default_factory=list)

    def summary(self) -> Dict[str, Any]:
        if not self.latencies_ms:
            return {"model_name": self.model_name, "error": "no measurements"}
        
        latency_mean = statistics.mean(self.latencies_ms)
        latency_median = statistics.median(self.latencies_ms)
        latency_p95 = float(np.percentile(self.latencies_ms, 95))
        latency_min = min(self.latencies_ms)
        latency_max = max(self.latencies_ms)
        
        fps_mean = 1000.0 / latency_mean if latency_mean > 0 else 0
        fps_median = 1000.0 / latency_median if latency_median > 0 else 0
        
        result = {
            "model_name": self.model_name,
            "model_path": self.model_path,
            "input_size": self.input_size,
            "warmup_iterations": self.warmup_iterations,
            "timed_iterations": self.timed_iterations,
            "latency_ms_mean": round(latency_mean, 2),
            "latency_ms_median": round(latency_median, 2),
            "latency_ms_p95": round(latency_p95, 2),
            "latency_ms_min": round(latency_min, 2),
            "latency_ms_max": round(latency_max, 2),
            "fps_mean": round(fps_mean, 1),
            "fps_median": round(fps_median, 1),
        }
        
        if self.ram_mb_samples:
            result["ram_mb_mean"] = round(statistics.mean(self.ram_mb_samples), 1)
            result["ram_mb_max"] = round(max(self.ram_mb_samples), 1)
        
        if self.power_mw_samples:
            result["power_mw_mean"] = round(statistics.mean(self.power_mw_samples), 0)
            result["power_w_mean"] = round(statistics.mean(self.power_mw_samples) / 1000, 2)
        
        if self.gpu_util_samples:
            result["gpu_util_mean"] = round(statistics.mean(self.gpu_util_samples), 1)
        
        if self.gpu_temp_samples:
            result["gpu_temp_c_mean"] = round(statistics.mean(self.gpu_temp_samples), 1)
            result["gpu_temp_c_max"] = round(max(self.gpu_temp_samples), 1)
        
        return result


@dataclass
class AccuracyMetrics:
    """Accuracy benchmark results."""
    model_name: str
    num_images: int
    num_ground_truth: int
    num_predictions: int
    true_positives: int = 0
    false_positives: int = 0
    false_negatives: int = 0
    iou_thresholds: List[float] = field(default_factory=lambda: [0.5])
    ap_per_threshold: Dict[float, float] = field(default_factory=dict)
    
    def summary(self) -> Dict[str, Any]:
        precision = self.true_positives / (self.true_positives + self.false_positives) if (self.true_positives + self.false_positives) > 0 else 0
        recall = self.true_positives / (self.true_positives + self.false_negatives) if (self.true_positives + self.false_negatives) > 0 else 0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0
        
        return {
            "model_name": self.model_name,
            "num_images": self.num_images,
            "num_ground_truth": self.num_ground_truth,
            "num_predictions": self.num_predictions,
            "true_positives": self.true_positives,
            "false_positives": self.false_positives,
            "false_negatives": self.false_negatives,
            "precision": round(precision, 4),
            "recall": round(recall, 4),
            "f1_score": round(f1, 4),
            "mAP_0.5": round(self.ap_per_threshold.get(0.5, 0), 4),
            "mAP_0.5_0.95": round(statistics.mean(list(self.ap_per_threshold.values())) if self.ap_per_threshold else 0, 4),
        }


# =============================================================================
# Tegrastats Monitor
# =============================================================================

class TegrastatsMonitor:
    """Background thread to monitor Jetson power/memory via tegrastats."""
    
    def __init__(self, interval_ms: int = 100):
        self.interval_ms = interval_ms
        self.process: Optional[subprocess.Popen] = None
        self.thread: Optional[threading.Thread] = None
        self.running = False
        self.samples: List[Dict[str, float]] = []
        self._lock = threading.Lock()
    
    def start(self):
        """Start tegrastats monitoring in background."""
        self.running = True
        self.samples = []
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
    
    def stop(self) -> List[Dict[str, float]]:
        """Stop monitoring and return collected samples."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
        with self._lock:
            return list(self.samples)
    
    def _monitor_loop(self):
        """Run tegrastats and parse output."""
        try:
            self.process = subprocess.Popen(
                ["tegrastats", "--interval", str(self.interval_ms)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            
            while self.running and self.process.poll() is None:
                line = self.process.stdout.readline()
                if line:
                    sample = self._parse_tegrastats_line(line)
                    if sample:
                        with self._lock:
                            self.samples.append(sample)
        except Exception as e:
            print(f"[WARN] Tegrastats error: {e}")
    
    def _parse_tegrastats_line(self, line: str) -> Optional[Dict[str, float]]:
        """Parse a single tegrastats output line."""
        sample = {}
        
        # RAM 4741/15656MB
        ram_match = re.search(r'RAM (\d+)/(\d+)MB', line)
        if ram_match:
            sample['ram_used_mb'] = float(ram_match.group(1))
            sample['ram_total_mb'] = float(ram_match.group(2))
        
        # GR3D_FREQ 0%
        gpu_match = re.search(r'GR3D_FREQ (\d+)%', line)
        if gpu_match:
            sample['gpu_util'] = float(gpu_match.group(1))
        
        # VDD_CPU_GPU_CV 916mW/916mW
        power_match = re.search(r'VDD_CPU_GPU_CV (\d+)mW', line)
        if power_match:
            sample['power_mw'] = float(power_match.group(1))
        
        # gpu@45.468C
        gpu_temp_match = re.search(r'gpu@([\d.]+)C', line)
        if gpu_temp_match:
            sample['gpu_temp_c'] = float(gpu_temp_match.group(1))
        
        return sample if sample else None


# =============================================================================
# Detector Wrappers
# =============================================================================

def load_yolov8_detector(engine_path: Path, input_size: int) -> Any:
    """Load YOLOv8 TensorRT detector."""
    from ultralytics import YOLO
    print(f"[INFO] Loading YOLOv8 from: {engine_path}")
    model = YOLO(str(engine_path))
    return model


def load_rfdetr_detector(engine_path: Path, input_size: int) -> Any:
    """Load RF-DETR TensorRT detector."""
    from airhound_perception.rfdetr_detector import RFDETRDetector
    print(f"[INFO] Loading RF-DETR from: {engine_path}")
    detector = RFDETRDetector(
        model_path=str(engine_path),
        conf=0.25,
        input_size=input_size,
        verbose=True,
    )
    return detector


def infer_yolov8(model: Any, image_bgr: np.ndarray, input_size: int) -> List[Detection]:
    """Run YOLOv8 inference."""
    results = model.predict(
        source=image_bgr[:, :, ::-1],  # BGR to RGB
        imgsz=input_size,
        conf=0.25,
        iou=0.45,
        device="0",
        verbose=False,
    )
    
    detections = []
    if results and results[0].boxes is not None:
        boxes = results[0].boxes.xyxy.cpu().numpy()
        confs = results[0].boxes.conf.cpu().numpy()
        clss = results[0].boxes.cls.cpu().numpy().astype(int)
        
        for (x1, y1, x2, y2), conf, cls in zip(boxes, confs, clss):
            detections.append(Detection(
                xyxy=(float(x1), float(y1), float(x2), float(y2)),
                conf=float(conf),
                cls=int(cls),
                label="drone",
            ))
    
    return detections


def infer_rfdetr(detector: Any, image_bgr: np.ndarray) -> List[Detection]:
    """Run RF-DETR inference."""
    return detector.infer(image_bgr)


# =============================================================================
# mAP Calculation
# =============================================================================

def compute_iou(box1: Tuple[float, ...], box2: Tuple[float, ...]) -> float:
    """Compute IoU between two boxes in xyxy format."""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    
    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    union_area = box1_area + box2_area - inter_area
    
    return inter_area / union_area if union_area > 0 else 0


def load_yolo_labels(label_path: Path, img_width: int, img_height: int) -> List[Tuple[float, ...]]:
    """Load YOLO format labels and convert to xyxy."""
    boxes = []
    if not label_path.exists():
        return boxes
    
    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
                # class cx cy w h (normalized)
                cx, cy, w, h = map(float, parts[1:5])
                
                # Convert to xyxy (absolute)
                x1 = (cx - w / 2) * img_width
                y1 = (cy - h / 2) * img_height
                x2 = (cx + w / 2) * img_width
                y2 = (cy + h / 2) * img_height
                
                boxes.append((x1, y1, x2, y2))
    
    return boxes


def compute_ap(
    predictions: List[Tuple[float, Tuple[float, ...]]],  # (conf, box)
    ground_truths: List[Tuple[float, ...]],  # boxes
    iou_threshold: float,
) -> Tuple[float, int, int, int]:
    """
    Compute Average Precision at a given IoU threshold.
    
    Returns: (AP, TP, FP, FN)
    """
    if not ground_truths:
        # No ground truth - all predictions are FP
        return 0.0, 0, len(predictions), 0
    
    if not predictions:
        # No predictions - all ground truths are FN
        return 0.0, 0, 0, len(ground_truths)
    
    # Sort predictions by confidence (descending)
    predictions = sorted(predictions, key=lambda x: x[0], reverse=True)
    
    gt_matched = [False] * len(ground_truths)
    tp = 0
    fp = 0
    
    precisions = []
    recalls = []
    
    for conf, pred_box in predictions:
        best_iou = 0
        best_gt_idx = -1
        
        for gt_idx, gt_box in enumerate(ground_truths):
            if gt_matched[gt_idx]:
                continue
            
            iou = compute_iou(pred_box, gt_box)
            if iou > best_iou:
                best_iou = iou
                best_gt_idx = gt_idx
        
        if best_iou >= iou_threshold:
            tp += 1
            gt_matched[best_gt_idx] = True
        else:
            fp += 1
        
        precision = tp / (tp + fp)
        recall = tp / len(ground_truths)
        precisions.append(precision)
        recalls.append(recall)
    
    fn = len(ground_truths) - tp
    
    # Compute AP using 11-point interpolation or all-point interpolation
    if not recalls:
        return 0.0, tp, fp, fn
    
    # All-point interpolation
    precisions = np.array(precisions)
    recalls = np.array(recalls)
    
    # Make precision monotonically decreasing
    for i in range(len(precisions) - 2, -1, -1):
        precisions[i] = max(precisions[i], precisions[i + 1])
    
    # Compute AP as area under PR curve
    ap = 0.0
    prev_recall = 0.0
    for i, (p, r) in enumerate(zip(precisions, recalls)):
        ap += p * (r - prev_recall)
        prev_recall = r
    
    return ap, tp, fp, fn


# =============================================================================
# Benchmark Functions
# =============================================================================

def run_performance_benchmark(
    model_name: str,
    model: Any,
    infer_func,
    input_size: int,
    model_path: str,
    warmup_iters: int = WARMUP_ITERATIONS,
    timed_iters: int = PERF_ITERATIONS,
) -> PerformanceMetrics:
    """Run performance benchmark for a single model."""
    print(f"\n{'='*60}")
    print(f"Performance Benchmark: {model_name}")
    print(f"{'='*60}")
    
    metrics = PerformanceMetrics(
        model_name=model_name,
        model_path=model_path,
        input_size=input_size,
        warmup_iterations=warmup_iters,
        timed_iterations=timed_iters,
    )
    
    # Create synthetic test image at native resolution
    test_image = np.random.randint(0, 255, (input_size, input_size, 3), dtype=np.uint8)
    
    # Warmup
    print(f"[INFO] Warming up ({warmup_iters} iterations)...")
    for _ in range(warmup_iters):
        infer_func(model, test_image)
    
    # Sync CUDA
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.synchronize()
    except:
        pass
    
    # Start tegrastats monitoring
    monitor = TegrastatsMonitor(interval_ms=TEGRASTATS_INTERVAL_MS)
    monitor.start()
    
    # Timed runs
    print(f"[INFO] Running {timed_iters} timed iterations...")
    for i in range(timed_iters):
        t0 = time.perf_counter()
        infer_func(model, test_image)
        
        # Sync CUDA for accurate timing
        try:
            import torch
            if torch.cuda.is_available():
                torch.cuda.synchronize()
        except:
            pass
        
        t1 = time.perf_counter()
        metrics.latencies_ms.append((t1 - t0) * 1000)
        
        if (i + 1) % 25 == 0:
            print(f"  [{i+1}/{timed_iters}] Latest: {metrics.latencies_ms[-1]:.1f} ms")
    
    # Stop monitoring and collect samples
    samples = monitor.stop()
    
    for sample in samples:
        if 'ram_used_mb' in sample:
            metrics.ram_mb_samples.append(sample['ram_used_mb'])
        if 'power_mw' in sample:
            metrics.power_mw_samples.append(sample['power_mw'])
        if 'gpu_util' in sample:
            metrics.gpu_util_samples.append(sample['gpu_util'])
        if 'gpu_temp_c' in sample:
            metrics.gpu_temp_samples.append(sample['gpu_temp_c'])
    
    summary = metrics.summary()
    print(f"\n[RESULT] {model_name}:")
    print(f"  FPS (mean):     {summary.get('fps_mean', 'N/A')}")
    print(f"  Latency (mean): {summary.get('latency_ms_mean', 'N/A')} ms")
    print(f"  Latency (p95):  {summary.get('latency_ms_p95', 'N/A')} ms")
    print(f"  Power (mean):   {summary.get('power_w_mean', 'N/A')} W")
    print(f"  GPU Temp (max): {summary.get('gpu_temp_c_max', 'N/A')} C")
    
    return metrics


def run_accuracy_benchmark(
    model_name: str,
    model: Any,
    infer_func,
    input_size: int,
    images_dir: Path,
    labels_dir: Path,
    num_images: Optional[int] = None,
) -> AccuracyMetrics:
    """Run accuracy benchmark (mAP) for a single model."""
    print(f"\n{'='*60}")
    print(f"Accuracy Benchmark: {model_name}")
    print(f"{'='*60}")
    
    image_files = sorted(images_dir.glob("*.jpg"))
    if num_images:
        image_files = image_files[:num_images]
    
    print(f"[INFO] Evaluating on {len(image_files)} images...")
    
    # IoU thresholds for mAP@0.5:0.95
    iou_thresholds = [0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95]
    
    # Collect all predictions and ground truths
    all_predictions: List[Tuple[float, Tuple[float, ...]]] = []
    all_ground_truths: List[Tuple[float, ...]] = []
    
    total_gt = 0
    total_pred = 0
    
    for i, img_path in enumerate(image_files):
        # Load image
        image = cv2.imread(str(img_path))
        if image is None:
            continue
        
        img_h, img_w = image.shape[:2]
        
        # Load ground truth labels
        label_path = labels_dir / (img_path.stem + ".txt")
        gt_boxes = load_yolo_labels(label_path, img_w, img_h)
        all_ground_truths.extend(gt_boxes)
        total_gt += len(gt_boxes)
        
        # Run inference
        detections = infer_func(model, image)
        
        for det in detections:
            all_predictions.append((det.conf, det.xyxy))
            total_pred += 1
        
        if (i + 1) % 200 == 0:
            print(f"  [{i+1}/{len(image_files)}] GT: {total_gt}, Pred: {total_pred}")
    
    print(f"[INFO] Total ground truths: {total_gt}")
    print(f"[INFO] Total predictions: {total_pred}")
    
    # Compute AP at each IoU threshold
    metrics = AccuracyMetrics(
        model_name=model_name,
        num_images=len(image_files),
        num_ground_truth=total_gt,
        num_predictions=total_pred,
        iou_thresholds=iou_thresholds,
    )
    
    # For overall TP/FP/FN, use IoU=0.5
    for iou_thresh in iou_thresholds:
        # For mAP, we need to compute AP per image and average
        # Simplified: compute global AP
        ap, tp, fp, fn = compute_ap(all_predictions, all_ground_truths, iou_thresh)
        metrics.ap_per_threshold[iou_thresh] = ap
        
        if iou_thresh == 0.5:
            metrics.true_positives = tp
            metrics.false_positives = fp
            metrics.false_negatives = fn
        
        print(f"  AP@{iou_thresh:.2f}: {ap:.4f}")
    
    summary = metrics.summary()
    print(f"\n[RESULT] {model_name}:")
    print(f"  mAP@0.5:      {summary['mAP_0.5']:.4f}")
    print(f"  mAP@0.5:0.95: {summary['mAP_0.5_0.95']:.4f}")
    print(f"  Precision:    {summary['precision']:.4f}")
    print(f"  Recall:       {summary['recall']:.4f}")
    print(f"  F1 Score:     {summary['f1_score']:.4f}")
    
    return metrics


def generate_markdown_report(
    perf_results: Dict[str, PerformanceMetrics],
    acc_results: Dict[str, AccuracyMetrics],
    output_path: Path,
) -> str:
    """Generate a paper-ready markdown comparison table."""
    
    lines = [
        "# RF-DETR vs YOLOv8 Benchmark Results",
        "",
        f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"**Platform:** NVIDIA Jetson Orin Nano (16GB), JetPack 6.x",
        "",
        "## Performance Comparison",
        "",
        "| Metric | YOLOv8 (TRT) | RF-DETR (TRT) | Difference |",
        "|--------|--------------|---------------|------------|",
    ]
    
    yolo_perf = perf_results.get("YOLOv8", PerformanceMetrics("N/A", "", 0, 0, 0)).summary() if "YOLOv8" in perf_results else {}
    rfdetr_perf = perf_results.get("RF-DETR", PerformanceMetrics("N/A", "", 0, 0, 0)).summary() if "RF-DETR" in perf_results else {}
    
    def fmt_diff(yolo_val, rfdetr_val, higher_better=True):
        if yolo_val and rfdetr_val:
            diff = rfdetr_val - yolo_val
            sign = "+" if diff > 0 else ""
            better = (diff > 0) == higher_better
            return f"{sign}{diff:.1f} {'*' if better else ''}"
        return "N/A"
    
    metrics_to_compare = [
        ("Input Size", "input_size", "", False),
        ("FPS (mean)", "fps_mean", "fps", True),
        ("Latency mean (ms)", "latency_ms_mean", "ms", False),
        ("Latency p95 (ms)", "latency_ms_p95", "ms", False),
        ("Power (W)", "power_w_mean", "W", False),
        ("GPU Temp max (C)", "gpu_temp_c_max", "C", False),
        ("RAM (MB)", "ram_mb_mean", "MB", False),
    ]
    
    for label, key, unit, higher_better in metrics_to_compare:
        yolo_val = yolo_perf.get(key, "N/A")
        rfdetr_val = rfdetr_perf.get(key, "N/A")
        
        if isinstance(yolo_val, (int, float)) and isinstance(rfdetr_val, (int, float)):
            diff = fmt_diff(yolo_val, rfdetr_val, higher_better)
        else:
            diff = "N/A"
        
        lines.append(f"| {label} | {yolo_val} | {rfdetr_val} | {diff} |")
    
    lines.extend([
        "",
        "_* indicates better performance_",
        "",
        "## Accuracy Comparison",
        "",
        "| Metric | YOLOv8 | RF-DETR | Difference |",
        "|--------|--------|---------|------------|",
    ])
    
    yolo_acc = acc_results.get("YOLOv8", AccuracyMetrics("N/A", 0, 0, 0)).summary() if "YOLOv8" in acc_results else {}
    rfdetr_acc = acc_results.get("RF-DETR", AccuracyMetrics("N/A", 0, 0, 0)).summary() if "RF-DETR" in acc_results else {}
    
    acc_metrics = [
        ("mAP@0.5", "mAP_0.5", True),
        ("mAP@0.5:0.95", "mAP_0.5_0.95", True),
        ("Precision", "precision", True),
        ("Recall", "recall", True),
        ("F1 Score", "f1_score", True),
    ]
    
    for label, key, higher_better in acc_metrics:
        yolo_val = yolo_acc.get(key, "N/A")
        rfdetr_val = rfdetr_acc.get(key, "N/A")
        
        if isinstance(yolo_val, (int, float)) and isinstance(rfdetr_val, (int, float)):
            diff = rfdetr_val - yolo_val
            sign = "+" if diff > 0 else ""
            better = diff > 0
            diff_str = f"{sign}{diff:.4f} {'*' if better else ''}"
        else:
            diff_str = "N/A"
        
        lines.append(f"| {label} | {yolo_val} | {rfdetr_val} | {diff_str} |")
    
    lines.extend([
        "",
        "## Configuration",
        "",
        f"- **YOLOv8 Engine:** `{YOLOV8_ENGINE}`",
        f"- **RF-DETR Engine:** `{RFDETR_ENGINE}`",
        f"- **Validation Images:** {VALID_IMAGES}",
        f"- **Warmup Iterations:** {WARMUP_ITERATIONS}",
        f"- **Timed Iterations:** {PERF_ITERATIONS}",
        "",
    ])
    
    content = "\n".join(lines)
    
    with open(output_path, 'w') as f:
        f.write(content)
    
    return content


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="RF-DETR vs YOLOv8 Benchmark")
    parser.add_argument("--perf-only", action="store_true", help="Skip accuracy benchmark")
    parser.add_argument("--accuracy-only", action="store_true", help="Skip performance benchmark")
    parser.add_argument("--num-images", type=int, default=None, help="Limit images for accuracy test")
    parser.add_argument("--warmup", type=int, default=WARMUP_ITERATIONS, help="Warmup iterations")
    parser.add_argument("--iters", type=int, default=PERF_ITERATIONS, help="Timed iterations")
    parser.add_argument("--skip-yolo", action="store_true", help="Skip YOLOv8 benchmark")
    parser.add_argument("--skip-rfdetr", action="store_true", help="Skip RF-DETR benchmark")
    args = parser.parse_args()
    
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    
    print("=" * 70)
    print("RF-DETR vs YOLOv8 Comprehensive Benchmark")
    print("=" * 70)
    print(f"Timestamp: {timestamp}")
    print(f"YOLOv8 Engine: {YOLOV8_ENGINE}")
    print(f"RF-DETR Engine: {RFDETR_ENGINE}")
    print(f"Dataset: {DATASET_PATH}")
    print()
    
    # Verify files exist
    if not args.skip_yolo and not YOLOV8_ENGINE.exists():
        print(f"[ERROR] YOLOv8 engine not found: {YOLOV8_ENGINE}")
        return 1
    
    if not args.skip_rfdetr and not RFDETR_ENGINE.exists():
        print(f"[ERROR] RF-DETR engine not found: {RFDETR_ENGINE}")
        return 1
    
    # Results storage
    perf_results: Dict[str, PerformanceMetrics] = {}
    acc_results: Dict[str, AccuracyMetrics] = {}
    
    # Load models
    models = {}
    
    if not args.skip_yolo:
        yolo_model = load_yolov8_detector(YOLOV8_ENGINE, YOLOV8_INPUT_SIZE)
        models["YOLOv8"] = {
            "model": yolo_model,
            "infer": lambda m, img: infer_yolov8(m, img, YOLOV8_INPUT_SIZE),
            "input_size": YOLOV8_INPUT_SIZE,
            "path": str(YOLOV8_ENGINE),
        }
    
    if not args.skip_rfdetr:
        rfdetr_model = load_rfdetr_detector(RFDETR_ENGINE, RFDETR_INPUT_SIZE)
        models["RF-DETR"] = {
            "model": rfdetr_model,
            "infer": lambda m, img: infer_rfdetr(m, img),
            "input_size": RFDETR_INPUT_SIZE,
            "path": str(RFDETR_ENGINE),
        }
    
    # Run performance benchmarks
    if not args.accuracy_only:
        for name, cfg in models.items():
            metrics = run_performance_benchmark(
                model_name=name,
                model=cfg["model"],
                infer_func=cfg["infer"],
                input_size=cfg["input_size"],
                model_path=cfg["path"],
                warmup_iters=args.warmup,
                timed_iters=args.iters,
            )
            perf_results[name] = metrics
    
    # Run accuracy benchmarks
    if not args.perf_only:
        for name, cfg in models.items():
            metrics = run_accuracy_benchmark(
                model_name=name,
                model=cfg["model"],
                infer_func=cfg["infer"],
                input_size=cfg["input_size"],
                images_dir=VALID_IMAGES,
                labels_dir=VALID_LABELS,
                num_images=args.num_images,
            )
            acc_results[name] = metrics
    
    # Compile results
    results = {
        "timestamp": timestamp,
        "platform": "NVIDIA Jetson Orin Nano 16GB",
        "jetpack": "6.x",
        "config": {
            "warmup_iterations": args.warmup,
            "timed_iterations": args.iters,
            "num_validation_images": args.num_images or "all",
        },
        "performance": {name: m.summary() for name, m in perf_results.items()},
        "accuracy": {name: m.summary() for name, m in acc_results.items()},
    }
    
    # Save JSON
    json_path = OUTPUT_DIR / f"benchmark_{timestamp}.json"
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n[INFO] JSON results saved to: {json_path}")
    
    # Generate markdown report
    md_path = OUTPUT_DIR / f"benchmark_{timestamp}.md"
    md_content = generate_markdown_report(perf_results, acc_results, md_path)
    print(f"[INFO] Markdown report saved to: {md_path}")
    
    # Print summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(md_content)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
