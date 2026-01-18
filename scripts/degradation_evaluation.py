#!/usr/bin/env python3
"""
Degradation Robustness Evaluation for AIRHOUND.

This script evaluates RF-DETR and YOLOv8 detection accuracy under various
synthetic degradation conditions, generating data for the SPIE paper on
"Machine Learning from Challenging Data".

Usage:
    python3 degradation_evaluation.py --models both --degradations all --severities 5
    
Outputs:
    - JSON results with mAP at each degradation level
    - Comparison plots for the paper

Author: AIRHOUND Team (EPPL Lab, Embry-Riddle)
Date: January 2026
"""

from __future__ import annotations

import sys
import json
import time
import argparse
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional, Tuple
from datetime import datetime

import cv2
import numpy as np

# Add paths for local modules
sys.path.insert(0, str(Path.home() / "AIRHOUND" / "scripts"))
sys.path.insert(0, str(Path.home() / "AIRHOUND" / "ws_ros2" / "src" / "airhound_perception"))
sys.path.insert(0, str(Path.home() / "dev_workspace" / "airhoundPerception"))

from synthetic_degradation import DegradationPipeline, DegradationType


@dataclass
class EvaluationResult:
    """Results for a single evaluation run."""
    model: str
    degradation_type: str
    severity: float
    num_images: int
    num_gt_boxes: int
    mAP_0_5: float
    mAP_0_5_0_95: float
    precision: float
    recall: float
    f1: float
    eval_time_seconds: float


class DetectorWrapper:
    """Unified interface for different detectors."""
    
    def __init__(self, model_type: str, conf_threshold: float = 0.001):
        self.model_type = model_type
        self.conf_threshold = conf_threshold
        self.model = None
        self._load_model()
    
    def _load_model(self):
        if self.model_type == "yolov8":
            self._load_yolov8()
        elif self.model_type == "rfdetr":
            self._load_rfdetr()
        else:
            raise ValueError(f"Unknown model type: {self.model_type}")
    
    def _load_yolov8(self):
        """Load YOLOv8 model."""
        from ultralytics import YOLO
        model_path = Path.home() / "dev_workspace" / "airhoundPerception" / "models" / "yolov8Detector.pt"
        self.model = YOLO(str(model_path))
        print(f"[YOLOv8] Loaded model from {model_path}")
    
    def _load_rfdetr(self):
        """Load RF-DETR ONNX model."""
        import onnxruntime as ort
        model_path = Path.home() / "AIRHOUND" / "models" / "drone_rfdetr_best.onnx"
        
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 6
        session_options.inter_op_num_threads = 6
        
        self.model = ort.InferenceSession(
            str(model_path),
            sess_options=session_options,
            providers=["CPUExecutionProvider"],  # CPU since GPU ORT not available
        )
        self.input_name = self.model.get_inputs()[0].name
        self.input_size = 560
        print(f"[RF-DETR] Loaded ONNX model from {model_path}")
    
    def predict(self, image: np.ndarray) -> List[Dict]:
        """
        Run inference and return detections.
        
        Returns list of dicts: [{"box": [x1,y1,x2,y2], "score": float}, ...]
        """
        if self.model_type == "yolov8":
            return self._predict_yolov8(image)
        elif self.model_type == "rfdetr":
            return self._predict_rfdetr(image)
        return []
    
    def _predict_yolov8(self, image: np.ndarray) -> List[Dict]:
        """Run YOLOv8 inference."""
        results = self.model(image, conf=self.conf_threshold, verbose=False)
        
        detections = []
        for result in results:
            boxes = result.boxes
            for i in range(len(boxes)):
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                detections.append({
                    "box": [float(x1), float(y1), float(x2), float(y2)],
                    "score": conf,
                })
        
        return sorted(detections, key=lambda x: x["score"], reverse=True)
    
    def _predict_rfdetr(self, image: np.ndarray) -> List[Dict]:
        """Run RF-DETR ONNX inference."""
        orig_h, orig_w = image.shape[:2]
        
        # Preprocess
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (self.input_size, self.input_size))
        img_norm = img_resized.astype(np.float32) / 255.0
        img_tensor = np.transpose(img_norm, (2, 0, 1))[None]
        img_tensor = np.ascontiguousarray(img_tensor)
        
        # Inference
        outputs = self.model.run(None, {self.input_name: img_tensor})
        
        # Postprocess
        # outputs[0] = dets: (1, 300, 4) - boxes in cxcywh normalized
        # outputs[1] = labels: (1, 300, 1) - class logits
        boxes = outputs[0][0]  # (300, 4)
        logits = outputs[1][0]  # (300, 1)
        
        # Sigmoid for confidence
        scores = 1 / (1 + np.exp(-logits.flatten()))
        
        detections = []
        for box, score in zip(boxes, scores):
            if score < self.conf_threshold:
                continue
            
            cx, cy, w, h = box
            cx *= orig_w
            cy *= orig_h
            w *= orig_w
            h *= orig_h
            
            x1 = max(0, cx - w/2)
            y1 = max(0, cy - h/2)
            x2 = min(orig_w, cx + w/2)
            y2 = min(orig_h, cy + h/2)
            
            detections.append({
                "box": [x1, y1, x2, y2],
                "score": float(score),
            })
        
        return sorted(detections, key=lambda x: x["score"], reverse=True)


def load_yolo_labels(label_path: Path, img_w: int, img_h: int) -> List[List[float]]:
    """Load YOLO format labels and convert to xyxy."""
    boxes = []
    if not label_path.exists():
        return boxes
    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
                cls, cx, cy, w, h = map(float, parts[:5])
                x1 = (cx - w/2) * img_w
                y1 = (cy - h/2) * img_h
                x2 = (cx + w/2) * img_w
                y2 = (cy + h/2) * img_h
                boxes.append([x1, y1, x2, y2])
    return boxes


def compute_iou(box1: List[float], box2: List[float]) -> float:
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


def compute_ap(tp: List[int], fp: List[int], scores: List[float], total_gt: int) -> float:
    """Compute Average Precision using 101-point interpolation."""
    if total_gt == 0 or len(scores) == 0:
        return 0.0
    
    indices = np.argsort(scores)[::-1]
    tp = np.array(tp)[indices]
    fp = np.array(fp)[indices]
    
    tp_cumsum = np.cumsum(tp)
    fp_cumsum = np.cumsum(fp)
    
    precision = tp_cumsum / (tp_cumsum + fp_cumsum)
    recall = tp_cumsum / total_gt
    
    # 101-point interpolation
    ap = 0
    for t in np.linspace(0, 1, 101):
        prec_at_recall = precision[recall >= t]
        if len(prec_at_recall) > 0:
            ap += np.max(prec_at_recall)
    ap /= 101
    
    return ap


def evaluate_detector(
    detector: DetectorWrapper,
    images_dir: Path,
    labels_dir: Path,
    degradation_pipeline: Optional[DegradationPipeline],
    degradation_type: str,
    severity: float,
    max_images: Optional[int] = None,
) -> EvaluationResult:
    """
    Evaluate a detector on a dataset with optional degradation.
    """
    image_files = sorted(images_dir.glob("*.jpg"))
    if max_images:
        image_files = image_files[:max_images]
    
    # IoU thresholds for mAP calculation
    iou_thresholds = [0.5 + 0.05 * i for i in range(10)]
    
    # Store results for each IoU threshold
    results_by_iou = {iou: {"tp": [], "fp": [], "scores": [], "total_gt": 0} for iou in iou_thresholds}
    
    total_gt = 0
    start_time = time.time()
    
    for idx, img_path in enumerate(image_files):
        # Load and optionally degrade image
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        
        if degradation_pipeline and severity > 0:
            img = degradation_pipeline.apply(img, degradation_type, severity)
        
        orig_h, orig_w = img.shape[:2]
        
        # Load ground truth
        label_path = labels_dir / (img_path.stem + ".txt")
        gt_boxes = load_yolo_labels(label_path, orig_w, orig_h)
        total_gt += len(gt_boxes)
        
        # Run inference
        detections = detector.predict(img)
        
        # Match predictions to ground truth for each IoU threshold
        for iou_thresh in iou_thresholds:
            results_by_iou[iou_thresh]["total_gt"] += len(gt_boxes)
            gt_matched = [False] * len(gt_boxes)
            
            for det in detections:
                pred_box = det["box"]
                best_iou = 0
                best_gt_idx = -1
                
                for gt_idx, gt_box in enumerate(gt_boxes):
                    if gt_matched[gt_idx]:
                        continue
                    iou = compute_iou(pred_box, gt_box)
                    if iou > best_iou:
                        best_iou = iou
                        best_gt_idx = gt_idx
                
                if best_iou >= iou_thresh and best_gt_idx >= 0:
                    results_by_iou[iou_thresh]["tp"].append(1)
                    results_by_iou[iou_thresh]["fp"].append(0)
                    gt_matched[best_gt_idx] = True
                else:
                    results_by_iou[iou_thresh]["tp"].append(0)
                    results_by_iou[iou_thresh]["fp"].append(1)
                
                results_by_iou[iou_thresh]["scores"].append(det["score"])
        
        if (idx + 1) % 100 == 0:
            print(f"    [{idx + 1}/{len(image_files)}]")
    
    eval_time = time.time() - start_time
    
    # Compute mAP@0.5
    ap_50 = compute_ap(
        results_by_iou[0.5]["tp"],
        results_by_iou[0.5]["fp"],
        results_by_iou[0.5]["scores"],
        results_by_iou[0.5]["total_gt"]
    )
    
    # Compute mAP@0.5:0.95
    aps = []
    for iou in iou_thresholds:
        ap = compute_ap(
            results_by_iou[iou]["tp"],
            results_by_iou[iou]["fp"],
            results_by_iou[iou]["scores"],
            results_by_iou[iou]["total_gt"]
        )
        aps.append(ap)
    map_50_95 = np.mean(aps)
    
    # Compute P/R/F1 at conf=0.25
    conf_thresh = 0.25
    tp_conf = 0
    fp_conf = 0
    fn_conf = 0
    
    for idx, img_path in enumerate(image_files):
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        
        if degradation_pipeline and severity > 0:
            img = degradation_pipeline.apply(img, degradation_type, severity)
        
        orig_h, orig_w = img.shape[:2]
        label_path = labels_dir / (img_path.stem + ".txt")
        gt_boxes = load_yolo_labels(label_path, orig_w, orig_h)
        
        detections = [d for d in detector.predict(img) if d["score"] >= conf_thresh]
        
        gt_matched = [False] * len(gt_boxes)
        for det in detections:
            pred_box = det["box"]
            best_iou = 0
            best_gt_idx = -1
            
            for gt_idx, gt_box in enumerate(gt_boxes):
                if gt_matched[gt_idx]:
                    continue
                iou = compute_iou(pred_box, gt_box)
                if iou > best_iou:
                    best_iou = iou
                    best_gt_idx = gt_idx
            
            if best_iou >= 0.5 and best_gt_idx >= 0:
                tp_conf += 1
                gt_matched[best_gt_idx] = True
            else:
                fp_conf += 1
        
        fn_conf += sum(1 for m in gt_matched if not m)
    
    precision = tp_conf / (tp_conf + fp_conf) if (tp_conf + fp_conf) > 0 else 0
    recall = tp_conf / (tp_conf + fn_conf) if (tp_conf + fn_conf) > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0
    
    return EvaluationResult(
        model=detector.model_type,
        degradation_type=degradation_type,
        severity=severity,
        num_images=len(image_files),
        num_gt_boxes=total_gt,
        mAP_0_5=ap_50,
        mAP_0_5_0_95=map_50_95,
        precision=precision,
        recall=recall,
        f1=f1,
        eval_time_seconds=eval_time,
    )


def run_full_evaluation(
    models: List[str],
    degradation_types: List[str],
    severity_levels: List[float],
    images_dir: Path,
    labels_dir: Path,
    output_dir: Path,
    max_images: Optional[int] = None,
) -> List[EvaluationResult]:
    """Run full evaluation across all models, degradations, and severities."""
    
    output_dir.mkdir(parents=True, exist_ok=True)
    pipeline = DegradationPipeline(seed=42)
    
    results = []
    total_runs = len(models) * len(degradation_types) * len(severity_levels)
    run_idx = 0
    
    for model_type in models:
        print(f"\n{'='*60}")
        print(f"Loading {model_type.upper()} model...")
        detector = DetectorWrapper(model_type)
        
        for deg_type in degradation_types:
            for severity in severity_levels:
                run_idx += 1
                print(f"\n[{run_idx}/{total_runs}] {model_type} | {deg_type} | severity={severity:.2f}")
                
                result = evaluate_detector(
                    detector=detector,
                    images_dir=images_dir,
                    labels_dir=labels_dir,
                    degradation_pipeline=pipeline if deg_type != "clean" else None,
                    degradation_type=deg_type if deg_type != "clean" else "motion_blur",
                    severity=severity if deg_type != "clean" else 0.0,
                    max_images=max_images,
                )
                
                results.append(result)
                print(f"    mAP@0.5: {result.mAP_0_5:.4f} | P: {result.precision:.4f} | R: {result.recall:.4f}")
                
                # Save intermediate results
                with open(output_dir / "evaluation_results.json", "w") as f:
                    json.dump([asdict(r) for r in results], f, indent=2)
    
    return results


def generate_comparison_table(results: List[EvaluationResult]) -> str:
    """Generate a markdown comparison table from results."""
    
    # Group by degradation type
    by_degradation = {}
    for r in results:
        key = (r.degradation_type, r.severity)
        if key not in by_degradation:
            by_degradation[key] = {}
        by_degradation[key][r.model] = r
    
    table = "| Degradation | Severity | YOLOv8 mAP@0.5 | RF-DETR mAP@0.5 | Δ |\n"
    table += "|-------------|----------|----------------|-----------------|---|\n"
    
    for (deg_type, severity), models in sorted(by_degradation.items()):
        yolo_map = models.get("yolov8", None)
        rfdetr_map = models.get("rfdetr", None)
        
        yolo_val = f"{yolo_map.mAP_0_5:.4f}" if yolo_map else "N/A"
        rfdetr_val = f"{rfdetr_map.mAP_0_5:.4f}" if rfdetr_map else "N/A"
        
        if yolo_map and rfdetr_map:
            delta = rfdetr_map.mAP_0_5 - yolo_map.mAP_0_5
            delta_str = f"{delta:+.4f}"
        else:
            delta_str = "N/A"
        
        table += f"| {deg_type} | {severity:.2f} | {yolo_val} | {rfdetr_val} | {delta_str} |\n"
    
    return table


def main():
    parser = argparse.ArgumentParser(description="Degradation Robustness Evaluation")
    parser.add_argument("--models", nargs="+", default=["yolov8", "rfdetr"],
                        choices=["yolov8", "rfdetr"], help="Models to evaluate")
    parser.add_argument("--degradations", nargs="+", 
                        default=["clean", "motion_blur", "gaussian_noise", "occlusion", "low_light"],
                        help="Degradation types to test")
    parser.add_argument("--severities", type=int, default=5,
                        help="Number of severity levels (0.0 to 1.0)")
    parser.add_argument("--max-images", type=int, default=None,
                        help="Maximum images to evaluate per condition")
    parser.add_argument("--output", type=str, default=None,
                        help="Output directory")
    
    args = parser.parse_args()
    
    # Paths
    images_dir = Path.home() / "AIRHOUND" / "data" / "valid" / "images"
    labels_dir = Path.home() / "AIRHOUND" / "data" / "valid" / "labels"
    
    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = Path.home() / "AIRHOUND" / "benchmark_results" / f"degradation_eval_{datetime.now().strftime('%Y-%m-%d_%H-%M')}"
    
    # Generate severity levels
    if args.severities == 1:
        severity_levels = [0.5]
    else:
        severity_levels = [i / (args.severities - 1) for i in range(args.severities)]
    
    # For "clean", we only need severity=0
    degradation_types = args.degradations
    
    print("=" * 60)
    print("DEGRADATION ROBUSTNESS EVALUATION")
    print("=" * 60)
    print(f"Models: {args.models}")
    print(f"Degradations: {degradation_types}")
    print(f"Severity levels: {severity_levels}")
    print(f"Images directory: {images_dir}")
    print(f"Max images: {args.max_images or 'all'}")
    print(f"Output: {output_dir}")
    print("=" * 60)
    
    # Run evaluation
    results = run_full_evaluation(
        models=args.models,
        degradation_types=degradation_types,
        severity_levels=severity_levels,
        images_dir=images_dir,
        labels_dir=labels_dir,
        output_dir=output_dir,
        max_images=args.max_images,
    )
    
    # Generate summary
    print("\n" + "=" * 60)
    print("EVALUATION COMPLETE")
    print("=" * 60)
    
    table = generate_comparison_table(results)
    print("\n" + table)
    
    # Save table
    with open(output_dir / "comparison_table.md", "w") as f:
        f.write("# Degradation Robustness Comparison\n\n")
        f.write(f"Generated: {datetime.now().isoformat()}\n\n")
        f.write(table)
    
    print(f"\nResults saved to: {output_dir}")


if __name__ == "__main__":
    main()
