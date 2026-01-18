# Degradation Robustness Evaluation Results

## AIRHOUND - SPIE Defense + Security 2026 (DS112)
### Machine Learning from Challenging Data

**Evaluation Date:** January 17-18, 2026  
**Duration:** ~17 hours  
**Dataset:** 1,983 images with drone annotations  
**Models:** RF-DETR (ONNX) vs YOLOv8n (PyTorch)

---

## Executive Summary

RF-DETR demonstrates **significantly superior robustness** to image degradation compared to YOLOv8, particularly under:
- **Gaussian noise:** Up to +40% mAP advantage at severe levels
- **Low light:** Up to +39% mAP advantage at severe levels
- **Motion blur:** Consistent advantage, though both models struggle at high severity

This finding directly supports the SPIE DS112 theme: **"Machine Learning from Challenging Data"**.

---

## Key Findings

### 1. Gaussian Noise (Primary Finding)

| Severity | YOLOv8 mAP@0.5 | RF-DETR mAP@0.5 | RF-DETR Advantage |
|----------|----------------|-----------------|-------------------|
| 0.00 (clean) | 0.957 | 0.953 | -0.4% |
| 0.25 | 0.904 | 0.937 | +3.2% |
| 0.50 | 0.726 | 0.885 | **+15.9%** |
| 0.75 | 0.504 | 0.807 | **+30.3%** |
| 1.00 | 0.318 | 0.716 | **+39.8%** |

**Interpretation:** RF-DETR's transformer attention mechanism provides remarkable noise resilience. At maximum noise severity, RF-DETR retains 75% of its clean performance while YOLOv8 drops to 33%.

### 2. Low Light Conditions

| Severity | YOLOv8 mAP@0.5 | RF-DETR mAP@0.5 | RF-DETR Advantage |
|----------|----------------|-----------------|-------------------|
| 0.00 (clean) | 0.957 | 0.953 | -0.4% |
| 0.25 | 0.946 | 0.936 | -1.0% |
| 0.50 | 0.837 | 0.888 | **+5.1%** |
| 0.75 | 0.566 | 0.804 | **+23.9%** |
| 1.00 | 0.275 | 0.670 | **+39.4%** |

**Interpretation:** Similar pattern to noise - RF-DETR maintains strong performance in challenging lighting while YOLOv8 degrades rapidly beyond 50% severity.

### 3. Motion Blur

| Severity | YOLOv8 mAP@0.5 | RF-DETR mAP@0.5 | RF-DETR Advantage |
|----------|----------------|-----------------|-------------------|
| 0.00 (clean) | 0.957 | 0.953 | -0.4% |
| 0.25 | 0.584 | 0.723 | **+13.9%** |
| 0.50 | 0.256 | 0.334 | +7.8% |
| 0.75 | 0.148 | 0.172 | +2.4% |
| 1.00 | 0.076 | 0.094 | +1.9% |

**Interpretation:** Motion blur is challenging for both architectures. RF-DETR maintains an advantage but both models fall below usable thresholds at high severity. This motivates the need for complementary approaches (Kalman filter, PINN prediction) during motion blur events.

### 4. Occlusion

| Severity | YOLOv8 mAP@0.5 | RF-DETR mAP@0.5 | RF-DETR Advantage |
|----------|----------------|-----------------|-------------------|
| 0.00 (clean) | 0.957 | 0.953 | -0.4% |
| 0.25 | 0.948 | 0.935 | -1.3% |
| 0.50 | 0.915 | 0.916 | +0.1% |
| 0.75 | 0.873 | 0.869 | -0.5% |
| 1.00 | 0.753 | 0.778 | +2.5% |

**Interpretation:** Both models handle occlusion well, with similar performance. This suggests occlusion handling is less dependent on architecture and more on training data augmentation.

---

## Clean Baseline Comparison

| Metric | YOLOv8 | RF-DETR | Winner |
|--------|--------|---------|--------|
| mAP@0.5 | 0.957 | 0.953 | YOLOv8 (+0.4%) |
| mAP@0.5:0.95 | 0.584 | 0.541 | YOLOv8 (+4.3%) |
| Precision | 0.922 | 0.949 | RF-DETR (+2.7%) |
| Recall | 0.971 | 0.931 | YOLOv8 (+4.0%) |
| F1 Score | 0.946 | 0.940 | YOLOv8 (+0.6%) |

**Note:** YOLOv8 has a slight edge on clean data, but this advantage disappears and reverses under degradation.

---

## Degradation-Averaged Performance

Average mAP@0.5 across all severity levels (0.0-1.0):

| Degradation Type | YOLOv8 Avg | RF-DETR Avg | Winner |
|------------------|------------|-------------|--------|
| Gaussian Noise | 0.682 | **0.859** | RF-DETR (+17.7%) |
| Low Light | 0.716 | **0.850** | RF-DETR (+13.4%) |
| Motion Blur | 0.404 | **0.455** | RF-DETR (+5.1%) |
| Occlusion | 0.889 | 0.890 | Tie |

---

## Paper Narrative

### For SPIE DS112 Submission

> "Our evaluation demonstrates that RF-DETR's transformer-based attention mechanism provides substantial robustness advantages over CNN-based YOLOv8 under challenging visual conditions. At severe Gaussian noise levels (σ = 1.0), RF-DETR achieves 71.6% mAP@0.5 compared to YOLOv8's 31.8% — a relative improvement of 125%. Similar advantages are observed under low-light conditions. This robustness is critical for autonomous drone pursuit in real-world environments where sensor noise, varying illumination, and motion artifacts are common."

### Key Claims Supported by Data

1. **RF-DETR outperforms YOLOv8 under degradation** - Strongly supported (up to 40% advantage)
2. **Transformer attention provides noise resilience** - Supported by gaussian noise results
3. **Both models require prediction during severe blur** - Supported (both <20% mAP at 0.75 blur)
4. **Degradation robustness motivates PINN integration** - Supported (dropout recovery needed)

---

## Generated Artifacts

### Plots (in `benchmark_results/degradation_eval_2026-01-17_18-18/`)

| File | Description | Use in Paper |
|------|-------------|--------------|
| `degradation_comparison_final.png/pdf` | 4-panel line plots by degradation type | Figure: Degradation Comparison |
| `delta_heatmap_final.png/pdf` | Heatmap of RF-DETR advantage | Figure: Advantage Heatmap |
| `severe_degradation_comparison.png/pdf` | Bar chart at severity ≥0.75 | Figure: Severe Conditions |

### Data Files

| File | Description |
|------|-------------|
| `evaluation_results.json` | Raw evaluation metrics (JSON) |
| `comparison_table.md` | Markdown table of all results |
| `evaluation.log` | Execution log with timing |

---

## Evaluation Configuration

```yaml
models:
  - yolov8:
      path: ~/dev_workspace/airhoundPerception/models/yolov8Detector.pt
      inference: PyTorch (CUDA)
  - rfdetr:
      path: ~/AIRHOUND/models/drone_rfdetr_best.onnx
      inference: ONNX Runtime (CPU)  # Note: GPU ORT not available

degradations:
  - gaussian_noise: [0.0, 0.25, 0.5, 0.75, 1.0]
  - motion_blur: [0.0, 0.25, 0.5, 0.75, 1.0]
  - low_light: [0.0, 0.25, 0.5, 0.75, 1.0]
  - occlusion: [0.0, 0.25, 0.5, 0.75, 1.0]

dataset:
  images: 1983
  annotations: YOLO format
  source: Combined drone detection dataset
```

---

## Next Steps

1. **TensorRT Conversion** - Convert RF-DETR to TensorRT for Jetson deployment (Week 2)
2. **Latency Benchmarking** - Compare inference speed on Jetson Orin
3. **Integration Testing** - Verify RF-DETR works in ROS2 pipeline
4. **Paper Figures** - Finalize plots for manuscript

---

## Acknowledgments

Evaluation performed on development workstation with:
- Ubuntu 22.04
- Python 3.10
- PyTorch 2.x (YOLOv8)
- ONNX Runtime 1.x (RF-DETR)

---

*Generated: January 18, 2026*  
*AIRHOUND Team - EPPL Lab, Embry-Riddle Aeronautical University*
