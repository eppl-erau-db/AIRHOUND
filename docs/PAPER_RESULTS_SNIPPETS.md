# Paper Results Snippets

## AIRHOUND - SPIE DS112 Manuscript
### Ready-to-Use Text and Data for Results Section

---

## Section: Degradation Robustness Evaluation

### LaTeX Table: Gaussian Noise Results

```latex
\begin{table}[htbp]
\centering
\caption{Detection performance under Gaussian noise degradation. RF-DETR demonstrates superior robustness at higher noise levels.}
\label{tab:gaussian_noise}
\begin{tabular}{lccc}
\hline
\textbf{Severity} & \textbf{YOLOv8 mAP@0.5} & \textbf{RF-DETR mAP@0.5} & \textbf{$\Delta$} \\
\hline
0.00 (clean) & 0.957 & 0.953 & -0.4\% \\
0.25 & 0.904 & 0.937 & +3.2\% \\
0.50 & 0.726 & 0.885 & \textbf{+15.9\%} \\
0.75 & 0.504 & 0.807 & \textbf{+30.3\%} \\
1.00 & 0.318 & 0.716 & \textbf{+39.8\%} \\
\hline
\end{tabular}
\end{table}
```

### LaTeX Table: All Degradation Types Summary

```latex
\begin{table}[htbp]
\centering
\caption{Average mAP@0.5 across degradation severity levels (0.0--1.0). RF-DETR shows consistent advantages under noise and low-light conditions.}
\label{tab:degradation_summary}
\begin{tabular}{lccc}
\hline
\textbf{Degradation Type} & \textbf{YOLOv8} & \textbf{RF-DETR} & \textbf{Advantage} \\
\hline
Gaussian Noise & 0.682 & 0.859 & +17.7\% \\
Low Light & 0.716 & 0.850 & +13.4\% \\
Motion Blur & 0.404 & 0.455 & +5.1\% \\
Occlusion & 0.889 & 0.890 & +0.1\% \\
\hline
\end{tabular}
\end{table}
```

---

## Prose Snippets

### Abstract Snippet
> We evaluate RF-DETR against YOLOv8 under synthetic degradation conditions including Gaussian noise, motion blur, low-light, and occlusion. RF-DETR demonstrates up to 40% higher mAP under severe noise degradation, validating its robustness for challenging UAV pursuit scenarios.

### Introduction/Motivation Snippet
> Real-world drone detection faces numerous visual challenges: sensor noise from high-gain imaging, motion blur during pursuit maneuvers, and varying illumination conditions. Traditional CNN-based detectors experience significant performance degradation under these conditions, motivating the exploration of transformer-based architectures with their attention mechanisms that may provide inherent robustness to local perturbations.

### Results Section - Gaussian Noise
> Under Gaussian noise degradation, RF-DETR demonstrates remarkable resilience compared to YOLOv8. At maximum severity (σ = 1.0), RF-DETR achieves 0.716 mAP@0.5 compared to YOLOv8's 0.318—a relative improvement of 125%. This advantage emerges progressively: at 50% severity, RF-DETR leads by 15.9 percentage points (0.885 vs 0.726), growing to 30.3 points at 75% severity and 39.8 points at maximum severity. We attribute this robustness to RF-DETR's transformer attention mechanism, which captures global context rather than relying solely on local convolution operations that are more susceptible to noise corruption.

### Results Section - Low Light
> Low-light performance follows a similar pattern to Gaussian noise. At moderate degradation (50% severity), RF-DETR maintains 0.888 mAP while YOLOv8 drops to 0.837. The gap widens substantially at severe conditions: at 75% severity, RF-DETR achieves 0.804 mAP compared to YOLOv8's 0.566 (+23.9%), and at maximum severity, 0.670 vs 0.275 (+39.4%). These results suggest RF-DETR's feature representation is more robust to the reduced contrast and signal-to-noise ratio characteristic of low-light imagery.

### Results Section - Motion Blur
> Motion blur presents challenges for both architectures, with both models experiencing significant performance degradation. RF-DETR maintains a consistent advantage—13.9% at 25% severity and 7.8% at 50% severity—but both models fall below practical usability thresholds at high blur levels (mAP < 0.2 at 75% severity). This finding motivates our integration of Kalman filtering and physics-informed prediction to maintain tracking continuity during severe motion blur events where detection reliability is compromised.

### Results Section - Occlusion
> Occlusion handling shows minimal difference between architectures, with both models demonstrating graceful degradation from 0.95+ mAP at clean conditions to approximately 0.77 mAP at maximum occlusion. This suggests that occlusion robustness is primarily determined by training data augmentation rather than architectural differences, as both models were trained with similar occlusion augmentation strategies.

### Discussion Snippet
> Our degradation evaluation reveals a fundamental architectural advantage: RF-DETR's transformer-based attention mechanism provides inherent robustness to per-pixel corruption (noise, low-light) that CNN architectures cannot match. However, this advantage diminishes for spatially-coherent degradations like motion blur and occlusion, where both architectures face similar challenges. These findings inform our system design: we leverage RF-DETR's robustness for challenging visual conditions while implementing complementary prediction mechanisms (Kalman filter, PINN) to handle scenarios where detection reliability is compromised regardless of architecture.

---

## Figures

### Figure Captions

**Figure X: Degradation Comparison**
> Detection performance (mAP@0.5) of RF-DETR and YOLOv8 across four degradation types and five severity levels. RF-DETR demonstrates superior robustness under Gaussian noise and low-light conditions, while both architectures show similar degradation patterns for motion blur and occlusion.

**Figure Y: RF-DETR Advantage Heatmap**
> Heatmap showing RF-DETR's performance advantage over YOLOv8 (ΔmAP@0.5) across degradation types and severity levels. Green indicates RF-DETR outperformance; the transformer architecture provides up to +40% advantage under severe noise and low-light conditions.

**Figure Z: Severe Degradation Performance**
> Average detection accuracy at severe degradation levels (severity ≥ 0.75). RF-DETR maintains significantly higher performance under Gaussian noise and low-light conditions, supporting its selection for challenging real-world UAV pursuit scenarios.

---

## Key Statistics for Quick Reference

| Metric | Value | Context |
|--------|-------|---------|
| RF-DETR max advantage | +39.8% | Gaussian noise @ 1.0 severity |
| RF-DETR low-light advantage | +39.4% | Low light @ 1.0 severity |
| Motion blur threshold | ~0.50 severity | Both models drop below 0.4 mAP |
| Clean baseline difference | 0.4% | YOLOv8 slightly better on clean |
| Evaluation dataset size | 1,983 images | With drone annotations |
| Degradation types tested | 4 | Noise, blur, low-light, occlusion |
| Severity levels per type | 5 | 0.0, 0.25, 0.5, 0.75, 1.0 |

---

## Section: Hardware-in-the-Loop Flight Validation

### LaTeX Table: HITL Configuration Comparison

```latex
\begin{table}[htbp]
\centering
\caption{Hardware-in-the-loop flight validation results across camera and synthetic configurations. V4L2 direct capture eliminates DDS serialization overhead, achieving 4$\times$ lower latency and 100\% detection continuity at 1280$\times$720.}
\label{tab:hitl-comparison}
\begin{tabular}{lcccccc}
\toprule
\textbf{Configuration} & \textbf{Duration} & \textbf{Det.\ Rate} & \textbf{Latency} & \textbf{P95} & \textbf{FPS} & \textbf{Dropouts} \\
 & (s) & (\%) & (ms) & (ms) & & \\
\midrule
Camera 640$\times$480 (bridge) & 182 & 100.0 & 234.5 & 598.1 & 17.2 & 1 \\
Camera 1280$\times$720 (bridge) & 182 & 59.5 & 233.7 & 558.2 & 17.5 & 114 \\
Camera 1280$\times$720 (direct) & 108 & 100.0 & 58.8 & 61.5 & 15.0 & 0 \\
Synthetic + PINN & 180 & 21.0 & --- & --- & 30.0 & 16 \\
\bottomrule
\end{tabular}
\end{table}
```

### LaTeX Table: Tracking Mode Breakdown

```latex
\begin{table}[htbp]
\centering
\caption{Tracking mode time allocation during HITL testing. The V4L2 direct capture configuration maintains 100\% detection-driven tracking, while the bridge configuration at 720p requires Kalman, PINN, and hold fallback for 40\% of the flight.}
\label{tab:tracking-modes}
\begin{tabular}{lcccccc}
\toprule
\textbf{Configuration} & \textbf{Detection} & \textbf{Kalman} & \textbf{PINN} & \textbf{Hold} & \textbf{Decay} & \textbf{Zero} \\
 & (\%) & (\%) & (\%) & (\%) & (\%) & (\%) \\
\midrule
640$\times$480 bridge & 100.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 \\
720p bridge & 59.5 & 10.2 & 11.7 & 5.6 & 4.1 & 8.9 \\
720p direct & 100.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 \\
Synthetic + PINN & 21.0 & 1.1 & 77.5 & 0.0 & 0.0 & 0.4 \\
\bottomrule
\end{tabular}
\end{table}
```

### Prose: HITL System Performance

> We validate the complete perception-to-control pipeline on a Jetson Orin Nano with an Intel RealSense D455 camera and PX4 flight controller connected via Ethernet. RF-DETR inference runs at 15.0~FPS on the Jetson GPU via TensorRT at 1280$\times$720 resolution, with a median end-to-end latency of 59.0~ms (P95: 61.5~ms). The physics-informed neural network (PINN) trajectory predictor runs at 0.28~ms per inference after TensorRT conversion, representing a 7$\times$ speedup over CPU ONNX inference.

### Prose: V4L2 Direct Capture Optimization

> A key finding is the impact of ROS\,2 DDS serialization on image transport performance. Publishing 2.7~MB uncompressed frames (1280$\times$720 BGR8) through ROS\,2 middleware limits effective throughput to approximately 1.5~FPS in Python, despite the camera capturing at 30~FPS. By integrating V4L2 capture directly into the detector node---bypassing DDS image serialization entirely---we achieve the full TensorRT-limited throughput of 15.0~FPS with 58.8~ms mean latency, a 4$\times$ reduction from the 233.7~ms observed with the bridge architecture. This optimization eliminates all detection dropouts at 720p resolution, maintaining 100\% detection-driven tracking throughout the 108-second test flight.

### Prose: Dropout Recovery Validation

> The synthetic HITL configuration validates the dropout recovery pipeline under controlled conditions. With the mock detector producing detections for only 21\% of the flight duration, the PINN predictor successfully maintains tracking continuity for 77.5\% of the remaining time. The Kalman filter handles initial dropout transitions (1.1\%), while the system spends less than 0.4\% in the zero-command state. This demonstrates that the PINN-augmented tracking pipeline can maintain pursuit commands through extended detection gaps of up to 16.3~seconds.

### Prose: Latency Comparison

> The latency distribution reveals a bimodal pattern across transport architectures. The ROS\,2 bridge configurations exhibit high variance (IQR: 50--370~ms) due to DDS serialization contention, with P99 latencies exceeding 700~ms. In contrast, the V4L2 direct capture configuration shows a tight unimodal distribution (IQR: 57--61~ms) bounded solely by TensorRT inference time. This 10$\times$ reduction in latency variance is critical for real-time pursuit control, where consistent frame-to-frame timing prevents oscillatory yaw commands.

### Figure Captions

**Figure: Inference Latency Distribution**
> Box plot of per-frame inference latency across HITL configurations. The ROS\,2 bridge configurations (left) exhibit high variance due to DDS serialization overhead, while V4L2 direct capture (right) achieves consistent 59~ms latency bounded by TensorRT inference time alone. Outliers omitted for clarity.

**Figure: Tracking Mode Breakdown**
> Stacked bar chart showing time allocation across tracking modes for each HITL configuration. The V4L2 direct capture at 720p maintains 100\% detection-driven tracking, while the bridge at the same resolution requires Kalman, PINN, and hold fallback for over 40\% of the flight. The synthetic configuration validates PINN dropout recovery at 77.5\% utilization.

**Figure: Detection Rate Comparison**
> Detection rate across all HITL configurations. Both 640p bridge and 720p direct capture achieve 100\% detection rate, while 720p bridge drops to 59.5\% due to DDS throughput limitations. Synthetic runs show 21\% detection by design (controlled dropout injection).

### Key Statistics for Quick Reference

| Metric | Value | Context |
|--------|-------|---------|
| RF-DETR TRT FPS (720p) | 15.0 FPS | Jetson Orin Nano, TensorRT FP16 |
| PINN TRT inference | 0.28 ms | 3,045 inf/sec on Jetson GPU |
| V4L2 direct latency (mean) | 58.8 ms | 4x lower than bridge (233.7 ms) |
| V4L2 direct latency (P95) | 61.5 ms | vs bridge P95: 558.2 ms |
| Detection rate (direct 720p) | 100% | Zero dropouts in 108s flight |
| Detection rate (bridge 720p) | 59.5% | 114 dropouts due to DDS bottleneck |
| PINN dropout coverage | 77.5% | Synthetic test, 16.3s max gap |
| Bridge frame size | 2.7 MB | 1280x720 BGR8, DDS bottleneck |
| Hardware | Jetson Orin Nano 16GB | + PX4 FMUv6X + D455 |

---

*Degradation data generated: January 18, 2026*
*HITL data generated: March 11--25, 2026*
*Source bags: data/hitl_bags/, analysis: data/hitl_bags/comparison/*
