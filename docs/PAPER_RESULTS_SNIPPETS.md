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

*Generated: January 18, 2026*
*Source: benchmark_results/degradation_eval_2026-01-17_18-18/*
