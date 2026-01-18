# SPIE Defense + Security 2026 Conference Roadmap
## Machine Learning from Challenging Data (DS112)
### AIRHOUND: Depth-Aware Tracking with Physics-Informed Prediction

---

## Overview

**Paper Title:** Predictive Target Pursuit for Autonomous UAVs using RF-DETR with Depth-Aware State Estimation and Physics-Informed Trajectory Prediction

**Conference:** SPIE Defense + Security 2026  
**Track:** Machine Learning from Challenging Data (DS112)  
**Paper ID:** 14030-26  
**Presentation Format:** Poster (90 min session)

### Key Deadlines

| Date | Milestone |
|------|-----------|
| ~~Oct 15, 2025~~ | Abstract submitted |
| ~~Jan 5, 2026~~ | Acceptance notification received |
| Feb 3, 2026 | Student Conference Support grant application |
| Feb 20, 2026 | Last day to change presenter/contact author |
| Feb 23, 2026 | Manuscript submission system opens |
| **April 1, 2026** | Poster PDF due, hotel booking, registration |
| **April 8, 2026** | **MANUSCRIPT DUE** |
| April 10, 2026 | Registration fees increase |
| **April 28, 2026** | Poster presentation (5:30 PM, 90 min) |

### Paper Contributions (Novel Elements)

1. **RF-DETR vs YOLOv8 Comparative Analysis** - Transformer vs CNN detection under synthetic degradation (motion blur, occlusions, noise)
2. **Depth-Fused 3D State Estimation** - Intel RealSense D455 depth integration for scale-invariant tracking with full 3D position and velocity estimation
3. **Physics-Informed Neural Network (PINN) Trajectory Prediction** - MLP with velocity-bounded physics loss for anticipatory control and dropout recovery
4. **Full 3D Anticipatory Control** - PINN-driven yaw, pitch viewing, and altitude adjustment for complete visual tracking *(conditional)*
5. **Real Flight Validation** - End-to-end testing on NVIDIA Jetson Orin edge platform

### Conference Theme Alignment

**"Machine Learning from Challenging Data"** maps to:
- RF-DETR handles occlusions better (transformer attention mechanism)
- Depth-aware tracking handles scale variation (challenging at varying distances)
- PINN handles temporal dropouts (challenging prediction through missing detections)
- Synthetic degradation pipeline tests robustness to motion blur, noise, occlusions
- **Full 3D control demonstrates practical utilization of PINN predictions**

---

## Progress Log

### Completed Milestones

| Date | Milestone | Notes |
|------|-----------|-------|
| Jan 17-18, 2026 | **Degradation Evaluation Complete** | RF-DETR vs YOLOv8 under 4 degradation types × 5 severity levels. Key finding: RF-DETR +40% mAP advantage under severe noise. See `docs/DEGRADATION_EVAL_RESULTS.md` |
| Jan 18, 2026 | Perception 3D Publisher | Added `/perception/target_3d` topic for 3D position (PR #11 merged) |
| Jan 18, 2026 | Documentation Scaffolding | Updated README, config, launch files with RF-DETR and depth integration |

### Key Results Summary

**Degradation Robustness (Jan 18, 2026):**
- Gaussian Noise @ 1.0 severity: RF-DETR 0.716 vs YOLOv8 0.318 mAP (**+40% advantage**)
- Low Light @ 1.0 severity: RF-DETR 0.670 vs YOLOv8 0.275 mAP (**+39% advantage**)
- Motion Blur: Both models struggle at high severity (need Kalman/PINN for recovery)
- Clean baseline: YOLOv8 0.957 vs RF-DETR 0.953 (comparable)

---

## 3D Control Scope: Altitude + Pitch Viewing

### Summary

This project implements **Detect + Track with yaw + pitch viewing + altitude control**, enabling full 3D PINN utilization (~90% of PINN output used for control).

### Control Axes

| Control Axis | Purpose | Implementation |
|--------------|---------|----------------|
| **Yaw** | Keep target centered horizontally | Yaw rate setpoint |
| **Pitch Viewing** | Keep target centered vertically | Small pitch angle adjustments |
| **Altitude** | Maintain consistent distance to target | Z velocity setpoint |

### Staged Implementation Approach

| Stage | Scope | Timing |
|-------|-------|--------|
| **Stage 1** | Altitude control only | Weeks 5-6 |
| **Stage 2** | Add pitch viewing | Weeks 7-8 |
| **Full Integration** | Yaw + Altitude + Pitch combined | Weeks 8-9 |

### Paper Narrative

> "When detection drops out, the PINN predicts target position in full 3D space. The controller proactively adjusts yaw, pitch, and altitude to maintain visual tracking — demonstrating anticipatory control across all viewing axes."

### Conditional Status

This is a **conditional goal** with a Week 6 decision point:
- If depth + PINN are working well by Week 6 → proceed with altitude + pitch viewing
- If not → revert to yaw-only for paper, document extended control as future work

---

## Meeting Structure

### Weekly Team Check-In

| Day | Time | Duration | Format |
|-----|------|----------|--------|
| Friday | 2:00 PM | ~30 minutes | Check-in (not a work meeting) |

**Starting:** Friday, January 16, 2026

**Purpose:**

- Progress updates from subteams
- Cross-team coordination
- Blockers and decision points

**Note:** If you cannot make this time, contact the project lead to arrange a separate meeting with relevant team members.

### Subteam Meetings

Subteams are expected to meet independently to complete their work. Meeting frequency and format is determined by each subteam.

---

## Team Structure

### Overview (7 People)

| Role | Subteam | Coding Level |
|------|---------|--------------|
| **Perception + Project Lead** | Perception & Tracking | Heavy |
| **Co-Lead (Hardware Platform)** | Systems & Flight | Variable |
| **Tracking/PINN** (x2) | Perception & Tracking | Heavy |
| **Controls** | Systems & Flight | Moderate |
| **Flight Ops** | Systems & Flight | Minimal |
| **Middleware + QA** | Systems & Flight | Light-Moderate |

### Subteam Structure

#### Subteam 1: Perception & Tracking
**Lead:** Perception + Project Lead

| Role | Focus |
|------|-------|
| Perception Lead | RF-DETR, TensorRT, depth subscriber, degradation pipeline |
| Tracking/PINN | Kalman filter, depth fusion, PINN development |
| Tracking/PINN | PINN training, 3D state estimation, Jetson deployment |

**Primary Work:** RF-DETR integration, depth fusion, Kalman filter, PINN development/training

#### Subteam 2: Systems & Flight
**Lead:** Co-Lead (Hardware Platform)

| Role | Focus |
|------|-------|
| Co-Lead | Hardware oversight, integration support, debugging |
| Controls | PID tuning, control node updates, pitch/altitude implementation |
| Flight Ops | Flight execution, safety, data collection |
| Middleware + QA | ROS2, rosbags, SITL, test scenarios |

**Primary Work:** PX4 tuning, flight testing, ROS2 integration, rosbag pipelines, safety protocols

### Coding Level Assessment

```
Heavy Coding    ████████████████████  Perception Lead
                ████████████████████  Tracking/PINN (x2)

Moderate        ████████████          Controls
                ████████████          Middleware + QA

Low             ███                   Flight Ops

Variable        ██████████?           Co-Lead (Hardware Platform)
```

### Role Responsibilities

#### Perception + Project Lead
- RF-DETR integration, TensorRT conversion
- Depth subscriber, synthetic degradation pipeline
- Overall project coordination, manuscript writing
- Cross-subteam oversight

#### Tracking/PINN (2 people)
- Kalman filter integration and 3D extension
- Depth fusion (2D bbox + depth → 3D position)
- PINN development, training, and Jetson deployment

#### Controls (1 person)
- PX4 PID tuning (now 3 axes: yaw, pitch, altitude)
- Control node updates for pitch viewing and altitude
- Safety limit implementation
- Flight behavior analysis

**Key Tasks:**

- [ ] Implement altitude control (Z velocity setpoint)
- [ ] Implement pitch viewing (pitch angle setpoint)
- [ ] Define and implement safety limits *(see Safety Parameters section)*
- [ ] Tune PIDs for all three axes

#### Flight Ops (1 person)
- Execute flights, safety protocols
- Data collection, rosbag management
- Hardware checks, flight cage coordination
- Additional test scenarios for vertical tracking

#### Middleware + QA (1 person)
- ROS2 integration, rosbag workflows
- SITL setup, test scenarios
- Vertical motion SITL scenarios

#### Co-Lead (Hardware Platform)
- Hardware platform expertise
- Integration debugging support
- Systems & Flight subteam oversight
- Mentorship for Controls and Flight Ops

### Manuscript Section Ownership

| Section | Primary Owner | Contributors |
|---------|---------------|--------------|
| Abstract | Lead | All |
| Introduction / Related Work | Lead | Tracking team (PINN lit review) |
| System Architecture | Middleware + QA | Perception, Tracking, Controls |
| RF-DETR Training & Optimization | Perception | - |
| Depth Integration & 3D Estimation | Tracking team | Perception |
| PINN Trajectory Prediction | Tracking team | - |
| **3D Anticipatory Control** | Controls | Tracking team |
| Experimental Setup | Flight Ops | Middleware + QA |
| Results & Analysis | Lead | All contribute data |
| Discussion & Conclusion | Lead | All review |

---

## Safety Parameters

**IMPORTANT:** Flight Ops and Controls must define the following safety parameters before altitude/pitch implementation begins. These values are placeholders — the actual limits must be determined based on:
- Hardware capabilities
- Flight cage constraints
- Risk assessment

### Required Safety Definitions

| Parameter | Placeholder | Owner | Notes |
|-----------|-------------|-------|-------|
| `ALTITUDE_FLOOR` | TBD (e.g., 1.0m) | Flight Ops + Controls | Minimum altitude above ground |
| `ALTITUDE_CEILING` | TBD (e.g., 3.0m) | Flight Ops + Controls | Maximum altitude (cage/safety limit) |
| `PITCH_MAX` | TBD (e.g., ±10-15°) | Controls | Maximum pitch angle for viewing |
| `PITCH_RATE_MAX` | TBD (e.g., 5°/s) | Controls | Maximum pitch rate of change |
| `Z_VELOCITY_MAX` | TBD (e.g., 1.0 m/s) | Controls | Maximum vertical velocity |
| `TARGET_DISTANCE_MIN` | TBD (e.g., 1.5m) | Flight Ops | Minimum allowed distance to target |
| `TARGET_DISTANCE_MAX` | TBD (e.g., 5.0m) | Flight Ops | Maximum allowed distance to target |

### Safety Implementation Checklist

- [ ] Flight Ops: Define altitude floor/ceiling based on flight cage
- [ ] Flight Ops: Define target distance bounds
- [ ] Controls: Define pitch limits based on platform stability
- [ ] Controls: Implement software limits in control node
- [ ] Controls: Implement failsafe (revert to hover if limits exceeded)
- [ ] Flight Ops: Review and approve before hardware testing

---

## Technical Architecture

### Current State (What Exists)

| Component | Status | Location |
|-----------|--------|----------|
| YOLOv8 TensorRT inference | Working | [`yolo_detector.py`](https://github.com/eppl-erau-db/AIRHOUND/blob/main/ws_ros2/src/airhound_perception/airhound_perception/yolo_detector.py) |
| RF-DETR training complete | Weights saved | Local: `rf-detr-training/weights/drone_rfdetr_*/` |
| RF-DETR ONNX export | Complete | Local: `drone_rfdetr_best.onnx` |
| RF-DETR ROS2 integration | **Placeholder only** | TBD: `detectors/rfdetr_detector.py` |
| Kalman filter (6-state) | Written, **not integrated** | TBD: `airhound_nodes/kalman_filter.py` |
| D455 depth config | Configured, **not subscribed** | TBD: `realsense_profile.yaml` |
| Yaw tracking node | Working | [`tracking.py`](https://github.com/eppl-erau-db/AIRHOUND/blob/main/ws_ros2/src/tracking_geometry/tracking_geometry/tracking.py) |
| Detector factory | Working | TBD: `detectors/factory.py` |

### Target State (End of Project)

```
D455 Camera
    ├── RGB Stream (30 fps) ──────────────────────────────┐
    │                                                      ▼
    └── Depth Stream (30 fps) ───┐              ┌─────────────────────┐
                                 │              │  Detection Node     │
                                 │              │  (RF-DETR/YOLOv8)   │
                                 │              │  TensorRT FP16      │
                                 │              └─────────┬───────────┘
                                 │                        │ 2D bbox
                                 ▼                        ▼
                        ┌─────────────────────────────────────────────┐
                        │         Depth Fusion Node                   │
                        │  bbox + depth → 3D position (x, y, z)       │
                        └─────────────────┬───────────────────────────┘
                                          │ 3D position
                                          ▼
                        ┌─────────────────────────────────────────────┐
                        │         3D Kalman Filter                    │
                        │  State: [x, y, z, vx, vy, vz]              │
                        │  Constant-velocity model                    │
                        └─────────────────┬───────────────────────────┘
                                          │ filtered state + velocity
                                          ▼
                        ┌─────────────────────────────────────────────┐
                        │         PINN Prediction Node                │
                        │  MLP with physics loss (|v| ≤ 10 m/s)      │
                        │  Predicts future position during dropout    │
                        └─────────────────┬───────────────────────────┘
                                          │ predicted 3D state
                                          ▼
                        ┌─────────────────────────────────────────────┐
                        │         3D Control Node                     │
                        │  Horizontal error → yaw rate               │
                        │  Vertical error → pitch angle (viewing)    │
                        │  Depth error → Z velocity (altitude)       │
                        └─────────────────┬───────────────────────────┘
                                          │ /cmd_setpoint
                                          │ (yaw, pitch, z_vel)
                                          ▼
                        ┌─────────────────────────────────────────────┐
                        │         PX4 Offboard Bridge                 │
                        │  TrajectorySetpoint streaming               │
                        │  (yaw_rate, pitch, z_velocity)             │
                        └─────────────────────────────────────────────┘
```

### Control Logic

```python
def compute_3d_control(predicted_state, current_uav_state, safety_params):
    """
    Compute control outputs for full 3D visual tracking.
    
    predicted_state: [x, y, z, vx, vy, vz] from PINN or Kalman
    current_uav_state: current UAV position/orientation
    safety_params: defined safety limits
    """
    
    # --- YAW CONTROL (existing) ---
    # Horizontal error in image frame → yaw rate
    horizontal_error = compute_horizontal_error(predicted_state)
    yaw_rate = pid_yaw(horizontal_error)
    
    # --- ALTITUDE CONTROL (Stage 1) ---
    # Depth error → Z velocity
    current_distance = predicted_state[2]  # z component
    distance_error = TARGET_DISTANCE - current_distance
    z_velocity = pid_altitude(distance_error)
    z_velocity = clamp(z_velocity, -Z_VELOCITY_MAX, Z_VELOCITY_MAX)
    
    # Enforce altitude limits
    if current_altitude <= ALTITUDE_FLOOR:
        z_velocity = max(z_velocity, 0)  # Only allow up
    if current_altitude >= ALTITUDE_CEILING:
        z_velocity = min(z_velocity, 0)  # Only allow down
    
    # --- PITCH VIEWING (Stage 2) ---
    # Vertical error in image frame → pitch angle
    vertical_error = compute_vertical_error(predicted_state)
    pitch_angle = pid_pitch(vertical_error)
    pitch_angle = clamp(pitch_angle, -PITCH_MAX, PITCH_MAX)
    
    return yaw_rate, pitch_angle, z_velocity
```

### PINN Architecture Specification

**Type:** Simple MLP (not LSTM/GRU - latency concerns on Jetson with depth + detection workload)

**Input:** Current state vector `[x, y, z, vx, vy, vz, dt]` where `dt` is prediction horizon

**Output:** Predicted state `[x', y', z', vx', vy', vz']` at time `t + dt`

**PINN Output Utilization:**

| PINN Output | Control Use |
|-------------|-------------|
| x', y' (horizontal) | Yaw anticipation |
| z' (vertical/depth) | Altitude anticipation |
| vx', vy' | Yaw rate prediction |
| vz' | Pitch viewing anticipation |

**Physics Loss Function:**

```python
def physics_loss(pred_state, true_state, v_max=10.0):
    # Data loss
    mse_loss = MSE(pred_state, true_state)
    
    # Physics constraint: velocity magnitude ≤ v_max
    vx, vy, vz = pred_state[3], pred_state[4], pred_state[5]
    v_magnitude = sqrt(vx² + vy² + vz²)
    velocity_penalty = max(0, v_magnitude - v_max)²
    
    # Combined loss
    return mse_loss + lambda_physics * velocity_penalty
```

**Training Data:**

1. Phase 1: SITL flights with scripted target motion (available immediately)
2. Phase 2: Fuse with real flight data (after flights begin)

**Online Update (Proof-of-Concept):**

- Limited fine-tuning of final layer weights during flight
- Not core system component - demonstration only
- Can be disabled if latency issues arise

---

## 12-Week Timeline

### Phase 1: RF-DETR Integration & Baseline (Weeks 1-3)
**Dates:** Jan 13 - Jan 31, 2026

#### Week 1 (Jan 13-17)

**Perception:**

- [ ] Implement real RF-DETR inference in `RFDetrDetector` class (replace placeholder)
- [ ] Load ONNX model and run inference on test images
- [ ] Verify output format matches YOLOv8 (`vision_msgs/Detection2DArray`)

**Tracking Team:**

- [ ] Integrate existing Kalman filter into `tracking.py`
- [ ] Test Kalman filter with recorded rosbags
- [ ] Document Kalman filter API for 3D extension

**Middleware + QA:**

- [ ] Verify Gazebo SITL environment is functional
- [ ] Create scripted target motion scenarios for PINN training data
- [ ] Set up rosbag recording pipeline

**Controls:**

- [ ] Review current PID parameters
- [ ] Document tuning strategy for hardware flights
- [ ] Review control node architecture for altitude/pitch extension

**Flight Ops:**

- [ ] Verify hardware platform still flight-ready
- [ ] Coordinate flight cage availability for semester
- [ ] Review safety protocols with team
- [ ] Begin defining safety parameters (altitude floor/ceiling, distance bounds)

**Lead:**

- [ ] Assign specific team members to roles
- [ ] Confirm subteam structure and meeting expectations
- [ ] Create shared data organization structure

#### Week 2 (Jan 20-24)

**Perception:**

- [ ] Run RF-DETR ONNX → TensorRT conversion on Jetson
- [ ] Benchmark RF-DETR TensorRT: FPS, latency, GPU memory
- [ ] Compare to YOLOv8 TensorRT baseline metrics

**Tracking Team:**

- [ ] Kalman filter integrated and tested in SITL
- [ ] Begin depth fusion node design
- [ ] Research PINN architecture details (loss function, training pipeline)

**Middleware + QA:**

- [ ] Record SITL rosbags with scripted target motion (PINN training data)
- [ ] Verify model swapping works (YOLOv8 ↔ RF-DETR via config)
- [ ] Create vertical motion SITL scenarios

**Controls:**

- [ ] Test control pipeline in SITL with Kalman-filtered state
- [ ] Identify any latency issues
- [ ] Document PX4 TrajectorySetpoint fields for pitch/Z control

**Flight Ops:**

- [ ] Prepare flight test checklist
- [ ] Schedule first hardware flight for Week 3
- [ ] Finalize safety parameter definitions

#### Week 3 (Jan 27-31)

**Perception:**

- [ ] RF-DETR TensorRT running stable on Jetson
- [x] ~~Begin synthetic degradation pipeline (motion blur, noise)~~ **COMPLETED EARLY (Week 1)** - See `docs/DEGRADATION_EVAL_RESULTS.md`
- [x] ~~Document RF-DETR vs YOLOv8 baseline metrics~~ **COMPLETED EARLY (Week 1)** - Full evaluation with 4 degradation types × 5 severity levels

**Tracking Team:**

- [ ] Begin depth subscriber implementation
- [ ] Design 3D Kalman filter extension (6-state → includes z, vz)

**Middleware + QA:**

- [ ] Complete SITL dataset for PINN training (10+ scripted scenarios)
- [ ] Include vertical motion scenarios in dataset

**Controls:**

- [ ] Support first hardware flight
- [ ] Begin altitude control design (Z velocity setpoint)

**Flight Ops:**

- [ ] **MILESTONE: First hardware validation flight with YOLOv8**
- [ ] Log flight data, identify issues

**Lead:**

- [ ] Compile Phase 1 results
- [ ] **Decision point: Is RF-DETR working?** If NO: Fallback to YOLOv8-only paper

---

### Phase 2: Depth Integration & 3D Estimation (Weeks 4-6)
**Dates:** Feb 3 - Feb 21, 2026

#### Week 4 (Feb 3-7)

**Perception:**

- [ ] Add depth topic subscriber to detection node
- [ ] Extract depth at bbox center for distance estimation
- [ ] Test depth accuracy at various ranges

**Tracking Team:**

- [ ] Implement depth fusion: 2D bbox + depth → 3D position (x, y, z)
- [ ] Use camera intrinsics for projection
- [ ] Extend Kalman filter to 3D state: `[x, y, z, vx, vy, vz]`

**Middleware + QA:**

- [ ] Verify depth stream quality in SITL
- [ ] Record depth + RGB synchronized rosbags

**Controls:**

- [ ] Continue PID tuning based on Week 3 flight data
- [ ] Implement altitude control node (Stage 1)
- [ ] Implement safety limits (altitude floor/ceiling, Z velocity max)

**Flight Ops:**

- [ ] Second hardware flight (YOLOv8 + Kalman)
- [ ] Test at varying target distances

**Lead:**

- [ ] Submit Student Conference Support grant application (due Feb 3)

#### Week 5 (Feb 10-14)

**Perception:**

- [x] ~~Complete synthetic degradation pipeline (blur, occlusion, noise)~~ **COMPLETED EARLY (Week 1)**
- [x] ~~Test degradation on both YOLOv8 and RF-DETR~~ **COMPLETED EARLY (Week 1)** - Key finding: RF-DETR +40% advantage under severe noise

**Tracking Team:**

- [ ] 3D Kalman filter working with depth fusion
- [ ] Begin PINN development: MLP architecture, training loop
- [ ] Set up PyTorch training pipeline for PINN

**Middleware + QA:**

- [ ] Test 3D tracking in SITL
- [ ] Prepare rosbag dataset for PINN training

**Controls:**

- [ ] Test altitude control in SITL
- [ ] Tune altitude PID
- [ ] Begin pitch viewing design (Stage 2)

**Flight Ops:**

- [ ] Third hardware flight (test depth integration)
- [ ] Test altitude control in controlled hover

#### Week 6 (Feb 17-21)

**Perception:**

- [ ] Document depth integration performance
- [ ] Compare detection accuracy at near/mid/far ranges

**Tracking Team:**

- [ ] Train PINN on SITL trajectory data
- [ ] Implement physics loss (velocity constraint ≤ 10 m/s)
- [ ] Validate PINN predictions offline

**Middleware + QA:**

- [ ] Full system integration test in SITL

**Controls:**

- [ ] Altitude control validated in SITL
- [ ] Implement pitch viewing control
- [ ] Implement pitch safety limits

**Lead:**

- [ ] Last day to change presenter (Feb 20)
- [ ] **Decision point: Is depth integration working?** If NO: Focus on 2D + degradation
- [ ] **Decision point: Is altitude control working?** If NO: Revert to yaw-only

---

### Phase 3: Comparative Flight Testing (Weeks 7-9)
**Dates:** Feb 24 - Mar 14, 2026

#### Week 7 (Feb 24-28)

**All Roles - Intensive Flight Testing**

**Flight Test Matrix:**


| Scenario | YOLOv8 | RF-DETR | Notes |
|----------|--------|---------|-------|
| Steady hover | [ ] | [ ] | Baseline |
| Slow linear (horizontal) | [ ] | [ ] | Yaw tracking |
| Moderate speed (horizontal) | [ ] | [ ] | Yaw tracking |
| Target moves up/down | [ ] | [ ] | Pitch viewing |
| Target approaches/retreats | [ ] | [ ] | Altitude control |
| Diagonal 3D motion | [ ] | [ ] | Combined control |
| Natural occlusions | [ ] | [ ] | Dropout recovery |
| Varying distance | [ ] | [ ] | Depth accuracy |

**Per-Flight Data Collection:**

- [ ] Weather conditions logged
- [ ] Model/mode documented (YOLO/RF-DETR, clean/degraded)
- [ ] Control mode documented (yaw-only, altitude, pitch, full 3D)
- [ ] Rosbag recorded and labeled
- [ ] PX4 logs downloaded

**Controls:**

- [ ] Test pitch viewing in hardware flights
- [ ] Tune pitch PID based on flight data
- [ ] Full 3D control integration (yaw + altitude + pitch)

**Tracking Team:**

- [ ] Collect flight data for PINN fine-tuning
- [ ] Continue PINN training with real trajectory data

#### Week 8 (Mar 3-7)

**Flight Ops + All:**

- [ ] Continue flight test matrix
- [ ] Target: 15-20 flights completed by end of week
- [ ] Include 3D tracking scenarios

**Tracking Team:**

- [ ] PINN trained on combined sim + real data
- [ ] Test PINN inference on Jetson (latency check)
- [ ] Implement proof-of-concept online update mechanism

**Controls:**

- [ ] Full 3D control validated (yaw + pitch + altitude)
- [ ] Document control performance metrics

**Lead:**

- [ ] Begin manuscript outline
- [ ] Draft Introduction section

#### Week 9 (Mar 10-14)

**Flight Ops + All:**

- [ ] Complete remaining flight scenarios
- [ ] **MILESTONE: 25+ flights completed**
- [ ] Ensure 3D control scenarios are adequately covered

**Tracking Team:**

- [ ] PINN deployed and tested on Jetson
- [ ] Compare: Linear extrapolation → Kalman → PINN
- [ ] Document prediction accuracy metrics

**Perception:**

- [ ] Finalize RF-DETR vs YOLOv8 comparison data

**Lead:**

- [ ] Draft System Architecture section
- [ ] **Decision point: Is PINN showing improvement?** If NO: Paper uses Kalman as prediction method
- [ ] **Decision point: Is full 3D control working?** If NO: Paper uses yaw + altitude only (or yaw-only fallback)

---

### Phase 4: Analysis & Manuscript Writing (Weeks 10-12)
**Dates:** Mar 17 - Apr 8, 2026

#### Week 10 (Mar 17-21)

**Analysis Focus - All Roles Contribute Data**

**Metrics to Extract:**

- [ ] Detection: mAP, precision, recall, FPS, latency (YOLOv8 vs RF-DETR)
- [ ] Tracking: dropout duration, recovery time, 3D position error
- [ ] Prediction: Kalman vs PINN comparison, RMSE, max error
- [ ] Control: yaw response time, tracking error
- [ ] Altitude control: distance maintenance accuracy
- [ ] Pitch viewing: vertical tracking error
- [ ] 3D control: combined tracking performance

**Perception:**

- [ ] Generate detection comparison plots
- [ ] Document degradation impact on each model

**Tracking Team:**

- [ ] Generate PINN vs Kalman comparison plots
- [ ] Document 3D estimation accuracy

**Controls:**

- [ ] Generate control performance plots (yaw, pitch, altitude)
- [ ] Document 3D tracking accuracy

**Lead:**

- [ ] Draft RF-DETR Training section
- [ ] Draft Experimental Setup section

#### Week 11 (Mar 24-28)

**Writing Focus**

**Lead:**

- [ ] Draft Results section
- [ ] Create comparison tables (detection, tracking, prediction, control)
- [ ] Draft Discussion section

**All Roles:**

- [ ] Review and provide feedback on draft sections
- [ ] Generate remaining figures

**Figures to Create:**

1. [ ] Hardware platform photo
2. [ ] System architecture diagram
3. [ ] RF-DETR vs YOLOv8 detection comparison (bar chart)
4. [ ] Depth fusion visualization
5. [ ] 3D tracking trajectory plot
6. [ ] PINN vs Kalman prediction comparison
7. [ ] Degradation impact plot (mAP vs degradation level)
8. [ ] Control performance (yaw error over time)
9. [ ] Altitude control performance (distance error over time)
10. [ ] Pitch viewing performance (vertical error over time)
11. [ ] 3D tracking demonstration (combined axes)

#### Week 12 (Mar 31 - Apr 8)

**Finalization**

- [ ] Complete all manuscript sections
- [ ] Final figures and tables inserted
- [ ] Proofread and format check (SPIE template)
- [ ] All authors review final draft
- [ ] Faculty advisor review (Dr. Drakunov)
- [ ] **SUBMIT MANUSCRIPT by April 8**

**Poster Preparation (parallel):**

- [ ] Design poster layout
- [ ] Key figures selected for poster
- [ ] **Poster PDF due April 1**

---

## Fallback Decision Points

| Week | Checkpoint | Go/No-Go Decision |
|------|------------|-------------------|
| **Week 3** | RF-DETR working on Jetson? | If NO: Paper focuses on YOLOv8 + Kalman + depth |
| **Week 6** | Depth integration working? | If NO: Paper focuses on 2D tracking + degradation comparison |
| **Week 6** | **Altitude control working?** | If NO: Revert to yaw-only control |
| **Week 7** | Sufficient flight data? | If NO: Supplement with extensive SITL results |
| **Week 9** | PINN outperforms Kalman? | If NO: Paper uses Kalman as prediction method, mentions PINN as future work |
| **Week 9** | **Full 3D control working?** | If NO: Paper uses yaw + altitude (or yaw-only), document pitch as future work |

### Control Fallback Tiers

| Tier | Scope | Requirement |
|------|-------|-------------|
| **Tier 1 (Full)** | Yaw + Pitch Viewing + Altitude | All 3 axes validated in hardware |
| **Tier 2** | Yaw + Altitude | Altitude working, pitch not stable |
| **Tier 3** | Yaw only | Minimum viable control |

### Minimum Viable Paper

If all else fails, the paper can be submitted with:
- RF-DETR vs YOLOv8 comparison under synthetic degradation
- Kalman filter for dropout handling
- Yaw-only control
- SITL + limited hardware validation

This matches the current accepted abstract and is submittable even if depth/PINN/extended control don't work out.

---

## Data Organization

```
AIRHOUND_SPIE_Data/
├── sitl/
│   ├── pinn_training/           # Scripted motion for PINN training
│   │   ├── scenario_01_linear.bag
│   │   ├── scenario_02_circular.bag
│   │   ├── scenario_03_vertical.bag
│   │   ├── scenario_04_diagonal_3d.bag
│   │   └── ...
│   └── validation/              # SITL validation runs
├── flights/
│   ├── yolov8/
│   │   ├── clean/
│   │   │   ├── flight_001_baseline.bag
│   │   │   └── ...
│   │   └── degraded/
│   └── rfdetr/
│       ├── clean/
│       └── degraded/
├── models/
│   ├── yolov8_tensorrt/
│   ├── rfdetr_tensorrt/
│   └── pinn_weights/
├── analysis/
│   ├── notebooks/               # Jupyter notebooks for analysis
│   ├── plots/                   # Generated figures
│   └── metrics.csv              # Extracted metrics spreadsheet
└── manuscript/
    ├── figures/
    ├── tables/
    └── spie_submission/
```

---

## Hardware Reference

| Component | Specification |
|-----------|---------------|
| **Companion Computer** | NVIDIA Jetson Orin 16GB |
| **Camera** | Intel RealSense D455 (RGB + Depth, 30fps, 1280x720) |
| **Flight Controller** | PX4 Pixhawk |
| **OS** | Ubuntu 22.04, ROS2 Humble |
| **Detection** | YOLOv8n / RF-DETR (TensorRT FP16) |

### D455 Depth Specifications
- Depth range: 0.4m - 6m (optimal)
- Depth resolution: 1280x720 @ 30fps
- Depth accuracy: <2% at 4m
- RGB-D alignment: Hardware synchronized

---

## Resources

### SPIE Submission
- Manuscript Guidelines: https://spie.org/conferences-and-exhibitions/authors-and-presenters/manuscript-submission-guidelines
- LaTeX Template: Overleaf "SPIE Proceedings" template
- Conference Page: https://spie.org/ds/conferencedetails/machine-learning-from-challenging-data

### Technical References
- RF-DETR: https://rfdetr.roboflow.com/
- RealSense D455: https://www.intelrealsense.com/depth-camera-d455/
- PX4 Offboard: https://docs.px4.io/main/en/flight_modes/offboard.html
- PINN Survey: Raissi et al., "Physics-Informed Neural Networks" (2019)

### Repositories
- AIRHOUND Main: https://github.com/eppl-erau-db/AIRHOUND
- RF-DETR Training: Local (`rf-detr-training/`)

---

## Appendix A: PINN Training Details

### Dataset Requirements
- Minimum: 1000 trajectory segments (each 30-60 frames)
- Format: `[t, x, y, z, vx, vy, vz]` per frame
- Source: SITL scripted motion + real flight data
- Include vertical and 3D diagonal motion patterns

### Training Configuration
```yaml
model:
  type: MLP
  hidden_layers: [64, 128, 64]
  activation: ReLU
  input_dim: 7   # [x, y, z, vx, vy, vz, dt]
  output_dim: 6  # [x', y', z', vx', vy', vz']

training:
  epochs: 100
  batch_size: 256
  learning_rate: 0.001
  optimizer: Adam

physics_loss:
  v_max: 10.0  # m/s
  lambda_physics: 0.1  # weight for physics constraint
```

### Online Update (Proof-of-Concept)
```yaml
online_update:
  enabled: true
  update_layers: ["fc_final"]  # Only update last layer
  learning_rate: 0.0001        # Lower LR for stability
  update_frequency: 100        # Update every N frames
  max_updates_per_flight: 50   # Limit total updates
```

---

## Appendix B: Safety Parameters Template

**This section must be completed by Flight Ops and Controls before Week 4.**

```yaml
# SAFETY PARAMETERS - TO BE DEFINED BY FLIGHT OPS + CONTROLS
# Do not proceed with altitude/pitch implementation until these are finalized

altitude_limits:
  floor: TBD        # meters above ground (e.g., 1.0)
  ceiling: TBD      # meters above ground (e.g., 3.0)
  rationale: ""     # Why these values were chosen

pitch_limits:
  max_angle: TBD    # degrees (e.g., 15)
  max_rate: TBD     # degrees/second (e.g., 5)
  rationale: ""

velocity_limits:
  z_max: TBD        # m/s vertical (e.g., 1.0)
  rationale: ""

distance_limits:
  target_min: TBD   # meters (e.g., 1.5)
  target_max: TBD   # meters (e.g., 5.0)
  target_nominal: TBD  # meters (desired distance)
  rationale: ""

failsafe:
  action: "hover"   # What to do if limits exceeded
  timeout: TBD      # seconds before failsafe triggers

approved_by:
  flight_ops: ""
  controls: ""
  date: ""
```

---

*Last Updated: January 2026*
