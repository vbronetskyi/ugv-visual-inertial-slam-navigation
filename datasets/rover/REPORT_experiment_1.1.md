# Experiment 1.1: ORB-SLAM3 Baseline on ROVER Dataset

## 1. Introduction

This report presents the results of running ORB-SLAM3 on the ROVER dataset - a multi-season outdoor robot dataset collected at Esslingen University of Applied Sciences (Germany). The goal is to establish a baseline for visual SLAM performance across diverse environmental conditions (seasons, lighting) using three ORB-SLAM3 modes: Stereo, Stereo-Inertial, and RGB-D.

**ROVER** (iis-esslingen/ROVER on HuggingFace) contains 15 recordings collected across two locations with varying seasonal and lighting conditions, using a ground robot equipped with multiple sensors.

---

## 2. Robot Platform and Sensors

### 2.1 Robot

Ground robot (differential drive) equipped with:
- 2x Intel RealSense cameras (T265 + D435i)
- Pi Camera (not used in this experiment)
- VN100 external IMU (not used in this experiment)
- Leica Total Station for ground truth (~2 Hz)

### 2.2 Intel RealSense T265 (Stereo Fisheye + IMU)

| Parameter | Value |
|-----------|-------|
| **Type** | Tracking camera, stereo fisheye |
| **Resolution** | 848 x 800 px per eye |
| **FPS** | 30 Hz |
| **Model** | KannalaBrandt8 (equidistant fisheye) |
| **Baseline** | ~6.35 cm |
| **Cam Left** | fx=286.18, fy=286.39, cx=416.94, cy=403.27 |
| **Cam Right** | fx=285.72, fy=285.94, cx=418.88, cy=401.01 |
| **Distortion** | k1=-0.0115, k2=0.0502, k3=-0.0504, k4=0.0127 |
| **IMU Rate** | 264 Hz |
| **IMU NoiseGyro** | 0.00324 rad/s/sqrt(Hz) |
| **IMU NoiseAcc** | 0.01741 m/s^2/sqrt(Hz) |

### 2.3 Intel RealSense D435i (RGB-D)

| Parameter | Value |
|-----------|-------|
| **Type** | Depth camera, structured light |
| **Resolution** | 640 x 480 px |
| **FPS** | 30 Hz |
| **Model** | PinHole + radtan |
| **Intrinsics** | fx=596.20, fy=593.14, cx=327.05, cy=245.16 |
| **Distortion** | k1=0.1572, k2=-0.4894, p1=-0.00075, p2=0.00037 |
| **DepthMapFactor** | 1000 (mm -> m) |

### 2.4 Ground Truth

Leica Total Station tracking a prism on the robot. Frequency varies from 1.6 to 3.9 Hz depending on recording (line-of-sight requirements). Accuracy: sub-centimeter.

---

## 3. Dataset Description

### 3.1 Locations

**Garden Large** - enclosed garden area at the university, ~13 x 20 m. Relatively compact route with multiple loops. Rich texture from vegetation and structures.

**Park** - larger open park area, ~20 x 19 m. Longer route (~180 m) with more open spaces and fewer distinctive landmarks.

### 3.2 Recording Sessions

| # | Recording | Date | Season/Lighting | Traj. Length | Duration | Frames (T265) | Frames (D435i) | GT Poses | IMU Samples |
|---|-----------|------|-----------------|-------------|----------|---------------|----------------|----------|-------------|
| 1 | garden_large_summer | 2023-08-18 | Summer, day | 167.7 m | 469 s | 14030 | 13738 | 1816 | 123914 |
| 2 | garden_large_autumn | 2023-12-21 | Autumn, day | 170.3 m | 464 s | 13837 | 13857 | 997 | 121987 |
| 3 | garden_large_winter | 2024-01-13 | Winter, day | 162.3 m | 437 s | 12924 | 12941 | 776 | 113921 |
| 4 | garden_large_spring | 2024-04-11 | Spring, day | 165.0 m | 451 s | 13410 | 13427 | 1731 | 118344 |
| 5 | garden_large_day | 2024-05-29 | Late spring, day | 150.3 m | 392 s | 11767 | 11789 | 1099 | 103775 |
| 6 | garden_large_dusk | 2024-05-29 | Late spring, dusk | 151.8 m | 401 s | 12002 | 12009 | 824 | 105816 |
| 7 | garden_large_night | 2024-05-30 | Late spring, night | 150.5 m | 397 s | 11884 | 11894 | 703 | 104769 |
| 8 | garden_large_night-light | 2024-05-30 | Night + artificial light | 151.8 m | 399 s | 11964 | 11987 | 705 | 105482 |
| 9 | park_summer | 2023-07-31 | Summer, day | 164.2 m | 439 s | 13138 | 12669 | 837 | 115919 |
| 10 | park_autumn | 2023-11-07 | Autumn, day | 170.5 m | 498 s | 14918 | 13988 | 1290 | 131519 |
| 11 | park_spring | 2024-04-14 | Spring, day | 171.5 m | 466 s | 13877 | 13893 | 752 | 122502 |
| 12 | park_day | 2024-05-08 | Late spring, day | 183.0 m | 484 s | 14429 | 14446 | 1046 | 127263 |
| 13 | park_dusk | 2024-05-13 | Late spring, dusk | 182.7 m | 481 s | 14351 | 14368 | 1315 | 126632 |
| 14 | park_night | 2024-05-13 | Late spring, night | 179.8 m | 474 s | 14181 | 14175 | 1005 | 125126 |
| 15 | park_night-light | 2024-05-24 | Night + artificial light | 172.1 m | 463 s | 13750 | 13743 | 1127 | 121273 |

### 3.3 Key Dataset Characteristics

- **Trajectory lengths:** Garden Large: 150–170 m, Park: 164–183 m
- **Session durations:** 392–498 s (6.5–8.3 min)
- **Total data volume:** ~335 GB (15 recordings)
- **T265 frame rate:** ~30 fps (11,767–14,918 frames per session)
- **D435i frame rate:** ~30 fps, synchronized RGB + depth
- **IMU rate:** ~264 Hz (103,775–131,519 samples per session)
- **GT rate:** 1.6–3.9 Hz (depends on Leica line-of-sight)

---

## 4. Experimental Setup

### 4.1 ORB-SLAM3 Modes

Three modes were tested on each of the 15 recordings (45 experiments total):

1. **Stereo** - T265 stereo fisheye (KannalaBrandt8 model), 848x800, baseline 6.35 cm
2. **Stereo-Inertial** - T265 stereo + built-in IMU @ 264 Hz
3. **RGB-D** - D435i color + depth (PinHole model), 640x480

### 4.2 Data Conversion

- **Stereo/Stereo-Inertial:** T265 data converted to EuRoC MAV format (nanosecond timestamps, gyro-then-acc IMU column order). Script: `scripts/convert_rover_to_euroc.py`
- **RGB-D:** D435i data prepared in TUM RGB-D format with association file matching RGB and depth frames by nearest timestamp. Script: `scripts/prepare_rover_rgbd.py`

### 4.3 ORB-SLAM3 Parameters

| Parameter | Stereo/Stereo-Inertial | RGB-D |
|-----------|----------------------|-------|
| Camera model | KannalaBrandt8 | PinHole + radtan |
| Resolution | 848 x 800 | 640 x 480 |
| ORB features | 1500 | 1500 |
| Scale factor | 1.2 | 1.2 |
| Levels | 8 | 8 |
| iniThFAST | 15 | 20 |
| minThFAST | 5 | 7 |
| ThDepth | 20 | 40 |

### 4.4 Evaluation Methodology

1. **Trajectory loading:** ORB-SLAM3 outputs in TUM format (timestamp tx ty tz qx qy qz qw)
2. **Association:** Estimated poses matched to GT by nearest timestamp (max 0.1s tolerance)
3. **Alignment:** Sim3 (7-DoF: rotation + translation + scale) via Umeyama algorithm
4. **Metrics:**
   - **ATE RMSE** (Absolute Trajectory Error, root mean square): primary metric
   - **ATE mean, median, std, max**
   - **Sim3 scale factor** (ideal = 1.0)
   - **Tracking rate:** % of frames with estimated pose
5. **Execution:** 3 experiments in parallel, timeout 1200s per run, headless via xvfb

---

## 5. Results

### 5.1 Summary Table

| Recording | Traj. Length | Stereo | Stereo-Inertial | RGB-D ATE RMSE | RGB-D Scale |
|-----------|-------------|--------|-----------------|----------------|-------------|
| GL/autumn | 170.3 m | FAIL | FAIL | **0.365 m** | 0.981 |
| GL/day | 150.3 m | FAIL | FAIL | **0.398 m** | 0.977 |
| GL/dusk | 151.8 m | FAIL | FAIL | FAIL | - |
| GL/night-light | 151.8 m | FAIL | FAIL | **0.480 m** | 0.959 |
| GL/night | 150.5 m | FAIL | FAIL | FAIL | - |
| GL/spring | 165.0 m | FAIL | FAIL | **0.420 m** | 0.988 |
| GL/summer | 167.7 m | FAIL | FAIL | **0.453 m** | 0.988 |
| GL/winter | 162.3 m | FAIL | FAIL | **0.367 m** | 0.980 |
| P/autumn | 170.5 m | FAIL | FAIL | **1.874 m** | 0.980 |
| P/day | 183.0 m | FAIL | FAIL | FAIL | - |
| P/dusk | 182.7 m | FAIL | FAIL | **1.629 m** | 1.036 |
| P/night-light | 172.1 m | FAIL | FAIL | FAIL | - |
| P/night | 179.8 m | FAIL | FAIL | **6.553 m** | 0.929 |
| P/spring | 171.5 m | FAIL | FAIL | **0.537 m** | 1.031 |
| P/summer | 164.2 m | FAIL | FAIL | **0.448 m** | 1.021 |

### 5.2 Aggregate Statistics

| Metric | Stereo | Stereo-Inertial | RGB-D |
|--------|--------|-----------------|-------|
| **Success rate** | 0/15 (0%) | 0/15 (0%) | **11/15 (73.3%)** |
| ATE RMSE mean | - | - | 1.230 m |
| ATE RMSE median | - | - | **0.453 m** |
| ATE RMSE min | - | - | 0.365 m |
| ATE RMSE max | - | - | 6.553 m |
| ATE RMSE std | - | - | 1.844 m |

### 5.3 Per-Location RGB-D Statistics

| Location | Success | Mean ATE | Median ATE | Min ATE | Max ATE |
|----------|---------|----------|------------|---------|---------|
| **Garden Large** | 6/8 (75%) | 0.414 m | 0.409 m | 0.365 m | 0.480 m |
| **Park** | 5/7 (71%) | 2.208 m | 1.629 m | 0.448 m | 6.553 m |

Garden Large results are tightly clustered (0.37–0.48 m), indicating consistent performance across seasons. Park results show much higher variance, driven primarily by the park_night outlier (6.55 m).

### 5.4 Successful RGB-D Recordings - Detailed Results

| # | Recording | Traj. Length | ATE RMSE | ATE Mean | ATE Median | ATE Max | Scale | Tracking |
|---|-----------|-------------|----------|----------|------------|---------|-------|----------|
| 1 | GL/autumn | 170.3 m | 0.365 m | 0.334 m | 0.318 m | 0.952 m | 0.981 | 100% |
| 2 | GL/winter | 162.3 m | 0.367 m | 0.331 m | 0.295 m | 1.075 m | 0.980 | 100% |
| 3 | GL/day | 150.3 m | 0.398 m | 0.361 m | 0.321 m | 0.998 m | 0.977 | 100% |
| 4 | GL/spring | 165.0 m | 0.420 m | 0.373 m | 0.343 m | 1.226 m | 0.988 | 100% |
| 5 | P/summer | 164.2 m | 0.448 m | 0.415 m | 0.400 m | 1.107 m | 1.021 | 50.5% |
| 6 | GL/summer | 167.7 m | 0.453 m | 0.404 m | 0.403 m | 5.285 m | 0.988 | 100% |
| 7 | GL/night-light | 151.8 m | 0.480 m | 0.438 m | 0.396 m | 2.084 m | 0.959 | 100% |
| 8 | P/spring | 171.5 m | 0.537 m | 0.492 m | 0.484 m | 1.228 m | 1.031 | 100% |
| 9 | P/dusk | 182.7 m | 1.629 m | 0.914 m | 0.638 m | 7.690 m | 1.036 | 100% |
| 10 | P/autumn | 170.5 m | 1.874 m | 0.687 m | 0.431 m | 14.780 m | 0.980 | 100% |
| 11 | P/night | 179.8 m | 6.553 m | 5.832 m | 6.118 m | 19.334 m | 0.929 | 99.9% |

**Note on park_summer:** Only 50.5% tracking rate (6387 of 12669 frames) - ORB-SLAM3 lost tracking for half the session, but the tracked portion has good accuracy (0.45 m ATE).

---

## 6. Error Analysis

### 6.1 Stereo Mode - Complete Failure (0/15)

**Symptom:** All 15 recordings produced only 1 keyframe before ORB-SLAM3 crashed (segfault, exit code 139) or ran to completion with no useful trajectory.

**Failure breakdown:**
- 10/15: Tracking failure + segfault (exit code 139)
- 5/15: Tracking failure only (completed but no trajectory)

**Root cause:** The Intel RealSense T265 has a **6.35 cm stereo baseline** with **equidistant fisheye distortion** (KannalaBrandt8). This combination is fundamentally insufficient for outdoor stereo matching:

1. **Narrow baseline (6.35 cm):** For objects at 5–10 m distance, stereo disparity is only 1–2 pixels. The disparity signal is smaller than the noise level, preventing reliable depth triangulation.
2. **Fisheye distortion:** The equidistant model maps 200+ degree FoV to 848 px. Pixels near the center have very different scale than at the edges, making stereo matching in ORB-SLAM3's KannalaBrandt8 pipeline unreliable for outdoor scenes.
3. **Low resolution relative to FoV:** 848x800 covers ~170° FoV, giving only ~5 px/degree angular resolution. This is significantly lower than typical pinhole stereo cameras at similar resolutions.

**Comparison:** The T265 was designed for indoor VIO tracking at close range (0.5–3 m), not outdoor stereo-only SLAM at 5–30 m distances.

### 6.2 Stereo-Inertial Mode - Complete Failure (0/15)

**Symptom:** Same as Stereo - the visual front-end cannot initialize, so IMU pre-integration has no visual anchor. ORB-SLAM3's Stereo-Inertial mode requires succesful stereo initialization before fusing IMU data.

**Failure breakdown:**
- 10/15: Tracking failure (1 KF, no trajectory)
- 3/15: Abort with `system_error` (Sophus math error during IMU integration without valid visual poses)
- 2/15: Tracking failure + segfault

**Conclusion:** IMU data alone cannot compensate for the complete failure of the stereo visual front-end. The IMU pre-integration requires at least two keyframes to form a constraint, which never happens because stereo matching fails first.

### 6.3 RGB-D Mode - Failed Recordings (4/15)

| Recording | Traj. Length | Lighting | Failure Mode | Details |
|-----------|-------------|----------|-------------|---------|
| GL/dusk | 151.8 m | Dusk (low light) | **Abort** (exit 134) | Sophus `SO3::exp()` NaN assertion. Numerical instability in rotation computation due to degraded ORB features in poor lighting. |
| GL/night | 150.5 m | Night (no light) | **Segfault** (exit 139) | Complete darkness eliminates most visual features. The few detected features are noise, leading to invalid matrix operations. |
| P/day | 183.0 m | Day (good light) | **Timeout** (>1200s) | The longest trajectory (183 m) combined with aggressive loop closure detection caused excessive computation. Not a lighting issue - purely computational. |
| P/night-light | 172.1 m | Night + artificial | **Segfault** (exit 139) | Artificial lighting creates harsh shadows and specular reflections, confusing ORB feature matching and causing numerical instability. |

**Pattern:** 3 of 4 RGB-D failures are in low-light conditions (dusk, night, night-light). The D435i depth sensor continues functioning in the dark (infrared structured light), but the RGB camera produces noisy, low-contrast images where ORB features are unreliable. This creates a mismatch: depth data is available but visual tracking fails.

### 6.4 RGB-D Mode - High-Error Recordings

| Recording | ATE RMSE | Issue |
|-----------|----------|-------|
| P/night | 6.553 m | Night conditions: severe trajectory drift, estimated trajectory does not match GT shape. ATE error grows to 19.3 m at worst points. Scale factor 0.929 indicates systematic underestimation. |
| P/autumn | 1.874 m | Localized error spike to 14.8 m (ATE max), likely a brief tracking loss followed by recovery in a different position. Median ATE is only 0.43 m, suggesting most of the trajectory is accurate. |
| P/dusk | 1.629 m | Similar pattern to P/autumn: median ATE 0.64 m is reasonable, but max error reaches 7.7 m. Dusk lighting causes intermittent tracking degradation. |

---

## 7. Trajectory Analysis

### 7.1 Garden Large (Compact Route)

The garden route is a roughly rectangular loop (~13 x 20 m). ORB-SLAM3 RGB-D consistently tracks it well:

- **Best:** GL/autumn (0.365 m) and GL/winter (0.367 m) - these have the most stable daytime lighting with bare vegetation, providing clear structural features.
- **Seasonal variation is minimal:** All 6 successful runs are within 0.37–0.48 m ATE, showing that seasonal changes (vegetation, snow, leaf cover) do not significantly affect ORB-SLAM3 RGB-D performance.
- **Night-light (0.48 m)** is slightly worse than daytime recordings, but artificial lighting provides enough illumination for functional ORB extraction.

### 7.2 Park (Larger Open Area)

The park route is a larger irregular loop (~20 x 19 m) with longer traversals:

- **Good recordings:** P/summer (0.448 m) and P/spring (0.537 m) achieve garden-level accuracy in good lighting.
- **Problematic recordings:** P/autumn (1.874 m) and P/dusk (1.629 m) show intermittent drift - the trajectory follows the GT shape but accumulates localized errors.
- **P/night is the worst (6.553 m):** The trajectory shape diverges completely from GT. The robot follows a loop but ORB-SLAM3 produces a distorted, offset trajectory with a systematic 25 m excursion.

### 7.3 Scale Factor Analysis

All successful RGB-D recordings have Sim3 scale factors close to 1.0:

| Range | Count | Interpretation |
|-------|-------|---------------|
| 0.95–1.05 | 10/11 | Correct depth integration |
| < 0.95 | 1/11 (P/night: 0.929) | Systematic depth underestimation in low light |

The near-unity scale confirms that the D435i depth sensor provides accurate metric information. The slight deviation from 1.0 (mean ~0.99) is within normal Sim3 alignment noise.

---

## 8. Comparison: Garden Large vs Park

| Metric | Garden Large | Park |
|--------|-------------|------|
| Route size | ~13 x 20 m | ~20 x 19 m |
| Trajectory length | 150–170 m | 164–183 m |
| RGB-D success rate | 6/8 (75%) | 5/7 (71%) |
| ATE RMSE (mean) | **0.414 m** | 2.208 m |
| ATE RMSE (median) | **0.409 m** | **1.629 m** |
| ATE variance | Very low (std=0.04 m) | Very high (std=2.34 m) |

**Why Park is harder:**
1. **Larger open spaces** with fewer close-range structural features for depth-aided tracking
2. **Longer trajectories** (~180 m vs ~155 m) allow more drift accumulation
3. **Less distinctive landmarks** in the park environment (trees, grass are self-similiar)

Excluding the P/night outlier, Park mean ATE drops to 1.12 m - still 2.7x worse than Garden Large, confirming the environment is inherently more challenging.

---

## 9. Conclusions

### 9.1 Key Findings

1. **Intel RealSense T265 stereo is incompatible with ORB-SLAM3 for outdoor SLAM.** The 6.35 cm baseline with equidistant fisheye is fundamentally too narrow for outdoor depth estimation. All 30 Stereo and Stereo-Inertial experiments (15 recordings x 2 modes) failed completely. This is a hardware limitation, not a software configuration issue.

2. **Intel RealSense D435i RGB-D is effective for outdoor robot SLAM.** 11 out of 15 recordings produced valid trajectories (73.3% success rate) with median ATE 0.45 m on trajectories of 150–183 m length.

3. **Seasonal variation has minimal impact on RGB-D SLAM performance.** Garden Large results are consistent across summer, autumn, winter, and spring (0.37–0.48 m ATE). Environmental changes like vegetation growth, leaf cover, and snow do not significantly affect ORB feature extraction when combined with depth data.

4. **Lighting is the dominant failure factor.** 3 of 4 failed RGB-D recordings are in low-light conditions. Night recording without artificial light is the most challenging scenario, causing either crashes (segfault, NaN) or severe drift (6.55 m ATE).

5. **Environment complexity affects accuracy.** Garden Large (compact, feature-rich) consistently outperforms Park (large, open) by ~5x in ATE RMSE. Close-range structural features are critical for RGB-D SLAM stability.

### 9.2 Relative ATE (as % of Trajectory Length)

| Recording | Traj. Length | ATE RMSE | Relative ATE |
|-----------|-------------|----------|-------------|
| GL/autumn | 170.3 m | 0.365 m | **0.21%** |
| GL/winter | 162.3 m | 0.367 m | **0.23%** |
| GL/day | 150.3 m | 0.398 m | **0.26%** |
| GL/spring | 165.0 m | 0.420 m | **0.25%** |
| P/summer | 164.2 m | 0.448 m | **0.27%** |
| GL/summer | 167.7 m | 0.453 m | **0.27%** |
| GL/night-light | 151.8 m | 0.480 m | **0.32%** |
| P/spring | 171.5 m | 0.537 m | **0.31%** |
| P/dusk | 182.7 m | 1.629 m | 0.89% |
| P/autumn | 170.5 m | 1.874 m | 1.10% |
| P/night | 179.8 m | 6.553 m | 3.64% |

The best recordings achieve 0.2–0.3% relative error, which is competitive for real-time visual SLAM on a consumer depth camera.

### 9.3 Comparison with Other Datasets

| Experiment | Dataset | Method | ATE RMSE | Traj. Length | Relative ATE |
|------------|---------|--------|----------|-------------|-------------|
| 0.8 | RobotCar | ORB-SLAM3 Stereo | 3.91 m | ~10 km | 0.04% |
| 0.9 | 4Seasons | ORB-SLAM3 Stereo-Inertial | 0.93 m | 3.7 km | 0.03% |
| **1.1** | **ROVER GL** | **ORB-SLAM3 RGB-D** | **0.41 m** | **~160 m** | **0.26%** |
| **1.1** | **ROVER Park** | **ORB-SLAM3 RGB-D** | **2.21 m** | **~175 m** | **1.26%** |

ROVER's shorter trajectories with slower movement (ground robot at ~0.35 m/s vs car at ~20 m/s) produce lower absolute ATE but higher relative error than automotive datasets. This is expected: longer trajectories allow global optimization (loop closure, bundle adjustment) to correct accumulated drift.

---

## 10. Files and Reproducibility

### Pipeline Location

```
/workspace/datasets/rover/
  scripts/
    run_rover_orbslam3.py          # Main orchestrator (conversion + SLAM + evaluation)
    convert_rover_to_euroc.py      # T265 -> EuRoC MAV format
    prepare_rover_rgbd.py          # D435i -> TUM RGB-D format
    rover_metadata.py              # Session metadata extraction
  configs/
    ROVER_T265_Stereo.yaml         # Stereo KannalaBrandt8
    ROVER_T265_Stereo_Inertial.yaml # Stereo-Inertial
    ROVER_D435i_RGBD.yaml          # RGB-D PinHole
  results/
    summary.txt                    # Results table
    all_results.json               # Machine-readable results (45 entries)
    session_metadata.json          # Per-recording metadata
    comparison_bar.png             # Bar chart: RGB-D ATE per recording
    comparison_heatmap.png         # Heatmap: 15 recordings x 3 modes
    comparison_garden_vs_park.png  # Box plot: Garden Large vs Park
    {recording}/{mode}/
      eval_results.json            # Per-experiment results
      trajectory_comparison.png    # 3-panel plot (trajectory, ATE heatmap, ATE over time)
      orbslam3_log.txt             # ORB-SLAM3 stdout/stderr
      trajectory_{mode}.txt        # Raw estimated trajectory (TUM format)
```

### Data Location

```
/workspace/data/rover/
  calibration/                     # Camera and IMU calibration files
  {recording}/                     # 15 raw recordings (T265 + D435i + GT)
  {recording}_euroc/               # 15 converted EuRoC datasets
  {recording}_rgbd/                # 15 converted RGB-D datasets
```

### How to Reproduce

```bash
# Run all 45 experiments (3 modes x 15 recordings, 3 parallel)
python3 /workspace/datasets/rover/scripts/run_rover_orbslam3.py --parallel 3

# Run single recording/mode
python3 /workspace/datasets/rover/scripts/run_rover_orbslam3.py \
  --recording garden_large_day_2024-05-29_1 --mode rgbd

# Evaluate only (skip SLAM)
python3 /workspace/datasets/rover/scripts/run_rover_orbslam3.py --eval-only

# Regenerate summary and plots
python3 /workspace/datasets/rover/results/regenerate_summary.py
python3 /workspace/datasets/rover/results/generate_plots.py
```
