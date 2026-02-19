# SLAM Pipeline - Experiment Log (RobotCar + Multi-Dataset)

## Overview

This project evaluates SLAM and visual localization methods across multiple outdoor datasets: Oxford RobotCar, 4Seasons, and ROVER. The goal is to find methods that produce accurate, complete trajectories across diverse conditions (seasons, lighting, weather).

### Results Summary

| # | Method | Dataset | ATE RMSE | Coverage | Verdict |
|---|--------|---------|----------|----------|---------|
| 0.7 | hloc (7 configs) | RobotCar Seasons | 64.5% correct | - | ALIKED+LG+OpenIBL best |
| 0.8 | ORB-SLAM3 Stereo | RobotCar (full) | 3.91 m | 72.7% | First successful visual SLAM |
| 0.9 | **ORB-SLAM3 Stereo-Inertial** | **4Seasons office_loop_1** | **0.93 m** | **99.99%** | **Best result** - real IMU @ 2000Hz |
| 1.0 | hloc self-test | 4Seasons office_loop_1 | 17.1% @ 0.5m/5° | - | Sparse model limitation |
| 1.1 | ORB-SLAM3 (3 modes) | ROVER (15 recordings) | 0.45 m median (RGB-D) | 73.3% | T265 fisheye stereo fails; D435i RGB-D works |
| 1.1b | ORB-SLAM3 Stereo PinHole | ROVER GL/day (undistort) | 0.53 m | 100% | Fisheye->PinHole fix works! All 3 modes OK |

---

## Experiment 1.1b: T265 Fisheye -> PinHole Fix

- **Date:** 2026-02-18
- **Problem:** T265 KannalaBrandt8 stereo completely failed (0/15) due to fisheye distortion + 6.35cm baseline
- **Solution:** Undistort T265 fisheye -> pinhole (640×480, 110° hFoV, fx=224.07) using `cv2.fisheye.initUndistortRectifyMap`. Zero distortion in output; ORB-SLAM3 handles stereo rectification via T_c1_c2.
- **Test recording:** garden_large_day_2024-05-29_1

### Results (garden_large_day)

| Mode | ATE RMSE | Scale | Tracking | KF | Loops |
|------|----------|-------|----------|----|-------|
| Stereo PinHole | **0.527 m** | 1.214 | 100% | 3892 | 4 |
| Stereo-Inertial PinHole | **0.652 m** | 1.221 | 100% | 1392 | 0 |
| RGB-D (reference) | 0.398 m | 0.977 | 100% | - | - |

### Key findings
- Undistorting fisheye -> pinhole completely solves the stereo initialization problem
- Scale ~1.21 (21% overestimate) - expected for 6.35cm baseline outdoors
- Stereo found 4 loop closures, demonstrating full SLAM pipeline functionality
- RGB-D still most accurate due to direct depth measurement

### Files
- Script: `scripts/rectify_t265_stereo.py`
- Configs: `configs/ROVER_T265_PinHole_Stereo.yaml`, `configs/ROVER_T265_PinHole_Stereo_Inertial.yaml`
- Results: `results/garden_large_day_2024-05-29_1/stereo_pinhole/`, `results/garden_large_day_2024-05-29_1/stereo_inertial_pinhole/`

---

## Experiment 1.1: ORB-SLAM3 Baseline on ROVER Dataset

- **Date:** 2026-02-18
- **Dataset:** ROVER (iis-esslingen/ROVER on HuggingFace) - 15 multi-season recordings across 2 locations
  - **garden_large:** 8 recordings (summer, autumn, winter, spring, day, dusk, night, night-light)
  - **park:** 7 recordings (summer, autumn, spring, day, dusk, night, night-light)
- **Robot:** Ground robot with Intel RealSense T265 (stereo fisheye 848x800 @ 30fps, IMU @ 264Hz) + Intel RealSense D435i (RGB-D 640x480 @ 30fps) + Leica Total Station GT (~2Hz)
- **Modes tested:** Stereo (T265 KannalaBrandt8), Stereo-Inertial (T265 + IMU), RGB-D (D435i PinHole)
- **Total experiments:** 45 (15 recordings x 3 modes), run 3 in parallel

### Configurations

**T265 Stereo/Stereo-Inertial (KannalaBrandt8):**
- Camera1/2: 848x800, fx=286.18/285.72, fy=286.39/285.94, cx=416.94/418.88, cy=403.27/401.01
- Distortion (equidistant): k1=-0.0115, k2=0.0502, k3=-0.0504, k4=0.0127
- Baseline: ~6.35 cm, ThDepth=20, 1500 ORB features
- IMU: NoiseGyro=0.00324, NoiseAcc=0.01741, Frequency=264

**D435i RGB-D (PinHole + radtan):**
- Camera: 640x480, fx=596.20, fy=593.14, cx=327.05, cy=245.16
- Distortion: k1=0.1572, k2=-0.4894, p1=-0.00075, p2=0.00037
- DepthMapFactor=1000, Stereo.b=0.05, 1500 ORB features

### Results

| Recording | Stereo | Stereo-Inertial | RGB-D ATE RMSE | RGB-D Scale |
|-----------|--------|-----------------|----------------|-------------|
| GL/autumn | FAIL (segfault) | FAIL | **0.365 m** | 0.981 |
| GL/day | FAIL (segfault) | FAIL | **0.398 m** | 0.977 |
| GL/dusk | FAIL (segfault) | FAIL | FAIL (Sophus NaN) | - |
| GL/night-light | FAIL (segfault) | FAIL (segfault) | **0.480 m** | 0.959 |
| GL/night | FAIL | FAIL | FAIL (segfault) | - |
| GL/spring | FAIL (segfault) | FAIL | **0.420 m** | 0.988 |
| GL/summer | FAIL | FAIL | **0.453 m** | 0.988 |
| GL/winter | FAIL (segfault) | FAIL | **0.367 m** | 0.980 |
| P/autumn | FAIL (segfault) | FAIL | **1.874 m** | 0.980 |
| P/day | FAIL | FAIL | FAIL (timeout) | - |
| P/dusk | FAIL (segfault) | FAIL (abort) | **1.629 m** | 1.036 |
| P/night-light | FAIL (segfault) | FAIL (abort) | FAIL (segfault) | - |
| P/night | FAIL | FAIL (segfault) | **6.553 m** | 0.929 |
| P/spring | FAIL (segfault) | FAIL (abort) | **0.537 m** | 1.031 |
| P/summer | FAIL | FAIL | **0.448 m** | 1.021 |

### Statistics

| Metric | Stereo | Stereo-Inertial | RGB-D |
|--------|--------|-----------------|-------|
| Success rate | 0/15 (0%) | 0/15 (0%) | 11/15 (73.3%) |
| ATE RMSE mean | - | - | 1.230 m |
| ATE RMSE median | - | - | 0.453 m |
| ATE RMSE best | - | - | 0.365 m (GL/autumn) |
| ATE RMSE worst | - | - | 6.553 m (P/night) |

**Per-location RGB-D:**
- **garden_large:** 6/8 success, mean 0.414 m, median 0.409 m (tight 0.37–0.48 m range)
- **park:** 5/7 success, mean 2.208 m, median 1.629 m (skewed by P/night outlier at 6.55 m)

### Key Findings

1. **T265 stereo fisheye (KannalaBrandt8) completely fails in ORB-SLAM3 outdoor SLAM.** All 15 recordings produced only 1 keyframe before crashing. The 6.35 cm baseline is too small for outdoor stereo matching with equidistant fisheye distortion. This is a fundamental hardware limitation, not a configuration issue.

2. **Stereo-Inertial mode cannot recover from stereo failure.** With the stereo front-end unable to initialize, IMU integration has no visual anchor. All 15 recordings failed similarly.

3. **D435i RGB-D is the only viable ORB-SLAM3 mode on ROVER.** 11/15 recordings succeeded with median ATE 0.45 m. The depth sensor compensates for the narrow baseline and provides direct metric scale.

4. **Garden Large location is significantly easier than Park.** Garden Large ATE clusters tightly at 0.37–0.48 m, while Park shows higher variance (0.45–6.55 m). The park route is likely longer and/or has more challenging geometry.

5. **Night conditions degrade RGB-D performance.** The worst results are P/night (6.55 m ATE) and the failures are GL/night (segfault), GL/dusk (NaN), P/night-light (segfault). Low-light reduces ORB feature quality while depth sensor remains functional, causing trajectory drift.

6. **Scale factors are consistently near 1.0** for successful RGB-D runs (0.93–1.04), confirming correct depth integration.

### Files

- **Scripts:** `scripts/convert_rover_to_euroc.py`, `scripts/prepare_rover_rgbd.py`, `scripts/run_rover_orbslam3.py`
- **Configs:** `configs/ROVER_T265_Stereo.yaml`, `configs/ROVER_T265_Stereo_Inertial.yaml`, `configs/ROVER_D435i_RGBD.yaml`
- **Results:** `results/summary.txt`, `results/all_results.json`
- **Plots:** `results/comparison_bar.png`, `results/comparison_heatmap.png`, `results/comparison_garden_vs_park.png`
- **Per-recording:** `results/{recording}/{mode}/eval_results.json`, `trajectory_comparison.png`, `orbslam3_log.txt`

---

## Experiment 1.0: hloc Self-Test on 4Seasons

- **Date:** 2026-02-17
- **Dataset:** 4Seasons office_loop_1 (self-test: query = reference)
- **Method:** hloc with ALIKED + LightGlue + NetVLAD
- **Result:** 17.1% correct @ 0.5m/5°, 59.1% @ 5m/10°
- **Conclusion:** Sparse SfM model (only 2813 points from 15177 images) limits localization precision

## Experiment 0.9: ORB-SLAM3 Stereo-Inertial on 4Seasons

- **Date:** 2026-02-17
- **Dataset:** 4Seasons office_loop_1 (3.7 km, 501s, 15177 frames)
- **Sensor:** Stereo 800x400 @ 30fps (30 cm baseline) + ADIS16465 IMU @ 2000 Hz
- **Result:** ATE RMSE **0.93 m**, scale 0.997, tracking 99.99%
- **Key:** Real 2000 Hz IMU enables correct pre-integration - best result across all experiments

## Experiment 0.8: ORB-SLAM3 Stereo on RobotCar

- **Date:** 2026-02-15
- **Dataset:** Oxford RobotCar 2014-11-28-12-07-13 (Bumblebee XB3, 1280x960, 16Hz, 24cm baseline)
- **Result:** ATE RMSE **3.91 m**, scale 0.966, tracking 72.7%
- **Key:** First successful visual SLAM - trajectory follows GT road shape. Stereo-Inertial failed (no raw IMU data, pseudo-IMU incompatible)

## Experiment 0.7: hloc on RobotCar Seasons

- **Date:** 2026-02-14
- **Dataset:** RobotCar Seasons v2 (1024x1024 undistorted)
- **Method:** 7 hloc configurations (SP+SG, SP+LG, ALIKED+LG with NetVLAD/OpenIBL)
- **Result:** ALIKED + LightGlue + OpenIBL best at **64.5%** correct localizations
