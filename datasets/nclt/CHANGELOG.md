# SLAM pipeline - experiment log

## overview

evaluating odometry and SLAM methods on the NCLT dataset (University of Michigan North Campus Long-Term, 4 seasonal sessions, ~3-7 km each). Looking for a method that produces accurate, complete trajectories across all seasons.

**Dataset:** Velodyne HDL-32E LiDAR (10 Hz), Ladybug3 omnidirectional camera (5 Hz, 6 cameras: 5 side + 1 top), MS25 IMU (34-50 Hz), wheel odometry (100 Hz), GPS/RTK. Ground truth from survey-grade SLAM with manual corrections.

### Results Summary

| # | Method | Dataset | ATE RMSE | Coverage | Verdict |
|---|--------|---------|----------|----------|---------|
| 0.1 | LiDAR ICP + GPS loop closure | NCLT (4 sessions) | 174 / 188 m | 100% | **Best NCLT method** - only one with recognizable trajectories |
| 0.2 | ORB-SLAM3 mono (Cam0) | NCLT | 2.9 m* | 0.5% | Failure - Cam0 is sky-facing |
| 0.3 | ORB-SLAM3 mono v2 (Cam0, f=221) | NCLT | 45.6 m* | 24% / 19% | Fragmented - 35 disconnected maps on spring |
| 0.4 | DROID-SLAM / DPVO / DPV-SLAM (Cam0) | NCLT | 110–194 m | 54–100% | Failure - trajectories don't follow road shape |
| 0.5 | ORB-SLAM3 tuned (Cam3/4/5, 23 configs) | NCLT | N/A | 28% max | Failure - no config achieves viable tracking |
| 0.7 | hloc visual localization (7 configs) | RobotCar Seasons | 64.5% correct | - | ALIKED+LG+OpenIBL best |
| 0.8 | **ORB-SLAM3 Stereo** | **RobotCar (full)** | **3.91 m** | **72.7%** | **First successful visual SLAM** - trajectory follows GT |
| 0.9 | **ORB-SLAM3 Stereo-Inertial** | **4Seasons office_loop_1** | **0.93 m** | **99.99%** | **Best result** - real IMU @ 2000Hz, scale 0.997 |

*Misleading ATE - covers tiny fraction of trajectory or comes from disconnected maps.

### things I learned the hard way

1. **Ladybug3 Cam0 is the TOP camera pointing at the sky.** Experiments 0.2-0.4 unknowingly tracked clouds and tree canopy. This single error invalidates all visual SLAM results from those experiments - discovered in exp 0.5 after I finally went through the camera images one by one.

2. **Switching to side cameras (Cam3/4/5) doesn't save visual SLAM.** After testing all 6 cameras with 23 parameter configs, the best tracking rate on 3000 frames was only 27.9%, and no trajectory followed the GT road shape.

3. **ORB-SLAM3 is extremely non-deterministic on this data.** Same config + same data produces tracking rates from 5% to 70% across runs. Systematic tuning is basically meaningless without many repetitions per config.

4. **Visual SLAM failures are hardware-level, not tuning:**
   - 5 Hz camera (methods expect 20-30 Hz)
   - ~120 deg FOV fisheye treated as pinhole (distortion coeffs not available)
   - No viable IMU fusion (MS25 at 34 Hz too slow for ORB-SLAM3 VIO)
   - Poor/variable image quality (overexposed, low-texture frames)

5. **LiDAR ICP is the only odometry that works** - 174 m spring / 188 m summer ATE with 100% trajectory coverage. Drift is big but trajectories are recognizable and complete.

---

## Week 1: Data Pipeline
- **Goal:** Build data loaders for NCLT dataset
- **Data:** 4 sessions: 2012-01-08 (winter), 2012-04-29 (spring), 2012-08-04 (summer), 2012-10-28 (autumn)
- **Loaders built:**
  - `VelodyneLoader` - HDL-32E binary format (8 bytes/point: x,y,z uint16 + intensity,laser_id uint8)
  - `GroundTruthLoader` - CSV (utime, x, y, z, qx, qy, qz; qw computed from unit norm)
  - `SensorLoader` - IMU (ms25, 50Hz), GPS (5Hz), RTK GPS (~1Hz), odometry (10/100Hz), KVH gyro, wheels
  - `HokuyoLoader` - 2D LiDAR (1081 ranges, FOV -135° to +135°)
- **Key finding:** Ground truth CSV has 7 columns (no qw), qw = sqrt(1 - qx²-qy²-qz²)
- **Results:** Verified data loading, coordinate conversions, calibration

## Week 2: LiDAR Odometry Baseline

### Experiment 2.1: Custom ICP (sparse - every 10th scan)
- **Date:** 2025-02-10
- **Session:** 2012-04-29
- **Method:** Point-to-plane ICP (Open3D), frame-to-frame, 0.5m voxel, 50 iterations, 2.0m threshold
- **Sampling:** Every 10th scan (500 of 12,971 scans)
- **Parameters:** `voxel_size=0.5, max_iter=50, threshold=2.0`
- **Results:**
  - ATE RMSE: 191.1m, ATE Mean: 179.0m
  - RPE Trans RMSE: 3.22 m/frame, RPE Rot RMSE: 14.85 deg/frame
  - Trajectory length: 1,336.9m (GT: 1,273.1m, +5.0%)
  - Processing: 500 poses, ~15s, ~33 scans/s

Way too sparse. Every 10th scan is a ~2s gap, scan-to-scan overlap is poor and ICP loses lock on corners.

### Experiment 2.2: Custom ICP (dense - every 2nd scan)
- **Date:** 2025-02-10
- **Session:** 2012-04-29
- **Method:** Point-to-plane ICP, 0.3m voxel, 100 iterations, 1.5m threshold
- **Sampling:** Every 2nd scan (6,486 of 12,971 scans)
- **Parameters:** `voxel_size=0.3, max_iter=100, threshold=1.5`
- **Results:**
  - ATE RMSE: 419.0m, ATE Mean: 368.6m
  - RPE Trans RMSE: 0.57 m/frame, RPE Rot RMSE: 5.67 deg/frame
  - Trajectory length: 3,276.4m (GT: 3,184.3m, +2.9%)
  - Processing: 6,482 poses, 229.5s, 28.3 scans/s

82% RPE improvement over sparse is nice but doesn't save us - 13% drift per km over a 3.3 km trajectory gives 419 m ATE. No loop closure = unbounded drift, so the next step is obvious.

### Experiment 2.3: ICP + Loop Closure + Pose Graph
- **Date:** 2025-02-10
- **Session:** 2012-04-29
- **Method:**
  1. Scan Context descriptors (60 sectors × 20 rings, max range 80m) with rotation-invariant matching (column shifts)
  2. FPFH + RANSAC global registration for loop closure verification
  3. ICP refinement (point-to-plane, 1.0m threshold)
  4. 2D pose graph optimization (sparse Gauss-Newton with Levenberg-Marquardt damping)
- **Parameters:** `SC_thresh=0.35, RANSAC_fitness>0.20, ICP_fitness>0.30, LC_dedup_window=50, PG_lc_weight=5.0, PG_damping=1e-3`
- **Results:**
  - Loop closures found: 7 (from 7 deduplicated candidates)
  - ATE RMSE: 417.7m (-0.3% vs ICP-only)
  - RPE Trans RMSE: 0.71 m/frame (+25% - optimization redistributed error)
  - Optimization converged in 50 iterations, cost 860.6

Only 7 loop closures is way too few to fix anything. Scan context is clearly too conservative on NCLT - the descriptor probably isn't invariant enough to the vegetation changes between revisits. Need either GPS-aided loop closure or a learned descriptor.
- **Files:** `results/week2_full/plots/trajectory_icp_vs_lc.png`, `results/week2_full/icp_lc_trajectory.txt`

### KISS-ICP Attempt (abandoned)
- **Date:** 2025-02-10
- **Method:** KISS-ICP v1.2.3 on full session
- **Issue:** Voxel map growth caused processing to drop from 74 it/s to 2 it/s after 100 scans. Estimated 40+ hours for full session. Memory grew to 12GB+
- **Conclusion:** KISS-ICP impractical for long NCLT trajectories without map management

## Week 3: Odometry-Aided ICP + Local Map + GPS Loop Closure

### Critical Bug Fixes in Data Loaders
- **ms25.csv**: Had 10 columns (utime, mag_x/y/z, accel_x/y/z, rot_x/y/z), NOT 11 with quaternions. Previous loader mislabeled magnetometer values as quaternions - feeding these to SLERP produced garbage rotations
- **ms25_euler.csv**: Had 4 columns (utime, roll, pitch, yaw), NOT 10. Previous loader assigned wrong column names
- **odometry_mu_100hz.csv**: 7 columns are integrated poses (utime, x, y, z, roll, pitch, yaw), NOT velocities. Previous loader had wrong column semantics
- **Open3D transform() in-place bug**: `pcd.transform(T)` modifies the point cloud in-place. v1 code transformed source to global, then tried to undo - both references pointed to same object. Fixed by passing T_pred as ICP init parameter instead of transforming point clouds

### Experiment 3.1: IMU Preintegration + Local Map (BROKEN - v1)
- **Date:** 2026-02-10
- **Session:** 2012-04-29
- **Method:** IMU preintegration (SLERP orientation + trapezoidal acceleration integration) for ICP initial guess, sliding-window local map (20 scans), ground removal (RANSAC), GPS loop closure, pose graph optimization
- **Sampling:** Every 2nd scan (6,486 of 12,971 scans)
- **Results:**
  - ATE RMSE: 397.9m (only 5% improvement from 419m baseline)
  - RPE Trans RMSE: 2.57 m/frame (4.5x worse than baseline!)
  - Trajectory length: 10,538m (GT: 3,184m, **3.3x too long**)

Turned out the ms25.csv magnetometer values were being fed to SLERP as quaternions (wrong column order in the loader). Random orientations -> ICP diverges -> corrupted local map -> cascading errors. Fixed the loader and redid it, see 3.2 below.

### Experiment 3.2: Odometry-Aided ICP + Local Map + GPS Loop Closure (v2)
- **Date:** 2026-02-10
- **Session:** 2012-04-29
- **Method:**
  1. Wheel odometry prediction (100Hz integrated poses, interpolated to LiDAR timestamps) for ICP initial guess
  2. Ground plane removal (RANSAC, distance threshold 0.25m, vertical normal check)
  3. Sliding-window local map (20 scans, 0.5m voxel downsampled) for scan-to-map ICP
  4. GPS-aided loop closure (RTK GPS lat/lon -> ENU, 15m proximity radius, minimum 200-frame gap)
  5. ICP verification of GPS loop closure candidates (fitness > 0.3)
  6. 2D pose graph optimization (sparse Gauss-Newton + LM damping)
- **Parameters:** `ICP_voxel=0.3, ICP_threshold=1.5, ICP_max_iter=80, localmap_size=20, localmap_voxel=0.5, ground_dist=0.25, GPS_LC_radius=15.0m, GPS_LC_min_gap=200, PG_lc_weight=5.0, PG_damping=1e-3`
- **Results:**
  - ATE RMSE: **174.3m** (58.4% improvement from 419m baseline)
  - ATE Mean: 155.5m
  - RPE Trans RMSE: 0.5637 m/frame (1.1% improvement)
  - RPE Rot RMSE: 4.61 deg/frame
  - Trajectory length: 3,239.2m (GT: 3,184.3m, +1.7%)
  - GPS loop closures: 107 verified (107 candidates, 100% verification rate)
  - Pose graph: converged in 50 iterations, final cost 0.6
  - Processing: ~20 minutes total
- **what worked vs Week 2:**
  - Odometry prediction is way better ICP init than identity - prevents local minima
  - Local map matching adds more geometric context than pure frame-to-frame
  - Ground removal kills the planar ambiguity issue
  - GPS loop closures (107 vs 7 from scan context) give us many more global constraints
- **Remaining issues:** 174 m ATE still far from <50 m target. Drift accumulates in areas without GPS coverage. Loop closures cluster in GPS-dense regions, leaving long unconstrained segments.
- **Files:** `results/week3/plots/`, `results/week3/odom_localmap_trajectory.txt`, `results/week3/odom_localmap_lc_trajectory.txt`

## Week 0: Cross-Season Baseline Evaluation

### Project Reorganization
- Restructured project into clean `src/`, `scripts/`, `configs/`, `results/` layout
- All SLAM modules under `src/slam/`, data loaders under `src/data_loaders/`, evaluation under `src/evaluation/`
- Results directories renamed: `week2_icp_baseline`, `week2_icp_loop_closure`, `week3_imu_gps`, `week0_seasonal`, `week0_orbslam3`

### Experiment 0.1: LiDAR Pipeline Cross-Season Evaluation
- **Date:** 2026-02-11
- **Sessions:** All 4 NCLT sessions (winter, spring, summer, autumn)
- **Method:** Best LiDAR pipeline from Week 3 (odom-aided ICP + local map + GPS loop closure)
- **Parameters:** `ICP_voxel=0.3, ICP_threshold=1.5, ICP_max_iter=80, localmap_size=20, localmap_voxel=0.5, ground_dist=0.25, GPS_LC_radius=15.0m, GPS_LC_min_gap=200`
- **Results:**

| Session | Season | ATE RMSE | RPE Trans | Loop Closures | Traj Length | GT Length | Time |
|---------|--------|----------|-----------|---------------|-------------|----------|------|
| 2012-01-08 | Winter | **30.2m** | 0.666 m/f | 352 | 7341m | 6497m | 2770s |
| 2012-04-29 | Spring | 174.0m | 0.565 m/f | 107 | 3237m | 3184m | 1199s |
| 2012-08-04 | Summer | 188.2m | 0.630 m/f | 201 | 6006m | 5493m | 1962s |
| 2012-10-28 | Autumn | 151.1m | 0.685 m/f | 242 | 6574m | 5684m | 2115s |

- **Mean ATE RMSE: 135.9m**
- **Best:** Winter 2012-01-08 (30.2m) - cleanest GPS coverage, most loop closures
- **Worst:** Summer 2012-08-04 (188.2m) - foliage occludes GPS, fewer effective loop closures
- **Key observations:**
  - Winter session dramatically outperforms others (5-6x better ATE)
  - Seasonal vegetation affects GPS availability -> fewer/worse loop closures
  - All sessions show trajectory inflation (estimated > ground truth length) indicating residual drift
  - RPE is relatively consistent across seasons (~0.6 m/frame), confirming drift is local-ICP quality
  - ATE variation is driven by loop closure quality, not local odometry
- **Files:** `results/week0_seasonal/`, `results/week0_seasonal/plots/`, `results/week0_seasonal/comparison_table.txt`

### Experiment 0.2: ORB-SLAM3 Monocular Baseline
- **Date:** 2026-02-11
- **Sessions:** 2012-04-29 (spring), 2012-08-04 (summer) - only sessions with Ladybug3 camera images
- **Method:** ORB-SLAM3 monocular mode on Ladybug3 Camera 0 images (resized to 808x616)
- **Setup challenges:**
  - Built ORB-SLAM3 from source with C++17 (fixed `bool++` in LoopClosing.h)
  - Camera config required `File.version: "1.0"` for Camera1.fx format
  - Viewer params must be floats, Camera.fps must be integer
  - Pangolin needs X display -> used `xvfb-run` + `QT_QPA_PLATFORM=offscreen` for headless operation
  - OpenCV Qt plugin conflict -> `QT_QPA_PLATFORM=offscreen` + removed `QT_PLUGIN_PATH`
  - Full resolution (1616x1232) too slow -> half resolution (808x616) with halved intrinsics
- **Parameters:** `nFeatures=1500, scaleFactor=1.2, nLevels=8, iniThFAST=20, minThFAST=7, fx=fy=200, cx=404, cy=308`
- **Results:**

| Session | Poses Tracked | ATE RMSE | RPE Trans | Scale | Status |
|---------|---------------|----------|-----------|-------|--------|
| 2012-04-29 | 23 / 5000 | 2.9m | 0.892 m/f | 4.454 | Very sparse tracking |
| 2012-08-04 | 0 / 5000 | - | - | - | Complete failure |

Bottom line: ORB-SLAM3 mono is unusable on NCLT Ladybug3. Only 23/5000 KFs tracked on spring (0.5%), summer failed entirely with 33 map resets. The 2.9 m ATE on 23 poses looks great but only covers a tiny segment, so it's meaningless. The camera is fisheye approximated as pinhole (significant outer-region distortion), forward-facing on a vehicle that turns a lot -> constant tracking loss. Would need a proper fisheye/equirectangular camera model + maybe stereo.
- **Files:** `results/week0_orbslam3/2012-04-29/`, `configs/nclt_ladybug3_half.yaml`

### Experiment 0.3: ORB-SLAM3 v2 - Proper Calibration + Mono-Inertial Investigation
- **Date:** 2026-02-11
- **Sessions:** 2012-04-29 (spring), 2012-08-04 (summer)
- **Goal:** Redo ORB-SLAM3 evaluation with proper camera calibration and mono-inertial mode
- **Camera calibration:**
  - NCLT `cam_params.zip` unavailable (website 404, S3 403) - no official intrinsics
  - COLMAP self-calibration on 300 sequential images from Cam0 -> SIMPLE_PINHOLE f≈221, cx=404, cy=309
  - Previous v1 config (fx=200) was actually close; Ladybug3 has ~120° FOV per camera, not ~80° as initially assumed
  - Half resolution 808×616 images used throughout
- **Mono-inertial investigation (failed):**
  - MS25 IMU at 34 Hz (ORB-SLAM3 expects ~200 Hz), only ~7 IMU samples per frame at 5 Hz camera
  - Image quality issues: first 500 images completely overexposed (mean pixel = 254.9), only 34% of images have ≥500 ORB features with default FAST threshold
  - ORB-SLAM3 mono-inertial requires 50 inlier matches before IMU init (vs 30 for pure mono)
  - Pre-IMU-init: any tracking failure triggers full map reset, preventing recovery
  - Source code modifications attempted: lowered inlier threshold to 20, disabled pre-init map resets
  - Result: IMU partially initializes (VIBA 1/2 observed) but never stabilizes - constant map resets
  - **Root cause:** combination of low camera FPS (5 Hz), low IMU rate (34 Hz), and poor image quality makes mono-inertial infeasible on NCLT Ladybug3
- **Pure monocular mode (successful):**
  - Aggressive config: 3000 ORB features, FAST thresholds 10/3 (vs default 20/7), PinHole f=221
  - Evaluation: Sim(3) Umeyama alignment (unknown scale in monocular)
- **Parameters:** `nFeatures=3000, scaleFactor=1.2, nLevels=8, iniThFAST=10, minThFAST=3, fx=fy=221, cx=404, cy=309`
- **Results:**

| Run | Poses | Tracking | GT covered | Maps | ATE RMSE | Notes |
|-----|-------|----------|------------|------|----------|-------|
| Spring 3k (best 10 min segment) | 2981/3000 | 99.4% | 791m of 3186m (25%) | 1 | 45.6m | Best 10-min segment only |
| Spring full (21k) | 5038/21010 | 24.0% | 1305m of 3186m (41%) | 35 | 100.3m* | *Poses from 35 separate coordinate frames |
| Summer 5k | 959/5000 | 19.2% | 221m of 5497m (4%) | 8 | 54.8m* | *Poses from 8 separate coordinate frames |

  *Full spring and summer ATE values are **unreliable**: poses come from multiple disconnected maps (different coordinate systems), and a single Sim(3) transform cannot meaningfully align them. These numbers should not be compared with LiDAR ICP results.

- **Comparison with LiDAR ICP (NOT directly comparable):**
  - LiDAR ICP: 174.0m ATE over **full 3.2 km** spring trajectory (100% coverage)
  - ORB-SLAM3 best: 45.6m ATE over **791m** best 10-min segment (25% of trajectory)
  - These are not comparable: LiDAR is evaluated on 4× longer trajectory where drift accumulates more. Additionally, monocular scale is unknown and recovered via Sim(3) fitting

So what did we learn? Mono-inertial on NCLT is infeasible - 5 Hz camera + 34 Hz IMU + bad images means constant map resets and the IMU never fully initializes. Pure mono does work on short well-behaved segments (99.4% tracking on best 10 min with aggressive ORB). But full session is a mess: 24% spring, 19% summer, dozens of fragmented maps. The bottleneck is image quality - first 500 images are overexposed, many later ones are low-texture, only ~34% of frames have ≥500 ORB features. And monocular scale is all over the place (factors 23-137 across runs) - only recoverable via GT alignment.

Overall ORB-SLAM3 mono is not usable for full-session NCLT. LiDAR ICP stays the more reliable choice despite higher absolute ATE, because at least it gives 100% coverage. The 45.6 m ATE on the best 10-min segment shows visual tracking CAN work on these images under good conditions, but the camera's inconsistent quality kills any long-term operation.
- **Files:** `results/week0_orbslam3_v2/`, `configs/nclt_mono_aggressive.yaml`, `scripts/run_week0_orbslam3_v2.py`, `scripts/prepare_orbslam3_mono_inertial.py`, `scripts/calibrate_camera_colmap.py`

### Experiment 0.4: Deep Visual SLAM Benchmark
- **Date:** 2026-02-12
- **Sessions:** 2012-04-29 (spring), 2012-08-04 (summer)
- **Methods:** DROID-SLAM, DPVO (visual odometry), DPV-SLAM (DPVO + loop closure)
- **Camera:** Ladybug3 Cam0, f=221, cx=404, cy=308, 808x616 half-res (COLMAP calibrated)
- **Evaluation:** Sim(3) Umeyama alignment (monocular scale unknown), ATE RMSE
- **Results:** (6/6 method/session combinations succeeded)

| Method | Session | Poses | ATE RMSE | Scale | Coverage | Runtime | GPU |
|--------|---------|-------|----------|-------|----------|---------|-----|
| DROID-SLAM | spring | 705/1847 | 110.0m | 3.27 | 60.4% | 13min | 10.3GB |
| DROID-SLAM | summer | 933/933 | 142.3m | 1.46 | 54.1% | 20min | 10.1GB |
| DPVO | spring | 12986/21510 | 142.0m | 0.33 | 100.0% | 11min | 0.6GB |
| DPVO | summer | 21238/21238 | 183.4m | 0.36 | 89.5% | 10min | 0.6GB |
| DPV-SLAM | spring | 726/1780 | 166.2m | 0.12 | 97.3% | 22min | 3.6GB |
| DPV-SLAM | summer | 21858/21858 | 193.6m | 0.26 | 91.5% | 28min | 3.9GB |

- **Reference LiDAR ICP:** Spring 174.0m, Summer 188.2m (100% coverage)

- **Bottom line - no method actually works here:**
  - **Trajectory shape:** Visual inspection of trajectory plots shows NO method produces trajectories that follow the ground truth road shape. GT traces clear structured roads; all estimated trajectories are tangled, incoherent blobs
  - **Scale factors show the trajectories are broken:** DROID-SLAM 3.27× (world appears 3.3× too big), DPVO 0.33× (3× too small), DPV-SLAM 0.12× (8× too small). These are not "monocular scale ambiguity" - these are methods that cannot estimate consistent motion
  - **ATE numbers are deceptive:** Sim(3) Umeyama alignment finds the best rotation+translation+scale to minimize error even on completely wrong trajectory shapes. DROID-SLAM's 110m "beating" LiDAR's 174m is meaningless - it covers only 60% of the trajectory and the shape is wrong
  - **DROID-SLAM global BA skipped:** terminate() OOM'd (needs 21GB, only 16GB available). Without final bundle adjustment, keyframe poses are unoptimized raw tracking output
  - **DPV-SLAM loop closure hurt performance:** Scale went from 0.33 (DPVO) to 0.12 (DPV-SLAM) on spring, confirming false loop closures introduced incorrect constraints
  - **ATE vs distance plot shows no structure:** Errors oscillate 50-450m with no pattern - consistant with a non-functional system, not meaningful drift

What goes wrong:
  1. 5 Hz camera rate - all three methods are designed/trained for 20-30 Hz video. At 5 Hz inter-frame motion is 4-6x larger than expected, which breaks optical flow and the learned correspondences
  2. Fisheye distortion left unhandled - Ladybug3 is ~120 deg FOV fisheye but we passed the images raw with pinhole intrinsics (f=221); outer-region distortion corrupts feature matching
  3. Can't even undistort - NCLT `cam_params.zip` is gone (website 404, S3 403)
  4. Image quality - first ~500 frames overexposed (mean pixel close to 255), lots of later frames low-texture
  5. Training domain mismatch - DROID-SLAM/DPVO are trained on indoor (TartanAir) and standard datasets; NCLT outdoor fisheye is way out of distribution

Compared to ORB-SLAM3 in exp 0.3: ORB-SLAM3's best tracked bits (45.6 m ATE on 791 m) at least follow the GT road shape locally - deep methods don't. Deep methods track more frames (DPVO 100%) but produce globally incoherent trajectories. ORB-SLAM3 fails by losing tracking; deep methods fail by tracking confidently but wrongly, which is arguably worse.

So: deep visual SLAM (DROID-SLAM, DPVO, DPV-SLAM) is not viable on NCLT Ladybug3. A 5 Hz fisheye at 808x616 is just fundamentally incompatible with methods built for 20+ Hz pinhole cameras. The ATE numbers (110-194 m) should NOT be read as meaningful accuracy - they're artifacts of Sim(3) alignment on wrong-shaped trajectories. LiDAR ICP (174-188 m) remains the only method that produces recognizable trajectories here, despite its own significant drift.
- **Files:** `results/week0_visual_slam/`

### Experiment 0.5: ORB-SLAM3 Systematic Parameter Tuning
_took me 2 days to realise Cam0 was pointed at the sky, and thats what broke 0.2-0.4. funny in retrospect but very not funny at the time_
- **Date:** 2026-02-13
- **Session:** 2012-04-29 (spring)
- **Goal:** Determine whether ORB-SLAM3 monocular can work on NCLT Ladybug3 with proper camera selection and optimized parameters
- **Critical discovery:** Ladybug3 Cam0 is the TOP camera (sky-facing). ALL previous visual SLAM experiments (0.2, 0.3, 0.4) used sky images - this was the root cause of the visual SLAM failures.
- **Method:** 4-phase walkthrough:
  1. **Phase 0 - Camera investigation:** Analyzed all 6 Ladybug3 cameras (brightness, contrast, ORB features, orientation)
  2. **Phase 1 - Quick validation (1000 frames):** Tested top cameras with baseline and CLAHE preprocessing
  3. **Phase 2 - Parameter sweep (3000 frames, 23 runs):** nFeatures (5 values), FAST thresholds (5), scale/levels (6), CLAHE (3), camera swap (2), resolution (2)
  4. **Phase 3 - Full session (21510 frames):** Attempted with best config - timed out after 45 minutes
- **Camera ranking:**
  - Cam0: 851 ORB, **sky-facing (useless)**
  - Cam3: 2193 ORB, buildings/structured scenes - best on short runs
  - Cam4: 1143 ORB, highest contrast (75.1) - best on longer runs (27.9% tracking)
  - Cam5: 2349 ORB, high contrast (72.0) - variable results (5–70% across runs)
  - Cam1: 1632 ORB, very dark (brightness=34) - 7.6% even with CLAHE
- **Results:**

| Best run | Camera | Config | Tracking | Keyframes | ATE |
|----------|--------|--------|----------|-----------|-----|
| fast_20_7 | Cam5+CLAHE | nFeat=1000, FAST 20/7 | 26.0% | 779 | N/A |
| Cam4 swap | Cam4 | nFeat=1000, FAST 20/7 | **27.9%** | **838** | N/A |
| scale_1.2_10 | Cam5+CLAHE | nFeat=1000, scale 1.2, 10 levels | 14.2% | 426 | N/A |
| Phase 1 best | Cam5+CLAHE | baseline | 69.7% (1K frames) | 697 | N/A |

- **Key findings:**
  - Max tracking rate on 3000 frames: **27.9%** (Cam4) - far below usable
  - Extreme non-determinism: same config gives 2-4x different tracking rates across runs (Cam5+CLAHE: 29%-70%)
  - More features != better tracking: nFeatures=5000 gave the worst result (0.9%); Cam1 with 4895 ORB features tracked only 7.6%
  - CLAHE is unreliable: big improvements on 1000 frames vanished on 3000 frames
  - No trajectory follows GT road shape, regardless of params - all estimated paths are chaotic blobs
  - Full session timed out: Phase 3 on 21510 frames blew past the 45-minute limit

Ultimately ORB-SLAM3 mono is NOT viable on NCLT Ladybug3 and no amount of parameter tuning will fix it. The root causes (5 Hz fisheye, unmodeled distortion, no IMU fusion, extreme non-determinism) are all hardware/data-level. LiDAR ICP (174 m ATE, 100% coverage) remains the only functional odometry method on this dataset.
- **Files:** `results/week0_orbslam3_tuned/`, `scripts/run_orbslam3_tuning.py`

### Experiment 0.7: hloc Visual Localization on RobotCar Seasons
- **Date:** 2026-02-14
- **Dataset:** RobotCar Seasons v2 (Grasshopper camera, 1024x1024, undistorted)
- **Method:** hloc (Hierarchical Localization) - 7 feature/matcher configurations
- **Best result:** ALIKED + LightGlue + OpenIBL -> 64.5% correct localizations
- **Files:** `results/week0_hloc/`

### Experiment 0.8: ORB-SLAM3 Stereo on Full RobotCar Dataset
- **Date:** 2026-02-15
- **Dataset:** Full Oxford RobotCar (2014-11-28-12-07-13, overcast-reference)
- **Sensor:** Bumblebee XB3 wide stereo pair (1280×960, 16 Hz, 24 cm baseline)
- **Ground truth:** INS navigation solution (NovAtel SPAN-CPT, 50 Hz) interpolated to camera times
- **Data preparation:**
  1. Downloaded GPS/INS + stereo_wide_left/right from mrgdatashare.robots.ox.ac.uk
  2. Demosaiced raw Bayer GBRG -> grayscale, undistorted via SDK LUT maps
  3. Converted to EuRoC directory format (6001 stereo pairs, ~375 seconds)
  4. Generated ground truth in TUM format from INS (1401.1 m total distance)
- **Stereo-Inertial attempt (FAILED):**
  - RobotCar does NOT release raw IMU data (accel/gyro) - only fused INS solution
  - Synthesized pseudo-IMU by differentiating INS velocities (-> accel) and Euler angles (-> gyro)
  - ORB-SLAM3 bugs found: `IMU.fastInit` not read in new Settings path; fixed in Tracking.cc + Settings.cc/h
  - Even with fixes (lowered accel threshold 0.5->0.02, disabled motion check), IMU init diverges:
    - Pseudo-IMU too smooth for acceleration init (all frames near [-0.003, -0.058, -9.804])
    - After VIBA 2 (second visual-inertial BA), system segfaults - IMU optimization corrupts map
  - **Root cause:** Pseudo-IMU from differentiated INS is fundamentally incompatible with ORB-SLAM3's tightly-coupled visual-inertial system. The preintegration model assumes raw sensor noise, not smoothed navigation outputs
- **Stereo-only mode (SUCCESS):**
  - Config: PinHole fx=fy=983.044, cx=643.647, cy=493.379, no distortion (pre-undistorted via LUT), nFeatures=2000
  - Stereo baseline: 0.239983 m from calibration
  - Run completed with 4 maps (1459 + 40 + 199 + 641 KFs)
  - Tracking lost permanently after 309.7s of 420.6s total (final 111s = no tracking)
- **Parametars:**
  - `Camera.type: PinHole, fx=fy=983.044, cx=643.647, cy=493.379`
  - `Stereo.ThDepth: 40.0, nFeatures=2000, scaleFactor=1.2, nLevels=8`
- **Results:**

| Metric | Value |
|--------|-------|
| Tracked frames | 4365 / 6001 (72.7%) |
| GT trajectory length | 834.1 m (matched subset) |
| Sim(3) scale | 0.9658 |
| ATE RMSE (Sim3) | **3.91 m** |
| ATE Mean | 3.69 m |
| ATE Median | 3.87 m |
| ATE Max | 6.75 m |
| ATE RMSE (SE3) | 5.91 m |
| RPE Trans RMSE (1 frame) | 0.4353 m |
| RPE Rot RMSE (1 frame) | 0.5545 deg |
| RPE Trans RMSE (~1 sec) | 6.7068 m |
| Maps created | 4 |

Takeaways: scale 0.97 means stereo gives correct metric scale with only 3.4% error, nice. Unlike NCLT, the trajectory actually follows the GT road shape. 3.91 m ATE RMSE is reasonable for stereo-only SLAM over 834 m without IMU/GPS fusion. The 72.7% tracking is because we lost the final 27% of the sequence (111 s) - accumulated map fragmentation. RPE at 1 s (6.7 m) is high because of relocalization discontinuities; the trajectory has jump artifacts from map switches. Stereo-Inertial mode just isn't viable without raw IMU data - pseudo-IMU from differentiated INS systematically breaks ORB-SLAM3's tightly-coupled VI optimizer.

RobotCar stereo works way better than NCLT mono fisheye because: (a) stereo gives metric scale, (b) 16 Hz vs 5 Hz camera, (c) pre-undistorted pinhole images, (d) structured urban scenes with good texture.
- **Files:** `datasets/robotcar/results/robotcar_orbslam3/`, `datasets/robotcar/configs/RobotCar_Stereo.yaml`, `datasets/robotcar/configs/RobotCar_Stereo_Inertial.yaml`, `datasets/robotcar/scripts/evaluate_robotcar_orbslam3.py`, `datasets/robotcar/scripts/synthesize_imu.py`, `datasets/robotcar/scripts/prepare_stereo_euroc.py`, `datasets/robotcar/scripts/make_ground_truth.py`

### Experiment 0.9: ORB-SLAM3 Stereo-Inertial on 4Seasons
- **Date:** 2026-02-17
- **Dataset:** 4Seasons - office_loop_1 (recording_2020-03-24_17-36-22, Spring, sunny)
- **Sequence:** ~3.7 km outdoor route, 501 s, 15177 stereo frames @ 30 FPS
- **Sensor setup:** Stereo 800×400 (30 cm baseline), ADIS16465 IMU @ 2000 Hz, RTK-GNSS ground truth
- **Method:** ORB-SLAM3 Stereo-Inertial (IMU tightly coupled)
- **Config:** PinHole fx=fy=501.48, 1500 ORB features, IMU noise from ADIS16465 datasheet, fastInit=1
- **Key fix:** `IMU.fastInit` parameter was never read in ORB-SLAM3's `newParameterLoader` path - fixed in Settings.cc/h and Tracking.cc (discovered in Experiment 0.8)
- **Results:**

| Metric | Value |
|--------|-------|
| ATE RMSE (Sim3) | **0.93 m** |
| ATE Mean | 0.82 m |
| ATE Median | 0.75 m |
| ATE Max | 3.06 m |
| Sim3 Scale | 0.997 |
| RPE RMSE | 0.40 m |
| Tracking rate | 99.99% (15175/15177) |
| Keyframes | 3535 |
| Loop closures | Multiple detected |

This is the best result so far - sub-meter ATE on a 3.7 km route. Scale 0.997 confirms real IMU integration is working (vs the pseudo-IMU failure in 0.8). 99.99% tracking is near-perfect, only 2 frames lost out of 15177. The real 2000 Hz IMU is the key differentiator vs RobotCar's pseudo-IMU: proper preintegration, correct motion estimation, stable VIBA optimization. Loop closures are detected but some get rejected ("BAD LOOP") due to large rotation discrepancy - could probably tune thresholds further.

Compared to 0.8 (RobotCar stereo-only): 4x better ATE (0.93 vs 3.91), near-perfect tracking (99.99% vs 72.7%), correct scale (0.997 vs 0.966).
- **Files:** `datasets/robotcar/results/4seasons/office_loop_1/`, `datasets/robotcar/configs/4Seasons_Stereo_Inertial.yaml`, `datasets/robotcar/scripts/convert_4seasons_to_euroc.py`, `datasets/robotcar/scripts/evaluate_4seasons.py`, `datasets/robotcar/scripts/run_4seasons_experiment.sh`
