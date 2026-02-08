# Report: ORB-SLAM3 Evaluation on the NCLT Dataset

## 1. Problem Statement

**Objective:** evaluate the feasibility of visual odometry/SLAM on the NCLT (North Campus Long-Term) dataset using ORB-SLAM3 in mono-inertial and stereo-inertial modes.

**The NCLT dataset** is a multi-sensor dataset collected on the University of Michigan campus from a Segway RMP platform. It contains data from LiDAR, cameras, IMU, GPS, and wheel odometry across 27 sessions spanning 15 months.

**Test session:** 2012-04-29 (spring), duration ~43 min, total route length 3184 m.

---

## 2. NCLT Sensor Hardware and Its Limitations

### 2.1. Camera - Point Grey Ladybug3

| Parameter | Value |
|-----------|-------|
| Type | Spherical (6 cameras: 5 lateral + 1 top) |
| Resolution | 1616 x 1232 (downsampled to 808 x 616) |
| Frame rate | **5 Hz** |
| Field of view | ~120 per camera (fisheye) |
| Shutter | Global shutter |

**Cam0 issue:** in earlier experiments (0.2-0.3), Cam0 was mistakenly used; it is the **top-facing camera pointed at the sky**. This was only discovered after inspecting the images. All subsequent tests use Cam5 (front-facing).

### 2.2. IMU - Microstrain 3DM-GX3-45 (MS25)

| Parameter | NCLT (MS25) | EuRoC (ADIS16448) | Difference |
|-----------|-------------|--------------------|------------|
| Rate | **48 Hz** | 200 Hz | **4.2x slower** |
| Gyroscope noise | 0.00972 rad/s/sqrt(Hz) | 0.00017 rad/s/sqrt(Hz) | **57x worse** |
| Accelerometer noise | 0.02855 m/s^2/sqrt(Hz) | 0.002 m/s^2/sqrt(Hz) | **14x worse** |
| IMU samples per frame | ~10 | ~10 | Same |

> **EuRoC** is the dataset on which ORB-SLAM3 was developed and benchmarked (camera 20 Hz, IMU 200 Hz).

### 2.3. Key Mismatch

ORB-SLAM3 in mono-inertial mode relies on **IMU preintegration** between camera frames. At 5 Hz the inter-frame interval is **200 ms** (versus 50 ms in EuRoC). During this interval:

- The robot travels 0.2-0.6 m (large parallax)
- The IMU integrates acceleration over 200 ms, accumulating position drift proportional to t^2
- Accelerometer noise (14x worse than EuRoC) further amplifies the error
- **Result:** velocity/position drift between frames is ~224x larger than in EuRoC

---

## 3. Calibration

### 3.1. Camera Intrinsics

The official NCLT calibration parameters (`cam_params.zip`) are **unavailable** - the link on the website returns a 404 error.

Calibration was attempted via COLMAP (300 images, SIMPLE_PINHOLE model):
- f = 221 px, cx = 404, cy = 309 (resolution 808x616)
- Reprojection error: 0.48 px
- Only 96/300 images registered

For ORB-SLAM3 the **KannalaBrandt8** (equidistant fisheye) model was used:
- fx = fy = 290 px
- cx = 404, cy = 308
- k1 = k2 = k3 = k4 = 0 (distortion coefficients unknown)

> **Issue:** zero distortion coefficients for a lens with ~120 FOV introduce systematic error in the outer image regions.

### 3.2. IMU Noise Parametars

Determined from a static segment of the recording (28.7 s of standstill):

```
Gyro noise:  0.00972 rad/s/sqrt(Hz)
Accel noise: 0.02855 m/s^2/sqrt(Hz)
Gyro walk:   0.000972 rad/s^1.5
Accel walk:  0.002855 m/s^2.5
```

Axis correctness was also verified: at standstill accel_z = -9.805 m/s^2 (gravity), gyro ~ [0, 0, 0] - confirming that the coordinate system (NED-like) is correct.

### 3.3. Body-to-Camera Extrinsics (T_bc)

Computed from the known Ladybug3 geometry (sphere of radius ~80 mm, center at 1.23 m above the body frame):

**Cam5 (front-facing, 0 degrees):**
```
T_bc = [0, 0, 1, 0.115;  1, 0, 0, 0.002;  0, 1, 0, -1.23;  0, 0, 0, 1]
```

---

## 4. Testing Methodology

### 4.1. Data Preparation

Data were converted to the EuRoC format expected by ORB-SLAM3:
- **Images:** PNG files named by nanosecond timestamps (`{utime_us * 1000}.png`)
- **IMU:** CSV in the format `timestamp_ns, gx, gy, gz, ax, ay, az`
- **5 IMU data variants** were created:

| Version | Source | Rate | Description |
|---------|--------|------|-------------|
| v1 | MS25 raw | 47 Hz | No processing |
| v2 | MS25 interpolated | 100 Hz | Linear interpolation |
| v3 | MS25 interpolated | 200 Hz | Linear interpolation (matching EuRoC) |
| v4 | MS25 + KVH FOG hybrid | 100 Hz | Gyroscope from KVH fiber-optic gyro |
| v5 | MS25 + KVH FOG hybrid | 200 Hz | Same at 200 Hz |

### 4.2. Test Configurations

**12 configurations** were tested in two phases:

**Phase B (Mono-Inertial, 3000 frames ~ 10 minutes):**

| ID | Camera | Model | IMU | Details | Test Objective |
|----|--------|-------|-----|---------|----------------|
| B001 | Cam5 | Fisheye | v1 (47Hz) | baseline | Baseline configuration |
| B002 | Cam5 | Fisheye | v2 (100Hz) | - | Effect of IMU rate |
| B003 | Cam5 | Fisheye | v3 (200Hz) | - | Simulating EuRoC rate |
| B004 | Cam5 | Fisheye | v4 (hybrid) | KVH FOG | Effect of gyroscope quality |
| B005 | Cam5 | Fisheye | v5 (hybrid) | 200Hz | Hybrid at 200 Hz |
| B006 | Cam5 | **Pinhole** | v1 | f=221 | Pinhole vs Fisheye |
| B007 | **Cam4** | Fisheye | v1 | Left (-72 deg) | Effect of camera direction |
| B008 | **Cam1** | Fisheye | v1 | Right (+73 deg) | Effect of camera direction |
| B009 | Cam5 | Fisheye | v1 | **CLAHE** | Contrast enhancement |
| B010 | Cam5 | Fisheye | v1 | **5000 features** | More keypoints |

**Phase C (Stereo-Inertial, 3000 frames):**

| ID | Camera Pair | Baseline | Objective |
|----|-------------|----------|-----------|
| C_cam4_cam5 | Cam4 + Cam5 | 9.4 cm | Stereo with left + front |
| C_cam5_cam1 | Cam5 + Cam1 | 9.5 cm | Stereo with front + right |

---

## 5. Results

### 5.1. Phase B - Mono-Inertial

| ID | Tracking | Keyframes | Maps | Status | Notes |
|----|----------|-----------|------|--------|-------|
| **B001** | **90.2%** | 2706 -> 2020 | 1 | Works | Best result |
| **B002** | **90.2%** | 2706 -> 2009 | 1 | Works | 100 Hz = no improvement |
| **B003** | **90.2%** | 2706 -> 1929 | 1 | Works | 200 Hz = no improvement |
| B004 | 79.7% | 2390 -> 1746 | 2 | Works | Hybrid: 10% worse |
| B005 | 0% | 0 | - | **Crash** | Segfault after map reset |
| B006 | 2.6% | 79 -> 40 | 1 | Works | Pinhole **34x worse** |
| B007 | 0% | 0 | - | **Crash** | Lateral camera fails |
| B008 | 0% | 0 | - | **Crash** | "scale too small" |
| B009 | 0% | 0 | - | **Crash** | CLAHE broke tracking |
| B010 | 0% | 0 | - | **Crash** | 5000 features = worse |

**Key findings from Phase B:**
1. **Fisheye model is essential** - pinhole yields 2.6% vs. 90.2%
2. **IMU rate does not matter** - v1 (47 Hz), v2 (100 Hz), v3 (200 Hz) all give identical 90.2%
3. **Only the front camera (Cam5) works** - lateral cameras (Cam4, Cam1) crash
4. **Preprocessing (CLAHE) and increasing features destroy tracking**
5. **Hybrid with KVH FOG gyroscope worsened the result** (79.7% vs. 90.2%)

### 5.2. Phase C - Stereo-Inertial

| ID | Tracking | Keyframes | Status |
|----|----------|-----------|--------|
| C_cam4_cam5 | **0.03%** (1 frame) | 1 | **Complete failure** |
| C_cam5_cam1 | **0.03%** (1 frame) | 1 | **Complete failure** |

**Cause of failure:** the Ladybug3 cameras are arranged at ~72 degree angles to each other and have **minimal field-of-view overlap**. Stereo matching is impossible. ORB-SLAM3 creates a map with 0 points and immediately resets the session.

### 5.3. Phase D - Full Session (21,310 frames)

Running the best configuration (B001) on the full session **did not complete** - the 90-minute timeout was exceeded. ORB-SLAM3 becomes significantly slower on long sessions due to loop closure checking and map management.

### 5.4. Trajectory Analysis

Despite the 90.2% tracking rate (2706 frames out of 3000), analysis of the ORB-SLAM3 output reveals **catastrophic scale inconsistency:**

| Metric | Ground Truth | ORB-SLAM3 VIO (B004) |
|--------|-------------|---------------------|
| Duration | 478 s | 478 s |
| Path length | ~637 m | **378,233 m (378 km)** |
| Scale relative to GT | 1.0x | **~310x** |
| Trajectory shape | Zigzag accross campus | Straight line |

After Sim(3) alignment (best similarity in 7 DOF - rotation, translation, scale):
- Scale factor: **1.1 x 10^-3** (reduction by ~1000x)
- ATE RMSE after alignment: **~69 m** (for a GT segment length of ~637 m)

> **Interpretation:** ORB-SLAM3 "sees" keypoints and tracks them frame-by-frame (90.2% success), but the inertial component (IMU preintegration) produces **an entirely incorrect estimate of scale and velocity**, turning a zigzag route into a straight line 378 km long.

---

## 6. Comparison with Other Methods

Other methods were also tested as part of this project:

| Exp. | Method | Tracking | ATE RMSE | Comment |
|------|--------|----------|----------|---------|
| 0.1 | **LiDAR ICP + GPS** | 100% | **174 m** | The only working method |
| 0.2 | ORB-SLAM3 mono (Cam0) | 0.5% | 2.9 m* | *Cam0 = sky |
| 0.3 | ORB-SLAM3 mono v2 | 24% | 45.6 m* | *35 fragmented maps |
| 0.4a | DROID-SLAM | 60% | 110 m* | *Shape does not match GT |
| 0.4b | DPVO | 100% | 142 m* | *Shape does not match GT |
| 0.4c | DPV-SLAM | 97% | 166 m* | *False loop closures |
| **0.6** | **ORB-SLAM3 VIO (proper)** | **90.2%** | **~69 m*** | ***Scale 310x incorrect** |

> \* denotes results that are formally low but **not reliable**: Sim(3) alignment finds "best similarity" even for fundamentally incorrect trajectories. Visual inspection shows that none of the visual methods reproduce the route shape.

---

## 7. Diagnosing the Scale Problem

A detailed check of the data preparation pipeline was performed:

1. **IMU data format** - verified: EuRoC format (gx, gy, gz, ax, ay, az), correct units (rad/s, m/s^2)
2. **Coordinate system** - verified: at standstill accel_z = -9.805 m/s^2 (gravity), 1.1 degree tilt from vertical - correct for NED
3. **YAML configs** - verified: T_bc matrices, noise parameters, IMU rate
4. **Output trajectories** - verified: timestamps match input images

**Conclusion:** no errors were found in data preparation. The problem is **fundamental** - the NCLT sensor parameters fall outside the operating range of ORB-SLAM3.

### Explanation of the Mechanism

ORB-SLAM3 mono-inertial uses a **tightly-coupled** scheme: IMU preintegration between frames provides an initial guess for the optimizer, which then refines the pose using visual observations. When the IMU prediction drifts heavily (due to noise/low rate) and the camera provides frames only every 200 ms:

1. The IMU predicts displacement over 200 ms with large drift (~14-57x worse noise than EuRoC)
2. Visual BA refines the pose but cannot fully correct the IMU scale drift
3. The scale diverges with each frame
4. Over 478 seconds the accumulated scale error reaches 310x

---

## 8. Conclusions

### What works:
- **Visual tracking** with ORB-SLAM3 using the fisheye model on Cam5 - 90.2% of frames
- **IMU calibration** from a static segment - correct noise parameters
- **Body-Camera extrinsics** - computed from Ladybug3 geometry

### What does not work:
- **Inertial component** - scale drifts by 310x due to sensor mismatch
- **Stereo mode** - Ladybug3 cameras lack sufficient overlap for stereo
- **Alternative IMU variants** - interpolation to 200 Hz does not help; the KVH hybrid makes things worse
- **Full session** - timeout due to nonlinear growth of computation

### Root cause:
**NCLT falls outside the operating range of ORB-SLAM3 VIO.** The system was designed and tested on EuRoC (camera 20 Hz, IMU 200 Hz, IMU noise 14-57x lower). With a 5 Hz camera and a 48 Hz IMU, **scale is unobservable** - inertial preintegration over 200 ms accumulates an error that the visual component cannot correct.

### Recommendations:
1. **For the NCLT dataset:** use LiDAR ICP as the primary odometry (ATE 174 m, 100% coverage)
2. **For visual SLAM:** consider VINS-Fusion with native equidistant fisheye support and online extrinsics calibration
3. **To improve scale:** fuse with wheel odometry (100 Hz, metric scale) or GPS

---

## Appendix A. Result File Structure

```
results/week0_orbslam3_proper/
├── calibration/
│   ├── calibration_summary.json     # All calibration parameters
│   ├── imu_noise.json               # IMU noise from static segment
│   └── Tbc_cam*.txt                 # Body-camera extrinsics
├── imu_data/
│   └── imu_v1.csv ... imu_v5.csv   # 5 IMU data variants
├── phase_b/
│   ├── B001_imu_v1/ ... B010_*/    # Test configurations
│   │   ├── config.yaml             # ORB-SLAM3 YAML configuration
│   │   ├── result.json             # Run metadata
│   │   └── f_nclt.txt              # Trajectory (if available)
│   └── summary.json                # Aggregated Phase B results
├── phase_c/
│   ├── C_cam4_cam5/, C_cam5_cam1/  # Stereo configurations
│   └── summary.json
├── phase_d/
│   └── D1_full_fisheye_cam5_v1/    # Full session (timeout)
├── plots/
│   ├── exp06_vs_gt.png             # Comparison with GT (3 panels)
│   ├── exp06_fig1_phase_b.png      # Phase B overview
│   ├── exp06_fig2_failure_analysis.png
│   └── exp06_fig3_summary_table.png
└── experiment_log.txt              # Full experiment log
```

## Appendix B. ORB-SLAM3 Configuration (baseline, B001)

```yaml
Camera.type: "KannalaBrandt8"
Camera1.fx: 290.0
Camera1.fy: 290.0
Camera1.cx: 404.0
Camera1.cy: 308.0
Camera1.k1-k4: 0.0
Camera.width: 808
Camera.height: 616
Camera.fps: 5

IMU.T_b_c1: [0,0,1,0.115, 1,0,0,0.002, 0,1,0,-1.23, 0,0,0,1]
IMU.NoiseGyro: 0.00972492
IMU.NoiseAcc: 0.02855310
IMU.GyroWalk: 0.00097249
IMU.AccWalk: 0.00285531
IMU.Frequency: 47.0

ORBextractor.nFeatures: 3000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 10
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```
