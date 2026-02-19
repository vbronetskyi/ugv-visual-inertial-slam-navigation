# ORB-SLAM3 Benchmark on ROVER Dataset: Analysis and Results

## 1. Platform: ROVER UGV

**ROVER** (Robust Outdoor Visual Evaluation Robot) is a wheeled unmanned ground vehicle (UGV) developed at Esslingen University of Applied Sciences for research in visual navigation under varying environmental conditions.

### Sensor Suite

| Sensor | Type | Resolution | Frequency | Purpose |
|--------|------|------------|-----------|---------|
| **Intel RealSense T265** | Stereo IR + IMU | 848x800 x2 (fisheye) | 30 fps / IMU 264 Hz | Visual odometry |
| **Intel RealSense D435i** | RGB-D + IMU | 640x480 (RGB + Depth) | 30 fps / IMU 30 Hz | Depth imaging |
| **Leica Total Station** | Laser tracker | Sub-cm accuracy | 1.6--3.9 Hz | Ground truth |

**Key platform characteristics:**
- Differential drive (two driven wheels)
- Average travel speed: ~0.35 m/s
- Reflective prism for Leica Total Station mounted on the roof
- Full dataset size: ~576 GB

### Sensors in Detail

**Intel RealSense T265** is a stereo camera with two infrared fisheye lenses (KannalaBrandt8 model, FoV ~170 degrees) and a built-in IMU. The baseline between cameras is only **6.35 cm**, which is critically short for outdoor environments. The IR cameras have their own emitters and can operate in low-light conditions, but stereo matching degrades at distances beyond 5 m due to negligible disparity (1--2 pixels).

**Intel RealSense D435i** is an RGB-D camera with structured IR illumination for depth measurement. Effective range is 0.1--10 m. The depth sensor operates in darkness (it is IR-based), but the RGB camera is a standard color sensor that cannot see in complete darkness.

**Leica Total Station** is a geodetic instrument that tracks the prism on the robot with sub-centimeter accuracy. The measurement frequency is non-uniform (1.6--3.9 Hz) and depends on the line of sight.

---

## 2. Dataset: ROVER

### Routes

The dataset contains **23 recordings** across **3 locations** with varying lighting conditions and seasons:

| Location | Description | Area Size | Recordings | Frames per Recording | Trajectory Length |
|----------|-------------|-----------|------------|---------------------|-------------------|
| **Garden Large (GL)** | University garden, walls, bushes, curbs | ~13x20 m | 8 | 12K--14K | 150--170 m |
| **Park (P)** | Open park, trees, pathways | ~20x19 m | 7 | 13K--15K | 164--183 m |
| **Campus Large (CL)** | University campus, large loops | ~120x80 m | 8 | 21K | 250--350 m |

### Recording Conditions

Each location was recorded under 7--8 different conditions:
- **day** - bright daylight, best conditions
- **spring / summer / autumn / winter** - seasonal variations (different vegetation, snow cover)
- **dusk** - twilight, transitional lighting
- **night-light** - nighttime with artificial lighting (streetlamps)
- **night** - complete darkness (no artificial illumination)

### Data Format

Each recording has the following structure:
```
{recording}/
├── groundtruth.txt          # TUM: timestamp x y z qx qy qz qw
├── realsense_T265/
│   ├── cam_left/            # IR fisheye images (848x800, grayscale)
│   ├── cam_right/           # IR fisheye images
│   └── imu/imu.txt          # 264 Hz: timestamp, ax, ay, az, gx, gy, gz
└── realsense_D435i/
    ├── rgb/                 # Color images (640x480)
    ├── depth/               # Depth maps (640x480, uint16, mm)
    └── imu/imu.txt          # 30 Hz IMU
```

---

## 3. Three ORB-SLAM3 Modes (Heatmap Columns)

### Stereo PinHole (Stereo PH)

**Sensor:** T265 stereo IR cameras (undistorted fisheye to pinhole 640x480)

**Principle:** Classical stereo visual odometry. ORB-SLAM3 extracts ORB features from both cameras, finds stereo correspondences, triangulates 3D points, and tracks camera motion. Scale is determined from the known baseline (6.35 cm).

**Preprocessing:** T265 fisheye images are rectified (undistorted) to a pinhole model with 110-degree horizontal FoV (fx=fy=224.07, 640x480) using OpenCV fisheye remap.

**Parameters:** 2000 ORB features, iniThFAST=15, minThFAST=5

### Stereo-Inertial PinHole (SI PH)

**Sensor:** T265 stereo IR cameras + T265 IMU (264 Hz)

**Principle:** Stereo PH combined with inertial navigation. The IMU provides rotation and acceleration estimates between frames, enabling camera motion prediction. In theory, this should yield more accurate scale and better robustness to motion blur. In practice, the IMU is also used for loop closure verification.

**Critical issue on ROVER:** The T265 IMU has a gravity estimation error of ~3.4 degrees, leading to a loop closure verification threshold of 0.008 rad - most loop closures are rejected as "BAD LOOP". Without loop closures, drift goes uncorrected.

**Parameters:** Same ORB parameters + full IMU calibration (noise, bias walks, Tbc)

### RGB-D

**Sensor:** D435i color camera + depth sensor

**Principle:** ORB features are extracted from the RGB image, and depth is read directly from the depth map for each point. This eliminates the need for stereo matching - each frame provides full 3D information. Scale is determined by the depth sensor, which is calibrated in millimeters.

**Parameters:** 1500 ORB features, iniThFAST=20, minThFAST=7 (more conservative than Stereo PH)

---

## 4. Comparison Results

### Summary Table

| Mode | Success | Median ATE | Mean ATE | Median Scale | Comment |
|------|---------|-----------|----------|-------------|---------|
| **RGB-D** | **22/23 (96%)** | **0.53 m** | 1.64 m | **0.988** | Best: accurate scale, stable |
| **Stereo PH** | 21/23 (91%) | 0.88 m | 1.91 m | 1.228 | Second: works, but scale ~+22% |
| **SI PH** | 12/23 (52%) | 2.08 m | 2.92 m | 1.186 | Worst: IMU degrades performance |

### Why RGB-D Performs Best

1. **Direct depth measurement:** The D435i depth sensor provides accurate metric depth for every pixel. Scale = 0.988 (near-ideal, error <2%).
2. **No reliance on stereo matching:** Unlike Stereo PH, it does not require finding correspondences between two images. Each frame is self-sufficient.
3. **Robustness:** Even under challenging conditions (twilight, artificial lighting), the depth sensor operates reliably.

### Why Stereo PH Ranks Second

1. **Systematic scale error of ~22%:** The T265 baseline (6.35 cm) is too short for outdoor environments. At distances of 5--10 m, stereo disparity is only 1--2 pixels, leading to systematic scale overestimation (scale approximately 1.22).
2. **Requires rectification:** Without fisheye-to-pinhole undistortion, ORB features cannot be matched (the original fisheye KB8 model yielded 0/23 successes).
3. **Loop closures work:** Unlike SI, loop closures in Stereo PH are not blocked by the IMU, so drift is corrected.

### Why Stereo-Inertial Performs Worst

1. **IMU degrades performance:** The T265 IMU has a gravity vector estimation error of ~3.4 degrees. This causes ORB-SLAM3 to reject all loop closures (BAD_LOOP threshold = 0.008 rad), leaving drift uncorrected.
2. **Scale collapse:** 5 out of 23 recordings exhibit complete scale collapse (scale < 0.1), meaning the trajectory shrinks to a point. This occurs when the IMU bias diverges.
3. **No scale improvement:** The Stereo-Inertial scale (1.186) is practically identical to Stereo PH (1.228) - the IMU adds no useful scale information.
4. **Conclusion:** The T265 IMU acts as an "actively harmful component" -- it does not improve accuracy but blocks loop closures. In 9 out of 11 direct comparisons, SI performs worse than pure Stereo PH.

---

## 5. Heatmap Analysis by Condition

### Effect of Lighting

| Conditions | Stereo PH | RGB-D | Comment |
|------------|-----------|-------|---------|
| **Day/Spring/Summer** | 0.37--0.58 m | 0.37--0.54 m | Both perform well |
| **Autumn/Winter** | 0.39--0.71 m | 0.37--0.48 m | Seasonal changes have minimal impact |
| **Dusk (twilight)** | 0.84--1.51 m | 1.63--4.60 m | Stereo PH more stable (IR cameras) |
| **Night-light** | 1.39--7.63 m | 0.45--0.53 m | RGB-D better (depth sensor works) |
| **Night (darkness)** | 0.88 m | X (crash) | T265 IR sees, D435i RGB does not |

**Key observation:** RGB-D and Stereo PH have **opposite weaknesses** under nighttime conditions:
- **RGB-D** works under night-light (depth sensor OK) but **fails in complete night** (the RGB camera cannot see)
- **Stereo PH** works at night (IR cameras with emitters) but **degrades under night-light** (artificial lighting creates high-contrast shadows that disrupt stereo matching, leading to scale collapse)

### Effect of Route

| Route | Stereo PH median | RGB-D median | Comment |
|-------|-----------------|-------------|---------|
| **Garden Large** | 0.55 m | 0.42 m | Best: compact area, rich textures |
| **Park** | 0.54 m | 0.48 m | Good: larger area, fewer textures |
| **Campus Large** | 1.63 m | 0.66 m | Worst: 21K frames, long loops, greater drift |

Campus Large presents the most challenges due to:
1. **Long sequences (21K frames):** More time for drift accumulation
2. **Large loops (250--350 m):** Loop closures must correct greater drift
3. **Crashes:** CL/day and CL/dusk crash with `vector::reserve` during Global Bundle Adjustment - the large pose graph causes memory allocation failure during loop closure optimization

### Specific Problematic Recordings

| Recording | Stereo PH | RGB-D | Cause |
|-----------|-----------|-------|-------|
| **CL/day** | X (crash) | 0.66 m | Crash during loop closure BA on 21K frames (memory allocation failure) |
| **CL/dusk** | X (crash) | 4.60 m | Same issue + RGB-D drifts due to twilight |
| **GL/night-light** | 6.36 m (s=0.58) | 0.48 m | Stereo PH: artificial lighting disrupts stereo matching, scale collapse |
| **P/night-light** | 7.63 m (s=1.03) | 0.45 m | Stereo PH: same issue; low-light config helped with scale but ATE remains high |
| **GL/night** | 0.88 m | X (crash) | RGB-D: D435i RGB camera cannot see in complete darkness. T265 IR can. |
| **CL/night** | 7.79 m (s=2.83) | 9.83 m | Both poor: CL/night has brightness of only 24/255. Tracking loss for both. |

### Effect of the Sophus NaN Patch

During the experiments, a bug was discovered in ORB-SLAM3: the `SO3::expAndTheta()` function in the Sophus library crashes (SIGABRT) on NaN quaternion values that arise from degenerate geometry. The patch returns an identity quaternion instead of crashing.

**Patch results:**
- Stereo PH: 15/23 --> 21/23 (6 recordings recovered)
- RGB-D: remained at 22/23 after an additional low-light retry

---

## 6. Conclusions

1. **RGB-D is the best mode for ROVER:** Median ATE = 0.53 m with near-ideal scale (0.988). The D435i depth sensor eliminates the stereo matching problem caused by the short baseline.

2. **Stereo PH is a viable fallback:** Median ATE = 0.88 m with a systematic scale error of +22%. Requires fisheye-to-pinhole rectification.

3. **Stereo-Inertial is not recommended for ROVER:** The T265 IMU blocks loop closures and does not improve scale. It performs worse than pure Stereo PH in the majority of cases.

4. **Lighting matters more than season:** The difference between day and night is orders of magnitude (0.4 m vs 7+ m or crash). The difference between spring and winter is minimal (<0.1 m).

5. **RGB-D and Stereo PH are complementary:** RGB-D fails in complete darkness (RGB camera limitation), while Stereo PH fails under artificial lighting (scale collapse). An ideal system should combine a depth sensor with IR cameras.

6. **Campus Large serves as a stress test:** 21K frames with long loops expose bugs in ORB-SLAM3 (vector::reserve crash, Sophus NaN) that do not manifest on shorter routes.

7. **The T265 baseline (6.35 cm) is the limiting factor:** A systematic scale error of ~22% is present across all Stereo PH/SI results. Outdoor environments require a baseline of at least 10--15 cm.
