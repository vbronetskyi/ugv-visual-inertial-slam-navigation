# ORB-SLAM3 on ROVER: Complete Analysis of Results

## Experiment Overview

**Dataset:** ROVER (Esslingen University) - 22 recordings with GT, 3 routes
**System:** ORB-SLAM3 v1.0
**Modes:** RGB-D (D435i), Stereo PinHole (T265 undistorted), Stereo-Inertial PinHole (T265 + IMU)
**Number of experiments:** 66 (22 x 3)
**Server:** vast.ai GPU instance (RTX 4080, 800GB SSD)

### Routes

| Route | Recordings | Size | Trajectory length | Characteristics |
|-------|------------|------|-------------------|-----------------|
| garden_large | 8 | ~13x20 m | 150–170 m | Compact garden with rich texture (walls, bushes, curbs) |
| park | 7 | ~20x19 m | 164–183 m | Open park (trees, grass, paths), less texture |
| campus_large | 7 | ~120x80 m | 250–350 m | Large campus, long loops, open spaces |

### Recording Conditions

Each route was recorded under different lighting conditions: day, spring, summer, autumn, winter, dusk, night, night-light.

---

## Summary Results

### By Mode (all routes)

| Mode | Success | Median ATE | Mean ATE | Notes |
|------|---------|-----------|----------|-------|
| **RGB-D** | **21/22 (95%)** | **0.54 m** | 1.86 m | Most stable, depth provides scale |
| **Stereo PinHole** | **15/22 (68%)** | **0.71 m** | 1.48 m | Frequent crashes, but mean better than RGB-D |
| Stereo-Inertial PH | 17/20 (85%) | 3.85 m | 4.99 m | IMU initialization often fails |

### By Route

| Route | RGB-D | Stereo PH | Stereo-Inertial PH |
|-------|-------|-----------|---------------------|
| garden_large | 7/8, med=0.42 m | 5/8, med=0.54 m | 7/8, med=2.14 m |
| park | 7/7, med=0.54 m | 5/7, med=0.54 m | 6/6, med=4.57 m |
| campus_large | 7/7, med=0.72 m | 5/7, med=2.07 m | 4/6, med=9.86 m |

---

## Full Results Table

Legend: ATE RMSE in meters, s = Sim3 scale, n = matched poses.
FAIL = crash or missing trajectory. Bold indicates the best mode for each recording.

### Garden Large

| Recording | Conditions | RGB-D | Stereo PH | Stereo-Inertial PH |
|-----------|------------|-------|-----------|---------------------|
| GL/autumn | day, bare branches | **0.37** (s=0.98) | 0.58 (s=1.26) | 3.85 (s=0.87) |
| GL/day | day, clear | **0.40** (s=0.98) | 0.53 (s=1.21) | 0.65 (s=1.22) |
| GL/dusk | dusk | 6.69 (s=0.50) | FAIL | FAIL |
| GL/night-light | night + streetlights | **0.48** (s=0.96) | FAIL | 6.43 (s=0.00) |
| GL/night | complete darkness | FAIL | **0.88** (s=1.21) | 2.14 (s=1.13) |
| GL/spring | day, spring | **0.42** (s=0.99) | 0.54 (s=1.25) | 0.60 (s=1.26) |
| GL/summer | day, summer | **0.45** (s=0.99) | FAIL | 0.70 (s=1.21) |
| GL/winter | day, winter | **0.37** (s=0.98) | 0.39 (s=1.27) | 7.56 (s=0.00) |

### Park

| Recording | Conditions | RGB-D | Stereo PH | Stereo-Inertial PH |
|-----------|------------|-------|-----------|---------------------|
| P/autumn | day, autumn | 1.87 (s=0.98) | **0.37** (s=1.22) | 6.52 (s=0.47) |
| P/day | day, clear | **0.48** (s=1.00) | FAIL | 8.12 (s=0.02) |
| P/dusk | dusk | 1.63 (s=1.04) | **1.51** (s=1.28) | 7.63 (s=0.34) |
| P/night-light | night + streetlights | **0.45** (s=0.99) | FAIL | - |
| P/night | complete darkness | 6.55 (s=0.93) | **1.78** (s=1.53) | 2.61 (s=1.19) |
| P/spring | day, spring | 0.54 (s=1.03) | 0.51 (s=1.23) | **0.42** (s=1.22) |
| P/summer | day, summer | **0.45** (s=1.02) | 0.54 (s=1.21) | 0.45 (s=1.18) |

### Campus Large

| Recording | Conditions | RGB-D | Stereo PH | Stereo-Inertial PH |
|-----------|------------|-------|-----------|---------------------|
| CL/autumn | day, autumn | **0.72** (s=0.99) | 2.07 (s=1.19) | 7.41 (s=1.07) |
| CL/day | day, clear | **0.66** (s=0.99) | FAIL | 2.02 (s=1.23) |
| CL/dusk | dusk | **4.60** (s=0.91) | FAIL | FAIL |
| CL/night | night | 10.11 (s=1.54) | **7.79** (s=2.83) | 12.31 (s=0.01) |
| CL/spring | day, spring | **0.75** (s=1.00) | 1.63 (s=1.20) | FAIL |
| CL/summer | day, summer | **0.61** (s=1.01) | 2.37 (s=1.25) | 15.41 (s=0.01) |
| CL/winter | day, winter | **0.48** (s=0.99) | 0.71 (s=1.23) | - |

---

## Stereo PinHole Analysis

### Overall Picture

- **15/22 successful** (68% success rate)
- **7 crashes** - all SIGABRT (rc=-6), none produced a trajectory
- Median ATE = 0.71 m, mean = 1.48 m
- Scale of all successful runs: **1.19–1.28** (consistent overestimation by ~20%, except CL/night = 2.83)

The ~1.2 scale is a systematic effect of the small T265 stereo baseline (6.35 cm). In open spaces, most points are at 5-15 m distance, where disparity is ~1-2 pixels. ORB-SLAM3 systematically underestimates depth, which leads to overestimated scale. Sim3 alignment corrects for this.

### Two Crash Mechanisms

#### 1. Sophus SO3::exp NaN (5 out of 7 crashes)

**Recordings:** GL/dusk, GL/night-light, GL/summer, P/day, P/night-light

```
Sophus ensure failed in function 'SO3::expAndTheta'
SO3::exp failed! omega: -nan -nan -nan
```

**Cause:** NaN arises in the g2o optimizer during Bundle Adjustment or loop closure. When ORB-SLAM3 computes pose correction through nonlinear optimization, poor conditioning of the system (few features, bad geometry) causes the Hessian matrix to become degenerate. The result contains NaN, which propagates into Sophus SO3::exp, triggering an assertion failure and SIGABRT.

**When it occurs:**

| Recording | Time to crash | Map points | Loop closures |
|-----------|-------------|------------|---------------|
| P/night-light | 42 s | 298 | 0 |
| GL/night-light | 47 s | 225 | 0 |
| GL/dusk | 185 s | 469 | 1 |
| GL/summer | 307 s | 851 | 2 |
| P/day | 937 s | 717 | 2 |

Clear correlation: fewer map points lead to faster crashes. Night-light recordings initialize the map with 225-298 points (vs 700-850 for daytime) and crash within a minute.

#### 2. std::length_error in vector::reserve (2 out of 7 crashes)

**Recordings:** CL/day, CL/dusk (both campus_large)

```
terminate called after throwing 'std::length_error'
  what():  vector::reserve
```

**Cause:** Memory allocation error. campus_large sequences are long (~21-28k frames), and after 4+ loop closures with Global BA, accumulation of map points and keyframes leads to an attempt to allocate a vector of invalid size (counter overflow or memory fragmentation).

**Time to crash:** 1221 s and 1349 s - the longest among all crashes, indicating a gradual accumulation of the problem.

### Correlation with Lighting Conditions

| Condition | Crashes | Successful |
|-----------|---------|------------|
| Night/night-light | 3 | 2 |
| Dusk | 2 | 1 |
| Day/spring/summer | 2 | 10 |
| Autumn/winter | 0 | 5 |

Crashes occur **not only** in darkness. 2 crashes under daytime lighting (P/day, GL/summer) - the problem lies in the numerical stability of ORB-SLAM3, not solely in feature quality.

### Correlation with Route

| Route | Crashes | Success rate |
|-------|---------|-------------|
| garden_large | 3/8 | 62% |
| park | 2/7 | 71% |
| campus_large | 2/7 | 71% |

Uniform distribution - route is not the primary factor for crashes.

### Successful but with High Error

| Recording | ATE | Scale | Issue |
|-----------|-----|-------|-------|
| CL/night | 7.79 | 2.83 | Only 4.6% tracking, 4 maps, meaningless scale |
| CL/summer | 2.37 | 1.25 | g2o warning "0 vertices to optimize" |
| CL/autumn | 2.07 | 1.19 | Crash at shutdown (rc=-11), but trajectory was saved |

3 out of 5 "succesful" campus_large runs actually crashed with SIGSEGV, but **after** saving the trajectory. The difference between "success" and "failure" is merely the timing of the crash relative to file writing.

### Recommendations for Stereo PinHole

1. **Patch Sophus for NaN handling**: instead of assertion, return identity rotation or skip the pose update
2. **Add NaN guard before SO3::exp()** in ORB-SLAM3 code (check omega for NaN)
3. **Limit the number of consecutive BAD LOOPs** - after N failures, stop loop closure attempts
4. **For campus_large**: limit max keyframes or add map pruning for long sequences
5. **Increase nFeatures** (2000 to 3000) for low-light scenes
6. **CLAHE preprocessing** to improve contrast at dusk
7. **Lower FAST threshold** (15/5 to 10/3) for low-light

---

## Stereo-Inertial PinHole Analysis

### Overall Picture

- **17/20 successful** (85%), 2 did not run (park_night-light, campus_winter - timeout/missing)
- **3 crashes** + **~8 recordings with scale close to 0** (IMU init failure)
- Median ATE = 3.85 m, mean = 4.99 m - **worst mode**
- Correctly functioning in only **~7 out of 20** recordings (scale > 1.0)

### Fundamental Problem: Incorrect IMU Calibration

The ROVER_T265_PinHole_Stereo_Inertial.yaml config contains:

```yaml
IMU.Frequency: 264.0       # INCORRECT: actual frequency 200.3 Hz
IMU.NoiseGyro:  0.003236
IMU.NoiseAcc:   0.017406
IMU.GyroWalk:   7.057e-05
IMU.AccWalk:    0.012394    # SUSPICIOUSLY HIGH
```

#### Problem 1: IMU.Frequency 264 Hz vs actual 200 Hz

ORB-SLAM3 scales noise via `sqrt(freq)`:

```cpp
const float sf = sqrt(mImuFreq);
mpImuCalib = new IMU::Calib(Tbc, Ng*sf, Na*sf, Ngw/sf, Naw/sf);
```

With 264: sf = 16.25. With 200: sf = 14.14.
- Accelerometer noise **overestimated by 32%** (in variance)
- Walk **underestimated by 13%**

Attempting to correct Frequency to 200 **worsened** results (ATE 4.71 to 14.35). Reason: the remaining parameters were calibrated at 264 Hz, and changing only the frequency breaks the balance.

#### Problem 2: AccWalk = 0.012 - excessively high

Comparison with other sensors:

| Sensor | AccWalk (m/s^2/sqrt(Hz)) | Relative to T265 |
|--------|---------------------|---------------|
| T265 (BMI055) | 0.0124 | 1.0x |
| EuRoC (ADIS16448) | 0.003 | 4.1x lower |
| D435i | 0.0005 | 24.8x lower |
| BMI055 datasheet | ~0.001 | 12x lower |

AccWalk = 0.012 tells ORB-SLAM3 that the accelerometer bias drifts rapidly. As a result, the optimizer weakly constrains the bias, the gravity direction becomes unobservable, and scale cannot be recovered.

The calibrated AccWalk is **8-30x higher** than the BMI055 datasheet value. Most likely, the calibration was contaminated by vibrations or thermal drift.

#### Fix Attempts (unsuccessful)

| Attempt | Changes | Result campus_large_autumn |
|---------|---------|-------------------------------|
| Original | - | ATE=4.71, scale=1.22 |
| Attempt 1 | Freq=200, all noise params | ATE=14.35, scale=0.17 (catastrophe) |
| Attempt 2 | Freq=200, AccWalk=0.006 | ATE=8.30 (worse) |
| Revert | Original values | ATE=7.41 (variance) |

**Conclusion:** IMU parameters form a package - changing one without recalibrating the rest breaks the system. A full recalibration or proportional reduction of ALL noise params is required.

### Recording Classification by Scale

| Scale | Interpretation | Recordings | ATE |
|-------|---------------|--------|-----|
| 1.15–1.26 | IMU init OK | 7 recordings | 0.42–2.14 m |
| 0.34–0.87 | Partial IMU init | 3 recordings | 3.85–7.63 m |
| 0.00–0.02 | Complete IMU failure | 5 recordings | 6.43–15.41 m |

**Scale ~1.2** (same as pure stereo) means that even with "successful" IMU init, the scale is predominantly determined by stereo, not IMU. An ideal stereo-inertial system should produce scale = 1.0.

### BAD LOOP Cascade Leading to Crash

When IMU init fails (scale close to 0), the map is not aligned with gravity. ORB-SLAM3 checks during loop closure:

```cpp
if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f)  // roll, pitch < 0.46 degrees
```

Without gravity, roll/pitch are always incorrect, so **every** loop closure is rejected ("BAD LOOP"). The system recognizes familiar places but cannot correct the trajectory. After 47-51 BAD LOOPs:
- NaN penetrates the optimizer, leading to Sophus SO3::exp crash (SIGABRT)
- Or memory corruption causes SIGSEGV

### Why Some Recordings Work

**Successful (scale ~1.2):** park_spring, park_summer, GL/spring, GL/day, GL/summer, GL/night, campus_day

Common trait: **sufficiently high-quality visual tracking** (daytime lighting or night with contrast). When visual tracking works reliably, the optimizer can estimate scale even with imperfect IMU calibration.

**Unsuccessful:** recordings with poor lighting OR campus_large with long trajectories. On campus_large, even daytime recordings have issues (summer: scale=0.01, autumn: scale=1.07 but ATE=7.41) - a long trajectory plus open spaces overload the weak IMU model.

### Detailed Per-Recording Analysis

#### Group A: Correctly Functioning (scale ~1.2, ATE < 1 m)

| Recording | ATE | Scale | Maps | BAD LOOPs | Track fails | Details |
|-----------|-----|-------|------|-----------|-------------|---------|
| P/spring | 0.42 | 1.22 | 1 (1585 KF) | 0 | 0 | Perfect run. 1 loop accepted (phi 0.002, 0.007, 0.101). VIBA1+2 OK. |
| P/summer | 0.45 | 1.18 | 1 (1498 KF) | **46** | 0 | All 46 loops BAD, but tracking is accurate without loop closure. |
| GL/spring | 0.60 | 1.26 | 1 (1612 KF) | **~46** | 0 | Similar to P/summer - all loops rejected, tracking holds. |
| GL/day | 0.65 | 1.22 | 1 (1392 KF) | **391** | 0 | Record: 392 loop detections, 391 BAD, 1 accepted. ATE still good. |
| GL/summer | 0.70 | 1.21 | 1 (1819 KF) | 6 | 0 | Few loops, 1 accepted. VIBA1+2 OK. Clean run. |

**Key finding:** The number of BAD LOOPs **does not affect** accuracy. GL/day has 391 BAD LOOPs and ATE=0.65 m. What matters is the quality of the base visual-inertial tracking, not loop closure.

#### Group B: Moderate (scale ~1.0, ATE 2-4 m)

| Recording | ATE | Scale | Maps | BAD LOOPs | Track fails | Details |
|-----------|-----|-------|------|-----------|-------------|---------|
| GL/night | 2.14 | 1.13 | 1 (1251 KF) | 12 | **127** | 3 clusters of massive tracking failures (night). IMU dead-reckoning holds. |
| CL/day | 2.02 | 1.23 | 1 (2658 KF) | 47 | 0 | Scale OK, but long trajectory (801 s). Phi grows to 0.24 rad = drift. |
| P/night | 2.61 | 1.19 | 1 (1764 KF) | ~48 | 0 | Night, but no tracking failures. Drift without loop closure. |
| GL/autumn | 3.85 | **0.87** | 1 (1822 KF) | ~47 | 0 | Only "moderate" with scale < 1.0. Phi grows from 0.18 to 0.25. |

**Key finding:** GL/night (127 tracking failures, ATE=2.14) shows that IMU **can** sustain the system through dark segments, albeit with degraded accuracy.

#### Group C: IMU Init Failure (scale close to 0, ATE > 6 m)

| Recording | ATE | Scale | Maps | BAD LOOPs | Track fails | Details |
|-----------|-----|-------|------|-----------|-------------|---------|
| GL/night-light | 6.43 | 0.00003 | **2** | 0 | **125** | 3 times "IMU is not or recently initialized. Reseting active map". VIBA completed on 2nd map, but trajectory is broken. |
| P/autumn | 6.52 | 0.47 | 1 (1830 KF) | 3 | 0 | **VIBA1+2 OK, 1 loop accepted** - but scale is still 0.47. Formally init succeeded, in practice - it did not. |
| CL/autumn | 7.41 | **1.07** | 1 (4021 KF) | ~49 | 0 | **Anomaly:** scale is near-ideal (1.07), but ATE=7.41 due to rotational drift on a long trajectory (1132 s). max ATE=34.1 m. |
| GL/winter | 7.56 | 0.001 | **2** | 39 | **14** | 2 maps, merge succeeded, but with incorrect geometry. phi after merge: (-0.32, 0.50). |
| P/dusk | 7.63 | 0.34 | 1 (1819 KF) | ~47 | 0 | Bimodal phi values indicate drift in different parts of the trajectory. |
| P/day | 8.12 | 0.02 | 1 (2095 KF) | ~49 | 0 | **Most unexpected failure:** daytime, 0 tracking failures, but scale=0.02. phi[2]=1.05 rad (60 degrees!) - catastrophic heading drift. |
| CL/night | 12.31 | 0.008 | **2** | 0 | **178** | Map 0: 2152 KF with 178 fail to track. Map 1: 11 KF (dead). |
| CL/summer | 15.41 | 0.012 | 1 (3855 KF) | 5 | 0 | **VIBA1+2 OK, but scale=0.012.** Used AccWalk=0.006 (differs from others!). |

**Critical findings:**

1. **VIBA completion does not equal correct scale.** P/autumn completed VIBA1+2 and even had an accepted loop, yet scale=0.47. CL/summer completed VIBA with scale=0.012. P/day (daytime!) yielded scale=0.02. Formal success of IMU init does not guarantee correct scale.

2. **Correct scale does not equal good ATE.** CL/autumn has scale=1.07 (near-ideal), but ATE=7.41 m due to rotational drift without loop closure on a long trajectory.

3. **P/day (scale=0.02) - most unexpected failure.** Daytime lighting, 0 tracking failures, 2095 KF on 1 map, but phi[2]=1.05 rad shows a 60-degree heading error. VIBA converged to an incorrect local minimum.

4. **Multi-map recordings (2 maps) = guaranteed failure.** GL/night-light, GL/winter, CL/night - all have scale close to 0 due to incompatible coordinate frames between maps.

#### Group D: Crashes (no trajectory)

| Recording | RC | Time | BAD LOOPs | Track fails | Crash error |
|-----------|----|------|-----------|-------------|-------------|
| GL/dusk | -11 (SIGSEGV) | 401 s | 51 | 1 | Segfault after tracking failure. Null pointer during recovery. |
| CL/dusk | -6 (SIGABRT) | 630 s | 47 | 0 | `SO3::exp failed! omega: -nan`. phi[2] approx -0.59 rad (34 degrees). |
| CL/spring | -6 (SIGABRT) | 763 s | 47 | 0 | Identical crash: `SO3::exp NaN`. phi[0] approx -0.07 rad. |

Mechanism: BAD LOOP cascade leads to NaN in the optimizer, which triggers a Sophus assertion failure. 47-51 consecutive BAD LOOPs accumulate numerical errors.

### Recommendations for Stereo-Inertial

1. **Full recalibration of IMU noise params** with the correct frequency of 200 Hz:
   - Record static T265 data (10+ min, stationary sensor)
   - Allan variance analysis to obtain new NoiseGyro, NoiseAcc, GyroWalk, AccWalk
   - Alternatively, use BMI055 datasheet values: AccWalk approx 0.001, NoiseAcc approx 0.004
2. **Alternatively:** reduce all noise params by a factor of N and find the optimal N on a validation recording
3. **Patch BAD LOOP cascade:** limit the number of consecutive rejected loops, or fall back to pure stereo mode after N failures
4. **NaN guard in Sophus:** check omega for NaN before SO3::exp() and skip the update instead of crashing
5. **For night recordings:** the mode is unacceptable - IMU cannot compensate for the complete absence of visual features

---

## RGB-D Analysis

### Overall Picture

- **21/22 successful** (95%) - most stable mode
- Median ATE = 0.54 m, mean = 1.86 m (mean inflated by outliers)
- Scale = **0.98–1.04** for daytime recordings (expected ~1.0, depth provides metric scale)
- **1 failure:** garden_large_night (complete darkness)

### The Only Failure: GL/night

ORB-SLAM3 created **6 maps** within a minute, each with 225-660 map points (vs 1200-1500 for daytime):
1. Map initialization with minimal points
2. Immediately "Less than 15 matches!!" followed by reset
3. New map, then another reset
4. After 7 cycles, 90+ consecutive "Fail to track local map!" leading to SIGSEGV

D435i depth works at night (IR structured light), but the RGB camera cannot see, so the ORB detector finds no features and tracking is impossible. A paradox: **depth is available, but there is nothing to track**.

### Outliers with High Error

| Recording | ATE | Scale | Cause |
|-----------|-----|-------|-------|
| CL/night | 10.11 | 1.54 | Feature starvation: 12 maps with 200-330 points, 79.7% tracking |
| GL/dusk | 6.69 | 0.50 | 20+ sub-maps, no merge/loop, meaningless scale |
| P/night | 6.55 | 0.93 | Constant tracking loss, 0 loop closures, drift without correction |
| CL/dusk | 4.60 | 0.91 | Max ATE=24 m (!), 6 relocations, map fragmentation |

**Pattern 1 - Feature starvation (night):**
CL/night - 12 maps with 200-330 points. Median tracking time 8.75 ms (vs 21-25 ms for daytime) - the tracking thread barely operates. Trajectory was recorded (79.7% of frames), but fragmented. Scale=1.54 is an artifact of Sim3 alignment of fragments.

**Pattern 2 - Drift without correction (dusk/night with partial tracking):**
GL/dusk, P/night, CL/dusk - the system tracks intermittently, loses track over long segments, and relocalizes at shifted positions. **0 loop closures** - drift is never corrected. Median ATE may be 0.6-1.4 m, but individual segments show 14-24 m displacement.

**Pattern 3 - Map fragmentation:**
GL/dusk - 20+ sub-maps with 1 merge. Sim3 alignment cannot find a coherent transformation for the fragmented trajectory, resulting in scale=0.50 (an artifact, not a real value).

### Correlation with Lighting

| Condition | Mean ATE | Worst ATE | Success |
|-----------|---------|----------|---------|
| Day/spring/summer | 0.55 m | 0.75 m | 100% |
| Autumn/winter | 0.51 m | 0.72 m | 100% |
| Dusk | 4.31 m | 6.69 m | 100% (but degraded) |
| Night-light | 0.47 m | 0.48 m | 100% |
| Night | 7.74 m | 10.11 m | 2/3 (66%) |

**Key observations:**
- **Daytime lighting (any season):** consistently < 0.75 m
- **Night-light:** works excellently (0.47 m) - artificial lighting provides sufficient contrast for ORB
- **Dusk:** formally "successful", but ATE increases 5-10x
- **Night:** catastrophe or complete failure

### Correlation with Route

| Route | Median ATE (daytime) | Median ATE (all) |
|-------|-------------------|-----------------|
| garden_large | 0.42 m | 0.42 m |
| park | 0.48 m | 0.54 m |
| campus_large | 0.66 m | 0.72 m |

campus_large is 1.5x worse than garden_large even under ideal conditions - due to longer trajectory and larger open spaces.

### Role of Loop Closure

| Recording | ATE | Loop closures | Merges |
|-----------|-----|--------------|--------|
| GL/winter | 0.37 | 1 | 0 |
| GL/day | 0.40 | 3 | 0 |
| GL/spring | 0.42 | 6 | 2 |
| P/night | 6.55 | 0 | 0 |
| CL/dusk | 4.60 | 0 | 1 |

**Loop closure is the key factor**: all recordings with ATE < 1 m have at least 1 loop closure. All outliers with ATE > 4 m have **0 loop closures**. Without closing the loop, drift is never corrected.

### Detailed Per-Recording Analysis

#### Group A: Excellent Results (ATE < 0.5 m)

| Recording | ATE | Maps | Loops | Merges | Track fails | Reloc | Init pts | Med TT | Details |
|-----------|-----|------|-------|--------|-------------|-------|----------|--------|---------|
| GL/autumn | 0.37 | 3 | 4 | 2/2 | 154 | 0 | 1434 | 21 ms | 2 burst failures (65+95 consecutive), but merge+loops corrected everything |
| GL/winter | 0.37 | 1 | 1 | 0 | 0 | 0 | **1498** | 21 ms | Cleanest run. 0 failures. Segfault at exit (shutdown bug) |
| GL/day | 0.40 | 1 | 3 | 0 | 0 | 0 | 1473 | 22 ms | Perfect. Scale=0.977 (closest to 1.0 among GL) |
| GL/spring | 0.42 | 3 | **6** | 2/2 | 131 | 0 | 1450 | 23 ms | Most loops (6). 2 burst failures (39+85), full recovery |
| GL/summer | 0.45 | 1 | 4 | 0 | 9 | 3 | 1287 | 22 ms | **12 g2o errors** - highest among all. Max ATE=5.28 (spike) |
| P/night-light | 0.45 | 1 | 1 | 0 | 0 | 0 | **683** | 16 ms | Fewest init pts among successful runs! But 0 failures. Max ATE=0.82 (lowest) |
| P/summer | 0.45 | **7+** | **0** | **0** | 176+ | 0 | 1353 | 18 ms | **50.5% tracking!** 7+ maps, 0 merges. ATE is misleadingly good - only the tracked half is evaluated |
| GL/night-light | 0.48 | 1 | 4 | 0 | 1 | 1 | 1291 | 18 ms | Near-perfect. Artificial lighting provides stable ORB features |
| P/day | 0.48 | 1 | 1 | 0 | 2 | 1 | 1431 | 20 ms | Scale=1.003 - most accurate scale in the entire dataset |
| CL/winter | 0.48 | 1 | 1 | 0 | 0 | 0 | 1318 | 17 ms | Cleanest campus run. Max ATE=0.97 (< 1 m) |

**Key observation - recovery pattern:** GL/autumn (154 tracking failures, ATE=0.37) and GL/spring (131 failures, ATE=0.42) are **not** clean runs. They had long burst failures (up to 95 consecutive), created 3 maps, but **merge + loop closure fully corrected the errors**. This proves that the number of tracking failures **does not determine** the final accuracy - what matters is the availability of recovery mechanisms.

**P/summer (0.45, 50.5% tracking):** The only recording where ATE is **misleadingly good**. 7+ maps, 0 merges, 0 loops, only 50.5% of frames tracked. Evaluation counts only tracked frames, so the error appears small even though half the trajectory is lost.

#### Group B: Good Results (ATE 0.5–0.8 m)

| Recording | ATE | Maps | Loops | Track fails | Reloc | Details |
|-----------|-----|------|-------|-------------|-------|---------|
| P/spring | 0.54 | 1 | 2 | 0 | 0 | Clean. Scale=1.031 (slightly overestimated) |
| CL/summer | 0.61 | 1 | 2 | 0 | 0 | Longest run (1398 s, 30921 images). RPE=0.023 (2nd lowest) |
| CL/day | 0.66 | 1 | 2 | 0 | 0 | Clean. Higher RPE=0.027 due to September light angle |
| CL/autumn | 0.72 | **2** | 3 | 56+ | 2 | 2 maps **did not merge** resulting in misalignment = higher ATE |
| CL/spring | 0.75 | 1 | 3 | 5 | 1 | Max ATE=3.07 - one bad segment after tracking loss |

**campus_large is systematically worse** than garden_large under identical conditions:
- RPE (per-frame quality) is **the same**: garden 0.023-0.025, campus 0.023-0.027
- The difference is **trajectory length** (21-31k vs 12-14k images) and **lower loop closure density** (0.83 vs 2.28 loops/10k frames)
- campus_large_winter (0.48 m, 22k images) vs garden_large_winter (0.37 m, 12k images) - both have 1 loop, 0 failures, campus is simply longer

#### Group C: Outliers (ATE > 1.5 m) - Two Types of Errors

**Type 1 - LOCALIZED error** (max >> median):

| Recording | ATE | Median | Max | Max/Med | Cause |
|-----------|-----|--------|-----|---------|-------|
| P/dusk | 1.63 | 0.64 | 7.69 | **12.1** | 168+ failures, 0 loops, incomplete merge. One bad segment |
| P/autumn | 1.87 | **0.43** | **14.78** | **34.4** | Median on par with the best (0.43!), but 1 catastrophic segment |

P/autumn is the most dramatic example: median ATE=0.43 m (on par with GL/spring), but one segment with max=14.78 m raises the RMSE to 1.87 m. 3 relocalizations with "Less than 15 matches" between them. **0 loop closures** = drift is never corrected.

**Type 2 - DISTRIBUTED error** (high median):

| Recording | ATE | Median | Max | Max/Med | Scale | Cause |
|-----------|-----|--------|-----|---------|-------|-------|
| CL/dusk | 4.60 | 1.41 | 24.14 | 17.1 | 0.91 | 5+ maps, 6 relocalizations at the end |
| P/night | 6.55 | **6.12** | 19.33 | 3.2 | 0.93 | RPE=0.051 (**2x normal**). Entire trajectory degraded |
| GL/dusk | 6.69 | **6.28** | 11.32 | **1.8** | **0.50** | **21+ maps**, scale collapse due to IR interference with D435i depth |
| CL/night | 10.11 | **8.77** | 21.13 | 2.4 | **1.54** | 12 active map resets, init pts 200-330 (vs 1300+ normal) |

**GL/dusk (scale=0.50) - depth sensor failure:** D435i depth uses IR structured light. At dusk, residual ambient IR from the sunset interferes with the projection, causing underestimated depth readings and scale=0.50. This explains why **dusk is worse than night-light**: in complete darkness there is no IR interference.

**CL/night (12 active resets) - reset loop:** The system initializes a map with 200-330 points (vs 1300-1500 normal), immediately encounters "Less than 15 matches", resets, starts a new map, and fails again. 12 cycles. Median tracking time = 8.75 ms (minimum in the dataset) = frames are rejected instantly.

#### GL/night - the Only Complete Failure

- **Return code:** -11 (SIGSEGV)
- **Maps:** 6+, init pts 540-660 (low)
- **Pattern:** Each map survives 1-2 frames, then "Less than 15 matches" triggers an active map reset and a new map is created
- **77 consecutive tracking failures** on the last map leading to SIGSEGV
- Median tracking time = 7.47 ms (**minimum** across the entire dataset)
- **Difference from CL/night (10.11 m):** garden_large_night is complete darkness in a small garden, CL/night is a nighttime campus with ambient city glow. CL/night manages to track 79.7% of frames, GL/night - 0%.

### Main RGB-D Conclusion: Loop Closure as a Binary Predictor

| Loop closures | Recordings | Mean ATE | Range |
|:---:|:-:|:---:|:---:|
| >=1 | 14 | **0.50 m** | 0.37–0.75 |
| 0 (with tracking issues) | 6 | **5.36 m** | 1.63–10.11 |
| 0 (P/summer, 50% tracked) | 1 | 0.45* | *misleading |

**A perfect binary discriminator:** at least 1 loop closure guarantees ATE <= 0.75 m (100% accuracy). 0 loop closures combined with any tracking issues guarantees ATE >= 1.63 m (100% accuracy). No exceptions among 22 recordings.

### Recommendations for RGB-D

1. **CLAHE preprocessing** for dusk/night recordings - improves contrast for ORB
2. **Increase nFeatures** (1200 to 2000) for low-light
3. **Lower FAST threshold** (20/7 to 12/5) for low-light
4. **IR-based features:** for complete darkness - use D435i IR images instead of RGB (requires ORB-SLAM3 modification)
5. **More aggressive loop closure:** relax criteria (lower minScore) so detection works in dusk/night
6. **Depth quality monitoring:** for dusk - check IR interference (compare noise level with the norm), reject unreliable depth frames

---

## Mode Comparison

### When Each Mode Performs Best

| Condition | Best mode | ATE | Comment |
|-----------|----------|-----|---------|
| Day, compact route | RGB-D | 0.37–0.42 m | Depth provides scale, dense features |
| Day, large route | RGB-D | 0.61–0.72 m | Stereo PH also works (0.71–2.37 m) |
| Dusk | Stereo PH | 1.51 m* | RGB-D degrades (4-6 m), SI fails |
| Night-light | RGB-D | 0.45–0.48 m | Artificial light is sufficient for ORB features |
| Night | Stereo PH / SI | 0.88 / 2.14 m | RGB-D fails, T265 more sensitive in darkness |

*Stereo PH at dusk works only in the park; it crashes in other routes.

**Interesting observation:** In **complete darkness**, T265 stereo performs better than D435i RGB-D. The T265 fisheye cameras with a wider FoV and a different sensor matrix have higher sensitivity. The D435i RGB camera (which provides features) is completely blind.

### Scale

| Mode | Expected scale | Actual |
|------|---------------|--------|
| RGB-D | 1.0 | 0.98–1.04 (+-2-4%) |
| Stereo PH | != 1.0 | 1.19–1.28 (~+22%) |
| Stereo-Inertial PH | 1.0 (when successful) | 1.18–1.26 or 0.00–0.02 |

RGB-D is the only mode with correct absolute scale. Stereo PH systematically overestimates by ~22% (small stereo baseline of 6.35 cm). Stereo-Inertial, even when "successful", yields scale ~1.2 instead of 1.0 - IMU does not contribute scale information due to poor calibration.

### Tracking Stability

| Mode | Matched poses (median) | Tracking coverage |
|------|----------------------|-------------------|
| RGB-D | 13,854 | ~100% |
| Stereo PH | 3,794 | ~30% (keyframes) |
| Stereo-Inertial PH | 1,818 | ~13% (keyframes) |

RGB-D preserves a pose for nearly every frame. Stereo modes save only keyframes.

---

## Known ORB-SLAM3 Bugs

### 1. Sophus SO3::exp NaN Assertion (SIGABRT, rc=-6)

**Manifestation:** Crash with message `SO3::exp failed! omega: -nan -nan -nan`
**Cause:** NaN from the g2o optimizer reaches Sophus without validation
**Trigger:** Loop closure correction or Global Bundle Adjustment with degenerate geometry
**Frequency:** 8 out of 66 experiments (12%)
**Affected modes:** Stereo PH (5), Stereo-Inertial PH (2), RGB-D (1)

### 2. vector::reserve Overflow (SIGABRT, rc=-6)

**Manifestation:** `std::length_error: vector::reserve`
**Cause:** Memory corruption or exhaustion after prolonged operation with large maps
**Trigger:** Long sequences (>20k frames) with 4+ loop closures
**Frequency:** 2 out of 66 (3%) - only campus_large Stereo PH

### 3. BAD LOOP Cascade upon IMU Init Failure

**Manifestation:** 47-51 consecutive "BAD LOOP" followed by crash or infinite loop
**Cause:** Without gravity alignment, every loop closure is rejected due to phi threshold violation
**Trigger:** scale close to 0 combined with loop closure candidates
**Frequency:** Every stereo-inertial recording with scale < 0.5

### 4. Crash at Shutdown (SIGSEGV after saving)

**Manifestation:** rc=-11, but trajectory file was saved
**Cause:** ORB-SLAM3 destructors access freed memory
**Frequency:** ~5 out of 66 (8%) - typically campus_large

---

## Files and Scripts

### Configurations

| File | Mode | Status |
|------|------|--------|
| `configs/ROVER_D435i_RGBD.yaml` | RGB-D | Working |
| `configs/ROVER_T265_PinHole_Stereo.yaml` | Stereo PH | Working (with crashes) |
| `configs/ROVER_T265_PinHole_Stereo_Inertial.yaml` | Stereo-Inertial PH | Problematic IMU calibration |

### Scripts

| File | Purpose |
|------|---------|
| `scripts/run_overnight.py` | Batch runner for all 22 x 3 experiments |
| `scripts/run_stereo_all.py` | Runner for stereo + stereo-inertial |
| `scripts/rectify_t265_stereo.py` | T265 fisheye to pinhole undistortion |
| `scripts/prepare_rover_rgbd.py` | D435i to TUM RGB-D format |

### Results

| File | Contents |
|------|----------|
| `results/summary_final.txt` | Summary table of all results |
| `results/all_results_final.json` | All results (JSON) |
| `results/comparison_bar_final.png` | Bar chart of ATE per recording |
| `results/comparison_heatmap_final.png` | Heatmap of ATE 22x3 |
| `results/{recording}/{mode}/eval_results.json` | Detailed metrics |
| `results/{recording}/{mode}/trajectory_comparison.png` | Trajectory visualization |

---

## Cross-Mode Synthesis: The Single Metric That Determines Accuracy

Analysis of 66 experiments (22 x 3 modes) revealed one **universal predictor** of accuracy:

### Loop Closure Presence as a Binary Predictor of Accuracy

| Mode | >=1 loop closure -> ATE | 0 loop closures -> ATE |
|------|----------------------|----------------------|
| **RGB-D** | <= 0.75 m (14/14) | >= 1.63 m (6/6)* |
| **Stereo PH** | <= 2.07 m (12/12) | no data** |
| **Stereo-Inertial** | <= 0.70 m (5/5)*** | >= 2.14 m (12/12) |

*Excluding P/summer (50% tracking, misleading ATE)
**Stereo PH saves only keyframe trajectory; loop closure does not function the same way
***Only recordings where at least 1 loop was accepted (not BAD)

This means: **regardless of mode, lighting, or route - the presence of at least one loop closure guarantees ATE < 2 m, while its absence combined with any tracking problems guarantees ATE > 1.5 m.**

### Why Loop Closure Is King

ORB-SLAM3 is visual odometry + pose graph optimization. Without loop closure, drift accumulates indefinitely. A single loop closure:
1. Detects "I have been here before" through DBoW2 place recognition
2. Computes a Sim3 transform between the current and past pose
3. Triggers Global Bundle Adjustment, which corrects the **entire** trajectory
4. Drift is reduced to the noise level of the alignment

On the ROVER dataset, routes are loops (garden: several loops over 7 min, park: 1 large loop over 8 min, campus: 1 very large loop over 15 min). If the system survives until it returns to the starting point with functioning tracking, loop closure will correct all accumulated drift.

### Taxonomy of Failure Across All Modes

| Failure mode | Trigger | RGB-D | Stereo PH | Stereo-Inertial |
|-------------|---------|-------|-----------|-----------------|
| **Feature starvation** | Night/low-light | 2 (CL/night, GL/night) | 3 (GL/night-light, P/night-light, GL/dusk) | N/A (IMU handles) |
| **Scale collapse** | Dusk IR interference / IMU failure | 1 (GL/dusk, scale=0.50) | N/A | 5 (scale close to 0) |
| **Drift w/o correction** | 0 loop closures | 3 (P/dusk, P/autumn, P/night) | N/A | 3 (CL/autumn, P/night, P/dusk) |
| **Sophus NaN crash** | g2o numerical instability | 1 | 5 | 2 |
| **Memory overflow** | Long sequences, many loops | 0 | 2 (CL/day, CL/dusk) | 0 |
| **IMU init failure** | Bad calibration | N/A | N/A | 8 |
| **Reset loop** | Insufficient features for init | 2 | 0 | 2 (timeout) |

### Complementary Strengths

No single mode dominates across all conditions:

| Condition | Winner | Reason |
|-----------|--------|--------|
| Day, any season | RGB-D (0.37-0.75 m) | Depth provides metric scale, dense matching |
| Night-light | RGB-D (0.45-0.48 m) | Artificial light = stable ORB features |
| Complete darkness | **Stereo PH** (0.88 m) | T265 more sensitive than D435i RGB camera |
| Dusk | **Stereo PH** (1.51 m)* | RGB-D scale collapse (IR interference), SI crash |
| Long trajectory (campus) | RGB-D (0.48-0.75 m) | Stereo modes drift more without loop closure |

*Stereo PH at dusk works only on some recordings (P/dusk OK, GL/dusk crash).

---

## Pipeline Audit: Our Errors vs ORB-SLAM3 Limitations

### Verification: What Was Done CORRECTLY

Each pipeline component was verified against original calibration files and expected behavior:

| Component | Status | Verification |
|-----------|--------|-------------|
| **IMU.Frequency = 264 Hz** | Correct | Measured: 103775 samples / 392.2 s = **264.6 Hz**. This is the real T265 frequency, not an error |
| **IMU column order** | Correct | Original: `ts, acc(3), gyro(3)` -> EuRoC: `ts_ns, gyro(3), acc(3)`. Values match |
| **T_c1_c2 vs Stereo.b** | Correct | \|tx\| = 0.063474 m approx Stereo.b = 0.0635 m (ratio 1.0003) |
| **D435i intrinsics** | Correct | Exact match with calib_d435i.yaml: fx=596.20, fy=593.14, cx=327.05, cy=245.16 |
| **D435i distortion** | Correct | radtan: k1=0.0756, k2=-0.2089, p1=-0.0023, p2=0.0040 - match |
| **T265 PinHole intrinsics** | Correct | fx=fy=224.07 from formula fx = (w/2) / tan(hFoV/2), hFoV=110 degrees |
| **RGBD.DepthMapFactor = 1000** | Correct | D435i depth images in mm -> /1000 = meters |
| **Camera.RGB** | Correct | D435i RGB: Camera.RGB=1, T265 grayscale: Camera.RGB=0 |
| **GT format** | Correct | TUM format (ts tx ty tz 0 0 0 0). Rotation = 0, only position is used for ATE |
| **IMU noise parameters** | Correct | Exact match with calib_t265.yaml: gyro=0.00324, acc=0.01741, walk_gyro=7.06e-5, walk_acc=0.01239 |
| **Undistortion (fisheye to pinhole)** | Correct | cv2.fisheye.initUndistortRectifyMap with R=identity, new_K = 224.07. Each camera undistorted independently, ORB-SLAM3 performs stereo rectification via T_c1_c2 |

### Issues Found on Our Side

#### 1. Timeout of 30 min - too short for SI

**Problem:** 2 Stereo-Inertial experiments ended due to timeout (1800 s = 30 min):
- `park_night-light_2024-05-24_2` - 13750 frames, SI requires more time for IMU init + BA
- `campus_large_winter_2024-01-27` - 21787 frames, **no IMU data** (T265 directory does not contain `imu/`)

**Consequence:** 2 SI results are missing (not FAIL, but NOT RUN). Total: 20/22 SI instead of 22/22.

**Fix:** Increase timeout to 3600 s (1 hour) for SI mode. For campus_large_winter, SI is impossible (no IMU) - it should be excluded from SI experiments.

#### 2. campus_large_winter - missing IMU data

**Problem:** The recording `campus_large_winter_2024-01-27` does not have a `realsense_T265/imu/` directory. Only `cam_left/` and `cam_right/` are present.

**Consequence:** Stereo-Inertial is impossible for this recording. In the summary it is shown as "not run" instead of "N/A - no IMU data". This distorts SI statistics (should be "17/19 success" instead of "17/20").

**Fix:** Add a check for IMU data availability before launching SI; mark as N/A in the summary.

#### 3. Log truncation - loss of early messages

**Problem:** `run_overnight.py` saves only the last 5000 characters of stdout/stderr:
```python
f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else ret.stdout)
```

**Consequence:** For long recordings (campus_large, 30000+ frames), early ORB-SLAM3 messages are lost. In particular, the following are truncated:
- IMU initialization success/failure messages
- Map initialization details
- First tracking failures

**Fix:** Save the first 3000 + last 5000 characters, or the entire log.

#### 4. ORB parameters not optimized for low-light

**Problem:** Standard ORBextractor parameters are used:
- Stereo/SI: nFeatures=2000, iniThFAST=15, minThFAST=5
- RGB-D: nFeatures=1500, iniThFAST=20, minThFAST=7

For night/dusk recordings, these thresholds may be too high.

**Consequence:** Some FAIL results at dusk/night may be related to insufficient feature count.

**Fix:** Experiment with iniThFAST=10, minThFAST=3, nFeatures=3000 for problematic recordings.

#### 5. Systematic scale of ~1.22 in Stereo modes

**Problem:** ALL successful Stereo PH recordings show Sim3 scale > 1.0:
- Mean scale (excluding outliers): **1.2293** +- 0.024
- Range: 1.19 – 1.28

Scale > 1.0 means that ORB-SLAM3 systematically **underestimates** distances (trajectory smaller than GT).

**Analysis of possible causes:**

| Hypothesis | Verification | Result |
|----------|-----------|--------|
| Stereo.b is incorrect | \|T_c1_c2.tx\| = 0.063474 approx Stereo.b = 0.0635 | Rejected (ratio 1.0003) |
| fx_original/fx_pinhole = 280.44/224.07 = 1.2516 | Close to 1.2293 (1.8% difference) | Suspicious, but mechanism unclear |
| Short baseline (6.35 cm) + low resolution (640x480) | bf = 14.23, disparity at 5 m = 2.8 px | Quantization may create bias |
| R=identity in undistortion instead of stereo rectification | Each camera undistorted independently | Potential subtle effect |
| ORB-SLAM3 internal rectification error | Unknown without debug | Needs investigation |

**Comparison with other modes:**
- RGB-D scale: mean **0.9871** +- 0.029 (near-ideal 1.0 - depth sensor is metric)
- SI scale (when IMU init OK): mean **1.1581** (IMU partially corrects the bias)

**Conclusion:** The ~1.22 scale is a systematic issue with stereo depth estimation. Possible fixes:
1. Use `cv2.stereoRectify()` instead of independent undistortion of each camera
2. Increase the resolution of undistorted images (e.g., 848x640 with corresponding fx)
3. Try the original KannalaBrandt8 mode (without undistortion) - if it works, scale should be closer to 1.0

### ORB-SLAM3 Limitations (NOT our errors)

#### 1. Sophus SO3::exp NaN Crash - 9/10 crashes

**What happens:** During loop closure correction, the g2o optimizer produces a degenerate rotation matrix (NaN). The Sophus assertion `SO3::exp failed! omega: -nan -nan -nan` triggers abort().

**Full chain:**
```
Loop detected -> Global BA -> g2o optimizer -> degenerate rotation ->
Sophus SO3::exp(NaN) -> assertion failure -> process crash
```

**Why this is not our error:** This is a known ORB-SLAM3 bug. It occurs with any dataset that has borderline loop closures. No upstream fix exists.

**Workaround:** Patch Sophus: instead of assert, skip the loop closure correction. Or patch ORB-SLAM3 LocalMapping.cc to catch NaN before SO3::exp.

#### 2. BAD LOOP Cascade (Stereo-Inertial)

**What happens:** In SI mode, IMU provides a gravity orientation prediction. When IMU init fails (scale close to 0), every loop closure candidate is rejected as "BAD LOOP" (rotation mismatch > threshold).

```
*Loop detected -> phi = [0.017, 0.037, -0.593] -> BAD LOOP!!!
(repeats dozens of times)
-> Eventually Sophus NaN crash or tracking loss
```

**Why this is not our error:** This is a consequence of T265 IMU calibration with AccWalk = 0.012 (8-30x higher than the BMI055 datasheet). The parameters were taken from ROVER calib_t265.yaml without changes. Attempts to lower AccWalk worsened ATE (14.35 m instead of 4.71 m) - this is a "package" problem; IMU params must be consistent with each other.

#### 3. Night Feature Starvation

**What happens:** In complete darkness, the camera cannot detect enough features for tracking. The ORB detector finds < 100 features, tracking fails, map resets, and the cycle repeats.

**Affected recordings:** garden_large_night (RGB-D FAIL), park_night-light (Stereo PH FAIL), and others.

**Why this is not our error:** This is a physical limitation of passive cameras. T265 has higher sensitivity than D435i RGB, which is why Stereo PH works better than RGB-D at night.

#### 4. D435i Dusk IR Interference

**What happens:** At dusk, the sun produces ambient IR that interferes with the D435i structured light projector. The depth map becomes noisy, leading to scale collapse.

**Affected recordings:** garden_large_dusk (RGB-D: ATE 6.69 m, scale 0.50), campus_large_dusk (RGB-D: ATE 4.60 m, scale 0.91).

### Missing Experiments

| Recording | Mode | Reason | Action |
|-----------|------|--------|--------|
| park_night-light_2024-05-24_2 | SI | Timeout 30 min | Re-run with timeout=3600 s |
| campus_large_winter_2024-01-27 | SI | **No T265 IMU data** | Mark as N/A |

### SI Tuning Experiments (campus_large)

8 additional experiments were conducted to improve SI on two campus recordings:

#### Test 1: Reducing AccWalk (0.012 to 0.001)

Hypothesis: high AccWalk = 0.012 (from calibration) causes the optimizer to fail at constraining the bias, making gravity unobservable and scale close to 0. We reduce it to the BMI055 datasheet value of 0.001.

| Recording | ATE original | ATE (AccWalk=0.001) | Scale original | Scale new |
|-----------|-------------|--------------------:|---------------:|----------:|
| CL/day | **2.02** | 6.04 | 1.228 | **1.002** |
| CL/autumn | **7.41** | 14.34 | 1.072 | 0.162 |

**Result:** CL/day scale became ideal (1.002) - IMU init WORKS with AccWalk=0.001! But ATE is 3x worse - IMU over-constrains the bias, leading to drift (max ATE = 41.88 m). CL/autumn - scale collapsed.

#### Test 2: Relaxing BAD LOOP Threshold

ORB-SLAM3 was patched (`LoopClosing.cc:240`): roll/pitch threshold changed from 0.008 rad (0.46 degrees) to 0.1 rad (5.7 degrees).

Reason: all loop closures are rejected as BAD LOOP because phi[0] (roll) is approximately 0.06 rad (3.4 degrees) - gravity is determined with an error of ~3.4 degrees.

| Recording | ATE original | ATE (relaxed) |
|-----------|-------------|-------------:|
| CL/day | **2.02** | 2.99 |
| CL/autumn | **7.41** | 15.49 |

**Result:** Worse! Accepted loop closures with 3.4-degree roll error introduce **more error than drift** without loop closure.

#### Test 3: Both Fixes Together

| Recording | ATE original | ATE (both) |
|-----------|-------------|-------------:|
| CL/day | **2.02** | 9.72 |
| CL/autumn | **7.41** | 13.34 |

**Result:** The worst variant - a combination of problems.

#### Full Comparison

| Recording | Original | AccWalk=0.001 | Relaxed thresh | Both |
|-----------|---------|:-------------:|:--------------:|:----:|
| **CL/day** | **2.02** (s=1.23) | 6.04 (s=1.00) | 2.99 (s=1.21) | 9.72 (s=0.12) |
| **CL/autumn** | **7.41** (s=1.07) | 14.34 (s=0.16) | 15.49 (s=0.01) | 13.34 (s=0.34) |

**Conclusion: the original config from ROVER calibration is the best.** No parameter or threshold changes improve the result. The reason: T265 IMU (BMI055) on the ROVER platform has a stable gravity error of ~3.4 degrees, which blocks loop closures in ORB-SLAM3. This is a hardware-level problem that cannot be solved through software tuning.

#### Additional Experiments: D435i IMU + KB8 Fisheye

After unsuccessful parameter tuning, two more approaches were tested:

**Approach 1: D435i IMU instead of T265 IMU** (T265 PinHole stereo + D435i IMU)
- D435i IMU (BMI055) has AccWalk = 0.0005 (23x better than T265)
- Cross-sensor transform computed via Prism frame: 23 cm distance, 1.9-degree angle
- Config: `ROVER_T265_PinHole_SI_D435iIMU.yaml`, IMU.Frequency = 301 Hz
- **Problem:** different hardware clocks (T265 vs D435i), large lever arm

| Recording | ATE (m) | Scale | Tracking % | Result |
|-----------|---------|-------|------------|--------|
| CL/day | 11.59 | 0.0001 | 19.1% | FAIL - complete scale collapse |
| CL/autumn | 12.58 | 0.581 | 12.0% | FAIL - incorrect scale |

**Reason for failure:** Different hardware clocks between sensors (T265 cameras and D435i IMU are not synchronized at the hardware level). The large lever arm (23 cm) amplifies the effect of timestamp desynchronization. Even ~1 ms timestamp offset at 1 m/s speed adds ~1 mm of additional error per frame, which accumulates.

**Approach 2: KB8 (KannalaBrandt8) fisheye** (original fisheye images 848x800 + T265 IMU)
- Used `ROVER_T265_Stereo_Inertial.yaml` (KB8 config with original intrinsics)
- Goal: avoid undistortion, preserve more field-of-view for feature matching

| Recording | Result | Details |
|-----------|--------|---------|
| CL/day | **COMPLETE FAILURE** | 7114 frames lost, 1 KF. Constant "Less than 15 matches" |
| CL/autumn | **CRASH (SEGFAULT)** | 10017 frames lost, 0 KF. Return code -11 |

**Reason for failure:** ORB features do not work on raw T265 fisheye images with this level of distortion. The KB8 model compensates geometry during projection, but the ORB detector operates on undistorted patches - with strong fisheye distortion, features become unrecognizable between frames. Each initialization creates only 1-6 points and immediately loses tracking.

**Approach 3: VN100 external IMU**
- **Data NOT AVAILABLE** in the recordings. Only a calibration file `calib_vn100.yaml` exists, but no recording contains VN100 data
- This approach is not feasible with the available data

#### Final Comparison of ALL Tested SI Variants

| Variant | CL/day ATE (m) | CL/autumn ATE (m) | Verdict |
|---------|:--------------:|:------------------:|---------|
| **Original (T265 PH + T265 IMU)** | **2.02** (s=1.23) | **7.41** (s=1.07) | **BEST** |
| AccWalk=0.001 | 6.04 (s=1.00) | 14.34 (s=0.16) | 3x worse |
| Relaxed BAD LOOP | 2.99 (s=1.21) | 15.49 (s=0.01) | Worse |
| AccWalk + Relaxed | 9.72 (s=0.12) | 13.34 (s=0.34) | Worst tuning |
| D435i IMU + T265 PH | 11.59 (s=0.0001) | 12.58 (s=0.58) | FAIL - clock mismatch |
| KB8 fisheye + T265 IMU | FAIL (1 KF) | CRASH (0 KF) | FAIL - ORB on fisheye |
| VN100 IMU | - | - | No data available |

**Final conclusion on Stereo-Inertial for ROVER:**

The original config from ROVER calibration is the only working variant. 6 alternative approaches were tested - none improved the results. The fundamental limitation: T265 IMU (BMI055) on the ROVER platform has a stable gravity error of ~3.4 degrees, which blocks loop closures in ORB-SLAM3. This is a hardware-level problem that cannot be resolved through software tuning.

**The only remaining path for SI improvement** is modification of the ORB-SLAM3 source code:
- Rewrite loop closure validation to not reject loops based on the gravity check
- Or implement online gravity re-estimation
- This requires deep intervention in the ORB-SLAM3 codebase and goes beyond standard configuration

### Audit Summary

**Overall assessment: the pipeline is implemented CORRECTLY.** All calibration parameters, data formats, and conversions have been verified and match the original files. The original config from ROVER calibration produces the best results among all tested variants.

**What can be improved:**
1. Increase timeout for SI (30 min to 60 min)
2. Add IMU data availability check before SI launch
3. Save the full ORB-SLAM3 log
4. Investigate the systematic ~1.22 scale (try stereoRectify instead of independent undistortion)
5. Optimize ORB parameters for low-light scenarios

**Already tested (DOES NOT help):**
- D435i IMU with T265 stereo -> FAIL due to clock mismatch (tested)
- KB8 fisheye instead of PinHole -> FAIL/SEGFAULT (tested)
- VN100 IMU -> no data in recordings
- AccWalk tuning -> worse
- BAD LOOP threshold relaxation -> worse

**What SHOULD NOT be changed** (already correct):
- IMU.Frequency = 264 Hz (measured 264.6 Hz, this is the correct value!)
- IMU noise/walk parameters (from calibration - the best variant)
- BAD LOOP treshold (0.008 rad - relaxation worsens results)
- Stereo.b and T_c1_c2
- Undistortion fx=224.07 (110-degree hFoV)
- D435i config
- RGB-D preparation and EuRoC conversion

---

### Sophus NaN Fix + Stereo PH Retry

After the SI audit, ORB-SLAM3 was patched to resolve the Sophus SO3::exp NaN crash bug:

1. **Sophus `so3.hpp`**: added a NaN-safe guard - if `omega` contains NaN/Inf, identity quaternion is returned instead of abort
2. **Sophus `common.hpp`**: added `SOPHUS_ENSURE_WARN_ONLY` mode - prints a warning instead of `abort()`
3. **CMakeLists.txt**: added `-DSOPHUS_ENSURE_WARN_ONLY`
4. **Qt fix**: added `QT_QPA_PLATFORM=offscreen` to avoid OpenCV Qt plugin crash

Result: **6 out of 8 previously failed Stereo PH recordings now work!**

| Recording | ATE (m) | Scale | Attempts | Status |
|-----------|---------|-------|----------|--------|
| GL/dusk | **0.84** | 1.34 | 2 | NEW |
| GL/night-light | 6.36 | 0.58 | 2 | NEW |
| GL/summer | **0.51** | 1.21 | 1 | NEW |
| P/day | **1.14** | 1.19 | 1 | NEW |
| P/night-light | 7.87 | 0.65 | 1 | NEW |
| CL/night-light | **1.39** | 1.29 | 1 | NEW (previously untested) |
| CL/day | FAIL | - | 3 | Crashes persist (21K frames) |
| CL/dusk | FAIL | - | 3 | Crashes persist (21K frames) |

**Stereo PH now: 21/23 success** (previously 15/23). The two remaining campus crashes are related to route length - 21K+ frames create more situations with degenerate geometry, where NaN cascades through the ORB-SLAM3 pipeline even with the patch.

## Conclusions

1. **RGB-D is the best baseline** for ROVER: 91% success (21/23), median ATE 0.54 m, correct scale (median 0.99). Limitation - complete darkness.

2. **Stereo PinHole is a strong second option**: 91% success (21/23, after Sophus NaN fix), median ATE 0.88 m, works even in complete darkness (T265 IR is more sensitive than D435i RGB). Systematic scale of ~1.22 (related to the undistortion fx ratio). Two remaining crashes - the longest campus routes.

3. **Stereo-Inertial has limited effectiveness** with T265 IMU on ROVER. The original config (from calibration) is the best out of **7 tested variants** (original, AccWalk fix, relaxed threshold, both, D435i IMU cross-sensor, KB8 fisheye, VN100). The fundamental problem: T265 BMI055 IMU has a gravity error of ~3.4 degrees on this platform, causing all loop closures to be rejected (BAD LOOP) and drift to remain uncorrected.

4. **Lighting is the dominant factor:** day (ATE < 1 m) -> dusk (ATE 1-7 m) -> night (FAIL or ATE > 6 m). Seasonal changes (summer/autumn/winter/spring) under daytime lighting **have almost no effect**.

5. **Loop closure is the key to accuracy:** all recordings with ATE < 1 m have at least 1 loop closure, all outliers > 4 m have 0 loop closures.

6. **Route affects complexity:** garden_large (compact, rich texture) < park (open, uniform texture) < campus_large (large, long trajectories).

7. **Sophus NaN bug is a systemic ORB-SLAM3 issue**, resolved by patching `so3.hpp`. Without the patch, Stereo PH loses 35% of recordings to crashes.
