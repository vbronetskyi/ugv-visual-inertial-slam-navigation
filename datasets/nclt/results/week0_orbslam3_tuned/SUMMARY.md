# Experiment 0.5: ORB-SLAM3 Systematic Parameter Tuning - NCLT Spring 2012-04-29

Generated: 2026-02-13

## Key Finding

**ORB-SLAM3 monocular is fundamentally incompatible with the NCLT Ladybug3 camera system.** After a systematic investigation of all 6 cameras, 23 parameter configurations, preprocessing methods, and resolution variations, the maximum sustained tracking rate on 3000 frames was only **27.9%** - and even that produced trajectories bearing no resemblance to the ground truth road shape. The experiment also revealed that **all previous visual SLAM experiments (0.2–0.4) used Cam0, the sky-facing top camera**, explaining their catastrophic failures.

---

## Phase 0: Camera Investigation

The Ladybug3 has 6 cameras: 5 side-facing at 72° intervals + 1 top-facing (Cam0).

| Camera | Brightness | Contrast | ORB Features | Orientation | Notes |
|--------|-----------|----------|-------------|-------------|-------|
| **Cam0** | 94 | 73.6 | 851 | **TOP (sky)** | Used in ALL prior experiments - root cause of failure |
| Cam1 | 34 | 17.5 | 1632 | side | Very dark, low contrast |
| Cam2 | 44 | 28.9 | 2170 | side | Dark |
| Cam3 | 50 | 36.9 | 2193 | side | Buildings, structured scenes |
| Cam4 | 74 | 75.1 | 1143 | side | Best contrast |
| Cam5 | 83 | 72.0 | 2349 | side | Most ORB features, bright |

**Critical discovery:** Cam0 faces the sky. Every visual SLAM experiment before this one (ORB-SLAM3 v1/v2, DROID-SLAM, DPVO, DPV-SLAM) attempted to track features on clouds and tree canopy. This single finding explains why all visual methods produced chaotic, non-functional trajectories.

## Phase 1: Quick Validation (1000 frames)

Tested top 3 cameras (by ORB×Contrast heuristic) with baseline and CLAHE preprocessing:

| Camera | Config | Tracking Rate | Keyframes |
|--------|--------|--------------|-----------|
| Cam3 | baseline | **61.6–68.7%** | 616–687 |
| Cam5 | CLAHE | 28.7–69.7% | 287–697 |
| Cam5 | baseline | 5.8–12.7% | 58–127 |
| Cam4 | CLAHE | 21.1% | 211 |
| Cam4 | baseline | 0.7% | 7 |

**Non-determinism alert:** Repeated runs on identical data/config produced wildly varying tracking rates. Cam5+CLAHE ranged from 28.7% to 69.7% across two attempts. Cam3 baseline ranged from 61.6% to 68.7%. This level of non-determinism (RANSAC, random feature selection) makes any single run unreliable for comparison.

**Additional test - Cam1+CLAHE (3000 frames):** Only **7.6%** tracking despite having the highest ORB feature count after CLAHE (4895 features). Dark images with repetitive vegetation texture do not produce trackable features.

## Phase 2: Systematic Parameter Sweep (3000 frames, 23 runs)

All runs on Cam5+CLAHE (selected by Phase 1's 2nd run as best camera). Total runtime: ~4 hours.

### Step 1: nFeatures sweep
| nFeatures | Tracking | Keyframes | ATE RMSE |
|-----------|----------|-----------|----------|
| 1000 | 9.5% | 284 | N/A |
| 2000 | 2.9% | 87 | 573.6m |
| 3000 | 5.7% | 172 | N/A |
| 5000 | 0.9% | 26 | 67.6m |
| 8000 | 3.2% | 96 | N/A |

No clear trend - more features does NOT help. nFeatures=5000 tracked only 26 keyframes (0.9%).

### Step 2: FAST threshold sweep
| Config | Tracking | Keyframes |
|--------|----------|-----------|
| FAST 20/7 (default) | **26.0%** | **779** |
| FAST 15/5 | 9.4% | 282 |
| FAST 10/3 | 9.4% | 282 |
| FAST 7/3 | 9.3% | 280 |
| FAST 5/2 | 8.7% | 261 |

Default FAST thresholds (20/7) unexpectedly performed best. Lower thresholds (more features detected) actually degraded tracking - likely due to more unstable/noisy features overwhelming the matcher.

### Step 3: Scale/levels sweep
| Config | Tracking | Keyframes |
|--------|----------|-----------|
| scale 1.2, 10 levels | **14.2%** | **426** |
| scale 1.2, 8 levels (default) | 8.9% | 268 |
| scale 1.1, 12 levels | 8.0% | 240 |
| scale 1.1, 10 levels | 6.1% | 182 |
| scale 1.15, 10 levels | 5.2% | 157 |
| scale 1.3, 8 levels | 8.8% | 263 |

More pyramid levels helped slightly (1.2/10 = 14.2%), but results are still poor.

### Step 4: CLAHE preprocessing
| Config | Tracking | Keyframes |
|--------|----------|-----------|
| No preprocessing | 9.1% | 272 |
| CLAHE clip=2.0 | 8.9% | 266 |
| CLAHE clip=4.0 | 9.0% | 269 |

CLAHE made negligible difference on 3000-frame runs (all ~9%). The dramatic effect seen in Phase 1 (1000 frames) was likely coincidence due to non-determinism.

### Step 5: Camera comparison
| Camera | Tracking | Keyframes |
|--------|----------|-----------|
| **Cam4** | **27.9%** | **838** |
| Cam3 | 5.3% | 158 |

Cam4 (highest contrast) outperformed Cam5 on 3000 frames - another reversal from Phase 1.

### Step 6: Resolution
| Resolution | Tracking | Keyframes | ATE RMSE |
|------------|----------|-----------|----------|
| Half (808×616) | 7.7% | 230 | N/A |
| 3/4 (1212×924) | 1.2% | 35 | 96.1m |

Higher resolution degraded performance (slower processing, more distortion in outer regions).

## Phase 3: Full Session Run

Full spring session (21,510 frames) on Cam4 with best Phase 2 parameters: **TIMED OUT** after 45 minutes (2700s limit). No full-session trajectory produced.

## Trajectory Quality

All trajectories from Phase 2 were Sim(3)-aligned to ground truth. **None follow the ground truth road shape.** The GT traces a clear structured road network; estimated trajectories are chaotic, tangled blobs. The ATE numbers (67–573m where computed) are artifacts of Sim(3) fitting on fundamentally wrong-shaped trajectories and should not be interpreted as meaningful accuracy.

---

## Conclusion

### What we learned

1. **Cam0 = sky camera.** This is the single most important finding. Experiments 0.2 (ORB-SLAM3 baseline), 0.3 (ORB-SLAM3 v2), and 0.4 (DROID-SLAM/DPVO/DPV-SLAM) all used Cam0 images - they were tracking clouds, not roads. While this explains the catastrophic failures, switching to side cameras (Cam3/4/5) did NOT solve the problem.

2. **ORB-SLAM3 is fundamentally unreliable on NCLT Ladybug3.** After testing 6 cameras, 5 nFeatures values, 5 FAST thresholds, 6 scale/level combos, 3 CLAHE settings, and 2 resolutions (23 runs total), the best sustained tracking rate on 3000 frames was **27.9%** (Cam4). This is far below the ~80%+ needed for a usable trajectory.

3. **Non-determinism makes systematic tuning unreliable.** The same configuration on the same data produces tracking rates varying by 2–4× across runs. This makes it impossible to draw confident conclusions from single runs, and would require multiple repetitions per configuration to establish statistical significance - impractical at ~10 min/run.

4. **More features ≠ better tracking.** nFeatures=5000 tracked only 0.9% (worst result). Cam1+CLAHE had 4895 ORB features but only 7.6% tracking. The quality and stability of features matters more than quantity, and the Ladybug3 fisheye images don't produce consistently matchable features.

5. **CLAHE is not a reliable fix.** Dramatic improvements on short sequences (1000 frames) disappeared on longer runs (3000 frames), revealing the effect was dominated by ORB-SLAM3's inherent randomness rather than genuine image quality improvement.

### Root causes (unchanged from Experiment 0.3)

| Factor | Impact | Can we fix it? |
|--------|--------|----------------|
| **5 Hz camera** | Inter-frame motion 4–6× larger than typical, exceeds feature matching assumptions | No - hardware limitation |
| **Fisheye → pinhole approximation** | ~120° FOV fisheye distortion unmodeled; outer-region features mislocalized | Partially - need Kannala-Brandt model (not supported by ORB-SLAM3 monocular) |
| **No IMU fusion** | Monocular scale unrecoverable; no motion prior to bridge tracking gaps | Infeasible - MS25 IMU at 34 Hz too slow for ORB-SLAM3 VIO (needs ~200 Hz) |
| **Image quality** | Variable exposure, overexposed sky regions, low-texture vegetation | Limited - CLAHE didn't help meaningfully |
| **ORB-SLAM3 non-determinism** | RANSAC + random feature selection → 2–4× variance across runs | No - fundamental to the algorithm |

### Verdict

**ORB-SLAM3 monocular is NOT viable for NCLT Ladybug3.** No parameter configuration achieved reliable tracking on the spring session. The combination of 5 Hz fisheye camera, unmodeled distortion, and lack of IMU fusion creates conditions that feature-based monocular SLAM cannot overcome through parameter tuning alone.

**LiDAR ICP remains the only functional odometry method on NCLT**, with Spring ATE = 174.0m over the full 3.2 km trajectory (100% coverage).

### What could work next

- **VINS-Fusion** with Kannala-Brandt fisheye model + IMU fusion - addresses two root causes simultaneously (fisheye + IMU)
- **Wheel odometry fusion** - provides metric scale and bridges visual tracking gaps
- **GPS constraints** - already proven effective in LiDAR pipeline (174m → potential further improvement)

---

## Files

| File | Description |
|------|-------------|
| `tuning_runs/all_runs.json` | All 23 Phase 2 run results (params, tracking, ATE) |
| `tuning_runs/run001–016/trajectory.txt` | Individual trajectory files |
| `tuning_log.txt` | Full timestamped execution log (~5.2 hours) |
| `best_config.yaml` | Best config from Phase 2 (Cam4, FAST 20/7) |
| `camera_samples/` | Sample images from all 6 cameras |
| `cam1_quick_test/result.json` | Cam1+CLAHE test results |
| `parameter_sweep.png` | 6-panel bar chart of all sweep results |
| `camera_comparison.png` | ORB features and contrast per camera |
| `trajectories_progress.png` | All trajectories vs ground truth |
