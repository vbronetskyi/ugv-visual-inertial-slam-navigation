# exp 17: Offline RGBD-Inertial SLAM with filter=60 IMU

## goal

Test ORB-SLAM3 RGBD-Inertial offline mode (avoiding live VIO sync bugs from exp 16) with the cleaner IMU from exp 15 (filter=60, Phidgets-level noise).

Hypothesis: cleaner IMU + offline = working VIO with ATE close to RGBD-only baseline (0.49m).

## Setup

- Two recordings tested:
  - `exp16_vio_filter60_failed/` - 569 frames, ~5m of actual robot motion (failed pure pursuit, robot spun in place). Mostly stationary.
  - **`exp17_clean_mapping_filter60/`** - 3316 frames, ~165m driven (full road route, no obstacles). Used for main test.
- IMU: filter_size=60 (from exp 15), 6 readings per camera frame, 60 Hz effective rate
- ORB-SLAM3 binary: `rgbd_inertial_offline` (avoids live IMU sync bug)
- Tested 3 yaml configs:
  - `v1_identity.yaml` - identity Tbc rotation, translation (0.3, 0, 0.1)
  - `v2_tighter_noise.yaml` - lower noise params, more ORB features
  - `v3_variantB.yaml` - Variant B Tbc from previous tests

## Results

### Sync issue solved

The "Empty IMU measurements vector" error from live VIO is **gone**. Offline binary correctly reads `imu_orbslam.txt` (full file, not 100-entry buffer), 6 IMU readings per frame, no sync errors.

### but trajectory quality still bad

| Config | Maps | Fail tracks | ATE | Scale |
|---|---|---|---|---|
| v3 Variant B (1500 frames) | 3 | 55 | 22.0m | 0.150 |
| v1 Identity (1500 frames) | 2 | 1 | 8.5m | 0.078 |
| RGBD-only baseline | - | - | 0.49m | 1.0 |

Both configs produce trajectories with **massive scale errors** (0.08-0.15 - meaning SLAM thinks robot moved 6-12x further than actual). Identity Tbc has fewer tracking failures (1 vs 55) but worse scale.

Raw VIO output for identity Tbc (1500 frames over 150 seconds, 84m actual motion):
- `tx`: -234 to +994 (range 1228m vs actual ~0)
- `ty`: -45 to +279 (range 324m vs actual ~9m)
- `tz`: -3 to +751 (range 754m vs actual ~84m)

The IMU integration is exploding - the robot is moving slowly and steadily, but VIO produces huge translation estimates. This is not a noise problem (filter=60 gives Phidgets-level noise), it's a **calibration / scale** problem.

### root cause: Tbc + scale ambiguity

ORB-SLAM3 VIO requires extremely accurate IMU-camera extrinsic calibration (Tbc) AND a non-degenerate motion pattern for IMU initialization. With:

1. **Slow straight-line driving** (mostly forward at 0.5 m/s, gentle turns) - accelerometer measures only gravity, no real linear acceleration -> IMU initialization is degenerate
- Identity Tbc translation guess - actual mounting offset between IMU and camera frames in the URDF may differ from the assumed (0.3, 0, 0.1)
3. PhysX velocity reporting - even with filtered acceleration, the underlying rigid body velocity has artifacts that accumulate over preintegration

Variant B Tbc has explicit rotation matching the OLD URF->FLU IMU convention. With the new convention (Python URF->FLU conversion in `run_husky_nav2.py`), Variant B is wrong - should be identity rotation. But identity also fails because the underlying motion pattern doesn't excite the IMU enough for proper scale recovery.

## trade-offs

### What works
- x Live IMU/camera sync bug solved (offline binary)
- x filter=60 IMU is clean (Phidgets-level)
- x Tracking actually runs (frames processed end to end)
- x All 3316 frames processed without resets in best case

### what doesn't work
-  Scale recovery: 0.08-0.15 (catastrophically wrong)
-  ATE: 8-22m (RGBD-only is 0.49m)
-  Cannot save trajectory for full 3316-frame dataset (segfault during shutdown - works for 1500 frames)

## Conclusion

**Filter improvement (exp 15) does NOT enable working VIO with PhysX IMU.**

The IMU is now clean enough for many uses (gyro yaw fusion etc.), but ORB-SLAM3 RGBD-Inertial mode still cannot produce a usable trajectory. The fundamental obstacle is:
1. Smooth/slow forward driving doesn't excite IMU initialization properly
2. PhysX rigid body dynamics produce velocity artifacts that don't match a real-world motion model
3. Tbc calibration is brittle and hard to verify without working VIO to test against

**Recommendation:** Stop pursuing VIO. Use:
- RGBD-only SLAM (baseline 0.49m ATE)
- IMU gyro yaw fusion in `tf_wall_clock_relay.py` (uses filter=60 IMU)
- The current SLAM frame navigation approach (exp 13/14, 145m route completion)

VIO with PhysX IMU is **not viable** for this project. Real Phidgets Spatial 1042 on a real Husky would likely work, since:
- Real wheels don't have PhysX contact solver artifacts
- Real motion has acceleration/braking that excites the IMU
- Real Husky has tested Tbc from manufacturer specs

## files

- `scripts/imu_offline_test.py` - convert IMU and run offline binary
- `config/v1_identity.yaml`, `v2_tighter_noise.yaml`, `v3_variantB.yaml` - three configs tested
- `logs/` - 7 raw run logs
- `results/CameraTrajectory_identity.txt` - best VIO output (still bad)
- `results/vio_identity_vs_gt.png` - visualization of VIO vs GT
- Recording: `/root/bags/husky_real/exp17_clean_mapping_filter60/`

## reproduction

```bash
# 1. Use clean recording from exp17_clean_mapping_filter60
REC=/root/bags/husky_real/exp17_clean_mapping_filter60

# 2. Convert IMU to ORB-SLAM3 format and gen associations
# (see scripts/convert_imu.py)

# 3. Run offline (use first 1500 frames to avoid segfault)
head -1500 $REC/associations.txt > $REC/associations_half.txt
cd /tmp/orb_offline && /workspace/third_party/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_offline \
    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    config/v1_identity.yaml \
    $REC \
    $REC/associations_half.txt \
    $REC/imu_orbslam.txt
```
