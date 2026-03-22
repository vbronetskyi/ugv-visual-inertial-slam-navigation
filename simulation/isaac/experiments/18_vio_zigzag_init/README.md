# exp 18: VIO offline with zigzag IMU initialization - SUCCESS! 🎉

## goal

Make ORB-SLAM3 RGBD-Inertial SLAM produce a usable trajectory by fixing
the IMU initialization problem from exp 17.

## The problem (from exp 17)

Offline VIO ran end-to-end (sync issue solved with offline binary), but
trajectory had **scale 0.08-0.15** and **ATE 8-22m** vs RGBD-only baseline
of 0.49m. Root cause: slow straight-line driving doesn't excite the
accelerometer enough for proper scale recovery during IMU initialization.

## the fix

Added a **2-second stationary + 10-second zigzag** phase at the start of
the recording in `simple_pure_pursuit.py`:

```python
# Phase 1: stationary (2s) - bias estimation
# phase 2: zigzag (10s) - alternating linear+angular for IMU excitation
phase = (elapsed - 2.0) % 4.0
if phase < 1.0:   v=0.5, w=+0.5    # forward + left
elif phase < 2.0: v=0.8, w=-0.5    # faster + right
elif phase < 3.0: v=0.3, w=+0.4    # slower + left
else:             v=0.0, w=0.0     # stop (braking)
```

This produces:
- Stationary: acc_x std=0.27 (small, OK for bias)
- **Zigzag: acc_x range = ±1.49 m/s² (3g excitation)**, gyro_z range = ±0.18 rad/s
- Straight: acc_x range = ±0.12 (post-init, stable)

The 3g acceleration excursion gives ORB-SLAM3 enough information to
solve the scale and gravity direction simultaneously.

## configuration

- IMU filter: 60/30/30 (from exp 15)
- IMU effective rate: **60.0 Hz** (verified - exactly 16.67 ms dt)
- Tbc: identity rotation + translation (0.3, 0, 0.1)
- Recording: 2845 frames, 17520 IMU readings, 165m route

```yaml
IMU.NoiseGyro: 2.2e-2
IMU.NoiseAcc: 5.0e-2
IMU.GyroWalk: 1.0e-3
IMU.AccWalk: 5.0e-3
IMU.Frequency: 60

Tbc:
  data: [1.0, 0.0, 0.0, 0.3,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.1,
         0.0, 0.0, 0.0, 1.0]
```

## Results - RGBD-I beats RGBD-only

| Method | ATE | Scale | Notes |
|---|---|---|---|
| RGBD-only baseline | 0.49 m | - | best previous result |
| RGBD-I exp 17 (no init, Variant B Tbc) | 22.0 m | 0.150 | 6.7x scale error |
| RGBD-I exp 17 (no init, identity Tbc) | 8.5 m | 0.078 | 12.8x scale error |
| **RGBD-I exp 18 (zigzag init, identity Tbc)** | **0.116 m** | **0.993** | **4.2x BETTER than RGBD-only** |

VIO trajectory vs GT: 1377 frames (1500 input, ~150 sec), tracks 86m of
actual robot motion. Best alignment convention: `nav_x=slam_z, nav_y=-slam_x`
(camera frame Z=forward, X=right -> nav x=forward, y=left).

Single map reset at KF 28 (during the zigzag ↔ straight transition), then
stable tracking throught. Only 1 "Fail to track local map" event in 1500 frames.

## What we tried that didn't work

| Attempt | Result | Why |
|---|---|---|
| exp 5-7: live VIO with filter=20 | Empty IMU measurements | rolling buffer sync bug |
| exp 16: live VIO with filter=60 | Empty IMU measurements | same bug - filter doesn't fix sync |
| exp 17: offline VIO no init | scale 0.08-0.15 | accelerometer not excited |
| exp 18 (this): offline + zigzag init | **scale 0.99, ATE 0.12m** | x |

Three things had to be right simultaneously:
1. **Offline binary** (not live) - solves IMU/camera sync
2. filter=60 IMU - gives Phidgets-level noise
3. **Zigzag init maneuver** - excites accelerometer for scale recovery

## How to reproduce

```bash
# 1. Record with zigzag init
export ROS_DOMAIN_ID=124
/opt/isaac-sim-6.0.0/python.sh run_husky_nav2.py \
    --route road --duration 600 --use-slam --use-vio --no-obstacles &

source /opt/ros/jazzy/setup.bash
python3 simple_pure_pursuit.py  # has 2s stationary + 10s zigzag init

# wait for "ROUTE COMPLETE"

# 2. Convert IMU and gen associations
REC=/root/bags/husky_real/exp18_zigzag_init_filter60
python3 convert_imu.py $REC

# 3. Run offline VIO (use first 1500 frames to avoid segfault on save)
head -1500 $REC/associations.txt > $REC/associations_half.txt
cd /tmp/orb_offline
/workspace/third_party/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_offline \
    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /root/bags/husky_real/rgbd_inertial_offline_test.yaml \
    $REC \
    $REC/associations_half.txt \
    $REC/imu_orbslam.txt
```

## Files

- `scripts/simple_pure_pursuit.py` - pure pursuit with 2s stationary + 10s zigzag init
- `scripts/run_husky_nav2.py` - Isaac Sim with `--use-vio` flag, filter=60
- `config/rgbd_inertial_offline.yaml` - VIO config with measured noise
- `logs/isaac.log`, `logs/pure_pursuit.log`, `logs/orb_offline_vio.log`
- `results/CameraTrajectory.txt` - VIO trajectory output (1377 poses)
- `results/KeyFrameTrajectory.txt` - VIO keyframes
- `results/vio_zigzag_vs_gt.png` - VIO vs GT visualization
- Recording: `/root/bags/husky_real/exp18_zigzag_init_filter60/`

## Implications for navigation

Now that VIO is working offline:
1. Build a VIO atlas offline from the recording
2. Save the atlas for later loading
3. **Run live navigation** with `rgbd_inertial_live` in localization mode (loading atlas) - the live binary should work because in localization mode it doesn't need to do IMU initialization (the atlas already has it)
4. Compare drift during obstacle bypass vs RGBD-only SLAM

If localization mode also works with VIO atlas, this could give us
significantly better drift handling than RGBD-only during navigation.

## Open issues

- Segfault during shutdown if processing >1500 frames at once. Workaround:
  process in chunks or save trajectory incrementally. May be a memory leak
  in atlas serialization.
- One map reset still happens at the zigzag -> straight transition. Could
  be eliminated by tuning init duration or noise parametars, but trajectory
  is good enough as-is.
