# exp 16: VIO mapping with filter=60 IMU - FAILED (different bug)

## Goal

Test ORB-SLAM3 RGBD-Inertial mapping with the cleaner IMU from exp 15
(filter_size=60, noise std matches real Phidgets Spatial 1042).

## Configuration

- IMU filter: 60/30/30 (linear/angular/orientation)
- ORB-SLAM3 noise params (`rgbd_inertial_mapping.yaml`):
  - `IMU.NoiseGyro: 2.2e-2` (was 1.0e-3)
  - `IMU.NoiseAcc: 5.0e-2` (was 1.0e-2)
  - `IMU.GyroWalk: 1.0e-3` (was 1.0e-6)
  - `IMU.AccWalk: 5.0e-3` (was 1.0e-4)
- Tbc: identity rotation + translation (0.3, 0, 0.1) - IMU/cam both in FLU
- Test: road route, no obstacles, simple pure pursuit
- SLAM binary: `rgbd_inertial_live`

## result: FAILED

```
frames=285 lost=281
Empty IMU measurements vector!!!
not IMU meas
```

281 of 285 frames lost. SLAM never initialized tracking. Same failure mode
as previous VIO attempts (exp 5-7), even though IMU noise is now Phidgets-level.

## why filter=60 didn't fix VIO

The filter improvement was real and measured (exp 15). But VIO has a
separate, pre-existing bug: **live IMU/camera timestamp sync**.

`rgbd_inertial_live` reads `/tmp/isaac_imu.txt` which is a 100-entry rolling
buffer maintained by `run_husky_nav2.py`. At 200 Hz, 100 entries = 0.5
seconds of IMU data.

When the live binary processes a camera frame at timestamp T, it tries to
find IMU readings between T_prev and T. If the file only contains the most
recent 100 entries, and the camera processing lags by more than 0.5s
(reading frames from disk, ORB extraction, etc.), the IMU window has moved
past the camera frame timestamp -> no measurements between consecutive
camera frames -> "Empty IMU measurements vector".

This is independent of IMU noise. It would happen even with a perfect
real-world IMU.

## Conclusion

- IMU filter=60 improvement (exp 15) is valid and shipped in
  `run_husky_nav2.py` - usefull for any IMU consumer (e.g., gyro yaw
  fusion in `tf_wall_clock_relay.py`)
- VIO live mode requires fixing the IMU file sync (larger ring buffer,
  proper timestamp matching, or offline approach)
- Recommendation: **don't pursue live VIO** - it's a separate engineering
  problem. Use offline ORB-SLAM3 RGBD-Inertial on recorded `imu.csv` for
  atlas building if VIO is needed.

## files

- `scripts/run_husky_nav2.py` - with `--use-vio` flag and filter=60
- `scripts/simple_pure_pursuit.py` - basic waypoint follower for mapping runs
- `config/rgbd_inertial_mapping.yaml` - updated noise params
- `logs/isaac.log`, `logs/pp.log`, `logs/slam.log` - raw outputs
- Recording: `/root/bags/husky_real/exp16_vio_filter60_failed/`

## Next steps (if pursuing VIO)

1. Increase IMU buffer to 2000 entries (10s at 200Hz) in `run_husky_nav2.py`
2. Or write IMU to a file that's append-only, never truncated, so live
   binary can scan back to find readings for any camera timestamp
3. Or run offline `rgbd_inertial_offline` after the recording is complete
4. Or fundamentally - use IMU only for gyro yaw (current approach, works fine)
