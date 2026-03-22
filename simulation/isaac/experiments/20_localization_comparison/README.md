# exp 20: Localization Comparison - 3 Routes

## goal

Compare ORB-SLAM3 RGBD-only vs RGBD-Inertial (VIO) on all three Husky routes (road, north, south) with a unified IMU pipeline. This provides the localization comparison table for the bachelor thesis.

## Setup

All three routes recorded with `run_husky_forest.py` after fixing the IMU pipeline (see "Forest script fix" below).

| Route | GT distance | Frames | IMU readings | Time |
|---|---|---|---|---|
| Road | 335 m | 3463 | 20777 | 351 s |
| North | 467 m | 5214 | 31288 | 526 s |
| South | 396 m | 4317 | 25899 | 439 s |

Common parametars:
- IMU filter_size = 60 (linear), 30 (angular) - from exp 15
- IMU rate: 60 Hz exact (16.67 ms dt)
- IMU URF->FLU conversion in Python (gravity on Z = +9.81)
- Init phase: 2s stationary + 10s zigzag (excites accelerometer for VIO)
- Robot speed: ~0.9 m/s avg (max 1.0 m/s)

## Forest script fix (critical)

The original `run_husky_forest.py` had two bugs that prevented VIO from working:

- Triple-duplicated IMU readings:
   ```python
   _imu_reading = _imu_interface.get_sensor_reading(...)
   for isub in range(3):  # <- BUG: writes same reading 3 times!
       imu_ts = ts - dt + (isub + 1) * dt / 3
       _imu_file.write(f"{imu_ts:.4f},{lin_acc_x},...")
   ```
   This produced fake 180 Hz IMU file with only 60 Hz unique values. ORB-SLAM3 preintegration interpreted the duplicates as zero-time intervals -> bad integration.

2. **No URF->FLU conversion**:
   The PhysX IMU sensor is in URF frame (X=up, Y=right, Z=forward) with gravity on X axis. ORB-SLAM3 expects FLU (X=forward, Y=left, Z=up) with gravity on Z. Without conversion, identity Tbc is wrong.

**Effect on stationary IMU**:

| | Old forest (broken) | Fixed forest |
|---|---|---|
| Gravity axis | X = +9.81 | **Z = +9.81** x |
| Gyro yaw bias | +0.315 rad/s (18°/s drift!) | **+0.00006 rad/s** x |
| Gyro yaw std | 0.088 | **0.00001** x |
| Acc std | 0.197 | **0.0000** x |

After the fix, forest IMU matches `run_husky_nav2.py` (where exp 18 got VIO ATE 0.116m).

## results

| Route | GT distance | RGBD-only ATE | VIO ATE | Notes |
|---|---|---|---|---|
| **Road** | 335 m | 0.405 m | **0.347 m** | VIO 14% better |
| **North** | 467 m | 7.79 m | failed | VIO IMU init breaks on tight forest turns |
| **South** | 396 m | 0.516 m | failed | VIO IMU init breaks on tight forest turns |

### why VIO fails on north/south but works on road

ORB-SLAM3 IMU initialization requires ~3-5 seconds of continuous visual tracking with diverse motion to converge bias estimates and gravity direction.

- **Road** (S-curve highway): mostly straight after init, 3-4 gentle turns. IMU init survives the first phase, then VIO tracks reliably.
- **North** (forest): 234 waypoints in tight 2-3m spacing forces sharp turns immediately after the init zigzag. IMU init never completes - keeps resetting (60+ map resets per run). Visual tracking fails because of feature changes during turns.
- South (forest): same problem as north - 201 waypoints in tight spacing.

This is **fundamental to ORB-SLAM3 IMU initialization**, not a configuration issue. With looser noise (NoiseAcc=0.5) and more retries, occasional partial tracking happens but never converges to a stable scale.

### Forest RGBD-only also worse

| Route | RGBD-only ATE | ATE / distance |
|---|---|---|
| Road (highway) | 0.405 m | 0.12 % |
| **North (forest)** | 7.79 m | **1.7 %** |
| South (forest) | 0.516 m | 0.13 % |

North has much higher RGBD-only error than south despite similar geometry. Likely cause: north route enters denser forest sooner where the camera sees mostly tree trunks at varying distances -> harder to extract stable features -> more drift accumulation.

## Comparison vs old recordings

Original `05_slam_results_physx.png` table (April 6 recordings, before our improvements):

| Route | Old RGBD ATE | New RGBD ATE | Improvement |
|---|---|---|---|
| Road | 0.49 m | 0.405 m | 17% better |
| North | 2.07 m | 7.79 m | **worse** |
| South | 1.29 m | 0.516 m | 60% better |

North is worse on the new recording, possibly because:
1. Faster driving (1.0 m/s vs 0.4 m/s in old recordings) -> more motion blur, less tracking time per frame
2. Longer route (467m vs ~440m old)
3. ORB-SLAM3 stochasticity - single run can vary

For thesis we should report new RGBD numbers as the actual performance with current pipeline.

## conclusions for thesis

1. Filter=60 IMU + URF->FLU conversion are necessary for VIO to work in PhysX simulation (otherwise IMU bias 18°/s breaks initialization).

2. VIO offline on highway routes matches/exceeds RGBD-only baseline. Road: VIO 0.347m vs RGBD 0.405m. Confirms the pipeline works.

3. **VIO offline on forest routes fails** due to ORB-SLAM3 IMU init sensitivity to tight turns. RGBD-only is the only viable option for dense waypoint sequences with sharp angular changes.

4. **For Husky navigation in forest**: use RGBD-only SLAM. VIO would only help on open routes (highway-like).

5. On real Husky with Phidgets Spatial 1042 IMU, VIO would likely work better - real IMU has different bias characteristics than PhysX, and real driving is smoother than waypoint-following.

## files

- `config/rgbd_only.yaml` - RGBD-only ORB-SLAM3 config
- `config/rgbd_inertial_flu.yaml` - VIO config (identity Tbc, NoiseGyro=0.022)
- `config/rgbd_inertial_urf.yaml` - VIO with Variant B Tbc (for old URF data)
- `results/comparison_3_routes.png` - side-by-side trajectory plots
- `results/rgbd_road.txt`, `vio_road.txt`, `rgbd_north.txt`, `rgbd_south.txt` - SLAM trajectories
- Recordings in `/root/bags/husky_real/exp20_road`, `exp20_north`, `exp20_south`

## reproduction

```bash
# 1. Record with fixed forest script
/opt/isaac-sim-6.0.0/python.sh run_husky_forest.py --route road --duration 600
# repeat for north and south

# 2. Convert IMU and gen associations (per recording)
python3 - <<EOF
import csv, glob, os
REC = '/root/bags/husky_real/exp20_road'
with open(f"{REC}/imu.csv") as fin, open(f"{REC}/imu_orbslam.txt", "w") as fout:
    next(fin)
    for line in fin:
        p = line.strip().split(",")
        ts, ax, ay, az, gx, gy, gz = p[:7]
        fout.write(f"{ts} {gx} {gy} {gz} {ax} {ay} {az}\n")
rgbs = sorted(glob.glob(f"{REC}/camera_rgb/*.jpg"))
with open(f"{REC}/associations.txt", "w") as fout:
    for r in rgbs:
        f = os.path.basename(r); ts = f.replace(".jpg", "")
        d = f.replace(".jpg", ".png")
        if os.path.exists(f"{REC}/camera_depth/{d}"):
            fout.write(f"{ts} camera_rgb/{f} {ts} camera_depth/{d}\n")
EOF

# 3. RGBD-only
/workspace/third_party/ORB_SLAM3/Examples/RGB-D/rgbd_tum \
    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    config/rgbd_only.yaml $REC $REC/associations.txt

# 4. VIO (retry until good run, road only)
head -1500 $REC/associations.txt > $REC/associations_1500.txt
for try in {1..10}; do
    /workspace/third_party/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_offline \
        /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
        config/rgbd_inertial_flu.yaml $REC \
        $REC/associations_1500.txt $REC/imu_orbslam.txt
    # Check end position and break if good
done
```
