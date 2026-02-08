# Week 1 notes: NCLT data pipeline

Quick rundown of what was built in week 1. The idea was to get all NCLT data
types loadable from Python before touching any SLAM algorithms.

## What got built

Data loaders in `src/data_loaders/`:

- `velodyne_loader.py` - reads the 7-byte-per-point binary format for the
  HDL-32E (`.bin` files in `velodyne_sync/`). Also reads the packet stream
  `velodyne_hits.bin` if needed.
- `sensor_loader.py` - MS25 IMU, GPS, RTK GPS, 10Hz and 100Hz odometry,
  KVH gyro, wheel speeds. All CSV.
- `ground_truth_loader.py` - pose CSV loader. qw isn't stored, it has to be
  recovered from the unit-quaternion constraint.
- `hokuyo_loader.py` - 30m and 4m hokuyo binary scans. Same scaling as velodyne.

Calibration framework in `src/calibration/` - body-to-lidar + placeholder for
camera extrinsics + intrinsics. Actual files are on the NCLT website.

Visualisation in `src/visualization/` - open3d point cloud viewer with colouring
modes (height, intensity, laser id, distance), 2D hokuyo plots, trajectory
viewer, and a projection-onto-image helper for Ladybug3.

## Binary format notes

Velodyne `.bin` synced scans: 7 bytes per point.
  - x, y, z as uint16, intensity and laser_id as uint8
  - `metric = raw * 0.005 - 100.0`
  - typical scan is ~39k points.

Hokuyo: 8 bytes timestamp + N ranges (1081 for 30m, 726 for 4m), same scaling.

Ground truth CSV: utime, x, y, z, qx, qy, qz (no qw). Around 100 Hz.

All timestamps are microseconds since epoch.

## Sizes for 2012-01-08

| Type | Count | Size |
|---|---|---|
| Velodyne scans | 28,127 | 17 GB |
| Ground truth | 835,468 poses | 111 MB |
| MS25 IMU | 270k samples | 48 MB |
| Odometry 100Hz | 612k | 66 MB |
| GPS | 46k | 3.3 MB |
| Hokuyo 30m | ~154k scans | 464 MB |
| Hokuyo 4m | ~211k scans | 79 MB |

Trajectory length from ground truth: ~6.5 km, avg speed 1.16 m/s.

## Gotchas

- First row of `groundtruth_*.csv` has NaN, have to skip it.
- Unit-quaternion recovery for qw: `qw = sqrt(1 - qx^2 - qy^2 - qz^2)`.
  Watch out for numerical issues when the sum is slightly >1.
- open3d hangs if you pass it an empty cloud. Check `len(points) > 0`.

## Next

Need ICP (point-to-point + point-to-plane), NDT, then chain sequential scans
into an odometry estimate and compare with GT. After that, loop closure + pose
graph.

## Refs

- NCLT site: https://robots.engin.umich.edu/nclt/
- NCLT paper: https://robots.engin.umich.edu/nclt/nclt.pdf
- Format tools: https://github.com/aljosaosep/NCLT-dataset-tools
