# Week 2 - custom ICP odometry vs GT on NCLT

quick baseline of point-to-plane ICP on the NCLT 2012-04-29 session. open
loop odometry, no IMU, no loop closure - just stack consecutive scans and
see how fast it diverges from GT.

## setup

- session 2012-04-29, 12 971 scans available, processed every 10th = 500
- trajectory length 1336.9 m, +-1100 s of driving (+-18 min)
- Open3D point-to-plane ICP, 0.5 m voxel downsampling + normals
- runs at +-30 scans/s on RTX 5080, KISS-ICP install was broken at the time
  so it's only the custom path here

## results

| metric | custom ICP |
|---|---|
| mean ATE | 179.0 m |
| RMSE | 191.1 m |
| stddev | 66.8 m |
| min err | 56.9 m |
| max err | 362.4 m |

drift grows roughly linearly for the first +-600 s, has a brief recovery
around 700-800 s (down to +-60 m, probably a long straight where ICP had
clean planar geometry to lock onto), then catastrophic drift for the
final segment up to +-360 m. final number is 13 % drift over the
trajectory length, which is what you'd expect from naive open-loop ICP.

plots in `plots/`:

- `trajectory_comparison.png` - GT (black) vs custom ICP (blue) top-down
- `ate_over_time.png` - error vs time, drift evolution
- `error_distribution.png` - histogram, bimodal with peaks at 60-80 m
  and 220-240 m, long tail to 360 m

## why it's bad

basically the floor: no loop closure, no global optimization, no IMU
prior, fixed voxel size, single-pass ICP. open-loop odometry on 1.3 km
of bumpy off-road logging is going to drift unbounded no matter what the
registration does.

things that would obviously help: process every scan instead of every
tenth (better continuity), add a constant-velocity prior, add IMU
pre-integration, scan-to-map instead of scan-to-scan, eventually loop
closure + pose-graph optimisation. the typical "decent" LiDAR odometry
runs at 0.5-2 % drift, so 6-27 m on this trajectory; we're at 13 %
which is +-10x worse.

## next

- get KISS-ICP installed and rerun on the same 500 scans for an apples
  comparison
- process all 12 971 scans (not every 10th), see if continuity alone
  fixes the bigger spikes
- once ORB-SLAM3 / DPV-SLAM is wired in, fold these LiDAR numbers into
  the same comparison

bottom line - this run isn't competitive, it's a sanity floor. anything
i add later (loop closure, IMU, smaller voxels) should beat 179 m mean.

date: 2026-02-10. environment RTX 5080, Open3D 0.18, +-42 s of total
runtime including data load.
