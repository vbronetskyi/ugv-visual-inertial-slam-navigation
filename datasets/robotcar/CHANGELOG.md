# RobotCar / RobotCar Seasons - experiment log

## Experiment 0.8 - ORB-SLAM3 Stereo on RobotCar

date: 2026-02-15
dataset: Oxford RobotCar, session 2014-11-28-12-07-13 (overcast-reference),
  Bumblebee XB3 stereo 1280x960 @ 16 Hz, 24 cm baseline. 6001 stereo pairs
  over 420.6 s.

ORB-SLAM3 Stereo on the full drive. no IMU: RobotCar only exposes an INS
navigation stream, not raw accel/gyro. a Stereo-Inertial attempt using a
pseudo-IMU (differentiated INS) was also run and failed - the synthetic IMU
is too smooth for ORB-SLAM3's tightly-coupled VI init. see
EXPERIMENTS_ROBOTCAR for that walkthrough

results:

| metric | value |
|--------|-------|
| tracked frames | 4365 / 6001 (72.7%) |
| sub-maps | 4 (fragmentation from tracking losses) |
| GT path length | 834.1 m |
| Sim(3) scale | 0.9658 |
| ATE RMSE (Sim3) | **3.91 m** |
| ATE max | 6.75 m |
| RPE trans/frame | 0.435 m |
| RPE rot/frame | 0.555 deg |

first successful visual SLAM in the project - the trajectory actually follows
the road shape. 72.7% coverage (lost the last ~27% to tracking failures).
scale 0.97 is sensible for stereo. Sim(3) alignment used only because tracking
drops lead to unconnected sub-maps; pure SE(3) eval is worse

files: `configs/RobotCar_Stereo.yaml`, `scripts/evaluate_robotcar_orbslam3.py`,
`scripts/prepare_stereo_euroc.py`, `scripts/make_ground_truth.py`,
`results/robotcar_orbslam3/`

## Experiment 0.7 - hloc on RobotCar Seasons

date: 2026-02-14
dataset: RobotCar Seasons v2 (Grasshopper 1024x1024 undistorted to pinhole
  fx=fy=400). 1906 training images with known poses used as the query split

7 hloc configs swept:

| method | 0.25m/2 deg | 0.5m/5 deg | 5m/10 deg | day 0.5m | night 0.5m | med trans | med rot |
|--------|-------------|------------|-----------|----------|------------|-----------|---------|
| SP+SG | 42.9% | 62.4% | 83.8% | 71.5% | 30.4% | 0.301 m | 0.782 deg |
| SP+LG | 42.4% | 62.0% | 84.1% | 71.2% | 29.4% | 0.301 m | 0.787 deg |
| DISK+LG | 42.5% | 62.4% | 85.0% | 71.5% | 30.6% | 0.300 m | 0.786 deg |
| ALIKED+LG | 42.9% | 62.3% | 86.0% | 71.2% | 30.9% | 0.297 m | 0.783 deg |
| SP+SG+OI | 43.6% | 64.2% | 86.8% | 71.2% | 39.4% | 0.296 m | 0.761 deg |
| SIFT+NN | 38.6% | 56.6% | 75.9% | 69.6% | 10.7% | 0.367 m | 0.872 deg |
| **ALIKED+LG+OI** | **44.6%** | **64.5%** | **89.6%** | 71.2% | **40.6%** | **0.288 m** | **0.756 deg** |

best: ALIKED + LightGlue + OpenIBL. 64.5% at (0.5m/5 deg), 40.6% at night.
- learned detectors beat SIFT by 6-8 pp overall
- OpenIBL gives +10% over NetVLAD at night (40.6% vs 30.9%)
- LightGlue ~= SuperGlue quality, runs faster
- night is the biggest gap - 30-40% vs 70%+ during the day

files: `scripts/run_full_benchmark.py`, `scripts/run_fixes_and_new.py`,
`results/robotcar_seasons_hloc/`
