# 4Seasons - experiment log

## Experiment 0.9 - ORB-SLAM3 Stereo-Inertial on 4Seasons office_loop_1

date: 2026-02-17
dataset: 4Seasons office_loop_1 (spring recording_2020-03-24). ~3.7 km route,
  501 s, 15177 stereo pairs @ 30 Hz, ADIS16465 IMU @ 2000 Hz, Septentrio RTK GT

converted 4Seasons -> EuRoC MAV layout using the custom converter, then ran
ORB-SLAM3 Stereo-Inertial. IMU calibration lifted from the dataset's
calib_imu_stereo.yaml. this is the first experiment with a real high-rate IMU,
so it's the first run where SI actually does what it's supposed to

results:

| metric | value |
|--------|-------|
| keyframes | 3536 |
| matched GT pairs | 3127 |
| Sim(3) scale | 0.9967 |
| ATE RMSE (Sim3) | **0.93 m** |
| ATE mean | 0.82 m |
| ATE median | 0.75 m |
| ATE max | 3.06 m |
| RPE RMSE | 0.40 m |

best result in the whole project. scale 0.997 is near-perfect, which is what
a real IMU + stereo is supposed to give. compared to RobotCar Stereo (3.91 m,
no IMU), this is 4x better ATE - mostly the IMU, plus 30 cm baseline vs 24
cm, plus 30 Hz vs 16 Hz

files: `configs/4Seasons_Stereo_Inertial.yaml`, `scripts/convert_4seasons_to_euroc.py`,
`scripts/evaluate_4seasons.py`, `scripts/run_4seasons_experiment.sh`,
`results/4seasons/`

## Experiment 1.0 - hloc self-test on 4Seasons

date: 2026-02-17
method: hloc with SuperPoint + SuperGlue + NetVLAD (and OpenIBL variant).
query = reference (self-localisation), not cross-condition

SfM model built from 1707 subsampled reference frames (every 2 m) -> 61688
triangulated 3D points. then 7588 non-reference frames localised against this
map

| metric | SP+SG+NetVLAD | SP+SG+OpenIBL |
|--------|---------------|---------------|
| localised | 7588 (100%) | 7588 (100%) |
| median trans err | 3.76 m | 3.79 m |
| median rot err | 0.98 deg | 0.97 deg |
| 0.25m/2 deg | 12.4% | 12.3% |
| 0.5m/5 deg | 17.1% | 17.2% |
| 5m/10 deg | 55.0% | 55.0% |

modest accuracy because the SfM model is sparse (only 1707 of 15177 frames as
references). queries with 200+ PnP inliers hit 0.59 m median, queries with
fewer than 10 inliers are at 163 m median. that's the gap

on same-season data NetVLAD and OpenIBL perform identically - OpenIBL's
night-time advantage only matters under condition changes

files: `scripts/run_4seasons_hloc.py`, `results/4seasons_hloc/`
