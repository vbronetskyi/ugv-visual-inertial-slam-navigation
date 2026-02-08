# 4Seasons pipeline

evaluation of ORB-SLAM3 Stereo-Inertial on the 4Seasons dataset from TU Munich.
first dataset in this project with a real high-rate IMU, so it's where
visual-inertial SLAM actually makes sense.

## Dataset

TU Munich 4Seasons has stereo imagery + a real 2000 Hz IMU for SLAM under
seasonal variation. same "office loop" route driven in spring, summer and
winter.

| Sensor | Model | Parameters |
|--------|-------|------------|
| Stereo | custom module | 2x 800x400, 30 Hz, global shutter, 30 cm baseline |
| IMU | Analog Devices ADIS16465 | 6-axis, 2000 Hz |
| GNSS | Septentrio RTK | ground truth with RTK corrections |

Primary session: **office_loop_1** (recording_2020-03-24, spring, sunny),
~3.7 km route, 501 s, 15177 stereo pairs, ~1M IMU samples, 4037 RTK GT poses.

## Why 4Seasons after RobotCar

Oxford RobotCar only exposes a fused INS navigation stream, not raw IMU.
synthesizing pseudo-IMU by differentiating INS doesn't work for ORB-SLAM3's
tightly-coupled VIO (tried, doesn't converge). 4Seasons gives raw 2000 Hz
IMU so Stereo-Inertial mode works as designed.

## Experiments ran here

- **0.9**: ORB-SLAM3 Stereo-Inertial on office_loop_1. **0.93 m ATE RMSE**,
  scale 0.997, tracking 99.99%. best result in the whole project
- **1.0**: hloc self-test (query = reference, same session). 17.1% at
  0.5m/5 deg with the SP+SG+NetVLAD SfM. limited by sparse ref sampling (only
  1707 of 15177 frames triangulated)

full writeup in [EXPERIMENTS_4SEASONS.md](EXPERIMENTS_4SEASONS.md).

## Pipeline

```bash
# download office_loop_1 (~19 GB) from 4seasons-dataset.com manually
# then convert to EuRoC format
python3 scripts/convert_4seasons_to_euroc.py --sequence office_loop_1

# run ORB-SLAM3 Stereo-Inertial + eval
bash scripts/run_4seasons_experiment.sh
python3 scripts/evaluate_4seasons.py

# hloc self-test on 4Seasons
python3 scripts/run_4seasons_hloc.py
```

## Limitations

- only one session (office_loop_1) processed so far. spring + summer + winter
  loops available in the dataset for cross-season eval later
- the hloc self-test has low accuracy not because hloc is bad, but because
  the reference SfM is sparse. denser references would help

## References

- **4Seasons** - Wenzel et al., 2020, DAGM GCPR - [paper](https://arxiv.org/abs/2009.06364) - [website](https://www.4seasons-dataset.com)
- **ORB-SLAM3** - Campos et al., 2021, IEEE TRO - [paper](https://arxiv.org/abs/2007.11898) - [code](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **ADIS16465 IMU** - [datasheet](https://www.analog.com/en/products/adis16465.html)
- **hloc** - Sarlin et al., 2019, CVPR - [code](https://github.com/cvg/Hierarchical-Localization)
