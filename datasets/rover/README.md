# ROVER pipeline - ORB-SLAM3 baseline

scripts + configs for evaluating ORB-SLAM3 on the **ROVER** dataset from
Esslingen University (Germany). HuggingFace link:
[iis-esslingen/ROVER](https://huggingface.co/datasets/iis-esslingen/ROVER).

plan: 15-ish recordings across 3 locations (garden_large, park, campus_large),
3 modes per recording (Stereo fisheye T265, Stereo-Inertial T265, RGB-D D435i).
first batch of results (garden_large, 8 recordings) is in, park + campus still
pending.

## Setup

Ubuntu 24.04, Python 3.10+, Xvfb for headless ORB-SLAM3 runs. pip deps:
`numpy matplotlib opencv-python evo`. ORB-SLAM3 must be built at
`../third_party/ORB_SLAM3/`.

Data download (all 15 recordings, ~335 GB) goes into `../data/rover/`.

## Pipeline

```bash
# 1. convert T265 fisheye stereo to EuRoC format
python3 scripts/convert_rover_to_euroc.py --recording garden_large_day

# 2. (optional) undistort T265 fisheye to pinhole for stereo mode
python3 scripts/rectify_t265_stereo.py --recording garden_large_day

# 3. convert D435i RGB-D to TUM format
python3 scripts/prepare_rover_rgbd.py --recording garden_large_day

# 4. run ORB-SLAM3 in all 3 modes on a single recording
python3 scripts/run_rover_orbslam3.py --recording garden_large_day

# or batch overnight (Xvfb, all recordings)
bash scripts/run_overnight.sh
```

Results are saved to `results/{recording}/{mode}/`.

## Sensor suite

Small overview, full calibration lives in the dataset's own docs.

### Intel RealSense T265 - stereo tracking camera
- 2x fisheye 848x800 @ 30 fps, KannalaBrandt8 distortion
- 6.35 cm baseline (short for outdoor stereo)
- built-in IMU @ 264 Hz. gyro noise 0.00324 rad/s/sqrt(Hz), accel 0.01741 m/s^2/sqrt(Hz)
- left cam: fx=286.18, fy=286.39, cx=416.94, cy=403.27

### Intel RealSense D435i - RGB-D
- 640x480 RGB + depth @ 30 fps, PinHole + radtan
- depth is IR structured light, keeps working in darkness
- fx=596.20, fy=593.14, cx=327.05, cy=245.16
- depth factor 1000 (values in mm)

### Leica Total Station - ground truth
- prism on the robot tracked by a total station. sub-cm accuracy
- 1.6-3.9 Hz depending on line-of-sight

## Locations

| Location | Recordings | Route | Conditions |
|----------|-----------|-------|------------|
| garden_large | 8 | ~13x20 m, loops around vegetation + walls | summer/autumn/winter/spring/day/dusk/night/night-light |
| park | 7 | ~20x19 m, open paths between trees | similar conditions, no `winter` |
| campus_large | several | ~120x80 m | various lighting/weather |

garden_large is compact and texture-rich (walls, bushes, curbs) so it's the
easy starting point. park is harder (repetitive grass, distant landmarks).
campus_large is the biggest and will need loop closure to stay accurate.

## Status

- **garden_large (8 recordings)**: all 3 modes run, results in `results/garden_large_*/`
- **park**: not run yet, configs ready
- **campus_large**: not run yet
- **fisheye-to-pinhole fix** for stereo modes: separate experiment (1.1b), pending

Detailed results writeup will go into `CHANGELOG.md` and
`EXPERIMENTS_ROVER.md` as the full matrix finishes.

## References

- **ROVER dataset** - Esslingen University - [download](https://huggingface.co/datasets/iis-esslingen/ROVER)
- **ORB-SLAM3** - Campos et al., 2021 - [paper](https://arxiv.org/abs/2007.11898) - [code](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **Intel RealSense D435i** - [product page](https://www.intelrealsense.com/depth-camera-d435i/)
- **Intel RealSense T265** - [product page](https://www.intelrealsense.com/tracking-camera-t265/)
- **evo** - trajectory evaluation - [code](https://github.com/MichaelGrupp/evo)
