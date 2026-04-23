# Visual-Inertial SLAM and Navigation for outdoor UGV

bachelor thesis.  goal is visual-inertial point-to-point navigation with
obstacle avoidance for an autonomous ground robot (Clearpath Husky A200 style)
operating in outdoor environments, using an RGB-D camera and an IMU

the project has two halves. first half is benchmarking existing visual SLAM
methods on four public outdoor datasets - to figure out what works, where it
breaks, and what the real robot pipeline needs to look like.  second half is
building that pipeline in Isaac Sim and running a teach-and-repeat campaign
across 9 routes, with stock-Nav2 and RGB-D-only ablations for comparison

![all 9 T&R routes on the simulation scene](simulation/isaac/results/final/18_scene_obstacles_routes.png)

## Results summary (headline only)

in chronological order (this is how the project actually went):

| Pipeline | best result | where to read more |
|---|---|---|
| [**NCLT**](datasets/nclt/) | LiDAR ICP + GPS LC - 30.2 m winter, 151-188 m other seasons | [`datasets/nclt/CHANGELOG.md`](datasets/nclt/CHANGELOG.md), [`reports/orbslam3_nclt_report.md`](datasets/nclt/reports/orbslam3_nclt_report.md) |
| [**NCLT Kaggle**](datasets/nclt_kaggle/) | MinkLoc3D scaffold, training pending | [`README.md`](datasets/nclt_kaggle/README.md), [`PROJECT_CONTEXT.md`](datasets/nclt_kaggle/PROJECT_CONTEXT.md) |
| [**RobotCar**](datasets/robotcar/) | ORB-SLAM3 Stereo - 3.91 m ATE RMSE, 72.7% tracking | [`CHANGELOG.md`](datasets/robotcar/CHANGELOG.md), [`EXPERIMENTS_ROBOTCAR.md`](datasets/robotcar/EXPERIMENTS_ROBOTCAR.md) |
| [**4Seasons**](datasets/4seasons/) | ORB-SLAM3 Stereo-Inertial - **0.93 m** ATE RMSE, 99.99% tracking | [`CHANGELOG.md`](datasets/4seasons/CHANGELOG.md), [`EXPERIMENTS_4SEASONS.md`](datasets/4seasons/EXPERIMENTS_4SEASONS.md) |
| [**ROVER**](datasets/rover/) | ORB-SLAM3 RGB-D - **0.37 m** best (GL autumn), 11/15 success.  closest sensor match to our Husky | [`CHANGELOG.md`](datasets/rover/CHANGELOG.md), [`EXPERIMENTS_ROVER.md`](datasets/rover/EXPERIMENTS_ROVER.md), [`REPORT_experiment_1.1.md`](datasets/rover/REPORT_experiment_1.1.md) |
| [**Gazebo sim**](simulation/gazebo/) | RTAB-Map RGB-D 9.23 m on forest, ORB-SLAM3 failed | [`experiments/`](simulation/gazebo/experiments/) |
| [**Isaac sim T&R**](simulation/isaac/) | our pipeline **100%** on route 09 vs 63% stock Nav2 | [`results/final/README.md`](simulation/isaac/results/final/README.md), [`routes/README.md`](simulation/isaac/routes/README.md) |

the 100 % headline is a single route (09 SE-NE, 36/36 WPs), not the whole
campaign.  cross-route numbers are in [`simulation/isaac/routes/README.md`](simulation/isaac/routes/README.md)

## Repository layout

```
datasets/
  nclt/           LiDAR ICP + ORB-SLAM3 + DROID-SLAM on NCLT campus dataset
  nclt_kaggle/    MinkLoc3D place recognition scaffold (Kaggle-hosted NCLT subset)
  robotcar/       hloc + ORB-SLAM3 Stereo on Oxford RobotCar
  4seasons/       ORB-SLAM3 Stereo-Inertial on TU Munich 4Seasons
  rover/          ORB-SLAM3 on ROVER UGV (RGB-D + stereo fisheye + SI) - last + closest to our Husky
simulation/
  gazebo/         Gazebo Harmonic + Nav2 baselines
  isaac/          Isaac Sim T&R campaign - 9 routes, 3 methods, full pipeline
```

each subdir is a self-contained pipeline with its own README, CHANGELOG / writeup,
configs, scripts, results.  see the pipeline READMEs above for the full story

## Where to read next

depending on what you want to know

- **just tell me what works on my UGV**: [`simulation/isaac/routes/README.md`](simulation/isaac/routes/README.md)
  is the main campaign + 9-route table.  the pipeline scripts live in
  [`simulation/isaac/scripts/common/`](simulation/isaac/scripts/common/) and
  [`simulation/isaac/scripts/nav_our_custom/`](simulation/isaac/scripts/nav_our_custom/)

- **thesis defence / full argument**: start with [`docs/thesis_reading_order.md`](docs/thesis_reading_order.md)
  which walks through the readmes in the order the thesis tells its story

- **reproducing a result**: each pipeline README has a `How to run` block
  with exact commands.  raw data needs to be downloaded separately (see each
  dataset's setup section)

- **per-experiment details**: Isaac has 79 experiments in
  [`simulation/isaac/experiments/`](simulation/isaac/experiments/), indexed in
  [`docs/experiment_index.md`](docs/experiment_index.md) with 1-line summaries

- **code-only look**: [`simulation/isaac/scripts/common/tf_wall_clock_relay_v55.py`](simulation/isaac/scripts/common/tf_wall_clock_relay_v55.py)
  and [`visual_landmark_matcher.py`](simulation/isaac/scripts/common/visual_landmark_matcher.py)
  are the core anchor-fusion algorithm.  they have changelogs in the
  docstrings covering exps 51-64

## Setup

- Ubuntu 24.04, Python 3.10+
- NVIDIA GPU + CUDA 12.x for hloc / deep SLAM / Isaac Sim
- ORB-SLAM3 built from source in `third_party/ORB_SLAM3/` (clone separately,
  not tracked here - see each dataset's setup for the commit used)
- Isaac Sim 6.0.0 via pip (Python 3.12 required for this version)
- ROS 2 Jazzy + Gazebo Harmonic for the Gazebo side

per-dataset dependencies live in each pipeline's README

## Thesis story at a glance

the reason the project started with datasets and ended with Isaac Sim is
that I didn't trust the Husky simulator at first.  the datasets gave
calibrated ground truth and let me see what methods do on real fisheye
cameras and real IMU noise.  order went:

1. **NCLT** (LiDAR + fisheye) - visual SLAM fails on 5 Hz fisheye + 48 Hz IMU,
   LiDAR ICP is the only thing that works
2. **NCLT Kaggle** - side project, MinkLoc3D place recognition scaffold for
   later loop-closure work
3. **RobotCar** - first working visual SLAM (Stereo 3.91 m ATE).  but
   Stereo-Inertial impossible because no raw IMU is published
4. **4Seasons** - proves IMU matters.  same class of stereo + a real 2000 Hz
   IMU, ATE drops to 0.93 m
5. **ROVER** - last dataset, closest match to our real Husky (D435i + T265 on
   a ground robot).  RGB-D wins at 0.37 m, stereo fisheye fails without
   undistortion

once D435i RGB-D + real IMU was clearly the winning combo, the sim story
became clear: build Isaac Sim, drive Husky with D435i + phidgets-class IMU,
and prove a teach-and-repeat pipeline can beat stock Nav2.  the 9-route
campaign is that proof

## References

### SLAM and odometry

- **ORB-SLAM3** - Campos et al., 2021, IEEE TRO - [paper](https://arxiv.org/abs/2007.11898) - [code](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **DROID-SLAM / DPVO / DPV-SLAM** - Teed et al., 2021-2023 - [DROID](https://github.com/princeton-vl/DROID-SLAM) - [DPVO](https://github.com/princeton-vl/DPVO)
- **RTAB-Map** - Labbe & Michaud, 2019, IJRR - [code](https://github.com/introlab/rtabmap)
- **KISS-ICP** - Vizzo et al., 2023, RAL - [paper](https://arxiv.org/abs/2209.15397) - [code](https://github.com/PRBonn/kiss-icp)

### Visual localization / place recognition

- **hloc** - Sarlin et al., 2019, CVPR - [code](https://github.com/cvg/Hierarchical-Localization)
- **SuperPoint / SuperGlue / LightGlue** - DeTone 2018 / Sarlin 2020 / Lindenberger 2023
- **ALIKED** - Zhao et al., 2023 - [paper](https://arxiv.org/abs/2304.03608)
- **MinkLoc3D** - Komorowski, 2021, WACV - [paper](https://arxiv.org/abs/2011.04530) - [code](https://github.com/jac99/MinkLoc3D)

### Datasets

- **NCLT** - Carlevaris-Bianco et al., 2016, IJRR - [website](https://robots.engin.umich.edu/nclt/)
- **ROVER** - Ligocki et al., 2024 - [HuggingFace](https://huggingface.co/datasets/iis-esslingen/ROVER)
- **Oxford RobotCar / Seasons** - Maddern 2017 / Sattler 2018 - [website](https://robotcar-dataset.robots.ox.ac.uk)
- **4Seasons** - Wenzel et al., 2020, DAGM GCPR - [website](https://www.4seasons-dataset.com)

### Nav stack + tooling

- **Nav2** - Macenski et al., 2020 - [code](https://github.com/ros-navigation/navigation2)
- **ROS 2 Jazzy** - [docs](https://docs.ros.org/en/jazzy/)
- **Isaac Sim 6.0** - NVIDIA - [docs](https://docs.isaacsim.omniverse.nvidia.com/)
- **Gazebo Harmonic** - [website](https://gazebosim.org/)
- **evo** / **Open3D** / **OpenCV** / **COLMAP** - [evo](https://github.com/MichaelGrupp/evo), [Open3D](https://github.com/isl-org/Open3D), [OpenCV](https://opencv.org/), [COLMAP](https://github.com/colmap/colmap)

### Hardware

- **Clearpath Husky A200** - [product page](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
- **Intel RealSense D435i / T265** - [D435i](https://www.intelrealsense.com/depth-camera-d435i/) / [T265](https://www.intelrealsense.com/tracking-camera-t265/)
- **Phidgets Spatial 1042** IMU - [product page](https://www.phidgets.com/?prodid=32)
