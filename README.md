# Visual-Inertial SLAM for outdoor UGV

bachelor thesis. goal is visual-inertial point-to-point navigation with obstacle
avoidance for an autonomous ground robot (Clearpath Husky A200 style) operating
in outdoor environments, using an RGB-D camera and an IMU.

current phase is benchmarking existing visual SLAM and odometry methods on public
outdoor datasets, to understand what works and where things break before
building a navigation stack on top. simulation of the Husky and the navigation
pipeline will get added here later.

## What's here

```
datasets/
  nclt/           LiDAR ICP + ORB-SLAM3 + DROID-SLAM on NCLT (Segway + Velodyne + Ladybug3)
  nclt_kaggle/    MinkLoc3D place recognition on Kaggle-hosted NCLT subset
  rover/          ORB-SLAM3 (RGB-D, stereo, stereo-inertial) on the ROVER UGV dataset
  robotcar/       hloc + ORB-SLAM3 Stereo on Oxford RobotCar (scaffold; experiments incoming)
  4seasons/       ORB-SLAM3 Stereo-Inertial on TU Munich 4Seasons (scaffold; experiments incoming)
```

each dataset subdir is a self-contained pipeline with its own README, configs,
scripts and (as results arrive) a CHANGELOG / EXPERIMENTS writeup.

## Why these five

- **NCLT** - long-term 4-season outdoor dataset with a Segway + Velodyne HDL-32E,
  Ladybug3 spherical camera, MS25 IMU, RTK GPS. good for LiDAR odometry baselines
  and for seeing how much seasonal change breaks visual SLAM
- **NCLT (Kaggle subset)** - pre-split hackathon version of NCLT used for training
  a compact 3D place-recognition descriptor (MinkLoc3D) for later loop closure
- **ROVER** - small ground UGV with Intel RealSense D435i + T265, closest public
  dataset to the Husky-style outdoor robot i'm targeting, with day/night/season
  variants
- **Oxford RobotCar** - 10 km urban route across 100+ sessions, used for the
  RobotCar Seasons visual localization benchmark (hloc) and for ORB-SLAM3 Stereo
  on the raw stereo stream
- **4Seasons** - TU Munich stereo + real 2000 Hz IMU. first dataset in this
  project where ORB-SLAM3 Stereo-Inertial has all the inputs it actually needs

## Setup

- Ubuntu 24.04, Python 3.10+
- NVIDIA GPU + CUDA 12.x (for hloc / deep SLAM methods)
- ORB-SLAM3 built from source in `third_party/ORB_SLAM3/` (clone separately, not
  tracked here)
- per-dataset dependencies live in each pipeline's README

## References

### SLAM and odometry

- **ORB-SLAM3** - Campos et al., 2021, IEEE TRO - [paper](https://arxiv.org/abs/2007.11898) - [code](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **DROID-SLAM** - Teed & Deng, 2021, NeurIPS - [paper](https://arxiv.org/abs/2108.10869) - [code](https://github.com/princeton-vl/DROID-SLAM)
- **DPVO / DPV-SLAM** - Teed et al., 2023 - [code](https://github.com/princeton-vl/DPVO)
- **KISS-ICP** - Vizzo et al., 2023, RAL - [paper](https://arxiv.org/abs/2209.15397) - [code](https://github.com/PRBonn/kiss-icp)

### Visual localization

- **hloc** - Sarlin et al., 2019, CVPR - [code](https://github.com/cvg/Hierarchical-Localization)
- **SuperPoint** - DeTone et al., 2018 - [paper](https://arxiv.org/abs/1712.07629)
- **SuperGlue / LightGlue** - Sarlin et al. 2020 / Lindenberger et al. 2023 - [LG code](https://github.com/cvg/LightGlue)
- **ALIKED** - Zhao et al., 2023 - [paper](https://arxiv.org/abs/2304.03608)
- **NetVLAD / OpenIBL** - global descriptors for place-based retrieval

### Place recognition

- **MinkLoc3D** - Komorowski, 2021, WACV - [paper](https://arxiv.org/abs/2011.04530) - [code](https://github.com/jac99/MinkLoc3D)
- **MinkowskiEngine** - Choy et al., 2019 - [code](https://github.com/NVIDIA/MinkowskiEngine)

### Datasets

- **NCLT** - Carlevaris-Bianco et al., 2016, IJRR - [paper](https://doi.org/10.1177/0278364915614638) - [website](https://robots.engin.umich.edu/nclt/)
- **ROVER** - Ligocki et al., 2024 - [paper](https://arxiv.org/abs/2412.02506) - [download](https://huggingface.co/datasets/iis-esslingen/ROVER)
- **Oxford RobotCar** - Maddern et al., 2017, IJRR - [paper](https://doi.org/10.1177/0278364916679498) - [website](https://robotcar-dataset.robots.ox.ac.uk)
- **RobotCar Seasons** (benchmark) - Sattler et al., 2018, CVPR - [paper](https://arxiv.org/abs/1707.09092) - [website](https://www.visuallocalization.net)
- **4Seasons** - Wenzel et al., 2020, DAGM GCPR - [paper](https://arxiv.org/abs/2009.06364) - [website](https://www.4seasons-dataset.com)

### Tools

- **evo** - trajectory evaluation - [code](https://github.com/MichaelGrupp/evo)
- **Open3D** - Zhou et al., 2018 - [code](https://github.com/isl-org/Open3D)
- **OpenCV** - [website](https://opencv.org/)
- **COLMAP** - Schönberger & Frahm, 2016 - [code](https://github.com/colmap/colmap)
