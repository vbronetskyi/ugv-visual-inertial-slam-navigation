# Project Context: NCLT SLAM Pipeline

## Project Overview

Building a 3D reconstruction and SLAM navigation pipeline for an outdoor UGV (Unmanned Ground Vehicle). The system combines LiDAR-based odometry with learned place recognition for loop closure, targeting deployment on edge hardware (Jetson Orin Nano / RPI5 + Hailo).

## Research Questions

- **RQ1**: Can a lightweight MinkLoc3D-based place recognition model achieve Recall@1 > 0.9 on NCLT with < 50ms inference on Jetson?
- **RQ2**: How much does loop closure from learned descriptors reduce ATE compared to odometry-only SLAM?
- **RQ3**: What voxel resolution works best in terms of the accuracy/latency trade-off for real-time operation?
- **RQ4**: Can INT8 quantization preserve >95% of FP32 recall while achieving 2x speedup on Hailo-8?
- **RQ5**: How does seasonal/weather variation accross NCLT sessions affect place recognition robustness?

## Dataset: NCLT (University of Michigan)

- **Source**: https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23
- **Original**: http://robots.engin.umich.edu/nclt/
- **Content**: 10 sessions (Jan–Dec 2012) of outdoor campus traversals
- **Sensors**: Velodyne HDL-32E LiDAR, Ladybug3 omnidirectional camera, IMU, GPS
- **Ground truth**: RTK GPS + IMU fused poses in `track.csv`
- **Kaggle variant**: Preprocessed with `train.csv`, `val.csv`, `test.csv` pair annotations

### Data Structure (Kaggle)
```
NCLT_preprocessed/
├── train.csv          # (anchor_idx, positive_idx, session_a, session_b)
├── val.csv
├── test.csv
└── sessions/
    └── YYYY-MM-DD/
        ├── velodyne/          # binary .bin point clouds (N x 4 float32: x,y,z,intensity)
        ├── images_small/      # downscaled camera images
        └── track.csv          # timestamp, x, y, z, roll, pitch, yaw
```

## Current Focus: Week 1, Setup & Exploration

- [x] Project structure and boilerplate
- [ ] Data loading and verification
- [ ] Point cloud visualization
- [ ] Baseline ICP odometry on single session
- [ ] Place recognition pair analysis

## Architecture Decisions

1. **Place Recognition Model**: MinkLoc3D (sparse 3D convolutions via MinkowskiEngine)
   - Reason: top-performing on outdoor benchmarks, supports sparse voxel input
2. **SLAM Backend**: Simple pose graph with Open3D ICP frontend
   - Reason: interpretable, easy to integrate loop closures, good baseline
3. **Edge Deployment**: ONNX export -> TensorRT (Jetson) or Hailo SDK
   - Reason: standard deployment path for both target platforms
4. **Training Platform**: Kaggle (free GPU) with experiment configs in YAML
   - Reason: no local GPU required, reproducible via notebook

## Code Conventions

- **Python**: 3.10+ (match statements OK, union type `X | Y` syntax OK)
- **Type hints**: Required on all public functions
- **Docstrings**: Google style
- **Formatting**: Black (default settings)
- **Imports**: stdlib -> third-party -> local, separated by blank lines
- **Logging**: Use `logging` module, not print statements
- **Config**: YAML files loaded via PyYAML, no hardcoded paths
- **Testing**: pytest, fixtures for dataset access

## Key File Locations

| Purpose | Path |
|---|---|
| Dataset config | `configs/dataset_config.yaml` |
| Training config | `configs/train_config.yaml` |
| SLAM config | `configs/slam_config.yaml` |
| Dataset loader | `src/datasets/nclt_dataset.py` |
| Pair dataset | `src/datasets/nclt_pairs.py` |
| Point cloud utils | `src/utils/point_cloud.py` |
| SLAM metrics | `src/evaluation/metrics.py` |
| Kaggle notebook | `notebooks/02_kaggle_training.ipynb` |

## Dependencies

Core: PyTorch, Open3D, MinkowskiEngine, NumPy, pandas
See `requirements.txt` for full list.
