# NCLT Kaggle Pipeline - Place Recognition with MinkLoc3D

## Overview

This pipeline implements LiDAR-based place recognition on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) using a MinkLoc3D model (sparse 3D convolutions via MinkowskiEngine). The goal is to train a lightweight 3D descriptor for loop closure detection in a SLAM system, with edge deployment targeting Jetson Orin Nano.

Training is done on Kaggle GPU using the preprocessed [NCLT hackathon dataset](https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23) (10 sessions, Jan--Dec 2012). A seperate [sensors addon dataset](https://www.kaggle.com/datasets/) provides IMU, GPS, odometry, and ground truth for multi-sensor fusion experiments.

---

## Research Questions

- **RQ1**: Can MinkLoc3D achieve Recall@1 > 0.9 on NCLT with < 50ms inference on Jetson Orin Nano?
- **RQ2**: How much does learned loop closure reduce ATE compared to odometry-only SLAM?
- **RQ3**: What voxel resolution works best in terms of the accuracy/latency trade-off for real-time operation?
- **RQ4**: Can INT8 quantization preserve >95% of FP32 recall while achieving 2x speedup on Hailo-8?
- **RQ5**: How does seasonal/weather variation across NCLT sessions affect place recognition robustness?

---

## Setup and Requirements

**System:** Ubuntu 24.04, Python 3.10+

**Dependencies:**
```bash
pip install -r requirements.txt
```

Core packages: PyTorch, Open3D, MinkowskiEngine, NumPy, pandas, scipy, scikit-learn. See `requirements.txt` for the full list.

**Dataset:** download the preprocessed NCLT dataset from [Kaggle](https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23) into `data/NCLT_preprocessed/`, or use the download script:

```bash
# install kaggle API, place kaggle.json in ~/.kaggle/
pip install kaggle
python scripts/download_nclt_sample.py --source kaggle
```

When running on Kaggle, data is automatically available at `/kaggle/input/nclt-iprofi-hack-23/NCLT_preprocessed/`. The notebook symlinks it transparently.

**Sensors addon** (optional, for IMU/GPS/odometry experiments): attach the `nclt-sensors-addon` dataset on Kaggle or download locally to `data/nclt_sensors_addon/`. The addon packages the MS25 IMU (`ms25.csv`, 3-axis accel + gyro + mag), consumer GPS (`gps.csv`), RTK GPS (`gps_rtk.csv`), KVH fiber-optic gyro heading (`kvh.csv`), wheel odometry (`odometry_mu.csv`), and dataset-aligned ground truth (`groundtruth_*.csv`) for each of the 10 sessions already included in the preprocessed LiDAR pack. It integrates via `src/datasets/sensor_loader.py`, which reads the CSVs, converts units to SI, and time-aligns each stream against the Velodyne utimes so the SLAM pipeline can fuse IMU/GPS/FOG with the point-cloud odometry.

---

## How to Run

```bash
# 1. install the project in editable mode
pip install -e .

# 2. verify data loading and preprocessing (local)
python scripts/download_nclt_sample.py --source kaggle

# 3. prepare kaggle dataset (pair annotations for training)
python scripts/prepare_kaggle_dataset.py

# 4. train place recognition model (local GPU or Kaggle notebook)
python scripts/train_place_recognition.py --config configs/train_config.yaml

# 5. evaluate SLAM pipeline (ICP odometry + learned loop closure)
python scripts/evaluate_slam.py --config configs/slam_config.yaml
```

### Training on Kaggle (recommended)

1. Fork the notebook `notebooks/02_kaggle_training.ipynb`
2. Upload to Kaggle and attach both datasets:
   - [NCLT Preprocessed](https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23) - LiDAR point clouds, images, poses
   - [NCLT Sensors Addon](https://www.kaggle.com/datasets/) - IMU, GPS, odometry, ground truth
3. Run all cells

---

## Dataset

Using the preprocessed NCLT dataset from Kaggle with **10 sessions** spanning January--December 2012. The original dataset was collected at the University of Michigan North Campus using a Segway RMP robot equipped with a Velodyne HDL-32E LiDAR, Ladybug3 omnidirectional camera, IMU, and RTK GPS.

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

### Train/Val/Test Split

| Split | Sessions |
|-------|----------|
| Train | 2012-01-08, 2012-01-22, 2012-02-12, 2012-02-18 |
| Val | 2012-03-31, 2012-05-26 |
| Test | 2012-08-04, 2012-10-28, 2012-11-04, 2012-12-01 |

### Place Recognition Parameters

- Positive threshold: 10 m (pairs closer than this are positives)
- Negative threshold: 25 m (pairs farther than this are negatives)
- Point cloud voxel size: 0.1 m
- Max points per scan: 50,000

See `data/README.md` for detailed download and setup instructions.

---

## Architecture

1. **Place Recognition Model**: MinkLoc3D (sparse 3D convolutions via MinkowskiEngine)
   - 256-dim descriptors, triplet loss with hard mining
   - Trained on Kaggle GPU, targeting ONNX export for edge deployment
2. **LiDAR Odometry**: ICP/GICP registration (Open3D)
   - Keyframe selection by distance (2 m) and rotation (15 deg)
3. **Loop Closure**: learned descriptor matching + ICP geometric verification
   - Spatial constraint: 50 m search radius
   - Temporal constraint: 30 s minimum time difference
4. **Pose Graph**: Levenberg-Marquardt optimization
5. **Edge Deployment**: ONNX export, TensorRT (Jetson) or Hailo SDK (Hailo-8)

---

## Project Structure

```
datasets/nclt_kaggle/
├── README.md                              <- this file
├── PROJECT_CONTEXT.md                     <- research context and code conventions
├── requirements.txt                       <- Python dependencies
├── setup.py                               <- editable install
├── configs/
│   ├── dataset_config.yaml                <- dataset paths, sessions, sensor config
│   ├── train_config.yaml                  <- model, optimizer, loss, augmentation
│   └── slam_config.yaml                   <- ICP, loop closure, pose graph
├── scripts/
│   ├── download_nclt_sample.py            <- download dataset from Kaggle API
│   ├── prepare_kaggle_dataset.py          <- generate pair annotations
│   ├── train_place_recognition.py         <- training script (CLI)
│   ├── evaluate_slam.py                   <- full SLAM evaluation pipeline
│   └── generate_mock_sensors.py           <- mock sensor data for testing
├── notebooks/
│   ├── 01_data_exploration.ipynb          <- dataset structure, point cloud visualization
│   ├── 02_kaggle_training.ipynb           <- training notebook for Kaggle GPU
│   ├── 03_evaluation.ipynb                <- trajectory metrics, error analysis
│   └── 04_sensor_exploration.ipynb        <- IMU, GPS, odometry, heading analysis
├── src/
│   ├── datasets/
│   │   ├── nclt_dataset.py                <- NCLT data loading
│   │   ├── nclt_pairs.py                  <- triplet pair generation
│   │   ├── sensor_loader.py               <- IMU, GPS, odometry, KVH loading
│   │   └── transforms.py                  <- augmentations, voxel downsample
│   ├── models/
│   │   ├── place_recognition.py           <- MinkLoc3D / PointNet wrapper
│   │   └── feature_extraction.py          <- descriptor extraction
│   ├── slam/
│   │   ├── lidar_odometry.py              <- ICP/GICP registration
│   │   └── pose_graph.py                  <- graph optimization
│   ├── evaluation/
│   │   ├── metrics.py                     <- ATE, RPE, Recall@K, Umeyama alignment
│   │   └── visualization.py               <- trajectory plots, error plots
│   └── utils/
│       ├── point_cloud.py                 <- load .bin, voxel downsample, ground removal
│       ├── io_utils.py                    <- YAML config, trajectory I/O
│       ├── imu_utils.py                   <- IMU parsing, preintegration, bias
│       ├── gps_utils.py                   <- LLA->ENU, haversine, GPS filtering
│       └── calibration.py                 <- sensor calibration
├── tests/
│   ├── test_dataset.py
│   ├── test_models.py
│   └── test_sensor_loader.py
└── data/
    ├── README.md                          <- data download instructions
    └── nclt_mock/                         <- mock sensor data for offline testing
```

---

## Experiment Results

No trained models or evaluation results yet. The pipeline is set up and ready for training.

Current status:
- [x] Project structure and boilerplate
- [x] Data loading and sensor parsing
- [x] Point cloud preprocessing pipeline
- [x] Model architecture (MinkLoc3D / PointNet fallback)
- [x] Training and evaluation scripts
- [x] Sensor exploration notebooks (IMU, GPS, odometry, heading)
- [ ] Training on Kaggle GPU
- [ ] Recall@K evaluation on test sessions
- [ ] SLAM integration (ICP + learned loop closure)
- [ ] Edge deployment (ONNX, TensorRT)

---

## Limitations

- No trained weights yet, so all RQs are open; Recall@K numbers below are only the plan. The Kaggle notebook boots but has not been run to completion on the Kaggle GPU quota.
- The preprocessed hackathon pack already voxelises at 0.1 m, so the RQ3 voxel sweep is bounded below by that; finer grids would need re-processing from the raw NCLT `.bin` files.
- Positive/negative thresholds (10 m / 25 m) were copied from the PointNetVLAD training recipe; they may be too loose for dense urban sections of NCLT.
- Edge deployment (Jetson / Hailo-8) has not been validated end-to-end, only assumed from the ONNX op set.

## References

- [NCLT Dataset](https://robots.engin.umich.edu/nclt/), official website
- [NCLT Kaggle (hackathon)](https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23) - preprocessed version
- J. Komorowski, "MinkLoc3D: Point Cloud Based Large-Scale Place Recognition," *WACV*, 2021.

```bibtex
@article{carlevaris2016university,
  title={University of {Michigan} {North} {Campus} long-term vision and lidar dataset},
  author={Carlevaris-Bianco, Nicholas and Ushani, Arash K and Eustice, Ryan M},
  journal={The International Journal of Robotics Research},
  volume={35}, number={9}, pages={1023--1035},
  year={2016}, publisher={SAGE Publications},
  doi={10.1177/0278364915614638}
}
```
