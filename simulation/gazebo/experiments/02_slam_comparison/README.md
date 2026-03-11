# Experiment 02: SLAM Comparison - RTAB-Map vs ORB-SLAM3

Compared two Visual SLAM algorithms on the same 216m route through a procedurally generated Gazebo world. The robot was driven manually via a web UI, recording RGB-D + IMU data for offline evaluation.

![Trajectory comparison](trajectory_comparison.png)

## Setup

| Parameter | Value |
|-----------|-------|
| World | 220 x 150m, 370 models (trees, rocks, buildings) |
| Route | (-105, -8) -> curved dirt road -> (82, -13), 216m total |
| Robot | Husky A200, 50kg, skid-steer, max 0.9 m/s |
| RGB-D camera | 640x480, fx=fy=382, ~10-15 Hz actual (30 Hz requested) |
| IMU | 250 Hz (Phidgets Spatial 1042) |
| LiDAR | disabled to save GPU for camera |
| Ground truth | Gazebo dynamic_pose, 50 Hz, <1cm accuracy |
| Drive method | manual click-to-drive via Flask web UI |
| Rosbag | uncompressed, ~41 GB for a typical 5 min drive |

The world has three zones: dense forest (west), open field with dirt road (center), and a village with houses (east). Camera FPS drops to ~6 Hz during rosbag recording.

## Results

| Algorithm | Mode | ATE RMSE | Keyframes | Coverage | Status |
|-----------|------|----------|-----------|----------|--------|
| RTAB-Map | RGB-D (live) | **9.23 m** | 269 | ~80m (37%) | partial - forest only |
| ORB-SLAM3 | RGB-D (offline) | n/a | - | 1.2m (<1%) | **failed** - 174 map resets |
| ORB-SLAM3 | RGB-D Inertial (offline) | n/a | - | ~300 frames | **failed** - IMU never initialized |

Ground truth: 108,327 poses over 216m route, recorded at 50 Hz.

### RTAB-Map

Ran live during manual drive (not offline replay - rosbag TF conflicts made replay impossible).

- tracked well through **forest section** (~80m): trees create depth discontinuities at 2-5m range, giving the depth matcher plenty of structure
- **lost tracking** when exiting forest into open field: flat green terrain with no depth variation
- scale factor 0.28 indicates significant wheel slip on skid-steer (robot thinks it moved ~3.6x less than it did)
- 660-690 visual features per frame in forest - drops to <100 on open grass
- database: 240 MB (rtabmap.db, gitignored)

### ORB-SLAM3 RGB-D

Ran offline on extracted frames (every 3rd frame = 3024 total).

- constant tracking loss: ORB detector finds 30-196 map points per map (needs ~500 for stable tracking)
- created 174 separate maps in one run - essentially re-initializes every few seconds
- total trajectory coverage: 1.2m x 0.5m (robot barely moves in SLAM's view)
- root cause: Gazebo ogre2 renderer produces smooth, low-contrast textures - ORB features need corners and edges that simply aren't there

### ORB-SLAM3 RGB-D Inertial

Attempted to fix tracking with IMU fusion (250 Hz Phidgets data).

- map resets dropped from **174 to 2** - IMU stabilizes frame-to-frame tracking significantly
- however, IMU initialization never completes: ORB-SLAM3 needs ~10 seconds of continuous visual tracking to calibrate IMU biases
- the algorithm can't sustain 10s of tracking in this low-texture environment, so it gets stuck in a loop: track -> lose -> reset -> track -> lose
- tested with multiple configs (lower noise params, aggressive ORB extraction, no usleep) - same result

## Why ORB-SLAM3 Fails in Gazebo

ORB-SLAM3 relies on sparse feature matching (ORB descriptors). It needs:
- **corners and edges** in the image - Gazebo procedural textures are smooth gradients with very few sharp features
- **texture repeatability** across frames - ogre2 rendering produces slightly different pixel values between frames due to floating-point shading, breaking descriptor matching
- **sufficient feature density** - needs ~500 map points for stable tracking; Gazebo scenes typically produce 30-200

This is a basic limit of testing feature-based SLAM in simulation, not a config issue.

### Would ArUco Markers Help?

ArUco markers are black-and-white fiducial squares designed for easy detection - high contrast, unique IDs, sub-pixel corner localization. Placing them on trees and buildings would give ORB-SLAM3 reliable features. However:
- we didn't use them because the goal was to evaluate SLAM on natural-looking terrain, not on a marker-augmented lab environment
- real outdoor environments don't have ArUco markers
- it would invalidate the comparison with real-world datasets (ROVER, NCLT) where no markers are present

## Why RTAB-Map Partially Works

RTAB-Map uses **dense depth matching** rather than sparse features. It doesn't need corners - it matches depth images directly. This works when:
- objects are at varied distances (2-5m in forest = strong depth signal)
- there's geometric structure (tree trunks, fallen logs)

It fails when:
- terrain is flat (open field = depth image is uniform ~10m plane)
- no close objects for depth matching

## Config Files

| File | Description |
|------|-------------|
| `config/gazebo_d435i.yaml` | ORB-SLAM3 RGB-D config (fx=fy=382, no distortion) |
| `config/gazebo_aggressive.yaml` | ORB-SLAM3 with 5000 features, lower FAST thresholds |
| `config/gazebo_d435i_inertial.yaml` | ORB-SLAM3 RGB-D Inertial with IMU params for Phidgets 1042 |

## Reproduction

```bash
# 1. launch simulation
cd /workspace/simulation
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
ros2 launch ugv_gazebo full_sim.launch.py headless:=true

# 2. drive and record (in another terminal)
python3 tools/web_nav.py
ros2 bag record -o bags/route_1_clean \
  --topics /camera/color/image_raw /camera/depth/image_rect_raw \
  /camera/camera_info /imu/data /odom /tf /clock

# 3. extract frames for ORB-SLAM3
python3 scripts/01_extract_frames.py
python3 scripts/02_fix_timestamps.py

# 4. run ORB-SLAM3
bash scripts/03_run_orb_slam3.sh

# 5. compare results
python3 scripts/06_compare_all.py
```

RTAB-Map runs live during step 2 (add `slam_type:=rtabmap` to launch command). Rosbag replay doesn't work due to TF time conflicts.

## Data Files

| File | Size | Description |
|------|------|-------------|
| `gt_trajectory.csv` | 4 MB | ground truth (108k poses) |
| `rtabmap_poses.txt` | 14 KB | RTAB-Map trajectory (269 keyframes, TUM format) |
| `CameraTrajectory.txt` | 25 KB | ORB-SLAM3 output (505 poses, mostly stationary) |
| `trajectory_comparison.png` | 900 KB | GT + RTAB-Map + ORB-SLAM3 overlay plot |
| `results.json` | 1 KB | ATE/RPE metrics for both algorithms |
| `rtabmap.db` | 282 MB | RTAB-Map database (gitignored, reproduce via step 2) |
| `slam_results.md` | - | detailed per-algorithm analysis |

## Next Steps

- test on real Husky A200 with actual D435i camera (real textures should help ORB-SLAM3)
- try ORB-SLAM3 RGB-D Inertial on real robot where IMU has actual vibrations from motor/terrain
- compare with LiDAR-based SLAM (already evaluated on NCLT dataset, see `datasets/nclt/`)
- consider adding photorealistic textures to Gazebo world (PBR materials, high-res ground textures)
