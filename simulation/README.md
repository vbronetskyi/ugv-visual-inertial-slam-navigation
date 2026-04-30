# simulation - clearpath husky a200

*[thesis root](../README.md) > simulation*

> two simulators: Gazebo Harmonic (early baseline work) and Isaac Sim 6 (main thesis T&R campaign).  Isaac Sim is where the final results come from

simulation environments for visual-inertial navigation experiments on the
clearpath husky a200 UGV with d435i rgb-d camera and phidgets imu

## simulators

### [gazebo/](gazebo/) - gazebo harmonic

primary simulation env for SLAM and navigation experiments.
procedurally generated 220x150m outdoor world with 370 models (trees, rocks,
buildings). integrated with ROS 2 Jazzy, Nav2, RTAB-Map, and ORB-SLAM3

- full ros2 sensor pipeline (camera, imu, lidar, odometry)
- ground truth from gazebo pose plugin
- web UI for click-to-drive navigation
- 3 completed experiments with quantitative results

### [isaac/](isaac/) - nvidia isaac sim 6.0.0

secondary sim env using NVIDIA PhysX for higher-fidelity physics.
husky a200 imported from URDF with d435i camera and imu in a procedural outdoor
scene. runs headless on RTX 3090

- RTX-accelerated rendering and physics
- URDF import with automatic USD conversion
- procedural outdoor terrain with obstacles and buildings
- ros2 bridge support (requires full isaac sim runtime, not pip install)

## folder structure

```
simulation/
├── gazebo/
│   ├── src/              ROS 2 packages (ugv_description, ugv_gazebo, ugv_navigation)
│   ├── tools/            web_nav.py, drive_route.py, record_drive.py
│   ├── scripts/          world generators (v1, v2)
│   ├── routes/           route definitions (JSON)
│   ├── docs/             environment, robot, simulation docs
│   ├── experiments/
│   │   ├── 00_navigation_test/
│   │   ├── 01_autonomous_drive/
│   │   └── 02_slam_comparison/
│   └── README.md
├── isaac/
│   ├── assets/           USD scene files, URDF, textures
│   ├── scripts/          scene building, testing, ros2 bridge
│   ├── experiments/
│   └── README.md
└── README.md             (you are here)
```

## links

- code (this repo): [github.com/vbronetskyi/ugv-visual-inertial-slam-navigation](https://github.com/vbronetskyi/ugv-visual-inertial-slam-navigation)
- dataset (Kaggle): [kaggle.com/datasets/vbronet/isaac-tr-forest-visual-inertial](https://www.kaggle.com/datasets/vbronet/isaac-tr-forest-visual-inertial)

the Kaggle dataset packages the 30 sensor bags used in the Isaac Sim
campaign - 15 routes x 2 phases (teach + repeat-with-obstacles), about
38 GB total. each bag carries 10 Hz RGB + aligned depth, 200 Hz
synthetic IMU, 50 Hz wheel odometry, and 10 Hz ground-truth pose, plus
the per-route teach landmarks and occupancy maps. anyone can rerun the
analysis without re-running the simulator.

## references

- Gazebo Harmonic, open source physics simulator, [website](https://gazebosim.org/)
- NVIDIA Isaac Sim, GPU-accelerated simulation, [docs](https://docs.isaacsim.omniverse.nvidia.com)
- ROS 2 Jazzy, robot middleware, [docs](https://docs.ros.org/en/jazzy/)
- Nav2, Macenski et al., 2020, [paper](https://arxiv.org/abs/2003.00368), [code](https://github.com/ros-navigation/navigation2)
- Clearpath Husky A200, [product page](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
- Intel RealSense D435i, [product page](https://www.intelrealsense.com/depth-camera-d435i/)

## Content map

- [`README.md`](README.md) - this file.  gazebo vs isaac overview
- [`gazebo/`](gazebo/) - Gazebo Harmonic + ROS 2 + Nav2 early experiments (3 experiments)
- [`isaac/`](isaac/) - Isaac Sim main thesis campaign (79 experiments + 15-route T&R campaign)

see the two pipeline READMEs for details: [`gazebo/README.md`](gazebo/README.md), [`isaac/README.md`](isaac/README.md)

## Where to read next

- **thesis main results**: `isaac/README.md` + `isaac/routes/README.md`
- **early Gazebo baselines**: `gazebo/README.md` + `gazebo/experiments/`
- **why we migrated to Isaac Sim**: `isaac/README.md` has the history
