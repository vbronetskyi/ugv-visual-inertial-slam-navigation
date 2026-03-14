# Simulation Experiments

This directory contains experiments conducted with the Husky A200 UGV in Gazebo Harmonic simulation (ROS 2 Jazzy, Ubuntu 24.04). Experiments progress from basic navigation integration to full SLAM evaluation and comparison.

## Experiments

| Experiment | Description | World | Key Result |
|---|---|---|---|
| [00_navigation_test](00_navigation_test/) | First Nav2 integration test | 390x390m | Successful autonomous navigation |
| [01_autonomous_drive](01_autonomous_drive/) | Route 1 drive + ORB-SLAM3 evaluation | 390x390m | ORB-SLAM3 ATE 35.4m (poor) |
| [02_slam_comparison](02_slam_comparison/) | RTAB-Map vs ORB-SLAM3 on new world | 220x150m | RTAB-Map ATE 7.91m, ORB-SLAM3 failed |

## Progression

1. **00_navigation_test** - Validated the full Nav2 stack (SLAM Toolbox + RPP controller) on a small area. Identified and fixed critical issues with costmap memory growth and controller compatibility for skid-steer drive. Documented fixes in the [fixes/](fixes/) directory.

2. **01_autonomous_drive** - Scaled up to a full 271m route through a 390x390m world with 541 objects. Established ground truth tracking via Gazebo `dynamic_pose/info` (50Hz, <1cm accuracy). Ran ORB-SLAM3 offline on recorded data; ATE of 35.4m indicated that Gazebo's low-texture rendering is insufficient for sparse visual SLAM.

3. **02_slam_comparison** - Built a new compact 220x150m world (370 models) and compared RTAB-Map (live, RGB-D) against ORB-SLAM3 (offline, RGB-D). RTAB-Map tracked through the forest section (ATE 7.91m over ~80m) but lost tracking in the open field. ORB-SLAM3 failed entirely with only 1.2m of trajectory coverage.

## Troubleshooting Fixes

The [fixes/](fixes/) directory contains detailed write-ups of issues encountered during development:

- [fix_01_global_costmap_oom.md](fixes/fix_01_global_costmap_oom.md) - planner_server OOM due to unbounded global costmap growth
- [fix_02_controller_skidsteer.md](fixes/fix_02_controller_skidsteer.md) - DWB controller incompatible with skid-steer; switched to RPP

## Common Setup

All experiments use the same simulation stack:

```bash
# Build
cd /workspace/simulation
colcon build --symlink-install --packages-select ugv_description ugv_gazebo ugv_navigation

# Launch
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch ugv_gazebo full_sim.launch.py headless:=true
```

Ground truth is collected via Gazebo `dynamic_pose/info` topic at 50Hz with <1cm accuracy.
