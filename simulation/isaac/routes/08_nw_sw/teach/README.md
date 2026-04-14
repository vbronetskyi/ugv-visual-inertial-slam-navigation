# 08_nw_sw - teach

Corner-to-corner roundtrip teach: left edge LT->LB
Route: (-90, +35) -> (-90, -35), smooth hairpin turnaround (r=1.5 m, 180° arc + parallel-offset
blended return of 10 pts), mirror outbound for the return leg.

## Numbers (from `_common/routes.json`)
- total waypoints: 205 (102 outbound + turnaround arc + mirrored return)
- total path length (incl. return): **163.7 m**
- spawn: x=-90.00, y=+35.00, yaw=-2.092 rad (-120°)
- min clearance from any scene-object edge: **≥ 2.1 m** (robot r=0.4 m, so
  physical body-to-obstacle gap ≥ 1.7 m)
- obstacle list (532 prims) from `_common/scene_obstacles.json`, extracted
  directly from `/opt/husky_forest_scene.usd`

## Pipeline (from `scripts/run_teach_single.sh`)

1. `register_routes.py` merges `_common/routes.json` into
   `/tmp/slam_routes.json` so `run_husky_forest.py` can pick up
   `--route 08_nw_sw` as a predefined path.
2. Isaac Sim launches with:
   `--synthetic-imu --route 08_nw_sw --duration 4500`
   plus `--spawn-x/--spawn-y/--spawn-yaw` for the corner start.
3. ORB-SLAM3 RGB-D-Inertial begins capturing from `isaac_slam_<ts>/`
   bag (rgb/depth/imu/gt).
4. `tf_wall_clock_relay.py --use-gt` publishes GT-based `map->base_link`
   tf - teach uses GT for tf, not VIO (teach is the reference).
5. `vio_drift_monitor.py` - dense VIO-vs-GT CSV + drift gate
   (aborts with exit 2 if mean drift > 2 m after 30 s settling).
6. `teach_run_depth_mapper.py` - builds a static occupancy costmap from
   depth camera -> `teach_map.{pgm,yaml}`.
7. `visual_landmark_recorder.py` - records ORB landmarks every 2 m
   of VIO displacement -> `landmarks.pkl`.
8. Wait loop exits when `ROUTE COMPLETE` appears in `isaac.log` -
   saves `traj_gt.csv` and stops all processes.

## How to run
```bash
ROS_DOMAIN_ID=85 bash scripts/run_teach_single.sh
#   (or) scripts/run_teach_with_retry.sh   - retries up to 3× on drift abort
```

## Outputs (teach/08_nw_sw/ after a successful run)
- `landmarks.pkl` - ORB landmarks for repeat stage
- `teach_map.{pgm,yaml}` - static costmap
- `vio_pose_dense.csv` - VIO-vs-GT per frame
- `vio_vs_gt.png` - drift plot (from plot_trajectory_map)
- `traj_gt.csv` - dense GT from Isaac
- `landmarks_debug.png`, logs, `run_info.txt`
