# 14_se_mid - SE corner -> N-mid

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 14_se_mid*


## route layout
- Spawn: (65.0, -35.0)
- Turnaround: (0.0, 15.0)
- **Teach path length** (round-trip): +-175 m
- **Obstacles at repeat**: 8 props in three 1.6-1.9 m wide walls perpendicular to teach line at +-30/55/80 % of outbound - 3*barrel_medium + 2*concrete + 3*cardbox_large

## teach run

VIO drift (teach): **0.43 / 0.71 m**.

Teach artefacts in `/root/isaac_tr_datasets/14_se_mid/teach/teach_outputs/`:
`landmarks.pkl`, `teach_map.{pgm,yaml}`, `vio_pose_dense.csv`, `traj_gt.csv`.

teach pipeline is the canonical one used 04-15: Isaac Sim
(`run_husky_forest.py --synthetic-imu --route 14_se_mid`) ->
ORB-SLAM3 RGB-D-I VIO -> GT-tf relay -> `vio_drift_monitor` (abort if
|VIO-GT| > 10 m) -> `teach_run_depth_mapper` (0.1 m occupancy) ->
`visual_landmark_recorder`.

## Repeat run (our custom T&R)

Best-of variant: **phase-2b with single-prop obstacle clusters**.

Pipeline (same as 04-09): Isaac Sim + obstacles -> ORB-SLAM3 RGB-D-I VIO ->
`tf_wall_clock_relay_v55 --slam-encoder` + `visual_landmark_matcher` ->
Nav2 `planner_server` on static teach map + depth obstacle_layer ->
`send_goals_hybrid` (4-7 m detour-ring) -> `pure_pursuit_path_follower` ->
`turnaround_supervisor`.

Per-run artefacts under `results/repeat_run/` (symlink to
`/root/isaac_tr_datasets/14_se_mid/repeat/results/repeat_run/`).

## repeat result (best variant)

| metric | value |
|---|---|
| **reach** (min GT distance to turnaround) | **3.7 m** x (<= 10 m) |
| **return** (GT distance from end to spawn) | **2.7 m** x (<= 10 m) |
| localisation drift (mean err map->base_link vs GT) | 6.46 m |
| GT path driven | 406 m |
| wall-clock duration | 42 min |

Thresholds: reach <= 10 m, return <= 10 m (with coverage >= 50 % for
return_success).

tight interior corridor through NE forest. three single-prop clusters
leave enough bypass width for the 4-7 m detour ring; both endpoints
closed cleanly (reach 3.7 m, return 2.7 m).
