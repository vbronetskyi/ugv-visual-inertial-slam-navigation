# 12_ne_mid - NE corner -> centre

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 12_ne_mid*


## route layout
- Spawn: (65.0, 35.0)
- Turnaround: (-20.9, -1.84)
- **Teach path length** (round-trip): +-200 m
- **Obstacles at repeat**: none - clean teach-line repeat

## teach run

VIO drift (teach): **0.52 / 0.86 m**.

Teach artefacts in `/root/isaac_tr_datasets/12_ne_mid/teach/teach_outputs/`:
`landmarks.pkl`, `teach_map.{pgm,yaml}`, `vio_pose_dense.csv`, `traj_gt.csv`.

teach pipeline is the canonical one used 04-15: Isaac Sim
(`run_husky_forest.py --synthetic-imu --route 12_ne_mid`) ->
ORB-SLAM3 RGB-D-I VIO -> GT-tf relay -> `vio_drift_monitor` (abort if
|VIO-GT| > 10 m) -> `teach_run_depth_mapper` (0.1 m occupancy) ->
`visual_landmark_recorder`.

## Repeat run (our custom T&R)

Best-of variant: **phase-1 (clean repeat, no obstacles)**.

Pipeline (same as 04-09): Isaac Sim -> ORB-SLAM3 RGB-D-I VIO ->
`tf_wall_clock_relay_v55 --slam-encoder` + `visual_landmark_matcher` ->
Nav2 `planner_server` on static teach map -> `send_goals_hybrid` ->
`pure_pursuit_path_follower` -> `turnaround_supervisor`.

Per-run artefacts under `results/repeat_run/` (symlink to
`/root/isaac_tr_datasets/12_ne_mid/repeat/results/repeat_run/`).

## repeat result (best variant)

| metric | value |
|---|---|
| **reach** (min GT distance to turnaround) | **2.8 m** x (<= 10 m) |
| **return** (GT distance from end to spawn) | **6.5 m** x (<= 10 m) |
| localisation drift (mean err map->base_link vs GT) | 1.62 m |
| GT path driven | 224 m |
| wall-clock duration | 18 min |

Thresholds: reach <= 10 m, return <= 10 m (with coverage >= 50 % for
return_success).

tightest overall result of the 10-15 batch: drift 1.6 m over 224 m, reach
2.8 m, return 6.5 m. mid-tier corridor density gives the matcher steady
anchor hits across the whole route.
