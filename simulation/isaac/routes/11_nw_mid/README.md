# 11_nw_mid - NW corner -> SW-interior

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 11_nw_mid*


## route layout
- Spawn: (-90.0, 35.0)
- Turnaround: (-24.32, -12.61)
- **Teach path length** (round-trip): +-190 m
- **Obstacles at repeat**: none - clean teach-line repeat (route crosses deep NW forest with few free corridors for side-passing)

## teach run

VIO drift (teach): **0.48 / 0.82 m**.

Teach artefacts in `/root/isaac_tr_datasets/11_nw_mid/teach/teach_outputs/`:
`landmarks.pkl`, `teach_map.{pgm,yaml}`, `vio_pose_dense.csv`, `traj_gt.csv`.

teach pipeline is the canonical one used 04-15: Isaac Sim
(`run_husky_forest.py --synthetic-imu --route 11_nw_mid`) ->
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
`/root/isaac_tr_datasets/11_nw_mid/repeat/results/repeat_run/`).

## repeat result (best variant)

| metric | value |
|---|---|
| **reach** (min GT distance to turnaround) | **7.1 m** x (<= 10 m) |
| **return** (GT distance from end to spawn) | **7.3 m** x (<= 10 m) |
| localisation drift (mean err map->base_link vs GT) | 6.15 m |
| GT path driven | 219 m |
| wall-clock duration | 18 min |

Thresholds: reach <= 10 m, return <= 10 m (with coverage >= 50 % for
return_success).

clean repeat was the chosen representative; the deep NW forest leaves
almost no corridors wide enough for the 4-7 m detour ring to find a
side-pass, so the obstacle phase was skipped here on purpose.
