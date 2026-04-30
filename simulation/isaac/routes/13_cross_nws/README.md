# 13_cross_nws - interior NW -> SE diagonal cross

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 13_cross_nws*


## route layout
- Spawn: (-30.0, 20.0)
- Turnaround: (27.42, -15.53)
- **Teach path length** (round-trip): +-166 m
- **Obstacles at repeat**: 8 props in three groups along outbound - 3-prop concrete wall + bench + 4-prop trashcan wall

## teach run

VIO drift (teach): **0.55 / 0.94 m**.

Teach artefacts in `/root/isaac_tr_datasets/13_cross_nws/teach/teach_outputs/`:
`landmarks.pkl`, `teach_map.{pgm,yaml}`, `vio_pose_dense.csv`, `traj_gt.csv`.

teach pipeline is the canonical one used 04-15: Isaac Sim
(`run_husky_forest.py --synthetic-imu --route 13_cross_nws`) ->
ORB-SLAM3 RGB-D-I VIO -> GT-tf relay -> `vio_drift_monitor` (abort if
|VIO-GT| > 10 m) -> `teach_run_depth_mapper` (0.1 m occupancy) ->
`visual_landmark_recorder`.

## Repeat run (our custom T&R)

Best-of variant: **phase-2 with 2-3 m wide wall obstacles**.

Pipeline (same as 04-09): Isaac Sim + obstacles -> ORB-SLAM3 RGB-D-I VIO ->
`tf_wall_clock_relay_v55 --slam-encoder` + `visual_landmark_matcher` ->
Nav2 `planner_server` on static teach map + depth obstacle_layer ->
`send_goals_hybrid` (4-7 m detour-ring) -> `pure_pursuit_path_follower` ->
`turnaround_supervisor`.

Per-run artefacts under `results/repeat_run/` (symlink to
`/root/isaac_tr_datasets/13_cross_nws/repeat/results/repeat_run/`).

## repeat result (best variant)

| metric | value |
|---|---|
| **reach** (min GT distance to turnaround) | **2.6 m** x (<= 10 m) |
| **return** (GT distance from end to spawn) | 28.7 m ✗ (> 10 m) |
| localisation drift (mean err map->base_link vs GT) | 17.06 m |
| GT path driven | 518 m |
| wall-clock duration | 47 min |

Thresholds: reach <= 10 m, return <= 10 m (with coverage >= 50 % for
return_success).

only route in 10-15 that failed the return check (28.7 m end-to-spawn).
reach was clean (2.6 m) but the interior cross passes through a visually
repetitive interior section where the matcher latched onto wrong
landmarks and pulled published pose 17 m off GT on the return.  this is
the clearest demonstration of the IMU+matcher fusion limit when teach
landmarks are ambiguous - no amount of detour ring saves you when the
matcher itself is publishing wrong corrections.
