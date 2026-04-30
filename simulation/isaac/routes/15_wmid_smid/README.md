# 15_wmid_smid - W-mid -> S-mid diagonal

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 15_wmid_smid*


## route layout
- Spawn: (-61.5, 8.5)
- Turnaround: (25.5, -31.55)
- **Teach path length** (round-trip): +-202 m
- **Obstacles at repeat**: 8 props - railing (perpendicular) + 3-prop concrete/barrel mix + 4-prop firehydrant/trashcan wall

## teach run

VIO drift (teach): **0.58 / 0.96 m**.

Teach artefacts in `/root/isaac_tr_datasets/15_wmid_smid/teach/teach_outputs/`:
`landmarks.pkl`, `teach_map.{pgm,yaml}`, `vio_pose_dense.csv`, `traj_gt.csv`.

teach pipeline is the canonical one used 04-15: Isaac Sim
(`run_husky_forest.py --synthetic-imu --route 15_wmid_smid`) ->
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
`/root/isaac_tr_datasets/15_wmid_smid/repeat/results/repeat_run/`).

## repeat result (best variant)

| metric | value |
|---|---|
| **reach** (min GT distance to turnaround) | **4.8 m** x (<= 10 m) |
| **return** (GT distance from end to spawn) | **6.5 m** x (<= 10 m) |
| localisation drift (mean err map->base_link vs GT) | 13.84 m |
| GT path driven | 488 m |
| wall-clock duration | 43 min |

Thresholds: reach <= 10 m, return <= 10 m (with coverage >= 50 % for
return_success).

longest repeat path (488 m vs 202 m teach = 2.4x overhead from 11
detours). drift 13.8 m mean because the route spends long stretches in
the landmark-sparse open area between spawn and the first cluster, but
endpoint closure still worked.
