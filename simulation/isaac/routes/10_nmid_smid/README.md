# 10_nmid_smid - N-mid -> S-mid diagonal

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 10_nmid_smid*


## route layout
- Spawn: (-20.0, 30.0)
- Turnaround: (24.75, -31.69)
- **Teach path length** (round-trip): +-182 m
- **Obstacles at repeat**: 9 props in three 2-3 m wide wall clusters - cardbox wall, concrete wall, trashcan cluster - placed at +-30/55/80 % of the outbound leg

## teach run

VIO drift (teach): **0.52 / 0.78 m**.

Teach artefacts in `/root/isaac_tr_datasets/10_nmid_smid/teach/teach_outputs/`:
`landmarks.pkl` (ORB visual landmarks), `teach_map.{pgm,yaml}` (depth-derived
occupancy), `vio_pose_dense.csv` (dense VIO+GT samples), `traj_gt.csv` (GT
trajectory for post-hoc eval only).

teach pipeline is the canonical one used 04-15: Isaac Sim
(`run_husky_forest.py --synthetic-imu --route 10_nmid_smid`) ->
ORB-SLAM3 RGB-D-I VIO -> GT-tf relay -> `vio_drift_monitor` (abort if
|VIO-GT| > 10 m) -> `teach_run_depth_mapper` (0.1 m occupancy) ->
`visual_landmark_recorder` (ORB + depth-variance filter).

## Repeat run (our custom T&R)

Best-of variant: **phase-2 with 2-3 m wide wall obstacles**.

Pipeline (same as 04-09): Isaac Sim + obstacles -> ORB-SLAM3 RGB-D-I VIO ->
`tf_wall_clock_relay_v55 --slam-encoder` (4 fusion regimes:
`no_anchor / ok / strong / jump`) + `visual_landmark_matcher` (ORB +
PnP-RANSAC) -> Nav2 `planner_server` on static teach map + depth
obstacle_layer -> `send_goals_hybrid` (WP projection + 4-7 m detour-ring +
explicit final/return WPs) -> `pure_pursuit_path_follower` ->
`turnaround_supervisor` (drops obstacles at 10 m from apex).

Per-run artefacts under `results/repeat_run/` (symlink to
`/root/isaac_tr_datasets/10_nmid_smid/repeat/results/repeat_run/`):
`goals.log`, `traj_gt.csv`, `nav2.log`, `pp_follower.log`, `tf_slam.log`,
`supervisor.log`, `anchor_matches.csv`.

## repeat result (best variant)

| metric | value |
|---|---|
| **reach** (min GT distance to turnaround) | **4.2 m** x (<= 10 m) |
| **return** (GT distance from end to spawn) | **4.8 m** x (<= 10 m) |
| localisation drift (mean err map->base_link vs GT) | 1.99 m |
| GT path driven | 322 m |
| wall-clock duration | 32 min |

Thresholds: reach <= 10 m, return <= 10 m (with coverage >= 50 % for
return_success).

cleanest result across 10-15. 9 props in three 2-3 m wide wall clusters; robot detoured around each via the 4-7 m ring and still closed both endpoints under 5 m.
