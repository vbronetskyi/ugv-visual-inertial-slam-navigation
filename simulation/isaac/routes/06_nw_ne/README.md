# 06_nw_ne - north edge (NW -> NE)

## Route layout
- **Spawn**: (-90.0, 35.0)
- **Turnaround**: (65, 35)
- **Planned length**: 493 waypoints, ~382 m total roundtrip
- **Label**: north edge (NW -> NE)

## teach run (already completed)

| Metric | Value |
|---|---|
| Bag: `/root/isaac_tr_datasets/06_nw_ne/teach/isaac_slam_1776816539` |
| Waypoints driven | 33319 |
| VIO drift mean | **0.647 m** |
| VIO drift max | **1.178 m** |

VIO vs GT plot: [`vio_vs_gt.png`](../../../isaac_tr_datasets/06_nw_ne/teach/teach_outputs/vio_vs_gt.png)

Teach artifacts (in dataset folder):
- `landmarks.pkl` - ORB visual landmarks for repeat localisation
- `teach_map.pgm` + `teach_map.yaml` - depth-derived occupancy (built by
  `teach_run_depth_mapper.py` from live `/depth_points` during teach)
- `vio_pose_dense.csv` - dense VIO + GT samples per frame
- `traj_gt.csv` - Isaac ground-truth trajectory for verification

## Repeat run

Pipeline (from `scripts/run_repeat.sh`, adapted from exp 72 pattern):
1. Isaac Sim with `--obstacles --route 06_nw_ne` spawns cones/tent/props
   listed in `spawn_obstacles.OBSTACLES["06_nw_ne"]` on the outbound path.
2. ORB-SLAM3 RGB-D-Inertial VIO starts from bag live-recording.
3. `tf_wall_clock_relay_v55.py --slam-encoder` publishes map->base_link TF
   derived from VIO + encoder fallback; align committed at spawn from 50
   GT samples.
4. `visual_landmark_matcher.py` matches live ORB frame vs `landmarks.pkl`
   -> `/anchor_correction` for drift correction.
5. **Nav2 planner-only** (no controller): `planner_server` + `map_server`
   with plugins `["static_layer", "obstacle_layer", "inflation_layer"]`.
   `obstacle_layer` subscribes to `/depth_points` - detects cones/props
   live and updates costmap.
6. `pure_pursuit_path_follower.py` consumes `/plan` -> `/cmd_vel`.
7. `send_goals_hybrid.py` reads `traj_gt.csv` sub-sampled at 4 m, fires
   ComputePathToPose for each WP; on costmap-cost high or plan fail, does
   a 4–7 m detour ring around the WP.
8. `turnaround_supervisor.py --final-x 65 --final-y 35 --near-radius 10.0`
   - when robot reaches <10 m from turnaround, writes
   `/tmp/isaac_remove_obstacles.txt` -> Isaac drops all obstacles. **Return
   leg is obstacle-free.**

### Obstacle placement (06_nw_ne)

- **6 quality props** (Isaac asset library):
  - 3× cardbox_large @ (-38.1, -4.9), (-38.1, -3.9), (-38.1, -2.9)
  - 1× **firehydrant** @ (-62.7, 18.8)
  - 1× railing @ (-0.1, 3.5)
  - 1× **dumpster_large** @ (35.0, 13.5)

Placement strategy: obstacles between ~20% and ~80% of the outbound leg,
minimum ~15 m from spawn (so VIO can warmup), minimum ~10 m from turnaround
(so supervisor fires before robot reaches them on the return).

Overview with obstacles: [`plan_obstacles.png`](results/repeat_run/plan_obstacles.png)

### How to run
```bash
ROS_DOMAIN_ID=85 bash scripts/run_repeat.sh
# or via orchestrator:  bash routes/run_all_repeat.sh 06_nw_ne
```

### Expected outputs (in `results/repeat_run/`)
- `goals.log` - per-WP REACHED / SKIP / DETOUR events + final RESULT line
- `anchor_matches.csv` - landmark-matcher -> tf_relay anchor correction log
- `nav2.log` + `pp_follower.log` - Nav2 planner + pure-pursuit diagnostics
- `supervisor.log` - turnaround trigger FIRE record
- `repeat_result.png` - GT + teach plan + obstacles + skips (post-run plot)
- `tf_slam.log`, `vio.log` - SLAM + VIO diagnostics

## actual repeat result

| Metric | Value |
|---|---|
| WPs reached | **85 / 94** (90.4%) |
| WPs skipped | 9 |
| Plan-fail SKIP events | 16 |
| DETOUR events | 16 |
| TIMEOUT events | 1 |
| Duration | 3835 s (63 min 55 s) |

Overview plot: [`repeat_result.png`](results/repeat_run/repeat_result.png)

Raw logs (in `results/repeat_run/`): `goals.log`, `anchor_matches.csv`,
`tf_slam.log`, `nav2.log`, `pp_follower.log`, `supervisor.log`.
## Baseline comparison

Three stacks, same teach WP list (4 m spacing), same obstacles, same simulator.
- **reach** = min GT distance to turnaround (x ≤ 10 m)
- **return** = GT end-pose distance to spawn (x ≤ 10 m, coverage ≥ 50 %)
- coverage = teach WPs within 3 m of any GT sample
- **drift** = `|published_pose − GT|` mean / p95 / max (m)

| stack | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| **our custom T&R** | 5.3 m x | 10.2 m ✗ | 56/94 (60%) | 5.69 / 9.05 / 9.15 |
| exp 74 stock Nav2 | 110.5 m ✗ | 62.0 m ✗ | 18/94 (19%) | 2.25 / 3.76 / 3.85 |
| exp 76 RGB-D only (no IMU) | 0.5 m x | 131.8 m ✗ | 58/94 (62%) | 4.34 / 8.74 / 8.91 |

Overview plot (all 3 GT trajectories on the scene map): [`baseline_compare.png`](results/repeat_run/baseline_compare.png)

Per-stack artefacts (goals.log, tf_slam.log, traj_gt.csv, ...):
- our custom: `results/repeat_run/`
- exp 74 stock Nav2: `../../experiments/74_pure_stock_nav2_baseline/results/run_06_nw_ne/`
- exp 76 RGB-D no IMU: `../../experiments/76_rgbd_no_imu_ours/results/run_06_nw_ne/`

