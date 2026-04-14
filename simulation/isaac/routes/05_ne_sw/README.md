# 05_ne_sw - NE -> SW diagonal

## Route layout
- Spawn: (65.0, 35.0)
- **Turnaround**: (-90, -35)
- **Planned length**: 510 waypoints, ~393 m total roundtrip
- Label: NE -> SW diagonal

## teach run (already completed)

| Metric | Value |
|---|---|
| Bag: `/root/isaac_tr_datasets/05_ne_sw/teach/isaac_slam_1776812865` |
| Waypoints driven | 34708 |
| VIO drift mean | **0.475 m** |
| VIO drift max | **0.990 m** |

VIO vs GT plot: [`vio_vs_gt.png`](../../../isaac_tr_datasets/05_ne_sw/teach/teach_outputs/vio_vs_gt.png)

Teach artifacts (in dataset folder):
- `landmarks.pkl` - ORB visual landmarks for repeat localisation
- `teach_map.pgm` + `teach_map.yaml` - depth-derived occupancy (built by
  `teach_run_depth_mapper.py` from live `/depth_points` during teach)
- `vio_pose_dense.csv` - dense VIO + GT samples per frame
- `traj_gt.csv` - Isaac ground-truth trajectory for verification

## repeat run

**Pipeline** (from `scripts/run_repeat.sh`, adapted from exp 72 pattern):
1. Isaac Sim with `--obstacles --route 05_ne_sw` spawns cones/tent/props
   listed in `spawn_obstacles.OBSTACLES["05_ne_sw"]` on the outbound path.
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
8. `turnaround_supervisor.py --final-x -90 --final-y -35 --near-radius 10.0`
   - when robot reaches <10 m from turnaround, writes
   `/tmp/isaac_remove_obstacles.txt` -> Isaac drops all obstacles. **Return
   leg is obstacle-free.**

### Obstacle placement (05_ne_sw)

- **6 quality props** (Isaac asset library):
  - 3× barrel_medium @ (-4.8, 2.1), (-4.8, 3.3), (-4.8, 4.5)
  - 1× **bench** @ (32.9, 11.4)
  - 1× concrete_block_a @ (-44.2, -4.0)
  - 1× **dumpster_small** @ (-82.8, -7.3)

Placement strategy: obstacles between ~20% and ~80% of the outbound leg,
minimum ~15 m from spawn (so VIO can warmup), minimum ~10 m from turnaround
(so supervisor fires before robot reaches them on the return).

Overview with obstacles: [`plan_obstacles.png`](results/repeat_run/plan_obstacles.png)

### how to run
```bash
ROS_DOMAIN_ID=85 bash scripts/run_repeat.sh
# or via orchestrator:  bash routes/run_all_repeat.sh 05_ne_sw
```

### expected outputs (in `results/repeat_run/`)
- `goals.log` - per-WP REACHED / SKIP / DETOUR events + final RESULT line
- `anchor_matches.csv` - landmark-matcher -> tf_relay anchor correction log
- `nav2.log` + `pp_follower.log` - Nav2 planner + pure-pursuit diagnostics
- `supervisor.log` - turnaround trigger FIRE record
- `repeat_result.png` - GT + teach plan + obstacles + skips (post-run plot)
- `tf_slam.log`, `vio.log` - SLAM + VIO diagnostics

## Actual repeat result

| Metric | Value |
|---|---|
| WPs reached | **86 / 96** (89.6%) |
| WPs skipped | 10 |
| Plan-fail SKIP events | 20 |
| DETOUR events | 16 |
| TIMEOUT events | 1 |
| Duration | 2753 s (45 min 53 s) |

Overview plot: [`repeat_result.png`](results/repeat_run/repeat_result.png)

Raw logs (in `results/repeat_run/`): `goals.log`, `anchor_matches.csv`,
`tf_slam.log`, `nav2.log`, `pp_follower.log`, `supervisor.log`.
## baseline comparison

Three stacks, same teach WP list (4 m spacing), same obstacles, same simulator.
- **reach** = min GT distance to turnaround (x ≤ 10 m)
- **return** = GT end-pose distance to spawn (x ≤ 10 m, coverage ≥ 50 %)
- coverage = teach WPs within 3 m of any GT sample
- **drift** = `|published_pose − GT|` mean / p95 / max (m)

| stack | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| **our custom T&R** | 2.5 m x | 31.4 m ✗ | 77/96 (80%) | 9.85 / 37.73 / 37.96 |
| exp 74 stock Nav2 | 132.7 m ✗ | 38.1 m ✗ | 10/96 (10%) | 1.25 / 1.98 / 2.04 |
| exp 76 RGB-D only (no IMU) | 108.8 m ✗ | 63.7 m ✗ | 6/96 (6%) | 6.93 / 7.90 / 7.96 |

Overview plot (all 3 GT trajectories on the scene map): [`baseline_compare.png`](results/repeat_run/baseline_compare.png)

Per-stack artefacts (goals.log, tf_slam.log, traj_gt.csv, ...):
- our custom: `results/repeat_run/`
- exp 74 stock Nav2: `../../experiments/74_pure_stock_nav2_baseline/results/run_05_ne_sw/`
- exp 76 RGB-D no IMU: `../../experiments/76_rgbd_no_imu_ours/results/run_05_ne_sw/`

