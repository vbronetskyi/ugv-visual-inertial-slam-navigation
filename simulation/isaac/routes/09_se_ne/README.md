# 09_se_ne - east edge (SE -> NE)

## route layout
- **Spawn**: (65.0, -35.0)
- Turnaround: (65, 35)
- Planned length: 201 waypoints, ~146 m total roundtrip
- **Label**: east edge (SE -> NE)

## teach run (already completed)

| Metric | Value |
|---|---|
| Bag: `/root/isaac_tr_datasets/09_se_ne/teach/isaac_slam_1776841614` |
| Waypoints driven | 13301 |
| VIO drift mean | **0.396 m** |
| VIO drift max | **0.643 m** |

VIO vs GT plot: [`vio_vs_gt.png`](../../../isaac_tr_datasets/09_se_ne/teach/teach_outputs/vio_vs_gt.png)

Teach artifacts (in dataset folder):
- `landmarks.pkl` - ORB visual landmarks for repeat localisation
- `teach_map.pgm` + `teach_map.yaml` - depth-derived occupancy (built by
  `teach_run_depth_mapper.py` from live `/depth_points` during teach)
- `vio_pose_dense.csv` - dense VIO + GT samples per frame
- `traj_gt.csv` - Isaac ground-truth trajectory for verification

## repeat run

**Pipeline** (from `scripts/run_repeat.sh`, adapted from exp 72 pattern):
1. Isaac Sim with `--obstacles --route 09_se_ne` spawns cones/tent/props
   listed in `spawn_obstacles.OBSTACLES["09_se_ne"]` on the outbound path.
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

### Obstacle placement (09_se_ne)

- 5 quality props (Isaac asset library):
  - 2× **cardbox_large** @ (76.7, -15.0), (76.7, -13.9)
  - 2× barrel_large @ (73.7, 24.5), (73.7, 25.7)
  - 1× **dumpster_small** @ (76.4, 9.5)

Placement strategy: obstacles between ~20% and ~80% of the outbound leg,
minimum ~15 m from spawn (so VIO can warmup), minimum ~10 m from turnaround
(so supervisor fires before robot reaches them on the return).

Overview with obstacles: [`plan_obstacles.png`](results/repeat_run/plan_obstacles.png)

### How to run
```bash
ROS_DOMAIN_ID=85 bash scripts/run_repeat.sh
# Or via orchestrator:  bash routes/run_all_repeat.sh 09_se_ne
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
| WPs reached | **36 / 36** (100.0%) |
| WPs skipped | 0 |
| Plan-fail SKIP events | 0 |
| DETOUR events | 1 |
| TIMEOUT events | 0 |
| Duration | 689 s (11 min 29 s) |

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
| **our custom T&R** | 3.7 m x | 4.0 m x | 29/36 (81%) | 5.21 / 5.72 / 5.72 |
| exp 74 stock Nav2 | 8.7 m x | 12.6 m ✗ | 22/36 (61%) | 0.61 / 0.95 / 1.81 |
| exp 76 RGB-D only (no IMU) | 1.4 m x | 4.9 m x | 17/36 (47%) | 10.50 / 16.95 / 27.88 |

Overview plot (all 3 GT trajectories on the scene map): [`baseline_compare.png`](results/repeat_run/baseline_compare.png)

Per-stack artefacts (goals.log, tf_slam.log, traj_gt.csv, ...):
- our custom: `results/repeat_run/`
- exp 74 stock Nav2: `../../experiments/74_pure_stock_nav2_baseline/results/run_09_se_ne/`
- exp 76 RGB-D no IMU: `../../experiments/76_rgbd_no_imu_ours/results/run_09_se_ne/`

