# 07_se_sw - south edge (SE -> SW)

## route layout
- **Spawn**: (65.0, -35.0)
- **Turnaround**: (-90, -35)
- **Planned length**: 502 waypoints, ~386 m total roundtrip
- **Label**: south edge (SE -> SW)

## Teach run (already completed)

| Metric | Value |
|---|---|
| Bag: `/root/isaac_tr_datasets/07_se_sw/teach/isaac_slam_1776837711` |
| Waypoints driven | 35532 |
| VIO drift mean | **0.421 m** |
| VIO drift max | **0.992 m** |

VIO vs GT plot: [`vio_vs_gt.png`](../../../isaac_tr_datasets/07_se_sw/teach/teach_outputs/vio_vs_gt.png)

Teach artifacts (in dataset folder):
- `landmarks.pkl` - ORB visual landmarks for repeat localisation
- `teach_map.pgm` + `teach_map.yaml` - depth-derived occupancy (built by
  `teach_run_depth_mapper.py` from live `/depth_points` during teach)
- `vio_pose_dense.csv` - dense VIO + GT samples per frame
- `traj_gt.csv` - Isaac ground-truth trajectory for verification

## repeat run

**Pipeline** (from `scripts/run_repeat.sh`, adapted from exp 72 pattern):
1. Isaac Sim with `--obstacles --route 07_se_sw` spawns cones/tent/props
   listed in `spawn_obstacles.OBSTACLES["07_se_sw"]` on the outbound path.
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

### Obstacle placement (07_se_sw)

- **7 quality props** (Isaac asset library):
  - 3× **trashcan** @ (25.8, -31.0), (25.8, -30.0), (25.8, -29.0)
  - 2× **barrel_large** @ (-46.4, -8.6), (-46.4, -7.4)
  - 1× concrete_block_b @ (-9.6, -18.4)
  - 1× **bench** @ (-83.0, -7.7)

Placement strategy: obstacles between ~20% and ~80% of the outbound leg,
minimum ~15 m from spawn (so VIO can warmup), minimum ~10 m from turnaround
(so supervisor fires before robot reaches them on the return).

Overview with obstacles: [`plan_obstacles.png`](results/repeat_run/plan_obstacles.png)

### how to run
```bash
ROS_DOMAIN_ID=85 bash scripts/run_repeat.sh
# Or via orchestrator:  bash routes/run_all_repeat.sh 07_se_sw
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
| WPs reached | **80 / 95** (84.2%) |
| WPs skipped | 15 |
| Plan-fail SKIP events | 24 |
| DETOUR events | 28 |
| TIMEOUT events | 6 |
| Duration | 5199 s (86 min 39 s) |

Overview plot: [`repeat_result.png`](results/repeat_run/repeat_result.png)

Raw logs (in `results/repeat_run/`): `goals.log`, `anchor_matches.csv`,
`tf_slam.log`, `nav2.log`, `pp_follower.log`, `supervisor.log`.
## baseline comparison

Three stacks, same teach WP list (4 m spacing), same obstacles, same simulator.
- reach = min GT distance to turnaround (x ≤ 10 m)
- **return** = GT end-pose distance to spawn (x ≤ 10 m, coverage ≥ 50 %)
- **coverage** = teach WPs within 3 m of any GT sample
- **drift** = `|published_pose − GT|` mean / p95 / max (m)

| stack | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| **our custom T&R** | 0.6 m x | 14.7 m ✗ | 70/95 (74%) | 3.76 / 5.83 / 5.94 |
| exp 74 stock Nav2 | 116.4 m ✗ | 29.9 m ✗ | 8/95 (8%) | 0.98 / 1.99 / 2.61 |
| exp 76 RGB-D only (no IMU) | 43.1 m ✗ | 112.3 m ✗ | 12/95 (13%) | 2.14 / 2.94 / 3.13 |

Overview plot (all 3 GT trajectories on the scene map): [`baseline_compare.png`](results/repeat_run/baseline_compare.png)

Per-stack artefacts (goals.log, tf_slam.log, traj_gt.csv, ...):
- our custom: `results/repeat_run/`
- exp 74 stock Nav2: `../../experiments/74_pure_stock_nav2_baseline/results/run_07_se_sw/`
- exp 76 RGB-D no IMU: `../../experiments/76_rgbd_no_imu_ours/results/run_07_se_sw/`

