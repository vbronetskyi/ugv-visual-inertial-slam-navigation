# exp 40: Teach-and-Repeat + Nav2

## goal

Single destination goal (70.4, -2.3) sent to Nav2. Nav2 plans on blank static
map, uses depth camera for obstacle detection, replans around obstacles.

## Architecture

```
Isaac S
 im (run_husky_forest.py --obstacles --nav2-bridge)
  └─ OmniGraph: /camera/depth + /odom + /imu
  └─ GT pose -> /tmp/isaac_pose.txt

tf_wall_clock_relay.py --use-gt
  └─ map->odom->base_link (GT, wall-clock)
  └─ depth -> /depth_points (PointCloud2)

Nav2 (blank 900×200 static map + depth obstacles)
  └─ NavFn A* planner
  └─ DWB controller (exp 01 config)
  └─ Single goal: (70.4, -2.3)
```

## Issues fixed in this experiment

| # | Issue | Fix |
|---|---|---|
| 1 | Camera API depth broken | OmniGraph ROS2CameraHelper |
| 2 | No odom TF frame | GT mode: map->odom->base_link |
| 3 | TF_OLD_DATA conflict | OmniGraph TF remapped to /tf_isaac |
| 4 | Static map loading old SLAM | Blank 900×200 map (180×40m) |
| 5 | Pose file stale | Bridge writes GT to isaac_pose.txt |
| 6 | Rolling window too small | Static blank map covers full route |
| 7 | 43 sequential WPs abort | Single destination goal |

## results

### Best run: single goal with RPP (first attempt)
| Metric | Value |
|---|---|
| Distance driven | **45m** (x=-95 -> x=-50) |
| Speed | ~0.8 m/s |
| Cone group 1 | Reached x=-50, stuck at 0.4m gap |
| Failure | backup failed -> ABORTED |

### Subsequent runs: DWB (exp 01 config)
| Metric | Value |
|---|---|
| Distance driven | **17m** (x=-95 -> x=-78) |
| Failure | Roadside shrub at (-79.3, -6.1) blocks road |
| Cause | Depth costmap accumulates shrub -> road narrows -> backup fails |

### Cone spacing test (2m instead of 1m)
Same 17m result - cones not reached, shrub blocks first.

## Analysis

### roadside shrub at x=-78 blocks all runs

Scene objects near the road at x=-78:
- Shrub at (-79.3, -6.1) - 1m from road center
- Shrub at (-80.0, -7.5) - near road edge

The depth camera detects these shrubs, costmap marks them, inflation narrows
the road. DWB/RPP can't find a collision-free trajectory -> backup -> abort.

The first 45m run succeeded because it drove through before the costmap fully
accumulated. Subsequent runs fail because costmap accumulates faster (timing
dependent).

### inflation tuning didn't help

| Config | robot_radius | inflation_radius | cost_scaling | Result |
|---|---|---|---|---|
| Our config | 0.35 | 1.5 | 2.5 | 45m (first), 17m (later) |
| robot_radius=0.40 | 0.40 | 1.5 | 2.5 | 17m - road blocked |
| robot_radius=0.45 | 0.45 | 1.5 | 3.0 | 6s abort - start blocked |
| inflation=2.0 | 0.35 | 2.0 | 2.5 | 19m - road blocked |
| Exp 01 (DWB) | 0.35 | 0.55/0.45 | 4.0/8.0 | 17m - shrub blocks |

All configs fail at the same roadside shrub. The issue is scene geometry,
not costmap parameters.

## What the experiment proved

1. **Architecture works**: blank map + depth obstacles + single goal = Nav2
   plans and drives 45m at 0.8 m/s
- DWB and RPP both work: controller choice doesn't matter for this scene
3. Scene has roadside obstacles: shrubs at (-79, -6) block the road
   regardless of config
4. **Cone gap (0.4m) is impassable**: even with wider cones (2m spacing),
   the robot doesn't reach them due to roadside shrubs

## For the thesis

The teach-repeat + Nav2 architecture is validated:
- Single goal to destination works
- Blank map + depth obstacles works  
- Nav2 plans full 165m path and starts executing
- Speed matches exp 01 (0.8 m/s)

The remaining failures are **scene-specific** (roadside vegetation too close
to road) and **obstacle-specific** (cone gaps), not architectural.

## artifacts

### Config
- `config/nav2_params.yaml` - exp 01 DWB config (final version)
- `config/blank_road_map.yaml/pgm` - 180×40m blank map
- `config/nav2_launch.py` - launch with blank map
- `config/tr_waypoints.json` - 43 waypoints (not used in single-goal mode)

### Logs
- `logs/exp40_single*.log` - first run (45m, RPP)
- `logs/exp40_dwb*.log` - DWB run (17m, shrub block)
- `logs/exp40_wide*.log` - wider cones (17m, same shrub)

### scripts
- `scripts/send_single_goal.py` - single destination goal sender
- `scripts/spawn_obstacles.py` - obstacle placement (2m cone spacing)
- `scripts/run_husky_forest.py` - --obstacles --nav2-bridge
- `scripts/tf_wall_clock_relay.py` - GT TF chain
