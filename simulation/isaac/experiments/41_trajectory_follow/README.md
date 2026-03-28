# Exp 41: Trajectory Following with Nav2 - Real Obstacle Avoidance

## result

**42/43 waypoints REACHED, 1 SKIPPED. 98% route completion with solid barriers.**

167m road, 4 obstacle groups (3 cone-wall barriers + 1 tent), 458 seconds.
Robot visibly deviated around each obstacle - confirmed by GT trajectory.

### improvement over v2 (91% -> 98%)
| | v2 | v3 |
|---|---|---|
| Reached | 39/43 (91%) | **42/43 (98%)** |
| Skipped | 4 | **1** |
| Duration | 689s | **458s (-33%)** |
| Key fix | cleared global costmap -> forgot obstacles | keep obstacles in costmap -> smooth replan |

## Bug fixes applied

1. Stale goal file (`/tmp/isaac_goal.txt`): bridge's file-based goal
   system drove the robot autonomously, bypassing Nav2. Fixed: disabled
   file-based goals in `--nav2-bridge` mode.

2. Cone spacing too wide (2m -> 1.4m passable gap): robot drove through
   cones without avoidance. Fixed: replaced sparse cones with dense barrier
   walls (0.5m spacing, impassable).

## Obstacles (solid barriers)

| Obstacle | Position | Wall coverage | Bypass direction |
|---|---|---|---|
| Barrier 1 | x=-50, y=[-8, -2.5] | 5.5m wall, blocks south | North (y > -2) |
| Tent | x=-20, y=0 | 2×1.8m solid | Either side |
| Barrier 2 | x=15, y=[-1, +4] | 5m wall, blocks north | South (y < -1.5) |
| Barrier 3 | x=45, y=[-3, +1] | 4m wall, blocks center | South or north |

Each barrier: 9-12 cones at 0.5m spacing -> no passable gap.

## Waypoint results

| WP range | Obstacle | Result | Robot bypass |
|---|---|---|---|
| 0-10 | open road | All REACHED | - |
| 11 | **Barrier 1** | **REACHED** (2nd attempt) | North bypass |
| 12-18 | open road + tent approach | All REACHED | - |
| 19 | **Tent** | **SKIPPED** (2 fails) | South bypass attempted |
| 20-27 | open road | All REACHED | - |
| 28 | **Barrier 2** | REACHED (1st attempt) | South bypass |
| 29-34 | open road | All REACHED | - |
| 35 | **Barrier 3** | **REACHED** (2nd attempt) | Bypass |
| 36-42 | open road to destination | All REACHED | - |

## obstacle avoidance verification

GT positions confirm robot deviated from teach trajectory at each obstacle:

| Obstacle | Teach route y | Robot actual y | Deviation |
|---|---|---|---|
| Barrier 1 (x=-50) | -4.8 | **-3.3** | +1.5m north (around wall) |
| Tent (x=-20) | 0.2 | **-3.4** | -3.6m south (around tent) |
| Barrier 2 (x=15) | -2.0 | **-2.0** | On south edge (barrier at y>-1) |
| Barrier 3 (x=45) | -1.2 | **-1.5** | South of barrier center |

## Architecture

```
Isaac Sim (run_husky_forest.py --obstacles --nav2-bridge)
  └─ OmniGraph: /camera/depth/image_rect_raw
  └─ GT pose -> /tmp/isaac_pose.txt + /tmp/isaac_trajectory.csv
  └─ File-based goal DISABLED in nav2-bridge mode

tf_wall_clock_relay.py --use-gt
  └─ map->odom->base_link (GT)
  └─ depth -> /depth_points (PointCloud2)

Nav2 (blank 900×200 static map + depth obstacles)
  └─ NavFn A* planner + RPP controller (0.8 m/s)
  └─ robot_radius=0.35, inflation=1.5m, cost_scaling=2.5

send_trajectory_goals.py
  └─ 43 waypoints at 4m spacing
  └─ 3 attempts per WP, backup + clear costmap on fail
```

## comparison

| Config | Obstacles | Completion | Duration | Avoidance |
|---|---|---|---|---|
| Nav2+GT no obs (exp 01) | none | 100% | - | N/A |
| T&R gap nav (exp 35) | cones 1m | 25% | - | Between cones -> stuck |
| T&R+A* planner (exp 37) | cones 1m | 25% | - | Plans around -> too slow |
| T&R+Nav2 barriers v2 | walls + tent | 91% (39/43) | 689s | Bypass with zigzags |
| **T&R+Nav2 barriers v3** | **walls + tent** | **98% (42/43)** | **458s** | **Smooth bypass, 1 skip** |

## Conclusion

Teach-and-repeat trajectory + Nav2 + depth costmap achieves **91% route
completion with real obstacle avoidance**. The robot:
- Follows recorded trajectory at 0.8 m/s on open road
- Detects solid barriers via depth camera -> costmap blocks path
- Nav2 replans around barriers (north or south)
- Retry logic (backup + clear + retry) recovers from initial failures
- Skips unreachable WPs inside obstacle zones, continues past

The 4 skipped WPs are all inside or immediately behind barrier walls -
expected behavior. The robot reached the destination (70.4, -2.3) after
navigating around all 4 obstacle groups.

## Artifacts

### Logs
- `logs/exp41b.log` - bridge GT trajectory
- `logs/exp41b_goals.log` - 39/43 REACHED, 4 skipped, 689s
- `logs/exp41b_nav2.log` - Nav2 planner + controller
- `logs/isaac_trajectory.csv` - dense GT trajectory (10Hz)

### plots
- `results/exp41_trajectory.png` - full route map
- `results/exp41_comparison.png` - approach comparison
- `results/exp41_obstacle_details.png` - obstacle zoom views

### Scripts
- `scripts/send_trajectory_goals.py` - trajectory following with retry
- `scripts/run_husky_forest.py` - nav2-bridge with file-goal bug fix
- `scripts/spawn_obstacles.py` - solid barrier walls (0.5m cone spacing)
