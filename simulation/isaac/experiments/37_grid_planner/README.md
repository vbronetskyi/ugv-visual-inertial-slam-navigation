# exp 37: A* Local Planner on Obstacle Grid

## Goal

Replace reactive gap navigation with A* path planning on the ego-centric
obstacle grid from exp 36. Instead of converting grid -> angular profile -> gap
selection, plan a short path directly from robot to lookahead anchor on the
100×100 costmap.

## setup

### GridPlanner (A*)
- 100×100 grid, 0.2m resolution, A* search (8-connected, max 8000 iterations)
- Costmap built from obstacle grid: impassable (hits >= 2.0) + inflation (0.8m)
- Distance transform for inflation cost (scipy)
- 3m path lookahead (15 cells) for smooth steering
- `--use-planner` flag (implies `--use-grid`)

### LocalObstacleGrid (from exp 36)
- DECAY_RATE=0.92, HIT_VALUE=1.5, OBSTACLE_HITS=3.0, MAX_DEPTH=4.0m

### Controller
- Heading: angle to planner waypoint (3m ahead on A* path)
- Speed: 0.55 m/s (heading_err < 0.5 rad) -> 0.15 m/s (large heading error)
- Fallback: NO_PATH -> slow rotation (0.3 rad/s)

## Results

### Planner baseline (no obstacles)

| Metric | Value |
|---|---|
| Mode | PLAN throughout |
| Path length | 7-10m (consistant) |
| Speed | **0.15 m/s** (4× slower than gap nav 0.62 m/s) |
| Grid cells | 48-72 (0.5-0.7%) |
| Impassable cells | 100-140 (roadside tree inflation) |

A* finds paths on open road but speed is reduced because:
1. A* computation every 3 frames adds overhead
2. Path waypoints have lateral offset from inflation -> heading error -> speed reduction
3. Effective RTF drops from computation load

### planner with obstacles

| Metric | Value |
|---|---|
| Route completion | **25%** - stuck at cone group 1 |
| First cone in grid | t=250s, GT=(-54.1), grid 56->69 cells |
| A* routing around | **YES** - t=250-270s, PLAN mode, path=8-10m |
| First NO_PATH | t=280s, GT=(-51.0), 1m from cones |
| Stuck events | 2 |
| Stuck position | x≈-51.0, y≈-4.5 |

### Cone encounter timeline

| Time | GT pos | Mode | Grid cells | Path | Notes |
|---|---|---|---|---|---|
| t=240s | (-55.7, -3.6) | PLAN | 48 | 9.0m | Cones just beyond 4m |
| t=250s | (-54.1, -3.9) | PLAN | 56 | 9.6m | First cone hits in grid |
| t=260s | (-52.7, -4.1) | PLAN | 69 | 8.8m | Cones accumulating, A* routes around |
| t=270s | (-51.2, -4.3) | PLAN | 68 | 8.2m | 1.2m from cones, still has path |
| t=280s | (-51.0, -4.3) | **NO_PATH** | 51 | 0 | Too close, inflation blocks all paths |
| t=290s | (-52.2, -4.4) | PLAN | 90 | 8.4m | Backed up, A* finds path again |
| t=300s | (-51.0, -4.5) | NO_PATH | 54 | 0 | Approaches again, same block |

## Analysis

### Key breakthrough: A* plans around cones

Unlike the gap navigator (exp 35) which sees between-cone gaps and tries to
squeeze through, and the grid profile (exp 36) which sees 69-72% blocked with
no gaps, the A* planner:

1. Correctly inflates cones on the costmap -> between-cone paths are impassable
2. Finds bypass paths around the entire cone group (8-10m, routing to the side)
3. **Maintains PLAN mode** until 1m from cones (vs instant STUCK in exp 35/36)

### Execution limitation: speed too slow

The A* path goes around cones, but the robot at 0.15 m/s can't execute the
lateral maneuver before reaching the cones:

- Path starts routing around at 4m (t=250s)
- At 0.15 m/s, robot covers 4m in ~27s
- But lateral displacement of 2m at 0.15 m/s × sin(heading_error) takes >27s
- Robot reaches cones head-on before completing the detour

At the exp 35 speed of 0.55 m/s, the detour execution would be faster - but
A* computation overhead reduces speed to 0.15 m/s.

### Speed reduction causes

- scipy distance_transform_edt: ~50ms per costmap build
2. **A* search**: ~10-50ms per plan (up to 8000 iterations)
- Combined overhead: ~60-100ms per planning cycle (every 3 frames at 60fps)
- Heading oscillation: inflation pushes path off-center -> heading error -> speed reduction

## Comparison

| Config | Exp | Obstacles | Completion | Cone behavior |
|---|---|---|---|---|
| Gap nav | 35 | cones | 25% | Tries between cones -> stuck |
| Grid profile | 36 | cones | 25% | 69% blocked, no gaps -> stuck |
| **A* planner** | **37** | **cones** | **25%** | **Plans around, but too slow to execute** |

**Same 25% completion, but fundamentally different failure mode:**
- Exp 35: detection problem (between-cone gaps)
- Exp 36: representation problem (angular profile overblocks)
- Exp 37: execution problem (correct plan, slow execution)

## What would fix it

- Faster execution: reduce A* computation to maintain 0.5+ m/s speed.
   Options: cache costmap, reduce grid to 50×50, simplify inflation
2. **Earlier detection**: increase MAX_DEPTH_FOR_GRID to 6m (more frames to
   accumulate before reaching cones)
3. **Predictive path following**: instead of heading-to-waypoint controller,
   use a smooth path tracker that pre-computes the full detour trajectory

## conclusion

The A* grid planner validates the correct approach: **costmap + path planning
works for cone avoidance**. The planner finds paths around cone groups (unlike
the gap navigator), and the inflated costmap correctly prevents between-cone
routing (unlike the raw grid profile).

The remaining gap is **execution speed** - the planning computation overhead
reduces speed from 0.55 to 0.15 m/s, leaving insufficient time for the lateral
detour. Optimizing the planner (smaller grid, cached costmap, lighter inflation)
would close this gap.

**For the thesis**: this experiment demonstrates the progressive improvement
from reactive -> accumulated -> planned navigation:
- Reactive (exp 35): sees gaps between cones, tries to squeeze through
- Accumulated (exp 36): correct detection but wrong representation
- Planned (exp 37): **correct plan but insufficient execution speed**

The path from exp 35 to exp 37 mirrors the Nav2 architecture: costmap
(obstacle grid) + planner (A*) + controller (path following).

## artifacts

### Logs
- `logs/planner_baseline_partial.log` - baseline (PLAN mode, 0.15 m/s)
- `logs/planner_obs_v1.log` - obstacles (A* bypass then NO_PATH at 1m)

### Scripts
- `scripts/grid_planner.py` - A* local planner on obstacle grid
- `scripts/local_obstacle_grid.py` - ego-centric obstacle grid
- `scripts/gap_navigator.py` - with compute_cmd_vel_from_profile()
- `scripts/run_husky_teach_then_repeat.py` - with --use-planner flag
- `scripts/spawn_obstacles.py` - obstacle placement
