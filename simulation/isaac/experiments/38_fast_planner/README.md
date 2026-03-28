# exp 38: Optimized A* Planner

## Goal

Optimize A* planner from exp 37 to maintain higher speed. Exp 37 proved A*
plans correctly around cones but robot moved at 0.15 m/s (too slow to execute
the lateral detour). Target: >0.4 m/s, bypassing cone group 1.

## Optimizations

| Optimization | Exp 37 | Exp 38 |
|---|---|---|
| Planner engine | GridPlanner (scipy) | FastGridPlanner (scipy + cache) |
| Replan frequency | Every 3 frames | Every 10 frames (cached) |
| A* max iterations | 8000 | 3000 + early termination |
| Planning time | 60-100ms | **3-9ms** |
| Speed controller | Binary (0.55/0.15) | 3-tier (0.6/0.4/0.2) |

### Key finding: scipy is already fast

Benchmark showed scipy distance_transform takes ~1ms - the bottleneck was NOT
costmap building but cumulative overhead from planning every 3 frames. Caching
(replan every 10 frames) was the main improvement.

## results

### Baseline (no obstacles)

| Metric | Value |
|---|---|
| Mode | PLAN throught |
| Speed | **0.21 m/s** (1.4× faster than exp 37's 0.15) |
| Planning time | 3-9ms (vs 60-100ms in exp 37) |
| Grid cells | 56-77 (0.5-0.8%) |

Speed improved but not to target 0.4+. Remaining bottleneck: A* path has
lateral offset from roadside inflation -> heading error -> speed reduction.

### With obstacles

| Metric | Value |
|---|---|
| Route completion | **25%** - stuck at cone group 1 |
| A* routing phase | t=180s: PLAN, path=10.2m (routing around cones from 3m) |
| NO_PATH | t=190s: 1m from cones, A* can't find path |
| Stuck | t=200s at (-51.0, -4.3) |
| Recovery | t=210s: PLAN restored at (-51.9), path=9.0m |
| Planning time | 3-6ms during approach, 0.3ms for NO_PATH (instant fail) |

### cone encounter timeline

| Time | GT pos | Mode | Path | Plan ms | Notes |
|---|---|---|---|---|---|
| t=170s | (-55.4, -3.6) | PLAN | 7.2m | 4.4 | Approaching cones |
| t=180s | (-52.9, -4.0) | PLAN | 10.2m | 5.7 | 3m from cones, A* routes around |
| t=190s | (-51.0, -4.3) | NO_PATH | 0 | 0.3 | 1m from cones, blocked |
| t=200s | (-51.0, -4.3) | NO_PATH | 0 | 0.3 | Stuck at cones |
| t=210s | (-51.9, -4.2) | PLAN | 9.0m | 3.4 | After recovery, path restored |

**Same pattern as exp 37**, just faster planning and slightly faster robot.

## Analysis

### speed is NOT the bottleneck - detection range is

| Factor | Exp 37 | Exp 38 | Impact |
|---|---|---|---|
| Planning time | 60-100ms | 3-9ms | 10× faster, minimal speed impact |
| Robot speed | 0.15 m/s | 0.21 m/s | 40% faster, not enough |
| Detection range | 4m | 4m | **Unchanged - still the bottleneck** |
| Lateral displacement | 1.1m in 27s | 1.5m in 19s | Need 2m, still insufficient |

The robot detects cones at 4m (MAX_DEPTH_FOR_GRID=4.0). At 0.21 m/s, it
covers 4m in 19s. The A* path goes around cones but the robot only manages
~1.5m lateral displacement before reaching the 1m NO_PATH zone.

**Required**: either detect cones at 6-8m (longer detection range) or
maintain 0.4+ m/s speed (more lateral displacement in the available distance).

### why speed stays at 0.21 m/s despite fast planning

The planning time dropped from 60ms to 4ms, but the simulation frame rate
didn't increase proportionally. The remaining overhead is:
1. Grid update (depth projection): ~5ms per frame
2. Isaac Sim rendering: ~16ms per frame (60fps cap)
3. Heading oscillation from inflation offset -> speed capped at 0.4 tier

The planner optimization was correct but the speed improvement was marginal
because most frame time is spent on rendering, not planning.

## Comparison accross experiments

| Exp | Approach | Plan time | Speed | Cone detection | Cone bypass |
|---|---|---|---|---|---|
| 35 | Gap nav (reactive) | 0ms | 0.55 m/s | 2-3m, 3 gaps | Tries between -> stuck |
| 36 | Grid profile | 0ms | 0.55 m/s | Accumulated, 69% blocked | No gaps -> stuck |
| 37 | A* planner | 60-100ms | 0.15 m/s | 4m, plans around | Too slow -> stuck at 1m |
| **38** | **A* cached** | **3-9ms** | **0.21 m/s** | **4m, plans around** | **Still too slow -> stuck at 1m** |

## Conclusion

Planning optimization (10× faster: 60ms -> 4ms) provides marginal speed
improvement (0.15 -> 0.21 m/s) because the rendering pipeline, not the planner,
dominates frame time. The A* planner correctly routes around cones but the
robot can't execute the detour before reaching the cones.

**The limiting factor is detection range (4m), not planning speed.**

Next step would be increasing MAX_DEPTH_FOR_GRID to 6-8m, but this was
already shown in exp 36 to cause over-accumulation of roadside trees (72%
blocked). The trade-off: longer detection range = more roadside noise.

**For the thesis**: this experiment isolates the performance bottleneck.
The planner (A*) is fast enough (4ms). The robot speed (0.21 m/s) is limited
by rendering overhead, not planning. The detection range (4m for cones) is
the fundamental constraint on obstacle avoidance effectiveness.

## artifacts

### Logs
- `logs/fast_baseline_partial.log` - baseline (PLAN, 0.21 m/s, 4ms)
- `logs/fast_obs_v1.log` - obstacles (A* bypass, NO_PATH at 1m)

### Scripts
- `scripts/fast_grid_planner.py` - cached A* planner
- `scripts/local_obstacle_grid.py` - ego-centric obstacle grid
- `scripts/run_husky_teach_then_repeat.py` - with --use-planner flag
- `scripts/spawn_obstacles.py` - obstacle placement
