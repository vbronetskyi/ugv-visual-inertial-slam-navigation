# exp 39: Hybrid Navigator (Gap Nav + A* Planner)

## goal

Combine gap navigator speed (0.55 m/s) with A* planner intelligence (routes
around obstacles). CRUISE mode for open road, PLAN mode near obstacles.

## Setup

### HybridNavigator
- CRUISE: gap navigator at 0.55 m/s (reactive, fast)
- PLAN: A* on obstacle grid (costmap planner, routes around)
- Transition: grid obstacle_cells > 100 OR center depth < 2.5m -> PLAN
- Restore: grid obstacle_cells < 60 -> CRUISE

### Grid tuning (v3)
- DECAY_RATE=0.997 (half-life ~4s at 60fps) - was 0.93 (decayed instantly)
- OBSTACLE_HITS=8.0 (higher threshold with slower decay)
- MAX_DEPTH=5.0m, HIT_VALUE=1.5

## results

### Baseline (no obstacles)

| Metric | Value |
|---|---|
| Speed | **0.55-0.6 m/s** (CRUISE, matching gap nav) |
| Mode | CRUISE throught (brief PLAN from roadside, instant restore) |

### with obstacles

| Metric | Value |
|---|---|
| Route completion | **25%** - stuck at cone group 1 |
| CRUISE speed | 0.55 m/s until cones |
| First cone detection | t=70s, GT=(-53.0), raw_blk rises to 37% |
| GAP_MODE at cones | t=80s, gaps=3/3, sel=+25° (between cones) |
| First pass | **t=90s, GT=(-50.4, -3.9)** - through between-cone gap! |
| First stuck | t=100s at (-50.4, -3.9) - pinned on exit |
| Total stuck events | 3 |
| A* trigger | **Never** - grid didn't reach 100 cells at cones |

### Cone encounter timeline

| Time | GT | Mode | Speed | Event |
|---|---|---|---|---|
| t=60s | (-58.6, -2.7) | CRUISE | 0.63 | Approaching cones |
| t=70s | (-53.0, -4.1) | CRUISE | 0.56 | 3m from cones, raw_blk=37% |
| t=80s | (-51.8, -4.2) | GAP_MODE | slow | Cones detected, 3 gaps, sel=+25° |
| t=90s | (-50.4, -3.9) | ROUTE_TRACKING | - | **Passed through between cones!** |
| t=100s | (-50.4, -3.9) | ROUTE_TRACKING | 0 | **Stuck** - pinned on cone exit |
| t=110s | (-50.9, -3.2) | GAP_MODE | slow | Recovery, north of cones |
| t=130s | (-51.3, -3.0) | GAP_MODE | slow | Circling cone group |

## Analysis

### What worked: CRUISE speed
The hybrid maintained 0.55-0.6 m/s on open road - matching the original
gap navigator. This is a major improvement over the A* planner alone (0.15-0.21 m/s).

### what happened at cones: gap nav drove through
The A* planner NEVER triggered. The grid's high OBSTACLE_HITS threshold (8.0)
required many frames of accumulation, but the robot at 0.55 m/s reached the
cones before the grid confirmed them. The gap navigator handled the encounter
in CRUISE mode - same as exp 35.

The POSITIVE: at 0.55 m/s (vs 0.26 in exp 35), the robot had more momentum
and passed through the between-cone gap at t=90s before getting stuck. In
exp 35, the robot got stuck immediately.

### Why A* didn't trigger
Grid accumulation timeline at 0.55 m/s:
- Cones visible in depth at ~3m (x=-53) -> t=70s
- At 0.55 m/s, robot reaches cones in ~5s (t=75)
- Each depth frame adds 1.5 hit, 20 frames/sec, ~100 frames in 5s
- But cone pixels are sparse (3 cones × ~5px each at 3m) -> few cells hit
- OBSTACLE_HITS=8.0 needs many frames hitting the SAME cell
- Grid decays at 0.997^100 = 0.74 per second - hits accumulate but slowly
- By t=75 (at cones), grid has ~50-60 confirmed cells (below 100 trigger)

### Trade-off: detection speed vs false positives
- OBSTACLE_HITS=4.0 (exp 38): detects faster but roadside false positives
- OBSTACLE_HITS=8.0 (this exp): no false positives but too slow for cones
- Need: object-specific thresholds or distance-dependent confirmation

## Comparison

| Exp | Speed | Cone detection | A* trigger | Result |
|---|---|---|---|---|
| 35 (gap nav) | 0.55 | 2-3m, GAP_MODE | N/A | Between cones -> stuck |
| 37 (A* only) | 0.15 | 4m, PLAN | Yes | Around cones -> too slow |
| 38 (fast A*) | 0.21 | 4m, PLAN | Yes | Around cones -> still slow |
| **39 (hybrid)** | **0.55** | **3m, GAP_MODE** | **No** | **Through gap -> stuck on exit** |

## conclusion

The hybrid navigator achieves CRUISE speed (0.55 m/s) on open road, matching
the original gap navigator. However, the A* planner never triggers because
the obstacle grid doesn't accumulate fast enough before the fast-moving robot
reaches the cones.

The fundamental tension: faster robot = less time for grid accumulation = gap
nav handles the encounter instead of A*. The A* planner works when the robot
is slow (exp 37-38) but the robot needs to be fast for effective lateral
maneuvers.

**For the thesis**: this experiment demonstrates the speed/detection trade-off
in hybrid navigation. Matching the gap between reactive and planner-based
approaches requires either much earlier detection (10m+) or predictive obstacle
awareness (map-based, like Nav2's global costmap).

## Artifacts

### logs
- `logs/hybrid_baseline_partial.log` - CRUISE baseline (0.55 m/s)
- `logs/hybrid_obs_v3.log` - obstacles (through gap then stuck)

### scripts
- `scripts/hybrid_navigator.py` - CRUISE/PLAN mode hybrid
- `scripts/fast_grid_planner.py` - cached A* planner
- `scripts/local_obstacle_grid.py` - tuned grid (DECAY=0.997, HITS=8)
- `scripts/run_husky_teach_then_repeat.py` - with --use-hybrid flag
