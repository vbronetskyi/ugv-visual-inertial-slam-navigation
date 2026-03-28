# Exp 36: Local Obstacle Grid

## goal

Replace single-frame reactive depth detection with an ego-centric obstacle
grid that accumulates depth hits over time. Expected benefits:
1. Earlier detection (5-6m vs 2-3m from single frame)
2. Cone groups merge into continuous obstacles
3. Stable obstacle representation

## Setup

### LocalObstacleGrid
- 20m × 20m grid, 0.2m resolution = 100×100 cells
- Robot always at grid center, grid shifts with movement
- Depth projected into grid cells every 4th column
- Hits decay per frame, confirmed at OBSTACLE_HITS treshold
- Ray-march from center -> obstacle profile for gap navigator

### Integration
- `--use-grid` flag in run_husky_teach_then_repeat.py
- Grid updated every DEPTH_EVERY frames with GT pose + filtered depth
- Profile passed to `gap_nav.compute_cmd_vel_from_profile()`
- Replaces single-frame `compute_cmd_vel()` when grid enabled

### Configurations tested

| Version | DECAY | HIT_VALUE | OBSTACLE_HITS | MAX_DEPTH | SECTORS |
|---------|-------|-----------|---------------|-----------|---------|
| v1 | 0.95 | 1.0 | 3.0 | 6.0m | 40 |
| v2 | 0.92 | 1.5 | 3.0 | 4.0m | 80 |

## Results

### grid baseline (no obstacles)

| Metric | Value |
|---|---|
| Mode | ROUTE_TRACKING throughout |
| Blocked ratio | 0% |
| Grid cells | 55-71 (0.5-0.7%) |
| Speed | 0.56 m/s (same as non-grid baseline) |

Grid correctly accumulates roadside objects without blocking the road.
Baseline navigation unaffected. (Partial run - killed after confirming.)

### Grid obstacles v1 (DECAY=0.95, MAX_DEPTH=6m, 40 sectors)

| Metric | Value |
|---|---|
| Route completion | **25%** - stuck at cone group 1 |
| Cone encounter | t=80s, blk=72%, gaps=0/0 |
| Grid cells | 67 (0.7%) |
| Failure mode | All sectors blocked, no valid gaps |

### Grid obstacles v2 (DECAY=0.92, MAX_DEPTH=4m, 80 sectors)

| Metric | Value |
|---|---|
| Route completion | **25%** - stuck at cone group 1 |
| Cone encounter | t=80s, blk=69%, gaps=0/0 |
| Grid cells | 66 (0.7%) |
| Failure mode | Same - all sectors blocked |

## analysis

### why the grid doesn't help

The grid correctly accumulates obstacle detections, but the **obstacle profile
semantics differ from single-frame depth**:

Single-frame depth profile (640 pixels):
- Most road pixels show depth=5m (far roadside trees) -> blk=0%
- Only pixels directly at obstacles show depth<2m -> blk=30-44%
- Many pixels between obstacles show far depth -> gaps found

**Grid obstacle profile** (40-80 sectors):
- Each sector ray-marches outward from robot center
- Hits the first confirmed obstacle cell in that direction
- Roadside trees accumulated from many frames -> confirmed at 2-3m in many sectors
- Result: 69-72% of sectors hit an obstacle -> blk=69-72%
- Remaining 28-31% is not enough for gaps after inflation

The grid reports the nearest confirmed obstacle per direction, which is
fundamentally different from "percentage of depth pixels blocked in this
column". The gap navigator was designed for the latter.

### roadside tree accumulation

The core issue: roadside trees at 2.5-3.5m are visible from many frames as
the robot passes. The grid accumulates these into confirmed obstacles. When
ray-marching, sectors pointing toward roadside hit these obstacles at 2-3m.
Combined with cone sectors, this fills 69-72% of the profile.

With single-frame depth, roadside trees are in the **edge columns** of the
640-pixel image. The center is clear. With the grid profile, sectors span
the full FOV uniformly, and a 2m roadside tree at any angle blocks that sector.

### What would work

To use the grid effectively, the gap navigator would need:
1. **Higher OBSTACLE_THRESHOLD** for grid profiles (e.g., 1.0m instead of 2.0m)
   - only react to very close obstacles
2. **Grid-specific gap finding** that considers distance to obstacle, not just
   whether an obstacle exists
3. **Separate roadside vs. obstacle classification** in the grid
4. **Path planning** (A* or similiar) on the grid instead of angular profile

Approach #4 (path planning on grid) is essentially what Nav2 does with its
costmap - the grid IS the costmap, and a planner (MPPI/DWB) finds paths
through it. Reimplementing this is out of scope for this thesis.

## Comparison

| Config | Obstacles | Completion | Detection |
|---|---|---|---|
| Single-frame (exp 35) | cones | 25% | 2-3m, 3 gaps, tries between cones |
| Grid v1 | cones | 25% | 72% blocked, 0 gaps |
| Grid v2 (tuned) | cones | 25% | 69% blocked, 0 gaps |

The grid doesn't improve obstacle avoidance - it makes it worse by over-blocking
from accumulated roadside trees.

## conclusion

The ego-centric obstacle grid successfully accumulates depth detections but
cannot be effectively used with the angular gap navigator because:

1. Grid profile semantics (nearest obstacle per direction) differ from depth
   profile semantics (percentage of blocked pixels per column)
2. Roadside vegetation accumulates into confirmed obstacles, filling most
   sectors
3. The gap navigator needs a costmap planner (A*) to work with the grid,
   which is equivalent to reimplementing Nav2's approach

For the thesis: this experiment validates that accumulated obstacle
detection is the right direction (as used by Nav2's costmap), but the
reactive gap navigator cannot exploit it. The gap between our reactive
approach and Nav2's planner-based aproach lies not in detection but in
planning - the ability to find collision-free paths through a known
obstacle field.

## Artifacts

### logs
- `logs/grid_baseline_partial.log` - baseline with grid (partial, confirmed OK)
- `logs/grid_obs_v1.log` - obstacles with grid v1 (72% blocked)
- `logs/grid_obs_v2_tuned.log` - obstacles with grid v2 tuned (69% blocked)

### Scripts
- `scripts/local_obstacle_grid.py` - ego-centric obstacle grid
- `scripts/gap_navigator.py` - with compute_cmd_vel_from_profile()
- `scripts/run_husky_teach_then_repeat.py` - with --use-grid flag
- `scripts/spawn_obstacles.py` - obstacle placement
