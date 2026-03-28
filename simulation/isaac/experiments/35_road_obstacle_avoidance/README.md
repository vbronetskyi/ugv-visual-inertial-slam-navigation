# Exp 35: Road Obstacle Avoidance

## Goal

Evaluate the teach-and-repeat pipeline on the road route with physical collision
obstacles (traffic cones + camping tent). The road environment (6-8m wide, flat
surface) eliminates the forest-specific problems identified in exp 34
(render/collision desync, dense vegetation blocking depth). Here, obstacles have
both render and collision meshes - depth sees them correctly.

**Research question**: Can depth-based gap navigation detect and bypass real
obstacles during route replay?

## setup

### route
- Road outbound: 84 anchors, 167m, from (-90.9, -5.5) to (70.4, -2.3)
- Pre-recorded in exp 20, loaded with `--skip-teach`
- Sinusoidal road: y varies from -5.5 to +2.0 accross x range

### Navigation pipeline
- **Heading**: GT-based (nearest anchor to GT position + 4-anchor lookahead ~8m)
- Obstacle avoidance: GapNavigator (OBSTACLE_THRESHOLD=2.0m, recovery cone ±69°)
- Odometry: Encoder (PhysX pose diff, 0.5% noise) + IMU gyro fusion (70/30)
- **Visual matching**: RobustAnchorLocalizer (cross-session ORB, weak on road)
- **Speed**: MAX_LINEAR=0.7 m/s (high-vis) or 0.55 m/s (low-vis, was 0.35)

### speed fix (v2)
Original baseline ran at 0.26 m/s due to two compounding reductions:
1. Low-vis mode (vis<0.4 -> MAX_LINEAR halved from 0.7 to 0.35)
2. Depth-based scaling (center depth ~2m -> another 0.4× reduction)

Fix: raised low-vis MAX_LINEAR to 0.55, skip depth scaling when blocked<10%
(open road). Result: 0.62 m/s average - 2.4× faster.

### Obstacles (spawned with `--obstacles` flag)
- 9 traffic cones: radius 0.3m, height 1.0m, orange, CollisionAPI
- 1 camping tent: body 2.0×1.8×1.4m, dark green, CollisionAPI
- Spawned at runtime via `spawn_obstacles.py`

### Obstacle placement (v2)

| Obstacle | X pos | Y positions | Route y | Distance from start |
|---|---|---|---|---|
| Cone group 1 (3) | -50 | -5.5, -4.5, -3.5 | -4.8 | 42m (25%) |
| Tent | -20 | 0.0 | 0.0 | 72m (43%) |
| Cone group 2 (3) | 15 | -3.0, -2.0, -1.0 | -2.0 | 108m (65%) |
| Cone group 3 (3) | 45 | -2.0, -1.0, 0.0 | -1.2 | 139m (83%) |

Each cone group has 3 cones at 1m spacing (2m span). Physical gap between
individual cones: 1m center-to-center minus 0.6m cone diameter = 0.4m.
Robot width = 0.67m - marginally fits between cones.

## results

### baseline v1 (no obstacles, original speed)

| Metric | Value |
|---|---|
| Route completion | **95%** (159m / 167m, anchor 79/83) |
| Duration | 595s (timed out at 600s) |
| Average speed | 0.26 m/s |
| Stuck events | 0 |
| Mode | ROUTE_TRACKING throught |

Limited by 600s timeout and 0.26 m/s speed.

### Baseline v2 (no obstacles, speed fix)

| Metric | Value |
|---|---|
| Route completion | **100%** (167m / 167m) |
| Duration | 270s |
| Final GT position | (70.4, -2.4) - at route endpoint |
| Average speed | **0.62 m/s** (2.4× faster than v1) |
| Stuck events | 0 |
| Mode | ROUTE_TRACKING throught |
| Blocked ratio | 0% |

Speed fix enabled robot to complete full route in 270s with 630s to spare.

### with obstacles v1 (cones at y=-5,-4,-3)

| Metric | Value |
|---|---|
| Route completion | **25%** (42m) - stuck at cone group 1 |
| Stuck events | 9 (all at x≈-50) |
| Stuck recovery | Ineffective - same location each time |

### with obstacles v3 (speed fix, improved stuck recovery)

| Metric | Value |
|---|---|
| Route completion | **25%** (42m) - stuck at cone group 1 |
| Duration | 140s (killed) |
| First cone detection | t=80s, blk=37%, GAP_MODE |
| First pass attempt | t=90s - squeezed between cones |
| First stuck | t=100s at (-50.4, -5.1) |
| Stuck events | 3 |
| Recovery behavior | 3s backup + alternating turn direction |

**Cone encounter timeline**:
- t=70s: Robot at (-54.3, -3.8), 4m from cones, blk=0%
- t=80s: GAP_MODE, blk=37%, 3 gaps found, sel=+3° (straight through)
- t=90s: GT=(-50.6, -4.9) - robot passed between cones! ROUTE_TRACKING
- t=100s: STUCK at (-50.4, -5.1) - pinned between cones
- t=110s: Recovery -> backup to (-51.0, -5.7), south of all cones
- t=120s: Still navigating around cones, GAP_MODE
- t=130+: Repeat stuck-recover cycle

## Analysis

### What worked
1. **Baseline: 100% route completion** at 0.62 m/s (speed fix)
2. **Obstacle detection**: Cones detected at 2-3m range (blk 37-44%)
3. **Gap selection**: Navigator found valid bypass gaps
4. Initial pass: Robot squeezed through gap between cones on first attempt
- Improved stuck recovery: Alternating turn directions, longer backup

### What failed
1. Cone gap too narrow: 0.4m physical gap vs 0.67m robot width
- Between-cone routing: Gap navigator selects gaps BETWEEN individual
   cones rather than AROUND the entire group
3. **Post-pass pinning**: Robot passes through gap but contacts cone on exit,
   gets pinned at y=-5.1 (between y=-5.5 and y=-4.5 cones)

### root cause: gap detection granularity

The gap navigator processes each obstacle pixel independently. Three cones
at 1m spacing appear as 3 seperate blocked regions with 2 gaps between them.
The inflation (robot-width erosion) should reject gaps narrower than the robot,
but at 2m depth the angular resolution makes the gaps appear just barely
passable - leading to the robot attempting to thread through rather than going
around.

**What's needed**: either morphological close to merge nearby obstacles into
single regions (tested - breaks on roadside vegetation), or a minimum gap
physical width check that rejects gaps < 1.3m (robot + safety margin).

### failed approaches
1. **Higher OBSTACLE_THRESHOLD (3.5m)**: Roadside trees at 2.5-3m caused 100%
   blocked on open road. Inflation kernel (proportional to threshold) over-
   inflated sparse edge obstacles.
2. Morphological close: Bridged roadside tree pixels into continuous wall.
   Center-only close still affected open road navigation.
3. **Early avoidance**: Narrow center strip (10% of image) still triggered
   false positives from near-center roadside objects.
4. **Physical width filter** (robot_width + 0.2m minimum): Correctly rejected
   narrow between-cone gaps (0.4m), but also rejected bypass gaps that are
   narrowed by roadside vegetation in the real depth image. Result: gaps=0/0
   -> STUCK immediately (worse than without filter).

## Comparison with Nav2

| Config | Exp | Obstacles | Completion | Speed | Method |
|---|---|---|---|---|---|
| Nav2 + GT | 01 | none | **100%** | - | planner + SLAM GT |
| Nav2 + SLAM | 13 | none | **87%** | - | planner + ORB-SLAM3 |
| Nav2 + SLAM | 28 | none | **51%** | - | planner (repro) |
| **T&R baseline v1** | **35** | **none** | **95%** | **0.26** | gap nav (slow) |
| **T&R baseline v2** | **35** | **none** | **100%** | **0.62** | gap nav (speed fix) |
| **T&R + obstacles** | **35** | **cones** | **25%** | **0.62** | gap nav + stuck recovery |

On open road: teach-and-repeat (100%) matches Nav2+GT and exceeds Nav2+SLAM.
With obstacles: gap navigator (25%) loses to any planner-based approach.

## conclusion

The teach-and-repeat pipeline with GT heading achieves **100% road completion
at 0.62 m/s** (baseline), validating the core navigation pipeline.

Obstacle avoidance with traffic cones fails at 25% due to:
1. Physical geometry: cone gaps (0.4m) vs robot width (0.67m)
2. **Detection granularity**: individual cones seen as separate obstacles with
   passable gaps between them, rather than one continuous obstacle group
3. **Late detection**: 2-3m range at 0.62 m/s = 3-5s reaction time

This demonstrates the fundamental limitation of **reactive** depth-based
navigation vs. planner-based approaches. Nav2 with a costmap inflates
obstacles into continuous regions and plans smooth paths 5-10m ahead. The gap
navigator only sees the current frame and makes instantaneous gap selections.

For the thesis: the 100% baseline proves the pipeline works. The obstacle
failure demonstrates where planner-based navigation (Nav2) would excel.

## Artifacts

### Logs
- `logs/road_noobs_baseline.csv` - baseline v1 CSV
- `logs/road_noobs_console.log` - baseline v1 console (95%, 0.26 m/s)
- `logs/road_noobs_v2_100pct.log` - baseline v2 console (100%, 0.62 m/s)
- `logs/road_obs_v1_stuck_at_cones.log` - obstacle v1 (9 stuck)
- `logs/road_obs_v2_stuck_at_cones.log` - obstacle v2 (3 stuck)
- `logs/road_obs_v3_speed_fix.log` - obstacle v3 with fixes (3 stuck)

### Plots
- `results/road_obstacle_trajectories.png` - GT trajectory comparison
- `results/road_obstacle_timeseries.png` - distance/blocked/error over time
- `results/nav2_comparison.png` - completion rate comparison

### Scripts
- `scripts/spawn_obstacles.py` - obstacle definitions and spawning
- `scripts/run_husky_teach_then_repeat.py` - with --obstacles flag, speed fix
- `scripts/gap_navigator.py` - v2: speed fix, OBSTACLE_THRESHOLD=2.0
- `scripts/robust_anchor_localizer.py` - visual anchor matching
- `scripts/route_follower.py` - pure pursuit route following
- `scripts/traversability_filter.py` - vegetation depth filter
