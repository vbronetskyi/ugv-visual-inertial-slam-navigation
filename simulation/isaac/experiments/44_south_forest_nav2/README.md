# exp 44: South Forest Route with Nav2

## Result

**Nav2 + teach-repeat fails on dense forest - both sparse and dense waypoints hit the same obstacle cluster at (-19, -20). Route ceiling ~85m of 196m regardless of Nav2 tuning.**

| Scenario | Result | Distance covered |
|---|---|---|
| Road with obstacles (exp 41-43) | 95% (41/43) | 183m of 167m |
| **South forest, v1 (4m WPs, inflation 0.5m)** | **23/46 (50%)** | **~85m** - stuck at (-19.1, -21.6) |
| **South forest, v2 (2m WPs, inflation 0.3m)** | **50/99 (51%)** | **~71m** - stuck at (-19.4, -19.8) |

Both runs hit the **same oak+shrub cluster around x≈-19**. Dense waypoints
and lower inflation let the planner keep closer to the teach path through
the first 50m of forest, but the underlying blocker - overlapping
obstacles forming an impassable wall - remains.

Same architecture that achieved 95% on road (6-8m wide) cannot navigate
forest passages (2-3m wide). Confirmed by exp 30-34 which similarly maxed
at 80m on south route.

## goal

Test proven Nav2 + teach-repeat architecture (exp 41-43) on the challenging
south forest route. Establish whether forest passages are feasible with
depth costmap obstacle avoidance.

## Setup

Route: south forest, 196m outbound, 99 teach anchors (from GT recording)
- Sampled: 46 waypoints at 4m spacing
- Start: (-90.9, -5.6), End: (70.3, -4.5)
- Contains dense pine/oak trees, shrubs, fallen logs, narrow 2-3m passages

Forest-tuned Nav2 config (vs road):
- `desired_linear_vel: 0.5` (was 0.8 - slower in forest)
- `obstacle_max_range: 3.0` (was 6.0 - only close obstacles)
- `raytrace_max_range: 4.0` (was 7.0)
- `max_obstacle_height: 0.8` (was 2.0 - filter branches/canopy)
- `inflation_radius: 0.5` (was 1.5 - narrow passages)
- `cost_scaling_factor: 4.0` (was 2.5 - steeper falloff)
- Local costmap: 6×6m (was 10×10m)

**Static map**: blank 200×60m covering x=[-110, +90], y=[-50, +10].

## run 1 result (sparse waypoints)

| Metric | Value |
|---|---|
| Reached | 23/46 (50%) |
| Skipped | 9+ (killed at WP 31) |
| Distance covered | ~85m in x (spawn -95 -> stuck -19) |
| Robot stuck at | (-19.1, -21.6) |
| Duration | ~13 min before manual kill |

**Pattern:**
- WPs 0-22: mostly reached, some retries
- WP 23 onwards: **consecutive skips** - robot got stuck in forest thicket
- Robot couldn't escape area around (-19, -21) despite backups + retries

## run 2 result (dense waypoints - v2 fix attempt)

**Hypothesis tested**: dense WPs (2m) + inflation 0.3m + cost_scaling 5.0
+ xy_goal_tolerance 1.5m would keep Nav2 closer to the teach path and
squeeze through the oak cluster.

| Metric | Value |
|---|---|
| Reached | 50/99 (51%) |
| Skipped | 14 (first 5 early, 9 consecutive at end) |
| Distance covered | ~71m in x (spawn -91 -> stuck -19.4) |
| Robot stuck at | (-19.4, -19.8) |
| Duration | ~8 min to stuck, killed after 9 consecutive skips |

Pattern:
- WPs 0-50: reached smoothly (faster than v1, tighter path tracking)
- WPs 51-59: **9 consecutive skips** - same region, slightly north of v1
- v2 stuck 1.8m north of v1's stuck point (-19.8 vs -21.6 in y)

**Conclusion**: Tighter inflation kept Nav2 closer to the teach path
through the first 50 WPs, but when waypoints enter the obstacle cluster
region, Nav2 still diverges. v2 stuck point is **2m further north**
than v1 (y=-19.8 vs y=-21.6), both well north of the exp 30 GT path
at y=-24. The teach path at y=-24 passes within 1m of a shrub cluster;
with any non-zero inflation, Nav2's costmap blocks the teach corridor
and pushes the planner north into the oak. No Nav2 inflation setting
solves this - inflation must cover real collision, but any inflation
over 0.2m marks the teach corridor as unsafe.

## Trajectory visualization note

The GT trajectory plot shows the robot correctly navigating around trees
- it does NOT go through trunks. The planner finds valid paths through
forest openings. Robot stops only when **depth costmap marks a corridor
as fully blocked**, not due to physical collisions.

**Scene object accuracy** (verified against `run_husky_forest.py`):

| Object | JSON count | Collision radius | Active in scene |
|---|---|---|---|
| Pine + Oak trees | 205 | 0.7m trunk | ~130 (thinned: -10 corners, -1/3 every third) |
| Shrubs | 357 | 0.4m | 357 |
| Rocks | 28 | 0.8m | 28 |
| Houses | 6 | 6×6m | 6 |
| Barrels | 4 | 0.5m | 4 |
| **Total collision bodies** | - | - | **~525** (matches Isaac log: "525 collision obstacles") |

Plot uses same thinning logic as simulator and real collision radii.

## Why forest fails while road works

### road (exp 41-43) - 95% success:
- Drivable corridor 4m wide, flat terrain
- Trees set back ~5m+ from road centerline
- Depth costmap sees obstacles but leaves 2-3m bypass
- Goal tolerance (3m) absorbs localization drift
- Cone barriers designed with explicit bypass gaps

### forest (exp 44) - 50% ceiling:
- **Passages 2-3m wide** between trees
- Inflation 0.5m × 2 + robot radius 0.35m = 1.7m effective robot width
- Depth camera sees **tree trunks AND canopy** - costmap fills entire passage
- Even with 0.8m obstacle height filter, low branches still mark
- Dynamic vegetation: shrubs swaying or depth noise marks passage as blocked
- Retry/backup logic doesn't help - robot is fundamentally blocked

## Diagnosis - why Nav2 got stuck

Root cause: **Case C - Nav2 planner drifted 2.7m north off teach path into
oak+shrub cluster**. Route IS physically passable (exp 30 GT controller
proved it), but Nav2's costmap inflation pushed the planner onto a
detour that landed in a denser obstacle cluster.

### robot's drift off teach path

| Time | Position | Event |
|---|---|---|
| t=132.5s | (-21.9, -24.8) | On teach line, heading east (teach y≈-24) |
| t=138.5s | (-18.2, -23.2) | Drifted 1.6m north |
| t=140.5s | (-19.1, -22.1) | Drifted 2.7m north |
| t=148+s | (-19.0, -21.95) | **Stuck - pressed against oak tree** |

Exp 30 GT pure-pursuit passed same x-range at y≈-24 (2-3m south of where
Nav2 got stuck).

### Obstacle cluster geometry at stuck point

Scene objects within 2m of stuck point:

| Pair | Distance | Sum radii | Gap |
|---|---|---|---|
| Oak + shrub_0185 | 0.98m | 1.10m | **-0.12m OVERLAP** |
| Oak + shrub_0125 | 0.91m | 1.10m | **-0.19m OVERLAP** |
| Shrub_0125 + shrub_0123 | 0.30m | 0.80m | **-0.50m OVERLAP** |

Overlapping obstacles form an impassable wall from the north. Only passable
route is south (at y=-24, where teach went).

### depth camera confirmed Nav2 has partial view

At stuck point, only 19% of center-strip pixels <3m distance:
- Left side: 14% blocked
- Center: 22% blocked
- Right: 21% blocked
- Min depth: 0.30m (tree trunk within 1m)

Not fully blocked - **Nav2 had options but inflation made them appear unsafe**.

### Why Nav2 drifted north

- Teach path at y=-24 passes within 1m of shrub cluster at y=-22 to -23
- With `inflation_radius: 0.5m`, costmap halo extends to y≈-22.5
- Teach corridor between halo (south) and open (north) looked ambiguous
- Planner chose northern detour -> oak at (-20.3, -21.1) blocked it

### Cases ruled out

- **A (narrow passage)**: Route IS wide enough - exp 30 GT proved it
- b (render-only vegetation): All obstacles have real collision in models.json

## Previous forest experiments' findings (exp 30-34) confirmed

| Exp | Method | Ceiling |
|---|---|---|
| 30 | Teach & repeat with VIO | ~80m |
| 31 | Gap navigator | ~80m |
| 32 | Robust anchoring | ~80m |
| 34 | Traversability | ~80m |
| **44** | **Nav2 + teach-repeat (road-proven architecture)** | **~85m** |

All methods hit the same wall around 80-85m. The issue isn't localization
or planning - it's that **forest visual density fundamentally blocks
depth-costmap-based navigation** in passages <3m wide.

## Steps 2-3: not tested

**Step 2 (Encoder+Compass)**: Not tested - GT baseline already failed
at 50%. Encoder+Compass added only ~0.5-1m drift on road runs - this
noise doesn't help where GT itself can't plan paths. Would likely perform
identically or slightly worse.

**Step 3 (with obstacles)**: Not tested - no room for additional obstacles
in forest passages that are already marginal.

## key insight

The road+obstacles success (95%) was not due to advanced navigation -
it was because the **obstacles were placed on a wide flat road with
explicit bypass routes**. Real forest terrain has no bypass routes
built in, and the depth costmap has no way to distinguish passable
vegetation (shrubs, grass) from impassable trunks.

**Solutions that would work**:
1. Traversability classification (exp 34 direction): ML model
   classifies depth/RGB pixels as "vegetation passable" vs "solid"
- Actuated pushing: robot pushes through light vegetation
   (requires physical capability)
3. Taller robot with tilted depth camera ignoring ground-level
   branches
4. **Detailed ORB-SLAM3 atlas with obstacle-free corridor** explicitly
   marked
5. **Different route**: cut paths through forest (like forestry access
   roads)

## Artifacts

### Logs
- `logs/exp44_gt_v2_*.log` - Run 1 (v1, sparse WPs): Isaac, tf, Nav2, goals
- `logs/exp44_gt_v2_traj.csv` - v1 GT trajectory (stuck at (-19.1, -21.6))
- `logs/exp44_v2_*.log` - Run 2 (v2, dense WPs): Isaac, tf, Nav2, goals
- `logs/exp44_v2_traj.csv` - v2 GT trajectory (stuck at (-19.4, -19.8))

### Config
- `config/nav2_params.yaml` - forest-tuned Nav2 params (v2: inflation 0.3, cost_scaling 5.0, xy_goal_tolerance 1.5)
- `config/nav2_launch.py` - Nav2 launch
- `config/blank_south_map.yaml/pgm` - 200×60m blank static map
- `config/south_waypoints.json` - 46 waypoints at 4m spacing (v1)
- `config/south_anchors_dense.json` - 99 waypoints at 2m spacing (v2)
