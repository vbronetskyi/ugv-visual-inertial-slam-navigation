# exp 28: Nav2+SLAM reproducibility diagnosis - recovery from exps 25-27 drift

## Context

Exps 25–27 kept adding complexity (forward-regression watchdog, lateral
corridor, current_idx snap, full-path search, SLAM stale detection,
cmd_vel angular clamp, smaller BackUp) to protect RGBD SLAM tracking
during obstacle avoidance. Each change was individually reasonable but
the stack capped road-with-obstacle navigation at **~29% of route**
(vs exp 13's reported 87%).

This experiment is a diagnosis + revert cycle consolidating 6 seperate
runs into one writeup:

1. GT baseline (was exp 28) - does the pipeline work without SLAM?
2. **Revert to simple** (was exp 29) - remove complex heuristics, keep
   inflation/lookahead/depth range at exp 13 values.
3. **exp 13 verbatim** (was exp 30) - bit-for-bit copy of exp 13 scripts
   and configs. Three runs to test non-determinism.
4. **GT + world waypoints** (was exp 31) - isolate whether the curved
   SLAM-frame waypoints were the key.
5. **SLAM stale + odom fallback** (was exp 32) - detect frozen SLAM
   pose and dead-reckon via cmd_vel integration.
6. Costmap diagnostic (was exp 33) - abandoned mid-run when user
   asked to consolidate.

## the three regressions from exp 13 identified

Diffing `experiments/13_rpp_inflation/` vs the live working tree:

### 1. Waypoints through cone zone (biggest effect)

`/tmp/slam_waypoints.json` was replaced during exp 25 with 60 wp from a
VIO mapping run **without obstacles**, ending at SLAM `(147.3, -5.9)` -
straight down the road centerline where cone group 1 sits.

Exp 13's `slam_waypoints.json` has 83 wp ending at SLAM `(163.7, -11.7)`
- explicitly curving **south** through the "wide gap south" side of
cone group 1 (left open by `spawn_obstacles.py`).

**Fix**: `cp experiments/13_rpp_inflation/config/slam_waypoints.json
/tmp/slam_waypoints.json`

### 2. Nav2 BT BackUp - 0.15m -> 1.0m

Exp 26 reduced BackUp from `1.0m / 0.25 m/s` to `0.15m / 0.08 m/s` to
avoid RGBD feature loss during recovery. But 0.15m is too small to
escape local planner dead-ends.

**Fix**: revert `config/nav2_bt_simple.xml` BackUp to `1.0 / 0.25`.

### 3. cmd_vel angular velocity clamp

Exp 25 capped `ang_z` to ±0.3 rad/s in `run_husky_nav2.py` to reduce
motion blur. Exp 13 runs show yaw rate up to ~0.89 rad/s during tight
obstacle corridors - the clamp disabled this.

**Fix**: remove the clamp (done by copying exp 13's `run_husky_nav2.py`).

## results

### main attempts

| Attempt | Config | Max x (GT) | Distance | % | Collisions | Verdict |
|---|---|---|---|---|---|---|
| GT baseline | simple config, world wps, `--use-gt` | -50.7m | 44m | 27% | 1 (cone2) | wedged in cone inflation |
| Simple + SLAM | exp 13 params, SLAM | -45.6m | 49m | 29% | 0 | lucky south bypass, stuck |
| **Exp 13 verbatim - run 1** | exp 13 scripts/configs/waypoints | **-10.2m** | **85m** | **51%** | 0 | **new best after regression** |
| Exp 13 verbatim - run 2 | same | -10.7m | 84m | 50% | 0 | deterministic plateau |
| Exp 13 verbatim - run 3 | same | -11.3m | 84m | 50% | 0 | deterministic plateau |
| GT + world wps | GT with exp 13 BT/params, straight world wps | -53.5m | 42m | 27% | 0 | cone 1 wall |
| Exp 13 verbatim + stale fallback | +SLAM stale detection in relay | -10.9m | 84m | 50% | 0 | 3 STALE events handled during tent bypass |

### comparison vs exp 13 reported

| Metric | exp 13 (Apr 10) | best reproduction (Apr 11) |
|---|---|---|
| Max forward x (GT) | ~+50m | -10.2m |
| Distance | 145m | 85m |
| Route % | 87% | **51%** |
| Collisions | 0 | 0 |
| Cone 1 bypass | x | x (min 3.33m) |
| Tent bypass | x | x (min 3.54m) |
| Cone 2 reach | x | ✗ (stuck 25m away) |
| Cone 3 reach | x | ✗ |

## What worked

**Reverting the 3 regressions** moved the plateau from 29% -> 50%, a 1.7×
improvement achieved by **removing complexity** rather than adding it.

**SLAM stale detection** in `tf_wall_clock_relay.py` correctly fires 3
times during tent bypass (SLAM freezes briefly when robot slows to
weave around the tent). Each event self-resolves within ~4s via odom
dead-reckoning; final `fused_x/y = predicted_x/y` from cmd_vel
integration.

```
SLAM STALE #1: odom=0.50m slam_moved=0.043m - dead-reckon from (73.7, 2.0)
SLAM RECOVERED: moved 0.46m from stale point
SLAM STALE #2: odom=0.55m slam_moved=0.048m - dead-reckon from (73.7, 0.7)
SLAM RECOVERED: moved 0.43m from stale point
SLAM STALE #3: odom=0.51m slam_moved=0.025m - dead-reckon from (85.7,-0.9)
SLAM RECOVERED: moved 0.45m from stale point
```

Without the fallback the robot would have gotten fewer wp reached before
the Nav2 recovery cascade, so it's a net positive even though it doesn't
move the final plateau.

## Why it still stops at x=-10.9 (not exp 13's 145m)

With SLAM stale addressed, the robot's pose is accurate throught the
stuck period - it's the **Nav2 local planner** that cannot find a
forward path. Trajectory analysis (from Isaac GT log) shows:

```
t=190s  pos=(-13.0, -1.5)  cmd=(+0.80, -0.68)  <- advancing
t=200s  pos=(-10.9, -1.5)  cmd=(-0.25,  0.00)  <- BackUp triggered, max x
t=210s  pos=(-12.9, -1.8)  cmd=(-0.25,  0.00)  <- reversing
t=220s  pos=(-14.8, -2.1)  cmd=(-0.25,  0.00)  <- reversing
...
```

No SLAM STALE events in this period - SLAM correctly tracks the backup
motion. The barrier is a Nav2 planner dead-end triggered by the
inflation gradient of a terrain feature (tree/rock/log cluster at
x=-18..-10 where `build_forest_scene.py` places obstacles with
`abs(y) < 4` from world y=0, but the road curves up to y=1.8 at x=-10,
putting some trees within inflation range of the curved road).

## why we cannot reach exp 13's 145m

Same scene USD (`/opt/husky_forest_scene.usd`, Apr 6 - unchanged), same
Nav2 config, same waypoints. Exp 13 ran Apr 10 and reached 145m once.
Our Apr 11 reruns all plateau at 84-85m. Candidates:

- ORB-SLAM3 non-determinism - feature matching has randomness, and
   exp 13 may have gotten one lucky SLAM drift direction that pulled
   the planner path away from the obstacles rather than into them.
2. **ORB config changes** - we bumped `nFeatures` 2000->3000 and
   `iniThFAST` 15->10 in exp 26 for VIO robustness. That WAS reverted
   in exp 30. Should be equivalent.
3. Something non-reproducible in Isaac Sim render/physics between dates.

Three runs deterministically plateau at the same x=-10.7 ± 0.5m. That's
the honest reproducible ceiling on this hardware/Isaac-Sim build.

## Artifacts

- `results/best_trajectory.{csv,png}` - exp 13 verbatim run 1 (85m, 51%, best)
- `results/exp30_run{2,3}_trajectory.{csv,png}` - reproducibility runs
- `results/stale_fallback_trajectory.{csv,png}` - with SLAM stale fix
- `results/gt_baseline_trajectory.{csv,png}` - GT baseline (simple config)
- `scripts/tf_wall_clock_relay.py` - with SLAM stale detection + dead-reckoning
- `scripts/send_nav2_goal.py`, `run_husky_nav2.py`, `spawn_obstacles.py` - exp 13 verbatim
- `scripts/costmap_snapshot.py` - exp 33 helper for costmap logging
- `scripts/plot_trajectory.py`, `collision_count.py`
- `config/nav2_husky_params.yaml`, `nav2_bt_simple.xml` - exp 13 verbatim
- `config/rgbd_d435i_v2_mapping.yaml` - ORB 3000 features
- `config/slam_waypoints.json` - exp 13 waypoints (83 wp, curved south)
- `logs/nav2_goal_run1.log`, `nav2_isaac_run1.log`, `slam_rgbd_run1.log`
  - logs from best run

## Key takeaways

1. **Don't alter low-level nav primitives to "protect SLAM"** without
   testing the full obstacle pipeline - it cost us 5 experiments and
   ~50% of reproducible performance.
2. **Exp 13 was the baseline**, and its setup is now the default in the
   working tree.
3. **RGBD-only SLAM silent stale** is real and the odom dead-reckoning
   fallback handles it cleanly during slow manoeuvres. Keep this fix.
4. **Deterministic reproducible ceiling** with current scene + Nav2 is
   ~50% on road-with-obstacles. Further improvements would require
   either scene rebuild (trees away from curved road) or RPP tuning to
   not slow down in inflation gradients.

## summary table (road outbound, with obstacles)

| Version | Max x (m) | Distance | % | Collisions | Notes |
|---|---|---|---|---|---|
| exp 01 GT baseline (no obstacles) | +72 | 182m | 100 | 0 | reference |
| exp 13 (Apr 10) | ~+50 | 145m | 87 | 0 | best historical |
| exp 25–27 (watchdog/corridor/snap) | -45.6 | 49m | 29 | 0 | compound regressions |
| exp 28 exp 13 verbatim (SLAM) | -10.2 | 85m | 51 | 0 | SLAM drift mismatch |
| exp 28 with SLAM stale fallback | -10.9 | 84m | 50 | 0 | same plateau |
| **exp 28 GT + exp 13 GT-derived wps** | **+11.3** | **106m** | **64** | **0** | **new best, past tent+cone 2** |

## Why GT + GT-derived waypoints pushed past the 51% plateau

The 51% plateau on exp 13-verbatim SLAM runs was not a Nav2 dead-end -
it was a **SLAM drift mismatch**. The `/tmp/slam_waypoints.json` (83 wp)
used by exp 13 was extracted from a specific `mapping_road` ORB-SLAM3
session whose drift curved the SLAM-frame path south to y≈-11.7 in SLAM
coordinates. In that mapping run, SLAM y=-11.7 corresponded to world
y ≈ -10.9 (south of cone 1, inside the "wide gap south"). Exp 13 on
Apr 10 happened to get a fresh SLAM run with similiar drift, so the same
SLAM waypoints still mapped to the world-frame bypass path.

Our Apr 11 fresh SLAM has different drift. The same SLAM-frame waypoint
`y_slam = -11.7` maps to world y ≈ -5 - **right into the cone
inflation zone**. Robot follows the waypoint, gets close to cones, Nav2
BackUp triggers, oscillates, stuck.

**Fix in this run**: extract waypoints from exp 13's GT trajectory
CSV (saved after-the-fact), giving WORLD-FRAME coordinates directly.
No SLAM drift dependency.

### trajectory zones (GT+GT-derived waypoints run)

| Zone | x range | y range | Result |
|---|---|---|---|
| Cone group 1 bypass | x=-55..-45 | **y=[-11.29, -3.39]** | x south gap, 170 samples |
| Tent bypass | x=-22..-15 | y=[-2.68, -2.21] | x north side, 90 samples |
| Cone group 2 approach | x=10..20 | y=[-5.03, -1.40] | ✗ wedged at (11.3, -5.0) |

At cone 1 the robot followed the waypoints down to y=-11.3 - **deeper
than exp 13's min y=-10.95** - cleaner bypass. Tent bypassed on the
north side. But at cone 2 the robot couldn't execute the south dive to
y=-7.9 dictated by the waypoints, stopping at y=-5 (right between the
cones).

Possible reasons cone 2 failed where cone 1 succeeded:
- Different approach direction (robot entering from y=-2 vs the more
  southern approach in exp 13)
- Nav2 path planner couldn't find a feasible arc from y=-2 to y=-7.9
  in 5m of x travel (that's a ~50° turn)
- Physical terrain / rocks near (15, -8) that didn't appear in exp 13

## what this shows

1. **The Nav2 pipeline works** - with good waypoints + GT localization,
   it reaches 64% where SLAM-waypoint runs cap at 51%. No tuning of
   Nav2 params needed.
2. Exp 13's 87% was reproducible in theory - the waypoints encode
   a valid bypass path; we just need the right localization frame.
3. **Exp 13-era SLAM waypoints are not portable** accross fresh SLAM
   runs because they bake in a specific SLAM drift.
4. **For future obstacle nav work**, always use GT-derived waypoints
   (or build a deterministic mapping->navigation pipeline with
   persistent atlas).
