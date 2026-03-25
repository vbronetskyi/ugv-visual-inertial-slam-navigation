# exp 27: Full-path lateral + current_idx snap

## goal

Exp 26 discovered that the window-based lateral corridor check missed cases
where the watchdog advanced `current_idx` far ahead of the robot's SLAM
position - segments in the window were all beyond the robot, projection
`t` values were <0, and the check silently returned 0 (false negative).

Exp 27 replaces window-based search with full-path search and snaps
`current_idx` back to the closest waypoint on every watchdog escape.

## Fixes

### Fix 1: Full-path lateral deviation search

`scripts/send_nav2_goal.py` - new `get_lateral_deviation_fullpath()` that
scans ALL segments of the current phase waypoint list, computing
perpendicular distance onto each segment (clamped `t ∈ [0,1]`), and returns
`(min_dist, closest_seg_idx, closest_point_xy)`. No window.

The old window-based method is kept as a thin wrapper over the new full-path
version.

### Fix 2: Snap current_idx to robot's SLAM position

`scripts/send_nav2_goal.py` - new `snap_current_idx_to_robot(rx, ry)`
searches all waypoints for the one with minimum euclidean distance from the
robot that is also approximately ahead (within 2m behind in the phase
direction). Returns the snapped index.

Called from `_handle_regression()` before the forward-skip step, so the
watchdog always starts from a plausible "where the robot actually is"
baseline.

### fix 3: Lateral corridor uses full-path closest point

When the lateral corridor triggers, `current_idx` is snapped to the closest
segment index, and the return-to-path goal is the projected closest point
(not just a midpoint from the old segment).

All other settings inherited from exp 26.

## Results

| Metric | exp 26 | **exp 27** |
|---|---|---|
| Max forward x (GT) | -37.2m (58m) | -20.8m (74m) ¹ |
| Route completion | 35% | 44% ¹ |
| Collisions | 0 | **0** |
| SLAM atlas resets (logged) | 0 | 0 |
| Watchdog escapes | 4 | **10** (kept firing without usefull effect) |
| Cone 1 bypass min | 1.48m | 1.39m x cleanest yet |

¹ The 74m / 44% figure is misleading. See "What went wrong" below.

## What worked

- **Full-path lateral search** correctly computes the perpendicular distance
  to the nearest segment regardless of `current_idx` state.
- **Snap logic** fires correctly: `snapped wp 22 -> 19 (closest to robot
  at (43.6,-3.1))` - idx gets pulled back to match robot's SLAM position
  after watchdog escapes.
- **Cone group 1 bypass is now 1.39m** (vs 1.48m exp 26, 1.70m exp 25/4) -
  monotonically improving with each tightening.

## what went wrong - silent SLAM tracking failure

After ~12 watchdog escapes around the cone 1 / tent area, the robot's SLAM
pose stuck at `(43.6, -3.1)` while the GT position drifted to
`(-21, +21)` - i.e. ~29m of linear disagreement and 20m north of the road
into the forest.

```
t=140s   GT=(-35, -5)   SLAM_x=43.6   (roughly consistant)
t=180s   GT=(-25, +3)   SLAM_x=43.6   (SLAM frozen)
t=220s   GT=(-21, +21)  SLAM_x=43.6   (SLAM completely lost)
```

The robot physically drove ~30m forward and 25m north - but RGBD SLAM
reported zero motion in its own frame. No "Fail to track" messages
appeared in the SLAM log (unlike exp 25 run 1 which logged an explicit
reset). This is a **silent tracking failure**: SLAM keeps returning the
last-known pose because it can't find enough features to update.

Consequences:
1. Watchdog monitors SLAM x - doesn't advance -> keeps firing every 30s.
2. Lateral corridor check uses SLAM `(rx, ry)` - sees `(43.6, -3.1)`, which
   is *on* the path. No lateral drift detected in SLAM frame.
3. Nav2 tries to reach goals in SLAM frame that are "ahead" of a phantom
   robot, so it sends commands that (when executed by the real robot in
   world frame) send it off-road.

**The new fullpath/snap fixes are correct but cannot help when their input
(SLAM pose) is stale.**

## Why the collision stats look generous

The `COLLISIONS=0` and `max_x=-20.8m` come from GT log, not SLAM. They
report that the *physical* robot never collided with a cone, because the
robot went far north around the tent area rather than through it. But "44%
route completion" doesn't mean the robot is 44% along the road - it means
the GT x-coordinate reached 74m from start, while the robot is actually
sitting 20m off in the forest.

The meaningful progress metric is **effective path distance** - the
length of the GT trajectory projected back onto the road centerline. Run
27's effective progress is probably ~55m (similiar to exp 26), not 74m.

## Sequence of events

```
t=0-100s:    smooth outbound, wp 1-19 reached cleanly
t=100-130s:  cone 1 aproach + bypass, min 1.39m, max_x_slam=43.6
t=130-160s:  first backup/recovery, watchdog #1 fires, snap idx 21->19->21
t=160-250s:  10 more watchdog escapes - SLAM stays stuck at x=43.6, GT
             drifts from (-35, -5) towards (-21, +21), robot crossing road
             centerline and heading northward into forest
```

## the real problem surfaced

RGBD-only SLAM in SLAM-frame mode is structurally unable to localize a
robot executing Nav2 obstacle avoidance on a narrow forest road:

1. Nav2 recovery behaviors (even gentle BackUp 0.15m/0.08 m/s) cause the
   camera to see a lot of close-range rotation/clutter when working a
   cone + bush cluster.
2. ORB-SLAM3 either loses features outright (exp 25 atlas reset case) or
   degrades to silent "stale pose" mode where it returns last-known-good
   without atlas reset.
3. All downstream controllers (Nav2 planner, our watchdog, lateral
   corridor, goal sender) trust the SLAM pose and make decisions based on
   it. When SLAM is stale, everything goes wrong.

## Files

- `scripts/send_nav2_goal.py` - with full-path lateral + snap logic
- `scripts/tf_wall_clock_relay.py` - SLAM frame mode
- `scripts/run_husky_nav2.py` - with cmd_vel angular clamp
- `scripts/collision_count.py`, `scripts/plot_trajectory.py`
- `config/nav2_husky_params.yaml` - RPP 2.0 lookahead, inflation 1.0
- `config/nav2_bt_simple.xml` - minimal gentle BackUp 0.15m/0.08 m/s
- `config/rgbd_d435i_v2_mapping.yaml` - ORB 3000 features
- `results/exp27_trajectory.csv` + `exp27_trajectory.png`
- `logs/nav2_goal.log`, `nav2_isaac.log`, `nav2_tf.log`, `slam_rgbd.log`

## Next experiment

**exp 28: Cross-validate SLAM with wheel odometry / detect silent stale.**

The fixes so far all trust SLAM unconditionally. Next step: add a cross
check in `tf_wall_clock_relay.py` - if `odom_dx`, `odom_dy` (integrated from
`cmd_vel`) accumulate more than ~2m but SLAM position hasn't changed, flag
SLAM as stale and fall back to odom-only integration until SLAM catches up.

Alternatively, **switch the run to GT odometry** to isolate the Nav2 / BT /
watchdog / lateral logic from SLAM fragility. If the pipeline works with
perfect odometry, we know the remaining issues are purely localization.
