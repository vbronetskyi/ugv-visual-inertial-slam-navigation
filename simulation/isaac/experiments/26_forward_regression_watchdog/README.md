# exp 26: Forward regression watchdog + tighter recovery

## goal

Exp 25 run 4 revealed a recurring failure: after RGBD-only Nav2 bypasses
obstacle 1 (cone group at x=-50), the robot enters a Nav2 recovery loop,
regresses backwards in GT, and can't break out. Depth-seen bushes block the
replanner's attempt to return to the road.

This experiment adds four targeted fixes to break out of recovery loops:

| Fix | Where | What |
|---|---|---|
| A. Forward regression watchdog | `send_nav2_goal.py` | if SLAM max-x doesn't advance for 30s, clear costmaps + skip idx forward + re-send goal |
| B. Minimal gentle BackUp | `nav2_bt_simple.xml` | BackUp dist 0.3->0.15m, speed 0.10->0.08 m/s |
| C. Clear costmap on regression | `send_nav2_goal.py` | watchdog calls `/clear_entirely_local_costmap` + global on trigger |
| D. Tighter lateral corridor | `send_nav2_goal.py` | MAX_LATERAL_DEV 4.0->3.0, RETURN_TO_PATH_DIST 2.0->1.5 |

All other settings inherited from exp 25 run 4: RGBD-only SLAM-frame mode,
ORB 3000 features, cmd_vel angular clamp 0.3 rad/s, RPP lookahead 2.0,
inflation 1.0, depth obstacle range 2.5.

## results

| Metric | Run 25/4 | **Run 26** |
|---|---|---|
| Max forward x (GT) | -45.8m (49m) | **-37.2m (58m)** <- +9m (+18%) |
| Route completion | 29% | **35%** |
| Collisions | 0 | **0** |
| SLAM atlas resets | 0 | **0** |
| Regression watchdog escapes | - | 4 (all after cone 1) |
| Cone 1 bypass min distance | 1.70m | **1.48m** |

New best for road outbound with obstacles. Cone 1 bypassed cleanly.

## What worked

- **Watchdog fires correctly** after 30s of no forward progress and advances
  the waypoint index. Without it run 25/4 was fully stuck in recovery loop.
- Smaller backup (0.15m/0.08 m/s) still keeps SLAM tracking healthy (0
  resets) and regresses less per trigger.
- Clear costmap calls via `/clear_entirely_*_costmap` services work and
  force NavFn to replan without stale depth obstacles.
- **Cone group 1 still bypassed cleanly** with the even tighter margin 1.48m
  (vs 1.70m run 25/4), showing that the softer recovery params work.

## What's still broken - lateral drift into forest

After 4 watchdog escapes around cone 1 area, the robot ended up at GT
`(-37, +8)`. y=+8 is 13m off the road centerline (road y ≈ -5 here) -
robot wandered into the forest on the north side while trying to go
around something.

**Why lateral corridor didn't trigger** even though dev was ~13m:

The window-based check `get_lateral_deviation()` only examines segments in
`[current_idx-2, current_idx+3]`. By the time the watchdog had advanced
`current_idx` to 36, the SLAM frame segments at those indices were at x≈80+,
far ahead of the robot's SLAM x=49. The projection onto each segment falls
behind the segment start (t<0), so the check returns 0 - false negative.

The lateral check needs to search the **entire path** for the closest
segment by perpendicular distance, not just a window around the current
index. When watchdog skips idx forward, the window shifts but the robot
lags behind, creating this mismatch.

## Sequence of events

```
t=0-100s:    smooth outbound, wp 1->21 reached, cmd.angular clamped ±0.30
t=100s:      wp 21 reached at GT (-51, -5.5) - approaching cone group 1
t=100-130s:  cone 1 bypass, min dist 1.48m (clean), max_x=-45.8
t=130-155s:  first recovery loop - gentle backup, no progress
t=155s:      REGRESSION #1 fires: max_x=49.4 (SLAM), robot_x=49.3, skip wp
t=180s:      REGRESSION #2 fires: same SLAM x, skip wp
t=210s:      REGRESSION #3 fires: same SLAM x, skip wp
t=210-240s:  robot drifts laterally y: -5 -> +1 -> +5 -> +8 (off-road north)
t=250s:      REGRESSION #4 fires, no effect - robot wedged at (-37, +8)
```

## files

- `scripts/send_nav2_goal.py` - with Fix A (watchdog) + Fix D (tighter lateral)
- `scripts/tf_wall_clock_relay.py` - SLAM frame mode with reset detection
- `scripts/run_husky_nav2.py` - with cmd_vel angular clamp
- `scripts/collision_count.py` - post-hoc collision counter against GT
- `scripts/plot_trajectory.py` - trajectory plot generator
- `config/nav2_husky_params.yaml` - RPP 2.0 lookahead, inflation 1.0
- `config/nav2_bt_simple.xml` - Fix B: minimal gentle BackUp 0.15m/0.08 m/s
- `config/rgbd_d435i_v2_mapping.yaml` - ORB 3000 features
- `results/exp26_trajectory.csv` + `exp26_trajectory.png`
- `logs/nav2_goal.log`, `nav2_isaac.log`, `nav2_tf.log`, `slam_rgbd.log`

## next experiment

**exp 27: Fix lateral corridor search over entire path, not window.**

Replace the window-based `get_lateral_deviation` with a full-path scan that
finds the closest segment regardless of `current_idx`. When robot drifts far
from the road, the check will fire reliably and force a return.

Also consider: snap `current_idx` back to the SLAM-x-closest waypoint when
the robot is behind the current index by more than 10m, to keep index ∼
aligned with physical progress.
