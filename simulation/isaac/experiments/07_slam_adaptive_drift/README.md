# exp 07: SLAM + adaptive drift monitor (best SLAM result before VIO)

## Goal

Fix exp 06's drift monitor oscillation with **adaptive thresholds and forward-only waypoints**.

## Setup

```python
# Adaptive drift bands
DRIFT_LOW = 2.0    # < 2 m: full speed 0.8 m/s
DRIFT_MED = 4.0    # 2-4 m: slow 0.4 m/s
DRIFT_HIGH = 7.0   # 4-7 m: crawl 0.2 m/s
                    # > 7 m: backup + replan

# Waypoint skipping
WAYPOINT_SKIP_MARGIN = 3.0
# skip wp when robot's X passes wp's X by margin (forward axis)

# Cooldown after obstacle bypass
COOLDOWN_AFTER_BYPASS = 20.0  # m of forward motion
```

Forward-only waypoint logic: if robot has driven past a waypoint along the forward axis, skip it. No backtracking. No "return to path" - just adapt speed and skip waypoints.

## result - best pre-VIO SLAM

**138 m, 76 % route, all 4 obstacle groups bypassed.** This was the best SLAM result before the SLAM-frame aproach (exp 09-14).

| Metric | exp 07 | exp 06 (drift v1) | exp 02 (raw SLAM) |
|---|---|---|---|
| Distance | **138 m** | 123 m | 106 m |
| Route % | 76 % | 67 % | 58 % |
| Obstacles bypassed | 4/4 | 3/4 | 1/4 |

## What we learned

1. **Adaptive speed is critical.** When drift grows, slow down - gives SLAM time to relocalize and reduces odometry/dead-reckoning error per frame.
2. **Forward-only waypoints prevent backtracking traps.** Robot never gets stuck "going back to a missed waypoint" - it always moves forward.
3. **Cooldown (20 m) prevents post-bypass oscillation.** After obstacle bypass, drift naturally spikes for a few seconds. Cooldown gives the system time to settle.
4. **Still stuck after the route's last obstacle.** Same fundamental problem - accumulated drift means robot ends up 5-10 m from the actual road in the late portion of the route.

## conclusion

Adaptive drift monitor is a **significant improvement** (138 m vs 106 m raw SLAM) but doesn't fully solve the drift problem. The SLAM-frame approach in exp 09 takes a different angle: instead of fighting drift, **work in SLAM coordinates so drift is consistant between mapping and navigation**.

## Files

- `scripts/send_nav2_goal.py` - adaptive drift monitor + waypoint skipping
- `scripts/tf_wall_clock_relay.py` - gyro yaw fusion (same as exp 05)
- `results/trajectory_plot.png`
