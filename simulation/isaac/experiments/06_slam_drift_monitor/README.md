# exp 06: SLAM + drift monitor v1 (return-to-path)

## Goal

Detect when robot drifts too far from the planned trajectory and force a "return to path" maneuver. Idea: if SLAM thinks robot is on the road but mapped trajectory says it's far away, the SLAM is wrong -> emit a corrective goal.

## setup

Drift monitor in `send_nav2_goal.py`:

```python
# Compare current pose to nearest waypoint on outbound trajectory
drift = closest_dist_to_path(rx, ry, outbound_wps)
if drift > 3.0 and not in_cooldown:
    # Insert intermediate goal: nearest path point
    nearest = find_closest_wp(rx, ry)
    insert_goal(nearest)
    cooldown_until = now() + 20.0
```

## Result

**123 m, 67 % route, bypassed cones.** Better than exp 02-05.

But: **drift monitor oscillates at the cone area.**

## What we learned

1. Return-to-path works in principle - robot is forced back toward the road when SLAM drifts too much.
2. **Treshold = 3 m caused oscillation.** At cone group 1, SLAM drift fluctuated around 3 m. Monitor kept inserting return-to-path goals -> robot couldn't reach them (still drifted) -> backed up -> triggered again -> infinite loop.
3. Cooldown helps but not enough. 20 s cooldown prevents the worst oscillation but the robot still gets stuck after a couple of corrections.

## Conclusion

Static drift threshold is too brittle. Either too low (oscillates) or too high (doesn't help). The next experiment (exp 07) uses adaptive thresholds that change based on context.

## Files

- `scripts/send_nav2_goal.py` - adds drift monitor with fixed threshold
- `scripts/tf_wall_clock_relay.py` - same as exp 05 (gyro yaw fusion)
- `results/trajectory_plot.png`
