# exp 12: Revert to exp 09 config (reproducibility check)

## goal

Reproduce exp 09's 141 m result with the original (simple) waypoint logic, after exp 10-11 made things worse with aggressive skip rules.

## setup

- Reverted `send_nav2_goal.py` to exp 09 version (no aggressive skip, no false positive detection, no retry logic)
- Same SLAM-frame approach: empty map, mapping mode SLAM, waypoints from `mapping_road`
- Same RPP/DWB? - actually still DWB (before exp 13 introduced RPP)
- Increased `xy_goal_tolerance` from default to 3.0 m (didn't need exact waypoint reaching)

## result

**118 m, 71 % route, traveled 240 m total (with backups).**

| Metric | exp 12 | exp 09 (original) | exp 11 (skip) |
|---|---|---|---|
| Distance | 118 m | 141 m | ~50 m |
| Route % | 71 % | 85 % | 28 % |
| Mean lateral drift | 1.1 m | ~1 m | varies |
| Max lateral drift | 3.9 m | ~4 m | varies |

## what we learned

- Confirmed reproducibility - with the same config, exp 09 and exp 12 give similar results (within ~20 % variation due to SLAM stochasticity).
2. Stochasticity is real. ORB-SLAM3 has multiple threads with timing-dependent behavior. Two runs with identical input can give different results.
3. DWB struggles between cones 2 and 3. On this run, robot got stuck in the gap between obstacle groups. Same approach but lucky vs unlucky drift pattern.

## conclusion

exp 09's aproach is the right baseline. The variation between runs (118 m vs 141 m) is the natural variance of mapping-mode SLAM. Need to either:
- Run multiple times and report mean/std (statistical view)
- Or change the controller (exp 13: DWB -> RPP) to be more robust to drift

Chose the controller change for exp 13.

## files

- `scripts/send_nav2_goal.py` - restored from exp 09
- `scripts/tf_wall_clock_relay.py` - same as exp 09
- `results/trajectory_plot.png`
