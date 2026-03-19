# exp 10: SLAM frame + localization mode (failed combination)

## goal

Combine the two best approaches: SLAM frame navigation (exp 09, 141 m) + atlas localization (exp 03). Hypothesis: localization mode -> drift pattern matches the atlas -> waypoints from the atlas align better with live pose.

## Setup

- ORB-SLAM3 RGBD-only with `LoadAtlasFromFile: husky_forest_atlas`
- Same SLAM-frame TF relay as exp 09
- Same SLAM-frame waypoints from exp 09
- Aggressive waypoint skip + timeout (3 attempts)

## result - failed

**~50 m, 28 % route.** Worse than exp 02-09.

Multiple sub-experiments:
- 10a (timeout=15s): cascading skips, route abandoned at 50 m
- 10b (timeout=45s): better, but still stuck at 80 m
- 10c (timeout=90s): hit cone obstacles, drift > 10 m

## What went wrong

1. **Atlas localization in SLAM frame is incoherent.** Atlas poses are in the original mapping run's coordinate frame. Live SLAM in localization mode tries to relocalize against this frame. But the live drift pattern is different from the mapping run's drift pattern. So "SLAM frame" is not consistent - there are two slightly different SLAM frames.
2. **Cascading waypoint skip.** Aggressive timeout-based skip caused all waypoints in the obstacle area to be skipped at once, then return waypoints became false-positive "reached" because Nav2 reports `remaining=0` for goals outside the local costmap.

## Conclusion

**Don't combine atlas localization with SLAM-frame navigation.** They assume different things about the coordinate frame. Either:
- Use atlas in world frame (exp 03) - works but drifts after maneuvers
- Use mapping mode in SLAM frame (exp 09) - works because mapping and navigation are in the same SLAM frame

After this experiment, returned to exp 09 mapping-mode aproach (exp 12) and added Regulated Pure Pursuit (exp 13).

## files

- `config/rgbd_d435i_v2_localization.yaml` - adds atlas loading
- `scripts/send_nav2_goal.py` - has aggressive skip logic
- `results/trajectory_plot.png`
