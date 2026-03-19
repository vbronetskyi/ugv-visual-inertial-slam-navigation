# exp 11: SLAM frame + improved waypoint skip logic

## goal

Fix cascading waypoint skip from exp 10. Add safeguards to prevent the skip logic from accidentally finishing the route prematurely.

## The cascade-skip bug

In exp 10, when robot got stuck at an obstacle:
1. Timeout fires -> skip waypoint A
2. Forward-skip detects robot is past waypoint A's X -> skip waypoints B, C, D
3. Robot is now told to reach waypoint E which is far away
4. Nav2 reports `distance_remaining = 0` (goal outside costmap) -> false positive "reached"
5. -> repeat for waypoints F, G, H, ...
6. Outbound complete in 1 second
7. Return waypoints all triggered the same way -> "navigation complete" while robot is at 30 m

## Fix attempts

- **11a**: Just MAX_CONSECUTIVE_SKIPS = 3 - blocks cascade but also blocks legitimate forward progress
- **11b**: + false positive detection (compare reported reached vs real distance via TF) - works for outbound, but return phase has wp at 100+ m away
- 11c: + retry false-positive instead of skipping - prevents cascade but robot gets stuck retrying unreachable goals
- **11d**: + skip waypoints > 15 m from robot ("too far to plan") - prevents Nav2 from sending goals it can't plan to

## results

| variant | distance | what happened |
|---|---|---|
| 11a | 50 m | cone area, stuck |
| 11b | 80 m | false positive flooding |
| 11c | 80 m | retry loops |
| 11d | 50 m | over-aggressive skip |

**None better than exp 09 (141 m).** The skip logic improvements were necessary safeguards but didn't actually improve route progress.

## conclusion

The skip logic is **hard to get right** without breaking the simple cases. Reverted to exp 09's simpler logic for exp 12, then changed the controller (DWB -> RPP) in exp 13 instead.

Lesson: when a complex set of fallback rules makes things worse, it's often a sign that the **underlying assumption is wrong**. In this case: aggressive skip assumes the robot can recover via skipping, but actually it just makes Nav2 confused about what it's supposed to be doing.

## Files

- `scripts/send_nav2_goal.py` - has all of: forward skip, timeout skip, false positive detection, retry logic
- `results/trajectory_plot.png`
