# exp 09: SLAM Frame Navigation - paradigm shift

## Goal

Stop fighting SLAM drift in world frame. Instead, **navigate entirely in SLAM coordinate frame**: waypoints and robot pose are both in the same drifted frame, so drift cancels out.

## the insight

Previous experiments published `map -> odom -> base_link` where `map` is the world frame. Robot was at GT position, but Nav2 saw it via SLAM which had drift. Result: the further along the route, the more "off" Nav2's view of robot position.

If we **publish SLAM pose directly as `odom -> base_link`** (no world frame conversion) and use waypoints extracted from a previous SLAM mapping run, then:
- Robot's reported position drifts with SLAM
- Waypoints are already in the SAME drifted frame
- Difference between reported position and waypoint = the actual progress

Drift becomes **invisible to Nav2** as long as both sources drift together.

## setup

1. Extract SLAM waypoints from a previous mapping run's `CameraTrajectory.txt`:
   ```python
   # SLAM camera frame: Z=forward, X=right
   nav_x = slam_z   # forward
   nav_y = -slam_x  # left
   # Sample every 2 m along the trajectory
   ```
   `mapping_road` (3651 frames, ATE 0.49 m) -> 87 outbound waypoints

2. TF relay in SLAM frame mode (`--slam-frame`):
   - `map -> odom = identity` (no world conversion)
   - `odom -> base_link` = fused SLAM pose directly
   - Uses sensor fusion (SLAM + cmd_vel + IMU gyro) but in SLAM coordinates

3. **Empty occupancy grid for Nav2 map** (200×200 m, all free space). Nav2 doesn't need a static map - depth obstacle layer handles obstacles in real time.

4. Send waypoints in SLAM frame (`send_nav2_goal.py --slam-frame`)

## result - best SLAM result up to that point

**141 m, 85 % route, 3/4 obstacle groups bypassed!** Drift consistently 0.3-1.5 m on straights.

| Metric | exp 09 (SLAM frame) | exp 07 (best world frame) |
|---|---|---|
| Distance | **141 m** | 138 m |
| Route % | **85 %** | 76 % |
| Obstacles bypassed | 3/4 | 4/4 (but stuck after) |
| Lateral drift on straights | 0.3-1.5 m | 5-8 m |

## what we learned

1. **The paradigm works.** Drift in world frame is 5-8 m; in SLAM frame, the "drift" between robot and waypoints is < 1 m on straights, < 3 m during obstacle bypass. Nav2 sees a consistent local picture.
2. **Stuck at the bush near cones 3.** Robot dove south into a tall bush at world (47, -8). The bush is taller than 1.2 m so the obstacle layer sees it; Nav2 backed up but couldn't find a path around.
3. **Drift patterns diverge after multiple bypasses.** The original mapping run had its own drift; the navigation run develops a slightly different drift. After 3 obstacle bypasses, the drifts are different enough that waypoints near the end point at off-road locations.

## Subsequent experiments

This experiment is the foundation of exp 10-14:
- **exp 10**: SLAM frame + localization mode (atlas) - failed (53 m), atlas in localization mode drifts differently
- exp 11: Aggressive waypoint skip - broke the cascade-skip cases
- **exp 12**: Revert to exp 09 config - confirmed reproducibility
- exp 13: Replace DWB with Regulated Pure Pursuit + larger inflation - **best SLAM result 145 m**
- **exp 14**: + sanitize waypoints + false positive detection - same 145 m, finishes outbound

## Files

- `scripts/tf_wall_clock_relay.py` - adds `--slam-frame` mode
- `scripts/send_nav2_goal.py` - adds `--slam-frame` flag
- `scripts/run_husky_nav2.py` - same as exp 02
- `config/slam_frame_map.yaml` - empty occupancy grid
- `config/empty_200x200.pgm` - 400×400 px all-free map
- `config/nav2_slam_frame_launch.py` - Nav2 launch with empty map
- `config/slam_waypoints.json` - 87 SLAM-frame waypoints
- `results/trajectory_plot.png`
- `results/live_trajectory_aligned.png` - VIO trajectory with proper Umeyama alignment

## Note

This experiment had a notable hiccup: the first run used the wrong source `CameraTrajectory.txt` (from a different SLAM session). The waypoints were correct in their own frame but didn't match the live SLAM frame. After re-extracting from `mapping_road/CameraTrajectory.txt`, the result matched expectations.
