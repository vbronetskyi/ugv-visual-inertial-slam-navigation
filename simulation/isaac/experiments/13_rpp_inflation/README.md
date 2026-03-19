# exp 13: Regulated Pure Pursuit + larger inflation - best SLAM result (145 m)

## Goal

Replace DWB with **Regulated Pure Pursuit (RPP)** controller. RPP has built-in features that DWB doesn't:
- Auto-slowdown near obstacles via `use_cost_regulated_linear_velocity_scaling`
- Auto-slowdown on tight turns via `use_regulated_linear_velocity_scaling`
- Smooth carrot-following without velocity-space sampling

Also: increase costmap inflation from 0.45 m -> **1.5 m globally, 1.0 m locally**. This gives the planner more room around obstacles, reducing the chance of robot driving too close.

## Setup

```yaml
# nav2_husky_params.yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.8
  lookahead_dist: 1.2
  use_velocity_scaled_lookahead_dist: true
  use_regulated_linear_velocity_scaling: true
  use_cost_regulated_linear_velocity_scaling: true
  regulated_linear_scaling_min_speed: 0.3
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.5
  inflation_cost_scaling_factor: 3.0
  cost_scaling_dist: 1.5
  allow_reversing: false  # important for skid-steer

local_costmap:
  inflation_layer:
    inflation_radius: 1.0
    cost_scaling_factor: 3.0

global_costmap:
  inflation_layer:
    inflation_radius: 1.5  # was 0.45
    cost_scaling_factor: 2.5
```

Everything else from exp 09 (SLAM frame, mapping mode, empty map, sensor fusion).

## Result - best SLAM-based navigation result

**145 m, 87 % route, max X = 50 m. All 4 obstacle groups bypassed!**

| Metric | exp 13 (RPP) | exp 09 (DWB) | exp 01 (GT baseline) |
|---|---|---|---|
| Distance | **145 m** | 141 m | 182 m |
| Route % | **87 %** | 85 % | 100 % |
| Obstacles | **4/4** | 3/4 | 4/4 |
| Avg speed | 0.63 m/s | 0.72 m/s | 0.62 m/s |

## what worked

1. Cost-regulated velocity is exactly what we needed. As the robot approaches an obstacle (high cost in inflation zone), RPP reduces linear velocity smoothly. No backup loops, no oscillation - robot just slows down and weaves around.
2. Larger inflation = wider safety margin. With 1.5 m inflation, the planner places paths 1.5 m away from obstacles. Even with SLAM drift, robot stays clear of the actual obstacle by ~0.5 m.
3. **`allow_reversing: false`** prevents the backup-loop trap that DWB falls into. RPP just keeps trying to find forward motion.
4. Pure pursuit follows the path smoothly - no velocity sampling means no tracking oscillation in tight spaces.

## where it fails

Stuck at the bush near cones 3 (world x ≈ 47, y ≈ -8). Same location as exp 09. The bush is taller than 1.2 m so the obstacle filter sees it; the bush is in the inflation zone so RPP slows down; but the path through the cones is narrower than the 1.5 m inflation requires -> no valid path. Robot turns around and gets confused.

## Conclusion

This is the best SLAM-based navigation result before VIO experiments (exp 18+). 145 m of 167 m total route, avoiding all 4 obstacle groups. The remaining 22 m to destination is blocked by terrain interaction (bush + narrow gap between cones), not SLAM drift.

## Files

- `config/nav2_husky_params.yaml` - RPP config + 1.5 m inflation
- `scripts/run_husky_nav2.py` - same as exp 09
- `scripts/send_nav2_goal.py` - same as exp 09 (no skip logic)
- `scripts/tf_wall_clock_relay.py` - same as exp 09
- `results/trajectory_plot.png`
