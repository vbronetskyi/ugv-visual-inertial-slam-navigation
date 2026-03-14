# Fix 01: planner_server OOM kill due to unbounded global_costmap

## Problem
planner_server silently killed by kernel after ~60-90s.
Global costmap grew unbounded (1913x1973 cells = 3.7M cells).
SLAM Toolbox map grew continuously -> costmap resize every 5s -> OOM.

## Fix
In nav2_params.yaml global_costmap section:
- Added: width: 30.0, height: 30.0, rolling_window: true
- Reduced: update_frequency: 1.0 -> 0.5
- Reduced: publish_frequency: 1.0 -> 0.3
- Reduced: inflation_radius: 0.8 -> 0.6

## Result
planner_server stable, global_costmap bounded to 30x30m rolling window.
