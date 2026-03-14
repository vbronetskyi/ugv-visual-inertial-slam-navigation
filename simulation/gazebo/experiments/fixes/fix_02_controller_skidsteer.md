# Fix 02: DWB -> RPP controller for skid-steer Husky

## Problem
DWB (Dynamic Window B) found 0 valid trajectories out of 440.
DWB assumes holonomic or diff-drive - not suitable for skid-steer.
RPP with collision_detection=true also failed (phantom collisions from LiDAR seeing terrain objects).

## Fix
- Switched controller: DWB -> Regulated Pure Pursuit (RPP)
- Set `use_collision_detection: false` (collision_monitor handles safety)
- RPP parameters: desired_linear_vel=0.5, lookahead 0.8-2.5m
- inflation_radius: 0.55 (matches inscribed radius)
- cost_scaling_factor: 2.0 (softer falloff)
- Auto-goal finding: search for free cells 2-3.5m from robot in mapped area

## Result
Navigation SUCCESS: 2.70m -> goal in 7.2 seconds
