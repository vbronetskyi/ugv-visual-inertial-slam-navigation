# exp 08: SLAM mapping mode + full sensor fusion

## goal

Combine three signals: SLAM position + wheel odometry + IMU gyro compass. Use mapping mode instead of localization to avoid the "unknown features after obstacle bypass" problem from exp 03.

Hypothesis: in mapping mode, every visible feature is added to the map -> no problem if robot leaves the original mapped trajectory.

## setup

In `tf_wall_clock_relay.py`:

```python
# position fusion: SLAM (70%) + odometry (30%)
SLAM_POS_ALPHA = 0.7
ODOM_POS_ALPHA = 0.3

# Yaw fusion: IMU gyro (85%) + SLAM correction (15%)
SLAM_YAW_ALPHA = 0.15
GYRO_YAW_ALPHA = 0.85

# jump detection - reject SLAM jumps > 0.5 m
JUMP_THRESHOLD = 0.5
```

ORB-SLAM3 in mapping mode (no atlas), `rgbd_live` binary.

## Result - WORST SLAM result

**47 m, 28 % route.** Worse than all previous SLAM attempts.

## What went wrong

1. **Mapping mode has no external reference.** Localization mode (exp 03) had 519 keyframes from a clean run. Mapping mode starts from zero - drift accumulates without bound.
2. **Wheel odometry adds noise.** Same lesson as exp 04 - `cmd_vel` integration on skid-steer is too noisy. Adding 30 % weight made things worse.
3. Robot got stuck on cone group 1. Drift 3.2 m laterally -> SLAM thought robot was 3 m off course -> Nav2 kept inserting return-to-path goals -> robot crashed into cones.

## What we learned

**Mapping mode is worse than localization mode.** Without an external reference (atlas), drift compounds with no correction. The "no unknown features" benefit of mapping mode is illusory - the new features are still in a drifted frame.

This was the **last attempt to fight SLAM drift directly with sensor fusion**. After this, the project pivoted to:
- exp 09: SLAM-frame navigation (use the same drift consistently between mapping and navigation)
- exp 13: Regulated Pure Pursuit + bigger inflation (let the controller handle drift)
- exp 15-19: VIO with proper IMU initialization

## files

- `scripts/tf_wall_clock_relay.py` - full position + yaw fusion
- `scripts/run_husky_nav2.py` - uses mapping mode SLAM
- `results/trajectory_plot.png`
