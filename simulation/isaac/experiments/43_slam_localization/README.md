# Exp 43: ORB-SLAM3 + Encoder Fusion

## result

**SLAM+Encoder fusion achieves 95% completion with obstacles - matching
encoder+compass.** Proper SE(3)->SE(2) frame conversion + stale detection makes
SLAM viable.

| Localization | No obstacles | With obstacles |
|---|---|---|
| GT (exp 41) | 43/43 (100%) | 41/43 (95%) |
| Encoder+Compass (exp 42) | 43/43 (100%) | 41/43 (95%) |
| **SLAM+Encoder (this exp)** | **42/43 (98%), 402s** | **41/43 (95%), 460s** |
| SLAM+Encoder (broken v1) | n/a | 23/43 (53%) |

## Goal

Test ORB-SLAM3 RGBD localization fused with encoder odometry as a position
source for Nav2 navigation. Compare with encoder+compass (exp 42) baseline.

## architecture

```
Isaac Sim (run_husky_forest.py --nav2-bridge [--obstacles])
  └─ GT pose -> /tmp/isaac_pose.txt (10Hz)
  └─ RGB+depth frames -> rec_dir/camera_{rgb,depth}/ (10Hz)

ORB-SLAM3 rgbd_live (mapping mode, fresh map per run)
  └─ Watches rec_dir for new frames
  └─ Writes SLAM pose -> /tmp/slam_pose.txt

tf_wall_clock_relay.py --slam-encoder
  ├─ SLAM tracking -> 70% SLAM + 30% encoder position, compass heading
  └─ SLAM stale/lost -> encoder+compass fallback (same as exp 42)
  └─ Publishes map->odom->base_link

Nav2 (same config as exp 41/42)
```

GT is used **only** to simulate noisy sensor readings (encoder = GT diff +
0.5% noise, compass = GT yaw + 3° noise) and as ORB-SLAM3 visual input
through the depth camera. Nav2 navigation receives only fused SLAM+encoder
output.

## Implementation

### bug v1: naive coordinate extraction (53% -> 95% after fix)

```python
# v1 (BROKEN): assumes camera stays perfectly horizontal
nav_x = spawn_x + (slam_z - slam_z_initial)    # camera Z (forward) -> nav X
nav_y = spawn_y - (slam_x - slam_x_initial)    # camera X (right) -> -nav Y
```

When the robot pitches on uneven terrain, camera z-axis no longer parallel
to ground. Small angular errors compound to 98m position error over 200m.

### Fix: proper SE(3)->SE(2) projection with axis swap

```python
# Camera optical -> FLU body axis swap as 4x4 matrix
T_FLU_from_cam = np.array([
    [0,  0, 1, 0],   # flu_x = +cam_z (forward)
    [-1, 0, 0, 0],   # flu_y = -cam_x (left)
    [0, -1, 0, 0],   # flu_z = -cam_y (up)
    [0,  0, 0, 1],
])

# At first valid SLAM pose: compute alignment transform
T_nav_slam = T_nav_origin @ T_FLU_from_cam @ inv(T_slam_origin)

# Each subsequent SLAM pose:
T_nav = T_nav_slam @ T_slam
nav_x, nav_y = T_nav[0, 3], T_nav[1, 3]
```

The axis swap as part of the alignment matrix ensures camera tilts/pitches
don't drift the projection over time.

### Stale SLAM detection

ORB-SLAM3 keeps reporting frames even when tracking is lost - the pose just
freezes. Two checks:

```python
# 1. Detect frozen SLAM pose while encoder shows motion
if encoder_motion > 0.1m and slam_motion < 0.01m:
    slam_frozen_count += 1
if slam_frozen_count > 10:  # ~2s frozen at 5Hz
    fallback_to_encoder()

# 2. Reject large SLAM-vs-encoder disagreements
if abs(slam_position - encoder_position) > 5.0m:
    fallback_to_encoder()
```

## Detailed Metrics (with obstacles)

Reference path: 157.5m straight-through baseline (exp 35, no obstacles).

| Method | Reached | Path length | CTE mean | CTE max | Loc err mean | Loc err max | Duration |
|---|---|---|---|---|---|---|---|
| GT (exp 41) | 41/43 (95%) | 194.5m | 1.68m | 7.36m | 0.00m | 0.00m | 491s |
| Encoder+Compass (exp 42) | 41/43 (95%) | 183.5m | 1.60m | 5.84m | 0.90m | 1.13m | 461s |
| **SLAM+Encoder (exp 43)** | **41/43 (95%)** | **184.6m** | **2.64m** | **4.78m** | **1.58m** | **2.34m** | **460s** |

Metric definitions:
- Path length: total distance robot drove (longer = more bypass detours)
- **CTE (Cross-Track Error)**: perpendicular distance from no-obstacle reference
- **Loc err**: distance between Nav2's belief (TF) and actual GT pose
- Duration: wall-clock seconds

## SLAM Tracking Coverage

How often SLAM was the active position source vs encoder fallback:

| Scenario | SLAM coverage | Encoder fallback |
|---|---|---|
| No obstacles (exp43_v4) | 47% | 53% |
| With obstacles (exp43_final) | 20% | 80% |

Cones and tent disrupt visual feature matching (high-contrast cones + flat
tent surface), so SLAM loses tracking faster with obstacles. Encoder fallback
covers the rest. Final position remains accurate because compass-corrected
encoder is bounded.

## insights

1. **All 3 methods -> identical 95% completion**. Nav2 + depth costmap absorb
   localization errors smaller than goal tolerance (3m).
2. Encoder+compass has best localization (0.9m mean error) - it derives
   from GT with explicit small noise model.
3. **SLAM+encoder has slightly worse localization** (1.58m) - camera frame
   conversion adds systematic offset, plus tracking gaps.
4. **GT cheats** - 0m error by construction, but 95% completion shows that
   the obstacle layout itself causes 2 unavoidable skips (WPs inside barrier).
5. **Encoder+compass shortest path** (183.5m) - confidently chooses good
   bypass. **GT longest** (194.5m) - most aggressive avoidance. **SLAM
   middle** (184.6m).
6. **CTE shows path quality**: encoder+compass closest to ideal path
   (1.60m), SLAM+encoder deviates more (2.64m) due to position jumps when
   switching SLAM↔encoder.

## Conclusion

SLAM+encoder fusion is viable when properly implemented with:
1. SE(3)->SE(2) projection including FLU-from-camera axis swap
2. Stale SLAM detection (frozen pose + jump detection)
3. Encoder+compass fallback for SLAM tracking gaps

For this teach-and-repeat scenario on a 6-8m wide road, **encoder+compass
is sufficient** and simpler. SLAM adds value when:
- Position needs <0.5m accuracy (precision tasks, narrow corridors)
- Multi-session navigation requiring consistent map coordinates
- GPS-denied environments where compass heading is unreliable
- Long-term operation where encoder drift would accumulate

For robust outdoor navigation: **encoder+compass + SLAM as optional
correction** with proper fallback gives best of both worlds.

## Artifacts

### logs
- `logs/exp43_final_*.log` - final run with obstacles (this README's main results)
- `logs/exp43_final_traj.csv` - GT trajectory of the final run
- `logs/exp43_v4_*.log` - no-obstacles run (98% result)
- `logs/exp43_obs_*.log` - earlier obstacles run (different RNG seed)
- `logs/exp43_v3_*.log` - SE(3) fix without stale detection (early iteration)
- `logs/exp42_slam_*.log` - original broken v1 (53% result, pre-fix)

### Plots
- `results/exp43_trajectory_comparison.png` - reference + waypoints + 3 methods
- `results/exp43_drift_no_obs.png` - SLAM vs encoder drift (no obstacles)
- `results/exp43_drift_obs.png` - SLAM vs encoder drift (with obstacles)
- `results/exp43_source_timeline.png` - SLAM/encoder mode timeline
- `results/exp43_completion_comparison.png` - completion rate bars

### Config
- `config/nav2_params.yaml` - same as exp 41/42
- ORB-SLAM3 config: `/root/bags/husky_real/rgbd_d435i_v2_mapping.yaml`
- ORB vocabulary: `/workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt`

### Scripts
- `scripts/tf_wall_clock_relay.py` - `--slam-encoder` mode with SE(3) fix
- `scripts/run_husky_forest.py` - saves RGB+depth for SLAM
- `scripts/send_trajectory_goals.py` - waypoint sender
- `scripts/plot_results.py` - plot generation
- `scripts/compute_metrics.py` - metrics from logs
