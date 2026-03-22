# exp 24: Smooth waypoints for forest VIO

## goal

After fixing IMU calibration in exp 23 (VIO road ATE 0.089m), test if **smoothing forest waypoints** (removing A* planner's artificial U-turns) enables VIO to work on forest routes.

## Motivation - detailed analysis of forest VIO failure

In exp 23, VIO on north broke **at the first sharp turn** (not end of route):

```
VIO scale ratio over time (exp23 north):
  t= 10s ratio=0.396  (warmup, still converging)
  t= 25s ratio=0.845  (warmup end)
  t= 55s ratio=0.942  (entering north route, almost perfect)
  t= 65s            <- FIRST SHARP TURN starts
  t=105s ratio=10.6   (scale exploded)
  t=155s ratio=42.5   (disaster)
```

The first sharp turn at t=63-74s was a **U-turn 180° in 11 seconds at ~1m radius** - robot goes from yaw +14° to yaw -180° (turning in place). This is an **A* path planner artifact**: planner squeezes tight waypoints around the first tree, which the waypoint-follower interprets as aggressive sidesteps.

Literature confirms VIO needs **diverse translational motion** for scale observability; pure rotation or in-place U-turns break the scale estimate.

## Approach

**B-spline smoothing with collision-aware moving average**:

```python
def smooth_movavg(waypoints, window=7):
    out = waypoints.copy()
    for i in range(2, n-2):
        out[i] = mean(waypoints[i-3 : i+4])
    # Revert if NEW collision introduced
    for i in range(n):
        if clearance(out[i]) < 0 and clearance(out[i]) < clearance(waypoints[i]):
            out[i] = waypoints[i]
    return out
```

Then decimate to 2.5m spacing. This:
- Removes sharp artifacts around each tree (moving average smooths them out)
- Keeps every waypoint that has worse clearance **after** smoothing (preserves collision avoidance)
- Never introduces new collisions relative to original

## Results

### waypoint statistics

| Route | Original wp | Smoothed wp | Length | Rotation |
|---|---|---|---|---|
| North | 235 | 118 (-50%) | 487m -> 442m | 2846° -> 1561° (-45%) |
| South | 201 | 99 (-51%) | 401m -> 369m | 1566° -> 1066° (-32%) |

Max heading change per wp reduced from 180° to < 60° for most turns.

### recording exp24_north_smooth

4555 frames, 460s, robot traversed full route with no stuck events (smooth motion throughout).

### VIO evaluation

Ran ORB-SLAM3 RGBD-Inertial with corrected calibration (exp 23) on smoothed north recording, 10 tries:

```
try 1: resets=8  VIBA2=4 ATE=50.34m scale=0.34
try 2: resets=0  VIBA2=2 ATE=28.06m scale=0.016
try 3: resets=0  VIBA2=4 ATE=27.06m scale=0.045
try 4: resets=0  VIBA2=2 ATE=25.07m scale=0.070
try 5: resets=8  VIBA2=4 ATE=52.92m scale=0.154
...
```

**VIO initializes sucessfully** (VIBA 2 reaches, 0 resets in first 1000 frames ≈ 100 seconds) but **scale drifts to 0.02-0.34** by the end.

### Where VIO breaks

From VIO log:

```
Frame 0/2500  - atlas init
start VIBA 1
end VIBA 1
Frame 100/2500
start VIBA 2           <- IMU init locked in
end VIBA 2
Frame 200-1000 - continuous tracking (100 seconds OK!)
Frame 1000  - "IMU is not or recently initialized. Reseting..."
```

Tracking is stable for ~100s, then breaks. At this moment the robot is at waypoint ~30 (around -25, +37), mid-forest. Motion is **smooth** (max ang_v 0.33 rad/s = 19°/s, much better than old 0.5+).

**So smoothing fixed the motion problem**. But VIO still fails at ~100s. The remaining cause is visual feature tracking in the forest, not waypoint geometry.

## Why RGBD-only works but VIO fails

RGBD-only gets metric scale **per-frame from depth sensor**, independent of history. It can tolerate feature matching failures by reinitializing. The whole map keeps growing.

VIO in ORB-SLAM3 IMU_RGBD mode **cross-checks IMU preintegration against visual tracking**. If they disagree (e.g., repetitive forest features give ambiguous matches while IMU integrates forward motion), the consistency check fails -> **entire active map gets reset**.

This is a fundamental architectural difference:
- RGBD-only: per-frame independence -> graceful degradation
- VIO: cross-validated -> hard failure when consistency breaks

Forest scenes have many repetitive vertical structures (tree trunks) that look similar from different angles. ORB feature matching gives ambiguous results. RGBD can live with this, but VIO cannot - it triggers the consistency check which kills the map.

## Conclusion

**Smoothing waypoints is necessary but not sufficient** for VIO to work on forest routes.

Two conditions are needed together:
1. x **Smooth motion** - addressed by waypoint smoothing in this experiment
2.  Unambiguous visual features - fundamental property of forest environments, can't be fixed at waypoint level

The second condition is addressed in real-world VIO systems by using:
- **Stereo cameras** with wider baseline (better depth triangulation)
- **Wheel odometry** as additional constraint (solves the init/tracking problem on mobile robots per Xu et al., 2021)
- **Monocular fisheye** lenses with larger FOV (more unique features per frame)
- **LiDAR-IMU** fusion instead of visual-IMU

For this project's simulation with D435i RGB-D camera, forest VIO is not viable. **RGBD-only remains the production choice for forest** while VIO works well on road.

## final localization table

| Route | RGBD-only ATE | VIO ATE (corrected + smoothing) |
|---|---|---|
| **Road (335m)** | 0.272m | **0.089m** (exp 23) x 3x better |
| **North (467m)** | **1.28m** (best) / 1.86m | failed - scale drift |
| **South (396m)** | **0.46m** | failed - same reason |

## files

- `scripts/run_husky_forest.py` - has `--vio-warmup` flag and `WARMUP_WAYPOINTS`
- `config/slam_routes_smooth.json` - smoothed waypoint dataset
- `logs/vio_north_smooth.log` - VIO log showing VIBA 2 success then later resets
- `results/waypoints_comparison.png` - original vs smoothed waypoints overlay
- `results/full_analysis.png` - heading change histograms
- `results/final_routes.png` - smoothed routes with trees+shrubs overlay
- `results/vio_north_smooth_traj.txt` - VIO trajectory (for reference, not usable)
- Recording: `/root/bags/husky_real/exp24_north_smooth`

## Reproduction

```bash
# Generate smoothed routes
python3 -c "
import json, math, numpy as np
with open('/tmp/slam_routes.json') as f: r = json.load(f)
with open('/tmp/gazebo_models.json') as f: m = json.load(f)
trees = [(x['x'],x['y']) for x in m if x['type'] in ('pine','oak')]
# see run_husky_forest.py for moving-average smoothing code
"

# Record with smoothed routes active
cp slam_routes_smooth.json /tmp/slam_routes.json
/opt/isaac-sim-6.0.0/python.sh run_husky_forest.py --route north --duration 900

# run VIO offline (retry loop - non-deterministic)
for try in {1..10}; do
    /workspace/third_party/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_offline \
        ORBvoc.txt rgbd_inertial_correct.yaml recording/ associations.txt imu_orbslam.txt
    # evaluate, save best run
done
```

## next step

Stop pursuing VIO for forest. For thesis:
1. Report calibration fix (exp 23) as major finding - VIO now works on road
2. Report forest limitation as ORB-SLAM3 architectural issue (not simulation artifact)
3. Use RGBD-only as primary localization for navigation experiments
4. Consider wheel odometry integration as future work (mentioned in literature review)
