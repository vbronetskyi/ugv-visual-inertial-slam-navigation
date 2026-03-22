# exp 19: VIO Atlas - partial success

## goal

Build a reusable VIO atlas offline, then use it in live localization mode for navigation. This would let us:
1. Build atlas once (offline, deterministic)
2. Use it many times during navigation (live, fast)
3. Get stable VIO localization without per-run init issues

## What worked

### 1. Atlas built sucessfully

Used `exp18_zigzag_init_filter60` recording (2845 frames + 17520 IMU). Ran `rgbd_inertial_offline` with `System.SaveAtlasToFile` enabled on first 2500 frames.

ORB-SLAM3 is non-deterministic (multithreaded), so initial attempts had bad scale or segfaults. **First try with 2500 frames worked**:

- Atlas: 159 MB
- 2398 trajectory poses
- ATE: **0.469 m** vs RGBD-only baseline 0.49 m (basically equal)
- Scale: **0.998** (essentially perfect)
- 0 fail-to-track events
- Single map (no resets)
- End: tx=45.5, tz=139.2 (matches actual GT motion 140m)

Saved as `husky_forest_vio_atlas.osa` in `/root/bags/husky_real/tum_road/`.

### 2. VIO waypoints extracted

75 waypoints in SLAM camera frame (Z=forward, X=right -> nav x=Z, nav y=-X), every 2m. Saved to `/tmp/slam_waypoints_vio.json`.

### 3. Live IMU buffer fix

Increased `IMU_BUFFER_SIZE` from 100 to 2000 entries (10s @ 200Hz) in `run_husky_nav2.py`. With this, live `rgbd_inertial_live` no longer hits "Empty IMU measurements vector" - IMU readings are available for every camera frame regardless of processing lag.

### 4. Zigzag init in main loop

Added 12-second init phase (2s stationary + 10s zigzag with 9 sub-phases) at the start of `run_husky_nav2.py` main loop. Only enabled with `--use-vio` flag. Excites accelerometer for VIO scale recovery.

### 5. Live VIO localization runs

With all the above:
- Atlas loaded from disk (518 KFs, 1 camera)
- VIO init phase completes
- Navigation begins
- 683+ frames processed, 0 lost x
- "start VIBA 1 / end VIBA 1" - Visual-Inertial Bundle Adjustment runs
- Tracking continues throught the route

## what doesn't work yet

**Relocalization vs new-map creation:**

```
Initialization of Atlas from file: husky_forest_vio_atlas
There are 1 cameras in the atlas
First KF:518; Map init KF:518
New Map created with 1824 points  <- !!!
```

ORB-SLAM3 in IMU_RGBD mode with `LoadAtlasFromFile` **doesn't relocalize against the loaded atlas keyframes**. It loads the atlas but creates a fresh map starting at KF 518. The new map is independent - its own coordinate origin, no link to atlas keyframes.

Result: live SLAM poses are in a NEW frame, not the atlas frame. The waypoints we extracted from atlas trajectory are in the OLD frame. Navigation would fail because waypoints don't match current robot pose.

## why ORB-SLAM3 doesn't relocalize in IMU_RGBD localization

- IMU_RGBD mode requires consistant IMU integration. After atlas load, the IMU bias estimates from the atlas don't apply to the new run (different IMU instance, possibly different bias).
- ORB-SLAM3 loads atlas as "old map" but starts a "new map" because IMU init is needed before bundle adjustment can use atlas constraints.
- Even when robot returns to a previously mapped area, relocalization may not trigger because the new map's local features become preferred.

This is a known limitation of ORB-SLAM3 IMU_RGBD localization mode.

## conclusion

We have a **working VIO mapping pipeline** (offline, ATE 0.469m), but **VIO live localization with pre-built atlas doesn't work as expected** - it creates a fresh map instead of relocalizing.

For navigation, the practical options remain:
1. **Use exp 13/14 SLAM-frame approach** (RGBD-only mapping mode + waypoints extracted from same session) - current best at 145m
- Run full VIO mapping inline during navigation - would work but no benefit over RGBD-only since waypoints are still per-run
3. **Use RGBD-only atlas for relocalization** (which works) and use IMU only for gyro yaw fusion in relay (current approach in `tf_wall_clock_relay.py`)

VIO offline ATE 0.469m matches RGBD-only 0.49m - VIO doesn't actually improve accuracy over RGBD-only on this dataset. The IMU helps with scale recovery (which RGBD-only doesn't need anyway because it has metric depth) and gyro stability (which we already get via the relay).

## files

- `scripts/run_husky_nav2.py` - has IMU buffer 2000, zigzag init, --use-vio flag
- `scripts/simple_pure_pursuit.py` - has zigzag init for standalone mapping
- `config/rgbd_inertial_localization_v2.yaml` - VIO localization mode config
- `results/husky_forest_vio_atlas.osa` - built atlas (159 MB)
- `results/CameraTrajectory.txt` - atlas build trajectory
- `results/slam_waypoints_vio.json` - VIO waypoints in atlas frame
- `results/vio_atlas_vs_gt.png` - atlas trajectory vs GT
- `logs/isaac_localization_test.log` - live localization test
- `logs/slam_localization.log` - SLAM-side log showing "New Map created"

## Recommendation

**Stop pursuing VIO localization for navigation.** Use the working SLAM-frame approach (exp 13/14, 145m, 87% completion). VIO offline is interesting as a research artifact but doesn't translate to live navigation gains for this project.

The IMU filter=60 improvement (exp 15) and IMU buffer increase (2000) remain valuable as **building blocks** for the relay's gyro yaw fusion, even without full VIO.
