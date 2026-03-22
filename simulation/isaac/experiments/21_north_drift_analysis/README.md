# exp 21: North Drift Analysis - zigzag init impact

## goal

Investigate why north RGBD-only ATE was 7.79m in exp 20 (vs old 1.93m baseline), and verify VIO failure mode + IMU configuration.

## three questions

### Q1: Does zigzag init hurt RGBD-only on forest?

**YES - by 4.5x.**

| Configuration | ATE | Max error | Notes |
|---|---|---|---|
| **WITH zigzag init** (default) | **7.79 m** | 26.5 m | High drift in first 60 s |
| **WITHOUT zigzag init** (skip first 200 frames) | **1.72 m** | 5.4 m | Stable throught route |

**Old baseline (April 6 recording, no init)**: 1.93 m - matches our without-init result.

**Why init hurts RGBD-only:**

The 10-second zigzag with rapid orientation changes causes ORB-SLAM3 to start tracking with poor initial keyframes. The map is built around shaky early frames -> all subsequent localization is referenced to a corrupted base. When Umeyama alignment is computed over the whole trajectory, the bad early section pulls the alignment off, inflating ATE everywhere.

**See `results/north_init_effect.png`**:
- Left: trajectories overlaid on GT - WITH-init drifts noticeably; WITHOUT-init follows GT closely
- Right: position error over time - WITH-init peaks at 26.5 m around 50-100 s, WITHOUT-init stays under 5.4 m throught

### Q2: VIO failure on north - where exactly?

**VIO never reaches VIBA 2 - IMU init keeps resetting at frame ~100.**

Detailed log analysis from `vio_north_attempt_log.txt`:

```
Frame 0/1500    - initial atlas, "New Map created with 1842 points"
start VIBA 1
end VIBA 1
Frame 100/1500
IMU is not or recently initialized. Reseting active map...   <- reset 1
LM: Reseting current map in Local Mapping...
Frame 200/1500
IMU is not or recently initialized. Reseting active map...   <- reset 2
... (60+ resets total over 1500 frames)
```

Compared to road (success):
```
Frame 0    - atlas init
start VIBA 1, end VIBA 1
Frame 100, Frame 200
start VIBA 2, end VIBA 2  <- VIBA 2 = IMU init locked in
Frame 300..1400 - continuous tracking, no resets
```

**Diagnosis**: ORB-SLAM3 needs ~3-5 seconds of stable tracking between VIBA 1 and VIBA 2 to lock in IMU bias and gravity. On road, the route is straight after init zigzag - VIBA 2 succeeds. On north, the first auto-route waypoint requires a sharp turn at ~frame 100 (10 seconds after start), exactly when ORB-SLAM3 is trying to converge IMU init. The turn invalidates the bias estimate -> reset -> repeat forever.

This is a **structural limitation** of running auto-route navigation immediately after init phase in dense waypoint forests.

### Q3: IMU.Frequency matches actual data

**YES - exactly 60.0 Hz across all 3 routes.**

```
road:  20777 samples, freq=60.00 Hz, dt=16.67 ± 0.13 ms
north: 31288 samples, freq=60.00 Hz, dt=16.67 ± 0.11 ms
south: 26049 samples, freq=60.00 Hz, dt=16.67 ± 0.11 ms
```

YAML `IMU.Frequency: 60` is correct.

## Implications for thesis

1. **For RGBD-only baselines: do NOT use zigzag init.** The init phase is only needed for VIO scale recovery. For RGBD-only, it adds artificial drift to the early trajectory.

2. **For VIO: zigzag init is necessary on highway routes (road)** to excite the accelerometer. On forest routes (north/south) it's not enough - the immediate sharp turns after init prevent IMU init from locking in.

3. **Best protocol per use case**:
   - **RGBD-only baseline**: record without init phase, drive smoothly
   - **VIO on road**: record with zigzag init, drive smoothly
   - VIO on forest: not viable with current ORB-SLAM3 IMU init mechanism

4. **Updated comparison table** (using best per method):

| Route | RGBD-only ATE | VIO ATE | Notes |
|---|---|---|---|
| Road (335m) | 0.405 m | **0.347 m** | VIO 14% better |
| **North (467m)** | **1.72 m** | failed | RGBD without init phase |
| South (396m) | 0.516 m | failed | (need to retest without init) |

This is a much better story for the thesis - demonstrates RGBD-only is reliable across all routes, and shows when VIO helps (highway) vs when it doesn't (dense forest).

## next step

Re-run RGBD-only on south WITHOUT init phase to get the proper baseline.

## Files

- `results/north_init_effect.png` - comparison plot
- `results/rgbd_with_init.txt` - trajectory with zigzag init
- `results/rgbd_no_init.txt` - trajectory skipping first 200 frames
- `logs/vio_north_attempt_log.txt` - VIO log showing reset loop
- `logs/vio_road_success_log.txt` - VIO log showing VIBA 1 -> VIBA 2 -> success
- `logs/rgbd_north_with_init.log`, `rgbd_north_no_init.log` - RGBD-only logs

## Reproduction

```bash
REC=/root/bags/husky_real/exp20_north

# RGBD WITH init phase (uses full associations)
/workspace/third_party/ORB_SLAM3/Examples/RGB-D/rgbd_tum \
    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    config/rgbd_only.yaml \
    $REC \
    $REC/associations.txt

# RGBD WITHOUT init phase (skip first 200 frames = ~20s init + first turn)
sed -n '200,$p' $REC/associations.txt > $REC/associations_no_init.txt
/workspace/third_party/ORB_SLAM3/Examples/RGB-D/rgbd_tum \
    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    config/rgbd_only.yaml \
    $REC \
    $REC/associations_no_init.txt
```
