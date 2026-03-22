# exp 23: IMU calibration fix - VIO finally works on road (ATE 0.089m)

## goal

Find and fix all IMU/Tbc calibration errors that were breaking VIO on forest routes (and silently degrading it on road).

## three errors found

### Error 1: Tbc translation off by 20 cm

Old yaml had `t_b_c = (0.3, 0, 0.1)` from constants in `tf_wall_clock_relay.py`.

Actual offsets from `husky_d435i/payloads/base.usda`:
- IMU position in base_link: `(0.190, 0.000, 0.149)`
- Camera position in base_link: `(0.292, 0.018, 0.281)` (sum of `top_plate_link` + `camera_realsense_bottom_screw_frame` + `camera_realsense_link`)
- **Correct t_b_c (FLU body frame)**: `(0.102, 0.018, 0.132)`

### error 2: IMU sensor frame is UBR, not URF

The `imu_link` USD prim has quaternion `(0, 0.7071, 0, 0.7071)` = 90° rotation around Y axis.

Calibration test (`scripts/imu_cal_test.py`) drives the robot at 4 motion states and dumps raw IMU readings:

```
=== STATIONARY ===
  RAW lin_acc: x=+9.8100 y=-0.0237 z=+0.0030     <- gravity on +X
  RAW ang_vel: x=-0.0000 y=+0.0003 z=-0.0002

=== YAW LEFT (CCW = +yaw in world) ===
  RAW ang_vel: x=+1.4022 y=-0.0015 z=+0.0003     <- yaw rate on +X

=== FORWARD DRIVE ===
  RAW lin_acc diff (vs stationary): y=-0.4362    <- forward accel on -Y
```

**Conclusion**: sensor frame is **UBR (Up-Backward-Right)**:
- Sensor X = Up (gravity, yaw axis)
- Sensor Y = -Forward = Backward
- Sensor Z = -Left = Right (cross product: UP × FORWARD = LEFT, so +X × -Y = -Z = LEFT, hence Z = -LEFT = RIGHT)

### error 3: Old `_imu_urf_to_flu` had forward and lateral SWAPPED

Old code:
```python
def _imu_urf_to_flu(ux, uy, uz):
    return (uz, -uy, ux)  # WRONG: this maps right->forward and forward->left
```

This worked accidentally for stationary gravity (because `uy=0, uz=0`), but during motion the forward acceleration was being written to the lateral axis and vice versa. VIO got nonsense values.

**Fixed code**:
```python
def _imu_urf_to_flu(sx, sy, sz):
    """Convert IMU vector from sensor UBR frame to body FLU frame."""
    return (-sy, -sz, sx)
    # flu_x (forward) = -sy (negate backward)
    # flu_y (left)    = -sz (negate right)
    # flu_z (up)      = +sx
```

### tbc rotation

Body frame after conversion = FLU.
Camera frame = OpenCV RDF (X=right, Y=down, Z=forward).

Rotation R_b_c (camera->body):
```
R_b_c = [[ 0, 0, 1],   # cam_X (right) = -body_Y (rights = -left)... wait, columns
         [-1, 0, 0],   # cam_Y (down)  = -body_Z
         [ 0,-1, 0]]   # cam_Z (fwd)   = +body_X
```

Each column is one camera axis expressed in body coordinates.

Full Tbc:
```yaml
Tbc:
   data: [ 0.0,  0.0,  1.0, 0.102,
          -1.0,  0.0,  0.0, 0.018,
           0.0, -1.0,  0.0, 0.132,
           0.0,  0.0,  0.0, 1.0]
```

## results

### Road (335 m route)

| Configuration | ATE | Scale | Resets/run | VIBA 2 success |
|---|---|---|---|---|
| Old broken (swapped axes, wrong Tbc) | 0.347 m | 0.99 | 0-2 | 1/6 runs |
| **Corrected (UBR->FLU + Tbc)** | **0.089 m** | **0.996** | **0** | **5/5 runs** |

VIO is now **3x better than RGBD-only** on road (0.089 m vs 0.272 m).

100 % run success rate vs 17 % before.

### North (467 m forest route)

| Try | Resets | VIBA 2 | ATE | Scale |
|---|---|---|---|---|
| 1 | 8 | 4 | 32.9 m | unknown |
| 2 | 0 | 4 | bad | bad |
| 3 | 0 | 4 | bad | bad |
| **4** | **0** | **4** | bad | bad |
| 5 | 0 | 4 | similar | similar |

VIO **initializes sucessfully** (VIBA 2 reaches, no resets) but scale drifts to 0.01-0.05 over the run. Forest sharp turns cause VIO to misestimate scale even after good init.

## why forest still fails

This is a separate failure mode from the calibration bug:

1. Calibration is now correct (proven by road ATE 0.089 m)
2. ORB-SLAM3 IMU init succeeds (VIBA 2)
3. But during dense waypoint navigation with 90° turns every 2-3 m,
   ORB-SLAM3's bundle adjustment can't keep IMU bias estimates locked
4. Scale drifts as IMU integration error accumulates between visual updates

This is the fundamental ORB-SLAM3 limitation mentioned in the paper: VIO requires *smooth* motion both during init and operation. Forest navigation with aggressive turns is not their target use case.

## Calibration test script

`scripts/imu_cal_test.py` is reusable - drives the robot through 4 known motions and prints the actual sensor frame mapping. Use this whenever IMU sensor configuration changes.

## Final localization comparison (with corrected pipeline)

| Route | RGBD-only ATE | VIO ATE |
|---|---|---|
| **Road (335 m)** | 0.272 m | **0.089 m** x 3x better |
| **North (467 m)** | 1.86 m | failed (scale drift) |
| **South (396 m)** | 0.460 m | failed (forest motion) |

VIO now shows clear advantage on road. Forest still requires RGBD-only.

## Files

- `scripts/imu_cal_test.py` - IMU axis discovery test
- `scripts/run_husky_forest.py` - has corrected `_imu_urf_to_flu`
- `config/rgbd_inertial_correct.yaml` - corrected Tbc and noise params
- `logs/imu_cal_test.log` - calibration test output
- `logs/vio_road_corrected.log` - successful VIO run on road
- `logs/vio_north_corrected.log` - VIO on north (init OK, scale drifts)
- `results/vio_road_traj.txt` - VIO road trajectory
- `results/vio_road_vs_gt.png` - VIO vs GT trajectory plot + error over time

## Reproduction

```bash
# 1. Run calibration test (one-time, when sensor config changes)
/opt/isaac-sim-6.0.0/python.sh scripts/imu_cal_test.py

# 2. Record route with corrected pipeline
/opt/isaac-sim-6.0.0/python.sh scripts/run_husky_forest.py --route road --duration 600

# 3. Convert IMU and gen associations
python3 -c "
import csv, glob, os
REC = '...'
with open(f'{REC}/imu.csv') as fin, open(f'{REC}/imu_orbslam.txt', 'w') as fout:
    next(fin)
    for line in fin:
        p = line.strip().split(',')
        ts, ax, ay, az, gx, gy, gz = p[:7]
        fout.write(f'{ts} {gx} {gy} {gz} {ax} {ay} {az}\n')
rgbs = sorted(glob.glob(f'{REC}/camera_rgb/*.jpg'))
with open(f'{REC}/associations.txt', 'w') as f:
    for r in rgbs:
        fn = os.path.basename(r); ts = fn.replace('.jpg','')
        d = fn.replace('.jpg','.png')
        if os.path.exists(f'{REC}/camera_depth/{d}'):
            f.write(f'{ts} camera_rgb/{fn} {ts} camera_depth/{d}\n')
"

# 4. Run VIO offline
head -1500 ${REC}/associations.txt > ${REC}/associations_1500.txt
/workspace/third_party/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_offline \
    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    config/rgbd_inertial_correct.yaml \
    ${REC} \
    ${REC}/associations_1500.txt \
    ${REC}/imu_orbslam.txt
```
