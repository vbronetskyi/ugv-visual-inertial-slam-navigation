# exp 15: PhysX IMU noise reduction for ORB-SLAM3 VIO

## goal

Find a way to get clean IMU readings from PhysX in Isaac Sim so ORB-SLAM3 RGBD-Inertial mode can work. Previous attempts at VIO failed because IMU readings during motion had massive noise from contact solver micro-oscillations.

## Setup

- Husky A200 with sphere wheel collisions (radius 0.165m)
- PhysX 200Hz, TGS solver
- IMU sensor on `imu_link` (200Hz sample rate)
- Test: 400 readings stationary, then 400 readings driving 0.5 m/s straight

## Approaches tested

### approach 1: Compliant contacts on wheels - INEFFECTIVE

Attempted to apply `PhysxMaterialAPI` with `compliantContactStiffness=1e6` and `compliantContactDamping=1e4` to wheel materials.

Why it failed:
- Wheel collision shapes in `payloads/Physics/physx.usda` have `purpose = "guide"`
- These collisions are not actively used by PhysX (or are bound through a different path)
- Material binding via `MaterialBindingAPI` has no effect on noise
- Tested both binding to a custom material and applying `PhysxMaterialAPI` directly to discovered collision prims

**Result:** Identical noise to baseline (0.22 m/s² moving with default filter).

### Approach 2: Custom Cylinder Geometry - N/A

Husky wheel collisions are Sphere primitives, not cylinders. Custom Cylinder Geometry option in Isaac Sim only applies to actual `Cylinder` collision shapes. Not applicable to this robot.

### Approach 3: Synthetic IMU from GT pose

Computed IMU readings from robot pose differentiation, then added Phidgets-like noise (`acc_std=0.02`, `gyro_std=0.005`).

**Variant A: differentiate position twice**
- Position -> velocity -> acceleration
- Result: stationary OK (0.02 m/s²), but **moving acc_std = 87 m/s²** - much WORSE than PhysX
- Reason: high-frequency jitter in position is squared by double differentiation

**Variant B: read velocity from PhysX `RigidBodyAPI` directly, single differentiation**
- velocity (direct) -> acceleration (1x diff)
- Result: stationary 0.02 m/s², **moving 5.6 m/s²** - still bad
- Reason: PhysX velocity itself is jittery at 200Hz

**Conclusion:** Synthetic IMU not viable without aggressive low-pass filtering of velocity, which would add significant lag.

###  Approach 4 (the actual fix): IMU sensor filter size

Isaac Sim's IMU sensor API has built-in moving-average filters. Production code already uses `linear_acceleration_filter_size=20` (100ms window at 200Hz). Tested larger filters.

| Filter size | Window | Stationary acc | Moving acc | Stationary gyro | Moving gyro |
|---|---|---|---|---|---|
| 1 (no filter) | 5ms | 0.009 | **3.20** | 0.0002 | **0.10** |
| 20 (production) | 100ms | 0.0001 | 0.218 | 0.0001 | 0.064 |
| 40 | 200ms | 0.0000 | 0.074 | 0.0000 | 0.037 |
| **60** | **300ms** | **0.0000** | **0.050** | **0.0000** | **0.022** |
| 100 | 500ms | 0.0000 | 0.049 | 0.0000 | 0.014 |
| Real Phidgets Spatial 1042 | - | ~0.020 | ~0.025 | ~0.005 | ~0.005 |

**filter=60 gives noise on par with real Phidgets** (within 2x of acc, 4x of gyro).

## recommendation

In `run_husky_nav2.py`, change IMU sensor creation from:

```python
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    sensor_period=1.0 / 200.0,
    linear_acceleration_filter_size=20,
    angular_velocity_filter_size=10,
    orientation_filter_size=10,
)
```

to:

```python
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    sensor_period=1.0 / 200.0,
    linear_acceleration_filter_size=60,
    angular_velocity_filter_size=30,
    orientation_filter_size=30,
)
```

For ORB-SLAM3 VIO config (`rgbd_d435i_v2_inertial.yaml`):
- `IMU.NoiseAcc: 0.05` (was 0.5 in previous tries)
- `IMU.NoiseGyro: 0.022`
- `IMU.AccWalk: 0.0001`
- `IMU.GyroWalk: 0.000019`

## Trade-offs

- **filter=60 = 300ms moving-average window**, ~150ms group delay
- Camera runs at 10Hz (100ms period), so IMU lag ~1.5 frames
- ORB-SLAM3 VIO uses interpolation between IMU samples, so 150ms group delay should be acceptable
- **filter=40** is a safer compromise: 200ms window, 100ms delay, still 0.074 m/s² (3x Phidgets)

## Files

- `scripts/imu_noise_test.py` - standalone test script supporting all variants:
  - `--filter N` - test with sensor filter size N
  - `--compliant` - try compliant contacts (ineffective)
  - `--synthetic` - synthetic IMU from rigid body velocities
- `logs/` - raw output from each test run
- `results/noise_vs_filter.png` - comparison plot

## How to reproduce

```bash
cd /workspace/simulation/isaac/scripts
/opt/isaac-sim-6.0.0/python.sh imu_noise_test.py --filter 1     # baseline
/opt/isaac-sim-6.0.0/python.sh imu_noise_test.py --filter 20    # production
/opt/isaac-sim-6.0.0/python.sh imu_noise_test.py --filter 60    # recommended
/opt/isaac-sim-6.0.0/python.sh imu_noise_test.py --filter 60 --synthetic   # synthetic + filter
```

## Next step

Run RGBD-Inertial ORB-SLAM3 with filter=60 IMU and updated noise model in yaml. Compare against previous VIO failures (which used filter=20). Expected: VIO initialization should succeed and tracking should remain stable through obstacle bypasses.
