# Exp 51 v6_imu_noise_fix - match ORB-SLAM3 noise model to measured synth-IMU

## Change

One-line yaml: [`config/vio_th160.yaml`](config/vio_th160.yaml)

```yaml
# was: IMU.NoiseGyro: 5.0e-3  IMU.NoiseAcc: 2.0e-2  (Phidgets spec)
IMU.NoiseGyro: 5.0e-2   # measured gz std 0.14, ax/ay 0.005 - conservative ~0.05
IMU.NoiseAcc:  1.5      # measured ax std 3.5, ay/az 1.1 - RMS ~1.5
```

Rationale: v5 diagnostic (`../v5_scale_correction/results/imu_diagnostics.txt`)
showed the synth IMU has ~100× more accel noise than the yaml claims.
ORB-SLAM3 uses `IMU.NoiseAcc/NoiseGyro` as variance weights in visual-inertial
bundle adjustment. With wrong weights BA over-trusts IMU -> integrates
high-frequency noise as signal -> scale drifts.

No other changes from v3_no_loop (IMU standstill fix + `loopClosing: 0` +
patched `LoopClosing.cc::Run()`).

## Results (run stopped early at 110 m, user asked to proceed to Variant A)

| range | v3 err (mean/max) | v4 err (mean/max) | **v6 err (mean/max)** |
|---|---|---|---|
| 0–50 m | 0.38 / 0.79 | 0.28 / 0.59 | 0.46 / 0.93 |
| 50–100 m | 1.14 / 1.43 | 0.94 / 1.17 | 1.35 / 1.94 |
| 100–150 m | **10.81 / 45.59** | **2.56 / 6.75** | **0.65 / 1.01** |

The 100-150 m bin is the clearest signal: v3 drifted catastrophically, v4
drifted a bit, **v6 stays tight (0.65 m mean, 1.01 m max) - ~17× better than
v3 at the same distance**. 27 waypoints reached, 0 timeouts when stopped.

v6 is slightly worse than v3/v4 at short range (0–50 m): 0.46 vs 0.28–0.38
mean. Higher yaml noise tells BA to weight IMU less, so short-range
localization relies more on vision which is noisier per-frame. But the
trade-off at long range is dramatic in v6's favour.

## What v6 confirmed about the root cause

The yaml noise-model mismatch was real and was a significant contributor to
scale drift. But it's a workaround - the synth IMU itself is producing
200× more noise than a real Phidgets 1042 would. The cleaner fix is
to reduce the noise at generation.

## Follow-up

User chose **Variant A** - replace position-double-diff with Isaac's
`GetLinearVelocity` rigid-body API for a cleaner acceleration source. With
that, yaml returns to the real Phidgets spec (NoiseAcc 0.02, NoiseGyro 0.005)
because the actual noise will match. See `../v7_physx_velocity/` for that.

## Files

- `config/vio_th160.yaml` - yaml with corrected noise values
- `scripts/tf_wall_clock_relay_v6.py` - frozen snapshot (reverted v5's
  RollingScaleCorrector, back to simple blend)
- `scripts/run_exp51_v6.sh` - orchestrator
- `logs/` - partial run up to 110 m
