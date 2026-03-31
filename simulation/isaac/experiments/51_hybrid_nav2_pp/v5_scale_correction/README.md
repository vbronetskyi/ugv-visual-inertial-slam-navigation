# Exp 51 v5_scale_correction - RollingScaleCorrector + root-cause diagnostics

## Original plan

After v3 (no loop closing) and v4 (anomaly guard) both showed gradual drift
past ~100 m, attempt a rolling encoder/VIO distance-ratio scale correction
in `tf_wall_clock_relay.py`:

```
scale = enc_dist / vio_dist over 20 s window, alpha=0.05
corrected_pos = origin + (slam_pos − origin) × scale
```

Encoder is accurate (0.9999×GT in sim), VIO gives accurate direction but
drifting scale -> combine strengths.

## Run observation

Scale converged sensibly for first ~50 m (0.99–1.02), then diverged:
- at dist=49 m: scale 0.99, err mean 0.64 m / max 1.54 m
- at dist=65 m: scale 0.78, err mean 5.72 m / max 15.90 m over 50–100 m bin

The scale correction made localization *worse* than v3/v4 in the same bin.
Stopped the run to diagnose rather than keep iterating on thresholds.

## Root-cause diagnostics (`scripts/imu_diagnostics.py`, results in `results/imu_diagnostics.txt`)

Three tests on the v5 run's bag (`isaac_slam_1776485681`, 115 s, 80 m):

### test 1 - accel RMS ratio

| quantity | value |
|---|---|
| GT-derived horizontal accel RMS (during motion) | 0.892 m/s² |
| Synth IMU horizontal accel RMS | 3.842 m/s² |
| **ratio synth/GT** | **4.31×** |

Synth IMU has 4× more high-frequency energy than the actual motion.

### test 2 - position recovery (double-integration)

Double-integrate synth IMU horizontal accel in world frame (using GT yaw)
and compare path length to GT:

| quantity | value |
|---|---|
| IMU-integrated path | 82.2 m |
| GT path length | 80.1 m |
| **recovery ratio** | **1.026** |

IMU energy is correct - the mean integral recovers distance to within
2.6 %. There is no systematic under/over-shooting of acceleration.

### Test 3 - LPF-window sensitivity

Recovery ratio is stable 1.02–1.04 for LPF windows 1, 3, 5, 7, 11, 21, 41.
Not a tuning problem in the smoother.

### Test 4 (follow-up) - actual noise std vs yaml spec

During constant-motion periods, compare detrended synth-IMU noise to
what `vio_th160.yaml` declares:

| axis | yaml spec | measured | factor |
|---|---|---|---|
| ax | 0.02 m/s² | **3.52** | **176×** |
| ay | 0.02 | 1.09 | 54× |
| az | 0.02 | 1.12 | 56× |
| gx | 0.005 rad/s | 0.005 | 1× |
| gy | 0.005 | 0.005 | 1× |
| gz | 0.005 | **0.139** | **28×** |

## What this says about the real cause of VIO scale drift

- **Not an IMU energy bias.** Test 2's 1.026 ratio rules out "IMU systematically
  under/overshoots acceleration". Double-integrated position is correct.
- **Not an LPF tuning issue.** Test 3 shows recovery ratio insensitive to smoother.
- Is a noise-model mismatch. Synth IMU is 50–176× noisier on the linear
  axes than the number we declare to ORB-SLAM3. ORB-SLAM3 uses
  `IMU.NoiseAcc` / `IMU.NoiseGyro` as variance weights in its visual-inertial
  BA. Tell it "IMU is ultra-precise (0.02 m/s²)" while actually feeding it
  noise std 1–3.5 m/s² -> BA over-trusts the IMU -> integrates high-frequency
  noise as if it were signal -> scale and direction drift.
- **The `ax` axis is worst (176×)** - matches the v2_imu_fix discovery that
  PhysX contact-solver jitter is amplified most along the drive axis. Even
  with the standstill fix, non-zero motion still carries that axis-aligned
  jitter through double-differentiation.

### Why v5 scale correction misbehaved

With the noise-model mismatch unaddressed, VIO's scale isn't a stable
under/overshoot - it wanders as BA alternately trusts and rejects IMU
constraints. The rolling ratio estimator then chases a target that itself
drifts, producing the observed scale=0.78 over-correction that made things
worse than raw VIO.

## Fix candidates

**Option X - correct `IMU.NoiseAcc` / `IMU.NoiseGyro` in yaml to match
reality.** Tell ORB-SLAM3 the actual noise levels (NoiseAcc ≈ 1.5, NoiseGyro
≈ 0.05) so BA weights the IMU appropriately. One-line yaml change, directly
targets the root cause. Expected: VIO scale drift drops significantly
without any fusion workaround.

**Option Y - reduce synth IMU noise.** Hard: the noise comes from PhysX
position jitter amplified by double-differentiation; LPF windows already
don't help (Test 3). Would need a fundamentally different IMU-generation
approach (e.g. query physics body velocity/accel directly instead of
position-diffing).

**Option Z - accept correct noise in yaml AND scale correction as safety
net.** If the noise-level fix removes most drift, leave the corrector off
and use only if scale still drifts post-fix.

## recommendation

Start with Option X. It's a clean config change whose theoretical
effect (noise-weighted fusion) directly addresses what the diagnostics
identified. Measure residual drift and only add fusion-layer workarounds
if needed.

## Files

- `scripts/tf_wall_clock_relay_v5.py` - frozen snapshot with
  `RollingScaleCorrector` integrated in `_tick_slam_encoder` (kept for
  reference; not the current fix direction)
- `scripts/imu_diagnostics.py` - the three tests above
- `scripts/run_exp51_v5.sh` - orchestrator
- `results/imu_diagnostics.txt` - full diagnostic output
- `logs/` - full run logs (scale corrector run that motivated the
  diagnostics)
