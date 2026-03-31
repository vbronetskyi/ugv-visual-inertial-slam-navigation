# Exp 51 v8_correct_yaml - IMU noise density calibrated from measurements

## Motivation

Deep audit revealed a **unit bug** in my v5 diagnostic. YAML `IMU.NoiseAcc`
and `IMU.NoiseGyro` are continuous noise **density** (per-√Hz units), not
discrete-step std. ORB-SLAM3 reads them at [`Tracking.cc:616-617`](../../../third_party/ORB_SLAM3/src/Tracking.cc#L616-L617):

```cpp
const float sf = sqrt(mImuFreq);                // √200 = 14.14
new IMU::Calib(Tbc, Ng*sf, Na*sf, Ngw/sf, Naw/sf);
```

So `NoiseAcc: 0.02` means per-step std = 0.02 × √200 = 0.283 m/s². This
reframes the whole diagnostic history.

## Correct noise values from measurements

On position-double-diff synth IMU (v2/v3/v5/v6 style), discrete-step std
during constant motion:

| axis | measured std | density (std / √200) | yaml spec density |
|---|---|---|---|
| ax | 3.87 m/s² | **0.274 m/s²/√Hz** | 0.02 (13.7× too low) |
| ay | 1.22 | 0.086 | 0.02 |
| az | 1.29 | 0.091 | 0.02 |
| gx | 0.0049 rad/s | 0.00035 rad/s/√Hz | 0.005 (14× too high - spec is pessimistic) |
| gy | 0.0051 | 0.00036 | 0.005 |
| gz | 0.042 | 0.003 | 0.005 (close, slightly high) |

## The three candidate yaml configurations

| version | NoiseAcc | NoiseGyro | Effect on BA |
|---|---|---|---|
| Original (v3) | 0.02 | 0.005 | Says ax std=0.28 but real is 3.87 -> **over-trusts IMU 14×** |
| v6 workaround | 1.5 | 0.05 | Says ax std=21 but real is 3.87 -> **under-trusts IMU 5.5×** |
| **v8 calibrated** | **0.275** | **0.005** | Says ax std=3.89 ≈ measured 3.87 -> **matches** |

v6 worked better than v3 (0.65 vs 10 m at 100-150 m) by accidentally being in
the right direction, but overshooting. v8 should do even better.

## why position-double-diff IMU (v6-style) is still the right generator

v7 tried replacing it with PhysX `get_linear_velocity()` single-diff.
Noise density dropped into spec (0.02 m/s²/√Hz) which *seems* cleaner,
but VIO drift was worse (2.82 m at 50-100 m vs v6's 1.35). Reason:
PhysX velocity has a systematic ~7% deficit - it's low-noise but
biased. ORB-SLAM3 needs correct mean, not just low noise.

Position-double-diff is noisier per-sample but **recovers correct mean
energy over integration** (ratio ≈ 1.03 in v5 diagnostic Test 2). With
the correct noise density declared in yaml, BA sees the IMU as
"noisy-but-unbiased" which is what it handles well.

## What v8 actually changes

Just the yaml `NoiseAcc: 0.02 -> 0.275` (and minor NoiseGyro cleanup).
Everything else identical to v3/v6:
- Master `run_husky_forest.py`: position-double-diff + standstill fix
- Master `tf_wall_clock_relay.py`: clean SLAM+encoder 95/5 blend
- Master `libORB_SLAM3.so`: patched to early-exit LoopClosing when disabled
- `loopClosing: 0` in yaml

## Files

- `config/vio_th160.yaml` - calibrated noise density values
- `scripts/run_husky_forest_v6.py` - frozen IMU generator (same as v6)
- `scripts/tf_wall_clock_relay_v6.py` - frozen TF relay (same as v6)
- `scripts/run_exp51_v8.sh` - orchestrator

## Expected results vs v6

With correct weighting, ORB-SLAM3 should integrate IMU appropriately -
not ignore it (v6's 5.5× over-correction) or over-trust it (v3's
14× under-correction). Target: <0.5 m mean err accross 0-150 m, and a
chance to stay tight past 150 m.
