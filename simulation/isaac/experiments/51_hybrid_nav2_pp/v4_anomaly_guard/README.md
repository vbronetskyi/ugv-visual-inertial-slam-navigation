# Exp 51 v4_anomaly_guard - detect SLAM position jumps in tf_relay

## hypothesis

v3 showed a "39 m VIO jump" at Frame 2100 (t=215 s). Detecting single-tick
SLAM jumps in `tf_wall_clock_relay.py` and freezing SLAM input for a few
seconds during each anomaly should let the encoder carry through the bad
update while keeping VIO accuracy the rest of the time.

Implementation: `SlamAnomalyGuard` in `tf_wall_clock_relay.py` - checks
per-tick displacement against a threshold; if exceeded, returns last-good
pose for a freeze window.

## First attempt (velocity treshold 2 m/s): catastrophic

- 836 guard activations in ~4000 s of runtime.
- Normal VIO jitter (±0.1 m per 50 ms tick) = 2 m/s apparent speed -> triggered
  the guard on almost every tick.
- Encoder carried localization for 5 s × 836 activations -> unbounded drift.
- Robot drove 909 m (route is ~300 m) cascading wildly off course. Final err
  293 m at gt=(43.5, 391.3) - essentially no localization.

Lesson: velocity-based detection catches normal noise, not just jumps.

## second attempt (per-tick displacement threshold 0.8 m): guard never fired

- 0 guard activations over a full 428 m drive.
- VIO per-tick displacement stayed ≤ 0.08 m (normal 0.85 m/s × 50 ms).
- Largest 100-frame VIO jump this run: 8.5 m (over 10 s = 0.85 m/s, normal).
- err still grew: 0.28 / 0.94 / 2.56 / 17.21 / 27.90 m for 0-50 / 50-100 /
  100-150 / 150-200 / 200-300 m bins.

| bin | v3 err (mean / max) | v4 err (mean / max) |
|---|---|---|
| 0–50 m | 0.38 / 0.79 | **0.28 / 0.59** |
| 50–100 m | 1.14 / 1.43 | 0.94 / 1.17 |
| 100–150 m | 10.81 / 45.59 | 2.56 / 6.75 |
| 150–200 m | 42.69 / 47.80 | 17.21 / 26.26 |
| 200–300 m | - (stopped) | 27.90 / 40.65 |

## what this actually shows

The v3 "39 m jump at Frame 2100" was a mis-interpretation. It wasn't a
sudden BA reoptimisation that jumps the map. Looking at per-100-frame VIO
displacements across this v4 run, the biggest move was **8.5 m / 100
frames (10 s)** - normal forward motion, not a jump.

The real failure mode is gradual scale drift - VIO accumulates
direction and distance error a little per frame. Over 300 m the
cumulative offset reaches 60 m. Individual frame-to-frame deltas stay in
the "normal" range (< 0.1 m/tick), so no per-tick anomaly detector can
catch it.

This matches the original v2_scale_correction diagnostic conclusion:
`scale = encoder_dist / vio_dist` trends away from 1 as the run
progresses. The "Umeyama ATE 5.27 m, scale 0.72" finding from v1 was
telling us exactly this - VIO distance diverges from GT distance
smoothly, not in jumps.

## Verdict

Guard is not usefull when the failure mode is smooth scale drift. The two
approaches that *could* address smooth drift:

1. **Distance-based SLAM_ALPHA decay.** `SLAM_ALPHA = 0.95` while
   `enc_total_dist < 80 m`, decay to ~0.3 above 120 m. Encoder takes
   over while VIO remains a smoothing input for yaw and local shape.
   Known limitation: tuned per-route; real Husky encoder has worse drift
   than the simulated 0.9999-ratio encoder here.
2. **Online scale correction via rolling encoder/VIO distance ratio.**
   Already tried in v2_scale_correction - didn't work because the rolling
   ratio was too noisy on stop-and-go Nav2 motion. Could revisit now
   that v2_imu_fix removed the phantom-motion-during-stops source of that
   noise; the ratio should be cleaner.

For the thesis, the pragmatic stopping point is: **first 100 m of
teach-and-repeat is tight (<1.5 m err), beyond that depends on whichever
of the two scale-drift mitigations we choose**.

## Files

- `scripts/tf_wall_clock_relay_v4.py` - frozen snapshot with
  `SlamAnomalyGuard` class and integration in `_tick_slam_encoder`
- `scripts/run_exp51_v4.sh` - orchestrator
- `results/err_series.csv` - `dist / slam_err / enc_err / guard_acts` per
  tf-relay log sample
- `logs/` - full run logs; `grep GUARD tf_slam.log` shows activations
  (0 in this successful run; 836 in the failed first-attempt run that
  logs above)
