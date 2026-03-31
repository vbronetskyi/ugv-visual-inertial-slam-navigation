# v10 results - alignment averaging exposed hidden VIO drift

## Summary

v10 applied two control-layer fixes on top of v9:
- **Fix A** - alignment averaging in `_slam_se3_to_nav` (50 samples
  SLAM-stationary window, SLERP quaternion average, jitter rejection).
- **Fix B** - home-zone goal collapsing in `send_goals_hybrid.py`.

Fix A worked **technically perfectly** (GT disp 0.0 cm, yaw std 0.000° on
first attempt in the second launch after reordering the run script).
**Fix B was never exercised** - the robot did not reach the home zone
because it got stuck at WP 37 around 158 m.

Despite the cleaner alignment, v10 performed **worse than v9** past 100 m
and got stuck earlier. Running this variant revealed a fact the v9
numbers had hidden.

## Numbers

| metric | v9 | **v10** |
|---|---|---|
| WPs reached | 89/95 (clean) | **33/95** |
| Timeouts | 0 | **1** (WP 37) |
| Max distance | 373 m | **158 m** |
| Anti-spin activations | 32 (clean) | 38 (before stuck) |
| Initial alignment yaw std | 1.8° (post-hoc Kabsch) | **0.000°** (live, first attempt) |

Per-distance err:

| bin | v9 (mean / max) | **v10 (mean / max)** | delta |
|---|---|---|---|
| 0-50 m | 0.50 / 0.97 | **0.18 / 0.39** | v10 **5× tighter** x |
| 50-100 m | 1.42 / 1.70 | 0.93 / 2.01 | v10 better mean |
| 100-150 m | **1.95 / 2.47** | **3.47 / 4.53** | v10 worse, cascade |
| 150-200 m | 1.28 / 1.70 | 4.12 / 4.55 | v10 stuck |

## Why v10 beat v9 at 0-50 m but lost 100-150 m

The initial alignment bias in v9 (+1.8° yaw) and the underlying VIO
drift were acting in **opposite directions** through the 100-150 m zone.
v9's 1.95 m mean err there was really a partial cancellation:

```
v9 err  ≈  |(alignment bias × travel)  +  (VIO drift, opposite sign)|
         ≈  |small||
```

v10 removes the alignment bias entirely, so the remaining err is the
full VIO drift unmasked:

```
v10 err  ≈  |(0)  +  (VIO drift, now unattenuated)|
         ≈  |full drift|
```

At 0-50 m there's very little VIO drift yet, so removing the alignment
bias is a pure win - err drops from 0.50 -> 0.18 m (5×). At 100-150 m the
VIO drift is larger, and no longer has an alignment bias to cancel
against, so err grows to 3.5 m - large enough to cross the Nav2
goal-tolerance (3 m) and trigger the cascade that got the robot stuck.

v9 was partially **"lucky"** - the alignment bias happened to reduce the
total err in the bin that matters most for Nav2 tolerance. v10 exposes
the **real** VIO drift magnitude.

## IMU audit in the failing zone (100-150 m)

To check whether the IMU itself was drifting, compared IMU body-frame
acceleration to GT-derived body-frame acceleration, straight-motion
samples only:

```
GT-derived body accel:   ax +0.001 ± 0.398  ay +0.011 ± 0.208
IMU body accel:          ax -0.001 ± 3.852  ay +0.010 ± 1.265
difference (IMU - GT):   ax -0.002             ay -0.000
```

**IMU mean matches GT within 2 mm/s² on both axes.** No bias. Only the
noise std is larger (known-documented; calibrated in yaml). IMU means
are also stable across all distance bins - no distance-dependent drift.

The drift source is **not** the IMU. It is ORB-SLAM3's visual component
(feature tracking / bundle adjustment) in repetitive forest - the same
thing that produced the original v1 Umeyama scale 0.72.

## What Fix A actually bought us

- Remove a one-time 1.8° calibration error -> perfect 0-50 m (0.18 m mean
  instead of 0.50).
- Expose real VIO drift magnitude honestly instead of hiding behind a
  bias coincidence.

What it does **not** do: fix the VIO drift itself. That drift is
fundamental to monocular/visual-inertial SLAM in repetitive features
without external correction.

## What Fix B would have done (if reached)

Home-zone collapse logic was correct (armed properly after robot left
home zone at 8.9 m distance). Never fired because the robot got stuck
at 158 m, long before reaching the home zone on return. No data on
whether it actually fixes the end-zone circling.

## Implications for v11 (follow-up)

The v10 experiment settled a key question: **further improvements have
to attack the VIO itself, not the alignment or control layer**. Control-
layer tweaks (tolerance, anti-spin, alignment) can only trade errors
around, not reduce them.

Practical options for v11:

1. **Map reuse (teach-and-repeat proper).** Save the ORB-SLAM3 Atlas
   during a clean teach run, load it on repeat so VIO localises against
   known map points instead of building its own. Eliminates drift
   accumulation - the teach map has global consistency.
2. **Encoder scale fusion revisited.** Now that alignment is clean and
   we know IMU is mean-correct, a `rolling_scale = enc_dist / vio_dist`
   correction could actually work. This was v5 but noisy because
   phantom-motion was not fixed; now it is.
3. **Accept v9 as the working answer.** The user-visible effect of v9
   was "robot drives full roundtrip". v10 is cleaner theoretically but
   worse practically. Pin v9's setup as the hybrid-VIO+Nav2 baseline
   and move on to exp 50 (obstacles).

My recommendation: (3) - pin v9, move to obstacles, then revisit VIO
improvements in a dedicated experiment if teach-and-repeat accuracy
becomes a thesis-scale concern.

## Files

- `config/vio_th160.yaml` - unchanged from v8 (NoiseAcc 0.275)
- `scripts/tf_wall_clock_relay_v10.py` - with alignment averaging
- `scripts/send_goals_hybrid.py` - with home-zone collapse (+ `left_homezone` guard so it doesn't fire at spawn)
- `scripts/run_exp51_v10.sh` - stops robot BEFORE tf_relay switch (so
  alignment window sees truly stationary robot)
- `logs/` - two runs: the first (v10 v1) had home-zone collapse fire at
  spawn (fixed); the relaunch (v10 v2) is the one reported above
- `results/err_series.csv` - dist, slam_err, enc_err per sample
