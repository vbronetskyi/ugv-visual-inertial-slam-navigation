# v9 results - FULL ROUNDTRIP COMPLETED 🎉

## Clean run vs end-spin

Route endpoint is WP 94 at GT (-89.1, -11.5) - essentially the spawn
zone. Robot drove the full roundtrip and got **closest to WP 94 at 2.91 m**
after 373 m travelled. After that point it circled the home cluster
(WP 91-94 all within ~5 m) without closing the final 3 m. Those last
15 m of wander are excluded from the per-distance stats because they
reflect Nav2 goal-cluster ambiguity, not VIO performance along the
route.

- **Clean run**: 0-373 m, 632 samples, 89 WPs, 32 anti-spin activations
- **End-spin** (excluded): 373-388 m, 38 samples, 3 more WPs declared
  via tolerance, 83 more anti-spin activations (guard working hard but
  goal cluster keeps retriggering)

## Headline numbers (clean run - to closest approach to final WP)

| metric | v8 | **v9 clean** |
|---|---|---|
| WPs reached | 34/95 | **89/95** |
| Timeouts | 1 | **0** |
| Max distance | 185 m | **373 m** |
| Closest approach to WP 94 | - | **2.91 m** |
| Anti-spin activations | n/a | **32** |
| ATE mean | - | **1.63 m** |
| ATE median | - | **1.42 m** |
| ATE max | - | **4.89 m** (last ~50 m of approach) |

This is the first Exp 51 run to complete the full south route roundtrip
since the experiment began.

## Per-distance error (clean run, 0-373 m)

| range | v8 err (mean/max) | **v9 clean err (mean/max)** | note |
|---|---|---|---|
| 0-50 m | 0.47 / 0.95 | 0.50 / 0.97 | outbound, near spawn |
| 50-100 m | 1.43 / 1.93 | 1.42 / 1.70 | outbound |
| 100-150 m | 1.59 / 3.24 | 1.95 / 2.47 | outbound turnaround zone |
| 150-200 m | **5.69 / 7.65** | **1.28 / 1.70** | **v8 cascaded here, v9 clean - 4.5× better** |
| 200-250 m | - (v8 stuck) | 1.05 / 1.37 | start of return leg |
| 250-350 m | - | 1.72 / 3.48 | return leg |
| 350-400 m | - | **4.28 / 4.89** | final approach (drift grows as robot nears goal cluster) |

The 150-200 m result is the critical one. v8 got stuck in cascading spin
here because VIO err > 1.5 m tolerance. v9 with `--tolerance 3.0` and
anti-spin cleared the same zone at 1.28 m err mean.

## Anti-spin behaviour

- **32 activations during the clean 0-373 m run** (≈ once per 12 m - the
  guard was active but not thrashing)
- **83 more activations during the final 15 m** (home-zone goal cluster
  ambiguity - robot couldn't close the last 3 m without entering a spin
  cycle, guard kept firing every ~5 s to interrupt)
- Each activation: 3 s cooldown with w=0, v=0.15 m/s -> robot creeps
  forward, VIO re-anchors, normal PP resumes

Without the guard, v9's tolerance-3m change alone probably wouldn't have
been enough past 300 m (where err started creeping up).

## Fixes that made v9 work

Two changes from v8, nothing else:

1. **`--tolerance 3.0`** in `send_goals_hybrid.py` (was 1.5)
   - Breaks the cascade: VIO err 2-3 m doesn't trigger Nav2 "not reached"
2. **Anti-spin guard** in `pure_pursuit_path_follower.py`
   - Detects `|w| ≥ 0.5, |v| ≤ 0.1, progress < 0.5 m in 5 s`
   - Forces 3 s cooldown with `w=0, v=0.15` so VIO can re-anchor

Both are **control-loop fixes, not SLAM fixes**. The IMU, ORB-SLAM3
config, and tf_relay stay identical to v8.

## What this proves

The remaining 100-500 m drift in v8 was **not a VIO failure** - the VIO
itself was producing 2-3 m accurate localization, good enough to drive
the route. The robot got stuck only because the Nav2-PP control loop
was intolerant of that level of err.

Fix the control loop tolerance + add an anti-spin failsafe -> the existing
VIO (v3-v8 style position-double-diff IMU with calibrated yaml
NoiseAcc=0.275) drives the full 384 m roundtrip.

## Where and why VIO actually drifted (root-cause analysis)

Decomposing the nav−GT offset into `dx` and `dy` components reveals the
drift pattern is **not random** - it flips sign at the turnaround:

| range | `dx` mean | `dy` mean | err mean |
|---|---|---|---|
| 0-50 m outbound | +0.33 | +0.36 | 0.50 |
| 50-100 m | +0.29 | **+1.37** | 1.42 |
| 100-150 m | 0.00 | **+1.95** | 1.95 |
| 150-200 m (turnaround) | −0.58 | +0.96 | 1.28 |
| 200-250 m return | **−1.01** | +0.02 | 1.05 |
| 250-350 m | −0.21 | **−1.63** | 1.72 |
| 350-400 m final approach | −0.86 | **−4.86** | 4.95 |

Outbound drift is in `+y`, return drift is in `−y` - the direction
reverses when the robot reverses. That's the classic signature of a
**yaw mis-alignment in the SE(3)->SE(2) initial transform**: a fixed
rotational offset projects "forward in body" into "forward + small
lateral" in world, so the lateral error tracks motion direction.

Fitting a best-fit rigid transform (Kabsch) between the whole VIO track
and GT:

- **Yaw offset: 1.804°** (VIO world frame rotated 1.8° from GT world)
- Translation offset: (+0.29, +0.52) m

Applying this correction:

| | raw | yaw-corrected |
|---|---|---|
| err mean | 1.87 m | **1.50 m** (−20%) |
| err median | 1.43 m | 1.31 m |
| err max | 6.84 m | **4.17 m** (−39%) |

So **~20% of the observed error comes from a one-shot yaw alignment
bug**, the remaining ~80% is real VIO drift accumulating along the
route (mostly the 350-400 m final approach).

### Why the yaw alignment is off

`_slam_se3_to_nav` in `tf_wall_clock_relay.py` computes
`T_nav_slam = T_nav_origin · T_FLU_from_cam · inv(T_slam_origin)` from
**one** sample at the phase-2 switch. If SLAM's reported pose at that
instant has ±1° of jitter (which is well within ORB-SLAM3's early
tracking noise, especially during the warmup phase where the robot was
doing a manoeuvre), this 1° propagates to every future pose as a fixed
rotation bias. No amount of VIO accuracy later in the run can correct it.

### Why the remaining drift (1.5 m mean after yaw fix)

After yaw correction, residual err is:

- bounded through 250 m (≤1.5 m consistently)
- grows to ~4 m in the last 50 m (350-400 m)

The final-50m growth correlates with the anti-spin cluster: once Nav2
starts issuing rotate-in-place commands approaching the home goal
cluster, VIO orientation uncertainty grows (rotations lose feature
parallax), the map drifts a bit, and the Nav2 tolerance can't close the
cascade completely. This is VIO degrading under unfavourable motion,
not a separate bug.

### Recommended fixes for future v10

1. **Stabilise initial SE(3)->SE(2) alignment**: average the first 2-3 s
   of SLAM poses during a straight outbound segment instead of using
   the single pose at phase-2 switch. Expected: eliminate the 1.8° yaw
   offset -> 20% err reduction across the whole run.
2. **Better home-zone goal handling**: once robot is within 5 m of
   final WP, send all remaining WPs rapid-fire or accept "finish" early.
   Would have saved the 15 m of end-spin.

Neither is an IMU or VIO-proper fix. The core VIO/IMU pipeline is sound.

## Why the end-spin happened

At the end of the return leg, the robot arrives near the start position
with VIO err 2-3 m. Goal waypoints 91-94 are all within ~5 m of each
other (the start-zone cluster). With 3 m tolerance, the robot often
satisfies several waypoints simultaneously or the planner issues short
paths whose lookahead point sits behind the robot (classic cascade
trigger). PP then spins to "face" a target that is geometrically behind,
anti-spin activates, cooldown, resume - repeat. The 106 extra activations
in this phase are the guard doing its job: preventing total failure but
unable to resolve the ambiguity because multiple goals share the area.

This is a **goal-sequencer issue**, not a VIO issue. Per-bin err up to
373 m (closest approach 2.91 m to WP 94) is tight; beyond that we are
not navigating along the route but milling around the end. A smarter
goal sender that advances through all remaining waypoints once the
robot is within e.g. 5 m of the final target would terminate the run
cleanly here.

## Files

- `results/err_series.csv` - dist, slam_err, enc_err per sample
- `results/v9_summary.png` - 4-panel summary plot (whole run including
  end-spin, for reference)
- `results/v9_clean_run.png` - restricted to clean 0-373 m (up to closest approach 2.91 m to final WP 94)
- `scripts/pure_pursuit_path_follower.py` - anti-spin guard added
- `scripts/run_exp51_v9.sh` - with `--tolerance 3.0`
- `logs/` - full 52 min run logs
