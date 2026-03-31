# Exp 51 v3_no_loop - disable ORB-SLAM3 loop closing

Eliminate the bad-loop-closure failure seen at ~150 m in v2_imu_fix. Teach-and-repeat
is a single pass; loop closure provides no benefit and proposes wrong matches in
visually repetitive forest.

## changes from v2_imu_fix

1. **`vio_th160.yaml`** adds `loopClosing: 0`.
2. **Patched `ORB_SLAM3/src/LoopClosing.cc::Run()`** - early-exit the loop when
   `!mbActiveLC`, drain the KF queue, idle. Without this patch, setting
   `loopClosing: 0` crashes the library with a segfault (the default code paths
   don't fully honour the flag in the RGB-D-Inertial live wrapper).
3. Rebuilt `libORB_SLAM3.so` and `rgbd_inertial_live`.

All other components unchanged: synthetic-IMU standstill fix from v2, Nav2 planner +
pure pursuit follower hybrid architecture.

## results

| bin | v3 err (mean / max, m) | v3 enc_err (mean) | v2 err (mean / max) |
|---|---|---|---|
| 0–50 m | **0.38 / 0.79** | 0.11 | 0.38 / 0.77 |
| 50–100 m | **1.14 / 1.43** | 0.41 | 1.17 / 1.57 |
| 100–150 m | 10.81 / 45.59 | 0.62 | 2.47 / 5.76 |
| 150–200 m | 42.69 / 47.80 | 0.65 | 20.81 / 34.39 |

| metric | v1 (orig) | v2 (IMU fix) | v3 (+ no loop) |
|---|---|---|---|
| WPs reached | 0 | 32 | **33** |
| TIMEOUTs | - | 2 | 1 |
| max dist | 51 m | 192 m | 174 m |
| BAD LOOP events | 24 | 24 (one accepted) | **0** |

### loop closure was disabled - but VIO still broke

At frame 2000->2100 (t = 205–215 s), raw VIO position jumped **−20.5, −33.5 m**
(39 m magnitude) in a single 10-second window. Encoder drift at the same time was
0.02 m. No `BAD LOOP`, no `VIBA 3+`, no visible reset in the log. GT shows the robot
was driving gently (22, −4.6) -> (32, −8.3) - ~11 m of smooth motion, gradual yaw,
no stops.

So the v3 patch removed the loop-closure failure mode (confirmed: 0 BAD LOOP), but
ORB-SLAM3 has at least one more silent-failure mode that triggers around the same
100–150 m point - likely **map merging** (seperate from loop closure) or local BA
re-optimisation with contaminated keyframes. Symptom is the same (sudden 30–40 m
position jump); cause differs.

## Where this leaves us

Good news:
- Fix A (IMU standstill) is validated again - first 100 m still achieves <1.5 m err.
- Loop-closure patch works and is verifiable (0 BAD LOOP accross the run).
- Hybrid Nav2 planner + pure pursuit follower architecture works - 33 waypoints
  followed smoothly before the VIO jump.

**Bad news:**
- Single-pass teach-and-repeat in dense forest hits another ORB-SLAM3 failure
  around the turnaround / 150 m mark, independent of loop closing.

## proposed next steps

1. **Diagnose the remaining silent jump.** Instrument ORB-SLAM3 to log every
   map-merging attempt, KF culling event, and BA re-optimisation with the pose
   delta it applied. The current binary is silent about these.
2. **Consider disabling map merging too** - `Atlas::CreateNewMap` / `MergeMaps`
   paths. Same rationale as loop closing: single pass doesn't benefit.
3. **Fallback: trust encoder past 100 m.** Encoder err stayed at 0.65 m through
   the whole 174 m run. If VIO can't be stabilised for long routes, SLAM_ALPHA
   in tf_wall_clock_relay could decay with distance (e.g. 0.95 -> 0.3 after 100 m),
   letting encoder carry the run while VIO corrects locally. This is effectively
   the original scale-correction direction we backed away from.

## files

- `scripts/run_exp51_v3.sh` - orchestrator
- `scripts/run_husky_forest_v2.py` - frozen copy of IMU fix
- `config/vio_th160.yaml` - with `loopClosing: 0`
- `config/` - nav2, map, route copies
- `logs/` - isaac, vio, tf_slam, nav2, goals, pp_follower
- `results/err_series.csv` - dist / slam_err / enc_err per tf-relay sample
- Source patch: `third_party/ORB_SLAM3/src/LoopClosing.cc` lines ~94–113 - see
  git diff; not copied here because the binary change is the reproducibility artefact.
