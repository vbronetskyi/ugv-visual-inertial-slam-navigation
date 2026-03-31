# Exp 51 - Hybrid Nav2 planner + Pure Pursuit follower with VIO localization

Goal: full south-route teach-and-repeat using ORB-SLAM3 RGB-D-Inertial VIO for
localization, Nav2 global planner for obstacle-aware paths, and a custom pure
pursuit follower for smooth control (avoids MPPI/DWB angular-velocity
oscillation that breaks VIO - see exp 49 analysis).

## Architecture

```
Isaac Sim (synthetic IMU + camera)  ->  ORB-SLAM3 VIO  ->  /slam_pose.txt
                                            down
tf_wall_clock_relay --slam-encoder ──->  map->odom->base_link TF
          (95% VIO, 5% encoder, compass yaw)
                                            down
Nav2 planner_server (NavFn on depth costmap)  ->  /plan
                                            down
pure_pursuit_path_follower.py  ->  /cmd_vel
                                            down
Isaac Sim cmd_vel listener (--synthetic-imu)
```

No Nav2 controller, no MPPI, no behavior tree recoveries. Nav2 is just a planner.

## variants

| version | change | WPs reached | max dist | verdict |
|---|---|---|---|---|
| **v1** (this dir, root `config/` + `scripts/`) | base hybrid setup | 0 | 51 m | fails at first turn |
| **v2_scale_correction** | encoder-distance scale correction in tf_relay | - (dropped) | - | papered over symptom; offline data showed rolling scale is too noisy to improve blended output. Diagnostic scripts here led to the real root cause. |
| **v2_imu_fix** | synthetic-IMU standstill detection in `run_husky_forest.py` | 32 | 192 m | first 150 m <2 m err; catastrophic BAD LOOP accept at 150 m |
| **v3_no_loop** | v2_imu_fix + `loopClosing: 0` + patched `LoopClosing.cc::Run()` early-exit | **33** | 174 m | first 100 m <1.5 m err; gradual drift past 125 m (not loop-closure related) |
| **v4_anomaly_guard** | v3 + `SlamAnomalyGuard` in tf_relay (per-tick displacement > 0.8 m -> freeze SLAM) | 33 | 428 m | guard never fired - the real problem isn't single-tick jumps, it's **gradual scale drift** (distributed across many ticks). err grew 0.3 m -> 28 m across 0-300 m. |
| **v5_scale_correction** | v3 + `RollingScaleCorrector` in tf_relay (scale = enc_dist/vio_dist over 20 s window). Stopped mid-run; diagnostics identified **IMU noise-model mismatch** as the real root cause. | - | - | Scale correction worsened err to 5.7 m at 50–100 m. Diagnostics: synth IMU has noise std 176× higher on ax than declared in `vio_th160.yaml` (3.52 vs 0.02 m/s²). ORB-SLAM3 over-trusts IMU in BA -> integrates noise as signal -> scale drifts. See `v5_scale_correction/README.md` for the 4-test diagnostic. |
| **v6_imu_noise_fix** | v3 + yaml noise values matched to measured reality (`NoiseAcc 1.5`, `NoiseGyro 0.05`). One-line config. | 27+ (stopped at 110 m) | 110 m | **0-50 m: 0.46 / 0.93. 100-150 m: 0.65 / 1.01 - ~17× better than v3** at same distance. The yaml fix works. |
| **v7_physx_velocity** | v3 + synth IMU rewritten to use Isaac rigid-body velocity API (single-diff for accel) + quat-diff gyro. Yaml back to Phidgets spec. | - | 99 m | **Accel noise 16× cleaner than v5** (ax std 0.24 vs 3.90). But PhysX `get_linear_velocity()` systematically under-reports by 7% -> IMU integrated path = 93% of GT -> VIO drifts worse than v6 (50-100 m err 2.82 vs v6's 1.35). Concept right, PhysX API isn't ground-truth enough to justify yaml 0.02. |
| **v8_correct_yaml** | v3/v6-style IMU (position-double-diff + standstill) + yaml `NoiseAcc: 0.275` (calibrated from measurement density = 3.87/√200). | **34** | 185 m | Fixes unit bug in v5 diagnostic. 0-50 m: 0.47/0.95. 100-150 m: 1.59/3.24. **150-200 m: 5.69/7.65** (3× better than v4's 17-26 m). Run ended with Nav2 cascade: robot spinning on WP 39 because VIO err > 1.5 m goal-tolerance -> rotate-in-place recovery -> more VIO uncertainty. **The real limit past 150 m isn't IMU or VIO, it's Nav2 tolerance tuning.** |
| **v9_robust_nav** 🎉 | v8 + goal-tolerance 1.5->3 m + anti-spin guard in PP (cooldown after 5 s of low-progress rotation). | **89/95 clean** (92/95 incl. end-spin) | **373 m clean** (closest 2.91 m to WP 94) | **First full roundtrip.** 0 timeouts. Clean run (0-373 m, up to closest approach to final WP): mean err 1.63 m, median 1.42 m, max 4.89 m. 150-200 m: 1.28/1.70 (vs v8's 5.69/7.65, 4.5× better). 32 anti-spin activations during clean run, 83 more while circling the home-zone goal cluster. Two control-loop fixes, zero SLAM/IMU changes - proves the Nav2 cascade was the blocker, not VIO. |
| **v10_alignment_and_homezone** | v9 + alignment averaging in `_slam_se3_to_nav` (50-sample SLERP with stationary + yaw-jitter gates) + home-zone goal collapse + run-order fix so robot stops BEFORE tf-relay switch. | 33/95 | 158 m | Alignment averaging worked technically perfectly (live yaw std **0.000°** on first attempt vs v9's implicit 1.8° bias). 0-50 m **5× tighter** (0.18 vs 0.50 mean). But 100-150 m WORSE (3.47 vs 1.95) - because v9's alignment bias had been accidentally cancelling real VIO drift in that zone. v10 exposes the unmasked drift, which exceeds the 3 m Nav2 tolerance -> cascade, robot stuck at WP 37. **Usefull negative result**: confirms the fundamental limit past 150 m is VIO itself, not alignment or control. Next would need map reuse (v11). |

Each subdirectory has its own README with numbers and files.

## Root cause timeline

**v1 failure - VIO phantom motion during stops.** Nav2 (even without its
controller) introduced 59.9 s of robot standstill over the run (inter-goal waits).
Synthetic IMU was computing acceleration by double-differentiating PhysX-reported
GT position at 200 Hz; contact-solver jitter (~0.1 mm/step) amplified to ±1.1 m/s²
phantom ax during stationary periods. ORB-SLAM3 integrated that -> 37 m of fake
motion -> Umeyama scale 0.72 / ATE 5.27 m RMSE. Exp 48 (pure pursuit only, no
stops) never saw this. Diagnostic in `v2_scale_correction/results/diagnostics_output.txt`.

**v2_imu_fix - standstill detection.** When GT position drifts < 15 mm over a
100 ms / 20-sample window, output pure gravity (body frame) + Phidgets sensor
noise, don't invoke the derivative chain. Key details that took a second attempt:
- Treshold 15 mm not 2 mm (PhysX jitter itself exceeds 2 mm).
- **Do not** reset `prev_vel_world = 0` on entering stationary - causes a 100+
  m/s² phantom spike on exit (`accel = real_vel / dt`). Keep motion-branch state
  untouched, only override the output.
- Add Phidgets noise in both branches - ORB-SLAM3 bias estimation is ill-
  conditioned on zero-variance input.

Result: first 150 m clean. Then ORB-SLAM3 loop-closing accepted a false loop
closure in repetitive forest (24 `BAD LOOP` rejections earlier; one got through)
-> 30 m map jump.

**v3_no_loop - disable ORB-SLAM3 loop closing.** Added `loopClosing: 0` to
yaml. This *should* work via `System.cc:101-106`, but the live binary segfaults
because `LoopClosing::Run()` doesn't fully honour the flag - the KF-insertion
path still writes and some downstream state isn't guarded. Patched
`LoopClosing.cc::Run()` to early-exit and drain the queue when
`!mbActiveLC`. Rebuilt `libORB_SLAM3.so`.

Result: 0 `BAD LOOP` events, first 100 m <1.5 m err. Still drifted past
~125 m. **Initially interpreted as a "39 m jump at Frame 2100" - this
interpretation was wrong.** v4's frame-by-frame analysis showed per-tick
VIO motion stays ≤ 0.1 m; the "jump" was cumulative drift over 100+ ticks
that looked like a jump when plotted per-keyframe. See the "What v3/v4/v5
together proved" section below.

## comparison to the baseline exp 48

Exp 48 did the same south route at 0.49 m ATE for full 390 m roundtrip, using
pure pursuit only (no Nav2). It had zero stops and no repetitive-feature
challenge because pure pursuit kept moving. Exp 51's value is showing that
adding Nav2 on top of VIO requires multiple fixes (IMU standstill, loop
closing disable, yaml noise correction). With all three applied (v6)
the first 150 m are tight (<1 m err at 100-150 m); beyond that the residual
drift is small and addressable with a scale corrector.

## Files

- `config/`, `scripts/` - v1 (base hybrid) setup
- `v2_scale_correction/` - offline-verified scale-correction proposal (dropped)
  plus the diagnostic scripts that led to the actual root cause
- `v2_imu_fix/` - IMU standstill fix, first working run
- `v3_no_loop/` - additionally disables ORB-SLAM3 loop closing (+ patched
  `LoopClosing.cc` early-exit)
- `v4_anomaly_guard/` - tested single-tick displacement guard; proved the
  "sudden jump" hypothesis wrong
- `v5_scale_correction/` - rolling scale corrector attempt + **IMU noise
  diagnostic that found the real root cause** (noise-model mismatch)
- `v6_imu_noise_fix/` - **current working configuration**: yaml `NoiseAcc 1.5`
  / `NoiseGyro 0.05` matched to measured synth-IMU noise
- `v7_physx_velocity/` - tried replacing synth-IMU with PhysX velocity API,
  7% systematic deficit made it worse; documents why v6 beats v7

## What v3/v4/v5 together proved

The failure-mode story was wrong twice before it was right:

- v2 / v3 thought it was "BA map shift" - a sudden 39 m VIO jump at
   Frame 2100. Wrong interpretation; v4 showed per-tick VIO motion stays
   ≤ 0.1 m with the biggest 10-second displacement at 8.5 m (normal driving).
   No sudden jumps exist.
2. **v4 / v5 thought it was "smooth scale drift"** - a gradual VIO
   under/overshoot that a rolling encoder/VIO ratio could correct.
   v5 attempted this and made err worse (5.7 m at 50–100 m).
3. **v5 diagnostics found the actual root cause**: the synthetic IMU is
   50–176× noisier on the linear axes than `vio_th160.yaml` declares.
   ORB-SLAM3 uses those noise numbers as variance weights in its
   visual-inertial BA. We told it IMU noise std is 0.02 m/s² while the
   real measured std is 3.52 m/s² on ax. BA therefore over-trusts IMU,
   integrates high-frequency noise as if it were signal, and VIO scale
   and direction drift as a result.

Test 2 from the v5 diagnostic rules out "IMU energy is biased": double-
integrating synth IMU in world frame recovers GT path to 1.026× - mean
energy is right, it's the *noise* declaration that's wrong.

**Unit-bug correction (after v7 audit):** my v5 diagnostic reported "IMU
176× noisier than spec" - that was wrong by factor √200 = 14. ORB-SLAM3's
`IMU.NoiseAcc` yaml value is continuous noise *density* (per-√Hz), not
discrete-step std. Correct comparison:

- yaml density × √freq = per-step std expected by ORB-SLAM3
- yaml 0.02 × √200 = 0.28 m/s² expected
- measured 3.87 m/s² -> **13.7× too noisy**, not 176×

v6's 1.5 m/s²/√Hz was the right direction (more noise than yaml claims)
but **5.5× over-corrected**. The calibrated value is 3.87/√200 = 0.275 m/s²/√Hz,
which v8 uses.

Three fixes all converge on the same root cause:
- v2_imu_fix removed phantom motion from PhysX jitter *during stops* -
  necessary for any run with Nav2-induced pauses to work at all.
- v3_no_loop removed false loop closures from repetitive forest -
  necessary to prevent catastrophic single-event map jumps.
- v5 diagnostic identified that even without those two, the noise-model
  mismatch causes ongoing gradual drift.

## verdict: v6 is the working configuration

Both v6 (yaml-adjusted) and v7 (cleaner IMU source) were tried:

- **v6** kept the noisy position-double-diff IMU but told ORB-SLAM3 the
  right noise level (`NoiseAcc 1.5, NoiseGyro 0.05`). Ratio 1.026
  (mean energy correct). Run result: 0.65 m mean err at 100-150 m.
- **v7** replaced the IMU source with PhysX `get_linear_velocity()`
  single-diff -> accel std 0.24 m/s² (16× cleaner than v5) -> yaml back
  to Phidgets 0.02. But PhysX velocity has a **7% systematic deficit**
  vs actual motion -> IMU integrated path 93% of GT -> VIO drifts more
  than v6 (50-100 m err 2.82 vs v6's 1.35).

The lesson: **what matters to VIO is "IMU mean energy matches reality"
more than "IMU noise is low"**. v5's noisy position-double-diff has
ratio 1.026 (mean correct, noise high); ORB-SLAM3 handles high noise
fine as long as yaml declares it. PhysX's velocity is lower-noise but
has bias -> no amount of yaml tuning fixes that.

## Status: full roundtrip works (v9 pinned as baseline)

v9 drives the full south roundtrip: **89/95 WPs, 0 timeouts, closest
approach to final WP 2.91 m** over 373 m. Clean-run (up to closest
approach): mean err 1.63 m, median 1.42 m, max 4.89 m. After that the
robot circles the home-zone goal cluster for another 15 m / 6 m of
added err growth - a goal-sequencer issue at route end, not a VIO
regression (see `v9_robust_nav/RESULTS.md`).

v10 attempted to tighten the 100-150 m zone further by fixing a 1.8°
initial-alignment bias. The fix worked technically (0.000° live yaw std
on first attempt) but revealed that v9's 1.8° bias had been accidentally
partly-cancelling the real VIO drift in the 100-150 m zone. With the
bias removed, unmasked drift crossed the 3 m Nav2 tolerance and caused
a cascade at WP 37 (158 m). v10 got less far than v9 despite cleaner
math. Pinning **v9** as the working baseline; further improvements need
to attack VIO itself (map reuse / Atlas save-load), not the alignment
or control layer.

The working configuration is:

- Synthetic IMU: position-double-diff + standstill fix (`run_husky_forest.py`)
- yaml: `NoiseAcc 0.275, NoiseGyro 0.005, loopClosing 0, Tbc` (v8)
- orb-slam3: rebuilt with `LoopClosing.cc` early-exit patch (v3)
- tf_wall_clock_relay: `--slam-encoder`, 95%/5% blend
- Nav2: planner-server only (no controller, no BT)
- **PP follower**: pure pursuit + anti-spin guard (v9)
- **Goal sender**: `--tolerance 3.0` (v9)

## Next step

Now that the hybrid VIO+Nav2 pipeline works end-to-end on the clean
route, the original exp 50 goal (dynamic obstacles + teach-and-repeat
re-navigation) becomes tractable. v9 config is the baseline.

## reproducibility notes

Each `v*` subdir pins its own `config/` and a snapshot of the IMU
generation code (`scripts/run_husky_forest_v*.py`). But the run scripts
(`run_exp51_v*.sh`) always launch the live master at
`simulation/isaac/scripts/run_husky_forest.py` - that master is mutable
and gets updated as we iterate. To reproduce a specific version:

1. `cp <v*>/scripts/run_husky_forest_v*.py simulation/isaac/scripts/run_husky_forest.py`
2. Use the experiment's own `config/vio_th160.yaml`
3. Run `scripts/run_exp51_v*.sh`

Current master state is v6-style (position-double-diff + standstill fix).
If you reproduce v7 you must copy its PhysX-velocity snapshot over master
first. The ORB-SLAM3 `LoopClosing.cc` patch (v3's early-exit) is baked
into the rebuilt `libORB_SLAM3.so`, not toggled per-run - disabling loop
closing requires both `loopClosing: 0` in yaml and this rebuilt library.

For a fresh experiment (e.g. exp 50 obstacles) that wants to use v6's
working configuration:

- Copy `v6_imu_noise_fix/config/vio_th160.yaml` as the baseline
- Master `run_husky_forest.py` already matches v6 (position-double-diff +
  standstill, as of this writing)
- Rebuilt `libORB_SLAM3.so` with the loop-closing patch is already in
  place
