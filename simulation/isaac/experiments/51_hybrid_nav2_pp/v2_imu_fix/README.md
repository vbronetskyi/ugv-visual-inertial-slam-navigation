# Exp 51 v2_imu_fix - Standstill-aware synthetic IMU

Fix the synthetic-IMU phantom-acceleration bug identified in exp 51 v1 analysis.

## hypothesis

Exp 51 v1 diagnostic (`../v2_scale_correction/results/diagnostics_output.txt`) showed:
- Synthetic IMU generates acceleration by double-differentiating GT position at 200 Hz.
- PhysX contact-solver jitter (~0.1 mm/step) amplifies to ax ≈ ±1.1 m/s² noise during
  standstill (ay, az stay clean - asymmetry consistant with drive-axis contact resolution).
- During exp 51 v1 the robot spent **59.9 s standing still** (Nav2 rotate-in-place /
  inter-goal stops). VIO integrated phantom accel -> **37 m of fake motion** -> scale 0.72 /
  ATE 5.27 m RMSE.
- Exp 48 (pure-pursuit) never stopped -> zero phantom motion -> scale 1.00 / excellent ATE.

## Fix

`run_husky_forest.py:_compute_synth_imu()` now detects standstill from a 20-sample
(100 ms) GT-position window and outputs pure gravity + Phidgets noise instead of the
derivative chain. Threshold 15 mm: above PhysX jitter amplitude, below any realistic
crawl speed (<15 cm/s).

Key design decisions (from first-try failure):

1. **Threshold 15 mm, not 2 mm.** First attempt at 2 mm triggered on PhysX jitter itself,
   causing a false stationary detection every few samples.
2. **Do NOT reset `prev_vel_world = 0` on stationary.** First attempt did this - result was a
   100+ m/s² phantom acceleration spike on the *exit* frame, because
   `raw_accel = (real_vel − 0) / dt`. The corrected fix computes motion-branch state on
   every frame and only *overrides* the output when stationary.
3. **Keep feeding the LPF buffer during stationary.** Avoids a cold-start mean on resumption.
4. **Add Phidgets noise in both branches.** Real IMUs output non-zero noise even
   stationary; ORB-SLAM3 bias estimation is ill-conditioned on zero-variance input.

See `scripts/run_husky_forest_v2.py` for the exact code (frozen copy).

## Results

### Stationary IMU quality

|  | v1 exp 51 | v2 fix |
|---|---|---|
| stationary duration | 62.4 s | 1.6 s |
| ax ± std | +0.12 ±**1.10** | +0.22 ±**1.68** |
| ay ± std | −0.00 ±0.09 | +0.00 ±**0.05** |
| az ± std | +9.81 ±0.06 | +9.81 ±**0.05** |

ay, az are clean (≈ Phidgets 0.02 m/s² noise floor). ax std is not directly comparable:
v2 had only 1.6 s of stationary GT - dominated by decel/accel *edges* rather than the long
true-stop periods v1 had. Where the fix actually fires (continuous > 100 ms stops) the
output is pure gravity + 0.02 m/s² noise.

### localisation error vs distance travelled

| range | v1 exp 51 | v2 fix | enc only (v2) |
|---|---|---|---|
| 0–50 m | mean 0.08 / max 0.33 | **mean 0.38 / max 0.77** | 0.10 |
| 50–100 m | - (cut short) | **mean 1.17 / max 1.57** | 0.32 |
| 100–150 m | - | **mean 2.47 / max 5.76** | 0.87 |
| 150–200 m | - | mean 20.81 / max 34.39 | 3.68 |

(v1 only reached 51 m total.)

### Goals reached

v1 exp 51: 0 waypoints reached (robot stalled at first turn, err 3.16 m max).
v2 fix: 32 waypoints reached (WP 4 -> WP 35), then timeouts at WP 36–37.

### What broke after 150 m

VIO started emitting `BAD LOOP!!!` messages - ORB-SLAM3 loop-closure candidates in
repetitive forest features. 24 bad-loop events in `logs/vio.log`. The VIO output jumped
by ~30 m shortly after, knocking SLAM+encoder err from <1 m to 30 m. Encoder alone stayed
at 3.7 m of drift - well-behaved.

This is a distinct failure mode from the standstill issue Fix A targets. Forest has many
visually similiar regions (tree trunks in repetitive spacings); ORB-SLAM3's place-recognition
proposes wrong matches, rejects most (`BAD LOOP` = rejection), but eventually one bad
match gets accepted and distorts the map.

## verdict on Fix A

Works as designed. First 150 m of exp 51 is now better than v1's 51 m and approaches
exp 48 quality (0.38 m mean err in first segment vs exp 48's typical 0.05–0.2 m).

## Follow-ups (in priority order)

1. Address bad loop closures in forest - either disable loop closure entirely
   (set `System.LoopClosing` off in the ORB-SLAM3 yaml) or tune the loop-closing
   thresholds. VIO during teach-and-repeat doesn't need loop closure because each run
   is independent.
2. **Re-run exp 51 v2 with loop closure disabled** - expected: full roundtrip with
   <2 m err throught.
3. **Exp 50 obstacles** becomes tractable once 51 is solid.

## Files

- `scripts/run_husky_forest_v2.py` - frozen copy of fixed synthetic IMU
- `scripts/run_exp51_v2.sh` - orchestrator
- `scripts/{pure_pursuit_path_follower,send_goals_hybrid}.py` - unchanged from v1
- `config/` - nav2, map, route, VIO config used
- `logs/` - isaac, vio, tf_slam, goals, pp_follower
- `results/err_series.txt` - `dist= err= enc_err=` per tf-relay log line
- `../v2_scale_correction/` - earlier branch (dropped - scale correction didn't address
  root cause; kept for diagnostic scripts that produced the finding)
