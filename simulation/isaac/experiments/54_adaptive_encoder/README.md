# exp 54 - real-speed cruise + adaptive SLAM/encoder blend

## Motivation (from exp 53 run 2 analysis)

Exp 53 reached 37/94 WPs at 540 m before SLAM drifted past recovery. The
root cause had two linked parts:

1. **Proximity limiter throttled robot to ~0.1 m/s** (28 % of ticks
   hit LETHAL) instead of the configured 0.25 m/s. Over 540 m this
   took 5 400 s.
2. **IMU bias walked freely for 5 400 s** with only 2 VIBA events.
   Gyro-bias density 0.001 rad/s²/√Hz × √5400 ≈ 4.2 °/s by the end
   -> lateral drift that grew linearly with distance. Pure yaw-bias
   signature: almost all error in Y axis.

The slow speed was the amplifier. A real Husky cruises at ~1 m/s.
Going half-speed doubled the time-for-bias-walk for every metre.

## Changes vs exp 53

### 1. `pure_pursuit_path_follower.py` - real-speed cruise

| Param | Exp 53 | Exp 54 |
|-------|--------|--------|
| `MAX_VEL` default | 0.25 m/s | **1.0 m/s** |
| `V_SLOW` (near inflation) | 0.18 m/s | **0.5 m/s** |
| `V_WARN` | 0.12 m/s | - (removed, 2-tier only) |
| `V_LETHAL` | 0.08 m/s | **0.2 m/s** |
| `PROX_SAMPLE_DIST` | 0.2-0.8 m | **0.4-1.4 m** (longer for 1 m/s stopping distance) |
| `PROX_COST_SLOW` | 60 | 50 |

Tier collapse: only SLOW and LETHAL now, no WARN middle. Simpler, less
ticking between regimes.

### 2. `tf_wall_clock_relay_v54.py` - adaptive SLAM+ENC blend

Previous: `SLAM_ALPHA = 0.95` hard-coded (95 % SLAM / 5 % encoder)
whenever SLAM tracking ok. Encoder was only a fallback when SLAM lost.

New: blend weight depends on SLAM-vs-encoder disagreement
(the only GT-free estimate of SLAM trust):

| `‖nav_slam − nav_enc‖` | `SLAM_ALPHA` | Rationale |
|------------------------|--------------|-----------|
| < 2 m     | 0.95 | SLAM and encoder agree -> trust VIO |
| 2 – 5 m   | 0.70 | SLAM slightly off - blend more encoder in |
| 5 – 10 m  | 0.40 | SLAM likely drifting - encoder primary contributor |
| > 10 m    | **0.10** | SLAM untrusted - lean heavily on encoder |

When SLAM is diverging (gyro bias walk with no VIBA updates), this
shifts the fused pose onto a predictable drifter (encoder ~3 %/m) instead
of an exploding one (SLAM, unknown drift rate).

Log line now includes `alpha=X.XX slam-enc=Y.Ym` so we can see the
blend working at runtime.

## Obstacle layout

Same as exp 52 run 4 / exp 53 (for direct comparability):

| Group | Position | Count |
|-------|----------|-------|
| 0 | x=−75 | 3 cones |
| 1 | x=−18 | 2 cones |
| 2 | x=+5  | 4 cones |
| Tent | (−45, −38) - on route | 1 × 2×1.8 m |

## files changed vs exp 53

```
scripts/pure_pursuit_path_follower.py   # cruise speed + 2-tier prox
scripts/tf_wall_clock_relay_v54.py      # adaptive blend (copy of global relay)
scripts/run_exp54_repeat.sh             # routes to v54 tf_relay + 1 m/s PP
config/nav2_*.yaml / .py                # E53 paths -> E54
```

All other logic (proactive WP projection, anti-spin, teach map,
obstacle patcher) is unchanged from exp 53.

## Reproducing

```bash
bash scripts/run_exp54_repeat.sh
# Output: results/repeat_run/ (override with REPEAT_OUT_DIR)
```

## Results

_Pending - run in progress._
