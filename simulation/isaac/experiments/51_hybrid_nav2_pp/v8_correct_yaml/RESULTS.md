# v8 run results

## Final stats

- **Distance travelled: 185 m**
- **Waypoints reached: 34 / 94**
- **Timeouts: 1** (WP 36)
- **WP 39 stuck** - robot spinning in place, d=6-7 m wouldn't decrease

## Error per distance bin

| range | n samples | err mean | err max | enc_err mean |
|---|---|---|---|---|
| 0-50 m | 83 | **0.47 m** | 0.95 | 0.06 |
| 50-100 m | 88 | 1.43 | 1.93 | 0.12 |
| 100-150 m | 104 | 1.59 | 3.24 | 0.89 |
| 150-200 m | 110 | 5.69 | 7.65 | 2.77 |

## Cross-comparison

| bin | v3 | v4 | v6 | **v8** |
|---|---|---|---|---|
| 0-50 m | 0.38/0.79 | 0.28/0.59 | 0.46/0.93 | **0.47/0.95** |
| 50-100 m | 1.14/1.43 | 0.94/1.17 | 1.35/1.94 | **1.43/1.93** |
| 100-150 m | 10.81/45.59 | 2.56/6.75 | **0.65/1.01** | 1.59/3.24 |
| 150-200 m | 42.69/47.80 | 17.21/26.26 | - stopped | **5.69/7.65** |

v8 reaches farther than v6 stopped at (110 m) but drifts more at 100-150 m.
Against v4 (which ran the full route), v8 is 3× better at 150-200 m.

## Why drift grows past 150 m - NOT IMU

Gyro integration analysis showed the IMU correctly tracks the robot's
*actual* motion - which becomes very violent past 150 m:

- GT yaw at t=200-250s oscillates ±100° every 10 seconds
- Actual yaw rate 20-36 °/s (robot physically spinning in place)
- IMU gz correctly reads these rates

Root cause is a **Nav2 feedback cascade**:

1. Around 150 m, VIO err grows to ~3 m (normal in dense forest)
2. Nav2 `goal-tolerance` is 1.5 m -> thinks robot hasn't reached WP
3. Nav2 issues rotate-in-place commands (`v=0.07 w=0.80`) as recovery
4. Robot spins in place -> each rotation adds to VIO uncertainty
5. VIO err grows -> more spinning -> cascading failure
6. Robot gets stuck in circles around a waypoint

This is a **control-loop/tolerance problem, not a SLAM quality problem**.
v8 IMU and ORB-SLAM3 are behaving correctly. The Nav2 hybrid setup
doesn't tolerate err > 1.5 m gracefully.

## What would fix this

1. **Increase `goal-tolerance` from 1.5 m to 3 m** in
   `send_goals_hybrid.py` -> avoids the spin-in-place cascade
2. **PP follower should time out and advance to next WP** after N seconds
   instead of waiting for d<tolerance
3. **Dampen rotation recovery** - if robot spins >180° without translating,
   Nav2 should cancel that goal

## Verdict on v8 vs v6

**v8 (NoiseAcc=0.275) vs v6 (NoiseAcc=1.5):** roughly equivalent for the
first 100 m, differ after that. Neither clearly wins - v6 stopped at
110 m so we can't compare 150 m+.

For the thesis, either is acceptable:
- v8 is the theoretically correct calibration
- v6's "wrong" 5.5× value happens to also work

The real limiting factor is the Nav2 tolerance feedback, not the IMU
noise model.

## Files

- `results/err_series.csv` - per-sample (dist, slam_err, enc_err)
- `logs/` - full run logs (185 m traveled, 34 WP reached)
