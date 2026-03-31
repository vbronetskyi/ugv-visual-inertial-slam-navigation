# Exp 51 v9_robust_nav - fix Nav2 feedback cascade past 150 m

## problem

After v8 ran the full hybrid stack with correctly calibrated IMU noise and
got 34 waypoints / 185 m, the robot got stuck on WP 39 in a spin-in-place
loop despite no physical obstacle. Investigation showed this is not a
SLAM or IMU failure - it is a closed-loop interaction between:

- VIO's natural accuracy limit in dense forest (~3 m err past 150 m)
- Nav2's 1.5 m goal-tolerance (tighter than the VIO accuracy it sees)
- The pure pursuit control law reaction to bad localization

### exactly what happens

1. Around 150 m, ORB-SLAM3's reported pose starts lagging GT by 3-4 m -
   normal in a repetitive-feature environment.
2. `send_goals_hybrid.py` checks `d = |robot_nav_pose − waypoint|` against
   `--tolerance 1.5`. The robot may be *physically* at the waypoint, but
   `robot_nav_pose` is 3 m off, so `d > 1.5` and Nav2 "re-requests" a plan.
3. The new plan is computed from `robot_nav_pose` (the wrong position).
   Its first few poses are near that wrong position - which is *behind*
   where the robot physically is.
4. Pure pursuit picks a lookahead point 2 m along the plan. Because the
   plan starts behind the robot, the lookahead point can end up *behind*
   the robot too.
5. PP computes `heading_err = angle_to_target - robot_yaw` -> close to
   ±180°. The saturation kicks in:
   - `cmd.linear.x = 0.25 * max(0.3, 1 − |err|/1.57) ≈ 0.075 m/s` (crawl)
   - `cmd.angular.z = clamp(GAIN × err, −0.8, 0.8) = ±0.8 rad/s` (max spin)
6. Robot rotates in place trying to "face" the target it thinks is behind
   it. Each rotation adds ORB-SLAM3 uncertainty (repetitive features,
   rotation-induced feature loss). VIO err grows more.
7. Nav2 plan still doesn't match physical position -> steps 2-6 repeat.
   Cascade: the more the robot spins, the worse VIO gets, the less
   recoverable the situation.

Symptoms observed in v8 at WP 39: robot physically drove an 11 m arc over
50 seconds but `d` toward WP 39 never decreased - robot was circling.

### why this is a control-loop problem, not an algorithm bug

The pure pursuit law, the IMU, and the VIO all behave correctly. The
planner returns valid paths. Each part works in isolation. But the
feedback interconnection (`plan <- nav_pose <- VIO <- physical motion <- PP
output`) has a runaway mode when VIO err exceeds the Nav2 tolerance: the
system reinforces its own localization uncertainty.

## Fixes in v9

Two changes that break the cascade, neither of which touches SLAM or IMU:

### 1. `--tolerance` raised from 1.5 -> 3.0 m

In `run_exp51_v9.sh`:
```
--tolerance 3.0   # was 1.5
```

Bigger than the 3 m VIO err we expect past 150 m, so Nav2 accepts the WP
as reached and advances to the next one before the cascade can start.
Still well under the 4 m route-waypoint spacing, so waypoints don't get
skipped unintentionally.

### 2. Anti-spin guard in `pure_pursuit_path_follower.py`

Runs alongside the pure pursuit law. Tracks a rolling position history
and declares a "spin-in-place" situation when all three hold:

- Current command has `|w| ≥ 0.5` rad/s (PP is emitting a turn)
- Current command has `|v| ≤ 0.1` m/s (not really translating)
- Robot displaced < 0.5 m over the last 5 seconds

On detection, force a **cooldown** of 3 s: emit `w=0, v=0.15 m/s` (creep
forward, no rotation). This gives VIO a chance to re-anchor against
straight-line motion before PP resumes normal control.

Parameters are conservative - during normal driving (even tight turns),
|v| stays well above 0.1 and displacement in 5 s well above 0.5 m, so
the guard never fires during healthy operation.

### what v9 does not change

- Master `run_husky_forest.py`: still v6-style IMU (position-double-diff +
  standstill fix)
- `vio_th160.yaml`: v8 calibrated noise (NoiseAcc 0.275, NoiseGyro 0.005,
  loopClosing 0)
- ORB-SLAM3 binary: same patched library from v3

## expected outcome

- Robot should clear 150 m zone without the WP 39 stall v8 hit.
- Anti-spin guard should activate 0-1 times (not continuously) - if it
  keeps firing, something else is wrong.
- ATE past 150 m will still be a few meters (VIO limit in forest), but
  the route should complete instead of collapsing.

## What to watch

- `grep "ANTI-SPIN" logs/pp_follower.log` - each activation is logged
- `send_goals_hybrid.log` - WPs reached / timeouts
- `tf_slam.log` - err per bin, same as before

If v9 reaches significantly more waypoints than v8's 34/94 (or completes
the full route), the cascade hypothesis is confirmed and the fix works.
If it still stalls past 150 m despite the guard, the problem is elsewhere
(e.g. the plan itself is wrong, or VIO is actually diverging hard).

## files

- `config/` - same as v8 (calibrated yaml)
- `scripts/pure_pursuit_path_follower.py` - anti-spin PP follower
- `scripts/send_goals_hybrid.py` - unchanged goal sender (tolerance set
  from command line in run script)
- `scripts/run_exp51_v9.sh` - orchestrator with `--tolerance 3.0`
- `scripts/run_husky_forest_v6.py` - frozen IMU generator (same as v6/v8)
