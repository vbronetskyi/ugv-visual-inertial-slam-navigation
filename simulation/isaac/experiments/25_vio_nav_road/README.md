# exp 25: VIO round-trip limitation + SLAM tracking loss in Nav2 recovery

## goal

Run Nav2 with obstacles on road route using the "best localization". After exp 23's
claim of VIO road ATE 0.089m, try VIO for mapping + live localization for navigation.

## key findings (both are fundamental limitations, not bugs)

### finding 1: VIO road 0.089m ATE is outbound-only

Exp 23 reported 0.089m ATE on road. Reproduction instructions in exp 23 README
use `head -1500 associations.txt` - i.e. only the outbound half (0-150s, before
turnaround). Missed this on first read.

Running VIO on full round-trip (3381 frames, 340s, out+turnaround+return) gives
**1.8-33m ATE** with 2/8 runs showing scale collapse (scale 0.01-0.08). The same
180° turnaround that breaks VIO on forest routes (exp 23/24) also breaks VIO on
road.

| Segment | ATE | Scale | Stability (runs) |
|---|---|---|---|
| Outbound only (head -1500) | **0.14-0.15m** | 0.994-0.997 | 3/3 consistent |
| Full round-trip (3381 frames) | 1.8-33m | 0.02-1.0 | 8/8 inconsistent |

**Conclusion**: VIO is only usable for one-way mapping runs. For round-trip
navigation, it cannot be used as a live localization source.

### Finding 2: SLAM-frame mode is fragile to Nav2 recovery behaviors

Attempted Plan B: RGBD-only live SLAM in SLAM-frame mode (no world alignment)
with VIO outbound waypoints. Nav2 with RPP controller, 1.5m inflation, obstacles
spawned.

What happened (run 1):

1. `0-70s`: Nav2 startup, robot waits at spawn (wp 1-2 not reachable until local
   costmap initialized). New STUCK retry logic correctly waits instead of cascading.
2. `70-140s`: Robot drives outbound wp 1-15 successfully. ATE drift 1-2m, fine.
3. `140s`: Robot approaches obstacle 1 at GT `(-50, -6)`. Nav2 path planner fails
   (path blocked by inflation zone around cone). Nav2 BT escalates to **Recovery**
   behavior tree -> BackUp action.
4. `140-278s`: Nav2 alternates between planner failures and BackUp recoveries.
   Robot oscillates locally while drifting ~20m in world frame.
5. `278s`: **RGBD-only SLAM reports `Fail to track local map!`**, triggers full
   atlas reset:
   ```
   LM: Active map reset, Done!!!
   mnFirstFrameId = 0
   Frame 2400 t=278.4 pos=(0,0,0)    <- SLAM reset to origin
   New Map created with 456 points
   ```
6. After reset, SLAM frame jumps: `tf_wall_clock_relay` publishes the new
   SLAM pose starting from (0,0) relative to where robot was at reset. All
   waypoints (which were in old SLAM frame) now appear 80+m away.
7. Nav2 sees far-away goal -> infinite planning failures -> infinite backup.
   Robot regresses from `d_start=69m` to `d_start=58m` in 90s.

**Root cause**: SLAM-frame mode assumes the SLAM coordinate frame is stable accross
the entire nav run. When ORB-SLAM3 loses tracking (inevitable during aggressive
backup + rotation around obstacles), it resets the atlas and the frame jumps.
Waypoints become meaningless in the new frame.

**Why RGBD-only loses tracking here and not in exp 13:**

- Exp 13 used a custom drift-corrected world frame (fused GT odom + SLAM), so a
  SLAM reset caused a visible jump but didn't break navigation - world frame
  continued via odom while SLAM recovered.
- Exp 25 uses pure SLAM-frame mode - no fallback when SLAM breaks.

## code improvements made this session (usefull for future experiments)

### 1. Sliding-window waypoint sanitization

`scripts/send_nav2_goal.py:117-140` - replace global-mean clamping with a local
sliding window. Global mean doesn't work on routes with natural bends (e.g. road
curves) because the mean drifts. Window-local mean handles curves correctly.

```python
def _sanitize_waypoints(self, waypoints, max_lateral_dev=4.0, window=10):
    for i, (x, y) in enumerate(waypoints):
        start = max(0, i - window); end = min(len(waypoints), i + window + 1)
        local_mean_y = sum(wp[1] for wp in waypoints[start:end]) / (end - start)
        clamped_y = max(local_mean_y - max_lateral_dev,
                        min(local_mean_y + max_lateral_dev, y))
```

### 2. Waypoint timeout skip

`scripts/send_nav2_goal.py:216-233` - if robot sits on the same waypoint for more
than 20s without reaching it, skip forward. Prevents indefinite hangs at a
borderline-unreachable waypoint.

### 3. No-progress retry (replaces the old cascade false-positive handler)

`scripts/send_nav2_goal.py:322-360` - the old handler would cascade through
waypoints in 10ms increments when Nav2 returned immediate "success" for aborted
goals. New logic:

- Record robot position at goal send time (`_goal_send_rx/ry`)
- When result arrives, check both `real_dist > 2.5` (wp not actually reached)
  AND `moved < 0.5` (robot didn't make progress since goal send) AND
  `goal_age < 2.0` (result came back too fast -> Nav2 abort)
- If all three: no-progress retry (wait 2s, retry same wp, 10 attempts max)
- After 10 retries: skip waypoint

Prevents the 60-waypoint cascade bug in 30 seconds. Stuck recovery is handled
more gracefully.

## Results

### VIO mapping (offline on recorded road)

`results/vio_mapping.png` - GT road trajectory vs VIO (SLAM frame) trajectory +
extracted waypoints.

`results/vio_ate_road.png` - VIO vs GT aligned with Umeyama (Sim3), ATE over time.
Shows characteristic pattern: 0.14m ATE for outbound, scale collapse after
turnaround for full round-trip.

### Nav2 run 1 (RGBD-only SLAM-frame + obstacles + VIO outbound waypoints)

**FAILED** at wp 16 around `(47, 1)` SLAM / `(-50, -6)` GT (obstacle 1 area):

| Metric | Value |
|---|---|
| Waypoints reached | 15/60 (outbound only) |
| GT distance covered | ~44m forward, then ~15m backward (regressing) |
| Failure mode | RGBD-only SLAM tracking loss -> atlas reset -> SLAM frame jump |
| Duration before giving up | 420s (ran to Nav2 recovery infinite loop) |

Logs: [logs/nav2_goal_run1.log](logs/nav2_goal_run1.log),
[logs/nav2_isaac_run1.log](logs/nav2_isaac_run1.log),
[logs/slam_rgbd_run1.log](logs/slam_rgbd_run1.log)

## takeaways for future navigation experiments

1. **Don't use SLAM-frame mode with RGBD-only live** on routes with aggressive
   obstacle avoidance. It's too fragile when BT recovery runs.
2. **Prefer world-frame mode with drift correction** (exp 13 approach) - SLAM
   resets are handled gracefully via odom fallback.
3. Limit Nav2's recovery aggressiveness. Options:
   - Disable BackUp recovery completely (use only Spin + Wait)
   - Increase inflation to keep the planner further from obstacles
   - Lower recovery retry count
4. **Waypoint source independent of localization**: best source for road
   navigation is GT outbound path from a mapping run, which has zero drift and
   is known-correct. VIO is marginally better (0.15m vs 0.27m RGBD ATE) only
   for one-way runs and costs a lot in fragility.

## Files

- `config/slam_waypoints_road.json` - 60 outbound + 60 return wp extracted from
  VIO outbound trajectory in SLAM nav frame (kept for reference)
- `results/vio_mapping.png` - VIO trajectory + waypoints overlay
- `results/vio_ate_road.png` - VIO vs GT alignment plot
- `results/vio_road_outbound.txt` - best VIO CameraTrajectory (3 runs, 0.144m ATE)
- `logs/nav2_goal_run1.log` - goal sender log (shows STUCK cascade)
- `logs/nav2_isaac_run1.log` - Isaac log with robot pose over time
- `logs/slam_rgbd_run1.log` - rgbd_live log showing atlas reset at frame 2400

## Run 2 - SLAM-friendly fixes, physical stuck instead

After run 1's findings, four fixes applied to protect RGBD SLAM tracking
from Nav2's recovery behavior:

### Fix 1: Remove BackUp from Nav2 BT

`config/nav2_bt_simple.xml` - replaced `BackUp` with a second `ClearEntireCostmap`
(global). Reasoning: BackUp's aggressive reverse motion breaks RGBD feature
tracking during obstacle recovery.

### Fix 2: SLAM atlas reset detection + offset recovery

`scripts/tf_wall_clock_relay.py` - added `SLAM_RESET_JUMP` detection. When raw
SLAM position jumps >5m in one tick (atlas reset), compute an offset so the
fused pose stays continuous in the original SLAM frame:

```python
new_offset_x = fused_x - raw_nav_x  # fused_x is last known good pose
new_offset_y = fused_y - raw_nav_y
# Subsequent raw poses get this offset added, so waypoints remain valid
```

Plus a 3-second cooldown during which SLAM input is ignored and the fused pose
is held from odom only.

### Fix 3: cmd_vel angular velocity clamp

`scripts/run_husky_nav2.py:538-544` - clamp `ang_z` to ±0.3 rad/s before
differential drive. Prevents any aggressive yaw rate (from recovery or from
RPP on tight turns) that would blur camera frames and break feature tracking.

### fix 4: ORB features 2000 -> 3000, lower FAST thresholds

`rgbd_d435i_v2_mapping.yaml` - `nFeatures 2000->3000`, `iniThFAST 15->10`,
`minThFAST 5->3`. More features extracted per frame, lower detection treshold
= more tracking survivability when camera motion is fast.

### Run 2 results

| Metric | Run 1 (no fixes) | Run 2 (all 4 fixes) |
|---|---|---|
| Waypoints reached | 15/60 | **20/60** |
| GT forward distance | 44m -> regressing | 48m (held) |
| SLAM atlas resets | 1 (catastrophic) | **0** |
| SLAM "Fail to track" events | 1 | **0** |
| Outcome | Nav2 backup loop | Physical stuck at obstacle |

Run 2 shows the SLAM protection fixes work. RGBD SLAM stayed healthy the entire
run: 0 resets, 0 tracking failures. The SLAM reset detection (fix 2) was not
triggered because the underlying cause (aggressive recovery motion) was already
eliminated by fixes 1 + 3.

### New failure mode: physical stuck without BackUp

Robot drove outbound to GT `(-45.7, -9.0)` around obstacle 1, got physically
caught on the obstacle's geometry, and stayed there for 120+ seconds. Nav2
commanded `(0.30, 0.30)` repeatedly - forward + max allowed turn - but the
robot couldn't escape because:

1. Without BackUp, there's no way to physically reverse out of a wedged position
2. Ang_vel capped at 0.3 rad/s is gentle enough that the robot can't pivot out
3. RPP's linear velocity is regulated by cost -> drops near zero in inflation zone

After ~120s of no progress, Nav2's 8-retry RecoveryNode exhausts and it reports
`Goal failed`. Goal sender skips ahead 18 waypoints at STUCK timeout but robot
still doesn't move. Run abandoned.

Fundamental tradeoff: BackUp is necessary to escape physical traps, but
breaks SLAM when triggered aggressively. A middle-ground fix would be a gentle
BackUp with `backup_speed: 0.1, backup_dist: 0.3` (vs old `0.25, 1.0`) - slow
enough that SLAM can track through it. Future experiment.

## Next steps

For future experiments targeting road navigation with obstacles:

- Gentle BackUp: re-enable BackUp with `backup_speed=0.1, backup_dist=0.3`
   - slow reverse, short distance. The ang_vel clamp + increased ORB features
   should keep SLAM healthy even during this.
- Or use exp 13's world-frame drift-corrected pipeline: the drift-corrected
   SLAM->world fusion recovers from SLAM tracking loss via odom fallback, so
   aggressive Nav2 recovery doesn't destroy the whole run.
3. GT-derived waypoints instead of VIO: since VIO's 0.15m accuracy only
   applies to outbound and is worse than the 0.3m drift of in-run RGBD SLAM,
   there's no practical advantage. Use GT from a mapping run directly.

Run 2 logs: [logs/nav2_goal_run2.log](logs/nav2_goal_run2.log),
[logs/nav2_isaac_run2.log](logs/nav2_isaac_run2.log),
[logs/nav2_tf_run2.log](logs/nav2_tf_run2.log),
[logs/slam_rgbd_run2.log](logs/slam_rgbd_run2.log).
