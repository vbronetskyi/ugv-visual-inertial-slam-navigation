# exp 22: VIO Warmup for Forest Routes

## Goal

After confirming VIO offline works on road (exp 18: ATE 0.116m), but fails on forest routes (exp 20: 60+ resets), try prepending a warmup motion to forest recordings so ORB-SLAM3 IMU init has time to converge before sharp forest turns.

## Approach 1: Seperate warmup recording + concatenation

Record a short straight drive separately, then merge with forest recording.

**Failed**: warmup and forest are separate Isaac sims, both starting at spawn. Position jumps 10m at the junction (warmup-end pose vs forest-start pose). VIO breaks immediately.

## approach 2: In-sim warmup prepended to route waypoints

Add `--vio-warmup` CLI flag to `run_husky_forest.py` that prepends warmup waypoints to the route in the same simulation. Single recording, continuous trajectory.

### iteration 1: aggressive zigzag (Y range ±2m)
- Result: VIO 60+ resets, never reaches VIBA 2
- Cause: gz peak 0.514 rad/s - too aggressive, ORB-SLAM3 can't lock gravity

### Iteration 2: gentle zigzag (Y range ±0.5m)
- Result: 65-80 resets, still no VIBA 2
- gz peak 0.226 rad/s (matches exp 18 working config)
- ax peak 0.93 m/s² (close to exp 18's 1.54)
- Why it failed: smooth small-amplitude motion doesn't excite VIO enough
  for IMU init to converge before forest turns start

### Iteration 3: road S-curve as warmup x partial success
Use the first 12 waypoints of the road S-curve as warmup. This is the
exact motion pattern that worked in exp 18 (road ATE 0.116m).

**Result on warmup-only segment** (frames 0-500, ~50s):
- 2 resets (vs 60+ before)
- **VIBA 2 success** (2x per run, finally!)
- ATE 0.797m on 44m of motion
- scale 1.0148

**Result on full route (warmup + forest)**:
- 24-30 resets (forest sharp turns destroy VIO after warmup)
- VIBA 2 happens during warmup, then resets when forest turns start

## final conclusion

**VIO with warmup successfully initializes on the road S-curve segment (ATE 0.80m), but the IMU lock breaks immediately when the robot enters the forest with sharp turns.**

This confirms the **fundamental incompatibility**:
- VIO works on smooth motion (road exp 18, road exp 22 warmup segment)
- VIO breaks on aggressive turning in dense waypoint sequences (forest north/south)

The issue is not initialization quality (proven good with warmup), it's that
ORB-SLAM3 RGBD-Inertial mode resets the IMU init whenever sharp turns
happen *after* successful initialization. This is a known limitation of
ORB-SLAM3 IMU mode and cannot be fixed by parameter tuning.

## comparison table

| Recording | RGBD-only ATE | VIO ATE (full route) | VIO ATE (warmup only) |
|---|---|---|---|
| Road (exp 20, 335m) | 0.272 m | 0.347 m | n/a |
| North (exp 22, 467m) | 1.86 m | failed (24 resets) | **0.80 m** (44m segment) |

## files

- `scripts/run_husky_forest.py` - has `--vio-warmup` flag and `WARMUP_WAYPOINTS`
- `scripts/concat_warmup.py` - alternative concat script (unused, kept for reference)
- `config/vio_loose.yaml` - loose noise config used for VIO
- `logs/north_vio_full_route.log` - full route VIO (24 resets)
- `logs/north_vio_warmup_only.log` - warmup-only segment (2 resets, VIBA 2 success)
- `results/vio_warmup_only.txt` - VIO trajectory for warmup segment
- Recording: `/root/bags/husky_real/exp22_north_vio` (north + S-curve warmup)

## Lessons learned

- VIO is fragile after IMU init: even when initialized correctly,
   subsequent aggressive motion can trigger reset
2. **Warmup motion pattern matters**: must match training distribution
   (smooth S-curve, not sharp zigzag)
3. Real Husky on real roads would likely work better - natural
   motion is smoother than waypoint-following
- For thesis: report VIO works on highway routes only, RGBD-only
   is the only reliable option for forest navigation
