# exp 53 - proactive re-route + proximity speed limit

## Motivation

Exp 52 run 4 failed in a new way: robot drove into the on-route
tent at (−45, −38). Causal chain:

1. Depth saw tent, Nav2 costmap marked it occupied + inflated.
2. Planner routed around it; plan published (~30 m detour for 6 m
   direct).
3. While robot executed detour, SLAM err grew 0.5 -> 2.4 m (tight
   maneuver in low-feature area near the tent).
4. Plan was correct **in nav frame**, but nav frame was 1.5 m off GT,
   so in GT the robot gradually pushed into the tent's footprint.
5. Contact with tent broke tracking (4 lost frames) ->
   ORB-SLAM3 re-localized against early keyframes near spawn
   -> nav pose jumped 50 m -> Nav2 costmap became wrong -> plans failed.

The exp 52 v9 hybrid stack had no defence against steps 4-5: the PP
follower knew nothing about obstacles, it just tracked the plan.

## what this experiment adds (vs exp 52)

Two targeted changes to exp 52's code, both under 80 lines, both
purely additive:

### 1. `pure_pursuit_path_follower.py` - proximity speed limiter

PP now subscribes to `/global_costmap/costmap` and, every tick, samples
the cost in a forward arc (0.3–1.5 m ahead of robot, ±0.3 m lateral).
Caps `cmd.linear.x` by the MAX cost seen:

| Max cost ahead | Speed cap | Why |
|----------------|-----------|-----|
| < 30           | `MAX_VEL` (0.25 m/s) | Free |
| 30–69          | 0.15 m/s  | Near inflation edge - start slowing |
| 70–98          | 0.08 m/s  | Inside inflation gradient - crawl |
| ≥ 99 or `unknown` | 0.03 m/s | Near stop - contact imminent |

Inflation radius is 1.2 m, so cost 30 roughly starts ~0.9 m from the
occupied centre. The robot begins slowing **before** contact range,
giving SLAM time to tighten and the planner time to re-plan without
the drift-in-motion feedback that killed run 4.

The existing v9 anti-spin layer is preserved untouched.

### 2. `send_goals_hybrid.py` - proactive WP projection

Goal sender subscribes to `/global_costmap/costmap`. On **every**
costmap update it rebuilds a `projected_wps` list from the original
route:

- For each future WP (current..end), check the cost at the WP cell.
- If cost ≥ 30 (inflated zone), BFS outward from that cell for the
  nearest cell with cost < 30 (capped at 3 m search radius).
- If BFS finds one, the projected WP for that index becomes the free
  cell's centre.
- If BFS finds none within 3 m, mark that WP `skip=True`; sender will
  move to WP+1 without attempting it.

Projected positions, not original, are used for `ComputePathToPose`.
Projection happens **as soon as the obstacle appears in the costmap**,
not when the robot gets close to it - so the plan the PP executes
already avoids the obstacle from the moment it's seen.

This stops run 3's failure mode (WP geometrically inside a cone wall
-> tolerance=1.0 m lets robot touch the cone).

## obstacle layout

Same as exp 52 run 4 (for direct comparison):

| Group | Position | Count |
|-------|----------|-------|
| 0 | x=−75, y=−24..−26 | 3 cones |
| 1 | x=−18, y=−24..−25 | 2 cones |
| 2 | x=+5,  y=−17..−20 | 4 cones |
| Tent | (−45, −38) - on route | 1 × 2×1.8 m |

Teach map (`teach/south_teach_map.*`) is a direct copy from exp 52 -
same depth-sensed static map, same VIO, same IMU config.

## files changed vs exp 52

```
scripts/pure_pursuit_path_follower.py   # + proximity speed limiter, ~55 new lines
scripts/send_goals_hybrid.py            # + proactive WP projection, ~70 new lines
scripts/run_exp53_repeat.sh             # E52 paths -> E53 paths
config/nav2_planner_only.yaml           # teach map path -> E53
config/nav2_launch_hybrid.py            # config + map paths -> E53
```

Everything else (Isaac scene, synthetic IMU, tf_wall_clock_relay,
ORB-SLAM3 VIO config, turnaround supervisor, costmap snapshotter,
plan logger, teach map, obstacle patcher) is byte-identical to
exp 52.

## reproducing

```bash
bash scripts/run_exp53_repeat.sh
# Output lands in results/repeat_run/
# override with: REPEAT_OUT_DIR=.../results/repeat_run_N bash ...
```

## Expected improvements over exp 52 run 4

- No tent contact - proximity limiter caps speed to 0.03 m/s when
  the robot's forward arc is inside the tent's inflation, preventing
  the drift-in-motion collision.
- **Cleaner detour path** - proactive projection moves WPs 17–19 (on
  top of tent's footprint) to the side once the tent enters the
  costmap, so the plan doesn't have to do a tolerance-based
  near-miss.
- Less SLAM drift during maneuvers - slower motion near obstacles
  -> smaller frame-to-frame parallax changes -> tighter VIO tracking.

## results

### run 1 - aggressive proximity limiter (archived)

First draft of PP proximity limiter was too wide (±0.3 m lateral × 1.5 m
ahead) with cost thresholds that treated normal tree inflation as
LETHAL. Robot froze at WP 6 (GT (−82.6, −24.8)) with 88 LETHAL hits
in 5 min, `v_cap = 0.03 m/s`, never progressed. Artifacts in
`results/run1_too_aggressive/`.

Fix applied:

| Param             | Run 1 | Run 2 |
|-------------------|-------|-------|
| `PROX_SAMPLE_DIST`| 0.3–1.5 m | 0.2–0.8 m |
| `PROX_SAMPLE_LAT` | ±0.3 m    | ±0.15 m (ego-tube width) |
| `PROX_COST_SLOW`  | 30        | 60 |
| `PROX_COST_WARN`  | 70        | 90 |
| `V_LETHAL`        | 0.03 m/s  | 0.08 m/s (always ≥ 0.08) |

### run 2 - usable proximity + proactive projection

| Metric                      | Value |
|-----------------------------|-------|
| WPs reached                 | **37 / 94** (39 %) |
| WPs TIMEOUT                 | 53 |
| GT distance travelled       | **539 m** |
| Proactive projections (total)| 502 |
| WPs skipped by projection   | 0 (always found free cell within 3 m) |
| Nav2 plans failed           | 489 (late-run, after SLAM drifted) |
| PP proximity hits           | 0 (on the fixed run 2 parametars - no false triggers) |

Comparison with the previous failures:

| Run                    | WPs | Distance | What stopped it |
|------------------------|-----|----------|-----------------|
| Exp 52 run 3 (no fixes)| 22  | 98 m     | WP inside cone group 1 - robot wedged |
| Exp 52 run 4 (+ tent)  | 14  | 87 m     | Drove into tent, SLAM re-localized 50 m away |
| **Exp 53 run 2**       | **37** | **539 m** | SLAM drift past 300 m, then plan failures |

The two new failure modes of exp 52 are both **gone**:

- Group 1 clearance: proactive projection moved WP 25 out of the
  cone wall as soon as the costmap saw it; robot detoured cleanly.
- **Tent**: robot maintained ≥ 0.08 m/s aproach, never contacted it,
  SLAM stayed locked through the whole detour.

The robot drove all the way to GT x ≈ 47 (turnaround is at x = 60),
which is 5–6× further than either exp 52 run.

### what failed in run 2

SLAM err grew with distance even without any single dramatic break:

| Distance | SLAM err |
|----------|----------|
| 0 m      | 0.00 m |
| 55 m     | 0.22 m |
| 104 m    | 0.81 m |
| 156 m    | 3.29 m |
| 240 m    | 7.67 m |
| 313 m    | 16.46 m |
| 396 m    | 31.73 m |

Past ~200 m the feature-sparse forest gives ORB-SLAM3 too little to
track reliably; VIO diverges gradually. `tf_wall_clock_relay` drops
into `ENCODER` mode (95 % encoder / 5 % SLAM when SLAM is lost) and
the robot continues on dead reckoning. But Nav2's costmap frame is
still tied to the stale SLAM pose, so plans go to goals 16–30 m away
from reality and the robot times out repeatedly.

This is not a proximity / projection failure - the two changes
work as intended. The underlying issue is SLAM's inability to
maintain long-horizon accuracy in visually repetitive forest, which
was already documented as the open problem of exp 51 v9.

![run 2 trajectory](results/repeat_run2/trajectory_map.png)

### next step (not in this experiment)

Improve long-horizon localisation. Options for a future experiment:

- Periodic SLAM map save + fresh session start at turnaround (loop
  closure off means no global optimisation, so atlas-level fusion
  is the only way to stop drift compounding).
- Couple the encoder more strongly (say 30 %/70 %) once `slam_f`
  stops advancing.
- Add ICP loop-closure on depth scans against the teach map to
  re-anchor the pose when drift exceeds a treshold.
