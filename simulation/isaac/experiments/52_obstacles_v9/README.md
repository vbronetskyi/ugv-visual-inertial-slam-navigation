# Exp 52 v9 - honest teach-and-repeat with depth-sensed obstacles

## goal

Run the v9 hybrid pipeline (Nav2 planner_server + custom pure-pursuit
follower + ORB-SLAM3 VIO + synthetic IMU) as a real-world teach-and-repeat:

- **Teach**: robot drives the south roundtrip route **without** obstacles.
  The only map it builds is a 2-D occupancy grid from the depth camera
  (via Bresenham ray-casting on /depth_points in the `map` frame).
- **Repeat**: robot drives the same route **with** cones placed in three
  groups. Nav2's `global_costmap.obstacle_layer` reads live /depth_points
  and inflates occupied cells into the static teach map, so the planner
  re-routes around whatever the depth camera sees.

Ground-truth positions are **not** fed into Nav2. Only IMU and RGB-D
are the allowed sensor proxies for a real Husky.

## Pipeline

```
Isaac Sim (RGB-D + synthetic IMU)
     │
     ├── /depth_points ──────────────┐
     │                               │
     ▼                               ▼
ORB-SLAM3 (RGB-D+IMU) ──► tf_wall_clock_relay (SLAM+ENCODER, 95/5 blend)
     │                               │
     │                               ▼
     │                          TF: map -> base_link
     │                               │
     └── /depth_points ──► Nav2 global_costmap (static_layer=teach map,
                                obstacle_layer=live /depth_points,
                                inflation_layer=1.2 m)
                                        │
                                        ▼
                             planner_server (NavFn, A*)
                                        │
                                /plan   ▼
                             pure_pursuit_follower  ->  /cmd_vel  -> Husky
```

## Key files

| Path                                        | Purpose |
|---------------------------------------------|---------|
| `scripts/run_exp52_teach.sh`                | Teach phase launcher (no obstacles, no Nav2; depth mapper running) |
| `scripts/run_exp52_repeat.sh`               | Repeat phase launcher (cones + Nav2 + supervisor + recorders) |
| `scripts/teach_run_depth_mapper.py`         | Builds south_teach_map.pgm/yaml from /depth_points + TF (log-odds, Bresenham) |
| `scripts/costmap_snapshotter.py`            | Saves full `/global_costmap/costmap` as .npy every 5 s + `snapshots_summary.csv` |
| `scripts/plan_logger.py`                    | Saves every `/plan` as CSV + `plans_summary.csv` |
| `scripts/turnaround_supervisor.py`          | Writes `/tmp/isaac_remove_obstacles.txt` when GT x passes 60 and drops back 2 m (triggers physical cone despawn) |
| `config/nav2_planner_only.yaml`             | Nav2 config: NavFn planner + static teach map + depth obstacle_layer + inflation 1.2 m |
| `config/nav2_launch_hybrid.py`              | Launches map_server + planner_server + lifecycle_manager (no controller, no BT) |
| `teach/south_teach_map.pgm / .yaml`         | Depth-sensed static map (2000 × 600 @ 0.1 m) |

## Teach phase - result

- 103 k depth frames integrated, 17.7 M points ray-cast (subsample 1/4).
- 26 k cells above `OCC_L_TH` log-odds -> line-of-sight obstacle picture
  of the forest + road corridor.
- Map written to `teach/south_teach_map.{pgm,yaml}`; honesty-check plot at
  `teach/south_teach_map_honesty_check.png`.
- The map is a depth-sensed snapshot from one drive, **not** god-view
  from the simulator.

## Repeat phase - runs

### Run 4 - reduced cones + on-route tent (latest)

Obstacle layout changed per user direction (see `scripts/patch_obstacles_exp52.py`):

| Group | Position | Count | Change from run 3 |
|-------|----------|-------|-------------------|
| 0 | x=−75, y=−24..−26 | 3 cones | unchanged |
| 1 | x=−18, y=−24..−25 | **2** cones | was 4 |
| 2 | x=+5,  y=−17..−20 | **4** cones | was 5 |
| Tent | (−45, −38) - on route | 1 × 2×1.8 m | moved from (−70, −48) off-route |

| Metric                      | Value |
|-----------------------------|-------|
| WPs reached                 | **14 / 94** (15 %) |
| GT distance travelled       | **86.9 m** |
| SLAM mean err (before break)| 0.89 m |
| ORB-SLAM3 frame losses      | 24 (all after break) |
| Plan failures               | 7 |
| Final outcome               | **ORB-SLAM3 lost track near tent, relocalized to wrong keyframe near spawn -> Nav2 position estimate jumped 50 m -> plans infeasible** |

![run 4 trajectory](results/repeat_run4/trajectory_map.png)

#### What worked

- Cone group 0 (3 cones at x=−75) cleared cleanly, same as run 3.
- Planner correctly routed around the on-route tent at (−45, −38): plan
  lengths ballooned from 6 m direct to 27–34 m detour (273–343 poses) as
  the tent entered the depth obstacle_layer.

#### What failed - SLAM relocalization during tent detour

At t+525 s (GT (−48, −37), dist travelled 57 m), after ~50 s of maneuvering
around the tent, ORB-SLAM3 lost tracking (4 frames lost) and re-localized
against an earlier keyframe - reporting `nav=(−92, −6)` while the robot
was physically at GT (−45, −36). That's a 50 m jump to near the spawn
position (spawn was (−88, −15)).

Consequence:
- Nav2's pose went from tight (err ≤ 2.2 m over the first 57 m of
  healthy tracking) to 54+ m off.
- Robot kept physically moving (encoder-derived enc_err grew linearly
  from 0.3 m to 2.7 m over the next 4 minutes) but Nav2 saw a static
  "stuck near spawn" robot.
- Plans started failing / going to 1700+ poses. `hybrid_goal_sender`
  kept re-requesting paths to WPs that, in the bad map frame, were
  50 m east of the robot.

#### Why this broke SLAM

With `loopClosing: 0` in the VIO config, ORB-SLAM3's loop closure
thread is disabled - but relocalization still runs when tracking
is lost. The tent detour forced the robot into a sequence of tight
turns next to a large geometric obstacle that occluded the feature-rich
road. Tracking dropped -> relocalization matched against early trajectory
keyframes (near spawn) because the surrounding forest looks similar
throughout the west side of the scene -> pose jumped.

Unlike run 3, where the failure was geometric (WP inside obstacle),
run 4's failure is a SLAM-level feedback collapse triggered by
visual aliasing in the forest during a recovery maneuver.

---

### Run 3 - initial attempt, full cone counts

| Metric                      | Value |
|-----------------------------|-------|
| Route                       | South roundtrip, 94 WPs (outbound + return) |
| WPs reached                 | **22 / 94** (23 %) |
| GT distance travelled       | **97.5 m** |
| SLAM mean pos err           | 0.33 m |
| SLAM max pos err            | 0.65 m |
| ORB-SLAM3 frame losses      | 0 |
| Nav2 plans logged           | 240 |
| Costmap snapshots           | 206 |
| Final outcome               | **Stuck at cone group 1** (WP 26) |

Trajectory (GT + SLAM estimate) overlaid on the scene + cone obstacles:

![trajectory](results/repeat_run/trajectory_map.png)

### What went well

- **SLAM+ENCODER align v10** committed cleanly: 50 samples, GT disp
  0.0 cm, yaw std 0.000°. VIO stayed lock throughout (0 lost frames
  in 98 m of driving).
- **Cone group 0** (3 cones at x = −75) was cleared cleanly: planner
  generated a north detour, PP executed, robot passed without contact.
- Mean SLAM err 0.33 m over 98 m of dense forest is well below the
  tolerances Nav2 needs.
- Depth-sensed obstacle_layer works: plan lengths for WP 26 ballooned
  from 7 m straight-line to 95–1050 poses (9.5–105 m detour) as the
  cones at x = −18 entered the costmap.

### what failed - cone group 1

The cones in group 1 sit at `(−18, −23), (−18, −24), (−18, −25), (−18, −26)`.
The pre-recorded route passes **through** this wall - WP 25 is at
`(−18.1, −24.4)`, right between cones 2 and 3.

With `tolerance: 1.0 m` in the planner config, Nav2 marks WP 25 as
"reached" when the closest feasible point is within 1.0 m of the goal.
That feasible point is inside the inflation-layer shoulder, roughly
1.0 m north of the cone line - and WP 25 REACHED fires at robot GT
`(−18.7, −23.0)`, about **0.7 m from the north cone** (cone radius
~0.3 m + robot body 0.5 m half-width -> the robot hull physically
touched the cone).

From that position, PP tries to drive east toward WP 26 at
`(−14.3, −22.9)`. Plans are computed (90+ poses, ~9 m detour) but the
robot can't move: it's wedged against the cone. PP keeps emitting
`v = 0.20 m/s, w = 0.2–0.4 rad/s` for ~190 s. The anti-spin guard
does not fire (robot isn't pure-rotating - it's just physically
blocked), and this hybrid stack has no BackUp/ClearCostmap recovery
behaviours.

### root cause, diagnosed

The **pre-recorded route waypoints pass through the cone positions**.
WP 25 at `(−18.1, −24.4)` is geometrically inside the cone wall. With
1.0 m planner tolerance, the robot can approach to within 1.0 m of the
WP - which for this WP means 1.0 m of the cone centre, i.e. direct
contact given robot + cone physical radii.

A planner that respects `robot_radius = 0.5 m + inflation = 1.2 m`
should never let a plan terminate closer than 1.7 m from an occupied
cell. But `tolerance: 1.0 m` says: if the goal itself is inside
inflated space, accept the closest reachable point and return that
path. PP then follows it into the contact zone.

### Fix (for the next run, not applied yet)

Either (a) pre-process route waypoints to project them away from any
occupied cell in the teach map by at least `robot_radius + inflation`,
or (b) reduce the planner's `tolerance` to 0.3 m and let
`hybrid_goal_sender.py` auto-skip to the next WP when the current one
is infeasible. Option (a) matches real teach-and-repeat practice -
teach map is known, so route WPs can be sanitised once before the
repeat run.

### Parallel failure - first attempt (run 2)

Run 2 at 12:10 was a seperate failure: ORB-SLAM3 went off the rails
right after init (IMU init saw "Empty IMU measurements vector"), nav
pose rocketed to (155, −240) while GT barely moved. This was resolved
by a simple relaunch - ORB-SLAM3 IMU init is stochastic and the third
attempt (run 3) initialised cleanly (yaw std 0.000°). Archived in
`results/repeat_run2_failed_encoder/`.

## recorded data layout

For reproducibility, every repeat run saves numeric data only - no
images generated from flight-time events:

```
results/repeat_run/
  tf_slam.log                          # full tf_relay output (err, nav, gt, yaw_err, dist, slam_f, lost)
  trajectory.csv                       # derived from tf_slam.log (ts, nav_xy, gt_xy, err, enc_err, yaw_err, dist, slam_f, lost)
  goals.log                            # WP send / REACHED / TIMEOUT events
  pp_follower.log                      # PP cmds (v, w, err, spin_t) + anti-spin events
  supervisor.log                       # turnaround fire events
  nav2.log                             # Nav2 node stdout/stderr
  isaac.log, vio.log                   # sim + SLAM backend logs
  snapshots/
    costmap_XXXX.npy                   # int8 array (W×H) 0=free, 100=occ, -1=unknown
    costmap_XXXX.json                  # {ts, origin_xy, res, robot_xy, n_occ, n_unknown, n_free}
    snapshots_summary.csv              # one row per snapshot
  plans/
    plan_XXXX.csv                      # (seq, wall_ts, x, y) per Nav2 replan
    plans_summary.csv                  # (plan_id, ts, n_poses, length_m, robot_xy, goal_xy)
  trajectory_map.png                   # canonical plot_trajectory_map() output
```

## reproducing

```bash
# teach (once)
bash scripts/run_exp52_teach.sh

# Repeat
bash scripts/run_exp52_repeat.sh

# analyse
python3 scripts/analyze_repeat.py
```
