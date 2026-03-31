# exp 45: South Forest with Nav2 SmacPlanner2D

## Result

**47/99 REACHED / 27 SKIPPED - peak 93m (vs exp 44's 89m with NavFn).**

SmacPlanner2D accounts for robot footprint when searching the costmap,
so it refuses 1-cell non-lethal gaps through inflation halos that NavFn
would pick but the 0.7m-wide robot physically can't fit through.
Marginally better route progress, but same fundamental stall point ~(−5, −9)
in the NE forest cluster where local costmap halo hits 55-65% inflated
and RegulatedPurePursuit's velocity scaling collapses to near-zero.

## Key changes vs exp 44

| Subsystem | Exp 44 | Exp 45 |
|---|---|---|
| Planner | `nav2_navfn_planner::NavfnPlanner` | `nav2_smac_planner::SmacPlanner2D` |
| Planner params | `tolerance=2.0`, `use_astar=true`, `cost_factor=0.55` | `tolerance=2.0`, `motion_model=MOORE`, `cost_travel_multiplier=2.0` |
| Velocity smoother | missing from launch (bypass) | wired via `cmd_vel_raw->smoother->cmd_vel` |
| `desired_linear_vel` | 0.5 m/s | 0.4 m/s (cmd->~1.4 m/s actual in Isaac Husky 3.4× scale) |
| `max_velocity` | - | [0.5, 0, 0.6] enforced by smoother |
| BT | simple 8×1m BackUp -> 8m drift | exp45 BT: 3 retries × 0.4m BackUp |
| Shrub phantom | 357 invisible colliders | 357 referenced from `veg_shrub_sm_02_inst` |

## components

### Config
- `config/nav2_params.yaml` - full stack (map, global/local costmap, controller, planner=SmacPlanner2D, behavior, BT, smoother)
- `config/nav2_launch.py` - Nav2 launch with velocity_smoother node
- `config/nav2_bt_exp45.xml` - minimal BT: 3 retries × 0.4m BackUp
- `config/south_anchors_dense.json` - 99 teach anchors at 2m spacing
- `config/blank_south_map.yaml/pgm` - 200×60m blank static map

### Scripts
- `scripts/run_exp45.sh [tag]` - single-command launch (Isaac -> TF -> Nav2 -> goals)

## How to reproduce

```bash
cd /workspace/simulation/isaac/experiments/45_south_forest_smac
./scripts/run_exp45.sh v1    # creates logs/exp45_v1_*.log

# Watch progress:
tail -f logs/exp45_v1_goals.log | grep -E "REACHED|SKIPPED"

# After done (or stuck):
cp /root/bags/husky_real/isaac_slam_*/groundtruth.csv logs/exp45_v1_traj.csv
```

Kill everything:
```bash
pkill -9 -f "isaacsim|nav2|trajectory_follower|tf_wall_clock"
```

## baseline run (= exp 44 v18)

Inherited as starting point. Logs in `logs/exp45_baseline_*`, costmap
PNGs in `results/costmap_debug_baseline/`.

## diagnosis of the NE-cluster stall

Pattern same as exp 44:
- Robot reaches WP 50–51 (teach line y=−24 at x=−20..−15) after 93m
- Enters NE region where teach path bends up to y=−5 through dense
  shrub+oak cluster
- Local costmap shows 55–65% inflated (no lethal), plan is valid
- Controller's `use_cost_regulated_linear_velocity_scaling` scales
  velocity proportional to free-space margin -> near-zero in thick halo
- 25s script timeout fires before WP reached -> SKIPPED
- Over ~20 consecutive SKIPs robot drifts/oscillates ~3m in circles

### What SmacPlanner2D helps with
- Plans no longer route through 0.4-0.6m halo gaps
- Path length increases (plans take wider routes when possible)
- Slightly further peak position

### What it doesn't fix
- Once robot is INSIDE continuous inflation halo (no wide corridor
  exists), planner has nothing to route through other than halo itself
- Controller conservatism unchanged - still collapses speed near halos
- Retry budget (25s script + 3×0.4m BT backup) insufficient for the
  amount of re-planning happening

## artifacts

### Logs (baseline = exp 44 v18)
- `logs/exp45_baseline_goals.log` - 47 REACHED, 27 SKIPPED
- `logs/exp45_baseline_traj.csv` - GT trajectory, final at (−1.87, −9.0)

### Costmap PNG snapshots at failed WPs
- `results/costmap_debug_baseline/wp{50..73}_att{1,2}_{global,local}.png`
- Green dot = robot, magenta = goal, white = Nav2 plan, orange = inflated

## Next directions

1. **MPPI controller** in place of RegulatedPurePursuit (trajectory
   sampling, more robust in dense halos)
2. **Pre-built static map with known obstacle positions** (no runtime
   depth accumulation -> no feedback drift)
3. **DWB with PathAlign critic** (sticks to teach path harder)
4. **Different route** - teach path avoiding the NE forest chokepoint
