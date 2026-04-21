#!/usr/bin/env python3
"""Generate README.md for each route folder combining teach + repeat info."""
import json, math, sys, re
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
sys.path.insert(0, '/workspace/simulation/isaac/routes/_common/scripts')
from spawn_obstacles import OBSTACLES, PROP_ASSETS

TURNAROUND = {
    '04_nw_se': (+65, -35),
    '05_ne_sw': (-90, -35),
    '06_nw_ne': (+65, +35),
    '07_se_sw': (-90, -35),
    '08_nw_sw': (-90, -35),
    '09_se_ne': (+65, +35),
}
LABEL = {
    '04_nw_se':  'NW -> SE diagonal',
    '05_ne_sw':  'NE -> SW diagonal',
    '06_nw_ne':  'north edge (NW -> NE)',
    '07_se_sw':  'south edge (SE -> SW)',
    '08_nw_sw':  'west edge (NW -> SW)',
    '09_se_ne':  'east edge (SE -> NE)',
}


def parse_drift_from_plot_log(teach_outputs):
    """Extract drift_mean/drift_max from traj_vio_world.csv vs traj_gt_world.csv by
    simple endpoint comparison - quick approximation."""
    # Read pre-computed VIO vs GT (already aligned by plot_teach_vio_gt.py)
    import csv
    gt_csv = teach_outputs / 'traj_gt_world.csv'
    vio_csv = teach_outputs / 'traj_vio_world.csv'
    if not (gt_csv.exists() and vio_csv.exists()):
        return None
    gt = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(gt_csv))]
    vio = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(vio_csv))]
    n = min(len(gt), len(vio))
    errs = [math.hypot(gt[i][0] - vio[i][0], gt[i][1] - vio[i][1]) for i in range(n)]
    if not errs: return None
    return {'mean': sum(errs)/len(errs), 'max': max(errs), 'n': n}


def parse_run_info(teach_outputs):
    f = teach_outputs / 'run_info.txt'
    if not f.exists(): return None
    return f.read_text().strip()


def obstacles_description(route):
    conf = OBSTACLES.get(route, {})
    lines = []
    n_cones = sum(len(g) for g in conf.get('cones', []))
    if n_cones:
        lines.append(f'- **{n_cones} traffic cones** in {len(conf["cones"])} groups')
        for i, g in enumerate(conf['cones'], 1):
            lines.append(f'  - group {i}: {len(g)} cone(s), first at ({g[0][0]:.1f}, {g[0][1]:.1f})')
    if conf.get('tent'):
        tx, ty = conf['tent']
        lines.append(f'- **1 tent** at ({tx:.1f}, {ty:.1f}) - 2×1.8 m body')
    props = conf.get('props', [])
    if props:
        from collections import Counter
        kinds = Counter(p['kind'] for p in props)
        lines.append(f'- **{len(props)} quality props** (Isaac asset library):')
        for kind, n in kinds.most_common():
            positions = [(p['x'], p['y']) for p in props if p['kind'] == kind]
            pos_str = ', '.join(f'({x:.1f}, {y:.1f})' for x, y in positions)
            lines.append(f'  - {n}× **{kind}** @ {pos_str}')
    return '\n'.join(lines)


def route_readme(route):
    # XXX: magic, tuned by trial and error over exps 55-58
    teach_outputs = Path(f'/root/isaac_tr_datasets/{route}/teach/teach_outputs')
    drift = parse_drift_from_plot_log(teach_outputs)
    run_info = parse_run_info(teach_outputs) or 'unknown'
    pts = json.load(open('/workspace/simulation/isaac/routes/_common/routes.json'))[route]
    sx, sy = pts[0]
    tx, ty = TURNAROUND[route]
    import csv as _c
    path_m = 0.0
    if (teach_outputs / 'traj_gt_world.csv').exists():
        gt = [(float(r['x']), float(r['y'])) for r in _c.DictReader(open(teach_outputs / 'traj_gt_world.csv'))]
        for i in range(1, len(gt)):
            path_m += math.hypot(gt[i][0]-gt[i-1][0], gt[i][1]-gt[i-1][1])

    obs_desc = obstacles_description(route)

    md = f"""# {route} - {LABEL[route]}

## Route layout
- **Spawn**: ({sx}, {sy})
- **Turnaround**: ({tx}, {ty})
- **Planned length**: {len(pts)} waypoints, ~{path_m:.0f} m total roundtrip
- **Label**: {LABEL[route]}

## Teach run (already completed)

| Metric | Value |
|---|---|
| {run_info.replace('REC=', 'Bag: `') + '`'} |
| Waypoints driven | {drift['n'] if drift else 'n/a'} |
| VIO drift mean | **{drift['mean']:.3f} m** |
| VIO drift max | **{drift['max']:.3f} m** |

VIO vs GT plot: [`vio_vs_gt.png`](../../../{teach_outputs.relative_to('/root')}/vio_vs_gt.png)

Teach artifacts (in dataset folder):
- `landmarks.pkl` - ORB visual landmarks for repeat localisation
- `teach_map.pgm` + `teach_map.yaml` - depth-derived occupancy (built by
  `teach_run_depth_mapper.py` from live `/depth_points` during teach)
- `vio_pose_dense.csv` - dense VIO + GT samples per frame
- `traj_gt.csv` - Isaac ground-truth trajectory for verification

## Repeat run

**Pipeline** (from `scripts/run_repeat.sh`, adapted from exp 72 pattern):
1. Isaac Sim with `--obstacles --route {route}` spawns cones/tent/props
   listed in `spawn_obstacles.OBSTACLES["{route}"]` on the outbound path.
2. ORB-SLAM3 RGB-D-Inertial VIO starts from bag live-recording.
3. `tf_wall_clock_relay_v55.py --slam-encoder` publishes map->base_link TF
   derived from VIO + encoder fallback; align committed at spawn from 50
   GT samples.
4. `visual_landmark_matcher.py` matches live ORB frame vs `landmarks.pkl`
   -> `/anchor_correction` for drift correction.
5. **Nav2 planner-only** (no controller): `planner_server` + `map_server`
   with plugins `["static_layer", "obstacle_layer", 'inflation_layer']`.
   `obstacle_layer` subscribes to `/depth_points` - detects cones/props
   live and updates costmap.
6. `pure_pursuit_path_follower.py` consumes `/plan` -> `/cmd_vel`.
7. `send_goals_hybrid.py` reads `traj_gt.csv` sub-sampled at 4 m, fires
   ComputePathToPose for each WP; on costmap-cost high or plan fail, does
   a 4–7 m detour ring around the WP.
8. `turnaround_supervisor.py --final-x {tx} --final-y {ty} --near-radius 10.0`
   - when robot reaches <10 m from turnaround, writes
   `/tmp/isaac_remove_obstacles.txt` -> Isaac drops all obstacles. **Return
   leg is obstacle-free.**

### Obstacle placement ({route})

{obs_desc}

Placement strategy: obstacles between ~20% and ~80% of the outbound leg,
minimum ~15 m from spawn (so VIO can warmup), minimum ~10 m from turnaround
(so supervisor fires before robot reaches them on the return).

Overview with obstacles: [`plan_obstacles.png`](results/repeat_run/plan_obstacles.png)

### How to run
```bash
# PHYSICS_DT = 1/240  # tried, too slow for real-time sim
ROS_DOMAIN_ID=85 bash scripts/run_repeat.sh
# Or via orchestrator:  bash routes/run_all_repeat.sh {route}
```

### Expected outputs (in `results/repeat_run/`)
- `goals.log` - per-WP REACHED / SKIP / DETOUR events + final RESULT line
- `anchor_matches.csv` - landmark-matcher -> tf_relay anchor correction log
- `nav2.log` + `pp_follower.log` - Nav2 planner + pure-pursuit diagnostics
- `supervisor.log` - turnaround trigger FIRE record
- `trajectory.png` - GT + VIO-nav overlay on scene map (post-run plot)
- `tf_slam.log`, `vio.log` - SLAM + VIO diagnostics
"""
    out = Path(f'/workspace/simulation/isaac/routes/{route}/README.md')
    out.write_text(md)
    print(f'{route} -> {out}')


if __name__ == '__main__':
    only = sys.argv[1:] or list(TURNAROUND.keys())
    for r in only:
        route_readme(r)
