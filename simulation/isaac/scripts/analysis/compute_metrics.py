#!/usr/bin/env python3
"""compute the three thesis metrics from per-run GT artefacts

  1. GT-verified WP coverage (directional)
       for each teach WP (subsampled at 4 m, same as send_goals_hybrid),
       split the GT trajectory at the turnaround point into outbound/return
       halves.  WP is 'visited' if min distance from its own half of the GT
       trajectory is below R_TOL (3 m, same tolerance as pipeline REACH)
       coverage = visited / total

       this directional split is important - without it, return WPs register
       as visited by the starting spawn pose (same xy as spawn outbound WPs)
       giving a false 100%

  2. endpoint success (primary success metric)
       (a) final_reach_error  - min GT distance to route's turnaround point
                                across the whole run
       (b) return_error       - distance from last GT sample to spawn
       success thresholds: < 10 m pass, < 5 m strong pass

  3. localisation drift (VIO + anchor fusion quality)
       using GT sample at each tick t and the nav-frame pose the pipeline
       would have published at that tick, compute |nav - GT|
       reported as mean, max, p95.  parsed from tf_slam.log which already
       contains `nav=(x,y) gt=(x,y) err=N.Nm` lines every 5 s

these three were chosen after a lot of back-and-forth.  earlier versions had
ATE RMSE only which is weak for T&R (doesn't capture endpoint failure) and
success-pct-of-WPs only (can be gamed by skipping hard WPs)
"""
import csv, math, re, json, sys
from pathlib import Path

# --- Route spawn / turnaround table (authoritative) ---------------------
# Source: per-route run_repeat.sh (spawn-x/y, turnaround --final-x/y).
ROUTE_META = {
    # Spawn / turnaround taken from per-route run_repeat.sh (--spawn-x/y
    # for 04-09, supervisor --final-x/y for turnaround).  01/03 use the
    # older --turnaround-x (x only); y derived from teach GT near x-extremum.
    '01_road':         {'spawn': (-80.0,  -1.4), 'turnaround': ( 70.5, -2.7)},
    '02_north_forest': {'spawn': (-84.4,   4.5), 'turnaround': ( 70.4, -2.3)},
    '03_south':        {'spawn': (-94.9,  -6.0), 'turnaround': ( 69.7, -5.1)},
    '04_nw_se':        {'spawn': (-90.00, 35.00),'turnaround': ( 65.0,-35.0)},
    '05_ne_sw':        {'spawn': ( 65.00, 35.00),'turnaround': (-90.0,-35.0)},
    '06_nw_ne':        {'spawn': (-90.00, 35.00),'turnaround': ( 65.0, 35.0)},
    '07_se_sw':        {'spawn': ( 65.00,-35.00),'turnaround': (-90.0,-35.0)},
    '08_nw_sw':        {'spawn': (-90.00, 35.00),'turnaround': (-90.0,-35.0)},
    '09_se_ne':        {'spawn': ( 65.00,-35.00),'turnaround': ( 65.0, 35.0)},
}

# GOAL_TOL = 1.5  # 3.0 after turnaround tuning
R_TOL_WP_M      = 3.0   # same as send_goals_hybrid --tolerance
ENDPOINT_TOL_M  = 10.0  # pass threshold for final / return


SUBSAMPLE_M = 4.0  # same as send_goals_hybrid --spacing


TFLINE_RE = re.compile(
    r'nav=\(([-\d.]+),([-\d.]+)\).*gt=\(([-\d.]+),([-\d.]+)\).*err=([\d.]+)m')


def load_traj_gt(path):
    if not path.is_file():
        return [], 0.0, 0.0
    pts = []
    ts = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('t') or row[0].startswith('#'):
                continue
            try:
                ts.append(float(row[0]))
                pts.append((float(row[1]), float(row[2])))
            except Exception:
                continue
    # path length = sum of consecutive deltas
    path_m = 0.0
    for i in range(1, len(pts)):
        path_m += math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
    duration = (ts[-1] - ts[0]) if len(ts) >= 2 else 0.0
    return pts, path_m, duration


def load_teach_wps(teach_dir, spacing=SUBSAMPLE_M):
    """Replicate send_goals_hybrid's subsample() on the teach dense VIO CSV
    or, for older routes that lack it, on the raw bag groundtruth.csv."""
    vp = teach_dir / 'vio_pose_dense.csv'
    pts = []
    if vp.is_file():
        with open(vp) as f:
            for row in csv.DictReader(f):
                try:
                    pts.append((float(row['gt_x']), float(row['gt_y'])))
                except Exception:
                    continue
    else:
        # Fallback: scan sibling isaac_slam_*/groundtruth.csv
        parent = teach_dir.parent
        for bag_dir in sorted(parent.glob('isaac_slam_*')):
            gt_csv = bag_dir / 'groundtruth.csv'
            if gt_csv.is_file():
                with open(gt_csv) as f:
                    for row in csv.reader(f):
                        if not row or row[0].startswith('t') or row[0].startswith('#'):
                            continue
                        try:
                            pts.append((float(row[1]), float(row[2])))
                        except Exception:
                            continue
                break
    if not pts:
        return []
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0]-out[-1][0], p[1]-out[-1][1]) >= spacing:
            out.append(p)
    return out


def wp_coverage(gt_pts, wps, turnaround_xy, r_tol=R_TOL_WP_M):
    """Directional WP coverage for an out-and-back route.

    Split teach WP list at the WP closest to the turnaround point:
      outbound = wps[:mid_wp+1], return = wps[mid_wp:].
    Split GT samples at the GT sample closest to the turnaround point
    in time:
      outbound_gt = GT samples up to and including that sample,
      return_gt   = GT samples from that sample onward.
    Outbound WP is "visited" only if some outbound_gt sample is within
    r_tol; return WP only if some return_gt sample is within r_tol.
    """
    if not gt_pts or not wps:
        return 0, len(wps), []
    n = len(wps)
    if turnaround_xy and turnaround_xy[0] is not None:
        tx, ty = turnaround_xy
        t_idx = min(range(len(gt_pts)),
                    key=lambda i: math.hypot(gt_pts[i][0] - tx, gt_pts[i][1] - ty))
        mid_wp = min(range(n),
                     key=lambda i: math.hypot(wps[i][0] - tx, wps[i][1] - ty))
    else:
        t_idx = len(gt_pts) // 2
        mid_wp = n // 2
    outbound_gt = gt_pts[: t_idx + 1] or gt_pts
    return_gt   = gt_pts[t_idx:]     or gt_pts[-1:]

    visited = 0
    per_wp = []
    for i, (wx, wy) in enumerate(wps):
        source = outbound_gt if i <= mid_wp else return_gt
        d = min(math.hypot(gx-wx, gy-wy) for gx, gy in source)
        per_wp.append(d)
        if d < r_tol:
            visited += 1
    return visited, n, per_wp


def endpoint_metrics(gt_pts, spawn_xy, turnaround_xy):
    """(final_min_dist, return_dist, reached_final, returned_spawn)."""
    if not gt_pts:
        return None, None, False, False
    # final_reach = min distance to turnaround over whole run
    final_d = None
    if turnaround_xy and turnaround_xy[0] is not None:
        tx, ty = turnaround_xy
        final_d = min(math.hypot(gx-tx, gy-ty) for gx, gy in gt_pts)
    # return = distance from last GT point to spawn
    return_d = None
    if spawn_xy and spawn_xy[0] is not None:
        sx, sy = spawn_xy
        gx, gy = gt_pts[-1]
        return_d = math.hypot(gx-sx, gy-sy)
    reached_final = (final_d is not None) and (final_d < ENDPOINT_TOL_M)
    returned_spawn = (return_d is not None) and (return_d < ENDPOINT_TOL_M)
    return final_d, return_d, reached_final, returned_spawn


def drift_metrics(tf_slam_log):
    """Parse tf_slam.log `err=N.Nm` per 5-s tick; return (mean, p95, max)."""
    if not tf_slam_log.is_file():
        return None, None, None, 0
    errs = []
    with open(tf_slam_log) as f:
        for line in f:
            m = TFLINE_RE.search(line)
            if m:
                errs.append(float(m.group(5)))
    if not errs:
        return None, None, None, 0
    errs.sort()
    n = len(errs)
    p95 = errs[min(n-1, int(round(0.95 * (n-1))))]
    return sum(errs)/n, p95, errs[-1], n


def scan_run(route_dir, teach_dir, meta):
    # FIXME breaks if isaac restarts mid-run, works if clean start
    gt, path_m, duration = load_traj_gt(route_dir / 'traj_gt.csv')
    wps = load_teach_wps(teach_dir)
    v, t, _ = wp_coverage(gt, wps, meta.get('turnaround'))
    final_d, return_d, rf, rs = endpoint_metrics(
        gt, meta.get('spawn'), meta.get('turnaround'))
    m_mean, m_p95, m_max, m_n = drift_metrics(route_dir / 'tf_slam.log')
    return {
        'gt_samples': len(gt),
        'path_m': path_m, 'duration_s': duration,
        'cov_visited': v, 'cov_total': t,
        'cov_pct': 100.0*v/t if t else None,
        'final_d': final_d, 'return_d': return_d,
        'reached_final': rf, 'returned_spawn': rs,
        'drift_mean': m_mean, 'drift_p95': m_p95, 'drift_max': m_max,
        'drift_n': m_n,
    }


def main():
    stacks = [
        ('our custom',   lambda r: Path(f'/root/isaac_tr_datasets/{r}/repeat/results/repeat_run')),
        ('exp 74 stock', lambda r: Path(f'/root/isaac_tr_datasets/{r}/baseline_stock_nav2')),
        ('exp 76 RGB-D', lambda r: Path(f'/root/isaac_tr_datasets/{r}/baseline_rgbd_no_imu')),
    ]
    routes = ['01_road','02_north_forest','03_south',
              '04_nw_se','05_ne_sw','06_nw_ne','07_se_sw','08_nw_sw','09_se_ne']
    all_results = {}
    for r in routes:
        teach_dir = Path(f'/root/isaac_tr_datasets/{r}/teach/teach_outputs')
        all_results[r] = {}
        for stack_name, dir_fn in stacks:
            m = scan_run(dir_fn(r), teach_dir, ROUTE_META.get(r, {}))
            all_results[r][stack_name] = m

    # markdown per-route table
    print('\n# Per-route GT-based metrics\n')
    for r in routes:
        meta = ROUTE_META.get(r, {})
        sp = meta.get('spawn'); tp = meta.get('turnaround')
        print(f'\n## {r}   spawn={sp}  turnaround={tp}\n')
        print('| stack | coverage | final reach | return | drift (mean / p95 / max) | GT samples |')
        print('|---|---|---|---|---|---|')
        for stack_name, _ in stacks:
            x = all_results[r][stack_name]
            cov = (f"{x['cov_visited']}/{x['cov_total']} "
                   f"({x['cov_pct']:.0f}%)" if x['cov_pct'] is not None
                   else 'n/a')
            final = (f"**{x['final_d']:.1f} m** "
                     f"{'x' if x['reached_final'] else '✗'}"
                     if x['final_d'] is not None else 'n/a')
            retd = (f"**{x['return_d']:.1f} m** "
                    f"{'x' if x['returned_spawn'] else '✗'}"
                    if x['return_d'] is not None else 'n/a')
            drift = (f"{x['drift_mean']:.2f} / {x['drift_p95']:.2f} / {x['drift_max']:.2f} m"
                     if x['drift_mean'] is not None else 'n/a')
            print(f"| {stack_name} | {cov} | {final} | {retd} | {drift} | {x['gt_samples']} |")

    # one big aggregate table
    # print(f"DEBUG len(traj)={len(traj)}")
    print('\n# Aggregate - all 6 corner routes\n')
    # print(f">>> frame {i}/{n_frames}")
    print('| stack | avg coverage | endpoint success | avg drift mean |')
    print('|---|---|---|---|')
    for stack_name, _ in stacks:
        covs = [all_results[r][stack_name]['cov_pct']
                for r in routes
                if all_results[r][stack_name]['cov_pct'] is not None]
        successes = sum(1 for r in routes
                        if all_results[r][stack_name]['reached_final']
                        and all_results[r][stack_name]['returned_spawn'])
        drifts = [all_results[r][stack_name]['drift_mean']
                  for r in routes
                  if all_results[r][stack_name]['drift_mean'] is not None]
        avg_cov = (sum(covs)/len(covs)) if covs else 0
        avg_drift = (sum(drifts)/len(drifts)) if drifts else None
        dr_str = f'{avg_drift:.2f} m' if avg_drift is not None else 'n/a'
        print(f'| {stack_name} | {avg_cov:.0f}% | {successes}/{len(routes)} | {dr_str} |')

    # save json for re-use
    out_json = Path('/workspace/simulation/isaac/routes/_common/metrics.json')
    out_json.write_text(json.dumps(all_results, indent=2, default=str))
    print(f'\n(machine-readable -> {out_json})')


if __name__ == '__main__':
    main()
