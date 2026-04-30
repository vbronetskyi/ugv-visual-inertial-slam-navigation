#!/usr/bin/env python3
"""one summary heatmap aggregating all 15 routes by group

6 groups (rows) x 4 algorithms (cols), three side-by-side panels for
WP coverage / reach / return.  each cell is the mean of that metric
over the routes in the group so the figure shows the group-level story
at a glance.  teach is the fourth column - no-obstacles GT reference,
+-0 m reach and +-100 % coverage, useful as the best-case column the
others compare against
"""
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap, Normalize

sys.path.insert(0, '/workspace/simulation/isaac/routes/_common/scripts')
from compute_metrics import ROUTE_META, R_TOL_WP_M, ENDPOINT_TOL_M, SUBSAMPLE_M

METRICS_JSON = Path('/workspace/simulation/isaac/routes/_common/metrics.json')
DSET = Path('/root/isaac_tr_datasets')
OUT = Path('/workspace/simulation/isaac/routes/route_groups/heatmap_aggregate.png')

GROUPS = [
    ('G1 forest',        ['02_north_forest', '03_south', '04_nw_se', '05_ne_sw',
                          '06_nw_ne', '07_se_sw', '08_nw_sw', '11_nw_mid',
                          '13_cross_nws', '15_wmid_smid']),
    ('G2 open',          ['01_road', '09_se_ne', '10_nmid_smid', '12_ne_mid',
                          '14_se_mid']),
    ('G3 short',         ['08_nw_sw', '09_se_ne', '10_nmid_smid', '11_nw_mid',
                          '12_ne_mid', '13_cross_nws', '14_se_mid', '15_wmid_smid']),
    ('G4 long',          ['01_road', '02_north_forest', '03_south', '04_nw_se',
                          '05_ne_sw', '06_nw_ne', '07_se_sw']),
    ('G5 cones+tent',    ['01_road', '02_north_forest', '03_south', '04_nw_se']),
    ('G6 mixed props',   ['05_ne_sw', '06_nw_ne', '07_se_sw', '08_nw_sw', '09_se_ne',
                          '10_nmid_smid', '11_nw_mid', '12_ne_mid', '13_cross_nws',
                          '14_se_mid', '15_wmid_smid']),
]

ALGOS = [
    ('exp 74 stock',  'stock Nav2\n(no matcher,\nno detour ring)'),
    ('exp 76 RGB-D',  'our stack,\nno IMU\n(RGB-D only VIO)'),
    ('our custom',    'our full stack\n(matcher + IMU\n+ detour + finisher)'),
]

# ---- gradient colourmaps (same as per-group heatmaps) -----------------
DIST_VMAX = 30.0
DIST_CMAP = LinearSegmentedColormap.from_list('dist', [
    (0.0,            '#15803d'),
    (5.0/DIST_VMAX,  '#84cc16'),
    (10.0/DIST_VMAX, '#fde047'),
    (20.0/DIST_VMAX, '#f97316'),
    (1.0,            '#b91c1c'),
])
DIST_NORM = Normalize(vmin=0, vmax=DIST_VMAX, clip=True)

COV_VMAX = 100.0
COV_CMAP = LinearSegmentedColormap.from_list('cov', [
    (0.0,            '#b91c1c'),
    (25.0/COV_VMAX,  '#f97316'),
    (50.0/COV_VMAX,  '#fde047'),
    (75.0/COV_VMAX,  '#84cc16'),
    (1.0,            '#15803d'),
])
COV_NORM = Normalize(vmin=0, vmax=COV_VMAX, clip=True)


# ---- compute teach metrics from raw teach data ------------------------
def load_xy(path):
    pts = []
    if not path.is_file(): return pts
    with open(path) as f:
        for r in csv.DictReader(f):
            try: pts.append((float(r['x']), float(r['y'])))
            except: continue
    return pts


def subsample(pts, spacing=SUBSAMPLE_M):
    if not pts: return []
    out = [pts[0]]; last = pts[0]
    for p in pts[1:]:
        if math.hypot(p[0]-last[0], p[1]-last[1]) >= spacing:
            out.append(p); last = p
    return out


def teach_metrics_for(rid):
    teach_dir = DSET / rid / 'teach'
    candidates = [
        teach_dir / 'teach_outputs' / 'traj_gt_world.csv',
        teach_dir / 'teach_outputs' / 'traj_gt.csv',
    ]
    candidates += list(teach_dir.glob('isaac_slam_*/groundtruth.csv'))
    pts = []
    for c in candidates:
        pts = load_xy(Path(c))
        if pts: break
    if not pts: return None
    spawn = ROUTE_META[rid]['spawn']
    turn = ROUTE_META[rid]['turnaround']
    dists_to_turn = [math.hypot(p[0]-turn[0], p[1]-turn[1]) for p in pts]
    turn_idx = min(range(len(pts)), key=lambda i: dists_to_turn[i])
    final_d = dists_to_turn[turn_idx]
    return_d = math.hypot(pts[-1][0]-spawn[0], pts[-1][1]-spawn[1])
    wps = subsample(pts)
    n_total = len(wps)
    out_gt = pts[:turn_idx+1]
    ret_gt = pts[turn_idx:]
    half = len(wps) // 2
    out_wps = wps[:half]; ret_wps = wps[half:]
    visited = 0
    for w in out_wps:
        if out_gt and min(math.hypot(p[0]-w[0], p[1]-w[1]) for p in out_gt) <= R_TOL_WP_M:
            visited += 1
    for w in ret_wps:
        if ret_gt and min(math.hypot(p[0]-w[0], p[1]-w[1]) for p in ret_gt) <= R_TOL_WP_M:
            visited += 1
    cov = 100.0 * visited / max(n_total, 1)
    return {'cov_pct': cov, 'final_d': final_d, 'return_d': return_d}


def collect_group_means(metrics, group_routes, algo_key, field):
    """mean of `field` for `algo_key` across `group_routes`. None if no data."""
    vals = []
    for rid in group_routes:
        if algo_key == 'teach':
            tm = teach_metrics_for(rid)
            if tm and tm.get(field) is not None:
                vals.append(tm[field])
        else:
            v = metrics.get(rid, {}).get(algo_key, {}).get(field)
            if v is not None:
                vals.append(float(v))
    if not vals: return None
    return sum(vals) / len(vals)


# ---- panels -----------------------------------------------------------
PANELS = [
    ('WP coverage',     'cov_pct',  '%',   'cov',   '{:.0f}'),
    ('reach distance',  'final_d',  'm',   'dist',  '{:.1f}'),
    ('return distance', 'return_d', 'm',   'dist',  '{:.1f}'),
]


def main():
    metrics = json.loads(METRICS_JSON.read_text())
    n_rows = len(GROUPS)
    n_cols = len(ALGOS)

    fig, axes = plt.subplots(1, len(PANELS),
                             figsize=(3.6 * len(PANELS) + 1.2, 4.8),
                             sharey=True)
    fig.suptitle('aggregated metrics per group (mean over routes), 6 groups x 3 algorithms',
                 fontsize=13, fontweight='bold', y=0.99)

    for ax, (title, field, units, kind, fmt) in zip(axes, PANELS):
        data = np.full((n_rows, n_cols), np.nan)
        for i, (gname, routes) in enumerate(GROUPS):
            for j, (akey, _) in enumerate(ALGOS):
                m = collect_group_means(metrics, routes, akey, field)
                if m is not None:
                    data[i, j] = m
        masked = np.ma.masked_invalid(data)
        if kind == 'dist':
            cmap = DIST_CMAP.copy(); cmap.set_bad(color='#e2e8f0')
            norm = DIST_NORM
        else:
            cmap = COV_CMAP.copy(); cmap.set_bad(color='#e2e8f0')
            norm = COV_NORM
        ax.imshow(masked, aspect='auto', cmap=cmap, norm=norm)
        ax.set_xticks(range(n_cols))
        ax.set_xticklabels([a[1] for a in ALGOS], fontsize=8)
        ax.set_yticks(range(n_rows))
        ax.set_yticklabels([g[0] for g in GROUPS], fontsize=10)
        ax.set_title(f'{title} ({units})', fontsize=10)
        for i in range(n_rows):
            for j in range(n_cols):
                v = data[i, j]
                txt = '—' if np.isnan(v) else fmt.format(v)
                ax.text(j, i, txt, ha='center', va='center',
                        fontsize=9, color='black', fontweight='bold')

    plt.tight_layout(rect=[0, 0.13, 1, 0.95])
    cax_cov = fig.add_axes([0.10, 0.06, 0.36, 0.022])
    cax_dist = fig.add_axes([0.56, 0.06, 0.36, 0.022])
    cb_cov = fig.colorbar(plt.cm.ScalarMappable(norm=COV_NORM, cmap=COV_CMAP),
                          cax=cax_cov, orientation='horizontal',
                          ticks=[0, 25, 50, 75, 100])
    cb_cov.set_label('WP coverage (%)', fontsize=9)
    cb_cov.ax.tick_params(labelsize=8)
    cb_dist = fig.colorbar(plt.cm.ScalarMappable(norm=DIST_NORM, cmap=DIST_CMAP),
                           cax=cax_dist, orientation='horizontal',
                           ticks=[0, 5, 10, 20, 30])
    cb_dist.set_label('reach / return distance (m)', fontsize=9)
    cb_dist.ax.tick_params(labelsize=8)

    fig.savefig(OUT, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'  wrote {OUT}')


if __name__ == '__main__':
    main()
