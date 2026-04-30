#!/usr/bin/env python3
"""per-group algorithm comparison heatmaps

reads metrics.json (15 routes x 3 algorithms x +-12 metric fields,
written by compute_metrics.py) and writes one heatmap PNG per group to
routes/route_groups/heatmap_<group>.png.  each figure has four
subpanels side by side - WP coverage, reach distance, return distance,
drift mean - rows are routes in the group, columns are the 3
algorithms, colour goes red=bad green=good (semantics flipped per
metric)
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap, Normalize

# smooth gradient with control points at the user-specified breakpoints.
# distances: 0 -> deep green, 5 -> lime, 10 -> yellow, 20 -> orange, 30+ red.
# the gradient between control points blends smoothly so each band stays in
# its colour family (0-5 reads as "greens", 5-10 as "yellows", etc.) without
# the hard step look.
DIST_VMAX = 30.0
DIST_CMAP = LinearSegmentedColormap.from_list(
    'dist',
    [
        (0.0,         '#15803d'),  # 0 m   - deep green
        (5.0/DIST_VMAX,  '#84cc16'),  # 5 m   - lime (end of green band)
        (10.0/DIST_VMAX, '#fde047'),  # 10 m  - yellow
        (20.0/DIST_VMAX, '#f97316'),  # 20 m  - orange
        (1.0,         '#b91c1c'),  # 30+ m - deep red
    ],
)
DIST_NORM = Normalize(vmin=0, vmax=DIST_VMAX, clip=True)

# coverage uses the same anchor pattern but inverted:
# high coverage = good = green, low coverage = bad = red.
COV_VMAX = 100.0
COV_CMAP = LinearSegmentedColormap.from_list(
    'cov',
    [
        (0.0,          '#b91c1c'),
        (25.0/COV_VMAX,'#f97316'),
        (50.0/COV_VMAX,'#fde047'),
        (75.0/COV_VMAX,'#84cc16'),
        (1.0,          '#15803d'),
    ],
)
COV_NORM = Normalize(vmin=0, vmax=COV_VMAX, clip=True)

DIST_COLORS = ['#15803d', '#84cc16', '#fde047', '#f97316', '#b91c1c']
COV_COLORS  = ['#b91c1c', '#f97316', '#fde047', '#84cc16', '#15803d']

METRICS_JSON = Path('/workspace/simulation/isaac/routes/_common/metrics.json')
OUT_DIR = Path('/workspace/simulation/isaac/routes/route_groups')
OUT_DIR.mkdir(parents=True, exist_ok=True)

ALGOS = [
    ('exp 74 stock',  'stock Nav2\n(no matcher,\nno detour ring)'),
    ('exp 76 RGB-D',  'our stack,\nno IMU\n(RGB-D only VIO)'),
    ('our custom',    'our full stack\n(matcher + IMU\n+ detour + finisher)'),
]
ALGO_KEYS = [a[0] for a in ALGOS]
ALGO_LABELS = [a[1] for a in ALGOS]

GROUPS = [
    ('G1_forest', 'group 1, routes through dense forest (10 routes)',
     ['02_north_forest', '03_south', '04_nw_se', '05_ne_sw', '06_nw_ne',
      '07_se_sw', '08_nw_sw', '11_nw_mid', '13_cross_nws', '15_wmid_smid']),
    ('G2_open', 'group 2, routes through open / minimal forest (5 routes)',
     ['01_road', '09_se_ne', '10_nmid_smid', '12_ne_mid', '14_se_mid']),
    ('G3_short', 'group 3, short routes, length 160-205 m (8 routes)',
     ['08_nw_sw', '09_se_ne', '10_nmid_smid', '11_nw_mid', '12_ne_mid',
      '13_cross_nws', '14_se_mid', '15_wmid_smid']),
    ('G4_long', 'group 4, long routes, length 333-407 m (7 routes)',
     ['01_road', '02_north_forest', '03_south', '04_nw_se', '05_ne_sw',
      '06_nw_ne', '07_se_sw']),
    ('G5_cones_tent', 'group 5, cone walls + tent obstacles (4 routes)',
     ['01_road', '02_north_forest', '03_south', '04_nw_se']),
    ('G6_mixed_props', 'group 6, mixed prop obstacles (11 routes)',
     ['05_ne_sw', '06_nw_ne', '07_se_sw', '08_nw_sw', '09_se_ne',
      '10_nmid_smid', '11_nw_mid', '12_ne_mid', '13_cross_nws',
      '14_se_mid', '15_wmid_smid']),
]


PANELS = [
    # (title,           field,      units, kind,    fmt)
    ('WP coverage',     'cov_pct',  '%',   'cov',   '{:.0f}'),
    ('reach distance',  'final_d',  'm',   'dist',  '{:.1f}'),
    ('return distance', 'return_d', 'm',   'dist',  '{:.1f}'),
]


def collect(metrics, route_ids, field):
    """returns NxA matrix of values, with NaN for missing routes/algos."""
    n = len(route_ids); a = len(ALGO_KEYS)
    out = np.full((n, a), np.nan)
    for i, rid in enumerate(route_ids):
        if rid not in metrics: continue
        for j, ak in enumerate(ALGO_KEYS):
            if ak not in metrics[rid]: continue
            v = metrics[rid][ak].get(field)
            if v is None: continue
            try: out[i, j] = float(v)
            except: pass
    return out


def render_group(group_id, title, route_ids, metrics):
    n = len(route_ids)
    fig, axes = plt.subplots(1, len(PANELS),
                             figsize=(3.4 * len(PANELS) + 2, max(3.5, 0.5 * n + 2.0)),
                             sharey=True)
    fig.suptitle(title, fontsize=13, fontweight='bold', y=0.99)

    for ax, (ptitle, field, units, kind, fmt) in zip(axes, PANELS):
        data = collect(metrics, route_ids, field)
        masked = np.ma.masked_invalid(data)
        if kind == 'dist':
            cmap = DIST_CMAP.copy(); cmap.set_bad(color='#e2e8f0')
            norm = DIST_NORM
        else:
            cmap = COV_CMAP.copy(); cmap.set_bad(color='#e2e8f0')
            norm = COV_NORM
        ax.imshow(masked, aspect='auto', cmap=cmap, norm=norm)
        ax.set_xticks(range(len(ALGO_KEYS)))
        ax.set_xticklabels(ALGO_LABELS, fontsize=8)
        ax.set_yticks(range(n))
        ax.set_yticklabels(route_ids, fontsize=9)
        ax.set_title(f'{ptitle} ({units})', fontsize=10)
        for i in range(n):
            for j in range(len(ALGO_KEYS)):
                v = data[i, j]
                if np.isnan(v):
                    txt, color = '—', '#64748b'
                else:
                    txt = fmt.format(v); color = 'black'
                ax.text(j, i, txt, ha='center', va='center',
                        fontsize=8.5, color=color, fontweight='bold')

    # two thin horizontal colour-bars at the bottom showing the gradient
    # control points (5 / 10 / 20 m for distances, 25 / 50 / 75 % for cov).
    # placed in their own axes below the heatmap row so they don't collide
    # with the algorithm tick labels.
    plt.tight_layout(rect=[0, 0.10, 1, 0.96])
    cax_cov = fig.add_axes([0.10, 0.04, 0.36, 0.018])
    cax_dist = fig.add_axes([0.56, 0.04, 0.36, 0.018])
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
    out = OUT_DIR / f'heatmap_{group_id}.png'
    fig.savefig(out, dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'  wrote {out}')


def main():
    metrics = json.loads(METRICS_JSON.read_text())
    for gid, title, routes in GROUPS:
        render_group(gid, title, routes, metrics)


if __name__ == '__main__':
    main()
