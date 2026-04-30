#!/usr/bin/env python3
"""anchor effectiveness figure - does the matcher actually reduce drift?

two-panel figure built from exp 56 data (cleanest matcher-active run).
top panel: localization error vs distance with green ticks at every
accepted anchor, background shaded by matcher state (fresh/stale/silent).
bottom: error distribution bucketed by "time since last accepted anchor",
which makes the relationship explicit - fresh anchor means sub-metre
err, silent over five minutes means metres of drift.  reads tf_slam.log
+ anchor_matches.csv from the exp 56 results
"""
import csv
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

ARC = Path('/root/isaac_archive/experiments/56_projection_cap_1m/results/repeat_run')
TF_LOG = ARC / 'tf_slam.log'
ANCHOR_CSV = ARC / 'anchor_matches.csv'

OUT_DIR = Path('/workspace/simulation/isaac/results/final/dev_history')
OUT_PATH = OUT_DIR / 'anchor_effectiveness.png'

TS_RE = re.compile(r'\[(\d+\.\d+)\]')
ERR_RE = re.compile(r'err=([\d.]+)m')
AGE_RE = re.compile(r'anchor_age=([-\d.]+)s')
DIST_RE = re.compile(r'dist=([\d.]+)m')

BUCKETS = [
    (0, 5,    'just published\n(<5 s)',        '#15803d'),
    (5, 30,   'recent\n(5-30 s)',              '#65a30d'),
    (30, 120, 'stale\n(30-120 s)',             '#ca8a04'),
    (120, 300,'going silent\n(2-5 min)',       '#ea580c'),
    (300, 99999,'silent\n(>5 min)',            '#dc2626'),
]


def load_log():
    out = []
    with open(TF_LOG, errors='ignore') as f:
        for line in f:
            ts = TS_RE.search(line); e = ERR_RE.search(line)
            a = AGE_RE.search(line); d = DIST_RE.search(line)
            if not (ts and e and a and d): continue
            out.append((float(ts.group(1)),
                        float(d.group(1)),
                        float(e.group(1)),
                        float(a.group(1))))
    return out


def load_anchors():
    pubs = []
    with open(ANCHOR_CSV) as f:
        for r in csv.DictReader(f):
            if r['outcome'].startswith('published'):
                try: pubs.append(float(r['ts']))
                except: pass
    return pubs


def bucket_for(age):
    for lo, hi, _, _ in BUCKETS:
        if lo <= age < hi:
            return (lo, hi)
    return None


def main():
    log = load_log()
    pubs = load_anchors()
    if not log:
        print('  no log data'); return
    t0 = log[0][0]
    times = np.array([r[0] - t0 for r in log])
    dists = np.array([r[1] for r in log])
    errs = np.array([r[2] for r in log])
    ages = np.array([r[3] for r in log])
    pubs_t = [p - t0 for p in pubs]
    # convert publish timestamps to distance via interpolation (monotone t->d)
    # log times are monotone, so np.interp on (t -> d) works
    pubs_d = np.interp(pubs_t, times, dists)

    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(2, 1, height_ratios=[2.0, 1.0], hspace=0.35)

    # top panel: err vs distance
    ax1 = fig.add_subplot(gs[0])
    # color the area below the curve by current matcher state
    state_colors = {b[:2]: b[3] for b in BUCKETS}
    valid = ages >= 0
    for i in range(len(dists) - 1):
        if not valid[i]: continue
        b = bucket_for(ages[i])
        if b is None: continue
        ax1.axvspan(dists[i], dists[i+1], color=state_colors[b],
                    alpha=0.10, zorder=1)
    ax1.plot(dists, errs, color='#0f172a', linewidth=1.4, zorder=4,
             label='localization error vs ground truth')
    # anchor publish events as green ticks at the bottom
    ymax_top = max(errs.max() * 1.1, 5.0)
    ax1.set_ylim(0, ymax_top)
    tick_y = ymax_top * 0.04
    ax1.scatter(pubs_d, [tick_y] * len(pubs_d), marker='|',
                color='#15803d', s=80, linewidths=1.4, zorder=5,
                label=f'matcher accepted ({len(pubs_d)} times)')
    ax1.set_xlabel('distance driven (m)', fontsize=11)
    ax1.set_ylabel('error against ground truth (m)', fontsize=11)
    ax1.set_title(
        'how anchors affect localization error — exp 56, south route, 1 390 m',
        fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=10)

    # background-color legend
    state_legend = [Patch(facecolor=c, alpha=0.25,
                          label=lbl.replace('\n', ' '))
                    for lo, hi, lbl, c in BUCKETS]
    ax1.legend(handles=ax1.get_legend_handles_labels()[0] + state_legend,
               loc='upper left', fontsize=9, ncol=2, framealpha=0.95)

    # bottom panel: err distribution per anchor-age bucket
    ax2 = fig.add_subplot(gs[1])
    bucket_data = []; bucket_labels = []; bucket_colors = []
    for lo, hi, label, color in BUCKETS:
        es = [e for e, a in zip(errs, ages) if a >= 0 and lo <= a < hi]
        if es:
            bucket_data.append(es)
            bucket_labels.append(f'{label}\nn={len(es)}')
            bucket_colors.append(color)

    bp = ax2.boxplot(bucket_data, tick_labels=bucket_labels, patch_artist=True,
                     widths=0.55, medianprops=dict(color='black', linewidth=1.5),
                     showfliers=True, flierprops=dict(marker='.', markersize=3,
                                                       markerfacecolor='#94a3b8',
                                                       markeredgecolor='none'))
    for patch, color in zip(bp['boxes'], bucket_colors):
        patch.set_facecolor(color); patch.set_alpha(0.55)

    # annotate medians
    for i, es in enumerate(bucket_data):
        med = sorted(es)[len(es) // 2]
        ax2.annotate(f'{med:.2f} m', xy=(i + 1, med),
                     xytext=(8, 4), textcoords='offset points',
                     fontsize=9, fontweight='bold', color='black')

    ax2.set_ylabel('error (m)', fontsize=11)
    ax2.set_title('error distribution as the matcher stays silent longer',
                  fontsize=11)
    ax2.grid(True, axis='y', alpha=0.3)
    ax2.set_ylim(0, max(max(es) for es in bucket_data) * 1.1)

    fig.savefig(OUT_PATH, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'  wrote {OUT_PATH}')


if __name__ == '__main__':
    main()
