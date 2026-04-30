#!/usr/bin/env python3
"""anchors-in-action figure for the visual landmark matcher

trajectory plot of 03_south from exp 64 (best-of-breed), overlaid with
every place the matcher tried to publish a correction and what happened
to it - used in chapter 5 to show the matcher pulling nav pose toward
teach landmarks, plus the spots where the sanity gate refused.  reads
anchor_matches.csv (one row per attempt) and tf_slam.log (nav and gt
traces) from the exp 64 results
"""
import csv
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
from plot_trajectory_map import plot_trajectory_map
from matplotlib.lines import Line2D

ARC = Path('/root/isaac_archive/experiments/64_best_of_breed/results/repeat_run')
ANCHOR_CSV = ARC / 'anchor_matches.csv'
TF_LOG = ARC / 'tf_slam.log'

OUT_DIR = Path('/workspace/simulation/isaac/results/final/dev_history')
OUT_DIR.mkdir(parents=True, exist_ok=True)
OUT_PATH = OUT_DIR / 'anchors_in_action.png'

NAV_GT_RE = re.compile(
    r'nav=\(([-\d.]+),([-\d.]+)\)\s+gt=\(([-\d.]+),([-\d.]+)\)'
)


def load_anchor_events():
    pubs, rejects, no_cands = [], [], []
    with open(ANCHOR_CSV) as f:
        for r in csv.DictReader(f):
            try:
                vx = float(r['vio_x']); vy = float(r['vio_y'])
            except Exception:
                continue
            try:
                ax_pos = float(r['anchor_x']); ay_pos = float(r['anchor_y'])
            except Exception:
                ax_pos = ay_pos = None
            out = r['outcome']
            if out.startswith('published') and ax_pos is not None:
                shift = math.hypot(ax_pos - vx, ay_pos - vy)
                pubs.append((vx, vy, ax_pos, ay_pos, shift))
            elif out.startswith('consistency_fail') and ax_pos is not None:
                shift = math.hypot(ax_pos - vx, ay_pos - vy)
                rejects.append((vx, vy, ax_pos, ay_pos, shift))
            elif out.startswith('no_candidates'):
                no_cands.append((vx, vy))
    return pubs, rejects, no_cands


def write_traces():
    nav_csv = OUT_DIR / '_anchors_nav.csv'
    gt_csv = OUT_DIR / '_anchors_gt.csv'
    nav_rows, gt_rows = [], []
    with open(TF_LOG, errors='ignore') as f:
        for line in f:
            m = NAV_GT_RE.search(line)
            if not m: continue
            nx, ny, gx, gy = (float(g) for g in m.groups())
            nav_rows.append((nx, ny)); gt_rows.append((gx, gy))
    for path, rows in ((nav_csv, nav_rows), (gt_csv, gt_rows)):
        with open(path, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['x', 'y']); w.writerows(rows)
    return nav_csv, gt_csv, len(nav_rows)


def _draw_anchor_layer(pubs, rejects, no_cands):
    """returns the matplotlib hook callback closing over the events."""
    def _hook(ax):
        # accepted corrections: thin green arrow VIO -> anchor + dot at anchor
        for vx, vy, ax_pos, ay_pos, shift in pubs:
            s = min(shift, 3.0) / 3.0
            color = (0.08, 0.55 + 0.25 * s, 0.20, 0.55)
            ax.annotate(
                '',
                xy=(ax_pos, ay_pos), xytext=(vx, vy),
                arrowprops=dict(arrowstyle='-|>', color=color,
                                lw=0.6, mutation_scale=5),
                zorder=11,
            )
            ax.scatter([ax_pos], [ay_pos], s=6, c='#15803d',
                       marker='o', zorder=12, edgecolors='none')

        # consistency-gate rejections: red x at the VIO source pose, plus a
        # thin dashed red line out to where the proposed correction would
        # have pulled the pose. the dashed end of the line shows the wrong-
        # cluster anchor target the matcher rejected.
        if rejects:
            for vx, vy, ax_pos, ay_pos, shift in rejects:
                ax.plot([vx, ax_pos], [vy, ay_pos],
                        color='#dc2626', linestyle=':', linewidth=0.8,
                        alpha=0.7, zorder=12)
                ax.scatter([ax_pos], [ay_pos], s=22, c='#dc2626',
                           marker='*', zorder=12, edgecolors='none',
                           alpha=0.7)
            rx = [r[0] for r in rejects]; ry = [r[1] for r in rejects]
            ax.scatter(rx, ry, s=70, c='#dc2626', marker='x',
                       linewidth=1.4, zorder=13)

        # no-candidates: gray square at VIO position (subtle)
        if no_cands:
            nx = [r[0] for r in no_cands]; ny = [r[1] for r in no_cands]
            ax.scatter(nx, ny, s=22, c='#94a3b8', marker='s',
                       zorder=10, edgecolors='none', alpha=0.65)

        ax._extra_legend_handles = [
            Line2D([0], [0], marker='o', color='#15803d', linestyle='none',
                   markersize=6,
                   label=f'matcher accepted ({len(pubs)} times, '
                         f'arrow shows where it pulled the pose)'),
            Line2D([0], [0], marker='x', color='#dc2626', linestyle='none',
                   markersize=8, markeredgewidth=1.4,
                   label=f'matcher refused to publish '
                         f'({len(rejects)} times — shift would have been over 5 m)'),
            Line2D([0], [0], marker='*', color='#dc2626', linestyle=':',
                   markersize=7, linewidth=0.8,
                   label='where the rejected match would have pulled the pose'),
            Line2D([0], [0], marker='s', color='#94a3b8', linestyle='none',
                   markersize=6,
                   label=f'no teach landmarks within 8 m '
                         f'({len(no_cands)} times)'),
        ]
    return _hook


def main():
    pubs, rejects, no_cands = load_anchor_events()
    print(f'  events: {len(pubs)} accepted, {len(rejects)} rejected, '
          f'{len(no_cands)} no-candidates')

    nav_csv, gt_csv, n_log = write_traces()
    print(f'  nav/gt samples from tf_slam.log: {n_log}')

    plot_trajectory_map(
        trajectories=[
            {'csv': str(gt_csv),
             'label': 'where the robot actually went',
             'color': '#dc2626', 'linewidth': 1.8,
             'x_col': 'x', 'y_col': 'y'},
            {'csv': str(nav_csv),
             'label': 'pose used by the planner (after every accepted match)',
             'color': '#f97316', 'linewidth': 1.0, 'linestyle': '--',
             'x_col': 'x', 'y_col': 'y'},
        ],
        output=str(OUT_PATH),
        title='visual landmark matcher in action — exp 64 best-of-breed, south route',
        metrics_lines=[
            'matcher tried 4 585 times over the run',
            f'accepted: {len(pubs)} corrections, about 10 percent',
            f'rejected by sanity check: {len(rejects)} wrong-cluster matches',
            f'no nearby teach landmarks: {len(no_cands)} times',
            'the rest were tried but had too few PnP inliers',
        ],
        with_obstacles=True, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 9.0),
        extra_artists=_draw_anchor_layer(pubs, rejects, no_cands),
    )

    for p in (nav_csv, gt_csv):
        try: p.unlink()
        except Exception: pass
    print(f'  wrote {OUT_PATH}')


if __name__ == '__main__':
    main()
