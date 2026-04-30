#!/usr/bin/env python3
import csv, json, re, sys
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
sys.path.insert(0, '/workspace/simulation/isaac/routes/_common/scripts')
sys.path.insert(0, '/workspace/repo/simulation/isaac/scripts')
sys.path.insert(0, '/workspace/repo/simulation/isaac/routes/_common/scripts')

from plot_trajectory_map import plot_trajectory_map
from plot_repeat_plan import TURNAROUND


DATASETS = Path('/root/isaac_tr_datasets')
ROUTES_DIR = Path('/workspace/repo/simulation/isaac/routes')
MARGIN_M = 6.0

TEACH_COL  = '#000000'   # black  - teach GT
REPEAT_COL = '#eab308'   # yellow - repeat GT
NAV_COL    = '#1d4ed8'   # blue   - published nav pose

# legacy 01/02/03 routes used the old `road` / `north` / `south` keys in
# OBSTACLES (cones + tent), so map them so the plot picks the obstacles up
LEGACY_OBS_ALIAS = {
    '01_road':         'road',
    '02_north_forest': 'north',
    '03_south':        'south',
}


NAV_RE = re.compile(
    r'nav=\(([\-\d.]+),([\-\d.]+)\)\s*gt=\(([\-\d.]+),([\-\d.]+)\)\s*err=([\-\d.]+)m')


def _read_teach_gt(route):
    p = DATASETS / route / 'teach' / 'teach_outputs' / 'vio_pose_dense.csv'
    if not p.is_file():
        return []
    pts = []
    with open(p) as f:
        for row in csv.DictReader(f):
            try:
                pts.append((float(row['gt_x']), float(row['gt_y'])))
            except (KeyError, ValueError):
                continue
    return pts


def _read_repeat_gt(route):
    p = DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'traj_gt.csv'
    if not p.is_file():
        return []
    pts = []
    with open(p) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#') or row[0].startswith('t'):
                continue
            try:
                pts.append((float(row[1]), float(row[2])))
            except ValueError:
                continue
    return pts


def _read_nav_pose(route):
    p = DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'tf_slam.log'
    if not p.is_file():
        return [], 0.0
    nav, errs = [], []
    for line in open(p):
        m = NAV_RE.search(line)
        if not m:
            continue
        try:
            nav.append((float(m.group(1)), float(m.group(2))))
            errs.append(float(m.group(5)))
        except ValueError:
            continue
    if not errs:
        return nav, 0.0
    finite = [e for e in errs if e < 50]   # drop jump transients
    drift_mean = sum(finite) / max(1, len(finite))
    return nav, drift_mean


def _bbox(*point_lists):
    xs, ys = [], []
    for pts in point_lists:
        for x, y in pts:
            xs.append(x); ys.append(y)
    if not xs:
        return None
    return (min(xs) - MARGIN_M, max(xs) + MARGIN_M,
            min(ys) - MARGIN_M, max(ys) + MARGIN_M)


def _csv_dump(out_dir, label, pts):
    p = out_dir / f'_tmp_{label}.csv'
    with open(p, 'w') as f:
        f.write('x,y\n')
        for x, y in pts:
            f.write(f'{x:.4f},{y:.4f}\n')
    return p


def plot_one(route):
    teach = _read_teach_gt(route)
    repeat = _read_repeat_gt(route)
    nav, drift_mean = _read_nav_pose(route)

    if not teach and not repeat and not nav:
        print(f'{route}: no data, skipped')
        return

    out_dir = ROUTES_DIR / route / 'repeat'
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / 'repeat_trajectory.png'

    trajs = []
    tmp_files = []
    for label, pts, col, lw, ls in (
        ('teach_gt',  teach,  TEACH_COL,  1.6, '--'),
        ('repeat_gt', repeat, REPEAT_COL, 2.0, '-'),
        ('nav_pose',  nav,    NAV_COL,    1.5, '-'),
    ):
        if not pts:
            continue
        csv_p = _csv_dump(out_dir, f'{route}_{label}', pts)
        tmp_files.append(csv_p)
        legend = {
            'teach_gt':  'Teach GT (taught route)',
            'repeat_gt': 'Repeat GT (actual robot path)',
            'nav_pose':  'Published nav pose (where robot thought it was)',
        }[label]
        trajs.append({
            'csv': str(csv_p), 'label': legend, 'color': col,
            'linewidth': lw, 'linestyle': ls,
            'x_col': 'x', 'y_col': 'y',
        })

    bb = _bbox(teach, repeat, nav, [TURNAROUND.get(route, (0, 0))])
    xlim, ylim = (bb[0], bb[1]), (bb[2], bb[3])
    width  = xlim[1] - xlim[0]
    height = ylim[1] - ylim[0]
    scale = 18.0 / max(width, height)
    figsize = (max(10.0, width * scale + 2),
               max(7.0, height * scale + 1))

    metrics = [
        f'route        : {route}  (repeat run)',
        f'teach GT pts : {len(teach)}',
        f'repeat GT pts: {len(repeat)}',
        f'nav pose pts : {len(nav)} ticks',
        f'drift mean   : {drift_mean:.2f} m',
    ]

    plot_trajectory_map(
        trajectories=trajs,
        output=str(out_png),
        title=(f'{route} - teach GT (black) + repeat GT (yellow) + '
               f'localised nav pose (blue), drift mean {drift_mean:.1f} m'),
        metrics_lines=metrics,
        with_obstacles=True,
        with_waypoints=False,
        route=LEGACY_OBS_ALIAS.get(route, route),
        reference_csv=None,
        xlim=xlim, ylim=ylim,
        figsize=figsize,
    )
    for p in tmp_files:
        try: p.unlink()
        except Exception: pass
    print(f'{route} -> {out_png}')


if __name__ == '__main__':
    only = sys.argv[1:] or sorted(TURNAROUND.keys())
    for r in only:
        try:
            plot_one(r)
        except Exception as e:
            print(f'{r}: FAIL {e}')
