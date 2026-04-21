#!/usr/bin/env python3
"""three-way route comparison plot

our custom T&R vs exp 74 stock Nav2 vs exp 76 RGB-D only, on a given route
uses the canonical plot_trajectory_map template so scene + obstacles +
waypoints are drawn consistent across plots

this is the main figure that goes into the thesis - same axes, same colors,
same scene context for all three baselines.  makes the comparison legible
without needing to read three separate captions
"""
import csv, re, sys
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
sys.path.insert(0, '/workspace/simulation/isaac/routes/_common/scripts')
from plot_trajectory_map import plot_trajectory_map
from plot_repeat_plan import MAP_XY

RESULT_RE = re.compile(
    r'RESULT:\s*reached\s+(\d+)/(\d+)\s+skipped\s+(\d+)\s+duration\s+(\d+)s')


def result_line(goals_log):
    if not goals_log.is_file(): return None
    m = RESULT_RE.search(goals_log.read_text())
    return m.groups() if m else None


def short_status(route_dir):
    rl = result_line(route_dir / 'goals.log')
    if rl:
        r, t, s, d = rl
        pct = 100.0 * int(r) / int(t)
        return f'{r}/{t} ({pct:.0f}%), {d}s'
    wd = route_dir / 'watchdog_abort.txt'
    if wd.is_file():
        note = wd.read_text().strip()
        return f'WD_ABORT: {note[:60]}'
    return 'FAILED_EARLY'


def plot_route(route):
    # NOTE: this file is shared across 9 routes, keep changes backwards compatible
    our = Path(f'/root/isaac_tr_datasets/{route}/repeat/results/repeat_run')
    e74 = Path(f'/root/isaac_tr_datasets/{route}/baseline_stock_nav2')
    e76 = Path(f'/root/isaac_tr_datasets/{route}/baseline_rgbd_no_imu')
    teach = Path(f'/root/isaac_tr_datasets/{route}/teach/teach_outputs/traj_gt_world.csv')

    our_st = short_status(our)
    e74_st = short_status(e74)
    e76_st = short_status(e76)

    def traj_len(p):
        if not p.is_file(): return 0
        try: return sum(1 for _ in open(p)) - 1
        except Exception: return 0

    our_gt_n = traj_len(our / 'traj_gt.csv')
    e74_gt_n = traj_len(e74 / 'traj_gt.csv')
    e76_gt_n = traj_len(e76 / 'traj_gt.csv')

    trajs = []
    # Teach GT reference (thin dashed black) - shows where the teach was driven.
    if teach.is_file():
        trajs.append({'csv': str(teach),
                      'label': 'teach GT (reference)',
                      'color': '#111827', 'x_col': 'x', 'y_col': 'y',
                      'linewidth': 1.2, 'linestyle': '--'})
    for src, label, color, w in [
        (our / 'traj_gt.csv', f'our custom T&R ({our_st}) [{our_gt_n} GT samples]', '#f97316', 2.4),
        (e74 / 'traj_gt.csv', f'exp 74 stock Nav2 ({e74_st}) [{e74_gt_n} GT samples]', '#2563eb', 1.8),
        (e76 / 'traj_gt.csv', f'exp 76 RGB-D no IMU ({e76_st}) [{e76_gt_n} GT samples]', '#dc2626', 1.8),
    ]:
        if src.is_file() and src.stat().st_size > 50:
            trajs.append({'csv': str(src), 'label': label, 'color': color,
                          'x_col': 'x', 'y_col': 'y', 'linewidth': w})

    if not trajs:
        print(f'[{route}] no traj_gt.csv at all - skipped')
        return

    # Always show the full map so the whole route is in view.
    xlim, ylim = (-115, 90), (-55, 55)

    # Save into route's own results dir (so route README can link it
    # directly) + a mirror in _baselines_common/plots/ for aggregate view.
    out = Path(f'/root/isaac_tr_datasets/{route}/repeat/results/repeat_run/baseline_compare.png')
    out.parent.mkdir(parents=True, exist_ok=True)
    mirror = Path(f'/workspace/simulation/isaac/experiments/_baselines_common/plots/compare_{route}.png')
    mirror.parent.mkdir(parents=True, exist_ok=True)
    plot_trajectory_map(
        trajectories=trajs,
        output=str(out),
        title=f'{route} - our custom T&R vs exp 74 stock Nav2 vs exp 76 RGB-D only',
        metrics_lines=[
            f'our:    {our_st}',
            f'exp74:  {e74_st}',
            f'exp76:  {e76_st}',
        ],
        with_obstacles=True,
        with_waypoints=False,
        route=route if route in MAP_XY else 'road',
        xlim=xlim,
        ylim=ylim,
        figsize=(16, 10),
    )
    # Mirror to _baselines_common/plots/
    import shutil
    try: shutil.copy(out, mirror)
    except Exception: pass
    print(f'{route} -> {out}')


if __name__ == '__main__':
    only = sys.argv[1:] or ['01_road','02_north_forest','03_south',
                            '04_nw_se','05_ne_sw','06_nw_ne',
                            '07_se_sw','08_nw_sw','09_se_ne']
    for r in only:
        try:
            plot_route(r)
        except Exception as e:
            print(f'[{r}] ERROR {e}')
