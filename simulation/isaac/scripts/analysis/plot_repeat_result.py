#!/usr/bin/env python3
"""Per-route repeat RESULT plot.

Draws on one canvas:
  - scene_obstacles (trees/rocks/houses) from scene_obstacles.json
  - teach planned WPs (routes.json, red dashed)
  - repeat GT actual trajectory (traj_gt.csv from repeat_run, solid green)
  - obstacles (props/cones/tent) from spawn_obstacles.OBSTACLES
  - skipped WP markers (red X) parsed from goals.log
  - 10m supervisor trigger ring + turnaround label

Saves to routes/<NN>/repeat/results/repeat_run/repeat_result.png.
"""
import csv, json, re, sys
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
sys.path.insert(0, '/workspace/simulation/isaac/routes/_common/scripts')
from plot_repeat_plan import draw_scene, draw_obstacles, TURNAROUND, MAP_XY
from spawn_obstacles import OBSTACLES
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def load_traj_gt(path):
    if not Path(path).is_file():
        return []
    with open(path) as f:
        rows = list(csv.reader(f))
    xs, ys = [], []
    for row in rows:
        if not row or row[0].startswith('#'):
            continue
        try:
            xs.append(float(row[1])); ys.append(float(row[2]))
        except Exception:
            continue
    return xs, ys


def load_skipped_wps(goals_log, route_json):
    """Parse goals.log for WP ID that was SKIPped and return their (x,y)."""
    if not Path(goals_log).is_file():
        return []
    pts = json.load(open(route_json))
    wp_ids = set()
    skip_re = re.compile(r"WP (\d+) SKIP")
    with open(goals_log) as f:
        for line in f:
            m = skip_re.search(line)
            if m:
                wp_ids.add(int(m.group(1)))
    out = []
    for i in wp_ids:
        if 0 <= i < len(pts):
            out.append((pts[i][0], pts[i][1], i))
    return out


def parse_result_line(goals_log):
    if not Path(goals_log).is_file():
        return None
    with open(goals_log) as f:
        for line in reversed(f.readlines()):
            m = re.search(r'RESULT:\s*reached\s*(\d+)/(\d+)\s*skipped\s*(\d+)\s*duration\s*(\d+)s', line)
            if m:
                return dict(reached=int(m.group(1)), total=int(m.group(2)),
                            skipped=int(m.group(3)), duration=int(m.group(4)))
    return None


def plot_result(name, out_png):
    # TODO think about decomposing this monster function
    base = Path(f'/workspace/simulation/isaac/routes/{name}/repeat/results/repeat_run')
    teach_pts = json.load(open('/workspace/simulation/isaac/routes/_common/routes.json'))[name]
    traj = load_traj_gt(base / 'traj_gt.csv')
    skips = load_skipped_wps(base / 'goals.log', '/workspace/simulation/isaac/routes/_common/routes.json')
    res = parse_result_line(base / 'goals.log')

    fig, ax = plt.subplots(figsize=(12, 7))
    draw_scene(ax)

    # teach planned route
    ax.plot([p[0] for p in teach_pts], [p[1] for p in teach_pts],
            color='#d62728', linewidth=1.4, linestyle='--',
            alpha=0.8, label='Teach plan', zorder=3)

    # repeat GT actual
    if traj:
        xs, ys = traj
        ax.plot(xs, ys, color='#16a34a', linewidth=1.8,
                alpha=0.95, label=f'Repeat GT ({len(xs)} samples)', zorder=4)

    draw_obstacles(ax, name)

    # skipped WPs
    if skips:
        for sx, sy, i in skips:
            ax.plot(sx, sy, marker='x', color='#b91c1c',
                    markersize=11, markeredgewidth=2.5, zorder=7)
        ax.plot([], [], marker='x', color='#b91c1c', linestyle='None',
                markersize=11, markeredgewidth=2.5, label=f'Skipped WP ({len(skips)})')

    # turnaround
    tx, ty = TURNAROUND[name]
    ax.add_patch(Circle((tx, ty), 10, facecolor='none', edgecolor='#ff8c00',
                        linewidth=1.5, linestyle=':', alpha=0.9, zorder=5,
                        label='Supervisor 10m trigger'))
    ax.plot(tx, ty, 'x', color='#ff8c00', markersize=12, markeredgewidth=2)

    # spawn
    sx, sy = teach_pts[0]
    ax.plot(sx, sy, 'o', color='#0369a1', markersize=14, markeredgecolor='black',
            markeredgewidth=1.5, label=f'Spawn ({sx:.0f}, {sy:.0f})', zorder=6)

    xlim, ylim = MAP_XY[name]
    ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    title = f'{name} - repeat result'
    if res:
        title += f'  ({res["reached"]}/{res["total"]} reached, {res['skipped']} skipped, {res["duration"]}s)'
    ax.set_title(title)

    n_cones = sum(len(g) for g in OBSTACLES.get(name, {}).get('cones', []))
    n_props = len(OBSTACLES.get(name, {}).get('props', []))
    n_tent = 1 if OBSTACLES.get(name, {}).get('tent') else 0
    desc = f'{n_cones} cones / {n_tent} tent / {n_props} props'
    info = f'Obstacles: {desc}'
    if res:
        info += f'\nReached: {res["reached"]}/{res["total"]}  Skipped: {res["skipped"]}  Duration: {res["duration"]}s'
    ax.text(0.02, 0.02, info,
            transform=ax.transAxes, fontsize=9, family='monospace',
            verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white', edgecolor='gray', alpha=0.9))

    ax.legend(loc='upper right', fontsize=8, framealpha=0.95)
    plt.tight_layout()
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_png, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'{name} -> {out_png}  ({"with traj" if traj else "no traj_gt.csv"})')


if __name__ == '__main__':
    only = sys.argv[1:] or list(TURNAROUND.keys())
    for name in only:
        out = f'/workspace/simulation/isaac/routes/{name}/repeat/results/repeat_run/repeat_result.png'
        plot_result(name, out)
