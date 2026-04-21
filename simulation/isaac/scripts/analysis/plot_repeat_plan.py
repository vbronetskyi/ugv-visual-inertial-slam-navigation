#!/usr/bin/env python3
import csv, json, sys, math
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
sys.path.insert(0, '/workspace/simulation/isaac/routes/_common/scripts')
from spawn_obstacles import OBSTACLES, PROP_ASSETS
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.lines import Line2D


TURNAROUND = {
    '04_nw_se': (+65, -35),
    '05_ne_sw': (-90, -35),
    '06_nw_ne': (+65, +35),
    '07_se_sw': (-90, -35),
    '08_nw_sw': (-90, -35),
    '09_se_ne': (+65, +35),
}
MAP_XY = {
    '04_nw_se': ((-100, 75), (-45, 42)),
    '05_ne_sw': ((-100, 75), (-45, 42)),
    '06_nw_ne': ((-100, 75), (-45, 42)),
    '07_se_sw': ((-100, 75), (-45, 42)),
    '08_nw_sw': ((-110, -70), (-45, 42)),
    '09_se_ne': ((50, 80), (-45, 42)),
}


def draw_scene(ax, scene_path='/workspace/simulation/isaac/routes/_common/scene_obstacles.json'):
    for m in json.load(open(scene_path)):
        if m['type'] in ('tree', 'roadside_tree'):
            ax.add_patch(Circle((m['x'], m['y']), 1.3, facecolor='#1e4d1e',
                                edgecolor='#0d2d0d', linewidth=0.3, alpha=0.7, zorder=2))
            ax.add_patch(Circle((m['x'], m['y']), 0.3, facecolor='#5a3a1a', alpha=0.9, zorder=2.1))
        elif m['type'] == 'shrub':
            ax.add_patch(Circle((m['x'], m['y']), 0.4, facecolor='#3a6b2e', alpha=0.55, zorder=2))
        elif m['type'] == 'rock':
            ax.add_patch(Circle((m['x'], m['y']), 0.8, facecolor='#6b6b6b',
                                edgecolor='#3a3a3a', linewidth=0.4, alpha=0.8, zorder=2))
        elif m['type'] == 'house':
            ax.add_patch(Rectangle((m['x'] - 3, m['y'] - 3), 6, 6, facecolor='#8b5a2b',
                                   edgecolor='#3a2810', linewidth=0.8, alpha=0.9, zorder=2))


PROP_COLOR = {
    'barrel_large': '#d97706', 'barrel_medium': '#d97706', 'barrel_small': '#d97706',
    'crate_plastic': '#94a3b8',
    'cardbox_large': '#a97a2e', 'cardbox_cube': '#a97a2e',
    'cardbox_flat': '#a97a2e', 'cardbox_small': '#a97a2e',
    'concrete_block_a': '#6b7280', 'concrete_block_b': '#6b7280',
    'dumpster_large': '#1d4ed8', 'dumpster_small': '#1d4ed8',
    'trashcan': '#16a34a',
    'firehydrant': '#dc2626',
    'railing': '#fbbf24',
    'bench': '#7c3aed',
}


def draw_obstacles(ax, route_name):
    conf = OBSTACLES.get(route_name, {})
    # legacy cones
    for group in conf.get('cones', []):
        for cx, cy in group:
            ax.add_patch(Circle((cx, cy), 0.3, facecolor='#ff6600',
                                edgecolor='#8B0000', linewidth=0.5, alpha=0.95, zorder=6))
    # legacy tent
    if conf.get('tent'):
        tx, ty = conf['tent']
        ax.add_patch(Rectangle((tx - 1.1, ty - 1.0), 2.2, 2.0, facecolor='#2d5a2d',
                               edgecolor='#0d2d0d', linewidth=0.8, alpha=0.9, zorder=6))
        ax.text(tx, ty + 1.5, 'Tent', ha='center', fontsize=8, color='#8B0000',
                fontweight='bold', zorder=7)
    # new props
    for p in conf.get('props', []):
        _, r_m = PROP_ASSETS.get(p['kind'], (None, 0.5))
        col = PROP_COLOR.get(p['kind'], '#888')
        ax.add_patch(Circle((p['x'], p['y']), r_m, facecolor=col,
                            edgecolor='#000', linewidth=0.6, alpha=0.9, zorder=6))


def plot_route(name, out_png):
    # NOTE: this file is shared across 9 routes, keep changes backwards compatible
    pts = json.load(open('/workspace/simulation/isaac/routes/_common/routes.json'))[name]
    fig, ax = plt.subplots(figsize=(12, 7))
    draw_scene(ax)

    # planned trajectory
    ax.plot([p[0] for p in pts], [p[1] for p in pts],
            color='#d62728', linewidth=1.8, linestyle='--', label=f'{name} planned route')

    draw_obstacles(ax, name)

    tx, ty = TURNAROUND[name]
    ax.add_patch(Circle((tx, ty), 10, facecolor='none', edgecolor='#ff8c00',
                        linewidth=1.5, linestyle=':', alpha=0.9, zorder=5,
                        label='Supervisor 10m trigger'))
    ax.plot(tx, ty, 'x', color='#ff8c00', markersize=12, markeredgewidth=2,
            label=f'Turnaround ({tx}, {ty})')

    # spawn marker
    sx, sy = pts[0]
    ax.plot(sx, sy, 'o', color='#16a34a', markersize=14, markeredgecolor='black',
            markeredgewidth=1.5, label=f'Spawn ({sx:.0f}, {sy:.0f})', zorder=6)

    xlim, ylim = MAP_XY[name]
    ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title(f'{name} - repeat plan with obstacles')

    n_cones = sum(len(g) for g in OBSTACLES.get(name, {}).get('cones', []))
    n_props = len(OBSTACLES.get(name, {}).get('props', []))
    n_tent = 1 if OBSTACLES.get(name, {}).get('tent') else 0
    desc = f'{n_cones} cones + {n_tent} tent + {n_props} props'
    ax.text(0.02, 0.02, f'Obstacles: {desc}\nTurnaround: fires when <10m from ({tx}, {ty})',
            transform=ax.transAxes, fontsize=9,
            family='monospace', verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white', edgecolor='gray', alpha=0.9))

    ax.legend(loc='upper right', fontsize=8, framealpha=0.95)
    plt.tight_layout()
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_png, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'{name} -> {out_png}')


if __name__ == '__main__':
    only = sys.argv[1:] or list(TURNAROUND.keys())
    for name in only:
        out = f'/workspace/simulation/isaac/routes/{name}/repeat/results/repeat_run/plan_obstacles.png'
        plot_route(name, out)
