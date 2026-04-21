#!/usr/bin/env python3
"""Reusable unified trajectory plot with scene map background.

Usage from any experiment:
    from plot_trajectory_map import plot_trajectory_map
    plot_trajectory_map(
        trajectories=[
            {'csv': 'path/to/traj.csv', 'label': 'My run - 95%',
             'color': '#1f77b4', 'x_col': 'x', 'y_col': 'y'},
        ],
        output='results/my_plot.png',
        title='My Trajectory Comparison',
        metrics_lines=[...],  # list of str for bottom-right box
        with_obstacles=True,  # show cone barriers + tent
        with_waypoints=True,
    )
"""
import csv
import json
import math
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Patch
from matplotlib.lines import Line2D

ROAD_WPS = [
    (-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
    (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
    (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
    (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
    (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5),
]

BARRIERS = [
    (-50, -8.0, -2.5, 'Barrier 1'),
    (15, -1.0, 4.0, 'Barrier 2'),
    (45, -3.0, 1.0, 'Barrier 3'),
]


def _route_points_from_json(route_name, routes_file='/tmp/slam_routes.json'):
    """Get route centerline from slam_routes.json for a given route."""
    try:
        with open(routes_file) as f:
            routes = json.load(f)
        return [(p[0], p[1]) for p in routes.get(route_name, [])]
    except Exception:
        return []


ROUTE_CONFIG = {
    'road': {
        'centerline': ROAD_WPS,
        'anchors': '/workspace/simulation/isaac/route_memory/road/anchors.json',
        'xlim': (-100, 110), 'ylim': (-28, 15),
    },
    'south': {
        'centerline': None,  # from slam_routes.json
        'anchors': '/workspace/simulation/isaac/route_memory/south/anchors.json',
        'xlim': (-110, 85), 'ylim': (-50, 5),
    },
    'north': {
        'centerline': None,
        'anchors': None,
        'xlim': (-110, 85), 'ylim': (-5, 50),
    },
}


def _parse_traj(path, x_col='gt_x', y_col='gt_y'):
    xs, ys = [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            xc = x_col if x_col in row else ('x' if 'x' in row else 'gt_x')
            yc = y_col if y_col in row else ('y' if 'y' in row else 'gt_y')
            xs.append(float(row[xc]))
            ys.append(float(row[yc]))
    return np.array(xs), np.array(ys)


def _load_waypoints(anchors_file, spacing=4.0):
    with open(anchors_file) as f:
        all_anchors = json.load(f)
    max_x = max(range(len(all_anchors)), key=lambda i: all_anchors[i]['x'])
    outbound = all_anchors[: max_x + 1]
    wps = []
    last = None
    for a in outbound:
        if last is None or math.hypot(a["x"] - last[0], a["y"] - last[1]) > spacing:
            wps.append(a)
            last = (a["x"], a["y"])
    if math.hypot(outbound[-1]["x"] - last[0], outbound[-1]["y"] - last[1]) > 1.0:
        wps.append(outbound[-1])
    return wps


def plot_trajectory_map(
    # TODO think about decomposing this monster function
    trajectories,
    output,
    title='Trajectory Map',
    metrics_lines=None,
    with_obstacles=True,
    with_waypoints=True,
    route='road',
    reference_csv="/workspace/simulation/isaac/experiments/35_road_obstacle_avoidance/logs/road_noobs_baseline.csv",
    scene_json='/tmp/gazebo_models.json',
    anchors_json=None,
    xlim=None,
    ylim=None,
    figsize=(20, 7),
):
    rc = ROUTE_CONFIG.get(route, ROUTE_CONFIG['road'])
    xlim = xlim or rc['xlim']
    ylim = ylim or rc['ylim']
    anchors_json = anchors_json or rc['anchors']
    if route != 'road':
        reference_csv = None  # no road reference for forest routes
        # with_obstacles kept as passed - south/north now support nav obstacles
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    ax.set_facecolor('#6b8e4e')  # grass

    # Road corridor - only on the actual "road" route in scene
    # (south/north forest routes have no physical road)
    if route == 'road':
        rx_arr = np.array([p[0] for p in ROAD_WPS])
        ry_arr = np.array([p[1] for p in ROAD_WPS])
        road_xs = np.linspace(xlim[0] - 5, xlim[1] + 5, 300)
        road_y = np.interp(road_xs, rx_arr, ry_arr)
        ax.fill_between(road_xs, road_y - 2.0, road_y + 2.0,
                        color='#c9a66b', alpha=0.9, zorder=1)
        ax.fill_between(road_xs, road_y - 1.2, road_y + 1.2,
                        color='#d9b783', alpha=0.6, zorder=1)
    # The physical road in the scene is always at ROAD_WPS - show it regardless of route
    if route != 'road':
        rx_arr = np.array([p[0] for p in ROAD_WPS])
        ry_arr = np.array([p[1] for p in ROAD_WPS])
        road_xs = np.linspace(xlim[0] - 5, xlim[1] + 5, 300)
        road_y = np.interp(road_xs, rx_arr, ry_arr)
        mask = (road_y > ylim[0] - 1) & (road_y < ylim[1] + 1)
        ax.fill_between(road_xs[mask], road_y[mask] - 2.0, road_y[mask] + 2.0,
                        color='#c9a66b', alpha=0.7, zorder=1)
        ax.fill_between(road_xs[mask], road_y[mask] - 1.2, road_y[mask] + 1.2,
                        color='#d9b783', alpha=0.5, zorder=1)

    # Teach reference path (dashed black) - only for forest routes
    # (road route already uses reference_csv below)
    if route != 'road':
        pts = _route_points_from_json(route)
        if pts:
            px = [p[0] for p in pts]
            py = [p[1] for p in pts]
            ax.plot(px, py, color='black', linewidth=1.8, alpha=0.6,
                    linestyle='--', zorder=3,
                    label='Teach reference path')

    # Scene objects (real positions + sizes matching physical collision in Isaac Sim)
    # See run_husky_forest.py: trees collision r=0.7m, rocks 0.8m, shrubs 0.4m
    # The run_husky_forest.py THINS trees: removes 10 at west corners + every 3rd
    if os.path.exists(scene_json):
        try:
            with open(scene_json) as f:
                scene = json.load(f)
            # Roadside trees are placed unconditionally (convert_gazebo_to_isaac.py),
            # so they must not be filtered by the forest-thinning logic.
            roadside = [m for m in scene if m['name'].startswith('roadside_tree_')]
            forest_trees = [m for m in scene if m['type'] in ('pine', 'oak')
                            and not m['name'].startswith('roadside_tree_')]
            # Mimic thinning logic from run_husky_forest.py (forest only)
            def _dist(m, cx, cy):
                return math.hypot(m['x']-cx, m['y']-cy)
            west_sw = sorted(forest_trees, key=lambda m: _dist(m, -120, -80))
            west_nw = sorted(forest_trees, key=lambda m: _dist(m, -120, 80))
            removed_names = set(t['name'] for t in west_sw[:5]) | set(t['name'] for t in west_nw[:5])
            active_trees = list(roadside)
            skip = 0
            for m in forest_trees:
                if m['name'] in removed_names:
                    continue
                skip += 1
                if skip % 3 == 0:
                    continue
                active_trees.append(m)

            # Draw active trees (physical collision r=0.7m trunk + 1.5m canopy visual)
            for m in active_trees:
                ax.add_patch(Circle((m['x'], m['y']), 1.3,
                                    facecolor='#1e4d1e', edgecolor='#0d2d0d',
                                    linewidth=0.3, alpha=0.75, zorder=2))
                # Trunk (matches physical collision radius 0.7m)
                ax.add_patch(Circle((m['x'], m['y']), 0.3,
                                    facecolor='#5a3a1a', alpha=0.9, zorder=2.1))

            for m in scene:
                t = m['type']
                if t == 'shrub':
                    ax.add_patch(Circle((m['x'], m['y']), 0.4,
                                        facecolor='#3a6b2e', alpha=0.55, zorder=2))
                elif t == 'rock':
                    ax.add_patch(Circle((m['x'], m['y']), 0.8,
                                        facecolor='#6b6b6b', edgecolor='#3a3a3a',
                                        linewidth=0.4, alpha=0.8, zorder=2))
                elif t == 'house':
                    ax.add_patch(Rectangle((m['x']-3, m['y']-3), 6, 6,
                                           facecolor='#8b5a2b', edgecolor='#3a2810',
                                           linewidth=0.8, alpha=0.9, zorder=2))
                elif t == 'barrel':
                    ax.add_patch(Circle((m['x'], m['y']), 0.5,
                                        facecolor='#a85c1f', edgecolor='#3a2810',
                                        linewidth=0.4, alpha=0.9, zorder=2))
        except Exception as e:
            print(f"  [warn] scene objects: {e}")

    # Reference path (no obstacles)
    if reference_csv and os.path.exists(reference_csv):
        try:
            rx, ry = _parse_traj(reference_csv, x_col='gt_x', y_col='gt_y')
            ax.plot(rx, ry, 'k--', linewidth=1.8, alpha=0.6,
                    label='Reference path (no obstacles)', zorder=3)
        except Exception as e:
            print(f"  [warn] reference: {e}")

    # Trajectories
    for t in trajectories:
        try:
            tx, ty = _parse_traj(t['csv'],
                                 x_col=t.get('x_col', 'gt_x'),
                                 y_col=t.get('y_col', 'gt_y'))
            ax.plot(tx, ty, color=t['color'], linewidth=2, alpha=0.85,
                    label=t['label'], zorder=4)
        except Exception as e:
            print(f"  [warn] {t.get('label', t['csv'])}: {e}")

    # Obstacles (road hardcoded, or from spawn_obstacles for other routes)
    if with_obstacles and route == 'road':
        for bx, y1, y2, name in BARRIERS:
            n = 0
            for y in np.arange(y1, y2 + 0.01, 1.0):
                ax.add_patch(Circle((bx, y), 0.3,
                                    facecolor='#ff6600', edgecolor='#8B0000',
                                    linewidth=0.5, alpha=0.95, zorder=6))
                n += 1
            ax.text(bx, y2 + 1.0, f'{name}\n{n} cones', ha='center', fontsize=9,
                    color='#8B0000', fontweight='bold', zorder=7)
        ax.add_patch(Rectangle((-20-1.0, -0.9), 2.0, 1.8,
                               facecolor='#2d5a2d', edgecolor='#0d2d0d',
                               linewidth=0.8, alpha=0.9, zorder=6))
        ax.text(-20, 1.5, 'Tent', ha='center', fontsize=9,
                color='#8B0000', fontweight='bold', zorder=7)
    elif with_obstacles and route not in ('road',):
        try:
            import sys as _sys
            _sys.path.insert(0, '/workspace/simulation/isaac/scripts')
            from spawn_obstacles import OBSTACLES as _OBS, PROP_ASSETS as _PA
            _nav = _OBS.get(route, {})
            for group in _nav.get('cones', []):
                for cx, cy in group:
                    ax.add_patch(Circle((cx, cy), 0.3,
                                        facecolor='#ff6600', edgecolor='#8B0000',
                                        linewidth=0.5, alpha=0.95, zorder=6))
            if _nav.get('tent'):
                tx, ty = _nav['tent']
                ax.add_patch(Rectangle((tx-1.1, ty-1.0), 2.2, 2.0,
                                       facecolor='#2d5a2d', edgecolor='#0d2d0d',
                                       linewidth=0.8, alpha=0.9, zorder=6))
                ax.text(tx, ty+1.5, 'Tent', ha='center', fontsize=9,
                        color='#8B0000', fontweight='bold', zorder=7)
            _PROP_COLOR = {
                'barrel_large': '#d97706', 'barrel_medium': '#d97706',
                'barrel_small': '#d97706', 'crate_plastic': '#94a3b8',
                'cardbox_large': '#a97a2e', 'cardbox_cube': '#a97a2e',
                'cardbox_flat': '#a97a2e', 'cardbox_small': '#a97a2e',
                'concrete_block_a': '#6b7280', 'concrete_block_b': '#6b7280',
                'dumpster_large': '#1d4ed8', 'dumpster_small': '#1d4ed8',
                'trashcan': '#16a34a', 'firehydrant': '#dc2626',
                'railing': '#fbbf24', 'bench': '#7c3aed',
            }
            for p in _nav.get('props', []):
                _, r_m = _PA.get(p['kind'], (None, 0.5))
                col = _PROP_COLOR.get(p['kind'], '#888')
                ax.add_patch(Circle((p['x'], p['y']), r_m, facecolor=col,
                                    edgecolor='#000', linewidth=0.6, alpha=0.9,
                                    zorder=6))
                ax.text(p['x'], p['y'] + r_m + 0.6, p['kind'], ha='center',
                        fontsize=7, color='#0f172a', zorder=7)
        except Exception as e:
            print(f"  [warn] nav obstacles: {e}")

    # USD obstacles (rocks/trees from /World/Rocks etc.) not in gazebo_models
    _usd_file = '/tmp/usd_obstacles.json'
    if os.path.exists(_usd_file):
        try:
            with open(_usd_file) as f:
                usd_obs = json.load(f)
            for m in usd_obs:
                if m.get('type') == 'rock':
                    ax.add_patch(Circle((m['x'], m['y']), 0.8,
                                        facecolor='#6b6b6b', edgecolor='#3a3a3a',
                                        linewidth=0.4, alpha=0.7, zorder=2))
        except Exception as e:
            # print(f"DEBUG len(traj)={len(traj)}")
            print(f"  [warn] usd obstacles: {e}")

    # Waypoints
    if with_waypoints and os.path.exists(anchors_json):
        try:
            wps = _load_waypoints(anchors_json)
            wpx = [w['x'] for w in wps]
            wpy = [w['y'] for w in wps]
            ax.scatter(wpx, wpy, c='magenta', s=35, marker='o',
                       edgecolors='black', linewidths=0.6,
                       alpha=0.9, zorder=5, label=f'Waypoints (×{len(wps)})')
        except Exception as e:
            # print("DEBUG: isaac sim step")
            print(f"  [warn] waypoints: {e}")

    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.2, zorder=2, color='white')
    ax.set_aspect('equal')
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)

    # Scene legend handles
    scene_handles = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#1e4d1e',
               markersize=12, linestyle='none', label='Tree (r=1.5m)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#3a6b2e',
               markersize=5, linestyle='none', label='Shrub (r=0.4m)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#6b6b6b',
               markersize=7, linestyle='none', label='Rock (r=0.8m)'),
        Patch(facecolor='#8b5a2b', edgecolor='#3a2810', label='House (6×6m)'),
    ]
    if with_obstacles:
        scene_handles += [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff6600',
                   markersize=5, linestyle='none', label='Cone (barrier)'),
            Patch(facecolor='#2d5a2d', edgecolor='#0d2d0d', label='Tent (2×1.8m)'),
        ]

    handles, _ = ax.get_legend_handles_labels()
    handles = handles + scene_handles
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.0, -0.08),
              fontsize=9, frameon=True, framealpha=0.95, ncol=2)

    if metrics_lines:
        fig.text(0.98, 0.02, "\n".join(metrics_lines),
                 fontsize=9, family='monospace',
                 verticalalignment='bottom', horizontalalignment='right',
                 bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                           edgecolor='gray', alpha=0.95))

    plt.tight_layout()
    plt.savefig(output, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  saved {output}")
