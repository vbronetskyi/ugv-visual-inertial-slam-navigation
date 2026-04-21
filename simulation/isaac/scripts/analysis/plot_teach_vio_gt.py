#!/usr/bin/env python3
"""For each route, build a VIO-vs-GT teach plot using the canonical
plot_trajectory_map template.

Reads teach_outputs/vio_pose_dense.csv, computes Procrustes 2D alignment
(with reflection) VIO -> GT, saves vio_vs_gt.png into the route's
teach_outputs dir.
"""
import csv, math, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

ROUTES = ['04_nw_se', '05_ne_sw', '06_nw_ne', '07_se_sw', '08_nw_sw', '09_se_ne']
MAP_XY = {
    '04_nw_se':    ((-100, 75), (-45, 42)),
    '05_ne_sw':    ((-100, 75), (-45, 42)),
    '06_nw_ne':    ((-100, 75), (-45, 42)),
    '07_se_sw':    ((-100, 75), (-45, 42)),
    '08_nw_sw':    ((-110, -70), (-45, 42)),
    '09_se_ne':    ((50, 80), (-45, 42)),
}


def align_vio_to_gt(vio_xyz, gt_xy):
    """Procrustes 2D alignment VIO -> GT with 4 reflection candidates."""
    variances = np.var(vio_xyz, axis=0)
    h0, h1 = np.argsort(variances)[::-1][:2]
    xv_base = vio_xyz[:, h0]
    yv_base = vio_xyz[:, h1]
    xg, yg = gt_xy[:, 0], gt_xy[:, 1]
    cx_g, cy_g = xg.mean(), yg.mean()
    dxg, dyg = xg - cx_g, yg - cy_g
    best = None
    for fx, fy in [(1, 1), (-1, 1), (1, -1), (-1, -1)]:
        xv = xv_base * fx
        yv = yv_base * fy
        cx_v, cy_v = xv.mean(), yv.mean()
        dxv, dyv = xv - cx_v, yv - cy_v
        a = (dxv * dxg + dyv * dyg).sum()
        b = (dxv * dyg - dyv * dxg).sum()
        theta = math.atan2(b, a)
        c, s = math.cos(theta), math.sin(theta)
        rvx = c * dxv - s * dyv + cx_g
        rvy = s * dxv + c * dyv + cy_g
        err = np.hypot(rvx - xg, rvy - yg)
        if best is None or err.mean() < best[0]:
            best = (err.mean(), err, rvx, rvy, (fx, fy))
    return best  # (mean_err, err, rvx, rvy, flip)


def run_route(name):
    base = Path(f'/root/isaac_tr_datasets/{name}/teach/teach_outputs')
    csv_path = base / 'vio_pose_dense.csv'
    if not csv_path.exists():
        print(f'[{name}] missing {csv_path}')
        return
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        print(f'[{name}] empty csv')
        return
    arr = np.array([[float(r['vio_x']), float(r['vio_y']), float(r['vio_z']),
                     float(r['gt_x']),  float(r['gt_y'])] for r in rows])
    vio = arr[:, 0:3]
    gt  = arr[:, 3:5]
    mean_err, err, rvx, rvy, flip = align_vio_to_gt(vio, gt)
    gt_path_m = float(np.sum(np.hypot(np.diff(gt[:, 0]), np.diff(gt[:, 1]))))

    # Emit aligned CSVs for the plotter
    aligned_gt = base / 'traj_gt_world.csv'
    aligned_vio = base / 'traj_vio_world.csv'
    with open(aligned_gt, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['x', 'y'])
        for x, y in gt: w.writerow([x, y])
    with open(aligned_vio, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['x', 'y'])
        for x, y in zip(rvx, rvy): w.writerow([x, y])

    out_png = base / 'vio_vs_gt.png'
    xlim, ylim = MAP_XY[name]
    plot_trajectory_map(
        trajectories=[
            {'csv': str(aligned_gt),  'label': 'GT (Isaac, dense)',
             'color': '#2ca02c', 'x_col': 'x', 'y_col': 'y', 'linewidth': 2.2},
            {'csv': str(aligned_vio),
             'label': f'VIO (Procrustes+refl)  drift_max={err.max():.2f} m  mean={err.mean():.2f} m',
             'color': '#d62728', 'x_col': 'x', 'y_col': 'y',
             'linewidth': 1.4, 'linestyle': '--'},
        ],
        output=str(out_png),
        title=f'{name} - teach: GT vs VIO',
        metrics_lines=[
            f'gt_path = {gt_path_m:.1f} m   drift_max = {err.max():.3f} m   drift_mean = {err.mean():.3f} m',
            f'{len(rows)} samples   flip = {flip}',
        ],
        with_obstacles=False,
        with_waypoints=False,
        route='none',
        xlim=xlim,
        ylim=ylim,
    )
    print(f'[{name}] mean_err={err.mean():.3f} m  max={err.max():.3f} m  -> {out_png}')


if __name__ == '__main__':
    only = sys.argv[1:] or ROUTES
    for r in only:
        run_route(r)
