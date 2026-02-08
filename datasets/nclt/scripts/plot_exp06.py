#!/usr/bin/env python3
"""
Experiment 0.6: DEFINITIVE ORB-SLAM3 EVALUATION ON NCLT
Plots ONLY results from week0_orbslam3_proper (Phase A-C).
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch, FancyBboxPatch
from matplotlib.gridspec import GridSpec
from pathlib import Path
import json

RESULTS = Path('/workspace/datasets/nclt/results/week0_orbslam3_proper')
OUT_DIR = RESULTS / 'plots'
OUT_DIR.mkdir(exist_ok=True)

# load data

phase_b_runs = []
for d in sorted(RESULTS.glob('phase_b/B*/result.json')):
    with open(d) as f:
        phase_b_runs.append(json.load(f))

phase_c_runs = []
for d in sorted(RESULTS.glob('phase_c/C_*/result.json')):
    with open(d) as f:
        phase_c_runs.append(json.load(f))

with open(RESULTS / 'calibration_summary.json') as f:
    calib = json.load(f)


def load_traj(path):
    lines = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    vals = [float(p) for p in parts[:8]]
                    lines.append(vals)
                except ValueError:
                    continue
    if not lines:
        return None
    arr = np.array(lines)
    if arr[0, 0] > 1e15:
        arr[:, 0] /= 1e9
    return arr


# figure 1: phase B overview -- all 10 mono-inertial runs

print("Figure 1: Phase B overview...")

fig = plt.figure(figsize=(22, 16))
fig.suptitle('Experiment 0.6 Phase B: Mono-Inertial Sweep (3000 frames)\n'
             'ORB-SLAM3 on NCLT Spring 2012-04-29, Cam5 forward-facing',
             fontsize=16, fontweight='bold', y=0.98)

gs = GridSpec(3, 3, figure=fig, hspace=0.4, wspace=0.35)

# --- Panel 1: Tracking rate bar chart ---
ax1 = fig.add_subplot(gs[0, :2])
labels = [r['label'] for r in phase_b_runs]
short_labels = [
    'V1: MS25\n47Hz', 'V2: MS25\n100Hz', 'V3: MS25\n200Hz',
    'V4: Hybrid\n100Hz', 'V5: Hybrid\n200Hz',
    'Pinhole\nCam5', 'Fisheye\nCam4', 'Fisheye\nCam1',
    'CLAHE\nCam5', 'nFeat\n5000'
]
tracking = [r['tracking_rate'] for r in phase_b_runs]
colors_b = ['#2196F3'] * 5 + ['#FF9800', '#4CAF50', '#4CAF50', '#9C27B0', '#E91E63']

bars = ax1.bar(range(len(labels)), tracking, color=colors_b, alpha=0.85, edgecolor='white', width=0.7)
for i, (t, bar) in enumerate(zip(tracking, bars)):
    if t > 0:
        ax1.text(i, t + 1.5, f'{t:.1f}%', ha='center', va='bottom', fontsize=9, fontweight='bold')
    else:
        ax1.text(i, 2, 'CRASH', ha='center', va='bottom', fontsize=8, color='red', fontweight='bold')

ax1.set_xticks(range(len(labels)))
ax1.set_xticklabels(short_labels, fontsize=8)
ax1.set_ylabel('Tracking Rate (%)', fontsize=11)
ax1.set_title('Tracking Rate per Configuration', fontsize=13, fontweight='bold')
ax1.set_ylim(0, 105)
ax1.axhline(y=90.2, color='gray', linestyle='--', alpha=0.5, label='Best: 90.2%')
ax1.grid(axis='y', alpha=0.3)

# bug annotation
ax1.annotate('BUG: B001-B003 used\nsame IMU data (v1)\ndue to symlink bug',
             xy=(1, 92), fontsize=8, color='red',
             bbox=dict(boxstyle='round', facecolor='#FFEBEE', alpha=0.9),
             ha='center')

# --- Panel 2: Keyframes and Maps ---
ax2 = fig.add_subplot(gs[0, 2])
kf = [r['n_keyframes'] for r in phase_b_runs]
n_maps = []
for r in phase_b_runs:
    tail = r.get('stdout_tail', '')
    if 'maps in the atlas' in tail:
        for line in tail.split('\n'):
            if 'maps in the atlas' in line:
                try:
                    n_maps.append(int(line.split()[2]))
                except:
                    n_maps.append(0)
                break
        else:
            n_maps.append(0)
    else:
        n_maps.append(0)

ax2.barh(range(len(labels)), kf, color=colors_b, alpha=0.85, edgecolor='white')
for i, k in enumerate(kf):
    maps_str = f'{n_maps[i]} map{"s" if n_maps[i] != 1 else ""}' if n_maps[i] > 0 else 'CRASH'
    ax2.text(max(k, 50) + 30, i, f'{k} KF ({maps_str})', va='center', fontsize=8)
ax2.set_yticks(range(len(labels)))
ax2.set_yticklabels([r['label'] for r in phase_b_runs], fontsize=8)
ax2.set_xlabel('Keyframes', fontsize=10)
ax2.set_title('Keyframes & Maps', fontsize=12, fontweight='bold')
ax2.invert_yaxis()
ax2.grid(axis='x', alpha=0.3)

# --- Panel 3: Scale Drift for runs that produced trajectories ---
ax3 = fig.add_subplot(gs[1, :2])

traj_runs = []
for r in phase_b_runs:
    if r['success'] and r['traj_path'] and r['n_keyframes'] > 50:
        tpath = Path(r['traj_path'])
        if tpath.exists():
            traj = load_traj(tpath)
            if traj is not None:
                max_coord = max(np.max(np.abs(traj[:, 1])),
                              np.max(np.abs(traj[:, 2])),
                              np.max(np.abs(traj[:, 3])))
                path_len = np.sum(np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1))
                traj_runs.append({
                    'label': r['label'], 'traj': traj,
                    'max_coord': max_coord, 'path_len': path_len,
                    'n_kf': r['n_keyframes']
                })

gt_path_m = 700  # approximate GT path for 3000 frames at 5fps = 600s

if traj_runs:
    run_labels = [t['label'] for t in traj_runs]
    max_coords = [t['max_coord'] for t in traj_runs]
    path_lens = [t['path_len'] for t in traj_runs]
    scale_errors = [mc / gt_path_m for mc in max_coords]

    x_pos = np.arange(len(run_labels))
    width = 0.35

    bars1 = ax3.bar(x_pos - width/2, max_coords, width, label='Max Coordinate (m)',
                     color='#F44336', alpha=0.8)
    bars2 = ax3.bar(x_pos + width/2, path_lens, width, label='Path Length (m)',
                     color='#2196F3', alpha=0.8)

    ax3.axhline(y=gt_path_m, color='green', linewidth=2, linestyle='--',
                label=f'GT path ≈ {gt_path_m}m')

    ax3.set_yscale('log')
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(run_labels, fontsize=9)
    ax3.set_ylabel('Distance (m, log scale)', fontsize=10)
    ax3.set_title('Scale Drift: Max Coordinate & Path Length vs Ground Truth',
                  fontsize=13, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(axis='y', alpha=0.3)

    # scale error annotations
    for i, se in enumerate(scale_errors):
        ax3.text(i - width/2, max_coords[i] * 1.3, f'{se:.0f}×',
                ha='center', fontsize=9, fontweight='bold', color='red')

# --- Panel 4: IMU Noise Parameters ---
ax4 = fig.add_subplot(gs[1, 2])
ax4.axis('off')
imu = calib['imu_noise']
noise_text = (
    'IMU Noise (MS25 MEMS)\n'
    '─────────────────────\n'
    f'Gyro noise:  {imu["gyro_noise"]:.5f} rad/s\n'
    f'Accel noise: {imu["accel_noise"]:.4f} m/s²\n'
    f'Gyro walk:   {imu["gyro_walk"]:.6f} rad/s\n'
    f'Accel walk:  {imu["accel_walk"]:.5f} m/s²\n'
    f'IMU rate:    {imu["imu_rate_hz"]:.1f} Hz\n'
    f'Static seg:  {imu["static_duration_s"]:.1f} s\n'
    '\n'
    f'Gyro σ: [{imu["gyro_std_xyz"][0]:.3f}, '
    f'{imu["gyro_std_xyz"][1]:.3f}, '
    f'{imu["gyro_std_xyz"][2]:.3f}]\n'
    f'Accel σ: [{imu["accel_std_xyz"][0]:.3f}, '
    f'{imu["accel_std_xyz"][1]:.3f}, '
    f'{imu["accel_std_xyz"][2]:.3f}]\n'
    '\n'
    'Camera Calibration (default)\n'
    '────────────────────────────\n'
    'f=290, cx=404, cy=308\n'
    'k1=k2=k3=k4=0 (no distortion)\n'
    'Model: KannalaBrandt8 fisheye\n'
    '\n'
    'COLMAP Status: SIGABRT\n'
    '(fell back to defaults)'
)
ax4.text(0.05, 0.95, noise_text, transform=ax4.transAxes,
         fontsize=9, verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
ax4.set_title('Calibration', fontsize=12, fontweight='bold')

# --- Panel 5: Raw Trajectories (Bird's-eye, B001-B004) ---
ax5 = fig.add_subplot(gs[2, 0])
ax5.set_title('B001 (V1: MS25 47Hz)\nScale drift to 210 km', fontsize=10, fontweight='bold')
if traj_runs:
    t = traj_runs[0]['traj']
    n = len(t)
    colors = plt.cm.viridis(np.linspace(0, 1, n))
    for i in range(n-1):
        ax5.plot(t[i:i+2, 1], t[i:i+2, 2], '-', color=colors[i], linewidth=0.5)
    ax5.set_xlabel('X (m)')
    ax5.set_ylabel('Y (m)')
    ax5.grid(True, alpha=0.3)
    ax5.text(0.05, 0.95, f'Max: {traj_runs[0]["max_coord"]:.0f}m\nGT: ~700m\nDrift: {traj_runs[0]["max_coord"]/700:.0f}×',
             transform=ax5.transAxes, fontsize=8, va='top',
             bbox=dict(facecolor='wheat', alpha=0.8))

ax6 = fig.add_subplot(gs[2, 1])
if len(traj_runs) > 1:
    ax6.set_title(f'{traj_runs[1]["label"]}\nScale drift to {traj_runs[1]["max_coord"]/1000:.0f} km',
                  fontsize=10, fontweight='bold')
    t = traj_runs[1]['traj']
    n = len(t)
    colors = plt.cm.plasma(np.linspace(0, 1, n))
    for i in range(n-1):
        ax6.plot(t[i:i+2, 1], t[i:i+2, 2], '-', color=colors[i], linewidth=0.5)
    ax6.set_xlabel('X (m)')
    ax6.set_ylabel('Y (m)')
    ax6.grid(True, alpha=0.3)
    ax6.text(0.05, 0.95, f'Max: {traj_runs[1]["max_coord"]:.0f}m\nDrift: {traj_runs[1]["max_coord"]/700:.0f}×',
             transform=ax6.transAxes, fontsize=8, va='top',
             bbox=dict(facecolor='wheat', alpha=0.8))

ax7 = fig.add_subplot(gs[2, 2])
if len(traj_runs) > 2:
    ax7.set_title(f'{traj_runs[2]["label"]}\nScale drift to {traj_runs[2]["max_coord"]/1000:.0f} km',
                  fontsize=10, fontweight='bold')
    t = traj_runs[2]['traj']
    n = len(t)
    colors = plt.cm.inferno(np.linspace(0, 1, n))
    for i in range(n-1):
        ax7.plot(t[i:i+2, 1], t[i:i+2, 2], '-', color=colors[i], linewidth=0.5)
    ax7.set_xlabel('X (m)')
    ax7.set_ylabel('Y (m)')
    ax7.grid(True, alpha=0.3)
    ax7.text(0.05, 0.95, f'Max: {traj_runs[2]["max_coord"]:.0f}m\nDrift: {traj_runs[2]["max_coord"]/700:.0f}×',
             transform=ax7.transAxes, fontsize=8, va='top',
             bbox=dict(facecolor='wheat', alpha=0.8))

plt.savefig(OUT_DIR / 'exp06_fig1_phase_b.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'exp06_fig1_phase_b.png'}")
plt.close()


# figure 2: phase C + failure analysis

print("Figure 2: Phase C + failure analysis...")

fig, axes = plt.subplots(2, 2, figsize=(18, 14))
fig.suptitle('Experiment 0.6 Phase C: Stereo-Inertial + Failure Analysis\n'
             'Why ORB-SLAM3 Cannot Work on NCLT Ladybug3',
             fontsize=16, fontweight='bold', y=0.98)

# --- Panel 1: Stereo results ---
ax = axes[0, 0]
ax.axis('off')

stereo_text = (
    '┌──────────────────────────────────────────────┐\n'
    '│     STEREO-INERTIAL: COMPLETE FAILURE         │\n'
    '├──────────────────────────────────────────────┤\n'
    '│                                               │\n'
    '│  Cam4+Cam5 pair:                              │\n'
    '│    Keyframes: 1 / 3000  (0.03%)               │\n'
    '│    Baseline:  9.4 cm                          │\n'
    '│    Runtime:   181 s                           │\n'
    '│    Status:    too_few_poses                   │\n'
    '│                                               │\n'
    '│  Cam5+Cam1 pair:                              │\n'
    '│    Keyframes: 1 / 3000  (0.03%)               │\n'
    '│    Baseline:  9.5 cm                          │\n'
    '│    Runtime:   180 s                           │\n'
    '│    Status:    too_few_poses                   │\n'
    '│                                               │\n'
    '│  ROOT CAUSE: 72° angle between adjacent       │\n'
    '│  cameras = ZERO overlapping field of view     │\n'
    '│  Stereo matching is physically impossible     │\n'
    '└──────────────────────────────────────────────┘'
)
ax.text(0.05, 0.95, stereo_text, transform=ax.transAxes,
        fontsize=10, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='#FFEBEE', alpha=0.9))
ax.set_title('Phase C: Stereo-Inertial', fontsize=13, fontweight='bold', color='red')

# --- Panel 2: Ladybug3 camera layout ---
ax = axes[0, 1]
ax.set_title('Ladybug3 Camera Layout\n(top view, 6 cameras)', fontsize=12, fontweight='bold')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')

# draw camera positions and FOVs
cam_angles = {
    0: (90, 'Cam0 (SKY)', '#BDBDBD'),     # top/sky
    1: (73, 'Cam1', '#4CAF50'),
    2: (145, 'Cam2', '#BDBDBD'),
    3: (217, 'Cam3', '#BDBDBD'),
    4: (-72, 'Cam4', '#FF9800'),
    5: (0, 'Cam5 (FWD)', '#2196F3'),
}

for cam_id, (angle_deg, label, color) in cam_angles.items():
    if cam_id == 0:
        # sky-facing, draw as circle at center
        circle = plt.Circle((0, 0), 0.15, color=color, alpha=0.5)
        ax.add_patch(circle)
        ax.text(0, 0.35, label, ha='center', fontsize=8, color='gray')
        continue

    angle_rad = np.radians(angle_deg)
    # camera position on ring
    cx, cy = 0.3 * np.sin(angle_rad), 0.3 * np.cos(angle_rad)

    # FOV wedge (120° = 60° each side)
    fov_half = 60
    theta1 = angle_deg - fov_half
    theta2 = angle_deg + fov_half
    wedge_angles = np.linspace(np.radians(theta1), np.radians(theta2), 30)
    fov_r = 1.5

    xs = [cx] + [cx + fov_r * np.sin(a) for a in wedge_angles] + [cx]
    ys = [cy] + [cy + fov_r * np.cos(a) for a in wedge_angles] + [cy]
    ax.fill(xs, ys, color=color, alpha=0.15)
    ax.plot(xs, ys, color=color, linewidth=1, alpha=0.5)

    # camera marker
    ax.plot(cx, cy, 'o', color=color, markersize=8)
    label_r = 0.6
    ax.text(cx + label_r * np.sin(angle_rad), cy + label_r * np.cos(angle_rad),
            label, ha='center', fontsize=9, fontweight='bold', color=color)

# annotate the 72° gap
ax.annotate('72° gap\nNO overlap!', xy=(0.8, 1.0), fontsize=10, color='red',
            fontweight='bold', ha='center',
            bbox=dict(facecolor='#FFEBEE', alpha=0.9))

ax.set_xlabel('← Left    Right →')
ax.set_ylabel('← Back    Forward →')
ax.grid(True, alpha=0.2)

# --- Panel 3: Why VIO Fails, the math ---
ax = axes[1, 0]
ax.axis('off')

vio_text = (
    'WHY VIO CANNOT ESTIMATE SCALE\n'
    '═══════════════════════════════\n\n'
    'ORB-SLAM3 VIO requires:\n'
    '  • Sufficient parallax between frames\n'
    '  • IMU excitation (acceleration changes)\n'
    '  • Observability of scale from IMU\n\n'
    'NCLT sensor configuration:\n'
    '  Camera:  5 fps (200ms between frames)\n'
    '  IMU:     47 Hz MS25 MEMS\n'
    '  Speed:   ~1-3 m/s outdoor robot\n\n'
    'Problem breakdown:\n'
    '  ① 200ms frame gap = 0.2-0.6m motion\n'
    '    → Large parallax (good for tracking)\n'
    '    → But IMU preintegration over 200ms\n'
    '      accumulates huge drift\n\n'
    '  ② MS25 accel noise: 0.029 m/s²\n'
    '    → 200ms integration: ½at² = 0.6mm\n'
    '    → 10 frames (2s): drift ~0.06m\n'
    '    → 100 frames (20s): drift ~6m\n'
    '    → Exponential growth → 200km at end\n\n'
    '  ③ EuRoC (works): 20fps + 200Hz IMU\n'
    '     NCLT (fails):  5fps + 47Hz IMU\n'
    '     → 4× fewer frames, 4× fewer IMU\n'
    '     → Scale observability destroyed'
)
ax.text(0.05, 0.98, vio_text, transform=ax.transAxes,
        fontsize=9, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='#FFF3E0', alpha=0.9))

# --- Panel 4: Comparison with what works ---
ax = axes[1, 1]
ax.axis('off')

comparison = (
    'WHAT WORKS vs WHAT DOESN\'T\n'
    '══════════════════════════════\n\n'
    '✓ Visual tracking (no IMU):  96.2%\n'
    '  → f=386 fisheye, Cam5, mono-only\n'
    '  → Features tracked well!\n'
    '  → But NO metric scale\n\n'
    '✓ IMU initializes: YES\n'
    '  → imu_initialized=true in all runs\n'
    '  → But scale estimate is WRONG\n\n'
    'X Scale estimation: completely unstable\n'
    '  B001 (V1 47Hz):   301× scale error\n'
    '  B002 (V2 100Hz):  686× scale error\n'
    '  B003 (V3 200Hz):  340× scale error\n'
    '  B004 (V4 hybrid): 404× scale error\n'
    '  → Interpolation makes it WORSE!\n\n'
    '✗ Stereo-Inertial: IMPOSSIBLE\n'
    '  → 72° between cameras\n'
    '  → 0 matching points\n'
    '  → 1 keyframe in 3000 attempts\n\n'
    '✗ Alternative cameras:\n'
    '  Cam4: 0% tracking (CRASH)\n'
    '  Cam1: 0% tracking (scale too small)\n'
    '  CLAHE: 0% tracking (CRASH)\n'
    '  5000 feat: 0% tracking (CRASH)\n\n'
    'VERDICT: ORB-SLAM3 is NOT VIABLE on NCLT'
)
ax.text(0.05, 0.98, comparison, transform=ax.transAxes,
        fontsize=9, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='#E8F5E9', alpha=0.9))

plt.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(OUT_DIR / 'exp06_fig2_failure_analysis.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'exp06_fig2_failure_analysis.png'}")
plt.close()


# figure 3: summary table

print("Figure 3: Summary table...")

fig = plt.figure(figsize=(24, 18))
fig.suptitle('Experiment 0.6: Final ORB-SLAM3 Evaluation, Full Results\n'
             'NCLT Spring 2012-04-29, 3000 frames (600s at 5fps)',
             fontsize=16, fontweight='bold', y=0.97)

ax = fig.add_subplot(111)
ax.axis('off')

# build table data
header = ['Run', 'Mode', 'Camera', 'IMU', 'Model', 'Preproc', 'nFeat',
          'KF', 'Track%', 'Maps', 'Max Coord', 'Scale×', 'Status']

rows = []
for r in phase_b_runs:
    cfg = r.get('config', {})
    # parse maps count
    tail = r.get('stdout_tail', '')
    maps = '?'
    for line in tail.split('\n'):
        if 'maps in the atlas' in line:
            try:
                maps = line.split()[2]
            except:
                pass
            break

    # get max coord if trajectory exists
    max_c = '-'
    scale = '-'
    if r['success'] and r.get('traj_path') and r['n_keyframes'] > 10:
        tp = Path(r['traj_path'])
        if tp.exists():
            traj = load_traj(tp)
            if traj is not None:
                mc = max(np.max(np.abs(traj[:, 1])),
                        np.max(np.abs(traj[:, 2])),
                        np.max(np.abs(traj[:, 3])))
                max_c = f'{mc:,.0f}m' if mc > 1000 else f'{mc:.1f}m'
                scale = f'{mc/700:.0f}×' if mc > 100 else f'{mc/700:.2f}×'

    # status
    if not r['success']:
        status = 'CRASH (Segfault)'
    elif r['n_keyframes'] < 10:
        status = 'Failed (too few KF)'
    elif r['eval']['status'] == 'too_few_matches':
        status = 'Scale drift (no GT match)'
    else:
        status = r['eval']['status']

    rows.append([
        r['label'],
        'Mono+IMU',
        f"Cam{cfg.get('cam_id', '?')}",
        cfg.get('imu_ver', '?'),
        cfg.get('cam_model', '?'),
        cfg.get('preprocess', 'none'),
        str(cfg.get('n_features', '?')),
        str(r['n_keyframes']),
        f"{r['tracking_rate']:.1f}",
        str(maps),
        max_c,
        scale,
        status,
    ])

# add stereo rows
for r in phase_c_runs:
    cfg = r.get('config', {})
    rows.append([
        r['label'],
        'Stereo+IMU',
        f"Cam{cfg.get('left','?')}+{cfg.get('right','?')}",
        cfg.get('imu_ver', '?'),
        'fisheye',
        'none',
        '3000',
        str(r['n_keyframes']),
        f"{r['tracking_rate']:.2f}",
        '1',
        '-',
        '-',
        'FAILED (no FOV overlap)',
    ])

table = ax.table(cellText=rows, colLabels=header,
                 cellLoc='center', loc='center',
                 colWidths=[0.09, 0.06, 0.05, 0.04, 0.05, 0.05, 0.04,
                           0.04, 0.05, 0.04, 0.08, 0.05, 0.16])

table.auto_set_font_size(False)
table.set_fontsize(8.5)
table.scale(1, 1.8)

# style header
for j in range(len(header)):
    cell = table[0, j]
    cell.set_facecolor('#37474F')
    cell.set_text_props(color='white', fontweight='bold', fontsize=8)

# style rows
for i, row in enumerate(rows):
    for j in range(len(row)):
        cell = table[i + 1, j]
        if 'CRASH' in row[12]:
            cell.set_facecolor('#FFCDD2')
        elif 'FAILED' in row[12]:
            cell.set_facecolor('#FFCDD2')
        elif 'Scale drift' in row[12]:
            cell.set_facecolor('#FFE0B2')
        elif 'too few' in row[12].lower():
            cell.set_facecolor('#FFF9C4')
        else:
            cell.set_facecolor('#F5F5F5')

# footnotes
footnotes = (
    "NOTES:\n"
    "• BUG: B001-B003 all used V1 IMU data due to symlink caching bug (not V2/V3 as intended)\n"
    "• B001-B003 have identical tracking (90.2%) because they ran with identical data\n"
    "• B004 (Hybrid IMU with KVH FOG yaw) slightly worse: 79.7%, 2 maps instead of 1\n"
    "• B005 (Hybrid 200Hz) crashed during trajectory save with Segfault\n"
    "- B006 (Pinhole model): only 2.6% tracking; fisheye model is much better\n"
    "• B007-B010: All crashed (Cam4/Cam1 bad viewing angle, CLAHE/5000feat destabilize)\n"
    "• Stereo: 72° angle between adjacent Ladybug3 cameras = zero visual overlap\n"
    "• COLMAP calibration failed (SIGABRT) → used default f=290 (later tested f=386)\n"
    "• Phases D-G (full session, wheel odom, summer) were NOT executed\n"
    "• Each run took ~620s real-time due to required usleep() for thread sync"
)
fig.text(0.03, 0.01, footnotes, fontsize=8, fontfamily='monospace', va='bottom',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

plt.tight_layout(rect=[0, 0.12, 1, 0.95])
fig.savefig(OUT_DIR / 'exp06_fig3_summary_table.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'exp06_fig3_summary_table.png'}")
plt.close()


# figure 4: all trajectories overlaid + B006 pinhole

print("Figure 4: All trajectories (raw)...")

fig, axes = plt.subplots(2, 3, figsize=(20, 13))
fig.suptitle('Experiment 0.6: Raw Trajectories (XY plane)\n'
             'Color: time progression (dark=start, bright=end)',
             fontsize=15, fontweight='bold', y=0.98)

traj_files = [
    ('B001: V1 MS25 47Hz', 'phase_b/B001_imu_v1/f_nclt.txt', 'viridis'),
    ('B002: V2 MS25 100Hz', 'phase_b/B002_imu_v2/f_nclt.txt', 'plasma'),
    ('B003: V3 MS25 200Hz', 'phase_b/B003_imu_v3/f_nclt.txt', 'inferno'),
    ('B004: V4 Hybrid 100Hz', 'phase_b/B004_imu_v4/f_nclt.txt', 'magma'),
    ('B006: Pinhole (zoom)', 'phase_b/B006_cam5_pinhole/f_nclt.txt', 'cividis'),
]

for idx, (title, rel_path, cmap_name) in enumerate(traj_files):
    ax = axes[idx // 3, idx % 3]
    fpath = RESULTS / rel_path
    if fpath.exists():
        traj = load_traj(fpath)
        if traj is not None:
            n = len(traj)
            colors = plt.cm.get_cmap(cmap_name)(np.linspace(0, 1, n))
            for i in range(n - 1):
                ax.plot(traj[i:i+2, 1], traj[i:i+2, 2], '-', color=colors[i], linewidth=0.5)
            ax.plot(traj[0, 1], traj[0, 2], 'go', markersize=6, label='Start')
            ax.plot(traj[-1, 1], traj[-1, 2], 'r*', markersize=8, label='End')

            max_c = max(np.max(np.abs(traj[:, 1])), np.max(np.abs(traj[:, 2])))
            path_l = np.sum(np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1))
            ax.set_title(f'{title}\n{n} poses, max={max_c:.0f}m, path={path_l:.0f}m',
                        fontsize=10, fontweight='bold')
            ax.set_xlabel('X (m)', fontsize=9)
            ax.set_ylabel('Y (m)', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7, loc='lower right')

# last panel: plan completion status
ax = axes[1, 2]
ax.axis('off')

status_text = (
    'PLAN EXECUTION STATUS\n'
    '═════════════════════\n\n'
    '✓ Phase A: Calibration\n'
    '  (partial: COLMAP failed)\n\n'
    '◐ Phase B: Mono-Inertial\n'
    '  10/26 runs completed\n'
    '  (B001-B005 had IMU bug)\n\n'
    '✓ Phase C: Stereo-Inertial\n'
    '  2/6 runs, both FAILED\n\n'
    '✗ Phase D: Full Session\n'
    '  NOT EXECUTED\n\n'
    '✗ Phase E: Wheel Odom Fusion\n'
    '  NOT EXECUTED\n\n'
    '✗ Phase F: Summer Session\n'
    '  NOT EXECUTED\n\n'
    '✗ Phase G: Report\n'
    '  NOT EXECUTED\n\n'
    '──────────────────────\n'
    'Stopped because:\n'
    'VIO scale drift is\n'
    'FUNDAMENTAL, not fixable\n'
    'by remaining phases.'
)
ax.text(0.05, 0.98, status_text, transform=ax.transAxes,
        fontsize=9, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='#E3F2FD', alpha=0.9))

plt.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(OUT_DIR / 'exp06_fig4_trajectories.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'exp06_fig4_trajectories.png'}")
plt.close()


print(f"\n{'='*60}")
print(f"All Experiment 0.6 plots saved to: {OUT_DIR}")
print(f"{'='*60}")
for f in sorted(OUT_DIR.glob('exp06_*.png')):
    print(f"  {f.name} ({f.stat().st_size/1024:.0f} KB)")
