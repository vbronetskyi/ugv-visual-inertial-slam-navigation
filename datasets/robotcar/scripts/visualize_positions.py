#!/usr/bin/env python3
"""Position visualization for RobotCar Seasons localization results"""

import sys
import numpy as np
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.lines import Line2D
import matplotlib.patheffects as pe
from scipy.spatial.transform import Rotation

sys.path.insert(0, '/workspace/third_party/hloc')

# paths
DATASET = Path('/workspace/data/robotcar_seasons')
OUTPUTS = DATASET / 'outputs'
IMAGES = DATASET / 'images'
RESULTS_DIR = Path('/workspace/datasets/robotcar/results/robotcar_seasons_hloc')
PLOTS_DIR = RESULTS_DIR / 'plots'
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

TRAIN_FILE = DATASET / 'robotcar_v2_train.txt'

CONDITIONS = ['dawn', 'dusk', 'night', 'night-rain', 'overcast-summer',
              'overcast-winter', 'rain', 'snow', 'sun']

CONDITION_COLORS = {
    'dawn':             '#FF9800',
    'dusk':             '#E91E63',
    'night':            '#311B92',
    'night-rain':       '#7B1FA2',
    'overcast-summer':  '#4CAF50',
    'overcast-winter':  '#607D8B',
    'rain':             '#2196F3',
    'snow':             '#00ACC1',
    'sun':              '#FBC02D',
}

BEST_METHOD_RESULTS = OUTPUTS / 'RobotCar_hloc_superpoint+superglue_openibl20.txt'
BEST_METHOD_NAME = 'SP+SG+OpenIBL'

ERROR_CMAP = LinearSegmentedColormap.from_list(
    'error', ['#1B5E20', '#66BB6A', '#FDD835', '#FF7043', '#B71C1C'], N=256)


# data loading
def load_ground_truth():
    """Load GT poses, returns dict: 'camera/timestamp.jpg' -> {condition, pos}"""
    gt = {}
    with open(TRAIN_FILE) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 13:
                continue
            name = parts[0]
            vals = list(map(float, parts[1:]))
            C = np.array([vals[3], vals[7], vals[11]])
            condition = name.split('/')[0]
            key = '/'.join(name.split('/')[1:])
            gt[key] = {'condition': condition, 'pos': C}
    return gt


def load_localization_results(results_file):
    """Load hloc results, returns dict: 'camera/timestamp.jpg' -> {pos, R_w2c}"""
    poses = {}
    with open(results_file) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            name = parts[0]
            qw, qx, qy, qz = map(float, parts[1:5])
            tx, ty, tz = map(float, parts[5:8])
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            t = np.array([tx, ty, tz])
            pos = -R.T @ t
            poses[name] = {'pos': pos, 'R_w2c': R}
    return poses


def load_db_positions():
    """Load database image positions from SfM model"""
    import pycolmap
    model = pycolmap.Reconstruction(str(OUTPUTS / 'sfm_superpoint+superglue'))
    positions = []
    for img_id, img in model.images.items():
        cfw = img.cam_from_world()
        R = cfw.rotation.matrix()
        t = cfw.translation
        pos = -R.T @ t
        positions.append(pos[:2])
    return np.array(positions)


def load_db_positions_dict():
    """Load DB positions as dict for nearest-neighbor lookup"""
    import pycolmap
    model = pycolmap.Reconstruction(str(OUTPUTS / 'sfm_superpoint+superglue'))
    db = {}
    for img_id, img in model.images.items():
        cfw = img.cam_from_world()
        R = cfw.rotation.matrix()
        t = cfw.translation
        pos = -R.T @ t
        db[img.name] = pos
    return db


def compute_errors(gt, loc):
    """Compute translation errors for matched keys"""
    errors = {}
    for key in set(gt.keys()) & set(loc.keys()):
        t_err = np.linalg.norm(loc[key]['pos'] - gt[key]['pos'])
        errors[key] = t_err
    return errors


# plot 1: Bird's eye view - TWO panels
def plot_birds_eye_view(db_pos, gt, loc, errors):
    """Two-panel bird's eye view: left=by condition, right=by error magnitude"""
    print("Generating bird's eye view...")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(28, 14))

    good_keys = [k for k, e in errors.items() if e <= 5.0]
    bad_keys = [k for k, e in errors.items() if e > 5.0]

    # ---- LEFT: colored by condition ----
    ax1.scatter(db_pos[:, 0], db_pos[:, 1], s=3, c='#E0E0E0', alpha=0.4, zorder=1)

    legend_handles = []
    for cond in CONDITIONS:
        cond_keys = [k for k in good_keys if gt[k]['condition'] == cond]
        if not cond_keys:
            continue
        pos = np.array([gt[k]['pos'][:2] for k in cond_keys])
        ax1.scatter(pos[:, 0], pos[:, 1], s=40, c=CONDITION_COLORS[cond],
                    alpha=0.85, edgecolors='white', linewidths=0.3, zorder=3)
        legend_handles.append(Line2D([0], [0], marker='o', color='w', markerfacecolor=CONDITION_COLORS[cond],
                                      markersize=10, label=f'{cond} ({len(cond_keys)})'))

    if bad_keys:
        bad_pos = np.array([gt[k]['pos'][:2] for k in bad_keys])
        ax1.scatter(bad_pos[:, 0], bad_pos[:, 1], s=60, c='red', marker='X',
                    linewidths=0.8, alpha=0.9, zorder=4)
        legend_handles.append(Line2D([0], [0], marker='X', color='w', markerfacecolor='red',
                                      markersize=10, label=f'Failed >5m ({len(bad_keys)})'))

    legend_handles.insert(0, Line2D([0], [0], marker='o', color='w', markerfacecolor='#CCCCCC',
                                     markersize=8, label=f'Database ({len(db_pos)})'))

    ax1.legend(handles=legend_handles, loc='upper left', fontsize=11,
               framealpha=0.9, edgecolor='#999')
    ax1.set_xlabel('X (meters)', fontsize=13)
    ax1.set_ylabel('Y (meters)', fontsize=13)
    ax1.set_title('By Condition', fontsize=16, fontweight='bold')
    ax1.set_aspect('equal')
    ax1.grid(alpha=0.15)

    # ---- RIGHT: colored by error magnitude ----
    ax2.scatter(db_pos[:, 0], db_pos[:, 1], s=3, c='#E0E0E0', alpha=0.4, zorder=1)

    all_keys = sorted(errors.keys(), key=lambda k: errors[k], reverse=True)
    all_pos = np.array([gt[k]['pos'][:2] for k in all_keys])
    all_err = np.array([errors[k] for k in all_keys])

    norm = Normalize(vmin=0, vmax=10)
    sc = ax2.scatter(all_pos[:, 0], all_pos[:, 1], s=40,
                     c=np.clip(all_err, 0, 10), cmap=ERROR_CMAP, norm=norm,
                     edgecolors='white', linewidths=0.3, alpha=0.85, zorder=3)

    cbar = fig.colorbar(sc, ax=ax2, shrink=0.7, pad=0.02)
    cbar.set_label('Translation error (m)', fontsize=12)
    cbar.ax.tick_params(labelsize=11)

    ax2.set_xlabel('X (meters)', fontsize=13)
    ax2.set_ylabel('Y (meters)', fontsize=13)
    ax2.set_title('By Error Magnitude (capped at 10m)', fontsize=16, fontweight='bold')
    ax2.set_aspect('equal')
    ax2.grid(alpha=0.15)

    n_good = len(good_keys)
    n_total = len(errors)
    fig.suptitle(f"Bird's Eye View - {BEST_METHOD_NAME}\n"
                 f"Localized within 5m: {n_good}/{n_total} ({100*n_good/n_total:.1f}%)",
                 fontsize=18, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    out = PLOTS_DIR / 'position_birds_eye.png'
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved {out.name}")


# plot 2: Per-condition positions - larger dots, visible arrows
def plot_per_condition_positions(gt, loc, errors):
    """9 subplots with GT dots, error arrows, clear coloring"""
    print("Generating per-condition position plots...")
    fig, axes = plt.subplots(3, 3, figsize=(24, 22))

    norm = Normalize(vmin=0, vmax=10)

    for idx, cond in enumerate(CONDITIONS):
        ax = axes[idx // 3][idx % 3]

        cond_keys = [k for k in errors if gt[k]['condition'] == cond]
        if not cond_keys:
            ax.set_title(f'{cond} - no data')
            continue

        cond_errors = np.array([errors[k] for k in cond_keys])
        gt_positions = np.array([gt[k]['pos'][:2] for k in cond_keys])
        est_positions = np.array([loc[k]['pos'][:2] for k in cond_keys])

        # sort by error - draw good first, bad on top
        order = np.argsort(cond_errors)

        # Draw error lines: GT -> estimated (only for errors > 0.5m and < 50m for readability)
        for i in order:
            err = cond_errors[i]
            if err < 0.5 or err > 50:
                continue
            gt_p = gt_positions[i]
            est_p = est_positions[i]
            color = ERROR_CMAP(norm(min(err, 10)))
            lw = min(2.0, 0.5 + err / 5.0)
            ax.plot([gt_p[0], est_p[0]], [gt_p[1], est_p[1]],
                    color=color, alpha=0.5, linewidth=lw, zorder=2)

        # gT dots - colored by error, larger
        sc = ax.scatter(gt_positions[order, 0], gt_positions[order, 1],
                        s=35, c=np.clip(cond_errors[order], 0, 10),
                        cmap=ERROR_CMAP, norm=norm,
                        edgecolors='black', linewidths=0.3, zorder=3)

        # mark failures (>5m) with extra ring
        fail_mask = cond_errors > 5.0
        if np.any(fail_mask):
            ax.scatter(gt_positions[fail_mask, 0], gt_positions[fail_mask, 1],
                       s=100, facecolors='none', edgecolors='red', linewidths=1.5, zorder=4)

        n_total = len(cond_keys)
        n_good = int(np.sum(cond_errors <= 5.0))
        pct = 100 * n_good / n_total
        med = np.median(cond_errors)

        # color the title by quality
        title_color = '#1B5E20' if pct >= 90 else ('#E65100' if pct >= 60 else '#B71C1C')
        ax.set_title(f'{cond}  -  {n_good}/{n_total} ({pct:.0f}%)  med={med:.2f}m',
                     fontsize=13, fontweight='bold', color=title_color)
        ax.set_aspect('equal')
        ax.grid(alpha=0.2)
        ax.tick_params(labelsize=9)

    # shared colorbar
    cbar_ax = fig.add_axes([0.93, 0.15, 0.015, 0.7])
    sm = plt.cm.ScalarMappable(cmap=ERROR_CMAP, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, cax=cbar_ax)
    cbar.set_label('Translation error (m)', fontsize=13)
    cbar.ax.tick_params(labelsize=11)

    # legend
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#1B5E20',
               markeredgecolor='black', markersize=10, label='< 1m (excellent)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#FDD835',
               markeredgecolor='black', markersize=10, label='1–5m (acceptable)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#B71C1C',
               markeredgecolor='red', markersize=12, markeredgewidth=1.5, label='> 5m (failed)'),
        Line2D([0], [0], color='#FF7043', linewidth=2, alpha=0.6, label='Error line (GT->est)'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=4,
               fontsize=12, framealpha=0.9, bbox_to_anchor=(0.45, 0.01))

    fig.suptitle(f'Per-Condition Localization - {BEST_METHOD_NAME}\n'
                 f'Each dot = GT position of a query image. Color = localization error. '
                 f'Lines connect GT to estimated position.',
                 fontsize=16, fontweight='bold', y=0.99)
    plt.subplots_adjust(left=0.04, right=0.91, top=0.93, bottom=0.06, hspace=0.25, wspace=0.2)
    out = PLOTS_DIR / 'position_per_condition.png'
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved {out.name}")


# plot 3: Example images - 5 best, 5 worst, larger panels
def plot_example_images(gt, loc, errors):
    """5 best + 5 worst, each row = query | DB match, with clear labels"""
    print("Generating example images grid...")

    # load DB positions for nearest-neighbor
    db_pos_dict = load_db_positions_dict()

    # build KD-tree for fast NN
    db_names = list(db_pos_dict.keys())
    db_coords = np.array([db_pos_dict[n][:2] for n in db_names])

    def find_nearest_db(query_pos):
        dists = np.linalg.norm(db_coords - query_pos[:2], axis=1)
        idx = np.argmin(dists)
        return db_names[idx], dists[idx]

    sorted_keys = sorted(errors.keys(), key=lambda k: errors[k])
    best_keys = sorted_keys[:5]
    worst_keys = sorted_keys[-5:][::-1]

    # layout: 2 sections (best/worst), each 2 rows (query/db) × 5 cols
    fig, axes = plt.subplots(4, 5, figsize=(30, 22))

    def fill_row(keys, row_query, row_db, section_label, label_color):
        for col, key in enumerate(keys):
            cond = gt[key]['condition']
            err = errors[key]

            # query image
            query_path = IMAGES / cond / key
            ax_q = axes[row_query][col]
            if query_path.exists():
                ax_q.imshow(mpimg.imread(str(query_path)))
            else:
                ax_q.text(0.5, 0.5, 'Not found', ha='center', va='center',
                          transform=ax_q.transAxes, fontsize=14)

            err_color = '#1B5E20' if err < 1 else ('#E65100' if err < 5 else '#B71C1C')
            err_str = f'{err:.3f}m' if err < 10 else f'{err:.0f}m'
            ax_q.set_title(f'QUERY: {cond}\nError: {err_str}',
                           fontsize=13, fontweight='bold', color=err_color)
            ax_q.axis('off')

            # border color
            for spine in ax_q.spines.values():
                spine.set_edgecolor(err_color)
                spine.set_linewidth(3)
                spine.set_visible(True)

            # nearest DB image
            gt_pos = gt[key]['pos']
            db_name, db_dist = find_nearest_db(gt_pos)
            ax_d = axes[row_db][col]
            db_path = IMAGES / db_name
            if db_path.exists():
                ax_d.imshow(mpimg.imread(str(db_path)))
            else:
                ax_d.text(0.5, 0.5, f'Not found:\n{db_name}', ha='center',
                          va='center', transform=ax_d.transAxes, fontsize=10)

            cam = db_name.split('/')[-2] if '/' in db_name else '?'
            ax_d.set_title(f'NEAREST DB ({cam}, {db_dist:.1f}m away)',
                           fontsize=11, color='#555')
            ax_d.axis('off')

        # section label on the left
        axes[row_query][0].text(-0.15, 0.5, f'{section_label}\nQuery',
                                 transform=axes[row_query][0].transAxes,
                                 fontsize=16, fontweight='bold', color=label_color,
                                 rotation=90, va='center', ha='center')
        axes[row_db][0].text(-0.15, 0.5, f'{section_label}\nDB Match',
                              transform=axes[row_db][0].transAxes,
                              fontsize=16, fontweight='bold', color=label_color,
                              rotation=90, va='center', ha='center')

    fill_row(best_keys,  row_query=0, row_db=1, section_label='BEST',  label_color='#1B5E20')
    fill_row(worst_keys, row_query=2, row_db=3, section_label='WORST', label_color='#B71C1C')

    # divider line between best and worst
    line = plt.Line2D([0.02, 0.98], [0.50, 0.50], transform=fig.transFigure,
                       color='black', linewidth=2, linestyle='--')
    fig.add_artist(line)

    fig.suptitle(f'Example Localized Images - {BEST_METHOD_NAME}\n'
                 f'Top: 5 best (smallest error)  |  Bottom: 5 worst (largest error)',
                 fontsize=20, fontweight='bold', y=0.99)
    plt.subplots_adjust(left=0.06, right=0.98, top=0.93, bottom=0.02, hspace=0.25, wspace=0.08)
    out = PLOTS_DIR / 'example_images_grid.png'
    plt.savefig(out, dpi=130)
    plt.close()
    print(f"  Saved {out.name}")


def main():
    print("Position Visualization for RobotCar Seasons")

    print("\nLoading data...")
    gt = load_ground_truth()
    print(f"  GT: {len(gt)} images")

    loc = load_localization_results(BEST_METHOD_RESULTS)
    print(f"  Localization results: {len(loc)} images")

    errors = compute_errors(gt, loc)
    n_good = sum(1 for e in errors.values() if e <= 5.0)
    print(f"  Matched: {len(errors)} images")
    print(f"  Within 5m: {n_good} ({100*n_good/len(errors):.1f}%)")

    db_pos = load_db_positions()
    print(f"  DB positions: {len(db_pos)}")

    print("\n--- Plot 1: Bird's eye view ---")
    plot_birds_eye_view(db_pos, gt, loc, errors)

    print("\n--- Plot 2: Per-condition positions ---")
    plot_per_condition_positions(gt, loc, errors)

    print("\n--- Plot 3: Example images ---")
    plot_example_images(gt, loc, errors)

    print(f"\nAll plots saved to: {PLOTS_DIR}")


if __name__ == '__main__':
    main()
