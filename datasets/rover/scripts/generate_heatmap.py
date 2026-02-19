#!/usr/bin/env python3
"""Heatmap of all ORB-SLAM3 results on ROVER dataset"""

import json
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, BoundaryNorm

RESULTS_DIR = "/workspace/datasets/rover/results"

# all 23 recordings in logical order (route, then lighting)
ALL_RECORDINGS = [
    # garden (8)
    "garden_large_day_2024-05-29_1",
    "garden_large_spring_2024-04-11",
    "garden_large_summer_2023-08-18",
    "garden_large_autumn_2023-12-21",
    "garden_large_winter_2024-01-13",
    "garden_large_dusk_2024-05-29_2",
    "garden_large_night_2024-05-30_1",
    "garden_large_night-light_2024-05-30_2",
    # park (7)
    "park_day_2024-05-08",
    "park_spring_2024-04-14",
    "park_summer_2023-07-31",
    "park_autumn_2023-11-07",
    "park_dusk_2024-05-13_1",
    "park_night_2024-05-13_2",
    "park_night-light_2024-05-24_2",
    # campus (8)
    "campus_large_day_2024-09-25",
    "campus_large_spring_2024-04-14",
    "campus_large_summer_2023-07-20",
    "campus_large_autumn_2023-11-07",
    "campus_large_winter_2024-01-27",
    "campus_large_dusk_2024-09-24_2",
    "campus_large_night_2024-09-24_3",
    "campus_large_night-light_2024-09-24_4",
]

SHORT_NAMES = {
    "garden_large_day_2024-05-29_1": "GL / day",
    "garden_large_spring_2024-04-11": "GL / spring",
    "garden_large_summer_2023-08-18": "GL / summer",
    "garden_large_autumn_2023-12-21": "GL / autumn",
    "garden_large_winter_2024-01-13": "GL / winter",
    "garden_large_dusk_2024-05-29_2": "GL / dusk",
    "garden_large_night_2024-05-30_1": "GL / night",
    "garden_large_night-light_2024-05-30_2": "GL / night-light",
    "park_day_2024-05-08": "P / day",
    "park_spring_2024-04-14": "P / spring",
    "park_summer_2023-07-31": "P / summer",
    "park_autumn_2023-11-07": "P / autumn",
    "park_dusk_2024-05-13_1": "P / dusk",
    "park_night_2024-05-13_2": "P / night",
    "park_night-light_2024-05-24_2": "P / night-light",
    "campus_large_day_2024-09-25": "CL / day",
    "campus_large_spring_2024-04-14": "CL / spring",
    "campus_large_summer_2023-07-20": "CL / summer",
    "campus_large_autumn_2023-11-07": "CL / autumn",
    "campus_large_winter_2024-01-27": "CL / winter",
    "campus_large_dusk_2024-09-24_2": "CL / dusk",
    "campus_large_night_2024-09-24_3": "CL / night",
    "campus_large_night-light_2024-09-24_4": "CL / night-light",
}

MODES = ["stereo_pinhole", "stereo_inertial_pinhole", "rgbd"]
MODE_LABELS = ["Stereo PH", "Stereo-Inertial PH", "RGB-D"]


def load_results():
    """load all eval_results.json into {(rec, mode): result} dict"""
    results = {}
    for rec in ALL_RECORDINGS:
        for mode in MODES:
            eval_json = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
            if os.path.exists(eval_json):
                with open(eval_json) as f:
                    results[(rec, mode)] = json.load(f)
    return results


def main():
    results = load_results()

    n_rec = len(ALL_RECORDINGS)
    n_modes = len(MODES)

    # build data matrices
    ate_data = np.full((n_rec, n_modes), np.nan)
    scale_data = np.full((n_rec, n_modes), np.nan)
    status = [["" for _ in range(n_modes)] for _ in range(n_rec)]

    for i, rec in enumerate(ALL_RECORDINGS):
        for j, mode in enumerate(MODES):
            r = results.get((rec, mode))
            if r is None:
                status[i][j] = "MISSING"
            elif "error" in r:
                status[i][j] = r["error"]
            else:
                ate = r.get("ate_sim3", {}).get("rmse")
                s = r.get("sim3_scale", 1.0)
                if ate is not None:
                    ate_data[i, j] = ate
                    scale_data[i, j] = s
                    # mark scale-collapsed as special
                    if s < 0.1 or s > 3.0:
                        status[i][j] = "BAD_SCALE"
                    else:
                        status[i][j] = "OK"
                else:
                    status[i][j] = "NO_ATE"

    # === heatmap ===
    fig, axes = plt.subplots(1, 2, figsize=(14, 12), gridspec_kw={'width_ratios': [3, 1.2]})

    # left: ATE heatmap
    ax = axes[0]
    # custom cmap: green (good) -> yellow -> red (bad) -> dark red
    cmap = LinearSegmentedColormap.from_list('ate',
        [(0, '#1a9641'), (0.15, '#91cf60'), (0.3, '#d9ef8b'),
         (0.5, '#fee08b'), (0.7, '#fc8d59'), (1.0, '#d73027')])
    cmap.set_bad(color='#e0e0e0')  # gray for NaN

    vmax = 10.0
    im = ax.imshow(ate_data, cmap=cmap, aspect='auto', vmin=0, vmax=vmax,
                   interpolation='nearest')

    # text annotations
    for i in range(n_rec):
        for j in range(n_modes):
            v = ate_data[i, j]
            st = status[i][j]
            if st == "MISSING":
                ax.text(j, i, '-', ha='center', va='center', fontsize=8,
                        color='#888888', fontweight='bold')
            elif st in ("no trajectory file", "timeout", "NO_ATE"):
                ax.text(j, i, 'FAIL', ha='center', va='center', fontsize=7,
                        color='#cc0000', fontweight='bold')
            elif st == "BAD_SCALE":
                s = scale_data[i, j]
                ax.text(j, i, f'{v:.1f}\n(s={s:.3f})', ha='center', va='center',
                        fontsize=6, color='white' if v > 4 else 'black', fontstyle='italic')
            else:
                s = scale_data[i, j]
                txt = f'{v:.2f}' if v < 10 else f'{v:.1f}'
                color = 'white' if v > 5 else 'black'
                ax.text(j, i, txt, ha='center', va='center',
                        fontsize=8, color=color, fontweight='bold' if v < 1 else 'normal')

    ax.set_xticks(range(n_modes))
    ax.set_xticklabels(MODE_LABELS, fontsize=11, fontweight='bold')
    ax.set_yticks(range(n_rec))
    ax.set_yticklabels([SHORT_NAMES[r] for r in ALL_RECORDINGS], fontsize=9)
    ax.set_title('ATE RMSE (m), lower is better', fontsize=13, fontweight='bold', pad=10)

    # horizontal separators between routes
    for y in [7.5, 14.5]:
        ax.axhline(y=y, color='black', linewidth=2)

    # route labels on left
    ax.text(-0.6, 3.5, 'GARDEN', ha='center', va='center', fontsize=10,
            fontweight='bold', rotation=90, transform=ax.get_yaxis_transform())
    ax.text(-0.6, 11, 'PARK', ha='center', va='center', fontsize=10,
            fontweight='bold', rotation=90, transform=ax.get_yaxis_transform())
    ax.text(-0.6, 19, 'CAMPUS', ha='center', va='center', fontsize=10,
            fontweight='bold', rotation=90, transform=ax.get_yaxis_transform())

    cb = plt.colorbar(im, ax=ax, shrink=0.6, pad=0.02)
    cb.set_label('ATE RMSE (m)', fontsize=10)

    # right: scale heatmap
    ax2 = axes[1]
    scale_cmap = LinearSegmentedColormap.from_list('scale',
        [(0, '#d73027'), (0.4, '#fc8d59'), (0.48, '#fee08b'),
         (0.5, '#1a9641'), (0.52, '#fee08b'), (0.6, '#fc8d59'), (1.0, '#d73027')])
    scale_cmap.set_bad(color='#e0e0e0')

    # clip scales for viz
    scale_vis = np.clip(scale_data, 0, 2.0)
    im2 = ax2.imshow(scale_vis, cmap=scale_cmap, aspect='auto', vmin=0, vmax=2.0,
                     interpolation='nearest')

    for i in range(n_rec):
        for j in range(n_modes):
            v = scale_data[i, j]
            st = status[i][j]
            if st in ("MISSING", "no trajectory file", "timeout", "NO_ATE"):
                ax2.text(j, i, '-', ha='center', va='center', fontsize=7, color='#888888')
            elif not np.isnan(v):
                txt = f'{v:.2f}' if 0.01 < v < 10 else f'{v:.0e}'
                color = 'white' if (v < 0.3 or v > 1.7) else 'black'
                ax2.text(j, i, txt, ha='center', va='center', fontsize=7, color=color)

    ax2.set_xticks(range(n_modes))
    ax2.set_xticklabels(MODE_LABELS, fontsize=9, fontweight='bold')
    ax2.set_yticks(range(n_rec))
    ax2.set_yticklabels(['' for _ in ALL_RECORDINGS])
    ax2.set_title('Scale (ideal = 1.0)', fontsize=11, fontweight='bold', pad=10)

    for y in [7.5, 14.5]:
        ax2.axhline(y=y, color='black', linewidth=2)

    cb2 = plt.colorbar(im2, ax=ax2, shrink=0.6, pad=0.04)
    cb2.set_label('Sim3 Scale', fontsize=9)

    plt.suptitle('ORB-SLAM3 on ROVER Dataset, All 23 Recordings x 3 Modes',
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0.03, 0.02, 1, 0.96])

    out_path = os.path.join(RESULTS_DIR, "heatmap_all_results.png")
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Heatmap saved to {out_path}")

    # print summary stats
    print("\n=== Summary ===")
    for j, (mode, label) in enumerate(zip(MODES, MODE_LABELS)):
        col = ate_data[:, j]
        valid = col[~np.isnan(col)]
        good_scale = scale_data[:, j]
        good_scale = good_scale[(~np.isnan(good_scale)) & (good_scale > 0.5) & (good_scale < 2.0)]
        n_fail = np.sum(np.isnan(col))
        n_bad_scale = np.sum([1 for i in range(n_rec) if status[i][j] == "BAD_SCALE"])
        n_missing = np.sum([1 for i in range(n_rec) if status[i][j] == "MISSING"])
        print(f"\n{label}:")
        print(f"  Success: {len(valid)}/23 ({n_fail} fail, {n_missing} missing)")
        print(f"  Bad scale (s<0.1 or s>3): {n_bad_scale}")
        if len(valid) > 0:
            print(f"  ATE: median={np.median(valid):.2f}, mean={np.mean(valid):.2f}, "
                  f"min={np.min(valid):.2f}, max={np.max(valid):.2f}")
        if len(good_scale) > 0:
            print(f"  Scale (valid only): median={np.median(good_scale):.3f}, "
                  f"mean={np.mean(good_scale):.3f}")


if __name__ == "__main__":
    main()
