#!/usr/bin/env python3
"""Generate comparison plots for ROVER ORB-SLAM3 results"""

import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from collections import OrderedDict

OUT_DIR = "/workspace/datasets/rover/results"
DATA_FILE = f"{OUT_DIR}/all_results.json"

# load data
with open(DATA_FILE) as f:
    results = json.load(f)

# helper: shorten recording names
def shorten(name: str) -> str:
    """garden_large_autumn_2023-12-21 -> GL/autumn, park_dusk_2024-05-13_1 -> P/dusk"""
    s = name
    s = s.replace("garden_large_", "GL/")
    s = s.replace("park_", "P/")
    # Strip date portion: everything from the first digit group onward after the condition
    # pattern: condition_YYYY-MM-DD or condition_YYYY-MM-DD_N
    import re
    s = re.sub(r"_\d{4}-\d{2}-\d{2}(_\d+)?$", "", s)
    return s

# gather unique recordings and modes in order
all_recordings = list(OrderedDict.fromkeys(r["recording"] for r in results))
mode_order = ["stereo", "stereo_inertial", "rgbd"]
mode_labels = {"stereo": "Stereo", "stereo_inertial": "Stereo-Inertial", "rgbd": "RGB-D"}
mode_colors = {"stereo": "#4472C4", "stereo_inertial": "#ED7D31", "rgbd": "#70AD47"}

# build lookup: (recording, mode) -> result dictlookup = {}
for r in results:
    lookup[(r["recording"], r["mode"])] = r

# successful results only
successful = [r for r in results if "ate_sim3" in r]

# Plot 1: comparison_bar.png
# only recordings that have at least one successful resultrec_with_success = []
for rec in all_recordings:
    for m in mode_order:
        key = (rec, m)
        if key in lookup and "ate_sim3" in lookup[key]:
            if rec not in rec_with_success:
                rec_with_success.append(rec)

short_labels = [shorten(r) for r in rec_with_success]
n_rec = len(rec_with_success)
n_modes = len(mode_order)
bar_width = 0.25
x = np.arange(n_rec)

fig, ax = plt.subplots(figsize=(14, 6))

for i, mode in enumerate(mode_order):
    vals = []
    for rec in rec_with_success:
        key = (rec, mode)
        if key in lookup and "ate_sim3" in lookup[key]:
            vals.append(lookup[key]["ate_sim3"]["rmse"])
        else:
            vals.append(0)  # no bar for failed
    bars = ax.bar(x + i * bar_width, vals, bar_width,
                  label=mode_labels[mode], color=mode_colors[mode],
                  edgecolor="white", linewidth=0.5)
    # annotate non-zero bars with value
    for j, v in enumerate(vals):
        if v > 0:
            ax.text(x[j] + i * bar_width, v + 0.05, f"{v:.2f}",
                    ha="center", va="bottom", fontsize=7, fontweight="bold")

# median of successful RGB-D resultsrgbd_rmses = [r["ate_sim3"]["rmse"] for r in successful if r["mode"] == "rgbd"]
median_val = np.median(rgbd_rmses)
ax.axhline(median_val, color="gray", linestyle="--", linewidth=1.2, alpha=0.7)
ax.text(n_rec - 0.5, median_val + 0.05, f"median = {median_val:.2f} m",
        ha="right", va="bottom", fontsize=9, color="gray")

ax.set_xticks(x + bar_width)  # center on the middle bar group
ax.set_xticklabels(short_labels, rotation=35, ha="right", fontsize=9)
ax.set_ylabel("ATE RMSE (m)", fontsize=11)
ax.set_title("ORB-SLAM3 RGB-D on ROVER: ATE RMSE", fontsize=13, fontweight="bold")
ax.legend(fontsize=9)
ax.set_xlim(-0.3, n_rec - 0.3)
ax.grid(axis="y", alpha=0.3)
fig.tight_layout()
fig.savefig(f"{OUT_DIR}/comparison_bar.png", dpi=150)
plt.close(fig)
print("Saved comparison_bar.png")

# Plot 2: comparison_heatmap.png
short_all = [shorten(r) for r in all_recordings]
n_all = len(all_recordings)

# build matrix (recordings x modes); NaN = failedmatrix = np.full((n_all, n_modes), np.nan)
for i, rec in enumerate(all_recordings):
    for j, mode in enumerate(mode_order):
        key = (rec, mode)
        if key in lookup and "ate_sim3" in lookup[key]:
            matrix[i, j] = lookup[key]["ate_sim3"]["rmse"]

# custom green-yellow-red colormapcmap = LinearSegmentedColormap.from_list("gyr", ["#2ca02c", "#ffdd44", "#d62728"])
cmap.set_bad(color="#d0d0d0")  # gray for NaN

fig, ax = plt.subplots(figsize=(10, 8))
masked = np.ma.masked_invalid(matrix)
im = ax.imshow(masked, cmap=cmap, aspect="auto", vmin=0,
               vmax=min(np.nanmax(matrix), 7.0))

# axis labelsax.set_xticks(range(n_modes))
ax.set_xticklabels([mode_labels[m] for m in mode_order], fontsize=10)
ax.set_yticks(range(n_all))
ax.set_yticklabels(short_all, fontsize=9)

# annotate cells
for i in range(n_all):
    for j in range(n_modes):
        val = matrix[i, j]
        if np.isnan(val):
            ax.text(j, i, "FAIL", ha="center", va="center",
                    fontsize=8, color="#666666", fontstyle="italic")
            # add hatching
            rect = mpatches.FancyBboxPatch((j - 0.5, i - 0.5), 1, 1,
                                            boxstyle="square,pad=0",
                                            fill=False, edgecolor="#999999",
                                            linewidth=0.5, hatch="///")
            ax.add_patch(rect)
        else:
            text_color = "white" if val > 3.5 else "black"
            ax.text(j, i, f"{val:.2f}", ha="center", va="center",
                    fontsize=9, fontweight="bold", color=text_color)

cbar = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
cbar.set_label("ATE RMSE (m)", fontsize=10)

ax.set_title("ORB-SLAM3 on ROVER: ATE RMSE Heatmap", fontsize=13, fontweight="bold")
fig.tight_layout()
fig.savefig(f"{OUT_DIR}/comparison_heatmap.png", dpi=150)
plt.close(fig)
print("Saved comparison_heatmap.png")

# Plot 3: comparison_garden_vs_park.png
garden_vals = []
garden_labels = []
park_vals = []
park_labels = []

for r in successful:
    if r["mode"] != "rgbd":
        continue
    rmse = r["ate_sim3"]["rmse"]
    short = shorten(r["recording"])
    if r["recording"].startswith("garden_large_"):
        garden_vals.append(rmse)
        garden_labels.append(short)
    elif r["recording"].startswith("park_"):
        park_vals.append(rmse)
        park_labels.append(short)

fig, ax = plt.subplots(figsize=(8, 6))

# box plots
bp = ax.boxplot([garden_vals, park_vals],
                labels=["Garden Large", "Park"],
                widths=0.45,
                patch_artist=True,
                medianprops=dict(color="black", linewidth=2),
                showfliers=False)

colors_box = ["#8BC8A4", "#F4A582"]
for patch, color in zip(bp["boxes"], colors_box):
    patch.set_facecolor(color)
    patch.set_alpha(0.6)

# overlay individual points with jitternp.random.seed(42)
for idx, (vals, labels) in enumerate([(garden_vals, garden_labels),
                                       (park_vals, park_labels)], start=1):
    jitter = np.random.uniform(-0.08, 0.08, size=len(vals))
    ax.scatter([idx + j for j in jitter], vals, color="black", s=50,
               zorder=5, edgecolors="white", linewidth=0.5)
    # annotate key points: highest and lowest for each group
    if vals:
        sorted_indices = np.argsort(vals)
        # annotate the best (lowest) and worst (highest)
        annotate_set = set()
        annotate_set.add(sorted_indices[0])   # best
        annotate_set.add(sorted_indices[-1])   # worst
        if len(sorted_indices) > 1:
            annotate_set.add(sorted_indices[-2])  # second worst if exists

        for ai in annotate_set:
            condition = labels[ai].split("/")[1]  # e.g. "autumn"
            ax.annotate(condition,
                        xy=(idx + jitter[ai], vals[ai]),
                        xytext=(12, 4 if vals[ai] < np.median(vals) else -12),
                        textcoords="offset points",
                        fontsize=8, fontstyle="italic",
                        arrowprops=dict(arrowstyle="-", color="gray", lw=0.7))

# stats annotationsfor idx, vals in enumerate([garden_vals, park_vals], start=1):
    if vals:
        med = np.median(vals)
        mu = np.mean(vals)
        ax.text(idx, max(vals) * 1.05 + 0.15,
                f"n={len(vals)}\nmed={med:.2f}\nmean={mu:.2f}",
                ha="center", va="bottom", fontsize=8,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.5))

ax.set_ylabel("ATE RMSE (m)", fontsize=11)
ax.set_title("ROVER RGB-D: Garden Large vs Park", fontsize=13, fontweight="bold")
ax.grid(axis="y", alpha=0.3)
fig.tight_layout()
fig.savefig(f"{OUT_DIR}/comparison_garden_vs_park.png", dpi=150)
plt.close(fig)
print("Saved comparison_garden_vs_park.png")

print("\nAll plots generated successfully.")
