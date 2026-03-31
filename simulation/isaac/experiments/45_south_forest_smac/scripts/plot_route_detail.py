#!/usr/bin/env python3
"""Detailed south route plot: all obstacles w/ real radii, original vs fixed route.
Uses same thinning + radii as run_husky_forest.py for accuracy.
"""
import json, math, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Patch
from matplotlib.lines import Line2D

SCENE = "/tmp/gazebo_models.json"
ROUTES = "/tmp/slam_routes.json"
BACKUP = "/tmp/slam_routes.json.bak"
OUT = "/workspace/simulation/isaac/experiments/45_south_forest_smac/results/south_route_detail.png"

with open(SCENE) as f: scene = json.load(f)
with open(ROUTES) as f: fixed = json.load(f)["south"]
with open(BACKUP) as f: orig = json.load(f)["south"]

fig, ax = plt.subplots(figsize=(22, 8))
ax.set_facecolor("#6b8e4e")

# draw everything that has collision in run_husky_forest.py
# trees: visual r=1.3 (canopy) + trunk r=0.3 (matches real 0.7m collision)
# shrubs r=0.4, rocks r=0.8, houses 6x6, barrels r=0.5
# (mirror thinning: remove 10 corner trees + every 3rd)
trees = [m for m in scene if m["type"] in ("pine", "oak")]
def _d(m, cx, cy): return math.hypot(m["x"]-cx, m["y"]-cy)
remove = set(t["name"] for t in sorted(trees, key=lambda m:_d(m,-120,-80))[:5])
remove |= set(t["name"] for t in sorted(trees, key=lambda m:_d(m,-120,80))[:5])
active_trees = []
skip = 0
for m in trees:
    if m["name"] in remove: continue
    skip += 1
    if skip % 3 == 0: continue
    active_trees.append(m)
for m in active_trees:
    ax.add_patch(Circle((m["x"], m["y"]), 1.3, facecolor="#1e4d1e",
                        edgecolor="#0d2d0d", linewidth=0.3, alpha=0.75, zorder=2))
    ax.add_patch(Circle((m["x"], m["y"]), 0.7, facecolor="#5a3a1a",
                        alpha=0.9, zorder=2.1))

for m in scene:
    t = m["type"]
    if t == "shrub":
        ax.add_patch(Circle((m["x"], m["y"]), 0.4,
                            facecolor="#3a6b2e", alpha=0.6, zorder=2))
    elif t == "rock":
        ax.add_patch(Circle((m["x"], m["y"]), 0.8,
                            facecolor="#6b6b6b", edgecolor="#3a3a3a",
                            linewidth=0.4, alpha=0.85, zorder=2))
    elif t == "house":
        ax.add_patch(Rectangle((m["x"]-3, m["y"]-3), 6, 6,
                               facecolor="#8b5a2b", edgecolor="#3a2810",
                               linewidth=0.8, alpha=0.9, zorder=2))
    elif t == "barrel":
        ax.add_patch(Circle((m["x"], m["y"]), 0.5,
                            facecolor="#a85c1f", edgecolor="#3a2810",
                            linewidth=0.4, alpha=0.9, zorder=2))

# routes
ox = [p[0] for p in orig]; oy = [p[1] for p in orig]
fx = [p[0] for p in fixed]; fy = [p[1] for p in fixed]
ax.plot(ox, oy, color="#d62728", linewidth=1.6, alpha=0.55,
        label=f"original south route (201 WPs)", zorder=4, linestyle="--")
ax.plot(fx, fy, color="white", linewidth=2.2, alpha=0.95,
        label=f"fixed (≥1.2m clearance)", zorder=5)
ax.scatter(fx[0], fy[0], c="lime", s=80, marker="o",
           edgecolors="black", linewidths=1, zorder=6, label="spawn")
max_x_idx = max(range(len(fixed)), key=lambda i: fixed[i][0])
ax.scatter(fx[max_x_idx], fy[max_x_idx], c="magenta", s=80, marker="s",
           edgecolors="black", linewidths=1, zorder=6, label="turnaround")

ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_title("South Forest Route - original vs 1.2m-clearance fixed")
ax.grid(alpha=0.2, color="white")
ax.set_aspect("equal"); ax.set_xlim(-110, 85); ax.set_ylim(-50, 5)

handles, _ = ax.get_legend_handles_labels()
handles += [
    Line2D([0],[0], marker="o", color="w", markerfacecolor="#1e4d1e",
           markersize=12, linestyle="none", label="tree canopy r=1.3m"),
    Line2D([0],[0], marker="o", color="w", markerfacecolor="#5a3a1a",
           markersize=7, linestyle="none", label="tree collision r=0.7m"),
    Line2D([0],[0], marker="o", color="w", markerfacecolor="#3a6b2e",
           markersize=5, linestyle="none", label="shrub r=0.4m"),
    Line2D([0],[0], marker="o", color="w", markerfacecolor="#6b6b6b",
           markersize=7, linestyle="none", label="rock r=0.8m"),
    Patch(facecolor="#8b5a2b", edgecolor="#3a2810", label="house 6×6m"),
]
ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(0,-0.08),
          ncol=4, fontsize=9, framealpha=0.95)
plt.tight_layout()
plt.savefig(OUT, dpi=150, bbox_inches="tight")
print(f"saved {OUT}")
