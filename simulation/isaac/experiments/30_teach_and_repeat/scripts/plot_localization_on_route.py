#!/usr/bin/env python3
"""
Phase 2 visualization: localized anchor positions on scene map + error plot.

usage:
  cd /workspace/simulation/isaac
  python3 experiments/30_teach_and_repeat/scripts/plot_localization_on_route.py
"""
import sys
import os
import glob
import json

import numpy as np
import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.image import imread

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../scripts"))
from anchor_localizer import AnchorLocalizer

ANCHOR_DIR = "/workspace/simulation/isaac/route_memory/south"
REC_DIR = "/root/bags/husky_real/exp20_south"
WEB_MAP = "/workspace/simulation/isaac/results/final/01_web_map.png"
WEB_EXTENT = [-120, 120, -80, 80]
OUT_DIR = os.path.join(os.path.dirname(__file__), "../results")

CONE_GROUPS = [
    [(-52, -5.5), (-50, -4.5), (-48, -5.0)],
    [(13, -2.5), (15, -1.5), (17, -2.0)],
    [(43, -1.0), (45, -0.5), (47, -1.0)],
]
TENT_POS = (-20, 0.2)
SPAWN = (-95, -6)
DEST = (72, -5)


def main():
    anchors = json.load(open(f"{ANCHOR_DIR}/anchors.json"))
    out_a = [a for a in anchors if a["direction"] == "outbound"]
    ret_a = [a for a in anchors if a["direction"] == "return"]
    routes = json.load(open("/tmp/slam_routes.json"))
    nav_south = np.array(routes["south"])

    # GT
    gt_pts = []
    for line in open(f"{REC_DIR}/groundtruth_tum.txt"):
        p = line.split()
        if len(p) >= 4:
            gt_pts.append((float(p[0]), float(p[1]), float(p[2])))
    gt = np.array(gt_pts)
    turn_idx = int(np.argmax(gt[:, 1]))

    # camera frames
    cam_files = sorted(
        glob.glob(f"{REC_DIR}/camera_rgb/*.jpg"),
        key=lambda f: float(os.path.splitext(os.path.basename(f))[0]),
    )
    cam_ts = np.array(
        [float(os.path.splitext(os.path.basename(f))[0]) for f in cam_files]
    )
    turn_frame = int(np.argmin(np.abs(cam_ts - gt[turn_idx, 0])))

    # Run localizer (every 5th frame for speed)
    print("Running localizer...")
    loc_out = AnchorLocalizer(ANCHOR_DIR, direction="outbound")
    loc_ret = AnchorLocalizer(ANCHOR_DIR, direction="return")

    def run_sampled(localizer, frames, step=5):
        pts = []
        for i in range(0, len(frames), step):
            frame = cv2.imread(frames[i])
            aid, conf, adata = localizer.localize(frame)
            ct = float(os.path.splitext(os.path.basename(frames[i]))[0])
            gi = int(np.argmin(np.abs(gt[:, 0] - ct)))
            pts.append((gt[gi, 1], gt[gi, 2], adata["x"], adata["y"], conf))
        return pts

    pts_out = run_sampled(loc_out, cam_files[:turn_frame])
    pts_ret = run_sampled(loc_ret, cam_files[turn_frame:])
    print(f"  outbound: {len(pts_out)} samples, return: {len(pts_ret)} samples")

    # --- Plot ---
    fig, axes = plt.subplots(2, 1, figsize=(20, 16),
                             gridspec_kw={"height_ratios": [1, 1]})
    fig.suptitle("Exp 30: Anchor Localization - South Route",
                 fontsize=15, fontweight="bold", y=0.98)

    # Top: scene map
    ax = axes[0]
    bg = imread(WEB_MAP)
    ax.imshow(bg, extent=WEB_EXTENT, aspect="equal", alpha=0.85, zorder=1)

    for group in CONE_GROUPS:
        for cx, cy in group:
            ax.add_patch(plt.Circle((cx, cy), 0.6, color="#FFEB3B",
                                    alpha=0.95, zorder=5))
    ax.add_patch(mpatches.FancyBboxPatch(
        (TENT_POS[0] - 1.5, TENT_POS[1] - 1), 3, 2,
        boxstyle="round,pad=0.3", facecolor="#FF5722",
        edgecolor="white", lw=1, alpha=0.9, zorder=5))

    ax.plot(gt[:turn_idx, 1], gt[:turn_idx, 2], color="gray", lw=1,
            alpha=0.4, zorder=3, label="GT trajectory")
    ax.plot(gt[turn_idx:, 1], gt[turn_idx:, 2], color="gray", lw=1,
            alpha=0.3, zorder=3, ls="--")

    ax.scatter([a["x"] for a in out_a], [a["y"] for a in out_a],
               c="#2196F3", s=30, edgecolors="white", linewidths=0.5,
               zorder=6, label=f"Anchors outbound ({len(out_a)})")
    ax.scatter([a["x"] for a in ret_a], [a["y"] for a in ret_a],
               c="#F44336", s=30, edgecolors="white", linewidths=0.5,
               zorder=6, label=f"Anchors return ({len(ret_a)})")

    ax.plot(nav_south[:, 0], nav_south[:, 1], color="#FFEB3B", lw=1.5,
            ls=":", alpha=0.7, zorder=4, label="Nav route (tree-safe)")

    for pts, color in [(pts_out, "#2196F3"), (pts_ret, "#F44336")]:
        for gx, gy, lx, ly, conf in pts[::3]:
            if conf > 0.3:
                ax.plot([gx, lx], [gy, ly], color=color, lw=0.5,
                        alpha=0.3, zorder=4)

    lox = [p[2] for p in pts_out]; loy = [p[3] for p in pts_out]
    lrx = [p[2] for p in pts_ret]; lry = [p[3] for p in pts_ret]
    ax.plot(lox, loy, color="#1565C0", lw=2.5, alpha=0.9, zorder=7,
            label="Localized (outbound)")
    ax.plot(lrx, lry, color="#C62828", lw=2.5, alpha=0.9, zorder=7,
            label="Localized (return)")

    ax.plot(*SPAWN, "s", color="#4CAF50", ms=14, zorder=10,
            markeredgecolor="white", markeredgewidth=1.5)
    ax.plot(*DEST, "*", color="#F44336", ms=18, zorder=10,
            markeredgecolor="white", markeredgewidth=1.5)

    for i, a in enumerate(anchors):
        if i % 20 == 0:
            c = "#1565C0" if a["direction"] == "outbound" else "#C62828"
            ax.annotate(f'{a["id"]}', (a["x"], a["y"]),
                        fontsize=7, fontweight="bold", ha="center",
                        xytext=(0, 8), textcoords="offset points",
                        color="white",
                        bbox=dict(boxstyle="round,pad=0.15", fc=c,
                                  alpha=0.7, ec="none"), zorder=10)

    ax.set_xlim(-110, 85)
    ax.set_ylim(-50, 25)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Scene map - GT (gray) vs Localized (blue/red)",
                 fontsize=11, style="italic")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.set_aspect("equal")

    # Bottom: error along route
    ax2 = axes[1]

    out_err = [np.hypot(p[0] - p[2], p[1] - p[3]) for p in pts_out]
    ret_err = [np.hypot(p[0] - p[2], p[1] - p[3]) for p in pts_ret]
    out_conf = [p[4] for p in pts_out]
    ret_conf = [p[4] for p in pts_ret]
    out_s = np.linspace(0, out_a[-1]["s"], len(out_err))
    ret_s = np.linspace(ret_a[0]["s"], ret_a[-1]["s"], len(ret_err))

    ax2.fill_between(out_s, 0, out_err, color="#2196F3", alpha=0.3)
    ax2.plot(out_s, out_err, color="#2196F3", lw=1,
             label=f"Outbound err (mean={np.mean(out_err):.1f}m)")
    ax2.fill_between(ret_s, 0, ret_err, color="#F44336", alpha=0.3)
    ax2.plot(ret_s, ret_err, color="#F44336", lw=1,
             label=f"Return err (mean={np.mean(ret_err):.1f}m)")

    ax3 = ax2.twinx()
    ax3.plot(out_s, out_conf, color="#2196F3", lw=0.5, ls="--", alpha=0.5)
    ax3.plot(ret_s, ret_conf, color="#F44336", lw=0.5, ls="--", alpha=0.5)
    ax3.set_ylabel("Confidence", color="gray")
    ax3.set_ylim(0, 1.1)

    ax2.axhline(2.0, color="orange", ls="--", alpha=0.5,
                label="anchor spacing (2m)")
    ax2.set_xlabel("Route progress s (m)")
    ax2.set_ylabel("Localization error (m)")
    ax2.set_title("Localization error: GT vs matched anchor position",
                  fontsize=11, style="italic")
    ax2.legend(loc="upper left", fontsize=9)
    ax2.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = f"{OUT_DIR}/localization_on_route.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
