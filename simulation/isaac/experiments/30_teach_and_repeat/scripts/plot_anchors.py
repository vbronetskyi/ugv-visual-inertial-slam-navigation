#!/usr/bin/env python3
"""Visualize route anchors: map + sample RGB frames."""
import json
import os
import cv2
import matplotlib.pyplot as plt

ANCHOR_DIR = "/workspace/simulation/isaac/route_memory/road"

with open(f"{ANCHOR_DIR}/anchors.json") as f:
    anchors = json.load(f)

fig, ax = plt.subplots(figsize=(20, 6))

try:
    with open("/tmp/slam_routes.json") as f:
        road = json.load(f).get("road", [])
    if road:
        rx = [p[0] for p in road]
        ry = [p[1] for p in road]
        ax.fill_between(rx, [y - 3 for y in ry], [y + 3 for y in ry],
                        color="#E8E8E8", alpha=0.5, label="road corridor")
except Exception as e:
    print(f"(no road overlay: {e})")

ax_x = [a["x"] for a in anchors]
ax_y = [a["y"] for a in anchors]
sc = ax.scatter(ax_x, ax_y, c=range(len(anchors)), cmap="viridis",
                s=25, zorder=5)

for i, a in enumerate(anchors):
    if i % 10 == 0:
        ax.annotate(str(a["id"]), (a["x"], a["y"]), fontsize=7)

ax.set_aspect("equal")
ax.set_xlabel("world x (m)")
ax.set_ylabel("world y (m)")
ax.set_title(f"Route anchors - {len(anchors)} points, "
             f"{anchors[-1]['s']:.0f}m total (exp20_road)")
ax.grid(True, alpha=0.3)
plt.colorbar(sc, ax=ax, label="anchor id")
plt.tight_layout()
plt.savefig(f"{ANCHOR_DIR}/anchors_map.png", dpi=150)
plt.close()
print(f"Saved {ANCHOR_DIR}/anchors_map.png")

sample_ids = [0, len(anchors) // 4, len(anchors) // 2,
              3 * len(anchors) // 4, len(anchors) - 1]
fig, axes = plt.subplots(1, len(sample_ids), figsize=(22, 5))
for idx, ax in zip(sample_ids, axes):
    a = anchors[idx]
    img = cv2.imread(os.path.join(ANCHOR_DIR, a["rgb"]))
    if img is None:
        ax.set_title(f"#{idx} (missing)")
        ax.axis("off")
        continue
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ax.set_title(f"anchor {a['id']}  s={a['s']:.0f}m\n"
                 f"({a['x']:.1f}, {a['y']:.1f})  yaw={a['yaw']:.2f}")
    ax.axis("off")
plt.tight_layout()
plt.savefig(f"{ANCHOR_DIR}/anchor_samples.png", dpi=120)
plt.close()
print(f"Saved {ANCHOR_DIR}/anchor_samples.png")
