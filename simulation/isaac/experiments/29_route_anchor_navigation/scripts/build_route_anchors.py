#!/usr/bin/env python3
"""
Build route anchors from a recording for teach-and-repeat navigation.

Every ANCHOR_SPACING metres along the GT trajectory, store:
 - RGB frame (nearest by timestamp)
 - depth frame
 - ORB features (descriptors + keypoints)
 - GT pose (x, y, z, yaw)
 - progress-along-route s
 - direction ("outbound" or "return")
"""
import os
import json
import glob
import shutil
import numpy as np
import cv2

REC_DIR = "/root/bags/husky_real/exp20_south"
ANCHOR_DIR = "/workspace/simulation/isaac/route_memory/south"
ANCHOR_SPACING = 2.0
TS_TOLERANCE = 0.15

os.makedirs(f"{ANCHOR_DIR}/rgb", exist_ok=True)
os.makedirs(f"{ANCHOR_DIR}/depth", exist_ok=True)

gt_points = []
with open(f"{REC_DIR}/groundtruth_tum.txt") as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        t = float(parts[0])
        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
        qz, qw = float(parts[6]), float(parts[7])
        yaw = 2.0 * np.arctan2(qz, qw)
        gt_points.append({"t": t, "x": x, "y": y, "z": z, "yaw": yaw})

print(f"GT points: {len(gt_points)}")

rgb_files = sorted(
    glob.glob(f"{REC_DIR}/camera_rgb/*.jpg") +
    glob.glob(f"{REC_DIR}/camera_rgb/*.png"),
    key=lambda f: float(os.path.splitext(os.path.basename(f))[0]),
)
rgb_ts = np.array([
    float(os.path.splitext(os.path.basename(f))[0]) for f in rgb_files
])
print(f"RGB frames: {len(rgb_files)}")

depth_files = {
    float(os.path.splitext(os.path.basename(f))[0]): f
    for f in sorted(
        glob.glob(f"{REC_DIR}/camera_depth/*.png") +
        glob.glob(f"{REC_DIR}/camera_depth/*.jpg"),
    )
}
print(f"Depth frames: {len(depth_files)}")

max_x_idx = max(range(len(gt_points)), key=lambda i: gt_points[i]["x"])
print(f"Turnaround at GT index {max_x_idx}, x={gt_points[max_x_idx]['x']:.1f}")

orb = cv2.ORB_create(nfeatures=1000)

anchors = []
last_x = last_y = None
s_accum = 0.0

for i in range(1, len(gt_points)):
    dx = gt_points[i]["x"] - gt_points[i - 1]["x"]
    dy = gt_points[i]["y"] - gt_points[i - 1]["y"]
    s_accum += float(np.hypot(dx, dy))

    gt = gt_points[i]
    if last_x is None:
        place = True
    else:
        place = np.hypot(gt["x"] - last_x, gt["y"] - last_y) >= ANCHOR_SPACING
    if not place:
        continue

    k = int(np.argmin(np.abs(rgb_ts - gt["t"])))
    if abs(rgb_ts[k] - gt["t"]) > TS_TOLERANCE:
        continue
    cam_path = rgb_files[k]
    cam_t = float(rgb_ts[k])

    aid = len(anchors)
    rgb_ext = os.path.splitext(cam_path)[1]
    rgb_dst = f"{ANCHOR_DIR}/rgb/{aid:04d}{rgb_ext}"
    shutil.copy2(cam_path, rgb_dst)

    depth_t_candidates = np.array(list(depth_files.keys()))
    dk = int(np.argmin(np.abs(depth_t_candidates - cam_t)))
    depth_rel = None
    if abs(depth_t_candidates[dk] - cam_t) < TS_TOLERANCE:
        depth_src = depth_files[depth_t_candidates[dk]]
        dep_ext = os.path.splitext(depth_src)[1]
        depth_dst = f"{ANCHOR_DIR}/depth/{aid:04d}{dep_ext}"
        shutil.copy2(depth_src, depth_dst)
        depth_rel = f"depth/{aid:04d}{dep_ext}"

    img = cv2.imread(cam_path, cv2.IMREAD_GRAYSCALE)
    kps, desc = orb.detectAndCompute(img, None)
    if desc is not None:
        np.save(f"{ANCHOR_DIR}/rgb/{aid:04d}_desc.npy", desc)
        kp_coords = np.array(
            [(kp.pt[0], kp.pt[1], kp.size, kp.angle) for kp in kps],
            dtype=np.float32,
        )
        np.save(f"{ANCHOR_DIR}/rgb/{aid:04d}_kp.npy", kp_coords)

    direction = "outbound" if i <= max_x_idx else "return"

    anchors.append({
        "id": aid,
        "s": round(s_accum, 2),
        "x": round(gt["x"], 4),
        "y": round(gt["y"], 4),
        "z": round(gt["z"], 4),
        "yaw": round(gt["yaw"], 4),
        "t": round(gt["t"], 4),
        "cam_t": round(cam_t, 4),
        "rgb": f"rgb/{aid:04d}{rgb_ext}",
        "depth": depth_rel,
        "n_features": int(len(kps)) if kps is not None else 0,
        "direction": direction,
    })
    last_x, last_y = gt["x"], gt["y"]

with open(f"{ANCHOR_DIR}/anchors.json", "w") as f:
    json.dump(anchors, f, indent=2)

nf = [a["n_features"] for a in anchors]
out_a = [a for a in anchors if a["direction"] == "outbound"]
ret_a = [a for a in anchors if a["direction"] == "return"]
with_depth = sum(1 for a in anchors if a["depth"] is not None)

print(f"\nRoute memory created:")
print(f"  Recording: {REC_DIR}")
print(f"  Anchors: {len(anchors)} (outbound={len(out_a)}, return={len(ret_a)})")
print(f"  With depth: {with_depth}/{len(anchors)}")
print(f"  Spacing: ~{ANCHOR_SPACING}m")
print(f"  Total route length: {s_accum:.1f}m")
print(f"  Start: ({anchors[0]['x']:.1f}, {anchors[0]['y']:.1f})")
print(f"  End:   ({anchors[-1]['x']:.1f}, {anchors[-1]['y']:.1f})")
print(f"  Features: avg={np.mean(nf):.0f} min={min(nf)} max={max(nf)}")
