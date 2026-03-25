#!/usr/bin/env python3
"""
Phase 2 offline test: run anchor localizer on exp20_south recording
and measure monotonicity, confidence, speed, localization error.

usage:
  cd /workspace/simulation/isaac
  python3 experiments/30_teach_and_repeat/scripts/test_localizer_offline.py
"""
import sys
import os
import glob
import time

import numpy as np
import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../scripts"))
from anchor_localizer import AnchorLocalizer

ANCHOR_DIR = "/workspace/simulation/isaac/route_memory/south"
REC_DIR = "/root/bags/husky_real/exp20_south"
OUT_DIR = os.path.join(os.path.dirname(__file__), "../results")


def load_gt(rec_dir):
    pts = []
    for line in open(f"{rec_dir}/groundtruth_tum.txt"):
        p = line.split()
        if len(p) >= 4:
            pts.append((float(p[0]), float(p[1]), float(p[2])))
    return np.array(pts)


def find_turnaround(gt, cam_ts):
    turn_idx = int(np.argmax(gt[:, 1]))
    turn_t = gt[turn_idx, 0]
    turn_frame = int(np.argmin(np.abs(cam_ts - turn_t)))
    return turn_frame, turn_t


def run_phase(localizer, frames, cam_ts, gt, label):
    results = []
    for i, cam_path in enumerate(frames):
        frame = cv2.imread(cam_path)
        t0 = time.time()
        aid, conf, adata = localizer.localize(frame)
        dt = (time.time() - t0) * 1000

        ct = float(os.path.splitext(os.path.basename(cam_path))[0])
        gi = int(np.argmin(np.abs(gt[:, 0] - ct)))
        err = np.hypot(gt[gi, 1] - adata["x"], gt[gi, 2] - adata["y"])

        results.append({
            "frame": i,
            "anchor_id": aid,
            "confidence": conf,
            "s": adata["s"],
            "dt_ms": dt,
            "gt_x": gt[gi, 1],
            "gt_y": gt[gi, 2],
            "loc_x": adata["x"],
            "loc_y": adata["y"],
            "error": err,
        })

        if i % 400 == 0:
            print(
                f"  [{label}] {i:4d}/{len(frames)} anchor={aid:3d} "
                f"conf={conf:.2f} s={adata['s']:6.1f}m err={err:.2f}m"
            )

    aids = [r["anchor_id"] for r in results]
    confs = [r["confidence"] for r in results]
    dts = [r["dt_ms"] for r in results]
    errs = [r["error"] for r in results]
    n = len(results)

    mono_v = sum(1 for i in range(1, len(aids)) if aids[i] < aids[i - 1] - 1)
    jumps5 = sum(1 for i in range(1, len(aids)) if abs(aids[i] - aids[i - 1]) > 5)
    n_anchors = len(localizer.anchors)

    print(f"\n  {label} RESULTS:")
    print(f"    Anchors: {min(aids)} -> {max(aids)} (final={aids[-1]}/{n_anchors-1})")
    print(f"    Confidence: mean={np.mean(confs):.3f}  >0.3: {100*sum(1 for c in confs if c>0.3)/n:.1f}%")
    print(f"    Mono violations: {mono_v}/{n} ({100*mono_v/n:.1f}%)")
    print(f"    Jumps >5: {jumps5}")
    print(f"    Speed: mean={np.mean(dts):.1f}ms ({1000/np.mean(dts):.0f} FPS)")
    print(f"    Loc error: mean={np.mean(errs):.2f}m  median={np.median(errs):.2f}m  max={max(errs):.2f}m")

    return results


def main():
    gt = load_gt(REC_DIR)
    cam_files = sorted(
        glob.glob(f"{REC_DIR}/camera_rgb/*.jpg"),
        key=lambda f: float(os.path.splitext(os.path.basename(f))[0]),
    )
    cam_ts = np.array(
        [float(os.path.splitext(os.path.basename(f))[0]) for f in cam_files]
    )
    turn_frame, turn_t = find_turnaround(gt, cam_ts)
    print(f"Recording: {len(cam_files)} frames, turnaround at frame {turn_frame}")

    print(f"\n{'='*50}\nOUTBOUND\n{'='*50}")
    loc_out = AnchorLocalizer(ANCHOR_DIR, direction="outbound")
    res_out = run_phase(loc_out, cam_files[:turn_frame], cam_ts, gt, "outbound")

    print(f"\n{'='*50}\nRETURN\n{'='*50}")
    loc_ret = AnchorLocalizer(ANCHOR_DIR, direction="return")
    res_ret = run_phase(loc_ret, cam_files[turn_frame:], cam_ts, gt, "return")

    # --- Plot ---
    fig, axes = plt.subplots(3, 2, figsize=(18, 14), sharex="col")
    fig.suptitle("Phase 2: Anchor Localizer Offline Test", fontsize=14, fontweight="bold")

    for col, (res, label, na) in enumerate([
        (res_out, "Outbound", len(loc_out.anchors)),
        (res_ret, "Return", len(loc_ret.anchors)),
    ]):
        aids = [r["anchor_id"] for r in res]
        confs = [r["confidence"] for r in res]
        errs = [r["error"] for r in res]
        mono_v = sum(1 for i in range(1, len(aids)) if aids[i] < aids[i - 1] - 1)

        axes[0][col].plot([r["frame"] for r in res], aids, lw=1)
        axes[0][col].axhline(na - 1, color="r", ls="--", alpha=0.5)
        axes[0][col].set_ylabel("Anchor ID")
        axes[0][col].set_title(
            f"{label} - violations: {mono_v}/{len(res)} = {100*mono_v/len(res):.1f}%"
        )
        axes[0][col].grid(alpha=0.3)

        axes[1][col].plot([r["frame"] for r in res], confs, lw=0.8, color="green")
        axes[1][col].axhline(0.3, color="r", ls="--")
        axes[1][col].set_ylabel("Confidence")
        axes[1][col].set_title(f"{label} - Confidence (mean={np.mean(confs):.3f})")
        axes[1][col].grid(alpha=0.3)

        axes[2][col].plot([r["frame"] for r in res], errs, lw=0.8, color="orange")
        axes[2][col].axhline(2.0, color="r", ls="--", label="anchor spacing")
        axes[2][col].set_ylabel("Loc error (m)")
        axes[2][col].set_xlabel("Frame")
        axes[2][col].set_title(
            f"{label} - Error (mean={np.mean(errs):.2f}m, median={np.median(errs):.2f}m)"
        )
        axes[2][col].legend(fontsize=8)
        axes[2][col].grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = f"{OUT_DIR}/localizer_test_v2.png"
    plt.savefig(out_path, dpi=150)
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
