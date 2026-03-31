#!/usr/bin/env python3
"""Four diagnostics to verify synthetic IMU and VIO pipeline correctness
BEFORE we paper over scale drift with encoder correction.

1. IMU quality comparison exp 48 vs exp 51
2. VIO offline-on-exp51 (deferred - requires bag->TUM conversion)
3. IMU↔camera timestamp alignment
4. Gravity magnitude during rotation
"""
import glob
import math
import os
import sys

import numpy as np

EXP48_BAG = "/root/bags/husky_real/isaac_slam_1776423021"
EXP51_BAG = "/root/bags/husky_real/isaac_slam_1776444185"


def header(s):
    print("\n" + "=" * 70)
    print(s)


def load_imu(bag):
    """imu.csv columns: timestamp, ax, ay, az, gx, gy, gz, qx, qy, qz, qw."""
    return np.loadtxt(f"{bag}/imu.csv", delimiter=",", skiprows=1)


def load_gt(bag):
    return np.loadtxt(f"{bag}/groundtruth.csv", delimiter=",", skiprows=1)


def inspect_imu_header(bag):
    """Print the IMU CSV header so we know column meanings."""
    with open(f"{bag}/imu.csv") as f:
        return f.readline().strip()


# ---------------------------------------------------------------------------
# Diagnostic 1: IMU quality comparison
# ---------------------------------------------------------------------------
def diag1():
    header("DIAGNOSTIC 1: IMU quality - exp 48 vs exp 51")
    for label, bag in [("exp48", EXP48_BAG), ("exp51", EXP51_BAG)]:
        hdr = inspect_imu_header(bag)
        print(f"\n[{label}] {bag.split('/')[-1]}")
        print(f"  header: {hdr}")
        imu = load_imu(bag)
        gt = load_gt(bag)
        dur = gt[-1, 0] - gt[0, 0]
        dist = float(np.sum(np.sqrt(np.sum(np.diff(gt[:, 1:3], axis=0) ** 2, axis=1))))
        print(f"  {dur:.0f}s, {dist:.0f}m, {len(imu)} IMU samples, {len(gt)} GT samples")

        # Hz from timestamps
        imu_dt = np.diff(imu[:, 0])
        print(f"  IMU rate: {1.0/np.median(imu_dt):.1f} Hz (median dt={np.median(imu_dt)*1000:.2f}ms)")

        # Stationary period (first 1 s based on GT velocity < 0.05 m/s)
        gt_vel = np.sqrt(np.sum(np.diff(gt[:, 1:3], axis=0) ** 2, axis=1)) / np.diff(gt[:, 0])
        stationary_end = gt[0, 0]
        for i, v in enumerate(gt_vel):
            if v > 0.05:
                stationary_end = gt[i, 0]
                break
        stat = imu[imu[:, 0] < stationary_end]
        motion = imu[imu[:, 0] > stationary_end + 1.0]
        print(f"  stationary window: {imu[0,0]:.1f}..{stationary_end:.1f}s ({len(stat)} samples)")

        if len(stat) > 100:
            print(f"  STATIONARY ax={stat[:,1].mean():+.3f}±{stat[:,1].std():.4f}  "
                  f"ay={stat[:,2].mean():+.3f}±{stat[:,2].std():.4f}  "
                  f"az={stat[:,3].mean():+.3f}±{stat[:,3].std():.4f}  "
                  f"|a|={np.sqrt((stat[:,1:4]**2).sum(axis=1)).mean():.3f}")
            print(f"  STATIONARY gx={stat[:,4].mean():+.5f}±{stat[:,4].std():.5f}  "
                  f"gy={stat[:,5].mean():+.5f}±{stat[:,5].std():.5f}  "
                  f"gz={stat[:,6].mean():+.5f}±{stat[:,6].std():.5f}")
        if len(motion) > 100:
            print(f"  MOTION     ax_std={motion[:,1].std():.3f}  "
                  f"ay_std={motion[:,2].std():.3f}  "
                  f"az_mean={motion[:,3].mean():.3f}  "
                  f"|a|_mean={np.sqrt((motion[:,1:4]**2).sum(axis=1)).mean():.3f}")


# ---------------------------------------------------------------------------
# Diagnostic 3: IMU ↔ camera timestamp alignment
# ---------------------------------------------------------------------------
def diag3():
    header("DIAGNOSTIC 3: IMU ↔ camera timestamp alignment")
    for label, bag in [("exp48", EXP48_BAG), ("exp51", EXP51_BAG)]:
        print(f"\n[{label}]")
        imu = load_imu(bag)
        rgb_files = sorted(glob.glob(f"{bag}/camera_rgb/*.jpg") +
                           glob.glob(f"{bag}/camera_rgb/*.png"))
        if not rgb_files:
            print("  no rgb files found")
            continue
        cam_ts = np.array([float(os.path.basename(f).rsplit('.', 1)[0])
                           for f in rgb_files])
        cam_ts.sort()
        print(f"  camera frames: {len(cam_ts)}  t=[{cam_ts[0]:.2f}..{cam_ts[-1]:.2f}]")
        print(f"  imu samples:   {len(imu)}   t=[{imu[0,0]:.2f}..{imu[-1,0]:.2f}]")
        cam_dt = np.diff(cam_ts)
        print(f"  camera rate: median={1.0/np.median(cam_dt):.1f}Hz  "
              f"min_dt={cam_dt.min()*1000:.1f}ms  max_dt={cam_dt.max()*1000:.1f}ms")

        # IMU samples between consecutive camera frames
        counts = np.zeros(len(cam_ts) - 1, dtype=int)
        j = 0
        for i in range(1, len(cam_ts)):
            while j < len(imu) and imu[j, 0] <= cam_ts[i - 1]:
                j += 1
            k = j
            while k < len(imu) and imu[k, 0] <= cam_ts[i]:
                k += 1
            counts[i - 1] = k - j
        print(f"  IMU between frames: mean={counts.mean():.1f}  "
              f"min={counts.min()}  max={counts.max()}  median={np.median(counts):.0f}")
        print(f"  frames with 0 IMU: {(counts==0).sum()} / {len(counts)}")
        print(f"  frames with <10 IMU: {(counts<10).sum()} / {len(counts)}")

        imu_dt = np.diff(imu[:, 0])
        gaps = np.where(imu_dt > 0.02)[0]
        print(f"  IMU gaps >20ms: {len(gaps)}")
        for g in gaps[:5]:
            print(f"    t={imu[g,0]:.2f}s gap={imu_dt[g]*1000:.0f}ms")


# ---------------------------------------------------------------------------
# Diagnostic 4: Gravity magnitude during rotation
# ---------------------------------------------------------------------------
def diag4():
    header("DIAGNOSTIC 4: accel magnitude during rotation-in-place")
    for label, bag in [("exp48", EXP48_BAG), ("exp51", EXP51_BAG)]:
        print(f"\n[{label}]")
        imu = load_imu(bag)
        gt = load_gt(bag)
        # Resample GT velocity
        gt_t = gt[:, 0]
        gt_v = np.zeros(len(gt))
        gt_v[1:] = np.sqrt(np.sum(np.diff(gt[:, 1:3], axis=0) ** 2, axis=1)) / np.diff(gt_t)

        # Find rotation-in-place periods: low linear velocity, high |gz|
        # Slice by 2-second windows
        win = 2.0
        t_start = imu[0, 0]
        t_end = imu[-1, 0]
        reports = []
        for t0 in np.arange(t_start, t_end - win, win):
            m = (imu[:, 0] >= t0) & (imu[:, 0] < t0 + win)
            seg = imu[m]
            if len(seg) < 50:
                continue
            # GT velocity at center
            gt_idx = np.searchsorted(gt_t, t0 + win / 2)
            if gt_idx >= len(gt_v):
                continue
            v = gt_v[gt_idx]
            gz = np.abs(seg[:, 6]).mean()
            acc_mag = np.sqrt((seg[:, 1:4] ** 2).sum(axis=1))
            reports.append((t0, v, gz, acc_mag.mean(), acc_mag.std()))

        reports = np.array(reports)
        if len(reports) == 0:
            continue
        # Rotation-in-place: v < 0.1 m/s, gz > 0.1 rad/s
        rip = reports[(reports[:, 1] < 0.1) & (reports[:, 2] > 0.1)]
        straight = reports[(reports[:, 1] > 0.3) & (reports[:, 2] < 0.1)]
        if len(rip) > 0:
            print(f"  ROTATE-IN-PLACE ({len(rip)} windows): "
                  f"|a| mean={rip[:,3].mean():.3f}±{rip[:,3].std():.3f}  "
                  f"(expect ~9.81)")
            print(f"  samples: " + "  ".join(
                [f"t={r[0]:.0f}:|a|={r[3]:.2f}±{r[4]:.2f}" for r in rip[:5]]))
        else:
            print(f"  no rotate-in-place periods found")
        if len(straight) > 0:
            print(f"  STRAIGHT  ({len(straight)} windows): "
                  f"|a| mean={straight[:,3].mean():.3f}±{straight[:,3].std():.3f}")


if __name__ == "__main__":
    diag1()
    diag3()
    diag4()
    print("\n" + "=" * 70)
    print("NOTE: Diagnostic 2 (offline VIO on exp 51) not executed - requires")
    print("      bag->TUM conversion which can take minutes. Run separately if")
    print("      diagnostics 1/3/4 are inconclusive.")
