#!/usr/bin/env python3
"""Deeper comparisons: stationary bias, per-segment scale, VIO-during-stop."""
import glob
import math
import os

import numpy as np

EXP48_BAG = "/root/bags/husky_real/isaac_slam_1776423021"
EXP51_BAG = "/root/bags/husky_real/isaac_slam_1776444185"
EXP48_VIO = "/workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/vio_final_camera.txt"
EXP51_VIO = "/workspace/simulation/isaac/experiments/51_hybrid_nav2_pp/logs/vio_final_camera.txt"


def header(s):
    print("\n" + "=" * 70)
    print(s)


def load_imu(bag):
    return np.loadtxt(f"{bag}/imu.csv", delimiter=",", skiprows=1)


def load_gt(bag):
    return np.loadtxt(f"{bag}/groundtruth.csv", delimiter=",", skiprows=1)


def diag_stationary_bias():
    header("1b: STATIONARY IMU bias (find long stops via GT velocity)")
    for label, bag in [("exp48", EXP48_BAG), ("exp51", EXP51_BAG)]:
        imu = load_imu(bag)
        gt = load_gt(bag)

        # Build GT velocity per sample
        dt = np.diff(gt[:, 0])
        vel = np.sqrt(np.sum(np.diff(gt[:, 1:3], axis=0) ** 2, axis=1)) / dt
        vel = np.concatenate([[0], vel])  # align to gt
        stationary_mask = vel < 0.02  # <2 cm/s
        total_stat_dur = np.sum(dt[stationary_mask[:-1]])

        # Extract IMU samples during stationary periods
        # For each GT sample that is stationary, gather nearby IMU
        stat_imu = []
        for i in np.where(stationary_mask)[0][::10]:  # every 10th stationary GT sample
            t = gt[i, 0]
            m = (imu[:, 0] > t - 0.05) & (imu[:, 0] < t + 0.05)
            stat_imu.append(imu[m])
        if not stat_imu:
            print(f"[{label}] no stationary samples")
            continue
        stat = np.vstack(stat_imu)
        print(f"\n[{label}] stationary duration={total_stat_dur:.1f}s  "
              f"({len(stat)} IMU samples sampled)")
        if len(stat) > 100:
            a = stat[:, 1:4]
            g = stat[:, 4:7]
            print(f"  ACCEL  ax={a[:,0].mean():+.4f}±{a[:,0].std():.4f}  "
                  f"ay={a[:,1].mean():+.4f}±{a[:,1].std():.4f}  "
                  f"az={a[:,2].mean():+.4f}±{a[:,2].std():.4f}  "
                  f"|a|={np.linalg.norm(a, axis=1).mean():.4f}")
            print(f"  GYRO   gx={g[:,0].mean():+.5f}±{g[:,0].std():.5f}  "
                  f"gy={g[:,1].mean():+.5f}±{g[:,1].std():.5f}  "
                  f"gz={g[:,2].mean():+.5f}±{g[:,2].std():.5f}")


def diag_vio_during_stop():
    header("2: VIO position during long robot stops")
    for label, bag, vio_path in [("exp48", EXP48_BAG, EXP48_VIO),
                                   ("exp51", EXP51_BAG, EXP51_VIO)]:
        if not os.path.exists(vio_path):
            print(f"[{label}] no VIO file: {vio_path}")
            continue
        vio = np.loadtxt(vio_path)  # t, tx, ty, tz, qx, qy, qz, qw
        gt = load_gt(bag)
        # match VIO ts to GT (clip to overlap)
        mask = (vio[:, 0] >= gt[0, 0]) & (vio[:, 0] <= gt[-1, 0])
        vio = vio[mask]
        if len(vio) < 10:
            continue
        gt_idx = np.searchsorted(gt[:, 0], vio[:, 0])
        gt_idx = np.clip(gt_idx, 0, len(gt) - 1)
        gm = gt[gt_idx]

        # GT velocity per VIO frame
        vio_gt_vel = np.zeros(len(vio))
        if len(gm) > 1:
            gdt = np.diff(gm[:, 0])
            gdt[gdt < 1e-6] = 1e-6
            vio_gt_vel[1:] = np.sqrt(np.sum(np.diff(gm[:, 1:3], axis=0) ** 2, axis=1)) / gdt

        # find stop periods (v<0.02 m/s) longer than 2 seconds
        stopped = vio_gt_vel < 0.02
        # find contiguous stop segments
        segs = []
        i = 0
        while i < len(stopped):
            if stopped[i]:
                j = i
                while j < len(stopped) and stopped[j]:
                    j += 1
                if vio[j - 1, 0] - vio[i, 0] >= 2.0:
                    segs.append((i, j - 1))
                i = j
            else:
                i += 1

        print(f"\n[{label}] {len(segs)} stop-segments ≥2s "
              f"(total {sum(vio[s[1],0]-vio[s[0],0] for s in segs):.1f}s)")
        total_drift_3d = 0.0
        for (a, b) in segs[:8]:
            d = vio[b, 1:4] - vio[a, 1:4]
            drift = np.linalg.norm(d)
            dur = vio[b, 0] - vio[a, 0]
            total_drift_3d += drift
            print(f"    t={vio[a,0]:.0f}..{vio[b,0]:.0f}s ({dur:.1f}s): "
                  f"VIO drift {drift:.3f}m (rate {drift/dur:.3f} m/s)")
        if len(segs) > 0:
            print(f"  total VIO drift during all stops: {total_drift_3d:.2f}m")


def diag_vio_vs_gt_per_segment():
    header("3: VIO vs GT distance - per minute")
    for label, bag, vio_path in [("exp48", EXP48_BAG, EXP48_VIO),
                                   ("exp51", EXP51_BAG, EXP51_VIO)]:
        if not os.path.exists(vio_path):
            continue
        vio = np.loadtxt(vio_path)
        gt = load_gt(bag)
        mask = (vio[:, 0] >= gt[0, 0]) & (vio[:, 0] <= gt[-1, 0])
        vio = vio[mask]
        gt_idx = np.searchsorted(gt[:, 0], vio[:, 0])
        gt_idx = np.clip(gt_idx, 0, len(gt) - 1)
        gm = gt[gt_idx]

        print(f"\n[{label}]")
        print(f"  {'t window':<12} {'VIO dist':>10} {'GT dist':>10} {'ratio':>8}")
        t0 = vio[0, 0]
        for win_start in np.arange(t0, vio[-1, 0] - 30, 30):
            m = (vio[:, 0] >= win_start) & (vio[:, 0] < win_start + 30)
            if m.sum() < 10:
                continue
            vs = vio[m]
            gs = gm[m]
            vd = np.sum(np.sqrt(np.sum(np.diff(vs[:, 1:4], axis=0) ** 2, axis=1)))
            gd = np.sum(np.sqrt(np.sum(np.diff(gs[:, 1:3], axis=0) ** 2, axis=1)))
            ratio = vd / gd if gd > 0.01 else float('nan')
            print(f"  {win_start-t0:4.0f}..{win_start-t0+30:4.0f}s  "
                  f"{vd:>9.1f}m {gd:>9.1f}m  {ratio:>7.2f}")


if __name__ == "__main__":
    diag_stationary_bias()
    diag_vio_during_stop()
    diag_vio_vs_gt_per_segment()
