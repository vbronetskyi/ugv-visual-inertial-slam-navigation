#!/usr/bin/env python3
"""3 tests to decide if scale drift comes from synthetic IMU or ORB-SLAM3.

Test 1: accel energy - IMU horizontal accel RMS vs GT accel RMS.
Test 2: position recovery - double-integrate synthetic IMU, compare to GT.
Test 3: LPF sensitivity - how would different LPF windows change recovery.
"""
import math
import sys

import numpy as np

BAG = sys.argv[1] if len(sys.argv) > 1 else "/root/bags/husky_real/isaac_slam_1776485681"


def test1_accel_rms(imu, gt):
    print("TEST 1: accel energy (GT-FD vs synthetic IMU, horizontal)")
    gt_dt = np.diff(gt[:, 0])
    gt_vx = np.diff(gt[:, 1]) / gt_dt
    gt_vy = np.diff(gt[:, 2]) / gt_dt
    gt_ax = np.diff(gt_vx) / gt_dt[:-1]
    gt_ay = np.diff(gt_vy) / gt_dt[:-1]
    gt_accel_mag = np.sqrt(gt_ax ** 2 + gt_ay ** 2)

    # Only compare during motion (GT speed > 0.1 m/s)
    gt_speed = np.sqrt(gt_vx ** 2 + gt_vy ** 2)
    mask = gt_speed[:-1] > 0.1
    if mask.sum() < 10:
        print("  too little motion, skipping")
        return None
    gt_rms = np.sqrt(np.mean(gt_accel_mag[mask] ** 2))

    # Synth horizontal: sqrt(ax^2 + ay^2) in body frame
    synth_mag = np.sqrt(imu[:, 1] ** 2 + imu[:, 2] ** 2)
    synth_rms = np.sqrt(np.mean(synth_mag ** 2))

    print(f"  GT horiz accel RMS (during motion): {gt_rms:.3f} m/s²")
    print(f"  Synth horiz accel RMS (body xy):    {synth_rms:.3f} m/s²")
    print(f"  Ratio synth/GT:                     {synth_rms / gt_rms:.3f}")
    print("  (<0.8 means LPF over-smooths; ~1.0 means accel energy preserved)")
    return synth_rms / gt_rms


def test2_position_recovery(imu, gt, lpf_window=None):
    """Double-integrate horizontal body accel (rotated to world via GT yaw)
    and compare to GT path length. Optionally re-apply a different LPF."""
    if lpf_window is not None and lpf_window != 1:
        # re-smooth ax, ay in body frame with specified window (centered moving average)
        kernel = np.ones(lpf_window) / lpf_window
        imu = imu.copy()
        imu[:, 1] = np.convolve(imu[:, 1], kernel, mode='same')
        imu[:, 2] = np.convolve(imu[:, 2], kernel, mode='same')

    # Interp GT yaw at IMU rate
    gt_yaw_interp = np.interp(imu[:, 0], gt[:, 0], gt[:, 4])

    # Use only the time overlap of IMU and GT
    mask = (imu[:, 0] >= gt[0, 0]) & (imu[:, 0] <= gt[-1, 0])
    imu_c = imu[mask]
    yaw_c = gt_yaw_interp[mask]
    if len(imu_c) < 100:
        return None

    # Find corresponding GT start
    t0 = imu_c[0, 0]
    i0 = int(np.searchsorted(gt[:, 0], t0))
    pos_x, pos_y = float(gt[i0, 1]), float(gt[i0, 2])
    vel_x, vel_y = 0.0, 0.0
    path_len = 0.0
    prev_pos = (pos_x, pos_y)
    # dt from consecutive IMU timestamps
    dts = np.diff(imu_c[:, 0])
    dts = np.clip(dts, 1e-4, 0.02)

    for i in range(1, len(imu_c)):
        dt = float(dts[i - 1])
        yaw = yaw_c[i]
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        ax_body = imu_c[i, 1]
        ay_body = imu_c[i, 2]
        ax_world = cos_y * ax_body - sin_y * ay_body
        ay_world = sin_y * ax_body + cos_y * ay_body
        vel_x += ax_world * dt
        vel_y += ay_world * dt
        pos_x += vel_x * dt
        pos_y += vel_y * dt
        step = math.hypot(pos_x - prev_pos[0], pos_y - prev_pos[1])
        path_len += step
        prev_pos = (pos_x, pos_y)

    # GT path length over same time window
    gt_mask = (gt[:, 0] >= imu_c[0, 0]) & (gt[:, 0] <= imu_c[-1, 0])
    gtc = gt[gt_mask]
    gt_dist = float(np.sum(np.sqrt(np.sum(np.diff(gtc[:, 1:3], axis=0) ** 2, axis=1))))

    ratio = path_len / gt_dist if gt_dist > 1e-3 else float('nan')
    return path_len, gt_dist, ratio


def main():
    print(f"\nBag: {BAG}")
    imu = np.loadtxt(f"{BAG}/imu.csv", delimiter=",", skiprows=1)
    gt = np.loadtxt(f"{BAG}/groundtruth.csv", delimiter=",", skiprows=1)
    print(f"IMU: {len(imu)} @ {1.0/np.median(np.diff(imu[:,0])):.0f} Hz")
    print(f"GT:  {len(gt)} samples over {gt[-1,0]-gt[0,0]:.0f}s")

    test1_accel_rms(imu, gt)

    print()
    print("TEST 2: position recovery (double-integrate synthetic IMU)")
    r = test2_position_recovery(imu, gt)
    if r:
        path_len, gt_dist, ratio = r
        print(f"  IMU-integrated path: {path_len:.1f} m")
        print(f"  GT path length:      {gt_dist:.1f} m")
        print(f"  Recovery ratio:      {ratio:.3f}")
        print("  (~1.0 = IMU energy fine, drift is in VIO; <0.9 or >1.1 = IMU-side bias)")

    print()
    print("TEST 3: LPF-window sensitivity of recovery ratio")
    print(f"  {'window':>8}  {'path (m)':>10}  {'ratio':>8}")
    for w in [1, 3, 5, 7, 11, 21, 41]:
        r = test2_position_recovery(imu, gt, lpf_window=w)
        if r:
            path_len, gt_dist, ratio = r
            print(f"  {w:>8}  {path_len:>10.1f}  {ratio:>8.3f}")


if __name__ == '__main__':
    main()
