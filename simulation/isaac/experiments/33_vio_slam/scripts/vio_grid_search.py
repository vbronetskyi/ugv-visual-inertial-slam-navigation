#!/usr/bin/env python3
"""
VIO SLAM grid search: ThDepth × IMU noise × ORB params.
Runs ORB-SLAM3 RGB-D Inertial with different configs and compares results.

Recording: /root/bags/husky_real/isaac_slam_1776061151 (200Hz IMU, south route)
"""
import subprocess, os, numpy as np, time, json

RD = "/root/bags/husky_real/isaac_slam_1776061151"
SLAM = "/workspace/third_party/ORB_SLAM3"
BASE_CFG = f"{RD}/vio_config.yaml"
OUT_DIR = "/workspace/simulation/isaac/experiments/33_vio_slam"

configs = [
    (80,  1e-2, 1e-1, 1e-6, 1e-4, 2000, 20, "th80_default"),
    (100, 1e-2, 1e-1, 1e-6, 1e-4, 2000, 20, "th100_default"),
    (100, 5e-2, 2e-1, 5e-3, 2e-2, 2000, 20, "th100_noisy"),
    (100, 2e-2, 8e-2, 2e-3, 1e-2, 2000, 20, "th100_balanced"),
    (100, 2e-2, 8e-2, 2e-3, 1e-2, 3000, 10, "th100_3kfeat"),
    (80,  2e-2, 8e-2, 2e-3, 1e-2, 3000, 10, "th80_3kfeat"),
    (120, 2e-2, 8e-2, 2e-3, 1e-2, 3000, 10, "th120_3kfeat"),
]

gt = np.array([[float(p) for p in l.split()]
               for l in open(f"{RD}/groundtruth_slam_trim.txt")])

def umeyama_align(slam, gt_full):
    pairs = []
    for row in slam:
        idx = np.argmin(np.abs(gt_full[:, 0] - row[0]))
        if np.abs(gt_full[idx, 0] - row[0]) < 0.15:
            pairs.append((row[1:4], gt_full[idx, 1:4]))
    if len(pairs) < 5:
        return None, 0, 999
    ms = np.array([p[0] for p in pairs])
    mg = np.array([p[1] for p in pairs])
    mu_s, mu_g = ms.mean(0), mg.mean(0)
    ds, dg = ms - mu_s, mg - mu_g
    H = ds.T @ dg / len(ms)
    U, D, Vt = np.linalg.svd(H)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = Vt.T @ S @ U.T
    c = np.trace(np.diag(D) @ S) * len(ms) / np.sum(ds ** 2)
    t = mu_g - c * R @ mu_s
    aligned = c * (slam[:, 1:4] @ R.T) + t
    errs = [np.linalg.norm(aligned[i] - pairs[i][1]) for i in range(len(pairs))]
    return aligned, c, np.mean(errs)

results = []
for th, ng, na, gw, aw, nf, ift, label in configs:
    lines = open(BASE_CFG).readlines()
    new = []
    for line in lines:
        if line.startswith("Stereo.ThDepth:"): new.append(f"Stereo.ThDepth: {float(th)}\n")
        elif line.startswith("IMU.NoiseGyro:"): new.append(f"IMU.NoiseGyro: {ng}\n")
        elif line.startswith("IMU.NoiseAcc:"): new.append(f"IMU.NoiseAcc: {na}\n")
        elif line.startswith("IMU.GyroWalk:"): new.append(f"IMU.GyroWalk: {gw}\n")
        elif line.startswith("IMU.AccWalk:"): new.append(f"IMU.AccWalk: {aw}\n")
        elif line.startswith("ORBextractor.nFeatures:"): new.append(f"ORBextractor.nFeatures: {nf}\n")
        elif line.startswith("ORBextractor.iniThFAST:"): new.append(f"ORBextractor.iniThFAST: {ift}\n")
        else: new.append(line)
    cfg = f"{OUT_DIR}/config/vio_{label}.yaml"
    open(cfg, "w").writelines(new)

    traj = "CameraTrajectory.txt"  # written to cwd
    if os.path.exists(traj): os.remove(traj)
    log = f"{OUT_DIR}/logs/vio_{label}.log"

    print(f"{label}...", end=" ", flush=True)
    t0 = time.time()
    subprocess.run([
        f"{SLAM}/Examples/RGB-D-Inertial/rgbd_inertial_offline",
        f"{SLAM}/Vocabulary/ORBvoc.txt", cfg, RD,
        f"{RD}/associations_trim.txt", f"{RD}/imu_orbslam.txt",
    ], stdout=open(log, "w"), stderr=subprocess.STDOUT, timeout=600)
    wall = time.time() - t0

    if not os.path.exists(traj):
        print(f"FAIL ({wall:.0f}s)")
        results.append(dict(label=label, th=th, tracked=0, pct=0, scale=0, ate=999))
        continue

    s = np.array([[float(p) for p in l.split()] for l in open(traj)])
    aligned, scale, ate = umeyama_align(s, gt)
    pct = 100 * len(s) / len(gt)
    print(f"{len(s)}/{len(gt)} ({pct:.0f}%) scale={scale:.3f} ATE={ate:.2f}m ({wall:.0f}s)")
    results.append(dict(label=label, th=th, tracked=len(s), pct=pct,
                        scale=scale, ate=ate, wall=wall))

print(f"\n{'Config':22s} {'ThD':>4s} {'Track':>6s} {'%':>5s} {'Scale':>6s} {'ATE':>7s}")
for r in sorted(results, key=lambda r: -r["pct"]):
    marker = " ***" if r["pct"] > 10 and 0.8 < r["scale"] < 1.3 and r["ate"] < 10 else ""
    print(f"{r['label']:22s} {r['th']:4d} {r['tracked']:6d} {r['pct']:5.1f} "
          f"{r['scale']:6.3f} {r['ate']:7.2f}m{marker}")

with open(f"{OUT_DIR}/results/grid_search.json", "w") as f:
    json.dump(results, f, indent=2)
print(f"\nSaved: {OUT_DIR}/results/grid_search.json")
