#!/usr/bin/env python3
"""Week 0: ORB-SLAM3 mono on NCLT Ladybug3 images.

prepares TUM-format image list then runs ORB-SLAM3 monocular. only spring
(2012-04-29) and summer (2012-08-04) sessions have camera images; the other
two are LiDAR-only.
"""
import os
import sys
import subprocess
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm
from scipy.spatial.transform import Rotation
import glob
import shutil

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.ground_truth_loader import GroundTruthLoader
from evaluation.metrics import compute_ate, compute_rpe, sync_trajectories

PROJECT_ROOT = Path(__file__).resolve().parent.parent
NCLT_DATA = Path('/workspace/nclt_data')
ORB_SLAM3_DIR = Path('/tmp/ORB_SLAM3')
VOCAB_PATH = ORB_SLAM3_DIR / 'Vocabulary' / 'ORBvoc.txt'
CONFIG_PATH = PROJECT_ROOT / 'configs' / 'nclt_ladybug3_half.yaml'


def prepare_tum_format(session, cam_id=0, subsample=2, max_images=2000):
    """Convert NCLT Ladybug3 images to TUM-style dataset (half-res 808x616).
    Returns path to dataset dir or None on failure.
    """
    img_dir = NCLT_DATA / 'images' / session / 'lb3' / f'Cam{cam_id}'
    if not img_dir.exists():
        print(f"  No images for {session} Cam{cam_id}")
        return None

    tiff_files = sorted(glob.glob(str(img_dir / '*.tiff')))
    if not tiff_files:
        print(f"  No TIFF files found in {img_dir}")
        return None

    tiff_files = tiff_files[::subsample][:max_images]
    print(f"  Converting {len(tiff_files)} images from {session} Cam{cam_id}...")

    out_dir = Path(f'/tmp/nclt_tum_{session}')
    rgb_dir = out_dir / 'rgb'
    rgb_dir.mkdir(parents=True, exist_ok=True)

    timestamps = []
    filenames = []

    for f in tqdm(tiff_files, desc='Convert images'):
        ts_us = int(Path(f).stem)
        ts_s = ts_us / 1e6

        img = cv2.imread(f, cv2.IMREAD_COLOR)
        if img is None:
            continue

        # resize to half resolution for faster ORB feature extraction
        img = cv2.resize(img, (808, 616), interpolation=cv2.INTER_AREA)

        out_name = f'{ts_s:.6f}.png'
        cv2.imwrite(str(rgb_dir / out_name), img)

        timestamps.append(ts_s)
        filenames.append(f'rgb/{out_name}')

    # write rgb.txt
    with open(out_dir / 'rgb.txt', 'w') as fout:
        fout.write("# NCLT Ladybug3 images in TUM format (half resolution)\n")
        fout.write("# timestamp filename\n")
        fout.write("#\n")
        for ts, fn in zip(timestamps, filenames):
            fout.write(f"{ts:.6f} {fn}\n")

    print(f"  Prepared {len(timestamps)} images in {out_dir}")
    return out_dir


def run_orbslam3(dataset_dir, session):
    """Run ORB-SLAM3 monocular, returns trajectory path or None"""
    mono_tum = ORB_SLAM3_DIR / 'Examples' / 'Monocular' / 'mono_tum'
    if not mono_tum.exists():
        print(f"  ERROR: mono_tum binary not found at {mono_tum}")
        return None

    # run ORB-SLAM3 headlessly (Pangolin + Qt need display workarounds)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = f"{ORB_SLAM3_DIR}/lib:{ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib:{ORB_SLAM3_DIR}/Thirdparty/g2o/lib:/usr/local/lib:" + env.get('LD_LIBRARY_PATH', '')
    # fix Qt plugin conflict: OpenCV's bundled Qt plugins interfere with system Qt
    env['QT_QPA_PLATFORM'] = 'offscreen'
    env.pop('QT_PLUGIN_PATH', None)

    # use xvfb-run to provide a virtual X display for Pangolin's OpenGL context
    cmd = [
        'xvfb-run', '-a',
        str(mono_tum),
        str(VOCAB_PATH),
        str(CONFIG_PATH),
        str(dataset_dir),
    ]

    print(f"  Running ORB-SLAM3 on {session}...")
    print(f"  Command: {' '.join(cmd)}")

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=3600,  # 60 min timeout
            env=env,
            cwd=str(dataset_dir),
        )

        print(f"  Return code: {result.returncode}")
        if result.stdout:
            # print last 20 lines of stdout
            lines = result.stdout.strip().split('\n')
            for line in lines[-20:]:
                print(f"    {line}")

        if result.returncode != 0 and result.stderr:
            print(f"  STDERR (last 10 lines):")
            for line in result.stderr.strip().split('\n')[-10:]:
                print(f"    {line}")

        # check for output trajectory files
        for candidate in ['KeyFrameTrajectory.txt', 'CameraTrajectory.txt',
                         'FrameTrajectory_TUM_Format.txt']:
            path = dataset_dir / candidate
            if path.exists():
                print(f"  Found trajectory: {path}")
                return path

        print(f"  WARNING: No trajectory file produced")
        return None

    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT: ORB-SLAM3 exceeded 60 min")
        return None
    except Exception as e:
        print(f"  ERROR running ORB-SLAM3: {e}")
        return None


def evaluate_orbslam3(traj_path, session, output_dir):
    """Evaluate ORB-SLAM3 trajectory against ground truth"""
    # load ORB-SLAM3 trajectory (TUM format: timestamp tx ty tz qx qy qz qw)
    est_traj = np.loadtxt(str(traj_path))
    if len(est_traj) == 0:
        print(f"  Empty trajectory for {session}")
        return None

    print(f"  ORB-SLAM3 trajectory: {len(est_traj)} poses")

    # load ground truth
    gt_loader = GroundTruthLoader()
    gt_df = gt_loader.load_ground_truth(session)
    gt_all = np.column_stack([
        gt_df['utime'].values / 1e6,
        gt_df['x'].values, gt_df['y'].values, gt_df['z'].values,
        gt_df['qx'].values, gt_df['qy'].values,
        gt_df['qz'].values, gt_df['qw'].values])

    # ORB-SLAM3 monocular has scale ambiguity - we need to align
    est_synced, gt_synced = sync_trajectories(est_traj, gt_all, tolerance=0.5)

    if len(est_synced) < 10:
        print(f"  Too few synced poses: {len(est_synced)}")
        return None

    # compute scale factor (Sim(3) alignment - just scale for mono)
    est_dists = np.linalg.norm(np.diff(est_synced[:, 1:4], axis=0), axis=1)
    gt_dists = np.linalg.norm(np.diff(gt_synced[:, 1:4], axis=0), axis=1)
    scale = np.median(gt_dists) / (np.median(est_dists) + 1e-10)
    print(f"  Scale factor: {scale:.3f}")

    # apply scale
    est_aligned = est_synced.copy()
    est_aligned[:, 1:4] *= scale

    # also align origin
    est_aligned[:, 1:4] -= est_aligned[0, 1:4]
    est_aligned[:, 1:4] += gt_synced[0, 1:4]

    # compute metrics
    ate = compute_ate(est_aligned, gt_synced)
    rpe = compute_rpe(est_aligned, gt_synced)

    traj_len = np.sum(np.linalg.norm(np.diff(est_aligned[:, 1:4], axis=0), axis=1))
    gt_len = np.sum(np.linalg.norm(np.diff(gt_synced[:, 1:4], axis=0), axis=1))

    result = {
        'ate_rmse': ate['rmse'],
        'ate_mean': ate['mean'],
        'rpe_trans': rpe['trans_rmse'],
        'rpe_rot': rpe['rot_rmse'],
        'n_poses': len(est_aligned),
        'n_total_frames': len(est_traj),
        'traj_len': traj_len,
        'gt_len': gt_len,
        'scale': scale,
        'ate_errors': ate['errors'],
        'est_traj': est_aligned,
        'gt_traj': gt_synced,
    }

    # save trajectory
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_dir / 'orbslam3_trajectory.txt', est_aligned, fmt='%.6f',
              header='timestamp x y z qx qy qz qw')
    np.savetxt(output_dir / 'gt_trajectory.txt', gt_synced, fmt='%.6f',
              header='timestamp x y z qx qy qz qw')

    # plot
    fig, ax = plt.subplots(figsize=(14, 12))
    ax.plot(gt_synced[:, 1], gt_synced[:, 2], 'k-', lw=2, label='Ground Truth', alpha=0.8)
    ax.plot(est_aligned[:, 1], est_aligned[:, 2], 'b-', lw=1.5,
            label=f'ORB-SLAM3 (ATE={ate["rmse"]:.1f}m)', alpha=0.7)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title(f'ORB-SLAM3 Monocular - {session}', fontsize=14, fontweight='bold')
    ax.legend(fontsize=11); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig(output_dir / 'trajectory_orbslam3.png', dpi=150)
    plt.close()

    print(f"\n  ORB-SLAM3 Results for {session}:")
    print(f"    ATE RMSE: {ate['rmse']:.1f}m")
    print(f"    RPE Trans: {rpe['trans_rmse']:.4f} m/frame")
    print(f"    Poses: {len(est_aligned)} / {len(est_traj)} tracked")
    print(f"    Scale: {scale:.3f}")

    return result


if __name__ == '__main__':
    SESSIONS_WITH_IMAGES = ['2012-04-29', '2012-08-04']
    OUTPUT_ROOT = PROJECT_ROOT / 'results' / 'week0_orbslam3'

    results = {}

    for session in SESSIONS_WITH_IMAGES:
        print(f"\n{'='*80}")
        print(f"  ORB-SLAM3: {session}")
        print(f"{'='*80}")

        output_dir = OUTPUT_ROOT / session

        # step 1: Prepare data
        dataset_dir = prepare_tum_format(session, cam_id=0, subsample=2, max_images=5000)
        if dataset_dir is None:
            continue

        # step 2: Run ORB-SLAM3
        traj_path = run_orbslam3(dataset_dir, session)
        if traj_path is None:
            print(f"  ORB-SLAM3 failed for {session}")
            continue

        # step 3: Evaluate
        result = evaluate_orbslam3(traj_path, session, output_dir)
        if result is not None:
            results[session] = result

        # cleanup temp images
        shutil.rmtree(dataset_dir / 'rgb', ignore_errors=True)

    # summary
    if results:
        print(f"\n{'='*80}")
        print("ORB-SLAM3 SUMMARY")
        print(f"{'='*80}")
        for s, r in results.items():
            print(f"  {s}: ATE RMSE={r['ate_rmse']:.1f}m, "
                  f"RPE={r['rpe_trans']:.4f}m/f, "
                  f"Scale={r['scale']:.3f}, "
                  f"Poses={r['n_poses']}/{r['n_total_frames']}")
    else:
        print("\n  No ORB-SLAM3 results produced.")
