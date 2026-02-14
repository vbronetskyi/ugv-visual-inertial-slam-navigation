#!/usr/bin/env python3
"""
Week 0 v2: ORB-SLAM3 Monocular evaluation on NCLT.

Runs ORB-SLAM3 in pure monocular mode on NCLT Ladybug3 Camera 0 images,
then evaluates against ground truth using Sim(3) alignment.

Prerequisites:
    - ORB-SLAM3 built at /tmp/ORB_SLAM3/
    - Data prepared with prepare_orbslam3_mono_inertial.py
    - NCLT ground truth available

Usage:
    python run_week0_orbslam3_v2.py --session 2012-04-29 [--max-images 5000]
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.ground_truth_loader import GroundTruthLoader

ORBSLAM3_DIR = Path('/tmp/ORB_SLAM3')
MONO_BINARY = ORBSLAM3_DIR / 'Examples/Monocular/mono_tum_vi'
VOCAB = ORBSLAM3_DIR / 'Vocabulary/ORBvoc.txt'
CONFIG = Path(__file__).resolve().parent.parent / 'configs' / 'nclt_mono_inertial.yaml'
AGGRESSIVE_CONFIG = Path('/tmp/nclt_mi_aggressive.yaml')
RESULTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week0_orbslam3_v2'


def umeyama_alignment(source, target):
    """Sim(3) alignment (Umeyama). Returns (scale, R, t, aligned)"""
    n, dim = source.shape
    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    src_c = source - mu_s
    tgt_c = target - mu_t
    var_s = np.sum(src_c ** 2) / n
    H = (tgt_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(H)
    S = np.eye(dim)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[dim - 1, dim - 1] = -1
    R = U @ S @ Vt
    scale = np.trace(np.diag(D) @ S) / var_s
    t = mu_t - scale * R @ mu_s
    aligned = scale * (source @ R.T) + t
    return scale, R, t, aligned


def run_orbslam3(session, data_dir, output_name, config_path, timeout=600):
    """Run ORB-SLAM3 monocular on prepared data, returns trajectory path or None"""
    env = {
        'LD_LIBRARY_PATH': ':'.join([
            str(ORBSLAM3_DIR / 'lib'),
            str(ORBSLAM3_DIR / 'Thirdparty/DBoW2/lib'),
            str(ORBSLAM3_DIR / 'Thirdparty/g2o/lib'),
            '/usr/local/lib',
        ]),
        'PATH': '/usr/bin:/usr/local/bin',
        'HOME': '/root',
    }

    cmd = [
        str(MONO_BINARY),
        str(VOCAB),
        str(config_path),
        str(data_dir / 'images'),
        str(data_dir / 'timestamps.txt'),
        output_name,
    ]

    print(f"  Running: {' '.join(cmd[-4:])}")
    print(f"  Config: {config_path}")
    print(f"  Timeout: {timeout}s")

    t_start = time.time()
    result = subprocess.run(
        cmd, env=env, capture_output=True, text=True,
        timeout=timeout, cwd='/tmp'
    )
    t_elapsed = time.time() - t_start

    # save log
    log_path = RESULTS_DIR / f'orbslam3_{session}_log.txt'
    with open(log_path, 'w') as f:
        f.write(result.stdout)
        if result.stderr:
            f.write('\n--- STDERR ---\n')
            f.write(result.stderr)

    # parse key stats from log
    n_maps = 0
    n_kfs = 0
    for line in result.stdout.split('\n'):
        if 'Map' in line and 'has' in line and 'KFs' in line:
            try:
                n_kfs += int(line.split('has')[1].split('KFs')[0].strip())
                n_maps += 1
            except (ValueError, IndexError):
                pass

    print(f"  Completed in {t_elapsed:.1f}s (exit code {result.returncode})")
    print(f"  Maps: {n_maps}, Keyframes: {n_kfs}")

    # find trajectory file
    traj_file = Path('/tmp') / f'f_{output_name}.txt'
    if traj_file.exists():
        n_poses = sum(1 for _ in open(traj_file))
        print(f"  Trajectory: {traj_file} ({n_poses} poses)")
        return traj_file
    else:
        print(f"  WARNING: Trajectory file not found: {traj_file}")
        return None


def evaluate_trajectory(traj_file, session, results_dir):
    # XXX: hardcoded path, move to config
    """evaluate ORB-SLAM3 trajectory against NCLT ground truth"""
    # load SLAM trajectory
    slam_data = np.loadtxt(str(traj_file))
    slam_ts_ns = slam_data[:, 0]
    slam_pos = slam_data[:, 1:4]
    slam_ts_us = slam_ts_ns / 1000.0

    # load ground truth
    gt_loader = GroundTruthLoader()
    gt_df = gt_loader.load_ground_truth(session)
    gt_utimes = gt_df['utime'].values.astype(np.float64)
    gt_pos = gt_df[['x', 'y', 'z']].values

    # synchronize
    matched_slam = []
    matched_gt = []
    for i, t in enumerate(slam_ts_us):
        idx = np.searchsorted(gt_utimes, t)
        best_idx, best_diff = None, np.inf
        for c in [idx - 1, idx]:
            if 0 <= c < len(gt_utimes):
                d = abs(gt_utimes[c] - t)
                if d < best_diff:
                    best_diff = d
                    best_idx = c
        if best_idx is not None and best_diff <= 10000:  # 10ms
            matched_slam.append(i)
            matched_gt.append(best_idx)

    matched_slam = np.array(matched_slam)
    matched_gt = np.array(matched_gt)
    slam_m = slam_pos[matched_slam]
    gt_m = gt_pos[matched_gt]

    n_total = len(slam_data)
    n_matched = len(matched_slam)

    # sim(3) alignment
    scale, R, t, slam_aligned = umeyama_alignment(slam_m, gt_m)

    # ATE
    errors = np.linalg.norm(gt_m - slam_aligned, axis=1)
    ate_rmse = np.sqrt(np.mean(errors ** 2))
    ate_mean = np.mean(errors)
    ate_median = np.median(errors)
    ate_max = np.max(errors)

    # gT trajectory length
    gt_len = np.sum(np.linalg.norm(np.diff(gt_m, axis=0), axis=1))

    # time info
    duration = (slam_ts_ns[-1] - slam_ts_ns[0]) / 1e9

    metrics = {
        'session': session,
        'n_total': n_total,
        'n_matched': n_matched,
        'tracking_pct': n_matched / n_total * 100 if n_total > 0 else 0,
        'duration_s': duration,
        'gt_length_m': gt_len,
        'scale': scale,
        'ate_rmse': ate_rmse,
        'ate_mean': ate_mean,
        'ate_median': ate_median,
        'ate_max': ate_max,
        'ate_pct': ate_rmse / gt_len * 100 if gt_len > 0 else 0,
    }

    # plot
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))

    ax1 = axes[0]
    ax1.plot(gt_m[:, 0], gt_m[:, 1], 'b-', lw=1.2, alpha=0.8, label='Ground Truth')
    ax1.plot(slam_aligned[:, 0], slam_aligned[:, 1], 'r-', lw=1.0, alpha=0.8,
             label='ORB-SLAM3 Mono (aligned)')
    ax1.plot(gt_m[0, 0], gt_m[0, 1], 'go', ms=10, mec='k', mew=1.5, label='Start', zorder=5)
    ax1.plot(gt_m[-1, 0], gt_m[-1, 1], 'rs', ms=10, mec='k', mew=1.5, label='End', zorder=5)
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_title(f'ORB-SLAM3 Mono, {session}\n'
                  f'ATE RMSE = {ate_rmse:.1f}m | Scale = {scale:.1f} | '
                  f'{n_matched}/{n_total} poses ({metrics["tracking_pct"]:.1f}%)')
    ax1.legend()
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    ax2 = axes[1]
    times = (slam_ts_us[matched_slam] - slam_ts_us[matched_slam[0]]) / 1e6
    ax2.plot(times, errors, 'b-', lw=0.8, alpha=0.7)
    ax2.axhline(y=ate_rmse, color='r', ls='--', lw=1.5, label=f'RMSE = {ate_rmse:.1f}m')
    ax2.axhline(y=ate_mean, color='orange', ls='--', lw=1.5, label=f'Mean = {ate_mean:.1f}m')
    ax2.fill_between(times, 0, errors, alpha=0.15, color='blue')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position Error (m)')
    ax2.set_title('Absolute Trajectory Error over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = results_dir / f'trajectory_{session}.png'
    plt.savefig(str(plot_path), dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Plot saved: {plot_path}")

    return metrics


def main():
    parser = argparse.ArgumentParser(description='Week 0 v2: ORB-SLAM3 Monocular on NCLT')
    parser.add_argument('--sessions', nargs='+', default=['2012-04-29', '2012-08-04'],
                        help='NCLT sessions to evaluate')
    parser.add_argument('--max-images', type=int, default=5000,
                        help='Max images per session (0=all)')
    parser.add_argument('--timeout', type=int, default=600,
                        help='ORB-SLAM3 timeout in seconds')
    parser.add_argument('--skip-run', action='store_true',
                        help='Skip ORB-SLAM3 run, only evaluate existing trajectories')
    args = parser.parse_args()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    config_path = AGGRESSIVE_CONFIG if AGGRESSIVE_CONFIG.exists() else CONFIG

    all_metrics = []

    for session in args.sessions:
        print(f"\n{'='*70}")
        print(f"Session: {session}")
        print(f"{'='*70}")

        data_dir = Path(f'/tmp/nclt_mono_inertial_{session}')
        if not data_dir.exists():
            # try 3k variant
            data_dir = Path(f'/tmp/nclt_mi_3k')
            if not data_dir.exists():
                print(f"  ERROR: Data directory not found for {session}")
                continue

        output_name = f'nclt_mono_{session}'

        if not args.skip_run:
            print(f"\n--- Running ORB-SLAM3 ---")
            traj_file = run_orbslam3(session, data_dir, output_name, config_path, args.timeout)
        else:
            traj_file = Path('/tmp') / f'f_{output_name}.txt'
            if not traj_file.exists():
                print(f"  Trajectory not found: {traj_file}")
                continue

        if traj_file and traj_file.exists():
            print(f"\n--- Evaluating ---")
            metrics = evaluate_trajectory(traj_file, session, RESULTS_DIR)
            all_metrics.append(metrics)

            print(f"\n--- Results for {session} ---")
            print(f"  Tracking: {metrics['n_matched']}/{metrics['n_total']} "
                  f"({metrics['tracking_pct']:.1f}%)")
            print(f"  Duration: {metrics['duration_s']:.1f}s")
            print(f"  GT length: {metrics['gt_length_m']:.1f}m")
            print(f"  Scale: {metrics['scale']:.1f}")
            print(f"  ATE RMSE: {metrics['ate_rmse']:.1f}m ({metrics['ate_pct']:.1f}%)")

    # summary table
    if all_metrics:
        print(f"\n{'='*70}")
        print("COMPARISON TABLE")
        print(f"{'='*70}")
        header = f"{'Session':<15} {'Poses':>8} {'Track%':>7} {'GT(m)':>7} {'Scale':>8} {'ATE(m)':>8} {'ATE%':>6}"
        print(header)
        print('-' * len(header))
        for m in all_metrics:
            print(f"{m['session']:<15} {m['n_matched']:>8} {m['tracking_pct']:>6.1f}% "
                  f"{m['gt_length_m']:>7.1f} {m['scale']:>8.1f} "
                  f"{m['ate_rmse']:>8.1f} {m['ate_pct']:>5.1f}%")

        # save metrics
        metrics_path = RESULTS_DIR / 'metrics.txt'
        with open(metrics_path, 'w') as f:
            f.write("ORB-SLAM3 Monocular Evaluation Results\n")
            f.write(f"Config: {config_path}\n")
            f.write(f"{'='*60}\n")
            for m in all_metrics:
                f.write(f"\nSession: {m['session']}\n")
                for k, v in m.items():
                    if k != 'session':
                        f.write(f"  {k}: {v}\n")
        print(f"\nMetrics saved: {metrics_path}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
