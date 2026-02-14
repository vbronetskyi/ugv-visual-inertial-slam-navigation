#!/usr/bin/env python3
"""Experiment 0.7: hloc cross-season visual localization on NCLT.

Build 3D map from spring Cam5 images (2012-04-29),
localize summer Cam5 images (2012-08-04) against that map.

hloc full run takes ~12h end-to-end, feature extraction is the bottleneck.
run in tmux, check back next morning
Tests seasonal robustness of visual localization on NCLT.

Pipeline: SuperPoint → NetVLAD retrieval → LightGlue matching → COLMAP SfM → PnP localization.

Usage:
    python3 scripts/run_week0_hloc.py           # Full pipeline
    python3 scripts/run_week0_hloc.py --sanity   # Quick 10-image sanity test
    python3 scripts/run_week0_hloc.py --small    # Small subset (every 10th/20th)
"""

import argparse
import json
import os
import pickle
import shutil
import sys
import time
from pathlib import Path

import cv2
import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

# paths
PROJECT = Path('/workspace/datasets/nclt')
NCLT_DATA = Path('/workspace/data/nclt')
RESULTS = PROJECT / 'results' / 'week0_hloc'
GT_DIR = NCLT_DATA / 'ground_truth'

SPRING = '2012-04-29'
SUMMER = '2012-08-04'
IMAGES_ROOT = NCLT_DATA / 'images_prepared'  # hloc image_dir
SPRING_IMAGES = IMAGES_ROOT / SPRING
SUMMER_IMAGES = IMAGES_ROOT / SUMMER


# ground truth utilities

def load_gt(session):
    """Load NCLT ground truth as (N, 7) array: utime, x, y, z, roll, pitch, yaw"""
    path = GT_DIR / f'groundtruth_{session}.csv'
    data = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) >= 7:
                data.append([float(p.strip()) for p in parts[:7]])
    return np.array(data)


def gt_pose_at(gt, utime):
    """Interpolate GT pose (x, y, z, roll, pitch, yaw) at given utime"""
    idx = np.searchsorted(gt[:, 0], utime)
    if idx == 0:
        return gt[0, 1:]
    if idx >= len(gt):
        return gt[-1, 1:]
    t0, t1 = gt[idx - 1, 0], gt[idx, 0]
    alpha = (utime - t0) / (t1 - t0) if t1 != t0 else 0.0
    return gt[idx - 1, 1:] + alpha * (gt[idx, 1:] - gt[idx - 1, 1:])


def xyzrpy_to_4x4(x, y, z, roll, pitch, yaw):
    R = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def poses_to_qtvec(T):
    """4x4 pose to (qw, qx, qy, qz, tx, ty, tz)"""
    R = Rotation.from_matrix(T[:3, :3])
    q = R.as_quat()  # [qx, qy, qz, qw]
    return np.array([q[3], q[0], q[1], q[2], T[0, 3], T[1, 3], T[2, 3]])


# image selection

def get_image_list(image_dir, every_nth=1):
    """Get sorted list of image filenames, subsampled every nth"""
    all_imgs = sorted([f.name for f in image_dir.iterdir() if f.suffix == '.png'])
    return all_imgs[::every_nth]


def utime_from_filename(fname):
    """Microsecond timestamp from filename like '1335704127712909.png'"""
    return int(Path(fname).stem)


# data preparation

def prepare_images(output_dir, spring_every, summer_every,
                    spring_max=None, summer_max=None):
    """Select image subsets for hloc. Uses original image directories directly.

    Image paths are relative to IMAGES_ROOT (nclt_data/images_prepared/):
        2012-04-29/xxx.png  (spring/db)
        2012-08-04/xxx.png  (summer/query)
    """
    spring_imgs = get_image_list(SPRING_IMAGES, spring_every)
    summer_imgs = get_image_list(SUMMER_IMAGES, summer_every)

    if spring_max is not None:
        spring_imgs = spring_imgs[:spring_max]
    if summer_max is not None:
        summer_imgs = summer_imgs[:summer_max]

    print(f'  Spring (db): {len(spring_imgs)} images (every {spring_every}th, max={spring_max})')
    print(f'  Summer (query): {len(summer_imgs)} images (every {summer_every}th, max={summer_max})')

    db_list = [f'{SPRING}/{img}' for img in spring_imgs]
    query_list = [f'{SUMMER}/{img}' for img in summer_imgs]

    return db_list, query_list


def prepare_gt_poses(db_list, query_list, output_dir):
    """Sync GT poses with image timestamps and save"""
    gt_spring = load_gt(SPRING)
    gt_summer = load_gt(SUMMER)

    gt_poses = {}
    for img_path in db_list + query_list:
        fname = Path(img_path).name
        utime = utime_from_filename(fname)
        session = SPRING if img_path.startswith('db/') else SUMMER
        gt = gt_spring if session == SPRING else gt_summer
        pose = gt_pose_at(gt, utime)
        T = xyzrpy_to_4x4(*pose)
        gt_poses[img_path] = T

    # save GT in COLMAP-like format
    gt_file = output_dir / 'gt_poses.txt'
    with open(gt_file, 'w') as f:
        f.write('# image_name qw qx qy qz tx ty tz\n')
        for name, T in gt_poses.items():
            q = poses_to_qtvec(T)
            f.write(f'{name} {q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f} '
                    f'{q[4]:.6f} {q[5]:.6f} {q[6]:.6f}\n')

    return gt_poses


# core hloc pipeline

def run_sfm_pipeline(image_dir, db_list, outputs, feature_conf_name='superpoint_aachen',
                     matcher_conf_name='superpoint+lightglue',
                     retrieval_conf_name='netvlad', num_retrieval=20):
    """Build SfM map from database images.

    Returns pycolmap.Reconstruction or None on failure.
    """
    from hloc import extract_features, match_features, reconstruction
    from hloc import pairs_from_retrieval, pairs_from_exhaustive

    feature_conf = extract_features.confs[feature_conf_name]
    matcher_conf = match_features.confs[matcher_conf_name]

    features_path = outputs / 'features.h5'
    sfm_pairs = outputs / 'pairs-sfm.txt'
    sfm_matches = outputs / 'matches-sfm.h5'
    sfm_dir = outputs / 'sfm'

    # 1. Extract local features (SuperPoint)
    t0 = time.time()
    print(f'\n  [SfM 1/4] Extracting features ({feature_conf_name})...')
    feature_path = extract_features.main(
        feature_conf, image_dir,
        image_list=db_list,
        feature_path=features_path,
        overwrite=False,
    )
    print(f'    Done in {time.time() - t0:.1f}s')

    # 2. Generate pairs via retrieval
    t0 = time.time()
    n_db = len(db_list)
    if n_db <= 100:
        # small dataset: exhaustive pairs
        print(f'  [SfM 2/4] Generating exhaustive pairs (n={n_db})...')
        pairs_from_exhaustive.main(sfm_pairs, image_list=db_list)
    else:
        # large dataset: retrieval-based pairs
        print(f'  [SfM 2/4] Extracting global features ({retrieval_conf_name}) + retrieval...')
        retrieval_conf = extract_features.confs[retrieval_conf_name]
        global_features = extract_features.main(
            retrieval_conf, image_dir,
            image_list=db_list,
            feature_path=outputs / 'global-feats-db.h5',
            overwrite=False,
        )
        pairs_from_retrieval.main(
            global_features, sfm_pairs,
            num_matched=num_retrieval,
            db_list=db_list,
        )
    print(f'    Done in {time.time() - t0:.1f}s')

    # count pairs
    n_pairs = sum(1 for _ in open(sfm_pairs))
    print(f'    Generated {n_pairs} pairs')

    # 3. Match features (LightGlue)
    t0 = time.time()
    print(f'  [SfM 3/4] Matching features ({matcher_conf_name})...')
    match_path = match_features.main(
        matcher_conf, sfm_pairs,
        features=feature_path,
        matches=sfm_matches,
        overwrite=False,
    )
    print(f'    Done in {time.time() - t0:.1f}s')

    # 4. Reconstruct
    t0 = time.time()
    print(f'  [SfM 4/4] Running COLMAP SfM...')
    import pycolmap
    model = reconstruction.main(
        sfm_dir, image_dir, sfm_pairs, feature_path, match_path,
        camera_mode=pycolmap.CameraMode.SINGLE,
        verbose=False,
        image_list=db_list,
        image_options=dict(
            camera_model='OPENCV_FISHEYE',
        ),
        mapper_options=dict(
            min_num_matches=15,
            ba_global_max_num_iterations=50,
        ),
    )
    dt = time.time() - t0
    print(f'    Done in {dt:.1f}s')

    if model is not None:
        n_images = model.num_reg_images()
        n_points = model.num_points3D()
        print(f'    Registered: {n_images}/{len(db_list)} images '
              f'({100 * n_images / len(db_list):.1f}%)')
        print(f'    3D points: {n_points}')
        me = model.compute_mean_reprojection_error()
        print(f'    Mean reproj error: {me:.3f} px')
    else:
        print('    SfM FAILED, no reconstruction')

    return model, feature_path


def create_query_list_file(query_list, output_path, width=808, height=616):
    """Create query list file with camera intrinsics for hloc localize_sfm.

    Format: image_name CAMERA_MODEL width height param1 param2 ...
    Using OPENCV_FISHEYE model for NCLT Ladybug3 Cam5 (~120 deg FOV).
    """
    # OPENCV_FISHEYE: fx, fy, cx, cy, k1, k2, k3, k4
    # approximate parameters for NCLT Cam5 (half-res 808x616)
    fx = fy = 290.0
    cx = width / 2.0
    cy = height / 2.0
    k1 = k2 = k3 = k4 = 0.0  # let COLMAP refine

    with open(output_path, 'w') as fout:
        for qname in query_list:
            fout.write(f'{qname} OPENCV_FISHEYE {width} {height} '
                       f'{fx} {fy} {cx} {cy} {k1} {k2} {k3} {k4}\n')

    return output_path


def run_localization(image_dir, model, query_list, db_list, feature_path,
                     outputs, matcher_conf_name='superpoint+lightglue',
                     retrieval_conf_name='netvlad', num_retrieval=20):
    """Localize query images against SfM model.

    Returns dict of {image_name: 4x4 pose matrix} for successfully localized images.
    """
    from hloc import extract_features, match_features, localize_sfm
    from hloc import pairs_from_retrieval, pairs_from_exhaustive

    matcher_conf = match_features.confs[matcher_conf_name]

    loc_pairs = outputs / 'pairs-loc.txt'
    loc_matches = outputs / 'matches-loc.h5'
    loc_results = outputs / 'localization_results.txt'
    query_list_file = outputs / 'queries_with_intrinsics.txt'

    # 1. Extract features for query images (append to existing h5)
    t0 = time.time()
    print(f'\n  [Loc 1/5] Extracting query features...')
    feature_conf = extract_features.confs['superpoint_aachen']
    extract_features.main(
        feature_conf, image_dir,
        image_list=query_list,
        feature_path=feature_path,
        overwrite=False,
    )
    print(f'    Done in {time.time() - t0:.1f}s')

    # 2. Generate localization pairs
    t0 = time.time()
    if len(db_list) <= 100:
        print(f'  [Loc 2/5] Generating exhaustive loc pairs...')
        pairs_from_exhaustive.main(
            loc_pairs, image_list=query_list, ref_list=db_list,
        )
    else:
        print(f'  [Loc 2/5] Retrieval-based loc pairs...')
        retrieval_conf = extract_features.confs[retrieval_conf_name]
        # extract global features for queries
        global_feats_query = extract_features.main(
            retrieval_conf, image_dir,
            image_list=query_list,
            feature_path=outputs / 'global-feats-query.h5',
            overwrite=False,
        )
        # also need db global features
        global_feats_db = outputs / 'global-feats-db.h5'
        if not global_feats_db.exists():
            global_feats_db = extract_features.main(
                retrieval_conf, image_dir,
                image_list=db_list,
                feature_path=global_feats_db,
                overwrite=False,
            )
        pairs_from_retrieval.main(
            global_feats_query, loc_pairs,
            num_matched=num_retrieval,
            db_descriptors=global_feats_db,
            db_list=db_list,
        )
    n_pairs = sum(1 for _ in open(loc_pairs))
    print(f'    Generated {n_pairs} loc pairs in {time.time() - t0:.1f}s')

    # 3. Match features
    t0 = time.time()
    print(f'  [Loc 3/5] Matching query features ({matcher_conf_name})...')
    match_path = match_features.main(
        matcher_conf, loc_pairs,
        features=feature_path,
        matches=loc_matches,
        overwrite=False,
    )
    print(f'    Done in {time.time() - t0:.1f}s')

    # 4. Create query list file with intrinsics
    print(f'  [Loc 4/5] Creating query intrinsics file...')
    create_query_list_file(query_list, query_list_file)

    # 5. Localize
    t0 = time.time()
    print(f'  [Loc 5/5] Running PnP localization...')
    localize_sfm.main(
        model, query_list_file,
        loc_pairs, feature_path, match_path,
        loc_results,
        ransac_thresh=12,
        covisibility_clustering=False,
    )
    print(f'    Done in {time.time() - t0:.1f}s')

    # parse results
    poses = {}
    if loc_results.exists():
        with open(loc_results) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) < 8:
                    continue
                name = parts[0]
                qw, qx, qy, qz = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
                tx, ty, tz = float(parts[5]), float(parts[6]), float(parts[7])
                R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = [tx, ty, tz]
                poses[name] = T

    print(f'    Localized: {len(poses)}/{len(query_list)} query images '
          f'({100 * len(poses) / max(len(query_list), 1):.1f}%)')

    return poses


# evaluation

def align_trajectories_sim3(est_xyz, gt_xyz):
    """Sim(3) alignment (Umeyama). Returns aligned positions and scale"""
    n = est_xyz.shape[0]
    mu_s = est_xyz.mean(0)
    mu_t = gt_xyz.mean(0)
    sc = est_xyz - mu_s
    tc = gt_xyz - mu_t
    var_s = np.sum(sc ** 2) / n
    cov = tc.T @ sc / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / var_s if var_s > 1e-10 else 1.0
    t = mu_t - s * R @ mu_s
    aligned = (s * (R @ est_xyz.T).T) + t
    return aligned, s, R, t


def evaluate_localization(loc_poses, gt_poses, query_list, output_dir):
    """Evaluate localization accuracy.

    Computes standard visual localization metrics:
    - Percentage within (0.25m, 2°), (0.5m, 5°), (5m, 10°)
    - Median/mean translation and rotation errors
    - ATE RMSE
    """
    trans_errors = []
    rot_errors = []
    loc_xyz = []
    gt_xyz = []
    loc_names = []

    for qname in query_list:
        # loc_poses keys might be just the filename without 'query/' prefix
        fname = Path(qname).name
        loc_key = None
        if qname in loc_poses:
            loc_key = qname
        elif fname in loc_poses:
            loc_key = fname

        if loc_key is None:
            continue

        gt_key = qname
        if gt_key not in gt_poses:
            continue

        T_est = loc_poses[loc_key]  # cam_from_world
        T_gt = gt_poses[gt_key]     # world_from_body

        # t_est from hloc is cam_from_world, T_gt is world_from_body
        # translation of camera in world = inverse of cam_from_world
        T_est_inv = np.linalg.inv(T_est)
        est_pos = T_est_inv[:3, 3]
        gt_pos = T_gt[:3, 3]

        loc_xyz.append(est_pos)
        gt_xyz.append(gt_pos)
        loc_names.append(qname)

        trans_err = np.linalg.norm(est_pos - gt_pos)
        R_diff = T_est_inv[:3, :3] @ T_gt[:3, :3].T
        rot_err = np.degrees(np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1)))

        trans_errors.append(trans_err)
        rot_errors.append(rot_err)

    trans_errors = np.array(trans_errors)
    rot_errors = np.array(rot_errors)
    loc_xyz = np.array(loc_xyz)
    gt_xyz = np.array(gt_xyz)

    n_total = len(query_list)
    n_loc = len(trans_errors)

    results = {
        'n_query': n_total,
        'n_localized': n_loc,
        'pct_localized': 100 * n_loc / max(n_total, 1),
    }

    if n_loc == 0:
        print('  WARNING: No images were localized!')
        results['status'] = 'no_localization'
        return results

    # standard thresholds
    thresholds = [(0.25, 2), (0.5, 5), (1.0, 5), (5.0, 10)]
    for t_thresh, r_thresh in thresholds:
        pct = 100 * np.mean((trans_errors < t_thresh) & (rot_errors < r_thresh))
        key = f'pct_{t_thresh}m_{r_thresh}deg'
        results[key] = pct
        print(f'    Within ({t_thresh}m, {r_thresh}°): {pct:.1f}%')

    results['median_trans_error_m'] = float(np.median(trans_errors))
    results['mean_trans_error_m'] = float(np.mean(trans_errors))
    results['median_rot_error_deg'] = float(np.median(rot_errors))
    results['mean_rot_error_deg'] = float(np.mean(rot_errors))
    print(f'    Median trans error: {results["median_trans_error_m"]:.2f} m')
    print(f'    Mean trans error: {results["mean_trans_error_m"]:.2f} m')
    print(f'    Median rot error: {results['median_rot_error_deg']:.2f}°')

    # sim(3) aligned ATE
    if n_loc >= 3:
        aligned, scale, _, _ = align_trajectories_sim3(loc_xyz, gt_xyz)
        ate_errors = np.linalg.norm(aligned - gt_xyz, axis=1)
        results['ate_rmse_sim3'] = float(np.sqrt(np.mean(ate_errors ** 2)))
        results['ate_mean_sim3'] = float(np.mean(ate_errors))
        results['sim3_scale'] = float(scale)
        print(f'    ATE RMSE (Sim3): {results["ate_rmse_sim3"]:.2f} m')
        print(f'    Sim(3) scale: {scale:.4f}')

        # also unaligned ATE
        ate_raw = np.linalg.norm(loc_xyz - gt_xyz, axis=1)
        results['ate_rmse_raw'] = float(np.sqrt(np.mean(ate_raw ** 2)))

    results['status'] = 'ok'

    # save per-image errors
    err_file = output_dir / 'per_image_errors.json'
    per_img = []
    for i, name in enumerate(loc_names):
        per_img.append({
            'image': name,
            'trans_error_m': float(trans_errors[i]),
            'rot_error_deg': float(rot_errors[i]),
            'est_xyz': loc_xyz[i].tolist(),
            'gt_xyz': gt_xyz[i].tolist(),
        })
    with open(err_file, 'w') as f:
        json.dump(per_img, f, indent=2)

    return results


def evaluate_sfm_map(model, gt_poses, db_list, output_dir):
    """Evaluate SfM reconstruction quality against GT"""
    if model is None:
        return {'status': 'no_model'}

    sfm_xyz = []
    gt_xyz = []
    registered = []

    for img_name in db_list:
        image = model.find_image_with_name(img_name)
        if image is None:
            continue
        # sfM gives cam_from_world, invert to get camera position in world
        cam_from_world = image.cam_from_world
        if callable(cam_from_world):
            cam_from_world = cam_from_world()
        R = cam_from_world.rotation.matrix()
        t = cam_from_world.translation
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        T_inv = np.linalg.inv(T)
        sfm_xyz.append(T_inv[:3, 3])

        if img_name in gt_poses:
            gt_xyz.append(gt_poses[img_name][:3, 3])
            registered.append(img_name)

    sfm_xyz = np.array(sfm_xyz)
    gt_xyz = np.array(gt_xyz)

    results = {
        'n_registered': model.num_reg_images(),
        'n_total': len(db_list),
        'pct_registered': 100 * model.num_reg_images() / max(len(db_list), 1),
        'n_3d_points': model.num_points3D(),
        'mean_reproj_error': model.compute_mean_reprojection_error(),
    }

    if len(sfm_xyz) >= 3 and len(gt_xyz) >= 3:
        aligned, scale, _, _ = align_trajectories_sim3(sfm_xyz, gt_xyz)
        ate = np.linalg.norm(aligned - gt_xyz, axis=1)
        results['sfm_ate_rmse'] = float(np.sqrt(np.mean(ate ** 2)))
        results['sfm_sim3_scale'] = float(scale)
        print(f'    SfM ATE RMSE (Sim3): {results["sfm_ate_rmse"]:.2f} m')
        print(f'    SfM Sim(3) scale: {scale:.4f}')

        np.savez(output_dir / 'sfm_trajectory.npz',
                 sfm_aligned=aligned, gt=gt_xyz, sfm_raw=sfm_xyz)

    return results


# plotting

def plot_sfm_vs_gt(output_dir):
    # NOTE: not thread-safe but we run single threaded anyway
    """Plot SfM trajectory aligned to GT"""
    data = np.load(output_dir / 'sfm_trajectory.npz')
    sfm = data['sfm_aligned']
    gt = data['gt']

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.plot(gt[:, 0], gt[:, 1], 'k-', linewidth=2, label='Ground Truth (Spring)', alpha=0.7)
    ax.plot(sfm[:, 0], sfm[:, 1], '-', color='#2196F3', linewidth=1.5,
            label='SfM reconstruction (Sim3 aligned)', alpha=0.8)
    ax.plot(gt[0, 0], gt[0, 1], 'go', markersize=12, zorder=10, label='Start')
    ax.plot(gt[-1, 0], gt[-1, 1], 'rs', markersize=10, zorder=10, label='End')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('SfM Map (Spring) vs Ground Truth', fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(output_dir / 'spring_sfm_vs_gt.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_dir / 'spring_sfm_vs_gt.png'}')


def plot_localization_vs_gt(loc_poses, gt_poses, query_list, output_dir):
    loc_xyz, gt_xyz = [], []
    for qname in query_list:
        fname = Path(qname).name
        loc_key = qname if qname in loc_poses else (fname if fname in loc_poses else None)
        if loc_key is None or qname not in gt_poses:
            continue
        T_est_inv = np.linalg.inv(loc_poses[loc_key])
        loc_xyz.append(T_est_inv[:3, 3])
        gt_xyz.append(gt_poses[qname][:3, 3])

    if len(loc_xyz) < 3:
        print('  Not enough localized images for trajectory plot')
        return

    loc_xyz = np.array(loc_xyz)
    gt_xyz = np.array(gt_xyz)
    aligned, scale, _, _ = align_trajectories_sim3(loc_xyz, gt_xyz)

    # also load spring GT for context
    gt_spring = load_gt(SPRING)

    fig, axes = plt.subplots(1, 2, figsize=(20, 10))

    # panel 1: Full trajectories
    ax = axes[0]
    ax.plot(gt_spring[:, 1], gt_spring[:, 2], '-', color='#BDBDBD', linewidth=1,
            label='Spring GT (reference map)', alpha=0.5)
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], 'k-', linewidth=2, label='Summer GT')
    ax.plot(aligned[:, 0], aligned[:, 1], '-', color='#E91E63', linewidth=1.5,
            label=f'Summer localized (Sim3, scale={scale:.3f})', alpha=0.8)
    ax.plot(gt_xyz[0, 0], gt_xyz[0, 1], 'go', markersize=12, zorder=10)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Cross-Season Localization: Summer vs Spring Map',
                 fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    # panel 2: Error over trajectory
    ax = axes[1]
    errors = np.linalg.norm(aligned - gt_xyz, axis=1)
    ax.plot(range(len(errors)), errors, '-', color='#E91E63', linewidth=1)
    ax.axhline(np.median(errors), color='k', linestyle='--', alpha=0.5,
               label=f'Median: {np.median(errors):.1f} m')
    ax.axhline(np.sqrt(np.mean(errors ** 2)), color='r', linestyle='--', alpha=0.5,
               label=f'RMSE: {np.sqrt(np.mean(errors**2)):.1f} m')
    ax.set_xlabel('Image index', fontsize=12)
    ax.set_ylabel('Position error (m)', fontsize=12)
    ax.set_title('Localization Error Along Trajectory', fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(output_dir / 'summer_localization_vs_gt.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_dir / "summer_localization_vs_gt.png"}')


def plot_localization_success(results, output_dir):
    """Plot localization success rate at different thresholds"""
    thresholds_t = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 25.0, 50.0]
    thresholds_r = [1, 2, 5, 10, 20, 45, 90]

    # load per-image errors
    err_file = output_dir / 'per_image_errors.json'
    if not err_file.exists():
        return
    with open(err_file) as f:
        per_img = json.load(f)

    trans_errors = np.array([e['trans_error_m'] for e in per_img])
    rot_errors = np.array([e['rot_error_deg'] for e in per_img])

    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    # translation CDF
    ax = axes[0]
    pcts = [100 * np.mean(trans_errors < t) for t in thresholds_t]
    ax.semilogx(thresholds_t, pcts, 'o-', color='#2196F3', linewidth=2, markersize=8)
    ax.set_xlabel('Translation threshold (m)', fontsize=12)
    ax.set_ylabel('% localized', fontsize=12)
    ax.set_title('Localization vs Translation Threshold', fontsize=13, fontweight='bold')
    ax.set_ylim([0, 105])
    ax.grid(True, alpha=0.3)
    for t, p in zip(thresholds_t, pcts):
        if p > 5:
            ax.annotate(f'{p:.0f}%', (t, p), textcoords="offset points",
                       xytext=(0, 10), ha='center', fontsize=9)

    # rotation CDF
    ax = axes[1]
    pcts = [100 * np.mean(rot_errors < r) for r in thresholds_r]
    ax.plot(thresholds_r, pcts, 'o-', color='#FF9800', linewidth=2, markersize=8)
    ax.set_xlabel('Rotation threshold (deg)', fontsize=12)
    ax.set_ylabel('% localized', fontsize=12)
    ax.set_title('Localization vs Rotation Threshold', fontsize=13, fontweight='bold')
    ax.set_ylim([0, 105])
    ax.grid(True, alpha=0.3)
    for r, p in zip(thresholds_r, pcts):
        if p > 5:
            ax.annotate(f'{p:.0f}%', (r, p), textcoords="offset points",
                       xytext=(0, 10), ha='center', fontsize=9)

    plt.tight_layout()
    fig.savefig(output_dir / 'localization_success_rate.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_dir / "localization_success_rate.png"}')


def plot_failure_map(loc_poses, gt_poses, query_list, output_dir):
    """Plot where localization succeeds/fails on the trajectory"""
    success_xy, fail_xy = [], []
    gt_spring = load_gt(SPRING)

    for qname in query_list:
        if qname not in gt_poses:
            continue
        gt_pos = gt_poses[qname][:3, 3]
        fname = Path(qname).name
        loc_key = qname if qname in loc_poses else (fname if fname in loc_poses else None)
        if loc_key is not None:
            success_xy.append(gt_pos[:2])
        else:
            fail_xy.append(gt_pos[:2])

    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    ax.plot(gt_spring[:, 1], gt_spring[:, 2], '-', color='#BDBDBD', linewidth=0.5,
            label='Spring route', alpha=0.4)

    if fail_xy:
        fail_xy = np.array(fail_xy)
        ax.scatter(fail_xy[:, 0], fail_xy[:, 1], c='red', s=8, alpha=0.5,
                  label=f'Failed ({len(fail_xy)})', zorder=5)
    if success_xy:
        success_xy = np.array(success_xy)
        ax.scatter(success_xy[:, 0], success_xy[:, 1], c='green', s=8, alpha=0.5,
                  label=f'Localized ({len(success_xy)})', zorder=6)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Localization Success/Failure Map (Summer queries)',
                 fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(output_dir / 'failure_map.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_dir / "failure_map.png"}')


def plot_comparison_all_methods(eval_results, output_dir):
    """Bar chart comparing hloc with all previous methods"""
    methods = [
        ('LiDAR ICP+GPS\n(Exp 0.1)', 174.0, 188.2, 100, 100),
        ('ORB-SLAM3 mono\n(Exp 0.3)', 45.6, None, 24, 0),
        ('DROID-SLAM\n(Exp 0.4)', 110.0, 142.3, 60, 54),
        ('DPVO\n(Exp 0.4)', 142.0, 183.4, 100, 90),
        ('DPV-SLAM\n(Exp 0.4)', 166.2, 193.6, 97, 92),
        ('ORB-SLAM3 VIO\n(Exp 0.6)', 69.0, None, 90, 0),
    ]

    # add hloc result
    hloc_ate = eval_results.get('ate_rmse_sim3')
    hloc_pct = eval_results.get('pct_localized', 0)
    if hloc_ate is not None:
        methods.append((f'hloc\n(Exp 0.7)', None, hloc_ate, 0, hloc_pct))

    fig, axes = plt.subplots(1, 2, figsize=(18, 7))

    # ATE comparison (summer where available, spring otherwise)
    ax = axes[0]
    names = [m[0] for m in methods]
    ates = []
    for m in methods:
        if m[2] is not None:
            ates.append(m[2])
        elif m[1] is not None:
            ates.append(m[1])
        else:
            ates.append(0)
    colors = ['#4CAF50', '#FF9800', '#2196F3', '#2196F3', '#2196F3', '#E91E63']
    if len(methods) > 6:
        colors.append('#9C27B0')
    bars = ax.bar(range(len(names)), ates, color=colors[:len(names)], alpha=0.8)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, fontsize=9)
    ax.set_ylabel('ATE RMSE (m)', fontsize=12)
    ax.set_title('ATE Comparison Across All Methods', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    for bar, ate in zip(bars, ates):
        if ate > 0:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 2,
                   f'{ate:.0f}m', ha='center', fontsize=10, fontweight='bold')

    # coverage comparison
    ax = axes[1]
    coverages = []
    for m in methods:
        # use summer coverage where available
        c = m[4] if m[4] > 0 else m[3]
        coverages.append(c)
    bars = ax.bar(range(len(names)), coverages, color=colors[:len(names)], alpha=0.8)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, fontsize=9)
    ax.set_ylabel('Coverage / Localization %', fontsize=12)
    ax.set_title('Trajectory Coverage Comparison', fontsize=14, fontweight='bold')
    ax.set_ylim([0, 110])
    ax.grid(True, alpha=0.3, axis='y')
    for bar, c in zip(bars, coverages):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
               f'{c:.0f}%', ha='center', fontsize=10, fontweight='bold')

    plt.tight_layout()
    fig.savefig(output_dir / 'comparison_all_methods.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_dir / "comparison_all_methods.png"}')


# ablation: feature density

def run_ablation_features(image_dir, db_list, query_list, gt_poses, base_outputs):
    """Test different max_keypoints values"""
    from hloc import extract_features, match_features, reconstruction
    from hloc import pairs_from_retrieval, localize_sfm

    keypoint_counts = [1024, 2048, 4096, 8192]
    ablation_results = {}
    ablation_dir = base_outputs / 'ablation_features'
    ablation_dir.mkdir(parents=True, exist_ok=True)

    for nkp in keypoint_counts:
        print(f'\n{"="*60}')
        print(f'ABLATION: max_keypoints = {nkp}')
        print(f'{"="*60}')

        out = ablation_dir / f'kp{nkp}'
        out.mkdir(parents=True, exist_ok=True)

        feature_conf = {
            'output': f'feats-superpoint-n{nkp}-r1024',
            'model': {
                'name': 'superpoint',
                'nms_radius': 3,
                'max_keypoints': nkp,
            },
            'preprocessing': {
                'grayscale': True,
                'resize_max': 1024,
            },
        }

        try:
            # extract features
            feature_path = extract_features.main(
                feature_conf, image_dir,
                image_list=db_list + query_list,
                feature_path=out / 'features.h5',
                overwrite=True,
            )

            # use existing pairs if available
            sfm_pairs = base_outputs / 'pairs-sfm.txt'
            loc_pairs = base_outputs / 'pairs-loc.txt'

            if not sfm_pairs.exists():
                print(f'  Skipping ablation kp={nkp}: no SfM pairs')
                continue

            # match SfM
            matcher_conf = match_features.confs['superpoint+lightglue']
            sfm_matches = match_features.main(
                matcher_conf, sfm_pairs,
                features=feature_path,
                matches=out / 'matches-sfm.h5',
                overwrite=True,
            )

            # reconstruct
            import pycolmap
            model = reconstruction.main(
                out / 'sfm', image_dir, sfm_pairs, feature_path, sfm_matches,
                camera_mode=pycolmap.CameraMode.SINGLE,
                verbose=False,
            )

            if model is None:
                ablation_results[nkp] = {'status': 'sfm_failed'}
                continue

            # match localization
            if loc_pairs.exists():
                loc_matches = match_features.main(
                    matcher_conf, loc_pairs,
                    features=feature_path,
                    matches=out / 'matches-loc.h5',
                    overwrite=True,
                )

                loc_results_file = out / 'loc_results.txt'
                localize_sfm.main(
                    model, image_dir / 'query',
                    loc_pairs, feature_path, loc_matches,
                    loc_results_file,
                    ransac_thresh=12,
                )

                # parse and evaluate
                loc_poses = {}
                with open(loc_results_file) as f:
                    for line in f:
                        parts = line.strip().split()
                        if len(parts) < 8 or parts[0].startswith('#'):
                            continue
                        name = parts[0]
                        qw, qx, qy, qz = [float(x) for x in parts[1:5]]
                        tx, ty, tz = [float(x) for x in parts[5:8]]
                        R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                        T = np.eye(4)
                        T[:3, :3] = R
                        T[:3, 3] = [tx, ty, tz]
                        loc_poses[name] = T

                res = evaluate_localization(loc_poses, gt_poses, query_list, out)
                res['n_registered_sfm'] = model.num_reg_images()
                ablation_results[nkp] = res

        except Exception as e:
            print(f'  ERROR in ablation kp={nkp}: {e}')
            ablation_results[nkp] = {'status': 'error', 'error': str(e)}

    # save and plot
    with open(ablation_dir / 'results.json', 'w') as f:
        json.dump({str(k): v for k, v in ablation_results.items()}, f, indent=2)

    if len(ablation_results) > 1:
        plot_ablation_features(ablation_results, ablation_dir)

    return ablation_results


def plot_ablation_features(results, output_dir):
    """Plot feature density ablation results"""
    kps = sorted([k for k in results if isinstance(results[k], dict) and results[k].get('status') == 'ok'])
    if len(kps) < 2:
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # ATE vs keypoints
    ax = axes[0]
    ates = [results[k].get('ate_rmse_sim3', 0) for k in kps]
    ax.plot(kps, ates, 'o-', color='#2196F3', linewidth=2, markersize=10)
    ax.set_xlabel('Max keypoints', fontsize=12)
    ax.set_ylabel('ATE RMSE (m)', fontsize=12)
    ax.set_title('ATE vs Feature Density', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # localization % vs keypoints
    ax = axes[1]
    pcts = [results[k].get('pct_localized', 0) for k in kps]
    ax.plot(kps, pcts, 'o-', color='#4CAF50', linewidth=2, markersize=10)
    ax.set_xlabel('Max keypoints', fontsize=12)
    ax.set_ylabel('% localized', fontsize=12)
    ax.set_title('Localization Rate vs Feature Density', fontsize=13, fontweight='bold')
    ax.set_ylim([0, 105])
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(output_dir / 'feature_density_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_dir / "feature_density_comparison.png"}')


# main experiment

def run_experiment(mode='full'):
    """Run the full hloc experiment. mode: 'sanity', 'small', or 'full'"""
    print('EXPERIMENT 0.7: hloc Visual Localization, Cross-Season on NCLT')
    print(f'Mode: {mode}')
    print(f'GPU: {os.popen("nvidia-smi --query-gpu=name --format=csv,noheader").read().strip()}')
    t_start = time.time()

    # ── Configuration ──
    if mode == 'sanity':
        spring_every, summer_every = 1, 1  # consecutive, limited below
        spring_max, summer_max = 50, 50    # 50 images each (~10 seconds)
        num_retrieval = 5
    elif mode == 'small':
        spring_every, summer_every = 10, 20  # ~2150 / ~1200
        spring_max, summer_max = None, None
        num_retrieval = 20
    else:  # full
        spring_every, summer_every = 3, 5  # ~7170 / ~4828
        spring_max, summer_max = None, None
        num_retrieval = 20

    outputs = RESULTS / mode
    outputs.mkdir(parents=True, exist_ok=True)
    image_dir = IMAGES_ROOT  # Use original image dirs, no symlinks

    # phase B: data preparation
    print(f'\n{"="*60}')
    print('PHASE B: Data Preparation')
    print(f'{"="*60}')

    t0 = time.time()
    db_list, query_list = prepare_images(
        outputs, spring_every, summer_every, spring_max, summer_max)
    gt_poses = prepare_gt_poses(db_list, query_list, outputs)
    print(f'  Data prepared in {time.time() - t0:.1f}s')

    # phase C: build 3D map from spring
    print(f'\n{"="*60}')
    print('PHASE C: Build 3D Map from Spring')
    print(f'{"="*60}')

    t0 = time.time()
    model, feature_path = run_sfm_pipeline(
        image_dir, db_list, outputs,
        feature_conf_name='superpoint_aachen',
        matcher_conf_name='superpoint+lightglue',
        retrieval_conf_name='netvlad',
        num_retrieval=num_retrieval,
    )
    sfm_time = time.time() - t0
    print(f'  Total SfM time: {sfm_time:.1f}s')

    if model is None:
        print('\nSfM FAILED. Cannot proceed to localization.')
        summary = {
            'mode': mode, 'status': 'sfm_failed',
            'sfm_time_s': sfm_time,
            'n_db': len(db_list), 'n_query': len(query_list),
        }
        with open(outputs / 'summary.json', 'w') as f:
            json.dump(summary, f, indent=2)
        return summary

    # evaluate SfM
    print('\n  Evaluating SfM map quality...')
    sfm_results = evaluate_sfm_map(model, gt_poses, db_list, outputs)

    # plot SfM
    if (outputs / 'sfm_trajectory.npz').exists():
        plot_sfm_vs_gt(outputs)

    # phase D: localize summer against spring map
    print(f'\n{"="*60}')
    print('PHASE D: Localize Summer against Spring Map')
    print(f'{"="*60}')

    t0 = time.time()
    loc_poses = run_localization(
        image_dir, model, query_list, db_list, feature_path,
        outputs,
        matcher_conf_name='superpoint+lightglue',
        retrieval_conf_name='netvlad',
        num_retrieval=num_retrieval,
    )
    loc_time = time.time() - t0
    print(f'  Total localization time: {loc_time:.1f}s')

    # evaluate localization
    print('\n  Evaluating localization accuracy...')
    eval_results = evaluate_localization(loc_poses, gt_poses, query_list, outputs)

    print('\n  Generating plots...')
    plot_localization_vs_gt(loc_poses, gt_poses, query_list, outputs)
    plot_localization_success(eval_results, outputs)
    plot_failure_map(loc_poses, gt_poses, query_list, outputs)
    plot_comparison_all_methods(eval_results, outputs)

    # phase E: ablation (full mode only)
    ablation_results = {}
    if mode == 'full':
        print(f'\n{"="*60}')
        print('PHASE E: Ablation Studies')
        print(f'{"="*60}')
        ablation_results = run_ablation_features(
            image_dir, db_list, query_list, gt_poses, outputs
        )

    # save summary
    total_time = time.time() - t_start

    summary = {
        'mode': mode,
        'status': 'ok',
        'total_time_s': total_time,
        'total_time_human': f'{total_time / 3600:.1f} hours',
        'spring_session': SPRING,
        'summer_session': SUMMER,
        'n_db_images': len(db_list),
        'n_query_images': len(query_list),
        'spring_subsample': spring_every,
        'summer_subsample': summer_every,
        'pipeline': {
            'features': 'SuperPoint (n=4096, nms=3, resize=1024)',
            'retrieval': 'NetVLAD',
            'matcher': 'LightGlue',
            'sfm': 'COLMAP (single camera)',
            'localization': 'PnP+RANSAC (thresh=12)',
        },
        'sfm': sfm_results,
        'sfm_time_s': sfm_time,
        'localization': eval_results,
        'loc_time_s': loc_time,
    }
    if ablation_results:
        summary['ablation_features'] = {str(k): v for k, v in ablation_results.items()}

    with open(outputs / 'summary.json', 'w') as f:
        json.dump(summary, f, indent=2)

    # print final summary
    print(f'\n{"="*70}')
    print('EXPERIMENT 0.7: FINAL RESULTS')
    print(f'{"="*70}')
    print(f'Total time: {total_time / 60:.1f} min ({total_time / 3600:.1f} hours)')
    print(f'\nSfM Map (Spring):')
    print(f'  Registered: {sfm_results.get("n_registered", 0)}/{sfm_results.get("n_total", 0)} '
          f'({sfm_results.get("pct_registered", 0):.1f}%)')
    print(f'  3D points: {sfm_results.get("n_3d_points", 0)}')
    print(f'  ATE RMSE (Sim3): {sfm_results.get("sfm_ate_rmse", "N/A")}')
    print(f'\nLocalization (Summer → Spring):')
    print(f'  Localized: {eval_results.get("n_localized", 0)}/{eval_results.get("n_query", 0)} '
          f'({eval_results.get("pct_localized", 0):.1f}%)')
    if eval_results.get('status') == 'ok':
        print(f'  Median trans error: {eval_results["median_trans_error_m"]:.2f} m')
        print(f'  Median rot error: {eval_results["median_rot_error_deg"]:.2f} deg')
        print(f'  ATE RMSE (Sim3): {eval_results.get("ate_rmse_sim3", "N/A")}')
        print(f'  Within (0.25m, 2°): {eval_results.get("pct_0.25m_2deg", 0):.1f}%')
        print(f'  Within (0.5m, 5°): {eval_results.get("pct_0.5m_5deg", 0):.1f}%')
        print(f'  Within (5m, 10°): {eval_results.get("pct_5.0m_10deg", 0):.1f}%')

    print(f'\nResults saved to: {outputs}')
    return summary


# entry point

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Experiment 0.7: hloc on NCLT')
    parser.add_argument('--sanity', action='store_true', help='Quick 10-image test')
    parser.add_argument('--small', action='store_true', help='Small subset (every 10th/20th)')
    args = parser.parse_args()

    if args.sanity:
        mode = 'sanity'
    elif args.small:
        mode = 'small'
    else:
        mode = 'full'

    run_experiment(mode)
