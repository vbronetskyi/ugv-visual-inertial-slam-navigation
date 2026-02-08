#!/usr/bin/env python3
"""hloc visual localization on 4Seasons dataset

Builds a 3D reference model from office_loop_1 (spring) using known VIO poses,
then localizes query sequences (summer/winter) against it

Usage:
  python3 run_4seasons_hloc.py [--self-test] [--skip-extract]
"""

import argparse
import json
import logging
import os
import sys
import shutil
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# hloc imports
sys.path.insert(0, '/workspace/third_party/hloc')
from hloc import (
    extract_features,
    match_features,
    localize_sfm,
    pairs_from_retrieval,
    triangulation,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('4seasons_hloc')

DATA_DIR = Path('/workspace/data/4seasons')
HLOC_OUTPUTS = DATA_DIR / 'hloc_outputs'
RESULTS_DIR = Path('/workspace/datasets/robotcar/results/4seasons_hloc')
IMAGE_DIR = DATA_DIR  # hloc image root, paths in query lists are relative from here

# camera intrinsics (undistorted pinhole from 4Seasons calibration)
FX = FY = 501.4757919305817
CX = 421.7953735163109
CY = 167.65799492501083
WIDTH, HEIGHT = 800, 400

# sequences
REFERENCE_SEQ = {
    'name': 'office_loop_1',
    'recording': 'recording_2020-03-24_17-36-22',
    'season': 'spring',
}

QUERY_SEQS = [
    {
        'name': 'office_loop_4',
        'recording': 'recording_2020-06-12_10-10-57',
        'season': 'summer',
    },
    {
        'name': 'office_loop_5',
        'recording': 'recording_2021-01-07_12-04-03',
        'season': 'winter',
    },
]

# hloc configs
FEATURE_CONF = extract_features.confs['superpoint_max']
MATCHER_CONF = match_features.confs['superglue']
RETRIEVAL_CONFS = {
    'netvlad': extract_features.confs['netvlad'],
    'openibl': extract_features.confs['openibl'],
}
NUM_RETRIEVAL = 20
SUBSAMPLE_DIST = 2.0  # meters between reference keyframes


# data loading
def load_vio_poses(result_path):
    """load VIO reference poses from result.txt"""
    poses = {}
    with open(result_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            poses[ts] = (np.array([tx, ty, tz]), np.array([qx, qy, qz, qw]))
    return poses


def load_gnss_gt(gnss_path):
    """Load GNSS ground truth from GNSSPoses.txt"""
    poses = {}
    with open(gnss_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 8:
                continue
            ts_s = int(parts[0]) / 1e9
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            poses[ts_s] = (np.array([tx, ty, tz]), np.array([qx, qy, qz, qw]))
    return poses


def get_image_timestamps(seq_dir, recording):
    """Get sorted list of image timestamps (nanosecond strings) from cam0"""
    cam0_dir = seq_dir / recording / 'undistorted_images' / 'cam0'
    if not cam0_dir.exists():
        return []
    timestamps = sorted([f.stem for f in cam0_dir.glob('*.png')])
    return timestamps


def get_image_relpath(seq_name, recording, ts_ns):
    """Get image path relative to IMAGE_DIR"""
    return f'{seq_name}/{recording}/undistorted_images/cam0/{ts_ns}.png'


# reference model building
def subsample_by_distance(timestamps_ns, vio_poses, min_dist=2.0):
    """Subsample timestamps to have at least min_dist meters between frames"""
    selected = []
    last_pos = None

    for ts_ns in timestamps_ns:
        ts_s = int(ts_ns) / 1e9
        # find closest VIO pose
        closest_ts = min(vio_poses.keys(), key=lambda t: abs(t - ts_s))
        if abs(closest_ts - ts_s) > 0.1:
            continue
        pos = vio_poses[closest_ts][0]
        if last_pos is None or np.linalg.norm(pos - last_pos) >= min_dist:
            selected.append((ts_ns, closest_ts))
            last_pos = pos

    return selected


def create_colmap_model(selected_frames, vio_poses, model_dir):
    """Create COLMAP text model (cameras/images/points3D) from VIO poses"""
    model_dir.mkdir(parents=True, exist_ok=True)

    # cameras.txt
    with open(model_dir / 'cameras.txt', 'w') as f:
        f.write('# Camera list with one line of data per camera:\n')
        f.write('# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n')
        f.write(f'1 PINHOLE {WIDTH} {HEIGHT} {FX} {FY} {CX} {CY}\n')

    # images.txt
    with open(model_dir / 'images.txt', 'w') as f:
        f.write('# Image list with two lines of data per image:\n')
        f.write('#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n')
        f.write('#   POINTS2D[] as (X, Y, POINT3D_ID)\n')

        for img_id, (ts_ns, vio_ts) in enumerate(selected_frames, start=1):
            pos, quat_xyzw = vio_poses[vio_ts]
            # convert world-to-camera: R_cw, t_cw = -R_cw @ pos
            R = Rotation.from_quat(quat_xyzw).as_matrix()
            t_cw = -R @ pos

            quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
            img_name = get_image_relpath(
                REFERENCE_SEQ['name'], REFERENCE_SEQ['recording'], ts_ns)

            f.write(f'{img_id} {quat_wxyz[0]} {quat_wxyz[1]} {quat_wxyz[2]} {quat_wxyz[3]} ')
            f.write(f'{t_cw[0]} {t_cw[1]} {t_cw[2]} 1 {img_name}\n')
            f.write('\n')  # empty line for 2D points

    # points3D.txt (empty)
    with open(model_dir / 'points3D.txt', 'w') as f:
        f.write('# 3D point list (empty - will be triangulated)\n')

    log.info(f'Created COLMAP model with {len(selected_frames)} images at {model_dir}')
    return model_dir


def write_image_list(image_names, output_path, with_intrinsics=False):
    """Write image list file for hloc"""
    with open(output_path, 'w') as f:
        for name in image_names:
            if with_intrinsics:
                f.write(f'{name} PINHOLE {WIDTH} {HEIGHT} {FX} {FY} {CX} {CY}\n')
            else:
                f.write(f'{name}\n')
    log.info(f'Wrote {len(image_names)} images to {output_path}')


# evaluation
def umeyama_alignment(src, dst):
    """Umeyama Sim3 alignment: find s, R, t such that dst ≈ s*R@src + t"""
    n = src.shape[0]
    mu_s, mu_d = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_s, dst - mu_d
    var_s = np.sum(src_c ** 2) / n
    H = (dst_c.T @ src_c) / n
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / var_s
    t = mu_d - s * R @ mu_s
    return s, R, t


def compute_alignment(vio_poses, gnss_poses):
    """Compute Sim3 alignment from VIO frame to GNSS frame using common timestamps"""
    vio_ts = sorted(vio_poses.keys())
    gnss_ts = sorted(gnss_poses.keys())

    src_pts, dst_pts = [], []
    gi = 0
    for vt in vio_ts:
        while gi < len(gnss_ts) - 1 and gnss_ts[gi + 1] <= vt:
            gi += 1
        if abs(gnss_ts[gi] - vt) < 0.1:
            src_pts.append(vio_poses[vt][0])
            dst_pts.append(gnss_poses[gnss_ts[gi]][0])

    src_pts = np.array(src_pts)
    dst_pts = np.array(dst_pts)
    log.info(f'Alignment: {len(src_pts)} common poses')
    s, R, t = umeyama_alignment(src_pts, dst_pts)
    log.info(f'Alignment: scale={s:.4f}')
    return s, R, t


def transform_pose(pos, s, R, t):
    """Apply Sim3 transform to a position"""
    return s * R @ pos + t


def load_hloc_results(results_path):
    """load hloc localization results as {image_name: (pos, R_cw)}"""
    poses = {}
    with open(results_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            name = parts[0]
            qw, qx, qy, qz = map(float, parts[1:5])
            tx, ty, tz = map(float, parts[5:8])
            R_cw = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            pos = -R_cw.T @ np.array([tx, ty, tz])  # camera position in world
            poses[name] = (pos, R_cw)
    return poses


def evaluate_self_test(loc_poses, vio_poses, image_ts_map):
    """evaluate self-test localization directly against VIO poses (same frame)"""
    trans_errors = []
    rot_errors = []
    vio_ts_arr = np.array(sorted(vio_poses.keys()))

    for img_name, (pos_est, R_cw_est) in loc_poses.items():
        if img_name not in image_ts_map:
            continue
        ts_s = image_ts_map[img_name]

        idx = np.argmin(np.abs(vio_ts_arr - ts_s))
        if abs(vio_ts_arr[idx] - ts_s) > 0.05:
            continue
        vio_pos, vio_quat = vio_poses[vio_ts_arr[idx]]

        trans_err = np.linalg.norm(pos_est - vio_pos)
        trans_errors.append(trans_err)

        # rotation error: both in same frame
        R_cw_gt = Rotation.from_quat(vio_quat).as_matrix()
        R_diff = R_cw_est @ R_cw_gt.T
        trace_val = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
        rot_err = np.degrees(np.arccos(trace_val))
        rot_errors.append(rot_err)

    trans_errors = np.array(trans_errors)
    rot_errors = np.array(rot_errors)

    if len(trans_errors) == 0:
        return None

    thresholds = [(0.25, 2), (0.5, 5), (5.0, 10)]
    pct = {}
    for t_th, r_th in thresholds:
        correct = np.sum((trans_errors < t_th) & (rot_errors < r_th))
        pct[f'{t_th}m/{r_th}deg'] = round(100 * correct / len(trans_errors), 1)

    return {
        'num_localized': len(trans_errors),
        'median_trans_m': round(float(np.median(trans_errors)), 4),
        'median_rot_deg': round(float(np.median(rot_errors)), 4),
        'mean_trans_m': round(float(np.mean(trans_errors)), 4),
        'accuracy': pct,
    }


def evaluate_localization(loc_poses, gnss_gt, image_ts_map, s, R, t):
    """Evaluate localized poses against GNSS GT (cross-season).

    s, R, t define the Sim3 alignment from VIO to GNSS frame.
    """
    trans_errors = []
    rot_errors = []
    gnss_ts_sorted = np.array(sorted(gnss_gt.keys()))

    for img_name, (pos_vio, R_cw_est) in loc_poses.items():
        if img_name not in image_ts_map:
            continue
        ts_s = image_ts_map[img_name]

        # find closest GT
        idx = np.argmin(np.abs(gnss_ts_sorted - ts_s))
        if abs(gnss_ts_sorted[idx] - ts_s) > 0.1:
            continue
        gt_pos, gt_quat = gnss_gt[gnss_ts_sorted[idx]]
        gt_R = Rotation.from_quat(gt_quat).as_matrix()

        # transform estimated pose to GNSS frame
        pos_gnss_est = transform_pose(pos_vio, s, R, t)

        # translation error
        trans_err = np.linalg.norm(pos_gnss_est - gt_pos)
        trans_errors.append(trans_err)

        # rotation error
        R_est_world = R @ R_cw_est.T  # approximate rotation in world frame
        R_diff = R_est_world @ gt_R.T
        trace_val = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
        rot_err = np.degrees(np.arccos(trace_val))
        rot_errors.append(rot_err)

    trans_errors = np.array(trans_errors)
    rot_errors = np.array(rot_errors)

    if len(trans_errors) == 0:
        return None

    # compute thresholds
    thresholds = [(0.25, 2), (0.5, 5), (5.0, 10)]
    pct = {}
    for t_th, r_th in thresholds:
        correct = np.sum((trans_errors < t_th) & (rot_errors < r_th))
        pct[f'{t_th}m/{r_th}deg'] = round(100 * correct / len(trans_errors), 1)

    results = {
        'num_localized': len(trans_errors),
        'median_trans_m': round(float(np.median(trans_errors)), 4),
        'median_rot_deg': round(float(np.median(rot_errors)), 4),
        'mean_trans_m': round(float(np.mean(trans_errors)), 4),
        'accuracy': pct,
    }
    return results


# main pipeline
def run_pipeline(args):
    """Run the full hloc pipeline on 4Seasons"""
    HLOC_OUTPUTS.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    ref_seq = REFERENCE_SEQ
    ref_dir = DATA_DIR / ref_seq['name']
    rec_dir = ref_dir / ref_seq['recording']

    # step 1: build reference model
    log.info('='*60)
    log.info('STEP 1: Building reference COLMAP model')
    log.info('='*60)

    # load VIO poses
    vio_poses = load_vio_poses(rec_dir / 'result.txt')
    log.info(f'Loaded {len(vio_poses)} VIO poses')

    # get image timestamps
    all_ref_ts = get_image_timestamps(ref_dir, ref_seq['recording'])
    log.info(f'Found {len(all_ref_ts)} reference images')

    # subsample by distance
    selected = subsample_by_distance(all_ref_ts, vio_poses, min_dist=SUBSAMPLE_DIST)
    log.info(f'Subsampled to {len(selected)} reference keyframes (every ~{SUBSAMPLE_DIST}m)')

    # create COLMAP model
    colmap_model_dir = HLOC_OUTPUTS / 'colmap_reference'
    create_colmap_model(selected, vio_poses, colmap_model_dir)

    # reference image list
    ref_images = [get_image_relpath(ref_seq['name'], ref_seq['recording'], ts_ns)
                  for ts_ns, _ in selected]
    ref_list_path = HLOC_OUTPUTS / 'ref_images.txt'
    write_image_list(ref_images, ref_list_path)

    # determine which query sequences are available
    available_queries = []
    for qseq in QUERY_SEQS:
        qdir = DATA_DIR / qseq['name']
        img_dir = qdir / qseq['recording'] / 'undistorted_images' / 'cam0'
        if img_dir.exists() and len(list(img_dir.glob('*.png'))) > 100:
            available_queries.append(qseq)
            log.info(f"Query available: {qseq['name']} ({qseq['season']})")
        else:
            log.warning(f"Query NOT available: {qseq['name']} - images not extracted")

    if args.self_test:
        log.info('Self-test mode: using odd frames from loop_1 as query')

    # step 2: extract features
    log.info('='*60)
    log.info('STEP 2: Extracting features')
    log.info('='*60)

    # collect all images that need features
    all_images = list(ref_images)

    # query images
    query_image_map = {}  # seq_name -> [(img_relpath, ts_s)]
    for qseq in available_queries:
        qdir = DATA_DIR / qseq['name']
        q_timestamps = get_image_timestamps(qdir, qseq['recording'])
        q_vio = load_vio_poses(qdir / qseq['recording'] / 'result.txt')
        q_images = []
        for ts_ns in q_timestamps:
            relpath = get_image_relpath(qseq['name'], qseq['recording'], ts_ns)
            q_images.append((relpath, int(ts_ns) / 1e9))
        query_image_map[qseq['name']] = q_images
        all_images.extend([img for img, _ in q_images])
        log.info(f"  {qseq['name']}: {len(q_images)} query images")

    # self-test: odd frames as query
    if args.self_test:
        self_test_images = []
        for i, ts_ns in enumerate(all_ref_ts):
            if i % 2 == 1:  # odd frames
                relpath = get_image_relpath(ref_seq['name'], ref_seq['recording'], ts_ns)
                self_test_images.append((relpath, int(ts_ns) / 1e9))
        query_image_map['self_test'] = self_test_images
        all_images.extend([img for img, _ in self_test_images])
        log.info(f'  self_test: {len(self_test_images)} query images')

    # write combined image list
    all_images_path = HLOC_OUTPUTS / 'all_images.txt'
    write_image_list(list(set(all_images)), all_images_path)

    if not args.skip_extract:
        # extract local features (SuperPoint)
        log.info('Extracting SuperPoint features...')
        feature_path = extract_features.main(
            FEATURE_CONF, IMAGE_DIR, image_list=all_images_path,
            feature_path=HLOC_OUTPUTS / 'feats-superpoint.h5')

        # extract global descriptors
        for ret_name, ret_conf in RETRIEVAL_CONFS.items():
            log.info(f'Extracting {ret_name} descriptors...')
            extract_features.main(
                ret_conf, IMAGE_DIR, image_list=all_images_path,
                feature_path=HLOC_OUTPUTS / f'global-{ret_name}.h5')
    else:
        feature_path = HLOC_OUTPUTS / 'feats-superpoint.h5'
        log.info('Skipping feature extraction (--skip-extract)')

    # step 3: build 3D model via triangulation
    log.info('='*60)
    log.info('STEP 3: Triangulating reference model')
    log.info('='*60)

    sfm_dir = HLOC_OUTPUTS / 'sfm_sp_sg'

    # reference pairs from NetVLAD retrieval (ref-ref only)
    ref_pairs_path = HLOC_OUTPUTS / 'pairs-ref-netvlad.txt'
    pairs_from_retrieval.main(
        HLOC_OUTPUTS / 'global-netvlad.h5',
        ref_pairs_path,
        num_matched=NUM_RETRIEVAL,
        query_list=ref_list_path,
        db_list=ref_list_path,
    )

    # match reference pairs
    ref_matches_path = match_features.main(
        MATCHER_CONF, ref_pairs_path, features=feature_path,
        matches=HLOC_OUTPUTS / 'matches-ref-sg.h5')

    # triangulate
    sfm_model = triangulation.main(
        sfm_dir, colmap_model_dir, IMAGE_DIR,
        ref_pairs_path, feature_path, ref_matches_path,
        estimate_two_view_geometries=True)
    log.info(f'Triangulated model: {sfm_model.summary()}')

    # step 4: localize queries
    log.info('='*60)
    log.info('STEP 4: Localizing query images')
    log.info('='*60)

    all_results = {}

    for qname, q_images in query_image_map.items():
        # write query list with intrinsics
        query_list_path = HLOC_OUTPUTS / f'queries_{qname}.txt'
        write_image_list([img for img, _ in q_images], query_list_path, with_intrinsics=True)

        # also write query names-only list for retrieval
        query_names_path = HLOC_OUTPUTS / f'query_names_{qname}.txt'
        write_image_list([img for img, _ in q_images], query_names_path, with_intrinsics=False)

        for ret_name in RETRIEVAL_CONFS:
            method_name = f'sp_sg_{ret_name}'
            log.info(f'Localizing {qname} with {method_name}...')

            # query-reference retrieval pairs
            loc_pairs_path = HLOC_OUTPUTS / f'pairs-{qname}-{ret_name}.txt'
            pairs_from_retrieval.main(
                HLOC_OUTPUTS / f'global-{ret_name}.h5',
                loc_pairs_path,
                num_matched=NUM_RETRIEVAL,
                db_list=ref_list_path,
                query_list=query_names_path,
            )

            # match query-reference pairs
            loc_matches_path = match_features.main(
                MATCHER_CONF, loc_pairs_path, features=feature_path,
                matches=HLOC_OUTPUTS / f'matches-{qname}-{ret_name}-sg.h5')

            # localize
            results_path = RESULTS_DIR / f'loc_{qname}_{method_name}.txt'
            localize_sfm.main(
                sfm_dir, query_list_path, loc_pairs_path,
                feature_path, loc_matches_path, results_path,
                covisibility_clustering=False)

            all_results[(qname, method_name)] = results_path

    # step 5: evaluate
    log.info('='*60)
    log.info('STEP 5: Evaluating results')
    log.info('='*60)

    # compute VIO->GNSS alignment from reference sequence
    ref_gnss = load_gnss_gt(rec_dir / 'GNSSPoses.txt')
    s_align, R_align, t_align = compute_alignment(vio_poses, ref_gnss)

    eval_results = {}

    for (qname, method_name), results_path in all_results.items():
        if not results_path.exists():
            log.warning(f'No results for {qname}/{method_name}')
            continue

        loc_poses = load_hloc_results(results_path)
        log.info(f'{qname}/{method_name}: {len(loc_poses)} localized poses')

        # build image->timestamp map (use basename as key since hloc
        # write_poses strips paths to basename only)
        img_ts = {Path(img).name: ts for img, ts in query_image_map[qname]}

        if qname == 'self_test':
            # self-test: compare directly against VIO poses (same frame)
            # avoids ~6m VIO->GNSS alignment noise
            metrics = evaluate_self_test(loc_poses, vio_poses, img_ts)
        else:
            qseq = next(q for q in QUERY_SEQS if q['name'] == qname)
            qdir = DATA_DIR / qseq['name'] / qseq['recording']
            gnss_gt = load_gnss_gt(qdir / 'GNSSPoses.txt')
            metrics = evaluate_localization(loc_poses, gnss_gt, img_ts, s_align, R_align, t_align)

        if metrics:
            eval_results[f'{qname}/{method_name}'] = metrics
            log.info(f"  {qname}/{method_name}:")
            log.info(f"    Localized: {metrics['num_localized']}")
            log.info(f"    Median trans: {metrics['median_trans_m']:.3f} m")
            log.info(f"    Median rot:   {metrics['median_rot_deg']:.2f} deg")
            for k, v in metrics['accuracy'].items():
                log.info(f"    {k}: {v}%")

    # save results
    results_json = RESULTS_DIR / 'eval_results.json'
    with open(results_json, 'w') as f:
        json.dump(eval_results, f, indent=2)
    log.info(f'Results saved to {results_json}')

    #  summary table 
    print('\n' + '='*70)
    print('4Seasons hloc Visual Localization Results')
    print('='*70)
    print(f'{"Query":<20} {"Method":<15} {"0.25m/2°":<10} {"0.5m/5°":<10} {"5m/10°":<10} {"Med.t(m)":<10}')
    print('-'*70)
    for key, m in eval_results.items():
        qn, mn = key.split('/')
        acc = m['accuracy']
        print(f'{qn:<20} {mn:<15} {acc.get("0.25m/2deg","N/A"):<10} '
              f'{acc.get("0.5m/5deg","N/A"):<10} {acc.get("5.0m/10deg","N/A"):<10} '
              f'{m["median_trans_m"]:<10.3f}')
    print('='*70)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(10, 6))
        methods = sorted(set(mn for _, mn in all_results.keys()))
        queries = sorted(set(qn for qn, _ in all_results.keys()))
        x = np.arange(len(queries))
        width = 0.35

        for i, method in enumerate(methods):
            vals = []
            for q in queries:
                key = f'{q}/{method}'
                if key in eval_results:
                    vals.append(eval_results[key]['accuracy'].get('0.5m/5deg', 0))
                else:
                    vals.append(0)
            ax.bar(x + i * width, vals, width, label=method)

        ax.set_ylabel('Accuracy @ 0.5m/5° (%)')
        ax.set_title('4Seasons hloc Visual Localization')
        ax.set_xticks(x + width / 2)
        ax.set_xticklabels(queries)
        ax.legend()
        ax.grid(True, alpha=0.3)

        plot_path = RESULTS_DIR / 'accuracy_comparison.png'
        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        plt.close()
        log.info(f'Plot saved to {plot_path}')
    except Exception as e:
        log.warning(f'Could not create plot: {e}')


def main():
    parser = argparse.ArgumentParser(description='hloc on 4Seasons')
    parser.add_argument('--self-test', action='store_true',
                        help='Run self-localization test on reference sequence')
    parser.add_argument('--skip-extract', action='store_true',
                        help='Skip feature extraction (reuse existing)')
    args = parser.parse_args()
    run_pipeline(args)


if __name__ == '__main__':
    main()
