#!/usr/bin/env python3
"""
RobotCar Seasons full benchmark. Runs a handful of hloc pipeline configs
and evaluates them. Meant for unattended overnight runs.

Methods tested:
  1. SuperPoint + SuperGlue + NetVLAD   (baseline, already computed)
  2. SuperPoint + LightGlue + NetVLAD   (faster matcher)
  3. DISK + LightGlue + NetVLAD         (alt learned features)
  4. ALIKED + LightGlue + NetVLAD       (recent efficient features)
  5. SuperPoint + SuperGlue + OpenIBL   (alt retrieval)
  6. SIFT + NN-ratio + NetVLAD          (classical baseline)
"""

import sys
import os
import time
import json
import signal
import logging
import traceback
import subprocess
import shutil
from pathlib import Path
from collections import defaultdict
from datetime import datetime

import numpy as np
import psutil
import torch
from scipy.spatial.transform import Rotation

sys.path.insert(0, '/workspace/third_party/hloc')
from hloc import (
    extract_features,
    match_features,
    localize_sfm,
    pairs_from_retrieval,
    triangulation,
)
from hloc.pipelines.RobotCar import colmap_from_nvm
from hloc.pipelines.RobotCar.pipeline import generate_query_list, CONDITIONS

DATASET = Path('/workspace/data/robotcar_seasons')
OUTPUTS = DATASET / 'outputs'
IMAGES = DATASET / 'images'
RESULTS_DIR = Path('/workspace/datasets/robotcar/results/robotcar_seasons_hloc')
TRAIN_FILE = DATASET / 'robotcar_v2_train.txt'
TEST_FILE = DATASET / 'robotcar_v2_test.txt'
PROGRESS_FILE = RESULTS_DIR / 'progress.json'

NUM_COVIS = 20
# CLAHE_CLIP = 3.0  # same as for daytime
NUM_LOC = 20

# fallback: use SP+SG reference SfM if triangulation crashes for other features
FALLBACK_SFM = 'sfm_superpoint+superglue'

# logging goes to stdout only, tee handles file output
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger('benchmark')

# signal name lookup for crash diagnostics
SIGNAL_NAMES = {v: v.name for v in signal.Signals}


def signal_name(retcode):
    """Convert negative return code to signal name"""
    if retcode >= 0:
        return str(retcode)
    sig_num = -retcode
    try:
        return f'{signal.Signals(sig_num).name} (signal {sig_num})'
    except ValueError:
        return f'signal {sig_num}'


# system diagnostics
def log_system_state(label=''):
    """Log memory, disk, GPU state for diagnostics"""
    prefix = f'[SYS {label}] ' if label else '[SYS] '

    # memory
    mem = psutil.virtual_memory()
    swap = psutil.swap_memory()
    log.info(f'{prefix}RAM: {mem.used/1e9:.1f}/{mem.total/1e9:.1f} GB '
             f'({mem.percent}%), Swap: {swap.used/1e9:.1f}/{swap.total/1e9:.1f} GB')

    # disk
    disk = shutil.disk_usage('/workspace')
    log.info(f'{prefix}Disk: {disk.used/1e9:.0f}/{disk.total/1e9:.0f} GB used, '
             f'{disk.free/1e9:.0f} GB free')

    # GPU
    if torch.cuda.is_available():
        allocated = torch.cuda.memory_allocated() / 1e9
        reserved = torch.cuda.memory_reserved() / 1e9
        total = torch.cuda.get_device_properties(0).total_memory / 1e9
        log.info(f'{prefix}GPU: {allocated:.1f}/{total:.1f} GB allocated, '
                 f'{reserved:.1f} GB reserved')

    # outputs dir size
    outputs_size = sum(f.stat().st_size for f in OUTPUTS.rglob('*') if f.is_file())
    log.info(f'{prefix}Outputs dir: {outputs_size/1e9:.1f} GB')


def check_disk_space(min_gb=20):
    disk = shutil.disk_usage('/workspace')
    free_gb = disk.free / 1e9
    if free_gb < min_gb:
        log.error(f'LOW DISK SPACE: {free_gb:.1f} GB free (need {min_gb} GB minimum)')
        return False
    return True


# custom feature configs (standardized to 4096 keypoints, resize 1024)
CUSTOM_FEATURE_CONFS = {
    'disk_4096': {
        'output': 'feats-disk-n4096-r1024',
        'model': {'name': 'disk', 'max_keypoints': 4096},
        'preprocessing': {'grayscale': False, 'resize_max': 1024},
    },
    'aliked_4096': {
        'output': 'feats-aliked-n4096-r1024',
        'model': {'name': 'aliked', 'model_name': 'aliked-n16', 'max_num_keypoints': 4096},
        'preprocessing': {'grayscale': False, 'resize_max': 1024},
    },
    'sift_4096': {
        'output': 'feats-sift-n4096-r1024',
        'model': {'name': 'dog', 'max_keypoints': 4096},
        'preprocessing': {'grayscale': True, 'resize_max': 1024},
    },
}

# method Definitions
METHODS = [
    {
        'name': 'SuperPoint+SuperGlue',
        'short': 'sp_sg_nv',
        'feature_conf_name': 'superpoint_aachen',
        'matcher_conf_name': 'superglue',
        'retrieval_conf_name': 'netvlad',
        'sfm_name': 'sfm_superpoint+superglue',
        'results_name': 'RobotCar_hloc_superpoint+superglue_netvlad20.txt',
        'already_done': True,
    },
    {
        'name': 'SuperPoint+LightGlue',
        'short': 'sp_lg_nv',
        'feature_conf_name': 'superpoint_aachen',
        'matcher_conf_name': 'superpoint+lightglue',
        'retrieval_conf_name': 'netvlad',
        'sfm_name': 'sfm_superpoint+lightglue',
        'results_name': 'RobotCar_hloc_superpoint+lightglue_netvlad20.txt',
    },
    {
        'name': 'DISK+LightGlue',
        'short': 'disk_lg_nv',
        'feature_conf_name': 'disk_4096',
        'matcher_conf_name': 'disk+lightglue',
        'retrieval_conf_name': 'netvlad',
        'sfm_name': 'sfm_disk+lightglue',
        'results_name': 'RobotCar_hloc_disk+lightglue_netvlad20.txt',
    },
    {
        'name': 'ALIKED+LightGlue',
        'short': 'aliked_lg_nv',
        'feature_conf_name': 'aliked_4096',
        'matcher_conf_name': 'aliked+lightglue',
        'retrieval_conf_name': 'netvlad',
        'sfm_name': 'sfm_aliked+lightglue',
        'results_name': 'RobotCar_hloc_aliked+lightglue_netvlad20.txt',
    },
    {
        'name': 'SuperPoint+SuperGlue+OpenIBL',
        'short': 'sp_sg_openibl',
        'feature_conf_name': 'superpoint_aachen',
        'matcher_conf_name': 'superglue',
        'retrieval_conf_name': 'openibl',
        'sfm_name': 'sfm_superpoint+superglue',  # reuse existing SfM
        'results_name': 'RobotCar_hloc_superpoint+superglue_openibl20.txt',
    },
    {
        'name': 'SIFT+NN-ratio',
        'short': 'sift_nn_nv',
        'feature_conf_name': 'sift_4096',
        'matcher_conf_name': 'NN-ratio',
        'retrieval_conf_name': 'netvlad',
        'sfm_name': 'sfm_sift+nn',
        'results_name': 'RobotCar_hloc_sift+nn_netvlad20.txt',
    },
]


def get_feature_conf(name):
    """Get feature extractor configuration by name"""
    if name in CUSTOM_FEATURE_CONFS:
        return CUSTOM_FEATURE_CONFS[name]
    return extract_features.confs[name]


def get_matcher_conf(name):
    """Get matcher configuration by name"""
    return match_features.confs[name]


def get_retrieval_conf(name):
    """Get retrieval configuration by name"""
    return extract_features.confs[name]


# progress tracking
def save_progress(progress):
    """Save progress state to disk for remote monitoring"""
    progress['last_updated'] = datetime.now().isoformat()
    PROGRESS_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(PROGRESS_FILE, 'w') as f:
        json.dump(progress, f, indent=2)


# phase A: Evaluation
def load_ground_truth(train_file):
    """Load GT poses from robotcar_v2_train.txt.

    Format per line: condition/camera/timestamp.jpg R00 R01 R02 C0 R10 R11 R12 C1 R20 R21 R22 C2 0 0 0 1
    R maps camera->world, C is camera center in world.

    Returns dict: key='camera/timestamp.jpg' -> {condition, R_cw, pos, R_w2c}
    """
    gt = {}
    with open(train_file) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 13:
                continue
            name = parts[0]
            vals = list(map(float, parts[1:]))

            R_cw = np.array([
                [vals[0], vals[1], vals[2]],
                [vals[4], vals[5], vals[6]],
                [vals[8], vals[9], vals[10]],
            ])
            C = np.array([vals[3], vals[7], vals[11]])

            name_parts = name.split('/')
            condition = name_parts[0]
            key = '/'.join(name_parts[1:])

            gt[key] = {
                'condition': condition,
                'R_cw': R_cw,
                'pos': C,
                'R_w2c': R_cw.T,
            }
    return gt


def load_localization_results(results_file):
    """Load hloc localization results.

    Format per line: camera/timestamp.jpg qw qx qy qz tx ty tz
    Quaternion+translation define the world-to-camera transform.

    Returns dict: key='camera/timestamp.jpg' -> {R_w2c, t, pos, quat}
    """
    poses = {}
    with open(results_file) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            name = parts[0]
            qw, qx, qy, qz = map(float, parts[1:5])
            tx, ty, tz = map(float, parts[5:8])

            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            t = np.array([tx, ty, tz])
            pos = -R.T @ t

            poses[name] = {
                'R_w2c': R,
                't': t,
                'pos': pos,
                'quat': [qw, qx, qy, qz],
            }
    return poses


def evaluate_method(loc_poses, gt_poses):
    """Evaluate localization results against ground truth.

    Returns dict with per-condition and aggregate accuracy at standard thresholds.
    """
    matched_keys = set(loc_poses.keys()) & set(gt_poses.keys())
    if len(matched_keys) == 0:
        gt_keys_jpg = {}
        for k, v in gt_poses.items():
            k_jpg = k.rsplit('.', 1)[0] + '.jpg'
            gt_keys_jpg[k_jpg] = v
        matched_keys = set(loc_poses.keys()) & set(gt_keys_jpg.keys())
        if len(matched_keys) > 0:
            gt_poses = gt_keys_jpg

    if len(matched_keys) == 0:
        log.warning('No matched keys between results and GT!')
        return None

    errors_by_condition = defaultdict(lambda: {'trans': [], 'rot': []})
    errors_by_camera = defaultdict(lambda: {'trans': [], 'rot': []})
    all_trans = []
    all_rot = []

    for key in matched_keys:
        est = loc_poses[key]
        gt = gt_poses[key]
        trans_err = np.linalg.norm(est['pos'] - gt['pos'])
        R_diff = est['R_w2c'] @ gt['R_w2c'].T
        trace_val = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
        rot_err = np.degrees(np.arccos(trace_val))

        all_trans.append(trans_err)
        all_rot.append(rot_err)
        condition = gt['condition']
        camera = key.split('/')[0]
        errors_by_condition[condition]['trans'].append(trans_err)
        errors_by_condition[condition]['rot'].append(rot_err)
        errors_by_camera[camera]['trans'].append(trans_err)
        errors_by_camera[camera]['rot'].append(rot_err)

    all_trans = np.array(all_trans)
    all_rot = np.array(all_rot)
    thresholds = [(0.25, 2), (0.5, 5), (5.0, 10)]

    def compute_accuracy(t_arr, r_arr):
        result = {}
        for t_th, r_th in thresholds:
            pct = 100.0 * np.mean((t_arr < t_th) & (r_arr < r_th))
            result[f'{t_th}m_{r_th}deg'] = round(float(pct), 2)
        result['median_trans_m'] = round(float(np.median(t_arr)), 4)
        result['median_rot_deg'] = round(float(np.median(r_arr)), 4)
        result['mean_trans_m'] = round(float(np.mean(t_arr)), 4)
        result['mean_rot_deg'] = round(float(np.mean(r_arr)), 4)
        result['n_images'] = len(t_arr)
        return result

    results = {
        'overall': compute_accuracy(all_trans, all_rot),
        'by_condition': {},
        'by_camera': {},
        'day': None,
        'night': None,
    }

    for cond in CONDITIONS:
        if cond in errors_by_condition:
            t = np.array(errors_by_condition[cond]['trans'])
            r = np.array(errors_by_condition[cond]['rot'])
            results['by_condition'][cond] = compute_accuracy(t, r)

    for cam in sorted(errors_by_camera.keys()):
        t = np.array(errors_by_camera[cam]['trans'])
        r = np.array(errors_by_camera[cam]['rot'])
        results['by_camera'][cam] = compute_accuracy(t, r)

    night_conds = {'night', 'night-rain'}
    day_conds = set(CONDITIONS) - night_conds
    day_t, day_r, night_t, night_r = [], [], [], []
    for cond, errs in errors_by_condition.items():
        if cond in night_conds:
            night_t.extend(errs['trans'])
            night_r.extend(errs['rot'])
        elif cond in day_conds:
            day_t.extend(errs['trans'])
            day_r.extend(errs['rot'])

    if day_t:
        results['day'] = compute_accuracy(np.array(day_t), np.array(day_r))
    if night_t:
        results['night'] = compute_accuracy(np.array(night_t), np.array(night_r))

    return results


def print_evaluation(method_name, eval_results, note=''):
    """Pretty-print evaluation results"""
    if eval_results is None:
        log.info(f'{method_name}: No evaluation results (no GT overlap)')
        return

    ov = eval_results['overall']
    suffix = f'  [{note}]' if note else ''
    log.info(f'\n{"="*70}')
    log.info(f'  {method_name} - Evaluation Results{suffix}')
    log.info(f'{"="*70}')
    log.info(f'  Overall ({ov["n_images"]} images):')
    log.info(f'    (0.25m, 2deg):  {ov["0.25m_2deg"]:.1f}%')
    log.info(f'    (0.5m, 5deg):   {ov['0.5m_5deg']:.1f}%')
    log.info(f'    (5m, 10deg):    {ov['5.0m_10deg']:.1f}%')
    log.info(f'    Median: {ov["median_trans_m"]:.3f}m / {ov["median_rot_deg"]:.3f}deg')

    if eval_results['day']:
        d = eval_results['day']
        log.info(f'  Day ({d["n_images"]} imgs): '
                 f'{d["0.25m_2deg"]:.1f}% / {d["0.5m_5deg"]:.1f}% / {d["5.0m_10deg"]:.1f}%')
    if eval_results['night']:
        n = eval_results['night']
        log.info(f'  Night ({n["n_images"]} imgs): '
                 f'{n["0.25m_2deg"]:.1f}% / {n["0.5m_5deg"]:.1f}% / {n["5.0m_10deg"]:.1f}%')

    log.info(f'\n  {"Condition":<20} {"N":>5} {"0.25m/2":>10} {"0.5m/5":>10} {"5m/10":>10} {"Med.t":>8} {"Med.r":>8}')
    log.info(f'  {"-"*75}')
    for cond in CONDITIONS:
        if cond in eval_results['by_condition']:
            c = eval_results['by_condition'][cond]
            log.info(f'  {cond:<20} {c["n_images"]:>5} '
                     f'{c["0.25m_2deg"]:>9.1f}% {c["0.5m_5deg"]:>9.1f}% {c["5.0m_10deg"]:>9.1f}% '
                     f'{c["median_trans_m"]:>7.3f}m {c["median_rot_deg"]:>7.3f}d')
    log.info('')


# phase B: Submission Preparation
def prepare_submission(results_file, test_file, output_file):
    """Filter localization results to test split and format for visuallocalization.net"""
    test_names = set()
    with open(test_file) as f:
        for line in f:
            name = line.strip()
            if name:
                test_names.add(name)

    results = {}
    with open(results_file) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 8:
                results[parts[0]] = ' '.join(parts[1:])

    output_file = Path(output_file)
    output_file.parent.mkdir(parents=True, exist_ok=True)

    matched = 0
    with open(output_file, 'w') as f:
        for test_name in sorted(test_names):
            if test_name in results:
                f.write(f'{test_name} {results[test_name]}\n')
                matched += 1
                continue
            stem = test_name.rsplit('.', 1)[0]
            for ext in ['.jpg', '.png', '.JPG', '.PNG']:
                alt_name = stem + ext
                if alt_name in results:
                    f.write(f'{test_name} {results[alt_name]}\n')
                    matched += 1
                    break

    log.info(f'  Submission: {matched}/{len(test_names)} test images matched -> {output_file.name}')
    return matched


# phase C: Pipeline Execution
def ensure_prerequisites(outputs, sift_sfm, sfm_pairs, query_list_pattern):
    """Ensure shared prerequisites exist (sift_sfm, covis pairs, query lists)."""
    if not (sift_sfm / 'cameras.bin').exists():
        log.info('Building SIFT SfM model from NVM...')
        colmap_from_nvm.main(
            DATASET / '3D-models/all-merged/all.nvm',
            DATASET / '3D-models/overcast-reference.db',
            sift_sfm,
        )

    if not sfm_pairs.exists():
        log.info('Generating covisibility pairs...')
        from hloc import pairs_from_covisibility
        pairs_from_covisibility.main(sift_sfm, sfm_pairs, num_matched=NUM_COVIS)

    for condition in CONDITIONS:
        ql = Path(str(query_list_pattern).format(condition=condition))
        if not ql.exists():
            log.info(f'Generating query list for {condition}...')
            generate_query_list(DATASET, IMAGES / condition, str(ql))


def run_triangulation_subprocess(reference_sfm, sift_sfm, images, sfm_pairs,
                                  feature_path, sfm_match_path, method_name):
    """Run COLMAP triangulation in a subprocess to survive segfaults.

    Returns True on success, False on failure.
    Logs detailed diagnostics on failure.
    """
    tri_script = f"""\
import sys, os
sys.path.insert(0, '/workspace/third_party/hloc')
from pathlib import Path
from hloc import triangulation

# print memory before triangulationimport psutil
mem = psutil.virtual_memory()
print(f"[subprocess] RAM before: {{mem.used/1e9:.1f}}/{{mem.total/1e9:.1f}} GB ({{mem.percent}}%)")
print(f"[subprocess] Starting triangulation...", flush=True)

triangulation.main(
    Path('{reference_sfm}'),
    Path('{sift_sfm}'),
    Path('{images}'),
    Path('{sfm_pairs}'),
    Path('{feature_path}'),
    Path('{sfm_match_path}'),
)

mem = psutil.virtual_memory()
print(f"[subprocess] RAM after: {{mem.used/1e9:.1f}}/{{mem.total/1e9:.1f}} GB ({{mem.percent}}%)")
print("[subprocess] Triangulation completed successfully.", flush=True)
"""
    log.info(f'[{method_name}]   Running triangulation in subprocess (PID will follow)...')
    log_system_state(f'{method_name} pre-triangulation')

    try:
        proc = subprocess.Popen(
            [sys.executable, '-u', '-c', tri_script],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True,
        )
        log.info(f'[{method_name}]   Subprocess PID: {proc.pid}')

        # stream output line by line
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                log.info(f'[{method_name}]   {line}')

        proc.wait(timeout=7200)

        if proc.returncode == 0:
            log.info(f'[{method_name}]   Triangulation completed successfully (exit 0)')
            return True
        else:
            sig = signal_name(proc.returncode)
            log.error(f'[{method_name}]   Triangulation CRASHED: exit code {proc.returncode} ({sig})')
            log_system_state(f'{method_name} post-crash')

            # check if partial output exists
            db_path = Path(reference_sfm) / 'database.db'
            if db_path.exists():
                log.error(f'[{method_name}]   Partial database.db: {db_path.stat().st_size/1e9:.2f} GB')
            imgs_path = Path(reference_sfm) / 'images.bin'
            if imgs_path.exists():
                log.error(f'[{method_name}]   images.bin exists: {imgs_path.stat().st_size/1e6:.1f} MB')
            else:
                log.error(f'[{method_name}]   images.bin NOT created (crash during 3D triangulation)')

            return False

    except subprocess.TimeoutExpired:
        log.error(f'[{method_name}]   Triangulation TIMEOUT (>2 hours)')
        proc.kill()
        return False
    except Exception as e:
        log.error(f'[{method_name}]   Triangulation subprocess error: {e}')
        return False


def run_single_method(method, outputs, sift_sfm, sfm_pairs, query_list_pattern, timings):
    # XXX: depends on hloc internals, breaks if they refactor
    """Run the full pipeline for a single method configuration.

    If triangulation fails, falls back to SP+SG reference SfM (with a note in results).
    Returns (results_path, used_fallback_sfm) or (None, False) on total failure.
    """
    name = method['name']
    feature_conf = get_feature_conf(method['feature_conf_name'])
    matcher_conf = get_matcher_conf(method['matcher_conf_name'])
    retrieval_conf = get_retrieval_conf(method['retrieval_conf_name'])

    reference_sfm = outputs / method['sfm_name']
    results_path = outputs / method['results_name']
    used_fallback = False

    # If results already exist and are non-empty, skip
    if results_path.exists() and results_path.stat().st_size > 0:
        n_lines = sum(1 for _ in open(results_path))
        log.info(f'[{name}] Results already exist ({n_lines} poses). Skipping pipeline.')
        return results_path, False

    # --- Step 1: Extract features ---
    log.info(f'[{name}] Step 1/6: Feature extraction ({feature_conf["output"]})')
    t0 = time.time()
    feature_path = outputs / (feature_conf['output'] + '.h5')
    if feature_path.exists():
        log.info(f'[{name}]   Cached: {feature_path.name} ({feature_path.stat().st_size/1e9:.2f} GB)')
    else:
        if not check_disk_space(20):
            return None, False
        features = extract_features.main(feature_conf, IMAGES, outputs, as_half=True)
        feature_path = features
        log.info(f'[{name}]   Extracted: {feature_path.stat().st_size/1e9:.2f} GB')
    timings[name]['feature_extraction'] = time.time() - t0
    torch.cuda.empty_cache()

    # --- Step 2: Match SfM pairs ---
    log.info(f'[{name}] Step 2/6: SfM pair matching')
    t0 = time.time()
    sfm_match_path = outputs / f'{feature_conf["output"]}_{matcher_conf["output"]}_{sfm_pairs.stem}.h5'
    if sfm_match_path.exists():
        log.info(f'[{name}]   Cached: {sfm_match_path.name} ({sfm_match_path.stat().st_size/1e9:.2f} GB)')
    else:
        if not check_disk_space(20):
            return None, False
        match_features.main(matcher_conf, sfm_pairs, feature_conf['output'], outputs)
        log.info(f'[{name}]   Matched: {sfm_match_path.stat().st_size/1e9:.2f} GB')
    timings[name]['sfm_matching'] = time.time() - t0
    torch.cuda.empty_cache()

    # --- Step 3: Triangulate reference SfM ---
    log.info(f'[{name}] Step 3/6: Triangulation ({method["sfm_name"]})')
    t0 = time.time()
    if (reference_sfm / 'images.bin').exists():
        log.info(f'[{name}]   Cached: {reference_sfm.name}/')
    else:
        # clean up any partial previous attempt
        if reference_sfm.exists():
            log.info(f'[{name}]   Cleaning partial SfM dir: {reference_sfm.name}/')
            shutil.rmtree(reference_sfm)

        success = run_triangulation_subprocess(
            reference_sfm, sift_sfm, IMAGES, sfm_pairs,
            feature_path, sfm_match_path, name,
        )

        if not success:
            # fallback: use SP+SG reference SfM
            fallback_sfm = outputs / FALLBACK_SFM
            if (fallback_sfm / 'images.bin').exists() and fallback_sfm != reference_sfm:
                log.warning(f'[{name}]   FALLBACK: Using {FALLBACK_SFM} instead of {method["sfm_name"]}')
                log.warning(f'[{name}]   This means localization uses SP+SG 3D map but {name} features for matching.')
                log.warning(f'[{name}]   Results are still valid for comparing matching quality, but not a full pipeline test.')
                reference_sfm = fallback_sfm
                used_fallback = True
                # clean up failed directory
                failed_dir = outputs / method['sfm_name']
                if failed_dir.exists():
                    shutil.rmtree(failed_dir)
            else:
                log.error(f'[{name}]   No fallback SfM available. Skipping method entirely.')
                return None, False

    timings[name]['triangulation'] = time.time() - t0

    # --- Step 4: Global descriptors + retrieval pairs ---
    retrieval_name = retrieval_conf['output']
    loc_pairs = outputs / f'pairs-query-{method["retrieval_conf_name"]}{NUM_LOC}.txt'

    log.info(f'[{name}] Step 4/6: Retrieval ({retrieval_name})')
    t0 = time.time()
    global_desc_path = outputs / (retrieval_name + '.h5')
    if global_desc_path.exists():
        log.info(f'[{name}]   Cached: {global_desc_path.name}')
    else:
        extract_features.main(retrieval_conf, IMAGES, outputs)
    timings[name]['retrieval_extraction'] = time.time() - t0
    torch.cuda.empty_cache()

    t0 = time.time()
    if loc_pairs.exists():
        log.info(f'[{name}]   Cached: {loc_pairs.name}')
    else:
        pairs_from_retrieval.main(
            global_desc_path, loc_pairs, NUM_LOC,
            query_prefix=CONDITIONS,
            db_model=reference_sfm,
        )
    timings[name]['retrieval_pairs'] = time.time() - t0

    # --- Step 5: Match localization pairs ---
    log.info(f'[{name}] Step 5/6: Localization pair matching')
    t0 = time.time()
    loc_match_path = outputs / f'{feature_conf["output"]}_{matcher_conf["output"]}_{loc_pairs.stem}.h5'
    if loc_match_path.exists():
        log.info(f'[{name}]   Cached: {loc_match_path.name}')
    else:
        if not check_disk_space(20):
            return None, False
        match_features.main(matcher_conf, loc_pairs, feature_conf['output'], outputs)
    timings[name]['loc_matching'] = time.time() - t0
    torch.cuda.empty_cache()

    # --- Step 6: Localize ---
    log.info(f'[{name}] Step 6/6: Localization')
    t0 = time.time()
    localize_sfm.main(
        reference_sfm,
        Path(str(query_list_pattern).format(condition='*')),
        loc_pairs,
        feature_path,
        loc_match_path,
        results_path,
        covisibility_clustering=False,
        prepend_camera_name=True,
    )
    timings[name]['localization'] = time.time() - t0

    n_lines = sum(1 for _ in open(results_path))
    log.info(f'[{name}] Done! Localized {n_lines} images.' +
             (' [FALLBACK SfM]' if used_fallback else ''))
    return results_path, used_fallback


# phase D: Detailed Analysis
def find_hardest_images(all_eval, all_results_files, gt_poses):
    """Find images that are hardest to localize across all methods"""
    image_success = defaultdict(int)
    image_best_error = {}

    for method_name, eval_res in all_eval.items():
        if eval_res is None:
            continue
        results_file = all_results_files.get(method_name)
        if results_file is None:
            continue

        loc_poses = load_localization_results(results_file)
        for key in set(loc_poses.keys()) & set(gt_poses.keys()):
            est = loc_poses[key]
            gt = gt_poses[key]
            trans_err = np.linalg.norm(est['pos'] - gt['pos'])
            R_diff = est['R_w2c'] @ gt['R_w2c'].T
            rot_err = np.degrees(np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1)))

            if trans_err < 5.0 and rot_err < 10.0:
                image_success[key] += 1
            if key not in image_best_error or trans_err < image_best_error[key][0]:
                image_best_error[key] = (trans_err, rot_err, gt['condition'])

    all_gt_keys = set(gt_poses.keys())
    hardest = []
    for key in all_gt_keys:
        if image_success.get(key, 0) == 0 and key in image_best_error:
            t, r, cond = image_best_error[key]
            hardest.append({'image': key, 'condition': cond,
                           'best_trans_err': round(t, 2), 'best_rot_err': round(r, 2)})

    hardest.sort(key=lambda x: x['best_trans_err'], reverse=True)
    return hardest[:20]


def analyze_feature_counts(outputs):
    import h5py

    feature_files = list(outputs.glob('feats-*.h5'))
    counts = {}
    for ff in feature_files:
        if 'matches' in ff.name:
            continue
        fname = ff.stem
        try:
            with h5py.File(str(ff), 'r') as hf:
                cond_counts = defaultdict(list)
                for img_name in hf.keys():
                    parts = img_name.split('/')
                    cond = parts[0] if len(parts) == 3 else 'reference'
                    n_kp = hf[img_name]['keypoints'].shape[0]
                    cond_counts[cond].append(n_kp)

                avg_counts = {}
                for cond, kp_list in cond_counts.items():
                    avg_counts[cond] = {
                        'mean': round(float(np.mean(kp_list)), 1),
                        'median': round(float(np.median(kp_list)), 1),
                        'min': int(np.min(kp_list)),
                        'max': int(np.max(kp_list)),
                        'n_images': len(kp_list),
                    }
                counts[fname] = avg_counts
        except Exception as e:
            log.warning(f'Could not analyze {ff.name}: {e}')
    return counts


# phase F: Plots
def generate_all_plots(all_eval, timings, feature_counts):
    """Generate all comparison plots"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plot_dir = RESULTS_DIR / 'plots'
    plot_dir.mkdir(parents=True, exist_ok=True)

    valid_methods = {k: v for k, v in all_eval.items() if v is not None}
    if not valid_methods:
        log.warning('No valid evaluation results for plotting')
        return

    method_names = list(valid_methods.keys())
    short_names = [m.replace('SuperPoint+', 'SP+').replace('SuperGlue', 'SG')
                   .replace('LightGlue', 'LG').replace('+NetVLAD', '')
                   .replace('+OpenIBL', '+OI').replace('NN-ratio', 'NN') for m in method_names]
    colors = plt.cm.Set2(np.linspace(0, 1, max(len(method_names), 3)))

    # --- Plot 1: Accuracy heatmap ---
    fig, ax = plt.subplots(figsize=(14, max(4, len(method_names) * 0.8 + 2)))
    matrix = []
    for cond in CONDITIONS:
        col = []
        for mn in method_names:
            bc = valid_methods[mn].get('by_condition', {})
            col.append(bc.get(cond, {}).get('0.5m_5deg', 0))
        matrix.append(col)
    matrix = np.array(matrix).T
    im = ax.imshow(matrix, cmap='RdYlGn', vmin=0, vmax=100, aspect='auto')
    ax.set_xticks(range(len(CONDITIONS)))
    ax.set_xticklabels(CONDITIONS, rotation=45, ha='right', fontsize=9)
    ax.set_yticks(range(len(short_names)))
    ax.set_yticklabels(short_names, fontsize=9)
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            color = 'white' if matrix[i, j] < 50 else 'black'
            ax.text(j, i, f'{matrix[i,j]:.0f}', ha='center', va='center', fontsize=8, color=color)
    plt.colorbar(im, ax=ax, label='% localized (0.5m, 5 deg)')
    ax.set_title('Accuracy at (0.5m, 5 deg) - Method x Condition', fontsize=12)
    plt.tight_layout()
    plt.savefig(plot_dir / 'accuracy_heatmap.png', dpi=150)
    plt.close()
    log.info('  Saved accuracy_heatmap.png')

    # --- Plot 2: Grouped bar chart ---
    fig, ax = plt.subplots(figsize=(16, 6))
    x = np.arange(len(CONDITIONS))
    width = 0.8 / len(method_names)
    for i, (mn, sn) in enumerate(zip(method_names, short_names)):
        vals = [valid_methods[mn].get('by_condition', {}).get(c, {}).get('0.5m_5deg', 0) for c in CONDITIONS]
        ax.bar(x + i * width - 0.4 + width/2, vals, width, label=sn, color=colors[i])
    ax.set_xticks(x)
    ax.set_xticklabels(CONDITIONS, rotation=45, ha='right')
    ax.set_ylabel('% localized (0.5m, 5 deg)')
    ax.set_title('Accuracy by Condition')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(plot_dir / 'accuracy_by_condition.png', dpi=150)
    plt.close()
    log.info('  Saved accuracy_by_condition.png')

    # --- Plot 3: Day vs Night ---
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax_idx, (thresh, label) in enumerate(
        [('0.25m_2deg', '0.25m/2d'), ('0.5m_5deg', '0.5m/5d'), ('5.0m_10deg', '5m/10d')]
    ):
        day_vals = [valid_methods[mn].get('day', {}).get(thresh, 0) for mn in method_names]
        night_vals = [valid_methods[mn].get('night', {}).get(thresh, 0) for mn in method_names]
        x = np.arange(len(method_names))
        w = 0.35
        axes[ax_idx].bar(x - w/2, day_vals, w, label='Day', color='gold')
        axes[ax_idx].bar(x + w/2, night_vals, w, label='Night', color='midnightblue')
        axes[ax_idx].set_xticks(x)
        axes[ax_idx].set_xticklabels(short_names, rotation=45, ha='right', fontsize=8)
        axes[ax_idx].set_ylabel('% localized')
        axes[ax_idx].set_title(f'Day vs Night ({label})')
        axes[ax_idx].legend()
        axes[ax_idx].set_ylim(0, 105)
        axes[ax_idx].grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(plot_dir / 'day_vs_night.png', dpi=150)
    plt.close()
    log.info('  Saved day_vs_night.png')

    # --- Plot 4: Per-camera ---
    fig, ax = plt.subplots(figsize=(10, 5))
    cameras = ['left', 'rear', 'right']
    x = np.arange(len(method_names))
    width = 0.25
    cam_colors = ['steelblue', 'coral', 'mediumseagreen']
    for i, cam in enumerate(cameras):
        vals = [valid_methods[mn].get('by_camera', {}).get(cam, {}).get('0.5m_5deg', 0) for mn in method_names]
        ax.bar(x + i * width - width, vals, width, label=cam, color=cam_colors[i])
    ax.set_xticks(x)
    ax.set_xticklabels(short_names, rotation=45, ha='right')
    ax.set_ylabel('% localized (0.5m, 5 deg)')
    ax.set_title('Accuracy by Camera')
    ax.legend()
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(plot_dir / 'per_camera.png', dpi=150)
    plt.close()
    log.info('  Saved per_camera.png')

    # --- Plot 5: Speed vs Accuracy ---
    if timings:
        fig, ax = plt.subplots(figsize=(10, 6))
        for i, mn in enumerate(method_names):
            if mn in timings:
                total_time = sum(timings[mn].values()) / 3600
                acc = valid_methods[mn]['overall']['0.5m_5deg']
                ax.scatter(total_time, acc, s=100, color=colors[i], edgecolors='black', linewidths=0.5, zorder=3)
                ax.annotate(short_names[i], (total_time, acc), textcoords='offset points', xytext=(5, 5), fontsize=8)
        ax.set_xlabel('Total pipeline time (hours)')
        ax.set_ylabel('% localized (0.5m, 5 deg)')
        ax.set_title('Speed vs Accuracy')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(plot_dir / 'speed_vs_accuracy.png', dpi=150)
        plt.close()
        log.info('  Saved speed_vs_accuracy.png')

    # --- Plot 6: Cumulative error curves ---
    gt = load_ground_truth(TRAIN_FILE)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for i, mn in enumerate(method_names):
        results_file = OUTPUTS / [m for m in METHODS if m['name'] == mn][0]['results_name']
        if not results_file.exists():
            continue
        loc_poses = load_localization_results(results_file)
        matched = set(loc_poses.keys()) & set(gt.keys())
        if not matched:
            continue
        t_errs = sorted([np.linalg.norm(loc_poses[k]['pos'] - gt[k]['pos']) for k in matched])
        r_errs = sorted([
            np.degrees(np.arccos(np.clip((np.trace(loc_poses[k]['R_w2c'] @ gt[k]['R_w2c'].T) - 1) / 2, -1, 1)))
            for k in matched])
        cdf = np.linspace(0, 100, len(t_errs))
        axes[0].plot(t_errs, cdf, label=short_names[i], color=colors[i])
        axes[1].plot(r_errs, cdf, label=short_names[i], color=colors[i])
    axes[0].set_xlabel('Translation error (m)')
    axes[0].set_ylabel('% images')
    axes[0].set_title('Cumulative Translation Error')
    axes[0].set_xlim(0, 10)
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[1].set_xlabel('Rotation error (deg)')
    axes[1].set_ylabel('% images')
    axes[1].set_title('Cumulative Rotation Error')
    axes[1].set_xlim(0, 20)
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_dir / 'cumulative_error_curves.png', dpi=150)
    plt.close()
    log.info('  Saved cumulative_error_curves.png')

    # --- Plot 7: Feature count per condition ---
    if feature_counts:
        fig, ax = plt.subplots(figsize=(14, 6))
        feat_methods = list(feature_counts.keys())
        x = np.arange(len(CONDITIONS))
        width = 0.8 / max(len(feat_methods), 1)
        fc_colors = plt.cm.Set1(np.linspace(0, 1, len(feat_methods)))
        for i, fm in enumerate(feat_methods):
            vals = [feature_counts[fm].get(cond, {}).get('mean', 0) for cond in CONDITIONS]
            ax.bar(x + i * width - 0.4 + width/2, vals, width,
                   label=fm.replace('feats-', ''), color=fc_colors[i % len(fc_colors)])
        ax.set_xticks(x)
        ax.set_xticklabels(CONDITIONS, rotation=45, ha='right')
        ax.set_ylabel('Mean keypoints per image')
        ax.set_title('Feature Count by Condition')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3, axis='y')
        plt.tight_layout()
        plt.savefig(plot_dir / 'feature_count_per_condition.png', dpi=150)
        plt.close()
        log.info('  Saved feature_count_per_condition.png')


# phase F: Report Generation
def generate_report(all_eval, timings, feature_counts, hardest_images):
    """Generate REPORT.md with analysis"""
    lines = ['# RobotCar Seasons - hloc Ablation Study Results\n',
             f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}\n']

    lines.append('## Summary Table\n')
    lines.append('| Method | (0.25m,2d) | (0.5m,5d) | (5m,10d) | Med.t(m) | Med.r(d) | Day 0.5m/5d | Night 0.5m/5d | Time(h) | Note |')
    lines.append('|--------|-----------|----------|---------|---------|---------|------------|--------------|---------|------|')

    for m in METHODS:
        mn = m['name']
        if mn not in all_eval or all_eval[mn] is None:
            lines.append(f'| {mn} | - | - | - | - | - | - | - | - | FAILED |')
            continue
        ev = all_eval[mn]
        ov = ev['overall']
        d_val = ev.get('day', {}).get('0.5m_5deg', '-')
        n_val = ev.get('night', {}).get('0.5m_5deg', '-')
        t_hrs = sum(timings.get(mn, {}).values()) / 3600 if mn in timings else 0
        d_str = f'{d_val:.1f}%' if isinstance(d_val, (int, float)) else d_val
        n_str = f'{n_val:.1f}%' if isinstance(n_val, (int, float)) else n_val
        note = ev.get('_note', '')
        lines.append(
            f'| {mn} | {ov["0.25m_2deg"]:.1f}% | {ov["0.5m_5deg"]:.1f}% | '
            f'{ov["5.0m_10deg"]:.1f}% | {ov["median_trans_m"]:.3f} | '
            f'{ov["median_rot_deg"]:.3f} | {d_str} | {n_str} | {t_hrs:.1f} | {note} |')
    lines.append('')

    lines.append('## Comparison with Published Results\n')
    lines.append('Published SuperPoint+SuperGlue on RobotCar Seasons (Sarlin et al. 2020):')
    lines.append('- Day: ~49/70/88% at (0.25m,2d)/(0.5m,5d)/(5m,10d)')
    lines.append('- Night: ~17/27/40% at the same thresholds\n')
    lines.append('Note: Our evaluation uses the training split only (~1900 images). '
                 'Official numbers use the full test set via visuallocalization.net.\n')

    lines.append('## Analysis\n')
    valid = {k: v for k, v in all_eval.items() if v is not None}
    if valid:
        day_best = max(valid.items(), key=lambda x: x[1].get('day', {}).get('0.5m_5deg', 0))
        night_best = max(valid.items(), key=lambda x: x[1].get('night', {}).get('0.5m_5deg', 0))
        lines.append(f'**Best daytime:** {day_best[0]} ({day_best[1].get("day", {}).get("0.5m_5deg", 0):.1f}% at 0.5m/5d)\n')
        lines.append(f'**Best nighttime:** {night_best[0]} ({night_best[1].get("night", {}).get("0.5m_5deg", 0):.1f}% at 0.5m/5d)\n')

        sp_sg = valid.get('SuperPoint+SuperGlue')
        sp_lg = valid.get('SuperPoint+LightGlue')
        if sp_sg and sp_lg:
            sg_acc = sp_sg['overall']['0.5m_5deg']
            lg_acc = sp_lg['overall']['0.5m_5deg']
            diff = lg_acc - sg_acc
            verdict = 'matches SuperGlue' if abs(diff) < 1 else ('better' if diff > 0 else 'worse')
            lines.append(f'**LightGlue vs SuperGlue:** {lg_acc:.1f}% vs {sg_acc:.1f}% -> LightGlue {verdict}.\n')

        for alt_name in ['DISK+LightGlue', 'ALIKED+LightGlue']:
            alt = valid.get(alt_name)
            base = sp_lg or sp_sg
            if alt and base:
                lines.append(f'**{alt_name}:** {alt["overall"]["0.5m_5deg"]:.1f}% vs '
                             f'{base["overall"]["0.5m_5deg"]:.1f}% (SP baseline)\n')

        all_conds = {}
        for mn, ev in valid.items():
            for cond, cv in ev.get('by_condition', {}).items():
                all_conds.setdefault(cond, []).append(cv['0.5m_5deg'])
        if all_conds:
            hardest_cond = min(all_conds.items(), key=lambda x: np.mean(x[1]))
            easiest_cond = max(all_conds.items(), key=lambda x: np.mean(x[1]))
            lines.append(f'\n**Hardest condition:** {hardest_cond[0]} (avg {np.mean(hardest_cond[1]):.1f}%)')
            lines.append(f'\n**Easiest condition:** {easiest_cond[0]} (avg {np.mean(easiest_cond[1]):.1f}%)\n')

    if hardest_images:
        lines.append('\n## Hardest Images (failed by all methods at 5m/10d)\n')
        lines.append('| Image | Condition | Best trans err | Best rot err |')
        lines.append('|-------|-----------|---------------|-------------|')
        for img in hardest_images[:20]:
            lines.append(f'| {img["image"]} | {img["condition"]} | {img["best_trans_err"]:.1f}m | {img["best_rot_err"]:.1f}d |')
        lines.append('')

    lines.append('\n## Recommendations\n')
    lines.append('1. Use the best-performing method as the primary result for the thesis.')
    lines.append('2. Report per-condition breakdown to show robustness across weather.')
    lines.append('3. Submit test results to visuallocalization.net for official numbers.')
    lines.append('4. Night performance gap is a key challenge for future work.')
    lines.append('5. Compare pipeline runtime to highlight practical trade-offs.\n')

    report_text = '\n'.join(lines)
    report_path = RESULTS_DIR / 'REPORT.md'
    with open(report_path, 'w') as f:
        f.write(report_text)
    log.info(f'Report saved to {report_path}')
    return report_text


def main():
    """Run the full benchmark"""
    start_time = time.time()
    log.info('=' * 70)
    log.info('  RobotCar Seasons Full Benchmark')
    log.info(f'  Started: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    log.info('=' * 70)
    log_system_state('startup')

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    sift_sfm = OUTPUTS / 'sfm_sift'
    sfm_pairs = OUTPUTS / f'pairs-db-covis{NUM_COVIS}.txt'
    query_list_pattern = str(OUTPUTS / '{condition}_queries_with_intrinsics.txt')

    progress = {
        'status': 'running',
        'start_time': datetime.now().isoformat(),
        'methods': {},
        'current_method': None,
    }
    save_progress(progress)

    log.info('\n--- Ensuring prerequisites ---')
    ensure_prerequisites(OUTPUTS, sift_sfm, sfm_pairs, query_list_pattern)

    log.info('\n--- Loading ground truth ---')
    gt_poses = load_ground_truth(TRAIN_FILE)
    log.info(f'Loaded {len(gt_poses)} GT poses from training split')

    all_eval = {}
    all_results_files = {}
    timings = defaultdict(dict)

    for method_idx, method in enumerate(METHODS):
        method_name = method['name']
        log.info(f'\n{"#"*70}')
        log.info(f'# Method {method_idx+1}/{len(METHODS)}: {method_name}')
        log.info(f'{"#"*70}')
        log_system_state(method_name)

        progress['current_method'] = f'{method_idx+1}/{len(METHODS)}: {method_name}'
        progress['methods'][method_name] = {'status': 'running'}
        save_progress(progress)

        try:
            method_start = time.time()
            used_fallback = False

            if method.get('already_done'):
                results_path = OUTPUTS / method['results_name']
                if not results_path.exists():
                    log.error(f'[{method_name}] Results file not found: {results_path}')
                    progress['methods'][method_name] = {'status': 'error', 'error': 'results not found'}
                    save_progress(progress)
                    continue
                timings[method_name] = {'total': 0}
            else:
                result = run_single_method(
                    method, OUTPUTS, sift_sfm, sfm_pairs, query_list_pattern, timings
                )
                results_path, used_fallback = result if result[0] is not None else (None, False)
                if results_path is None:
                    progress['methods'][method_name] = {'status': 'error', 'error': 'pipeline failed'}
                    save_progress(progress)
                    continue

            all_results_files[method_name] = results_path

            log.info(f'\n[{method_name}] Evaluating against GT...')
            loc_poses = load_localization_results(results_path)
            log.info(f'[{method_name}] Loaded {len(loc_poses)} localization results')

            eval_results = evaluate_method(loc_poses, gt_poses)
            if eval_results and used_fallback:
                eval_results['_note'] = 'fallback SfM (own triangulation crashed)'
            all_eval[method_name] = eval_results
            note = 'FALLBACK SfM' if used_fallback else ''
            print_evaluation(method_name, eval_results, note=note)

            method_dir = RESULTS_DIR / method['short']
            method_dir.mkdir(parents=True, exist_ok=True)
            if eval_results:
                with open(method_dir / 'eval_results.json', 'w') as f:
                    json.dump(eval_results, f, indent=2)

            method_time = time.time() - method_start
            timings[method_name]['total'] = method_time
            progress['methods'][method_name] = {
                'status': 'done',
                'time_s': round(method_time, 1),
                'n_localized': len(loc_poses),
                'used_fallback_sfm': used_fallback,
            }
            if eval_results:
                progress['methods'][method_name]['accuracy_0.5m_5deg'] = eval_results['overall']['0.5m_5deg']
            save_progress(progress)

            log.info(f'[{method_name}] Total time: {method_time/60:.1f} min')

        except Exception as e:
            log.error(f'[{method_name}] FAILED with exception: {e}')
            log.error(traceback.format_exc())
            log_system_state(f'{method_name} post-error')
            progress['methods'][method_name] = {'status': 'error', 'error': str(e)}
            save_progress(progress)
            torch.cuda.empty_cache()
            continue

    # ---- Phase B: Prepare test submissions ----
    log.info(f'\n{"="*70}')
    log.info('  Phase B: Preparing test submissions')
    log.info(f'{"="*70}')
    for method in METHODS:
        results_path = OUTPUTS / method['results_name']
        if results_path.exists():
            sub_path = RESULTS_DIR / 'submissions' / f'{method["short"]}_submission.txt'
            prepare_submission(results_path, TEST_FILE, sub_path)

    # ---- Phase D: Detailed analysis ----
    log.info(f'\n{"="*70}')
    log.info('  Phase D: Detailed analysis')
    log.info(f'{"="*70}')

    hardest_images = find_hardest_images(all_eval, all_results_files, gt_poses)
    if hardest_images:
        log.info(f'Found {len(hardest_images)} images that no method localized within (5m, 10deg)')
        for img in hardest_images[:5]:
            log.info(f'  {img["image"]} ({img["condition"]}): {img["best_trans_err"]:.1f}m / {img["best_rot_err"]:.1f}deg')

    log.info('\nAnalyzing feature counts per condition...')
    feature_counts = analyze_feature_counts(OUTPUTS)
    for fc_name, conds in feature_counts.items():
        log.info(f'  {fc_name}:')
        for cond in CONDITIONS:
            if cond in conds:
                log.info(f'    {cond}: mean={conds[cond]["mean"]:.0f}, min={conds[cond]["min"]}, max={conds[cond]["max"]}')

    # ---- Phase F: Plots ----
    log.info(f'\n{"="*70}')
    log.info('  Phase F: Generating plots')
    log.info(f'{"="*70}')
    try:
        generate_all_plots(all_eval, dict(timings), feature_counts)
    except Exception as e:
        log.error(f'Plot generation failed: {e}')
        log.error(traceback.format_exc())

    # ---- Phase F: Report ----
    log.info(f'\n{"="*70}')
    log.info('  Phase F: Generating report')
    log.info(f'{"="*70}')
    try:
        report = generate_report(all_eval, dict(timings), feature_counts, hardest_images)
        log.info('\n' + report)
    except Exception as e:
        log.error(f'Report generation failed: {e}')
        log.error(traceback.format_exc())

    # ---- Save all results ----
    log.info(f'\n{"="*70}')
    log.info('  Saving all results')
    log.info(f'{"="*70}')

    def to_serializable(obj):
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    all_results = {
        'methods': {},
        'timings': {},
        'feature_counts': feature_counts,
        'hardest_images': hardest_images,
        'generated': datetime.now().isoformat(),
        'total_time_hours': (time.time() - start_time) / 3600,
    }
    for m in METHODS:
        mn = m['name']
        if mn in all_eval and all_eval[mn] is not None:
            all_results['methods'][mn] = all_eval[mn]
        if mn in timings:
            all_results['timings'][mn] = {k: round(v, 1) for k, v in timings[mn].items()}

    with open(RESULTS_DIR / 'all_results.json', 'w') as f:
        json.dump(all_results, f, indent=2, default=to_serializable)
    log.info('Saved all_results.json')

    total_time = time.time() - start_time
    progress['status'] = 'done'
    progress['total_time_hours'] = round(total_time / 3600, 2)
    save_progress(progress)

    log.info(f'\n{"="*70}')
    log.info(f'  BENCHMARK COMPLETE')
    log.info(f'  Total time: {total_time/3600:.1f} hours')
    log.info(f'  Results: {RESULTS_DIR}')
    log.info(f'{"="*70}')
    log_system_state('final')

    log.info('\n--- SUBMISSION INSTRUCTIONS ---')
    log.info('Submit results at: https://www.visuallocalization.net/')
    log.info(f'Submission files: {RESULTS_DIR / "submissions"}/')
    for f in sorted((RESULTS_DIR / 'submissions').glob('*.txt')):
        log.info(f'  - {f.name}')


if __name__ == '__main__':
    main()
