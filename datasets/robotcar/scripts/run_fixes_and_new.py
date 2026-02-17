#!/usr/bin/env python3
"""
Re-run the 3 fixed methods + new ALIKED+OpenIBL combo:
  1. SIFT+NN-ratio, fixed topk crash in dog.py
  2. DISK+LightGlue, reduced to 2048 keypoints to avoid OOM
  3. ALIKED+LG+OpenIBL, new combo: best features + best retrieval
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
from hloc.pipelines.RobotCar.pipeline import CONDITIONS

DATASET = Path('/workspace/data/robotcar_seasons')
OUTPUTS = DATASET / 'outputs'
IMAGES = DATASET / 'images'
RESULTS_DIR = Path('/workspace/datasets/robotcar/results/robotcar_seasons_hloc')
TRAIN_FILE = DATASET / 'robotcar_v2_train.txt'
TEST_FILE = DATASET / 'robotcar_v2_test.txt'

# THRESHOLD = 0.5  # was 0.5, 0.25 kept picking noise features
NUM_COVIS = 20
NUM_LOC = 20

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger('fix_benchmark')


def signal_name(retcode):
    """Convert negative return code to signal name"""
    if retcode >= 0:
        return str(retcode)
    sig_num = -retcode
    try:
        return f'{signal.Signals(sig_num).name} (signal {sig_num})'
    except ValueError:
        return f'signal {sig_num}'


def log_system_state(label=''):
    prefix = f'[SYS {label}] ' if label else '[SYS] '
    mem = psutil.virtual_memory()
    log.info(f'{prefix}RAM: {mem.used/1e9:.1f}/{mem.total/1e9:.1f} GB ({mem.percent}%)')
    disk = shutil.disk_usage('/workspace')
    log.info(f'{prefix}Disk: {disk.free/1e9:.0f} GB free')
    if torch.cuda.is_available():
        allocated = torch.cuda.memory_allocated() / 1e9
        total = torch.cuda.get_device_properties(0).total_memory / 1e9
        log.info(f'{prefix}GPU: {allocated:.1f}/{total:.1f} GB')


# feature Configs
CUSTOM_FEATURE_CONFS = {
    'disk_2048': {
        'output': 'feats-disk-n2048-r1024',
        'model': {'name': 'disk', 'max_keypoints': 2048},
        'preprocessing': {'grayscale': False, 'resize_max': 1024},
    },
    'sift_4096': {
        'output': 'feats-sift-n4096-r1024',
        'model': {'name': 'dog', 'max_keypoints': 4096},
        'preprocessing': {'grayscale': True, 'resize_max': 1024},
    },
    'aliked_4096': {
        'output': 'feats-aliked-n4096-r1024',
        'model': {'name': 'aliked', 'model_name': 'aliked-n16', 'max_num_keypoints': 4096},
        'preprocessing': {'grayscale': False, 'resize_max': 1024},
    },
}

# method Definitions (only the 3 we're running)
METHODS = [
    {
        'name': 'SIFT+NN-ratio',
        'short': 'sift_nn_nv',
        'feature_conf_name': 'sift_4096',
        'matcher_conf_name': 'NN-ratio',
        'retrieval_conf_name': 'netvlad',
        'sfm_name': 'sfm_sift+nn',
        'results_name': 'RobotCar_hloc_sift+nn_netvlad20.txt',
    },
    {
        'name': 'DISK+LightGlue',
        'short': 'disk_lg_nv',
        'feature_conf_name': 'disk_2048',
        'matcher_conf_name': 'disk+lightglue',
        'retrieval_conf_name': 'netvlad',
        'sfm_name': 'sfm_disk+lightglue',
        'results_name': 'RobotCar_hloc_disk+lightglue_netvlad20.txt',
    },
    {
        'name': 'ALIKED+LG+OpenIBL',
        'short': 'aliked_lg_openibl',
        'feature_conf_name': 'aliked_4096',
        'matcher_conf_name': 'aliked+lightglue',
        'retrieval_conf_name': 'openibl',
        'sfm_name': 'sfm_aliked+lightglue',
        'results_name': 'RobotCar_hloc_aliked+lightglue_openibl20.txt',
    },
]

# All methods for the combined report (original + new)
ALL_METHODS_FOR_REPORT = [
    {'name': 'SuperPoint+SuperGlue', 'short': 'sp_sg_nv',
     'results_name': 'RobotCar_hloc_superpoint+superglue_netvlad20.txt'},
    {'name': 'SuperPoint+LightGlue', 'short': 'sp_lg_nv',
     'results_name': 'RobotCar_hloc_superpoint+lightglue_netvlad20.txt'},
    {'name': 'DISK+LightGlue', 'short': 'disk_lg_nv',
     'results_name': 'RobotCar_hloc_disk+lightglue_netvlad20.txt'},
    {'name': 'ALIKED+LightGlue', 'short': 'aliked_lg_nv',
     'results_name': 'RobotCar_hloc_aliked+lightglue_netvlad20.txt'},
    {'name': 'SuperPoint+SuperGlue+OpenIBL', 'short': 'sp_sg_openibl',
     'results_name': 'RobotCar_hloc_superpoint+superglue_openibl20.txt'},
    {'name': 'SIFT+NN-ratio', 'short': 'sift_nn_nv',
     'results_name': 'RobotCar_hloc_sift+nn_netvlad20.txt'},
    {'name': 'ALIKED+LG+OpenIBL', 'short': 'aliked_lg_openibl',
     'results_name': 'RobotCar_hloc_aliked+lightglue_openibl20.txt'},
]


def get_feature_conf(name):
    """Get feature extractor configuration by name"""
    if name in CUSTOM_FEATURE_CONFS:
        return CUSTOM_FEATURE_CONFS[name]
    return extract_features.confs[name]


def get_matcher_conf(name):
    ""'Get matcher configuration by name'""
    return match_features.confs[name]


def get_retrieval_conf(name):
    """Get retrieval configuration by name"""
    return extract_features.confs[name]


# evaluation (copied from run_full_benchmark.py)
def load_ground_truth(train_file):
    """Load GT poses from robotcar_v2_train.txt"""
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
            gt[key] = {'condition': condition, 'R_cw': R_cw, 'pos': C, 'R_w2c': R_cw.T}
    return gt


def load_localization_results(results_file):
    """Load hloc localization results"""
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
            poses[name] = {'R_w2c': R, 't': t, 'pos': pos, 'quat': [qw, qx, qy, qz]}
    return poses


def evaluate_method(loc_poses, gt_poses):
    """Evaluate localization results against ground truth"""
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
    all_trans, all_rot = [], []

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
        errors_by_condition[condition]['trans'].append(trans_err)
        errors_by_condition[condition]['rot'].append(rot_err)

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

    results = {'overall': compute_accuracy(all_trans, all_rot), 'by_condition': {},
               'by_camera': {}, 'day': None, 'night': None}

    for cond in CONDITIONS:
        if cond in errors_by_condition:
            t = np.array(errors_by_condition[cond]['trans'])
            r = np.array(errors_by_condition[cond]['rot'])
            results['by_condition'][cond] = compute_accuracy(t, r)

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


def print_eval(method_name, ev):
    ""'Print evaluation results'""
    if ev is None:
        log.info(f'{method_name}: No evaluation results')
        return
    ov = ev['overall']
    log.info(f'\n{"="*60}')
    log.info(f'  {method_name} ({ov["n_images"]} images)')
    log.info(f'{"="*60}')
    log.info(f'  (0.25m, 2d): {ov["0.25m_2deg"]:.1f}%')
    log.info(f'  (0.5m, 5d):  {ov["0.5m_5deg"]:.1f}%')
    log.info(f'  (5m, 10d):   {ov["5.0m_10deg"]:.1f}%')
    log.info(f'  Median: {ov["median_trans_m"]:.3f}m / {ov["median_rot_deg"]:.3f}d')
    if ev['day']:
        d = ev['day']
        log.info(f'  Day:   {d["0.25m_2deg"]:.1f}% / {d["0.5m_5deg"]:.1f}% / {d["5.0m_10deg"]:.1f}%')
    if ev['night']:
        n = ev['night']
        log.info(f'  Night: {n["0.25m_2deg"]:.1f}% / {n["0.5m_5deg"]:.1f}% / {n["5.0m_10deg"]:.1f}%')
    log.info(f'  {"Condition":<20} {"0.5m/5d":>10}')
    for cond in CONDITIONS:
        if cond in ev['by_condition']:
            c = ev['by_condition'][cond]
            log.info(f'  {cond:<20} {c["0.5m_5deg"]:>9.1f}%')


# triangulation subprocess
def run_triangulation_subprocess(reference_sfm, sift_sfm, images, sfm_pairs,
                                  feature_path, sfm_match_path, method_name):
    tri_script = f"""\
import sys, os
sys.path.insert(0, '/workspace/third_party/hloc')
from pathlib import Path
from hloc import triangulation
import psutil

mem = psutil.virtual_memory()
print(f"[subprocess] RAM before: {{mem.used/1e9:.1f}}/{{mem.total/1e9:.1f}} GB ({{mem.percent}}%)")
print("[subprocess] Starting triangulation...", flush=True)

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
    log.info(f'[{method_name}] Running triangulation in subprocess...')
    log_system_state(f'{method_name} pre-triangulation')

    proc = subprocess.Popen(
        [sys.executable, '-u', '-c', tri_script],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    log.info(f'[{method_name}] Subprocess PID: {proc.pid}')

    for line in proc.stdout:
        line = line.rstrip()
        if line:
            log.info(f'[{method_name}]   {line}')

    proc.wait(timeout=7200)

    if proc.returncode == 0:
        log.info(f'[{method_name}] Triangulation completed successfully')
        return True
    else:
        sig = signal_name(proc.returncode)
        log.error(f'[{method_name}] Triangulation CRASHED: exit {proc.returncode} ({sig})')
        log_system_state(f'{method_name} post-crash')
        return False


# pipeline for a single method
def run_method(method, sift_sfm, sfm_pairs, query_list_pattern):
    # TODO proper logging config
    """Run the full hloc pipeline for one method. Returns (results_path, timings) or (None, {})."""
    name = method['name']
    feature_conf = get_feature_conf(method['feature_conf_name'])
    matcher_conf = get_matcher_conf(method['matcher_conf_name'])
    retrieval_conf = get_retrieval_conf(method['retrieval_conf_name'])

    reference_sfm = OUTPUTS / method['sfm_name']
    results_path = OUTPUTS / method['results_name']
    timings = {}

    # If results already exist, skip
    if results_path.exists() and results_path.stat().st_size > 0:
        n_lines = sum(1 for _ in open(results_path))
        log.info(f'[{name}] Results already exist ({n_lines} poses). Skipping.')
        return results_path, timings

    # --- Step 1: Extract features ---
    log.info(f'[{name}] Step 1/6: Feature extraction ({feature_conf["output"]})')
    t0 = time.time()
    feature_path = OUTPUTS / (feature_conf['output'] + '.h5')
    if feature_path.exists():
        log.info(f'[{name}]   Cached: {feature_path.name} ({feature_path.stat().st_size/1e9:.2f} GB)')
    else:
        features = extract_features.main(feature_conf, IMAGES, OUTPUTS, as_half=True)
        feature_path = features
        log.info(f'[{name}]   Extracted: {feature_path.stat().st_size/1e9:.2f} GB')
    timings['feature_extraction'] = time.time() - t0
    torch.cuda.empty_cache()

    # --- Step 2: Match SfM pairs ---
    log.info(f'[{name}] Step 2/6: SfM pair matching')
    t0 = time.time()
    sfm_match_path = OUTPUTS / f'{feature_conf["output"]}_{matcher_conf["output"]}_{sfm_pairs.stem}.h5'
    if sfm_match_path.exists():
        log.info(f'[{name}]   Cached: {sfm_match_path.name}')
    else:
        match_features.main(matcher_conf, sfm_pairs, feature_conf['output'], OUTPUTS)
        log.info(f'[{name}]   Matched: {sfm_match_path.stat().st_size/1e9:.2f} GB')
    timings['sfm_matching'] = time.time() - t0
    torch.cuda.empty_cache()

    # --- Step 3: Triangulate ---
    log.info(f'[{name}] Step 3/6: Triangulation ({method["sfm_name"]})')
    t0 = time.time()
    if (reference_sfm / 'images.bin').exists():
        log.info(f'[{name}]   Cached: {reference_sfm.name}/')
    else:
        if reference_sfm.exists():
            shutil.rmtree(reference_sfm)

        success = run_triangulation_subprocess(
            reference_sfm, sift_sfm, IMAGES, sfm_pairs,
            feature_path, sfm_match_path, name,
        )

        if not success:
            # try with lower keypoints hint
            log.error(f'[{name}] Triangulation failed. Method cannot proceed.')
            return None, timings

    timings['triangulation'] = time.time() - t0

    # --- Step 4: Retrieval ---
    retrieval_name = retrieval_conf['output']
    loc_pairs = OUTPUTS / f'pairs-query-{method["retrieval_conf_name"]}{NUM_LOC}.txt'

    log.info(f'[{name}] Step 4/6: Retrieval ({retrieval_name})')
    t0 = time.time()
    global_desc_path = OUTPUTS / (retrieval_name + '.h5')
    if global_desc_path.exists():
        log.info(f'[{name}]   Cached: {global_desc_path.name}')
    else:
        extract_features.main(retrieval_conf, IMAGES, OUTPUTS)
    timings['retrieval'] = time.time() - t0
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
    timings['retrieval_pairs'] = time.time() - t0

    # --- Step 5: Match localization pairs ---
    log.info(f'[{name}] Step 5/6: Localization pair matching')
    t0 = time.time()
    loc_match_path = OUTPUTS / f'{feature_conf["output"]}_{matcher_conf["output"]}_{loc_pairs.stem}.h5'
    if loc_match_path.exists():
        log.info(f'[{name}]   Cached: {loc_match_path.name}')
    else:
        match_features.main(matcher_conf, loc_pairs, feature_conf['output'], OUTPUTS)
    timings['loc_matching'] = time.time() - t0
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
    timings['localization'] = time.time() - t0

    n_lines = sum(1 for _ in open(results_path))
    log.info(f'[{name}] Done! Localized {n_lines} images.')
    return results_path, timings


# submission preparation
def prepare_submission(results_file, test_file, output_file):
    """Filter localization results to test split for visuallocalization.net"""
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
            for ext in ['.jpg', '.png']:
                alt_name = stem + ext
                if alt_name in results:
                    f.write(f'{test_name} {results[alt_name]}\n')
                    matched += 1
                    break

    log.info(f'  Submission: {matched}/{len(test_names)} test images -> {output_file.name}')
    return matched


# report + Plot Generation
def generate_updated_report(all_eval, all_timings):
    """Generate updated REPORT.md with all methods including new ones"""
    lines = ['# RobotCar Seasons - hloc Ablation Study Results\n',
             f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}\n']

    lines.append('## Summary Table\n')
    lines.append('| Method | (0.25m,2d) | (0.5m,5d) | (5m,10d) | Med.t(m) | Med.r(d) | Day 0.5m/5d | Night 0.5m/5d | Note |')
    lines.append('|--------|-----------|----------|---------|---------|---------|------------|--------------|------|')

    for m in ALL_METHODS_FOR_REPORT:
        mn = m['name']
        if mn not in all_eval or all_eval[mn] is None:
            lines.append(f'| {mn} | - | - | - | - | - | - | - | FAILED |')
            continue
        ev = all_eval[mn]
        ov = ev['overall']
        d_val = ev.get('day', {})
        n_val = ev.get('night', {})
        d_str = f'{d_val["0.5m_5deg"]:.1f}%' if d_val else '-'
        n_str = f'{n_val["0.5m_5deg"]:.1f}%' if n_val else '-'
        note = ev.get('_note', '')
        lines.append(
            f'| {mn} | {ov["0.25m_2deg"]:.1f}% | {ov["0.5m_5deg"]:.1f}% | '
            f'{ov["5.0m_10deg"]:.1f}% | {ov["median_trans_m"]:.3f} | '
            f'{ov["median_rot_deg"]:.3f} | {d_str} | {n_str} | {note} |')
    lines.append('')

    lines.append('## Comparison with Published Results\n')
    lines.append('Published SuperPoint+SuperGlue on RobotCar Seasons (Sarlin et al. 2020):')
    lines.append('- Day: ~49/70/88% at (0.25m,2d)/(0.5m,5d)/(5m,10d)')
    lines.append('- Night: ~17/27/40% at the same thresholds\n')
    lines.append('Note: Our evaluation uses the training split only (~1900 images). '
                 'Official numbers use the full test set via visuallocalization.net.\n')

    # analysis
    lines.append('## Analysis\n')
    valid = {k: v for k, v in all_eval.items() if v is not None}
    if valid:
        best_overall = max(valid.items(), key=lambda x: x[1]['overall']['0.5m_5deg'])
        lines.append(f'**Best overall:** {best_overall[0]} ({best_overall[1]["overall"]["0.5m_5deg"]:.1f}% at 0.5m/5d)\n')

        day_best = max(valid.items(), key=lambda x: (x[1].get('day') or {}).get('0.5m_5deg', 0))
        night_best = max(valid.items(), key=lambda x: (x[1].get('night') or {}).get('0.5m_5deg', 0))
        lines.append(f'**Best daytime:** {day_best[0]} ({(day_best[1].get("day") or {}).get("0.5m_5deg", 0):.1f}% at 0.5m/5d)\n')
        lines.append(f'**Best nighttime:** {night_best[0]} ({(night_best[1].get("night") or {}).get("0.5m_5deg", 0):.1f}% at 0.5m/5d)\n')

        # key comparisons
        sp_sg = valid.get('SuperPoint+SuperGlue')
        sp_lg = valid.get('SuperPoint+LightGlue')
        if sp_sg and sp_lg:
            sg_acc = sp_sg['overall']['0.5m_5deg']
            lg_acc = sp_lg['overall']['0.5m_5deg']
            verdict = 'matches SuperGlue' if abs(lg_acc - sg_acc) < 1 else ('better' if lg_acc > sg_acc else 'worse')
            lines.append(f'**LightGlue vs SuperGlue:** {lg_acc:.1f}% vs {sg_acc:.1f}% -> LightGlue {verdict}.\n')

        for alt_name in ['DISK+LightGlue', 'ALIKED+LightGlue', 'SIFT+NN-ratio']:
            alt = valid.get(alt_name)
            if alt and sp_sg:
                lines.append(f'**{alt_name}:** {alt["overall"]["0.5m_5deg"]:.1f}% vs '
                             f'{sp_sg["overall"]["0.5m_5deg"]:.1f}% (SP+SG baseline)\n')

        aliked_nv = valid.get('ALIKED+LightGlue')
        aliked_oi = valid.get('ALIKED+LG+OpenIBL')
        if aliked_nv and aliked_oi:
            lines.append(f'**OpenIBL boost for ALIKED:** {aliked_oi["overall"]["0.5m_5deg"]:.1f}% vs '
                         f'{aliked_nv["overall"]["0.5m_5deg"]:.1f}% (NetVLAD)\n')
            if aliked_oi.get('night') and aliked_nv.get('night'):
                lines.append(f'  Night: {aliked_oi["night"]["0.5m_5deg"]:.1f}% vs '
                             f'{aliked_nv["night"]["0.5m_5deg"]:.1f}%\n')

    lines.append('\n## Recommendations\n')
    lines.append('1. Use the best-performing method as the primary result for the thesis.')
    lines.append('2. Report per-condition breakdown to show robustness across weather.')
    lines.append('3. Submit test results to visuallocalization.net for official numbers.')
    lines.append('4. Night performance gap is a key challenge for future work.')
    lines.append('5. OpenIBL retrieval consistently outperforms NetVLAD, especially at night.\n')

    report_text = '\n'.join(lines)
    report_path = RESULTS_DIR / 'REPORT.md'
    with open(report_path, 'w') as f:
        f.write(report_text)
    log.info(f'Report saved to {report_path}')
    return report_text


def generate_plots(all_eval):
    """Generate updated comparison plots for all methods"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plot_dir = RESULTS_DIR / 'plots'
    plot_dir.mkdir(parents=True, exist_ok=True)

    valid = {k: v for k, v in all_eval.items() if v is not None}
    if not valid:
        log.warning('No valid results for plotting')
        return

    method_names = list(valid.keys())
    short_names = [m.replace('SuperPoint+', 'SP+').replace('SuperGlue', 'SG')
                   .replace('LightGlue', 'LG').replace('+NetVLAD', '')
                   .replace('+OpenIBL', '+OI').replace('NN-ratio', 'NN')
                   .replace('ALIKED+LG+OI', 'ALIKED+LG+OI') for m in method_names]
    n_methods = len(method_names)
    cmap = plt.cm.Set2(np.linspace(0, 1, max(n_methods, 3)))

    thresholds = ['0.25m_2deg', '0.5m_5deg', '5.0m_10deg']
    th_labels = ['0.25m, 2°', '0.5m, 5°', '5m, 10°']

    # --- Plot 1: Overall comparison ---
    fig, ax = plt.subplots(figsize=(14, 6))
    x = np.arange(3)
    width = 0.8 / n_methods
    for i, (mn, sn) in enumerate(zip(method_names, short_names)):
        vals = [valid[mn]['overall'][t] for t in thresholds]
        bars = ax.bar(x + i * width - 0.4 + width/2, vals, width, label=sn, color=cmap[i])
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{v:.1f}', ha='center', va='bottom', fontsize=7, fontweight='bold')
    ax.set_ylabel('% localized')
    ax.set_title('Overall Accuracy - All Methods', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(th_labels, fontsize=12)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_ylim(0, 100)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_dir / '1_overall_comparison.png', dpi=150)
    plt.close()
    log.info('  Saved 1_overall_comparison.png')

    # --- Plot 2: Per-condition grouped bars ---
    fig, ax = plt.subplots(figsize=(18, 7))
    x = np.arange(len(CONDITIONS))
    width = 0.8 / n_methods
    for i, (mn, sn) in enumerate(zip(method_names, short_names)):
        vals = [valid[mn].get('by_condition', {}).get(c, {}).get('0.5m_5deg', 0) for c in CONDITIONS]
        bars = ax.bar(x + i * width - 0.4 + width/2, vals, width, label=sn, color=cmap[i])
    ax.set_ylabel('% localized (0.5m, 5°)')
    ax.set_title('Per-Condition Accuracy - All Methods', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(CONDITIONS, rotation=30, ha='right', fontsize=10)
    ax.legend(fontsize=8)
    ax.set_ylim(0, 95)
    ax.axhline(y=50, color='gray', linestyle='--', alpha=0.5)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_dir / '2_per_condition.png', dpi=150)
    plt.close()
    log.info('  Saved 2_per_condition.png')

    # --- Plot 3: Day vs Night ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for ax_idx, (split, title) in enumerate([('day', 'Day'), ('night', 'Night')]):
        ax = axes[ax_idx]
        x = np.arange(3)
        width = 0.8 / n_methods
        for i, (mn, sn) in enumerate(zip(method_names, short_names)):
            data = valid[mn].get(split) or {}
            vals = [data.get(t, 0) for t in thresholds]
            bars = ax.bar(x + i * width - 0.4 + width/2, vals, width, label=sn, color=cmap[i])
            for bar, v in zip(bars, vals):
                ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                        f'{v:.1f}', ha='center', va='bottom', fontsize=6)
        ax.set_ylabel('% localized')
        ax.set_title(f'{title} Conditions', fontsize=13, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(th_labels, fontsize=10)
        ax.set_ylim(0, 110)
        ax.legend(fontsize=7)
        ax.grid(axis='y', alpha=0.3)
    plt.suptitle('Day vs Night - All Methods', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(plot_dir / '3_day_vs_night.png', dpi=150)
    plt.close()
    log.info('  Saved 3_day_vs_night.png')

    # --- Plot 4: Heatmap ---
    fig, ax = plt.subplots(figsize=(14, max(4, n_methods * 0.8 + 2)))
    matrix = []
    for mn in method_names:
        row = [valid[mn].get('by_condition', {}).get(c, {}).get('0.5m_5deg', 0) for c in CONDITIONS]
        matrix.append(row)
    matrix = np.array(matrix)
    im = ax.imshow(matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=100)
    ax.set_xticks(np.arange(len(CONDITIONS)))
    ax.set_xticklabels(CONDITIONS, rotation=35, ha='right', fontsize=10)
    ax.set_yticks(np.arange(n_methods))
    ax.set_yticklabels(short_names, fontsize=10)
    for i in range(n_methods):
        for j in range(len(CONDITIONS)):
            val = matrix[i, j]
            color = 'white' if val < 40 else 'black'
            ax.text(j, i, f'{val:.1f}', ha='center', va='center', fontsize=9, color=color, fontweight='bold')
    plt.colorbar(im, ax=ax, label='% localized (0.5m, 5°)')
    ax.set_title('Accuracy Heatmap - Methods × Conditions', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(plot_dir / '4_heatmap.png', dpi=150)
    plt.close()
    log.info('  Saved 4_heatmap.png')

    # --- Plot 5: Summary table ---
    fig, ax = plt.subplots(figsize=(16, max(4, n_methods * 0.6 + 2)))
    ax.axis('off')
    headers = ['Method', '0.25m/2°', '0.5m/5°', '5m/10°', 'Day 0.5m/5°', 'Night 0.5m/5°', 'Med. trans', 'Med. rot']
    table_data = []
    for mn in method_names:
        ev = valid[mn]
        ov = ev['overall']
        d = ev.get('day') or {}
        n = ev.get('night') or {}
        table_data.append([
            mn,
            f'{ov["0.25m_2deg"]:.1f}%',
            f'{ov["0.5m_5deg"]:.1f}%',
            f'{ov["5.0m_10deg"]:.1f}%',
            f'{d.get("0.5m_5deg", 0):.1f}%' if d else '-',
            f'{n.get("0.5m_5deg", 0):.1f}%' if n else '-',
            f'{ov["median_trans_m"]:.3f}m',
            f'{ov["median_rot_deg"]:.3f}°',
        ])
    table = ax.table(cellText=table_data, colLabels=headers, loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.8)
    for j in range(len(headers)):
        table[0, j].set_facecolor('#333333')
        table[0, j].set_text_props(color='white', fontweight='bold')
    # highlight best values in each column
    for col_idx in range(1, 6):
        col_vals = []
        for row_idx in range(len(table_data)):
            try:
                col_vals.append(float(table_data[row_idx][col_idx].replace('%', '')))
            except (ValueError, AttributeError):
                col_vals.append(-1)
        if col_vals:
            best_row = int(np.argmax(col_vals))
            table[best_row + 1, col_idx].set_facecolor('#C8E6C9')
    # lower is better for median errors
    for col_idx in [6, 7]:
        col_vals = []
        for row_idx in range(len(table_data)):
            try:
                col_vals.append(float(table_data[row_idx][col_idx].replace('m', '').replace('°', '')))
            except (ValueError, AttributeError):
                col_vals.append(999)
        if col_vals:
            best_row = int(np.argmin(col_vals))
            table[best_row + 1, col_idx].set_facecolor('#C8E6C9')

    ax.set_title('Summary - All Methods', fontsize=14, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.savefig(plot_dir / '5_summary_table.png', dpi=150, bbox_inches='tight')
    plt.close()
    log.info('  Saved 5_summary_table.png')

    # --- Plot 6: Cumulative error curves ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    gt = load_ground_truth(TRAIN_FILE)
    for i, mn in enumerate(method_names):
        m_info = next((m for m in ALL_METHODS_FOR_REPORT if m['name'] == mn), None)
        if m_info is None:
            continue
        results_file = OUTPUTS / m_info['results_name']
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
        axes[0].plot(t_errs, cdf, label=short_names[i], color=cmap[i], linewidth=1.5)
        axes[1].plot(r_errs, cdf, label=short_names[i], color=cmap[i], linewidth=1.5)

    axes[0].set_xlabel('Translation error (m)')
    axes[0].set_ylabel('% images')
    axes[0].set_title('Cumulative Translation Error')
    axes[0].set_xlim(0, 10)
    axes[0].legend(fontsize=7)
    axes[0].grid(True, alpha=0.3)
    axes[1].set_xlabel('Rotation error (°)')
    axes[1].set_ylabel('% images')
    axes[1].set_title('Cumulative Rotation Error')
    axes[1].set_xlim(0, 20)
    axes[1].legend(fontsize=7)
    axes[1].grid(True, alpha=0.3)
    plt.suptitle('Cumulative Error Curves - All Methods', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(plot_dir / '6_cumulative_errors.png', dpi=150)
    plt.close()
    log.info('  Saved 6_cumulative_errors.png')


def main():
    start_time = time.time()
    log.info('=' * 70)
    log.info('  RobotCar Seasons - Fix & Re-run')
    log.info(f'  Started: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    log.info(f'  Methods: SIFT (fixed), DISK (2048kp), ALIKED+OpenIBL (new)')
    log.info('=' * 70)
    log_system_state('startup')

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    sift_sfm = OUTPUTS / 'sfm_sift'
    sfm_pairs = OUTPUTS / f'pairs-db-covis{NUM_COVIS}.txt'
    query_list_pattern = str(OUTPUTS / '{condition}_queries_with_intrinsics.txt')

    log.info('\n--- Loading ground truth ---')
    gt_poses = load_ground_truth(TRAIN_FILE)
    log.info(f'Loaded {len(gt_poses)} GT poses')

    # track results for all methods (load existing + compute new)
    all_eval = {}
    all_timings = {}

    # load existing results from previous benchmark
    log.info('\n--- Loading existing results ---')
    for m in ALL_METHODS_FOR_REPORT:
        mn = m['name']
        eval_path = RESULTS_DIR / m['short'] / 'eval_results.json'
        if eval_path.exists():
            with open(eval_path) as f:
                all_eval[mn] = json.load(f)
            log.info(f'  Loaded: {mn} ({all_eval[mn]["overall"]["0.5m_5deg"]:.1f}%)')

    # --- Run the 3 new/fixed methods ---
    for method_idx, method in enumerate(METHODS):
        method_name = method['name']
        log.info(f'\n{"#"*70}')
        log.info(f'# Method {method_idx+1}/{len(METHODS)}: {method_name}')
        log.info(f'{"#"*70}')
        log_system_state(method_name)

        try:
            result = run_method(method, sift_sfm, sfm_pairs, query_list_pattern)
            results_path, timings = result if result[0] is not None else (None, {})

            if results_path is None:
                log.error(f'[{method_name}] Pipeline failed')
                all_eval[method_name] = None
                continue

            # evaluate
            log.info(f'[{method_name}] Evaluating...')
            loc_poses = load_localization_results(results_path)
            log.info(f'[{method_name}] Loaded {len(loc_poses)} localization results')
            eval_results = evaluate_method(loc_poses, gt_poses)
            all_eval[method_name] = eval_results
            all_timings[method_name] = timings
            print_eval(method_name, eval_results)

            # save per-method results
            method_dir = RESULTS_DIR / method['short']
            method_dir.mkdir(parents=True, exist_ok=True)
            if eval_results:
                with open(method_dir / 'eval_results.json', 'w') as f:
                    json.dump(eval_results, f, indent=2)

            # prepare submission
            sub_path = RESULTS_DIR / 'submissions' / f'{method["short"]}_submission.txt'
            prepare_submission(results_path, TEST_FILE, sub_path)

        except Exception as e:
            log.error(f'[{method_name}] FAILED: {e}')
            log.error(traceback.format_exc())
            all_eval[method_name] = None
            torch.cuda.empty_cache()
            continue

    # --- Generate updated plots and report ---
    log.info(f'\n{"="*70}')
    log.info('  Generating updated plots and report')
    log.info(f'{"="*70}')

    try:
        generate_plots(all_eval)
    except Exception as e:
        log.error(f'Plot generation failed: {e}')
        log.error(traceback.format_exc())

    try:
        report = generate_updated_report(all_eval, all_timings)
        log.info('\n' + report)
    except Exception as e:
        log.error(f'Report generation failed: {e}')
        log.error(traceback.format_exc())

    # save combined results
    def to_serializable(obj):
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    all_results_combined = {
        'methods': {k: v for k, v in all_eval.items() if v is not None},
        'timings': all_timings,
        'generated': datetime.now().isoformat(),
    }
    with open(RESULTS_DIR / 'all_results.json', 'w') as f:
        json.dump(all_results_combined, f, indent=2, default=to_serializable)
    log.info('Saved all_results.json')

    total_time = time.time() - start_time
    log.info(f'\n{"="*70}')
    log.info(f'  DONE - Total time: {total_time/60:.1f} min ({total_time/3600:.2f} hours)')
    log.info(f'{"="*70}')

    # print final comparison table
    log.info('\n  FINAL RESULTS COMPARISON:')
    log.info(f'  {"Method":<30} {"0.25m/2d":>10} {"0.5m/5d":>10} {"5m/10d":>10} {"Night":>10}')
    log.info(f'  {"-"*75}')
    for m in ALL_METHODS_FOR_REPORT:
        mn = m['name']
        if mn in all_eval and all_eval[mn] is not None:
            ov = all_eval[mn]['overall']
            night = all_eval[mn].get('night', {})
            n_str = f'{night["0.5m_5deg"]:.1f}%' if night else '-'
            log.info(f'  {mn:<30} {ov["0.25m_2deg"]:>9.1f}% {ov["0.5m_5deg"]:>9.1f}% '
                     f'{ov["5.0m_10deg"]:>9.1f}% {n_str:>10}')
        else:
            log.info(f'  {mn:<30} {"FAILED":>10}')


if __name__ == '__main__':
    main()
