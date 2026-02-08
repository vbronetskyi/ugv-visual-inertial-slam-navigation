#!/usr/bin/env python3
"""Re-run only SfM + localization + evaluation using existing features/matches.

Reuses: features.h5, matches-sfm.h5, pairs-sfm.txt, global-feats, matches-loc.h5, pairs-loc.txt
Rebuilds: SfM reconstruction (with OPENCV_FISHEYE), localization, evaluation, plots.
"""

import sys
sys.path.insert(0, '/workspace/third_party/hloc')
sys.path.insert(0, '/workspace/datasets/nclt/scripts')

import shutil
import time
import json
import numpy as np
from pathlib import Path
import pycolmap

from hloc import reconstruction, localize_sfm
from run_week0_hloc import (
    prepare_images, prepare_gt_poses, create_query_list_file,
    evaluate_localization, evaluate_sfm_map,
    plot_sfm_vs_gt, plot_localization_vs_gt,
    plot_localization_success, plot_failure_map, plot_comparison_all_methods,
    IMAGES_ROOT, RESULTS, SPRING, SUMMER,
)
from scipy.spatial.transform import Rotation

mode = 'full'
outputs = RESULTS / mode

# prepare data
spring_every, summer_every = 3, 5
db_list, query_list = prepare_images(outputs, spring_every, summer_every, None, None)
gt_poses = prepare_gt_poses(db_list, query_list, outputs)
print(f'DB: {len(db_list)}, Query: {len(query_list)} images')

# re-run SfM with OPENCV_FISHEYE
sfm_dir = outputs / 'sfm'
if sfm_dir.exists():
    shutil.rmtree(sfm_dir)
sfm_dir.mkdir(parents=True)

sfm_pairs = outputs / 'pairs-sfm.txt'
feature_path = outputs / 'features.h5'
match_path = outputs / 'matches-sfm.h5'

print(f'\n{"="*60}')
print('Re-running COLMAP SfM with OPENCV_FISHEYE camera model...')
print(f'{"="*60}')
t0 = time.time()

model = reconstruction.main(
    sfm_dir, IMAGES_ROOT, sfm_pairs, feature_path, match_path,
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
sfm_time = time.time() - t0
print(f'SfM done in {sfm_time:.1f}s')

if model is None:
    print('SfM FAILED!')
    sys.exit(1)

n_reg = model.num_reg_images()
print(f'Registered: {n_reg}/{len(db_list)} ({100*n_reg/len(db_list):.1f}%)')
print(f'3D points: {model.num_points3D()}')
print(f'Mean reproj error: {model.compute_mean_reprojection_error():.3f} px')

# check camera params
for cam_id, cam in model.cameras.items():
    print(f'Camera: {cam}')

# check position spread
positions = []
for img_id in list(model.images.keys())[:100]:
    img = model.images[img_id]
    cfw = img.cam_from_world
    if callable(cfw): cfw = cfw()
    R = cfw.rotation.matrix()
    t = np.array(cfw.translation)
    T = np.eye(4)
    T[:3,:3] = R; T[:3,3] = t
    T_inv = np.linalg.inv(T)
    positions.append(T_inv[:3,3])
positions = np.array(positions)
print(f'SfM position spread (100 imgs): {np.linalg.norm(positions.max(0) - positions.min(0)):.1f} m')

# evaluate SfM
print('\nEvaluating SfM map...')
sfm_results = evaluate_sfm_map(model, gt_poses, db_list, outputs)
if (outputs / 'sfm_trajectory.npz').exists():
    plot_sfm_vs_gt(outputs)

# re-run localization
print(f'\n{"="*60}')
print('Running localization...')
print(f'{"="*60}')

# create query list with OPENCV_FISHEYE intrinsics written per line
query_list_file = outputs / 'queries_with_intrinsics.txt'
create_query_list_file(query_list, query_list_file)

loc_pairs = outputs / 'pairs-loc.txt'
loc_matches = outputs / 'matches-loc.h5'
loc_results = outputs / 'localization_results.txt'
if loc_results.exists():
    loc_results.unlink()

t0 = time.time()
localize_sfm.main(
    model, query_list_file,
    loc_pairs, feature_path, loc_matches,
    loc_results,
    ransac_thresh=12,
    covisibility_clustering=False,
)
loc_time = time.time() - t0
print(f'Localization done in {loc_time:.1f}s')

# parse hloc results: one line per localised query
loc_poses = {}
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
        T[:3,:3] = R; T[:3,3] = [tx, ty, tz]
        loc_poses[name] = T

print(f'Localized: {len(loc_poses)}/{len(query_list)}')

# evaluate localization
print(f'\n{"="*60}')
print('Evaluation')
print(f'{"="*60}')
eval_results = evaluate_localization(loc_poses, gt_poses, query_list, outputs)

print('\nGenerating plots...')
plot_localization_vs_gt(loc_poses, gt_poses, query_list, outputs)
plot_localization_success(eval_results, outputs)
plot_failure_map(loc_poses, gt_poses, query_list, outputs)
plot_comparison_all_methods(eval_results, outputs)

# save summary
total_time = sfm_time + loc_time
summary = {
    'mode': mode,
    'status': 'ok',
    'camera_model': 'OPENCV_FISHEYE',
    'total_time_s': total_time,
    'spring_session': SPRING,
    'summer_session': SUMMER,
    'n_db_images': len(db_list),
    'n_query_images': len(query_list),
    'pipeline': {
        'features': 'SuperPoint (n=4096, nms=3, resize=1024)',
        'retrieval': 'NetVLAD',
        'matcher': 'LightGlue',
        'sfm': 'COLMAP (OPENCV_FISHEYE, single camera)',
        'localization': 'PnP+RANSAC (thresh=12)',
    },
    'sfm': sfm_results,
    'localization': eval_results,
}
with open(outputs / 'summary.json', 'w') as f:
    json.dump(summary, f, indent=2)

print(f'\n{"="*60}')
print('DONE')
print(f'{"="*60}')
print(json.dumps(summary, indent=2))
