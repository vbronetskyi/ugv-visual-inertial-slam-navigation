#!/usr/bin/env python3
"""Resume hloc experiment: run evaluation + plots on already-computed results"""

import sys
sys.path.insert(0, '/workspace/third_party/hloc')
sys.path.insert(0, '/workspace/datasets/nclt/scripts')

from run_week0_hloc import (
    evaluate_localization, evaluate_sfm_map,
    plot_sfm_vs_gt, plot_localization_vs_gt,
    plot_localization_success, plot_failure_map, plot_comparison_all_methods,
    prepare_images, prepare_gt_poses,
    IMAGES_ROOT, RESULTS, SPRING, SUMMER,
)

from pathlib import Path
import pycolmap
import numpy as np
from scipy.spatial.transform import Rotation
import json
import time

mode = 'full'
outputs = RESULTS / mode

# re-generate image lists and GT poses
spring_every, summer_every = 3, 5
db_list, query_list = prepare_images(outputs, spring_every, summer_every, None, None)
gt_poses = prepare_gt_poses(db_list, query_list, outputs)

print(f'DB images: {len(db_list)}, Query images: {len(query_list)}')

model = pycolmap.Reconstruction(str(outputs / 'sfm'))
print(f'SfM: {model.num_reg_images()} registered, {model.num_points3D()} 3D points')

print('\n=== SfM Evaluation ===')
sfm_results = evaluate_sfm_map(model, gt_poses, db_list, outputs)
print(json.dumps(sfm_results, indent=2))

if (outputs / 'sfm_trajectory.npz').exists():
    plot_sfm_vs_gt(outputs)
    print('SfM plot saved.')

# parse localization results (hloc PnP output, one pose per query line)
loc_results_file = outputs / 'localization_results.txt'
loc_poses = {}
with open(loc_results_file) as f:
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
        loc_poses[name] = T

print(f'\nParsed {len(loc_poses)} localization poses')

# evaluate localization
print('\n=== Localization Evaluation ===')
eval_results = evaluate_localization(loc_poses, gt_poses, query_list, outputs)
print(json.dumps(eval_results, indent=2))

# plotsprint('\n=== Generating Plots ===')
plot_localization_vs_gt(loc_poses, gt_poses, query_list, outputs)
print('  localization_vs_gt.png')
plot_localization_success(eval_results, outputs)
print('  localization_success.png')
plot_failure_map(loc_poses, gt_poses, query_list, outputs)
print('  failure_map.png')
plot_comparison_all_methods(eval_results, outputs)
print('  comparison_all_methods.png')

# save summary
summary = {
    'mode': mode,
    'status': 'ok',
    'spring_session': SPRING,
    'summer_session': SUMMER,
    'n_db_images': len(db_list),
    'n_query_images': len(query_list),
    'pipeline': {
        'features': 'SuperPoint (n=4096, nms=3, resize=1024)',
        'retrieval': 'NetVLAD',
        'matcher': 'LightGlue',
        'sfm': 'COLMAP (single camera)',
        'localization': 'PnP+RANSAC (thresh=12)',
    },
    'sfm': sfm_results,
    'localization': eval_results,
}

with open(outputs / 'summary.json', 'w') as f:
    json.dump(summary, f, indent=2)
print(f'\nSummary saved to {outputs / "summary.json"}')
