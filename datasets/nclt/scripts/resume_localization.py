#!/usr/bin/env python3
"""Resume hloc from localization step (SfM + matching already done)"""

import sys
sys.path.insert(0, '/workspace/third_party/hloc')

from pathlib import Path
from hloc import localize_sfm
import pycolmap
import time
import numpy as np
from scipy.spatial.transform import Rotation

RESULTS = Path('/workspace/datasets/nclt/results/week0_hloc/full')
model = pycolmap.Reconstruction(str(RESULTS / 'sfm'))
query_list_file = RESULTS / 'queries_with_intrinsics.txt'
loc_pairs = RESULTS / 'pairs-loc.txt'
feature_path = RESULTS / 'features.h5'
match_path = RESULTS / 'matches-loc.h5'
loc_results = RESULTS / 'localization_results.txt'

# delete old results if any
if loc_results.exists():
    loc_results.unlink()

print(f'SfM model: {model.summary()}')
print(f'Running PnP localization...')
t0 = time.time()
localize_sfm.main(
    model, query_list_file,
    loc_pairs, feature_path, match_path,
    loc_results,
    ransac_thresh=12,
    covisibility_clustering=False,
)
print(f'Done in {time.time() - t0:.1f}s')

n_loc = sum(1 for line in open(loc_results) if line.strip() and not line.startswith('#'))
n_query = sum(1 for line in open(query_list_file) if line.strip())
print(f'\nLocalized {n_loc} / {n_query} queries')
print(f'Results saved to {loc_results}')
