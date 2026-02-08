#!/usr/bin/env python3
"""
Self-calibrate NCLT Ladybug3 camera using COLMAP Structure-from-Motion.

Runs COLMAP feature extraction + matching + reconstruction on a subset of images
to estimate camera intrinsics (fx, fy, cx, cy) and fisheye distortion (k1-k4).

Usage:
    python calibrate_camera_colmap.py --session 2012-04-29 [--n-images 300] [--cam-id 0]
"""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

NCLT_DATA = Path('/workspace/nclt_data')
PROJECT_ROOT = Path(__file__).resolve().parent.parent


def prepare_colmap_images(session, cam_id=0, n_images=300, subsample=3):
    """Copy subset of images for COLMAP calibration, returns path or None"""
    img_dir = NCLT_DATA / 'images' / session / 'lb3' / f'Cam{cam_id}'
    if not img_dir.exists():
        print(f"ERROR: Image directory not found: {img_dir}")
        return None

    tiff_files = sorted(img_dir.glob('*.tiff'))
    # take every Nth image, up to n_images
    selected = tiff_files[::subsample][:n_images]

    colmap_dir = Path(f'/tmp/colmap_nclt_{session}')
    colmap_img_dir = colmap_dir / 'images'
    colmap_img_dir.mkdir(parents=True, exist_ok=True)

    print(f"  Preparing {len(selected)} images for COLMAP...")
    for f in tqdm(selected, desc="Copy+resize images"):
        img = cv2.imread(str(f), cv2.IMREAD_COLOR)
        if img is None:
            continue
        # resize to half resolution
        img = cv2.resize(img, (808, 616), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(colmap_img_dir / f'{f.stem}.png'), img)

    print(f"  Prepared {len(list(colmap_img_dir.glob('*.png')))} images in {colmap_img_dir}")
    return colmap_dir


def run_colmap(colmap_dir):
    """Run COLMAP SfM pipeline for camera calibration, returns recon path or None"""
    db_path = colmap_dir / 'database.db'
    sparse_dir = colmap_dir / 'sparse'
    sparse_dir.mkdir(exist_ok=True)

    # remove old database if exists
    if db_path.exists():
        db_path.unlink()

    # step 1: Feature extraction with OPENCV_FISHEYE camera model
    # OPENCV_FISHEYE has 8 params: fx, fy, cx, cy, k1, k2, k3, k4
    print("\n--- COLMAP Feature Extraction ---")
    cmd_extract = [
        'colmap', 'feature_extractor',
        '--database_path', str(db_path),
        '--image_path', str(colmap_dir / 'images'),
        '--ImageReader.camera_model', 'OPENCV_FISHEYE',
        '--ImageReader.single_camera', '1',
        # initial guess for focal length (equidistant ~579 at half res)
        '--ImageReader.camera_params', '579,579,404,308,0.01,-0.005,0.001,0.0',
        '--SiftExtraction.max_num_features', '3000',
    ]
    result = subprocess.run(cmd_extract, capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        print(f"  Feature extraction failed: {result.stderr[-500:]}")
        return None
    print("  Feature extraction OK")

    # step 2: Sequential matching (faster than exhaustive for ordered sequences)
    print("\n--- COLMAP Sequential Matching ---")
    cmd_match = [
        'colmap', 'sequential_matcher',
        '--database_path', str(db_path),
        '--SequentialMatching.overlap', '10',
        '--SequentialMatching.loop_detection', '0',
    ]
    result = subprocess.run(cmd_match, capture_output=True, text=True, timeout=1200)
    if result.returncode != 0:
        print(f"  Matching failed: {result.stderr[-500:]}")
        return None
    print("  Sequential matching OK")

    # step 3: Sparse reconstruction (mapper)
    print("\n--- COLMAP Mapper ---")
    cmd_mapper = [
        'colmap', 'mapper',
        '--database_path', str(db_path),
        '--image_path', str(colmap_dir / 'images'),
        '--output_path', str(sparse_dir),
        '--Mapper.ba_refine_focal_length', '1',
        '--Mapper.ba_refine_principal_point', '1',
        '--Mapper.ba_refine_extra_params', '1',
    ]
    result = subprocess.run(cmd_mapper, capture_output=True, text=True, timeout=3600)
    if result.returncode != 0:
        print(f"  Mapper failed: {result.stderr[-500:]}")
        return None

    # find the reconstruction with most images
    recon_dirs = sorted(sparse_dir.iterdir())
    if not recon_dirs:
        print("  ERROR: No reconstruction produced")
        return None

    best_dir = recon_dirs[0]
    print(f"  Mapper OK - {len(recon_dirs)} reconstruction(s)")
    return best_dir


def extract_calibration(recon_dir, output_path):
    """extract camera calibration from COLMAP reconstruction and save to JSON"""
    # export cameras as text
    cameras_txt = recon_dir / 'cameras.txt'

    # If binary format, convert to text
    if not cameras_txt.exists():
        cmd = [
            'colmap', 'model_converter',
            '--input_path', str(recon_dir),
            '--output_path', str(recon_dir),
            '--output_type', 'TXT',
        ]
        subprocess.run(cmd, capture_output=True, text=True)

    if not cameras_txt.exists():
        print("  ERROR: cameras.txt not found")
        return None

    # parse cameras.txt
    # format: CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
    # OPENCV_FISHEYE: fx, fy, cx, cy, k1, k2, k3, k4
    with open(cameras_txt) as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split()
            model = parts[1]
            width = int(parts[2])
            height = int(parts[3])
            params = [float(p) for p in parts[4:]]

            if model == 'OPENCV_FISHEYE':
                calib = {
                    'model': 'OPENCV_FISHEYE',
                    'width': width,
                    'height': height,
                    'fx': params[0],
                    'fy': params[1],
                    'cx': params[2],
                    'cy': params[3],
                    'k1': params[4],
                    'k2': params[5],
                    'k3': params[6],
                    'k4': params[7],
                }
            elif model in ('SIMPLE_RADIAL_FISHEYE', 'RADIAL_FISHEYE'):
                # simpler fisheye models - convert to our format
                calib = {
                    'model': model,
                    'width': width,
                    'height': height,
                    'fx': params[0],
                    'fy': params[0],  # Same focal length for simple models
                    'cx': params[1] if len(params) > 1 else width / 2,
                    'cy': params[2] if len(params) > 2 else height / 2,
                    'k1': params[3] if len(params) > 3 else 0,
                    'k2': params[4] if len(params) > 4 else 0,
                    'k3': 0,
                    'k4': 0,
                }
            else:
                # handle pinhole/radial models too
                calib = {
                    'model': model,
                    'width': width,
                    'height': height,
                    'fx': params[0],
                    'fy': params[1] if len(params) > 1 else params[0],
                    'cx': params[2] if len(params) > 2 else width / 2,
                    'cy': params[3] if len(params) > 3 else height / 2,
                    'k1': params[4] if len(params) > 4 else 0,
                    'k2': params[5] if len(params) > 5 else 0,
                    'k3': params[6] if len(params) > 6 else 0,
                    'k4': params[7] if len(params) > 7 else 0,
                }
            break

    if not calib:
        print("  ERROR: Could not parse camera parameters")
        return None

    # count registered images
    images_txt = recon_dir / 'images.txt'
    n_registered = 0
    if images_txt.exists():
        with open(images_txt) as f:
            for line in f:
                if not line.startswith('#') and line.strip():
                    n_registered += 1
        n_registered //= 2  # Each image has 2 lines in images.txt

    calib['n_registered_images'] = n_registered

    print(f"\n=== Calibration Results ===")
    print(f"  Model: {calib['model']}")
    print(f"  Resolution: {calib['width']}x{calib['height']}")
    print(f"  fx={calib['fx']:.2f}, fy={calib['fy']:.2f}")
    print(f"  cx={calib['cx']:.2f}, cy={calib['cy']:.2f}")
    print(f"  k1={calib['k1']:.6f}, k2={calib['k2']:.6f}, k3={calib['k3']:.6f}, k4={calib['k4']:.6f}")
    print(f"  Registered images: {n_registered}")

    # save to JSON
    with open(output_path, 'w') as f:
        json.dump(calib, f, indent=2)
    print(f"  Saved to {output_path}")

    return calib


def main():
    parser = argparse.ArgumentParser(description="Self-calibrate NCLT camera via COLMAP")
    parser.add_argument('--session', default='2012-04-29', help='NCLT session')
    parser.add_argument('--cam-id', type=int, default=0, help='Camera ID (0-5)')
    parser.add_argument('--n-images', type=int, default=300, help='Number of images')
    parser.add_argument('--subsample', type=int, default=3, help='Take every Nth image')
    args = parser.parse_args()

    output_path = PROJECT_ROOT / 'configs' / 'nclt_cam0_calibrated.json'

    print(f"=== COLMAP Camera Calibration ===")
    print(f"Session: {args.session}, Camera: Cam{args.cam_id}")
    print(f"Using {args.n_images} images (every {args.subsample}th)")

    # step 1: Prepare images
    print("\n--- Preparing images ---")
    colmap_dir = prepare_colmap_images(args.session, args.cam_id, args.n_images, args.subsample)
    if colmap_dir is None:
        return 1

    # step 2: Run COLMAP
    print("\n--- Running COLMAP ---")
    recon_dir = run_colmap(colmap_dir)
    if recon_dir is None:
        print("\nCOLMAP failed. Using approximate parameters.")
        # save approximate calibration
        approx = {
            'model': 'OPENCV_FISHEYE',
            'width': 808, 'height': 616,
            'fx': 579.0, 'fy': 579.0,
            'cx': 404.0, 'cy': 308.0,
            'k1': 0.01, 'k2': -0.005, 'k3': 0.001, 'k4': 0.0,
            'source': 'approximate',
            'n_registered_images': 0,
        }
        with open(output_path, 'w') as f:
            json.dump(approx, f, indent=2)
        print(f"  Saved approximate calibration to {output_path}")
        return 1

    # step 3: Extract calibration
    print("\n--- Extracting calibration ---")
    calib = extract_calibration(recon_dir, output_path)
    if calib is None:
        return 1

    print("\nDone!")
    return 0


if __name__ == '__main__':
    sys.exit(main())
