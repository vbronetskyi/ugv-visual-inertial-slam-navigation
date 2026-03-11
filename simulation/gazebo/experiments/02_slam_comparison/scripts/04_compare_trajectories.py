#!/usr/bin/env python3
"""
Compare ORB-SLAM3 (and optionally RTAB-Map) trajectory with Ground Truth
Produces trajectory_comparison.png and prints ATE/RPE metrics

Usage: python3 04_compare_trajectories.py
"""
import numpy as np
import math
import json
from PIL import Image as PILImage, ImageDraw
import xml.etree.ElementTree as ET

WORLD_X = 220.0; WORLD_Y = 150.0
SZ_X = 1000; SZ_Y = int(SZ_X * WORLD_Y / WORLD_X)
GT_FILE = '/workspace/simulation/bags/route_1_clean/gt_trajectory.csv'
SLAM_FILE = '/workspace/simulation/orb_slam3_data/CameraTrajectory.txt'
TEX_FILE = '/workspace/simulation/src/ugv_gazebo/worlds/terrain_texture.png'
SDF_FILE = '/workspace/simulation/src/ugv_gazebo/worlds/outdoor_terrain.sdf'
OUT_FILE = '/workspace/simulation/experiments/02_slam_comparison/trajectory_comparison.png'

def w2px(wx, wy):
    return int((wx + WORLD_X/2) / WORLD_X * SZ_X), int((WORLD_Y/2 - wy) / WORLD_Y * SZ_Y)

def umeyama_align(slam_xy, gt_xy):
    """Umeyama alignment: scale + rotation + translation to match GT"""
    sm = slam_xy.mean(0); gm = gt_xy.mean(0)
    sc = slam_xy - sm; gc = gt_xy - gm
    H = sc.T @ gc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0: Vt[-1,:] *= -1; R = Vt.T @ U.T
    scale = np.trace(R @ H) / np.trace(sc.T @ sc)
    t = gm - scale * R @ sm
    aligned = (scale * (R @ slam_xy.T).T) + t
    return aligned, scale

# load GT
gt = []
gt_times = []
for l in open(GT_FILE):
    p = l.strip().split(',')
    try:
        gt_times.append(float(p[0]))
        gt.append((float(p[1]), float(p[2])))
    except Exception:
        pass
gt = np.array(gt)
gt_times = np.array(gt_times)
gt_times -= gt_times[0]  # normalize to 0

print(f"GT: {len(gt)} points, ({gt[0,0]:.0f},{gt[0,1]:.0f}) -> ({gt[-1,0]:.0f},{gt[-1,1]:.0f})")
gt_dist = sum(math.hypot(gt[i,0]-gt[i-1,0], gt[i,1]-gt[i-1,1]) for i in range(1, len(gt)))
print(f"GT distance: {gt_dist:.0f}m")

# load ORB-SLAM3
slam = []
slam_times = []
try:
    for l in open(SLAM_FILE):
        p = l.strip().split()
        if len(p) == 8:
            slam_times.append(float(p[0]))
            slam.append((float(p[1]), float(p[2])))
    slam = np.array(slam)
    slam_times = np.array(slam_times)
    slam_times -= slam_times[0]
    print(f"ORB-SLAM3: {len(slam)} poses")
except:
    slam = np.array([])
    print("ORB-SLAM3: no trajectory")

# match timestamps + Umeyama align
aligned = np.array([])
ate = 0
if len(slam) > 10:
    matched = []
    for i, st in enumerate(slam_times):
        idx = np.argmin(np.abs(gt_times - st))
        if abs(gt_times[idx] - st) < 2.0:
            matched.append((slam[i,0], slam[i,1], gt[idx,0], gt[idx,1]))
    matched = np.array(matched)

    if len(matched) > 10:
        slam_xy = matched[:,:2]; gt_xy = matched[:,2:4]
        aligned, scale = umeyama_align(slam_xy, gt_xy)
        errors = np.sqrt(((aligned - gt_xy)**2).sum(axis=1))
        ate_rmse = np.sqrt((errors**2).mean())
        ate_mean = errors.mean()

        print(f"\n=== ATE (Absolute Trajectory Error) ===")
        print(f"  Matched: {len(matched)} poses")
        print(f"  RMSE: {ate_rmse:.2f}m")
        print(f"  Mean: {ate_mean:.2f}m")
        print(f"  Median: {np.median(errors):.2f}m")
        print(f"  Max: {errors.max():.2f}m")
        print(f"  Scale: {scale:.3f}")
        ate = ate_mean

        # compute RPE
        step = 10
        rpe = []
        for i in range(step, len(aligned)):
            sd = np.linalg.norm(aligned[i]-aligned[i-step])
            gd = np.linalg.norm(gt_xy[i]-gt_xy[i-step])
            if gd > 0.1: rpe.append(abs(sd-gd))
        if rpe:
            rpe = np.array(rpe)
            print(f"\n=== RPE (step={step}) ===")
            print(f"  RMSE: {np.sqrt((rpe**2).mean()):.2f}m")
            print(f"  Mean: {rpe.mean():.2f}m")

        # dump metrics to json
        results = {
            'ate_rmse': float(ate_rmse), 'ate_mean': float(ate_mean),
            'ate_max': float(errors.max()), 'scale': float(scale),
            'matched_poses': len(matched), 'total_slam_poses': len(slam),
            'gt_distance': float(gt_dist)
        }
        json.dump(results, open('/workspace/simulation/experiments/02_slam_comparison/results.json','w'), indent=2)

# === plot ===
img = PILImage.open(TEX_FILE).resize((SZ_X, SZ_Y))
draw = ImageDraw.Draw(img)

# draw SDF models
MODEL_COLORS = {
    'Oak': (34,139,34), 'Pine': (0,100,0), 'Rock': (160,140,120),
    'house': (180,160,130), 'collapsed': (140,90,80), 'ruin': (140,90,80),
    'Barrel': (255,165,0), 'Cone': (255,69,0), 'fallen': (90,60,30),
}
tree_sdf = ET.parse(SDF_FILE)
for inc in tree_sdf.getroot().iter('include'):
    n=inc.find('name'); p=inc.find('pose')
    if n is None or p is None: continue
    parts=[float(x) for x in p.text.split()]
    px,py = w2px(parts[0],parts[1])
    if not (0<=px<SZ_X and 0<=py<SZ_Y): continue
    nl=n.text.lower()
    color=(128,128,128)
    for k,c in MODEL_COLORS.items():
        if k.lower() in nl: color=c; break
    if 'house' in nl or 'collapsed' in nl:
        draw.rectangle([px-8,py-8,px+8,py+8], fill=color, outline=(60,40,30))
    elif 'fallen' in nl:
        yaw=parts[5] if len(parts)>5 else 0
        dx=int(6*math.cos(yaw)); dy=int(-6*math.sin(yaw))
        draw.line([px-dx,py-dy,px+dx,py+dy], fill=color, width=2)
    elif 'tree' in nl or 'pine' in nl or 'oak' in nl:
        draw.ellipse([px-5,py-5,px+5,py+5], fill=color, outline=(255,255,255), width=1)
    elif 'rock' in nl:
        draw.polygon([(px,py-4),(px+4,py),(px,py+4),(px-4,py)], fill=color)

# gT in greenfor i in range(1, len(gt), 3):
    draw.line([w2px(gt[i-1,0],gt[i-1,1]), w2px(gt[i,0],gt[i,1])], fill=(0,230,80), width=3)

# ORB-SLAM3 aligned in redif len(aligned) > 1:
    for i in range(1, len(aligned)):
        draw.line([w2px(aligned[i-1,0],aligned[i-1,1]), w2px(aligned[i,0],aligned[i,1])], fill=(255,40,40), width=3)

# start/end dots
sx,sy = w2px(gt[0,0],gt[0,1])
draw.ellipse([sx-6,sy-6,sx+6,sy+6], fill=(0,255,0), outline=(255,255,255), width=2)
ex,ey = w2px(gt[-1,0],gt[-1,1])
draw.ellipse([ex-6,ey-6,ex+6,ey+6], fill=(255,0,0), outline=(255,255,255), width=2)

# legend
draw.rectangle([5,5,280,70], fill=(0,0,0,200))
draw.line([15,22,45,22], fill=(0,230,80), width=3)
draw.text((50,15), "Ground Truth", fill=(255,255,255))
draw.line([15,45,45,45], fill=(255,40,40), width=3)
draw.text((50,38), f"ORB-SLAM3 (ATE={ate:.1f}m)", fill=(255,255,255))

img.save(OUT_FILE)
print(f"\nSaved: {OUT_FILE}")
