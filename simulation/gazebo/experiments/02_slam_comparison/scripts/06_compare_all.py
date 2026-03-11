#!/usr/bin/env python3
"""
Compare RTAB-Map and ORB-SLAM3 trajectories with Ground Truth
Produces comparison plot and prints metrics for both

Usage: python3 06_compare_all.py
"""
import numpy as np, math, json
from PIL import Image as PILImage, ImageDraw
import xml.etree.ElementTree as ET

WORLD_X = 220.0; WORLD_Y = 150.0
SZ_X = 1200; SZ_Y = int(SZ_X * WORLD_Y / WORLD_X)
BASE = '/workspace/simulation'
GT_FILE = f'{BASE}/experiments/02_slam_comparison/gt_trajectory.csv'
RTAB_FILE = f'{BASE}/experiments/02_slam_comparison/rtabmap_poses.txt'
ORB_FILE = f'{BASE}/orb_slam3_data/CameraTrajectory.txt'
TEX_FILE = f'{BASE}/src/ugv_gazebo/worlds/terrain_texture.png'
SDF_FILE = f'{BASE}/src/ugv_gazebo/worlds/outdoor_terrain.sdf'
OUT_FILE = f'{BASE}/experiments/02_slam_comparison/trajectory_comparison.png'

def w2px(wx, wy):
    return int((wx + WORLD_X/2) / WORLD_X * SZ_X), int((WORLD_Y/2 - wy) / WORLD_Y * SZ_Y)

def umeyama_align(src, dst):
    sm = src.mean(0); dm = dst.mean(0)
    sc = src - sm; dc = dst - dm
    H = sc.T @ dc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0: Vt[-1,:] *= -1; R = Vt.T @ U.T
    scale = np.trace(R @ H) / np.trace(sc.T @ sc)
    t = dm - scale * R @ sm
    return (scale * (R @ src.T).T) + t, scale

def compute_ate(aligned, gt_matched):
    errors = np.sqrt(((aligned - gt_matched)**2).sum(axis=1))
    return {
        'rmse': float(np.sqrt((errors**2).mean())),
        'mean': float(errors.mean()),
        'median': float(np.median(errors)),
        'max': float(errors.max()),
        'count': len(errors)
    }

# load GT
gt = []; gt_t = []
for l in open(GT_FILE):
    p = l.strip().split(',')
    try: gt_t.append(float(p[0])); gt.append((float(p[1]), float(p[2])))
    except Exception:
        pass
gt = np.array(gt); gt_t = np.array(gt_t); gt_t -= gt_t[0]
gt_dist = sum(math.hypot(gt[i,0]-gt[i-1,0], gt[i,1]-gt[i-1,1]) for i in range(1,len(gt)))
print(f"GT: {len(gt)} points, {gt_dist:.0f}m, ({gt[0,0]:.0f},{gt[0,1]:.0f})->({gt[-1,0]:.0f},{gt[-1,1]:.0f})")

results = {}

# RTAB-Maprtab = []; rtab_t = []
try:
    for l in open(RTAB_FILE):
        if l.startswith('#'): continue
        p = l.strip().split()
        if len(p) >= 4:
            rtab_t.append(float(p[0])); rtab.append((float(p[1]), float(p[2])))
    rtab = np.array(rtab); rtab_t = np.array(rtab_t); rtab_t -= rtab_t[0]
    print(f"RTAB-Map: {len(rtab)} poses")

    # match timestamps for alignment
    matched_r = []
    for i, st in enumerate(rtab_t):
        idx = np.argmin(np.abs(gt_t - st))
        if abs(gt_t[idx] - st) < 3.0:
            matched_r.append((rtab[i,0], rtab[i,1], gt[idx,0], gt[idx,1]))
    matched_r = np.array(matched_r)

    if len(matched_r) > 10:
        rtab_aligned, rtab_scale = umeyama_align(matched_r[:,:2], matched_r[:,2:4])
        rtab_ate = compute_ate(rtab_aligned, matched_r[:,2:4])
        print(f"  ATE RMSE: {rtab_ate['rmse']:.2f}m, Mean: {rtab_ate['mean']:.2f}m, Scale: {rtab_scale:.3f}")
        print(f"  Matched: {rtab_ate['count']}/{len(rtab)} poses")
        results['rtabmap'] = {**rtab_ate, 'scale': float(rtab_scale), 'total_poses': len(rtab)}
    else:
        rtab_aligned = np.array([])
        print("  Not enough matches for alignment")
except Exception as e:
    rtab_aligned = np.array([])
    print(f"RTAB-Map: {e}")

# ORB-SLAM3orb = []; orb_t = []
try:
    for l in open(ORB_FILE):
        p = l.strip().split()
        if len(p) == 8:
            orb_t.append(float(p[0])); orb.append((float(p[1]), float(p[2])))
    orb = np.array(orb); orb_t = np.array(orb_t); orb_t -= orb_t[0]
    print(f"ORB-SLAM3: {len(orb)} poses")

    matched_o = []
    for i, st in enumerate(orb_t):
        idx = np.argmin(np.abs(gt_t - st))
        if abs(gt_t[idx] - st) < 3.0:
            matched_o.append((orb[i,0], orb[i,1], gt[idx,0], gt[idx,1]))
    matched_o = np.array(matched_o)

    if len(matched_o) > 10:
        orb_aligned, orb_scale = umeyama_align(matched_o[:,:2], matched_o[:,2:4])
        orb_ate = compute_ate(orb_aligned, matched_o[:,2:4])
        print(f"  ATE RMSE: {orb_ate['rmse']:.2f}m, Mean: {orb_ate['mean']:.2f}m, Scale: {orb_scale:.3f}")
        print(f"  Matched: {orb_ate['count']}/{len(orb)} poses")
        results['orbslam3'] = {**orb_ate, 'scale': float(orb_scale), 'total_poses': len(orb)}
    else:
        orb_aligned = np.array([])
        print("  Not enough matches for alignment")
except Exception as e:
    orb_aligned = np.array([])
    print(f"ORB-SLAM3: {e}")

# render comparison plot
img = PILImage.open(TEX_FILE).resize((SZ_X, SZ_Y))
draw = ImageDraw.Draw(img)

# draw model markers
MODEL_COLORS = {
    'Oak':(34,139,34),'Pine':(0,100,0),'Rock':(160,140,120),
    'house':(180,160,130),'collapsed':(140,90,80),'ruin':(140,90,80),
    'Barrel':(255,165,0),'Cone':(255,69,0),'fallen':(90,60,30),
}
tree_sdf = ET.parse(SDF_FILE)
for inc in tree_sdf.getroot().iter('include'):
    n=inc.find('name'); p=inc.find('pose')
    if n is None or p is None: continue
    parts=[float(x) for x in p.text.split()]
    px,py=w2px(parts[0],parts[1])
    if not (0<=px<SZ_X and 0<=py<SZ_Y): continue
    nl=n.text.lower(); color=(128,128,128)
    for k,c in MODEL_COLORS.items():
        if k.lower() in nl: color=c; break
    if 'house' in nl or 'collapsed' in nl:
        draw.rectangle([px-8,py-8,px+8,py+8],fill=color,outline=(60,40,30))
    elif 'tree' in nl or 'pine' in nl or 'oak' in nl:
        r=5; draw.ellipse([px-r,py-r,px+r,py+r],fill=color,outline=(255,255,255),width=1)
    elif 'rock' in nl:
        draw.polygon([(px,py-4),(px+4,py),(px,py+4),(px-4,py)],fill=color)

# gT in greenfor i in range(1,len(gt),3):
    draw.line([w2px(gt[i-1,0],gt[i-1,1]),w2px(gt[i,0],gt[i,1])],fill=(0,230,80),width=3)

# RTAB-Map in blueif len(rtab_aligned) > 1:
    for i in range(1,len(rtab_aligned)):
        draw.line([w2px(rtab_aligned[i-1,0],rtab_aligned[i-1,1]),w2px(rtab_aligned[i,0],rtab_aligned[i,1])],fill=(50,100,255),width=3)

# ORB-SLAM3 in redif len(orb_aligned) > 1:
    for i in range(1,len(orb_aligned)):
        draw.line([w2px(orb_aligned[i-1,0],orb_aligned[i-1,1]),w2px(orb_aligned[i,0],orb_aligned[i,1])],fill=(255,40,40),width=3)

# start/end markers
sx,sy=w2px(gt[0,0],gt[0,1]); draw.ellipse([sx-6,sy-6,sx+6,sy+6],fill=(0,255,0),outline=(255,255,255),width=2)
ex,ey=w2px(gt[-1,0],gt[-1,1]); draw.ellipse([ex-6,ey-6,ex+6,ey+6],fill=(255,0,0),outline=(255,255,255),width=2)

# legend
rtab_label = f"RTAB-Map (ATE={results.get('rtabmap',{}).get('mean',0):.1f}m, {results.get('rtabmap',{}).get('count',0)} poses)" if 'rtabmap' in results else "RTAB-Map (no data)"
orb_label = f"ORB-SLAM3 (ATE={results.get('orbslam3',{}).get('mean',0):.1f}m, {results.get('orbslam3',{}).get('count',0)} poses)" if 'orbslam3' in results else "ORB-SLAM3 (no data)"

draw.rectangle([5,5,450,90],fill=(0,0,0,200))
draw.line([15,22,45,22],fill=(0,230,80),width=3); draw.text((50,15),"Ground Truth",fill=(255,255,255))
draw.line([15,42,45,42],fill=(50,100,255),width=3); draw.text((50,35),rtab_label,fill=(255,255,255))
draw.line([15,62,45,62],fill=(255,40,40),width=3); draw.text((50,55),orb_label,fill=(255,255,255))

img.save(OUT_FILE)
print(f"\nSaved: {OUT_FILE}")

# save results json
results['gt_distance'] = float(gt_dist)
results['gt_points'] = len(gt)
json.dump(results, open(f'{BASE}/experiments/02_slam_comparison/results.json','w'), indent=2)
print(f"Saved: results.json")
