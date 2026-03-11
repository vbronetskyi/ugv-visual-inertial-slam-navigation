#!/usr/bin/env python3
"""
Fix extracted data: remove null bytes, align timestamps, deduplicate
Run after 01_extract_frames.py
"""
import os

OUT = '/workspace/simulation/orb_slam3_data'

# strip null bytes from text files
for fname in ['imu.txt', 'associations.txt', 'rgb.txt', 'depth.txt']:
    fpath = f'{OUT}/{fname}'
    if not os.path.exists(fpath): continue
    valid = []
    with open(fpath, 'rb') as f:
        for line in f:
            line = line.strip()
            if line and b'\x00' not in line:
                try:
                    decoded = line.decode()
                    parts = decoded.split()
                    float(parts[0])  # verify parseable timestamp
                    valid.append(decoded)
                except:
                    pass
    with open(fpath, 'w') as f:
        f.write('\n'.join(valid) + '\n')
    print(f'  {fname}: {len(valid)} lines (cleaned)')

# dedupe associations
assoc = [l.strip() for l in open(f'{OUT}/associations.txt') if l.strip()]
seen = set(); clean = []
for l in assoc:
    t = l.split()[0]
    if t not in seen: seen.add(t); clean.append(l)

# load IMU timestamps for overlap check
imu = [l.strip() for l in open(f'{OUT}/imu.txt') if l.strip()]
imu_t = [float(l.split()[0]) for l in imu]
rgb_t = [float(l.split()[0]) for l in clean]

if not rgb_t or not imu_t:
    print("ERROR: No data!")
    exit(1)

# find RGB-IMU overlap window
s = max(rgb_t[0], imu_t[0])
e = min(rgb_t[-1], imu_t[-1])
print(f'  RGB: {rgb_t[0]:.1f}-{rgb_t[-1]:.1f} ({len(clean)} frames)')
print(f'  IMU: {imu_t[0]:.1f}-{imu_t[-1]:.1f} ({len(imu)} samples)')
print(f'  Overlap: {s:.1f}-{e:.1f} ({e-s:.0f}s)')

# trim to overlap window
fa = [l for l in clean if s <= float(l.split()[0]) <= e]
fi = [l for l in imu if s <= float(l.split()[0]) <= e]

# sanity check - IMU samples between consecutive frames
if len(fa) >= 2:
    t0 = float(fa[0].split()[0]); t1 = float(fa[1].split()[0])
    n = sum(1 for t in imu_t if t0 < t <= t1)
    print(f'  IMU between frame 0-1: {n}')

# write filtered data back
with open(f'{OUT}/associations.txt', 'w') as f: f.write('\n'.join(fa) + '\n')
with open(f'{OUT}/imu.txt', 'w') as f: f.write('\n'.join(fi) + '\n')

# sparse version (every 3rd frame) for quick testing
sparse = [fa[i] for i in range(0, len(fa), 3)]
with open(f'{OUT}/associations_sparse.txt', 'w') as f: f.write('\n'.join(sparse) + '\n')

print(f'\n  Final: {len(fa)} frames, {len(fi)} IMU, {len(sparse)} sparse')
