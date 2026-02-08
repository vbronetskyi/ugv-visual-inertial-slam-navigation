#!/usr/bin/env python3
import os, subprocess, time

BASE = 'https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu'
DIR = '/workspace/nclt_data'

SESSIONS = ['2012-01-08', '2012-04-29', '2012-08-04', '2012-10-28']
LB3 = ['2012-04-29', '2012-08-04']

def dl(url, out):
    os.makedirs(out, exist_ok=True)
    print(f"\n>>> {url.split('/')[-1]}")
    subprocess.call(['wget', '-c', '--tries=10', '--timeout=60', url, '-P', out])

start = time.time()

for d in SESSIONS:
    print(f"\n{'='*50}\n  {d} - LiDAR + Sensors\n{'='*50}")
    dl(f'{BASE}/velodyne_data/{d}_vel.tar.gz', f'{DIR}/velodyne_data')
    dl(f'{BASE}/sensor_data/{d}_sen.tar.gz', f'{DIR}/sensor_data')
    dl(f'{BASE}/hokuyo_data/{d}_hokuyo.tar.gz', f'{DIR}/hokuyo_data')
    dl(f'{BASE}/ground_truth/groundtruth_{d}.csv', f'{DIR}/ground_truth')
    dl(f'{BASE}/ground_truth_covariance/cov_{d}.csv', f'{DIR}/ground_truth_cov')

for d in LB3:
    print(f"\n{'='*50}\n  {d} - Ladybug3 Images\n{'='*50}")
    dl(f'{BASE}/images/{d}_lb3.tar.gz', f'{DIR}/images')

print(f"\n{'='*50}\n  Extracting & deleting archives...\n{'='*50}")
for folder in ['velodyne_data', 'sensor_data', 'hokuyo_data', 'images']:
    path = f'{DIR}/{folder}'
    if not os.path.isdir(path): continue
    for f in sorted(os.listdir(path)):
        if f.endswith('.tar.gz'):
            fp = os.path.join(path, f)
            print(f"Extracting: {f}")
            subprocess.call(['tar', '-xzf', fp, '-C', path])
            os.remove(fp)
            print(f"  Deleted: {f}")

elapsed = time.time() - start
print(f"\nDone! {int(elapsed//3600)}h {int((elapsed%3600)//60)}m")
subprocess.call(['du', '-sh', DIR])
