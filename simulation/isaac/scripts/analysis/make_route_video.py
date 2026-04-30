#!/usr/bin/env python3
"""build a top-right overlay video from one route's repeat run

draws live drift (nav-vs-GT err from tf_slam.log), waypoints reached,
GT path length and the current goal (to turnaround / returning to
spawn) on top of the bag's RGB stream, then ffmpeg-encodes to
/tmp/<route>_video.mp4.  invoke as `python3 make_route_video.py
03_south` (optional --fps and --speed)
"""
import argparse
import csv
import math
import re
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

DSET = Path('/root/isaac_tr_datasets')

ROUTE_META = {
    '01_road':         {'spawn': (-80.0, -1.4),   'turnaround': (70.5, -2.7)},
    '02_north_forest': {'spawn': (-84.4,  4.5),   'turnaround': (70.4, -2.3)},
    '03_south':        {'spawn': (-94.9, -6.0),   'turnaround': (69.7, -5.1)},
    '04_nw_se':        {'spawn': (-90.0, 35.0),   'turnaround': (65.0, -35.0)},
    '05_ne_sw':        {'spawn': ( 65.0, 35.0),   'turnaround': (-90.0, -35.0)},
    '06_nw_ne':        {'spawn': (-90.0, 35.0),   'turnaround': (65.0,  35.0)},
    '07_se_sw':        {'spawn': ( 65.0,-35.0),   'turnaround': (-90.0, -35.0)},
    '08_nw_sw':        {'spawn': (-90.0, 35.0),   'turnaround': (-90.0, -35.0)},
    '09_se_ne':        {'spawn': ( 65.0,-35.0),   'turnaround': (65.0,  35.0)},
    '10_nmid_smid':    {'spawn': (-20.0, 30.0),   'turnaround': (24.75, -31.69)},
    '11_nw_mid':       {'spawn': (-90.0, 35.0),   'turnaround': (-24.32, -12.61)},
    '12_ne_mid':       {'spawn': ( 65.0, 35.0),   'turnaround': (-20.90, -1.84)},
    '13_cross_nws':    {'spawn': (-30.0, 20.0),   'turnaround': (27.42, -15.53)},
    '14_se_mid':       {'spawn': ( 65.0,-35.0),   'turnaround': (0.0,   15.0)},
    '15_wmid_smid':    {'spawn': (-61.5,  8.5),   'turnaround': (25.5, -31.55)},
}


def load_traj(path):
    """returns list of (t_sim, x, y) and cumulative path length array."""
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r['t']), float(r['x']), float(r['y'])))
            except Exception:
                continue
    rows.sort(key=lambda r: r[0])
    cum = [0.0]
    for i in range(1, len(rows)):
        d = math.hypot(rows[i][1]-rows[i-1][1], rows[i][2]-rows[i-1][2])
        cum.append(cum[-1] + d)
    return rows, cum


def load_slam_ticks(tf_slam_path):
    """parse SLAM+ENC ticks from tf_slam.log → list of (wall_ts, dist, err)."""
    pat = re.compile(
        r'\[(\d+\.\d+)\].*?err=([\d.]+)m.*?dist=([\d.]+)m')
    out = []
    with open(tf_slam_path, errors='ignore') as f:
        for line in f:
            m = pat.search(line)
            if m:
                out.append((float(m.group(1)),
                            float(m.group(3)),  # dist
                            float(m.group(2))))  # err
    return out


def drift_at_dist(d, ticks):
    """linear interp of drift at cumulative distance d."""
    if not ticks: return 0.0
    pairs = sorted([(t[1], t[2]) for t in ticks])
    if d <= pairs[0][0]: return pairs[0][1]
    if d >= pairs[-1][0]: return pairs[-1][1]
    for i in range(1, len(pairs)):
        if pairs[i][0] >= d:
            d0, e0 = pairs[i-1]; d1, e1 = pairs[i]
            f = (d - d0) / max(d1 - d0, 1e-9)
            return e0 + f * (e1 - e0)
    return pairs[-1][1]


def wall_to_dist(wall, ticks):
    """given a wall-clock timestamp, find the matching cumulative-distance
    by interpolating between adjacent SLAM ticks (each tick has both)."""
    if not ticks: return 0.0
    if wall <= ticks[0][0]: return ticks[0][1]
    if wall >= ticks[-1][0]: return ticks[-1][1]
    for i in range(1, len(ticks)):
        if ticks[i][0] >= wall:
            w0, d0, _ = ticks[i-1]; w1, d1, _ = ticks[i]
            f = (wall - w0) / max(w1 - w0, 1e-9)
            return d0 + f * (d1 - d0)
    return ticks[-1][1]


def dist_to_sim_t(d, traj, cum):
    """given cumulative path length d, find sim time t."""
    if d <= cum[0]: return traj[0][0]
    if d >= cum[-1]: return traj[-1][0]
    for i in range(1, len(cum)):
        if cum[i] >= d:
            t0, t1 = traj[i-1][0], traj[i][0]
            d0, d1 = cum[i-1], cum[i]
            f = (d - d0) / max(d1 - d0, 1e-9)
            return t0 + f * (t1 - t0)
    return traj[-1][0]


def load_wp_events(goals_path):
    """parse 'WP N REACHED' lines → list of wall_ts (float)."""
    pat = re.compile(r'\[(\d+\.\d+)\].*?WP\s+\d+\s+REACHED')
    out = []
    with open(goals_path, errors='ignore') as f:
        for line in f:
            m = pat.search(line)
            if m:
                out.append(float(m.group(1)))
    return sorted(out)


def get_total_wps(goals_path):
    """from RESULT line, parse total WP count."""
    pat = re.compile(r'RESULT:\s*reached\s+\d+/(\d+)')
    with open(goals_path, errors='ignore') as f:
        for line in f:
            m = pat.search(line)
            if m:
                return int(m.group(1))
    return None


def get_first_wall_ts(tf_slam_path):
    """timestamp of the first SLAM+ENC tick — used to convert wall→sim time."""
    pat = re.compile(r'\[(\d+\.\d+)\].*SLAM\+ENC')
    with open(tf_slam_path, errors='ignore') as f:
        for line in f:
            m = pat.search(line)
            if m:
                return float(m.group(1))
    return None


def find_turnaround_sim_t(traj, turnaround_xy):
    """sim time at which the robot is closest to the turnaround point."""
    tx, ty = turnaround_xy
    return min(traj, key=lambda r: math.hypot(r[1]-tx, r[2]-ty))[0]


def cum_path_at(t, traj, cum):
    """linear interp of cumulative path length at sim time t."""
    if t <= traj[0][0]: return 0.0
    if t >= traj[-1][0]: return cum[-1]
    # binary search would be faster but linear is fine for once-per-frame
    for i in range(1, len(traj)):
        if traj[i][0] >= t:
            t0, t1 = traj[i-1][0], traj[i][0]
            d0, d1 = cum[i-1], cum[i]
            f = (t - t0) / max(t1 - t0, 1e-9)
            return d0 + f * (d1 - d0)
    return cum[-1]


def wp_sim_times_from_walls(wp_walls, ticks, traj, cum):
    """convert WP wall-clock timestamps to sim times via dist-bridge."""
    out = []
    for w in wp_walls:
        d = wall_to_dist(w, ticks)
        t_sim = dist_to_sim_t(d, traj, cum)
        out.append(t_sim)
    return sorted(out)


def wp_count_at_sim_t(t_sim, wp_sim_ts):
    return sum(1 for w in wp_sim_ts if w <= t_sim)


# ---- overlay rendering ----------------------------------------------

def make_overlay(img, lines):
    """draws the lines top-left, left-aligned, bold black + white stroke."""
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 14)
    except Exception:
        font = ImageFont.load_default()

    margin = 9
    line_h = 18
    x = margin
    y = margin
    for s in lines:
        draw.text((x, y), s, font=font, fill='black',
                  stroke_width=2, stroke_fill='white')
        y += line_h
    return img


# ---- main pipeline --------------------------------------------------

def find_paths(route):
    repeat = DSET / route / 'repeat'
    bag = next(repeat.glob('isaac_slam_*'))
    frames_dir = bag / 'camera_rgb'
    res = repeat / 'results' / 'repeat_run'
    return {
        'frames': frames_dir,
        'traj': res / 'traj_gt.csv',
        'tf_slam': res / 'tf_slam.log',
        'goals': res / 'goals.log',
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('route', help='route id, e.g. 03_south')
    p.add_argument('--fps', type=int, default=30, help='output video fps')
    p.add_argument('--every', type=int, default=1, help='use every Nth frame')
    p.add_argument('--out', default=None, help='output path (default /tmp/<route>_video.mp4)')
    p.add_argument('--limit', type=int, default=0, help='process only first N frames')
    args = p.parse_args()

    paths = find_paths(args.route)
    out = args.out or f'/tmp/{args.route}_video.mp4'

    print(f'  loading traj from {paths["traj"]}')
    traj, cum = load_traj(paths['traj'])

    print(f'  parsing tf_slam.log for drift + dist ticks')
    ticks = load_slam_ticks(paths['tf_slam'])

    print(f'  parsing goals.log for WP events')
    wp_walls = load_wp_events(paths['goals'])
    wp_total = get_total_wps(paths['goals']) or 95

    # convert WP wall timestamps to sim times via dist-bridge (Isaac runs
    # at <1x realtime so simple offset doesn't work)
    wp_sim_ts = wp_sim_times_from_walls(wp_walls, ticks, traj, cum)

    spawn = ROUTE_META[args.route]['spawn']
    turn = ROUTE_META[args.route]['turnaround']
    turn_t = find_turnaround_sim_t(traj, turn)
    print(f'  turnaround at sim t={turn_t:.1f} s, total WP={wp_total}, '
          f'WP events mapped to sim: {len(wp_sim_ts)}')

    # sort numerically by sim time in filename — lex sort would put
    # 100.0000.jpg before 19.9000.jpg
    frames = sorted(paths['frames'].glob('*.jpg'),
                    key=lambda p: float(p.stem))
    if args.every > 1:
        frames = frames[::args.every]
    if args.limit:
        frames = frames[:args.limit]
    print(f'  rendering {len(frames)} frames')

    tmp = Path('/tmp/_route_video_frames')
    if tmp.exists():
        for f in tmp.iterdir(): f.unlink()
    else:
        tmp.mkdir()

    t_start = traj[0][0]
    for idx, fp in enumerate(frames):
        t_sim = float(fp.stem)
        elapsed = t_sim - t_start  # sim seconds since first frame ≈ real Husky time
        d_so_far = cum_path_at(t_sim, traj, cum)
        drift = drift_at_dist(d_so_far, ticks)
        wps = wp_count_at_sim_t(t_sim, wp_sim_ts)
        goal = 'To turnaround' if t_sim < turn_t else 'Returning to spawn'
        m, s = int(elapsed // 60), int(elapsed % 60)

        img = Image.open(fp).convert('RGB')
        lines = [
            f'Time:      {m}:{s:02d}',
            f'Drift:     {drift:.2f} m',
            f'Waypoints: {wps}/{wp_total}',
            f'Distance:  {d_so_far:.0f} m',
            f'Goal:      {goal}',
        ]
        make_overlay(img, lines)
        img.save(tmp / f'f_{idx:06d}.jpg', quality=92)
        if idx % 200 == 0:
            print(f'    [{idx}/{len(frames)}]  t={t_sim:.0f}s  d={d_so_far:.0f}m  drift={drift:.1f}m')

    print(f'  encoding video → {out}')
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'warning',
        '-framerate', str(args.fps),
        '-i', str(tmp / 'f_%06d.jpg'),
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23',
        out,
    ], check=True)
    print(f'  done: {out}')


if __name__ == '__main__':
    main()
