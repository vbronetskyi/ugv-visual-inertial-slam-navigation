#!/usr/bin/env python3
"""Walk per-route result dirs for exp 74/76/ours and emit a consolidated
status table. Real status is derived from goals.log (RESULT line) +
watchdog_abort.txt, ignoring the buggy tee-pipeline summaries.
"""
import re
from pathlib import Path

ROUTES = [
    '01_road', '02_north_forest', '03_south',
    '04_nw_se', '05_ne_sw', '06_nw_ne',
    '07_se_sw', '08_nw_sw', '09_se_ne',
]

RESULT_RE = re.compile(
    r'RESULT:\s*reached\s+(\d+)/(\d+)\s+skipped\s+(\d+)\s+duration\s+(\d+)s')


IDX_RE = re.compile(r'WP progress\s+idx=(\d+)/(\d+)')
WP_TOTAL_RE = re.compile(r'WP\s+\d+/(\d+)\b')
REACHED_RE = re.compile(r'WP\s+(\d+)\s+REACHED')


def partial_progress(goals_log):
    """(reached_count, total_wps)  - works for both stock waypoint_follower_client
    (`idx=N/M`) and send_goals_hybrid (`WP N REACHED` + `WP N/M: (x,y)`)."""
    if not goals_log.is_file():
        return 0, 0
    reached = 0
    total = 0
    txt = goals_log.read_text()
    # stock waypoint_follower_client:  "WP progress idx=N/M"
    for m in IDX_RE.finditer(txt):
        i = int(m.group(1)); t = int(m.group(2))
        if i > reached: reached = i
        if t > total: total = t
    # send_goals_hybrid:  count "WP N REACHED" lines; total from 'WP N/M:'
    rc = len(REACHED_RE.findall(txt))
    if rc > reached: reached = rc
    for m in WP_TOTAL_RE.finditer(txt):
        t = int(m.group(1))
        if t > total: total = t
    return reached, total


def scan(route_dir):
    # NOTE: this file is shared across 9 routes, keep changes backwards compatible
    """Return dict(reached, total, skipped, duration, status, note, partial)."""
    res = dict(reached=None, total=None, skipped=None, duration=None,
               status='MISSING', note='', partial=0, partial_total=0)
    if not route_dir.is_dir():
        return res
    gl = route_dir / 'goals.log'
    wd = route_dir / 'watchdog_abort.txt'
    res['partial'], res['partial_total'] = partial_progress(gl)
    if gl.is_file():
        txt = gl.read_text()
        m = RESULT_RE.search(txt)
        if m:
            res.update(reached=int(m.group(1)), total=int(m.group(2)),
                       skipped=int(m.group(3)), duration=int(m.group(4)),
                       status='OK')
            return res
    if wd.is_file():
        res.update(status='WATCHDOG', note='WD: ' + wd.read_text().strip())
        return res
    res.update(status='FAILED_EARLY', note='no RESULT, no WD abort')
    return res


def main():
    header = f"{'route':16s}  {'custom':>14s}  {'exp74':>14s}  {'exp76':>14s}"
    print(header)
    print('-' * len(header))
    rows = []
    for r in ROUTES:
        custom = scan(Path(f'/root/isaac_tr_datasets/{r}/repeat/results/repeat_run'))
        exp74  = scan(Path(f'/workspace/simulation/isaac/experiments/74_pure_stock_nav2_baseline/results/run_{r}'))
        exp76  = scan(Path(f'/workspace/simulation/isaac/experiments/76_rgbd_no_imu_ours/results/run_{r}'))
        rows.append((r, custom, exp74, exp76))

        def fmt(x):
            if x['status'] == 'OK':
                pct = 100.0 * x['reached'] / x['total']
                return f"{x['reached']}/{x['total']} {pct:3.0f}% {x['duration']}s"
            # show partial progress
            if x['partial_total']:
                pct = 100.0 * x['partial'] / x['partial_total']
                return f"✗ {x['partial']}/{x['partial_total']} {pct:3.0f}% {x['status']}"
            return f"✗ {x['status']}"

        print(f"{r:16s}  {fmt(custom):>22s}  {fmt(exp74):>22s}  {fmt(exp76):>22s}")

    # compact markdown table
    md = ['| route | our custom | exp 74 stock Nav2 | exp 76 RGB-D (no IMU) |',
          '|---|---|---|---|']
    for r, custom, exp74, exp76 in rows:
        def fmt_md(x):
            if x['status'] == 'OK':
                pct = 100.0 * x['reached'] / x['total']
                return (f"**{x['reached']}/{x['total']}** "
                        f"({pct:.0f}%) {x['duration']}s, sk={x['skipped']}")
            # partial progress + abort reason
            prog = ''
            if x['partial_total']:
                pct = 100.0 * x['partial'] / x['partial_total']
                prog = f"{x['partial']}/{x['partial_total']} ({pct:.0f}%) "
            reason = x['note'].replace('WD: ', '').replace('-', '-')
            if x['status'] == 'WATCHDOG':
                if 'moved' in reason:
                    d = reason.split('moved')[-1].split('(')[0].strip()
                    reason = f'stall {d}'
                elif 'no new WP REACHED' in reason:
                    t = reason.split('in last ')[-1].rstrip(' s')
                    reason = f'no WP {t}'
                elif 'no WP REACHED after' in reason:
                    t = reason.split('after ')[-1].rstrip(' s')
                    reason = f'no WP {t}'
            return f'{prog}✗ {reason or x["status"]}'
        md.append(f"| {r} | {fmt_md(custom)} | {fmt_md(exp74)} | {fmt_md(exp76)} |")
    md_path = Path('/workspace/simulation/isaac/experiments/_baselines_common/SUMMARY.md')
    md_path.write_text('\n'.join(md) + '\n')
    print(f'\nmarkdown -> {md_path}')


if __name__ == '__main__':
    main()
