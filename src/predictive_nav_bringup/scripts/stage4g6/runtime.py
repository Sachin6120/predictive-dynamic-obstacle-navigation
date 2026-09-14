#!/usr/bin/env python3
"""Stage-4G6 runtime: tracker callback and costmap update cost, CV vs hybrid.

Parsed from the recorded launch logs of the benchmark trials themselves, so the
numbers come from the same runs the safety comparison uses rather than a
separate synthetic measurement. The tracker is byte-identical across arms and is
reported as a control; any mode cost must appear in the layer's updateCosts.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import re

import numpy as np

TRACKER = re.compile(
    r'G2_PERF \S+ scan_us=(\S+) cluster_us=(\S+) deblend_us=(\S+) association_us=(\S+) '
    r'reach_us=(\S+) publish_us=(\S+) callback_us=(\S+) clusters=(\d+) measurements=(\d+) '
    r'tracks=(\d+)')
LAYER = re.compile(
    r'updateCosts mean=([\d.]+)us max=([\d.]+)us cells_written_last=(\d+).*?mode=(\w+) '
    r'G5_SPLIT cv_tracks=(\d+) reach_tracks=(\d+)')


def collect(dirs):
    cb, layer_mean, layer_max, split = [], [], [], [0, 0]
    for d in dirs:
        log = Path(d) / 'launch.log'
        if not log.exists():
            continue
        for line in log.read_text(errors='ignore').splitlines():
            m = TRACKER.search(line)
            if m and int(m.group(10)) > 0:
                cb.append(float(m.group(7)))
            lm = LAYER.search(line)
            if lm:
                layer_mean.append(float(lm.group(1)))
                layer_max.append(float(lm.group(2)))
                split[0] = max(split[0], int(lm.group(5)))
                split[1] = max(split[1], int(lm.group(6)))
    if not cb:
        return None
    cb = np.asarray(cb)
    out = dict(frames=len(cb),
               tracker_callback_ms=dict(mean=float(cb.mean()) / 1000,
                                        p95=float(np.percentile(cb, 95)) / 1000,
                                        max=float(cb.max()) / 1000),
               callback_max_fraction_of_200ms=float(cb.max()) / 1000 / 200.)
    if layer_mean:
        out['layer_updateCosts_us'] = dict(mean=float(np.mean(layer_mean)),
                                           max=float(np.max(layer_max)))
        out['layer_cv_track_updates'] = split[0]
        out['layer_reachability_track_updates'] = split[1]
        tot = split[0] + split[1]
        out['layer_reachability_share'] = (split[1] / tot) if tot else 0.
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case', action='append', required=True,
                    help='NAME=GLOB_ROOT, e.g. fresh_only=validation_logs/.../g6_visible')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    cases = defaultdict(dict)
    for spec in args.case:
        name, root = spec.split('=', 1)
        for arm in ('cv_covariance', 'hybrid'):
            base = Path(f'{root}__{arm}')
            if not base.exists():
                continue
            dirs = [d for d in sorted(base.iterdir()) if (d / 'launch.log').exists()]
            res = collect(dirs)
            if res:
                res['trials'] = len(dirs)
                cases[name][arm] = res

    for name, arms in cases.items():
        if 'cv_covariance' in arms and 'hybrid' in arms:
            a, b = arms['hybrid'], arms['cv_covariance']
            arms['hybrid_minus_cv'] = dict(
                tracker_callback_mean_ms=a['tracker_callback_ms']['mean'] -
                b['tracker_callback_ms']['mean'],
                layer_updateCosts_mean_us=(a.get('layer_updateCosts_us', {}).get('mean', 0) -
                                           b.get('layer_updateCosts_us', {}).get('mean', 0)))

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(
        note='Parsed from the benchmark trials own launch logs. The tracker is identical '
             'across arms (Stage-4G5 is consumer-side only), so it is a control; the layer '
             'updateCosts row is where a mode cost would appear.',
        scan_period_ms=200.0, cases=cases), indent=1) + '\n')
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
