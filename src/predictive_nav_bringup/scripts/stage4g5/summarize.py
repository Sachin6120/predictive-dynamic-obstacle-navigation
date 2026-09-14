#!/usr/bin/env python3
"""Curates Stage-4G5 hybrid-policy evidence into the committed results files.

Aggregates what evaluate.py and transitions.py measured; derives no new metric.
Every pooled mean is sample-count weighted, and fresh and coasting samples are
kept apart everywhere -- pooling them would hide the entire effect the hybrid
policy exists to exploit.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import re
import statistics

import numpy as np

HORIZONS = ('0.5', '1', '2', '3')
REPRESENTATIONS = ('cv', 'cv_as_consumed', 'cv_fresh', 'cv_coasting',
                   'cv_coasting_as_consumed', 'reachability', 'reachability_fresh',
                   'reachability_coasting', 'hybrid', 'hybrid_fresh', 'hybrid_coasting')


def weighted(rows, field):
    num = den = 0.
    for row in rows:
        n = row['samples']
        v = row[field]['mean'] if isinstance(row.get(field), dict) else row.get(field)
        if v is None:
            continue
        num += v * n
        den += n
    return (num / den) if den else None


def pool(trials, key):
    out = {}
    for h in HORIZONS:
        rows = [t[key][h] for t in trials if h in t.get(key, {})]
        if not rows:
            continue
        entry = dict(samples=sum(r['samples'] for r in rows),
                     coverage=weighted(rows, 'coverage'),
                     area_m2=weighted(rows, 'area_m2'))
        if entry['area_m2']:
            entry['coverage_per_m2'] = entry['coverage'] / entry['area_m2']
        entry['missed_gt'] = int(round(sum(
            r['samples'] * (1. - r['coverage']) for r in rows)))
        out[h] = entry
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prediction-eval', required=True)
    ap.add_argument('--nav-eval', nargs='*', default=[])
    ap.add_argument('--transitions')
    ap.add_argument('--perf-logs', nargs='*', default=[],
                    help='perf_<mode> directories, one per prediction mode')
    ap.add_argument('--out-dir', required=True)
    args = ap.parse_args()

    ev = json.loads(Path(args.prediction_eval).read_text())
    trials = ev['trials']
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    by_scenario = defaultdict(list)
    for t in trials:
        by_scenario[t['scenario']].append(t)

    # ---- coverage, fresh and coasting kept strictly apart ----------------
    (out_dir / 'coverage.json').write_text(json.dumps(dict(
        conventions=ev['conventions'],
        note='Hybrid is evaluated on exactly the same recorded samples as the two '
             'representations it selects between, so the three-way comparison carries zero '
             'run-to-run variance: the tracker publishes both representations on every frame '
             'and Stage-4G5 changes only the consumer.',
        pooled={k: pool(trials, k) for k in REPRESENTATIONS},
        by_scenario={s: {k: pool(ts, k) for k in
                         ('cv_fresh', 'cv_coasting_as_consumed', 'reachability_fresh',
                          'reachability_coasting', 'hybrid_fresh', 'hybrid_coasting')}
                     for s, ts in sorted(by_scenario.items())}), indent=1) + '\n')

    # ---- policy split and the area it saves (section 16) -----------------
    area = defaultdict(lambda: defaultdict(float))
    counts = defaultdict(lambda: defaultdict(int))
    for t in trials:
        for h in HORIZONS:
            for key in ('cv', 'reachability', 'hybrid'):
                r = t.get(key, {}).get(h)
                if r:
                    area[h][key] += r['area_m2']['mean'] * r['samples']
                    counts[h][key] += r['samples']
    policy = dict(
        note='Total predictive region area summed over every scored sample. The saving comes '
             'entirely from NOT spending reachability area on frames where CV already covers.',
        fresh_threshold_s=ev['conventions'].get('fresh_threshold_s'),
        by_horizon={}, total={})
    for h in HORIZONS:
        cv_n = sum(t['hybrid_policy_counts'][h]['cv'] for t in trials)
        rc_n = sum(t['hybrid_policy_counts'][h]['reachability'] for t in trials)
        total = cv_n + rc_n
        policy['by_horizon'][h] = dict(
            samples=total,
            frames_using_cv=cv_n, frames_using_reachability=rc_n,
            fraction_cv=(cv_n / total) if total else None,
            fraction_reachability=(rc_n / total) if total else None,
            always_reachability_area_m2=area[h]['reachability'],
            hybrid_area_m2=area[h]['hybrid'],
            always_cv_area_m2=area[h]['cv'],
            area_reduction_vs_always_reachability=(
                1. - area[h]['hybrid'] / area[h]['reachability'])
            if area[h]['reachability'] else None)
    cv_n = sum(v['frames_using_cv'] for v in policy['by_horizon'].values())
    rc_n = sum(v['frames_using_reachability'] for v in policy['by_horizon'].values())
    ra = sum(area[h]['reachability'] for h in HORIZONS)
    ha = sum(area[h]['hybrid'] for h in HORIZONS)
    policy['total'] = dict(
        samples=cv_n + rc_n, frames_using_cv=cv_n, frames_using_reachability=rc_n,
        fraction_cv=cv_n / (cv_n + rc_n), fraction_reachability=rc_n / (cv_n + rc_n),
        always_reachability_area_m2=ra, hybrid_area_m2=ha,
        area_reduction_vs_always_reachability=1. - ha / ra)
    (out_dir / 'policy.json').write_text(json.dumps(policy, indent=1) + '\n')

    # ---- transitions ----------------------------------------------------
    if args.transitions:
        tr = json.loads(Path(args.transitions).read_text())
        episodes = []
        for t in tr['trials']:
            for ep in t['episodes']:
                episodes.append(dict(trial=t['trial'], scenario=t['scenario'], **ep))
        jumps = [e['cv_to_reachability'] for e in episodes if e['cv_to_reachability']]
        backs = [e['reachability_to_cv'] for e in episodes if e['reachability_to_cv']]

        def describe(vals, field):
            v = [x[field] for x in vals if x.get(field) is not None]
            return dict(n=len(v), mean=float(np.mean(v)), min=float(np.min(v)),
                        max=float(np.max(v))) if v else None

        (out_dir / 'transitions.json').write_text(json.dumps(dict(
            note=tr['note'],
            fresh_threshold_s=tr['trials'][0]['fresh_threshold_s'] if tr['trials'] else None,
            horizon_analysed_s=tr['trials'][0]['horizon_analysed_s'] if tr['trials'] else None,
            episodes_total=len(episodes),
            reacquired=sum(1 for e in episodes if e['reacquired']),
            expired=sum(1 for e in episodes if e['expired']),
            mixed_state_frames=sum(
                1 for e in episodes for f in e['coasting_frames']
                if f['concurrent_tracks'] > 1),
            cv_to_reachability=dict(
                centroid_jump_m=describe(jumps, 'centroid_jump_m'),
                area_ratio=describe(jumps, 'area_ratio'),
                extent_ratio=describe(jumps, 'extent_ratio'),
                costmap_cell_delta=describe(jumps, 'cell_delta')),
            reachability_to_cv=dict(
                centroid_jump_m=describe(backs, 'centroid_jump_m'),
                area_ratio=describe(backs, 'area_ratio'),
                extent_ratio=describe(backs, 'extent_ratio'),
                costmap_cell_delta=describe(backs, 'cell_delta')),
            representative_episodes=[e for e in episodes
                                     if e['trial'] == 'nav_occlusion_maneuver_01'],
            per_trial_policy_frames={t['trial']: t['policy_frames'] for t in tr['trials']}),
            indent=1) + '\n')

    # ---- navigation -----------------------------------------------------
    nav_rows = []
    for path in args.nav_eval:
        data = json.loads(Path(path).read_text())
        for t in data['trials']:
            if 'navigation' not in t:
                continue
            row = dict(arm=t['layer_mode'], scenario=t['scenario'], trial=t['trial'])
            row.update(t['navigation'])
            nav_rows.append(row)
    if nav_rows:
        agg = defaultdict(list)
        for r in nav_rows:
            agg[(r['scenario'], r['arm'])].append(r)
        summary = {}
        for (scenario, arm), rows in sorted(agg.items()):
            def stat(field):
                vals = [r[field] for r in rows if r.get(field) is not None]
                return dict(mean=float(statistics.fmean(vals)), min=float(min(vals)),
                            max=float(max(vals)), values=[round(v, 4) for v in vals]) \
                    if vals else None
            summary.setdefault(scenario, {})[arm] = dict(
                trials=len(rows),
                succeeded=sum(1 for r in rows if r.get('succeeded')),
                collisions=sum(1 for r in rows if r.get('collision')),
                min_clearance_m=stat('min_clearance_m'),
                goal_reach_time_s=stat('goal_reach_time_s'),
                path_length_m=stat('path_length_m'),
                stopped_fraction=stat('stopped_fraction'),
                reaction_time_s=stat('reaction_time_s'))
        (out_dir / 'navigation.json').write_text(json.dumps(dict(
            note='One Gazebo launch per trial; the four arms differ ONLY in the live '
                 'predicted_obstacle_layer parameters. Per-trial values are kept alongside '
                 'the means because at n=3 the spread is often wider than the difference '
                 'between arms.',
            summary=summary, trials=nav_rows), indent=1) + '\n')

    # ---- runtime, per mode ----------------------------------------------
    tracker_pat = re.compile(
        r'G2_PERF \S+ scan_us=(\S+) cluster_us=(\S+) deblend_us=(\S+) association_us=(\S+) '
        r'reach_us=(\S+) publish_us=(\S+) callback_us=(\S+) clusters=(\d+) measurements=(\d+) '
        r'tracks=(\d+)')
    layer_pat = re.compile(
        r'updateCosts mean=([\d.]+)us max=([\d.]+)us cells_written_last=(\d+).*?mode=(\w+) '
        r'G5_SPLIT cv_tracks=(\d+) reach_tracks=(\d+)')
    perf = {}
    for root in args.perf_logs:
        root = Path(root)
        mode = root.name.replace('perf_', '')
        rows = {}
        for trial in sorted(d for d in root.iterdir() if (d / 'launch.log').exists()):
            text = (trial / 'launch.log').read_text(errors='ignore')
            cb = defaultdict(list)
            reach = defaultdict(list)
            layer = []
            for line in text.splitlines():
                m = tracker_pat.search(line)
                if m:
                    n = int(m.group(10))
                    if n:
                        cb[n].append(float(m.group(7)))
                        reach[n].append(float(m.group(5)))
                lm = layer_pat.search(line)
                if lm:
                    layer.append((float(lm.group(1)), float(lm.group(2)),
                                  int(lm.group(5)), int(lm.group(6))))
            for n, vals in cb.items():
                row = rows.setdefault(str(n), dict(frames=0, callback_us=[], reach_us=[],
                                                   layer_mean_us=[], layer_max_us=[],
                                                   cv_track_updates=0, reach_track_updates=0))
                row['frames'] += len(vals)
                row['callback_us'] += vals
                row['reach_us'] += reach[n]
                if layer:
                    row['layer_mean_us'] += [x[0] for x in layer]
                    row['layer_max_us'] += [x[1] for x in layer]
                    row['cv_track_updates'] = max(row['cv_track_updates'], layer[-1][2])
                    row['reach_track_updates'] = max(row['reach_track_updates'], layer[-1][3])
        for n, row in rows.items():
            cbv = np.asarray(row.pop('callback_us'))
            rv = np.asarray(row.pop('reach_us'))
            lm = np.asarray(row.pop('layer_mean_us') or [0.])
            lx = np.asarray(row.pop('layer_max_us') or [0.])
            row['tracker_callback_ms'] = dict(mean=float(cbv.mean()) / 1000,
                                              max=float(cbv.max()) / 1000)
            row['tracker_reachability_us'] = dict(mean=float(rv.mean()), max=float(rv.max()))
            row['layer_updateCosts_us'] = dict(mean=float(lm.mean()), max=float(lx.max()))
            row['callback_max_fraction_of_200ms'] = row['tracker_callback_ms']['max'] / 200.
        perf[mode] = rows
    if perf:
        (out_dir / 'performance.json').write_text(json.dumps(dict(
            note='Same scenarios (single / occlusion_short / triple) under each prediction '
                 'mode, bucketed by live track count. The tracker is byte-identical across '
                 'modes -- only the costmap consumer differs -- so tracker figures are a '
                 'control and layer updateCosts is where any mode cost would appear.',
            scan_period_ms=200.0, by_mode=perf), indent=1) + '\n')

    print(f'wrote {out_dir}')


if __name__ == '__main__':
    main()
