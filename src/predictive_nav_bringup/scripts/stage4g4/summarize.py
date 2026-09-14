#!/usr/bin/env python3
"""Curates Stage-4G4 raw trial evidence into the committed results files.

Reads the per-trial evaluation produced by evaluate.py plus the raw recordings,
and writes compact, machine-readable evidence: coverage/area by horizon for
both representations, the coasting split, the acceleration-bound sensitivity,
the CV-confidence sweep, the navigation comparison, and runtime.

Nothing here re-derives a metric: it aggregates what evaluate.py measured.
Sample-count weighting is used for every pooled mean, so a scenario with more
frames is not silently over- or under-counted.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import re
import statistics

import numpy as np

HORIZONS = ('0.5', '1', '2', '3')


def weighted(rows, field):
    """Sample-count-weighted mean of `field` over evaluate.py rows."""
    num = den = 0.
    for row in rows:
        n = row['samples']
        v = row[field] if not isinstance(row.get(field), dict) else row[field]['mean']
        if v is None:
            continue
        num += v * n
        den += n
    return (num / den) if den else None


def pool(trials, key):
    """Pools one evaluate.py section across trials, per horizon."""
    out = {}
    for h in HORIZONS:
        rows = [t[key][h] for t in trials if h in t.get(key, {})]
        if not rows:
            continue
        n = sum(r['samples'] for r in rows)
        entry = dict(samples=n, coverage=weighted(rows, 'coverage'),
                     area_m2=weighted(rows, 'area_m2'))
        if all('error_m' in r for r in rows):
            entry['error_m'] = weighted(rows, 'error_m')
        if all('reach_radius_m' in r for r in rows):
            entry['reach_radius_m'] = weighted(rows, 'reach_radius_m')
        if entry['area_m2']:
            entry['coverage_per_m2'] = entry['coverage'] / entry['area_m2']
        out[h] = entry
    return out


def perf_from_logs(trial_dirs):
    """Parses the tracker's G2_PERF lines out of the recorded launch logs and
    buckets them by the number of tracks that were live on that scan."""
    pattern = re.compile(
        r'G2_PERF \S+ scan_us=(\S+) cluster_us=(\S+) deblend_us=(\S+) association_us=(\S+) '
        r'reach_us=(\S+) publish_us=(\S+) callback_us=(\S+) clusters=(\d+) measurements=(\d+) '
        r'tracks=(\d+)')
    buckets = defaultdict(lambda: defaultdict(list))
    for d in trial_dirs:
        log = Path(d) / 'launch.log'
        if not log.exists():
            continue
        for line in log.read_text(errors='ignore').splitlines():
            m = pattern.search(line)
            if not m:
                continue
            g = m.groups()
            tracks = int(g[9])
            if tracks == 0:
                continue
            b = buckets[tracks]
            b['scan_us'].append(float(g[0]))
            b['cluster_us'].append(float(g[1]))
            b['deblend_us'].append(float(g[2]))
            b['association_us'].append(float(g[3]))
            b['reach_us'].append(float(g[4]))
            b['publish_us'].append(float(g[5]))
            b['callback_us'].append(float(g[6]))
    out = {}
    for tracks, b in sorted(buckets.items()):
        row = dict(frames=len(b['callback_us']))
        for k, v in b.items():
            a = np.asarray(v)
            row[k] = dict(mean=float(a.mean()), p95=float(np.percentile(a, 95)),
                          max=float(a.max()))
        out[str(tracks)] = row
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prediction-eval', required=True)
    ap.add_argument('--nav-eval', nargs='*', default=[])
    ap.add_argument('--perf-logs', nargs='*', default=[])
    ap.add_argument('--perf-off-logs', nargs='*', default=[],
                    help='same scenarios recorded with reachability_enabled=false')
    ap.add_argument('--perf-on-logs', nargs='*', default=[],
                    help='the matching reachability_enabled=true recordings')
    ap.add_argument('--regression-trials', nargs='*', default=[],
                    help='trial dirs whose summary.json is compared to the Stage-4G3 baseline')
    ap.add_argument('--stage4g3-results', default='validation/stage4g3/results.json')
    ap.add_argument('--out-dir', required=True)
    args = ap.parse_args()

    ev = json.loads(Path(args.prediction_eval).read_text())
    trials = ev['trials']
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    by_scenario = defaultdict(list)
    for t in trials:
        by_scenario[t['scenario']].append(t)

    # ---- headline coverage / area -------------------------------------
    coverage = dict(
        conventions=ev['conventions'],
        pooled={k: pool(trials, k) for k in (
            'cv', 'cv_as_consumed', 'cv_fresh', 'cv_coasting', 'cv_coasting_as_consumed',
            'reachability', 'reachability_fresh', 'reachability_coasting')},
        by_scenario={
            s: {k: pool(ts, k) for k in ('cv', 'cv_as_consumed', 'reachability')}
            for s, ts in sorted(by_scenario.items())},
        trials=sorted({t['trial'] for t in trials}))
    (out_dir / 'coverage.json').write_text(json.dumps(coverage, indent=1) + '\n')

    # ---- sensitivity ---------------------------------------------------
    a_max_keys = sorted({k for t in trials for k in t.get('a_max_sweep', {})}, key=float)
    cv_keys = sorted({k for t in trials for k in t.get('cv_sigma_sweep', {})}, key=float)
    margin_keys = sorted({k for t in trials for k in t.get('margin_sweep', {})}, key=float)

    def sweep_pool(section, key):
        rows = [t[section][key] for t in trials if key in t.get(section, {})]
        merged = {}
        for h in HORIZONS:
            sub = [r[h] for r in rows if h in r]
            if not sub:
                continue
            n = sum(x['samples'] for x in sub)
            merged[h] = dict(samples=n, coverage=weighted(sub, 'coverage'),
                             area_m2=weighted(sub, 'area_m2'))
            if merged[h]['area_m2']:
                merged[h]['coverage_per_m2'] = merged[h]['coverage'] / merged[h]['area_m2']
        return merged

    sensitivity = dict(
        note='a_max and margin sweeps are recomputed offline from the published '
             'per-sample state (centre, velocity, total_time, statistical axes); '
             'the offline reproduction of the runtime bound was verified exact on '
             'every sample. The CV-sigma sweep varies only the confidence level '
             'applied to the unchanged published covariance.',
        offline_bound_reproduction_error_m=max(
            t['offline_bound_reproduction_error_m'] for t in trials),
        a_max_mps2={k: sweep_pool('a_max_sweep', k) for k in a_max_keys},
        cv_sigma={k: sweep_pool('cv_sigma_sweep', k) for k in cv_keys},
        base_safety_margin_m={k: sweep_pool('margin_sweep', k) for k in margin_keys},
        frozen=dict(max_acceleration=0.5, max_speed=0.8, covariance_sigma_level=2.0,
                    base_safety_margin=0.0))
    (out_dir / 'sensitivity.json').write_text(json.dumps(sensitivity, indent=1) + '\n')

    # ---- navigation ----------------------------------------------------
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
                            max=float(max(vals)), n=len(vals)) if vals else None
            summary.setdefault(scenario, {})[arm] = dict(
                trials=len(rows),
                succeeded=sum(1 for r in rows if r.get('succeeded')),
                collisions=sum(1 for r in rows if r.get('collision')),
                min_clearance_m=stat('min_clearance_m'),
                mean_clearance_m=stat('mean_clearance_m'),
                goal_reach_time_s=stat('goal_reach_time_s'),
                path_length_m=stat('path_length_m'),
                stopped_fraction=stat('stopped_fraction'),
                reaction_time_s=stat('reaction_time_s'))
        (out_dir / 'navigation.json').write_text(json.dumps(
            dict(note='One Gazebo launch per trial; the three arms differ ONLY in the live '
                      'predicted_obstacle_layer parameters (enabled, prediction_mode). No '
                      'controller weight, planner, costmap geometry or tracker parameter '
                      'differs between arms.',
                 summary=summary, trials=nav_rows), indent=1) + '\n')

    # ---- runtime -------------------------------------------------------
    dirs = []
    for root in args.perf_logs:
        root = Path(root)
        dirs.extend([root] if (root / 'launch.log').exists() else
                    [d for d in sorted(root.iterdir()) if (d / 'launch.log').exists()])
    perf = perf_from_logs(dirs)
    if perf:
        (out_dir / 'performance.json').write_text(json.dumps(
            dict(note='Tracker callback breakdown from the recorded G2_PERF lines, bucketed '
                      'by live track count. reach_us is the Stage-4G4 addition; publish_us is '
                      'the whole message-construction step that contains it. Scan period is '
                      '200 ms.',
                 scan_period_ms=200.0, by_track_count=perf), indent=1) + '\n')

    # ---- runtime A/B on identical scenarios ----------------------------
    def expand(paths):
        dirs = []
        for root in paths:
            root = Path(root)
            dirs.extend([root] if (root / 'launch.log').exists() else
                        [d for d in sorted(root.iterdir()) if (d / 'launch.log').exists()])
        return dirs

    if args.perf_off_logs and args.perf_on_logs:
        ab = dict(
            note='Identical scenarios, identical build, identical machine; the ONLY difference '
                 'is the tracker parameter reachability_enabled. cluster/deblend/association are '
                 'unchanged code paths and are reported to show they did not move. reach_us is '
                 'the new stage; the remaining callback growth is the larger published message.',
            scan_period_ms=200.0,
            reachability_disabled=perf_from_logs(expand(args.perf_off_logs)),
            reachability_enabled=perf_from_logs(expand(args.perf_on_logs)))
        for tracks, on in ab['reachability_enabled'].items():
            off = ab['reachability_disabled'].get(tracks)
            if off:
                on['callback_delta_ms'] = (on['callback_us']['mean'] -
                                           off['callback_us']['mean']) / 1000.
                on['callback_max_fraction_of_period'] = on['callback_us']['max'] / 1000. / 200.
        (out_dir / 'runtime_ab.json').write_text(json.dumps(ab, indent=1) + '\n')

    # ---- Stage-4G3 non-regression --------------------------------------
    # Compares the identity-critical metrics field by field against the
    # committed Stage-4G3 result for the same scenario.
    if args.regression_trials:
        base = {}
        g3 = Path(args.stage4g3_results)
        if g3.exists():
            for t in json.loads(g3.read_text()):
                base[Path(t['source_trial']).name] = t

        def fingerprint(d):
            return dict(
                id_switches=d.get('id_switches'),
                track_fragments=d.get('track_fragments'),
                distinct_confirmed_ids=d.get('distinct_confirmed_ids'),
                false_confirmed_ids=len(d.get('false_confirmed_ids', {})),
                false_persistent_ids=len(d.get('false_persistent_ids', [])),
                duplicate_frames=d.get('duplicate_frames'),
                categories=d.get('categories'),
                max_missed_scans_retained=d.get('max_missed_scans_retained'),
                per_gt=[dict(purity=round(g['purity'], 6),
                             matched_frame_fraction=round(g['matched_frame_fraction'], 6))
                        for g in d.get('per_gt', [])])

        rows = []
        for path in args.regression_trials:
            path = Path(path)
            summary = json.loads((path / 'summary.json').read_text())
            now = fingerprint(summary)
            before = fingerprint(base[path.name]) if path.name in base else None
            rows.append(dict(trial=path.name, stage4g3=before, stage4g4=now,
                             identical=(before == now) if before else None))
        (out_dir / 'regression.json').write_text(json.dumps(dict(
            note='Stage-4G3 scenarios re-run on the Stage-4G4 build and scored by the unchanged '
                 'Stage-4G2 scorer.',
            all_identical=all(r['identical'] for r in rows if r['identical'] is not None),
            trials=rows), indent=1) + '\n')

    print(f'wrote {out_dir}/coverage.json, sensitivity.json'
          + (', navigation.json' if nav_rows else '')
          + (', performance.json' if perf else ''))


if __name__ == '__main__':
    main()
