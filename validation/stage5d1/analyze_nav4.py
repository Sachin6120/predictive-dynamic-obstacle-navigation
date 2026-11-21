#!/usr/bin/env python3
"""Stage-5D.1 four-arm navigation summary: class source x policy window.

Reads the per-trial JSON files validation/stage5d1/run_nav4.py produced and
emits one machine-generated summary. Nothing in the Stage-5D.1 tables is
transcribed by hand, which is the rule every table under validation/ follows.

REUSES STAGE-5D'S PARSER. `parse_log`, `stats` and `CONDITION_CLASS` are
IMPORTED from validation/stage5d/analyze_nav.py rather than restated, so the
counters, the near-miss threshold and the condition->class map cannot drift
between the two stages. Only the arm axis, the window column, the dispersion
columns and the four causal comparisons are added here.

UNCERTAINTY IS REPORTED, NOT ASSUMED AWAY. Three repetitions per condition is
enough to see a large effect and not enough to resolve a small one, so every
continuous metric carries n, mean, sd and the standard error of the mean, and
the causal comparisons print the difference together with a crude
two-sigma-of-the-difference band. No significance claim is made from a
difference inside that band.

LAYER p95/p99 IS NOT AVAILABLE. The layer's stats line instruments mean and
max only (predicted_obstacle_layer.cpp), so this summary reports mean and max
and says so, rather than inventing a percentile.

    validation/stage5d1/analyze_nav4.py --trials DIR --out summary.json
"""
import argparse
import glob
import importlib.util
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


S5D = _load('s5d_analyze_nav',
            os.path.join(HERE, '..', 'stage5d', 'analyze_nav.py'))
parse_log = S5D.parse_log
CONDITION_CLASS = S5D.CONDITION_CLASS
PROFILE_KEYS = S5D.PROFILE_KEYS
NEAR_MISS_M = S5D.NEAR_MISS_M

#: arm -> (class source, the max_influence_radius it was run at). Mirrors ARMS
#: in run_nav4.py. The window is ALSO read back out of each trial's own config
#: block and cross-checked, so a mislabelled arm shows up as an error rather
#: than being used silently.
ARM_SPEC = {
    'a': ('runtime', 0.55),
    'b': ('oracle', 0.55),
    'c': ('runtime', 3.00),
    'd': ('oracle', 3.00),
}
ARM_ORDER = ['a', 'b', 'c', 'd']

#: Local costmap update_frequency, nav2_stage4f_params.yaml:250. The layer's
#: stats line reports the rate it actually achieved, so a sustained shortfall
#: is a missed-deadline signal.
UPDATE_FREQUENCY_HZ = 5.0
RATE_TOLERANCE_HZ = 0.25


def dstats(values):
    """n / mean / sd / sem / min / max over the non-null values."""
    vals = [v for v in values if v is not None]
    if not vals:
        return {'n': 0, 'mean': None, 'sd': None, 'sem': None,
                'min': None, 'max': None}
    n = len(vals)
    mean = sum(vals) / n
    if n > 1:
        var = sum((v - mean) ** 2 for v in vals) / (n - 1)
        sd = math.sqrt(var)
        sem = sd / math.sqrt(n)
    else:
        sd = 0.0
        sem = 0.0
    return {'n': n, 'mean': round(mean, 4), 'sd': round(sd, 4),
            'sem': round(sem, 4), 'min': round(min(vals), 4),
            'max': round(max(vals), 4)}


def layer_rate(path):
    """The last layer-stats update rate in one launch log, Hz."""
    if not os.path.exists(path):
        return None
    rate = None
    with open(path, errors='ignore') as handle:
        for line in handle:
            if 'PredictedObstacleLayer' in line and 'stats:' in line:
                m = re.search(r'rate=([0-9.]+)Hz', line)
                if m:
                    rate = float(m.group(1))
    return rate


def collect(trials_dir):
    """One record per trial, from its JSON plus its launch log."""
    rows = []
    for path in sorted(glob.glob(os.path.join(trials_dir, '*.json'))):
        name = os.path.basename(path)[:-5]
        parts = name.rsplit('_', 2)
        if len(parts) != 3:
            continue
        cond, arm, idx = parts
        if cond not in CONDITION_CLASS or arm not in ARM_SPEC:
            continue
        with open(path) as handle:
            result = json.load(handle)
        met = result.get('metrics', {})
        cfg = result.get('config', {})
        log_path = os.path.join(trials_dir, 'logs', name + '.log')
        log = parse_log(log_path)
        source, window = ARM_SPEC[arm]
        # CROSS-CHECK: the window the trial actually ran at, read back out of
        # the trial's own recorded config, must be the one the arm claims.
        window_seen = cfg.get('policy_max_influence_radius')
        window_ok = (window_seen is not None and
                     abs(float(window_seen) - window) < 1e-9)
        clr = met.get('min_clearance')
        failure = met.get('failure') or {}
        infra = met.get('nav_status') is None
        rate = layer_rate(log_path)
        rows.append({
            'trial': name,
            'condition': cond,
            'arm': arm,
            'class_source': source,
            'window_m': window,
            'window_m_recorded': (float(window_seen)
                                  if window_seen is not None else None),
            'window_matches_arm': window_ok,
            'index': int(idx),
            'gt_class': CONDITION_CLASS[cond],
            'infrastructure_fault': infra,
            'success': bool(met.get('success')),
            'nav_status': met.get('nav_status'),
            'collision': bool(met.get('collision')),
            'near_miss': bool(clr is not None and 0.0 < clr < NEAR_MISS_M),
            'min_clearance_m': clr,
            'nav_time_s': met.get('nav_time'),
            'path_length_m': met.get('path_length'),
            'max_pred_cells': met.get('max_pred_cells'),
            'mean_pred_frac_window': met.get('mean_pred_frac_window'),
            'max_pred_frac_window': met.get('max_pred_frac_window'),
            'primary_reaction_time_s': met.get('primary_reaction_time'),
            'response_class': met.get('response_class'),
            'collision_monitor_interventions':
                met.get('collision_monitor_interventions'),
            'plan_publications': met.get('plan_publications'),
            'global_path_changes': met.get('global_path_changes'),
            'n_stops': met.get('n_stops'),
            'frames_tracked': met.get('frames_tracked'),
            'max_tracks': met.get('max_tracks'),
            'tracked_speed_max': met.get('tracked_speed_max'),
            'failure_cause': failure.get('cause'),
            'layer_mode': log['layer_mode'],
            'layer_update_us_mean': log['layer_update_us_mean'],
            'layer_update_us_max': log['layer_update_us_max'],
            'layer_update_rate_hz': rate,
            'layer_rate_shortfall':
                bool(rate is not None and
                     rate < UPDATE_FREQUENCY_HZ - RATE_TOLERANCE_HZ),
            'tracker_reach_us_mean': log['reach_us_mean'],
            'reach_contract_violations': log['reach_contract_violations'],
            'reach_contract_worst_speed': log['reach_contract_worst_speed'],
            'class_aware_banner': log['class_aware_banner'],
            'profile_samples': log['profile_samples'],
            'profile_samples_total': log['profile_samples_total'],
        })
    return rows


def aggregate(rows):
    """Per-arm aggregate over a list of trial records."""
    scored = [r for r in rows if not r['infrastructure_fault']]
    prof = {k: 0 for k in PROFILE_KEYS}
    for r in rows:
        for k in PROFILE_KEYS:
            prof[k] += r['profile_samples'][k]
    total = sum(prof.values())
    fallback = (prof['prof_fb_unknown'] + prof['prof_fb_lowconf'] +
                prof['prof_fb_immature'] + prof['prof_fb_badconfig'])
    return {
        'trials_launched': len(rows),
        'infrastructure_faults': sum(1 for r in rows if r['infrastructure_fault']),
        'trials': len(scored),
        'succeeded': sum(1 for r in scored if r['success']),
        'collisions': sum(1 for r in scored if r['collision']),
        'near_misses_below_%.2fm' % NEAR_MISS_M:
            sum(1 for r in scored if r['near_miss']),
        'collision_monitor_interventions':
            sum(r['collision_monitor_interventions'] or 0 for r in scored),
        'planner_or_controller_failures':
            sum(1 for r in scored if not r['success']),
        'failure_causes': sorted({r['failure_cause'] for r in scored
                                  if r['failure_cause']}),
        'nav_statuses': sorted({str(r['nav_status']) for r in scored}),
        'response_classes': sorted({str(r['response_class']) for r in scored}),
        # Arm integrity: every trial must have run at its arm's window, in
        # reachability mode, with the class-aware banner matching the source.
        'window_m_declared': rows[0]['window_m'] if rows else None,
        'window_m_recorded_observed':
            sorted({r['window_m_recorded'] for r in rows}),
        'window_mismatches': sum(1 for r in rows
                                 if not r['window_matches_arm']),
        'layer_mode_observed': sorted({r['layer_mode'] for r in rows
                                       if r['layer_mode']}),
        'class_aware_banner_seen': sorted({r['class_aware_banner']
                                           for r in rows}),
        'min_clearance_m': dstats([r['min_clearance_m'] for r in scored]),
        'nav_time_s': dstats([r['nav_time_s'] for r in scored]),
        'path_length_m': dstats([r['path_length_m'] for r in scored]),
        'max_pred_cells': dstats([r['max_pred_cells'] for r in scored]),
        'mean_pred_frac_window':
            dstats([r['mean_pred_frac_window'] for r in scored]),
        'max_pred_frac_window':
            dstats([r['max_pred_frac_window'] for r in scored]),
        'primary_reaction_time_s':
            dstats([r['primary_reaction_time_s'] for r in scored]),
        'n_stops': dstats([r['n_stops'] for r in scored]),
        'global_path_changes': dstats([r['global_path_changes'] for r in scored]),
        # PERFORMANCE. p95/p99 are NOT instrumented by the layer; mean and max
        # are what it reports, so they are what is reported here.
        'layer_update_us_mean': dstats([r['layer_update_us_mean']
                                        for r in scored]),
        'layer_update_us_max': dstats([r['layer_update_us_max'] for r in scored]),
        'layer_update_us_percentiles': 'NOT INSTRUMENTED (mean/max only)',
        'layer_update_rate_hz': dstats([r['layer_update_rate_hz']
                                        for r in scored]),
        'layer_rate_shortfalls_vs_%.1fHz' % UPDATE_FREQUENCY_HZ:
            sum(1 for r in scored if r['layer_rate_shortfall']),
        'tracker_reach_us': dstats([r['tracker_reach_us_mean'] for r in scored]),
        'reachability_contract_violations':
            sum(r['reach_contract_violations'] for r in rows),
        'reachability_contract_worst_speed':
            dstats([r['reach_contract_worst_speed'] for r in rows])['max'],
        'profile_samples': prof,
        'profile_samples_total': total,
        'fallback_rate': round(fallback / total, 4) if total else None,
        'class_closure_rate':
            round(prof['prof_class'] / total, 4) if total else None,
        'transition_rate':
            round(prof['prof_transition'] / total, 4) if total else None,
        'global_profile_rate':
            round(prof['prof_global'] / total, 4) if total else None,
        'unsafe_profile_events': prof['prof_fb_badconfig'],
    }


#: The four comparisons brief section 10 asks for, as (left, right, question).
CAUSAL = [
    ('a', 'b', 'Does PERFECT CLASSIFICATION help while the 0.55 m window '
               'remains? NO => the classifier is not the primary bottleneck '
               'under current policy.'),
    ('b', 'd', 'Does EXPANDING THE WINDOW help even with perfect labels? '
               'YES => the 0.55 m policy window is suppressing semantic '
               'information.'),
    ('c', 'd', 'With semantic geometry visible, how much is lost to the '
               'RUNTIME classifier / ambiguity policy?'),
    ('a', 'c', 'Does exposing more geometry help the CURRENT RUNTIME system? '
               'This is the practically relevant deployment comparison.'),
]

#: The continuous metrics the causal comparisons are drawn on.
CAUSAL_METRICS = ('min_clearance_m', 'nav_time_s', 'path_length_m',
                  'max_pred_cells', 'mean_pred_frac_window',
                  'layer_update_us_mean')


def compare(left, right, agg):
    """Difference right - left per metric, with a two-sigma band."""
    out = {}
    for m in CAUSAL_METRICS:
        a, b = agg[left][m], agg[right][m]
        if a['mean'] is None or b['mean'] is None:
            out[m] = None
            continue
        diff = b['mean'] - a['mean']
        # Standard error of the difference of two independent means.
        sed = math.sqrt((a['sem'] or 0.0) ** 2 + (b['sem'] or 0.0) ** 2)
        band = 2.0 * sed
        out[m] = {
            'left_mean': a['mean'], 'right_mean': b['mean'],
            'difference': round(diff, 4),
            'two_sigma_band': round(band, 4),
            'resolved_at_2sigma': bool(abs(diff) > band),
            'ratio': (round(b['mean'] / a['mean'], 4)
                      if a['mean'] not in (0, None) else None),
        }
    for k in ('succeeded', 'collisions', 'planner_or_controller_failures',
              'near_misses_below_%.2fm' % NEAR_MISS_M):
        out[k] = {'left': agg[left][k], 'right': agg[right][k]}
    return out


def main():
    """Summarize the campaign."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', required=True, help='campaign output directory')
    ap.add_argument('--out', required=True, help='summary JSON path')
    args = ap.parse_args()

    rows = collect(args.trials)
    if not rows:
        raise SystemExit(f'no trial JSON found under {args.trials}')

    present = {r['arm'] for r in rows}
    arms = [a for a in ARM_ORDER if a in present]
    conds = sorted({r['condition'] for r in rows})
    classes = sorted({r['gt_class'] for r in rows})
    agg = {a: aggregate([r for r in rows if r['arm'] == a]) for a in arms}

    summary = {
        'note':
            'Stage-5D.1 four-arm bounded navigation campaign. Exactly two '
            'factors vary, crossed: class source (RUNTIME classifier + shipped '
            'ambiguity policy vs ORACLE configured-global premise) and '
            'max_influence_radius (0.55 m production vs 3.00 m analysis-only, '
            'the layer\'s own declareParameter default and the half-extent of '
            'the 6x6 m local costmap). The nav2 params file -- hence MPPI and '
            'every critic weight, the planner, the costmap geometry and '
            'resolution -- is the SAME UNMODIFIED Stage-4F file in all four '
            'arms, so MPPI cannot have been tuned per arm. The window is '
            'varied through the layer\'s own documented runtime parameter; no '
            'production YAML is modified. The ORACLE arms are an UPPER-BOUND '
            'MEASUREMENT, not a deployable configuration: the node runs the '
            'bit-identical Stage-5C.1 global path with class_aware FALSE and '
            'has no class input at all, and which premise to configure is '
            'chosen outside the robot by the campaign runner from the scenario '
            'it is spawning. No ground-truth label reaches the tracker, the '
            'costmap layer, the planner or the controller.',
        'near_miss_threshold_m': NEAR_MISS_M,
        'layer_percentiles': 'p95/p99 NOT INSTRUMENTED by the layer; mean and '
                             'max are reported instead.',
        'arm_spec': {a: {'class_source': ARM_SPEC[a][0],
                         'max_influence_radius_m': ARM_SPEC[a][1]}
                     for a in arms},
        'trials_total': len(rows),
        'arms': agg,
        'causal_comparisons': {
            f'{l.upper()}_vs_{r.upper()}': {'question': q,
                                            'metrics': compare(l, r, agg)}
            for l, r, q in CAUSAL if l in agg and r in agg},
        'per_condition': {
            c: {a: aggregate([r for r in rows
                              if r['condition'] == c and r['arm'] == a])
                for a in arms if any(r['condition'] == c and r['arm'] == a
                                     for r in rows)}
            for c in conds},
        'per_true_class': {
            k: {a: aggregate([r for r in rows
                              if r['gt_class'] == k and r['arm'] == a])
                for a in arms if any(r['gt_class'] == k and r['arm'] == a
                                     for r in rows)}
            for k in classes},
        'per_trial': rows,
    }

    with open(args.out, 'w') as handle:
        json.dump(summary, handle, indent=1)

    print(f'{len(rows)} trials, arms={arms}')
    print(f"{'metric':36s}" + ''.join(f'{a.upper():>16s}' for a in arms))
    lines = [
        ('trials', lambda n: n['trials']),
        ('succeeded', lambda n: n['succeeded']),
        ('collisions', lambda n: n['collisions']),
        ('near misses <%.2fm' % NEAR_MISS_M,
         lambda n: n['near_misses_below_%.2fm' % NEAR_MISS_M]),
        ('planner/controller failures',
         lambda n: n['planner_or_controller_failures']),
        ('worst clearance m', lambda n: n['min_clearance_m']['min']),
        ('mean clearance m', lambda n: n['min_clearance_m']['mean']),
        ('nav time s', lambda n: n['nav_time_s']['mean']),
        ('path length m', lambda n: n['path_length_m']['mean']),
        ('peak pred cells', lambda n: n['max_pred_cells']['mean']),
        ('mean pred frac window', lambda n: n['mean_pred_frac_window']['mean']),
        ('layer mean us', lambda n: n['layer_update_us_mean']['mean']),
        ('layer max us', lambda n: n['layer_update_us_max']['max']),
        ('layer rate Hz', lambda n: n['layer_update_rate_hz']['mean']),
        ('rate shortfalls',
         lambda n: n['layer_rate_shortfalls_vs_%.1fHz' % UPDATE_FREQUENCY_HZ]),
        ('fallback rate', lambda n: n['fallback_rate']),
        ('contract violations',
         lambda n: n['reachability_contract_violations']),
        ('window mismatches', lambda n: n['window_mismatches']),
    ]
    for label, fn in lines:
        print(f'{label:36s}' + ''.join(f'{str(fn(agg[a])):>16s}' for a in arms))
    print()
    for key, node in summary['causal_comparisons'].items():
        print(f'--- {key}: {node["question"]}')
        for m, v in node['metrics'].items():
            if isinstance(v, dict) and 'difference' in v:
                flag = 'RESOLVED' if v['resolved_at_2sigma'] else 'within noise'
                print(f'    {m:26s} {v["left_mean"]:>10} -> {v["right_mean"]:>10} '
                      f'diff {v["difference"]:>+10} +-{v["two_sigma_band"]:<9} {flag}')
    print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
