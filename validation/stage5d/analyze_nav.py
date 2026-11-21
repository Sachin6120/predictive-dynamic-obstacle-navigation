#!/usr/bin/env python3
"""
Stage-5D navigation campaign summary: GLOBAL vs RUNTIME CLASS-AWARE.

Reads the per-trial JSON files scripts/stage5d/run_nav.py produced and emits
one machine-generated summary. Nothing in the Stage-5D tables is transcribed
by hand, which is the same rule every table under validation/ follows.

TWO SOURCES, DELIBERATELY KEPT APART
------------------------------------
* the trial JSON -- navigation outcome, clearance against the robot's real
  simulated collision geometry, timing, path length, planner and controller
  failures, and the planner-visible predictive cell counts the trial runner
  sampled from the LIVE costmap;
* the launch LOG -- the tracker's cumulative G2_PERF line and the costmap
  layer's stats line. These carry the Stage-5C.1 contract-breach counters and
  the Stage-5D per-reason profile counters, and they are the only place the
  fallback rate, the UNKNOWN-profile rate and the transition rate of a LIVE
  run can be read, so they are parsed rather than inferred from the offline
  replay.

NEAR MISS is reported at an explicit stated threshold rather than as a word: a
trial whose minimum surface-to-surface clearance fell below NEAR_MISS_M
without reaching zero. The threshold is printed in the output so it cannot be
mistaken for a measured property of the system, and no gate depends on it.

    validation/stage5d/analyze_nav.py --trials DIR --out summary.json
"""
import argparse
import glob
import json
import os
import re

#: Clearance below which a trial is called a near miss, m. The robot's
#: footprint half-width is 0.153 m and Nav2's inflation radius is 0.70 m, so
#: 0.20 m is well inside the band the planner is meant to keep clear while
#: still being a real separation. REPORTING ONLY.
NEAR_MISS_M = 0.20

#: The cumulative per-reason profile counters the tracker prints. Names match
#: the G2_PERF line in lidar_obstacle_tracker_node.cpp, which matches the
#: PROFILE_* constants in ReachabilityPrediction.msg.
PROFILE_KEYS = ('prof_global', 'prof_class', 'prof_fb_unknown',
                'prof_fb_lowconf', 'prof_fb_immature', 'prof_fb_badconfig',
                'prof_transition')

#: condition -> ground-truth class of its obstacle body. Mirrors
#: ACTOR_GT_CLASS in scripts/stage5d/run_nav.py. Used for the per-class
#: breakdown only; a mismatch would show up there rather than being used
#: silently.
CONDITION_CLASS = {
    'ped_cross': 'PEDESTRIAN',
    'ped_longitudinal': 'PEDESTRIAN',
    'bike_cross': 'BIKE',
    'veh_cross': 'VEHICLE',
    'veh_longitudinal': 'VEHICLE',
    'unknown_cross': 'UNKNOWN',
}

NOTE = (
    'Stage-5D bounded navigation campaign. The two arms differ in exactly one '
    'line of one file: tracker_params.yaml vs stage5d_classaware_tracker.yaml, '
    'reachability_class_aware false -> true. The nav2 params file -- hence '
    'MPPI and every critic weight, the planner, the costmap geometry and '
    'resolution, and prediction_mode: reachability -- is the SAME FILE for '
    'both arms, so MPPI cannot have been tuned per arm. Map, world, start, '
    'goal, obstacle body, obstacle trajectory, release time, sensor model, '
    'AMCL, the Kalman filter, the CV predictor, the Stage-5B classifier, '
    'compound clustering, the reachability mathematics and the Stage-5C.1 '
    'contract are shared by construction. Ground truth is used by the trial '
    'runner for scoring and for commanding the scripted obstacle only, and '
    'never reaches /scan, the tracker, the costmap layer, the planner or the '
    'controller.')


def stats(values):
    """n/mean/min/max of a list, rounded, or nulls when it is empty."""
    vals = [v for v in values if v is not None]
    if not vals:
        return {'n': 0, 'mean': None, 'min': None, 'max': None}
    return {'n': len(vals),
            'mean': round(sum(vals) / len(vals), 4),
            'min': round(min(vals), 4),
            'max': round(max(vals), 4)}


def parse_log(path):
    """Tracker and layer counters from one launch log.

    G2_PERF and the layer stats line are both CUMULATIVE over the node's
    lifetime, so the last occurrence of each is the whole-trial total.
    """
    out = {'profile_samples': {k: 0 for k in PROFILE_KEYS},
           'profile_samples_total': 0,
           'reach_contract_violations': 0,
           'reach_contract_worst_speed': 0.0,
           'class_aware_banner': False,
           'layer_mode': None,
           'layer_update_us_mean': None,
           'layer_update_us_max': None,
           'reach_us_mean': None,
           'tracker_class_labels': {}}
    if not os.path.exists(path):
        return out
    reach_us = []
    last_perf = None
    last_layer = None
    with open(path, errors='ignore') as handle:
        for line in handle:
            if 'G2_PERF' in line:
                last_perf = line
                m = re.search(r'reach_us=([0-9.]+)', line)
                if m:
                    reach_us.append(float(m.group(1)))
            elif 'PredictedObstacleLayer' in line and 'stats:' in line:
                last_layer = line
            elif 'Stage-5D class-aware reachability ENABLED' in line:
                out['class_aware_banner'] = True
    if last_perf is not None:
        total = 0
        for key in PROFILE_KEYS:
            m = re.search(key + r'=(\d+)', last_perf)
            if m:
                out['profile_samples'][key] = int(m.group(1))
                total += int(m.group(1))
        out['profile_samples_total'] = total
        m = re.search(r'reach_contract_violations=(\d+)', last_perf)
        if m:
            out['reach_contract_violations'] = int(m.group(1))
        m = re.search(r'reach_contract_worst_speed=([0-9.]+)', last_perf)
        if m:
            out['reach_contract_worst_speed'] = float(m.group(1))
    if last_layer is not None:
        m = re.search(r'updateCosts mean=([0-9.]+)us max=([0-9.]+)us', last_layer)
        if m:
            out['layer_update_us_mean'] = float(m.group(1))
            out['layer_update_us_max'] = float(m.group(2))
        m = re.search(r'mode=(\S+)', last_layer)
        if m:
            out['layer_mode'] = m.group(1)
    if reach_us:
        out['reach_us_mean'] = round(sum(reach_us) / len(reach_us), 3)
    return out


def collect(trials_dir):
    """One record per trial, from its JSON plus its launch log."""
    rows = []
    for path in sorted(glob.glob(os.path.join(trials_dir, '*.json'))):
        name = os.path.basename(path)[:-5]
        parts = name.rsplit('_', 2)
        if len(parts) != 3:
            continue
        cond, arm, idx = parts
        if cond not in CONDITION_CLASS:
            continue
        with open(path) as handle:
            result = json.load(handle)
        met = result.get('metrics', {})
        log = parse_log(os.path.join(trials_dir, 'logs', name + '.log'))
        clr = met.get('min_clearance')
        failure = met.get('failure') or {}
        # INFRASTRUCTURE FAULT vs NAVIGATION FAILURE. A trial whose simulator
        # or robot never came up produced no navigation attempt at all, so
        # nav_status is null and there is nothing to score. Counting it as a
        # planner/controller failure would attribute a tooling fault to the
        # arm under test, so it is separated out and reported on its own
        # line -- the same distinction run_stage4f_bench.py already draws when
        # it prints INFRASTRUCTURE FAULT.
        infra = met.get('nav_status') is None
        rows.append({
            'trial': name,
            'condition': cond,
            'arm': arm,
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
            'primary_reaction_time_s': met.get('primary_reaction_time'),
            'response_class': met.get('response_class'),
            'collision_monitor_interventions':
                met.get('collision_monitor_interventions'),
            'plan_publications': met.get('plan_publications'),
            'global_path_changes': met.get('global_path_changes'),
            'frames_tracked': met.get('frames_tracked'),
            'max_tracks': met.get('max_tracks'),
            'tracked_speed_max': met.get('tracked_speed_max'),
            'failure_cause': failure.get('cause'),
            # --- from the log ---
            'layer_mode': log['layer_mode'],
            'layer_update_us_mean': log['layer_update_us_mean'],
            'layer_update_us_max': log['layer_update_us_max'],
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
        'layer_mode_observed': sorted({r['layer_mode'] for r in rows
                                       if r['layer_mode']}),
        'class_aware_banner_seen': sorted({r['class_aware_banner']
                                           for r in rows}),
        'min_clearance_m': stats([r['min_clearance_m'] for r in scored]),
        'nav_time_s': stats([r['nav_time_s'] for r in scored]),
        'path_length_m': stats([r['path_length_m'] for r in scored]),
        'max_pred_cells': stats([r['max_pred_cells'] for r in scored]),
        'mean_pred_frac_window':
            stats([r['mean_pred_frac_window'] for r in scored]),
        'primary_reaction_time_s':
            stats([r['primary_reaction_time_s'] for r in scored]),
        'layer_updateCosts_us': {
            'mean_of_means': stats(
                [r['layer_update_us_mean'] for r in scored])['mean'],
            'max': stats([r['layer_update_us_max'] for r in scored])['max'],
        },
        'tracker_reach_us': stats([r['tracker_reach_us_mean'] for r in scored]),
        'reachability_contract_violations':
            sum(r['reach_contract_violations'] for r in rows),
        'reachability_contract_worst_speed':
            stats([r['reach_contract_worst_speed'] for r in rows])['max'],
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


def main():
    """Emit the summary."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', required=True, help='campaign output directory')
    ap.add_argument('--out', required=True, help='summary JSON path')
    args = ap.parse_args()

    rows = collect(args.trials)
    if not rows:
        raise SystemExit(f'no trial results found under {args.trials}')

    # Explicit order: the global arm is the control and reads first.
    arm_order = ['global', 'classaware']
    present = {r['arm'] for r in rows}
    arms = ([a for a in arm_order if a in present] +
            sorted(present - set(arm_order)))
    conds = [c for c in CONDITION_CLASS if any(r['condition'] == c for r in rows)]
    classes = sorted({r['gt_class'] for r in rows})

    summary = {
        'note': NOTE,
        'near_miss_threshold_m': NEAR_MISS_M,
        'trials_total': len(rows),
        'arms': {arm: aggregate([r for r in rows if r['arm'] == arm])
                 for arm in arms},
        'per_condition': {
            cond: {arm: aggregate([r for r in rows
                                   if r['condition'] == cond and r['arm'] == arm])
                   for arm in arms}
            for cond in conds},
        'per_true_class': {
            cls: {arm: aggregate([r for r in rows
                                  if r['gt_class'] == cls and r['arm'] == arm])
                  for arm in arms}
            for cls in classes},
        'per_trial': rows,
    }

    with open(args.out, 'w') as handle:
        json.dump(summary, handle, indent=1)

    # Human-readable digest, so a reader does not have to open the JSON.
    key = 'near_misses_below_%.2fm' % NEAR_MISS_M
    print(f'{len(rows)} trials, arms={arms}')
    head = (f"{'metric':34s}" + ''.join(f'{a:>22s}' for a in arms))
    print(head)
    for label, path in (
            ('trials launched', ('trials_launched',)),
            ('infrastructure faults', ('infrastructure_faults',)),
            ('trials scored', ('trials',)),
            ('succeeded', ('succeeded',)),
            ('collisions', ('collisions',)),
            (f'near misses (<{NEAR_MISS_M} m)', (key,)),
            ('planner/controller failures', ('planner_or_controller_failures',)),
            ('min clearance mean (m)', ('min_clearance_m', 'mean')),
            ('min clearance worst (m)', ('min_clearance_m', 'min')),
            ('nav time mean (s)', ('nav_time_s', 'mean')),
            ('path length mean (m)', ('path_length_m', 'mean')),
            ('peak predictive cells mean', ('max_pred_cells', 'mean')),
            ('layer updateCosts mean (us)',
             ('layer_updateCosts_us', 'mean_of_means')),
            ('layer updateCosts max (us)', ('layer_updateCosts_us', 'max')),
            ('tracker reach_us mean', ('tracker_reach_us', 'mean')),
            ('contract violations',
             ('reachability_contract_violations',)),
            ('profile samples', ('profile_samples_total',)),
            ('fallback rate', ('fallback_rate',)),
            ('class-closure rate', ('class_closure_rate',)),
            ('transition rate', ('transition_rate',)),
            ('global-profile rate', ('global_profile_rate',))):
        cells = []
        for arm in arms:
            node = summary['arms'][arm]
            for p in path:
                node = node[p] if node is not None else None
            cells.append('--' if node is None else str(node))
        print(f'{label:34s}' + ''.join(f'{c:>22s}' for c in cells))
    print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
