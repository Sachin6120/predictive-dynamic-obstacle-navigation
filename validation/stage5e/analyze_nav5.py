#!/usr/bin/env python3
"""Stage-5E final three-arm navigation summary: CV vs global vs class-aware.

Reads the per-trial JSON files validation/stage5e/run_nav5.py produced and
emits one machine-generated summary. Nothing in the Stage-5E tables is
transcribed by hand, which is the rule every table under validation/ follows.

REUSES THE STAGE-5D PARSER. `parse_log`, `PROFILE_KEYS` and `NEAR_MISS_M` are
IMPORTED from validation/stage5d/analyze_nav.py rather than restated, so the
counters, the near-miss threshold and the log grammar cannot drift between
Stage-5D, Stage-5D.1 and Stage-5E. The condition->class map is EXTENDED from
Stage-5D's with the three added conditions rather than rewritten.

WHAT IS ADDED HERE, AND ONLY HERE
---------------------------------
  * the three-arm axis, with per-trial ARM INTEGRITY cross-checks: the layer
    mode, the influence window and the class-aware banner each trial actually
    ran with are read back out of the trial's own record and its launch log and
    checked against what the arm claims. A mislabelled arm becomes an error
    rather than a silent result;
  * the CLEARANCE DISTRIBUTION, as order statistics (min, p10, p25, median,
    p75, max) rather than a mean alone -- the mean of a bimodal
    yield/detour distribution is not a safety statement;
  * the CV PREDICTION-QUALITY block, read from each trial's own
    `prediction_quality` metrics. The tracker publishes CV predictions in ALL
    THREE arms (only consumption differs), so prediction error by horizon is
    measured per arm and its agreement across arms is itself a check that the
    arms really did share one tracker input;
  * the three pairwise comparisons, each printed as the difference together
    with a crude two-sigma-of-the-difference band. NO SIGNIFICANCE CLAIM is
    made for a difference inside that band;
  * FAILURE CLASSIFICATION into the Stage-5E section 9 taxonomy, mapped from
    the trial scorer's own evidenced cause label. `unclassified` is carried
    through as OTHER rather than guessed at.

UNCERTAINTY IS REPORTED, NOT ASSUMED AWAY. Five repetitions per condition per
arm resolves a large effect and not a small one, so every continuous metric
carries n, mean, sd and the standard error of the mean.

LAYER p95/p99 IS NOT AVAILABLE. The layer's stats line instruments mean and
max only (predicted_obstacle_layer.cpp), so this summary reports mean and max
and says so, rather than inventing a percentile.

    validation/stage5e/analyze_nav5.py --trials DIR --out summary.json
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
PROFILE_KEYS = S5D.PROFILE_KEYS
NEAR_MISS_M = S5D.NEAR_MISS_M

#: Stage-5D's condition->true-class map, EXTENDED with the three conditions
#: Stage-5E adds. Every added condition uses the pedestrian actor.
CONDITION_CLASS = dict(S5D.CONDITION_CLASS)
CONDITION_CLASS.update({
    'ped_oblique': 'PEDESTRIAN',
    'ped_close_pass': 'PEDESTRIAN',
    'ped_corner': 'PEDESTRIAN',
})

#: Conditions in which the geometry does NOT force the robot to act. Reported
#: separately because a predictor that paints cost here is paying a price for
#: nothing, and pooling them with the conflict conditions hides that.
NO_CONFLICT_CONDITIONS = ('ped_close_pass',)

#: arm -> (expected layer prediction_mode, expected class-aware banner,
#:         reachability published?). Mirrors ARMS in run_nav5.py.
ARM_SPEC = {
    'cv': ('cv_covariance', False, False),
    'global': ('reachability', False, True),
    'classaware': ('reachability', True, True),
}
ARM_ORDER = ['cv', 'global', 'classaware']

#: The production influence window every arm ran at.
WINDOW_M = 0.55

#: Local costmap update_frequency, nav2_stage4f_params.yaml:250. The layer's
#: stats line reports the rate it actually achieved, so a sustained shortfall
#: is a missed-deadline signal.
UPDATE_FREQUENCY_HZ = 5.0
RATE_TOLERANCE_HZ = 0.25

#: The trial scorer's own evidenced cause label -> the Stage-5E section 9
#: taxonomy. `unclassified` maps to OTHER rather than being guessed at, and
#: `startup_or_simulation_failure` maps to SIM_INFRASTRUCTURE, which the
#: project convention already excludes from an arm's score.
CAUSE_TAXONOMY = {
    'startup_or_simulation_failure': 'SIM_INFRASTRUCTURE',
    'tracking_failure': 'TRACKING_FAILURE',
    'costmap_representation_failure': 'COSTMAP_UNDERREPRESENTED',
    'prediction_error': 'PREDICTION_FAILURE',
    'collision_monitor_intervention': 'PLANNER_FIELD_COLLAPSE',
    'planner_failure': 'PLANNER_FIELD_COLLAPSE',
    'controller_decision_failure': 'COSTMAP_OVERCONSERVATIVE',
    'unclassified': 'OTHER',
}

#: Horizons the tracker publishes, s.
HORIZONS = ('0.5', '1.0', '1.5', '2.0', '2.5', '3.0')


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


def quantile(sorted_vals, q):
    """Linear-interpolated quantile of an already-sorted list."""
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return round(sorted_vals[0], 4)
    pos = q * (len(sorted_vals) - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = pos - lo
    return round(sorted_vals[lo] + frac * (sorted_vals[hi] - sorted_vals[lo]), 4)


def distribution(values):
    """Order statistics of a clearance sample: the shape, not just the mean."""
    vals = sorted(v for v in values if v is not None)
    if not vals:
        return {'n': 0}
    return {'n': len(vals), 'min': round(vals[0], 4),
            'p10': quantile(vals, 0.10), 'p25': quantile(vals, 0.25),
            'median': quantile(vals, 0.50), 'p75': quantile(vals, 0.75),
            'max': round(vals[-1], 4)}


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


#: The G2_PERF per-scan timing fields the tracker prints. Stage-5D's parser
#: extracts `reach_us` only; the WHOLE tracker callback matters for Stage-5E's
#: computation comparison, because the question is what an arm costs in total,
#: not only what its own stage costs. Mean over the per-scan lines, us.
PERF_FIELDS = ('scan_us', 'cluster_us', 'deblend_us', 'compound_us',
               'association_us', 'publish_us', 'callback_us')


def tracker_perf(path):
    """MEDIAN of each G2_PERF timing field over one launch log, microseconds.

    The G2_PERF line is printed per scan and its timing fields are INSTANT
    values (the cumulative counters on the same line are the prof_* and
    contract ones Stage-5D's parser reads).

    THE MEDIAN, NOT THE MEAN, and deliberately: `scan_us` and `callback_us`
    are strongly BIMODAL in a headless run co-scheduled with Gazebo, Nav2 and
    MPPI -- within one trial they take either a few hundred microseconds or
    about ten milliseconds, depending on what else held the CPU -- so their
    mean is a statement about simulator contention rather than about
    computation, and it moves between arms for reasons that have nothing to do
    with the predictor. The median is the robust summary and is reported for
    every field so one definition covers the whole row. The ARM-ATTRIBUTABLE
    stages (`reach_us`, read by Stage-5D's parser, and `publish_us`) are
    stable under either statistic, which is why the comparison rests on them.
    """
    out = {f: None for f in PERF_FIELDS}
    if not os.path.exists(path):
        return out
    acc = {f: [] for f in PERF_FIELDS}
    with open(path, errors='ignore') as handle:
        for line in handle:
            if 'G2_PERF' not in line:
                continue
            for f in PERF_FIELDS:
                m = re.search(f + r'=([0-9.]+)', line)
                if m:
                    acc[f].append(float(m.group(1)))
    for f in PERF_FIELDS:
        if acc[f]:
            vals = sorted(acc[f])
            out[f] = round(vals[len(vals) // 2], 3)
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
        if cond not in CONDITION_CLASS or arm not in ARM_SPEC:
            continue
        with open(path) as handle:
            result = json.load(handle)
        met = result.get('metrics', {})
        cfg = result.get('config', {})
        log_path = os.path.join(trials_dir, 'logs', name + '.log')
        log = parse_log(log_path)
        want_mode, want_banner, publishes = ARM_SPEC[arm]
        # ARM INTEGRITY. The window, the layer mode and the class-aware banner
        # are read back out of what the trial RECORDED, not out of what the
        # runner intended, so a mislabelled arm is an error, not a result.
        window_seen = cfg.get('policy_max_influence_radius')
        window_ok = (window_seen is not None and
                     abs(float(window_seen) - WINDOW_M) < 1e-9)
        mode_ok = log['layer_mode'] == want_mode
        banner_ok = bool(log['class_aware_banner']) == want_banner
        clr = met.get('min_clearance')
        failure = met.get('failure') or {}
        cause = failure.get('cause')
        infra = met.get('nav_status') is None
        rate = layer_rate(log_path)
        perf = tracker_perf(log_path)
        pq = met.get('prediction_quality') or {}
        dbh = pq.get('displacement_error_by_horizon_m') or {}
        rows.append({
            'trial': name,
            'condition': cond,
            'arm': arm,
            'index': int(idx),
            'gt_class': CONDITION_CLASS[cond],
            'no_conflict': cond in NO_CONFLICT_CONDITIONS,
            # --- arm integrity -------------------------------------------
            'layer_mode': log['layer_mode'],
            'layer_mode_matches_arm': mode_ok,
            'window_m_recorded': (float(window_seen)
                                  if window_seen is not None else None),
            'window_matches_production': window_ok,
            'class_aware_banner': bool(log['class_aware_banner']),
            'banner_matches_arm': banner_ok,
            'reachability_published_expected': publishes,
            # --- navigation ----------------------------------------------
            'infrastructure_fault': infra,
            'success': bool(met.get('success')),
            'nav_status': met.get('nav_status'),
            'timeout': met.get('nav_status') == 'TIMEOUT',
            'collision': bool(met.get('collision')),
            'near_miss': bool(clr is not None and 0.0 < clr < NEAR_MISS_M),
            'min_clearance_m': clr,
            'nav_time_s': met.get('nav_time'),
            'path_length_m': met.get('path_length'),
            'primary_reaction_time_s': met.get('primary_reaction_time'),
            'response_class': met.get('response_class'),
            'collision_monitor_interventions':
                met.get('collision_monitor_interventions'),
            'plan_publications': met.get('plan_publications'),
            'global_path_changes': met.get('global_path_changes'),
            'n_stops': met.get('n_stops'),
            'stop_duration_total_s': met.get('stop_duration_total'),
            'max_abs_lateral_offset_approach_m':
                met.get('max_abs_lateral_offset_approach'),
            'frames_tracked': met.get('frames_tracked'),
            'max_tracks': met.get('max_tracks'),
            'tracked_speed_max': met.get('tracked_speed_max'),
            'failure_cause': cause,
            'failure_class': CAUSE_TAXONOMY.get(cause) if cause else None,
            'failure_evidence': failure.get('evidence'),
            # --- costmap / computation ------------------------------------
            'max_pred_cells': met.get('max_pred_cells'),
            'mean_pred_frac_window': met.get('mean_pred_frac_window'),
            'max_pred_frac_window': met.get('max_pred_frac_window'),
            'layer_update_us_mean': log['layer_update_us_mean'],
            'layer_update_us_max': log['layer_update_us_max'],
            'layer_update_rate_hz': rate,
            'layer_rate_shortfall':
                bool(rate is not None and
                     rate < UPDATE_FREQUENCY_HZ - RATE_TOLERANCE_HZ),
            'tracker_reach_us_mean': log['reach_us_mean'],
            'tracker_perf_us': perf,
            # --- prediction ------------------------------------------------
            'n_prediction_sets': pq.get('n_prediction_sets'),
            'tracking_position_error_m': (pq.get('tracking_position_error_m')
                                          or {}).get('mean'),
            'ADE_m': (pq.get('ADE_m') or {}).get('mean'),
            'FDE_m': (pq.get('FDE_m') or {}).get('mean'),
            'displacement_error_by_horizon_m': {
                h: (dbh.get(h) or {}).get('mean') for h in HORIZONS},
            # --- reachability contract / profiles --------------------------
            'reach_contract_violations': log['reach_contract_violations'],
            'reach_contract_worst_speed': log['reach_contract_worst_speed'],
            'profile_samples': log['profile_samples'],
            'profile_samples_total': log['profile_samples_total'],
        })
    return rows


def aggregate(rows):
    """Per-arm aggregate over a list of trial records."""
    # INFRASTRUCTURE FAULT vs NAVIGATION FAILURE. A trial whose simulator or
    # robot never came up produced no navigation attempt, so there is nothing
    # to score; counting it as a planner failure would attribute a tooling
    # fault to the arm under test. Excluded here and reported on its own line,
    # the same distinction run_stage4f_bench.py already prints.
    scored = [r for r in rows if not r['infrastructure_fault']]
    prof = {k: 0 for k in PROFILE_KEYS}
    for r in rows:
        for k in PROFILE_KEYS:
            prof[k] += r['profile_samples'][k]
    total = sum(prof.values())
    fallback = (prof['prof_fb_unknown'] + prof['prof_fb_lowconf'] +
                prof['prof_fb_immature'] + prof['prof_fb_badconfig'])
    conflict = [r for r in scored if not r['no_conflict']]
    failures = {}
    for r in scored:
        if r['failure_class']:
            failures.setdefault(r['failure_class'], []).append(r['trial'])
    return {
        'trials_launched': len(rows),
        'infrastructure_faults': sum(1 for r in rows if r['infrastructure_fault']),
        'infrastructure_fault_trials': [r['trial'] for r in rows
                                        if r['infrastructure_fault']],
        'trials_scored': len(scored),
        'succeeded': sum(1 for r in scored if r['success']),
        'success_rate': (round(sum(1 for r in scored if r['success']) / len(scored), 4)
                         if scored else None),
        'collisions': sum(1 for r in scored if r['collision']),
        'near_misses_below_%.2fm' % NEAR_MISS_M:
            sum(1 for r in scored if r['near_miss']),
        'timeouts': sum(1 for r in scored if r['timeout']),
        'collision_monitor_interventions':
            sum(r['collision_monitor_interventions'] or 0 for r in scored),
        'planner_or_controller_failures':
            sum(1 for r in scored if not r['success']),
        'trials_with_no_global_plan':
            sum(1 for r in scored if r['plan_publications'] == 0),
        'nav_statuses': sorted({str(r['nav_status']) for r in scored}),
        'response_classes': sorted({str(r['response_class']) for r in scored}),
        'failure_classes': {k: sorted(v) for k, v in sorted(failures.items())},
        # --- ARM INTEGRITY -------------------------------------------------
        'integrity': {
            'layer_mode_observed': sorted({r['layer_mode'] for r in rows
                                           if r['layer_mode']}),
            'layer_mode_mismatches': sum(1 for r in rows
                                         if not r['layer_mode_matches_arm']),
            'window_m_recorded_observed':
                sorted({r['window_m_recorded'] for r in rows}),
            'window_mismatches': sum(1 for r in rows
                                     if not r['window_matches_production']),
            'class_aware_banner_observed':
                sorted({r['class_aware_banner'] for r in rows}),
            'banner_mismatches': sum(1 for r in rows
                                     if not r['banner_matches_arm']),
        },
        # --- NAVIGATION ----------------------------------------------------
        'min_clearance_m': dstats([r['min_clearance_m'] for r in scored]),
        'min_clearance_distribution':
            distribution([r['min_clearance_m'] for r in scored]),
        'min_clearance_m_conflict_only': dstats(
            [r['min_clearance_m'] for r in conflict]),
        'min_clearance_distribution_conflict_only':
            distribution([r['min_clearance_m'] for r in conflict]),
        'nav_time_s': dstats([r['nav_time_s'] for r in scored]),
        'path_length_m': dstats([r['path_length_m'] for r in scored]),
        'primary_reaction_time_s':
            dstats([r['primary_reaction_time_s'] for r in scored]),
        'n_stops': dstats([r['n_stops'] for r in scored]),
        'stop_duration_total_s': dstats([r['stop_duration_total_s']
                                         for r in scored]),
        'global_path_changes': dstats([r['global_path_changes'] for r in scored]),
        'max_abs_lateral_offset_approach_m':
            dstats([r['max_abs_lateral_offset_approach_m'] for r in scored]),
        'frames_tracked': dstats([r['frames_tracked'] for r in scored]),
        'tracked_speed_max': dstats([r['tracked_speed_max'] for r in scored]),
        # --- COSTMAP / COMPUTATION ----------------------------------------
        'max_pred_cells': dstats([r['max_pred_cells'] for r in scored]),
        'mean_pred_frac_window':
            dstats([r['mean_pred_frac_window'] for r in scored]),
        'max_pred_frac_window':
            dstats([r['max_pred_frac_window'] for r in scored]),
        'layer_update_us_mean': dstats([r['layer_update_us_mean']
                                        for r in scored]),
        'layer_update_us_max': dstats([r['layer_update_us_max'] for r in scored]),
        'layer_update_us_percentiles': 'NOT INSTRUMENTED (mean/max only)',
        'layer_update_rate_hz': dstats([r['layer_update_rate_hz']
                                        for r in scored]),
        'layer_rate_shortfalls_vs_%.1fHz' % UPDATE_FREQUENCY_HZ:
            sum(1 for r in scored if r['layer_rate_shortfall']),
        'costmap_update_failures': sum(1 for r in scored
                                       if r['max_pred_cells'] == 0),
        'tracker_reach_us': dstats([r['tracker_reach_us_mean'] for r in scored]),
        # The WHOLE tracker callback, not only the reachability stage: what an
        # arm costs in total is the deployable question.
        'tracker_perf_us_median_per_trial': {
            f: dstats([r['tracker_perf_us'][f] for r in scored])
            for f in PERF_FIELDS},
        # --- PREDICTION ----------------------------------------------------
        # The tracker publishes CV predictions in ALL arms, so these are
        # measured per arm; their agreement across arms is itself evidence
        # that the arms shared one tracker input.
        'cv_prediction': {
            'n_prediction_sets': dstats([r['n_prediction_sets'] for r in scored]),
            'tracking_position_error_m':
                dstats([r['tracking_position_error_m'] for r in scored]),
            'ADE_m': dstats([r['ADE_m'] for r in scored]),
            'FDE_m': dstats([r['FDE_m'] for r in scored]),
            'displacement_error_by_horizon_m': {
                h: dstats([r['displacement_error_by_horizon_m'][h]
                           for r in scored]) for h in HORIZONS},
        },
        # --- REACHABILITY CONTRACT / PROFILES ------------------------------
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
        # The ONLY profile reason that is a safety defect rather than a policy
        # choice: a profile selected from an invalid configuration.
        'unsafe_profile_events': prof['prof_fb_badconfig'],
    }


#: The three pairwise comparisons Stage-5E section 11 needs, as
#: (left, right, question).
CAUSAL = [
    ('cv', 'global',
     'Does consuming the global reachable set instead of the CV covariance '
     'change navigation behaviour?'),
    ('cv', 'classaware',
     'Does consuming the class-aware reachable set instead of the CV '
     'covariance change navigation behaviour?'),
    ('global', 'classaware',
     'Does semantic class add anything to the reachable set at the planner '
     'level? (Stage-5D asked this; Stage-5E re-asks it on the wider '
     'condition set.)'),
]

#: The metrics the pairwise comparison is drawn on.
COMPARE_METRICS = ('min_clearance_m', 'nav_time_s', 'path_length_m',
                   'max_pred_cells', 'layer_update_us_mean', 'n_stops',
                   'primary_reaction_time_s')


def compare(a, b):
    """Difference b - a on each metric, with a two-sigma-of-difference band.

    The band is sqrt(sem_a^2 + sem_b^2) * 2. It is CRUDE and is labelled as
    such: with five repetitions per condition per arm it is a sanity bound on
    run-to-run noise, not a hypothesis test. A difference whose magnitude is
    inside the band supports NO claim of superiority -- Stage-5E section 6.
    """
    out = {}
    for key in COMPARE_METRICS:
        sa, sb = a[key], b[key]
        if sa['mean'] is None or sb['mean'] is None:
            out[key] = None
            continue
        diff = sb['mean'] - sa['mean']
        band = 2.0 * math.sqrt((sa['sem'] or 0.0) ** 2 + (sb['sem'] or 0.0) ** 2)
        out[key] = {
            'left_mean': sa['mean'], 'right_mean': sb['mean'],
            'difference': round(diff, 4),
            'two_sigma_band': round(band, 4),
            'outside_band': bool(abs(diff) > band),
        }
    for key in ('succeeded', 'collisions', 'timeouts',
                'near_misses_below_%.2fm' % NEAR_MISS_M,
                'planner_or_controller_failures', 'trials_scored'):
        out[key] = {'left': a[key], 'right': b[key]}
    return out


def main():
    """Summarise the campaign."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rows = collect(args.trials)
    if not rows:
        raise SystemExit(f'no trial records under {args.trials}')
    arms = sorted({r['arm'] for r in rows}, key=ARM_ORDER.index)
    conds = sorted({r['condition'] for r in rows},
                   key=lambda c: list(CONDITION_CLASS).index(c))

    per_arm = {a: aggregate([r for r in rows if r['arm'] == a]) for a in arms}
    per_condition = {
        c: {a: aggregate([r for r in rows
                          if r['condition'] == c and r['arm'] == a])
            for a in arms}
        for c in conds}
    per_class = {
        k: {a: aggregate([r for r in rows
                          if r['gt_class'] == k and r['arm'] == a])
            for a in arms}
        for k in sorted({r['gt_class'] for r in rows})}

    summary = {
        'note': (
            'Stage-5E FINAL three-arm bounded navigation campaign: CV-Kalman '
            'vs global reachability vs runtime class-aware reachability, all '
            'at the production max_influence_radius 0.55 m. Exactly one thing '
            'varies: which predictor the costmap layer consumes, and the '
            'tracker publication flag that necessarily accompanies it. The '
            'nav2 params file -- hence MPPI and every critic weight, the '
            'planner, the costmap geometry and resolution -- is the SAME '
            'UNMODIFIED Stage-4F file in all three arms, so MPPI cannot have '
            'been tuned per arm. The three tracker files are pairwise '
            'one-line diffs: reachability_enabled false->true (CV->global) '
            'and reachability_class_aware false->true (global->class-aware). '
            'Map, world, start, goal, actor body, actor trajectory, release '
            'time, sensor model, AMCL, the Kalman filter, the CV predictor, '
            'the Stage-5B classifier, compound clustering, the reachability '
            'mathematics, the Stage-5C.1 contract and the capability-profile '
            'table are shared by construction, and the classifier executes '
            'identically in all three arms. NO ORACLE ARM: it is not a '
            'deployable configuration and is excluded from the architecture '
            'comparison. Ground truth is used by the trial runner for scoring '
            'and for commanding the scripted actor only, and never reaches '
            '/scan, the tracker, the costmap layer, the planner or the '
            'controller. PRODUCTION CONFIGURATION WAS NOT CHANGED: no file '
            'under src/ was written.'),
        'near_miss_threshold_m': NEAR_MISS_M,
        'production_window_m': WINDOW_M,
        'layer_percentiles': ('p95/p99 NOT INSTRUMENTED by the layer; mean '
                              'and max are reported instead.'),
        'two_sigma_band_caveat': (
            'The pairwise bands are 2*sqrt(sem_left^2 + sem_right^2): a crude '
            'run-to-run noise bound at five repetitions per condition per '
            'arm, NOT a hypothesis test. A difference inside the band '
            'supports no claim of superiority.'),
        'arm_spec': {a: {'layer_prediction_mode': ARM_SPEC[a][0],
                         'class_aware': ARM_SPEC[a][1],
                         'reachability_published': ARM_SPEC[a][2],
                         'max_influence_radius_m': WINDOW_M} for a in arms},
        'conditions': {c: {'true_class': CONDITION_CLASS[c],
                           'forces_a_reaction': c not in NO_CONFLICT_CONDITIONS}
                       for c in conds},
        'conditions_not_run': {
            'reversal_or_acceleration_change': (
                'NOT RUN. The shared trial runner drives the scripted '
                'obstacle one-way at constant velocity '
                '(stage4f_trial.py:659). A reversal would require changing '
                'the scorer every earlier stage used. Existing two-arm '
                'evidence for this scenario class: '
                'validation/stage5c/planner_comparison.json '
                '(nav_reversal, CV vs global reachability, 3 trials per arm) '
                'and validation/stage4g8/results.csv (g8_reversal, CV '
                'production, 10 trials).'),
            'multi_obstacle': (
                'NOT RUN. The shared launch spawns exactly ONE scripted '
                'obstacle; adding a second would change the launch for every '
                'earlier stage. Stage-5D declined this condition for the same '
                'reason. Existing single-arm evidence: '
                'validation/stage4g8/results.csv (g8_multi2, g8_multi3, CV '
                'production).'),
        },
        'trials_total': len(rows),
        'arms': per_arm,
        'causal_comparisons': [
            {'left': left, 'right': right, 'question': question,
             'result': compare(per_arm[left], per_arm[right])}
            for left, right, question in CAUSAL
            if left in per_arm and right in per_arm],
        'per_condition': per_condition,
        'per_true_class': per_class,
        'per_trial': rows,
    }
    with open(args.out, 'w') as handle:
        json.dump(summary, handle, indent=1)
        handle.write('\n')
    print(f'{len(rows)} trials -> {args.out}')
    for a in arms:
        s = per_arm[a]
        print(f"  {a:11s} scored={s['trials_scored']:3d} "
              f"success={s['succeeded']:3d} coll={s['collisions']} "
              f"near={s['near_misses_below_%.2fm' % NEAR_MISS_M]} "
              f"timeout={s['timeouts']} "
              f"clr_mean={s['min_clearance_m']['mean']} "
              f"clr_min={s['min_clearance_m']['min']} "
              f"nav={s['nav_time_s']['mean']} "
              f"cells={s['max_pred_cells']['mean']} "
              f"layer_us={s['layer_update_us_mean']['mean']} "
              f"breaches={s['reachability_contract_violations']} "
              f"unsafe={s['unsafe_profile_events']}")


if __name__ == '__main__':
    main()
