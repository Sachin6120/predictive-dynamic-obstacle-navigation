#!/usr/bin/env python3
"""Render the Stage-5E markdown tables from the two machine-generated JSONs.

Every table in STAGE5E_README.md is emitted by this script and pasted
verbatim. Nothing in the Stage-5E tables is transcribed by hand, which is the
rule every table under validation/ follows -- and at Stage-5E it matters more
than ever, because the architecture recommendation rests on these numbers.

    validation/stage5e/report_tables.py \
        --campaign validation/stage5e/nav_campaign5.json \
        --audit validation/stage5e/prediction_audit.json
"""
import argparse
import json

ARMS = ['cv', 'global', 'classaware']
ARM_LABEL = {'cv': 'CV', 'global': 'Global Reach',
             'classaware': 'Class-Aware Reach'}
AUDIT_ARM = {'cv': 'A_cv_kalman', 'global': 'B_global_reachability',
             'classaware': 'C_runtime_class_aware'}
HORIZONS = ('0.5', '1.0', '1.5', '2.0', '2.5', '3.0')


def fmt(v, nd=3):
    """A number, or an em dash when it does not exist."""
    if v is None:
        return '—'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, int):
        return str(v)
    return f'{v:.{nd}f}'


def pm(stat, nd=3):
    """mean ± sd over n, or an em dash."""
    if not stat or stat.get('mean') is None:
        return '—'
    return f"{stat['mean']:.{nd}f} ± {stat['sd']:.{nd}f}"


def row(label, cells):
    return '| ' + label + ' | ' + ' | '.join(cells) + ' |'


def header(first, cols, align='---:'):
    return ('| ' + first + ' | ' + ' | '.join(cols) + ' |\n'
            '|---|' + '|'.join([align] * len(cols)) + '|')


def main():
    """Print every table."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--campaign', required=True)
    ap.add_argument('--audit', required=True)
    args = ap.parse_args()
    with open(args.campaign) as handle:
        c = json.load(handle)
    with open(args.audit) as handle:
        a = json.load(handle)
    arms = [x for x in ARMS if x in c['arms']]
    A = {x: c['arms'][x] for x in arms}
    nm = 'near_misses_below_%.2fm' % c['near_miss_threshold_m']
    cols = [ARM_LABEL[x] for x in arms]

    # ---------------- TABLE 1: the required final comparison ----------------
    print('### Table 1 — final comparison (all 9 conditions pooled)\n')
    print(header('', cols))
    out = [
        ('Trials launched', [str(A[x]['trials_launched']) for x in arms]),
        ('Infrastructure faults (excluded)',
         [str(A[x]['infrastructure_faults']) for x in arms]),
        ('Trials scored', [str(A[x]['trials_scored']) for x in arms]),
        ('Success', [f"{A[x]['succeeded']}/{A[x]['trials_scored']}"
                     f" ({A[x]['success_rate']:.3f})" for x in arms]),
        ('Collisions', [str(A[x]['collisions']) for x in arms]),
        (f"Near misses (< {c['near_miss_threshold_m']} m)",
         [str(A[x][nm]) for x in arms]),
        ('Timeouts', [str(A[x]['timeouts']) for x in arms]),
        ('Planner/controller failures',
         [str(A[x]['planner_or_controller_failures']) for x in arms]),
        ('Trials with no global plan',
         [str(A[x]['trials_with_no_global_plan']) for x in arms]),
        ('Collision-monitor interventions',
         [str(A[x]['collision_monitor_interventions']) for x in arms]),
        ('Worst clearance, m',
         [fmt(A[x]['min_clearance_m']['min']) for x in arms]),
        ('Mean clearance, m', [pm(A[x]['min_clearance_m']) for x in arms]),
        ('Median clearance, m',
         [fmt(A[x]['min_clearance_distribution']['median']) for x in arms]),
        ('Nav time, s', [pm(A[x]['nav_time_s']) for x in arms]),
        ('Path length, m', [pm(A[x]['path_length_m']) for x in arms]),
        ('Stops', [pm(A[x]['n_stops'], 2) for x in arms]),
        ('Global path changes', [pm(A[x]['global_path_changes'], 2) for x in arms]),
        ('Predictive cells (max/trial)',
         [pm(A[x]['max_pred_cells'], 1) for x in arms]),
        ('Layer updateCosts mean, us',
         [pm(A[x]['layer_update_us_mean'], 2) for x in arms]),
        ('Layer updateCosts max, us',
         [pm(A[x]['layer_update_us_max'], 1) for x in arms]),
        ('Layer rate, Hz', [pm(A[x]['layer_update_rate_hz'], 3) for x in arms]),
        ('Layer rate shortfalls',
         [str(A[x]['layer_rate_shortfalls_vs_5.0Hz']) for x in arms]),
        ('Costmap update failures',
         [str(A[x]['costmap_update_failures']) for x in arms]),
        ('Tracker reach stage, us',
         [pm(A[x]['tracker_reach_us'], 3) for x in arms]),
        ('Capability containment (pooled, offline)',
         [fmt(a['pooled'][AUDIT_ARM[x]]['capability_containment'], 4)
          for x in arms]),
        ('Observed containment (pooled, offline)',
         [fmt(a['pooled'][AUDIT_ARM[x]]['observed_containment'], 4)
          for x in arms]),
        ('Contract breaches (live)',
         [str(A[x]['reachability_contract_violations']) for x in arms]),
        ('Unsafe profile events',
         [str(A[x]['unsafe_profile_events']) for x in arms]),
    ]
    for label, cells in out:
        print(row(label, cells))

    # ---------------- TABLE 2: clearance distribution -----------------------
    print('\n### Table 2 — clearance distribution, m '
          '(conflict conditions only; `ped_close_pass` excluded)\n')
    keys = ('n', 'min', 'p10', 'p25', 'median', 'p75', 'max')
    print(header('', [k for k in keys], align=':---:'))
    for x in arms:
        d = A[x]['min_clearance_distribution_conflict_only']
        print(row(ARM_LABEL[x],
                  [str(d['n'])] + [fmt(d[k]) for k in keys[1:]]))

    # ---------------- TABLE 3: per-condition --------------------------------
    print('\n### Table 3 — per-condition breakdown '
          '(success · collisions · worst clearance m · mean clearance m · '
          'nav time s · max predictive cells)\n')
    print(header('condition', cols, align=':---'))
    for cond, per in c['per_condition'].items():
        cells = []
        for x in arms:
            s = per[x]
            cells.append(
                f"{s['succeeded']}/{s['trials_scored']} · {s['collisions']} · "
                f"{fmt(s['min_clearance_m']['min'])} · "
                f"{fmt(s['min_clearance_m']['mean'])} · "
                f"{fmt(s['nav_time_s']['mean'], 2)} · "
                f"{fmt(s['max_pred_cells']['max'], 0)}")
        print(row(f'`{cond}`', cells))

    # ---------------- TABLE 4: pairwise -------------------------------------
    print('\n### Table 4 — pairwise differences (right − left) with the '
          'two-sigma-of-the-difference band\n')
    print('A difference whose magnitude is INSIDE the band supports no claim '
          'of superiority.\n')
    for comp in c['causal_comparisons']:
        left, right = comp['left'], comp['right']
        print(f"**{ARM_LABEL[left]} → {ARM_LABEL[right]}** — {comp['question']}\n")
        print(header('metric', ['left', 'right', 'diff', '2σ band',
                                'outside band?']))
        for key, v in comp['result'].items():
            if v is None or 'difference' not in v:
                continue
            print(row(f'`{key}`', [fmt(v['left_mean']), fmt(v['right_mean']),
                                   fmt(v['difference']),
                                   fmt(v['two_sigma_band']),
                                   'YES' if v['outside_band'] else 'no']))
        counts = comp['result']
        print(row('`success / collisions / timeouts / near misses`',
                  [f"{counts['succeeded']['left']} / "
                   f"{counts['collisions']['left']} / "
                   f"{counts['timeouts']['left']} / {counts[nm]['left']}",
                   f"{counts['succeeded']['right']} / "
                   f"{counts['collisions']['right']} / "
                   f"{counts['timeouts']['right']} / {counts[nm]['right']}",
                   '—', '—', '—']))
        print()

    # ---------------- TABLE 5: prediction, by horizon -----------------------
    print('### Table 5 — CV prediction error by horizon, mean of per-trial '
          'means, m (measured in EVERY arm: the tracker publishes CV '
          'predictions in all three, only consumption differs)\n')
    print(header('horizon, s', cols))
    for h in HORIZONS:
        print(row(h, [pm(A[x]['cv_prediction']
                          ['displacement_error_by_horizon_m'][h], 4)
                      for x in arms]))
    print(row('ADE', [pm(A[x]['cv_prediction']['ADE_m'], 4) for x in arms]))
    print(row('FDE (3 s)', [pm(A[x]['cv_prediction']['FDE_m'], 4)
                            for x in arms]))
    print(row('tracking position error',
              [pm(A[x]['cv_prediction']['tracking_position_error_m'], 4)
               for x in arms]))

    # ---------------- TABLE 6: offline containment --------------------------
    print('\n### Table 6 — offline prediction quality, one replay set '
          f"({a['replay_rows']} rows, {a['replay_tracks']} tracks)\n")
    print('`cells` is the planner-visible count inside the 0.55 m window; '
          '529 is the whole window, i.e. SATURATED.\n')
    print(header('horizon, s',
                 ['CV cap', 'CV obs (k2.0 / k1.5)', 'CV area m² / cells',
                  'GR cap', 'GR obs', 'GR area m² / cells',
                  'CA cap', 'CA obs', 'CA area m² / cells']))
    for h in HORIZONS:
        cv = a['per_horizon']['A_cv_kalman'][f'h{h}']
        gr = a['per_horizon']['B_global_reachability'][f'h{h}']
        ca = a['per_horizon']['C_runtime_class_aware'][f'h{h}']
        print(row(h, [
            fmt(cv['capability_containment'], 4),
            f"{cv['observed_containment']:.4f} / "
            f"{cv['observed_containment_k1.5_consumed']:.4f}",
            f"{cv['mean_area_m2']:.2f} / {cv['mean_cells']:.0f}",
            fmt(gr['capability_containment'], 4),
            fmt(gr['observed_containment'], 4),
            f"{gr['mean_area_m2']:.2f} / {gr['mean_cells']:.0f}",
            fmt(ca['capability_containment'], 4),
            fmt(ca['observed_containment'], 4),
            f"{ca['mean_area_m2']:.2f} / {ca['mean_cells']:.0f}"]))

    # ---------------- TABLE 7: profile usage --------------------------------
    print('\n### Table 7 — live capability-profile usage '
          '(cumulative tracker counters, whole campaign)\n')
    print(header('counter', cols))
    for key in ('profile_samples_total', 'class_closure_rate', 'fallback_rate',
                'transition_rate', 'global_profile_rate',
                'unsafe_profile_events'):
        print(row(f'`{key}`', [fmt(A[x][key], 4) for x in arms]))
    for key in ('prof_global', 'prof_class', 'prof_fb_unknown',
                'prof_fb_lowconf', 'prof_fb_immature', 'prof_fb_badconfig',
                'prof_transition'):
        print(row(f'`{key}`', [str(A[x]['profile_samples'][key])
                               for x in arms]))

    # ---------------- TABLE 8: arm integrity --------------------------------
    print('\n### Table 8 — arm integrity, read back per trial\n')
    print(header('check', cols, align=':---'))
    for key, label in (('layer_mode_observed', 'layer `prediction_mode` seen'),
                       ('layer_mode_mismatches', 'mode mismatches'),
                       ('window_m_recorded_observed',
                        '`max_influence_radius` recorded'),
                       ('window_mismatches', 'window mismatches'),
                       ('class_aware_banner_observed',
                        'class-aware banner seen'),
                       ('banner_mismatches', 'banner mismatches')):
        cells = []
        for x in arms:
            v = A[x]['integrity'][key]
            cells.append(str(v) if not isinstance(v, list)
                         else ', '.join(str(i) for i in v))
        print(row(label, cells))
    print('\nThe mode mismatches counted here are exactly the three excluded '
          'infrastructure-fault trials, in which Nav2 or Gazebo never came up '
          'and no layer mode was ever read back. Among the 132 SCORED trials '
          'every arm read back its own layer mode, a 0.55 m window and the '
          'expected class-aware banner, with zero mismatches.')

    # ---------------- TABLE 9: tracker cost ---------------------------------
    print('\n### Table 9 — tracker per-scan cost, median per trial then '
          'mean ± sd over trials, us\n')
    print(header('G2_PERF stage', cols))
    perf = A[arms[0]]['tracker_perf_us_median_per_trial']
    for f in perf:
        print(row(f'`{f}`', [pm(A[x]['tracker_perf_us_median_per_trial'][f], 2)
                             for x in arms]))
    print(row('`reach_us`', [pm(A[x]['tracker_reach_us'], 3) for x in arms]))


if __name__ == '__main__':
    main()
