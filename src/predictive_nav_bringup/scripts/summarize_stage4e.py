#!/usr/bin/env python3
"""Stage-4E summariser: turn per-trial JSON into the final A/B statistics.

Reads every <dir>/*.json trial result (skipping the *_series.json raw traces),
groups by mode, and emits the comparison table plus a machine-readable summary.
Nothing here is transcribed by hand; the table printed to the terminal and the
JSON/CSV written next to it come from the same computation.

  summarize_stage4e.py validation_logs/stage4e/primary \
      --out validation/stage4e_crossing_ab.json --csv validation/stage4e_trials.csv
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as st


# Metrics reported as mean +- sd over trials. (key, label, unit, higher_better)
SCALARS = [
    ('nav_time', 'Navigation time', 's', False),
    ('path_length', 'Path length', 'm', False),
    ('min_clearance', 'Min clearance', 'm', True),
    ('min_center_separation', 'Min centre separation', 'm', True),
    ('primary_reaction_time', 'Reaction time (R1)', 's', None),
    ('n_stops', 'Stops', 'count', None),
    ('stop_duration_total', 'Stop duration', 's', None),
    ('min_v_nav_approach', 'Min commanded v (approach)', 'm/s', None),
    ('mean_v_nav_approach', 'Mean commanded v (approach)', 'm/s', None),
    ('max_abs_w_nav_approach', 'Max |w| (approach)', 'rad/s', None),
    ('max_lateral_dev_approach', 'Max lateral deviation', 'm', None),
    ('cmd_variation', 'Command variation', 'm/s/sample', None),
    ('max_pred_cells', 'Predictive cells (max)', 'cells', None),
    ('max_pred_frac_window', 'Predictive window coverage', 'frac', None),
    ('max_master_hi_ahead', 'Master cells >=180 ahead (max)', 'cells', None),
    ('mean_master_mean_ahead_approach', 'Mean master cost ahead', 'cost', None),
    ('global_path_changes', 'Global route changes', 'count', None),
    ('plan_publications', 'Global plan publications', 'count', None),
    ('collision_monitor_interventions', 'Collision-monitor interventions',
     'count', None),
]


def load(directory):
    trials = []
    for path in sorted(glob.glob(os.path.join(directory, '*.json'))):
        if path.endswith('_series.json') or os.path.basename(path) == 'summary.json':
            continue
        with open(path) as f:
            try:
                trials.append(json.load(f))
            except json.JSONDecodeError:
                print(f'  ! unreadable: {path}')
    return trials


def agg(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return {
        'n': len(vals),
        'mean': round(st.mean(vals), 4),
        'sd': round(st.pstdev(vals), 4) if len(vals) > 1 else 0.0,
        'min': round(min(vals), 4),
        'max': round(max(vals), 4),
    }


def fmt(a, unit=''):
    if a is None:
        return '        --'
    if a['n'] == 1:
        return f"{a['mean']:.3f}"
    return f"{a['mean']:.3f} +- {a['sd']:.3f}"


def welch_t(a, b):
    """Welch's t statistic and dof for two independent samples.

    Reported, with the caveat that repeated runs of one deterministic
    simulator scenario are not independent draws from a population -- they
    measure startup/timing jitter only. It is a description of separation
    between the arms, not a population inference.
    """
    a = [x for x in a if x is not None]
    b = [x for x in b if x is not None]
    if len(a) < 2 or len(b) < 2:
        return None
    va, vb = st.variance(a), st.variance(b)
    na, nb = len(a), len(b)
    se2 = va / na + vb / nb
    if se2 <= 0:
        return {'t': None, 'df': None,
                'note': 'zero variance in both arms (perfectly repeatable)'}
    t = (st.mean(b) - st.mean(a)) / math.sqrt(se2)
    df = se2 ** 2 / (((va / na) ** 2 / (na - 1)) + ((vb / nb) ** 2 / (nb - 1)))
    return {'t': round(t, 3), 'df': round(df, 1)}


def summarize(trials, label):
    out = {'label': label, 'n_trials': len(trials)}
    by_mode = {}
    for mode in ('reactive', 'predictive'):
        sel = [t for t in trials if t.get('mode') == mode]
        m = [t.get('metrics', {}) for t in sel]
        d = {
            'n_trials': len(sel),
            'n_success': sum(1 for x in m if x.get('success')),
            'n_collision': sum(1 for x in m if x.get('collision')),
            'statuses': {},
        }
        for x in m:
            s = x.get('nav_status', 'NO_RESULT')
            d['statuses'][s] = d['statuses'].get(s, 0) + 1
        d['success_rate'] = (round(d['n_success'] / len(sel), 3) if sel else None)
        for key, _lab, _u, _h in SCALARS:
            d[key] = agg([x.get(key) for x in m])
        # advance-warning block lives one level down
        d['advance_lead_s'] = agg(
            [(x.get('advance_warning') or {}).get('lead_s') for x in m])
        d['first_pred_cost_at_crossing_t'] = agg(
            [(x.get('advance_warning') or {}).get(
                'first_predictive_cost_at_crossing_t') for x in m])
        d['first_obstacle_at_crossing_t'] = agg(
            [(x.get('advance_warning') or {}).get(
                'first_obstacle_at_crossing_t') for x in m])
        # reaction detail
        for rk in ('R1_speed_reduction', 'R2_angular', 'R3_lateral_deviation',
                   'R4_stop'):
            vals = [(x.get('reactions') or {}).get(rk) for x in m]
            d[rk + '_t'] = agg([v['t'] if v else None for v in vals])
            d[rk + '_robot_dist_to_crossing'] = agg(
                [v['robot_dist_to_crossing'] if v else None for v in vals])
            d[rk + '_obstacle_dist_to_crossing'] = agg(
                [v['obstacle_dist_to_crossing'] if v else None for v in vals])
            d[rk + '_n_fired'] = sum(1 for v in vals if v)
        by_mode[mode] = d
    out['modes'] = by_mode

    r, p = by_mode['reactive'], by_mode['predictive']
    lead = None
    if r['primary_reaction_time'] and p['primary_reaction_time']:
        lead = round(r['primary_reaction_time']['mean']
                     - p['primary_reaction_time']['mean'], 3)
    out['predictive_reaction_lead_s'] = lead

    rm = [t.get('metrics', {}) for t in trials if t.get('mode') == 'reactive']
    pm = [t.get('metrics', {}) for t in trials if t.get('mode') == 'predictive']
    out['welch'] = {
        k: welch_t([x.get(k) for x in rm], [x.get(k) for x in pm])
        for k in ('min_clearance', 'nav_time', 'primary_reaction_time',
                  'path_length')
    }
    return out


def print_table(s):
    r, p = s['modes']['reactive'], s['modes']['predictive']
    w = 30
    print()
    print('=' * 78)
    print(f"STAGE-4E A/B  --  {s['label']}")
    print('=' * 78)
    print(f"{'':<{w}} {'Reactive':>20} {'Predictive':>20}")
    print('-' * 78)
    print(f"{'Trials':<{w}} {r['n_trials']:>20} {p['n_trials']:>20}")
    print(f"{'Navigation success':<{w}} "
          f"{str(r['n_success']) + '/' + str(r['n_trials']):>20} "
          f"{str(p['n_success']) + '/' + str(p['n_trials']):>20}")
    print(f"{'Success rate':<{w}} {str(r['success_rate']):>20} "
          f"{str(p['success_rate']):>20}")
    print(f"{'Collisions (body contact)':<{w}} {r['n_collision']:>20} "
          f"{p['n_collision']:>20}")
    for key, lab, unit, _h in SCALARS:
        lab2 = f'{lab} [{unit}]'
        print(f'{lab2:<{w}} {fmt(r[key]):>20} {fmt(p[key]):>20}')
    print('-' * 78)
    print(f"{'Worst min clearance [m]':<{w}} "
          f"{(r['min_clearance']['min'] if r['min_clearance'] else '--'):>20} "
          f"{(p['min_clearance']['min'] if p['min_clearance'] else '--'):>20}")
    print(f"{'Advance warning lead [s]':<{w}} {fmt(r['advance_lead_s']):>20} "
          f"{fmt(p['advance_lead_s']):>20}")
    print(f"{'R4 stop event fired':<{w}} {r['R4_stop_n_fired']:>20} "
          f"{p['R4_stop_n_fired']:>20}")
    print('-' * 78)
    print(f"PREDICTIVE REACTION LEAD TIME vs reactive: "
          f"{s['predictive_reaction_lead_s']} s "
          f"(positive = predictive reacts earlier)")
    print(f"Status breakdown  reactive: {r['statuses']}")
    print(f"                predictive: {p['statuses']}")
    print(f"Welch t (reactive vs predictive): {json.dumps(s['welch'])}")
    print('=' * 78)


def write_csv(trials, path):
    cols = ['trial_id', 'mode', 'scenario', 'nav_status', 'success', 'collision',
            'min_clearance', 'min_center_separation', 'nav_time', 'path_length',
            'primary_reaction_time', 'n_stops', 'stop_duration_total',
            'min_v_nav_approach', 'mean_v_nav_approach',
            'max_lateral_dev_approach', 'max_pred_cells',
            'max_pred_frac_window', 'max_master_hi_ahead',
            'mean_master_mean_ahead_approach', 'global_path_changes',
            'collision_monitor_interventions', 'advance_lead_s',
            'first_predictive_cost_at_crossing_t', 'first_obstacle_at_crossing_t']
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=cols)
        wr.writeheader()
        for t in trials:
            m = t.get('metrics', {})
            aw = m.get('advance_warning') or {}
            row = {c: m.get(c) for c in cols}
            row.update({
                'trial_id': t.get('trial_id'), 'mode': t.get('mode'),
                'scenario': t.get('scenario'),
                'advance_lead_s': aw.get('lead_s'),
                'first_predictive_cost_at_crossing_t': aw.get(
                    'first_predictive_cost_at_crossing_t'),
                'first_obstacle_at_crossing_t': aw.get(
                    'first_obstacle_at_crossing_t'),
            })
            wr.writerow(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--label', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--csv', default=None)
    args = ap.parse_args()

    trials = []
    for d in args.dirs:
        trials += load(d)
    if not trials:
        raise SystemExit('no trial results found')

    label = args.label or os.path.basename(os.path.normpath(args.dirs[0]))
    s = summarize(trials, label)
    print_table(s)

    if args.out:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        with open(args.out, 'w') as f:
            json.dump(s, f, indent=1)
        print(f'wrote {args.out}')
    if args.csv:
        write_csv(trials, args.csv)
        print(f'wrote {args.csv}')


if __name__ == '__main__':
    main()
