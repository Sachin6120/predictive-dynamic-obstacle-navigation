#!/usr/bin/env python3
"""Stage-4F summariser: per-trial JSON -> benchmark tables, CSV, JSON, plots.

Groups trials by (condition, mode) and emits everything the report needs:
a per-condition A/B table, a compact cross-scenario summary, the speed study,
the prediction-quality table, the failure register, and optional plots.
Nothing is transcribed by hand.

    summarize_stage4f.py validation_logs/stage4f/bench \
        --out validation/stage4f_benchmark.json \
        --csv validation/stage4f_trials.csv \
        --plots validation/plots
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as st

SAFETY = [
    ('min_clearance', 'Min clearance', 'm'),
    ('min_center_separation', 'Min centre separation', 'm'),
    ('collision_monitor_interventions', 'Collision-monitor interventions', 'n'),
]
EFFICIENCY = [
    ('nav_time', 'Navigation time', 's'),
    ('path_length', 'Path length', 'm'),
    ('n_stops', 'Stop events', 'n'),
    ('stop_duration_total', 'Stop duration', 's'),
]
REACTION = [
    ('primary_reaction_time', 'Reaction time R1', 's'),
    ('min_v_nav_approach', 'Min commanded v (approach)', 'm/s'),
    ('mean_v_nav_approach', 'Mean commanded v (approach)', 'm/s'),
]
TRAJECTORY = [
    ('max_abs_lateral_offset_approach', 'Max lateral offset', 'm'),
    ('integrated_abs_lateral_approach', 'Integrated |lateral|', 'm.s'),
    ('max_abs_w_nav_approach', 'Max |w| (approach)', 'rad/s'),
    ('cmd_variation', 'Command variation', 'm/s/sample'),
]
PLANNING = [
    ('global_path_changes', 'Global route changes', 'n'),
    ('plan_publications', 'Global plan publications', 'n'),
]
PREDICTION = [
    ('max_pred_cells', 'Predictive cells (max)', 'n'),
    ('max_pred_frac_window', 'Predictive window coverage', 'frac'),
    ('max_master_hi_ahead', 'Master cells >=180 ahead', 'n'),
]
ALL_METRICS = SAFETY + EFFICIENCY + REACTION + TRAJECTORY + PLANNING + PREDICTION


def load(dirs):
    trials = []
    for d in dirs:
        for path in sorted(glob.glob(os.path.join(d, '*.json'))):
            base = os.path.basename(path)
            if base.endswith('_series.json') or base == 'summary.json':
                continue
            try:
                with open(path) as f:
                    t = json.load(f)
            except json.JSONDecodeError:
                print(f'  ! unreadable: {path}')
                continue
            tid = t.get('trial_id', base)
            # trial ids are "<condition>_<mode>_<nn>[_tag]"
            for mode in ('reactive', 'predictive'):
                if f'_{mode}_' in tid:
                    t['condition'] = tid.split(f'_{mode}_')[0]
                    break
            else:
                t['condition'] = tid
            trials.append(t)
    return trials


def agg(values):
    v = [x for x in values if x is not None]
    if not v:
        return None
    s = sorted(v)
    return {'n': len(v), 'mean': round(st.mean(v), 4),
            'sd': round(st.pstdev(v), 4) if len(v) > 1 else 0.0,
            'median': round(s[len(s) // 2], 4),
            'min': round(s[0], 4), 'max': round(s[-1], 4)}


def fmt(a):
    return '--' if a is None else f"{a['mean']:.3f} ± {a['sd']:.3f}"


def cohens_d(a, b):
    """Standardised mean difference (predictive - reactive), pooled sd.

    Reported as an effect size because it does not pretend the trials are
    independent draws from a population -- they are repeats of one
    deterministic scenario under startup/timing jitter. Read it as "how big is
    the separation relative to the spread", not as an inferential statistic.
    """
    a = [x for x in a if x is not None]
    b = [x for x in b if x is not None]
    if len(a) < 2 or len(b) < 2:
        return None
    va, vb = st.variance(a), st.variance(b)
    pooled = math.sqrt(((len(a) - 1) * va + (len(b) - 1) * vb) /
                       (len(a) + len(b) - 2))
    if pooled <= 0:
        return None
    return round((st.mean(b) - st.mean(a)) / pooled, 2)


def mean_ci(values, z=1.96):
    """Normal-approximation CI on the mean. Same caveat as cohens_d."""
    v = [x for x in values if x is not None]
    if len(v) < 2:
        return None
    se = st.stdev(v) / math.sqrt(len(v))
    m = st.mean(v)
    return [round(m - z * se, 4), round(m + z * se, 4)]


def summarize_condition(trials, cond):
    sel = [t for t in trials if t['condition'] == cond]
    out = {'condition': cond, 'n_trials': len(sel), 'modes': {}}
    for mode in ('reactive', 'predictive'):
        ms = [t.get('metrics', {}) for t in sel if t.get('mode') == mode]
        d = {'n_trials': len(ms),
             'n_success': sum(1 for m in ms if m.get('success')),
             'n_collision': sum(1 for m in ms if m.get('collision')),
             'statuses': {}, 'response_classes': {}}
        for m in ms:
            s = m.get('nav_status', 'NO_RESULT')
            d['statuses'][s] = d['statuses'].get(s, 0) + 1
            rc = m.get('response_class', 'none')
            d['response_classes'][rc] = d['response_classes'].get(rc, 0) + 1
        d['success_rate'] = round(d['n_success'] / len(ms), 3) if ms else None
        for key, _l, _u in ALL_METRICS:
            d[key] = agg([m.get(key) for m in ms])
        d['min_clearance_ci95'] = mean_ci([m.get('min_clearance') for m in ms])
        d['nav_time_ci95'] = mean_ci([m.get('nav_time') for m in ms])
        d['reaction_time_ci95'] = mean_ci([m.get('primary_reaction_time')
                                           for m in ms])
        d['advance_lead_s'] = agg([(m.get('advance_warning') or {}).get('lead_s')
                                   for m in ms])
        # prediction quality: mean over trials of each trial's mean
        pq = [m.get('prediction_quality') or {} for m in ms]
        d['tracking_position_error_m'] = agg(
            [(p.get('tracking_position_error_m') or {}).get('mean') for p in pq])
        d['tracking_velocity_error_mps'] = agg(
            [(p.get('tracking_velocity_error_mps') or {}).get('mean') for p in pq])
        d['ADE_m'] = agg([(p.get('ADE_m') or {}).get('mean') for p in pq])
        d['FDE_m'] = agg([(p.get('FDE_m') or {}).get('mean') for p in pq])
        by_h = {}
        for p in pq:
            for h, v in (p.get('displacement_error_by_horizon_m') or {}).items():
                by_h.setdefault(h, []).append((v or {}).get('mean'))
        d['displacement_error_by_horizon_m'] = {
            h: agg(v) for h, v in sorted(by_h.items(), key=lambda z: float(z[0]))}
        out['modes'][mode] = d

    r, p = out['modes']['reactive'], out['modes']['predictive']
    out['reaction_lead_s'] = (
        round(r['primary_reaction_time']['mean'] - p['primary_reaction_time']['mean'], 3)
        if r['primary_reaction_time'] and p['primary_reaction_time'] else None)
    rm = [t.get('metrics', {}) for t in sel if t.get('mode') == 'reactive']
    pm = [t.get('metrics', {}) for t in sel if t.get('mode') == 'predictive']
    out['effect_size_cohens_d'] = {
        k: cohens_d([m.get(k) for m in rm], [m.get(k) for m in pm])
        for k in ('min_clearance', 'nav_time', 'primary_reaction_time',
                  'path_length', 'max_abs_lateral_offset_approach')}
    return out


def print_condition(c):
    r, p = c['modes']['reactive'], c['modes']['predictive']
    w = 32
    print('\n' + '=' * 80)
    print(f"CONDITION: {c['condition']}   ({r['n_trials']} reactive / "
          f"{p['n_trials']} predictive)")
    print('=' * 80)
    print(f"{'':<{w}} {'Reactive':>21} {'Predictive':>21}")
    print('-' * 80)
    rs = f"{r['n_success']}/{r['n_trials']}"
    ps = f"{p['n_success']}/{p['n_trials']}"
    print(f"{'Success':<{w}} {rs:>21} {ps:>21}")
    print(f"{'Collisions (body contact)':<{w}} "
          f"{r['n_collision']:>21} {p['n_collision']:>21}")
    for group, title in ((SAFETY, 'SAFETY'), (EFFICIENCY, 'EFFICIENCY'),
                         (REACTION, 'REACTION'), (TRAJECTORY, 'TRAJECTORY'),
                         (PLANNING, 'PLANNING'), (PREDICTION, 'PREDICTIVE COST')):
        print(f'-- {title} ' + '-' * (76 - len(title)))
        for key, lab, unit in group:
            name = f'{lab} [{unit}]'
            print(f'{name:<{w}} {fmt(r[key]):>21} {fmt(p[key]):>21}')
    print('-' * 80)
    if r['min_clearance'] and p['min_clearance']:
        rw = f"{r['min_clearance']['min']:.4f}"
        pw = f"{p['min_clearance']['min']:.4f}"
        print(f"{'Worst min clearance [m]':<{w}} {rw:>21} {pw:>21}")
    print(f"{'Advance warning lead [s]':<{w}} "
          f"{fmt(r['advance_lead_s']):>21} {fmt(p['advance_lead_s']):>21}")
    print(f"{'Response classes  reactive':<{w}} {r['response_classes']}")
    print(f"{'                predictive':<{w}} {p['response_classes']}")
    print(f"PREDICTIVE REACTION LEAD: {c['reaction_lead_s']} s "
          f"(positive = predictive reacts earlier)")
    print(f"Cohen's d (predictive - reactive): "
          f"{json.dumps(c['effect_size_cohens_d'])}")
    print(f"95% CI on mean min clearance  reactive {r['min_clearance_ci95']}  "
          f"predictive {p['min_clearance_ci95']}")
    print(f"Status  reactive: {r['statuses']}   predictive: {p['statuses']}")


def _cell(a, key, fmt_str, width):
    """One right-aligned table cell from an aggregate dict, or '--'."""
    txt = '--' if not a else format(a[key], fmt_str)
    return f'{txt:>{width}}'


def print_compact(summary):
    print('\n' + '=' * 108)
    print('STAGE-4F COMPACT BENCHMARK SUMMARY')
    print('=' * 108)
    print(f"{'condition':<12}{'mode':<12}{'succ':>7}{'coll':>5}"
          f"{'minClr[m]':>11}{'worst':>9}{'navT[s]':>10}{'path[m]':>9}"
          f"{'react[s]':>10}{'lat[m]':>8}{'lead[s]':>9}")
    print('-' * 108)
    for c in summary['conditions']:
        for mode in ('reactive', 'predictive'):
            d = c['modes'][mode]
            if not d['n_trials']:
                continue
            succ = f"{d['n_success']}/{d['n_trials']}"
            lead = c['reaction_lead_s'] if mode == 'predictive' else None
            leadtxt = '--' if lead is None else f'{lead:.2f}'
            print(f"{c['condition']:<12}{mode:<12}{succ:>7}{d['n_collision']:>5}"
                  + _cell(d['min_clearance'], 'mean', '.3f', 11)
                  + _cell(d['min_clearance'], 'min', '.3f', 9)
                  + _cell(d['nav_time'], 'mean', '.2f', 10)
                  + _cell(d['path_length'], 'mean', '.2f', 9)
                  + _cell(d['primary_reaction_time'], 'mean', '.2f', 10)
                  + _cell(d['max_abs_lateral_offset_approach'], 'mean', '.3f', 8)
                  + f'{leadtxt:>9}')
    print('=' * 108)


def print_prediction_quality(summary):
    print('\n' + '=' * 86)
    print('PREDICTION QUALITY (predictive arm; ground truth used for scoring only)')
    print('=' * 86)
    print(f"{'condition':<12}{'trkErr[m]':>11}{'velErr[m/s]':>13}"
          f"{'ADE[m]':>9}{'FDE[m]':>9}   displacement error by horizon [m]")
    print('-' * 86)
    for c in summary['conditions']:
        d = c['modes']['predictive']
        if not d['n_trials']:
            continue
        be = d['displacement_error_by_horizon_m']
        hz = '  '.join(f"{h}s:{(v or {}).get('mean', float('nan')):.2f}"
                       for h, v in be.items()
                       if h in ('0.5', '1.0', '2.0', '3.0'))
        def g(k):
            return f"{d[k]['mean']:.3f}" if d.get(k) else '--'
        print(f"{c['condition']:<12}{g('tracking_position_error_m'):>11}"
              f"{g('tracking_velocity_error_mps'):>13}"
              f"{g('ADE_m'):>9}{g('FDE_m'):>9}   {hz}")
    print('=' * 86)


def print_failures(trials):
    fails = []
    for t in trials:
        m = t.get('metrics', {})
        if m.get('failure'):
            fails.append((t.get('trial_id'), t.get('mode'), m['failure']))
    print('\n' + '=' * 100)
    print(f'FAILURE REGISTER ({len(fails)} failed or colliding trials, all retained)')
    print('=' * 100)
    if not fails:
        print('  none')
        return {}
    by_cause = {}
    for tid, mode, f in fails:
        by_cause.setdefault(f['cause'], []).append(tid)
        print(f"  {tid:<32} {f['nav_status']:<10} cause={f['cause']}")
        print(f"     {f['evidence']}")
        print(f"     robot={f['robot_pose'][:2]} obstacle={f['obstacle_pose']} "
              f"clr={f['clearance']} cmd_v={f['cmd_vel_nav'][0]} "
              f"pred_cells={f['predictive_cells']} tracked={f['frames_tracked_fraction']}")
    print('-' * 100)
    print('  by cause: ' + json.dumps({k: len(v) for k, v in by_cause.items()}))
    print('=' * 100)
    return {k: len(v) for k, v in by_cause.items()}


def write_csv(trials, path):
    cols = ['trial_id', 'condition', 'mode', 'scenario', 'nav_status', 'success',
            'collision', 'min_clearance', 'min_center_separation', 'nav_time',
            'path_length', 'n_stops', 'stop_duration_total',
            'primary_reaction_time', 'min_v_nav_approach', 'mean_v_nav_approach',
            'max_abs_lateral_offset_approach', 'integrated_abs_lateral_approach',
            'max_abs_w_nav_approach', 'cmd_variation', 'response_class',
            'global_path_changes', 'plan_publications',
            'collision_monitor_interventions', 'max_pred_cells',
            'max_pred_frac_window', 'advance_lead_s', 'ADE_m', 'FDE_m',
            'tracking_position_error_m', 'failure_cause']
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=cols)
        wr.writeheader()
        for t in trials:
            m = t.get('metrics', {})
            pq = m.get('prediction_quality') or {}
            row = {c: m.get(c) for c in cols}
            row.update({
                'trial_id': t.get('trial_id'), 'condition': t.get('condition'),
                'mode': t.get('mode'), 'scenario': t.get('scenario'),
                'advance_lead_s': (m.get('advance_warning') or {}).get('lead_s'),
                'ADE_m': (pq.get('ADE_m') or {}).get('mean'),
                'FDE_m': (pq.get('FDE_m') or {}).get('mean'),
                'tracking_position_error_m':
                    (pq.get('tracking_position_error_m') or {}).get('mean'),
                'failure_cause': (m.get('failure') or {}).get('cause'),
            })
            wr.writerow(row)


def make_plots(summary, trials, outdir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib unavailable; skipping plots')
        return []
    os.makedirs(outdir, exist_ok=True)
    conds = [c['condition'] for c in summary['conditions']]
    written = []

    def bygroup(key):
        rs, ps = [], []
        for c in summary['conditions']:
            rs.append((c['modes']['reactive'].get(key) or {}).get('mean'))
            ps.append((c['modes']['predictive'].get(key) or {}).get('mean'))
        return rs, ps

    def errs(key):
        re_, pe = [], []
        for c in summary['conditions']:
            re_.append((c['modes']['reactive'].get(key) or {}).get('sd', 0))
            pe.append((c['modes']['predictive'].get(key) or {}).get('sd', 0))
        return re_, pe

    panels = [('min_clearance', 'Minimum clearance [m]', 'clearance'),
              ('primary_reaction_time', 'Reaction time R1 [s]', 'reaction_time'),
              ('nav_time', 'Navigation time [s]', 'nav_time'),
              ('path_length', 'Path length [m]', 'path_length')]
    for key, title, fname in panels:
        rs, ps = bygroup(key)
        re_, pe = errs(key)
        x = range(len(conds))
        fig, ax = plt.subplots(figsize=(7.5, 3.6))
        ax.bar([i - 0.19 for i in x], [v or 0 for v in rs], 0.38,
               yerr=[v or 0 for v in re_], capsize=3, label='reactive',
               color='#c44e52')
        ax.bar([i + 0.19 for i in x], [v or 0 for v in ps], 0.38,
               yerr=[v or 0 for v in pe], capsize=3, label='predictive',
               color='#4c72b0')
        ax.set_xticks(list(x))
        ax.set_xticklabels(conds, rotation=20, ha='right')
        ax.set_ylabel(title)
        ax.set_title(f'Stage-4F: {title} by scenario (mean ± sd)')
        ax.legend()
        ax.grid(axis='y', alpha=0.3)
        fig.tight_layout()
        p = os.path.join(outdir, f'stage4f_{fname}.png')
        fig.savefig(p, dpi=120)
        plt.close(fig)
        written.append(p)

    # reaction lead vs obstacle speed (speed study)
    speeds = {'perp_025': 0.25, 'perp_050': 0.50, 'perp_075': 0.75}
    pts = [(speeds[c['condition']], c['reaction_lead_s'])
           for c in summary['conditions']
           if c['condition'] in speeds and c['reaction_lead_s'] is not None]
    if len(pts) >= 2:
        pts.sort()
        fig, ax = plt.subplots(figsize=(5.5, 3.6))
        ax.plot([p[0] for p in pts], [p[1] for p in pts], 'o-', color='#4c72b0')
        ax.set_xlabel('Obstacle speed [m/s]')
        ax.set_ylabel('Predictive reaction lead [s]')
        ax.set_title('Stage-4F: reaction lead vs obstacle speed')
        ax.grid(alpha=0.3)
        fig.tight_layout()
        p = os.path.join(outdir, 'stage4f_lead_vs_speed.png')
        fig.savefig(p, dpi=120)
        plt.close(fig)
        written.append(p)

    # ADE/FDE vs horizon
    fig, ax = plt.subplots(figsize=(5.5, 3.6))
    plotted = False
    for c in summary['conditions']:
        be = c['modes']['predictive'].get('displacement_error_by_horizon_m') or {}
        hs = sorted((float(h) for h in be), key=float)
        ys = [(be[f'{h:.1f}'] or {}).get('mean') for h in hs]
        if hs and all(y is not None for y in ys):
            ax.plot(hs, ys, 'o-', label=c['condition'])
            plotted = True
    if plotted:
        ax.set_xlabel('Prediction horizon [s]')
        ax.set_ylabel('Mean displacement error [m]')
        ax.set_title('Stage-4F: prediction error vs horizon')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = os.path.join(outdir, 'stage4f_error_vs_horizon.png')
        fig.savefig(p, dpi=120)
        written.append(p)
    plt.close(fig)
    return written


ARENA_WALLS = [(0.0, -4.0, 10.15, 0.15), (0.0, 4.0, 10.15, 0.15),
               (-5.0, 0.0, 0.15, 8.15), (5.0, 0.0, 0.15, 8.15),
               (-3.0, 3.0, 0.30, 0.30), (1.2, 3.0, 0.30, 0.30),
               (-1.5, -3.0, 0.30, 0.30), (2.5, -3.0, 0.30, 0.30)]


def make_demo_figures(dirs, trials, outdir, cases):
    """Render the demonstration evidence: robot and obstacle trajectories for a
    representative trial of each named case, with the reaction instant marked.

    This is the §15 demo evidence, generated from the recorded series rather
    than from a screen capture, so it is reproducible and carries the numbers.
    """
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Circle
    except ImportError:
        print('matplotlib unavailable; skipping demo figures')
        return []
    os.makedirs(outdir, exist_ok=True)
    written = []

    by_id = {t.get('trial_id'): t for t in trials}

    def find_series(tid):
        for d in dirs:
            p2 = os.path.join(d, f'{tid}_series.json')
            if os.path.exists(p2):
                return p2
        return None

    fig, axes = plt.subplots(1, len(cases), figsize=(5.0 * len(cases), 4.6))
    if len(cases) == 1:
        axes = [axes]

    for ax, (cond, mode, title) in zip(axes, cases):
        cands = [t for t in trials
                 if t.get('condition') == cond and t.get('mode') == mode
                 and t.get('metrics', {}).get('min_clearance') is not None]
        if not cands:
            ax.set_title(f'{title}\n(no trial)')
            continue
        # median-clearance trial == representative, not cherry-picked best
        cands.sort(key=lambda t: t['metrics']['min_clearance'])
        tr = cands[len(cands) // 2]
        sp = find_series(tr['trial_id'])
        if not sp:
            ax.set_title(f'{title}\n(no series)')
            continue
        S = json.load(open(sp))['samples']
        m = tr['metrics']

        for cx, cy, sx, sy in ARENA_WALLS:
            ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                   color='#666666', zorder=1))
        rx = [s['rx'] for s in S]
        ry = [s['ry'] for s in S]
        v = [s['v_nav'] for s in S]
        sc = ax.scatter(rx, ry, c=v, cmap='viridis', s=7, vmin=0.0, vmax=0.5,
                        zorder=3)
        ox = [s['ox'] for s in S if s.get('ox') is not None]
        oy = [s['oy'] for s in S if s.get('oy') is not None]
        if ox:
            ax.plot(ox, oy, '-', color='#c44e52', lw=1.6, zorder=2,
                    label='obstacle (ground truth)')
            ax.add_patch(Circle((ox[-1], oy[-1]), 0.20, color='#c44e52',
                                alpha=0.35, zorder=2))

        rt = m.get('primary_reaction_time')
        if rt is not None:
            k = min(range(len(S)), key=lambda i: abs(S[i]['t'] - rt))
            ax.plot(S[k]['rx'], S[k]['ry'], 'v', color='black', ms=11, zorder=5,
                    label=f'reaction t={rt:.2f}s')
            if S[k].get('ox') is not None:
                ax.plot(S[k]['ox'], S[k]['oy'], 'v', color='#c44e52', ms=9,
                        zorder=5)
        ax.plot(rx[0], ry[0], 'o', color='black', ms=7, zorder=5)
        ax.plot(3.0, 0.0, '*', color='#55a868', ms=15, zorder=5, label='goal')

        ax.set_xlim(-5.4, 5.4)
        ax.set_ylim(-4.4, 4.4)
        ax.set_aspect('equal')
        ax.grid(alpha=0.25)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title(f"{title}\nmin clearance {m['min_clearance']:.3f} m, "
                     f"nav {m['nav_time']:.1f} s, {m.get('response_class')}",
                     fontsize=10)
        ax.legend(fontsize=7, loc='upper left')
    cb = fig.colorbar(sc, ax=axes, fraction=0.025, pad=0.02)
    cb.set_label('commanded forward speed [m/s]')
    p3 = os.path.join(outdir, 'stage4f_demo_trajectories.png')
    fig.savefig(p3, dpi=130, bbox_inches='tight')
    plt.close(fig)
    written.append(p3)
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--out')
    ap.add_argument('--csv')
    ap.add_argument('--plots')
    ap.add_argument('--order', default='')
    ap.add_argument('--demo', action='store_true',
                    help='render the representative-trajectory demo figure')
    args = ap.parse_args()

    trials = load(args.dirs)
    if not trials:
        raise SystemExit('no trial results found')
    conds = sorted({t['condition'] for t in trials})
    if args.order:
        want = [c.strip() for c in args.order.split(',')]
        conds = [c for c in want if c in conds] + [c for c in conds if c not in want]

    summary = {'n_trials': len(trials),
               'conditions': [summarize_condition(trials, c) for c in conds]}
    for c in summary['conditions']:
        print_condition(c)
    print_compact(summary)
    print_prediction_quality(summary)
    summary['failures_by_cause'] = print_failures(trials)

    if args.out:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        with open(args.out, 'w') as f:
            json.dump(summary, f, indent=1)
        print(f'\nwrote {args.out}')
    if args.csv:
        write_csv(trials, args.csv)
        print(f'wrote {args.csv}')
    if args.plots:
        for p in make_plots(summary, trials, args.plots):
            print(f'wrote {p}')
        if args.demo:
            cases = [('perp_050', 'reactive', 'A. Crossing - REACTIVE'),
                     ('perp_050', 'predictive', 'A. Crossing - PREDICTIVE'),
                     ('noconflict', 'predictive', 'D. No conflict - PREDICTIVE')]
            for p in make_demo_figures(args.dirs, trials, args.plots, cases):
                print(f'wrote {p}')


if __name__ == '__main__':
    main()
