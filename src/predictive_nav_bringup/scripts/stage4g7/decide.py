#!/usr/bin/env python3
"""Stage-4G7 statistics and the FINAL production decision.

Applies the four gates pre-registered in validation/stage4g7/DECISION_RULE.md:
A practical effect, B replication, C mechanism, D cost. All four are required.
The thresholds appear here and nowhere else in code.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np

A_GAIN = .10          # m, required gain for A1 / A2 / A3
A4_EVENTS = 2         # collisions per 20 trials
D1_GOAL, D2_PATH, D3_STOP = .15, .08, .08
NEAR_MISS = .10
BOOTSTRAP = 10000
RNG = np.random.default_rng(20260915)
PRIMARY = 'g7_gap08'
REPLICATION_CANDIDATES = ('g7_gap06', 'g7_gap10', 'g7_turn')


def clean(rows, f):
    return np.array([r[f] for r in rows
                     if r.get(f) is not None and np.isfinite(r[f])], float)


def describe(v):
    if not len(v):
        return None
    return dict(n=int(len(v)), mean=float(v.mean()),
                sd=float(v.std(ddof=1)) if len(v) > 1 else 0.,
                median=float(np.median(v)), min=float(v.min()), max=float(v.max()),
                p10=float(np.percentile(v, 10)))


def hedges_g(a, b):
    if len(a) < 2 or len(b) < 2:
        return None
    sp = np.sqrt(((len(a) - 1) * a.var(ddof=1) + (len(b) - 1) * b.var(ddof=1)) /
                 (len(a) + len(b) - 2))
    if sp == 0:
        return None
    return float((a.mean() - b.mean()) / sp * (1 - 3 / (4 * (len(a) + len(b)) - 9)))


def boot(a, b):
    if len(a) < 2 or len(b) < 2:
        return None
    d = np.array([RNG.choice(a, len(a), True).mean() - RNG.choice(b, len(b), True).mean()
                  for _ in range(BOOTSTRAP)])
    lo, hi = np.percentile(d, [2.5, 97.5])
    return dict(diff=float(a.mean() - b.mean()), lo=float(lo), hi=float(hi),
                excludes_zero=bool(lo > 0 or hi < 0))


def welch(a, b):
    if len(a) < 2 or len(b) < 2:
        return None
    va, vb = a.var(ddof=1) / len(a), b.var(ddof=1) / len(b)
    se = np.sqrt(va + vb)
    if se == 0:
        return None
    tt = (a.mean() - b.mean()) / se
    df = (va + vb) ** 2 / (va ** 2 / (len(a) - 1) + vb ** 2 / (len(b) - 1))
    from math import lgamma, exp, log
    x = df / (df + tt * tt)
    a_, b_ = df / 2, .5
    if x <= 0:
        p = 0.
    elif x >= 1:
        p = 1.
    else:
        lb = lgamma(a_) + lgamma(b_) - lgamma(a_ + b_)
        front = exp(log(x) * a_ + log(1 - x) * b_ - lb) / a_
        f, c, dd = 1., 1., 0.
        for i in range(300):
            m = i // 2
            num = 1. if i == 0 else (
                (m * (b_ - m) * x) / ((a_ + 2 * m - 1) * (a_ + 2 * m)) if i % 2 == 0
                else -((a_ + m) * (a_ + b_ + m) * x) / ((a_ + 2 * m) * (a_ + 2 * m + 1)))
            dd = 1. + num * dd
            dd = 1e-30 if abs(dd) < 1e-30 else dd
            dd = 1. / dd
            c = 1. + num / c
            c = 1e-30 if abs(c) < 1e-30 else c
            f *= c * dd
            if abs(1. - c * dd) < 1e-10:
                break
        p = front * (f - 1.)
    return dict(t=float(tt), df=float(df), p=float(min(max(p, 0.), 1.)))


def compare(cv, hy):
    out = dict(n_cv=len(cv), n_hybrid=len(hy))
    for f in ('min_clearance_target_m', 'mean_clearance_target_m', 'min_clearance_m',
              'goal_reach_time_s', 'path_length_m', 'stopped_fraction',
              'stopped_duration_s', 'stop_count', 'reaction_time_s', 'reaction_lead_s',
              'gap_duration_s', 'conflict_overlap', 'coasting_frames',
              'cv_coasting_coverage', 'reach_coasting_coverage',
              'gap_costmap_cells_mean', 'gap_speed_mean', 'gap_speed_min',
              'post_gap_speed_mean', 'costmap_cells_mean_all',
              'reacquire_to_min_clearance_s', 'observations_before_gap'):
        a, b = clean(hy, f), clean(cv, f)
        e = dict(cv=describe(b), hybrid=describe(a))
        if len(a) > 1 and len(b) > 1:
            e['hybrid_minus_cv'] = boot(a, b)
            e['hedges_g'] = hedges_g(a, b)
            e['welch'] = welch(a, b)
        out[f] = e
    for name, pred in (('succeeded', lambda r: bool(r.get('succeeded'))),
                       ('target_collision', lambda r: bool(r.get('target_collision'))),
                       ('collision', lambda r: bool(r.get('collision'))),
                       ('near_miss_target', lambda r: bool(r.get('near_miss_target'))),
                       ('target_is_limiting', lambda r: bool(r.get('target_is_limiting'))),
                       ('target_expired', lambda r: bool(r.get('target_expired'))),
                       ('min_clearance_in_gap', lambda r: bool(r.get('min_clearance_in_gap')))):
        out[name] = dict(cv=sum(1 for r in cv if pred(r)), cv_n=len(cv),
                         hybrid=sum(1 for r in hy if pred(r)), hybrid_n=len(hy))
    return out


def gate_a(c):
    m = c['min_clearance_target_m']
    g = {}
    if m['cv'] and m['hybrid'] and m.get('hybrid_minus_cv'):
        d = m['hybrid_minus_cv']
        g['A1'] = dict(met=bool(d['diff'] >= A_GAIN and d['excludes_zero']),
                       gain_m=d['diff'], ci=[d['lo'], d['hi']], required=A_GAIN)
        g['A2'] = dict(met=bool(m['hybrid']['min'] - m['cv']['min'] >= A_GAIN),
                       cv_worst=m['cv']['min'], hybrid_worst=m['hybrid']['min'],
                       gain_m=m['hybrid']['min'] - m['cv']['min'], required=A_GAIN)
        g['A3'] = dict(met=bool(m['hybrid']['p10'] - m['cv']['p10'] >= A_GAIN),
                       cv_p10=m['cv']['p10'], hybrid_p10=m['hybrid']['p10'],
                       gain_m=m['hybrid']['p10'] - m['cv']['p10'], required=A_GAIN)
    tc = c['target_collision']
    need = A4_EVENTS * (min(tc['cv_n'], tc['hybrid_n']) / 20.)
    g['A4'] = dict(met=bool((tc['cv'] - tc['hybrid']) >= need), cv=tc['cv'],
                   hybrid=tc['hybrid'], required_reduction=need)
    return g


def gate_c(c):
    """Mechanism: target limiting, reachability actually in use, and a
    costmap/command difference BEFORE reacquisition."""
    lim = c['target_is_limiting']
    c1 = dict(met=bool(lim['hybrid'] / max(lim['hybrid_n'], 1) > .5 and
                       lim['cv'] / max(lim['cv_n'], 1) > .5),
              cv_fraction=lim['cv'] / max(lim['cv_n'], 1),
              hybrid_fraction=lim['hybrid'] / max(lim['hybrid_n'], 1))
    cf = c['coasting_frames']
    c2 = dict(met=bool(cf['hybrid'] and cf['hybrid']['mean'] > 0),
              mean_coasting_frames=cf['hybrid']['mean'] if cf['hybrid'] else 0.)
    cells = c['gap_costmap_cells_mean']
    spd = c['gap_speed_mean']
    cells_diff = cells.get('hybrid_minus_cv') if cells else None
    spd_diff = spd.get('hybrid_minus_cv') if spd else None
    c3 = dict(met=bool((cells_diff and cells_diff['excludes_zero']) or
                       (spd_diff and spd_diff['excludes_zero'])),
              costmap_cells_in_gap=cells_diff, speed_in_gap=spd_diff)
    return dict(C1_target_is_limiting=c1, C2_reachability_in_use=c2,
                C3_costmap_or_command_differs_before_reacquisition=c3,
                met=bool(c1['met'] and c2['met'] and c3['met']))


def gate_d(c):
    out = {}
    for cid, f, kind, lim in (('D1', 'goal_reach_time_s', 'rel', D1_GOAL),
                              ('D2', 'path_length_m', 'rel', D2_PATH),
                              ('D3', 'stopped_fraction', 'abs', D3_STOP)):
        e = c[f]
        if not (e['cv'] and e['hybrid']):
            out[cid] = dict(violated=None)
            continue
        inc = e['hybrid']['mean'] - e['cv']['mean']
        rel = inc / e['cv']['mean'] if e['cv']['mean'] else None
        out[cid] = dict(violated=bool((rel if kind == 'rel' else inc) > lim),
                        increase=inc, relative=rel, limit=lim)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    rows, failures = [], []
    for p in args.trials:
        d = json.loads(Path(p).read_text())
        rows.extend(d['trials'])
        failures.extend(d.get('setup_failures', []))
    by = defaultdict(lambda: defaultdict(list))
    for r in rows:
        by[r['scenario']][r['layer_mode']].append(r)

    scenarios = {}
    for sc, arms in sorted(by.items()):
        cv, hy = arms.get('cv_covariance', []), arms.get('hybrid', [])
        if not cv or not hy:
            scenarios[sc] = dict(n_cv=len(cv), n_hybrid=len(hy), note='one arm missing')
            continue
        c = compare(cv, hy)
        c['gate_A'] = gate_a(c)
        c['gate_C'] = gate_c(c)
        c['gate_D'] = gate_d(c)
        c['A_met'] = [k for k, v in c['gate_A'].items() if v.get('met')]
        c['D_violated'] = [k for k, v in c['gate_D'].items() if v.get('violated')]
        scenarios[sc] = c

    prim = scenarios.get(PRIMARY, {})
    a_met = prim.get('A_met', [])
    # Gate B: does a qualifying effect reproduce elsewhere, same sign and >= half
    # the magnitude?
    repl = {}
    if a_met and prim.get('min_clearance_target_m', {}).get('hybrid_minus_cv'):
        base = prim['min_clearance_target_m']['hybrid_minus_cv']['diff']
        for sc in REPLICATION_CANDIDATES:
            e = scenarios.get(sc, {}).get('min_clearance_target_m', {})
            if not e.get('hybrid_minus_cv'):
                continue
            d = e['hybrid_minus_cv']['diff']
            repl[sc] = dict(diff=d, same_sign=bool(np.sign(d) == np.sign(base)),
                            at_least_half=bool(abs(d) >= abs(base) / 2),
                            reproduces=bool(np.sign(d) == np.sign(base) and
                                            abs(d) >= abs(base) / 2))
    b_met = sum(1 for v in repl.values() if v['reproduces']) >= 1

    keep = bool(a_met and b_met and prim.get('gate_C', {}).get('met')
                and not prim.get('D_violated'))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(
        note='Gates A/B/C/D as pre-registered in DECISION_RULE.md. All four required.',
        thresholds=dict(A_gain_m=A_GAIN, A4_events_per_20=A4_EVENTS, D1=D1_GOAL,
                        D2=D2_PATH, D3=D3_STOP, near_miss_m=NEAR_MISS,
                        bootstrap=BOOTSTRAP, primary=PRIMARY),
        n_trials=len(rows), n_setup_failures=len(failures), setup_failures=failures,
        final_decision=dict(
            primary_scenario=PRIMARY, gate_A_met=a_met,
            gate_B_replication=repl, gate_B_met=b_met,
            gate_C_met=prim.get('gate_C', {}).get('met'),
            gate_D_violated=prim.get('D_violated'),
            recommendation='KEEP_HYBRID' if keep else 'SHIP_CV_ONLY'),
        scenarios=scenarios), indent=1) + '\n')
    print(f"{len(rows)} trials, {len(failures)} setup failures")
    print(f"A={a_met} B={b_met} C={prim.get('gate_C', {}).get('met')} "
          f"D_violated={prim.get('D_violated')}")
    print(f"FINAL: {'KEEP_HYBRID' if keep else 'SHIP_CV_ONLY'}")


if __name__ == '__main__':
    main()
