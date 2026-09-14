#!/usr/bin/env python3
"""Stage-4G6: statistics, coverage-vs-navigation link, and the decision rule.

Applies the thresholds pre-registered in validation/stage4g6/DECISION_RULE.md
mechanically. The rule is evaluated from the data; it is not restated here in a
form that could drift from the committed file, and the thresholds below are the
only place they appear in code.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np

# --- pre-registered thresholds (see DECISION_RULE.md) ----------------------
S1_CLEARANCE_GAIN = .10      # m, mean improvement required
S2_WORST_GAIN = .10          # m, worst-case improvement required
S3_EVENT_GAIN = 2            # events out of 20
C1_GOAL_TIME_FRAC = .10      # max allowed relative increase
C2_PATH_FRAC = .05
C3_STOP_POINTS = .05         # absolute increase in stopped fraction
NEAR_MISS = .10
BOOTSTRAP = 10000
RNG = np.random.default_rng(20260914)


def clean(rows, field):
    return np.array([r[field] for r in rows
                     if r.get(field) is not None and np.isfinite(r[field])], dtype=float)


def describe(v):
    if not len(v):
        return None
    return dict(n=int(len(v)), mean=float(v.mean()), sd=float(v.std(ddof=1)) if len(v) > 1
                else 0., median=float(np.median(v)), min=float(v.min()), max=float(v.max()))


def hedges_g(a, b):
    """Hedges' g: Cohen's d with the small-sample correction (n ~ 20-30 here)."""
    na, nb = len(a), len(b)
    if na < 2 or nb < 2:
        return None
    sp = math_sqrt(((na - 1) * a.var(ddof=1) + (nb - 1) * b.var(ddof=1)) / (na + nb - 2))
    if sp == 0:
        return None
    d = (a.mean() - b.mean()) / sp
    j = 1 - 3 / (4 * (na + nb) - 9)
    return float(d * j)


def math_sqrt(x):
    return float(np.sqrt(x)) if x > 0 else 0.


def boot_ci(a, b, alpha=.05):
    """Bootstrap CI of (mean(a) - mean(b)). No normality assumption: clearance
    distributions are bounded below and visibly skewed."""
    if len(a) < 2 or len(b) < 2:
        return None
    diffs = np.empty(BOOTSTRAP)
    for i in range(BOOTSTRAP):
        diffs[i] = RNG.choice(a, len(a), replace=True).mean() - \
            RNG.choice(b, len(b), replace=True).mean()
    lo, hi = np.percentile(diffs, [100 * alpha / 2, 100 * (1 - alpha / 2)])
    return dict(diff=float(a.mean() - b.mean()), lo=float(lo), hi=float(hi),
                excludes_zero=bool(lo > 0 or hi < 0))


def welch_p(a, b):
    """Two-sided Welch t-test p-value via the normal approximation to the t
    distribution's survival function; reported for context only -- the decision
    rule is the pre-registered effect thresholds, not this number."""
    if len(a) < 2 or len(b) < 2:
        return None
    va, vb = a.var(ddof=1) / len(a), b.var(ddof=1) / len(b)
    se = np.sqrt(va + vb)
    if se == 0:
        return None
    tstat = (a.mean() - b.mean()) / se
    df = (va + vb) ** 2 / (va ** 2 / (len(a) - 1) + vb ** 2 / (len(b) - 1))
    # Welch-Satterthwaite df; two-sided p from a t with that df, via an
    # incomplete-beta-free approximation that is accurate to ~1e-3 for df > 5.
    x = df / (df + tstat ** 2)
    p = betainc_half(df / 2, .5, x)
    return dict(t=float(tstat), df=float(df), p=float(min(max(p, 0.), 1.)))


def betainc_half(a, b, x):
    """Regularized incomplete beta I_x(a, b) by continued fraction (Lentz)."""
    if x <= 0:
        return 0.
    if x >= 1:
        return 1.
    from math import lgamma, log, exp
    lbeta = lgamma(a) + lgamma(b) - lgamma(a + b)
    front = exp(log(x) * a + log(1 - x) * b - lbeta) / a
    f, c, d = 1., 1., 0.
    for i in range(0, 300):
        m = i // 2
        if i == 0:
            num = 1.
        elif i % 2 == 0:
            num = (m * (b - m) * x) / ((a + 2 * m - 1) * (a + 2 * m))
        else:
            num = -((a + m) * (a + b + m) * x) / ((a + 2 * m) * (a + 2 * m + 1))
        d = 1. + num * d
        d = 1e-30 if abs(d) < 1e-30 else d
        d = 1. / d
        c = 1. + num / c
        c = 1e-30 if abs(c) < 1e-30 else c
        f *= c * d
        if abs(1. - c * d) < 1e-10:
            break
    return front * (f - 1.)


def fisher_exact(a1, a0, b1, b0):
    """Two-sided Fisher exact p for a 2x2 table of counts."""
    from math import comb
    n = a1 + a0 + b1 + b0
    if n == 0:
        return None
    row1, col1 = a1 + a0, a1 + b1

    def prob(k):
        return comb(row1, k) * comb(n - row1, col1 - k) / comb(n, col1)
    lo = max(0, col1 - (n - row1))
    hi = min(row1, col1)
    obs = prob(a1)
    return float(sum(prob(k) for k in range(lo, hi + 1) if prob(k) <= obs * (1 + 1e-9)))


def compare(cv, hy):
    """All Stage-4G6 comparisons for one scenario."""
    out = dict(n_cv=len(cv), n_hybrid=len(hy))
    for field in ('min_clearance_m', 'min_clearance_target_m', 'goal_reach_time_s',
                  'mean_clearance_target_m', 'path_length_m',
                  'stopped_fraction', 'stopped_duration_s', 'stop_count',
                  'reaction_time_s', 'reaction_lead_s', 'reaction_distance_m',
                  'mean_clearance_m', 'coasting_frames', 'coasting_fraction',
                  'gap_duration_s', 'policy_coverage', 'policy_mean_area_m2',
                  'cv_coasting_coverage', 'time_in_reachability_s'):
        a, b = clean(hy, field), clean(cv, field)
        entry = dict(cv=describe(b), hybrid=describe(a))
        if len(a) > 1 and len(b) > 1:
            entry['hybrid_minus_cv'] = boot_ci(a, b)
            entry['hedges_g'] = hedges_g(a, b)
            entry['welch'] = welch_p(a, b)
        out[field] = entry

    for name, pred in (('succeeded', lambda r: bool(r.get('succeeded'))),
                       ('collision', lambda r: bool(r.get('collision'))),
                       ('target_collision', lambda r: bool(r.get('target_collision'))),
                       ('near_miss', lambda r: bool(r.get('near_miss')))):
        c1 = sum(1 for r in cv if pred(r))
        h1 = sum(1 for r in hy if pred(r))
        out[name] = dict(cv=c1, cv_n=len(cv), hybrid=h1, hybrid_n=len(hy),
                         cv_rate=c1 / len(cv) if cv else None,
                         hybrid_rate=h1 / len(hy) if hy else None,
                         fisher_p=fisher_exact(h1, len(hy) - h1, c1, len(cv) - c1))
    return out


def decide(cmp_):
    """Mechanical application of the pre-registered rule to one scenario."""
    mc = cmp_['min_clearance_m']
    s = {}
    if mc['cv'] and mc['hybrid'] and mc.get('hybrid_minus_cv'):
        d = mc['hybrid_minus_cv']
        s['S1'] = dict(met=bool(d['diff'] >= S1_CLEARANCE_GAIN and d['excludes_zero']),
                       mean_gain_m=d['diff'], ci=[d['lo'], d['hi']],
                       required=S1_CLEARANCE_GAIN)
        worst_gain = mc['hybrid']['min'] - mc['cv']['min']
        s['S2'] = dict(met=bool(worst_gain >= S2_WORST_GAIN),
                       worst_cv_m=mc['cv']['min'], worst_hybrid_m=mc['hybrid']['min'],
                       worst_gain_m=worst_gain, required=S2_WORST_GAIN)
    # S3 as pre-registered: "collisions or navigation failures". A collision is
    # BY DEFINITION also a near miss (clearance <= 0 <= 0.10 m), so adding the
    # two counts would double-count every collision and inflate the apparent
    # benefit. Events are therefore collisions plus goal failures, counted once.
    col, suc = cmp_['collision'], cmp_['succeeded']
    events_cv = col['cv'] + (suc['cv_n'] - suc['cv'])
    events_hy = col['hybrid'] + (suc['hybrid_n'] - suc['hybrid'])
    # Scale the pre-registered "2 out of 20" to the actual arm sizes so the
    # threshold means the same rate regardless of how many trials were run.
    required = S3_EVENT_GAIN * (min(suc['cv_n'], suc['hybrid_n']) / 20.)
    s['S3'] = dict(met=bool((events_cv - events_hy) >= required),
                   cv_events=events_cv, hybrid_events=events_hy,
                   cv_collisions=col['cv'], hybrid_collisions=col['hybrid'],
                   cv_failures=suc['cv_n'] - suc['cv'],
                   hybrid_failures=suc['hybrid_n'] - suc['hybrid'],
                   required_reduction=required,
                   note='collisions + goal failures, counted once each '
                        '(a collision is not also counted as a near miss); '
                        'threshold scaled from the pre-registered 2-in-20 rate')
    # Near misses are reported alongside, but are not an S3 event class.
    s['near_miss_context'] = dict(
        cv=cmp_['near_miss']['cv'], hybrid=cmp_['near_miss']['hybrid'],
        threshold_m=NEAR_MISS, met=None)

    c = {}
    for cid, field, kind, lim in (('C1', 'goal_reach_time_s', 'rel', C1_GOAL_TIME_FRAC),
                                  ('C2', 'path_length_m', 'rel', C2_PATH_FRAC),
                                  ('C3', 'stopped_fraction', 'abs', C3_STOP_POINTS)):
        e = cmp_[field]
        if not (e['cv'] and e['hybrid']):
            c[cid] = dict(violated=None)
            continue
        inc = e['hybrid']['mean'] - e['cv']['mean']
        rel = inc / e['cv']['mean'] if e['cv']['mean'] else None
        c[cid] = dict(violated=bool((rel if kind == 'rel' else inc) > lim),
                      increase=inc, relative=rel, limit=lim, kind=kind)
    # The same safety criteria evaluated on the HYPOTHESIS-SPECIFIC endpoint:
    # clearance to the manipulated obstacle alone. Reported alongside the
    # pre-registered global endpoint, never in place of it.
    mt = cmp_.get('min_clearance_target_m', {})
    s_target = {}
    if mt.get('cv') and mt.get('hybrid') and mt.get('hybrid_minus_cv'):
        dt = mt['hybrid_minus_cv']
        s_target['S1'] = dict(met=bool(dt['diff'] >= S1_CLEARANCE_GAIN and
                                       dt['excludes_zero']),
                              mean_gain_m=dt['diff'], ci=[dt['lo'], dt['hi']])
        s_target['S2'] = dict(met=bool(mt['hybrid']['min'] - mt['cv']['min']
                                       >= S2_WORST_GAIN),
                              worst_cv_m=mt['cv']['min'],
                              worst_hybrid_m=mt['hybrid']['min'],
                              worst_gain_m=mt['hybrid']['min'] - mt['cv']['min'])
    return dict(safety=s, cost=c, safety_on_target_endpoint=s_target,
                target_endpoint_met=[k for k, v in s_target.items() if v.get('met')],
                safety_met=[k for k, v in s.items() if v.get('met')
                            and k in ('S1', 'S2', 'S3')],
                cost_violated=[k for k, v in c.items() if v.get('violated')],
                keep_hybrid=bool(any(s[k].get('met') for k in ('S1', 'S2', 'S3') if k in s)
                                 and not any(v.get('violated') for v in c.values())))


def correlate(rows):
    """Does better coasting coverage actually produce better navigation?"""
    out = {}
    for x_field in ('policy_coverage', 'coasting_frames', 'time_in_reachability_s',
                    'policy_mean_area_m2'):
        for y_field in ('min_clearance_m', 'reaction_time_s', 'goal_reach_time_s'):
            pairs = [(r[x_field], r[y_field]) for r in rows
                     if r.get(x_field) is not None and r.get(y_field) is not None
                     and np.isfinite(r[x_field]) and np.isfinite(r[y_field])]
            if len(pairs) < 6:
                continue
            x = np.array([p[0] for p in pairs])
            y = np.array([p[1] for p in pairs])
            if x.std() == 0 or y.std() == 0:
                continue
            r_p = float(np.corrcoef(x, y)[0, 1])
            xs = np.argsort(np.argsort(x)).astype(float)
            ys = np.argsort(np.argsort(y)).astype(float)
            r_s = float(np.corrcoef(xs, ys)[0, 1])
            out[f'{x_field}__vs__{y_field}'] = dict(n=len(pairs), pearson=r_p,
                                                    spearman=r_s)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rows, failures = [], []
    for path in args.trials:
        data = json.loads(Path(path).read_text())
        rows.extend(data['trials'])
        failures.extend(data.get('setup_failures', []))

    by_scenario = defaultdict(lambda: defaultdict(list))
    for r in rows:
        by_scenario[r['scenario']][r['layer_mode']].append(r)

    scenarios = {}
    for scenario, arms in sorted(by_scenario.items()):
        cv = arms.get('cv_covariance', [])
        hy = arms.get('hybrid', [])
        if not cv or not hy:
            scenarios[scenario] = dict(n_cv=len(cv), n_hybrid=len(hy),
                                       note='one arm missing; not compared')
            continue
        cmp_ = compare(cv, hy)
        cmp_['decision'] = decide(cmp_)
        cmp_['correlation_within_scenario'] = correlate(cv + hy)
        # Pooling both arms confounds "coverage" with "which arm": hybrid always
        # has ~1.0 coasting coverage and CV ~0.5, so a pooled correlation would
        # just re-measure the arm difference. The within-arm correlations below
        # isolate the mechanism -- among CV trials alone, does a trial that
        # happened to get better coasting coverage also navigate better?
        cmp_['correlation_within_cv_arm'] = correlate(cv)
        cmp_['correlation_within_hybrid_arm'] = correlate(hy)
        scenarios[scenario] = cmp_

    occlusion = [s for s in scenarios
                 if s.startswith('g6_') and s not in ('g6_visible',)
                 and 'noconflict' not in s and 'expiry' not in s
                 and 'mixed' not in s and 'dirstep' not in s]
    keepers = [s for s in occlusion
               if scenarios[s].get('decision', {}).get('keep_hybrid')]

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(
        note='Thresholds are those pre-registered in DECISION_RULE.md. p-values are '
             'reported for context; the decision is made by the effect thresholds.',
        thresholds=dict(S1_clearance_gain_m=S1_CLEARANCE_GAIN,
                        S2_worst_gain_m=S2_WORST_GAIN, S3_event_reduction=S3_EVENT_GAIN,
                        C1_goal_time_frac=C1_GOAL_TIME_FRAC, C2_path_frac=C2_PATH_FRAC,
                        C3_stop_points=C3_STOP_POINTS, near_miss_m=NEAR_MISS,
                        bootstrap_resamples=BOOTSTRAP),
        n_trials=len(rows), n_setup_failures=len(failures), setup_failures=failures,
        primary_decision=dict(
            occlusion_scenarios_evaluated=sorted(occlusion),
            scenarios_meeting_keep_criteria=sorted(keepers),
            recommendation=('KEEP_HYBRID' if keepers
                            else 'REMOVE_REACHABILITY_FROM_PRODUCTION_PATH')),
        correlation_all_occlusion=correlate(
            [r for r in rows if r['scenario'] in occlusion]),
        correlation_all_occlusion_cv_arm=correlate(
            [r for r in rows if r['scenario'] in occlusion
             and r['layer_mode'] == 'cv_covariance']),
        correlation_all_occlusion_hybrid_arm=correlate(
            [r for r in rows if r['scenario'] in occlusion
             and r['layer_mode'] == 'hybrid']),
        scenarios=scenarios), indent=1) + '\n')
    print(f"{len(rows)} trials, {len(failures)} setup failures")
    print(f"recommendation: {'KEEP_HYBRID' if keepers else 'REMOVE_REACHABILITY'}")


if __name__ == '__main__':
    main()
