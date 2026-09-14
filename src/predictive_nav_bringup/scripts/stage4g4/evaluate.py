#!/usr/bin/env python3
"""Stage-4G4 offline comparison of CV-Gaussian vs bounded-motion reachability.

Ground truth is used ONLY here, in evaluation, never at runtime: the regions
scored below were produced online by the tracker from its own filtered state
and published on /tracked_objects. This script reads the recording.

FAIRNESS CONVENTIONS (section 11 of the Stage-4G4 brief), fixed before any
result was looked at:

  * Both representations are scored at the SAME horizons {0.5, 1, 2, 3} s and
    against the SAME ground truth interpolated at the time each representation
    itself claims.
  * CV region  = the k-sigma ellipse of the published Stage-4C position
    covariance, k = 2.0 (the same k the Stage-4D costmap layer uses).
  * Reach region = that same k-sigma ellipse, Minkowski-inflated by the
    deterministic bounded-motion radius. Reachability is therefore a strict
    superset of CV by construction at equal k; the question this script answers
    is not "does it cover more" (it must) but "how much area does the extra
    coverage cost", which is why every table reports area and efficiency
    alongside coverage.
  * base_safety_margin is 0.0 in the runtime configuration, so neither
    representation receives additive inflation the other does not.

TIME ANCHORS ARE DIFFERENT AND THAT IS THE POINT. A CV sample asserts a
position at (track.stamp + h): it is anchored at the last real observation and
does not know how stale it is. A reachability sample asserts a region at
(scan.stamp + h) and pays for the observation age explicitly. Each is scored
against ground truth at the absolute time IT claims, which is the only
comparison that asks each model the question it actually answers.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

GATE = .45                  # identical to the Stage-4G2/4G3 scorer
HORIZONS = (.5, 1., 2., 3.)
SIGMA_K = 2.0
ROBOT_FOOTPRINT = dict(cx=-.064, hx=.1325, hy=.153)
OBSTACLE_RADIUS = .20


def interp_gt(track, t):
    """Ground-truth [x, y] of one object at absolute time t, or None if t lies
    outside the recorded odometry span (never extrapolated)."""
    a = np.asarray(track)
    if len(a) < 2 or t < a[0, 0] or t > a[-1, 0]:
        return None
    return np.array([np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2])])


def assignment(points, gt, gate=GATE):
    """Maximum-cardinality, minimum-distance one-to-one matching. Copied in
    behaviour from the Stage-4G2 scorer so track->object identity is decided
    exactly as it was at Stage-4G2/4G3."""
    if not len(points) or not len(gt):
        return {}
    d = np.linalg.norm(np.array(points)[:, None, :] - np.array(gt)[None, :, :], axis=2)
    penalty = (min(d.shape) + 1) * gate
    cost = np.concatenate((np.where(d <= gate, d, penalty * 3),
                           np.full((len(points), len(points)), penalty)), axis=1)
    ri, ci = linear_sum_assignment(cost)
    return {int(c): int(r) for r, c in zip(ri, ci) if c < len(gt) and d[r, c] <= gate}


def reach_radius(t, speed, a_max, v_max):
    """The production bound, reimplemented for the offline a_max sweep.

    Must stay identical to reachability.hpp::reachRadius. Verified against the
    published reach_radius on every recorded sample (see `bound_mismatch`)."""
    if t <= 0 or a_max <= 0:
        return 0.
    cap = v_max + max(speed, 0.)
    t_c = cap / a_max
    if t <= t_c:
        return .5 * a_max * t * t
    return .5 * a_max * t_c * t_c + cap * (t - t_c)


def ellipse_probe(dx, dy, semi_major, semi_minor, yaw):
    """Returns (inside, rho, radial_distance_to_boundary_m).

    rho is the normalized elliptic radius: <= 1 inside. The distance reported
    is measured ALONG THE RAY from the region centre through the point, which
    is exact on that ray and is documented as such rather than presented as the
    true minimum distance to the curve. Negative means inside."""
    if semi_major <= 0 or semi_minor <= 0:
        return False, float('inf'), float('inf')
    c, s = math.cos(yaw), math.sin(yaw)
    u = (dx * c + dy * s) / semi_major
    v = (-dx * s + dy * c) / semi_minor
    rho = math.hypot(u, v)
    r_point = math.hypot(dx, dy)
    if rho <= 1e-12:
        return True, 0., -min(semi_major, semi_minor)
    return rho <= 1., rho, r_point * (1. / rho - 1.) * (1. if rho > 1. else -1.)


def footprint_clearance(rx, ry, ryaw, ox, oy):
    """Surface-to-surface distance between the robot's oriented collision box
    and the obstacle cylinder; <= 0 means the bodies intersect. Identical to
    stage4f_trial.py so Stage-4G4 clearance numbers are comparable to 4F."""
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    dx, dy = ox - rx, oy - ry
    lx = c * dx - s * dy - ROBOT_FOOTPRINT['cx']
    ly = s * dx + c * dy
    ex = max(abs(lx) - ROBOT_FOOTPRINT['hx'], 0.)
    ey = max(abs(ly) - ROBOT_FOOTPRINT['hy'], 0.)
    return math.hypot(ex, ey) - OBSTACLE_RADIUS


def describe(values):
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    if not len(v):
        return None
    return dict(n=int(len(v)), mean=float(v.mean()), p50=float(np.percentile(v, 50)),
                p95=float(np.percentile(v, 95)), max=float(v.max()))


def score_trial(raw, sweep_a_max, v_max, margins, cv_sigmas, fresh_threshold):
    gt = raw['gt']
    frames = raw['arrays']
    out = dict(scenario=raw['scenario'], layer_mode=raw.get('layer_mode', 'keep'),
               frames=len(frames), gt_objects=len(gt))

    # per horizon -> lists
    cv = defaultdict(lambda: defaultdict(list))
    rc = defaultdict(lambda: defaultdict(list))
    # A CV sample is anchored at the track's last observation and carries no
    # notion of how stale that is. `cv_consumed` scores the SAME sample against
    # ground truth at (scan + h) -- the time a downstream consumer reading the
    # current message would take it to describe. The gap between `cv` and
    # `cv_consumed` on coasting frames is precisely the cost of age-blindness,
    # and it is the thing the reachability representation is designed to fix.
    cvc = defaultdict(lambda: defaultdict(list))
    cv_fresh = defaultdict(lambda: defaultdict(list))
    cv_coast = defaultdict(lambda: defaultdict(list))
    cvc_coast = defaultdict(lambda: defaultdict(list))
    rc_fresh = defaultdict(lambda: defaultdict(list))
    rc_coast = defaultdict(lambda: defaultdict(list))
    # Stage-4G5 hybrid policy, evaluated on exactly the same recorded samples as
    # the two representations it selects between. Because the tracker publishes
    # BOTH on every frame and Stage-4G5 changes only the consumer, hybrid needs
    # no separate Gazebo runs: it is a per-sample selection over this recording,
    # which also means the three-way comparison carries zero run-to-run
    # variance. A fresh sample takes the CV region scored AS CONSUMED (which at
    # observation_age == 0 is identical to scoring it at its own anchor, since
    # the two anchors coincide); a coasting sample takes the reachable set.
    hy = defaultdict(lambda: defaultdict(list))
    hy_fresh = defaultdict(lambda: defaultdict(list))
    hy_coast = defaultdict(lambda: defaultdict(list))
    policy_counts = Counter()
    sweep = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    cv_sigma = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    # The same confidence sweep, scored AS CONSUMED and restricted to coasting
    # frames. This answers the obvious objection to the headline result: if a
    # plain CV ellipse is more area-efficient in aggregate, can age-blindness
    # simply be bought off by raising k? The answer has to be measured, because
    # the coasting failure is a centre BIAS, not insufficient spread.
    cv_sigma_coast = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    margin_study = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    bound_mismatch = 0.
    invalid_regions = 0
    coasting_samples = 0
    ages = []

    for frame in frames:
        t = frame['t']
        now_gt = [interp_gt(g, t) for g in gt]
        valid = [i for i, p in enumerate(now_gt) if p is not None]
        if not valid or not frame['tracks']:
            continue
        pairs = assignment([tr['p'] for tr in frame['tracks']],
                           [now_gt[i] for i in valid])
        for local, tr_idx in pairs.items():
            obj = valid[local]
            track = frame['tracks'][tr_idx]

            # ---- CV-Gaussian, anchored at the track's last observation -----
            coasting = track.get('missed', 0) > 0
            # The age the hybrid policy uses, taken from the published samples
            # so producer and consumer cannot disagree about it.
            observation_age = (track['reachability'][0]['age']
                               if track.get('reachability') else
                               max(t - track.get('stamp', t), 0.))
            for pred in track.get('predictions', []):
                h = round(pred['dt'], 3)
                if h not in HORIZONS:
                    continue
                truth = interp_gt(gt[obj], pred['stamp'])
                if truth is None:
                    continue
                mean = np.array(pred['p'])
                err = float(np.linalg.norm(truth - mean))
                cov = np.array(pred['covariance']).reshape(2, 2)
                cov = .5 * (cov + cov.T)
                w, vec = np.linalg.eigh(cov)
                w = np.maximum(w, 0.)
                a = SIGMA_K * math.sqrt(w[1])
                b = SIGMA_K * math.sqrt(w[0])
                yaw = math.atan2(vec[1, 1], vec[0, 1])
                d = truth - mean
                inside, rho, dist = ellipse_probe(d[0], d[1], a, b, yaw)
                for store in (cv, cv_coast if coasting else cv_fresh):
                    store[h]['error'].append(err)
                    store[h]['inside'].append(1. if inside else 0.)
                    store[h]['area'].append(math.pi * a * b)
                    store[h]['boundary'].append(dist)

                # Same region, scored at the time a consumer would assume.
                truth_now = interp_gt(gt[obj], t + h)
                if truth_now is not None:
                    dn = truth_now - mean
                    ins_n, _, dist_n = ellipse_probe(dn[0], dn[1], a, b, yaw)
                    for store in ((cvc,) if not coasting else (cvc, cvc_coast)):
                        store[h]['error'].append(float(np.linalg.norm(dn)))
                        store[h]['inside'].append(1. if ins_n else 0.)
                        store[h]['area'].append(math.pi * a * b)
                        store[h]['boundary'].append(dist_n)
                    # Hybrid FRESH branch: the policy picks CV here.
                    if observation_age <= fresh_threshold:
                        for store in (hy, hy_fresh):
                            store[h]['error'].append(float(np.linalg.norm(dn)))
                            store[h]['inside'].append(1. if ins_n else 0.)
                            store[h]['area'].append(math.pi * a * b)
                            store[h]['boundary'].append(dist_n)
                        policy_counts[('cv', h)] += 1

                    if coasting:
                        for k in cv_sigmas:
                            ka = k * math.sqrt(w[1])
                            kb = k * math.sqrt(w[0])
                            ins, _, _ = ellipse_probe(dn[0], dn[1], ka, kb, yaw)
                            cv_sigma_coast[k][h]['inside'].append(1. if ins else 0.)
                            cv_sigma_coast[k][h]['area'].append(math.pi * ka * kb)
                # The CV confidence level is a CHOICE, so the tradeoff is
                # reported as a curve rather than a single point: the honest
                # question is how much area each representation needs for a
                # given coverage, not which wins at one arbitrary k.
                for k in cv_sigmas:
                    ka = k * math.sqrt(w[1])
                    kb = k * math.sqrt(w[0])
                    ins, _, _ = ellipse_probe(d[0], d[1], ka, kb, yaw)
                    cv_sigma[k][h]['inside'].append(1. if ins else 0.)
                    cv_sigma[k][h]['area'].append(math.pi * ka * kb)

            # ---- Reachability, anchored at this scan ------------------------
            for reg in track.get('reachability', []):
                h = round(reg['dt'], 3)
                if h not in HORIZONS:
                    continue
                if not reg['valid']:
                    invalid_regions += 1
                    continue
                truth = interp_gt(gt[obj], reg['stamp'])
                if truth is None:
                    continue
                if reg['age'] > 1e-6:
                    coasting_samples += 1
                ages.append(reg['age'])
                center = np.array(reg['p'])
                a, b = reg['axes']
                yaw = reg['sigma'][2]
                d = truth - center
                inside, rho, dist = ellipse_probe(d[0], d[1], a, b, yaw)
                targets = [rc, rc_coast if reg['age'] > 1e-6 else rc_fresh]
                # Hybrid COASTING branch: the policy picks reachability here.
                if reg['age'] > fresh_threshold:
                    targets += [hy, hy_coast]
                    policy_counts[('reachability', h)] += 1
                for store in targets:
                    store[h]['error'].append(float(np.linalg.norm(d)))
                    store[h]['inside'].append(1. if inside else 0.)
                    store[h]['area'].append(math.pi * a * b)
                    store[h]['boundary'].append(dist)
                    store[h]['reach_radius'].append(reg['reach_radius'])
                    store[h]['capped'].append(1. if reg['capped'] else 0.)

                # Reconstruct p0/v0 from the published sample so the sweep uses
                # exactly the state the tracker used, then re-derive the region
                # for other bounds. Only the deterministic term changes.
                speed = float(np.hypot(*reg['v']))
                bound_mismatch = max(
                    bound_mismatch,
                    abs(reach_radius(reg['total'], speed, .5, v_max) - reg['reach_radius']))
                sa, sb = reg['sigma'][0], reg['sigma'][1]
                for a_max in sweep_a_max:
                    r = reach_radius(reg['total'], speed, a_max, v_max)
                    ins, _, _ = ellipse_probe(d[0], d[1], sa + r, sb + r, yaw)
                    sweep[a_max][h]['inside'].append(1. if ins else 0.)
                    sweep[a_max][h]['area'].append(math.pi * (sa + r) * (sb + r))
                for m in margins:
                    r = reg['reach_radius']
                    ins, _, _ = ellipse_probe(d[0], d[1], sa + r + m, sb + r + m, yaw)
                    margin_study[m][h]['inside'].append(1. if ins else 0.)
                    margin_study[m][h]['area'].append(math.pi * (sa + r + m) * (sb + r + m))

    def pack(store):
        packed = {}
        for h in HORIZONS:
            d = store.get(h)
            if not d or not d['inside']:
                continue
            row = dict(samples=len(d['inside']),
                       coverage=float(np.mean(d['inside'])),
                       area_m2=describe(d['area']))
            if 'error' in d:
                row['error_m'] = describe(d['error'])
            if 'boundary' in d:
                row['boundary_distance_m'] = describe(d['boundary'])
            if 'reach_radius' in d:
                row['reach_radius_m'] = describe(d['reach_radius'])
                row['speed_capped_fraction'] = float(np.mean(d['capped']))
            row['coverage_per_m2'] = (row['coverage'] / row['area_m2']['mean']
                                      if row['area_m2'] and row['area_m2']['mean'] > 0 else None)
            packed[f'{h:g}'] = row
        return packed

    out['cv'] = pack(cv)
    out['cv_as_consumed'] = pack(cvc)
    out['cv_fresh'] = pack(cv_fresh)
    out['cv_coasting'] = pack(cv_coast)
    out['cv_coasting_as_consumed'] = pack(cvc_coast)
    out['reachability'] = pack(rc)
    out['reachability_fresh'] = pack(rc_fresh)
    out['reachability_coasting'] = pack(rc_coast)
    out['hybrid'] = pack(hy)
    out['hybrid_fresh'] = pack(hy_fresh)
    out['hybrid_coasting'] = pack(hy_coast)
    out['hybrid_policy_counts'] = {
        f'{h:g}': dict(cv=policy_counts[('cv', h)],
                       reachability=policy_counts[('reachability', h)])
        for h in HORIZONS}
    out['fresh_threshold_s'] = fresh_threshold
    out['a_max_sweep'] = {f'{a:g}': pack(sweep[a]) for a in sweep_a_max}
    out['cv_sigma_sweep'] = {f'{k:g}': pack(cv_sigma[k]) for k in cv_sigmas}
    out['cv_sigma_sweep_coasting_as_consumed'] = {
        f'{k:g}': pack(cv_sigma_coast[k]) for k in cv_sigmas}
    out['margin_sweep'] = {f'{m:g}': pack(margin_study[m]) for m in margins}
    out['invalid_regions_skipped'] = invalid_regions
    out['coasting_region_samples'] = coasting_samples
    out['observation_age_s'] = describe(ages)
    out['offline_bound_reproduction_error_m'] = bound_mismatch
    return out


def score_navigation(raw):
    """Focused navigation metrics for a `navigate: true` trial."""
    nav = raw.get('nav', {})
    scans = [s for s in raw.get('scans', []) if s.get('robot')]
    if not scans:
        return dict(requested=nav.get('requested', False), status=nav.get('status'),
                    note='no robot pose recorded')
    t = np.array([s['t'] for s in scans])
    pose = np.array([s['robot'] for s in scans])
    path = float(np.sum(np.hypot(np.diff(pose[:, 0]), np.diff(pose[:, 1]))))

    clearances = []
    for i, s in enumerate(scans):
        best = None
        for g in raw['gt']:
            p = interp_gt(g, s['t'])
            if p is None:
                continue
            c = footprint_clearance(pose[i, 0], pose[i, 1], pose[i, 2], p[0], p[1])
            best = c if best is None else min(best, c)
        if best is not None:
            clearances.append(best)
    clearances = np.array(clearances) if clearances else np.array([])

    # Speed profile from the recorded poses; a "stop" is a sustained near-zero
    # speed while the goal has not been reached.
    dt = np.diff(t)
    speed = np.concatenate([[0.], np.hypot(np.diff(pose[:, 0]), np.diff(pose[:, 1])) /
                            np.maximum(dt, 1e-3)])
    moving = speed > .05
    goal_time = None
    if len(t):
        reached = np.where(pose[:, 0] >= 2.75)[0]
        goal_time = float(t[reached[0]]) if len(reached) else None

    # Reaction: first time the robot's speed drops below 60% of its own
    # free-run nominal while an obstacle is within 2 m. Threshold mirrors the
    # Stage-4F speed_reaction_fraction so the number means the same thing.
    nominal = float(np.percentile(speed[moving], 75)) if moving.any() else 0.
    reaction = None
    for i in range(len(t)):
        if i >= len(clearances):
            break
        if clearances[i] < 2. and nominal > 0 and speed[i] < .6 * nominal:
            reaction = float(t[i])
            break

    return dict(
        requested=nav.get('requested', False),
        status=nav.get('status'),
        succeeded=nav.get('status') == 4,
        lifecycle_end=nav.get('lifecycle_end'),
        path_length_m=path,
        goal_reach_time_s=goal_time,
        min_clearance_m=float(clearances.min()) if len(clearances) else None,
        mean_clearance_m=float(clearances.mean()) if len(clearances) else None,
        collision=bool((clearances <= 0).any()) if len(clearances) else None,
        collision_frames=int((clearances <= 0).sum()) if len(clearances) else 0,
        stopped_frames=int((~moving).sum()),
        stopped_fraction=float((~moving).mean()) if len(moving) else None,
        nominal_speed_mps=nominal,
        reaction_time_s=reaction)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--logs', nargs='+', required=True,
                    help='trial directories, or directories of trial directories')
    ap.add_argument('--out', required=True)
    ap.add_argument('--a-max-sweep', default='0.25,0.5,1.0')
    ap.add_argument('--margin-sweep', default='0.0,0.1,0.2')
    ap.add_argument('--cv-sigma-sweep', default='1.0,1.5,2.0,3.0')
    ap.add_argument('--fresh-threshold', type=float, default=.1,
                    help='Stage-4G5 hybrid policy: observation age at or below which a track '
                         'uses CV. Default 0.1 s = half the measured 5 Hz scan interval.')
    ap.add_argument('--max-speed', type=float, default=.8)
    args = ap.parse_args()

    sweep = [float(x) for x in args.a_max_sweep.split(',')]
    margins = [float(x) for x in args.margin_sweep.split(',')]
    cv_sigmas = [float(x) for x in args.cv_sigma_sweep.split(',')]

    trials = []
    for root in args.logs:
        root = Path(root)
        candidates = [root] if (root / 'raw.json').exists() else sorted(
            d for d in root.iterdir() if (d / 'raw.json').exists())
        trials.extend(candidates)
    if not trials:
        raise SystemExit('no trial directories containing raw.json were found')

    results = []
    for trial in trials:
        raw = json.loads((trial / 'raw.json').read_text())
        row = score_trial(raw, sweep, args.max_speed, margins, cv_sigmas,
                          args.fresh_threshold)
        row['trial'] = trial.name
        row['source'] = str(trial)
        if raw.get('definition', {}).get('navigate'):
            row['navigation'] = score_navigation(raw)
        results.append(row)
        print(f"scored {trial.name}: {row['frames']} frames, "
              f"{row['coasting_region_samples']} coasting region samples", flush=True)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(
        conventions=dict(sigma_k=SIGMA_K, gate_m=GATE, horizons_s=list(HORIZONS),
                         a_max_sweep=sweep, margin_sweep=margins, cv_sigma_sweep=cv_sigmas,
                         max_speed=args.max_speed, fresh_threshold_s=args.fresh_threshold,
                         note='CV region is the k-sigma covariance ellipse; reach region is '
                              'that ellipse Minkowski-inflated by the deterministic bound. '
                              'Each is scored against GT at the absolute time it claims.'),
        trials=results), indent=1) + '\n')
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
