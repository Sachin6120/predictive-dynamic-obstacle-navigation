#!/usr/bin/env python3
"""Stage-4G8 final-robustness extraction: one row per trial.

Merges the two evidence sources already produced per trial rather than
recomputing either:
  * summary.json  - the UNCHANGED Stage-4G2 scorer (ID switches, purity,
    matched fraction, position/velocity error, false confirmed/persistent
    tracks, failure categories, costmap representation, prediction checks).
  * raw.json      - recording, from which navigation, clearance, reaction and
    prediction ADE/FDE are derived here.

Coasting is detected from the tracker's own missed_count, not from the
reachability field, because the FINAL PRODUCTION configuration does not publish
reachability at all.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

ROBOT = dict(cx=-.064, hx=.1325, hy=.153)
OBSTACLE_RADIUS = .20
GOAL_X = 2.75
STOP_SPEED = .05
REACTION_FRACTION = .60
REACTION_RANGE = 2.0
GATE = .45
HORIZONS = (.5, 1., 2., 3.)


def clearance(rx, ry, ryaw, ox, oy):
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    dx, dy = ox - rx, oy - ry
    lx = c * dx - s * dy - ROBOT['cx']
    ly = s * dx + c * dy
    return math.hypot(max(abs(lx) - ROBOT['hx'], 0.),
                      max(abs(ly) - ROBOT['hy'], 0.)) - OBSTACLE_RADIUS


def interp_gt(track, t):
    a = np.asarray(track)
    if len(a) < 2 or t < a[0, 0] or t > a[-1, 0]:
        return None
    return np.array([np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2])])


def analyse(raw, summary, prov):
    t0 = raw['t0']
    gt = raw['gt']
    row = dict(scenario=raw['scenario'], layer_mode=raw.get('layer_mode', 'keep'),
               world=raw.get('world', 'stage4f_benchmark'),
               production=bool(prov.get('production')),
               scan_noise_sigma=prov.get('scan_noise_sigma', 0.0),
               pose_offset=prov.get('pose_offset', [0, 0, 0]),
               jitter=prov.get('jitter', 0.0), trial_seed=prov.get('trial_seed'),
               nav_status=raw.get('nav', {}).get('status'),
               succeeded=raw.get('nav', {}).get('status') == 4,
               frames=len(raw['arrays']), gt_objects=len(gt))

    # ---- tracking quality, straight from the unchanged Stage-4G2 scorer ----
    if summary:
        row.update(
            id_switches=summary.get('id_switches'),
            track_fragments=summary.get('track_fragments'),
            distinct_confirmed_ids=summary.get('distinct_confirmed_ids'),
            false_confirmed_ids=len(summary.get('false_confirmed_ids', {})),
            false_persistent_ids=len(summary.get('false_persistent_ids', [])),
            duplicate_frames=summary.get('duplicate_frames'),
            categories=summary.get('categories'),
            max_missed_scans=summary.get('max_missed_scans_retained'),
            costmap_grids=summary.get('costmap_grid_frames'),
            costmap_nonzero=summary.get('costmap_nonzero_frames'),
            costmap_multi_eligible=summary.get('costmap_multi_eligible_frames'),
            costmap_multi_represented=summary.get('costmap_multi_represented_frames'),
            prediction_samples=summary.get('prediction_samples_checked'),
            prediction_failures=summary.get('prediction_math_failures'))
        per = summary.get('per_gt', [])
        pe = [g['position_error_m']['mean'] for g in per if g.get('position_error_m')]
        ve = [g['velocity_error_mps']['mean'] for g in per if g.get('velocity_error_mps')]
        pu = [g['purity'] for g in per if g.get('purity') is not None]
        mf = [g['matched_frame_fraction'] for g in per
              if g.get('matched_frame_fraction') is not None]
        row.update(position_error_m=float(np.mean(pe)) if pe else None,
                   velocity_error_mps=float(np.mean(ve)) if ve else None,
                   purity=float(np.mean(pu)) if pu else None,
                   matched_fraction=float(np.mean(mf)) if mf else None)

    # ---- navigation / clearance -------------------------------------------
    scans = [s for s in raw.get('scans', []) if s.get('robot')]
    if scans and gt:
        t = np.array([s['t'] for s in scans]) - t0
        pose = np.array([s['robot'] for s in scans])
        step = np.hypot(np.diff(pose[:, 0]), np.diff(pose[:, 1]))
        speed = np.concatenate([[0.], step / np.maximum(np.diff(t), 1e-3)])
        row['path_length_m'] = float(step.sum())
        clear = []
        for i, s in enumerate(scans):
            best = None
            for g in gt:
                p = interp_gt(g, s['t'])
                if p is None:
                    continue
                c = clearance(pose[i, 0], pose[i, 1], pose[i, 2], p[0], p[1])
                best = c if best is None else min(best, c)
            clear.append(best if best is not None else np.nan)
        clear = np.array(clear, float)
        cf = clear[np.isfinite(clear)]
        row['min_clearance_m'] = float(cf.min()) if len(cf) else None
        row['mean_clearance_m'] = float(cf.mean()) if len(cf) else None
        row['collision'] = bool((cf <= 0).any()) if len(cf) else None
        t_min = float(t[int(np.nanargmin(clear))]) if len(cf) else None
        reached = np.where(pose[:, 0] >= GOAL_X)[0]
        row['goal_reach_time_s'] = float(t[reached[0]]) if len(reached) else None
        moving = speed > STOP_SPEED
        row['stopped_fraction'] = float((~moving).mean())
        row['stopped_duration_s'] = float(np.sum(np.diff(t)[~moving[1:]]))
        started = int(np.argmax(moving)) if moving.any() else 0
        m = moving[started:]
        row['stop_count'] = int(np.sum((~m[1:]) & (m[:-1])))
        nominal = float(np.percentile(speed[moving], 75)) if moving.any() else 0.
        row['reaction_time_s'] = row['reaction_lead_s'] = None
        if nominal > 0:
            for i in range(len(t)):
                if np.isfinite(clear[i]) and clear[i] < REACTION_RANGE and \
                   speed[i] < REACTION_FRACTION * nominal:
                    row['reaction_time_s'] = float(t[i])
                    if t_min is not None:
                        row['reaction_lead_s'] = float(t_min - t[i])
                    break

    # ---- coasting (from missed_count) and CV prediction ADE/FDE ------------
    coast = 0
    ade, fde = [], []
    for frame in raw['arrays']:
        for tr in frame['tracks']:
            if tr.get('missed', 0) > 0:
                coast += 1
            truth_now = None
            best, bi = GATE, None
            for gi, g in enumerate(gt):
                p = interp_gt(g, frame['t'])
                if p is None:
                    continue
                dd = math.dist(tr['p'], p)
                if dd < best:
                    best, bi, truth_now = dd, gi, p
            if bi is None:
                continue
            for pr in tr.get('predictions', []):
                h = round(pr['dt'], 3)
                if h not in HORIZONS:
                    continue
                truth = interp_gt(gt[bi], pr['stamp'])
                if truth is None:
                    continue
                err = math.dist(pr['p'], truth)
                ade.append(err)
                if abs(h - 3.0) < 1e-6:
                    fde.append(err)
    row['coasting_frames'] = coast
    row['prediction_ade_m'] = float(np.mean(ade)) if ade else None
    row['prediction_fde_3s_m'] = float(np.mean(fde)) if fde else None
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--logs', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    trials, failures = [], []
    for root in args.logs:
        root = Path(root)
        cands = [root] if (root / 'raw.json').exists() or \
            (root / 'setup_failure.json').exists() else sorted(
                d for d in root.rglob('*') if d.is_dir() and
                ((d / 'raw.json').exists() or (d / 'setup_failure.json').exists()))
        for trial in cands:
            if (trial / 'setup_failure.json').exists() and not (trial / 'raw.json').exists():
                f = json.loads((trial / 'setup_failure.json').read_text())
                f['trial'] = trial.name
                failures.append(f)
                continue
            if not (trial / 'raw.json').exists():
                continue
            summary = json.loads((trial / 'summary.json').read_text()) \
                if (trial / 'summary.json').exists() else None
            prov = json.loads((trial / 'provenance.json').read_text()) \
                if (trial / 'provenance.json').exists() else {}
            row = analyse(json.loads((trial / 'raw.json').read_text()), summary, prov)
            row['trial'] = trial.name
            row['source'] = str(trial)
            trials.append(row)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(
        definitions=dict(goal_x=GOAL_X, gate_m=GATE, horizons_s=list(HORIZONS),
                         clearance='oriented footprint box vs obstacle cylinder, '
                                   'identical to stage4f_trial.py',
                         coasting='tracker missed_count > 0 (production publishes '
                                  'no reachability field)'),
        n_trials=len(trials), n_setup_failures=len(failures),
        setup_failures=failures, trials=trials), indent=1) + '\n')
    print(f'{len(trials)} trials, {len(failures)} setup failures -> {args.out}')


if __name__ == '__main__':
    main()
