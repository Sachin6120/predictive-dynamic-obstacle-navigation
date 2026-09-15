#!/usr/bin/env python3
"""Stage-4G7 per-trial extraction: safety, navigation, and the causal chain.

One row per trial. The target is ground-truth object 0 in every Stage-4G7
scenario -- the obstacle hidden by static geometry, and the only one whose
predicted region differs between the arms. Clearance is reported BOTH
target-specific and global, and the identity of the clearance-limiting object is
recorded, because Stage-4G6 showed a global-only metric can credit prediction
for proximity to an obstacle prediction never touched.

The chain fields (cells/speed inside the observation gap) exist to test whether
Hybrid actually changes the costmap and the robot's motion BEFORE the target is
re-observed. If it does not, a coverage advantage cannot have caused a safety
advantage, whatever the clearance numbers say.
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
NEAR_MISS = .10
HORIZON = .5
FRESH_THRESHOLD = .1
GATE = .45


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


def inside(dx, dy, a, b, yaw):
    if a <= 0 or b <= 0:
        return False
    c, s = math.cos(yaw), math.sin(yaw)
    return ((dx * c + dy * s) / a) ** 2 + ((-dx * s + dy * c) / b) ** 2 <= 1.


def analyse(raw):
    t0 = raw['t0']
    gt = raw['gt']
    scans = [s for s in raw.get('scans', []) if s.get('robot')]
    row = dict(scenario=raw['scenario'], layer_mode=raw.get('layer_mode', 'keep'),
               world=raw.get('world'), nav_status=raw.get('nav', {}).get('status'),
               succeeded=raw.get('nav', {}).get('status') == 4, frames=len(raw['arrays']))
    if not scans or not gt:
        row['setup_failure'] = 'no robot pose or no ground truth'
        return row

    t = np.array([s['t'] for s in scans]) - t0
    pose = np.array([s['robot'] for s in scans])
    step = np.hypot(np.diff(pose[:, 0]), np.diff(pose[:, 1]))
    speed = np.concatenate([[0.], step / np.maximum(np.diff(t), 1e-3)])
    row['path_length_m'] = float(step.sum())

    # ---- clearance: global, target-specific, and who limits it -------------
    glob, tgt, limiter = [], [], []
    for i, s in enumerate(scans):
        best, best_i, best_t = None, None, None
        for gi, g in enumerate(gt):
            p = interp_gt(g, s['t'])
            if p is None:
                continue
            c = clearance(pose[i, 0], pose[i, 1], pose[i, 2], p[0], p[1])
            if best is None or c < best:
                best, best_i = c, gi
            if gi == 0:
                best_t = c
        glob.append(best if best is not None else np.nan)
        tgt.append(best_t if best_t is not None else np.nan)
        limiter.append(best_i)
    glob = np.array(glob, float)
    tgt = np.array(tgt, float)
    gf, tf = glob[np.isfinite(glob)], tgt[np.isfinite(tgt)]
    row['min_clearance_m'] = float(gf.min()) if len(gf) else None
    row['min_clearance_target_m'] = float(tf.min()) if len(tf) else None
    row['mean_clearance_target_m'] = float(tf.mean()) if len(tf) else None
    row['collision'] = bool((gf <= 0).any()) if len(gf) else None
    row['target_collision'] = bool((tf <= 0).any()) if len(tf) else None
    row['near_miss_target'] = bool((tf <= NEAR_MISS).any()) if len(tf) else None
    row['min_clearance_limited_by_object'] = (
        int(limiter[int(np.nanargmin(glob))]) if len(gf) else None)
    row['target_is_limiting'] = row['min_clearance_limited_by_object'] == 0
    t_min = float(t[int(np.nanargmin(tgt))]) if len(tf) else None
    row['time_of_min_target_clearance_s'] = t_min

    # ---- navigation -------------------------------------------------------
    reached = np.where(pose[:, 0] >= GOAL_X)[0]
    row['goal_reach_time_s'] = float(t[reached[0]]) if len(reached) else None
    moving = speed > STOP_SPEED
    row['stopped_fraction'] = float((~moving).mean())
    row['stopped_duration_s'] = float(np.sum(np.diff(t)[~moving[1:]]))
    started = int(np.argmax(moving)) if moving.any() else 0
    m = moving[started:]
    row['stop_count'] = int(np.sum((~m[1:]) & (m[:-1])))
    nominal = float(np.percentile(speed[moving], 75)) if moving.any() else 0.
    row['nominal_speed_mps'] = nominal
    row['reaction_time_s'] = row['reaction_distance_m'] = row['reaction_lead_s'] = None
    if nominal > 0:
        for i in range(len(t)):
            if np.isfinite(tgt[i]) and tgt[i] < REACTION_RANGE and \
               speed[i] < REACTION_FRACTION * nominal:
                row['reaction_time_s'] = float(t[i])
                row['reaction_distance_m'] = float(tgt[i])
                if t_min is not None:
                    row['reaction_lead_s'] = float(t_min - t[i])
                break

    # ---- the target's track, and the observation gap -----------------------
    coast_times, obs_before, ids = [], 0, set()
    cov_cv = cov_cv_n = cov_rc = cov_rc_n = 0
    for frame in raw['arrays']:
        ft = frame['t']
        truth = interp_gt(gt[0], ft)
        if truth is None or not frame['tracks']:
            continue
        near = min(frame['tracks'], key=lambda tr: math.dist(tr['p'], truth))
        if math.dist(near['p'], truth) > GATE:
            continue
        ids.add(near['id'])
        age = near['reachability'][0]['age'] if near.get('reachability') else 0.
        if age > FRESH_THRESHOLD:
            coast_times.append(ft - t0)
            # Coverage of BOTH representations on the SAME coasting frame: the CV
            # region as a consumer reads it, and the reachable set.
            gtp = interp_gt(gt[0], ft + HORIZON)
            if gtp is not None:
                for pr in near.get('predictions', []):
                    if abs(pr['dt'] - HORIZON) > 1e-6:
                        continue
                    cv = np.array(pr['covariance']).reshape(2, 2)
                    cv = .5 * (cv + cv.T)
                    w, vec = np.linalg.eigh(cv)
                    w = np.maximum(w, 0.)
                    cov_cv_n += 1
                    cov_cv += inside(gtp[0] - pr['p'][0], gtp[1] - pr['p'][1],
                                     2 * math.sqrt(w[1]), 2 * math.sqrt(w[0]),
                                     math.atan2(vec[1, 1], vec[0, 1]))
                for r in near.get('reachability', []):
                    if abs(r['dt'] - HORIZON) > 1e-6 or not r['valid']:
                        continue
                    g2 = interp_gt(gt[0], r['stamp'])
                    if g2 is None:
                        continue
                    cov_rc_n += 1
                    cov_rc += inside(g2[0] - r['p'][0], g2[1] - r['p'][1],
                                     r['axes'][0], r['axes'][1], r['sigma'][2])
        else:
            obs_before += 1 if not coast_times else 0
    runs, cur = [], []
    for ts in coast_times:
        if cur and ts - cur[-1] > .35:
            runs.append(cur)
            cur = []
        cur.append(ts)
    if cur:
        runs.append(cur)
    longest = max(runs, key=len) if runs else []
    row['target_track_ids'] = sorted(ids)
    row['target_expired'] = len(ids) > 1
    row['observations_before_gap'] = obs_before
    row['coasting_frames'] = len(coast_times)
    row['gap_runs'] = len(runs)
    row['gap_start_s'] = longest[0] if longest else None
    row['gap_end_s'] = longest[-1] if longest else None
    row['gap_duration_s'] = (longest[-1] - longest[0] + .2) if longest else 0.
    row['cv_coasting_coverage'] = (cov_cv / cov_cv_n) if cov_cv_n else None
    row['reach_coasting_coverage'] = (cov_rc / cov_rc_n) if cov_rc_n else None
    row['coasting_coverage_samples'] = cov_cv_n
    row['min_clearance_in_gap'] = bool(
        longest and t_min is not None and longest[0] - .3 <= t_min <= longest[-1] + .7)
    # conflict_overlap: fraction of the pre-conflict interval spent unobserved.
    if longest and t_min is not None and t_min > longest[0]:
        row['conflict_overlap'] = min(row['gap_duration_s'] / (t_min - longest[0]), 1.)
    else:
        row['conflict_overlap'] = 0.
    row['reacquire_to_min_clearance_s'] = (
        (t_min - longest[-1]) if (longest and t_min is not None) else None)

    # ---- THE CHAIN: costmap and command inside the gap ---------------------
    grids = sorted(raw.get('grids', []), key=lambda g: g['t'])
    if longest:
        lo, hi = longest[0] + t0 - .1, longest[-1] + t0 + .1
        win = [g for g in grids if lo <= g['t'] <= hi]
        row['gap_costmap_grids'] = len(win)
        row['gap_costmap_cells_mean'] = float(np.mean([len(g['cells']) for g in win])) \
            if win else None
        row['gap_costmap_cells_max'] = int(max((len(g['cells']) for g in win), default=0))
        sel = (t >= longest[0] - .1) & (t <= longest[-1] + .1)
        row['gap_speed_mean'] = float(speed[sel].mean()) if sel.any() else None
        row['gap_speed_min'] = float(speed[sel].min()) if sel.any() else None
        # The window between gap end and closest approach: has the robot already
        # committed by then?
        if t_min is not None and t_min > longest[-1]:
            sel2 = (t > longest[-1]) & (t <= t_min)
            row['post_gap_speed_mean'] = float(speed[sel2].mean()) if sel2.any() else None
    else:
        row['gap_costmap_grids'] = 0
        row['gap_costmap_cells_mean'] = row['gap_costmap_cells_max'] = None
        row['gap_speed_mean'] = row['gap_speed_min'] = None
    row['costmap_cells_mean_all'] = float(np.mean([len(g['cells']) for g in grids])) \
        if grids else None
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
                d for d in root.iterdir() if d.is_dir())
        for trial in cands:
            if (trial / 'setup_failure.json').exists() and not (trial / 'raw.json').exists():
                f = json.loads((trial / 'setup_failure.json').read_text())
                f['trial'] = trial.name
                failures.append(f)
                continue
            if not (trial / 'raw.json').exists():
                continue
            row = analyse(json.loads((trial / 'raw.json').read_text()))
            row['trial'] = trial.name
            row['source'] = str(trial)
            trials.append(row)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(
        definitions=dict(target='ground-truth object 0 (the occluded obstacle)',
                         gate_m=GATE, near_miss_m=NEAR_MISS, horizon_s=HORIZON,
                         fresh_threshold_s=FRESH_THRESHOLD, goal_x=GOAL_X,
                         clearance='oriented footprint box vs obstacle cylinder, '
                                   'identical to stage4f_trial.py'),
        n_trials=len(trials), n_setup_failures=len(failures),
        setup_failures=failures, trials=trials), indent=1) + '\n')
    print(f'{len(trials)} trials, {len(failures)} setup failures -> {args.out}')


if __name__ == '__main__':
    main()
