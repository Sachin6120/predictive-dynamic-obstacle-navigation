#!/usr/bin/env python3
"""Stage-4G5 transition analysis across the CV -> reachability -> CV switch.

Walks every coasting episode in a recording and reports, frame by frame, what
the hybrid policy selected and what that selection cost: the region's centroid,
area and maximum extent, the costmap cells actually painted on that cycle, and
the frame-to-frame discontinuity at each switch.

Ground truth is not used here at all -- this describes what the system did, not
how right it was. Coverage correctness lives in evaluate.py.

The costmap cell count is the count for the WHOLE grid on that cycle, so it is
attributable to one track only in single-target episodes; multi-track frames are
flagged so the number is never read as per-track when it is not.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

HORIZON_KEY = 0.5          # the horizon the transition is characterised at


def region_at(track, horizon, policy):
    """Centroid / area / max extent of the region the policy actually selected."""
    if policy == 'cv':
        for p in track.get('predictions', []):
            if abs(p['dt'] - horizon) < 1e-6:
                cov = p['covariance']
                # 2x2 symmetric eigenvalues, k-sigma semi-axes (k = 2, matching
                # the layer's sigma_level convention used throughout Stage-4G4).
                a, b, c, d = cov[0], .5 * (cov[1] + cov[2]), .5 * (cov[1] + cov[2]), cov[3]
                tr, det = a + d, a * d - b * b
                disc = max(tr * tr / 4 - det, 0.)
                l1, l2 = tr / 2 + math.sqrt(disc), max(tr / 2 - math.sqrt(disc), 0.)
                sa, sb = 2. * math.sqrt(l1), 2. * math.sqrt(l2)
                return dict(center=p['p'], semi_major=sa, semi_minor=sb,
                            area=math.pi * sa * sb)
        return None
    for r in track.get('reachability', []):
        if abs(r['dt'] - horizon) < 1e-6:
            if not r['valid']:
                return None
            sa, sb = r['axes']
            return dict(center=r['p'], semi_major=sa, semi_minor=sb, area=math.pi * sa * sb)
    return None


def analyse(raw, fresh_threshold):
    t0 = raw['t0']
    grids = sorted(raw.get('grids', []), key=lambda g: g['t'])

    def cells_near(t):
        """Painted cells on the costmap cycle closest to this scan."""
        if not grids:
            return None
        g = min(grids, key=lambda g: abs(g['t'] - t))
        return len(g['cells']) if abs(g['t'] - t) < .3 else None

    # Per-track timeline of policy + region.
    timeline = defaultdict(list)
    seen_ids = set()
    for frame in raw['arrays']:
        t = frame['t']
        ids_now = {tr['id'] for tr in frame['tracks']}
        for tr in frame['tracks']:
            age = tr['reachability'][0]['age'] if tr.get('reachability') else 0.
            policy = 'cv' if age <= fresh_threshold else 'reachability'
            reg = region_at(tr, HORIZON_KEY, policy)
            timeline[tr['id']].append(dict(
                t=round(t - t0, 4), id=tr['id'], observation_age=round(age, 4),
                missed=tr['missed'], policy=policy,
                center=[round(x, 4) for x in reg['center']] if reg else None,
                area_m2=round(reg['area'], 5) if reg else None,
                max_extent_m=round(reg['semi_major'], 5) if reg else None,
                costmap_cells=cells_near(t),
                concurrent_tracks=len(ids_now)))
        seen_ids |= ids_now

    # Episodes: a maximal run of coasting frames, with the fresh frame on each
    # side so the two switches are visible.
    episodes = []
    for tid, rows in sorted(timeline.items()):
        i = 0
        while i < len(rows):
            if rows[i]['policy'] != 'reachability':
                i += 1
                continue
            j = i
            while j < len(rows) and rows[j]['policy'] == 'reachability':
                j += 1
            before = rows[i - 1] if i > 0 else None
            after = rows[j] if j < len(rows) else None

            def delta(a, b):
                if not a or not b or not a['center'] or not b['center']:
                    return None
                return dict(
                    centroid_jump_m=round(math.dist(a['center'], b['center']), 5),
                    area_ratio=round(b['area_m2'] / a['area_m2'], 4)
                    if a['area_m2'] else None,
                    extent_ratio=round(b['max_extent_m'] / a['max_extent_m'], 4)
                    if a['max_extent_m'] else None,
                    cell_delta=(b['costmap_cells'] - a['costmap_cells'])
                    if (a['costmap_cells'] is not None and b['costmap_cells'] is not None)
                    else None)

            episodes.append(dict(
                track_id=tid,
                missed_scans=len(rows[i:j]),
                start_t=rows[i]['t'],
                end_t=rows[j - 1]['t'],
                reacquired=after is not None,
                expired=after is None,
                frame_before_switch=before,
                coasting_frames=rows[i:j],
                frame_after_reacquisition=after,
                cv_to_reachability=delta(before, rows[i]),
                reachability_to_cv=delta(rows[j - 1], after)))
            i = j

    # Did a track vanish and come back with a NEW id? That is expiry, not a
    # reacquisition, and it is reported separately.
    first_last = {tid: (rows[0]['t'], rows[-1]['t']) for tid, rows in timeline.items()}
    return dict(
        scenario=raw['scenario'], layer_mode=raw.get('layer_mode', 'keep'),
        fresh_threshold_s=fresh_threshold,
        horizon_analysed_s=HORIZON_KEY,
        track_ids=sorted(seen_ids),
        track_spans={str(k): [round(a - t0, 3), round(b - t0, 3)]
                     for k, (a, b) in sorted(first_last.items())},
        policy_frames={
            'cv': sum(1 for rows in timeline.values() for r in rows if r['policy'] == 'cv'),
            'reachability': sum(1 for rows in timeline.values()
                                for r in rows if r['policy'] == 'reachability')},
        episodes=episodes)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--logs', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--fresh-threshold', type=float, default=.1)
    args = ap.parse_args()

    trials = []
    for root in args.logs:
        root = Path(root)
        trials.extend([root] if (root / 'raw.json').exists() else
                      sorted(d for d in root.iterdir() if (d / 'raw.json').exists()))

    out = []
    for trial in trials:
        raw = json.loads((trial / 'raw.json').read_text())
        row = analyse(raw, args.fresh_threshold)
        row['trial'] = trial.name
        out.append(row)
        print(f"{trial.name}: {len(row['episodes'])} coasting episode(s), "
              f"cv {row['policy_frames']['cv']} / reach {row['policy_frames']['reachability']} "
              f"track-frames", flush=True)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(
        note='Per-track hybrid policy timeline and the discontinuity at each switch. '
             'costmap_cells is the whole-grid count on the nearest costmap cycle, so it is '
             'attributable to one track only where concurrent_tracks == 1.',
        trials=out), indent=1) + '\n')
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
