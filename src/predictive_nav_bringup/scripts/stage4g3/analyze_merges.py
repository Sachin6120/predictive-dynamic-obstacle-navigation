#!/usr/bin/env python3
"""Stage-4G3 merge inspection: where the Stage-4G2 close crossing actually failed.

Answers the question Stage-4G3 must answer before changing any tracker code:
when two objects fall into one LiDAR cluster, does that cluster still contain
geometrically separable returns from BOTH objects (case A, deblendable), or is
one object physically occluded (case B, an observability gap that only track
coasting can cover)?

It re-derives the tracker's perception front-end offline from recorded scans
(see frontend.py) and, critically, VERIFIES that reconstruction against the
centroids the runtime tracker actually published before trusting any number.

The cylinder surface attribution used to label returns is a DIAGNOSTIC only.
It never reaches the tracker, and the deblending rule it justifies uses only
LiDAR points and the tracker's own Kalman predictions.

Usage:
  python3 analyze_merges.py --logs validation_logs/stage4g2 \
      --map src/predictive_nav_bringup/maps/stage4f_benchmark.yaml \
      --out validation/stage4g3/merge_inspection.json
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from frontend import P, load_map, dynamic_points, clusters_of

# Radius of the simulated cylinders, used ONLY to attribute recorded returns to
# a surface for diagnosis. Not a tracker parameter and never used at runtime.
SURFACE_RADIUS = 0.20
SURFACE_TOLERANCE = 0.10
# A GT object counts as "present" in a component only with >= 2 returns, so a
# single stray point cannot manufacture merge evidence.
MIN_RETURNS_FOR_PRESENCE = 2


def trials(logs):
    return sorted(Path(logs).glob('*/*/raw.json'))


def verify_frontend(paths, gmap):
    """The offline front-end must reproduce the runtime centroids exactly."""
    frames = exact = count_ok = 0
    worst = 0.0
    for path in paths:
        raw = json.loads(path.read_text())
        runtime = raw.get('clusters', {})
        for scan in raw['scans']:
            key = f"{scan['t']:.6f}"
            if key not in runtime:
                continue
            frames += 1
            pts = dynamic_points(scan['points'], scan['sensor'], gmap)
            mine = np.array([c[1]['centroid'] for c in clusters_of(pts)]) if len(pts) \
                else np.empty((0, 2))
            theirs = np.array(runtime[key]).reshape(-1, 2)
            if len(mine) != len(theirs):
                continue
            count_ok += 1
            if not len(mine):
                exact += 1
                continue
            d = np.linalg.norm(mine[:, None, :] - theirs[None, :, :], axis=2)
            err = float(d.min(axis=1).max())
            worst = max(worst, err)
            if err < 0.02:
                exact += 1
    return dict(frames=frames, cluster_count_match=count_ok,
                centroid_match_within_2cm=exact, worst_centroid_error_m=worst)


def gt_at(gt, t):
    return np.array([[np.interp(t, g[:, 0], g[:, k]) for k in (1, 2)] for g in gt])


def attribute(pts, centers):
    residual = np.abs(np.linalg.norm(pts[:, None, :] - centers[None, :, :], axis=2)
                      - SURFACE_RADIUS)
    return residual.argmin(axis=1), residual.min(axis=1) < SURFACE_TOLERANCE


def max_nn_gap(points):
    if len(points) < 2:
        return 0.0
    d = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    return float(np.max(d.min(axis=1)))


def describe(values):
    a = np.array(values, float)
    if not len(a):
        return None
    return dict(n=int(len(a)), min=float(a.min()),
                p50=float(np.percentile(a, 50)), p95=float(np.percentile(a, 95)),
                max=float(a.max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    gmap = load_map(Path(args.map))
    paths = trials(args.logs)
    fidelity = verify_frontend(paths, gmap)

    single_diam, single_pts, merged_diam, merged_pts = [], [], [], []
    within_gaps, cross_gaps = [], []
    timelines = {}
    case_counts = Counter()

    for path in paths:
        raw = json.loads(path.read_text())
        if not raw['gt']:
            continue
        name = path.parent.name
        gt = [np.array(g) for g in raw['gt']]
        rows = []
        for scan in raw['scans']:
            t = scan['t']
            if not (raw['t0'] <= t <= raw['end']):
                continue
            centers = gt_at(gt, t)
            pts = dynamic_points(scan['points'], scan['sensor'], gmap)
            if not len(pts):
                continue
            labels, on_surface = attribute(pts, centers)
            visible = [int(np.sum(on_surface & (labels == i))) for i in range(len(gt))]
            for idx, stats in clusters_of(pts):
                lab, ok = labels[idx], on_surface[idx]
                present = [i for i in range(len(gt))
                           if np.sum(ok & (lab == i)) >= MIN_RETURNS_FOR_PRESENCE]
                if len(present) >= 2:
                    merged_diam.append(stats['diameter'])
                    merged_pts.append(stats['n'])
                    member = pts[idx]
                    a = member[ok & (lab == present[0])]
                    b = member[ok & (lab == present[1])]
                    cross = float(np.min(np.linalg.norm(a[:, None, :] - b[None, :, :],
                                                        axis=2)))
                    within = max(max_nn_gap(a), max_nn_gap(b))
                    cross_gaps.append(cross)
                    within_gaps.append(within)
                    case_counts['A_observable_merge'] += 1
                    rows.append(dict(t=round(t - raw['t0'], 3), case='A',
                                     diameter=round(stats['diameter'], 4),
                                     points=stats['n'], visible=visible,
                                     cross_gap_m=round(cross, 4),
                                     within_gap_m=round(within, 4),
                                     true_separation_m=round(
                                         float(np.linalg.norm(
                                             centers[present[0]] - centers[present[1]])), 4)))
                elif len(present) == 1 and len(gt) > 1:
                    single_diam.append(stats['diameter'])
                    single_pts.append(stats['n'])
                    hidden = [i for i in range(len(gt)) if i not in present]
                    # Case B is a cluster holding one object while another GT
                    # object is close enough to have merged but returns nothing.
                    near = [i for i in hidden
                            if np.linalg.norm(centers[i] - centers[present[0]]) < 0.9]
                    if near and all(visible[i] < MIN_RETURNS_FOR_PRESENCE for i in near):
                        case_counts['B_true_occlusion'] += 1
                        rows.append(dict(t=round(t - raw['t0'], 3), case='B',
                                         diameter=round(stats['diameter'], 4),
                                         points=stats['n'], visible=visible,
                                         true_separation_m=round(float(np.linalg.norm(
                                             centers[near[0]] - centers[present[0]])), 4)))
                elif len(present) == 1:
                    single_diam.append(stats['diameter'])
                    single_pts.append(stats['n'])
        if rows:
            timelines[name] = rows

    sd = np.array(single_diam)
    md = np.array(merged_diam)
    separation = {}
    for thr in (0.50, 0.55, 0.60, 0.65, 0.70, 0.75):
        separation[f'{thr:.2f}'] = dict(
            single_object_clusters_flagged=float(np.mean(sd > thr)) if len(sd) else None,
            merged_clusters_flagged=float(np.mean(md > thr)) if len(md) else None)

    result = dict(
        note='Offline reconstruction of the runtime perception front-end over the '
             'recorded Stage-4G2 scans. Surface attribution is diagnostic only and '
             'never reaches the tracker.',
        frontend_fidelity=fidelity,
        frontend_parameters=P,
        single_object_cluster=dict(diameter_m=describe(single_diam),
                                   points=describe(single_pts)),
        merged_cluster=dict(diameter_m=describe(merged_diam),
                            points=describe(merged_pts)),
        gap_structure=dict(
            max_gap_between_returns_on_one_object_m=describe(within_gaps),
            min_gap_between_two_merged_objects_m=describe(cross_gaps),
            frames_where_cross_gap_exceeds_within_gap=int(
                np.sum(np.array(cross_gaps) > np.array(within_gaps))) if cross_gaps else 0,
            frames_total=len(cross_gaps)),
        diameter_threshold_separation=separation,
        case_counts=dict(case_counts),
        timelines=timelines)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'timelines'}, indent=1))


if __name__ == '__main__':
    main()
