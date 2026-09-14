#!/usr/bin/env python3
"""Offline reconstruction of the runtime tracker perception front-end.

Reimplements, in Python, exactly what lidar_obstacle_tracker_node.cpp does:
  1. range-aware static rejection against an EDT of the occupancy grid
  2. 0.35 m single-link connected-component clustering with the same
     min/max point and max-diameter filters

Fidelity is not assumed: reconstruct() is checked against the centroids the
RUNTIME tracker actually published (raw.json['clusters']) before any
measurement derived from it is trusted.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt

# tracker_params.yaml, Stage-4G2 values (frozen).
P = dict(static_reject_radius=.15, range_aware=True, base_static_margin=.06,
         localization_margin=.06, angular_sampling_scale=.030,
         max_static_reject_radius=.40, occupied_threshold=65,
         unknown_static=False, cluster_distance=.35, cluster_min_points=3,
         cluster_max_points=200, cluster_max_diameter=1.2)


def load_map(yaml_path):
    import yaml
    meta = yaml.safe_load(open(yaml_path))
    pgm = yaml_path.parent / meta['image']
    with open(pgm, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # map_server: occupancy = (1 - (pixel/maxv)) scaled to 0..100, row 0 of the
    # PGM is the TOP row, i.e. the highest y in map coordinates.
    occ = (1.0 - img.astype(float) / maxv) * 100.0
    occ = np.flipud(occ)
    static = occ >= P['occupied_threshold']
    res, origin = meta['resolution'], meta['origin']
    # Exact Euclidean distance (in metres) from each cell to the nearest static
    # cell -- the same quantity the node's dt_1d/build_static_distance_field
    # computes at runtime.
    dist = distance_transform_edt(~static, sampling=res)
    return dict(dist=dist, res=res, ox=origin[0], oy=origin[1],
                w=w, h=h, static=static)


def reject_radius(rng):
    if not P['range_aware']:
        return P['static_reject_radius']
    r = P['base_static_margin'] + P['localization_margin'] + \
        P['angular_sampling_scale'] * rng
    return min(max(r, P['static_reject_radius']), P['max_static_reject_radius'])


def dynamic_points(points, sensor, gmap):
    """Apply range-aware static rejection to map-frame scan endpoints."""
    pts = np.asarray(points, float)
    if not len(pts):
        return pts
    rng = np.linalg.norm(pts - np.asarray(sensor, float)[None, :], axis=1)
    mx = np.floor((pts[:, 0] - gmap['ox']) / gmap['res']).astype(int)
    my = np.floor((pts[:, 1] - gmap['oy']) / gmap['res']).astype(int)
    inside = (mx >= 0) & (mx < gmap['w']) & (my >= 0) & (my < gmap['h'])
    d = np.full(len(pts), np.inf)
    d[inside] = gmap['dist'][my[inside], mx[inside]]
    # Outside the map is never rejected, matching is_static_point()'s
    # out-of-bounds -> false.
    keep = ~(d <= np.array([reject_radius(r) for r in rng]))
    return pts[keep]


def components(pts, link=None):
    """Single-link connected components, identical semantics to cluster_points."""
    link = P['cluster_distance'] if link is None else link
    n = len(pts)
    visited = np.zeros(n, bool)
    out = []
    for seed in range(n):
        if visited[seed]:
            continue
        visited[seed] = True
        idx = [seed]
        k = 0
        while k < len(idx):
            cur = idx[k]
            k += 1
            reach = np.flatnonzero((~visited) &
                                   (np.linalg.norm(pts - pts[cur], axis=1) <= link))
            for j in reach:
                visited[j] = True
                idx.append(int(j))
        out.append(np.array(idx))
    return out


def cluster_stats(pts, idx):
    m = pts[idx]
    ext = m.max(axis=0) - m.min(axis=0)
    return dict(n=len(idx), centroid=m.mean(axis=0).tolist(),
                diameter=float(np.hypot(*ext)), extent_x=float(ext[0]),
                extent_y=float(ext[1]),
                spread=float(np.max(np.linalg.norm(
                    m - m.mean(axis=0), axis=1))) if len(m) else 0.0)


def clusters_of(pts):
    """Accepted clusters (post filters) as (indices, stats) pairs."""
    res = []
    for idx in components(pts):
        if not (P['cluster_min_points'] <= len(idx) <= P['cluster_max_points']):
            continue
        st = cluster_stats(pts, idx)
        if st['diameter'] > P['cluster_max_diameter']:
            continue
        res.append((idx, st))
    return res
