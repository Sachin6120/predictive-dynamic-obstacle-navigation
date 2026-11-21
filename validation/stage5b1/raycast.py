#!/usr/bin/env python3
"""Deterministic ray-cast reference geometry.

Replicates, exactly:
  * the TB3 waffle LDS scan plane (360 beams, 6.28 rad span, nearest hit only)
  * build_clusters()  single-link connected components, cluster_distance 0.35
  * extractFeatures() length/thickness/fill_ratio/max_gap/angular_span

Used to characterise what the sensor CAN distinguish, with no Gazebo and no
ground-truth leakage into the measured quantities.
"""
import math
import numpy as np

N_BEAMS = 360
ANGLE_MIN = 0.0
ANGLE_MAX = 6.28
ANGLE_INC = (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1)
RANGE_MAX = 10.0

CLUSTER_DISTANCE = 0.35
CLUSTER_MIN_POINTS = 3
CLUSTER_MAX_POINTS = 200
CLUSTER_MAX_DIAMETER = 1.2
THICKNESS_FLOOR = 0.05


# ---------------------------------------------------------------- geometry
def box_segments(cx, cy, sx, sy, yaw):
    """World-frame edge segments of an axis-box at (cx,cy) rotated by yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = sx / 2.0, sy / 2.0
    local = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    pts = [(cx + c * lx - s * ly, cy + s * lx + c * ly) for lx, ly in local]
    return [(pts[i], pts[(i + 1) % 4]) for i in range(4)]


def circle_shape(cx, cy, r, n=64):
    pts = [(cx + r * math.cos(2 * math.pi * i / n),
            cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    return [(pts[i], pts[(i + 1) % n]) for i in range(n)]


def ray_segment(ox, oy, dx, dy, p, q):
    """Distance along ray to segment pq, or None."""
    ex, ey = q[0] - p[0], q[1] - p[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((p[0] - ox) * ey - (p[1] - oy) * ex) / den
    u = ((p[0] - ox) * dy - (p[1] - oy) * dx) / den
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def scan(sensor, segments, noise=0.0, rng=None):
    """Nearest-hit ray cast. Returns Nx2 world-frame endpoints."""
    ox, oy = sensor
    out = []
    for i in range(N_BEAMS):
        a = ANGLE_MIN + i * ANGLE_INC
        dx, dy = math.cos(a), math.sin(a)
        best = None
        for p, q in segments:
            t = ray_segment(ox, oy, dx, dy, p, q)
            if t is not None and t < RANGE_MAX and (best is None or t < best):
                best = t
        if best is None:
            continue
        if noise and rng is not None:
            best += rng.normal(0.0, noise)
        out.append((ox + best * dx, oy + best * dy))
    return np.array(out) if out else np.zeros((0, 2))


# ------------------------------------------------- frozen primary clustering
def build_clusters(pts, link=CLUSTER_DISTANCE):
    n = len(pts)
    visited = np.zeros(n, bool)
    accepted = []
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
        if not (CLUSTER_MIN_POINTS <= len(idx) <= CLUSTER_MAX_POINTS):
            continue
        m = pts[idx]
        ext = m.max(axis=0) - m.min(axis=0)
        if math.hypot(ext[0], ext[1]) > CLUSTER_MAX_DIAMETER:
            continue
        accepted.append(np.array(idx))
    return accepted


# ------------------------------------------------- frozen feature extraction
def extract_features(m, sensor):
    """Port of extractFeatures(); m is Nx2 member points."""
    f = dict(length=0.0, thickness=0.0, elongation=1.0, point_count=len(m),
             range=0.0, angular_span=0.0, fill_ratio=0.0, max_gap=0.0,
             valid=False, axis=None, centroid=None)
    n = len(m)
    if n < 3:
        return f
    c = m.mean(axis=0)
    d = m - c
    sxx = float((d[:, 0] ** 2).mean())
    sxy = float((d[:, 0] * d[:, 1]).mean())
    syy = float((d[:, 1] ** 2).mean())
    tr, det = sxx + syy, sxx * syy - sxy * sxy
    disc = max(tr * tr / 4.0 - det, 0.0)
    l1 = tr / 2.0 + math.sqrt(disc)
    if abs(sxy) > 1e-12:
        ax = np.array([l1 - syy, sxy])
    else:
        ax = np.array([1.0, 0.0]) if sxx >= syy else np.array([0.0, 1.0])
    nrm = np.linalg.norm(ax)
    ax = ax / nrm if nrm > 1e-12 else np.array([1.0, 0.0])
    perp = np.array([-ax[1], ax[0]])
    pa = d @ ax
    pp = d @ perp
    f['length'] = float(pa.max() - pa.min())
    f['thickness'] = float(pp.max() - pp.min())
    f['elongation'] = f['length'] / max(f['thickness'], THICKNESS_FLOOR)
    f['range'] = float(math.hypot(c[0] - sensor[0], c[1] - sensor[1]))
    f['centroid'] = c
    f['axis'] = ax

    b = np.arctan2(m[:, 1] - sensor[1], m[:, 0] - sensor[0])
    order = np.argsort(b)
    bs = b[order]
    # shortest arc containing all bearings
    gaps = np.diff(bs)
    wrap = (bs[0] + 2 * math.pi) - bs[-1]
    allg = np.append(gaps, wrap)
    kmax = int(np.argmax(allg))
    span = 2 * math.pi - allg[kmax]
    f['angular_span'] = float(span)
    exp_beams = span / ANGLE_INC + 1.0
    f['fill_ratio'] = float(min(1.0, n / exp_beams)) if exp_beams > 0 else 0.0

    ms = m[order]
    rolled = np.roll(ms, -1, axis=0)
    steps = np.linalg.norm(rolled - ms, axis=1)
    steps = np.delete(steps, kmax % len(steps)) if len(steps) > 1 else steps
    f['max_gap'] = float(steps.max()) if len(steps) else 0.0
    f['valid'] = True
    return f
