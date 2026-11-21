#!/usr/bin/env python3
"""Characterise bike fragmentation vs genuinely-distinct close object pairs."""
import os
import math, sys
import sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from raycast import (box_segments, circle_shape, scan, build_clusters,
                     extract_features, CLUSTER_DISTANCE)

SENSOR = (-3.0, 0.0)
rng = np.random.default_rng(7)

# ---- footprints, straight from the SDF files -------------------------------
def bike(cx, cy, yaw):
    segs = []
    for off in (-0.525, 0.525):
        c, s = math.cos(yaw), math.sin(yaw)
        segs += box_segments(cx + c * off, cy + s * off, 0.57, 0.04, yaw)
    return segs

def pedestrian(cx, cy, yaw):
    segs = []
    c, s = math.cos(yaw), math.sin(yaw)
    for lx, ly in ((-0.14, -0.08), (0.14, 0.08)):
        segs += box_segments(cx + c*lx - s*ly, cy + s*lx + c*ly, 0.12, 0.12, yaw)
    return segs

def cyl(cx, cy):                      # Stage-4G3 dynamic_obstacle, r = 0.2
    return circle_shape(cx, cy, 0.2)

def planter(cx, cy, yaw):
    return box_segments(cx, cy, 0.8, 0.6, yaw)   # approx from metrics (len .80 thk .26)


def pair_of(clusters, pts):
    """Return the two largest accepted clusters' features, or None."""
    if len(clusters) != 2:
        return None
    fs = [extract_features(pts[i], SENSOR) for i in clusters]
    if not all(f['valid'] for f in fs):
        return None
    return fs


def pair_metrics(fa, fb):
    ca, cb = fa['centroid'], fb['centroid']
    sep = float(np.linalg.norm(ca - cb))
    u = (cb - ca) / sep
    # collinearity: each fragment axis vs the pair axis
    cos_a = abs(float(np.dot(fa['axis'], u)))
    cos_b = abs(float(np.dot(fb['axis'], u)))
    # combined envelope
    return dict(sep=sep, cos_a=cos_a, cos_b=cos_b,
                align=min(cos_a, cos_b),
                thick=max(fa['thickness'], fb['thickness']),
                len_a=fa['length'], len_b=fb['length'],
                elo=min(fa['elongation'], fb['elongation']))


def union_features(clusters, pts):
    idx = np.concatenate(clusters)
    return extract_features(pts[idx], SENSOR)


print("=" * 100)
print("A.  BIKE  -- ray-cast, broadside (yaw=pi/2 => flank) and oblique, over range")
print("=" * 100)
print(f"{'aspect':<10}{'range':>6}{'ncl':>5}{'sep':>7}{'align':>7}{'thick':>7}"
      f"{'len_a':>7}{'len_b':>7}{'elong':>7} | {'UNION: len':>11}{'thk':>6}{'fill':>6}{'gap':>6}{'pts':>5}")
bike_rows = []
for aspect, yaw in (('flank', math.pi / 2), ('oblique', math.pi / 4)):
    for r in (2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0):
        cx, cy = SENSOR[0] + r, 0.0
        pts = scan(SENSOR, bike(cx, cy, yaw), noise=0.01, rng=rng)
        cl = build_clusters(pts)
        fs = pair_of(cl, pts)
        if fs is None:
            print(f"{aspect:<10}{r:>6.1f}{len(cl):>5}  -- not a 2-cluster frame --")
            continue
        pm = pair_metrics(*fs)
        uf = union_features(cl, pts)
        bike_rows.append((aspect, r, pm, uf))
        print(f"{aspect:<10}{r:>6.1f}{len(cl):>5}{pm['sep']:>7.3f}{pm['align']:>7.3f}"
              f"{pm['thick']:>7.3f}{pm['len_a']:>7.3f}{pm['len_b']:>7.3f}{pm['elo']:>7.1f}"
              f" | {uf['length']:>11.3f}{uf['thickness']:>6.3f}{uf['fill_ratio']:>6.2f}"
              f"{uf['max_gap']:>6.3f}{uf['point_count']:>5}")

print()
print("=" * 100)
print("B.  TWO STAGE-4G3 CYLINDERS (r=0.2) -- the protected close-object geometry")
print("=" * 100)
print(f"{'true_sep':>9}{'crossgap':>9}{'ncl':>5}{'sep':>7}{'align':>7}{'thick':>7}"
      f"{'len_a':>7}{'len_b':>7}{'elong':>7} | {'UNION: len':>11}{'thk':>6}{'fill':>6}{'gap':>6}")
for tsep in (0.46, 0.50, 0.55, 0.60, 0.67, 0.75, 0.85, 0.95, 1.00, 1.04, 1.10, 1.20):
    cx = SENSOR[0] + 3.0
    segs = cyl(cx, -tsep / 2) + cyl(cx, tsep / 2)
    pts = scan(SENSOR, segs, noise=0.01, rng=rng)
    cl = build_clusters(pts)
    fs = pair_of(cl, pts)
    cg = tsep - 0.4
    if fs is None:
        print(f"{tsep:>9.2f}{cg:>9.2f}{len(cl):>5}  -- not a 2-cluster frame "
              f"(merged by primary clustering or rejected) --")
        continue
    pm = pair_metrics(*fs)
    uf = union_features(cl, pts)
    print(f"{tsep:>9.2f}{cg:>9.2f}{len(cl):>5}{pm['sep']:>7.3f}{pm['align']:>7.3f}"
          f"{pm['thick']:>7.3f}{pm['len_a']:>7.3f}{pm['len_b']:>7.3f}{pm['elo']:>7.1f}"
          f" | {uf['length']:>11.3f}{uf['thickness']:>6.3f}{uf['fill_ratio']:>6.2f}{uf['max_gap']:>6.3f}")

print()
print("=" * 100)
print("C.  TWO PEDESTRIANS side by side (Stage-5B pedestrian footprint)")
print("=" * 100)
print(f"{'true_sep':>9}{'ncl':>5}{'sep':>7}{'align':>7}{'thick':>7}{'len_a':>7}{'len_b':>7}"
      f"{'elong':>7} | {'UNION: len':>11}{'thk':>6}{'fill':>6}{'gap':>6}")
for tsep in (0.50, 0.60, 0.70, 0.85, 0.95, 1.00, 1.04, 1.10, 1.20):
    cx = SENSOR[0] + 3.0
    segs = pedestrian(cx, -tsep/2, math.pi/2) + pedestrian(cx, tsep/2, math.pi/2)
    pts = scan(SENSOR, segs, noise=0.01, rng=rng)
    cl = build_clusters(pts)
    fs = pair_of(cl, pts)
    if fs is None:
        print(f"{tsep:>9.2f}{len(cl):>5}  -- not a 2-cluster frame --")
        continue
    pm = pair_metrics(*fs); uf = union_features(cl, pts)
    print(f"{tsep:>9.2f}{len(cl):>5}{pm['sep']:>7.3f}{pm['align']:>7.3f}{pm['thick']:>7.3f}"
          f"{pm['len_a']:>7.3f}{pm['len_b']:>7.3f}{pm['elo']:>7.1f}"
          f" | {uf['length']:>11.3f}{uf['thickness']:>6.3f}{uf['fill_ratio']:>6.2f}{uf['max_gap']:>6.3f}")

print()
print("=" * 100)
print("D.  PEDESTRIAN + PLANTER (mixed close pair)")
print("=" * 100)
for tsep in (0.70, 0.90, 1.04, 1.20):
    cx = SENSOR[0] + 3.0
    segs = pedestrian(cx, -tsep/2, math.pi/2) + planter(cx, tsep/2 + 0.3, 0.0)
    pts = scan(SENSOR, segs, noise=0.01, rng=rng)
    cl = build_clusters(pts)
    fs = pair_of(cl, pts)
    if fs is None:
        print(f"  sep={tsep:.2f} ncl={len(cl)}  -- not a 2-cluster frame --")
        continue
    pm = pair_metrics(*fs); uf = union_features(cl, pts)
    print(f"  sep={tsep:.2f} ncl={len(cl)} pairsep={pm['sep']:.3f} align={pm['align']:.3f} "
          f"thick={pm['thick']:.3f} elong={pm['elo']:.1f} | union len={uf['length']:.3f} "
          f"fill={uf['fill_ratio']:.2f} gap={uf['max_gap']:.3f}")
