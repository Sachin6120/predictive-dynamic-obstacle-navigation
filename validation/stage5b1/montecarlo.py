#!/usr/bin/env python3
import os
import math, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from raycast import (box_segments, circle_shape, scan, build_clusters, extract_features)

SENSOR = (-3.0, 0.0)

def bike(cx, cy, yaw):
    segs = []
    c, s = math.cos(yaw), math.sin(yaw)
    for off in (-0.525, 0.525):
        segs += box_segments(cx + c*off, cy + s*off, 0.57, 0.04, yaw)
    return segs

def pedestrian(cx, cy, yaw):
    segs = []
    c, s = math.cos(yaw), math.sin(yaw)
    for lx, ly in ((-0.14,-0.08),(0.14,0.08)):
        segs += box_segments(cx + c*lx - s*ly, cy + s*lx + c*ly, 0.12, 0.12, yaw)
    return segs

def cyl(cx, cy):
    return circle_shape(cx, cy, 0.2)

def feats_pair(pts):
    cl = build_clusters(pts)
    if len(cl) != 2:
        return None, len(cl)
    fs = [extract_features(pts[i], SENSOR) for i in cl]
    if not all(f['valid'] for f in fs):
        return None, len(cl)
    idx = np.concatenate(cl)
    return (fs[0], fs[1], extract_features(pts[idx], SENSOR), cl, pts), len(cl)

def metrics(fa, fb, cl, pts):
    ca, cb = fa['centroid'], fb['centroid']
    sep = float(np.linalg.norm(cb - ca))
    u = (cb - ca)/sep
    align = min(abs(float(np.dot(fa['axis'],u))), abs(float(np.dot(fb['axis'],u))))
    A, B = pts[cl[0]], pts[cl[1]]
    gap = float(np.min(np.linalg.norm(A[:,None,:]-B[None,:,:], axis=2)))
    return sep, align, gap

rng = np.random.default_rng(20261120)
pop = {}

# --- bike, all aspects and ranges
rows = []
for _ in range(400):
    yaw = rng.uniform(0, 2*math.pi)
    r = rng.uniform(2.0, 5.5)
    bear = rng.uniform(-0.6, 0.6)
    cx = SENSOR[0] + r*math.cos(bear); cy = SENSOR[1] + r*math.sin(bear)
    pts = scan(SENSOR, bike(cx, cy, yaw), noise=0.01, rng=rng)
    if len(pts) < 3: continue
    res, n = feats_pair(pts)
    if res is None: continue
    fa, fb, uf, cl, pts2 = res
    sep, align, gap = metrics(fa, fb, cl, pts2)
    rows.append(dict(sep=sep, align=align, gap=gap,
                     tmax=max(fa['thickness'], fb['thickness']),
                     lmin=min(fa['length'], fb['length']),
                     lmax=max(fa['length'], fb['length']),
                     ulen=uf['length'], uthk=uf['thickness'],
                     ufill=uf['fill_ratio'], ugap=uf['max_gap']))
pop['BIKE frag pair'] = rows

# --- two cylinders (Stage-4G3), separations that actually yield 2 clusters
rows = []
for _ in range(400):
    tsep = rng.uniform(0.70, 1.60)
    r = rng.uniform(2.0, 5.5)
    ang = rng.uniform(0, math.pi)
    cx = SENSOR[0] + r; cy = 0.0
    dx, dy = math.cos(ang)*tsep/2, math.sin(ang)*tsep/2
    pts = scan(SENSOR, cyl(cx-dx, cy-dy)+cyl(cx+dx, cy+dy), noise=0.01, rng=rng)
    if len(pts) < 3: continue
    res, n = feats_pair(pts)
    if res is None: continue
    fa, fb, uf, cl, pts2 = res
    sep, align, gap = metrics(fa, fb, cl, pts2)
    rows.append(dict(sep=sep, align=align, gap=gap,
                     tmax=max(fa['thickness'], fb['thickness']),
                     lmin=min(fa['length'], fb['length']),
                     lmax=max(fa['length'], fb['length']),
                     ulen=uf['length'], uthk=uf['thickness'],
                     ufill=uf['fill_ratio'], ugap=uf['max_gap']))
pop['4G3 cyl pair'] = rows

# --- two pedestrians
rows = []
for _ in range(400):
    tsep = rng.uniform(0.65, 1.60)
    r = rng.uniform(2.0, 5.5)
    ang = rng.uniform(0, math.pi)
    y1 = rng.uniform(0, 2*math.pi); y2 = rng.uniform(0, 2*math.pi)
    cx = SENSOR[0] + r; cy = 0.0
    dx, dy = math.cos(ang)*tsep/2, math.sin(ang)*tsep/2
    pts = scan(SENSOR, pedestrian(cx-dx, cy-dy, y1)+pedestrian(cx+dx, cy+dy, y2),
               noise=0.01, rng=rng)
    if len(pts) < 3: continue
    res, n = feats_pair(pts)
    if res is None: continue
    fa, fb, uf, cl, pts2 = res
    sep, align, gap = metrics(fa, fb, cl, pts2)
    rows.append(dict(sep=sep, align=align, gap=gap,
                     tmax=max(fa['thickness'], fb['thickness']),
                     lmin=min(fa['length'], fb['length']),
                     lmax=max(fa['length'], fb['length']),
                     ulen=uf['length'], uthk=uf['thickness'],
                     ufill=uf['fill_ratio'], ugap=uf['max_gap']))
pop['PED pair'] = rows

def q(v, p): return float(np.percentile(v, p))

print("="*104)
print("MONTE-CARLO FEATURE DISTRIBUTIONS  (ray-cast, 0.01 m range noise, random aspect/range)")
print("="*104)
fields = ['sep','align','gap','tmax','lmin','lmax','ulen','uthk','ufill','ugap']
for name, rows in pop.items():
    print(f"\n{name}   n={len(rows)}")
    print(f"  {'field':<7}{'min':>9}{'p5':>9}{'p50':>9}{'p95':>9}{'max':>9}")
    for fl in fields:
        v = [r[fl] for r in rows]
        print(f"  {fl:<7}{min(v):>9.3f}{q(v,5):>9.3f}{q(v,50):>9.3f}{q(v,95):>9.3f}{max(v):>9.3f}")

# ---------------------------------------------------------------- rule sweep
RULE = dict(align_min=0.98, uthk_max=0.08, lmax_min=0.42, lmin_min=0.25,
            sep_lo=0.80, sep_hi=1.35, gap_lo=0.35, gap_hi=0.80,
            ulen_lo=1.00, ulen_hi=2.00, ufill_hi=0.90)

def passes(r, R=RULE):
    reasons = []
    if not (r['align'] >= R['align_min']): reasons.append('align')
    if not (r['uthk'] <= R['uthk_max']): reasons.append('uthk')
    if not (r['lmax'] >= R['lmax_min']): reasons.append('lmax')
    if not (r['lmin'] >= R['lmin_min']): reasons.append('lmin')
    if not (R['sep_lo'] <= r['sep'] <= R['sep_hi']): reasons.append('sep')
    if not (R['gap_lo'] < r['gap'] <= R['gap_hi']): reasons.append('gap')
    if not (R['ulen_lo'] <= r['ulen'] <= R['ulen_hi']): reasons.append('ulen')
    if not (r['ufill'] <= R['ufill_hi']): reasons.append('ufill')
    return (len(reasons) == 0), reasons

print()
print("="*104)
print("SINGLE-FRAME COMPOUND-HYPOTHESIS RULE -- acceptance per population")
print("="*104)
for k, v in RULE.items():
    print(f"  {k:<12} = {v}")
print()
from collections import Counter
for name, rows in pop.items():
    acc = 0; rej = Counter()
    for r in rows:
        ok, why = passes(r)
        if ok: acc += 1
        else:
            for w in why: rej[w] += 1
    print(f"{name:<18} accepted {acc:>4}/{len(rows):<4} ({acc/len(rows)*100:5.1f}%)   "
          f"rejected-by: {dict(rej.most_common())}")

print()
print("="*104)
print("MARGIN OF EACH DISCRIMINATING BOUND  (how far the nearest negative sits)")
print("="*104)
neg = pop['4G3 cyl pair'] + pop['PED pair']
bike = pop['BIKE frag pair']
import numpy as np
for fl, op, bound in (('align','>=',0.98), ('uthk','<=',0.08),
                      ('lmax','>=',0.42), ('sep','range',(0.80,1.35))):
    bv = [r[fl] for r in bike]; nv = [r[fl] for r in neg]
    if op == '>=':
        print(f"  {fl:<6} >= {bound}: bike min={min(bv):.3f}  "
              f"negatives passing this bound alone = {sum(1 for x in nv if x>=bound)}/{len(nv)}")
    elif op == '<=':
        print(f"  {fl:<6} <= {bound}: bike max={max(bv):.3f}  "
              f"negatives passing this bound alone = {sum(1 for x in nv if x<=bound)}/{len(nv)}")
    else:
        lo, hi = bound
        print(f"  {fl:<6} in [{lo},{hi}]: bike min={min(bv):.3f} max={max(bv):.3f}  "
              f"negatives passing alone = {sum(1 for x in nv if lo<=x<=hi)}/{len(nv)}")
