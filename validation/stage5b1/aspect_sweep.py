#!/usr/bin/env python3
"""Why does the compound rule lose the bicycle at a radial aspect?"""
import os
import math, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from raycast import box_segments, scan, build_clusters, extract_features

SENSOR = (-3.0, 0.0)
rng = np.random.default_rng(99)
RULE = dict(align_min=0.98, uthk_max=0.08, lmax_min=0.42, lmin_min=0.42,
            sep_lo=0.80, sep_hi=1.35, gap_lo=0.35, gap_hi=0.80,
            ulen_lo=1.00, ulen_hi=2.00, ufill_hi=0.90, frag_thk_max=0.10,
            lmax_hi=0.80)

def bike(cx, cy, yaw):
    segs = []
    c, s = math.cos(yaw), math.sin(yaw)
    for off in (-0.525, 0.525):
        segs += box_segments(cx + c*off, cy + s*off, 0.57, 0.04, yaw)
    return segs

print("ASPECT SWEEP: yaw is the bike's heading; 90 deg = broadside (flank),")
print("0 deg = RADIAL (pointing away from the sensor, wheels self-occluding).")
print()
print(f"{'yaw_deg':>8}{'ncl':>5}{'sep':>7}{'align':>7}{'lmin':>7}{'lmax':>7}"
      f"{'uthk':>7}{'gap':>7}{'ulen':>7}  verdict / failing bounds")
print("-"*104)
for deg in range(0, 95, 5):
    yaw = math.radians(deg)
    accepted = 0
    trials = 0
    agg = []
    for r in (2.5, 3.0, 3.5, 4.0):
        pts = scan(SENSOR, bike(SENSOR[0]+r, 0.0, yaw), noise=0.01, rng=rng)
        if len(pts) < 3: continue
        cl = build_clusters(pts)
        if len(cl) != 2: continue
        fs = [extract_features(pts[i], SENSOR) for i in cl]
        if not all(f['valid'] for f in fs): continue
        trials += 1
        ca, cb = fs[0]['centroid'], fs[1]['centroid']
        sep = float(np.linalg.norm(cb-ca)); u = (cb-ca)/sep
        align = min(abs(float(np.dot(fs[0]['axis'],u))), abs(float(np.dot(fs[1]['axis'],u))))
        A, B = pts[cl[0]], pts[cl[1]]
        gap = float(np.min(np.linalg.norm(A[:,None,:]-B[None,:,:],axis=2)))
        uf = extract_features(pts[np.concatenate(cl)], SENSOR)
        lmin = min(fs[0]['length'],fs[1]['length']); lmax=max(fs[0]['length'],fs[1]['length'])
        tmax = max(fs[0]['thickness'],fs[1]['thickness'])
        fails=[]
        if align < RULE['align_min']: fails.append('align')
        if lmin < RULE['lmin_min'] or lmax > RULE['lmax_hi']: fails.append('frag_len')
        if tmax > RULE['frag_thk_max']: fails.append('frag_thk')
        if not (RULE['sep_lo']<=sep<=RULE['sep_hi']): fails.append('sep')
        if not (RULE['gap_lo']<=gap<=RULE['gap_hi']): fails.append('gap')
        if uf['thickness'] > RULE['uthk_max']: fails.append('uthk')
        if not (RULE['ulen_lo']<=uf['length']<=RULE['ulen_hi']): fails.append('ulen')
        if uf['fill_ratio'] > RULE['ufill_hi']: fails.append('ufill')
        if not fails: accepted += 1
        agg.append((sep,align,lmin,lmax,uf['thickness'],gap,uf['length'],fails))
    if not agg:
        print(f"{deg:>8}{0:>5}   -- bicycle does not present two clusters at this aspect --")
        continue
    m = lambda i: float(np.median([a[i] for a in agg]))
    allf = {}
    for a in agg:
        for f in a[7]: allf[f]=allf.get(f,0)+1
    v = f"{accepted}/{trials} accepted" + (f"   fails: {allf}" if allf else "")
    print(f"{deg:>8}{trials:>5}{m(0):>7.3f}{m(1):>7.3f}{m(2):>7.3f}{m(3):>7.3f}"
          f"{m(4):>7.3f}{m(5):>7.3f}{m(6):>7.3f}  {v}")
