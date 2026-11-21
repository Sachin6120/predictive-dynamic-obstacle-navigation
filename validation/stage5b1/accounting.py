#!/usr/bin/env python3
"""Stage-5B.1 compound-association accounting from the recorded raw rows."""
import glob
import gzip
import json
import os
import sys
from collections import Counter, defaultdict

RAW = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/sachin/predictive_nav_ws/validation/stage5b/raw'

rows = []
for p in sorted(glob.glob(os.path.join(RAW, '*.jsonl.gz'))):
    op = gzip.open if p.endswith('.gz') else open
    with op(p, 'rt') as fh:
        for line in fh:
            rows.append(json.loads(line))

det = [r for r in rows if r.get('scenario_ok', True) and r.get('features_valid')]
print(f'rows={len(rows)}  detections with geometry={len(det)}')
print()

print('=' * 92)
print('FALSE COMPOUND ASSOCIATIONS  (a compound on anything but BIKE)')
print('=' * 92)
tot_false = 0
for c in ('PEDESTRIAN', 'BIKE', 'VEHICLE', 'UNKNOWN'):
    sub = [r for r in det if r['gt_class'] == c]
    cs = [r for r in sub if r.get('from_compound')]
    if not sub:
        continue
    kind = 'recovered (intended)' if c == 'BIKE' else (
        'FALSE COMPOUND' if cs else 'none (correct)')
    if c != 'BIKE':
        tot_false += len(cs)
    print(f'  {c:<12} detections={len(sub):>5}  compound={len(cs):>5}  {kind}')
print(f'\n  TOTAL FALSE COMPOUND ASSOCIATIONS: {tot_false}')
print()

print('=' * 92)
print('BIKE PER-FRAME ACCOUNTING -- what the stage did on every bike scan')
print('=' * 92)
bike = [r for r in rows if r['gt_class'] == 'BIKE' and r.get('scenario_ok', True)]
frames = defaultdict(list)
for r in bike:
    frames[(r['scenario'], round(r['stamp'], 3))].append(r)

cat = Counter()
for (scen, t), trs in frames.items():
    comp = [r for r in trs if r.get('from_compound')]
    frag = [r for r in trs if not r.get('from_compound') and r.get('features_valid')]
    coast = [r for r in trs if not r.get('features_valid')]
    if comp:
        cat['compound emitted'] += 1
    elif len(frag) >= 2:
        cat['TWO FRAGMENTS, no compound (missed pair)'] += 1
    elif len(frag) == 1:
        cat['one fragment only (no pair to form)'] += 1
    elif coast:
        cat['all tracks coasting (no geometry)'] += 1
    else:
        cat['other'] += 1
total = sum(cat.values())
for k, v in cat.most_common():
    print(f'  {k:<42} {v:>5}  ({v / total * 100:5.1f}%)')
print(f'  {"TOTAL bike frames":<42} {total:>5}')
print()

print('=' * 92)
print('PER-SCENARIO')
print('=' * 92)
hdr = (f'  {"scenario":<34}{"frames":>7}{"comp":>6}{"missed":>8}{"single":>8}'
       f'{"tracks":>8}  labels')
print(hdr)
for scen in sorted({r['scenario'] for r in bike}):
    fs = {k: v for k, v in frames.items() if k[0] == scen}
    c = m = s1 = 0
    for _, trs in fs.items():
        comp = [r for r in trs if r.get('from_compound')]
        frag = [r for r in trs if not r.get('from_compound') and r.get('features_valid')]
        if comp:
            c += 1
        elif len(frag) >= 2:
            m += 1
        elif len(frag) == 1:
            s1 += 1
    ids = {r['track_id'] for r in bike if r['scenario'] == scen}
    lab = Counter(r['pred_class'] for r in bike if r['scenario'] == scen)
    print(f'  {scen:<34}{len(fs):>7}{c:>6}{m:>8}{s1:>8}{len(ids):>8}  '
          + ' '.join(f'{k}:{v}' for k, v in lab.most_common()))
