#!/usr/bin/env python3
"""Pairwise fragment statistics from the recorded Stage-5B raw rows."""
import gzip, json, math, statistics as st
from collections import defaultdict
import sys

RAW = '/home/sachin/predictive_nav_ws/validation/stage5b/raw'
FILES = ['actor_bike', 'actor_pedestrian', 'actor_planter',
         'actor_vehicle', 'actor_vehicle_compact']

rows = []
for f in FILES:
    with gzip.open(f'{RAW}/{f}.jsonl.gz', 'rt') as fh:
        for line in fh:
            rows.append(json.loads(line))

# group by (scenario, stamp) -> list of track rows
frames = defaultdict(list)
for r in rows:
    frames[(r['scenario'], round(r['stamp'], 3))].append(r)

# per scenario: how many simultaneous tracks
multi = defaultdict(list)
for (scen, stamp), trs in frames.items():
    if len(trs) >= 2:
        multi[scen].append((stamp, trs))

print("=" * 78)
print("SIMULTANEOUS-TRACK FRAMES PER SCENARIO (fragmentation evidence)")
print("=" * 78)
print(f"{'scenario':<42} {'frames>=2tr':>11} {'tot frames':>11}")
tot_frames = defaultdict(int)
for (scen, stamp), trs in frames.items():
    tot_frames[scen] += 1
for scen in sorted(tot_frames):
    m = len(multi.get(scen, []))
    if m:
        print(f"{scen:<42} {m:>11} {tot_frames[scen]:>11}")

print()
print("=" * 78)
print("PAIR GEOMETRY ON FRAMES WITH EXACTLY 2 TRACKS")
print("=" * 78)
hdr = (f"{'scenario':<34}{'n':>4}{'sep_med':>9}{'sep_sd':>8}{'sep_min':>8}"
       f"{'sep_max':>8}{'dspd_med':>9}{'len_med':>8}{'thk_med':>8}{'fill':>6}")
print(hdr)
pair_rec = {}
for scen in sorted(multi):
    seps, dspd, lens, thks, fills = [], [], [], [], []
    series = []
    for stamp, trs in sorted(multi[scen]):
        if len(trs) != 2:
            continue
        a, b = trs
        sep = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
        seps.append(sep)
        dspd.append(abs(a['speed'] - b['speed']))
        for t in (a, b):
            if t.get('features_valid'):
                lens.append(t['cluster_length'])
                thks.append(t['cluster_thickness'])
                fills.append(t['cluster_fill_ratio'])
        series.append((stamp, sep))
    if len(seps) < 3:
        continue
    pair_rec[scen] = series
    sd = st.pstdev(seps) if len(seps) > 1 else 0.0
    print(f"{scen:<34}{len(seps):>4}{st.median(seps):>9.3f}{sd:>8.4f}"
          f"{min(seps):>8.3f}{max(seps):>8.3f}{st.median(dspd):>9.3f}"
          f"{st.median(lens) if lens else float('nan'):>8.3f}"
          f"{st.median(thks) if thks else float('nan'):>8.3f}"
          f"{st.median(fills) if fills else float('nan'):>6.2f}")
