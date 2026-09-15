#!/usr/bin/env python3
"""Stage-4G8: one consolidated robustness table plus the machine-readable results.

Groups trials by (scenario, perturbation, arm) and reports exactly the columns
the Stage-4G8 brief asks for. Aggregates only; derives no new metric.
"""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path

import numpy as np


def key_of(r):
    """The perturbation label for a trial, built from what was actually applied."""
    bits = []
    if r.get('scan_noise_sigma'):
        bits.append(f"noise{r['scan_noise_sigma']:g}")
    po = r.get('pose_offset') or [0, 0, 0]
    if any(po):
        bits.append(f"pose{po[0]:g}/{po[1]:g}/{po[2]:g}")
    if r.get('jitter'):
        bits.append(f"jitter{r['jitter']:g}")
    if r.get('world') and r['world'] != 'stage4f_benchmark':
        bits.append(r['world'].replace('stage4g7_', 'world-'))
    return '+'.join(bits) if bits else 'nominal'


def agg(v):
    a = np.array([x for x in v if x is not None and np.isfinite(x)], float)
    return a if len(a) else None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', nargs='+', required=True)
    ap.add_argument('--out-dir', required=True)
    args = ap.parse_args()
    rows, failures = [], []
    for p in args.trials:
        d = json.loads(Path(p).read_text())
        rows.extend(d['trials'])
        failures.extend(d.get('setup_failures', []))

    groups = defaultdict(list)
    for r in rows:
        groups[(r['scenario'], key_of(r), r['layer_mode'])].append(r)

    table = []
    for (sc, pert, arm), rs in sorted(groups.items()):
        def f(k):
            return agg([r.get(k) for r in rs])
        mc, gt_, pl = f('min_clearance_m'), f('goal_reach_time_s'), f('path_length_m')
        row = dict(
            scenario=sc, perturbation=pert, arm=arm, trials=len(rs),
            success=sum(1 for r in rs if r.get('succeeded')),
            collisions=sum(1 for r in rs if r.get('collision')),
            mean_clearance_m=round(float(mc.mean()), 4) if mc is not None else None,
            worst_clearance_m=round(float(mc.min()), 4) if mc is not None else None,
            p10_clearance_m=round(float(np.percentile(mc, 10)), 4) if mc is not None else None,
            reaction_lead_s=round(float(f('reaction_lead_s').mean()), 3)
            if f('reaction_lead_s') is not None else None,
            nav_time_s=round(float(gt_.mean()), 3) if gt_ is not None else None,
            path_len_m=round(float(pl.mean()), 3) if pl is not None else None,
            stopped_fraction=round(float(f('stopped_fraction').mean()), 4)
            if f('stopped_fraction') is not None else None,
            id_switches=int(sum(r.get('id_switches') or 0 for r in rs)),
            false_confirmed=int(sum(r.get('false_confirmed_ids') or 0 for r in rs)),
            false_persistent=int(sum(r.get('false_persistent_ids') or 0 for r in rs)),
            purity=round(float(f('purity').mean()), 4) if f('purity') is not None else None,
            matched_fraction=round(float(f('matched_fraction').mean()), 4)
            if f('matched_fraction') is not None else None,
            position_error_m=round(float(f('position_error_m').mean()), 4)
            if f('position_error_m') is not None else None,
            velocity_error_mps=round(float(f('velocity_error_mps').mean()), 4)
            if f('velocity_error_mps') is not None else None,
            prediction_ade_m=round(float(f('prediction_ade_m').mean()), 4)
            if f('prediction_ade_m') is not None else None,
            prediction_fde_3s_m=round(float(f('prediction_fde_3s_m').mean()), 4)
            if f('prediction_fde_3s_m') is not None else None,
            coasting_frames=round(float(f('coasting_frames').mean()), 2)
            if f('coasting_frames') is not None else None,
            costmap_multi_ok=all((r.get('costmap_multi_represented') or 0) >=
                                 (r.get('costmap_multi_eligible') or 0) for r in rs),
            prediction_failures=int(sum(r.get('prediction_failures') or 0 for r in rs)))
        table.append(row)

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    with (out / 'results.csv').open('w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    (out / 'results.json').write_text(json.dumps(dict(
        note='One row per (scenario, perturbation, arm). Tracking columns come from '
             'the unchanged Stage-4G2 scorer; navigation, clearance and prediction '
             'ADE/FDE are derived from the recordings.',
        n_trials=len(rows), n_setup_failures=len(failures), setup_failures=failures,
        rows=table), indent=1) + '\n')
    print(f'{len(table)} groups, {len(rows)} trials -> {out}/results.csv')


if __name__ == '__main__':
    main()
