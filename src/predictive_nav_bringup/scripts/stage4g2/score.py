#!/usr/bin/env python3
"""Evaluation-only GT assignment and scan-supported failure diagnostics.

Scoring gate .45 m = .20 m cylinder surface bias + .15 m (3 sigma
measurement noise) + .10 m localization allowance. Maximum-cardinality,
minimum-Euclidean-distance one-to-one GT matching; no tracker IDs in cost.
GT is interpolated at scan time (not the latest asynchronous odometry).
Retained predictions are never counted as detections. Metrics include the
initial velocity convergence period; object confirmation happens pre-release.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import re
import numpy as np
from scipy.optimize import linear_sum_assignment

GATE = .45


def assignment(points, gt, gate=GATE):
    if not len(points) or not len(gt):
        return {}
    d = np.linalg.norm(np.array(points)[:,None,:]-np.array(gt)[None,:,:], axis=2)
    # One private dummy per row; penalty > maximum possible valid cost sum.
    penalty = (min(d.shape)+1)*gate
    cost = np.concatenate((np.where(d<=gate,d,penalty*3),
                           np.full((len(points),len(points)),penalty)),axis=1)
    ri,ci = linear_sum_assignment(cost)
    return {int(c):int(r) for r,c in zip(ri,ci) if c<len(gt) and d[r,c]<=gate}


def describe(values):
    return dict(mean=float(np.mean(values)),p95=float(np.percentile(values,95)),
                max=float(np.max(values))) if len(values) else None


def visibility(scan, gt):
    """Assign actual LiDAR endpoints to cylinder surfaces for diagnostics ONLY.

    .10 m tolerance accommodates range noise/map-frame localization. Connected
    components here explain the actual clusterer's .35 m connectivity, but do
    not produce runtime observations. Merge evidence requires >=2 returns from
    each of two GT surfaces in the same component, not just close GT centers.
    """
    points=np.array(scan['points'])
    if not len(gt) or not len(points):
        return [0]*len(gt),[]
    residual=abs(np.linalg.norm(points[:,None,:]-np.array(gt)[None,:,:],axis=2)-.2)
    labels=residual.argmin(axis=1)
    keep=residual.min(axis=1)<.10
    pts=points[keep]
    labels=labels[keep]
    counts=[int(np.sum(labels==i)) for i in range(len(gt))]
    groups=[]
    visited=set()
    for i in range(len(pts)):
        if i in visited:
            continue
        group=[i]
        visited.add(i)
        for j in group:
            for k in np.flatnonzero(np.linalg.norm(pts-pts[j],axis=1)<=.35):
                if int(k) not in visited:
                    visited.add(int(k)); group.append(int(k))
        c=Counter(int(labels[j]) for j in group)
        members=[k for k,v in c.items() if v>=2]
        if len(members)>1:
            groups.append(members)
    return counts,groups


def score(path):
    raw=json.loads((path/'raw.json').read_text())
    n=len(raw['gt'])
    gt=[np.array(g) for g in raw['gt']]
    scans={f"{s['t']:.6f}":s for s in raw['scans']}
    counts=[Counter() for _ in gt]
    seq=[[] for _ in gt]
    errors=[[] for _ in gt]
    velocities=[[] for _ in gt]
    fragments=[0]*n
    switches=[0]*n
    previous=[None]*n
    last_matched=[False]*n
    ever=[False]*n
    false=Counter()
    false_times=defaultdict(list)
    duplicate_frames=0
    pred_checks=0
    pred_failures=0
    timeline=[]
    confirmed=[]
    miss_max=0
    all_ids=set()
    dt_misses=defaultdict(list)
    gap_events=[]
    last_by_id={}
    supported=matched_supported=0
    resolved_seps=[]
    merged_seps=[]
    categories=Counter()
    for frame in raw['arrays']:
        t=frame['t']
        if not raw['t0']<=t<=raw['end']:
            continue
        if any(t<g[0,0] or t>g[-1,0] for g in gt):
            raise ValueError('GT does not bracket scan timestamp')
        state=np.array([[np.interp(t,g[:,0],g[:,k]) for k in range(1,5)] for g in gt]).reshape(n,4)
        centers=state[:,:2]
        tracks=frame['tracks']
        fresh=[tr for tr in tracks if tr['missed']==0]
        current=[[tr['p'][k]+tr['v'][k]*max(0,t-tr['stamp']) for k in range(2)] for tr in tracks]
        retained=assignment(current,centers)
        matches=assignment([tr['p'] for tr in fresh],centers)
        confirmed.append(len(tracks))
        matched_idx=set(matches.values())
        false_ids=[]
        duplicate=False
        for idx,tr in enumerate(fresh):
            if idx not in matched_idx:
                near=bool(n and min(np.linalg.norm(centers-np.array(tr['p']),axis=1))<=GATE)
                duplicate |= near
                if not near:
                    false[tr['id']]+=1
                    false_times[tr['id']].append(t)
                    false_ids.append(tr['id'])
        duplicate_frames+=int(duplicate)
        for tr in tracks:
            all_ids.add(tr['id'])
            miss_max=max(miss_max,tr['missed'])
            if tr['missed']:
                dt_misses[tr['id']].append(t-tr['stamp'])
            old=last_by_id.get(tr['id'])
            if old and old['missed'] and not tr['missed']:
                gap_events.append(dict(id=tr['id'], missed_scans=old['missed'],
                    since_last_observation_s=t-old['stamp'], t=t-raw['t0'], same_id=True))
            last_by_id[tr['id']]=tr
            P=np.array(tr['covariance']).reshape(4,4)
            for pr in tr['predictions']:
                dt=pr['dt']; F=np.eye(4); F[0,2]=F[1,3]=dt
                expected=(F@P@F.T)[:2,:2]+np.eye(2)*.25*dt**4/4
                pred_checks+=1
                valid=np.allclose(pr['p'],np.array(tr['p'])+np.array(tr['v'])*dt,atol=1e-8)
                valid &= np.allclose(np.array(pr['covariance']).reshape(2,2),expected,atol=1e-8)
                valid &= abs(pr['stamp']-tr['stamp']-dt)<1e-6
                pred_failures+=int(not valid)
        scan=scans.get(f'{t:.6f}')
        vis,merges=visibility(scan,centers) if scan else ([None]*n,[])
        centroids=raw['clusters'].get(f'{t:.6f}',[])
        cluster_matches=assignment(centroids,centers)
        if merges:
            categories['A_cluster_merge']+=1
        if any(v is not None and v<3 for v in vis):
            categories['D_insufficient_lidar_returns']+=1
        if scan is None:
            categories['scan_tf_unavailable_for_diagnostics']+=1
        if n>1:
            sep=min(np.linalg.norm(centers[a]-centers[b]) for a in range(n) for b in range(a))
            if merges: merged_seps.append(float(sep))
            if len(cluster_matches)==n and not merges: resolved_seps.append(float(sep))
        else:
            sep=None
        ids=[]
        retained_ids=[]
        for i in range(n):
            matched=i in matches
            ids.append(fresh[matches[i]]['id'] if matched else None)
            retained_ids.append(tracks[retained[i]]['id'] if i in retained else None)
            if i in cluster_matches and not merges:
                supported+=1
                matched_supported+=int(matched)
                if not matched:
                    categories['B_or_C_resolved_cluster_without_fresh_track']+=1
            if matched:
                tr=fresh[matches[i]]
                tid=tr['id']
                counts[i][tid]+=1
                if previous[i] is not None and previous[i]!=tid:
                    switches[i]+=1
                if ever[i] and not last_matched[i]:
                    fragments[i]+=1
                previous[i]=tid
                ever[i]=True
                errors[i].append(float(np.linalg.norm(np.array(tr['p'])-centers[i])))
                velocities[i].append(float(np.linalg.norm(np.array(tr['v'])-state[i,2:])))
            last_matched[i]=matched
            seq[i].append(ids[-1])
        timeline.append(dict(t=round(t-raw['t0'],6), gt=state.tolist(), ids=ids,
            retained_ids=retained_ids, confirmed=len(tracks), clusters=centroids,
            visible_returns=vis, merge_groups=merges, separation=sep,
            tracks=[{k:tr[k] for k in ('id','p','raw','v','missed','stamp','observations')} for tr in tracks],
            false_ids=false_ids))
    performance=defaultdict(list)
    for line in (path/'launch.log').read_text().splitlines():
        m=re.search(r'G2_PERF ([\d.]+) (.*)',line)
        if m and raw['t0']<=float(m[1])<=raw['end']:
            for key,val in re.findall(r'(\w+)=([\d.]+)',m[2]):
                performance[key].append(float(val))
    per_gt=[]
    for i in range(n):
        matched=sum(counts[i].values())
        per_gt.append(dict(object=i,id_counts=dict(counts[i]),id_switches=switches[i],
            track_fragments=fragments[i], identity_fragments=max(0,len(counts[i])-1),
            purity=max(counts[i].values(),default=0)/matched if matched else 0,
            matched_frames=matched, matched_frame_fraction=matched/len(timeline) if timeline else 0,
            position_error_m=describe(errors[i]),velocity_error_mps=describe(velocities[i])))
    summary=dict(scenario=raw['scenario'],definition=raw['definition'],frames=len(timeline),gt_objects=n,
        evaluation_gate_m=GATE,algorithm='unchanged KF-predicted Euclidean gated greedy',
        confirmed_tracks=describe(confirmed),per_gt=per_gt,
        id_switches=sum(switches),track_fragments=sum(fragments),
        false_confirmed_ids=dict(false),false_fresh_frames=sum(false.values()),
        false_persistent_ids=[i for i,ts in false_times.items() if max(ts)-min(ts)>=1.],
        duplicate_frames=duplicate_frames, distinct_confirmed_ids=len(all_ids),
        resolvable_cluster_object_frames=supported,
        resolvable_cluster_matched_fraction=matched_supported/supported if supported else None,
        categories=dict(categories), min_resolved_center_separation_m=min(resolved_seps,default=None),
        merge_center_separation_m=describe(merged_seps), max_missed_scans_retained=miss_max,
        reacquisitions=gap_events, prediction_samples_checked=pred_checks,
        prediction_math_failures=pred_failures, performance_us={k:describe(v) for k,v in performance.items()},
        performance_frames=len(performance.get('callback_us',[])),
        costmap_grid_frames=len(raw['grids']),costmap_nonzero_frames=sum(bool(g['cells']) for g in raw['grids']),
        costmap_multi_eligible_frames=sum(len(g.get('eligible_ids',[]))>=2 for g in raw['grids']),
        costmap_multi_represented_frames=sum(len(g.get('represented_ids',[]))>=2 for g in raw['grids']),
        nav=raw['nav'])
    (path/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (path/'timeline.json').write_text(json.dumps(timeline,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ['scenario','frames','id_switches','track_fragments','categories','nav']}))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial',type=Path)
    score(p.parse_args().trial)
