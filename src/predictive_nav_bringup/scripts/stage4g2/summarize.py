#!/usr/bin/env python3
"""Curate compact Stage-4G2 evidence; keep raw simulation logs out of git."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess

import numpy as np
from score import assignment, describe


def analyze(trial):
    raw=json.loads((trial/'raw.json').read_text())
    timeline=json.loads((trial/'timeline.json').read_text())
    summary=json.loads((trial/'summary.json').read_text())
    active={}
    for line in (trial/'launch.log').read_text().splitlines():
        m=re.search(r'G2_PERF ([\d.]+).* tracks=(\d+)',line)
        if m: active[round(float(m[1]),6)]=int(m[2])
    differences=[]
    comparable=0
    for old,f in zip(timeline,timeline[1:]):
        # Only compare decisions with every pre-existing track confirmed.
        # For these recorded runs that means two prior confirmed tracks,
        # two actual centroids, and no extra active tentative track (profile).
        if len(old['tracks'])!=2 or len(f['clusters'])!=2:
            continue
        if active.get(round(old['t']+raw['t0'],6)) != len(old['tracks']):
            continue
        predicted=[np.array(t['p'])+np.array(t['v'])*np.clip(
            f['t']+raw['t0']-t['stamp'],.001,1.) for t in old['tracks']]
        edges=sorted((float(np.linalg.norm(p-np.array(c))),i,j)
            for i,p in enumerate(predicted) for j,c in enumerate(f['clusters']))
        greedy={}; used=set()
        for d,i,j in edges:
            if d<=.6 and j not in greedy and i not in used:
                greedy[j]=i; used.add(i)
        optimum=assignment(predicted,f['clusters'],.6)
        comparable+=1
        if greedy!=optimum:
            differences.append(dict(t=f['t'],greedy=greedy,global_assignment=optimum))
    disappearance=[]
    for old,f in zip(timeline,timeline[1:]):
        current={t['id'] for t in f['tracks']}
        for tr in old['tracks']:
            if tr['id'] not in current:
                disappearance.append(dict(id=tr['id'],t=f['t'],
                    since_last_observation_s=f['t']+raw['t0']-tr['stamp'],
                    previous_missed=tr['missed']))
    summary['disappearances']=disappearance
    summary['decision_comparison']={'comparable_two_track_two_centroid_frames':comparable,
        'greedy_vs_global_differences':differences,
        'scope':'same recorded pre-decision states; diagnostic, not a counterfactual full replay'}
    summary['retained_frame_fraction_per_gt']=[sum(f['retained_ids'][i] is not None for f in timeline)/len(timeline)
        for i in range(summary['gt_objects'])]
    summary['first_ids']=timeline[0]['ids']
    summary['last_ids']=timeline[-1]['ids']
    summary['gt_min_center_separation_m']=min((f['separation'] for f in timeline if f['separation'] is not None),default=None)
    summary['source_trial']=str(trial)
    summary['raw_sha256']=hashlib.sha256((trial/'raw.json').read_bytes()).hexdigest()
    return summary,timeline


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--regressions',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    trials=[]
    with (args.out/'crossing_timelines.csv').open('x',newline='') as stream:
        fields=['trial','t','gt','ids','retained_ids','visible_returns','merge_groups','clusters','tracks']
        writer=csv.DictWriter(stream,fieldnames=fields,lineterminator="\n")
        writer.writeheader()
        for directory in [args.baseline,args.regressions]:
            for path in sorted(directory.glob('*/summary.json')):
                summary,timeline=analyze(path.parent)
                trials.append(summary)
                if summary['scenario'] in ['crossing','crossing_resolved','occlusion','occlusion_long']:
                    for f in timeline:
                        lower,upper = (1.8,6.8) if 'crossing' in summary['scenario'] else ((2.5,4.5) if summary['scenario']=='occlusion' else (3.8,8.2))
                        if not lower <= f['t'] <= upper:
                            continue
                        def compact(value):
                            if isinstance(value,float): return round(value,5)
                            if isinstance(value,list): return [compact(v) for v in value]
                            if isinstance(value,dict): return {k:compact(v) for k,v in value.items()}
                            return value
                        f=compact(f)
                        writer.writerow({'trial':path.parent.name,**{k:json.dumps(f[k],separators=(',',':'))
                            if isinstance(f[k],(list,dict)) else f[k] for k in fields if k!='trial'}})
    (args.out/'results.json').write_text(json.dumps(trials,indent=2)+'\n')
    with (args.out/'trials.csv').open('x',newline='') as stream:
        fields=['scenario','trial','gt_objects','frames','id_switches','track_fragments','identity_fragments',
            'mean_purity','matched_fraction','position_error_m','velocity_error_mps','false_confirmed_ids',
            'false_persistent_ids','duplicate_frames','merge_frames','max_missed_scans','first_ids','last_ids']
        writer=csv.DictWriter(stream,fieldnames=fields,lineterminator="\n");writer.writeheader()
        for s in trials:
            gs=s['per_gt'];total=sum(g['matched_frames'] for g in gs)
            row={k:s[k] for k in ['scenario','gt_objects','frames','id_switches','track_fragments','duplicate_frames','first_ids','last_ids']}
            row.update(trial=Path(s['source_trial']).name,
                identity_fragments=sum(g['identity_fragments'] for g in gs),
                mean_purity=float(np.mean([g['purity'] for g in gs])) if gs else None,
                matched_fraction=total/(len(gs)*s['frames']) if gs else None,
                false_confirmed_ids=len(s['false_confirmed_ids']),false_persistent_ids=len(s['false_persistent_ids']),
                merge_frames=s['categories'].get('A_cluster_merge',0),max_missed_scans=s['max_missed_scans_retained'])
            for column,key in [('position_error_m','position_error_m'),('velocity_error_mps','velocity_error_mps')]:
                row[column]=sum(g[key]['mean']*g['matched_frames'] for g in gs if g[key])/total if total else None
            writer.writerow(row)
    performance={}
    for objects,name,directory in [(1,'single',args.baseline),(2,'parallel',args.baseline),(3,'triple',args.regressions)]:
        values={k:[] for k in ['scan_us','cluster_us','association_us','callback_us','clusters','tracks']}
        for path in directory.glob(name+'_*/raw.json'):
            raw=json.loads(path.read_text())
            for line in (path.parent/'launch.log').read_text().splitlines():
                m=re.search(r'G2_PERF ([\d.]+) (.*)',line)
                if m and raw['t0']<=float(m[1])<=raw['end']:
                    for key,val in re.findall(r'(\w+)=([\d.]+)',m[2]):
                        values[key].append(float(val))
        performance[objects]={'frames':len(values['callback_us']),**{k:describe(v) for k,v in values.items()}}
    (args.out/'performance.json').write_text(json.dumps(performance,indent=2)+'\n')
    print(json.dumps(performance,indent=2))
    for logfile,name in [('costmap_regression.log','costmap.json'),('lifecycle_regression.log','lifecycle.json')]:
        logfile=args.baseline.parent/logfile
        result=next(json.loads(line) for line in logfile.read_text().splitlines() if line.startswith('{"pass"'))
        (args.out/name).write_text(json.dumps(result,indent=2)+'\n')
    root=Path(__file__).resolve().parents[4]
    protected=['src/predictive_nav_costmap/src/predicted_obstacle_layer.cpp',
        'src/predictive_nav_tracking/config/tracker_params.yaml',
        'src/predictive_nav_tracking/include/predictive_nav_tracking/kalman_filter.hpp',
        'src/predictive_nav_bringup/config/nav2_stage4f_params.yaml']
    manifest={'checkpoint':'eab2218765a19474d15daa26ab7cf7ac64631ba4',
        'stage4f_reference':subprocess.check_output(['git','rev-parse','stage4f-validated^{commit}'],cwd=root,text=True).strip(),
        'protected_files':{f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in protected},
        'protected_files_unchanged':subprocess.check_output(['git','diff','eab2218','--',*protected],cwd=root,text=True)=='',
        'tracker_source_sha256_at_curation':hashlib.sha256((root/'src/predictive_nav_tracking/src/lidar_obstacle_tracker_node.cpp').read_bytes()).hexdigest(),
        'note':'Tracking functions unchanged; only opt-in callback timing instrumentation added. Raw data hashes are recorded per trial.'}
    (args.out/'provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':
    main()
