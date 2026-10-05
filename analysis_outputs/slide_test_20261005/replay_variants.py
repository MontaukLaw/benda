import json
from pathlib import Path
import statistics
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker
from centroid_filter import OneEuroAxis

output = Path(__file__).parent
audit = json.loads((output/'audit.json').read_text(encoding='utf-8'))
with zipfile.ZipFile(audit['archive']) as z:
    meta=json.loads(z.read('manifest.json'))
    records=[json.loads(line) for line in z.read('records.jsonl').splitlines()]
touch=[r['frame'] for r in records if r['kind']=='touchpad']
ends={r['frame']['phase_index']:r['frame']['sample_timestamp'] for r in records if r['kind']=='slide_test' and r['frame'].get('event')=='phase_end'}
phases=meta['phases']
intervals=[dict(axis=p['axis'],trial=p['trial'],start=p['sample_timestamp'],
    end=ends[p['phase_index']+1],slide_end=ends[p['phase_index']]) for p in phases if p['phase']=='slide']
variants={}
for drop in (15,20,25,30,50,'disabled'):
    settings=dict(meta['context']['touchpad'])
    settings.pop('source')
    settings['raw_release_drop_percent']=drop if isinstance(drop,int) else 80
    tracker=TouchpadTracker(**settings)
    tracker.set_source('host_signal')
    tracker.set_enabled(True)
    if drop=='disabled':
        tracker._release_guard=lambda *args,**kwargs:(False,None)
    results=[dict(p,net=[0.,0.],forward=0.,reverse=0.,frames=0,segments=0,blocks={}) for p in intervals]
    previous=False
    for t in touch:
        tracker.feed(dict(sequence=t['source_sample_sequence'],sample_timestamp=t['sample_timestamp'],
            has_raw=True,channel_count=20,raw=[1]*20,host_signal=t['host_signal'],host_state=t['host_state'],
            host_generation=t['host_generation'],host_ready=True))
        for p in results:
            if p['start']<=t['sample_timestamp']<p['end']:
                axis=0 if p['axis']=='horizontal' else 1
                p['net']=[a+b for a,b in zip(p['net'],tracker.delta)]
                p['forward']+=max(0,tracker.delta[axis])
                p['reverse']+=max(0,-tracker.delta[axis])
                p['frames']+=tracker.contact
                p['segments']+=tracker.contact and not previous
                if tracker.motion_block_reason:
                    k=tracker.motion_block_reason
                    p['blocks'][k]=p['blocks'].get(k,0)+1
                break
        previous=tracker.contact
    summary={}
    for axis in ('horizontal','vertical'):
        a=0 if axis=='horizontal' else 1
        group=[p for p in results if p['axis']==axis]
        values=[p['net'][a] for p in group]
        summary[axis]=dict(mean=statistics.mean(values),cv=statistics.stdev(values)/statistics.mean(values),
            values=values,mean_reverse=statistics.mean(p['reverse'] for p in group),
            max_reverse=max(p['reverse'] for p in group),cross=[p['net'][1-a] for p in group],
            segments=sum(p['segments'] for p in group))
    variants[str(drop)]=dict(summary=summary,trials=results)
    print('GUARD',drop,json.dumps(summary,ensure_ascii=False))

series=json.loads((output/'series.json').read_text(encoding='utf-8'))
for key in ('soft','fixed_reference'):
    results=[]
    for s in series:
        filters=[OneEuroAxis(2.5,.8),OneEuroAxis(2.5,.8)]
        old=previous=None
        net=[0.,0.]
        for t in s['traces']:
            xy=t[key]
            if not t['contact'] or xy is None:
                for f in filters:f.reset()
                old=None
            else:
                dt=t['time']-previous['time'] if previous else .01
                new=[f.update(v,max(.001,dt)) for f,v in zip(filters,xy)]
                if old:
                    net=[a+400*(b-c) for a,b,c in zip(net,new,old)]
                old=new
            previous=t
        results.append(dict(axis=s['axis'],trial=s['trial'],net=net))
    summary={}
    for axis in ('horizontal','vertical'):
        a=0 if axis=='horizontal' else 1
        group=[p for p in results if p['axis']==axis]
        values=[p['net'][a] for p in group]
        summary[axis]=dict(mean=statistics.mean(values),cv=statistics.stdev(values)/statistics.mean(values),
            values=values,cross=[p['net'][1-a] for p in group])
    variants[key]=dict(summary=summary,trials=results)
    print('POSITION',key,json.dumps(summary,ensure_ascii=False))
(output/'variants.json').write_text(json.dumps(variants,ensure_ascii=False,indent=2),encoding='utf-8')
