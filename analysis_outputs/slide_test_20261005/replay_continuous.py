"""对固定测试包逐采样重放连续权重方案，保留旧模式对照。"""
import json
import hashlib
from pathlib import Path
import statistics
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker

output = Path(__file__).parent
audit = json.loads((output/'audit.json').read_text(encoding='utf-8'))
with zipfile.ZipFile(audit['archive']) as z:
    meta = json.loads(z.read('manifest.json'))
    data = z.read('records.jsonl')
    assert hashlib.sha256(data).hexdigest()==meta['records_sha256']==audit['sha256']
    records = [json.loads(line) for line in data.splitlines()]
touch = [r['frame'] for r in records if r['kind']=='touchpad']
hosts = {r['frame']['sequence']:r['frame'] for r in records if r['kind']=='host_state_machine'}
ends = {r['frame']['phase_index']:r['frame']['sample_timestamp'] for r in records
        if r['kind']=='slide_test' and r['frame'].get('event')=='phase_end'}
intervals = [dict(axis=p['axis'],trial=p['trial'],start=p['sample_timestamp'],
                 end=ends[p['phase_index']+1]) for p in meta['phases'] if p['phase']=='slide']
variants = {}
for source in ('host_signal','host_continuous'):
    settings = dict(meta['context']['touchpad']); settings.pop('source')
    tracker = TouchpadTracker(**settings); tracker.set_source(source); tracker.set_enabled(True)
    results = [dict(p,net=[0.,0.],segments=0,contact_frames=0,blocks={},steps=[]) for p in intervals]
    previous = False
    for t in touch:
        h = hosts.get(t['source_sample_sequence'])
        if h is None: continue
        tracker.feed(dict(sequence=t['source_sample_sequence'], sample_timestamp=t['sample_timestamp'],
                          has_raw=True,channel_count=20,raw=[1]*20,host_signal=t['host_signal'],
                          host_state=t['host_state'],host_generation=t['host_generation'],host_ready=True,
                          host_input_val=h['input_val']))
        for p in results:
            if p['start']<=t['sample_timestamp']<p['end']:
                p['net'] = [a+b for a,b in zip(p['net'],tracker.delta)]
                p['segments'] += tracker.contact and not previous
                p['contact_frames'] += tracker.contact
                if tracker.contact: p['steps'].append(max(abs(v) for v in tracker.centroid_delta))
                if tracker.motion_block_reason:
                    k=tracker.motion_block_reason; p['blocks'][k]=p['blocks'].get(k,0)+1
                break
        previous=tracker.contact
    summary = {}
    for axis in ('horizontal','vertical'):
        a=0 if axis=='horizontal' else 1
        group=[p for p in results if p['axis']==axis]; values=[p['net'][a] for p in group]
        steps=sorted(v for p in group for v in p['steps'])
        summary[axis]=dict(mean=statistics.mean(values),cv=statistics.stdev(values)/statistics.mean(values)
                          if statistics.mean(values) else None,values=values,cross=[p['net'][1-a] for p in group],
                          segments=sum(p['segments'] for p in group),p95_step=steps[int(.95*len(steps))] if steps else None)
    variants[source]=dict(summary=summary,trials=results)
    print(source,json.dumps(summary,ensure_ascii=False))
matched = {}
for axis in ('horizontal','vertical'):
    a=0 if axis=='horizontal' else 1
    pairs=list(zip(variants['host_signal']['trials'],variants['host_continuous']['trials']))
    pairs=[(old,new) for old,new in pairs if new['axis']==axis and new['contact_frames']]
    matched[axis]=dict(trials=[new['trial'] for _,new in pairs])
    for mode,index in (('host_signal',0),('host_continuous',1)):
        values=[pair[index]['net'][a] for pair in pairs]
        matched[axis][mode]=dict(mean=statistics.mean(values),cv=statistics.stdev(values)/statistics.mean(values))
variants['comparison']=dict(matched=matched,geometry=dict(pitch_mm=8.,diameter_mm=5.),
    excluded=[dict(axis=p['axis'],trial=p['trial'],reason='position reference unavailable at archived test start')
              for p in variants['host_continuous']['trials'] if not p['contact_frames']],
    note='Counterfactual replay only. New UI waits for reference before first slide. Unequal unknown actual axis distances prevent physical sensitivity calibration.')
print('MATCHED',json.dumps(matched,ensure_ascii=False))
(output/'continuous_results.json').write_text(json.dumps(variants,ensure_ascii=False,indent=2),encoding='utf-8')
