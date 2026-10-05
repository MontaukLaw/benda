"""核对两套16毫米测试包，并比较实际输出及同输入模式回放。"""
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics as st
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker

OUT=Path(__file__).parent
FOLDER=ROOT/'PY_SRIPT/algo_monitor/recordings/slide_tests'
ARCHIVES=sorted(FOLDER.glob('slide_test_20261005_14*.zip'))[-2:]

def pct(values,p=.95):
    values=sorted(values)
    return values[round((len(values)-1)*p)] if values else 0.

def summarize(trials):
    result={}
    for axis in ('horizontal','vertical'):
        group=[p for p in trials if p['axis']==axis]
        main=[p['main'] for p in group]; cross=[p['cross'] for p in group]
        result[axis]=dict(mean=st.mean(main),sd=st.stdev(main),cv=st.stdev(main)/abs(st.mean(main)) if st.mean(main) else None,
            min=min(main),max=max(main),values=main,cross=cross,mean_cross=st.mean(cross),mean_abs_cross=st.mean(abs(v) for v in cross),
            mean_abs_cross_ratio=st.mean(abs(p['cross']/p['main']) for p in group if p['main']),
            mean_reverse=st.mean(p['reverse'] for p in group),max_reverse=max(p['reverse'] for p in group),
            segments=sum(len(p['segments']) for p in group),
            stop_reasons=dict(Counter(s.get('reason') for p in group for s in p['segments'])),
            sharp_steps=sum(p['sharp_steps'] for p in group),sharp_with_state_change=sum(p['sharp_state_change'] for p in group),
            raw_step_p95=pct([v for p in group for v in p['_raw_steps']]),
            output_step_p95=pct([v for p in group for v in p['_output_steps']]),
            release_cue_motion_trials=sum(math.hypot(*p['release_net'])>1 for p in group))
    return result

def trial_metrics(rows,phase,end):
    axis=0 if phase['axis']=='horizontal' else 1
    start=phase['sample_timestamp']; slide_end=phase['end']
    net=[sum(r[k] for r in rows) for k in ('dx','dy')]
    release=[sum(r[k] for r in rows if r['sample_timestamp']>=slide_end) for k in ('dx','dy')]
    segments=[]; current=None; previous=None; raw_steps=[]; output_steps=[]; sharp=sharp_state=0
    cum=[0.,0.]; traces=[]; largest=[]
    for r in rows:
        time=r['sample_timestamp']-start
        delta=[r['dx'],r['dy']]
        cum=[a+b for a,b in zip(cum,delta)]
        if r['contact']:
            if current is None:
                current=dict(start=time,first=r['raw_xy'],net=[0.,0.],first_sequence=r['source_sample_sequence'],frames=0)
            current['net']=[a+b for a,b in zip(current['net'],delta)]; current['frames']+=1
            current['end']=time;current['last']=r['raw_xy'];current['last_sequence']=r['source_sample_sequence']
        elif current is not None:
            current['reason']=r.get('motion_block_reason');segments.append(current);current=None
        if r['contact'] and previous and previous['contact']:
            d=[a-b for a,b in zip(r['raw_xy'],previous['raw_xy'])]
            length=math.hypot(*d); raw_steps.append(length);output_steps.append(math.hypot(*delta))
            changed=[ch for ch,(a,b) in enumerate(zip(previous['host_state'],r['host_state'])) if a!=b]
            sharp+=length>.1;sharp_state+=length>.1 and bool(changed)
            if length>.1:
                largest.append(dict(sequence=r['source_sample_sequence'],time=time,delta=d,pixel_delta=delta,
                    old=previous['raw_xy'],new=r['raw_xy'],changed=changed,old_state=previous['host_state'],new_state=r['host_state']))
        traces.append(dict(time=time,contact=r['contact'],raw=r['raw_xy'],position=r['filtered_xy'],
            cum=cum,delta=delta,block=r.get('motion_block_reason'),states=r['host_state'],sequence=r['source_sample_sequence']))
        previous=r
    if current: current['reason']='window_end';segments.append(current)
    return dict(axis=phase['axis'],trial=phase['trial'],start=start,end=end,slide_end=slide_end,timestamp=phase['timestamp'],
        net=net,main=net[axis],cross=net[1-axis],release_net=release,
        forward=sum(max(0,(r['dx'],r['dy'])[axis]) for r in rows),reverse=sum(max(0,-(r['dx'],r['dy'])[axis]) for r in rows),
        first_contact=min((r['sample_timestamp']-start for r in rows if r['contact']),default=None),
        last_contact=max((r['sample_timestamp']-start for r in rows if r['contact']),default=None),
        segments=segments,sharp_steps=sharp,sharp_state_change=sharp_state,
        blocks=dict(Counter(r.get('motion_block_reason') for r in rows)),
        largest_steps=sorted(largest,key=lambda v:math.hypot(*v['delta']),reverse=True)[:8],
        _raw_steps=raw_steps,_output_steps=output_steps,traces=traces)

def analyze(archive):
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        meta=json.loads(z.read('manifest.json')); data=z.read('records.jsonl')
    sha=hashlib.sha256(data).hexdigest()
    assert sha==meta['records_sha256']
    assert hashlib.sha256((archive.with_suffix('')/'records.jsonl').read_bytes()).hexdigest()==sha
    lines=data.splitlines();records=[json.loads(line) for line in lines];del data
    host={r['frame']['sequence']:r['frame'] for r in records if r['kind']=='host_state_machine'}
    touch=[r['frame'] for r in records if r['kind']=='touchpad']
    ends={r['frame']['phase_index']:r['frame']['sample_timestamp'] for r in records if r['kind']=='slide_test' and r['frame'].get('event')=='phase_end'}
    phases=[dict(p,end=ends.get(p['phase_index'])) for p in meta['phases']]
    intervals=[dict(p,window_end=phases[p['phase_index']+1]['end']) for p in phases if p['phase']=='slide']
    ts=[t['sample_timestamp'] for t in touch]
    trials=[trial_metrics(touch[bisect.bisect_left(ts,p['sample_timestamp']):bisect.bisect_left(ts,p['window_end'])],p,p['window_end']) for p in intervals]
    mismatches=Counter();reference_changes=Counter();last=None
    for t in touch:
        h=host.get(t['source_sample_sequence'])
        if h is None: mismatches['missing_host']+=1;continue
        for tk,hk in (('host_signal','signal'),('host_state','state'),('host_input_val','input_val'),('host_baseline','baseline')):
            if t.get(tk)!=h[hk]:mismatches[tk]+=1
        reference=t.get('position_reference')
        if reference is not None and last is not None and reference!=last:
            reference_changes['updates']+=1
            if any(s not in (0,-2) for s in t['host_state']):reference_changes['while_nonidle']+=1
        last=reference
        if reference:
            expected=[max(0,v-b) if s!=-2 else 0 for v,b,s in zip(h['input_val'],reference,h['state'])]
            expected=[v*(1-math.exp(-(v/max(n,t['settings']['position_noise_floor']))**2)) for v,n in zip(expected,t['position_noise'])]
            if max(abs(a-b) for a,b in zip(expected,t['position_weights']))>1e-7:mismatches['position_weights']+=1
    derived_gaps={kind:[(a,b) for a,b in zip(values,values[1:]) if b!=a+1] for kind,values in
        (('host',list(host)),('touch',[t['source_sample_sequence'] for t in touch]))}
    summary=summarize(trials)
    alternate='host_signal' if meta['context']['touchpad']['source']=='host_continuous' else 'host_continuous'
    tracker=TouchpadTracker(**{k:v for k,v in meta['context']['touchpad'].items() if k!='source'})
    tracker.set_source(alternate);tracker.set_enabled(True);replayed=[]
    for t in touch:
        h=host[t['source_sample_sequence']]
        tracker.feed(dict(sequence=h['sequence'],sample_timestamp=h['sample_timestamp'],has_raw=True,channel_count=20,
            raw=[1]*20,host_signal=h['signal'],host_state=h['state'],host_input_val=h['input_val'],
            host_generation=h['generation'],host_ready=all(s!=-1 for s in h['state'])))
        replayed.append(dict(t,dx=tracker.delta[0],dy=tracker.delta[1],raw_xy=tracker.raw_position,filtered_xy=tracker.position,
            contact=tracker.contact,motion_block_reason=tracker.motion_block_reason))
    alt_trials=[trial_metrics(replayed[bisect.bisect_left(ts,p['sample_timestamp']):bisect.bisect_left(ts,p['window_end'])],p,p['window_end']) for p in intervals]
    result=dict(archive=str(archive),started=meta['started_at'],ended=meta['ended_at'],outcome=meta['outcome'],sha256=sha,
        context=meta['context'],counts=dict(Counter(r['kind'] for r in records)),manifest_counts=meta['counts'],
        parsed_types=dict(Counter(r['frame']['type'] for r in records if r['kind']=='frame')),
        log_sequence_gaps=[(a['sequence'],b['sequence']) for a,b in zip(records,records[1:]) if b['sequence']!=a['sequence']+1],
        events=[r for r in records if r['kind'] in ('connected','disconnected','serial_error','tx_error','log_gap')],
        ports=dict(Counter(r.get('port') for r in records)),derived_sequence_gaps=derived_gaps,
        manifest_gaps=meta['derived_sequence_gaps'],max_sensor_gap=meta['largest_sensor_gap_seconds'],
        host_generations=dict(Counter(h['generation'] for h in host.values())),host_touch_mismatches=dict(mismatches),
        position_reference_changes=dict(reference_changes),summary=summary,trials=trials,
        alternate_source=alternate,alternate_summary=summarize(alt_trials),alternate_trials=alt_trials,
        first_sequence=records[0]['sequence'],last_sequence=records[-1]['sequence'])
    hashes={r['sequence']:hashlib.sha256(line).hexdigest() for r,line in zip(records,lines)}
    print('RESULT',meta['context']['touchpad']['source'],json.dumps(summary,ensure_ascii=False),flush=True)
    print('ALTERNATE',alternate,json.dumps(result['alternate_summary'],ensure_ascii=False),flush=True)
    (OUT/(meta['context']['touchpad']['source']+'.json')).write_text(json.dumps(result,ensure_ascii=False),encoding='utf-8')
    (OUT/(meta['context']['touchpad']['source']+'_record_hashes.json')).write_text(json.dumps(hashes),encoding='utf-8')
    return result

if __name__=='__main__':
    results=[analyze(a) for a in ARCHIVES]
    a,b=[r['context'] for r in results]
    comparison=dict(sources=[c['touchpad']['source'] for c in (a,b)],
        parameter_differences={k:[a[k],b[k]] for k in ('raw_gains','host_parameters','hardware_signature','channel_grid','sensor_geometry','protocol') if a.get(k)!=b.get(k)},
        touchpad_differences={k:[a['touchpad'].get(k),b['touchpad'].get(k)] for k in set(a['touchpad'])|set(b['touchpad']) if a['touchpad'].get(k)!=b['touchpad'].get(k)},
        summary={r['context']['touchpad']['source']:r['summary'] for r in results})
    (OUT/'comparison.json').write_text(json.dumps(comparison,ensure_ascii=False,indent=2),encoding='utf-8')
    print('COMPARISON',json.dumps(comparison,ensure_ascii=False),flush=True)
