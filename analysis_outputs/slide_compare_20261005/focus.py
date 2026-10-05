"""按日志中的参考重放实际位移，并提取起终点压力分布。"""
from collections import Counter
import json
import math
from pathlib import Path
import statistics as st
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker
from centroid_filter import signal_centroid
OUT=Path(__file__).parent

results={}
for mode in ('host_continuous','host_signal'):
    report=json.loads((OUT/(mode+'.json')).read_text(encoding='utf-8'))
    with zipfile.ZipFile(report['archive']) as z:records=[json.loads(line) for line in z.read('records.jsonl').splitlines()]
    hosts={r['frame']['sequence']:r['frame'] for r in records if r['kind']=='host_state_machine'}
    touch=[r['frame'] for r in records if r['kind']=='touchpad']
    tracker=TouchpadTracker(**{k:v for k,v in report['context']['touchpad'].items() if k!='source'})
    tracker.set_source(mode);tracker.set_enabled(True)
    # 连续模式固定采用原日志的逐帧参考，单独验证质心、卸载门控和位移链路。
    if mode=='host_continuous':tracker._update_position_reference=lambda *args:False
    errors=Counter();max_error=0.;old_reference=None
    for t in touch:
        h=hosts[t['source_sample_sequence']]
        if mode=='host_continuous':
            reference=t.get('position_reference')
            if reference!=old_reference:
                tracker._release_samples.clear();tracker.release_blocked=False
            tracker.position_reference=reference;tracker.position_noise=t.get('position_noise');old_reference=reference
        tracker.feed(dict(sequence=h['sequence'],sample_timestamp=h['sample_timestamp'],has_raw=True,channel_count=20,
            raw=[1]*20,host_signal=h['signal'],host_state=h['state'],host_input_val=h['input_val'],
            host_generation=h['generation'],host_ready=all(s!=-1 for s in h['state'])))
        e=max(abs(a-b) for a,b in zip(tracker.delta,(t['dx'],t['dy'])));max_error=max(e,max_error)
        if e>1e-7:errors['delta']+=1
        if tracker.contact!=t['contact']:errors['contact']+=1
    trials=[]
    for trial in report['trials']:
        a=0 if trial['axis']=='horizontal' else 1
        rows=[t for t in touch if trial['start']<=t['sample_timestamp']<trial['end']]
        active=[t for t in rows if t['contact']]
        before=[h for h in hosts.values() if trial['start']-.5<=h['sample_timestamp']<trial['start']]
        reference=active[0].get('position_reference') or [st.mean(h['input_val'][ch] for h in before) for ch in range(20)]
        def describe(t):
            h=hosts[t['source_sample_sequence']];diff=[max(0,v-b) for v,b in zip(h['input_val'],reference)]
            total=sum(diff);row=[sum(diff[row+4*c] for c in range(5))/total for row in range(4)]
            col=[sum(diff[4*c:4*c+4])/total for c in range(5)]
            nonactive=sum(v for v,s in zip(diff,h['state']) if s!=1)/total
            state_share={str(s):sum(v for v,state in zip(diff,h['state']) if state==s)/total for s in set(h['state'])}
            activate_xy=signal_centroid([v if s==1 else 0 for v,s in zip(diff,h['state'])],local_2x2_enabled=False)
            old=hosts[h['sequence']-1]['input_val'];alpha=report['context']['host_parameters']['input_alpha']
            # 此反推口径已在rolling_audit中逐帧与实际RX的Raw核对，不另做输入滤波。
            instant=[max(0,(v-(1-alpha)*p)/alpha-b) for v,p,b in zip(h['input_val'],old,reference)]
            raw_total=sum(instant)
            raw_slowback_share=sum(v for v,s in zip(instant,h['state']) if s==2)/raw_total
            return dict(sequence=h['sequence'],xy=t['raw_xy'],soft_xy=signal_centroid(diff,local_2x2_enabled=False),
                total=total,row=row,col=col,nonactive_weight_share=nonactive,state_weight_share=state_share,
                activate_only_xy=activate_xy,
                instant_raw_xy=signal_centroid(instant,local_2x2_enabled=False),instant_raw_slowback_share=raw_slowback_share,
                largest_channels=sorted(range(20),key=lambda ch:diff[ch],reverse=True)[:6])
        first,last=describe(active[0]),describe(active[-1])
        peak=max(sum(max(0,v-b) for v,b in zip(hosts[t['source_sample_sequence']]['input_val'],reference)) for t in active)
        stop=next((t for t in rows if t['sample_timestamp']>active[-1]['sample_timestamp']),None)
        cumulative=0.;quarter_cross=None
        for t in active:
            cumulative+=(t['dx'],t['dy'])[a]
            if cumulative>=trial['main']*.75:
                quarter_cross=sum((r['dx'],r['dy'])[1-a] for r in active if r['sample_timestamp']<=t['sample_timestamp']);break
        trials.append(dict(axis=trial['axis'],trial=trial['trial'],first=first,last=last,peak_pressure=peak,
            duration=active[-1]['sample_timestamp']-active[0]['sample_timestamp'],
            stop_sequence=stop['source_sample_sequence'] if stop else None,
            final_quarter_cross=trial['cross']-(quarter_cross or 0.),
            filter_net_vs_raw_endpoint=[trial['net'][i]-400*(last['xy'][i]-first['xy'][i]) for i in (0,1)]))
    results[mode]=dict(max_replay_pixel_error=max_error,replay_errors=dict(errors),trials=trials)
    print('REPLAY',mode,max_error,dict(errors))
    print('FOOTPRINT',mode,json.dumps(trials,ensure_ascii=False))
(OUT/'focus.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
