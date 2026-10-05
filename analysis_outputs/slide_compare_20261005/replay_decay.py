"""在相同记录输入及参考下验证残留衰减，避免两次手动动作混淆。"""
import bisect
import json
from pathlib import Path
import sys
import zipfile
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker
from compare import trial_metrics,summarize
OUT=Path(__file__).parent
result={}
for mode in ('host_continuous','host_signal'):
    report=json.loads((OUT/(mode+'.json')).read_text(encoding='utf-8'))
    with zipfile.ZipFile(report['archive']) as z:records=[json.loads(line) for line in z.read('records.jsonl').splitlines()]
    hosts={r['frame']['sequence']:r['frame'] for r in records if r['kind']=='host_state_machine'}
    touch=[r['frame'] for r in records if r['kind']=='touchpad'];ts=[t['sample_timestamp'] for t in touch]
    variants={}
    for decay,restore,strength in ((0,20,60),(80,20,40),(80,20,60),(80,20,80),(80,20,100)):
        settings={k:v for k,v in report['context']['touchpad'].items() if k!='source'}
        settings.update(position_decay_enabled=decay>0,position_decay_ms=decay or 80,position_restore_ms=restore,position_decay_percent=strength)
        tracker=TouchpadTracker(**settings);tracker.set_source('host_continuous');tracker.set_enabled(True)
        if mode=='host_continuous':tracker._update_position_reference=lambda *args:False
        old_reference=None;rows=[]
        for t in touch:
            h=hosts[t['source_sample_sequence']]
            if mode=='host_continuous':
                reference=t['position_reference']
                if reference!=old_reference:tracker._release_samples.clear();tracker.release_blocked=False
                tracker.position_reference=reference;tracker.position_noise=t['position_noise'];old_reference=reference
            tracker.feed(dict(sequence=h['sequence'],sample_timestamp=h['sample_timestamp'],has_raw=True,channel_count=20,
                raw=[1]*20,host_signal=h['signal'],host_state=h['state'],host_input_val=h['input_val'],
                host_generation=h['generation'],host_ready=all(s!=-1 for s in h['state'])))
            rows.append(dict(t,dx=tracker.delta[0],dy=tracker.delta[1],raw_xy=tracker.raw_position,filtered_xy=tracker.position,
                contact=tracker.contact,motion_block_reason=tracker.motion_block_reason))
        trials=[]
        for p in report['trials']:
            phase=dict(axis=p['axis'],trial=p['trial'],sample_timestamp=p['start'],end=p['slide_end'],timestamp=p['timestamp'])
            values=rows[bisect.bisect_left(ts,p['start']):bisect.bisect_left(ts,p['end'])]
            trials.append(trial_metrics(values,phase,p['end']))
        valid=[t for t in trials if t['segments']]
        summary=summarize(valid)
        for t in trials:t.pop('traces');t.pop('_raw_steps');t.pop('_output_steps')
        variants[f'{decay}/{restore}/{strength}']=dict(summary=summary,trials=trials,
            excluded=[dict(axis=t['axis'],trial=t['trial']) for t in trials if not t['segments']])
        print(mode,decay,restore,strength,json.dumps(summary,ensure_ascii=False),flush=True)
    result[mode]=variants
(OUT/'decay_replay.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
