import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from host_state_machine import HostParameters, _Channel

folder=Path(__file__).parent
audit=json.loads((folder/'audit.json').read_text(encoding='utf-8'))
rows=json.loads((folder/'ch11_rows.json').read_text(encoding='utf-8'))
prototype=json.loads((folder/'prototype_replay.json').read_text(encoding='utf-8'))
seed_index=next(i for i,r in enumerate(rows) if i>20 and r['state']==0 and all(x['state']==0 for x in rows[i-10:i]))
targets={c['end']['sequence']:c for c in audit['reload_examples']}
results=[]
for margin in (0,300):
    params=dict(audit['parameters'],reload_reference_margin=margin)
    p=HostParameters.from_settings(params)
    c=_Channel(p.window)
    seed=rows[seed_index]
    c.commit_state(0)
    c.input,c.baseline=seed['input'],seed['baseline']
    c.started=seed['time']-p.initial_seconds
    c.history.extend((r['time'],r['input']) for r in rows[seed_index-p.window+1:seed_index+1])
    errors={k:0 for k in ('state','input','baseline','signal','rate')}
    transitions=[]
    examples=[]
    for row in rows[seed_index+1:]:
        old=c.state
        c.feed(row['raw']*audit['gains'][11],row['time'],True,p)
        for key,value in (('state',c.state),('input',c.input),('baseline',c.baseline),('signal',c.signal),('rate',c.decision_rate)):
            expected=row[key]
            if math.isnan(value) and math.isnan(expected):continue
            if not math.isfinite(value) or not math.isfinite(expected) or abs(value-expected)>1e-7:errors[key]+=1
        if old!=c.state:transitions.append(dict(sequence=row['sequence'],old=old,new=c.state,input=c.input,baseline=c.baseline,reference=c.release_reference))
        if row['sequence'] in targets:
            target=targets[row['sequence']]
            examples.append(dict(timestamp=row['timestamp'],recorded_state=row['state'],new_state=c.state,input=c.input,
                baseline=c.baseline,release_reference=c.release_reference,new_signal=c.signal,
                old_signal=row['signal'],old_event=row['activation'],
                recorded_pre_release_baseline=target['reference'],recorded_minimum=target['minimum']))
    expected=next(r for r in prototype if r['margin']==margin)['transitions']
    mismatches=sum(any(t[k]!=r[k] for k in ('sequence','old','new','input','baseline','reference')) for t,r in zip(transitions,expected))
    assert len(transitions)==len(expected) and mismatches==0
    if margin==0:assert not any(errors.values()),errors
    results.append(dict(margin=margin,seed=seed,replayed_samples=len(rows)-seed_index-1,
        original_log_mismatches=errors if margin==0 else None,prototype_mismatches=mismatches,
        recover_reloads=sum(t['old']==3 and t['new']==1 for t in transitions),
        slowback_reloads=sum(t['old']==2 and t['new']==1 for t in transitions),
        idle_activations=sum(t['old']==0 and t['new']==1 for t in transitions),examples=examples,
        selected_rebound_samples_blocked=sum(e['new_state']!=1 for e in examples)))
(folder/'final_replay.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
for r in results:
    print('margin',r['margin'],'samples',r['replayed_samples'],'legacy errors',r['original_log_mismatches'],
        'prototype errors',r['prototype_mismatches'],'recover reloads',r['recover_reloads'],
        'examples blocked',r['selected_rebound_samples_blocked'],len(r['examples']))
