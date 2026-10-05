import collections
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from host_state_machine import HostParameters, _Channel

folder=Path(__file__).parent
audit=json.loads((folder/'audit.json').read_text(encoding='utf-8'))
rows=json.loads((folder/'ch6_rows.json').read_text(encoding='utf-8'))
start=next(i for i,r in enumerate(rows) if i>20 and r['state']==0 and all(x['state']==0 for x in rows[i-10:i]))
targets={r['sequence']:r for r in rows if r['activation'] and r['activation'][0]['from_state']==0 and abs(r['rate'])<20}
variants=[{},dict(activate_threshold=500.),dict(activate_threshold=600.),dict(activate_threshold=800.),
    dict(recover_frames=20),dict(recover_frames=30),dict(stable_rate=.5,recover_frames=20),
    dict(activate_threshold=500.,stable_rate=.5,recover_frames=20),
    dict(reload_reference_margin=1000.),dict(reload_reference_margin=1700.),
    dict(activate_threshold=500.,reload_reference_margin=1000.)]
results=[]
for changes in variants:
    params=dict(audit['parameters'],**changes)
    p=HostParameters.from_settings(params)
    c=_Channel(p.window)
    seed=rows[start]
    c.commit_state(0)
    c.input,c.baseline=seed['input'],seed['baseline']
    c.started=seed['time']-p.initial_seconds
    c.history.extend((r['time'],r['input']) for r in rows[start-p.window+1:start+1])
    errors=collections.Counter()
    counts=collections.Counter()
    acts=[]
    examples=[]
    last_activation=None
    runs=[]
    for row in rows[start+1:]:
        old=c.state
        c.feed(row['raw']*audit['gains'][6],row['time'],True,p)
        counts[c.state]+=1
        if not changes:
            for key,value in (('state',c.state),('input',c.input),('baseline',c.baseline),('signal',c.signal),('rate',c.decision_rate)):
                expected=row[key]
                if math.isnan(value) and math.isnan(expected):continue
                if not math.isfinite(value) or not math.isfinite(expected) or abs(value-expected)>1e-7:errors[key]+=1
        if old!=1 and c.state==1:
            acts.append(dict(timestamp=row['timestamp'],sequence=row['sequence'],**c.last_activation))
            last_activation=acts[-1]
            runs.append(dict(start=row['time'],timestamp=row['timestamp'],end=row['time'],source=old))
        if c.state==1 and runs:runs[-1]['end']=row['time']
        if row['sequence'] in targets:
            examples.append(dict(timestamp=row['timestamp'],recorded_state=row['state'],new_state=c.state,
                old_baseline=row['baseline'],new_baseline=c.baseline,input=c.input,reference=c.release_reference))
    results.append(dict(changes=changes,states=dict(counts),activations_by_source=dict(collections.Counter(a['from_state'] for a in acts)),
        active_runs=[dict(run,duration=run['end']-run['start']) for run in runs],
        legacy_mismatches=dict(errors),examples=examples,activations=acts))
(folder/'parameter_replay.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
for r in results:
    print('VARIANT',r['changes'],'activations',r['activations_by_source'],'active samples',r['states'].get(1,0),'errors',r['legacy_mismatches'])
    print('low-rate target points blocked',sum(e['new_state']!=1 for e in r['examples']),len(r['examples']))
    print('longest idle-triggered runs',[(x['timestamp'],round(x['duration'],2)) for x in sorted([x for x in r['active_runs'] if x['source']==0],key=lambda x:x['duration'],reverse=True)[:4]])
    print('last slow rebound',[e for e in r['examples'] if '11:41:53' in e['timestamp']])
