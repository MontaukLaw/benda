import importlib.util
import json
import math
from pathlib import Path
import sys
import types

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
folder=Path(__file__).parent
audit=json.loads((folder/'audit.json').read_text(encoding='utf-8'))
rows=json.loads((folder/'ch11_rows.json').read_text(encoding='utf-8'))
text=(folder/'legacy_host_state_machine.py').read_text(encoding='utf-8')
module=types.ModuleType('prototype_host')
sys.modules[module.__name__]=module
text=text.replace('and difference > p.activate_threshold)',
    'and difference > p.activate_threshold and (self.margin <= 0 or self.input > (self.reference if self.reference is not None else self.baseline) + self.margin))')
text=text.replace('reloading = (rate >= p.reload_rate',
    'rising_candidate = (rate >= p.reload_rate and (self.margin <= 0 or self.input > (self.reference if self.reference is not None else self.baseline)))\n            reloading = (rising_candidate')
text=text.replace('elif not rate >= p.reload_rate:','elif not rising_candidate:')
exec(compile(text,'prototype_host','exec'),module.__dict__)

class Candidate(module._Channel):
    def __init__(self,window,margin):
        super().__init__(window)
        self.reference=None
        self.margin=margin
    def commit_state(self,state,activation=None):
        if state==2 and self.state==1:self.reference=max(self.baseline,self.reference if self.reference is not None else self.baseline)
        if state in (-2,0):self.reference=None
        super().commit_state(state,activation)

p=module.HostParameters.from_settings(audit['parameters'])
start=next(i for i,r in enumerate(rows) if i>20 and r['state']==0 and all(x['state']==0 for x in rows[i-10:i]))
results=[]
targets={c['end']['sequence'] for c in audit['reload_examples']}
for margin in (0,200,300,500,800,1700):
    c=Candidate(p.window,margin)
    seed=rows[start]
    c.commit_state(0)
    c.input,c.baseline=seed['input'],seed['baseline']
    c.started=seed['time']-p.initial_seconds
    c.history.extend((r['time'],r['input']) for r in rows[start-p.window+1:start+1])
    transitions=[]
    mismatches={k:0 for k in ('state','input','baseline','signal','rate')}
    examples=[]
    for row in rows[start+1:]:
        old=c.state
        c.feed(row['raw']*audit['gains'][11],row['time'],True,p)
        for key,actual in (('state',c.state),('input',c.input),('baseline',c.baseline),('signal',c.signal),('rate',c.decision_rate)):
            expected=row[key]
            if math.isnan(actual) and math.isnan(expected):continue
            if not math.isfinite(actual) or not math.isfinite(expected) or abs(actual-expected)>1e-7:mismatches[key]+=1
        if old!=c.state:transitions.append(dict(timestamp=row['timestamp'],sequence=row['sequence'],old=old,new=c.state,
            input=c.input,baseline=c.baseline,reference=c.reference,activation=c.last_activation if c.state==1 else None))
        if row['sequence'] in targets:examples.append(dict(timestamp=row['timestamp'],recorded_state=row['state'],new_state=c.state,input=c.input,baseline=c.baseline,reference=c.reference))
    results.append(dict(margin=margin,seed=seed,legacy_mismatches=mismatches,transitions=transitions,
        recover_reloads=sum(t['old']==3 and t['new']==1 for t in transitions),
        slowback_reloads=sum(t['old']==2 and t['new']==1 for t in transitions),
        idle_activations=sum(t['old']==0 and t['new']==1 for t in transitions),examples=examples))
(folder/'prototype_replay.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
for r in results:
    print(r['margin'],'recover',r['recover_reloads'],'slowback',r['slowback_reloads'],'idle',r['idle_activations'],'errors',r['legacy_mismatches'] if r['margin']==0 else '')
    print('examples',r['examples'][:4])
