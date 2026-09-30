import csv, json
from pathlib import Path
from datetime import datetime
import numpy as np
P=Path(__file__).resolve().parent
rows=list(csv.DictReader((P/'recent_5min.csv').open(encoding='utf-8-sig')))
r=[x for x in rows if x['type']=='debug']
t=np.array([datetime.fromisoformat(x['timestamp']).timestamp() for x in r])
d={k:np.array([[float(x[f'{k}_{i}']) for i in range(20)] for x in r]) for k in ('raw','baseline','signal','state','fast_k_percent','slow_k_percent')}
def at(clock): return int(np.argmin(abs(t-datetime.fromisoformat('2026-09-30T'+clock+'+08:00').timestamp())))
def pt(j,c): return [r[j]['timestamp'][11:23]]+[float(d[k][j,c]) for k in d]
print('ORDER: time raw baseline signal state fastK slowK')
for c in (0,3,4,7,8,11,12,15,16,19):
    print('CH',c,'MAX',pt(int(np.argmax(d['signal'][:,c])),c))
    print('LAST', [pt(at('16:10:'+s),c) for s in ('00','05','10','15','20','25','28.320')])
    a=at('16:09:40'); z=d['state'][a:,c]
    changes=np.r_[a,np.flatnonzero(z[1:]!=z[:-1])+a+1,len(t)-1]
    print('TRANSITIONS_LAST', [pt(int(j),c) for j in changes[-8:]])
    active=np.isin(d['state'][:,c],[1,2,3,6]); starts=np.r_[0,np.flatnonzero(active[1:]!=active[:-1])+1]; ends=np.r_[starts[1:]-1,len(t)-1]
    runs=[]
    for a,b in zip(starts,ends):
        if active[a] and t[b]>=t[-1]-60:
            delta=np.maximum(0,d['raw'][a:b+1,c]-d['baseline'][a:b+1,c])
            runs.append(dict(start=pt(a,c),end=pt(b,c),seconds=round(t[b]-t[a],3),peak_raw_delta=float(delta.max()),final_ratio=float(delta[-1]/delta.max()) if delta.max() else 0))
    print('RECENT_ACTIVE',json.dumps(sorted(runs,key=lambda x:x['seconds'],reverse=True)[:2]+runs[-1:]))
for clock in ('16:07:59.350','16:07:59.364','16:08:07.256','16:08:07.300','16:08:08','16:08:10'):
    j=at(clock); print('GAP_FRAME',r[j]['timestamp'],{k:d[k][j].tolist() for k in d})
temps=[float(x['temperature_celsius']) for x in rows if x['type']=='temperature']
print('TEMPERATURE',min(temps),max(temps),temps[0],temps[-1])
