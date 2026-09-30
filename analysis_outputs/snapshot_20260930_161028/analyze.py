import csv
import hashlib
import json
import shutil
from collections import Counter
from datetime import datetime
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
REC = ROOT / 'PY_SRIPT/algo_monitor/recordings'
for name in ('recent_5min.csv', 'recent_5min.meta.json'):
    if not (OUT/name).exists():
        shutil.copy2(REC/name, OUT/name)
meta = json.loads((OUT/'recent_5min.meta.json').read_text(encoding='utf-8'))
digest = hashlib.sha256((OUT/'recent_5min.csv').read_bytes()).hexdigest()
assert digest == meta['csv_sha256'], (digest, meta['csv_sha256'])
with (OUT/'recent_5min.csv').open(encoding='utf-8-sig', newline='') as f:
    rows = list(csv.DictReader(f))
samples = [r for r in rows if r['type'] in ('debug', 'signal')]
t = np.array([datetime.fromisoformat(r['timestamp']).timestamp() for r in samples])
data = {key: np.array([[float(r[f'{key}_{ch}']) for ch in range(20)] for r in samples])
        for key in ('raw','baseline','signal','state','fast_k_percent','slow_k_percent')}
def stamp(index): return samples[int(index)]['timestamp'][11:23]
def segments(values):
    cuts = np.r_[0, np.flatnonzero((values[1:] != values[:-1]) | (np.diff(t) > .5))+1, len(t)]
    return [(int(a),int(b-1)) for a,b in zip(cuts[:-1],cuts[1:])]
def describe(a,b,ch):
    return dict(start=stamp(a), end=stamp(b), seconds=round(float(t[b]-t[a]),3),
        samples=b-a+1, **{k: [float(v[a,ch]),float(v[b,ch]),float(v[a:b+1,ch].min()),float(v[a:b+1,ch].max())]
                         for k,v in data.items()})
summary = dict(meta=meta, sha256_verified=True, row_types=dict(Counter(r['type'] for r in rows)),
    gaps=[dict(start=stamp(i),end=stamp(i+1),seconds=round(float(t[i+1]-t[i]),3)) for i in np.flatnonzero(np.diff(t)>.1)],
    median_interval=float(np.median(np.diff(t))), channels=[])
for ch in range(20):
    sig=data['signal'][:,ch]; state=data['state'][:,ch]
    active=[describe(a,b,ch) for a,b in segments(sig>0) if sig[a]>0]
    floating=[describe(a,b,ch) for a,b in segments(state) if state[a]==6]
    summary['channels'].append(dict(channel=ch, max_signal=float(sig.max()), final_signal=float(sig[-1]),
        final_raw=float(data['raw'][-1,ch]),final_baseline=float(data['baseline'][-1,ch]),final_state=int(state[-1]),
        nonzero_samples=int((sig>0).sum()), state_counts=dict(Counter(map(int,state))),
        longest_nonzero=sorted(active,key=lambda x:x['seconds'],reverse=True)[:3],
        longest_floating=sorted(floating,key=lambda x:x['seconds'],reverse=True)[:3]))
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print('INTEGRITY',json.dumps({k:summary[k] for k in ('sha256_verified','row_types','gaps','median_interval')}))
for c in summary['channels']:
    print('CH',c['channel'],'peak',c['max_signal'],'last',c['final_signal'],'raw/base',c['final_raw'],c['final_baseline'],
          'state',c['final_state'],'nonzero',c['nonzero_samples'],'state counts',c['state_counts'])
    for run in c['longest_floating'][:2]: print(' FLOAT',json.dumps(run))

roll=REC/'rolling'
files=sorted(roll.glob('roll_20260930_151101_929680_5bc70e43_*.jsonl'))
kinds=Counter(); types=Counter(); ports=Counter(); events=[]; commands=[]; seq_gaps=[]; frame_gaps=[]
previous=None; prev_frame=None; malformed=0; match_frames=0
for path in files:
    # 此快照从47号分段中开始；只读覆盖它的前后一段。
    number=int(path.stem.rsplit('_',1)[1])
    if not 46<=number<=51: continue
    with path.open(encoding='utf-8') as f:
        for line in f:
            try: r=json.loads(line)
            except json.JSONDecodeError: malformed+=1; continue
            ts=r['time_ns']/1e9
            if ts<t[0]-10 or ts>t[-1]+1: continue
            if previous and r['sequence']!=previous['sequence']+1:
                seq_gaps.append([previous['timestamp'],r['timestamp'],previous['sequence'],r['sequence']])
            previous=r
            if not t[0]<=ts<t[-1]+.001: continue
            kinds[r['kind']]+=1; ports[r.get('port')]+=1
            if r['kind']=='frame':
                ft=r['frame']['type']; types[ft]+=1
                if ft in ('debug','signal'):
                    match_frames+=1
                    if prev_frame and ts-prev_frame['time_ns']/1e9>.1:
                        frame_gaps.append([prev_frame['timestamp'],r['timestamp'],ts-prev_frame['time_ns']/1e9])
                    prev_frame=r
                elif ft!='temperature': events.append(r)
            elif r['kind']=='tx': commands.append(r)
            elif r['kind'] not in ('rx','centroid'): events.append(r)
log=dict(kinds=dict(kinds),frame_types=dict(types),ports=dict(ports),sequence_gaps=seq_gaps,
         sensor_gaps=frame_gaps,events=events,commands=commands,malformed=malformed,sensor_frames=match_frames)
(OUT/'rolling_audit.json').write_text(json.dumps(log,ensure_ascii=False,indent=2),encoding='utf-8')
print('ROLLING',json.dumps(log,ensure_ascii=False))
