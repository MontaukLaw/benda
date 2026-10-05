import collections
import gzip
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
paths = sorted((ROOT / 'PY_SRIPT/algo_monitor/recordings/rolling').glob('roll_20261003_135329_379513_72789ad0_*.jsonl'))
kinds, types, ports = collections.Counter(), collections.Counter(), collections.Counter()
events, gaps, samples = [], [], []
last_seq = None
first = last = None
bad = 0
for path in paths:
    for line in path.open(encoding='utf-8'):
        try: r = json.loads(line)
        except json.JSONDecodeError:
            bad += 1
            continue
        first = first or r['timestamp']
        last = r['timestamp']
        seq = r['sequence']
        if last_seq is not None and seq != last_seq + 1:
            gaps.append([last_seq, seq, r['timestamp']])
        last_seq = seq
        kinds[r['kind']] += 1
        ports[r.get('port')] += 1
        if r['kind'] == 'frame':
            f = r['frame']
            types[f['type']] += 1
            if f['type'] in ('debug', 'signal'):
                samples.append(r)
            elif f['type'] != 'temperature':
                events.append(r)
        elif r['kind'] not in ('rx', 'tx', 'centroid'):
            events.append(r)
        elif r['kind'] == 'tx':
            events.append(r)
ts = [r['time_ns'] / 1e9 for r in samples]
dts = sorted(b-a for a,b in zip(ts,ts[1:]))
report = dict(files=[p.name for p in paths],first=first,last=last,kinds=kinds,types=types,ports=ports,
              sequence_gaps=gaps,bad_lines=bad,events=events,samples=len(samples),
              fps=(len(samples)-1)/(ts[-1]-ts[0]),median_dt=dts[len(dts)//2],
              sample_gaps=[(samples[i]['timestamp'],dt) for i,dt in enumerate(b-a for a,b in zip(ts,ts[1:])) if dt>0.1])
channels = []
for ch in range(20):
    runs = []
    start = 0
    for i in range(1,len(samples)+1):
        if i == len(samples) or samples[i]['frame']['state'][ch] != samples[start]['frame']['state'][ch]:
            a,b=samples[start],samples[i-1]
            dur=(b['time_ns']-a['time_ns'])/1e9
            if dur>2 and a['frame']['state'][ch] in (3,4,5,6,7,8):
                runs.append(dict(state=a['frame']['state'][ch],duration=round(dur,3),start=a['timestamp'],end=b['timestamp'],
                    raw_start=a['frame']['raw'][ch],raw_end=b['frame']['raw'][ch],baseline_start=a['frame']['baseline'][ch],baseline_end=b['frame']['baseline'][ch],
                    signal_start=a['frame']['signal'][ch],signal_end=b['frame']['signal'][ch],
                    fast_start=a['frame']['fast_k_percent'][ch],slow_start=a['frame']['slow_k_percent'][ch],
                    fast_end=b['frame']['fast_k_percent'][ch],slow_end=b['frame']['slow_k_percent'][ch]))
            start=i
    tail=[r['frame'] for r in samples if r['time_ns']>=samples[-1]['time_ns']-60e9]
    states=collections.Counter(r['state'][ch] for r in tail)
    channels.append(dict(channel=ch,tail_states=states,tail_raw_range=[min(r['raw'][ch] for r in tail),max(r['raw'][ch] for r in tail)],
        final={k:samples[-1]['frame'][k][ch] for k in ('raw','baseline','signal','state','fast_k_percent','slow_k_percent')},runs=runs))
report['channels']=channels
OUT.mkdir(exist_ok=True,parents=True)
(OUT/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
with gzip.open(OUT/'samples.json.gz','wt',encoding='utf-8') as f: json.dump(samples,f)
print(json.dumps({k:v for k,v in report.items() if k not in ('events','channels')},ensure_ascii=False))
for ch in channels:
    print(ch['channel'], ch['final'], 'long floating', [r for r in ch['runs'] if r['state']==6 and r['duration']>30],
          'max slowback', max([r['duration'] for r in ch['runs'] if r['state']==4] or [0]))
print('EVENTS',json.dumps([e for e in events if e.get('frame',{}).get('type') != 'capsense_channel_config_status'],ensure_ascii=False))
