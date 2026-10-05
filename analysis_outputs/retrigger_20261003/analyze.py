"""固定最新rolling会话快照，核对完整性并提取通道再次激活上下文。"""
from pathlib import Path
import collections
import gzip
import json

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
folder=ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'
latest=max(folder.glob('roll_*.jsonl'),key=lambda p:p.stat().st_mtime)
prefix=latest.stem.rsplit('_',1)[0]
paths=[(p,p.stat().st_size) for p in sorted(folder.glob(prefix+'_*.jsonl'))]
samples=[]
events=[]
kinds=collections.Counter()
types=collections.Counter()
ports=collections.Counter()
gaps=[]
bad=[]
partial=[]
first=last=None
prev_seq=None
prev_time=None
backward_times=[]
for p,size in paths:
    with p.open('rb') as f:
        lines=f.read(size).splitlines(keepends=True)
    for i,line in enumerate(lines):
        if not line.endswith(b'\n'):
            partial.append([p.name,i+1])
            continue
        try: r=json.loads(line)
        except Exception as e:
            bad.append([p.name,i+1,str(e)])
            continue
        first=first or r['timestamp']
        last=r['timestamp']
        if prev_seq is not None and r['sequence'] != prev_seq+1:
            gaps.append([prev_seq,r['sequence'],r['timestamp']])
        if prev_time is not None and r['time_ns']<prev_time:
            backward_times.append([r['timestamp'],r['time_ns']-prev_time])
        prev_seq=r['sequence']
        prev_time=r['time_ns']
        kinds[r['kind']]+=1
        ports[r.get('port')]+=1
        if r['kind']=='frame':
            f=r['frame']
            types[f['type']]+=1
            if f['type'] in ('debug','signal'):
                samples.append(r)
            elif f['type']!='temperature': events.append(r)
        elif r['kind'] not in ('rx','centroid'): events.append(r)
cut=samples[-1]['time_ns']-8*60*1e9
rows=[r for r in samples if r['time_ns']>=cut]
dts=[(b['time_ns']-a['time_ns'])/1e9 for a,b in zip(samples,samples[1:])]
channels=[]
for ch in range(20):
    transitions=[]
    for i in range(1,len(rows)):
        a,b=rows[i-1],rows[i]
        sa,sb=a['frame']['state'][ch],b['frame']['state'][ch]
        if sa not in (1,2,3,6) and sb in (1,2,3,6):
            before=[r for r in rows[max(0,i-500):i] if r['time_ns']>=b['time_ns']-2e9]
            after=[r for r in rows[i:i+1000] if r['time_ns']<=b['time_ns']+5e9]
            def values(r):
                return dict(time=r['timestamp'],**{k:r['frame'][k][ch] for k in ('raw','baseline','signal','state','fast_k_percent','slow_k_percent')})
            active_end=next((r for r in after[1:] if r['frame']['state'][ch] not in (1,2,3,6)),None)
            transitions.append(dict(index=i,time=b['timestamp'],from_state=sa,to_state=sb,
                previous=values(a),entry=values(b),
                raw_before_range=[min(r['frame']['raw'][ch] for r in before),max(r['frame']['raw'][ch] for r in before)],
                peak_raw_5s=max(r['frame']['raw'][ch] for r in after),
                next_inactive=values(active_end) if active_end else None))
    channels.append(dict(channel=ch,reactivation_count=len(transitions),transitions=transitions,
        final={k:rows[-1]['frame'][k][ch] for k in ('raw','baseline','signal','state','fast_k_percent','slow_k_percent')}))
report=dict(session=prefix,first=first,last=last,files=[dict(name=p.name,size=size) for p,size in paths],
    kinds=kinds,types=types,ports=ports,sequence_gaps=gaps,backward_times=backward_times,
    bad_lines=bad,partial_tail=partial,sample_count=len(samples),sample_gaps=[dict(time=samples[i+1]['timestamp'],seconds=dt) for i,dt in enumerate(dts) if dt>.1],
    fps=(len(samples)-1)*1e9/(samples[-1]['time_ns']-samples[0]['time_ns']),
    recent_start=rows[0]['timestamp'],recent_end=rows[-1]['timestamp'],recent_count=len(rows),
    recent_fps=(len(rows)-1)*1e9/(rows[-1]['time_ns']-rows[0]['time_ns']),events=events,channels=channels)
OUT.mkdir(parents=True,exist_ok=True)
(OUT/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
with gzip.open(OUT/'samples.json.gz','wt',encoding='utf8') as f: json.dump(rows,f)
print(json.dumps({k:v for k,v in report.items() if k not in ('events','channels','files')},ensure_ascii=False))
print('events',json.dumps([e for e in events if e.get('frame',{}).get('type')!='capsense_channel_config_status'],ensure_ascii=False))
print('reactivations',[(c['channel'],c['reactivation_count']) for c in channels])
for ch in (11,16):
    print('CH',ch,'final',channels[ch]['final'])
    for e in channels[ch]['transitions'][-25:]: print(json.dumps(e,ensure_ascii=False))
