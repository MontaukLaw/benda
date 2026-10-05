import collections
from datetime import datetime
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser

CH=int(sys.argv[1]) if len(sys.argv)>1 else 6
directory=ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'
latest=max(directory.glob('roll_*.jsonl'),key=lambda p:p.stat().st_mtime)
prefix=latest.name.rsplit('_',1)[0]
files=[(p,p.stat().st_size) for p in sorted(directory.glob(prefix+'_*.jsonl'))]
out=ROOT/f'analysis_outputs/ch{CH}_host_rebound_20261005'
out.mkdir(exist_ok=True)
parser=AlgoFrameParser()
decoded=collections.deque()
kinds,types,ports=collections.Counter(),collections.Counter(),collections.Counter()
gaps,mismatches,events,configs,diagnostics=[],[],[],[],[]
firmware_states=collections.Counter()
raw_only=0
firmware_signal_nonzero=firmware_baseline_nonzero=0
firmware_finite_fast_k=firmware_finite_slow_k=0
raws=[]
rawtimes=[]
offsets=collections.Counter()
rows=[]
states=[collections.Counter() for _ in range(20)]
first=last=lastseq=None
for path,size in files:
    with path.open('rb') as stream:
        while stream.tell()<size:
            line=stream.readline(size-stream.tell())
            if not line.endswith(b'\n'):break
            r=json.loads(line)
            first=first or r['timestamp'];last=r['timestamp']
            k=r['kind'];kinds[k]+=1;ports[r.get('port')]+=1
            if lastseq is not None and r['sequence']!=lastseq+1:gaps.append([lastseq,r['sequence'],r['timestamp']])
            lastseq=r['sequence']
            if k=='rx':decoded.extend(parser.feed(bytes.fromhex(r['hex'])))
            elif k=='frame':
                f=r['frame'];types[f['type']]+=1
                if not decoded or json.dumps(decoded.popleft(),sort_keys=True)!=json.dumps(f,sort_keys=True):mismatches.append(r['sequence'])
                if f['type'] in ('debug','signal'):
                    raws.append(tuple(f.get('raw',[])))
                    raw_only+=bool(f.get('raw_only'))
                    firmware_states.update(f.get('state',[]))
                    firmware_signal_nonzero+=any(f.get('signal',[]))
                    firmware_baseline_nonzero+=any(f.get('baseline',[]))
                    firmware_finite_fast_k+=any(math.isfinite(x) for x in f.get('fast_k_percent',[]))
                    firmware_finite_slow_k+=any(math.isfinite(x) for x in f.get('slow_k_percent',[]))
                    rawtimes.append(r['monotonic_ns']/1e9)
                elif f['type']!='temperature':diagnostics.append(r)
            elif k=='centroid':
                f=r['frame'];raw=f.get('raw',[])
                matches=[i for i in range(max(0,len(raws)-5),len(raws)) if list(raws[i])==raw]
                if len(matches)==1:offsets[f['source_sample_sequence']-matches[0]]+=1
            elif k=='host_state_config':configs.append(r)
            elif k=='host_state_machine':
                f=r['frame']
                for ch,state in enumerate(f['state']):states[ch][state]+=1
                rows.append(dict(timestamp=r['timestamp'],sequence=f['sequence'],time=f['sample_timestamp'],
                    generation=f['generation'],state=f['state'][CH],input=f['input_val'][CH],baseline=f['baseline'][CH],
                    signal=f['signal'][CH],rate=f['decision_rate'][CH],reference=f.get('release_reference',[None]*20)[CH],activation=[e for e in f.get('activation_events',[]) if e['channel']==CH]))
            if k=='host_state_machine' and rows[-1]['activation']:
                rows[-1]['all_signal']=r['frame']['signal']
                rows[-1]['all_state']=r['frame']['state']
                rows[-1]['all_input']=r['frame']['input_val']
                rows[-1]['all_baseline']=r['frame']['baseline']
            if k in ('connected','disconnected','tx','error','log_gap'):events.append(r)

offset=offsets.most_common(1)[0][0] if offsets else None
raw_matches=0
for row in rows:
    i=row['sequence']-offset if offset is not None else -1
    row['raw']=raws[i][CH] if 0<=i<len(raws) and len(raws[i])==20 else None
    raw_matches+=row['raw'] is not None
cycles=[]
transitions=[]
old=None
cycle=None
for row in rows:
    if old is None or row['state']!=old['state'] or row['generation']!=old['generation']:
        transitions.append(dict(previous=old,current=row))
    if old is not None and row['generation']!=old['generation']:cycle=None
    if old is not None and old['state']==1 and row['state']==2:
        cycle=dict(release=row,pre_release=old,minimum=row['input'],minimum_row=row,recover=None,rows=0)
        cycles.append(cycle)
    if cycle:
        cycle['rows']+=1
        if row['input']<cycle['minimum']:
            cycle['minimum']=row['input'];cycle['minimum_row']=row
        if row['state']==3 and cycle['recover'] is None:cycle['recover']=row
        if old is not None and old['state'] in (2,3) and row['state'] in (0,1):
            cycle.update(end=row,end_reason='idle' if row['state']==0 else 'reload',
                reference=cycle['pre_release']['baseline'],
                above_reference=row['input']-cycle['pre_release']['baseline'],
                baseline_shift=row['baseline']-cycle['pre_release']['baseline'])
            cycle=None
    old=row

recorded=json.loads((ROOT/'PY_SRIPT/algo_monitor/sensor_display_config.json').read_text(encoding='utf-8'))
parameter_source='logged' if configs else 'current saved file; session configuration not retained'
params=configs[-1]['frame']['parameters'] if configs else recorded['host_state_machine']
gains=configs[-1]['frame']['raw_gains'] if configs else recorded['array_bars']['raw_gains']
ema_errors=[]
for a,b in zip(rows,rows[1:]):
    if a['generation']==b['generation'] and b['sequence']==a['sequence']+1 and b['raw'] is not None:
        error=abs(b['input']-((1-params['input_alpha'])*a['input']+params['input_alpha']*b['raw']*gains[CH]))
        if error>1e-7:ema_errors.append([b['sequence'],error])
reloads=[c for c in cycles if c.get('end_reason')=='reload']
report=dict(session=prefix,first=first,last=last,files=[dict(name=p.name,bytes=size) for p,size in files],
    retained_from_segment=int(files[0][0].stem.rsplit('_',1)[1]),kinds=dict(kinds),types=dict(types),ports=dict(ports),
    sequence_gaps=gaps,rx_parser_mismatches=mismatches,pending_decoded=len(decoded),events=events,configs=configs,
    diagnostics=diagnostics,sensor_count=len(raws),host_count=len(rows),offset_candidates=offsets.most_common(5),
    matched_raw=raw_matches,ema_errors=ema_errors[:20],ema_error_count=len(ema_errors),
    max_sensor_gap=max((b-a for a,b in zip(rawtimes,rawtimes[1:])),default=0),
    sensor_gaps=[dict(index=i,seconds=b-a) for i,(a,b) in enumerate(zip(rawtimes,rawtimes[1:])) if b-a>.15],
    state_counts=[dict(c) for c in states],parameters=params,gains=gains,parameter_source=parameter_source,
    raw_only_samples=raw_only,firmware_states=dict(firmware_states),firmware_signal_nonzero=firmware_signal_nonzero,firmware_baseline_nonzero=firmware_baseline_nonzero,firmware_finite_fast_k=firmware_finite_fast_k,firmware_finite_slow_k=firmware_finite_slow_k,
    generations=dict(collections.Counter(r['generation'] for r in rows)),cycles=cycles,
    reload_count=len(reloads),recover_reload_count=sum(c['end']['activation'][0]['from_state']==3 for c in reloads),
    reload_above_reference_counts={str(t):sum(c['above_reference']>t for c in reloads) for t in (0,100,200,300,500,800)},
    reload_examples=sorted(reloads,key=lambda c:c['above_reference'])[:10])
(out/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
(out/f'ch{CH}_rows.json').write_text(json.dumps(rows,ensure_ascii=False),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('cycles','reload_examples','diagnostics','events','configs','ema_errors')},ensure_ascii=False,indent=2))
for c in report['reload_examples'][:5]:print('EXAMPLE',json.dumps(c,ensure_ascii=False))
