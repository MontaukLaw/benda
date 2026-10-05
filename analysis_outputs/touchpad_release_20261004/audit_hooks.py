import collections
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser

directory = ROOT / 'PY_SRIPT/algo_monitor/recordings/rolling'
latest = max(directory.glob('roll_*.jsonl'), key=lambda p: p.stat().st_mtime)
prefix = latest.name.rsplit('_', 1)[0]
files = [(p, p.stat().st_size) for p in sorted(directory.glob(prefix + '_*.jsonl'))]
if '--captured' in sys.argv:
    previous = json.loads(Path(__file__).with_name('hooks_audit.json').read_text(encoding='utf-8'))
    prefix = previous['session']
    files = [(directory / f['name'], f['bytes']) for f in previous['files']]
parser = AlgoFrameParser()
decoded = collections.deque()
kinds, types, ports = collections.Counter(), collections.Counter(), collections.Counter()
gaps, mismatches, events, configs, diagnostics = [], [], [], [], []
touch, hosts, sensors = [], {}, {}
lastseq = None
first = last = None
sensor_times = []
for path, size in files:
    with path.open('rb') as stream:
        while stream.tell() < size:
            line = stream.readline(size-stream.tell())
            if not line.endswith(b'\n'):
                break
            r = json.loads(line)
            first = first or r['timestamp']
            last = r['timestamp']
            k = r['kind']
            kinds[k] += 1
            ports[r.get('port')] += 1
            if lastseq is not None and r['sequence'] != lastseq + 1:
                gaps.append([lastseq, r['sequence'], r['timestamp']])
            lastseq = r['sequence']
            if k == 'rx':
                decoded.extend(parser.feed(bytes.fromhex(r['hex'])))
            elif k == 'frame':
                f = r['frame']
                types[f['type']] += 1
                if not decoded or json.dumps(decoded.popleft(), sort_keys=True) != json.dumps(f, sort_keys=True):
                    mismatches.append(r['sequence'])
                if f['type'] in ('debug', 'signal'):
                    sensor_times.append(r['monotonic_ns']/1e9)
                    sensors[len(sensor_times)] = f
                else:
                    diagnostics.append(r)
            elif k == 'host_state_machine':
                hosts[r['frame']['sequence']] = r['frame']
            elif k == 'touchpad':
                touch.append(dict(timestamp=r['timestamp'], log_sequence=r['sequence'], **r['frame']))
            elif k == 'host_state_config':
                configs.append(r)
            if k in ('connected','disconnected','log_gap','error','tx'):
                events.append(r)

def details(a, b):
    s = b['source_sample_sequence']
    ha, hb = hosts.get(a['source_sample_sequence'], {}), hosts.get(s,{})
    wa, wb = a.get('host_signal',[]), b.get('host_signal',[])
    changed = []
    for ch in range(len(wa)):
        if wa[ch] != wb[ch] or a['host_state'][ch] != b['host_state'][ch]:
            changed.append(dict(ch=ch, row=ch%4, before=wa[ch], after=wb[ch],
                old_state=a['host_state'][ch],new_state=b['host_state'][ch],
                rate=hb.get('decision_rate',[None]*20)[ch],
                input=hb.get('input_val',[None]*20)[ch],
                baseline=hb.get('baseline',[None]*20)[ch],
                raw=sensors.get(s,{}).get('raw',[None]*20)[ch]))
    changed.sort(key=lambda v: abs(v['after']-v['before']),reverse=True)
    return dict(timestamp=b['timestamp'],sample=s,dt=b['sample_timestamp']-a['sample_timestamp'],
                dy=b['dy'],old_xy=a['raw_xy'],new_xy=b['raw_xy'],
                old_filtered=a['filtered_xy'],new_filtered=b['filtered_xy'],
                old_active=a.get('host_active_channels'),new_active=b.get('host_active_channels'),
                pairs=b.get('host_adjacent_pairs'),old_total=sum(wa),new_total=sum(wb),
                old_rows=[sum(wa[ch] for ch in range(len(wa)) if ch%4==row) for row in range(4)],
                new_rows=[sum(wb[ch] for ch in range(len(wb)) if ch%4==row) for row in range(4)],
                changes=changed[:6],settings=b['settings'],block=b.get('motion_block_reason'))

steps=[]
for i,(a,b) in enumerate(zip(touch,touch[1:]),1):
    if a['contact'] and b['contact'] and a['raw_xy'] and b['raw_xy'] and b['dy'] != 0:
        d=details(a,b)
        d['raw_dy']=(b['raw_xy'][1]-a['raw_xy'][1])*b['settings']['sensitivity']
        d['index']=i
        steps.append(d)

# 连续向上输出视为一段，保留每段完整的前后通道变化证据。
runs=[]
for i,t in enumerate(touch):
    if t['contact'] and t['dy'] < -.05:
        if not runs or runs[-1]['end_index'] != i-1:
            runs.append(dict(start_index=i,end_index=i,dy=0.0,count=0,peak=0.0))
        run=runs[-1]
        run['end_index']=i
        run['dy']+=t['dy']
        run['count']+=1
        run['peak']=min(run['peak'],t['dy'])
for run in runs:
    i,j=run['start_index'],run['end_index']
    a,b=touch[max(0,i-1)],touch[j]
    run.update(start=touch[i]['timestamp'],end=b['timestamp'],duration=b['sample_timestamp']-touch[i]['sample_timestamp'],
        before_down=sum(max(0,x['dy']) for x in touch[:i] if x['sample_timestamp']>=touch[i]['sample_timestamp']-.5),
        before_net=sum(x['dy'] for x in touch[:i] if x['sample_timestamp']>=touch[i]['sample_timestamp']-.5),
        old_active=a.get('host_active_channels'),new_active=b.get('host_active_channels'),
        old_xy=a['raw_xy'],new_xy=b['raw_xy'],old_total=sum(a.get('host_signal',[])),new_total=sum(b.get('host_signal',[])),
        first_step=details(a,touch[i]),last_step=details(touch[max(i,j-1)],b))
    related=[s for s in steps if i<=s['index']<=j]
    run['sharpest']=min(related,key=lambda d:d['raw_dy']) if related else None
    run['state_cuts']=[s for s in related if any(c['old_state']==1 and c['new_state']!=1 for c in s['changes'])]

summary=dict(session=prefix,first=first,last=last,files=[dict(name=p.name,bytes=s) for p,s in files],
    kinds=dict(kinds),types=dict(types),ports=dict(ports),sequence_gaps=gaps,parser_mismatches=mismatches,
    pending_decoded=len(decoded),sensor_count=len(sensor_times),
    max_sensor_gap=max((b-a for a,b in zip(sensor_times,sensor_times[1:])),default=0),
    sensor_gaps=[dict(index=i+2,seconds=b-a) for i,(a,b) in enumerate(zip(sensor_times,sensor_times[1:])) if b-a>.15],
    events=events,configs=configs,diagnostics=diagnostics,
    touch_count=len(touch),touch_sources=dict(collections.Counter(t['source'] for t in touch)),
    touch_settings=list({json.dumps(t['settings'],sort_keys=True) for t in touch}),
    block_reasons=dict(collections.Counter(t.get('motion_block_reason') for t in touch)),
    host_generations=dict(collections.Counter(h['generation'] for h in hosts.values())),
    touch_host_mismatches=sum(t.get('host_signal')!=hosts.get(t['source_sample_sequence'],{}).get('signal') or t.get('host_state')!=hosts.get(t['source_sample_sequence'],{}).get('state') for t in touch),
    sharp_up=sorted(steps,key=lambda d:d['raw_dy'])[:25],
    filtered_up=sorted(steps,key=lambda d:d['dy'])[:15],
    upward_runs=sorted([r for r in runs if r['dy']<-10],key=lambda r:r['dy']),
    reversals=sorted([r for r in runs if r['dy']<-10 and r['before_net']>10],key=lambda r:r['dy']))
out=Path(__file__).with_name('hooks_audit.json')
out.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
Path(__file__).with_name('hooks_touch.json').write_text(json.dumps(touch,ensure_ascii=False),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('sharp_up','filtered_up','upward_runs','reversals','diagnostics','events','configs')},ensure_ascii=False,indent=2))
print('CONFIGS',json.dumps(configs,ensure_ascii=False))
print('TOP REVERSALS',json.dumps(summary['reversals'][:8],ensure_ascii=False))
print('SHARP UP',json.dumps(summary['sharp_up'][:6],ensure_ascii=False))

if '--replay' in sys.argv:
    from host_state_machine import HostStateMachine, HostParameters
    from touchpad import TouchpadTracker
    cfg=configs[0]['frame']
    model=HostStateMachine(HostParameters.from_settings(cfg['parameters']))
    model.set_gains(cfg['raw_gains'])
    errors=collections.Counter()
    max_errors=collections.defaultdict(float)
    initial=None
    for seq,h in hosts.items():
        if seq==min(hosts):
            model.last_sequence=seq-1
        result=model.feed(dict(sequence=seq,sample_timestamp=h['sample_timestamp'],has_raw=True,
                               channel_count=20,raw=sensors[seq]['raw']))
        for field in ('input_val','baseline','signal','pressure_rate','decision_rate','state'):
            for x,y in zip(h[field],result[field]):
                if math.isnan(x) and math.isnan(y):continue
                error=abs(x-y)
                max_errors[field]=max(max_errors[field],error)
                if error>1e-7 or math.isnan(error):errors[field]+=1
        if initial is None and all(s==0 for s in h['state']):initial=h
    settings=touch[0]['settings']
    tracker=TouchpadTracker(**{k:v for k,v in settings.items() if k!='source'})
    tracker.set_source('host_signal')
    tracker.set_enabled(True)
    terrors=collections.Counter()
    tmax=collections.defaultdict(float)
    guard_events=[]
    targets={x['sample'] for x in summary['sharp_up'][:12]}
    for t in touch:
        seq=t['source_sample_sequence']
        h=hosts[seq]
        timestamp=t['sample_timestamp']
        prior=next(((tm,w) for tm,w in tracker._release_samples if timestamp-tm<=.08),None)
        metric=None
        if prior:
            tm,w=prior
            new=t['host_signal']
            losses=sum(max(0,a-b) for a,b in zip(w,new))
            gains=sum(max(0,b-a) for a,b in zip(w,new))
            metric=dict(timestamp=t['timestamp'],window_seconds=timestamp-tm,
                losses=losses,gains=gains,drop_ratio=losses/sum(w) if sum(w) else 0,
                rising=[dict(ch=ch,increase=b-a) for ch,(a,b) in enumerate(zip(w,new)) if b-a>12],
                prior_signal=list(w),current_signal=new)
        sample=dict(sequence=seq,sample_timestamp=timestamp,has_raw=True,raw=sensors[seq]['raw'],channel_count=20,
                    host_signal=t['host_signal'],host_state=t['host_state'],host_generation=t['host_generation'],host_ready=True)
        tracker.feed(sample)
        for key,x,y in [('dx',t['dx'],tracker.delta[0]),('dy',t['dy'],tracker.delta[1])]:
            tmax[key]=max(tmax[key],abs(x-y))
            if abs(x-y)>1e-7:terrors[key]+=1
        for key,x,y in [('contact',t['contact'],tracker.contact),('block',t.get('motion_block_reason'),tracker.motion_block_reason)]:
            if x!=y:terrors[key]+=1
        if seq in targets:
            metric.update(blocked=tracker.release_blocked,raw=sensors[seq]['raw'],host=h,initial_baseline=initial['baseline'])
            guard_events.append(metric)
    extra=dict(host_replay_errors=dict(errors),host_replay_max_errors=dict(max_errors),
        touch_replay_errors=dict(terrors),touch_replay_max_errors=dict(tmax),initial=initial,
        guards=guard_events,raw_only_samples=sum(bool(f.get('raw_only')) for f in sensors.values()),
        real_state_values=sorted({s for f in sensors.values() for s in f['state']}),
        real_signal_nonzero=sum(any(f['signal']) for f in sensors.values()),
        real_baseline_nonzero=sum(any(f['baseline']) for f in sensors.values()))
    Path(__file__).with_name('hooks_replay.json').write_text(json.dumps(extra,ensure_ascii=False,indent=2),encoding='utf-8')
    print('REPLAY',json.dumps({k:v for k,v in extra.items() if k not in ('initial','guards')},ensure_ascii=False))
    for g in guard_events[:5]:
        print('GUARD',json.dumps({k:v for k,v in g.items() if k not in ('host','raw','prior_signal','current_signal','initial_baseline')},ensure_ascii=False))
