"""固定读取边界，审核最新会话最后五分钟的全部日志与拇指轨迹。"""
import json, math, sys
from pathlib import Path
from collections import Counter, defaultdict
from datetime import datetime, timezone, timedelta
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).parent
directory=ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'
latest=max(directory.glob('roll_*.jsonl'),key=lambda p:p.stat().st_mtime_ns)
session=latest.name.rsplit('_',1)[0]
files=[(p,p.stat().st_size) for p in sorted(directory.glob(session+'_*.jsonl'))]
with files[-1][0].open('rb') as f:
    f.seek(max(0,files[-1][1]-100000))
    tail=f.read(files[-1][1]-f.tell()).splitlines()
valid=[]
for line in tail:
    try:valid.append(json.loads(line))
    except ValueError:pass
end=max(r['time_ns'] for r in valid); start=end-300_000_000_000
stamp=lambda t:datetime.fromtimestamp(t/1e9,timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')
stats=dict(session=session,start_ns=start,end_ns=end,start=stamp(start),end=stamp(end),files=[dict(name=p.name,read_bytes=n) for p,n in files])
kinds=Counter();types=Counter();ports=Counter();events=[];gaps=[];seq_gaps=[];all_gaps=[]
prev_seq=None;sample_t=[];sample_seq=[];touch=[];frames=[];diagnostics={};tx=[];rx_bytes=0;bad=[];minutes=defaultdict(Counter)
channel_values=defaultdict(list);channel_states=[];configurations=Counter();source_gaps=[];last_tp=None
for path,size in files:
    with path.open('rb') as source:
        used=0
        while used<size:
            line=source.readline(size-used);used+=len(line)
            if not line:break
            try:r=json.loads(line)
            except ValueError:bad.append(dict(file=path.name,offset=used));continue
            t=r['time_ns'];kind=r['kind'];seq=r.get('sequence')
            if seq is not None:
                if prev_seq is not None and seq!=prev_seq+1:
                    item=dict(time=stamp(t),previous=prev_seq,current=seq)
                    all_gaps.append(item)
                    if t>=start:seq_gaps.append(item)
                prev_seq=seq
            if kind not in ('rx','frame','touchpad','centroid','tx'):
                events.append(dict(time=stamp(t),kind=kind,message=r.get('message'),dropped=r.get('dropped_since_last')))
            if t<start or t>end:continue
            kinds[kind]+=1;ports[str(r.get('port'))]+=1
            minute=stamp(t)[:16];minutes[minute][kind]+=1
            if kind=='log_gap':gaps.append(r)
            if kind=='rx':rx_bytes+=r.get('byte_count',0)
            if kind=='tx':tx.append(dict(time=stamp(t),hex=r.get('hex'),bytes=r.get('byte_count')))
            f=r.get('frame',{})
            if kind=='frame':
                ft=f.get('type');types[ft]+=1
                if ft in ('debug','signal'):
                    sample_t.append(t);sample_seq.append(seq)
                    if ft=='debug':
                        frames.append(dict(time_ns=t,sequence=seq,frame=f))
                        for key in ('raw','baseline','signal','fast_k_percent','slow_k_percent'):
                            if key in f:channel_values[key].append(f[key])
                        channel_states.append(f.get('state',[]))
                else:
                    diagnostics.setdefault(ft,dict(first=dict(time=stamp(t),frame=f)))['last']=dict(time=stamp(t),frame=f)
            elif kind=='touchpad':
                touch.append(dict(time_ns=t,sequence=seq,frame=f))
                configurations[json.dumps(f.get('settings'),sort_keys=True)]+=1
                ss=f.get('source_sample_sequence')
                if last_tp is not None and ss!=last_tp+1:source_gaps.append(dict(time=stamp(t),previous=last_tp,current=ss))
                last_tp=ss
                if f.get('contact'):minutes[minute]['contact']+=1
stats.update(kinds=kinds,frame_types=types,ports=ports,session_events=events,log_gaps=gaps,sequence_gaps=seq_gaps,
             session_sequence_gaps=all_gaps,invalid_lines=bad,rx_bytes=rx_bytes,tx=tx,diagnostics=diagnostics,
             touch_settings=configurations,touch_source_gaps=source_gaps,per_minute=minutes)
def percentiles(values):
    if not len(values):return None
    return dict(zip(('min','p50','p90','p95','p99','max'),map(float,np.percentile(values,[0,50,90,95,99,100]))))
dt=np.diff(sample_t)/1e9
stats['sampling']=dict(count=len(sample_t),first=stamp(sample_t[0]),last=stamp(sample_t[-1]),
                       rate=(len(sample_t)-1)/((sample_t[-1]-sample_t[0])/1e9),dt=percentiles(dt),
                       gaps_over_150ms=[dict(time=stamp(sample_t[i+1]),seconds=float(v)) for i,v in enumerate(dt) if v>.15])
stats['channels']={}
for key,values in channel_values.items():
    a=np.array(values)
    stats['channels'][key]=dict(min=a.min(axis=0).tolist(),max=a.max(axis=0).tolist(),p50=np.median(a,axis=0).tolist())
stats['channels']['states']=[dict(Counter(row)) for row in np.array(channel_states).T.tolist()]
stats['touch']=dict(count=len(touch),contact=sum(bool(r['frame'].get('contact')) for r in touch),
                   reasons=dict(Counter(str(r['frame'].get('motion_block_reason')) for r in touch)))
groups=[];group=[]
for r in touch:
    f=r['frame']
    if f.get('contact') and f.get('filtered_xy') is not None:
        if group and (f['source_sample_sequence']!=group[-1]['frame']['source_sample_sequence']+1 or f['sample_timestamp']-group[-1]['frame']['sample_timestamp']>.15):
            groups.append(group);group=[]
        group.append(r)
    elif group:groups.append(group);group=[]
if group:groups.append(group)
summary=[]
for idx,g in enumerate(groups):
    if len(g)<2:continue
    f=np.array([r['frame']['filtered_xy'] for r in g]);raw=np.array([r['frame']['raw_xy'] for r in g])
    steps=np.diff(f,axis=0);length=np.linalg.norm(steps,axis=1).sum();net=f[-1]-f[0];dist=np.linalg.norm(net)
    _,s,v=np.linalg.svd(f-f.mean(axis=0),full_matrices=False)
    axis=v[0];angle=math.degrees(math.atan2(axis[1],axis[0]))%180
    lateral=np.abs((f-f[0])@np.array([-net[1],net[0]])/dist) if dist>.001 else np.zeros(len(f))
    item=dict(id=idx,start=stamp(g[0]['time_ns']),end=stamp(g[-1]['time_ns']),duration=g[-1]['frame']['sample_timestamp']-g[0]['frame']['sample_timestamp'],
              points=len(g),start_xy=f[0].tolist(),end_xy=f[-1].tolist(),net=net.tolist(),net_length=float(dist),path_length=float(length),
              bounds=np.ptp(f,axis=0).tolist(),angle=angle,pca_minor_major=float(s[1]/s[0]) if s[0]>0 else 0,
              net_minor_major=float(min(abs(net))/max(abs(net))) if max(abs(net))>0 else 0,
              lateral_max=float(max(lateral)),raw_filtered_distance=percentiles(np.linalg.norm(raw-f,axis=1)),
              raw_path_length=float(np.linalg.norm(np.diff(raw,axis=0),axis=1).sum()))
    summary.append(item)
stats['gestures']=summary
stats['meaningful_gestures']=[r for r in summary if r['path_length']>=.3 and r['duration']>=.2]
stats['overall_filter_distance']=percentiles([math.dist(r['frame']['raw_xy'],r['frame']['filtered_xy']) for r in touch if r['frame'].get('contact') and r['frame'].get('raw_xy') and r['frame'].get('filtered_xy')])
OUT.mkdir(parents=True,exist_ok=True)
(OUT/'audit.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf-8')
with (OUT/'touchpad.jsonl').open('w',encoding='utf-8') as f:
    for r in touch:f.write(json.dumps(r,ensure_ascii=False)+'\n')
with (OUT/'sensor_frames.jsonl').open('w',encoding='utf-8') as f:
    for r in frames:f.write(json.dumps(r,ensure_ascii=False)+'\n')
brief={k:stats[k] for k in ('session','start','end','kinds','frame_types','ports','session_events','log_gaps','sequence_gaps','invalid_lines','sampling','touch','touch_settings','touch_source_gaps','overall_filter_distance','tx')}
brief['meaningful_gesture_count']=len(stats['meaningful_gestures'])
brief['per_minute']=minutes
brief['gesture_examples']=sorted(stats['meaningful_gestures'],key=lambda r:r['path_length'],reverse=True)[:12]
print(json.dumps(brief,ensure_ascii=False,indent=2))
