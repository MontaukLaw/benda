"""分析最新保留会话的Raw、质心和释放时间；缺失参考时不伪造参考差值。"""
import json
import math
import sys
from pathlib import Path
from collections import Counter, deque

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser

paths=sorted((root/'PY_SRIPT/algo_monitor/recordings/rolling').glob('roll_20261004_141552_330023_99ea4de7_*.jsonl'))
cutoff='2026-10-04T14:51:00'
kinds,types,ports,modes,offsets,settings=Counter(),Counter(),Counter(),Counter(),Counter(),Counter()
sensors,motions,centroids,events,segments,gaps,partial=[],[],[],[],[],[],[]
parser,pending=AlgoFrameParser(),deque()
sensor_count=last_seq=0
first=last=None
first_seq=None
rx_mismatches=0
refs=[]
last_sensor_wall=None
max_sensor_gap=0
for path in paths:
    size=path.stat().st_size
    segment_start=None
    for line in path.read_bytes()[:size].splitlines():
        try:rec=json.loads(line)
        except ValueError:
            partial.append(path.name);continue
        wall=rec['timestamp'];first=first or wall;last=wall;segment_start=segment_start or wall
        seq=rec.get('sequence')
        if seq is not None:
            if first_seq is None:first_seq=seq
            elif seq!=last_seq+1:gaps.append([last_seq,seq])
            last_seq=seq
        kind=rec['kind'];kinds[kind]+=1;ports[rec.get('port')]+=1
        if kind=='rx':pending.extend(parser.feed(bytes.fromhex(rec['hex'])))
        elif kind=='frame':
            f=rec['frame'];types[f['type']]+=1
            # 开始文件可能位于RX帧中间，单独记录首个解析不一致，不当作后续断流。
            if not pending or json.dumps(pending.popleft(),sort_keys=True)!=json.dumps(f,sort_keys=True):rx_mismatches+=1
            if f['type'] in ('debug','signal'):
                sensor_count+=1
                if last_sensor_wall is not None:max_sensor_gap=max(max_sensor_gap,(rec['time_ns']-last_sensor_wall)/1e9)
                last_sensor_wall=rec['time_ns']
                modes[(f.get('protocol_version'),f.get('raw_only'),tuple(f.get('state',[])))]+=1
                if wall>=cutoff:sensors.append(dict(ordinal=sensor_count,wall=wall,time_ns=rec['time_ns'],raw=f.get('raw')))
            elif f['type']!='temperature':events.append(rec)
        elif kind=='centroid':
            f=rec['frame']
            if f.get('source_sample_sequence') is not None:offsets[f['source_sample_sequence']-sensor_count]+=1
            if wall>=cutoff and len(centroids)<5:centroids.append(rec)
        elif kind=='touchpad':
            f=rec['frame'];settings[json.dumps(f['settings'],sort_keys=True)]+=1
            if f.get('reference') is not None:refs.append(rec)
            if wall>=cutoff:motions.append(rec)
        else:events.append(rec)
    segments.append(dict(file=path.name,bytes=size,start=segment_start,end=last))
offset=offsets.most_common(1)[0][0]
sensor_map={s['ordinal']+offset:s for s in sensors}
source_gaps=[]
for a,b in zip(motions,motions[1:]):
    if b['frame']['source_sample_sequence']!=a['frame']['source_sample_sequence']+1:
        source_gaps.append([a['frame']['source_sample_sequence'],b['frame']['source_sample_sequence']])

def point(rec):
    f=rec['frame'];s=sensor_map.get(f['source_sample_sequence'])
    return dict(wall=rec['timestamp'],source=f['source_sample_sequence'],time=f['sample_timestamp'],
        contact=f['contact'],raw_xy=f['raw_xy'],filtered_xy=f['filtered_xy'],dx=f['dx'],dy=f['dy'],cursor=f['cursor'],
        raw=s['raw'] if s else None,gains=f['raw_gains'],settings=f['settings'],reason=f['motion_block_reason'],message=f['message'])

# 针对整体下降很明显但尚未停止的帧，保留其后释放过程。
window=deque()
candidates=[]
last_candidate=-1e9
for i,rec in enumerate(motions):
    p=point(rec);s=sensor_map.get(p['source'])
    if not s:continue
    while window and p['time']-window[0]['time']>.08:window.popleft()
    if window and p['contact'] and p['time']-window[0]['time']>=.025:
        old=window[0]
        changes=[(n-o)*g for n,o,g in zip(p['raw'],old['raw'],p['gains'])]
        losses=sum(max(0,-v) for v in changes);gains=sum(max(0,v) for v in changes)
        if losses>6000 and gains<losses*.2 and p['time']-last_candidate>.8:
            candidates.append((i,losses,gains,changes))
            last_candidate=p['time']
    window.append(p)

examples=[]
for i,losses,gains,changes in candidates:
    start=max(0,i-12)
    end=min(len(motions),i+120)
    block=next((j for j in range(i,end) if not motions[j]['frame']['contact']),None)
    if block is None:continue
    anchor=point(motions[max(0,i-4)])
    endp=point(motions[block-1])
    if not anchor['raw_xy'] or not endp['raw_xy']:continue
    dist=math.dist(anchor['raw_xy'],endp['raw_xy'])
    if dist<.18:continue
    tail_end=min(len(motions),block+55)
    span=[point(r) for r in motions[start:tail_end]]
    examples.append(dict(trigger=point(motions[i]),anchor=anchor,last_contact=endp,release=point(motions[block]),
        raw_centroid_distance=dist,losses_over_80ms=losses,gains_over_80ms=gains,
        weighted_raw_changes=changes,total_delta=[sum(r['frame'][k] for r in motions[i:block]) for k in ('dx','dy')],
        trace=span[::max(1,len(span)//12)]))
examples.sort(key=lambda e:e['raw_centroid_distance'],reverse=True)
report=dict(start=first,end=last,first_sequence=first_seq,last_sequence=last_seq,segments=segments,
    kinds=dict(kinds),types=dict(types),ports=dict(ports),log_sequence_gaps=gaps,partial_lines=partial,
    sensor_count=sensor_count,max_sensor_gap_seconds=max_sensor_gap,source_offset=offset,
    source_offset_candidates=offsets.most_common(5),source_gaps_last_5min=source_gaps,
    rx_parse_mismatches=rx_mismatches,reference_count=len(refs),settings=dict(settings),
    diagnostic_modes=[dict(version=v,raw_only=r,states=s,count=c) for (v,r,s),c in modes.items()],
    events=events,centroid_examples=centroids,
    raw_unloading_candidates=len(candidates),return_examples=len(examples),examples=examples[:12])
out=Path(__file__).with_name('return_audit.json')
out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('events','centroid_examples','segments','examples','diagnostic_modes')},ensure_ascii=False,indent=2))
for e in examples[:8]:
    print('EXAMPLE',e['trigger']['wall'],e['release']['wall'],'xy',e['anchor']['raw_xy'],e['last_contact']['raw_xy'],
          'delta',e['total_delta'],'loss',losses,'rise',gains)
print('CENTROID',json.dumps(centroids[:1],ensure_ascii=False))
