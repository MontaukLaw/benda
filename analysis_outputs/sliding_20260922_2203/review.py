"""分析固定日志中的实际显示轨迹，不把红点事件重复计作串口采样。"""
from collections import Counter
import gzip
import json
import math
from pathlib import Path
import numpy as np

OUT = Path(__file__).resolve().parent
samples = json.load(gzip.open(OUT/'samples.json.gz', 'rt', encoding='utf-8'))
events = json.loads((OUT/'events.json').read_text(encoding='utf-8'))
centroids = [r for r in events if r['kind'] == 'centroid']


def quantile(values):
    return dict(zip(('p50','p95','p99','max'), np.percentile(values,[50,95,99,100]).round(4).tolist())) if values else {}


def reason(record):
    frame=record['frame']
    if frame['displayed_xy'] is not None: return 'visible'
    if not frame['fresh']: return 'stale'
    if frame['threshold_hidden']: return 'threshold'
    if frame['release_hidden']: return 'release'
    return 'zero'


groups=[]
for record in centroids:
    key=record['frame']['signal_threshold']
    if not groups or groups[-1][0]!=key: groups.append((key,[]))
    groups[-1][1].append(record)

stats=[]; largest=[]; brief_hidden=[]; episodes=[]
for threshold, rows in groups:
    visible=[r for r in rows if reason(r)=='visible']
    steps=[]; raw_steps=[]; offsets=[]
    reasons=Counter(reason(r) for r in rows)
    hidden_spans=[]; visible_spans=[]; span_start=0
    for i in range(1,len(rows)+1):
        if i==len(rows) or (reason(rows[i])=='visible')!=(reason(rows[span_start])=='visible'):
            end=rows[i]['time_ns'] if i<len(rows) else rows[-1]['time_ns']
            span=dict(start=rows[span_start]['timestamp'],end=rows[i]['timestamp'] if i<len(rows) else rows[-1]['timestamp'],
                      duration=(end-rows[span_start]['time_ns'])/1e9,frames=i-span_start,
                      complete=span_start>0 and i<len(rows))
            if reason(rows[span_start])=='visible':
                visible_spans.append(span)
                episodes.append(dict(threshold=threshold,**span))
            else:
                hidden_spans.append(span)
                if span['complete'] and span['duration']<.1:
                    before,after=rows[span_start-1]['frame'],rows[i]['frame']
                    detail=dict(threshold=threshold,**span,
                                min_max_signal=min(r['frame']['max_signal'] for r in rows[span_start:i]),
                                reasons=dict(Counter(reason(r) for r in rows[span_start:i])),
                                xy_before=before['displayed_xy'],xy_after=after['displayed_xy'],
                                endpoint_distance=math.dist(before['displayed_xy'],after['displayed_xy']),
                                source_before=before['source_sample_sequence'],source_after=after['source_sample_sequence'])
                    brief_hidden.append(detail)
            span_start=i
    for before,after in zip(rows,rows[1:]):
        a,b=before['frame'],after['frame']
        if a['displayed_xy'] is None or b['displayed_xy'] is None: continue
        step=math.dist(a['displayed_xy'],b['displayed_xy'])
        raw_step=math.dist(a['raw_xy'],b['raw_xy'])
        steps.append(step); raw_steps.append(raw_step)
        largest.append(dict(threshold=threshold,time=after['timestamp'],step=step,raw_step=raw_step,
            dt=(after['time_ns']-before['time_ns'])/1e9,source_before=a['source_sample_sequence'],source_after=b['source_sample_sequence'],
            max_before=a['max_signal'],max_after=b['max_signal'],sum_before=a['sum_signal'],sum_after=b['sum_signal']))
    for r in visible:
        f=r['frame']; offsets.append(math.dist(f['raw_xy'],f['displayed_xy']))
    stats.append(dict(threshold=threshold,start=rows[0]['timestamp'],end=rows[-1]['timestamp'],
        display_records=len(rows),reasons=dict(reasons),visible_episodes=len(visible_spans),
        brief_visible_under_100ms=sum(s['complete'] and s['duration']<.1 for s in visible_spans),
        brief_hidden_under_100ms=sum(s['complete'] and s['duration']<.1 for s in hidden_spans),
        raw_steps=quantile(raw_steps),display_steps=quantile(steps),display_steps_over_half_cell=sum(s>.5 for s in steps),
        raw_display_offset=quantile(offsets)))

dt=np.diff([r['time_ns']/1e9 for r in samples])
matched=sum(r['frame']['signal'] == samples[r['frame']['source_sample_sequence']-1]['frame']['signal'] for r in centroids if r['frame']['fresh'])
report=dict(sensor_frames=len(samples),fps=(len(samples)-1)/((samples[-1]['time_ns']-samples[0]['time_ns'])/1e9),
    sensor_interval=quantile(dt.tolist()),display_interval=quantile(np.diff([r['time_ns']/1e9 for r in centroids]).tolist()),
    matched_signal_records=matched,fresh_records=sum(r['frame']['fresh'] for r in centroids),
    threshold_violations=sum(r['frame']['displayed_xy'] is not None and r['frame']['max_signal']<r['frame']['signal_threshold'] for r in centroids),
    phases=stats,brief_hidden=brief_hidden,largest_visible_steps=sorted(largest,key=lambda r:r['step'],reverse=True)[:12])
(OUT/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'episodes.json').write_text(json.dumps(episodes,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({key:value for key,value in report.items() if key not in ('brief_hidden','largest_visible_steps')},ensure_ascii=False,indent=2))
print('BRIEF HIDDEN',json.dumps(brief_hidden[-12:],ensure_ascii=False))
print('LARGEST',json.dumps(report['largest_visible_steps'][:5],ensure_ascii=False))
