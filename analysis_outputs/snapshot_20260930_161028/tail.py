import json
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parents[2]
P=Path(__file__).resolve().parent
rows=[]; gap_records=[]; kinds=Counter(); prev_rx=None; rx_gaps=[]
for number in (49,51,52):
    path=ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'/f'roll_20260930_151101_929680_5bc70e43_{number:05d}.jsonl'
    for line in path.open(encoding='utf-8'):
        r=json.loads(line); clock=r['timestamp'][11:23]
        if '16:07:59.000'<=clock<='16:08:07.500':
            kinds[r['kind']]+=1
            if r['kind']=='rx':
                if prev_rx and (r['time_ns']-prev_rx['time_ns'])/1e9>.1:
                    rx_gaps.append([prev_rx['timestamp'],r['timestamp'],(r['time_ns']-prev_rx['time_ns'])/1e9])
                prev_rx=r
                if '16:07:59.365'<clock<'16:08:07.256': gap_records.append(r)
            elif r['kind'] not in ('frame','centroid'): gap_records.append(r)
        if r['kind']=='frame' and r['frame']['type'] in ('debug','signal') and '16:10:27'<=clock<='16:11:28.320': rows.append(r)
print('GAP_AUDIT',dict(kinds),rx_gaps,'during',gap_records[:10])
print('SUPPLEMENT',rows[0]['timestamp'],rows[-1]['timestamp'],len(rows))
out={}
for c in (0,3,4,7,8,11,12,15,16,19):
    runs=[]; previous=None
    for r in rows:
        f=r['frame']; state=f['state'][c]
        v=dict(time=r['timestamp'],raw=f['raw'][c],baseline=f['baseline'][c],signal=f['signal'][c],state=state,fast=f['fast_k_percent'][c],slow=f['slow_k_percent'][c])
        if state!=previous:
            runs.append(dict(first=v,last=v,count=1,min_signal=v['signal'],max_signal=v['signal'],min_raw=v['raw'],max_raw=v['raw']))
            previous=state
        else:
            s=runs[-1];s['last']=v;s['count']+=1
            s['min_signal']=min(s['min_signal'],v['signal']);s['max_signal']=max(s['max_signal'],v['signal'])
            s['min_raw']=min(s['min_raw'],v['raw']);s['max_raw']=max(s['max_raw'],v['raw'])
    out[c]=runs
    print('CH',c,json.dumps(runs[-4:]))
(P/'supplement_after_snapshot.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
