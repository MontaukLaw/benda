"""核对增益版本会话，量化卸载门控条件，不把诊断帧计为采样。"""
import json
from pathlib import Path
import sys
from collections import Counter, deque

root = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker
from frame_parser import AlgoFrameParser

paths = sorted((root/'PY_SRIPT/algo_monitor/recordings/rolling').glob('roll_20261004_114750_449917_d00b1a10_*.jsonl'))
kinds,types,ports=Counter(),Counter(),Counter()
samples,motions,events,gaps=[],[],[],[]
parser,pending=AlgoFrameParser(),deque()
last_seq=first=last=None
rx_mismatch=0
for path in paths:
    size=path.stat().st_size
    for line in path.read_bytes()[:size].splitlines():
        try: rec=json.loads(line)
        except ValueError: continue
        first=first or rec['timestamp'];last=rec['timestamp']
        seq=rec.get('sequence')
        if seq is not None:
            if last_seq is not None and seq!=last_seq+1:gaps.append([last_seq,seq])
            last_seq=seq
        kind=rec['kind'];kinds[kind]+=1;ports[rec.get('port')]+=1
        if kind=='rx':pending.extend(parser.feed(bytes.fromhex(rec['hex'])))
        elif kind=='frame':
            f=rec['frame'];types[f['type']]+=1
            if not pending or json.dumps(pending.popleft(),sort_keys=True)!=json.dumps(f,sort_keys=True):rx_mismatch+=1
            if f['type'] in ('debug','signal'):samples.append(rec)
            elif f['type']!='temperature':events.append(rec)
        elif kind=='touchpad':motions.append(rec)
        elif kind!='centroid':events.append(rec)

reference=next(r for r in motions if r['frame'].get('reference') is not None)
f=reference['frame'];settings=f['settings']
tracker=TouchpadTracker(**settings)
tracker.set_raw_gains(f['raw_gains']);tracker.set_enabled(True)
tracker.reference=tuple(f['reference']);tracker.noise=tuple(f['noise'])
fail_counts=Counter();examples=[];source_gaps=[];prev_source=None
for rec in motions:
    if rec['timestamp']<=reference['timestamp']:continue
    f=rec['frame'];stamp=f['sample_timestamp'];seq=f['source_sample_sequence']
    if prev_source is not None and seq!=prev_source+1:source_gaps.append([prev_source,seq])
    prev_source=seq
    sensor=samples[seq-1]['frame']
    diffs=[max(0,v*g-r) for v,g,r in zip(sensor['raw'],tracker.raw_gains,tracker.reference)]
    recent=[s for s in tracker._release_samples if stamp-s[0]<=.08]
    if recent and stamp-recent[0][0]>=.025 and f['contact']:
        old=recent[0][1]
        gains=sum(max(0,n-o) for n,o in zip(diffs,old));losses=sum(max(0,o-n) for n,o in zip(diffs,old))
        noise=6*sum(tracker.noise);drop=losses/sum(old) if sum(old) else 0
        rise=max((n-o)/noise_ch for n,o,noise_ch in zip(diffs,old,tracker.noise))
        if losses>noise and drop>=.1 and gains<losses*.3:
            reasons=[]
            if drop<=.25:reasons.append('drop_below_25_percent')
            if gains>=losses*.2:reasons.append('gain_above_20_percent')
            if rise>6:reasons.append('channel_rise_above_6_noise')
            for reason in reasons:fail_counts[reason]+=1
            if reasons and len(examples)<12:
                examples.append(dict(timestamp=rec['timestamp'],source=seq,
                    drop_percent=100*drop,gain_percent=100*gains/losses,
                    channel_rise_noise_multiple=rise,dx=f['dx'],dy=f['dy'],reasons=reasons))
    tracker.feed(dict(sensor,sequence=seq,sample_timestamp=stamp,has_raw=True))

result=dict(start=first,end=last,ports=dict(ports),kinds=dict(kinds),types=dict(types),
    sequence_gaps=gaps,source_sample_gaps=source_gaps,rx_parse_mismatches=rx_mismatch,
    settings=settings,raw_gains=f['raw_gains'],blocked_reasons=dict(Counter(r['frame'].get('motion_block_reason') for r in motions)),
    unloading_candidate_failures=dict(fail_counts),examples=examples,events=events)
Path(__file__).with_name('latest_unload_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('events','examples')},ensure_ascii=False,indent=2))
print(json.dumps(examples[:5],ensure_ascii=False,indent=2))
