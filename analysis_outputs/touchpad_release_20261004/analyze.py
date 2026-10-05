import json
import math
import statistics
import bisect
import sys
from pathlib import Path
from collections import Counter, deque

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
from centroid_filter import signal_centroid
parser, pending = AlgoFrameParser(), deque()
rx_frames = rx_mismatches = 0
paths = sorted((root / 'PY_SRIPT/algo_monitor/recordings/rolling').glob('roll_20261004_112932_707592_59eabcbe_*.jsonl'))
kinds, types, ports = Counter(), Counter(), Counter()
samples, motions, events, gaps, segments = [], [], [], [], []
last_sequence = None
first = last = None
for path in paths:
    size = path.stat().st_size
    data = path.read_bytes()[:size]
    lines = data.splitlines()
    segment_first = segment_last = None
    for line in lines:
        try:
            rec = json.loads(line)
        except (ValueError, UnicodeDecodeError):
            events.append({'partial': path.name})
            continue
        first = first or rec['timestamp']
        last = rec['timestamp']
        segment_first = segment_first or last
        segment_last = last
        seq = rec.get('sequence')
        if seq is not None:
            if last_sequence is not None and seq != last_sequence + 1:
                gaps.append([last_sequence, seq])
            last_sequence = seq
        kind = rec['kind']
        kinds[kind] += 1
        ports[rec.get('port')] += 1
        if kind == 'rx':
            pending.extend(parser.feed(bytes.fromhex(rec['hex'])))
        if kind == 'frame':
            frame = rec['frame']
            rx_frames += 1
            if not pending or json.dumps(pending.popleft(), sort_keys=True) != json.dumps(frame, sort_keys=True):
                rx_mismatches += 1
            types[frame['type']] += 1
            if frame['type'] in ('debug', 'signal'):
                samples.append(rec)
            else:
                events.append(rec)
        elif kind == 'touchpad':
            motions.append(rec)
        elif kind not in ('rx', 'centroid'):
            events.append(rec)
    segments.append(dict(file=path.name, bytes=size, start=segment_first, end=segment_last))

# 仅debug/signal计入源采样序号；核对同一时钟下的首次运动帧关联。
offsets = Counter()
sensor_times = [r['monotonic_ns']/1e9 for r in samples]
for motion in motions[:200]:
    ft = motion['frame']['sample_timestamp']
    insert = bisect.bisect_left(sensor_times, ft)
    nearest = min(range(max(0,insert-4),min(len(samples),insert+4)), key=lambda i: abs(sensor_times[i]-ft))
    offsets[motion['frame']['source_sample_sequence']-(nearest+1)] += 1
offset = 0
reference = noise = None
ref_events = []
for rec in motions:
    f = rec['frame']
    if f.get('reference') is not None:
        reference, noise = f['reference'], f['noise']
        ref_events.append(dict(timestamp=rec['timestamp'], reference=reference, noise=noise, status=f.get('reference_status')))
    index = f['source_sample_sequence'] - offset - 1
    if 0 <= index < len(samples) and reference:
        sensor = samples[index]
        differences = [max(0, v-r) for v,r in zip(sensor['frame']['raw'], reference)]
        f['audit_max'] = max(differences)
        f['audit_sum'] = sum(differences)
        f['audit_top'] = sorted(enumerate(differences), key=lambda v: -v[1])[:5]
        f['audit_raw'] = sensor['frame']['raw']
        f['audit_sensor_timestamp'] = sensor['timestamp']
        f['audit_time_error'] = sensor['monotonic_ns']/1e9-f['sample_timestamp']

strokes, current = [], []
for rec in motions:
    if rec['frame']['contact']:
        current.append(rec)
    elif current:
        strokes.append((current, rec))
        current = []
if current:
    strokes.append((current, None))

def brief(rec):
    if rec is None:
        return None
    f = rec['frame']
    return dict(time=rec['timestamp'], source=f['source_sample_sequence'],
                contact=f['contact'], cursor=f['cursor'], dx=f['dx'], dy=f['dy'],
                raw_xy=f['raw_xy'], filtered_xy=f['filtered_xy'],
                max_difference=f.get('audit_max'), sum_difference=f.get('audit_sum'),
                top_channels=f.get('audit_top'), raw=f.get('audit_raw'),
                sensor_time=f.get('audit_sensor_timestamp'), time_error=f.get('audit_time_error'),
                message=f['message'], block=f.get('motion_block_reason'))

summaries = []
centroid_errors = []
for stroke, released in strokes:
    start, end = stroke[0], stroke[-1]
    f0, f1 = start['frame'], end['frame']
    peak_index = max(range(len(stroke)), key=lambda i: stroke[i]['frame'].get('audit_sum', 0))
    peak = stroke[peak_index]
    for rec in stroke[2:]:
        f = rec['frame']
        index = f['source_sample_sequence']-1
        raw = [statistics.median(col) for col in zip(*(s['frame']['raw'] for s in samples[index-2:index+1]))]
        diffs = [max(0,v-r) for v,r in zip(raw, reference)]
        weights = [v*(1-math.exp(-(v/n)**2)) for v,n in zip(diffs, noise)]
        centroid_errors.append(math.dist(signal_centroid(weights,False),f['raw_xy']))
    # 保存释放前1秒的位移及原始响应，供核对回缩过程。
    tail = [r for r in stroke if f1['sample_timestamp']-r['frame']['sample_timestamp'] <= 1.0]
    summaries.append(dict(start=brief(start), end=brief(end), release=brief(released),
        duration=f1['sample_timestamp']-f0['sample_timestamp'], frames=len(stroke),
        total_delta=[sum(r['frame'][key] for r in stroke) for key in ('dx','dy')],
        peak=brief(peak), tail_delta=[sum(r['frame'][key] for r in tail) for key in ('dx','dy')],
        tail=[brief(r) for r in tail[::max(1, len(tail)//12)]]))

intervals = [(b['monotonic_ns']-a['monotonic_ns'])/1e9 for a,b in zip(samples,samples[1:])]
motion_gaps = [[a['frame']['source_sample_sequence'],b['frame']['source_sample_sequence'],b['timestamp']]
              for a,b in zip(motions,motions[1:]) if b['frame']['source_sample_sequence'] != a['frame']['source_sample_sequence']+1]
report = dict(start=first,end=last,segments=segments,kinds=dict(kinds),types=dict(types),
    raw_rx_reparsed_frames=rx_frames, raw_rx_parse_mismatches=rx_mismatches,
    centroid_verified_frames=len(centroid_errors), centroid_max_error=max(centroid_errors),
    ports=dict(ports), sequence_gaps=gaps, sensor_count=len(samples),
    sensor_interval_median=statistics.median(intervals), sensor_interval_max=max(intervals),
    sensor_gaps_over_150ms=sum(t>.15 for t in intervals),
    source_offset=offset,source_offset_candidates=offsets.most_common(5),
    motion_sequence_gaps=motion_gaps,events=events,reference_events=ref_events,
    settings_counts=dict(Counter(json.dumps(r['frame']['settings']) for r in motions)),
    raw_only_samples=sum(bool(r['frame'].get('raw_only')) for r in samples),
    block_counts=dict(Counter(r['frame'].get('motion_block_reason') for r in motions)),
    strokes=summaries)
out = Path(__file__).with_name('audit.json')
out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=True), encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('strokes','events','reference_events')}, ensure_ascii=False, indent=2))
print('EVENTS', json.dumps([r for r in events if r.get('frame',{}).get('type')!='temperature'], ensure_ascii=False))
print('REFERENCE_COUNT', len(ref_events))
for i,s in enumerate(summaries):
    print('STROKE', i, s['start']['time'][11:23], s['end']['time'][11:23], round(s['duration'],2),
          'delta', [round(v,1) for v in s['total_delta']], 'tail', [round(v,1) for v in s['tail_delta']],
          'max', round(s['end']['max_difference'] or 0), 'release', s['release']['message'] if s['release'] else None)
print('REPORT',out)
