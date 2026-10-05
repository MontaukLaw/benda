import collections
import heapq
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
parser = AlgoFrameParser()
decoded = collections.deque()
kinds, types, ports, blocks, ends, firmware = [collections.Counter() for _ in range(6)]
hosts = collections.OrderedDict()
gaps, parse_errors, events, configs, diagnostics = [], [], [], [], []
first = last = previous_seq = previous_sensor_time = None
sensor_count = 0
max_sensor_gap = 0
sample_gaps = []
settings = set()
generations = collections.Counter()
touch_count = aligned = unaligned = input_mismatch = 0
pixel_error = 0
previous = None
segment = None
segments = []
big_steps, cut_steps = [], []
continuing_steps = cut_count = 0
path = cut_path = 0
active_isolated = 0

def xy(weights):
    total = sum(weights)
    return [sum((ch // 4 + .5) * w for ch, w in enumerate(weights)) / total,
            sum((ch % 4 + .5) * w for ch, w in enumerate(weights)) / total] if total else None

def top(heap, magnitude, item):
    heapq.heappush(heap, (magnitude, item['sample'], item))
    if len(heap) > 20:
        heapq.heappop(heap)

for p, size in files:
    try:
        stream = p.open('rb')
    except FileNotFoundError:
        events.append({'missing_file': p.name})
        continue
    with stream:
        while stream.tell() < size:
            line = stream.readline(size - stream.tell())
            if not line.endswith(b'\n'):
                break
            r = json.loads(line)
            first = first or r['timestamp']
            last = r['timestamp']
            kind = r['kind']
            kinds[kind] += 1
            ports[r.get('port')] += 1
            if previous_seq is not None and r['sequence'] != previous_seq + 1:
                gaps.append([previous_seq, r['sequence'], r['timestamp']])
            previous_seq = r['sequence']
            if kind == 'rx':
                decoded.extend(parser.feed(bytes.fromhex(r['hex'])))
            elif kind == 'frame':
                f = r['frame']
                types[f['type']] += 1
                if not decoded or json.dumps(decoded.popleft(), sort_keys=True) != json.dumps(f, sort_keys=True):
                    parse_errors.append(r['sequence'])
                if f['type'] in ('debug', 'signal'):
                    sensor_count += 1
                    ts = r['monotonic_ns'] / 1e9
                    if previous_sensor_time is not None:
                        gap = ts - previous_sensor_time
                        max_sensor_gap = max(max_sensor_gap, gap)
                        if gap > .15:
                            sample_gaps.append([r['timestamp'], gap])
                    previous_sensor_time = ts
                    firmware['raw_only'] += bool(f.get('raw_only'))
                    firmware['nonzero_signal'] += any(f.get('signal', []))
                    firmware['nonzero_baseline'] += any(f.get('baseline', []))
                    firmware['finite_k_frames'] += any(math.isfinite(v) for v in f.get('k', []))
                    for state in set(f.get('state', [])):
                        firmware['state_' + str(state)] += 1
                else:
                    if len(diagnostics) < 100:
                        diagnostics.append(r)
            elif kind == 'host_state_machine':
                h = r['frame']
                hosts[h['sequence']] = h
                if len(hosts) > 1000:
                    hosts.popitem(last=False)
                generations[h['generation']] += 1
            elif kind == 'host_state_config':
                configs.append(r)
            elif kind == 'touchpad':
                t = dict(timestamp=r['timestamp'], **r['frame'])
                touch_count += 1
                settings.add(json.dumps(t['settings'], sort_keys=True))
                blocks[str(t.get('motion_block_reason'))] += 1
                h = hosts.get(t['source_sample_sequence'])
                if h is None:
                    unaligned += 1
                else:
                    aligned += 1
                    input_mismatch += t.get('host_signal') != h['signal'] or t.get('host_state') != h['state']
                pixel_error = max(pixel_error, abs(t['dx'] - t['centroid_dx'] * t['settings']['sensitivity']),
                                  abs(t['dy'] - t['centroid_dy'] * t['settings']['sensitivity']))
                if t['contact']:
                    paired = {ch for pair in t['host_adjacent_pairs'] for ch in pair}
                    active_isolated += any(ch not in paired for ch in t['host_active_channels'])
                    if segment is None:
                        segment = dict(start=t['timestamp'], start_sample=t['source_sample_sequence'],
                                       start_time=t['sample_timestamp'], frames=0, dx=0, dy=0, path=0,
                                       raw_start=t['raw_xy'], filtered_start=t['filtered_xy'])
                    segment['frames'] += 1
                    segment['dx'] += t['dx']
                    segment['dy'] += t['dy']
                    segment['path'] += math.hypot(t['dx'], t['dy'])
                elif segment is not None:
                    reason = t.get('motion_block_reason') or ('threshold' if '阈值' in t.get('message', '') else t.get('message'))
                    ends[str(reason)] += 1
                    segment.update(end=previous['timestamp'], duration=previous['sample_timestamp'] - segment['start_time'],
                                   end_reason=reason, raw_end=previous['raw_xy'], filtered_end=previous['filtered_xy'])
                    segments.append(segment)
                    segment = None
                if previous and previous['contact'] and t['contact'] and previous['raw_xy'] and t['raw_xy']:
                    a = previous
                    delta = [b - c for b, c in zip(t['raw_xy'], a['raw_xy'])]
                    length = math.hypot(*delta)
                    continuing_steps += 1
                    path += length
                    outgoing = [ch for ch in range(20) if a['host_state'][ch] == 1 and t['host_state'][ch] != 1]
                    incoming = [ch for ch in range(20) if a['host_state'][ch] != 1 and t['host_state'][ch] == 1]
                    item = dict(timestamp=t['timestamp'], sample=t['source_sample_sequence'],
                                dt=t['sample_timestamp'] - a['sample_timestamp'], raw_delta=delta,
                                pixel_delta=[t['dx'], t['dy']], old_xy=a['raw_xy'], new_xy=t['raw_xy'],
                                old_active=a['host_active_channels'], new_active=t['host_active_channels'],
                                outgoing=outgoing, incoming=incoming, pairs=t['host_adjacent_pairs'],
                                old_signal=a['host_signal'], new_signal=t['host_signal'])
                    if h:
                        item.update(input=h['input_val'], baseline=h['baseline'], rate=h['decision_rate'])
                    top(big_steps, length, item)
                    if outgoing:
                        cut_count += 1
                        cut_path += length
                        cut_weights = list(a['host_signal'])
                        for ch in outgoing:
                            cut_weights[ch] = 0
                        cut_xy = xy(cut_weights)
                        item['cut_only_delta'] = [b - c for b, c in zip(cut_xy, a['raw_xy'])] if cut_xy else None
                        top(cut_steps, length, item)
                previous = t
            if kind in ('connected', 'disconnected', 'log_gap', 'error', 'tx'):
                events.append(r)

result = dict(session=prefix, first=first, last=last, files=[dict(name=p.name, bytes=s) for p, s in files],
              kinds=dict(kinds), types=dict(types), ports=dict(ports), sequence_gaps=gaps,
              parser_mismatches=parse_errors, pending_decoded=len(decoded), sensor_count=sensor_count,
              max_sensor_gap=max_sensor_gap, sensor_gaps=sample_gaps, firmware=dict(firmware),
              events=events, configs=configs, diagnostics=diagnostics, touch_count=touch_count,
              touch_settings=[json.loads(s) for s in settings], host_generations=dict(generations),
              matched_touch_frames=aligned, unmatched_touch_frames=unaligned, touch_host_mismatches=input_mismatch,
              pixel_conversion_max_error=pixel_error, block_reasons=dict(blocks), contact_end_reasons=dict(ends),
              complete_contact_segments=len(segments), contact_segments=segments,
              continuing_steps=continuing_steps, raw_path_cells=path, state_cut_steps=cut_count,
              state_cut_path_cells=cut_path, contact_frames_with_isolated_active=active_isolated,
              largest_steps=[v[2] for v in sorted(big_steps, reverse=True)],
              largest_cut_steps=[v[2] for v in sorted(cut_steps, reverse=True)])
Path(__file__).with_name('audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: v for k, v in result.items() if k not in ('files', 'events', 'configs', 'diagnostics', 'contact_segments', 'largest_steps', 'largest_cut_steps')}, ensure_ascii=False, indent=2))
for item in result['largest_steps'][:6]:
    print('STEP', json.dumps(item, ensure_ascii=False))
print('CONFIGS', json.dumps(configs, ensure_ascii=False))
print('EVENTS', json.dumps(events, ensure_ascii=False))
