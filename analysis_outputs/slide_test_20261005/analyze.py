import bisect
import collections
import hashlib
import heapq
import json
import math
from pathlib import Path
import statistics
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
from centroid_filter import signal_centroid
from touchpad import TouchpadTracker

folder = ROOT / 'PY_SRIPT/algo_monitor/recordings/slide_tests'
archive = max(folder.glob('*.zip'), key=lambda p: p.stat().st_mtime)
output = Path(__file__).parent
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    meta = json.loads(z.read('manifest.json'))
    data = z.read('records.jsonl')
assert hashlib.sha256(data).hexdigest() == meta['records_sha256']
source = archive.with_suffix('') / 'records.jsonl'
assert hashlib.sha256(source.read_bytes()).hexdigest() == meta['records_sha256']
records = [json.loads(line) for line in data.splitlines()]
del data
context = meta['context']
times = {r['frame']['phase_index']: r['frame']['sample_timestamp'] for r in records
         if r['kind'] == 'slide_test' and r['frame'].get('event') == 'phase_end'}
phases = [dict(p, end=times[p['phase_index']]) for p in meta['phases']]
phase_starts = [p['sample_timestamp'] for p in phases]
hosts = {r['frame']['sequence']: dict(r['frame'], timestamp=r['timestamp']) for r in records if r['kind'] == 'host_state_machine'}
touch = [dict(r['frame'], timestamp=r['timestamp']) for r in records if r['kind'] == 'touchpad']
log_gaps = [(a['sequence'], b['sequence']) for a, b in zip(records, records[1:]) if b['sequence'] != a['sequence'] + 1]
test_kinds = dict(collections.Counter(r['kind'] for r in records))
test_types = dict(collections.Counter(r['frame']['type'] for r in records if r['kind'] == 'frame'))
first_log, last_log = records[0]['sequence'], records[-1]['sequence']
prefix = 'roll_' + context['rolling_session']
roll_dir = ROOT / 'PY_SRIPT/algo_monitor/recordings/rolling'
files = [(p, p.stat().st_size) for p in sorted(roll_dir.glob(prefix + '_*.jsonl'))]
parser = AlgoFrameParser()
decoded = collections.deque()
raw_samples = {}
sensor_sequence = 0
roll_kinds, roll_types = collections.Counter(), collections.Counter()
roll_gaps, parser_errors, events, configs, diagnostics = [], [], [], [], []
previous_log = None
first_time = last_time = None
test_raw_comparison = {}
count_rolling_test = 0
firmware = collections.Counter()
done = False
for p, size in files:
    with p.open('rb') as stream:
        while stream.tell() < size:
            line = stream.readline(size - stream.tell())
            if not line.endswith(b'\n'):
                break
            r = json.loads(line)
            if r['sequence'] > last_log:
                done = True
                break
            first_time = first_time or r['timestamp']
            last_time = r['timestamp']
            kind = r['kind']
            roll_kinds[kind] += 1
            if previous_log is not None and r['sequence'] != previous_log + 1:
                roll_gaps.append([previous_log, r['sequence']])
            previous_log = r['sequence']
            if first_log <= r['sequence'] <= last_log:
                original = records[r['sequence'] - first_log]
                assert original == r
                count_rolling_test += 1
            if kind == 'rx':
                decoded.extend(parser.feed(bytes.fromhex(r['hex'])))
            elif kind == 'frame':
                f = r['frame']
                roll_types[f['type']] += 1
                if not decoded or json.dumps(decoded.popleft(), sort_keys=True) != json.dumps(f, sort_keys=True):
                    parser_errors.append(r['sequence'])
                if f['type'] in ('debug', 'signal'):
                    sensor_sequence += 1
                    raw_samples[sensor_sequence] = f
                    firmware['raw_only'] += bool(f.get('raw_only'))
                    firmware['nonzero_baseline'] += any(f.get('baseline', []))
                    firmware['nonzero_signal'] += any(f.get('signal', []))
                    firmware['finite_k'] += any(math.isfinite(v) for v in f.get('fast_k_percent', []) + f.get('slow_k_percent', []))
                    for s in set(f.get('state', [])):
                        firmware['state_' + str(s)] += 1
                elif f['type'] != 'temperature':
                    diagnostics.append(r)
            elif kind == 'host_state_config':
                configs.append(r)
            if kind in ('connected', 'disconnected', 'log_gap', 'error', 'tx'):
                events.append(r)
        if done:
            break
assert count_rolling_test == len(records)

input_error = 0.
for a, b in zip(hosts.values(), list(hosts.values())[1:]):
    raw = raw_samples[b['sequence']]['raw']
    expected = [.2 * gain * v + .8 * old for gain, v, old in zip(context['raw_gains'], raw, a['input_val'])]
    input_error = max(input_error, max(abs(x-y) for x, y in zip(expected, b['input_val'])))
tracker = TouchpadTracker(**{k: v for k, v in context['touchpad'].items() if k != 'source'})
tracker.set_source('host_signal')
tracker.set_enabled(True)
replay_errors = collections.Counter()
max_dxdy_error = 0.
host_mismatches = 0
for t in touch:
    h = hosts[t['source_sample_sequence']]
    host_mismatches += t['host_signal'] != h['signal'] or t['host_state'] != h['state']
    tracker.feed(dict(sequence=h['sequence'], sample_timestamp=h['sample_timestamp'], has_raw=True, channel_count=20,
        raw=raw_samples[h['sequence']]['raw'], host_signal=h['signal'], host_state=h['state'],
        host_generation=h['generation'], host_ready=all(s != -1 for s in h['state'])))
    for axis, expected, actual in zip(('dx', 'dy'), (t['dx'], t['dy']), tracker.delta):
        error = abs(expected - actual)
        max_dxdy_error = max(max_dxdy_error, error)
        if error > 1e-7:
            replay_errors[axis] += 1
    if tracker.contact != t['contact']:
        replay_errors['contact'] += 1
    if tracker.motion_block_reason != t.get('motion_block_reason'):
        replay_errors['block'] += 1

def phase_of(t):
    i = bisect.bisect_right(phase_starts, t) - 1
    return phases[i] if i >= 0 and t < phases[i]['end'] else None

def percentile(values, p):
    values = sorted(values)
    return values[round((len(values)-1)*p)] if values else 0

stats = []
top_steps = []
series = []
for p in phases:
    if p['phase'] != 'slide':
        continue
    axis = 0 if p['axis'] == 'horizontal' else 1
    start, slide_end = p['sample_timestamp'], p['end']
    release = phases[p['phase_index'] + 1]
    end = release['end']
    rows = [t for t in touch if start <= t['sample_timestamp'] < end]
    slide_rows = [t for t in rows if t['sample_timestamp'] < slide_end]
    base_start = phases[p['phase_index'] - 1]['sample_timestamp']
    before = [h for h in hosts.values() if max(base_start, start-1) <= h['sample_timestamp'] < start]
    release_reference = [statistics.median(h['input_val'][ch] for h in before) for ch in range(20)]
    net = [sum(t[k] for t in rows) for k in ('dx','dy')]
    slide_net = [sum(t[k] for t in slide_rows) for k in ('dx','dy')]
    positive = sum(max(0, (t['dx'],t['dy'])[axis]) for t in rows)
    negative = sum(max(0, -(t['dx'],t['dy'])[axis]) for t in rows)
    blocks = dict(collections.Counter(t.get('motion_block_reason') for t in slide_rows if t.get('motion_block_reason')))
    segments = []
    current = None
    active_steps = []
    prior = None
    previous_soft = None
    previous_reference = None
    soft_steps = []
    ref_steps = []
    soft_values = []
    ref_values = []
    traces = []
    cumulative = [0., 0.]
    for t in rows:
        h = hosts[t['source_sample_sequence']]
        soft_weights = [max(0, v-b) for v,b in zip(h['input_val'], h['baseline'])]
        soft = signal_centroid(soft_weights, local_2x2_enabled=False)
        ref_weights = [max(0, v-b-20) for v,b in zip(h['input_val'], release_reference)]
        reference = signal_centroid(ref_weights, local_2x2_enabled=False) if max(ref_weights) >= 500 else None
        if t['contact']:
            if current is None:
                current = dict(start=t['sample_timestamp'] - start, frames=0, dx=0, dy=0)
            current['frames'] += 1
            current['dx'] += t['dx']
            current['dy'] += t['dy']
        elif current is not None:
            current.update(end=prior['sample_timestamp']-start, end_reason=t.get('motion_block_reason'),
                           blocked_active=t['host_active_channels'], blocked_signal=sum(t['host_signal']))
            segments.append(current)
            current = None
        if prior and t['contact'] and prior['contact']:
            dt = t['sample_timestamp']-prior['sample_timestamp']
            delta = [a-b for a,b in zip(t['raw_xy'], prior['raw_xy'])]
            changed = [ch for ch in range(20) if t['host_state'][ch] != prior['host_state'][ch]]
            active_steps.append(dict(length=math.hypot(*delta), axis_delta=delta[axis], changed=changed,
                                     speed=math.hypot(t['dx'],t['dy']) / dt))
            if soft and previous_soft:
                soft_delta = [a-b for a,b in zip(soft, previous_soft)]
                soft_steps.append(math.hypot(*soft_delta))
            else:
                soft_delta = None
            if reference and previous_reference:
                ref_delta = [a-b for a,b in zip(reference, previous_reference)]
                ref_steps.append(math.hypot(*ref_delta))
            else:
                ref_delta = None
            item = dict(axis=p['axis'], trial=p['trial'], timestamp=t['timestamp'],
                        sample=t['source_sample_sequence'], relative_time=t['sample_timestamp']-start,
                        dt=dt, delta=delta, pixel_delta=[t['dx'],t['dy']], old_xy=prior['raw_xy'], new_xy=t['raw_xy'],
                        changed=changed, old_state=prior['host_state'], new_state=t['host_state'],
                        old_signal=prior['host_signal'], new_signal=t['host_signal'],
                        input=h['input_val'], baseline=h['baseline'], rate=h['decision_rate'],
                        soft_delta=soft_delta, fixed_reference_delta=ref_delta)
            heapq.heappush(top_steps, (math.hypot(*delta), item['sample'], item))
            if len(top_steps)>30:
                heapq.heappop(top_steps)
        if t['contact'] and soft:
            soft_values.append(soft)
        if reference and t['sample_timestamp'] < slide_end:
            ref_values.append(reference)
        cumulative = [a+b for a,b in zip(cumulative,(t['dx'],t['dy']))]
        traces.append(dict(time=t['sample_timestamp']-start, contact=t['contact'], raw=t['raw_xy'],
            position=t['filtered_xy'], cumulative=cumulative, soft=soft,
            fixed_reference=reference, total_signal=sum(t['host_signal']), total_reference=sum(ref_weights),
            states=t['host_state'], block=t.get('motion_block_reason')))
        previous_soft, previous_reference, prior = soft, reference, t
    if current:
        current.update(end=rows[-1]['sample_timestamp']-start, end_reason='window_end')
        segments.append(current)
    stat = dict(axis=p['axis'], trial=p['trial'], timestamp=p['timestamp'],
                total_frames=len(rows), slide_frames=len(slide_rows),
                contact_frames=sum(t['contact'] for t in slide_rows),
                net_pixels=net, slide_net_pixels=slide_net, main_pixels=net[axis], cross_pixels=net[1-axis],
                forward_pixels=positive, reverse_pixels=negative,
                peak_pixel_speed=max((s['speed'] for s in active_steps), default=0),
                segments=segments, blocks=blocks,
                first_contact_seconds=min((t['sample_timestamp']-start for t in rows if t['contact']),default=None),
                last_contact_seconds=max((t['sample_timestamp']-start for t in rows if t['contact']),default=None),
                active_steps=len(active_steps), state_change_steps=sum(bool(s['changed']) for s in active_steps),
                sharp_steps=sum(s['length']>.1 for s in active_steps),
                sharp_with_state_change=sum(s['length']>.1 and bool(s['changed']) for s in active_steps),
                centroid_step_p95=percentile([s['length'] for s in active_steps],.95),
                continuous_step_p95=percentile(soft_steps,.95), fixed_reference_step_p95=percentile(ref_steps,.95),
                reference_span_cells=[max(v[a] for v in ref_values)-min(v[a] for v in ref_values) for a in (0,1)] if ref_values else None,
                initial_baseline=before[-1]['baseline'], released_reference=release_reference,
                signal_peak=max(sum(t['host_signal']) for t in rows),
                active_channels=sorted({ch for t in rows for ch in t['host_active_channels']}))
    stats.append(stat)
    series.append(dict(axis=p['axis'],trial=p['trial'],traces=traces))

summary = {}
for axis in ('horizontal','vertical'):
    group = [s for s in stats if s['axis']==axis]
    values = [s['main_pixels'] for s in group]
    summary[axis] = dict(mean_pixels=statistics.mean(values), min_pixels=min(values), max_pixels=max(values),
        sd_pixels=statistics.stdev(values), cv=statistics.stdev(values)/abs(statistics.mean(values)),
        mean_cross_pixels=statistics.mean(s['cross_pixels'] for s in group),
        mean_abs_cross_pixels=statistics.mean(abs(s['cross_pixels']) for s in group),
        state_change_large_steps=sum(s['sharp_with_state_change'] for s in group), large_steps=sum(s['sharp_steps'] for s in group),
        complete_contacts=sum(len(s['segments']) for s in group),
        end_reasons=dict(collections.Counter(seg.get('end_reason') for s in group for seg in s['segments'])),
        mean_raw_step_p95=statistics.mean(s['centroid_step_p95'] for s in group),
        mean_continuous_step_p95=statistics.mean(s['continuous_step_p95'] for s in group),
        mean_reference_step_p95=statistics.mean(s['fixed_reference_step_p95'] for s in group))
result = dict(archive=str(archive), started=meta['started_at'], ended=meta['ended_at'],
    sha256=meta['records_sha256'], zip_and_source_sha_verified=True, matched_rolling_records=count_rolling_test,
    test_kinds=test_kinds, test_types=test_types, test_sequence_gaps=log_gaps,
    rolling_start=first_time, rolling_end=last_time, rolling_sequence_gaps=roll_gaps,
    rolling_kinds=dict(roll_kinds), rolling_types=dict(roll_types), parser_mismatches=parser_errors,
    sensor_count=sensor_sequence, firmware=dict(firmware), events=events, configs=configs, diagnostics=diagnostics,
    host_generation_counts=dict(collections.Counter(h['generation'] for h in hosts.values())),
    gains_lowpass_max_error=input_error, touch_host_mismatches=host_mismatches,
    touch_replay_errors=dict(replay_errors), touch_replay_max_pixel_error=max_dxdy_error,
    protocol=context['protocol'], manifest_gaps=meta['derived_sequence_gaps'],
    sensor_max_gap=meta['largest_sensor_gap_seconds'], summary=summary, trials=stats,
    largest_steps=[v[2] for v in sorted(top_steps,reverse=True)])
(output/'audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
(output/'series.json').write_text(json.dumps(series,ensure_ascii=False),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('events','configs','diagnostics','trials','largest_steps')},ensure_ascii=False,indent=2))
for stat in stats:
    print('TRIAL',json.dumps({k:stat[k] for k in ('axis','trial','main_pixels','cross_pixels','slide_net_pixels','forward_pixels','reverse_pixels','first_contact_seconds','last_contact_seconds','peak_pixel_speed','reference_span_cells','sharp_steps','sharp_with_state_change','segments')},ensure_ascii=False))
for item in result['largest_steps'][:5]:
    print('STEP',json.dumps(item,ensure_ascii=False))
