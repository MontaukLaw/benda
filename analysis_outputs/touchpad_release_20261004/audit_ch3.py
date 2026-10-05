import collections
from datetime import datetime
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
from host_state_machine import HostStateMachine, HostParameters

directory = ROOT / 'PY_SRIPT/algo_monitor/recordings/rolling'
latest = max(directory.glob('roll_*.jsonl'), key=lambda p: p.stat().st_mtime)
prefix = latest.name.rsplit('_', 1)[0]
files = [(p, p.stat().st_size) for p in sorted(directory.glob(prefix + '_*.jsonl'))]
parser = AlgoFrameParser()
parsed = collections.deque()
kinds = collections.Counter()
types = collections.Counter()
ports = collections.Counter()
counts = [collections.Counter() for _ in range(20)]
diagnostics = {}
configs = []
events = []
gaps = []
seq = None
first = last = None
parser_mismatch = []
sensor_times = []
sensor = {}
host = []
raw_ranges = [[math.inf, -math.inf] for _ in range(20)]
parameter_id = -1
for path, limit in files:
    with path.open('rb') as stream:
        while stream.tell() < limit:
            line = stream.readline(min(limit - stream.tell(), 1000000))
            if not line.endswith(b'\n'):
                break
            r = json.loads(line)
            first = first or r['timestamp']
            last = r['timestamp']
            kinds[r['kind']] += 1
            ports[r.get('port')] += 1
            if seq is not None and r['sequence'] != seq + 1:
                gaps.append([seq, r['sequence'], r['timestamp']])
            seq = r['sequence']
            if r['kind'] == 'rx':
                parsed.extend(parser.feed(bytes.fromhex(r['hex'])))
            if r['kind'] in ('connected', 'disconnected', 'log_gap', 'error', 'tx'):
                events.append(r)
            if r['kind'] == 'host_state_config':
                configs.append(dict(timestamp=r['timestamp'], log_sequence=r['sequence'], **r['frame']))
                parameter_id = len(configs) - 1
            if r['kind'] == 'frame':
                f = r['frame']
                types[f['type']] += 1
                if not parsed:
                    parser_mismatch.append([r['sequence'], 'missing raw rx'])
                else:
                    decoded = parsed.popleft()
                    if json.dumps(decoded, sort_keys=True) != json.dumps(f, sort_keys=True):
                        parser_mismatch.append([r['sequence'], f['type']])
                if f['type'] in ('debug', 'signal'):
                    ordinal = len(sensor_times) + 1
                    sensor_times.append(r['time_ns'] / 1e9)
                    sensor[ordinal] = f
                    for ch, value in enumerate(f.get('raw', ())):
                        raw_ranges[ch][0] = min(raw_ranges[ch][0], value)
                        raw_ranges[ch][1] = max(raw_ranges[ch][1], value)
                else:
                    diagnostics.setdefault(f['type'], []).append(dict(timestamp=r['timestamp'], frame=f))
            if r['kind'] == 'host_state_machine':
                f = r['frame']
                host.append((r['timestamp'], r['sequence'], parameter_id, f))
                for ch, state in enumerate(f['state']):
                    counts[ch][state] += 1

def slim(row):
    timestamp, log_seq, pid, f = row
    return dict(timestamp=timestamp, sample_timestamp=f['sample_timestamp'],
                sequence=f['sequence'], generation=f['generation'], config_id=pid,
                input=f['input_val'][3], baseline=f['baseline'][3],
                difference=f['input_val'][3] - f['baseline'][3],
                signal=f['signal'][3], rate=f['pressure_rate'][3], state=f['state'][3])

rows = [slim(row) for row in host]
runs = []
for row in rows:
    key = (row['generation'], row['state'])
    if not runs or runs[-1]['key'] != key:
        runs.append(dict(key=key, start=row, end=row, samples=1,
                         min_difference=row['difference'], max_difference=row['difference'],
                         min_rate=row['rate'] if math.isfinite(row['rate']) else None,
                         max_rate=row['rate'] if math.isfinite(row['rate']) else None,
                         below_minus20=0, below_release50=0))
    else:
        runs[-1]['end'] = row
        runs[-1]['samples'] += 1
    run = runs[-1]
    run['min_difference'] = min(run['min_difference'], row['difference'])
    run['max_difference'] = max(run['max_difference'], row['difference'])
    if math.isfinite(row['rate']):
        run['min_rate'] = min(run['min_rate'], row['rate']) if run['min_rate'] is not None else row['rate']
        run['max_rate'] = max(run['max_rate'], row['rate']) if run['max_rate'] is not None else row['rate']
        run['below_minus20'] += row['rate'] <= -20
    run['below_release50'] += row['difference'] < 50
for run in runs:
    run['duration'] = run['end']['sample_timestamp'] - run['start']['sample_timestamp']

errors = collections.Counter()
max_errors = collections.defaultdict(float)
model = HostStateMachine()
generation = None
missing_sensor = []
for timestamp, log_seq, pid, f in host:
    source = sensor.get(f['sequence'])
    if not source:
        missing_sensor.append(f['sequence'])
        continue
    if f['generation'] != generation:
        cfg = configs[pid] if pid >= 0 else {}
        model.configure(HostParameters.from_settings(cfg.get('parameters', {})))
        model.set_gains(cfg.get('raw_gains'))
        model.reset(f.get('reset_reason', ''))
        model.last_sequence = f['sequence'] - 1
        generation = f['generation']
    result = model.feed(dict(sequence=f['sequence'], sample_timestamp=f['sample_timestamp'],
                             has_raw=source['type'] == 'debug', channel_count=source['channel_count'],
                             raw=source.get('raw', ())))
    for field in ('input_val', 'baseline', 'signal', 'pressure_rate', 'state'):
        for actual, expected in zip(f[field], result[field]):
            if math.isnan(actual) and math.isnan(expected):
                continue
            error = abs(actual - expected)
            max_errors[field] = max(max_errors[field], error)
            if error > 1e-7 or math.isnan(error):
                errors[field] += 1

transitions = [dict(previous=runs[i-1]['end'] if i else None, current=run['start']) for i,run in enumerate(runs)]
rate_rows = [r for r in rows if r['state'] == 1 and math.isfinite(r['rate'])]
tail_start = rows[-1]['sample_timestamp'] - 120 if rows else 0
tail = [r for r in rows if r['sample_timestamp'] >= tail_start]
lowest = sorted(rate_rows, key=lambda r: r['rate'])[:15]
report = dict(session=prefix, files=[dict(name=p.name, bytes=size) for p,size in files],
              audit_time=datetime.now().astimezone().isoformat(), first=first, last=last,
              kinds=dict(kinds), frame_types=dict(types), ports=dict(ports), sequence_gaps=gaps,
              parser_mismatches=parser_mismatch[:20], parser_pending=len(parsed),
              max_sensor_gap=max((b-a for a,b in zip(sensor_times,sensor_times[1:])), default=0),
              raw_ranges=raw_ranges, diagnostics=diagnostics, events=events, configs=configs,
              host_counts=counts, host_first=rows[0] if rows else None, host_last=rows[-1] if rows else None,
              host_sample_gaps=[(a['sequence'],b['sequence']) for a,b in zip(rows,rows[1:])
                                if b['sequence'] != a['sequence']+1],
              runs=runs, transitions=transitions, lowest_activate_rates=lowest,
              replay=dict(mismatches=dict(errors), max_errors=dict(max_errors), missing_source=missing_sensor,
                          final_drop_count=model.channels[3].drop_count,
                          final_release_count=model.channels[3].release_count),
              tail_2min=dict(first=tail[0] if tail else None, last=tail[-1] if tail else None,
                            states=dict(collections.Counter(r['state'] for r in tail)),
                            min_difference=min((r['difference'] for r in tail), default=None),
                            max_difference=max((r['difference'] for r in tail), default=None),
                            min_rate=min((r['rate'] for r in tail if math.isfinite(r['rate'])), default=None)))
out = Path(__file__).with_name('ch3_audit.json')
out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({key:report[key] for key in ('session','audit_time','first','last','kinds','frame_types','ports',
      'sequence_gaps','parser_mismatches','parser_pending','max_sensor_gap','configs','host_first','host_last',
      'host_sample_gaps','replay','tail_2min','lowest_activate_rates')}, ensure_ascii=False, indent=2))
print('STATE RUNS', len(runs))
for run in sorted((r for r in runs if r['key'][1] == 1), key=lambda r:r['duration'], reverse=True)[:3]:
    print(json.dumps(run, ensure_ascii=False))
print('DIAGNOSTIC TYPES', {key:len(value) for key,value in diagnostics.items()})
print('SAVED', out)
