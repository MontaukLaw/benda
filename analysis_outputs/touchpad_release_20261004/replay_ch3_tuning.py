import collections
from dataclasses import replace
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from host_state_machine import _Channel, HostParameters

folder = ROOT / 'PY_SRIPT/algo_monitor/recordings/rolling'
files = [(p, p.stat().st_size) for p in sorted(folder.glob('roll_20261004_161923_730318_159d3977_*.jsonl'))]
raw = {}
sensor_count = 0
samples = []
for p,limit in files:
    with p.open('rb') as f:
        while f.tell() < limit:
            line = f.readline(min(limit-f.tell(),1000000))
            if not line.endswith(b'\n'): break
            r = json.loads(line)
            if r['kind'] == 'frame' and r['frame']['type'] in ('debug','signal'):
                sensor_count += 1
                if r['frame']['type'] == 'debug': raw[sensor_count] = r['frame']['raw'][3]
            elif r['kind'] == 'host_state_machine':
                s = r['frame']
                samples.append(dict(time=s['sample_timestamp'], stamp=r['timestamp'], sequence=s['sequence'],
                                    generation=s['generation'], raw=raw[s['sequence']],
                                    input=s['input_val'][3], baseline=s['baseline'][3], state=s['state'][3],
                                    rate=s['pressure_rate'][3], signal=s['signal'][3]))

default = HostParameters()
variants = dict(default=default, drop10=replace(default,drop_rate=10),
                release150=replace(default,release_threshold=150),
                hold100=replace(default,slowback_frames=100),
                reload800=replace(default,reload_threshold=800),
                reload60_800_5=replace(default,reload_rate=60,reload_threshold=800,reload_frames=5),
                hold100_reload60_800_5=replace(default,slowback_frames=100,reload_rate=60,reload_threshold=800,reload_frames=5),
                drop10_hold100_reload60_800_5=replace(default,drop_rate=10,slowback_frames=100,reload_rate=60,reload_threshold=800,reload_frames=5),
                window10=replace(default,window=10))
summary = {}
original_long_start = 448331.6581661
long_segment = [s for s in samples if original_long_start <= s['time'] <= 448345.3057861]
for name,p in variants.items():
    channel = None
    generation = None
    counts = collections.Counter()
    transitions = collections.Counter()
    runs = []
    mismatch = 0
    long_first_exit = None
    tail_states = collections.Counter()
    for s in samples:
        if s['generation'] != generation:
            generation = s['generation']
            channel = _Channel(p.window)
        before = channel.state
        channel.feed(s['raw'],s['time'],True,p)
        state = channel.state
        counts[state] += 1
        if before != state: transitions[f'{before}->{state}'] += 1
        if not runs or runs[-1]['state'] != state:
            runs.append(dict(state=state,start=s['time'],end=s['time'],stamp=s['stamp']))
        else: runs[-1]['end'] = s['time']
        if state != s['state']: mismatch += 1
        if s['time'] >= samples[-1]['time']-120: tail_states[state] += 1
        if s['time'] >= original_long_start and state != 1 and long_first_exit is None:
            long_first_exit = dict(timestamp=s['stamp'], elapsed=s['time']-original_long_start, state=state)
    actives = [r['end']-r['start'] for r in runs if r['state']==1]
    summary[name] = dict(states=dict(counts), transitions=dict(transitions), max_activate=max(actives,default=0),
                         median_activate=statistics.median(actives) if actives else 0,
                         original_long_segment_exit=long_first_exit, final_state=channel.state,
                         tail_2min=dict(tail_states), state_diff_from_record=mismatch)

observed_runs = []
for s in samples:
    if not observed_runs or observed_runs[-1]['state'] != s['state']:
        observed_runs.append(dict(state=s['state'],start=s,end=s,samples=1))
    else:
        observed_runs[-1]['end']=s
        observed_runs[-1]['samples']+=1
for r in observed_runs:
    r['duration']=r['end']['time']-r['start']['time']
late = [s for s in samples if samples[-1]['time']-s['time']<=30]
report = dict(audit_time=datetime.now().astimezone().isoformat(),first=samples[0]['stamp'],last=samples[-1]['stamp'],
              samples=len(samples),first_row=samples[0],last_row=samples[-1], variants=summary,
              observed_counts=dict(collections.Counter(s['state'] for s in samples)),
              longest_activate=sorted((r for r in observed_runs if r['state']==1),key=lambda r:r['duration'],reverse=True)[:5],
              last_30sec=dict(states=dict(collections.Counter(s['state'] for s in late)),
                              difference_range=[min(s['input']-s['baseline'] for s in late),max(s['input']-s['baseline'] for s in late)]),
              longest_active_rate_distribution={str(threshold):sum(s['rate']<=-threshold for s in long_segment) for threshold in (1,2,5,10,15,20)},
              latest_transitions=[dict(state=r['state'],timestamp=r['start']['stamp'],duration=r['duration'],samples=r['samples']) for r in observed_runs[-15:]])
Path(__file__).with_name('ch3_tuning_replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
