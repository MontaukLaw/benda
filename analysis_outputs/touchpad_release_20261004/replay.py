"""离线回放审计时保存的日志范围，不连接设备或输出系统鼠标。"""
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker

audit = json.loads(Path(__file__).with_name('audit.json').read_text(encoding='utf-8'))
samples, motions = [], []
for segment in audit['segments']:
    path = root/'PY_SRIPT/algo_monitor/recordings/rolling'/segment['file']
    for line in path.read_bytes()[:segment['bytes']].splitlines():
        rec = json.loads(line)
        if rec['kind'] == 'frame' and rec['frame']['type'] in ('debug','signal'):
            samples.append(rec['frame'])
        elif rec['kind'] == 'touchpad':
            motions.append(rec)
reference = audit['reference_events'][0]
tracker = TouchpadTracker(sensitivity=400, threshold=1000, min_cutoff=2.5)
tracker.set_enabled(True)
tracker.reference = tuple(reference['reference'])
tracker.noise = tuple(reference['noise'])
results, old_delta, new_delta = [], [0.,0.], [0.,0.]
for rec in motions:
    if rec['timestamp'] <= reference['timestamp']:
        continue
    f = rec['frame']
    sensor = samples[f['source_sample_sequence']-1]
    dx,dy = tracker.feed(dict(sensor, sequence=f['source_sample_sequence'],
                             sample_timestamp=f['sample_timestamp'],has_raw=sensor['type']=='debug'))
    if '2026-10-04T11:35:30.900' <= rec['timestamp'] <= '2026-10-04T11:35:31.400':
        results.append(dict(timestamp=rec['timestamp'],source=f['source_sample_sequence'],
            old_delta=[f['dx'],f['dy']],new_delta=[dx,dy],contact=tracker.contact,
            blocked=tracker.release_blocked,reason=tracker.motion_block_reason))
    if '2026-10-04T11:35:31.083' <= rec['timestamp'] <= '2026-10-04T11:35:31.386999':
        for index,(old,new) in enumerate(zip((f['dx'],f['dy']),(dx,dy))):
            old_delta[index] += old
            new_delta[index] += new
report = dict(session=audit['start'], gains=[1]*20,
    replayed_samples=len(motions), source='recorded_raw_and_sample_timestamps',
    comparison_window='11:35:31.083--11:35:31.386999',old_delta=old_delta,new_delta=new_delta,
    first_block=next((r for r in results if r['blocked']),None),details=results)
Path(__file__).with_name('replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='details'},ensure_ascii=False,indent=2))
