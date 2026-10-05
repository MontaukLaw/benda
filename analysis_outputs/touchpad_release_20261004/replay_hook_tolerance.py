import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker

touch=json.loads(Path(__file__).with_name('hooks_touch.json').read_text(encoding='utf-8'))
audit=json.loads(Path(__file__).with_name('hooks_audit.json').read_text(encoding='utf-8'))
targets={x['sample'] for x in audit['sharp_up'][:12]}
results=[]
for tolerance in (0,5,10,15,20):
    settings=dict(touch[0]['settings'])
    settings.pop('source')
    settings['raw_release_rise_percent']=tolerance
    tracker=TouchpadTracker(**settings)
    tracker.set_source('host_signal')
    tracker.set_enabled(True)
    events=[]
    moving=blocked=0
    negative_total=absolute_total=0
    for t in touch:
        tracker.feed(dict(sequence=t['source_sample_sequence'],sample_timestamp=t['sample_timestamp'],
            has_raw=True,channel_count=20,raw=[0]*20,host_signal=t['host_signal'],host_state=t['host_state'],
            host_generation=t['host_generation'],host_ready=True))
        dy=tracker.delta[1]
        moving+=tracker.contact
        blocked+=tracker.motion_block_reason=='coherent_release_tail'
        negative_total+=min(dy,0)
        absolute_total+=abs(dy)
        if t['source_sample_sequence'] in targets:
            events.append(dict(timestamp=t['timestamp'],old_dy=t['dy'],dy=dy,contact=tracker.contact,
                block=tracker.motion_block_reason))
    results.append(dict(tolerance=tolerance,moving_samples=moving,release_blocked_samples=blocked,
        negative_total=negative_total,absolute_total=absolute_total,events=events))
Path(__file__).with_name('hooks_tolerance_replay.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
for r in results:
    print(r['tolerance'],r['moving_samples'],r['release_blocked_samples'],round(r['negative_total'],1))
    for e in r['events']:
        if any(tm in e['timestamp'] for tm in ('19:42:18.','19:42:31.','19:43:20.')):print(e)
