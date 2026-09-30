"""按旧日志实际显示采样时刻回放新消失锁定，不更改原始日志。"""
import json
import math
from pathlib import Path
import sys

OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parents[1]/'PY_SRIPT/algo_monitor'))
from centroid_filter import CentroidFilter

events=json.loads((OUT/'events.json').read_text(encoding='utf-8'))
review=json.loads((OUT/'review.json').read_text(encoding='utf-8'))
filt=CentroidFilter()
replayed={}
held=0
previous=None
freeze_errors=[]
for record in events:
    if record['kind']!='centroid': continue
    frame=record['frame']
    filt.set_signal_threshold(frame['signal_threshold'])
    point=filt.update(frame['signal'],frame['sample_timestamp'],frame.get('state'))
    if filt.holding_position:
        held+=1
        if previous is not None and point!=previous:
            freeze_errors.append(record['sequence'])
    replayed[frame['source_sample_sequence']]=dict(point=point,held=filt.holding_position,
        signal_valid=filt.visibility()['signal_valid'],timestamp=record['timestamp'])
    previous=point

cases=[]
for gap in review['brief_hidden']:
    if gap['threshold']!=1000: continue
    start,end=gap['source_before'],gap['source_after']
    before=replayed[start]['point']; after=replayed[end]['point']
    between=[replayed[k] for k in replayed if start<k<end]
    cases.append(dict(start=gap['start'],duration_ms=gap['duration']*1000,
        all_gap_points_frozen=all(r['held'] and r['point']==before for r in between),
        old_reappearance_distance=gap['endpoint_distance'],
        new_resume_distance=math.dist(before,after) if before and after else None))

assert not freeze_errors,freeze_errors
assert all(c['all_gap_points_frozen'] for c in cases),cases
report=dict(hold_seconds=.1,held_records=held,coordinate_changes_while_held=len(freeze_errors),
    short_gaps_at_threshold_1000=cases,
    note='真实GUI时钟会在100毫秒到期时刷新隐藏；此回放仅使用日志显示采样时刻。')
(OUT/'hold_replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
