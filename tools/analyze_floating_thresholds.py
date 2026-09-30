"""离线扫描FLOATING慢加压候选阈值；不连接设备，不更改固件。"""
import csv
import json
import sqlite3
import hashlib
import math
from pathlib import Path
from datetime import datetime
from contextlib import closing
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT/'PY_SRIPT/algo_monitor'
CHANNELS = range(4, 8)


def read_csv(path):
    with path.open(encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def elapsed(row):
    return datetime.fromisoformat(row['timestamp']).timestamp()


def features(t, raw):
    """用真实时间计算200ms净增量；插值端点必须有数据夹住，窗口缺口不得超过180ms。"""
    out=np.full(len(t),np.nan)
    for i in range(len(t)):
        target=t[i]-.2
        j=int(np.searchsorted(t,target,side='right'))-1
        if j<0 or j>=i or np.max(np.diff(t[j:i+1]))>.18:
            continue
        before=np.interp(target,t[j:j+2],raw[j:j+2])
        out[i]=raw[i]-before
    return out


def describe(values):
    values=np.asarray(values,dtype=float);values=values[np.isfinite(values)]
    if not len(values):return {'n':0}
    return dict(n=len(values),minimum=float(min(values)),p10=float(np.percentile(values,10)),
                median=float(np.median(values)),p90=float(np.percentile(values,90)),
                p99=float(np.percentile(values,99)),maximum=float(max(values)))


def main():
    sources=[]
    scan_path=DATA/'force_position_maps/force_position_scan_20260915_162427_315.csv'
    scan=read_csv(scan_path);sources.append(scan_path)
    low,high=elapsed(scan[0]),elapsed(scan[-1]);motor=[];missing_sources=[]
    for stamp in ('20260915_161835_458','20260915_162442_207','20260915_162740_730'):
        path=DATA/f'force_press_tests/force_press_{stamp}.csv'
        if not path.exists():
            missing_sources.append(str(path));continue
        sources.append(path)
        for r in read_csv(path):
            if not low<=elapsed(r)<=high or r['sensor_four_channel_valid']!='True':continue
            age=float(r['sensor_age_ms'] or 'inf')
            if age>.25*1000:continue
            r['source_time']=elapsed(r)-age/1000
            motor.append(r)
    seen=set();unique=[]
    for r in sorted(motor,key=lambda r:r['source_time']):
        if r['sensor_sequence'] not in seen:
            seen.add(r['sensor_sequence']);unique.append(r)
    steps=[];boundaries={c:[] for c in CHANNELS}
    for rep in (range(2,11) if not missing_sources else ()):
        points=[r for r in scan if int(r['round_index'])==rep]
        rows=[r for r in unique if elapsed(points[0])<=r['source_time']<=elapsed(points[-1])]
        t=np.array([r['source_time'] for r in rows])
        raw={c:np.array([float(r[f'ch{c}_normalized_raw']) for r in rows]) for c in CHANNELS}
        delta={c:features(t,raw[c]) for c in CHANNELS}
        for c in CHANNELS:
            if rep<=7:
                b=[]
                for force in (6,12):
                    p=min(points,key=lambda r:abs(float(r['force_n'])-force))
                    i=int(np.argmin(abs(t-elapsed(p))))
                    b.append(raw[c][i]-float(rows[i][f'ch{c}_baseline']))
                boundaries[c].append(b)
        for previous,current in zip(points,points[1:]):
            f0,f1=float(previous['force_n']),float(current['force_n'])
            if f0<1 or f1-f0<.15:continue
            a=int(np.argmin(abs(t-elapsed(previous))));b=int(np.argmin(abs(t-elapsed(current))))
            if abs(t[a]-elapsed(previous))>.1 or abs(t[b]-elapsed(current))>.1:continue
            active=(t>=elapsed(previous))&(t<=elapsed(current))
            for c in CHANNELS:
                vals=delta[c][active];valid=vals[np.isfinite(vals)]
                if not len(valid):continue
                steps.append(dict(repeat=rep,role='fit' if rep<=7 else 'validation',channel=c,
                    point=int(current['point_index']),force_before=f0,force_after=f1,
                    raw_increment=raw[c][b]-raw[c][a],counts_per_n=(raw[c][b]-raw[c][a])/(f1-f0),
                    D=raw[c][b]-float(rows[b][f'ch{c}_baseline']),
                    maximum_delta_200ms=float(max(valid)),
                    floating_fraction=float(np.mean([int(r[f'ch{c}_state'])==6 for r,yes in zip(rows,active) if yes]))))
    boundaries={c:np.median(b,axis=0).tolist() for c,b in boundaries.items() if b}
    controls=[];conditions=[]
    # 已有两种实验提供低力长保持、高力短保持和释放后的对照。
    for session in ('creep_20260915_122904_016_3dc6c4ea','creep_20260915_150341_870_b1f18e03'):
        directory=DATA/'long_duration_creep_tests'/session
        manifest_path=directory/'manifest.json';sources.append(manifest_path)
        manifest=json.loads(manifest_path.read_text(encoding='utf8'))
        allrows=[];events=[]
        cols=['sensor_source_monotonic','sensor_sequence','trial_index','attempt_index','sensor_age_ms',
              'motor_speed_rpm','motor_position_counts','pressure_raw_n']
        cols += [f'ch{c}_{key}' for c in CHANNELS for key in ('normalized_raw','baseline','state','fast_k_percent')]
        for part in manifest['output_parts']:
            path=directory/part;sources.append(path)
            if Path(str(path)+'-wal').exists() and Path(str(path)+'-wal').stat().st_size:
                raise ValueError(f'未合并WAL：{path}')
            with closing(sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)) as db:
                db.row_factory=sqlite3.Row
                allrows.extend(dict(r) for r in db.execute('SELECT '+','.join(cols)+' FROM samples WHERE sensor_has_raw=1'))
                events += [json.loads(r[0]) for r in db.execute("SELECT payload_json FROM events WHERE event='trial_complete'")]
        for event in events:
            result=event['trial_result'];press=result['press_result'];release=result['release_result']
            rep=event['repeat_index'];fit=rep<= (2 if '122904' in session else 3)
            rows=[r for r in allrows if r['trial_index']==event['trial_index'] and r['attempt_index']==event.get('attempt_index',1)
                  and r['sensor_source_monotonic'] is not None and r['sensor_age_ms'] is not None and r['sensor_age_ms']<=250]
            rows.sort(key=lambda r:r['sensor_source_monotonic'])
            rows=list({r['sensor_sequence']:r for r in rows}.values())
            t=np.array([r['sensor_source_monotonic'] for r in rows])
            phases=[('hold_from_arrival',press['arrived_monotonic'],release['release_command_monotonic']-.05),
                    ('hold_1s_onward',press['arrived_monotonic']+1,release['release_command_monotonic']-.05),
                    ('hold_10s_onward',press['arrived_monotonic']+10,release['release_command_monotonic']-.05),
                    ('unloaded_10s_onward',release['release_zero_confirmed_monotonic']+10,event['controller_monotonic'])]
            conditions.append(dict(session=session,trial=event['trial_index'],target=event['high_force_n'],
                hold=event['high_hold_seconds'],repeat=rep,role='fit' if fit else 'validation'))
            for c in CHANNELS:
                raw=np.array([r[f'ch{c}_normalized_raw'] for r in rows],dtype=float)
                values=features(t,raw)
                for phase,a,b in phases:
                    # 全部200ms窗口都须落在所选控制阶段之内。
                    for i in np.flatnonzero((t>=a+.2)&(t<=b)&np.isfinite(values)):
                        r=rows[i]
                        controls.append(dict(session=session,trial=event['trial_index'],channel=c,role='fit' if fit else 'validation',
                            target=event['high_force_n'],phase=phase,time=t[i]-press['arrived_monotonic'],
                            D=raw[i]-r[f'ch{c}_baseline'],state=r[f'ch{c}_state'],delta_200ms=values[i],
                            fast_k_percent=r[f'ch{c}_fast_k_percent']))
    def band(c,D):return int(np.searchsorted(boundaries[c],D,side='right')) if c in boundaries else -1
    for row in steps+controls:row['band']=band(row['channel'],row['D'])
    grid=[]
    for c in CHANNELS:
        for b in (range(3) if boundaries else (-1,)):
            for threshold in (8,12,20,30,40,60,80,120,160,240,320,480):
                for role in ('fit','validation'):
                    pos=[r for r in steps if (r['channel'],r['band'],r['role'])==(c,b,role)]
                    neg=[r for r in controls if (r['channel'],r['band'],r['role'])==(c,b,role)
                         and r['phase']=='hold_1s_onward' and r['state']==6]
                    grid.append(dict(channel=c,band=b,threshold=threshold,role=role,steps=len(pos),
                        steps_hit=sum(r['maximum_delta_200ms']>=threshold for r in pos),hold_samples=len(neg),
                        hold_samples_hit=sum(r['delta_200ms']>=threshold for r in neg),
                        hold_trials=len(set((r['session'],r['trial']) for r in neg)),
                        hold_trials_hit=len(set((r['session'],r['trial']) for r in neg if r['delta_200ms']>=threshold))))
    output=ROOT/'analysis_outputs'/('floating_thresholds_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True)
    candidates=[]
    for c in CHANNELS:
        for target in (2,10,20):
            neg=[r for r in controls if r['channel']==c and r['target']==target
                 and r['phase']=='hold_from_arrival' and r['state']==6]
            train=[r for r in neg if r['role']=='fit'];validation=[r for r in neg if r['role']=='validation']
            if not train or not validation:continue
            max_train=max(r['delta_200ms'] for r in train)
            candidate=max(20,math.ceil(1.5*max_train/5)*5)
            candidates.append(dict(channel=c,target_n=target,train_n=len(train),validation_n=len(validation),
                train_max=max_train,validation_max=max(r['delta_200ms'] for r in validation),
                candidate_delta_200ms=candidate,
                validation_crossings=sum(r['delta_200ms']>=candidate for r in validation),
                validation_trials=len(set((r['session'],r['trial']) for r in validation)),
                status='negative_controls_only_not_validated_on_incremental_press'))
    for name,rows in (('scan_steps',steps),('control_windows',controls),('threshold_grid',grid),
                      ('candidate_noise_floors',candidates)):
        if not rows:continue
        with (output/(name+'.csv')).open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary=dict(boundaries=boundaries,conditions=conditions,steps=[],controls=[],sources=[],missing_sources=missing_sources,
                 scan_threshold_validation_available=not missing_sources)
    for path in sources:
        with path.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
        summary['sources'].append(dict(path=str(path),sha256=digest))
    for c in CHANNELS:
        for b in (range(3) if boundaries else ()):
            pos=[r for r in steps if r['channel']==c and r['band']==b]
            entry=dict(channel=c,band=b,D=describe([r['D'] for r in pos]),
                force=describe([r['force_after'] for r in pos]),sensitivity=describe([r['counts_per_n'] for r in pos]),
                delta_200ms=describe([r['maximum_delta_200ms'] for r in pos]))
            summary['steps'].append(entry);print('STEP',json.dumps(entry))
        for target in (2,10,20):
            for phase in ('hold_from_arrival','hold_1s_onward','hold_10s_onward','unloaded_10s_onward'):
                neg=[r for r in controls if r['channel']==c and r['target']==target and r['phase']==phase and r['state']==6]
                if neg:
                    entry=dict(channel=c,target=target,phase=phase,D=describe([r['D'] for r in neg]),
                               delta_200ms=describe([r['delta_200ms'] for r in neg]),
                               fast_k_percent=describe([r['fast_k_percent'] for r in neg]))
                    summary['controls'].append(entry);print('CONTROL',json.dumps(entry))
    (output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf8')
    print('OUTPUT',output)


if __name__=='__main__':main()
