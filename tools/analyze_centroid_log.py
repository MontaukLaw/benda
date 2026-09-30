"""固定最新滚动日志的读取范围，回放Signal质心并保留完整诊断采样。"""
from collections import Counter
from pathlib import Path
import gzip
import json
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))


def capture(output):
    logs = sorted((ROOT/'PY_SRIPT/algo_monitor/recordings/rolling').glob('roll_*.jsonl'))
    session = logs[-1].stem.rsplit('_', 1)[0]
    paths = [p for p in logs if p.name.startswith(session + '_')]
    samples, events, manifests = [], [], []
    kinds, types, ports = Counter(), Counter(), Counter()
    previous = None
    gaps, malformed = [], 0
    for path in paths:
        size = path.stat().st_size
        with path.open('rb') as source:
            data = source.read(size)
        manifests.append(dict(file=path.name, bytes=len(data), partial_tail=not data.endswith(b'\n')))
        for line in data.splitlines(keepends=True):
            if not line.endswith(b'\n'):
                continue
            try:
                record = json.loads(line)
            except ValueError:
                malformed += 1
                continue
            seq = record.get('sequence')
            if seq is not None:
                if previous is not None and seq != previous + 1:
                    gaps.append([previous, seq])
                previous = seq
            kinds[record['kind']] += 1
            ports[record.get('port')] += 1
            if record['kind'] == 'frame':
                types[record['frame']['type']] += 1
                if record['frame']['type'] in ('debug', 'signal'):
                    samples.append(record)
                else:
                    events.append(record)
            elif record['kind'] not in ('rx',):
                events.append(record)
    output.mkdir(parents=True, exist_ok=True)
    with gzip.open(output/'samples.json.gz', 'wt', encoding='utf-8') as target:
        json.dump(samples, target)
    (output/'events.json').write_text(json.dumps(events, ensure_ascii=False, indent=2), encoding='utf-8')
    audit = dict(session=session, files=manifests, kinds=dict(kinds), types=dict(types),
                 ports=dict(ports), sequence_gaps=gaps, malformed=malformed,
                 first=samples[0]['timestamp'], last=samples[-1]['timestamp'])
    (output/'audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding='utf-8')
    return samples, audit


def analyze(samples):
    s = np.array([r['frame']['signal'] for r in samples], dtype=float)
    t = np.array([r['time_ns'] for r in samples], dtype=float) / 1e9
    weights = np.maximum(s, 0)
    totals = weights.sum(axis=1)
    valid = totals > 0
    xy = np.full((len(s), 2), np.nan)
    xy[valid, 0] = weights[valid] @ (np.arange(20)//4+.5) / totals[valid]
    xy[valid, 1] = weights[valid] @ (np.arange(20)%4+.5) / totals[valid]
    steps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    def quantiles(values):
        return np.percentile(values, [50, 90, 95, 99, 100]).round(4).tolist() if len(values) else []
    report = dict(samples=len(s), fps=(len(s)-1)/(t[-1]-t[0]),
                  sample_interval_quantiles=quantiles(np.diff(t)), interruptions_over_500ms=int((np.diff(t)>.5).sum()),
                  zero_signal_frames=int((~valid).sum()),
                  total_signal_quantiles=quantiles(totals[valid]),
                  step_quantiles=quantiles(steps[np.isfinite(steps)]), regimes=[])
    for low, high in ((0,100),(100,300),(300,1000),(1000,1000000)):
        mask=(totals[1:]>=low)&(totals[1:]<high)&np.isfinite(steps)
        report['regimes'].append(dict(total_range=[low,high],pairs=int(mask.sum()),
                                     steps=quantiles(steps[mask]), jumps_over_half_cell=int((steps[mask]>.5).sum())))
    largest=np.argsort(np.nan_to_num(steps,nan=-1))[-10:][::-1]
    report['largest_jumps']=[]
    for i in largest:
        before,after=samples[i],samples[i+1]
        changed=np.flatnonzero(s[i] != s[i+1])
        report['largest_jumps'].append(dict(time=after['timestamp'],step=float(steps[i]),
            total_before=float(totals[i]), total_after=float(totals[i+1]),
            xy_before=xy[i].tolist(),xy_after=xy[i+1].tolist(),
            channels=[dict(ch=int(ch),signal=[s[i,ch],s[i+1,ch]],
                           raw=[before['frame'].get('raw',[None]*20)[ch],after['frame'].get('raw',[None]*20)[ch]],
                           base=[before['frame'].get('baseline',[None]*20)[ch],after['frame'].get('baseline',[None]*20)[ch]],
                           state=[before['frame'].get('state',[None]*20)[ch],after['frame'].get('state',[None]*20)[ch]]) for ch in changed]))
    return report


def replay(samples, output):
    from centroid_filter import CentroidFilter
    filt=CentroidFilter()
    start=samples[0]['time_ns']
    times=np.array([(r['time_ns']-start)/1e9 for r in samples])
    raw, filtered=[],[]
    for record,t in zip(samples,times):
        point=filt.update(record['frame']['signal'],t)
        raw.append(filt.raw_xy if filt.raw_xy is not None else (np.nan,np.nan))
        filtered.append(point if point is not None else (np.nan,np.nan))
    raw,filtered=np.array(raw),np.array(filtered)
    result=dict(parameters=filt.parameters(),note='按串口采样时刻回放，不包含旧版GUI显示时刻；步长降低不等于定位精度提高。')
    for name,coords in (('raw',raw),('filtered',filtered)):
        steps=np.linalg.norm(np.diff(coords,axis=0),axis=1)
        steps=steps[np.isfinite(steps)]
        result[name]=dict(step_p95=float(np.percentile(steps,95)),step_p99=float(np.percentile(steps,99)),
                          step_max=float(steps.max()),jumps_over_half_cell=int((steps>.5).sum()))
    synthetic=CentroidFilter()
    a=[0]*20; a[0]=100
    b=[0]*20; b[19]=100
    synthetic.update(a,0)
    for i in range(1,101):
        point=synthetic.update(b,i/100)
        if point[0]>=.5+4*.9 and point[1]>=.5+3*.9:
            result['synthetic_corner_step_90_percent_seconds']=i/100
            break
    (output/'replay.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    for axis,label in ((0,'X (cells)'),(1,'Y (cells)')):
        axes[axis].plot(times,raw[:,axis],color='#999999',lw=1,label='Raw centroid')
        axes[axis].plot(times,filtered[:,axis],color='#e83232',lw=1.5,label='One Euro')
        axes[axis].set_ylabel(label)
        axes[axis].legend(loc='upper left')
    axes[2].plot(times,[sum(r['frame']['signal']) for r in samples],lw=1)
    axes[2].set_yscale('symlog',linthresh=100)
    axes[2].set_ylabel('Total Signal (counts)')
    axes[2].set_xlabel('Seconds since '+samples[0]['timestamp'])
    for ax in axes:
        ax.grid(alpha=.2)
        ax.set_xlim(165,176)
    fig.suptitle('COM16 recorded Signal replay: raw vs independent X/Y smoothing')
    fig.tight_layout()
    fig.savefig(output/'centroid_replay.png',dpi=140)
    plt.close(fig)
    return result


if __name__ == '__main__':
    output = ROOT/'analysis_outputs/centroid_20260922'
    if '--replay' in sys.argv:
        with gzip.open(output/'samples.json.gz','rt',encoding='utf-8') as source:
            samples=json.load(source)
        print(json.dumps(replay(samples,output),ensure_ascii=False,indent=2))
    else:
        samples,audit=capture(output)
        report=analyze(samples)
        (output/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(audit=audit,analysis=report),ensure_ascii=False,indent=2))
