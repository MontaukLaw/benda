"""核对原始RX解析、松手参考漂移，并绘制实际轨迹。"""
from pathlib import Path
import json,sys,math
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
audit=json.loads((OUT/'audit.json').read_text(encoding='utf-8'))
from datetime import datetime
last_info=audit['files'][-1]
with (ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'/last_info['name']).open('rb') as f:
    f.seek(max(0,last_info['read_bytes']-100000))
    tail=f.read(last_info['read_bytes']-f.tell()).splitlines()
records=[]
for line in tail:
    try:records.append(json.loads(line))
    except ValueError:pass
end=max(r['time_ns'] for r in records);start=end-300_000_000_000
audit.update(start_ns=start,end_ns=end)
(OUT/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
refs=[];parser=AlgoFrameParser();decoded=Counter();mismatches=[];pending=[];first=None
for info in audit['files']:
    p=ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'/info['name']
    if not p.exists():continue
    with p.open('rb') as f:
        used=0
        while used<info['read_bytes']:
            line=f.readline(info['read_bytes']-used);used+=len(line)
            if not line:break
            if b'"kind":"touchpad"' in line and b'"reference":[' in line:
                r=json.loads(line);refs.append(dict(timestamp=r['timestamp'],reference=r['frame']['reference'],noise=r['frame']['noise'],time_ns=r['time_ns']))
            if info['name']<audit['files'][-6]['name']:continue
            r=json.loads(line);t=r['time_ns']
            if t>end:continue
            if r['kind']=='rx':
                for frame in parser.feed(bytes.fromhex(r['hex'])):
                    pending.append(frame)
            elif r['kind']=='frame':
                if pending:
                    parsed=pending.pop(0)
                    canonical=lambda x:json.loads(json.dumps(x,sort_keys=True))
                    if t>=start:
                        decoded[parsed['type']]+=1
                        if canonical(parsed)!=canonical(r['frame']):mismatches.append(r['sequence'])
                elif t>=start:mismatches.append(dict(unmatched=r['sequence']))
frames=[json.loads(line) for line in (OUT/'sensor_frames.jsonl').open(encoding='utf-8')]
touch=[json.loads(line) for line in (OUT/'touchpad.jsonl').open(encoding='utf-8')]
contact_sequence={r['frame']['source_sample_sequence'] for r in touch if r['frame'].get('contact')}
last_ref=max((r for r in refs if r['time_ns']<=end),key=lambda r:r['time_ns']) if refs else None
print('REFERENCES',json.dumps(refs,ensure_ascii=False))
print('RX_REPLAY',dict(decoded),'mismatch',mismatches[:8],'pending',len(pending))
extra=dict(references=refs,rx_replay=dict(decoded),rx_frame_mismatches=mismatches,pending_frames=len(pending))
if last_ref:
    idle=[r for r in frames if max(r['frame']['signal'])==0]
    raw=np.array([r['frame']['raw'] for r in idle]);diff=raw-np.array(last_ref['reference'])
    extra['zero_signal_reference_difference']=dict(count=len(idle),median=np.median(diff,axis=0).tolist(),
                p95=np.percentile(diff,95,axis=0).tolist(),max=diff.max(axis=0).tolist(),min=diff.min(axis=0).tolist())
    print('ZERO_SIGNAL_RAW_MINUS_REFERENCE',json.dumps(extra['zero_signal_reference_difference']))

groups=[];group=[]
for r in touch:
    if r['frame'].get('contact'):
        if group and r['frame']['source_sample_sequence']!=group[-1]['frame']['source_sample_sequence']+1:
            groups.append(group);group=[]
        group.append(r)
    elif group:groups.append(group);group=[]
if group:groups.append(group)
selected=[8,9,10,29,54,78]
plt.rcParams['font.family']='Microsoft YaHei'
fig,axes=plt.subplots(2,3,figsize=(12,8),layout='constrained')
for ax,idx in zip(axes.flat,selected):
    g=groups[idx];raw=np.array([r['frame']['raw_xy'] for r in g]);smooth=np.array([r['frame']['filtered_xy'] for r in g])
    ax.plot(raw[:,0],raw[:,1],color='#9da7b3',lw=1,alpha=.75,label='滤波前质心')
    ax.plot(smooth[:,0],smooth[:,1],color='#1976d2',lw=1.8,label='One Euro 后')
    ax.scatter(*smooth[0],c='#2e9c5a',s=45,zorder=5,label='起点')
    ax.scatter(*smooth[-1],c='#e87d20',s=35,zorder=5,label='终点')
    ax.set_xlim(0,5);ax.set_ylim(4,0);ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.set_xlabel('X（传感器格）');ax.set_ylabel('Y（传感器格）')
    ext=np.ptp(smooth,axis=0)
    ax.set_title(f"{datetime.fromtimestamp(g[0]['time_ns']/1e9).strftime('%H:%M:%S')}\nX跨度 {ext[0]:.2f} 格 / Y跨度 {ext[1]:.2f} 格",fontsize=11)
handles,labels=axes.flat[0].get_legend_handles_labels()
fig.legend(handles,labels,loc='outside lower center',ncol=4,frameon=False)
fig.suptitle('拇指实际轨迹：平滑以后，伴随的另一轴运动仍然存在\n连续接触段可能包含往返；未标注每段的主观目标方向',fontsize=15)
fig.savefig(OUT/'thumb_paths.png',dpi=160)
(OUT/'detail.json').write_text(json.dumps(extra,ensure_ascii=False,indent=2),encoding='utf-8')
