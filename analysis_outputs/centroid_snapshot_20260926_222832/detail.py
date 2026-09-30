from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime

P = Path(__file__).resolve().parent
d = np.load(P/'replay.npz')
t,s,raw,base,state = (d[k] for k in ('t','signal','raw','baseline','state'))
xy = d['allxy']; loc = d['localxy']; total=s.sum(axis=1)
centroids = json.loads((P/'centroids.json').read_text(encoding='utf-8'))
cx=np.arange(20)//4+.5
diff = np.maximum(raw-base,0)
diffxy = diff @ np.stack((cx,np.arange(20)%4+.5),axis=1)/np.maximum(diff.sum(axis=1,keepdims=True),1)
def stamp(i): return datetime.fromtimestamp(float(d['epoch'][i])).isoformat(timespec='milliseconds')
def event(i):
    change=s[i]-s[i-1]
    contributions=(cx-xy[i-1,0])*change/total[i]
    channels=np.argsort(np.abs(contributions))[-4:][::-1]
    return dict(time=stamp(i),t=float(t[i]), x=xy[[i-1,i],0].tolist(),y=xy[[i-1,i],1].tolist(),
                total=total[[i-1,i]].tolist(),channels=[dict(ch=int(ch),dx=float(contributions[ch]),
                signal=s[[i-1,i],ch].tolist(),raw=raw[[i-1,i],ch].tolist(),
                baseline=base[[i-1,i],ch].tolist(),state=state[[i-1,i],ch].tolist(),
                fast_k=d['fast_k_percent'][[i-1,i],ch].tolist(),
                slow_k=d['slow_k_percent'][[i-1,i],ch].tolist()) for ch in channels])
mask=(total[1:]>5000)&(total[:-1]>5000)&d['valid'][1:]&d['valid'][:-1]
dx=np.abs(np.diff(xy[:,0])); ldx=np.abs(np.diff(loc[:,0]))
rank=np.argsort(np.where(mask,dx,-1))[-15:][::-1]+1
report=dict(baseline_ptp=np.ptp(base,axis=0).tolist(),baseline_changes=np.sum(np.diff(base,axis=0)!=0,axis=0).tolist(),
            max_abs_baseline_step=np.max(np.abs(np.diff(base,axis=0)),axis=0).tolist(),
            all20_x_step_quantiles=np.nanpercentile(dx[mask],[50,95,99,100]).tolist(),
            local_x_step_quantiles=np.nanpercentile(ldx[mask],[50,95,99,100]).tolist(),
            all20_steps_over_point2=int(np.sum(mask&(dx>.2))),local_steps_over_point2=int(np.sum(mask&(ldx>.2))),
            major_steps=[event(i) for i in rank])
zero_event = (s[:-1]>500)&(s[1:]==0)&(state[1:]==4)&(state[:-1]!=4)
zero_dx = np.sum((cx[None,:]-xy[:-1,0,None])*(s[1:]-s[:-1])*zero_event,axis=1)/np.maximum(total[1:],1)
jump = mask&(dx>.2)
report['strong_x_jumps_with_slow_back_zero'] = int(np.sum(jump&zero_event.any(axis=1)))
report['strong_x_jumps_zero_dx_over_half'] = int(np.sum(jump&(np.abs(zero_dx)>dx*.5)))
report['slow_back_zero_channel_events_above500'] = int(zero_event.sum())
report['state_floating_fraction'] = float(np.mean(state==6))
mode_switches=[]
previous=None
for rec in centroids:
    f=rec['frame']; mode=(f['filter']['centroid_algorithm'],f['smoothing_enabled'])
    if mode!=previous: mode_switches.append([rec['timestamp'],*mode]); previous=mode
report['mode_switches']=mode_switches
(P/'detail.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
print('BASE PT P',report['baseline_ptp']);print('BASE CHANGES',report['baseline_changes'])
print('STEP STATS', {k:v for k,v in report.items() if 'step' in k and k!='major_steps'})
print('ZERO STATS', {k:v for k,v in report.items() if 'zero' in k or 'floating' in k})
for e in report['major_steps'][:8]: print('STEP',json.dumps(e))

ct=np.array([r['time_ns']/1e9-d['epoch'][0] for r in centroids])
cxy=np.array([r['frame']['displayed_xy'] or [np.nan,np.nan] for r in centroids])
slow_mask=(t>=104)&(t<=109)
slow_cm=(ct>=104)&(ct<=109)
print('SLOW RANGE', 'all raw',np.ptp(xy[slow_mask],axis=0),'display',np.nanmin(cxy[slow_cm],axis=0),np.nanmax(cxy[slow_cm],axis=0),
      'raw-base',np.ptp(diffxy[slow_mask],axis=0),'local',np.ptp(d['smoothlocal'][slow_mask],axis=0))
for start_sec,end_sec in ((104,109), (75.85,76.2)):
    print('RANGE STATES',start_sec,end_sec, np.unique(state[(t>=start_sec)&(t<=end_sec)],return_counts=True))

# 用已记录的显示坐标说明慢速横摆，用同一采样回放对照两种算法。
plt.rcParams['font.family'] = 'Microsoft YaHei'
plt.rcParams['axes.unicode_minus'] = False
fig,axs=plt.subplots(2,1,figsize=(11,6.5),sharex=True)
for axis in range(2):
    axs[axis].plot(t[slow_mask]-104,xy[slow_mask,axis],color='#9299a0',lw=1,label='20通道原始质心')
    axs[axis].plot(ct[slow_cm]-104,cxy[slow_cm,axis],color='#1266a0',lw=2,label='当时实际显示（20通道＋平滑）')
    axs[axis].plot(t[slow_mask]-104,d['smoothlocal'][slow_mask,axis],color='#e97818',lw=1.4,label='2×2＋同样平滑（离线回放）')
    axs[axis].set_ylabel(('横向 X','纵向 Y')[axis]+'（格）')
    axs[axis].grid(alpha=.2)
axs[0].legend(fontsize=9,ncol=1,loc='upper left')
axs[0].set_title('慢速移动片段：原始权重变化已包含横向摆动')
axs[1].set_xlabel('从 22:28:24.786 起经过的秒数')
fig.tight_layout();fig.savefig(P/'slow_slide_comparison.png',dpi=150);plt.close(fig)
for start,end in ((1,9),(99,112)):
    fig,axs=plt.subplots(4,1,figsize=(14,11),sharex=True)
    m=(t>=start)&(t<=end)
    cm=(ct>=start)&(ct<=end)
    for axis in range(2):
        axs[axis].plot(t[m],xy[m,axis],label='All 20 raw',color='0.6',lw=1)
        axs[axis].plot(ct[cm],cxy[cm,axis],label='Actual displayed',lw=1.5)
        axs[axis].plot(t[m],d['smoothlocal'][m,axis],label='2x2 filtered replay',lw=1,alpha=.7)
        axs[axis].plot(t[m],diffxy[m,axis],label='Raw-Baseline centroid (diagnostic only)',lw=1,alpha=.7)
        axs[axis].set_ylabel(('X','Y')[axis]+' (cell)'); axs[axis].legend(ncol=2,fontsize=9)
    for col in range(5): axs[2].plot(t[m],s[m,col*4:(col+1)*4].sum(axis=1)/total[m],label=f'Column {col}')
    axs[2].set_ylabel('Column Signal fraction'); axs[2].legend(ncol=5)
    for ch in (8,9,10,11,12,13,14,15,16,17,18,19): axs[3].plot(t[m],s[m,ch],label=f'CH{ch}',lw=1)
    axs[3].set_ylabel('Signal'); axs[3].legend(ncol=6,fontsize=9)
    axs[3].set_xlabel('Seconds from 22:26:40.786 (+08:00)')
    for ax in axs: ax.grid(alpha=.25)
    fig.tight_layout();fig.savefig(P/f'detail_{start}_{end}.png',dpi=140);plt.close(fig)
for sec in np.arange(103,110,.5):
    i=int(np.argmin(abs(t-sec)))
    top=np.argsort(s[i])[-5:][::-1]
    print('SLOW',stamp(i),np.round(xy[i],3), 'COL',np.round(s[i].reshape(5,4).sum(axis=1)/total[i],3),
          [(int(ch),int(s[i,ch]),int(raw[i,ch]-base[i,ch]),int(state[i,ch])) for ch in top])
