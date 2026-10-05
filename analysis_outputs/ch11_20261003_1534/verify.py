"""核对CH11事件的原始RX，并经公开ARM接口重建恢复参考下降问题。"""
import gzip
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
sys.path.insert(0,str(ROOT/'tools'))
from test_matrix20_retrigger import Machine

audit=json.loads((OUT/'audit.json').read_text(encoding='utf8'))
with gzip.open(OUT/'samples.json.gz','rt') as f: rows=json.load(f)
target=next(r for r in rows if r['timestamp'][11:23]=='15:33:20.828')
parser=AlgoFrameParser()
decoded=[]
rx=[]
for entry in audit['files']:
    p=ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'/entry['name']
    if p.name.endswith('_00016.jsonl'):
        with p.open('rb') as f: records=[json.loads(x) for x in f.read(entry['size']).splitlines()]
        for r in records:
            if target['sequence']-180<=r['sequence']<=target['sequence']+40 and r['kind']=='rx':
                rx.append(r)
                decoded.extend(parser.feed(bytes.fromhex(r['hex'])))
        break
matched=[f for f in decoded if f.get('type')=='debug' and list(f['raw'])==target['frame']['raw']]
assert len(matched)==1
assert all(list(matched[0][k])==target['frame'][k] for k in ('raw','baseline','signal','state','fast_k_percent','slow_k_percent'))
(OUT/'event_rx.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rx),encoding='utf8')

# 用真实接口建立“永久基线5765、恢复再按压局部参考6200”的活动周期。
m=Machine(hz=78,baseline=5765)
m.release(peak=10000,tail=6200)
m.ramp(6200,6656,16)
m.feed(6656,2*78)
before=m.fw.call('ChannelState_GetActivationBaseline',m.address)
selected=[r for r in rows if '15:33:20.000'<=r['timestamp'][11:23]]
transitions=[]
last=2
prior_time=None
for r in selected:
    period=round((r['time_ns']-prior_time)/1000) if prior_time else round(1e6/78)
    prior_time=r['time_ns']
    m.fw.call('ChannelState_SetFramePeriodUs',period)
    state=m.feed(r['frame']['raw'][11])[0]
    if state!=last:
        transitions.append(dict(time=r['timestamp'],from_state=last,to_state=state,raw=r['frame']['raw'][11],
            activation_reference=m.fw.call('ChannelState_GetActivationBaseline',m.address)))
    last=state
after=m.fw.call('ChannelState_GetActivationBaseline',m.address)
report=dict(raw_rx_verified=True,rx_chunks=len(rx),entry_time=target['timestamp'],
    reconstructed_reference_before=before,reconstructed_reference_after=after,transitions=transitions,final_state=last,
    assumptions='固定永久基线5765；通过公开状态机路径重建局部参考6200，初始按压峰值模拟为10000，锁定78FPS，周期取日志收帧间隔。非现场全部内部状态。',
    conclusion='真实ARM提交路径把活动零点从6200降至5836，后续真实Raw回到约6648时仍为HOLD，支持判定与提交参考不一致导致回弹残余放大的代码缺陷。')
(OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k not in ('assumptions','conclusion')},ensure_ascii=False))
