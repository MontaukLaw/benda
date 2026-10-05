"""用冻结日志逐帧执行改动前后真实ARM算法，不访问串口或烧录接口。"""
import argparse
import collections
import gzip
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'tools'))
import test_matrix20_firmware as firmware

parser=argparse.ArgumentParser()
parser.add_argument('--old',action='store_true')
args=parser.parse_args()
if args.old:
    firmware.ELF=ROOT/'build/retrigger_20261003/before/mtb-example-psoc4-msclp-finger-INDEX_FINGER.elf'
with gzip.open(OUT/'samples.json.gz','rt') as f: rows=json.load(f)
rows=[r for r in rows if r['timestamp'][11:23]>='14:58:45.000']
hz=round((len(rows)-1)*1e9/(rows[-1]['time_ns']-rows[0]['time_ns']))
fw=firmware.Firmware()
fw.call('ChannelState_SetSampleRateHz',hz)
fw.call('initialize_baseline_runtime')
fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'],struct.pack('<H',300))
for _ in range(3*hz): fw.sample(rows[0]['frame']['baseline'])
counts=[collections.Counter() for _ in range(20)]
transitions=[[] for _ in range(20)]
previous=None
last=None
checkpoints={}
for i,r in enumerate(rows):
    period=round((r['time_ns']-last)/1000) if last is not None else round(1e6/hz)
    fw.call('ChannelState_SetFramePeriodUs',period)
    last=r['time_ns']
    states,outputs=fw.sample(r['frame']['raw'])
    for ch,s in enumerate(states):
        counts[ch][s]+=1
        if previous is not None and previous[ch] in (4,5,7,8) and s==1:
            transitions[ch].append(dict(time=r['timestamp'],source=previous[ch],raw=outputs[ch][0],baseline=outputs[ch][1],signal=outputs[ch][2]))
    previous=states
    checkpoints[r['timestamp'][11:19]]=dict(states=states,baseline=[o[1] for o in outputs],signal=[o[2] for o in outputs])
    if i%2000==0: print(i,r['timestamp'],flush=True)
report=dict(mode='old' if args.old else 'new',hz=hz,start=rows[0]['timestamp'],end=rows[-1]['timestamp'],count=len(rows),
    assumptions='从14:58:45日志基线初始化IDLE并预填窗口；记录前内部状态、峰值和锁定FPS未知，不能把离线重放当作现场误触发真值。实际周期采用主机收帧间隔。',
    transition_counts=[len(t) for t in transitions],transitions=transitions,state_counts=counts,final=checkpoints[r['timestamp'][11:19]],checkpoints=checkpoints)
(OUT/('replay_old.json' if args.old else 'replay_new.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k not in ('assumptions','transitions','state_counts','checkpoints')},ensure_ascii=False),flush=True)
