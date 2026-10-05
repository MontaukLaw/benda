"""固定最新日志切档后区间，比较新旧真实ARM固件，不访问串口。"""
import argparse
import collections
import gzip
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
import test_matrix20_firmware as firmware

parser = argparse.ArgumentParser()
parser.add_argument('--old',action='store_true')
args=parser.parse_args()
OUT=Path(__file__).resolve().parent
with gzip.open(OUT/'samples.json.gz','rt') as f:
    rows=json.load(f)
rows=[r for r in rows if r['timestamp'][11:23]>='13:58:36.000']
if args.old:
    firmware.ELF=ROOT/'build/recovery_20261003/before/mtb-example-psoc4-msclp-finger-INDEX_FINGER.elf'
fw=firmware.Firmware()
hz=round((len(rows)-1)*1e9/(rows[-1]['time_ns']-rows[0]['time_ns']))
fw.call('ChannelState_SetSampleRateHz',hz)
fw.call('initialize_baseline_runtime')
base=rows[0]['frame']['baseline']
fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'],struct.pack('<H',300))
for _ in range(3*hz): fw.sample(base)
counts=[collections.Counter() for _ in range(20)]
checkpoints={}
last=None
for i,r in enumerate(rows):
    if not args.old:
        period=round((r['time_ns']-last)/1000) if last is not None else round(1e6/hz)
        fw.call('ChannelState_SetFramePeriodUs',period)
    last=r['time_ns']
    states,outputs=fw.sample(r['frame']['raw'])
    if r['timestamp'][11:19]>='14:02:30':
        for ch,state in enumerate(states): counts[ch][state]+=1
    checkpoints[r['timestamp'][11:19]]=dict(states=states,baseline=[o[1] for o in outputs],signal=[o[2] for o in outputs])
    if i%4000==0: print(i,r['timestamp'],states,flush=True)
report=dict(mode='old' if args.old else 'new',hz=hz,count=len(rows),start=rows[0]['timestamp'],end=rows[-1]['timestamp'],
    assumptions='使用13:58:36日志基线初始化为IDLE，重建窗口；不能恢复记录开始前的完整状态、微秒周期或硬件扫描。新固件逐帧周期采用主机收到帧的间隔。',
    final=checkpoints[r['timestamp'][11:19]],tail_state_counts=counts,checkpoints=checkpoints)
(OUT/('replay_old.json' if args.old else 'replay_new.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report['final']),flush=True)
