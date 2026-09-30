"""从快照内已确认的重启零点回放真实固件，补接一分钟原始采样。"""
import argparse
import csv
import gzip
import json
from pathlib import Path
import struct
import sys

P=Path(__file__).resolve().parent
ROOT=P.parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import test_matrix20_firmware as target
parser=argparse.ArgumentParser()
parser.add_argument('--elf',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--hz',type=int,default=95)
parser.add_argument('--assert-released',action='store_true')
args=parser.parse_args()
cache=P/'release_replay_input.json.gz'
if not cache.exists():
    samples=[]
    with (P/'recent_5min.csv').open(encoding='utf-8-sig',newline='') as source:
        for r in csv.DictReader(source):
            if r['type']=='debug' and r['timestamp'][11:23]>='16:08:07.256':
                samples.append(dict(time=r['timestamp'],raw=[int(r[f'raw_{c}']) for c in range(20)],
                    state=[int(r[f'state_{c}']) for c in range(20)],signal=[int(r[f'signal_{c}']) for c in range(20)]))
    for n in (51,52):
        path=ROOT/'PY_SRIPT/algo_monitor/recordings/rolling'/f'roll_20260930_151101_929680_5bc70e43_{n:05d}.jsonl'
        for line in path.open(encoding='utf-8'):
            r=json.loads(line)
            if r['kind']=='frame' and r['frame']['type']=='debug' and '16:10:28.321'<=r['timestamp'][11:23]<='16:11:28.320':
                f=r['frame'];samples.append(dict(time=r['timestamp'],raw=f['raw'],state=f['state'],signal=f['signal']))
    with gzip.open(cache,'wt',encoding='utf-8') as dest: json.dump(samples,dest)
with gzip.open(cache,'rt',encoding='utf-8') as source: samples=json.load(source)
target.ELF=args.elf
fw=target.Firmware()
fw.call('ChannelState_SetSampleRateHz',args.hz)
fw.call('initialize_baseline_runtime')
fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'],struct.pack('<H',300))
for _ in range(3*args.hz): fw.sample(samples[0]['raw'])
checkpoints={}; equal=total=0; tail=[]
for i,r in enumerate(samples):
    states, outputs=fw.sample(r['raw'])
    equal+=sum(a==b for a,b in zip(states,r['state']));total+=20
    result=dict(time=r['time'],state=states,signal=[o[2] for o in outputs],baseline=[o[1] for o in outputs],
                recorded_state=r['state'],recorded_signal=r['signal'])
    checkpoints[r['time'][11:19]]=result
    if r['time'][11:23]>='16:10:27.000': tail.append(result)
    if i%2000==0: print(i,r['time'],'nonzero',[c for c,o in enumerate(outputs) if o[2]],flush=True)
report=dict(elf=str(args.elf),hz=args.hz,samples=len(samples),state_match=equal/total,final=result,checkpoints=checkpoints,tail=tail)
args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
print('FINAL',json.dumps(result),'state_match',equal/total,flush=True)
if args.assert_released:
    assert result['state'] == (0,)*20, result
    assert result['signal'] == [0]*20, result
