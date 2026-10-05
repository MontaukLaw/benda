"""经完整SensorAlgorithm_Process重放CH11已核对的波形，对比原版和修复版。"""
import gzip
import json
from pathlib import Path
import struct
import sys
import argparse

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'tools'))
import test_matrix20_firmware as firmware

parser=argparse.ArgumentParser()
parser.add_argument('--full',action='store_true')
args=parser.parse_args()

with gzip.open(ROOT/'analysis_outputs/ch11_20261003_1534/samples.json.gz','rt') as f: rows=json.load(f)
selected=[r for r in rows if r['timestamp'][11:23]>=('15:29:16.000' if args.full else '15:33:20.000')]
new_elf=firmware.ELF
report={}
for mode in ('old','new'):
    firmware.ELF=new_elf if mode=='new' else ROOT/'build/ch11_rebound_20261003/before/mtb-example-psoc4-msclp-finger-INDEX_FINGER.elf'
    fw=firmware.Firmware()
    fw.call('ChannelState_SetSampleRateHz',78)
    fw.call('initialize_baseline_runtime')
    fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'],struct.pack('<H',300))
    base=selected[0]['frame']['baseline'].copy()
    def feed(raw,count=1):
        values=base.copy()
        values[11]=raw
        for _ in range(count): states,outputs=fw.sample(values)
        return states,outputs
    def ramp(start,end,count):
        for i in range(count): feed(round(start+(end-start)*(i+1)/count))
    for _ in range(3*78): fw.sample(base)
    if not args.full:
        ramp(5765,10000,16)
        feed(10000,78)
        feed(6200,16)
        ramp(6200,6656,16)
        states,outputs=feed(6656,2*78)
        assert states[11]==2
    else:
        states,outputs=fw.sample(base)
    before=dict(state=states[11],baseline=outputs[11][1],signal=outputs[11][2])
    previous=states[11]
    last_time=None
    transitions=[]
    first_idle=None
    peak_signal=0
    for r in selected:
        period=round((r['time_ns']-last_time)/1000) if last_time else round(1e6/78)
        last_time=r['time_ns']
        fw.call('ChannelState_SetFramePeriodUs',period)
        states,outputs=fw.sample(r['frame']['raw'])
        state=states[11]
        if state!=previous:
            transitions.append(dict(time=r['timestamp'],from_state=previous,to_state=state,raw=outputs[11][0],baseline=outputs[11][1],signal=outputs[11][2]))
        if first_idle is None and state==0:
            first_idle=r['timestamp']
        previous=state
        if r['timestamp'][11:23]>='15:33:20.700': peak_signal=max(peak_signal,outputs[11][2])
    final=dict(raw=outputs[11][0],baseline=outputs[11][1],signal=outputs[11][2],state=states[11])
    report[mode]=dict(initial=before,transitions=transitions,first_idle=first_idle,post_valley_peak_signal=peak_signal,final=final)
report['assumptions']=('从15:29:16重连首帧记录基线初始化IDLE并预填窗口，完整重放随后20路Raw；历史内部状态及锁定FPS无法读取。' if args.full else '固定真实15:33:20～15:33:57的20路Raw；起始CH11活动零点通过模拟按压/松手/再按压重建为6200，永久基线5765。历史内部峰值为模拟值。')+'锁定78FPS，按实际收帧周期重放。不能替代实机接触真值和扫描测试。'
if not args.full:
    assert report['old']['final']['state']==2 and report['old']['final']['signal']>700
    assert report['new']['final']['state']==0 and report['new']['final']['signal']==0
    assert not any(t['from_state'] in (4,5,7,8) and t['to_state']==1 for t in report['new']['transitions'])
(OUT/('replay_full.json' if args.full else 'replay.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({mode:report[mode] for mode in ('old','new')}),flush=True)
