"""经公开状态机接口重建松手起点，重放CH11/CH16真实Raw波形；不改写内部状态。"""
import gzip
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'tools'))
import test_matrix20_firmware as firmware
from test_matrix20_retrigger import Machine

with gzip.open(OUT/'samples.json.gz','rt') as f: rows=json.load(f)
rows=[r for r in rows if '14:59:03.000' <= r['timestamp'][11:23] <= '14:59:10.000']
new_elf=firmware.ELF
report={}
for mode in ('old','new'):
    firmware.ELF=new_elf if mode=='new' else ROOT/'build/retrigger_20261003/before/mtb-example-psoc4-msclp-finger-INDEX_FINGER.elf'
    channels={}
    for ch,base,peak,tail in ((11,7326,8500,6381),(16,8165,30000,19865)):
        m=Machine(hz=74,baseline=base)
        m.release(peak=peak,tail=tail)
        transitions=[]
        last_state=4
        last_time=None
        for r in rows:
            period=round((r['time_ns']-last_time)/1000) if last_time else round(1e6/74)
            last_time=r['time_ns']
            m.fw.call('ChannelState_SetFramePeriodUs',period)
            state=m.feed(r['frame']['raw'][ch])[0]
            if state==1 and last_state in (4,5,7,8):
                transitions.append(dict(time=r['timestamp'],raw=r['frame']['raw'][ch],source=last_state))
            last_state=state
        channels[ch]=dict(transitions=transitions,final_state=last_state,initial_baseline=base,initial_peak=peak,initial_tail=tail)
    report[mode]=channels
report['assumptions']='永久基线来自日志；释放前峰值为重建恢复状态采用的模拟值，历史参考/峰值未知。固定基线，仅验证状态判定，不能替代现场全链路。'
(OUT/'channel_replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(report,ensure_ascii=False))
