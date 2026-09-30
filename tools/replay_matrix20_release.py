"""对固定日志运行真实ARM算法；不打开串口，不执行烧录。"""
import argparse
import json
import struct
import gzip
from pathlib import Path
import test_matrix20_firmware as firmware

parser = argparse.ArgumentParser()
parser.add_argument('--elf', type=Path)
parser.add_argument('--hz', type=int, default=115)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--assert-released', action='store_true')
parser.add_argument('--input', type=Path, default=firmware.ROOT/'analysis_outputs/floating_20260922/full_debug_snapshot.json.gz')
parser.add_argument('--start', default='20:29:01.000')
args = parser.parse_args()
if args.elf:
    firmware.ELF = args.elf
with gzip.open(args.input,'rt',encoding='utf-8') as source:
    samples = json.load(source)
# 三参数切换后的已释放区间开始，避免把两种量程的Raw直接拼进同一基线。
samples = [r for r in samples if r['timestamp'][11:23] >= args.start]
fw = firmware.Firmware()
from elftools.elf.elffile import ELFFile
import io
elf = ELFFile(io.BytesIO(firmware.ELF.read_bytes()))
machine_size = next(sym['st_size']//20 for sym in elf.get_section_by_name('.symtab').iter_symbols() if sym.name=='channel_state_machines')
fw.call('ChannelState_SetSampleRateHz', args.hz)
fw.call('initialize_baseline_runtime')
fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H', 300))
for _ in range(3*args.hz):
    fw.sample(samples[0]['frame']['raw'])
checkpoints = {}
for i,r in enumerate(samples):
    states, outputs = fw.sample(r['frame']['raw'])
    if i % 1000 == 0:
        print(i, r['timestamp'], 'floating', [ch for ch,s in enumerate(states) if s==6], flush=True)
    second = r['timestamp'][11:19]
    checkpoints[second] = dict(state=states, baseline=[o[1] for o in outputs], signal=[o[2] for o in outputs],
        references=[struct.unpack('<4H',fw.uc.mem_read(fw.symbols['channel_state_machines']+ch*machine_size+452,8)) for ch in range(20)])
report = dict(hz=args.hz, samples=len(samples), final=checkpoints[second], checkpoints=checkpoints)
args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
print('FINAL', json.dumps(report['final']), flush=True)
if args.assert_released:
    assert report['final']['state'] == (0,)*20, report['final']
    assert report['final']['signal'] == [0]*20, report['final']
