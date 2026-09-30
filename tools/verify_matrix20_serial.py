"""COM口实测20通道V21；--apply-test临时改三参数后恢复；--dfu-image执行一次DFU。"""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import sys
import time
import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
from config import COMMAND_TYPE_GET_DEVICE_INFO, COMMAND_TYPE_GET_CAPSENSE_CHANNEL_CONFIG
from config import COMMAND_TYPE_GET_CAPSENSE_MATRIX_CONFIG, COMMAND_TYPE_SET_CAPSENSE_MATRIX_CONFIG


def command(port, kind, payload=b''):
    port.write(struct.pack('<4sBBBBH',b'DATA',21,kind,0,0,len(payload))+payload)


def collect(port, seconds):
    parser=AlgoFrameParser()
    frames=[]
    start=time.monotonic()
    for kind in (COMMAND_TYPE_GET_DEVICE_INFO,COMMAND_TYPE_GET_CAPSENSE_CHANNEL_CONFIG,COMMAND_TYPE_GET_CAPSENSE_MATRIX_CONFIG):
        command(port,kind)
    while time.monotonic()-start < seconds:
        frames.extend(parser.feed(port.read(port.in_waiting or 1)))
    elapsed=time.monotonic()-start
    debug=[frame for frame in frames if frame['type']=='debug']
    report=dict(elapsed_s=elapsed,counts=dict(Counter(f['type'] for f in frames)),debug_fps=len(debug)/elapsed,
                last_debug=debug[-1] if debug else None,
                diagnostics=[f for f in frames if f['type'] not in ('debug','temperature')])
    print('RX',report['counts'],'FPS',round(report['debug_fps'],2),flush=True)
    return report


def config(report):
    values=[f for f in report['diagnostics'] if f['type']=='capsense_channel_config_status']
    assert values and values[-1]['read_ok'], 'No successful channel configuration response'
    channels=values[-1]['channels']
    assert len(channels)==20
    for i, channel in enumerate(channels):
        assert channel['channel']==i
        for key in ('nsub','tx_clk_div','ref_cdac'):
            assert channel['configured_'+key]==channel['hardware_'+key], (i,key,channel)
    groups={(c['hardware_nsub'],c['hardware_tx_clk_div'],c['hardware_ref_cdac']) for c in channels}
    assert len(groups)==1,groups
    return next(iter(groups))


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--port',default='COM4')
    parser.add_argument('--seconds',type=float,default=5)
    parser.add_argument('--apply-test',action='store_true')
    parser.add_argument('--dfu-image',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'tmp/matrix20_hardware_validation.json')
    args=parser.parse_args()
    report={}
    if args.dfu_image:
        from dfu_upgrade import DfuSerialClient,DfuProgrammer,parse_cyacd2
        image=parse_cyacd2(args.dfu_image)
        client=DfuSerialClient(args.port,1000000)
        progress=[-1]
        def on_progress(current,total):
            percent=current*100//total
            if percent//20!=progress[0]:
                print('DFU',percent,'%',flush=True)
                progress[0]=percent//20
        try:
            DfuProgrammer(client,lambda message: None,on_progress).program(image)
            report['dfu_success']=True
        finally:
            client.close()
        time.sleep(3)
    with serial.Serial(args.port,1000000,timeout=.05) as port:
        report['initial']=collect(port,args.seconds)
        assert report['initial']['last_debug'] and report['initial']['last_debug']['channel_count']==20
        original=config(report['initial'])
        print('All 20 channels:',original,flush=True)
        if args.apply_test:
            requested=(400,120,15) if original != (400,120,15) else (300,100,12)
            try:
                command(port,COMMAND_TYPE_SET_CAPSENSE_MATRIX_CONFIG,struct.pack('<HHB',*requested))
                time.sleep(1)
                port.reset_input_buffer()
                report['changed']=collect(port,3)
                assert config(report['changed'])==requested
            finally:
                command(port,COMMAND_TYPE_SET_CAPSENSE_MATRIX_CONFIG,struct.pack('<HHB',*original))
                time.sleep(1)
                port.reset_input_buffer()
                report['restored']=collect(port,3)
                assert config(report['restored'])==original
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print('PASS',args.output,flush=True)


if __name__=='__main__':
    main()
