"""整理已编译HEX/CYACD2并验证传输产物一致性，不执行任何烧录。"""
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/ch11_rebound_20261003/firmware'
sys.path.insert(0,str(ROOT/'tools'))
from package_iap import build_image, crc32c, APP_SIGNATURE_ADDRESS
OUT.mkdir(parents=True,exist_ok=True)
files=[]
checks={}
for transport in ('UART','I2C'):
    source=ROOT/f'build/INDEX_FINGER/{transport}/APP_CY8CPROTO-041TP/Debug'
    prefix='mtb-example-psoc4-msclp-finger-INDEX_FINGER'
    target=OUT/transport
    target.mkdir(exist_ok=True)
    paths={}
    for suffix in ('.hex','_crc.cyacd2','.iap.bin'):
        p=target/('INDEX_FINGER_20CH_ch11_rebound_20261003'+suffix)
        shutil.copyfile(source/(prefix+suffix),p)
        paths[suffix]=p
        files.append(dict(path=str(p.relative_to(ROOT)),size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    binary=paths['.iap.bin'].read_bytes()
    base=0x3600
    memory={}
    extended=0
    for line in paths['.hex'].read_text().splitlines():
        data=bytes.fromhex(line[1:])
        assert sum(data)%256==0 and len(data)==data[0]+5
        size=data[0]
        address=int.from_bytes(data[1:3],'big')
        kind=data[3]
        payload=data[4:4+size]
        if kind==4: extended=int.from_bytes(payload,'big')<<16
        elif kind==2: extended=int.from_bytes(payload,'big')<<4
        elif kind==0:
            for i,value in enumerate(payload):
                absolute=extended+address+i
                assert absolute not in memory
                memory[absolute]=value
                assert not 0x1FD00<=absolute<0x20000
                if base<=absolute<APP_SIGNATURE_ADDRESS:
                    assert value==binary[absolute-base]
    assert build_image(memory)==binary
    rows=[]
    cy_data=bytearray()
    for line in paths['_crc.cyacd2'].read_text().splitlines():
        if not line.startswith(':'): continue
        data=bytes.fromhex(line[1:])
        address=int.from_bytes(data[:4],'little')
        payload=data[4:]
        assert len(payload)==256 and address%256==0
        assert base<=address and address+len(payload)<=0x1FD00
        for i,value in enumerate(payload):
            absolute=address+i
            if absolute in memory and absolute<APP_SIGNATURE_ADDRESS:
                assert value==memory[absolute]
        cy_data.extend(payload)
        rows.append(address)
    assert rows==list(range(base,base+len(binary),256))
    assert crc32c(cy_data[:-4])==int.from_bytes(cy_data[-4:],'little')
    checks[transport]=dict(hex_checksum_valid=True,iap_generated_from_hex=True,cyacd2_linked_bytes_match_hex=True,
        cyacd2_crc_valid=True,iap_crc_valid=True,
        rows=len(rows),start=hex(rows[0]),end_exclusive=hex(rows[-1]+256),dfu_protected_region_untouched=True)
sources={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in
    ('channel_state_machine.c','channel_state_machine.h','device_variant.h','sensor_algorithm.c','sensor_algorithm.h','baseline_tracker.c','main.c')}
manifest=dict(variant='INDEX_FINGER',channels=20,configuration='Debug',floating_enabled=False,
    recovery_retrigger=dict(delta_min=200,previous_peak_percent=5,delta_max=400,fast_counts_per_second_min=500,confirm_microseconds=80000),
    rebound_reference=dict(shared_detection_and_commit_reference=True,stable_preload_platform_preserved=True),files=files,checks=checks,sources=sources,flashed=False)
(OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(checks))
