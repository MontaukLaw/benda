"""核对最新滚动会话、测试包记录、RX解析及增益后的低通输入。"""
from collections import Counter,deque
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
OUT=Path(__file__).parent
results={mode:json.loads((OUT/(mode+'.json')).read_text(encoding='utf-8')) for mode in ('host_continuous','host_signal')}
hashes={mode:{int(k):v for k,v in json.loads((OUT/(mode+'_record_hashes.json')).read_text()).items()} for mode in results}
session=results['host_signal']['context']['rolling_session']
files=[(p,p.stat().st_size) for p in sorted((ROOT/'PY_SRIPT/algo_monitor/recordings/rolling').glob('roll_'+session+'_*.jsonl'))]
parser=AlgoFrameParser();decoded=deque();synchronized=False
raws={};recent=deque(maxlen=64);offsets=Counter();alpha_errors=Counter();max_alpha_error=0.
events=[];tx=[];configs=[];diagnostics=Counter();kinds=Counter();types=Counter();ports=Counter();firmware=Counter()
gaps=[];parse_errors=[];boundary_frames=[];previous=None;last_host=None;first_time=last_time=None
matches=Counter();hash_errors=[];sensor_index=0;partial_files=[];first_log=None;last_log=None
for p,size in files:
    try: stream=p.open('rb')
    except FileNotFoundError:partial_files.append(str(p));continue
    with stream:
        while stream.tell()<size:
            line=stream.readline(size-stream.tell())
            if not line.endswith(b'\n'):break
            r=json.loads(line);kind=r['kind'];seq=r['sequence']
            first_time=first_time or r['timestamp'];last_time=r['timestamp'];first_log=first_log or seq;last_log=seq
            if previous is not None and seq!=previous+1:gaps.append([previous,seq])
            previous=seq;kinds[kind]+=1;ports[r.get('port')]+=1
            for mode,lookup in hashes.items():
                if seq in lookup:
                    matches[mode]+=1
                    if hashlib.sha256(line.rstrip(b'\r\n')).hexdigest()!=lookup[seq]:hash_errors.append([mode,seq])
            if kind=='rx':decoded.extend(parser.feed(bytes.fromhex(r['hex'])))
            elif kind=='frame':
                f=r['frame'];types[f['type']]+=1
                if decoded:
                    parsed=decoded.popleft()
                    same=json.dumps(parsed,sort_keys=True)==json.dumps(f,sort_keys=True)
                    if same:synchronized=True
                    elif synchronized:parse_errors.append(seq)
                    else:boundary_frames.append(seq)
                elif synchronized:parse_errors.append(seq)
                else:boundary_frames.append(seq)
                if f['type'] in ('debug','signal'):
                    sensor_index+=1
                    if f.get('raw'):
                        raw=tuple(f['raw']);raws[sensor_index]=raw;recent.append((sensor_index,raw))
                    firmware['raw_only']+=bool(f.get('raw_only'))
                    firmware['nonzero_baseline']+=any(f.get('baseline',[]));firmware['nonzero_signal']+=any(f.get('signal',[]))
                    firmware['finite_K']+=any(math.isfinite(v) for v in f.get('fast_k_percent',[])+f.get('slow_k_percent',[]))
                    firmware.update('state_'+str(s) for s in set(f.get('state',[])))
                else:diagnostics[f['type']]+=1
            elif kind=='host_state_machine':
                h=r['frame'];c=results['host_signal']['context'];alpha=c['host_parameters']['input_alpha'];gains=c['raw_gains']
                if last_host and h['generation']==last_host['generation'] and h['sequence']==last_host['sequence']+1:
                    inferred=tuple(round((new-(1-alpha)*old)/(alpha*g)) if g else 0
                                   for new,old,g in zip(h['input_val'],last_host['input_val'],gains))
                    candidates=[idx for idx,raw in recent if raw==inferred]
                    if candidates:offsets[h['sequence']-candidates[-1]]+=1
                    if offsets:
                        offset=offsets.most_common(1)[0][0];raw=raws.get(h['sequence']-offset)
                        if raw:
                            expected=[alpha*g*v+(1-alpha)*old for g,v,old in zip(gains,raw,last_host['input_val'])]
                            err=max(abs(a-b) for a,b in zip(expected,h['input_val']));max_alpha_error=max(err,max_alpha_error)
                            alpha_errors['verified']+=1
                            if err>1e-7:alpha_errors['mismatch']+=1
                last_host=h
            elif kind=='host_state_config':configs.append(dict(sequence=seq,timestamp=r['timestamp'],frame=r['frame']))
            if kind in ('connected','disconnected','serial_error','tx_error','log_gap','error'):events.append(r)
            if kind=='tx':tx.append(r)
result=dict(session=session,files=[dict(name=p.name,bytes=size) for p,size in files],snapshot_start=first_time,snapshot_end=last_time,
    first_log=first_log,last_log=last_log,missing_rotated_files=partial_files,sequence_gaps=gaps,kinds=dict(kinds),types=dict(types),
    ports=dict(ports),events=events,tx=tx,configs=configs,diagnostics=dict(diagnostics),firmware=dict(firmware),
    parser_boundary_skipped=boundary_frames,parser_mismatches=parse_errors,archive_matches=dict(matches),archive_hash_mismatches=hash_errors,
    archive_record_counts={mode:len(values) for mode,values in hashes.items()},
    sensor_sequence_offsets=dict(offsets),alpha_validation=dict(alpha_errors),max_alpha_error=max_alpha_error)
(OUT/'rolling_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('files','events','tx','configs')},ensure_ascii=False,indent=2))
print('EVENTS',json.dumps(events,ensure_ascii=False));print('CONFIGS',json.dumps(configs,ensure_ascii=False));print('TX',json.dumps(tx,ensure_ascii=False))
