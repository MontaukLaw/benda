from pathlib import Path
import csv
import hashlib
import io
import json
import sys
from datetime import datetime
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
REC = ROOT / 'PY_SRIPT/algo_monitor/recordings'
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from centroid_filter import CentroidFilter

def dump(name, value):
    (OUT/name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')

data = (REC/'recent_3min.csv').read_bytes()
meta = json.loads((REC/'recent_3min.meta.json').read_text(encoding='utf-8'))
sha = hashlib.sha256(data).hexdigest()
assert sha == meta['csv_sha256'], (sha, meta['csv_sha256'])
(OUT/'source.csv').write_bytes(data)
dump('source.meta.json', meta)
rows = list(csv.DictReader(io.StringIO(data.decode('utf-8-sig'))))
samples = [r for r in rows if r['type'] in ('debug', 'signal')]
epoch = np.array([datetime.fromisoformat(r['timestamp']).timestamp() for r in samples])
t = epoch - epoch[0]
arrays = {key: np.array([[float(r[f'{key}_{ch}']) for ch in range(20)] for r in samples])
          for key in ('raw','baseline','signal','state','fast_k_percent','slow_k_percent')}
s = arrays['signal']
total = s.sum(axis=1)
valid = s.max(axis=1) >= 500
allxy, localxy, smoothall, smoothlocal, blocks = [], [], [], [], []
filters = [CentroidFilter(500, local_2x2_enabled=b) for b in (False, True)]
for i in range(len(samples)):
    for f, raw, smooth in zip(filters, (allxy, localxy), (smoothall, smoothlocal)):
        point = f.update(s[i], t[i], arrays['state'][i])
        raw.append(f.raw_xy or (np.nan, np.nan))
        smooth.append(point or (np.nan, np.nan))
    blocks.append(filters[1].selected_channels)
allxy, localxy, smoothall, smoothlocal = map(np.array, (allxy, localxy, smoothall, smoothlocal))
np.savez_compressed(OUT/'replay.npz', t=t, epoch=epoch, **arrays, allxy=allxy,
                    localxy=localxy, smoothall=smoothall, smoothlocal=smoothlocal, valid=valid)

logs = sorted((REC/'rolling').glob('roll_20260926_222633_991875_eeeec7c0_*.jsonl'))
events, centroids = [], []
kinds, types, ports = Counter(), Counter(), Counter()
gaps, tails, malformed = [], [], []
last_seq = None
matched_debug = 0
for path in logs:
    blob = path.read_bytes()
    if not blob.endswith(b'\n'): tails.append(path.name)
    for lineno, line in enumerate(blob.splitlines(keepends=True), 1):
        if not line.endswith(b'\n'): continue
        try: rec = json.loads(line)
        except ValueError:
            malformed.append([path.name, lineno]); continue
        stamp = rec['time_ns']/1e9
        if stamp > epoch[-1]+.001: continue
        seq = rec.get('sequence')
        if last_seq is not None and seq != last_seq+1: gaps.append([last_seq, seq])
        last_seq = seq
        kinds[rec['kind']] += 1
        ports[rec.get('port')] += 1
        if rec['kind'] == 'centroid':
            centroids.append(rec)
        elif rec['kind'] == 'frame':
            types[rec['frame']['type']] += 1
            if rec['frame']['type'] not in ('debug','signal'): events.append(rec)
            else: matched_debug += 1
        elif rec['kind'] not in ('rx',): events.append(rec)
dump('events.json', events)
dump('centroids.json', centroids)
audit = dict(meta=meta, sha256=sha, hash_match=True, rows=len(rows), samples=len(samples),
             frame_types=dict(Counter(r['type'] for r in rows)),
             span_seconds=float(t[-1]), fps=(len(t)-1)/float(t[-1]),
             dt_ms=np.percentile(np.diff(t)*1000,[0,50,95,99,100]).tolist(),
             csv_sequence_gaps=int(np.sum(np.diff([int(r['sequence']) for r in rows]) != 1)),
             log_files=[p.name for p in logs], log_kinds=dict(kinds), log_types=dict(types),
             log_ports=dict(ports), log_sequence_gaps=gaps, partial_tail_files=tails,
             malformed=malformed, log_sensor_count=matched_debug,
             centroid_modes=dict(Counter(r['frame'].get('filter',{}).get('centroid_algorithm','unknown') for r in centroids)),
             centroid_smoothing=dict(Counter(str(r['frame'].get('smoothing_enabled')) for r in centroids)),
             centroid_thresholds=dict(Counter(r['frame'].get('signal_threshold') for r in centroids)))
dump('audit.json',audit)
print(json.dumps(audit, ensure_ascii=False, indent=2))
print('DIAGNOSTIC EVENTS', [(r['timestamp'],r['kind'],r.get('frame',{}).get('type'),r.get('hex')) for r in events if r.get('frame',{}).get('type')!='temperature'])

fig, axs = plt.subplots(4,1,figsize=(15,11),sharex=True)
for axis in range(2):
    axs[axis].plot(t, np.where(valid,allxy[:,axis],np.nan),color='0.7',lw=.7,label='All 20 raw')
    axs[axis].plot(t, np.where(valid,smoothall[:,axis],np.nan),lw=1,label='All 20 filtered')
    axs[axis].plot(t, np.where(valid,smoothlocal[:,axis],np.nan),lw=.8,alpha=.8,label='2x2 filtered replay')
    axs[axis].set_ylabel(('X','Y')[axis]+' (cell)')
    axs[axis].legend(loc='upper right',ncol=3)
for col in range(5): axs[2].plot(t,s[:,col*4:(col+1)*4].sum(axis=1),label=f'Col {col} CH{col*4}-{col*4+3}',lw=.8)
axs[2].set_ylabel('Column Signal sum'); axs[2].legend(ncol=5)
axs[3].imshow(s.T,aspect='auto',extent=(0,t[-1],19.5,-.5),interpolation='nearest',cmap='magma')
axs[3].set_ylabel('Channel'); axs[3].set_xlabel('Seconds from 2026-09-26 22:26:40.786 +08:00')
for ax in axs[:3]: ax.grid(alpha=.25)
fig.tight_layout(); fig.savefig(OUT/'overview.png',dpi=140); plt.close(fig)
for start in range(0,112,5):
    m=(t>=start)&(t<start+5)&valid
    if m.sum():
        print('WINDOW',start,'N',m.sum(),'X',np.round(np.percentile(allxy[m,0],[0,50,100]),3),
              'Y',np.round(np.percentile(allxy[m,1],[0,50,100]),3),'total',np.round(np.percentile(total[m],[0,50,100])))
