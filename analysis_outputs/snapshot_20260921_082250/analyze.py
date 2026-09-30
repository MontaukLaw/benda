from pathlib import Path
import json
import hashlib
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'PY_SRIPT/algo_monitor'
OUT=Path(__file__).parent
meta=json.loads((BASE/'recordings/recent_3min.meta.json').read_text(encoding='utf-8'))
path=BASE/'recordings'/meta['archive_file']
assert hashlib.sha256(path.read_bytes()).hexdigest()==meta['csv_sha256']
assert hashlib.sha256((BASE/'recordings/recent_3min.csv').read_bytes()).hexdigest()==meta['csv_sha256']
d=pd.read_csv(path,low_memory=False)
d['t']=pd.to_datetime(d.timestamp)
sensor=d[d.type.isin(['debug','signal'])].copy()
gaps=sensor.t.diff().dt.total_seconds()
print('SENSOR',len(sensor),'gap',sensor.loc[gaps>.2,['timestamp']].assign(gap=gaps[gaps>.2]).to_string(index=False))
mapping=json.loads((BASE/'force_signal_pressure_maps/force_signal_pressure_map_latest.json').read_text(encoding='utf-8'))
nodes=pd.DataFrame(mapping['nodes'])
x=nodes.signal_sum_median.to_numpy(); y=nodes.measured_pressure_median_n.to_numpy()
def predict(v):
    j=np.clip(np.searchsorted(x,v),1,len(x)-1)
    return y[j-1]+(v-x[j-1])*(y[j]-y[j-1])/(x[j]-x[j-1])
tables=d[(d.type=='force_calibration_parameters')&(d.region==0)&(d.force_type==0)]
for row in tables.itertuples():
    if row.point_count==10:
        print('TABLE MATCH',row.timestamp,max(abs(getattr(row,f'raw_points_{i}')*1000-x[i]) for i in range(10)),max(abs(getattr(row,f'calibrated_points_{i}')-y[i]) for i in range(10)))
motion_path=BASE/'force_press_tests/zdt_x42s_1_3200/force_press_20260921_081243_118.csv'
m=pd.read_csv(motion_path,low_memory=False)
m['t']=pd.to_datetime(m.timestamp)
m=m[m.t.between(sensor.t.min(),sensor.t.max())].copy()
cols=['t','original_normal_force','measured_normal_force','optimized_normal_force','component_calibration_mask']
run=d[d.type=='run_force'][cols]
debug=sensor[['t']+[f'{k}_{i}' for k in ('signal','state','raw','baseline') for i in range(4,8)]]
aligned=pd.merge_asof(run.sort_values('t'),debug.sort_values('t'),on='t',direction='backward',tolerance=pd.Timedelta(milliseconds=30))
aligned['sensor_t']=aligned.t
aligned['signal_total']=aligned[[f'signal_{i}' for i in range(4,8)]].sum(axis=1,min_count=4)
aligned['floating']=aligned[[f'state_{i}' for i in range(4,8)]].eq(6).all(axis=1)
j=pd.merge_asof(m[m.event=='sensor_sample'].sort_values('t'),aligned.sort_values('t'),on='t',direction='nearest',tolerance=pd.Timedelta(milliseconds=50))
j['valid']=j.ocr_accepted.eq(True)&j.ocr_age_ms.between(0,500)&j.sensor_age_ms.between(0,500)&j.sensor_t.notna()
j['predicted']=predict(j.signal_total.to_numpy())
j['error']=j.original_normal_force-j.raw_pressure_n
holds=m[(m.event=='operation_complete')&m.message.fillna('').str.startswith('新电机到位')]
summary=[]; pieces=[]
for h in holds.itertuples():
    rel=m[(m.event=='operation_started')&(m.control_mode=='release')&(m.t>h.t)]
    end=rel.t.min() if len(rel) else sensor.t.max()
    g=j[(j.operation_id==h.operation_id)&j.t.ge(h.t)&j.t.lt(end)&j.valid&j.floating&j.measured_speed_rpm.abs().le(1)].copy()
    if g.empty: continue
    g['target']=h.target_force_n
    pieces.append(g)
    summary.append(dict(target=h.target_force_n,operation=h.operation_id,start=g.timestamp.iloc[0],end=g.timestamp.iloc[-1],samples=len(g),
        actual=g.raw_pressure_n.median(),output=g.original_normal_force.median(),predicted=g.predicted.median(),error=g.error.median(),
        signal=g.signal_total.median(),actual_start=g.raw_pressure_n.iloc[0],actual_end=g.raw_pressure_n.iloc[-1],
        output_start=g.original_normal_force.iloc[0],output_end=g.original_normal_force.iloc[-1],
        signals=[float(g[f'signal_{i}'].median()) for i in range(4,8)],
        baseline=[float(g[f'baseline_{i}'].median()) for i in range(4,8)]))
s=pd.DataFrame(summary)
print('PLATEAUS\n',s.to_string(index=False))
s.to_csv(OUT/'plateaus.csv',index=False,encoding='utf-8-sig')
pd.concat(pieces).to_csv(OUT/'held_samples.csv',index=False,encoding='utf-8-sig')
j.to_csv(OUT/'aligned.csv',index=False,encoding='utf-8-sig')
print('RUNTIME MATCH', (pd.concat(pieces).predicted-pd.concat(pieces).original_normal_force).abs().max())
cpath=Path(mapping['metadata']['source_path'])
c=json.loads(cpath.read_text(encoding='utf-8'))
print('CAL',cpath.name,c['complete'],c['message'])
for p in c['quality']['passes']:
    print('PASS',p['pass_index'],'retest',p['retest_targets'],'validation', {k:v for k,v in p.get('validation',{}).items() if k!='checks'})
v=pd.DataFrame(c['validation_records'])
last=v[v.pass_index==v.pass_index.max()].copy()
last['predicted']=predict(last.signal_sum.to_numpy());last['error']=last.predicted-last.final_force_n
print('FINAL VALIDATION\n',last[['captured_at','target_force_n','final_force_n','signal_sum','signals','predicted','error']].to_string(index=False))
last.to_csv(OUT/'previous_validation.csv',index=False,encoding='utf-8-sig')
print('TEMPERATURE',d[d.type=='temperature'][['timestamp']+[k for k in d.columns if 'temperature' in k]].iloc[[0,-1]].to_string(index=False))
# Include the short repeated 10 N warmups separately; these were not ordinary press holds.
warm=j[(j.operation_id==5)&j.valid&j.floating&(j.raw_pressure_n>8)].copy()
warm['g']=(warm.t.diff().dt.total_seconds()>.5).cumsum()
print('WARMUP10\n',warm.groupby('g').agg(start=('timestamp','first'),end=('timestamp','last'),pressure=('raw_pressure_n','median'),output=('original_normal_force','median'),signal=('signal_total','median')).to_string())
(OUT/'sources.json').write_text(json.dumps(dict(snapshot=meta,hash_verified=True,calibration_source=str(cpath),pressure_log=str(motion_path)),ensure_ascii=False,indent=2),encoding='utf-8')
