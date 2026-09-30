from pathlib import Path
import sys
import json
import hashlib
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'PY_SRIPT/algo_monitor'
sys.path.insert(0,str(BASE))
from high_force_calibration import build_high_force_mapping
from high_force_calibration_quality import assess_mapping

OUT=Path(__file__).parent
source=BASE/'force_press_tests/high_force_calibration/high_force_20260920_195453_003272.json'
c=json.loads(source.read_text(encoding='utf-8'))
r=pd.DataFrame(c['records'])
assert len(r)==50 and not r[['round','target_force_n']].duplicated().any()
print('TIMES',c['stage_timings'],c['round_timings'])
rows=[]
for target,g in r.groupby('target_force_n'):
    rows.append(dict(target=target, pressure_min=g.final_force_n.min(),pressure_max=g.final_force_n.max(),
        signal_min=g.signal_sum.min(),signal_max=g.signal_sum.max(),
        signal_span_pct=100*(g.signal_sum.max()-g.signal_sum.min())/g.signal_sum.median(),
        signal_r1=g.signal_sum.iloc[0],signal_r5=g.signal_sum.iloc[-1],
        signal_r1_r5_pct=100*(g.signal_sum.iloc[-1]/g.signal_sum.iloc[0]-1)))
summary=pd.DataFrame(rows)
summary.to_csv(OUT/'point_repeatability.csv',index=False,encoding='utf-8-sig')
print('REPEATABILITY\n',summary.round(4).to_string(index=False))
print('SIGNALS\n',r.pivot(index='target_force_n',columns='round',values='signal_sum').to_string())
print('PRESSURES\n',r.pivot(index='target_force_n',columns='round',values='final_force_n').to_string())
v=pd.DataFrame(c['validation_records'])
final=v[v.pass_index==v.pass_index.max()].to_dict('records')
result=[]; errors=[]
for n in (2,3,5):
    training=r[r['round']<=n].to_dict('records')
    doc=build_high_force_mapping(training,10,n)
    groups={'training':training,'final_validation':final}
    if n<5: groups['later_formal_rounds']=r[r['round']>n].to_dict('records')
    for name,entries in groups.items():
        a=assess_mapping(doc,entries,2)
        es=np.array([e['error_n'] for e in a['checks']])
        result.append(dict(rounds=n,dataset=name,count=len(entries),passed=sum(e['passed'] for e in a['checks']),
            max_error_n=max(abs(es)),mean_abs_error_n=np.mean(abs(es)),rmse_n=np.sqrt(np.mean(es**2)),
            failed_targets=a['failed_targets']))
        errors.extend(dict(rounds=n,dataset=name,**e) for e in a['checks'])
# Compare to the table after targeted repairs, on the same final validation set.
a=assess_mapping(c['document'],final,2)
es=np.array([e['error_n'] for e in a['checks']])
result.append(dict(rounds='5+targeted',dataset='final_validation',count=len(final),passed=sum(e['passed'] for e in a['checks']),
    max_error_n=max(abs(es)),mean_abs_error_n=np.mean(abs(es)),rmse_n=np.sqrt(np.mean(es**2)),failed_targets=a['failed_targets']))
print('MODELS\n',pd.DataFrame(result).to_string(index=False))
pd.DataFrame(result).to_csv(OUT/'round_count_comparison.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(errors).to_csv(OUT/'prediction_errors.csv',index=False,encoding='utf-8-sig')
# Resample each round at shared measured pressures, to avoid attributing pressure differences to sensor drift.
grid=[]
curves=[g.sort_values('final_force_n') for _,g in r.groupby('round')]
for target in sorted(r.target_force_n.unique()):
    pressure=float(r[r.target_force_n==target].final_force_n.median())
    if not all(g.final_force_n.min()<=pressure<=g.final_force_n.max() for g in curves): continue
    signals=[float(np.interp(pressure,g.final_force_n,g.signal_sum)) for g in curves]
    grid.append(dict(target=target,common_pressure=pressure,signal_span_pct=100*(max(signals)-min(signals))/np.median(signals),
                     signals=signals))
pd.DataFrame(grid).to_csv(OUT/'common_pressure_comparison.csv',index=False,encoding='utf-8-sig')
print('COMMON PRESSURE\n',pd.DataFrame(grid).to_string(index=False))
for p in c['quality']['passes']:
    print('RETEST',p['pass_index'],p['retest_targets'], 'final_max',p.get('validation',{}).get('max_error_n'))
(OUT/'source.json').write_text(json.dumps(dict(path=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    note='Offline replay only. Reducing acquisition rounds changes material loading history; real shortened-run accuracy is not measured.'),ensure_ascii=False,indent=2),encoding='utf-8')
