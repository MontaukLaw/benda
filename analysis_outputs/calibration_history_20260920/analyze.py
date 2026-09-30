from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'PY_SRIPT/algo_monitor'
OUT = Path(__file__).parent
sources = []
def read_json(path):
    raw = path.read_bytes()
    sources.append(dict(path=str(path.relative_to(ROOT)), sha256=hashlib.sha256(raw).hexdigest()))
    return json.loads(raw)

inventory = []
for path in sorted((BASE/'force_press_tests/high_force_calibration').glob('high_force_*.json')):
    data = read_json(path)
    inventory.append(dict(file=path.name, complete=data['complete'], records=len(data['records']),
                          message=data['message'], elapsed_s=data['elapsed_s']))
    if data['complete']:
        calibration = data
        source = path
pd.DataFrame(inventory).to_csv(OUT/'history_inventory.csv', index=False, encoding='utf-8-sig')
r = pd.DataFrame(calibration['records'])
r['captured_at'] = pd.to_datetime(r.captured_at)
nodes = pd.DataFrame(calibration['document']['nodes'])
x = nodes.signal_sum_median.to_numpy()
y = nodes.measured_pressure_median_n.to_numpy()
def predict(values, xx=x, yy=y):
    upper = np.clip(np.searchsorted(xx, values), 1, len(xx)-1)
    lower = upper-1
    return yy[lower]+(values-xx[lower])*(yy[upper]-yy[lower])/(xx[upper]-xx[lower])

r['predicted_n'] = predict(r.signal_sum.to_numpy())
r['error_n'] = r.predicted_n-r.final_force_n
r['error_pct'] = r.error_n/r.final_force_n*100
r['target_error_pct'] = (r.final_force_n/r.target_force_n-1)*100
r['signal_channels_sum_error'] = r.signals.map(sum)-r.signal_sum
print('INVENTORY', inventory)
print('CHECKS', dict(unique_keys=len(r[['round','target_force_n']].drop_duplicates()),
    floating_counts=r.floating_count.value_counts().to_dict(), sum_error=r.signal_channels_sum_error.abs().max(),
    samples=r.sample_count.agg(['min','max']).to_dict(), duration=r.capture_seconds.agg(['min','max']).to_dict()))
print('RETRIED',r[r.attempt>1][['round','target_force_n','attempt','final_force_n','signal_sum','error_n']].to_string(index=False))
print('ROUNDS',r.groupby('round').error_n.agg(['min','median','max']).to_string())
print('HIGH',r[r.target_force_n>=18].groupby('round').error_n.agg(['min','median','max']).to_string())
print('COUNTS', {str(t):int((r.error_pct.abs()>t).sum()) for t in (2,5,10)})
print('LAST COUNTS', {str(t):int((r[r['round']==5].error_pct.abs()>t).sum()) for t in (2,5,10)})
print('TOP ERRORS', r.loc[r.error_n.abs().nlargest(12).index, ['round','target_force_n','final_force_n','signal_sum','error_n','error_pct']].to_string(index=False))
r.pivot(index='target_force_n',columns='round',values='signal_sum').to_csv(OUT/'signals_by_round.csv',encoding='utf-8-sig')
r.pivot(index='target_force_n',columns='round',values='final_force_n').to_csv(OUT/'pressures_by_round.csv',encoding='utf-8-sig')
previous = r[r['round']==4].set_index('target_force_n')
last = r[r['round']==5].set_index('target_force_n')
comparison = pd.DataFrame(dict(pressure_r4=previous.final_force_n, pressure_r5=last.final_force_n,
    signal_r4=previous.signal_sum, signal_r5=last.signal_sum))
comparison['signal_change_pct'] = (comparison.signal_r5/comparison.signal_r4-1)*100
comparison['r4_table_error_r5_n'] = predict(last.signal_sum.to_numpy(),previous.signal_sum.to_numpy(),previous.final_force_n.to_numpy())-last.final_force_n
comparison.to_csv(OUT/'last_round_comparison.csv',encoding='utf-8-sig')
print('LAST PAIR', comparison.to_string())
# Separate coordinate medians can combine values from different physical presses.
mixed = []
for node in nodes.itertuples():
    g=r[r.target_force_n==node.nominal_target_force_n]
    signal_row=g.iloc[np.argmin(abs(g.signal_sum-node.signal_sum_median))]
    mixed.append(dict(target=node.nominal_target_force_n, signal_source_round=int(signal_row['round']),
        pressure_at_signal_source=signal_row.final_force_n, pressure_median=node.measured_pressure_median_n,
        difference_n=node.measured_pressure_median_n-signal_row.final_force_n))
mixed=pd.DataFrame(mixed)
mixed.to_csv(OUT/'median_pairing.csv',index=False,encoding='utf-8-sig')
print('MEDIAN PAIRING', mixed[mixed.difference_n.abs()>.05].to_string(index=False))
# Check logs overlapping the calibration: use only sensor_sample rows and fresh accepted meter readings.
frames=[]
for path in sorted((BASE/'force_press_tests/zdt_x42s_1_3200').glob('force_press_20260920_*.csv')):
    if path.name<'force_press_20260920_180920':
        continue
    raw=path.read_bytes()
    sources.append(dict(path=str(path.relative_to(ROOT)), sha256=hashlib.sha256(raw).hexdigest()))
    df=pd.read_csv(path,low_memory=False)
    df=df[df.event=='sensor_sample'].copy()
    df['t']=pd.to_datetime(df.timestamp)
    frames.append(df)
motion=pd.concat(frames,ignore_index=True).sort_values('t')
motion=motion[(motion.t>=pd.Timestamp(calibration['round_timings'][0]['started_at'])) & (motion.t<=r.captured_at.max())]
motion['fresh']=motion.ocr_accepted.eq(True)&motion.ocr_age_ms.between(0,500)&motion.sensor_age_ms.between(0,500)
motion['floating']=motion[[f'ch{c}_state' for c in range(4,8)]].eq(6).all(axis=1)
windows=[]
for row in r.itertuples():
    end=row.captured_at
    start=end-pd.Timedelta(seconds=row.capture_seconds)
    g=motion[(motion.t>=start)&(motion.t<=end)&motion.fresh&motion.floating]
    h=motion[(motion.t>=start-pd.Timedelta(seconds=15))&(motion.t<=end)&motion.fresh].copy()
    # Find the last contiguous loaded segment before capture; do not infer controller start times.
    unloaded=h[h.raw_pressure_n<.1]
    if len(unloaded): h=h[h.t>unloaded.t.max()]
    locked=h[h.floating]
    one=dict(round=row.round,target=row.target_force_n,log_samples=len(g))
    if len(g):
        one.update(pressure_start=g.raw_pressure_n.iloc[0],pressure_end=g.raw_pressure_n.iloc[-1],
            pressure_min=g.raw_pressure_n.min(),pressure_max=g.raw_pressure_n.max(),
            pressure_median=g.raw_pressure_n.median(), signal_min=g.ch4567_signal_sum.min(),signal_max=g.ch4567_signal_sum.max(),
            baseline_median=g.ch4567_baseline_sum.median(), position_median=g.position_counts.median(),
            speed_max=g.measured_speed_rpm.abs().max())
    if len(locked):
        one.update(first_observed_floating_pressure=locked.raw_pressure_n.iloc[0],
            max_pressure_before_capture=h.raw_pressure_n.max(),
            first_observed_floating_to_capture_s=(end-locked.t.iloc[0]).total_seconds())
    windows.append(one)
windows=pd.DataFrame(windows)
windows.to_csv(OUT/'capture_window_checks.csv',index=False,encoding='utf-8-sig')
r=r.merge(windows,left_on=['round','target_force_n'],right_on=['round','target'])
r.to_csv(OUT/'all_records_recheck.csv',index=False,encoding='utf-8-sig')
print('WINDOWS',windows.describe().to_string())
print('30N WINDOWS', windows[windows.target==30].to_string(index=False))
print('25N WINDOWS', windows[windows.target==25].to_string(index=False))
print('SIGNAL CHANGED WINDOWS',windows[windows.signal_max!=windows.signal_min][['round','target','signal_min','signal_max']].to_string(index=False))
print('LAST WINDOW DRIFT',windows.assign(drift=windows.pressure_end-windows.pressure_start).sort_values('drift')[['round','target','pressure_start','pressure_end']].head(10).to_string(index=False))
old=read_json(BASE/'force_press_tests/force_sweep_checkpoint_latest.json')
old_map=read_json(BASE/'force_signal_pressure_maps/force_signal_pressure_map_20260910_110406_071.json')
old_r=pd.DataFrame(old['entries'])
old_nodes=pd.DataFrame(old_map['nodes'])
old_r['table_prediction_n']=predict(old_r.signal_sum.to_numpy(),old_nodes.signal_sum_median.to_numpy(),old_nodes.measured_pressure_median_n.to_numpy())
old_r['error_n']=old_r.table_prediction_n-old_r.final_force_n
old_r['error_pct']=100*old_r.error_n/old_r.final_force_n
old_r.to_csv(OUT/'legacy_20260910_recheck.csv',index=False,encoding='utf-8-sig')
print('LEGACY',len(old_r),'release_recovered',old_r.release_idle_recovered.value_counts().to_dict(),
      'max_absolute_error_n',old_r.error_n.abs().max(),'median_absolute_error_n',old_r.error_n.abs().median(),
      'over_5pct',int((old_r.error_pct.abs()>5).sum()))
print('LEGACY DIRECTION',old_r.groupby('sweep_direction').error_n.agg(['min','median','max']).to_string())
print('LEGACY CYCLES',old_r.groupby('calibration_cycle_index').error_n.agg(['min','median','max']).to_string())
(OUT/'sources.json').write_text(json.dumps(dict(calibration=str(source),sources=sources),ensure_ascii=False,indent=2),encoding='utf-8')
