from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'PY_SRIPT/algo_monitor'
OUT = Path(__file__).parent
meta = json.loads((BASE/'recordings/recent_5min.meta.json').read_text(encoding='utf-8'))
source = BASE/'recordings'/meta['archive_file']
digest = hashlib.sha256(source.read_bytes()).hexdigest()
assert digest == meta['csv_sha256']
assert hashlib.sha256((BASE/'recordings/recent_5min.csv').read_bytes()).hexdigest() == digest
df = pd.read_csv(source, low_memory=False).copy()
df['t'] = pd.to_datetime(df.timestamp)
sensor = df[df.type.isin(['debug', 'signal'])].copy()
gaps = sensor.t.diff().dt.total_seconds()
print('SNAPSHOT', json.dumps(meta, ensure_ascii=False))
print('HASH', digest, 'sensor_samples', len(sensor), 'gaps>0.2', int((gaps>.2).sum()))
print(sensor.loc[gaps>.2, ['timestamp','type']].assign(gap=gaps[gaps>.2]).to_string(index=False))
tables = df[df.type == 'force_calibration_parameters']
print('TABLE EVENTS', tables[['timestamp','region','force_type','point_count','flags','valid','dirty']].to_string(index=False))
mapping = json.loads((BASE/'force_signal_pressure_maps/force_signal_pressure_map_latest.json').read_text(encoding='utf-8'))
nodes = pd.DataFrame(mapping['nodes'])
print('NODES', nodes[['nominal_target_force_n','signal_sum_median','measured_pressure_median_n', 'signal_sum_minimum','signal_sum_maximum']].to_string(index=False))
primary = tables[(tables.region == 0) & (tables.force_type == 0)]
for _, row in primary.iterrows():
    n = int(row.point_count)
    print('PRIMARY', row.timestamp, 'max node errors',
          max(abs(row[f'raw_points_{i}']*1000 - nodes.iloc[i].signal_sum_median) for i in range(n)) if n == len(nodes) else 'different count',
          max(abs(row[f'calibrated_points_{i}'] - nodes.iloc[i].measured_pressure_median_n) for i in range(n)) if n == len(nodes) else 'different count')
motion_path = BASE/'force_press_tests/zdt_x42s_1_3200/force_press_20260920_185943_597.csv'
motion = pd.read_csv(motion_path, low_memory=False).copy()
motion['t'] = pd.to_datetime(motion.timestamp)
motion = motion[(motion.t >= sensor.t.min()) & (motion.t <= sensor.t.max())].copy()
print('MOTION events', motion.event.value_counts().to_dict())
print('PRESS EVENTS', motion[motion.event == 'pressure_arrival_timing'][['timestamp','operation_id','target_force_n','raw_pressure_n']].to_string(index=False))
run = df[df.type == 'run_force'][['t','sequence','original_normal_force','component_calibration_mask']].rename(columns={'sequence':'run_sequence'})
debug = sensor[['t','sequence','type']+[f'{k}_{i}' for k in ('signal','state','raw','baseline') for i in range(4,8)]]
aligned = pd.merge_asof(run.sort_values('t'), debug.sort_values('t'), on='t', direction='backward', tolerance=pd.Timedelta(milliseconds=30))
aligned['signal_total'] = aligned[[f'signal_{i}' for i in range(4,8)]].sum(axis=1, min_count=4)
aligned['floating'] = aligned[[f'state_{i}' for i in range(4,8)]].eq(6).all(axis=1)
aligned['sensor_t'] = aligned.t
press = motion[motion.event == 'sensor_sample'].copy()
joined = pd.merge_asof(press.sort_values('t'), aligned.sort_values('t'), on='t', direction='nearest', tolerance=pd.Timedelta(milliseconds=50), suffixes=('_motion','_snapshot'))
joined['error_n'] = joined.original_normal_force - joined.raw_pressure_n
joined['valid_pressure'] = (joined.ocr_accepted.eq(True) & joined.ocr_age_ms.between(0,500) & joined.sensor_t.notna())
holds = motion[(motion.event == 'operation_complete') & (motion.control_mode == 'press') & motion.message.fillna('').str.startswith('新电机到位')]
joined['confirmed_hold'] = False
joined['nominal_target_n'] = np.nan
for _, hold in holds.iterrows():
    releases = motion[(motion.event == 'operation_started') & (motion.control_mode == 'release') & (motion.t > hold.t)]
    end = releases.t.min() if len(releases) else sensor.t.max()
    mask = joined.operation_id.eq(hold.operation_id) & joined.t.ge(hold.t) & joined.t.lt(end)
    joined.loc[mask, 'confirmed_hold'] = True
    joined.loc[mask, 'nominal_target_n'] = hold.target_force_n
valid = joined[joined.valid_pressure & joined.floating & joined.confirmed_hold &
               joined.measured_speed_rpm.abs().le(1) & (joined.raw_pressure_n >= 1)].copy()
print('MATCHED', len(joined), 'valid floating force>=1',len(valid))
# Split separate presses/plateaus on unloaded interval or a >1 N pressure jump or a time gap.
valid['group'] = ((valid.t.diff().dt.total_seconds() > 1) | (valid.raw_pressure_n.diff().abs() > 1)).cumsum()
summary = []
for group, data in valid.groupby('group'):
    if (data.t.iloc[-1]-data.t.iloc[0]).total_seconds() < .5: continue
    row = dict(group=int(group), target_n=data.nominal_target_n.iloc[0], start=data.timestamp.iloc[0], end=data.timestamp.iloc[-1], n=len(data),
               pressure_n=data.raw_pressure_n.median(), output_n=data.original_normal_force.median(),
               error_n=data.error_n.median(), error_pct=(100*data.error_n/data.raw_pressure_n).median(),
               signal=data.signal_total.median(), output_min=data.original_normal_force.min(), output_max=data.original_normal_force.max(),
               pressure_min=data.raw_pressure_n.min(), pressure_max=data.raw_pressure_n.max(),
               start_pressure=data.raw_pressure_n.iloc[0],end_pressure=data.raw_pressure_n.iloc[-1],
               start_output=data.original_normal_force.iloc[0],end_output=data.original_normal_force.iloc[-1])
    summary.append(row)
summary = pd.DataFrame(summary)
print('PLATEAUS\n', summary.to_string(index=False))
summary.to_csv(OUT/'plateaus.csv',index=False,encoding='utf-8-sig')
joined.to_csv(OUT/'aligned.csv', index=False,encoding='utf-8-sig')
cal = json.loads(Path(mapping['metadata']['source_path']).read_text(encoding='utf-8'))
records = pd.DataFrame(cal['records'])
print('CAL COMPLETE',cal.get('complete'), 'rest',cal['metadata'].get('rest_seconds'), 'records',len(records))
print('CAL 30N', records[records.target_force_n==30][['round','target_force_n','final_force_n','signal_sum','signals','captured_at']].to_string(index=False))
# Independently reproduce MCU interpolation, including its endpoint extrapolation.
x = nodes.signal_sum_median.to_numpy(); y = nodes.measured_pressure_median_n.to_numpy()
def predict(values):
    upper=np.clip(np.searchsorted(x,values,side='left'),1,len(x)-1)
    lower=upper-1
    return y[lower]+(values-x[lower])*(y[upper]-y[lower])/(x[upper]-x[lower])
valid['predicted_n'] = predict(valid.signal_total.to_numpy())
print('MODEL vs MCU', (valid.predicted_n-valid.original_normal_force).abs().describe().to_dict())
records['table_prediction_n']=predict(records.signal_sum.to_numpy())
records['table_error_n']=records.table_prediction_n-records.final_force_n
print('CAL ROUND ERROR\n',records[records.target_force_n>=20].groupby('round').table_error_n.agg(['min','median','max']).to_string())
records.to_csv(OUT/'calibration_recheck.csv',index=False,encoding='utf-8-sig')
(OUT/'metadata.json').write_text(json.dumps(dict(snapshot=meta, hash_verified=True, pressure_log=str(motion_path),
    calibration_source=mapping['metadata']['source_path'], generated_at=pd.Timestamp.now(tz='Asia/Shanghai').isoformat()),ensure_ascii=False,indent=2),encoding='utf-8')

report = ['# 2026-09-20 19:08:56 五分钟快照：标定后高力输出偏高', '',
    '结论：高力稳定段确有约2～3.5 N正偏差。MCU回读表与电脑20点表一致，按该表复算与MCU输出吻合；主要证据指向多轮标定的Signal响应没有收敛，聚合表不代表后期响应。高力末段斜率及外推放大偏差，FLOATING锁定又使输出不随保持期间实测压力下降。', '',
    '## 数据与对齐', '',
    f'- 固定快照：{source.name}，导出{meta["exported_at"]}。采样范围{meta["first_sensor_timestamp"]}～{meta["last_sensor_timestamp"]}，缓存完整。',
    f'- CSV SHA256：`{digest}`，与meta及recent_5min.csv均一致。只计debug/signal，共{len(sensor):,}个采样。',
    '- 19:05:59.033～19:06:01.017有1.984秒间断，与写Flash/复位时段重合；下表按压均在其后。',
    f'- 压力来源：{motion_path.name}。采用sensor_sample行缓存的压力计原始值，要求accepted、读数年龄≤500 ms；按时间匹配快照力帧（≤50 ms）。',
    '- 仅取运动日志确认寻压完成之后、释放开始之前，四路均FLOATING、速度≤1 RPM的重叠样本。表内压力和误差为对应区间中位值，不拿目标值代替实测值；区间是一次按压内的相关样本，不是多次独立验证。', '',
    '## 稳定保持段对照', '',
    '| 目标 N | 时段 | 压力计 N | MCU输出 N | 偏差 N | 相对偏差 | 样本数 |',
    '|---:|---|---:|---:|---:|---:|---:|']
for _, row in summary.iterrows():
    report.append(f'| {row.target_n:g} | {row.start[11:23]}～{row.end[11:23]} | {row.pressure_n:.3f} | {row.output_n:.3f} | +{row.error_n:.3f} | {row.error_pct:.2f}% | {row.n} |')
report += ['', '## 写入和插值核对', '',
    '- 19:05:58.941回读region 0主法向为20点valid/dirty；19:06:01.561复位后回读20点valid、非dirty。其他五张表仍无效默认表，节点与PC latest一致。最大Signal节点差0.000947 counts、压力节点差0.000001 N，属于float32表示误差。',
    f'- 稳定段用PC节点独立复算，最大MCU差值{(valid.predicted_n-valid.original_normal_force).abs().max():.8f} N，不支持地址冲突、写错表或插值算错的解释。',
    '- 高力最后两节点：(29505 counts, 27.665 N)、(30094 counts, 29.45 N)。斜率0.00303056 N/count，每增加100 counts换算增加约0.303 N。',
    '- 本次30 N按压锁定Signal=30991 counts，比末节点多897（2.98%），得到29.45+897×0.00303056=32.16841 N。',
    '- 源码sensor_algorithm.c的apply_piecewise_force_calibration在超出末节点时仍沿最后一段外推，主法向调用没有上限门槛；PC元数据虽写extrapolation=disabled，但不能据此认为MCU已禁止外推。这是实际实现与元数据语义不一致。', '',
    '## 五轮标定已存在漂移', '',
    '该表来自18:11:15～19:02:15的20点×5轮完整标定，点间设置20秒。相同目标分别取Signal及实测压力中位数后建表。', '',
    '| 轮次 | 30 N档实测 N | Signal和 counts | 聚合表复算 N | 误差 N |',
    '|---:|---:|---:|---:|---:|']
for _, row in records[records.target_force_n == 30].iterrows():
    report.append(f'| {int(row["round"])} | {row.final_force_n:.3f} | {row.signal_sum:.0f} | {row.table_prediction_n:.3f} | {row.table_error_n:+.3f} |')
report += ['',
    '- 第3轮到第5轮实测约29.4 N，但Signal由30094增到30846（+2.50%）；第5轮自身数据代入最终表就已高估2.319 N。当前30991只比第5轮高0.47%，说明主要偏差在标定聚合完成时已经存在。',
    '- 不是CH5单路异常：30 N档第3轮到第5轮，CH4/5/6/7分别增加188/183/197/184 counts。',
    '- 可确认跨轮力—Signal关系变化，尚不能只凭这份数据区分材料预处理、加载历史、温度或其他机械条件的贡献。验证多为高档向低档、部分点卸载后约2～4秒再次按压，与标定升序和20秒间隔不同，可能叠加历史效应。', '',
    '## FLOATING保持的额外影响', '',
    '- 30 N档19:07:29.479～19:07:33.035，压力从29.58降到29.09 N，MCU一直32.1684 N。锁定初始已有+2.5884 N偏差，之后压力松弛又扩大约0.49 N；因此不能把全部偏差归为锁定过早。',
    '- 同段Raw四路分别上升14/38/18/30 counts，Signal四路全部固定。Raw与真实压力变化方向也不一致，直接提高对Raw变化的灵敏度未必能改善力值准确性。', '',
    '## 建议的修正顺序', '',
    '1. 先把跨轮收敛及逐轮反算误差作为标定质量条件；将尚在变化的轮次作为预处理，不能只因节点递增就发布成功。需明确允许的误差标准，达不到时继续预处理/补测并保留原表。',
    '2. 固定标定和验证的采样时机、加载方向及休息条件，使用独立验证按压检查高力段。单用最后一轮可能更接近当前状态，但第5轮仍在漂移，不能直接当作已收敛。',
    '3. 明确表外输出策略并补齐30 N端有效覆盖；同时单独验证FLOATING保持时的真实力跟踪。只把显示钳到30 N会掩盖28 N、24 N等点的偏差。', '',
    '本次仅离线分析，未修改控制/固件代码、未写入K表、未操作电机。']
(OUT/'report.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
