"""对已结束的回弹会话离线计算可复核指标；不控制电机，也不改变原始记录。"""
from __future__ import annotations

import csv
import hashlib
import html
import json
import sqlite3
import uuid
from contextlib import closing
from datetime import datetime
from pathlib import Path

import numpy as np
from rebound_report_ui import CSS as REPORT_CSS, filter_controls, interaction_script
from rebound_test_protocol import CONTINUOUS_SHORT_PRESS_TEMPLATES

ANALYSIS_VERSION = 6
CHANNELS = (4, 5, 6, 7)
CHECKPOINTS = (1, 5, 10, 30, 60, 120, 300, 600)


def session_chain(directory):
    """按父子关系读取会话；只合并条件结果，不拼接各段单调时间。"""
    chain, seen = [], set()
    path = Path(directory).resolve()
    while True:
        if path in seen:
            raise ValueError("续测会话存在循环引用")
        seen.add(path)
        manifest = json.loads((path / 'manifest.json').read_text(encoding='utf-8'))
        if manifest.get('status') == 'running':
            raise ValueError("测试尚未结束，请结束并保存后再分析")
        if manifest.get('config', {}).get('protocol') != 'raw_rebound_v1':
            raise ValueError("此分析仅支持单次 Raw 回弹协议")
        chain.append((path, manifest))
        parent = manifest.get('resumed_from')
        if not parent:
            return list(reversed(chain))
        parent = Path(parent)
        if not parent.is_dir():
            parent = path.parent / parent.name
        if not parent.is_dir():
            raise ValueError(f"缺少原会话，不能完整分析续测链：{parent}")
        path = parent.resolve()


def median(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.median(values)) if len(values) else None


def first_confirmed(t, valid, seconds=1., max_gap=.25):
    """返回连续有效区间的确认时刻；缺帧、离带会重新计时。"""
    start = previous = None
    for at, ok in zip(t, valid):
        if not ok:
            start = previous = None
            continue
        if start is None or at - previous > max_gap:
            start = at
        previous = at
        if at - start >= seconds:
            return float(at)
    return None


def hold_raw_reference(t, raw, arrival, release_command, pre_raw, noise):
    """到位保持段末尾最多10秒；排除加载运动、释放及无效Raw。"""
    start = max(arrival, release_command - 10.)
    valid = (t >= start) & (t < release_command) & np.isfinite(raw)
    at = t[valid]
    value = median(raw[valid])
    duration = max(0., release_command - start)
    gap = float(max(np.diff(np.r_[start, at, release_command]))) if len(at) else duration
    response = None if value is None else value - pre_raw
    covered = duration > 0 and len(at) >= 2 and gap <= .25
    return dict(hold_raw_median=value, hold_response_counts=response,
                hold_raw_window_s=duration, hold_raw_sample_count=int(len(at)),
                hold_raw_max_gap_s=gap, hold_raw_covered=covered,
                percent_raw_valid=bool(covered and value is not None and value > 0),
                percent_response_valid=bool(covered and response is not None and response > max(2., 3*noise)))


def residual_percent(value, reference, valid):
    """分母无效或接近噪声时不输出误导性百分比；保留残差的符号。"""
    return float(100*value/reference) if valid and value is not None else None


def _read_session(path, manifest):
    cols = ['sensor_source_monotonic', 'trial_index', 'attempt_index',
            'sensor_sequence', 'die_temperature_celsius', 'die_temperature_age_seconds']
    cols += [f'ch{c}_{v}' for c in CHANNELS for v in ('hardware_raw', 'baseline', 'state')]
    blocks, pressure, events, sources = [], [], [], []
    parts = manifest.get('output_parts') or [p.name for p in sorted(path.glob('raw_data_part_*.sqlite3'))]
    if not parts:
        raise ValueError(f"没有原始数据分卷：{path}")
    for name in parts:
        db = path / name
        digest = hashlib.sha256()
        with db.open('rb') as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b''):
                digest.update(chunk)
        sources.append(dict(path=str(db), bytes=db.stat().st_size, sha256=digest.hexdigest()))
        with closing(sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True)) as conn:
            available = {row[1] for row in conn.execute('PRAGMA table_info(samples)')}
            # 旧档案缺少硬件 Raw 时，保留读取历史归一化计数的能力。
            selected = [c if c in available else c.replace('hardware_raw', 'normalized_raw') for c in cols]
            sources[-1]["raw_columns"] = {c: actual for c, actual in zip(cols, selected) if c.endswith("hardware_raw")}
            values = conn.execute('SELECT ' + ','.join(selected) + ' FROM samples ORDER BY id').fetchall()
            if values:
                blocks.append(np.asarray(values, dtype=float))
            for event, payload in conn.execute('SELECT event,payload_json FROM events ORDER BY id'):
                item = json.loads(payload)
                if event == 'pressure_sample':
                    pressure.append((item.get('sampled_monotonic'), item.get('raw_force_n'),
                                     bool(item.get('accepted')) and not item.get('invalidate', False)))
                else:
                    events.append((event, item))
    data = np.concatenate(blocks) if blocks else np.empty((0, len(cols)))
    p = np.asarray(pressure, dtype=float).reshape(-1, 3)
    if len(p):
        p = p[np.argsort(p[:, 0], kind='stable')]
        p = p[np.r_[True, np.diff(p[:, 0]) > 0]]
    return {k: data[:, i] for i, k in enumerate(cols)}, p, events, sources


def _force_stats(p, start, end):
    rows = p[(p[:, 0] >= start) & (p[:, 0] <= end)]
    valid = rows[(rows[:, 2] == 1) & np.isfinite(rows[:, 1])]
    if not len(valid):
        return dict(median_n=None, min_n=None, max_n=None, impulse_ns=None, max_gap_s=end-start)
    # 只积分相邻的有效新压力帧；无效帧与大于0.25秒的空档不插值。
    dt = np.diff(rows[:, 0])
    good = (rows[:-1, 2] == 1) & (rows[1:, 2] == 1) & (dt <= .25)
    good &= np.isfinite(rows[:-1, 1]) & np.isfinite(rows[1:, 1])
    area = np.sum(dt[good] * (np.maximum(rows[:-1, 1][good], 0) + np.maximum(rows[1:, 1][good], 0)) / 2)
    return dict(median_n=median(valid[:, 1]), min_n=float(min(valid[:, 1])),
                max_n=float(max(valid[:, 1])), impulse_ns=float(area),
                max_gap_s=float(max(np.diff(np.r_[start, valid[:, 0], end]))))


def analyze_rebound(directory, output_dir=None):
    """生成汇总、逐通道指标和一秒曲线；未返回用空值，绝不填观察上限。"""
    if output_dir is not None and Path(output_dir).exists():
        raise FileExistsError(f'为保留已有结果，请指定尚不存在的输出文件夹：{output_dir}')
    chain = session_chain(directory)
    trials, channels, curves, provenance, sessions, incomplete = [], [], [], [], [], []
    for path, manifest in chain:
        data, pressure, events, sources = _read_session(path, manifest)
        provenance.extend(sources)
        t_all = data['sensor_source_monotonic']
        good_t = t_all[np.isfinite(t_all)]
        duration = (datetime.fromisoformat(manifest['finished_at']) - datetime.fromisoformat(manifest['created_at'])).total_seconds()
        sessions.append(dict(session=path.name, status=manifest['status'], sample_count=len(t_all),
            duration_s=duration, sample_hz=len(t_all)/duration if duration else None,
            raw_max_gap_s=float(max(np.diff(good_t))) if len(good_t)>1 else None,
            raw_gaps_over_250ms=int(np.sum(np.diff(good_t)>.25)),
            recorder_dropped=manifest.get('dropped_count'), recorder_error=manifest.get('recorder_error'),
            created_at=manifest['created_at'], finished_at=manifest['finished_at'],
            resumed_from=manifest.get('resumed_from')))
        complete = [p for event, p in events if event=='trial_complete']
        template_id = manifest.get('config', {}).get('template_id')
        short_press = template_id in CONTINUOUS_SHORT_PRESS_TEMPLATES
        session_reference = {}
        for event, p in events:
            if event=='phase_start' and p.get('phase')=='creep_trial_failed':
                incomplete.append(dict(session=path.name, planned_trial_index=p.get('planned_trial_index'),
                                       attempt_index=p.get('attempt_index'), reason=p.get('reason')))
        for payload in complete:
            result = payload['trial_result']
            paired = result.get('category') == 'paired_rebound'
            pair = result.get('paired_results', {})
            if paired:
                # 主表以第二次小压力为对象；大压力历史单独保留，不冒充单次20 N回弹。
                payload = dict(payload, high_force_n=result['probe_force_n'],
                               high_hold_seconds=result['probe_hold_seconds'],
                               recovery_seconds=result['post_observation_seconds'])
                result = pair['probe']
            index, attempt = payload['trial_index'], payload.get('attempt_index', 1)
            planned = payload.get('planned_trial_index', index)
            plan = manifest.get('trials', [])
            trial_plan = plan[index-1] if 0 < index <= len(plan) else {}
            press = result['press_result']; release = result['release_result']
            start, arrival = press['motion_started_monotonic'], press['arrived_monotonic']
            command = release['release_command_monotonic']
            zero = release['release_zero_confirmed_monotonic']
            end = payload['controller_monotonic']
            mask = (data['trial_index']==index) & (data['attempt_index']==attempt) & np.isfinite(t_all)
            d = {k:v[mask] for k,v in data.items()}
            order = np.argsort(d['sensor_source_monotonic'], kind='stable')
            d = {k:v[order] for k,v in d.items()}; t=d['sensor_source_monotonic']
            # 连续短按复用前一次恢复末段，不要求这些采样属于当前trial_index。
            pre_data = data if short_press else d
            pre_t = pre_data['sensor_source_monotonic']
            pre_seconds = 1.0 if paired else 10.0
            pre_start = start-pre_seconds
            if paired:
                pre_start = max(pre_start, pair['high']['release_result']['release_zero_confirmed_monotonic'])
            pre = (pre_t>=pre_start) & (pre_t<start) & np.isfinite(pre_t)
            recovery = (t>=zero) & (t<=end)
            relative = t[recovery]-zero
            hold = _force_stats(pressure, arrival, command)
            loading = _force_stats(pressure, start, zero)
            unloaded = _force_stats(pressure, zero, end)
            fresh_temp = np.isfinite(d['die_temperature_celsius']) & (d['die_temperature_age_seconds'] <= 3)
            record = dict(session=path.name, planned_trial_index=planned, trial_index=index,
                attempt_index=attempt, repeat=payload.get('repeat_index'), target_n=payload['high_force_n'],
                hold_requested_s=payload['high_hold_seconds'], hold_actual_s=command-arrival,
                hold_median_n=hold['median_n'], hold_min_n=hold['min_n'], hold_max_n=hold['max_n'],
                load_peak_n=loading['max_n'], force_integral_ns=loading['impulse_ns'],
                force_max_gap_s=loading['max_gap_s'], release_confirm_delay_s=zero-command,
                unloaded_min_n=unloaded['min_n'], unloaded_max_n=unloaded['max_n'],
                temperature_pre_c=median(pre_data['die_temperature_celsius'][pre &
                    np.isfinite(pre_data['die_temperature_celsius']) & (pre_data['die_temperature_age_seconds']<=3)]),
                temperature_tail_c=median(d['die_temperature_celsius'][(t>=end-30) & fresh_temp]),
                window_s=end-zero, history_interrupted=payload.get('history_interrupted',False),
                quality_issues=list(payload.get('data_quality_issues', [])))
            expected_window = float(payload.get('recovery_seconds', end-zero))
            if expected_window > 0 and (not len(relative) or relative[-1]<max(0,expected_window-1) or relative[0]>.25 or (len(relative)>1 and max(np.diff(relative))>.25)):
                record['quality_issues'].append('recovery_sampling_gap')
            if loading['max_gap_s']>.25:
                record['quality_issues'].append('force_sampling_gap')
            if np.count_nonzero(pre)<(8 if paired else 10):
                record['quality_issues'].append('insufficient_pre_reference')
            if paired:
                high_press, high_release = pair['high']['press_result'], pair['high']['release_result']
                high_start = high_press['motion_started_monotonic']
                high_stats = _force_stats(pressure, high_press['arrived_monotonic'], high_release['release_command_monotonic'])
                high_pre = (t_all>=high_start-10) & (t_all<high_start) & np.isfinite(t_all)
                pre_at = pre_t[pre]
                if not len(pre_at) or max(np.diff(np.r_[pre_start, np.sort(pre_at), start]))>.25:
                    record['quality_issues'].append('pre_reference_sampling_gap')
                record.update(observation_template='paired_rebound',
                    large_target_n=payload['trial_result']['high_force_n'], large_hold_median_n=high_stats['median_n'],
                    large_hold_actual_s=high_release['release_command_monotonic']-high_press['arrived_monotonic'],
                    delay_requested_s=payload['trial_result']['recovery_seconds'],
                    delay_actual_s=start-high_release['release_zero_confirmed_monotonic'],
                    pre_reference_window_s=pre_seconds,
                    large_pre_reference_covered=bool(np.count_nonzero(high_pre)>=2 and
                        max(np.diff(np.r_[high_start-10, np.sort(t_all[high_pre]), high_start]))<=.25))
                if not record['large_pre_reference_covered']:
                    record['quality_issues'].append('large_pre_reference_sampling_gap')
            if short_press:
                at = np.sort(pre_t[pre])
                if (len(at)<2 or at[0]>start-9.75 or at[-1]<start-.25
                        or (len(at)>1 and max(np.diff(at))>.25)):
                    record['quality_issues'].append('pre_reference_sampling_gap')
                record['observation_template'] = template_id
            if record['window_s']<expected_window-1:
                record['quality_issues'].append('short_observation')
            trials.append(record)
            if trial_plan.get('calibration_model') == 'single_exponential_calibration_v1':
                record['analysis_role'] = trial_plan.get('analysis_role')
                record['calibration_model'] = trial_plan['calibration_model']
            for channel in CHANNELS:
                raw=d[f'ch{channel}_hardware_raw']; state=d[f'ch{channel}_state']
                pre_raw = pre_data[f'ch{channel}_hardware_raw'][pre]
                reference=median(pre_raw); baseline=median(pre_data[f'ch{channel}_baseline'][pre])
                if short_press and payload is complete[0]:
                    session_reference[channel] = reference
                if reference is None:
                    continue
                noise=1.4826*median(np.abs(pre_raw-reference))
                dt=raw[recovery]-reference
                idle=(state[recovery]==0)
                tail=(relative>=max(0,end-zero-30))&(relative<=end-zero)
                metrics=dict(session=path.name, planned_trial_index=planned, channel=channel,
                    target_n=record['target_n'], hold_requested_s=record['hold_requested_s'],
                    hold_actual_s=record['hold_actual_s'], hold_median_n=record['hold_median_n'],
                    pre_raw=reference, pre_baseline=baseline, pre_noise_mad_counts=noise,
                    pre_slope_counts_s=_slope(pre_t[pre]-start,pre_raw),
                    first_idle_s=float(relative[np.flatnonzero(idle)[0]]) if any(idle) else None,
                    confirmed_idle_1s=first_confirmed(relative,idle),
                    tail_residual_counts=median(dt[tail]),
                    tail_slope_counts_s=_slope(relative[tail],dt[tail]))
                metrics.update(hold_raw_reference(t, raw, arrival, command, reference, noise))
                if paired:
                    large_reference = median(data[f'ch{channel}_hardware_raw'][high_pre])
                    large_hold = median(raw[(t>=high_press['arrived_monotonic']) &
                                            (t<high_release['release_command_monotonic'])])
                    metrics.update(repeat=record['repeat'], delay_requested_s=record['delay_requested_s'],
                        delay_actual_s=record['delay_actual_s'], large_hold_median_n=record['large_hold_median_n'],
                        large_pre_raw=large_reference, large_hold_raw_median=large_hold,
                        residual_before_small_counts=None if large_reference is None else reference-large_reference,
                        small_hold_from_large_reference_counts=None if large_reference is None or metrics['hold_raw_median'] is None
                            else metrics['hold_raw_median']-large_reference,
                        large_pre_reference_covered=record['large_pre_reference_covered'])
                metrics['tail_rising'] = (metrics['tail_slope_counts_s'] is not None
                    and metrics['tail_slope_counts_s'] > max(.1, 3*noise/30))
                for seconds in CHECKPOINTS:
                    hit=(relative>=seconds-.5)&(relative<seconds+.5)
                    metrics[f'residual_{seconds}s']=median(dt[hit])
                if short_press:
                    initial = session_reference.get(channel)
                    offset = None if initial is None else reference-initial
                    metrics.update(repeat=record['repeat'], session_initial_raw=initial,
                        pre_shift_from_session_start_counts=offset,
                        residual_10s_from_session_start_counts=(None if offset is None or metrics['residual_10s'] is None
                                                               else metrics['residual_10s']+offset))
                    after_ten = [v for v in dt[(relative>=10)&(relative<=60)] if np.isfinite(v)]
                    metrics['maximum_abs_residual_10_to_60s_counts'] = max(map(abs,after_ten)) if after_ten else None
                    metrics['maximum_abs_residual_10_to_60s_percent_of_hold_response'] = residual_percent(
                        metrics['maximum_abs_residual_10_to_60s_counts'],metrics['hold_response_counts'],metrics['percent_response_valid'])
                for key in [f'residual_{seconds}s' for seconds in CHECKPOINTS] + ['tail_residual_counts']:
                    metrics[key+'_percent_of_hold_raw'] = residual_percent(
                        metrics[key], metrics['hold_raw_median'], metrics['percent_raw_valid'])
                    metrics[key+'_percent_of_hold_response'] = residual_percent(
                        metrics[key], metrics['hold_response_counts'], metrics['percent_response_valid'])
                # 一秒中位数降低计数量化噪声，缺少覆盖的秒不参与连续返回判断。
                binned=[]
                for sec in range(int(max(0,end-zero))):
                    hit=(relative>=sec)&(relative<sec+1)
                    at=relative[hit]
                    value=median(dt[hit])
                    covered=(len(at)>1 and at[0]-sec<=.25 and sec+1-at[-1]<=.25 and max(np.diff(at))<=.25)
                    binned.append(value if covered else None)
                    curves.append(dict(session=path.name,planned_trial_index=planned,channel=channel,
                                       seconds=sec+.5,residual_counts=value,covered=bool(covered),
                                       residual_percent_of_hold_raw=residual_percent(value,metrics['hold_raw_median'],metrics['percent_raw_valid']),
                                       residual_percent_of_hold_response=residual_percent(value,metrics['hold_response_counts'],metrics['percent_response_valid'])))
                for band in (2,5):
                    inside=np.array([v is not None and abs(v)<=band for v in binned])
                    # 5个完整相邻秒窗；保存确认时刻，不把首次穿越当永久恢复。
                    confirmed = next((i+1 for i in range(4,len(inside)) if all(inside[i-4:i+1])),None)
                    metrics[f'confirmed_raw_{band}counts_5s']=confirmed
                    metrics[f'left_band_after_{band}counts']=(bool(any(
                        v is not None and abs(v)>band for v in binned[confirmed:])) if confirmed is not None else None)
                    metrics[f'gap_after_{band}counts']=(any(v is None for v in binned[confirmed:]) if confirmed is not None else None)
                channels.append(metrics)
    # 同一原计划编号允许多次完整尝试时保留全部，报告重复而不静默覆盖。
    counts={i:sum(t['planned_trial_index']==i for t in trials) for i in {t['planned_trial_index'] for t in trials}}
    report=dict(analysis_version=ANALYSIS_VERSION, generated_at=datetime.now().astimezone().isoformat(),
        sessions=sessions, completed_conditions=len(counts), completed_attempts=len(trials),
        incomplete_attempts=incomplete, repeated_conditions=[i for i,n in counts.items() if n>1],
        trials=trials, channels=channels, source_files=provenance,
        definitions=dict(time_zero='release_zero_confirmed_monotonic', reference='median of last 10 seconds before press',
            hold_raw='median of finite Raw in [max(arrival, release_command-10), release_command)',
            hold_response='hold_raw minus pre-press reference; not firmware Signal',
            residual_percent_of_hold_raw='100 * signed residual / hold_raw',
            residual_percent_of_hold_response='100 * signed residual / hold_response',
            percent_validity='hold interval must have >=2 samples and no gap >0.25s; raw denominator >0; response denominator >max(2 counts,3*pre_noise_MAD_sigma)',
            raw_return='5 consecutive covered 1-second medians within +/-2 or +/-5 counts; confirmation time',
            idle_return='actual state=0 continuously for 1 second; max sample gap 0.25 seconds',
            tail='median of final 30 seconds of the actual unload observation', fixed_residual='median within +/-0.5 seconds; only within recorded observation',
            model_coefficients='not fitted; reference is not necessarily fully recovered baseline'))
    report['calibration_goals'] = [dict(session=path.name, **manifest['config']['calibration_goal'])
                                 for path,manifest in chain if manifest.get('config',{}).get('calibration_goal')]
    report['paired_observation'] = any(t.get('observation_template') == 'paired_rebound' for t in trials)
    if report['paired_observation']:
        report['definitions'].update(
            reference='paired trial: last 1 second before small press, clipped to after large release confirmation; other trials: last 10 seconds before press',
            paired_large_reference='last 10 seconds before large press, including preceding recovery in same session',
            paired_delay='small press motion_started_monotonic minus large release_zero_confirmed_monotonic',
            paired_residual_before_small='small pre-reference minus large pre-reference; neither reference guarantees full recovery')
    if any(t.get('observation_template') in CONTINUOUS_SHORT_PRESS_TEMPLATES for t in trials):
        report['short_press_observation'] = dict(acceptance_threshold=None,
            reference='last 10 seconds before each press, including preceding trial recovery in same session',
            session_start='reference before first completed press in each uninterrupted recording session',
            after_10s_max='maximum absolute recorded residual in [10,60] seconds; missing samples are not filled')
    # 每次计算使用独立目录；指定输出位置也不得覆盖已有文件夹。
    name = 'analysis_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8]
    output = Path(output_dir) if output_dir else Path(directory) / 'reports' / name
    output.mkdir(parents=True, exist_ok=False)
    for name,rows in [('trials.csv',trials),('channels.csv',channels),('recovery_1s.csv',curves)]:
        if rows:
            with (output/name).open('w',encoding='utf-8-sig',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for row in rows for k in row)));writer.writeheader();writer.writerows(rows)
    (output/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    _write_report(output,report)
    # 最后写入完成标记，历史列表不会把中途失败的输出当成完整报告。
    (output/'complete.json').write_text(json.dumps(dict(
        generated_at=report['generated_at'], analysis_version=ANALYSIS_VERSION,
        report_type='rebound_metrics', source_session=str(Path(directory).resolve())),
        ensure_ascii=False, indent=2), encoding='utf-8')
    return dict(output_dir=str(output.resolve()),report=report)


def _write_report(output, report):
    short_press = bool(report.get('short_press_observation'))
    def fmt(x):
        if x is None: return '未确认/缺数据'
        if isinstance(x, tuple): return '<br>'.join(fmt(v) for v in x)
        if isinstance(x, bool): return '是' if x else '否'
        if isinstance(x, int): return str(x)
        return f'{x:.2f}' if isinstance(x,(int,float)) else html.escape(str(x))
    def table(headers,rows,sort_keys=None):
        attributes=' id="recovery-table"' if sort_keys else ''
        head=''.join(f'<th data-sort="{sort_keys[i]}"><button type="button" class="sort-header">{h}</button></th>'
                     if sort_keys else f'<th>{h}</th>' for i,h in enumerate(headers))
        return '<div class="table-scroll"><table'+attributes+'><thead><tr>'+head+'</tr></thead><tbody>'+''.join(
            '<tr>'+''.join(f'<td>{fmt(v)}</td>' for v in row)+'</tr>' for row in rows)+'</tbody></table></div>'
    content=f'<h1>Raw 回弹分析：{report["completed_conditions"]} 个条件</h1>'
    content += '<p>报告生成时间：' + html.escape(str(report['generated_at'])) + '；计算版本：' + str(report['analysis_version']) + '。</p>'
    content += '<p>来源测试：' + '；'.join(html.escape(str(s['session'])) for s in report['sessions']) + '。</p>'
    if report.get('calibration_goals'):
        content += '<p><b>回弹系数采集：</b>CH4～7每通道拟合一组k、τ，不区分快慢时间段。前两轮用于拟合，第三轮用于独立检验。本报告仍为实测指标，尚未拟合系数。</p>'
        content += table(['原项号','数据用途'], [[x['planned_trial_index'],
            {'fit':'用于拟合系数','validation':'留出检验预测'}.get(x.get('analysis_role'),'未指定')]
            for x in report['trials']])
    content+='<p>有效采样 '+str(sum(s['sample_count'] for s in report['sessions']))+' 帧；独立会话 '+str(len(report['sessions']))+' 段。记录器零丢弃不代表传感器逐帧无损，详见实际采样率及间断统计。</p>'
    if report.get('paired_observation'):
        content += '<h2>大压力之后测小压力：5秒与60秒间隔对比</h2><p>下方主表均描述第二次2 N按压。大压力参考取20 N按压前最后10秒；小压力参考只取2 N发令前最后1秒，且必须在20 N确认卸载之后。两者都不保证材料已恢复。保持读数取实际保持段中位值。'
        content += '按压前残留＝小压力前Raw－大压力前Raw；小压力增量＝小压力保持Raw－小压力前Raw。间隔中的自然回弹仍会影响这个增量，不能直接当作纯2 N响应。末项松开后直接结束，因此没有该项的后续恢复曲线。</p>'
        content += table(['原项号','轮次','通道','计划间隔s','实际间隔s','大压力实测N','小压力实测N',
                          '大压力前Raw','大压力保持Raw','小压力前Raw','按压前残留counts','小压力保持Raw','小压力增量counts'],
            [[r.get(k) for k in ('planned_trial_index','repeat','channel','delay_requested_s','delay_actual_s',
                                'large_hold_median_n','hold_median_n','large_pre_raw','large_hold_raw_median','pre_raw',
                                'residual_before_small_counts','hold_raw_median','hold_response_counts')]
             for r in report['channels']])
        content += '<p>各会话独立计时；恢复表t=0为小压力确认卸载时刻，残差相对小压力前最后1秒参考。所有时间单位为秒，Raw单位为counts。</p>'
    else:
        content+='<p>各会话独立计时；t=0为确认卸载时刻。参考值是按压前最后10秒Raw中位数，不保证材料已经完全恢复。所有时间单位为秒，Raw单位为counts。</p>'
    content+='<h2>实测力与时间</h2>'+table(['原项号','目标N','计划保持s','实际保持s','保持中位N','峰值N','力积分N·s','按压前°C','末尾°C','质量标记'],[
        [r[k] for k in ('planned_trial_index','target_n','hold_requested_s','hold_actual_s','hold_median_n','load_peak_n','force_integral_ns','temperature_pre_c','temperature_tail_c','quality_issues')] for r in report['trials']])
    content+='<h2>四通道恢复</h2><p>Raw返回：连续5个完整一秒窗的中位数在容差内；报告确认时刻。末尾残差取松开后记录的最后30秒（记录600秒时，约为570～600秒）。首次返回后可能再次离带，不能据此宣称永久恢复。</p>'
    if short_press:
        content += '<p><b>短按后恢复：</b>重点比较10秒残差，同时查看10～60秒最大绝对残差，避免把瞬间接近零当作持续恢复。暂未设置合格标准。'
        content += '后续按压的参考取前一次恢复末10秒；“本段起点”是同一连续记录段首个完整项目的按压前参考。'
        content += '按压前相对起点偏移及10秒相对起点偏移，用于观察累积残留。续测重新建立本段起点；数据有缺口时最大值只代表已记录样本。</p>'
    content+='<p><b>按压Raw中位值</b>取电机到位后、释放发令前的最后最多10秒；保持1秒就取这1秒。'
    content+='它与按压前参考不同。<b>按压增量＝按压Raw中位值－按压前参考</b>。'
    content+='占比栏中：<b>Raw%</b>＝残差÷按压Raw中位值×100%；<b>增量%</b>＝残差÷按压增量×100%，表示相对按压响应还剩多少。'
    content+='负数表示低于按压前参考。分母缺数据、为零或增量接近噪声时不计算百分比；表格可横向滚动。</p>'
    headers=['原项号','通道','施压条件','按压Raw中位值<br>counts','按压增量<br>counts','取值窗口<br>s']
    residual_columns=[('10s','residual_10s'),('60s','residual_60s'),('300s','residual_300s')]
    if short_press:
        residual_columns=[(f'{s}s',f'residual_{s}s') for s in (1,5,10,30,60)]
    if any(t['window_s']>=599.5 for t in report['trials']):
        residual_columns.append(('600s','residual_600s'))
    residual_columns.append(('末尾','tail_residual_counts'))
    for title,_ in residual_columns:
        headers += [title+'残差<br>counts',title+'占比']
    headers += ['IDLE确认<br>s','±2返回确认<br>s','±5返回确认<br>s','±2返回后再离带','末尾上漂']
    short_columns = [('轮次','repeat'),('按压前相对本段起点偏移<br>counts','pre_shift_from_session_start_counts'),
                     ('10秒相对本段起点偏移<br>counts','residual_10s_from_session_start_counts'),
                     ('10～60秒最大绝对残差<br>counts','maximum_abs_residual_10_to_60s_counts'),
                     ('10～60秒最大绝对残差<br>占按压增量%','maximum_abs_residual_10_to_60s_percent_of_hold_response')] if short_press else []
    headers += [label for label,key in short_columns]
    paired_columns = [('轮次','repeat'),('大压力后计划间隔<br>s','delay_requested_s'),
                      ('大压力后实际间隔<br>s','delay_actual_s'),('小压力前残留<br>counts','residual_before_small_counts')] if report.get('paired_observation') else []
    headers += [label for label,key in paired_columns]
    rows=[]
    for r in report['channels']:
        condition=(f"目标 {r['target_n']:g} N · 保持 {r['hold_requested_s']:g} s",
                   f"实测中位 {fmt(r['hold_median_n'])} N · 实际保持 {r['hold_actual_s']:.2f} s")
        row=[r['planned_trial_index'],r['channel'],condition,r['hold_raw_median'],r['hold_response_counts'],r['hold_raw_window_s']]
        for _,key in residual_columns:
            def pct(suffix):
                value=r[key+suffix]
                return '不计算' if value is None else f'{value:.2f}%'
            row += [r[key],('Raw: '+pct('_percent_of_hold_raw'),'增量: '+pct('_percent_of_hold_response'))]
        row += [r['confirmed_idle_1s'],r['confirmed_raw_2counts_5s'],r['confirmed_raw_5counts_5s'],r['left_band_after_2counts'],r['tail_rising']]
        row += [r.get(key) for label,key in short_columns]
        row += [r.get(key) for label,key in paired_columns]
        rows.append(row)
    content+=filter_controls(report['channels'])
    sort_keys=['planned_trial_index','channel','target_n','hold_raw_median','hold_response_counts','hold_raw_window_s']
    for _,key in residual_columns:
        sort_keys += [key,key+'_percent_of_hold_response']
    sort_keys += ['confirmed_idle_1s','confirmed_raw_2counts_5s','confirmed_raw_5counts_5s','left_band_after_2counts','tail_rising']
    sort_keys += [key for label,key in short_columns]
    sort_keys += [key for label,key in paired_columns]
    content+=table(headers,rows,sort_keys)
    content+='<p>模型a、p、τ尚未拟合和验证：需要整轮留出验证，并处理实际非恒力、漂移与中断初始残余。计算口径、原始文件SHA256及采样质量见analysis.json；明细见trials.csv、channels.csv、recovery_1s.csv。</p>'
    (output/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Raw回弹分析</title><style>body{font:16px sans-serif;margin:24px;color:#172536}.table-scroll{overflow-x:auto;margin-bottom:24px}table{border-collapse:collapse}td,th{border:1px solid #bbc8d5;padding:8px;text-align:right;white-space:nowrap}th{background:#e9f1f8}h1{font-size:26px}'+REPORT_CSS+'</style>'+content+interaction_script(report['channels']),encoding='utf-8')


def _slope(t, values):
    valid=np.isfinite(t)&np.isfinite(values)
    return float(np.polyfit(t[valid],values[valid],1)[0]) if np.count_nonzero(valid)>2 else None
