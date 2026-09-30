"""自动标定的配对节点、误差验收和历史重点点位；不操作硬件。"""
import bisect
import math
import statistics

from force_signal_pressure_calibration import _segments_from_nodes
from force_signal_pressure_lookup import _validate_mapping_document

DEFAULT_ACCURACY_PERCENT = 2.0
MIN_ERROR_N = .05
WARMUP_MIN = 5
WARMUP_MAX = 15
WARMUP_HOLD_S = 5.0
WARMUP_REST_S = 10.0
WARMUP_SPREAD_PERCENT = 1.0
MAX_RETEST_PASSES = 3
RETEST_REPEATS = 3


def validate_accuracy(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not .5 <= value <= 10:
        raise ValueError('验收误差须为0.5～10%（低力点至少允许±0.05 N）')
    return float(value)


def paired_mapping(groups, metadata):
    """取最接近Signal中位数的一次真实配对，绝不拼接两次按压的坐标。"""
    nodes = []
    for target, entries in sorted(groups.items()):
        if not entries:
            raise ValueError(f'{target:g} N没有有效配对数据')
        for entry in entries:
            if entry.get('floating_count') != 4 or any(
                    not math.isfinite(entry[k]) or entry[k] <= 0 for k in ('signal_sum', 'final_force_n')):
                raise ValueError('标定值无效或通道未全部FLOATING')
        signals = [e['signal_sum'] for e in entries]
        forces = [e['final_force_n'] for e in entries]
        sm, pm = statistics.median(signals), statistics.median(forces)
        selected = min(entries, key=lambda e: (abs(e['signal_sum']-sm), abs(e['final_force_n']-pm)))
        signal, force = selected['signal_sum'], selected['final_force_n']
        nodes.append(dict(nominal_target_force_n=target, sample_count=len(entries),
            signal_sum_median=signal, signal_sum_minimum=min(signals), signal_sum_maximum=max(signals),
            measured_pressure_median_n=force, measured_pressure_minimum_n=min(forces),
            measured_pressure_maximum_n=max(forces), k_n_per_signal_count=force/signal,
            selected_record=dict(captured_at=selected.get('captured_at'), round=selected.get('round'),
                                 stage=selected.get('stage'), pass_index=selected.get('pass_index'))))
    if any(b['signal_sum_median'] <= a['signal_sum_median'] or
           b['measured_pressure_median_n'] <= a['measured_pressure_median_n'] for a, b in zip(nodes, nodes[1:])):
        raise ValueError('配对节点不严格递增，需要重新测试，不能强行排序或拼接坐标')
    metadata = dict(metadata, aggregation='paired_representative',
        node_field_semantics='legacy_median_field_names_contain_selected_actual_pair',
        source_row_count=sum(len(rows) for rows in groups.values()),
        minimum_calibrated_pressure_n=nodes[0]['measured_pressure_median_n'],
        maximum_calibrated_pressure_n=nodes[-1]['measured_pressure_median_n'],
        extrapolation='disabled', firmware_extrapolation='endpoint_linear_unvalidated')
    return _validate_mapping_document(dict(metadata=metadata, nodes=nodes, segments=_segments_from_nodes(nodes)))


def predict_pressure(document, signal):
    """验收按MCU端点线性外推复算，不用截断隐藏表外误差。"""
    nodes = document['nodes']
    values = [n['signal_sum_median'] for n in nodes]
    upper = max(1, min(len(nodes)-1, bisect.bisect_left(values, signal)))
    a, b = nodes[upper-1], nodes[upper]
    return a['measured_pressure_median_n']+(signal-a['signal_sum_median'])*(
        b['measured_pressure_median_n']-a['measured_pressure_median_n'])/(
        b['signal_sum_median']-a['signal_sum_median'])


def assess_mapping(document, records, accuracy_percent):
    validate_accuracy(accuracy_percent)
    checks = []
    for entry in records:
        force = entry['final_force_n']
        predicted = predict_pressure(document, entry['signal_sum'])
        allowed = max(MIN_ERROR_N, force*accuracy_percent/100)
        checks.append(dict(target_force_n=entry['target_force_n'], round=entry.get('round'),
            measured_n=force, predicted_n=predicted, error_n=predicted-force,
            allowed_error_n=allowed, passed=abs(predicted-force) <= allowed+1e-9,
            outside_nodes=not document['nodes'][0]['signal_sum_median'] <= entry['signal_sum'] <= document['nodes'][-1]['signal_sum_median']))
    failed = sorted({c['target_force_n'] for c in checks if not c['passed']})
    return dict(passed=bool(checks) and not failed, failed_targets=failed, checks=checks,
                max_error_n=max((abs(c['error_n']) for c in checks), default=0))


def warmup_convergence(records):
    if len(records) < WARMUP_MIN:
        return dict(passed=False, reason=f'热身至少{WARMUP_MIN}次', spread_percent=None)
    recent = records[-3:]
    normalized = [r['signal_sum']/r['final_force_n'] for r in recent]
    spread = 100*(max(normalized)-min(normalized))/statistics.median(normalized)
    pressures = [r['final_force_n'] for r in recent]
    pressure_ok = max(pressures)-min(pressures) <= .6
    return dict(passed=spread <= WARMUP_SPREAD_PERCENT and pressure_ok,
                spread_percent=spread, pressure_span_n=max(pressures)-min(pressures),
                reason=f'最近3次Signal/实测压力跨度{spread:.2f}%（要求≤{WARMUP_SPREAD_PERCENT:g}%），压力跨度{max(pressures)-min(pressures):.2f} N')


def historical_focus(document, records, targets, accuracy_percent):
    """历史仅决定复测优先级，不把旧Signal混入本次节点。"""
    assessment = assess_mapping(document, records, accuracy_percent)
    errors = {}
    for row in assessment['checks']:
        if not row['passed']:
            errors[row['target_force_n']] = max(errors.get(row['target_force_n'], 0), abs(row['error_n']))
    return sorted({min(targets, key=lambda p: abs(p-target))
                   for target in sorted(errors, key=errors.get, reverse=True)[:5]})
