"""从最新完整传感器标定记录生成Signal合力－压力分段表。"""

from __future__ import annotations

import csv
import copy
import json
import math
import os
import statistics
import sys
import tempfile
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

from config import FORCE_CALIBRATION_MAX_POINTS
from force_signal_pressure_lookup import signal_pressure_mapping_directory


class ForceSignalPressureCalibrationError(ValueError):
    pass


# 快调同时覆盖低力端、2 N交界和30 N高力端，避免只测高力再修正整张表。
QUICK_CORRECTION_TARGETS_N = (0.1, 0.5, 1.0, 1.5, 2.0) + tuple(float(value) for value in range(4, 31, 2))
# 两段共用2 N节点；使用整数生成低力档，避免浮点步进累积误差。
FORCE_CALIBRATION_LOW_TARGETS_N = tuple(value / 10.0 for value in range(1, 21))
FORCE_CALIBRATION_HIGH_TARGETS_N = tuple(float(value) for value in range(2, 31))
FORCE_CALIBRATION_TARGETS_N = FORCE_CALIBRATION_LOW_TARGETS_N + FORCE_CALIBRATION_HIGH_TARGETS_N[1:]
FORCE_CALIBRATION_PROFILE = "low_0p1_to_2_step_0p1_high_2_to_30_step_1"
QUICK_CORRECTION_MAX_ADJUSTMENT_RATIO = 0.25
QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT = (
    f"±{QUICK_CORRECTION_MAX_ADJUSTMENT_RATIO * 100.0:g}%"
)
QUICK_CORRECTION_NODE_CHANGE_TOLERANCE_COUNTS = 0.01
# 快调边界允许按标定实测跨度的2%有限外推，避免端点轻微漂移导致整轮作废。
QUICK_CORRECTION_EDGE_EXTRAPOLATION_RATIO = 0.02
QUICK_CORRECTION_EDGE_EXTRAPOLATION_MIN_N = 0.10
# 快调目标位于K表端点外时，再为名义目标保留1%的压力计读数余量。
QUICK_CORRECTION_TARGET_EDGE_MARGIN_RATIO = 0.01
QUICK_CORRECTION_TARGET_EDGE_MARGIN_MIN_N = 0.10
FIRMWARE_FORCE_CALIBRATION_MAX_POINTS = FORCE_CALIBRATION_MAX_POINTS


def high_force_calibration_targets(count):
    """1～30 N近似等间距整数点，供计划、采集和K表校验共用。"""
    if type(count) is not int or count not in (10, 20):
        raise ValueError('标定点数只能为10或20')
    return tuple(round(1+29*i/(count-1)) for i in range(count))


def select_firmware_calibration_nodes(
    nodes,
    maximum_points=FIRMWARE_FORCE_CALIBRATION_MAX_POINTS,
):
    """验证并返回两段共用2 N节点的完整48节点力标定表。"""
    nodes = tuple(nodes or ())
    maximum_points = int(maximum_points)
    if maximum_points != FIRMWARE_FORCE_CALIBRATION_MAX_POINTS:
        raise ForceSignalPressureCalibrationError("两段力标定节点容量必须统一为48")
    if len(nodes) != FIRMWARE_FORCE_CALIBRATION_MAX_POINTS:
        raise ForceSignalPressureCalibrationError(
            f"力标定表必须恰好包含{FIRMWARE_FORCE_CALIBRATION_MAX_POINTS}个节点，"
            "对应0.1～2 N每0.1 N及3～30 N每1 N；0 N由MCU虚拟原点处理"
        )

    signal_points = tuple(
        _finite_float(node.get("signal_sum_median"), "signal_sum_median")
        for node in nodes
    )
    pressure_points = tuple(
        _finite_float(
            node.get("measured_pressure_median_n"),
            "measured_pressure_median_n",
        )
        for node in nodes
    )
    if signal_points[0] <= 0.0 or pressure_points[0] <= 0.0:
        raise ForceSignalPressureCalibrationError("首个0.1 N实测节点必须有正的Signal和压力，不能以零信号生成K表")
    for index in range(1, len(nodes)):
        if signal_points[index] <= signal_points[index - 1]:
            raise ForceSignalPressureCalibrationError(
                "MCU力标定节点的Signal必须严格递增"
            )
        if pressure_points[index] < pressure_points[index - 1]:
            raise ForceSignalPressureCalibrationError(
                "MCU力标定节点的压力必须单调递增"
            )

    for index, node in enumerate(nodes):
        nominal_target = node.get("nominal_target_force_n")
        expected_target = FORCE_CALIBRATION_TARGETS_N[index]
        if nominal_target is None or not math.isclose(
            _finite_float(nominal_target, "nominal_target_force_n"),
            expected_target,
            abs_tol=1e-6,
        ):
            raise ForceSignalPressureCalibrationError(
                "力标定表须按0.1～2 N每0.1 N、3～30 N每1 N排列，共48节点"
            )

    return {
        "indices": tuple(range(FIRMWARE_FORCE_CALIBRATION_MAX_POINTS)),
        "nodes": nodes,
        "source_point_count": len(nodes),
        "maximum_error_n": 0.0,
        "selection_source": "two_range_48_nodes",
    }


def select_firmware_calibration_document_nodes(
    document,
    maximum_points=FIRMWARE_FORCE_CALIBRATION_MAX_POINTS,
):
    """返回完整高力10/20节点或原两段48节点表，不压缩实测节点。"""
    if not isinstance(document, dict) or not isinstance(document.get("nodes"), list):
        raise ForceSignalPressureCalibrationError("当前K值表格式无效")
    if document.get('metadata', {}).get('calibration_profile') == 'auto_1_to_30_n':
        from force_signal_pressure_lookup import _validate_mapping_document
        try:
            _validate_mapping_document(document)
            nodes = document['nodes']
            if len(nodes) not in (10, 20) or maximum_points < len(nodes):
                raise ValueError('自动标定仅支持10或20点')
            if (not .9 <= nodes[0]['measured_pressure_median_n'] <= 1.1
                    or not 27 <= nodes[-1]['measured_pressure_median_n'] <= 33):
                raise ValueError('自动标定实测首末点须分别位于1 N、30 N的±10%内')
            targets = tuple(float(node['nominal_target_force_n']) for node in nodes)
            legacy_targets = tuple(round(1+29*i/(len(nodes)-1), 2) for i in range(len(nodes)))
            # 完整旧小数计划仍可回读和写入；禁止混合新旧点位拼成一张表。
            if targets not in (high_force_calibration_targets(len(nodes)), legacy_targets):
                raise ValueError('自动标定节点须符合完整整数计划或原小数计划')
            for node in nodes:
                if (float(node['signal_sum_median']) <= 0 or float(node['measured_pressure_median_n']) <= 0):
                    raise ValueError('自动标定节点须覆盖1～30 N的完整计划且实测值为正')
        except (ValueError, TypeError, KeyError) as exc:
            raise ForceSignalPressureCalibrationError(str(exc)) from exc
        return dict(indices=tuple(range(len(nodes))), nodes=tuple(nodes), source_point_count=len(nodes),
                    maximum_error_n=0.0, selection_source='auto_high_force_nodes')
    return select_firmware_calibration_nodes(document["nodes"], maximum_points)


def sensor_calibration_data_directory():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "force_press_tests"
    return Path(__file__).resolve().with_name("force_press_tests")


def _finite_float(value, field_name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ForceSignalPressureCalibrationError(
            f"标定记录字段{field_name}无效"
        ) from exc
    if not math.isfinite(result):
        raise ForceSignalPressureCalibrationError(
            f"标定记录字段{field_name}不是有限数值"
        )
    return result


def _normalize_calibration_entries(entries, source_name):
    normalized = []
    completed_keys = set()
    expected_total = None
    for raw_entry in entries:
        if not isinstance(raw_entry, dict):
            raise ForceSignalPressureCalibrationError(
                f"{source_name}中存在无效标定轮次"
            )
        if str(raw_entry.get("control_mode", "sweep")) != "sweep":
            continue
        if raw_entry.get("force_unit") != "N":
            raise ForceSignalPressureCalibrationError(
                f"{source_name}中存在非 N 单位标定轮次"
            )
        try:
            level_index = int(raw_entry["pressure_level_index"])
            repeat_index = int(raw_entry["repeat_index"])
            round_total = int(raw_entry["auto_round_total"])
            sweep_direction = str(raw_entry.get("sweep_direction", "")).strip()
            calibration_cycle_index = int(
                raw_entry.get("calibration_cycle_index", 0) or 0
            )
            calibration_cycle_total = int(
                raw_entry.get("calibration_cycle_total", 0) or 0
            )
            target_force = _finite_float(
                raw_entry["target_force_n"],
                "target_force_n",
            )
            final_force = _finite_float(
                raw_entry["final_force_n"],
                "final_force_n",
            )
            signal_sum = _finite_float(
                raw_entry.get("signal_sum", raw_entry.get("ch4567_signal_sum")),
                "ch4567_signal_sum",
            )
            floating_count = int(raw_entry["floating_count"])
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ForceSignalPressureCalibrationError(
                f"{source_name}中存在不完整标定轮次"
            ) from exc
        if level_index <= 0 or repeat_index <= 0 or round_total <= 0:
            raise ForceSignalPressureCalibrationError(
                f"{source_name}中的档位、轮次或总轮数无效"
            )
        if floating_count != 4:
            raise ForceSignalPressureCalibrationError(
                f"{source_name}中存在未达到CH4～CH7全部FLOATING的结果"
            )
        if expected_total is None:
            expected_total = round_total
        elif expected_total != round_total:
            raise ForceSignalPressureCalibrationError(
                f"{source_name}混有不同的传感器标定计划"
            )
        if sweep_direction:
            if (
                sweep_direction not in ("ascending", "descending")
                or calibration_cycle_index <= 0
                or calibration_cycle_total <= 0
                or calibration_cycle_index > calibration_cycle_total
            ):
                raise ForceSignalPressureCalibrationError(
                    f"{source_name}中的往返轮次或升降方向无效"
                )
            key = (
                calibration_cycle_index,
                sweep_direction,
                level_index,
            )
        else:
            key = (level_index, repeat_index)
        if key in completed_keys:
            raise ForceSignalPressureCalibrationError(
                f"{source_name}中存在重复的档位轮次"
            )
        completed_keys.add(key)
        normalized.append(
            {
                "captured_at": str(raw_entry.get("captured_at", "")),
                "operation_id": str(raw_entry.get("operation_id", "")),
                "pressure_level_index": level_index,
                "repeat_index": repeat_index,
                "calibration_cycle_index": calibration_cycle_index,
                "calibration_cycle_total": calibration_cycle_total,
                "sweep_direction": sweep_direction,
                "target_force_n": target_force,
                "final_force_n": final_force,
                "signal_sum": signal_sum,
                "floating_count": floating_count,
            }
        )
    if not normalized:
        raise ForceSignalPressureCalibrationError(
            f"{source_name}中没有传感器标定结果"
        )
    if len(normalized) != expected_total:
        raise ForceSignalPressureCalibrationError(
            f"{source_name}尚未完成：已有{len(normalized)}/{expected_total}轮"
        )
    bidirectional_entries = [
        entry for entry in normalized if entry["sweep_direction"]
    ]
    if bidirectional_entries:
        if len(bidirectional_entries) != len(normalized):
            raise ForceSignalPressureCalibrationError(
                f"{source_name}混有旧版与双向往返标定轮次"
            )
        cycle_totals = {
            entry["calibration_cycle_total"] for entry in normalized
        }
        level_targets = {}
        for entry in normalized:
            level_index = entry["pressure_level_index"]
            target_force = entry["target_force_n"]
            previous_target = level_targets.setdefault(level_index, target_force)
            if not math.isclose(previous_target, target_force, abs_tol=1e-9):
                raise ForceSignalPressureCalibrationError(
                    f"{source_name}中同一档位混有不同目标压力"
                )
        if len(cycle_totals) != 1:
            raise ForceSignalPressureCalibrationError(
                f"{source_name}混有不同的往返轮数"
            )
        cycle_total = next(iter(cycle_totals))
        level_indices = tuple(sorted(level_targets))
        expected_level_indices = tuple(range(1, len(level_indices) + 1))
        expected_keys = {
            (cycle_index, direction, level_index)
            for cycle_index in range(1, cycle_total + 1)
            for direction in ("ascending", "descending")
            for level_index in level_indices
        }
        if level_indices != expected_level_indices or completed_keys != expected_keys:
            raise ForceSignalPressureCalibrationError(
                f"{source_name}没有完整覆盖每轮升序和降序的全部压力档位"
            )
    return normalized


def _load_checkpoint_candidate(path):
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError) as exc:
        raise ForceSignalPressureCalibrationError(
            f"读取恢复记录失败：{exc}"
        ) from exc
    if (
        not isinstance(document, dict)
        or document.get("status") != "completed"
        or document.get("format_version") not in (2, 3, 4)
        or document.get("force_unit") != "N"
    ):
        raise ForceSignalPressureCalibrationError("恢复记录不是已完成的传感器标定")
    entries = _normalize_calibration_entries(
        document.get("entries") or (),
        path.name,
    )
    return {
        "path": path,
        "source_type": "checkpoint",
        "entries": entries,
        "modified_at": path.stat().st_mtime,
    }


def _load_csv_candidate(path):
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as handle:
            rows = list(csv.DictReader(handle))
    except (OSError, csv.Error) as exc:
        raise ForceSignalPressureCalibrationError(
            f"读取{path.name}失败：{exc}"
        ) from exc
    sweep_rows = [row for row in rows if row.get("control_mode") == "sweep"]
    if not sweep_rows:
        raise ForceSignalPressureCalibrationError(
            f"{path.name}中没有传感器标定结果"
        )
    operation_groups = defaultdict(list)
    for row in sweep_rows:
        operation_groups[str(row.get("operation_id", ""))].append(row)
    valid_groups = []
    try:
        valid_groups.append(
            _normalize_calibration_entries(
                sweep_rows,
                path.name,
            )
        )
    except ForceSignalPressureCalibrationError:
        pass
    for operation_id, operation_rows in operation_groups.items():
        try:
            entries = _normalize_calibration_entries(
                operation_rows,
                f"{path.name}的操作{operation_id or '--'}",
            )
        except ForceSignalPressureCalibrationError:
            continue
        valid_groups.append(entries)
    if not valid_groups:
        raise ForceSignalPressureCalibrationError(
            f"{path.name}中没有完整的传感器标定批次"
        )
    entries = max(
        valid_groups,
        key=lambda group: max(entry["captured_at"] for entry in group),
    )
    return {
        "path": path,
        "source_type": "csv",
        "entries": entries,
        "modified_at": path.stat().st_mtime,
    }


def load_latest_complete_sensor_calibration():
    data_directory = sensor_calibration_data_directory()
    if not data_directory.is_dir():
        raise ForceSignalPressureCalibrationError("尚未生成传感器标定数据目录")
    candidates = []
    checkpoint_path = data_directory / "force_sweep_checkpoint_latest.json"
    if checkpoint_path.is_file():
        try:
            candidates.append(_load_checkpoint_candidate(checkpoint_path))
        except ForceSignalPressureCalibrationError:
            pass
    csv_paths = set()
    for pattern in (
        "force_sensor_calibration_results_*.csv",
        "force_sweep_results_*.csv",
        "force_sensor_results_*.csv",
    ):
        csv_paths.update(data_directory.glob(pattern))
    for path in csv_paths:
        try:
            candidates.append(_load_csv_candidate(path))
        except ForceSignalPressureCalibrationError:
            continue
    if not candidates:
        raise ForceSignalPressureCalibrationError(
            "没有找到已经完整结束的传感器标定数据"
        )
    return max(candidates, key=lambda candidate: candidate["modified_at"])


def build_signal_pressure_mapping(entries, source_path, source_type):
    grouped = defaultdict(list)
    for entry in entries:
        grouped[float(entry["target_force_n"])].append(entry)
    grouped_targets = tuple(sorted(grouped))
    if grouped_targets != FORCE_CALIBRATION_TARGETS_N:
        raise ForceSignalPressureCalibrationError(
            "正式K值表需要低力0.1～2 N每0.1 N及高力3～30 N每1 N，共48档；旧20节点记录不能充当新表"
        )

    nodes = []
    for target_force in grouped_targets:
        group = grouped[target_force]
        signals = [float(entry["signal_sum"]) for entry in group]
        pressures = [float(entry["final_force_n"]) for entry in group]
        nodes.append(
            {
                "nominal_target_force_n": round(target_force, 2),
                "sample_count": len(group),
                "signal_sum_median": round(statistics.median(signals), 3),
                "signal_sum_minimum": round(min(signals), 3),
                "signal_sum_maximum": round(max(signals), 3),
                "measured_pressure_median_n": round(statistics.median(pressures), 2),
                "measured_pressure_minimum_n": round(min(pressures), 2),
                "measured_pressure_maximum_n": round(max(pressures), 2),
            }
        )

    segments = []
    for index, (lower_node, upper_node) in enumerate(zip(nodes, nodes[1:])):
        lower_signal = float(lower_node["signal_sum_median"])
        upper_signal = float(upper_node["signal_sum_median"])
        lower_pressure = float(lower_node["measured_pressure_median_n"])
        upper_pressure = float(upper_node["measured_pressure_median_n"])
        if upper_signal <= lower_signal:
            raise ForceSignalPressureCalibrationError(
                f"第{index + 1}～{index + 2}个标定点的Signal合力没有递增"
            )
        if upper_pressure <= lower_pressure:
            raise ForceSignalPressureCalibrationError(
                f"第{index + 1}～{index + 2}个标定点的压力计实测压力没有递增"
            )
        k_value = (upper_pressure - lower_pressure) / (
            upper_signal - lower_signal
        )
        segments.append(
            {
                "lower_nominal_target_force_n": lower_node[
                    "nominal_target_force_n"
                ],
                "upper_nominal_target_force_n": upper_node[
                    "nominal_target_force_n"
                ],
                "lower_signal_sum": lower_signal,
                "upper_signal_sum": upper_signal,
                "k_n_per_signal_count": k_value,
                "intercept_n": lower_pressure - k_value * lower_signal,
            }
        )

    now = datetime.now().astimezone()
    return {
        "metadata": {
            "format_version": 2,
            "calibration_profile": FORCE_CALIBRATION_PROFILE,
            "range_boundary_n": 2.0,
            "low_step_n": 0.1,
            "high_step_n": 1.0,
            "force_unit": "N",
            "created_at": now.isoformat(timespec="seconds"),
            "feature": "ch4567_signal_sum",
            "pressure_reference": "final_force_n",
            "aggregation": "median",
            "required_sensor_state": "CH4_CH7_ALL_FLOATING",
            "source_type": str(source_type),
            "source_path": str(source_path),
            "source_csv": Path(source_path).name,
            "source_row_count": len(entries),
            "source_first_captured_at": min(
                entry["captured_at"] for entry in entries
            ),
            "source_last_captured_at": max(
                entry["captured_at"] for entry in entries
            ),
            "minimum_calibrated_pressure_n": nodes[0][
                "measured_pressure_median_n"
            ],
            "maximum_calibrated_pressure_n": nodes[-1][
                "measured_pressure_median_n"
            ],
            "interpolation": "piecewise_linear",
            "extrapolation": "disabled",
        },
        "nodes": nodes,
        "segments": segments,
    }


def _signal_for_pressure(nodes, pressure_n, target_pressure_n=None):
    """反查指定压力的Signal，并允许目标端点附近的小范围线性外推。"""
    pressure_n = _finite_float(pressure_n, "final_force_n")
    if target_pressure_n is not None:
        target_pressure_n = _finite_float(target_pressure_n, "target_force_n")
    pressures = [float(node["measured_pressure_median_n"]) for node in nodes]
    calibrated_span_n = pressures[-1] - pressures[0]
    edge_tolerance_n = max(
        QUICK_CORRECTION_EDGE_EXTRAPOLATION_MIN_N,
        calibrated_span_n * QUICK_CORRECTION_EDGE_EXTRAPOLATION_RATIO,
    )
    allowed_minimum_n = pressures[0] - edge_tolerance_n
    allowed_maximum_n = pressures[-1] + edge_tolerance_n
    if target_pressure_n is not None:
        target_margin_n = max(
            QUICK_CORRECTION_TARGET_EDGE_MARGIN_MIN_N,
            abs(target_pressure_n) * QUICK_CORRECTION_TARGET_EDGE_MARGIN_RATIO,
        )
        if target_pressure_n < pressures[0]:
            allowed_minimum_n = min(
                allowed_minimum_n,
                target_pressure_n - target_margin_n,
            )
        if target_pressure_n > pressures[-1]:
            allowed_maximum_n = max(
                allowed_maximum_n,
                target_pressure_n + target_margin_n,
            )
    if pressure_n < allowed_minimum_n or pressure_n > allowed_maximum_n:
        raise ForceSignalPressureCalibrationError(
            f"压力计实测值 {pressure_n:.2f} N超出当前K表"
            f" {pressures[0]:.2f}～{pressures[-1]:.2f} N范围，"
            f"本档只允许落在 {allowed_minimum_n:.2f}～"
            f"{allowed_maximum_n:.2f} N"
        )
    upper_index = 1
    while upper_index < len(nodes) - 1 and pressure_n > pressures[upper_index]:
        upper_index += 1
    lower_index = upper_index - 1
    lower_pressure = pressures[lower_index]
    upper_pressure = pressures[upper_index]
    lower_signal = float(nodes[lower_index]["signal_sum_median"])
    upper_signal = float(nodes[upper_index]["signal_sum_median"])
    return lower_signal + (
        (pressure_n - lower_pressure)
        * (upper_signal - lower_signal)
        / (upper_pressure - lower_pressure)
    )


def _interpolate_ratio(samples, pressure_n):
    """按实测压力对单轮修正比例执行端点保持的线性插值。"""
    if pressure_n <= samples[0]["measured_pressure_n"]:
        return samples[0]["applied_ratio"]
    if pressure_n >= samples[-1]["measured_pressure_n"]:
        return samples[-1]["applied_ratio"]
    for lower, upper in zip(samples, samples[1:]):
        lower_pressure = lower["measured_pressure_n"]
        upper_pressure = upper["measured_pressure_n"]
        if pressure_n <= upper_pressure:
            fraction = (pressure_n - lower_pressure) / (
                upper_pressure - lower_pressure
            )
            return lower["applied_ratio"] + fraction * (
                upper["applied_ratio"] - lower["applied_ratio"]
            )
    return samples[-1]["applied_ratio"]


def _segments_from_nodes(nodes):
    """根据修正后的节点重新生成连续的分段K值。"""
    segments = []
    for lower_node, upper_node in zip(nodes, nodes[1:]):
        lower_signal = float(lower_node["signal_sum_median"])
        upper_signal = float(upper_node["signal_sum_median"])
        lower_pressure = float(lower_node["measured_pressure_median_n"])
        upper_pressure = float(upper_node["measured_pressure_median_n"])
        if upper_signal <= lower_signal:
            raise ForceSignalPressureCalibrationError(
                "快速修正后Signal节点不再严格递增，已拒绝应用"
            )
        k_value = (upper_pressure - lower_pressure) / (
            upper_signal - lower_signal
        )
        segments.append(
            {
                "lower_nominal_target_force_n": lower_node[
                    "nominal_target_force_n"
                ],
                "upper_nominal_target_force_n": upper_node[
                    "nominal_target_force_n"
                ],
                "lower_signal_sum": lower_signal,
                "upper_signal_sum": upper_signal,
                "k_n_per_signal_count": k_value,
                "intercept_n": lower_pressure - k_value * lower_signal,
            }
        )
    return segments


def build_quick_corrected_mapping(
    document,
    entries,
    maximum_adjustment_ratio=QUICK_CORRECTION_MAX_ADJUSTMENT_RATIO,
):
    """用两段代表档位的单轮结果生成受原始表累计范围约束的Raw节点修正表。"""
    maximum_adjustment_ratio = _finite_float(
        maximum_adjustment_ratio,
        "maximum_adjustment_ratio",
    )
    if not 0.0 < maximum_adjustment_ratio <= QUICK_CORRECTION_MAX_ADJUSTMENT_RATIO:
        raise ForceSignalPressureCalibrationError(
            "快速修正上限必须处于0～"
            f"{QUICK_CORRECTION_MAX_ADJUSTMENT_RATIO * 100.0:g}%"
        )
    if not isinstance(document, dict) or not isinstance(document.get("nodes"), list):
        raise ForceSignalPressureCalibrationError("当前K值表格式无效")
    current_nodes = document["nodes"]
    if len(current_nodes) < 2:
        raise ForceSignalPressureCalibrationError("当前K值表至少需要两个节点")
    firmware_selection = select_firmware_calibration_document_nodes(document)

    entries_by_target = {}
    for raw_entry in entries or ():
        try:
            target_force = round(
                _finite_float(raw_entry["target_force_n"], "target_force_n"),
                2,
            )
            measured_pressure = _finite_float(
                raw_entry["final_force_n"],
                "final_force_n",
            )
            signal_sum = _finite_float(
                raw_entry.get("signal_sum", raw_entry.get("ch4567_signal_sum")),
                "ch4567_signal_sum",
            )
            floating_count = int(raw_entry["floating_count"])
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ForceSignalPressureCalibrationError(
                "快速修正结果缺少压力、Signal或FLOATING状态"
            ) from exc
        if target_force not in QUICK_CORRECTION_TARGETS_N:
            continue
        if target_force in entries_by_target:
            raise ForceSignalPressureCalibrationError(
                f"快速修正中{target_force:g} N存在重复结果"
            )
        if floating_count != 4 or signal_sum <= 0.0:
            raise ForceSignalPressureCalibrationError(
                f"快速修正中{target_force:g} N未取得有效FLOATING 4/4 Signal"
            )
        entries_by_target[target_force] = {
            "target_force_n": target_force,
            "measured_pressure_n": measured_pressure,
            "observed_signal_sum": signal_sum,
            "captured_at": str(raw_entry.get("captured_at", "")),
        }
    missing = [
        target for target in QUICK_CORRECTION_TARGETS_N
        if target not in entries_by_target
    ]
    if missing:
        raise ForceSignalPressureCalibrationError(
            "快速修正缺少 " + "、".join(f"{target:g} N" for target in missing)
        )

    samples = [entries_by_target[target] for target in QUICK_CORRECTION_TARGETS_N]
    previous_pressure = None
    previous_signal = None
    lower_ratio = 1.0 - maximum_adjustment_ratio
    upper_ratio = 1.0 + maximum_adjustment_ratio
    for sample in samples:
        pressure_n = sample["measured_pressure_n"]
        signal_sum = sample["observed_signal_sum"]
        if previous_pressure is not None and pressure_n <= previous_pressure:
            raise ForceSignalPressureCalibrationError(
                "快速修正的压力计实测值没有随两段目标档位严格递增"
            )
        if previous_signal is not None and signal_sum <= previous_signal:
            raise ForceSignalPressureCalibrationError(
                "快速修正的Signal没有随两段目标档位严格递增"
            )
        expected_signal = _signal_for_pressure(
            current_nodes,
            pressure_n,
            sample["target_force_n"],
        )
        inferred_ratio = signal_sum / expected_signal
        sample["expected_signal_sum"] = expected_signal
        sample["inferred_ratio"] = inferred_ratio
        sample["applied_ratio"] = min(
            upper_ratio,
            max(lower_ratio, inferred_ratio),
        )
        sample["limited"] = not math.isclose(
            sample["applied_ratio"],
            inferred_ratio,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        previous_pressure = pressure_n
        previous_signal = signal_sum

    metadata = document.get("metadata") or {}
    root_signals = metadata.get("quick_correction_root_signal_points")
    if not isinstance(root_signals, list) or len(root_signals) != len(current_nodes):
        root_signals = [
            float(node["signal_sum_median"]) for node in current_nodes
        ]
    else:
        root_signals = [
            _finite_float(value, "quick_correction_root_signal_points")
            for value in root_signals
        ]

    corrected_nodes = copy.deepcopy(current_nodes)
    node_adjustments = []
    for index, node in enumerate(corrected_nodes):
        current_signal = float(node["signal_sum_median"])
        root_signal = float(root_signals[index])
        pressure_n = float(node["measured_pressure_median_n"])
        local_ratio = _interpolate_ratio(samples, pressure_n)
        proposed_signal = current_signal * local_ratio
        corrected_signal = min(
            root_signal * upper_ratio,
            max(root_signal * lower_ratio, proposed_signal),
        )
        cumulative_limited = not math.isclose(
            corrected_signal,
            proposed_signal,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
        effective_ratio = corrected_signal / current_signal
        change_counts = corrected_signal - current_signal
        node["signal_sum_median"] = round(corrected_signal, 3)
        for field_name in ("signal_sum_minimum", "signal_sum_maximum"):
            if field_name in node:
                node[field_name] = round(
                    float(node[field_name]) * effective_ratio,
                    3,
                )
        node_adjustments.append(
            {
                "pressure_n": pressure_n,
                "before_signal_sum": current_signal,
                "after_signal_sum": corrected_signal,
                "effective_ratio": effective_ratio,
                "change_counts": change_counts,
                "change_ratio": effective_ratio - 1.0,
                "changed": (
                    abs(change_counts)
                    > QUICK_CORRECTION_NODE_CHANGE_TOLERANCE_COUNTS
                ),
                "root_ratio": corrected_signal / root_signal,
                "cumulative_limited": cumulative_limited,
            }
        )

    corrected_document = copy.deepcopy(document)
    corrected_document["nodes"] = corrected_nodes
    corrected_document["segments"] = _segments_from_nodes(corrected_nodes)
    corrected_metadata = corrected_document.setdefault("metadata", {})
    now = datetime.now().astimezone()
    corrected_metadata.update(
        {
            "created_at": now.isoformat(timespec="seconds"),
            "source_type": "quick_correction",
            "source_path": "0.1-30 N two-range single-pass RAM trial",
            "source_csv": "",
            "source_row_count": len(samples),
            "source_first_captured_at": min(
                sample["captured_at"] for sample in samples
            ),
            "source_last_captured_at": max(
                sample["captured_at"] for sample in samples
            ),
            "quick_correction_base_created_at": metadata.get(
                "quick_correction_base_created_at",
                metadata.get("created_at", ""),
            ),
            "quick_correction_parent_created_at": metadata.get("created_at", ""),
            "quick_correction_max_adjustment_ratio": maximum_adjustment_ratio,
            "quick_correction_root_signal_points": root_signals,
            "quick_correction_samples": copy.deepcopy(samples),
            "quick_correction_node_adjustments": node_adjustments,
        }
    )
    corrected_metadata.pop("firmware_force_calibration_node_indices", None)
    changed_adjustments = [
        adjustment
        for adjustment in node_adjustments
        if adjustment["changed"]
    ]
    return {
        "document": corrected_document,
        "samples": samples,
        "node_adjustments": node_adjustments,
        "limited_sample_count": sum(sample["limited"] for sample in samples),
        "limited_node_count": sum(
            adjustment["cumulative_limited"] for adjustment in node_adjustments
        ),
        "changed_node_count": len(changed_adjustments),
        "minimum_node_change_counts": (
            min(adjustment["change_counts"] for adjustment in changed_adjustments)
            if changed_adjustments
            else 0.0
        ),
        "maximum_node_change_counts": (
            max(adjustment["change_counts"] for adjustment in changed_adjustments)
            if changed_adjustments
            else 0.0
        ),
        "maximum_absolute_node_change_counts": max(
            (
                abs(adjustment["change_counts"])
                for adjustment in changed_adjustments
            ),
            default=0.0,
        ),
        "maximum_absolute_node_change_ratio": max(
            (
                abs(adjustment["change_ratio"])
                for adjustment in changed_adjustments
            ),
            default=0.0,
        ),
        "firmware_node_indices": firmware_selection["indices"],
    }


def _write_json_atomic(path, document):
    """完整落盘后原子替换；短暂占用重试，失败时保留可恢复的独立副本。"""
    path = Path(path)
    payload = json.dumps(document, ensure_ascii=False, indent=2)
    temporary_path, durable = None, False
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        # 每次保存使用独立名字，避免复用被读取程序或上次写入占用的.tmp文件。
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                prefix=f'.{path.name}.', suffix='.pending.json', delete=False) as handle:
            temporary_path = Path(handle.name)
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        durable = True
        for attempt in range(4):
            try:
                temporary_path.replace(path)
                return
            except OSError as exc:
                if attempt == 3 or not (isinstance(exc, PermissionError)
                        or getattr(exc, 'winerror', None) in (5, 32, 33)):
                    raise
                time.sleep((.05, .1, .2)[attempt])
    except OSError as exc:
        error = ForceSignalPressureCalibrationError(f"保存JSON记录失败：{exc}")
        # 只有已关闭并fsync成功的完整文件才允许自动采集作为备份继续使用。
        error.recovery_path = str(temporary_path) if durable else None
        if temporary_path is not None and not durable:
            try:
                temporary_path.unlink()
            except OSError:
                pass
        raise error from exc


def save_signal_pressure_mapping_document(document, archive_label="quick"):
    """在RAM试用确认后原子保存快速修正表，并保留独立历史文件。"""
    output_directory = signal_pressure_mapping_directory()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    safe_label = "".join(
        character for character in str(archive_label)
        if character.isalnum() or character in ("-", "_")
    ) or "quick"
    archive_path = output_directory / (
        f"force_signal_pressure_map_{safe_label}_{stamp}.json"
    )
    latest_path = output_directory / "force_signal_pressure_map_latest.json"
    _write_json_atomic(archive_path, document)
    _write_json_atomic(latest_path, document)
    return {
        "archive_path": archive_path,
        "latest_path": latest_path,
    }


def build_latest_signal_pressure_mapping():
    source = load_latest_complete_sensor_calibration()
    document = build_signal_pressure_mapping(
        source["entries"],
        source["path"],
        source["source_type"],
    )
    output_directory = signal_pressure_mapping_directory()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    archive_path = output_directory / f"force_signal_pressure_map_{stamp}.json"
    latest_path = output_directory / "force_signal_pressure_map_latest.json"
    _write_json_atomic(archive_path, document)
    _write_json_atomic(latest_path, document)
    return {
        "document": document,
        "source_path": source["path"],
        "archive_path": archive_path,
        "latest_path": latest_path,
    }
