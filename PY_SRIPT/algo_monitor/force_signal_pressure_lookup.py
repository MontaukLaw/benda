"""读取Signal合力－压力分段查找表并执行线性插值。"""

from __future__ import annotations

import bisect
import json
import math
import os
import sys
from pathlib import Path


EXPECTED_FEATURE = "ch4567_signal_sum"
EXPECTED_PRESSURE_REFERENCE = "final_force_n"


class ForceSignalPressureLookupError(ValueError):
    pass


def signal_pressure_mapping_directory():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "force_signal_pressure_maps"
    return Path(__file__).resolve().with_name("force_signal_pressure_maps")


def signal_pressure_latest_mapping_path():
    external_path = (
        signal_pressure_mapping_directory()
        / "force_signal_pressure_map_latest.json"
    )
    if not getattr(sys, "frozen", False) or external_path.is_file():
        return external_path
    bundle_root = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    return (
        bundle_root
        / "force_signal_pressure_maps"
        / "force_signal_pressure_map_latest.json"
    )


def _finite_float(value, field_name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ForceSignalPressureLookupError(
            f"Signal压力查找表字段{field_name}无效"
        ) from exc
    if not math.isfinite(result):
        raise ForceSignalPressureLookupError(
            f"Signal压力查找表字段{field_name}不是有限数值"
        )
    return result


def _validate_mapping_document(document):
    if not isinstance(document, dict):
        raise ForceSignalPressureLookupError("Signal压力查找表格式无效")
    metadata = document.get("metadata")
    if not isinstance(metadata, dict):
        raise ForceSignalPressureLookupError("Signal压力查找表缺少元数据")
    if (
        int(metadata.get("format_version", 0) or 0) != 2
        or metadata.get("force_unit") != "N"
    ):
        raise ForceSignalPressureLookupError(
            "Signal压力查找表不是当前 N 单位格式，请重新执行传感器标定"
        )
    if metadata.get("feature") != EXPECTED_FEATURE:
        raise ForceSignalPressureLookupError("Signal压力查找表不是CH4～CH7合力格式")
    if metadata.get("pressure_reference") != EXPECTED_PRESSURE_REFERENCE:
        raise ForceSignalPressureLookupError("Signal压力查找表不是压力计实测压力标定")

    nodes = document.get("nodes")
    segments = document.get("segments")
    if not isinstance(nodes, list) or len(nodes) < 2:
        raise ForceSignalPressureLookupError("Signal压力查找表至少需要两个标定点")
    if not isinstance(segments, list) or len(segments) != len(nodes) - 1:
        raise ForceSignalPressureLookupError("Signal压力查找表分段数量无效")

    previous_signal = None
    previous_pressure = None
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            raise ForceSignalPressureLookupError("Signal压力查找表包含无效标定点")
        signal_sum = _finite_float(
            node.get("signal_sum_median"),
            f"nodes[{index}].signal_sum_median",
        )
        pressure_n = _finite_float(
            node.get("measured_pressure_median_n"),
            f"nodes[{index}].measured_pressure_median_n",
        )
        if previous_signal is not None and signal_sum <= previous_signal:
            raise ForceSignalPressureLookupError("Signal压力标定点没有严格递增")
        if previous_pressure is not None and pressure_n <= previous_pressure:
            raise ForceSignalPressureLookupError("压力计实测压力标定点没有严格递增")
        previous_signal = signal_sum
        previous_pressure = pressure_n

    for index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            raise ForceSignalPressureLookupError("Signal压力查找表包含无效分段")
        lower_node = nodes[index]
        upper_node = nodes[index + 1]
        lower_signal = float(lower_node["signal_sum_median"])
        upper_signal = float(upper_node["signal_sum_median"])
        lower_pressure = float(lower_node["measured_pressure_median_n"])
        upper_pressure = float(upper_node["measured_pressure_median_n"])
        expected_k = (upper_pressure - lower_pressure) / (
            upper_signal - lower_signal
        )
        expected_b = lower_pressure - expected_k * lower_signal
        stored_k = _finite_float(
            segment.get("k_n_per_signal_count"),
            f"segments[{index}].k_n_per_signal_count",
        )
        stored_b = _finite_float(
            segment.get("intercept_n"),
            f"segments[{index}].intercept_n",
        )
        if (
            abs(stored_k - expected_k) > 1e-10
            or abs(stored_b - expected_b) > 1e-6
        ):
            raise ForceSignalPressureLookupError(
                f"Signal压力查找表第{index + 1}段K值与标定点不一致"
            )
    return document


def load_latest_signal_pressure_document():
    path = signal_pressure_latest_mapping_path()
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ForceSignalPressureLookupError("尚未生成Signal合力－压力查找表") from exc
    except (OSError, ValueError, TypeError) as exc:
        raise ForceSignalPressureLookupError(
            f"读取Signal合力－压力查找表失败：{exc}"
        ) from exc
    return _validate_mapping_document(document)


def estimate_pressure_from_signal_sum(document, signal_sum):
    """按相邻标定点的K值返回Signal合力对应的分段线性压力。"""
    signal_sum = _finite_float(signal_sum, "signal_sum")
    nodes = document["nodes"]
    calibrated_signals = [float(node["signal_sum_median"]) for node in nodes]
    minimum_signal = calibrated_signals[0]
    maximum_signal = calibrated_signals[-1]
    if signal_sum < minimum_signal:
        return {
            "pressure_n": None,
            "range_state": "below",
            "boundary_pressure_n": float(
                nodes[0]["measured_pressure_median_n"]
            ),
            "boundary_signal_sum": minimum_signal,
        }
    if signal_sum > maximum_signal:
        return {
            "pressure_n": None,
            "range_state": "above",
            "boundary_pressure_n": float(
                nodes[-1]["measured_pressure_median_n"]
            ),
            "boundary_signal_sum": maximum_signal,
        }

    segment_index = min(
        len(document["segments"]) - 1,
        max(0, bisect.bisect_right(calibrated_signals, signal_sum) - 1),
    )
    segment = document["segments"][segment_index]
    k_value = float(segment["k_n_per_signal_count"])
    intercept = float(segment["intercept_n"])
    return {
        "pressure_n": k_value * signal_sum + intercept,
        "range_state": "in_range",
        "segment_index": segment_index,
        "k_n_per_signal_count": k_value,
        "lower_signal_sum": calibrated_signals[segment_index],
        "upper_signal_sum": calibrated_signals[segment_index + 1],
    }
