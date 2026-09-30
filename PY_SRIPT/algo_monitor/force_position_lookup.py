"""压力－位置查找表的共享路径、读取、校验和目标位置换算。"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path


MAP_FORCE_STEP_G = 10
MAP_FORCE_STEP_N = 0.1
EXPECTED_MAPPING_MODE = "release_relative_travel"


class ForcePositionLookupError(ValueError):
    pass


def mapping_directory():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "force_position_maps"
    return Path(__file__).resolve().with_name("force_position_maps")


def latest_mapping_path():
    return mapping_directory() / "force_position_map_latest.json"


def load_latest_mapping_document():
    path = latest_mapping_path()
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ForcePositionLookupError("尚未生成压力－位置查找表") from exc
    except (OSError, ValueError, TypeError) as exc:
        raise ForcePositionLookupError(f"读取最新压力－位置查找表失败：{exc}") from exc
    if not isinstance(document, dict):
        raise ForcePositionLookupError("最新压力－位置查找表格式无效")
    return document


def resolve_mapping_release_reference(document):
    """校验查找表元数据并返回该表绑定的标定出发点。"""
    if not isinstance(document, dict):
        raise ForcePositionLookupError("压力－位置查找表格式无效")
    metadata = document.get("metadata")
    if not isinstance(metadata, dict):
        raise ForcePositionLookupError("压力－位置查找表缺少元数据")
    invalidated_reason = metadata.get("invalidated_reason")
    if invalidated_reason:
        raise ForcePositionLookupError("最新压力－位置查找表已经失效，请重新标定")
    if metadata.get("mapping_mode") != EXPECTED_MAPPING_MODE:
        raise ForcePositionLookupError("查找表不是相对标定出发点的行程格式，请重新标定")
    try:
        return int(metadata["release_reference_position_counts"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ForcePositionLookupError("查找表缺少有效的标定出发点") from exc


def resolve_force_position_target(document, target_force_g):
    """返回稳定压力档对应的标定基准、下压行程和绝对目标位置。"""
    release_reference = resolve_mapping_release_reference(document)
    metadata = document["metadata"]
    if metadata.get("force_unit") == "N":
        raise ForcePositionLookupError(
            "最新位置查找表已使用 N 单位；当前旧版 g 控制功能暂不可使用该表"
        )

    try:
        force_g = int(round(float(target_force_g)))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ForcePositionLookupError("目标压力无效") from exc
    mapping = document.get("mapping")
    if not isinstance(mapping, list) or not mapping:
        raise ForcePositionLookupError("压力－位置查找表没有可用档位")
    entry = None
    for item in mapping:
        if not isinstance(item, dict):
            continue
        try:
            item_force = int(round(float(item.get("force_g"))))
        except (TypeError, ValueError, OverflowError):
            continue
        if item_force == force_g:
            entry = item
            break
    if entry is None:
        raise ForcePositionLookupError(f"查找表中没有 {force_g} g 档位")
    if not entry.get("stable"):
        span = entry.get("position_span_counts")
        try:
            span_text = f"{float(span):.1f}"
        except (TypeError, ValueError, OverflowError):
            span_text = "--"
        raise ForcePositionLookupError(
            f"{force_g} g档下压行程极差 {span_text} counts，未通过稳定判据"
        )

    try:
        press_travel = int(entry["press_travel_counts"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ForcePositionLookupError("查找表缺少有效的下压行程") from exc
    if press_travel <= 0:
        raise ForcePositionLookupError(
            f"{force_g} g档下压行程 {press_travel} counts 无效"
        )

    try:
        position_span = float(entry.get("position_span_counts", math.nan))
    except (TypeError, ValueError, OverflowError):
        position_span = math.nan

    return {
        "force_g": force_g,
        "release_reference_position_counts": release_reference,
        "press_travel_counts": press_travel,
        "target_position_counts": release_reference - press_travel,
        "position_span_counts": position_span,
        "created_at": str(metadata.get("created_at", "--")),
        "source_created_at": str(
            metadata.get("source_created_at", metadata.get("created_at", "--"))
        ),
        "path": str(latest_mapping_path()),
    }


def resolve_force_position_target_n(document, target_force_n):
    """返回 0.10 N 档位对应的标定基准、下压行程和绝对目标位置。"""
    release_reference = resolve_mapping_release_reference(document)
    metadata = document["metadata"]
    if metadata.get("force_unit") != "N":
        raise ForcePositionLookupError(
            "当前查找表仍是旧的 g 单位，请先在“电机位置/压力标定”页重新标定"
        )
    try:
        force_n = round(float(target_force_n), 2)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ForcePositionLookupError("目标压力无效") from exc
    mapping = document.get("mapping")
    if not isinstance(mapping, list) or not mapping:
        raise ForcePositionLookupError("压力－位置查找表没有可用档位")
    entry = None
    for item in mapping:
        if not isinstance(item, dict):
            continue
        try:
            item_force = round(float(item.get("force_n")), 2)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isclose(item_force, force_n, abs_tol=0.0001):
            entry = item
            break
    if entry is None:
        raise ForcePositionLookupError(f"查找表中没有 {force_n:.2f} N 档位")
    if not entry.get("stable"):
        span = entry.get("position_span_counts")
        try:
            span_text = f"{float(span):.1f}"
        except (TypeError, ValueError, OverflowError):
            span_text = "--"
        raise ForcePositionLookupError(
            f"{force_n:.2f} N档下压行程极差{span_text} counts，未通过稳定判据"
        )
    try:
        press_travel = int(entry["press_travel_counts"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ForcePositionLookupError("查找表缺少有效的下压行程") from exc
    if press_travel <= 0:
        raise ForcePositionLookupError(
            f"{force_n:.2f} N档下压行程{press_travel} counts 无效"
        )
    try:
        position_span = float(entry.get("position_span_counts", math.nan))
    except (TypeError, ValueError, OverflowError):
        position_span = math.nan
    return {
        "force_n": force_n,
        "release_reference_position_counts": release_reference,
        "press_travel_counts": press_travel,
        "target_position_counts": release_reference - press_travel,
        "position_span_counts": position_span,
        "created_at": str(metadata.get("created_at", "--")),
        "source_created_at": str(
            metadata.get("source_created_at", metadata.get("created_at", "--"))
        ),
        "path": str(latest_mapping_path()),
    }
