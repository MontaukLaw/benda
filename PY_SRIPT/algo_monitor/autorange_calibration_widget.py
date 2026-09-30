import csv
import json
import math
import statistics
from collections import deque
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from qt_compat import QtCore, QtWidgets, Signal, Slot


REF_CDAC_TIERS = (12, 18, 27, 40, 60, 90, 135)
REFERENCE_REF_CDAC = 60
MAX_COUNT = 58928
VALID_RAW_LIMIT = round(MAX_COUNT * 0.82)
CAPTURE_FRAME_COUNT = 16
MANUAL_MODE_FLAG = 0x40
PENDING_FLAG = 0x01
PROJECT_SCHEMA_VERSION = 1
PROJECT_SUFFIX = ".arcal.json"
MIN_VALID_PAIR_COUNT = 6
TIME_SPAN_WARNING_SECONDS = 30.0
TIME_SPAN_FAILURE_SECONDS = 60.0


def _status_rank(status):
    return {
        "合格": 0,
        "采集中": 1,
        "警告": 2,
        "建议重测": 3,
    }.get(status, 0)


def _worse_status(current, candidate):
    return candidate if _status_rank(candidate) > _status_rank(current) else current


def _parse_sample_time(value):
    try:
        return datetime.fromisoformat(str(value))
    except (TypeError, ValueError):
        return None


def _linear_fit(pairs):
    if len(pairs) < 2:
        raise ValueError("有效配对样本少于2个")

    x_mean = sum(pair[0] for pair in pairs) / len(pairs)
    y_mean = sum(pair[1] for pair in pairs) / len(pairs)
    denominator = sum((pair[0] - x_mean) ** 2 for pair in pairs)
    if denominator <= 0.0:
        raise ValueError("输入Raw没有变化，无法拟合斜率")

    gain = sum(
        (pair[0] - x_mean) * (pair[1] - y_mean)
        for pair in pairs
    ) / denominator
    offset = y_mean - gain * x_mean
    errors = [(gain * pair[0] + offset) - pair[1] for pair in pairs]
    rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
    max_error = max(abs(error) for error in errors)
    target_variance = sum((pair[1] - y_mean) ** 2 for pair in pairs)
    r_squared = None
    if target_variance > 0.0:
        r_squared = 1.0 - (
            sum(error * error for error in errors) / target_variance
        )
    return {
        "gain": gain,
        "offset": offset,
        "rmse": rmse,
        "max_error": max_error,
        "r_squared": r_squared,
        "sample_count": len(pairs),
    }


def calculate_affine_coefficients(points):
    edge_paths = {
        12: ((12, 18), (18, 27), (27, 40), (40, 60)),
        18: ((18, 27), (27, 40), (40, 60)),
        27: ((27, 40), (40, 60)),
        40: ((40, 60),),
        60: (),
        90: ((90, 60),),
        135: ((135, 90), (90, 60)),
    }
    coefficients = []

    for channel_index in range(4):
        edge_fits = {}
        for source_ref, target_ref in {
            edge for path in edge_paths.values() for edge in path
        }:
            pairs = []
            for point in points:
                source = point["samples"].get(source_ref)
                target = point["samples"].get(target_ref)
                if source is None or target is None:
                    continue
                source_raw = float(source["raw"][channel_index])
                target_raw = float(target["raw"][channel_index])
                if source_raw >= VALID_RAW_LIMIT or target_raw >= VALID_RAW_LIMIT:
                    continue
                pairs.append((source_raw, target_raw))
            try:
                edge_fits[(source_ref, target_ref)] = _linear_fit(pairs)
            except ValueError as exc:
                raise ValueError(
                    f"CH{channel_index + 4} 的 Ref {source_ref}→{target_ref}：{exc}"
                ) from exc

        for ref_cdac in REF_CDAC_TIERS:
            gain = 1.0
            offset = 0.0
            path = edge_paths[ref_cdac]
            edge_sample_counts = []
            for edge in path:
                edge_fit = edge_fits[edge]
                gain = edge_fit["gain"] * gain
                offset = edge_fit["gain"] * offset + edge_fit["offset"]
                edge_sample_counts.append(edge_fit["sample_count"])

            direct_pairs = []
            for point in points:
                source = point["samples"].get(ref_cdac)
                target = point["samples"].get(REFERENCE_REF_CDAC)
                if source is None or target is None:
                    continue
                source_raw = float(source["raw"][channel_index])
                target_raw = float(target["raw"][channel_index])
                if source_raw >= VALID_RAW_LIMIT or target_raw >= VALID_RAW_LIMIT:
                    continue
                direct_pairs.append((source_raw, target_raw))

            if ref_cdac == REFERENCE_REF_CDAC:
                rmse = 0.0
                max_error = 0.0
                r_squared = 1.0
            elif direct_pairs:
                errors = [
                    (gain * source_raw + offset) - target_raw
                    for source_raw, target_raw in direct_pairs
                ]
                rmse = math.sqrt(
                    sum(error * error for error in errors) / len(errors)
                )
                max_error = max(abs(error) for error in errors)
                target_mean = sum(pair[1] for pair in direct_pairs) / len(direct_pairs)
                target_variance = sum(
                    (pair[1] - target_mean) ** 2 for pair in direct_pairs
                )
                r_squared = None
                if target_variance > 0.0:
                    r_squared = 1.0 - (
                        sum(error * error for error in errors) / target_variance
                    )
            else:
                rmse = None
                max_error = None
                r_squared = None

            coefficients.append({
                "channel": channel_index + 4,
                "ref_cdac": ref_cdac,
                "gain": gain,
                "offset": offset,
                "gain_q16": round(gain * 65536.0),
                "offset_int": round(offset),
                "rmse": rmse,
                "max_error": max_error,
                "r_squared": r_squared,
                "direct_sample_count": len(direct_pairs),
                "edge_min_sample_count": min(edge_sample_counts) if edge_sample_counts else len(direct_pairs),
                "fit_path": "identity" if not path else " → ".join(
                    [str(path[0][0])] + [str(edge[1]) for edge in path]
                ),
            })

    return coefficients


def analyze_calibration_quality(points):
    """对采样质量、时间跨度、留一验证和有效覆盖进行统一分析。"""
    results = {
        "points": {},
        "coverage": [],
        "recommendations": [],
    }
    if not points:
        return results

    empty_point = points[0]
    for point in points:
        point_id = point["id"]
        point_result = {
            "status": "合格",
            "score": 0.0,
            "reasons": [],
            "recommended_refs": set(),
            "remeasure_whole_point": False,
            "global_fit_warning": False,
            "complete_auto_point": False,
            "sweep_drift_detected": False,
            "samples": {},
        }
        complete_auto_point = (
            len(point["samples"]) == len(REF_CDAC_TIERS)
            and all(
                point["samples"].get(ref_cdac, {}).get("capture_mode")
                == "auto_bidirectional"
                for ref_cdac in REF_CDAC_TIERS
            )
        )
        point_result["complete_auto_point"] = complete_auto_point
        missing_refs = []
        sample_times = []
        sweep_drift_refs = set()

        for ref_cdac in REF_CDAC_TIERS:
            sample = point["samples"].get(ref_cdac)
            sample_result = {
                "status": "合格",
                "reasons": [],
                "usable_channels": [],
            }
            point_result["samples"][ref_cdac] = sample_result
            if sample is None:
                missing_refs.append(ref_cdac)
                sample_result["status"] = "采集中"
                sample_result["reasons"].append("尚未采集")
                continue

            saved_at = _parse_sample_time(sample.get("saved_at"))
            if saved_at is not None:
                sample_times.append(saved_at)

            for channel_index, channel in enumerate(range(4, 8)):
                raw_value = float(sample["raw"][channel_index])
                stddev = float(sample["stddev"][channel_index])
                noise_limit = max(5.0, raw_value * 0.001)
                if raw_value >= VALID_RAW_LIMIT:
                    sample_result["status"] = _worse_status(
                        sample_result["status"], "警告"
                    )
                    sample_result["reasons"].append(
                        f"CH{channel} Raw={raw_value:.0f}达到82%量程，排除拟合"
                    )
                else:
                    sample_result["usable_channels"].append(channel)
                if stddev > noise_limit:
                    sample_result["status"] = "建议重测"
                    sample_result["reasons"].append(
                        f"CH{channel}噪声σ={stddev:.1f}超过{noise_limit:.1f}"
                    )
                    point_result["recommended_refs"].add(ref_cdac)
                sweep_delta = sample.get("sweep_delta")
                empty_sample = empty_point["samples"].get(ref_cdac)
                if (
                    sweep_delta is not None
                    and len(sweep_delta) == 4
                    and empty_sample is not None
                ):
                    signal_delta = abs(
                        raw_value - float(empty_sample["raw"][channel_index])
                    )
                    sweep_limit = max(100.0, signal_delta * 0.01)
                    if abs(float(sweep_delta[channel_index])) > sweep_limit:
                        sample_result["status"] = "建议重测"
                        sample_result["reasons"].append(
                            f"CH{channel}双向差{float(sweep_delta[channel_index]):+.0f}"
                            f"超过{sweep_limit:.0f} counts"
                        )
                        point_result["recommended_refs"].add(ref_cdac)
                        sweep_drift_refs.add(ref_cdac)

            point_result["status"] = _worse_status(
                point_result["status"], sample_result["status"]
            )

        if sweep_drift_refs:
            point_result["sweep_drift_detected"] = True
            point_result["status"] = "建议重测"
            point_result["remeasure_whole_point"] = True
            point_result["reasons"].append(
                "双向扫描前后不一致：Ref "
                + "、".join(str(value) for value in sorted(sweep_drift_refs))
            )

        if missing_refs:
            point_result["status"] = _worse_status(point_result["status"], "采集中")
            point_result["reasons"].append(
                "缺少Ref " + "、".join(str(value) for value in missing_refs)
            )

        if len(sample_times) >= 2:
            time_span = (max(sample_times) - min(sample_times)).total_seconds()
            point_result["time_span_seconds"] = time_span
            if time_span > TIME_SPAN_FAILURE_SECONDS:
                if point is empty_point:
                    point_result["status"] = _worse_status(point_result["status"], "警告")
                    point_result["reasons"].append(
                        f"P0采集跨度{time_span:.1f}秒，建议确认空载基线未漂移"
                    )
                else:
                    point_result["status"] = "建议重测"
                    point_result["remeasure_whole_point"] = True
                    point_result["reasons"].append(
                        f"整组采集跨度{time_span:.1f}秒，慢漂移风险高"
                    )
            elif time_span > TIME_SPAN_WARNING_SECONDS:
                point_result["status"] = _worse_status(point_result["status"], "警告")
                point_result["reasons"].append(
                    f"整组采集跨度{time_span:.1f}秒，建议缩短或使用Ref60夹测"
                )

        results["points"][point_id] = point_result

    # 留一压力点验证可防止异常点被全量拟合本身吸收。
    if len(points) >= 4:
        for point_index, point in enumerate(points):
            if point_index == 0:
                continue
            training_points = [
                candidate
                for candidate_index, candidate in enumerate(points)
                if candidate_index != point_index
            ]
            try:
                coefficients = calculate_affine_coefficients(training_points)
            except ValueError:
                continue
            coefficient_map = {
                (item["channel"], item["ref_cdac"]): item
                for item in coefficients
            }
            point_result = results["points"][point["id"]]
            complete_auto_point = point_result["complete_auto_point"]
            sweep_drift_detected = point_result["sweep_drift_detected"]
            normalized_errors = []
            signed_errors_by_ref = {}

            for ref_cdac in REF_CDAC_TIERS:
                if ref_cdac == REFERENCE_REF_CDAC:
                    continue
                source = point["samples"].get(ref_cdac)
                target = point["samples"].get(REFERENCE_REF_CDAC)
                empty_target = empty_point["samples"].get(REFERENCE_REF_CDAC)
                if source is None or target is None or empty_target is None:
                    continue
                ref_errors = []
                for channel_index, channel in enumerate(range(4, 8)):
                    source_raw = float(source["raw"][channel_index])
                    target_raw = float(target["raw"][channel_index])
                    if source_raw >= VALID_RAW_LIMIT or target_raw >= VALID_RAW_LIMIT:
                        continue
                    training_raw_values = []
                    for training_point in training_points:
                        training_source = training_point["samples"].get(ref_cdac)
                        training_target = training_point["samples"].get(REFERENCE_REF_CDAC)
                        if training_source is None or training_target is None:
                            continue
                        training_source_raw = float(training_source["raw"][channel_index])
                        training_target_raw = float(training_target["raw"][channel_index])
                        if (
                            training_source_raw < VALID_RAW_LIMIT
                            and training_target_raw < VALID_RAW_LIMIT
                        ):
                            training_raw_values.append(training_source_raw)
                    if (
                        len(training_raw_values) < MIN_VALID_PAIR_COUNT
                        or source_raw < min(training_raw_values)
                        or source_raw > max(training_raw_values)
                    ):
                        continue
                    coefficient = coefficient_map[(channel, ref_cdac)]
                    error = (
                        coefficient["gain"] * source_raw
                        + coefficient["offset"]
                        - target_raw
                    )
                    signal_delta = abs(
                        target_raw - float(empty_target["raw"][channel_index])
                    )
                    allowed_error = max(100.0, signal_delta * 0.01)
                    ratio = abs(error) / allowed_error
                    normalized_errors.append((ratio, ref_cdac, channel, error))
                    ref_errors.append((error, ratio))
                if ref_errors:
                    signed_errors_by_ref[ref_cdac] = ref_errors

            if normalized_errors:
                worst_ratio, worst_ref, worst_channel, worst_error = max(normalized_errors)
                point_result["score"] = worst_ratio
                if worst_ratio > 5.0:
                    if complete_auto_point and not sweep_drift_detected:
                        point_result["status"] = _worse_status(
                            point_result["status"], "警告"
                        )
                        point_result["global_fit_warning"] = True
                        point_result["reasons"].append(
                            f"本点双向采集自洽，但与旧数据拟合的Ref{worst_ref}/"
                            f"CH{worst_channel}相差{worst_error:+.0f} counts"
                        )
                    else:
                        bad_refs = {
                            ref_cdac
                            for ratio, ref_cdac, _channel, _error in normalized_errors
                            if ratio > 5.0
                        }
                        point_result["status"] = "建议重测"
                        point_result["recommended_refs"].update(bad_refs)
                        point_result["reasons"].append(
                            f"留一验证最差为Ref{worst_ref}/CH{worst_channel}，"
                            f"误差{worst_error:+.0f} counts"
                        )
                elif worst_ratio > 2.0:
                    point_result["status"] = _worse_status(point_result["status"], "警告")
                    point_result["reasons"].append(
                        f"留一验证最差为Ref{worst_ref}/CH{worst_channel}，"
                        f"误差{worst_error:+.0f} counts"
                    )

            for ref_cdac, error_items in signed_errors_by_ref.items():
                if len(error_items) < 3:
                    continue
                errors = [item[0] for item in error_items]
                ratios = [item[1] for item in error_items]
                same_direction = all(value > 0.0 for value in errors) or all(
                    value < 0.0 for value in errors
                )
                if same_direction and statistics.median(ratios) > 4.0:
                    if complete_auto_point and not sweep_drift_detected:
                        point_result["status"] = _worse_status(
                            point_result["status"], "警告"
                        )
                        point_result["global_fit_warning"] = True
                        if not any("旧数据拟合" in reason for reason in point_result["reasons"]):
                            point_result["reasons"].append(
                                f"本点双向采集自洽，但Ref{ref_cdac}多通道与旧数据拟合不一致"
                            )
                    else:
                        point_result["status"] = "建议重测"
                        point_result["recommended_refs"].add(ref_cdac)
                        point_result["remeasure_whole_point"] = True
                        point_result["reasons"].append(
                            f"Ref{ref_cdac}有{len(errors)}个通道同方向偏移，疑似慢漂移"
                        )

    # 覆盖检查按通道和档位计数，避免大量饱和点造成“总点数很多”的假象。
    for channel_index, channel in enumerate(range(4, 8)):
        for ref_cdac in REF_CDAC_TIERS:
            valid_count = 0
            for point in points:
                source = point["samples"].get(ref_cdac)
                target = point["samples"].get(REFERENCE_REF_CDAC)
                if source is None or target is None:
                    continue
                if (
                    float(source["raw"][channel_index]) < VALID_RAW_LIMIT
                    and float(target["raw"][channel_index]) < VALID_RAW_LIMIT
                ):
                    valid_count += 1
            if valid_count < MIN_VALID_PAIR_COUNT:
                results["coverage"].append({
                    "channel": channel,
                    "ref_cdac": ref_cdac,
                    "valid_count": valid_count,
                    "required_count": MIN_VALID_PAIR_COUNT,
                })

    for point in points:
        point_result = results["points"][point["id"]]
        point_result["recommended_refs"] = sorted(point_result["recommended_refs"])
        if point_result["status"] == "建议重测":
            if point_result["remeasure_whole_point"]:
                action = "重测整个压力点"
            elif point_result["recommended_refs"]:
                action = "重测Ref " + "、".join(
                    str(value) for value in point_result["recommended_refs"]
                )
            else:
                action = "检查并重测异常数据"
            results["recommendations"].append({
                "point_id": point["id"],
                "point": point["name"],
                "status": point_result["status"],
                "score": point_result["score"],
                "action": action,
                "reason": "；".join(point_result["reasons"]),
            })
        elif point_result["global_fit_warning"]:
            results["recommendations"].append({
                "point_id": point["id"],
                "point": point["name"],
                "status": "警告",
                "score": point_result["score"],
                "action": "保留本点，复核旧手动点或拟合模型",
                "reason": "；".join(point_result["reasons"]),
            })

    if results["coverage"]:
        coverage_text = "、".join(
            f"CH{item['channel']}/Ref{item['ref_cdac']}({item['valid_count']})"
            for item in results["coverage"][:8]
        )
        if len(results["coverage"]) > 8:
            coverage_text += f"等{len(results['coverage'])}项"
        results["recommendations"].append({
            "point_id": "",
            "point": "新增",
            "status": "警告",
            "score": 0.0,
            "action": "增加有效中间压力点",
            "reason": f"有效配对不足{MIN_VALID_PAIR_COUNT}组：{coverage_text}",
        })

    return results


class AutorangeCalibrationWidget(QtWidgets.QWidget):
    control_requested = Signal(bool, int)
    status_message = Signal(str)

    def __init__(self, sensor_sample_provider, parent=None):
        super().__init__(parent)
        self._sensor_sample_provider = sensor_sample_provider
        self._connected = False
        self._device_status = None
        self._syncing_mode_button = False
        self._last_sequence = None
        self._last_status_ref = None
        self._was_device_stable = False
        self._capture_windows = {
            ref_cdac: deque(maxlen=CAPTURE_FRAME_COUNT)
            for ref_cdac in REF_CDAC_TIERS
        }
        self._points = [self._new_point(0)]
        self._coefficients = []
        self._build_ui()

        self._capture_timer = QtCore.QTimer(self)
        self._capture_timer.setInterval(10)
        self._capture_timer.timeout.connect(self._poll_live_sample)
        self._capture_timer.start()
        self._load_point(0)
        self._update_control_states()

    def _new_point(self, index):
        return {
            "name": f"P{index}",
            "note": "空载" if index == 0 else f"压力点 P{index}",
            "samples": {},
        }

    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        mode_row = QtWidgets.QHBoxLayout()
        self.mode_button = QtWidgets.QPushButton("进入手动校准模式")
        self.mode_button.setCheckable(True)
        self.mode_button.toggled.connect(self._on_mode_toggled)
        mode_row.addWidget(self.mode_button)
        mode_row.addWidget(QtWidgets.QLabel("Ref CDAC"))
        self.ref_combo = QtWidgets.QComboBox()
        for ref_cdac in REF_CDAC_TIERS:
            self.ref_combo.addItem(str(ref_cdac), ref_cdac)
        mode_row.addWidget(self.ref_combo)
        self.apply_ref_button = QtWidgets.QPushButton("切换档位")
        self.apply_ref_button.clicked.connect(self._request_selected_ref)
        mode_row.addWidget(self.apply_ref_button)
        self.save_current_button = QtWidgets.QPushButton("保存当前档位")
        self.save_current_button.setToolTip(
            "按固件实际Ref，把当前稳定窗口的CH4～CH7 Hardware Raw保存到对应行"
        )
        self.save_current_button.clicked.connect(self._save_active_ref)
        mode_row.addWidget(self.save_current_button)
        mode_row.addStretch(1)
        root.addLayout(mode_row)

        self.device_status_label = QtWidgets.QLabel("串口未连接")
        self.device_status_label.setWordWrap(True)
        root.addWidget(self.device_status_label)

        point_row = QtWidgets.QHBoxLayout()
        point_row.addWidget(QtWidgets.QLabel("压力点"))
        self.point_combo = QtWidgets.QComboBox()
        self.point_combo.currentIndexChanged.connect(self._load_point)
        point_row.addWidget(self.point_combo)
        point_row.addWidget(QtWidgets.QLabel("备注"))
        self.note_edit = QtWidgets.QLineEdit()
        self.note_edit.editingFinished.connect(self._save_current_note)
        point_row.addWidget(self.note_edit, 1)
        self.add_point_button = QtWidgets.QPushButton("新增压力点")
        self.add_point_button.clicked.connect(self._add_point)
        point_row.addWidget(self.add_point_button)
        self.delete_point_button = QtWidgets.QPushButton("删除当前点")
        self.delete_point_button.clicked.connect(self._delete_current_point)
        point_row.addWidget(self.delete_point_button)
        root.addLayout(point_row)

        self.tabs = QtWidgets.QTabWidget()
        root.addWidget(self.tabs, 1)

        sample_page = QtWidgets.QWidget()
        sample_layout = QtWidgets.QVBoxLayout(sample_page)
        sample_layout.setContentsMargins(0, 0, 0, 0)
        self.sample_table = QtWidgets.QTableWidget(len(REF_CDAC_TIERS), 6)
        self.sample_table.setHorizontalHeaderLabels([
            "Ref", "CH4 HW Raw", "CH5 HW Raw", "CH6 HW Raw", "CH7 HW Raw", "采样"
        ])
        self.sample_table.verticalHeader().setVisible(False)
        no_edit = getattr(
            QtWidgets.QAbstractItemView,
            "NoEditTriggers",
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers,
        )
        self.sample_table.setEditTriggers(no_edit)
        self.sample_table.horizontalHeader().setStretchLastSection(False)
        fixed_resize = getattr(
            QtWidgets.QHeaderView,
            "Fixed",
            QtWidgets.QHeaderView.ResizeMode.Fixed,
        )
        stretch_resize = getattr(
            QtWidgets.QHeaderView,
            "Stretch",
            QtWidgets.QHeaderView.ResizeMode.Stretch,
        )
        header = self.sample_table.horizontalHeader()
        header.setSectionResizeMode(0, fixed_resize)
        for column in range(1, 5):
            header.setSectionResizeMode(column, stretch_resize)
        header.setSectionResizeMode(5, fixed_resize)
        self.sample_table.setColumnWidth(0, 48)
        self.sample_table.setColumnWidth(5, 68)
        self.sample_table.verticalHeader().setDefaultSectionSize(34)
        scroll_bar_off = getattr(
            QtCore.Qt,
            "ScrollBarAlwaysOff",
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff,
        )
        self.sample_table.setVerticalScrollBarPolicy(scroll_bar_off)
        for row, ref_cdac in enumerate(REF_CDAC_TIERS):
            self.sample_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(ref_cdac)))
            for column in range(1, 6):
                self.sample_table.setItem(row, column, QtWidgets.QTableWidgetItem("--"))
        sample_table_height = (
            self.sample_table.horizontalHeader().sizeHint().height()
            + (len(REF_CDAC_TIERS) * self.sample_table.verticalHeader().defaultSectionSize())
            + self.sample_table.horizontalScrollBar().sizeHint().height()
            + (self.sample_table.frameWidth() * 2)
            + 4
        )
        self.sample_table.setFixedHeight(sample_table_height)
        sample_layout.addWidget(self.sample_table)
        self.live_raw_label = QtWidgets.QLabel("等待稳定的Hardware Raw采样")
        self.live_raw_label.setWordWrap(True)
        sample_layout.addWidget(self.live_raw_label)
        self.tabs.addTab(sample_page, "采集数据")

        coefficient_page = QtWidgets.QWidget()
        coefficient_layout = QtWidgets.QVBoxLayout(coefficient_page)
        coefficient_layout.setContentsMargins(0, 0, 0, 0)
        self.coefficient_table = QtWidgets.QTableWidget(0, 11)
        self.coefficient_table.setHorizontalHeaderLabels([
            "CH", "Ref", "a", "b", "Gain Q16", "Offset", "RMSE", "最大误差", "R²", "样本", "拟合路径"
        ])
        self.coefficient_table.verticalHeader().setVisible(False)
        self.coefficient_table.setEditTriggers(no_edit)
        self.coefficient_table.horizontalHeader().setStretchLastSection(True)
        coefficient_layout.addWidget(self.coefficient_table)
        self.tabs.addTab(coefficient_page, "仿射参数")

        action_row = QtWidgets.QHBoxLayout()
        self.calculate_button = QtWidgets.QPushButton("计算28组 a、b 与误差")
        self.calculate_button.clicked.connect(self._calculate)
        action_row.addWidget(self.calculate_button)
        self.export_button = QtWidgets.QPushButton("导出样本与参数 CSV")
        self.export_button.clicked.connect(self._export_csv)
        action_row.addWidget(self.export_button)
        self.clear_button = QtWidgets.QPushButton("清空全部采样")
        self.clear_button.clicked.connect(self._clear_all_samples)
        action_row.addWidget(self.clear_button)
        action_row.addStretch(1)
        root.addLayout(action_row)

        self.fit_status_label = QtWidgets.QLabel(
            "P0固定为空载；建议P1以后使用不同压力，并让相邻档位的有效Raw范围充分重叠。"
        )
        self.fit_status_label.setWordWrap(True)
        root.addWidget(self.fit_status_label)

    def set_connected(self, connected):
        self._connected = bool(connected)
        if not connected:
            self._device_status = None
            self._sync_mode_button(False)
            self.device_status_label.setText("串口未连接")
        self._update_control_states()

    def on_device_status(self, frame):
        self._device_status = dict(frame)
        manual_mode = bool(int(frame["flags"]) & MANUAL_MODE_FLAG)
        self._sync_mode_button(manual_mode)
        actual_ref = int(frame["actual_ref_cdac"])
        requested_ref = int(frame.get("requested_ref_cdac", actual_ref))
        stable = self._is_device_stable()
        if actual_ref != self._last_status_ref or (stable and not self._was_device_stable):
            if actual_ref in self._capture_windows:
                self._capture_windows[actual_ref].clear()
        self._last_status_ref = actual_ref
        self._was_device_stable = stable
        self.device_status_label.setText(
            f"固件模式={'手动校准' if manual_mode else '自动量程'}；"
            f"请求Ref={requested_ref}，实际Ref={actual_ref}；"
            f"{'已稳定，可以保存' if stable else '切档或等待稳定中'}"
        )
        self._update_control_states()

    def control_command_failed(self, message):
        self.fit_status_label.setText(message)
        if self._device_status is not None:
            manual_mode = bool(int(self._device_status["flags"]) & MANUAL_MODE_FLAG)
            self._sync_mode_button(manual_mode)
        self._update_control_states()

    def _sync_mode_button(self, checked):
        self._syncing_mode_button = True
        try:
            self.mode_button.setChecked(bool(checked))
            self.mode_button.setText(
                "退出手动校准并恢复自动量程" if checked else "进入手动校准模式"
            )
        finally:
            self._syncing_mode_button = False

    @Slot(bool)
    def _on_mode_toggled(self, checked):
        if self._syncing_mode_button:
            return
        if not self._connected:
            self._sync_mode_button(False)
            self.status_message.emit("无法进入校准模式：串口未连接")
            return
        ref_cdac = int(self.ref_combo.currentData()) if checked else 0
        if checked:
            self._capture_windows[ref_cdac].clear()
        self.mode_button.setText(
            "退出手动校准并恢复自动量程" if checked else "进入手动校准模式"
        )
        self.control_requested.emit(bool(checked), ref_cdac)
        self._update_control_states()

    @Slot()
    def _request_selected_ref(self):
        if not self._connected or not self.mode_button.isChecked():
            self.status_message.emit("请先连接设备并进入手动校准模式")
            return
        ref_cdac = int(self.ref_combo.currentData())
        self._capture_windows[ref_cdac].clear()
        self.control_requested.emit(True, ref_cdac)

    def _is_device_stable(self):
        if self._device_status is None:
            return False
        flags = int(self._device_status["flags"])
        return (
            bool(flags & MANUAL_MODE_FLAG)
            and not bool(flags & PENDING_FLAG)
            and int(self._device_status["blank_frames"]) == 0
            and int(self._device_status["actual_ref_cdac"])
            == int(self._device_status.get("requested_ref_cdac", -1))
        )

    @Slot()
    def _poll_live_sample(self):
        sample = self._sensor_sample_provider()
        if sample is None or not sample.get("has_raw") or sample.get("age_seconds", 1.0) > 0.5:
            self._update_control_states()
            return
        sequence = int(sample["sequence"])
        if sequence == self._last_sequence:
            return
        self._last_sequence = sequence
        if not self._is_device_stable():
            self._update_control_states()
            return
        actual_ref = int(self._device_status["actual_ref_cdac"])
        hardware_raw = tuple(int(value) for value in sample["hardware_raw"][4:8])
        if len(hardware_raw) != 4:
            return
        self._capture_windows[actual_ref].append(hardware_raw)
        self.live_raw_label.setText(
            f"实时 Ref={actual_ref}：CH4～CH7={hardware_raw}；"
            f"有效窗口 {len(self._capture_windows[actual_ref])}/{CAPTURE_FRAME_COUNT} 帧"
        )
        self._update_control_states()

    def _update_control_states(self):
        manual_mode = self.mode_button.isChecked()
        stable = self._is_device_stable()
        actual_ref = None
        if self._device_status is not None:
            actual_ref = int(self._device_status["actual_ref_cdac"])
        self.mode_button.setEnabled(self._connected)
        self.ref_combo.setEnabled(self._connected and manual_mode)
        self.apply_ref_button.setEnabled(self._connected and manual_mode)
        can_save_current = (
            self._connected
            and stable
            and actual_ref in self._capture_windows
            and len(self._capture_windows[actual_ref]) >= CAPTURE_FRAME_COUNT
        )
        self.save_current_button.setEnabled(can_save_current)
        current_index = self.point_combo.currentIndex()
        self.delete_point_button.setEnabled(current_index > 0)
        self.note_edit.setReadOnly(current_index == 0)

    @Slot()
    def _add_point(self):
        self._save_current_note()
        point = self._new_point(len(self._points))
        self._points.append(point)
        for window in self._capture_windows.values():
            window.clear()
        self._refresh_point_combo(len(self._points) - 1)
        self._coefficients = []
        self._show_coefficients()

    @Slot()
    def _delete_current_point(self):
        index = self.point_combo.currentIndex()
        if index <= 0:
            return
        del self._points[index]
        for point_index, point in enumerate(self._points):
            point["name"] = f"P{point_index}"
        self._refresh_point_combo(max(0, index - 1))
        self._coefficients = []
        self._show_coefficients()

    def _refresh_point_combo(self, selected_index):
        self.point_combo.blockSignals(True)
        self.point_combo.clear()
        for point in self._points:
            self.point_combo.addItem(point["name"])
        self.point_combo.setCurrentIndex(selected_index)
        self.point_combo.blockSignals(False)
        self._load_point(selected_index)

    @Slot(int)
    def _load_point(self, index):
        if not self._points:
            return
        index = max(0, min(index, len(self._points) - 1))
        if self.point_combo.count() != len(self._points):
            self._refresh_point_combo(index)
            return
        point = self._points[index]
        for window in self._capture_windows.values():
            window.clear()
        self.note_edit.setText(point["note"])
        for row, ref_cdac in enumerate(REF_CDAC_TIERS):
            saved = point["samples"].get(ref_cdac)
            if saved is None:
                for column in range(1, 6):
                    self.sample_table.item(row, column).setText("--")
                continue
            for channel_index in range(4):
                self.sample_table.item(row, channel_index + 1).setText(
                    f"{saved['raw'][channel_index]:.1f}"
                )
            self.sample_table.item(row, 5).setText(
                f"{saved['sample_count']}帧"
            )
        self._update_control_states()

    @Slot()
    def _save_current_note(self):
        index = self.point_combo.currentIndex()
        if index < 0 or index >= len(self._points):
            return
        if index == 0:
            self._points[0]["note"] = "空载"
            self.note_edit.setText("空载")
        else:
            self._points[index]["note"] = self.note_edit.text().strip() or f"压力点 P{index}"

    def _save_current_raw(self, ref_cdac):
        if not self._is_device_stable():
            self.status_message.emit("当前Ref尚未稳定，不能保存")
            return
        actual_ref = int(self._device_status["actual_ref_cdac"])
        if actual_ref != ref_cdac:
            self.status_message.emit(f"当前实际Ref为{actual_ref}，不能保存到Ref{ref_cdac}行")
            return
        frames = list(self._capture_windows[ref_cdac])
        if len(frames) < CAPTURE_FRAME_COUNT:
            self.status_message.emit(
                f"稳定样本只有{len(frames)}帧，请等待{CAPTURE_FRAME_COUNT}帧后再保存"
            )
            return
        raw_values = tuple(
            float(statistics.median(frame[channel_index] for frame in frames))
            for channel_index in range(4)
        )
        stddev_values = tuple(
            float(statistics.pstdev(frame[channel_index] for frame in frames))
            for channel_index in range(4)
        )
        point_index = self.point_combo.currentIndex()
        self._points[point_index]["samples"][ref_cdac] = {
            "raw": raw_values,
            "stddev": stddev_values,
            "minimum": tuple(min(frame[index] for frame in frames) for index in range(4)),
            "maximum": tuple(max(frame[index] for frame in frames) for index in range(4)),
            "sample_count": len(frames),
            "saved_at": datetime.now().isoformat(timespec="milliseconds"),
        }
        self._coefficients = []
        self._load_point(point_index)
        self._show_coefficients()
        self.fit_status_label.setText(
            f"已保存 {self._points[point_index]['name']} / Ref {ref_cdac}：{raw_values}"
        )

    @Slot()
    def _save_active_ref(self):
        if self._device_status is None:
            self.status_message.emit("尚未收到固件量程状态，不能保存")
            return
        self._save_current_raw(int(self._device_status["actual_ref_cdac"]))

    @Slot()
    def _calculate(self):
        self._save_current_note()
        try:
            self._coefficients = calculate_affine_coefficients(self._points)
        except ValueError as exc:
            self._coefficients = []
            self._show_coefficients()
            self.fit_status_label.setText(f"拟合失败：{exc}")
            self.status_message.emit(f"仿射校准拟合失败：{exc}")
            return False
        self._show_coefficients()
        self.tabs.setCurrentIndex(1)
        self.fit_status_label.setText(
            f"已计算{len(self._coefficients)}组参数；Raw ≥ {VALID_RAW_LIMIT}的样本已按接近饱和自动排除。"
        )
        return True

    def _show_coefficients(self):
        self.coefficient_table.setRowCount(len(self._coefficients))
        for row, coefficient in enumerate(self._coefficients):
            values = (
                f"CH{coefficient['channel']}",
                str(coefficient["ref_cdac"]),
                f"{coefficient['gain']:.9f}",
                f"{coefficient['offset']:.3f}",
                str(coefficient["gain_q16"]),
                str(coefficient["offset_int"]),
                "--" if coefficient["rmse"] is None else f"{coefficient['rmse']:.3f}",
                "--" if coefficient["max_error"] is None else f"{coefficient['max_error']:.3f}",
                "--" if coefficient["r_squared"] is None else f"{coefficient['r_squared']:.6f}",
                f"直接{coefficient['direct_sample_count']} / 相邻≥{coefficient['edge_min_sample_count']}",
                coefficient["fit_path"],
            )
            for column, value in enumerate(values):
                self.coefficient_table.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        if self._coefficients:
            self.coefficient_table.resizeColumnsToContents()

    @Slot()
    def _clear_all_samples(self):
        answer = QtWidgets.QMessageBox.question(
            self,
            "清空校准数据",
            "确认清空所有压力点和已计算参数吗？",
        )
        yes_value = getattr(
            QtWidgets.QMessageBox,
            "Yes",
            QtWidgets.QMessageBox.StandardButton.Yes,
        )
        if answer != yes_value:
            return
        self._points = [self._new_point(0)]
        self._coefficients = []
        self._refresh_point_combo(0)
        self._show_coefficients()
        self.fit_status_label.setText("校准数据已清空；P0为空载。")

    @Slot()
    def _export_csv(self):
        fit_error = None
        if not self._coefficients:
            try:
                self._coefficients = calculate_affine_coefficients(self._points)
                self._show_coefficients()
            except ValueError as exc:
                fit_error = str(exc)
                self._coefficients = []
        default_directory = Path(__file__).resolve().with_name("calibration_exports")
        default_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        selected_path, _filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "导出Ref CDAC仿射校准数据",
            str(default_directory / f"autorange_calibration_{timestamp}.csv"),
            "CSV files (*.csv)",
        )
        if not selected_path:
            return
        base_path = Path(selected_path)
        if base_path.suffix.lower() == ".csv":
            base_path = base_path.with_suffix("")
        samples_path = base_path.with_name(base_path.name + "_samples.csv")
        coefficients_path = base_path.with_name(base_path.name + "_coefficients.csv")
        try:
            self._write_samples_csv(samples_path)
            self._write_coefficients_csv(coefficients_path)
        except OSError as exc:
            self.status_message.emit(f"校准CSV导出失败：{exc}")
            return
        message = f"已导出：{samples_path.name}、{coefficients_path.name}"
        if fit_error is not None:
            message += f"；当前数据尚不能完整拟合：{fit_error}"
        self.fit_status_label.setText(message)
        self.status_message.emit(message)

    def _write_samples_csv(self, path):
        fieldnames = ["point", "is_empty_load", "note", "ref_cdac", "sample_count", "saved_at"]
        for channel in range(4, 8):
            fieldnames.extend([
                f"ch{channel}_hardware_raw",
                f"ch{channel}_stddev",
                f"ch{channel}_minimum",
                f"ch{channel}_maximum",
            ])
        with path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for point_index, point in enumerate(self._points):
                for ref_cdac in REF_CDAC_TIERS:
                    sample = point["samples"].get(ref_cdac)
                    if sample is None:
                        continue
                    row = {
                        "point": point["name"],
                        "is_empty_load": 1 if point_index == 0 else 0,
                        "note": point["note"],
                        "ref_cdac": ref_cdac,
                        "sample_count": sample["sample_count"],
                        "saved_at": sample["saved_at"],
                    }
                    for channel_index, channel in enumerate(range(4, 8)):
                        row[f"ch{channel}_hardware_raw"] = sample["raw"][channel_index]
                        row[f"ch{channel}_stddev"] = sample["stddev"][channel_index]
                        row[f"ch{channel}_minimum"] = sample["minimum"][channel_index]
                        row[f"ch{channel}_maximum"] = sample["maximum"][channel_index]
                    writer.writerow(row)

    def _write_coefficients_csv(self, path):
        fieldnames = [
            "channel", "ref_cdac", "reference_ref_cdac", "a_gain", "b_offset",
            "gain_q16", "offset_int", "rmse_counts", "max_abs_error_counts",
            "r_squared", "direct_sample_count", "edge_min_sample_count", "fit_path",
            "raw_exclusion_limit",
        ]
        with path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for coefficient in self._coefficients:
                writer.writerow({
                    "channel": coefficient["channel"],
                    "ref_cdac": coefficient["ref_cdac"],
                    "reference_ref_cdac": REFERENCE_REF_CDAC,
                    "a_gain": f"{coefficient['gain']:.12f}",
                    "b_offset": f"{coefficient['offset']:.6f}",
                    "gain_q16": coefficient["gain_q16"],
                    "offset_int": coefficient["offset_int"],
                    "rmse_counts": "" if coefficient["rmse"] is None else f"{coefficient['rmse']:.6f}",
                    "max_abs_error_counts": "" if coefficient["max_error"] is None else f"{coefficient['max_error']:.6f}",
                    "r_squared": "" if coefficient["r_squared"] is None else f"{coefficient['r_squared']:.9f}",
                    "direct_sample_count": coefficient["direct_sample_count"],
                    "edge_min_sample_count": coefficient["edge_min_sample_count"],
                    "fit_path": coefficient["fit_path"],
                    "raw_exclusion_limit": VALID_RAW_LIMIT,
                })
