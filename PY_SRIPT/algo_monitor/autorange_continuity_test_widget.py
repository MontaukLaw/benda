import csv
import math
import os
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from qt_compat import QtCore, QtWidgets, Signal, Slot


# 连续性测试固定覆盖七档，并从最灵敏档升到最大档后原路降回。
REF_CDAC_TIERS = (12, 18, 27, 40, 60, 90, 135)
TEST_SEQUENCE = REF_CDAC_TIERS + tuple(reversed(REF_CDAC_TIERS[:-1]))
MANUAL_MODE_FLAG = 0x40
AFFINE_ACTIVE_FLAG = 0x80
PENDING_FLAG = 0x01
VALID_RAW_LIMIT = 48321
NORMALIZED_FULL_SCALE = 58928
STATUS_QUERY_INTERVAL_SECONDS = 0.10
STEP_TIMEOUT_SECONDS = 10.0
SETTLE_CONFIRM_FRAMES = 3


def continuity_export_directory():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "calibration_exports"
    return Path(__file__).resolve().with_name("calibration_exports")


def percentile(values, fraction):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def make_station(target, observations):
    # 每个稳定档位先压缩成四通道均值、噪声和Hardware Raw峰值。
    raw_columns = [
        [observation["hardware_raw"][index] for observation in observations]
        for index in range(4)
    ]
    hardware_columns = [
        [observation["hardware_raw"][index] for observation in observations]
        for index in range(4)
    ]
    return {
        "cycle": int(target["cycle"]),
        "ref_cdac": int(target["ref_cdac"]),
        "command_time": float(target["command_time"]),
        "observations": tuple(observations),
        "raw_mean": tuple(statistics.fmean(column) for column in raw_columns),
        "raw_stddev": tuple(statistics.pstdev(column) for column in raw_columns),
        "hardware_mean": tuple(statistics.fmean(column) for column in hardware_columns),
        "hardware_maximum": tuple(max(column) for column in hardware_columns),
    }


def analyze_transition(previous_station, current_station, transition_observations, absolute_limit):
    # 相邻两个稳定档位分别生成CH4至CH7的连续性判定。
    direction = "升档" if current_station["ref_cdac"] > previous_station["ref_cdac"] else "降档"
    observations = list(transition_observations) + list(current_station["observations"])
    rows = []
    for channel_index, channel in enumerate(range(4, 8)):
        pre_mean = float(previous_station["raw_mean"][channel_index])
        post_mean = float(current_station["raw_mean"][channel_index])
        step_counts = post_mean - pre_mean
        absolute_step = abs(step_counts)
        combined_noise = math.hypot(
            previous_station["raw_stddev"][channel_index],
            current_station["raw_stddev"][channel_index],
        )
        series = [
            (float(observation["time"]), float(observation["hardware_raw"][channel_index]))
            for observation in observations
        ]
        peak_deviation = max(
            (abs(value - post_mean) for _sample_time, value in series),
            default=absolute_step,
        )
        overshoot = max(0.0, peak_deviation - absolute_step)
        settle_tolerance = max(10.0, 3.0 * current_station["raw_stddev"][channel_index])
        settling_ms = None
        for index in range(max(0, len(series) - SETTLE_CONFIRM_FRAMES + 1)):
            window = series[index:index + SETTLE_CONFIRM_FRAMES]
            if all(abs(value - post_mean) <= settle_tolerance for _sample_time, value in window):
                settling_ms = max(
                    0.0,
                    (window[0][0] - current_station["command_time"]) * 1000.0,
                )
                break
        saturated = (
            previous_station["hardware_maximum"][channel_index] >= VALID_RAW_LIMIT
            or current_station["hardware_maximum"][channel_index] >= VALID_RAW_LIMIT
        )
        if saturated:
            verdict = "无效"
            reason = "相邻档至少一侧Hardware Raw达到82%量程"
        else:
            warnings = []
            if absolute_step > absolute_limit * 2.0:
                verdict = "失败"
                warnings.append(f"稳态台阶超过{absolute_limit * 2.0:.0f} counts")
            else:
                verdict = "合格"
                if absolute_step > absolute_limit:
                    warnings.append(f"稳态台阶超过{absolute_limit:.0f} counts")
                if overshoot > absolute_limit:
                    warnings.append(f"过冲超过{absolute_limit:.0f} counts")
                if combined_noise > max(30.0, absolute_limit / 3.0):
                    warnings.append("稳态噪声偏大")
                if settling_ms is None:
                    warnings.append("过渡段未找到连续3帧稳定")
                if warnings:
                    verdict = "警告"
            reason = "；".join(warnings) if warnings else f"稳态台阶不超过{absolute_limit:.0f} counts"
        rows.append({
            "cycle": int(current_station["cycle"]),
            "direction": direction,
            "from_ref": int(previous_station["ref_cdac"]),
            "to_ref": int(current_station["ref_cdac"]),
            "channel": channel,
            "pre_raw_mean": pre_mean,
            "post_raw_mean": post_mean,
            "step_counts": step_counts,
            "absolute_step": absolute_step,
            "step_full_scale_percent": absolute_step * 100.0 / NORMALIZED_FULL_SCALE,
            "combined_noise": combined_noise,
            "peak_deviation": peak_deviation,
            "overshoot": overshoot,
            "settling_ms": settling_ms,
            "pre_hardware_mean": float(previous_station["hardware_mean"][channel_index]),
            "post_hardware_mean": float(current_station["hardware_mean"][channel_index]),
            "pre_hardware_maximum": int(previous_station["hardware_maximum"][channel_index]),
            "post_hardware_maximum": int(current_station["hardware_maximum"][channel_index]),
            "saturated": saturated,
            "verdict": verdict,
            "reason": reason,
        })
    return rows


def summarize_results(rows, stations):
    # 汇总只统计未饱和档位，避免把量程不足误判成仿射参数失配。
    usable = [row for row in rows if not row["saturated"]]
    absolute_steps = [row["absolute_step"] for row in usable]
    overshoots = [row["overshoot"] for row in usable]
    settling_values = [row["settling_ms"] for row in usable if row["settling_ms"] is not None]
    reference_stations = [station for station in stations if station["ref_cdac"] == REF_CDAC_TIERS[0]]
    round_trip_drift = 0.0
    for first, second in zip(reference_stations, reference_stations[1:]):
        for channel_index in range(4):
            round_trip_drift = max(
                round_trip_drift,
                abs(second["raw_mean"][channel_index] - first["raw_mean"][channel_index]),
            )
    bidirectional_drift = 0.0
    grouped_stations = {}
    for station in stations:
        grouped_stations.setdefault((station["cycle"], station["ref_cdac"]), []).append(station)
    for visits in grouped_stations.values():
        if len(visits) < 2:
            continue
        first = visits[0]
        second = visits[-1]
        for channel_index in range(4):
            bidirectional_drift = max(
                bidirectional_drift,
                abs(second["raw_mean"][channel_index] - first["raw_mean"][channel_index]),
            )
    up_steps = [row["absolute_step"] for row in usable if row["direction"] == "升档"]
    down_steps = [row["absolute_step"] for row in usable if row["direction"] == "降档"]
    verdict_counts = {
        verdict: sum(row["verdict"] == verdict for row in rows)
        for verdict in ("合格", "警告", "失败", "无效")
    }
    return {
        "row_count": len(rows),
        "usable_count": len(usable),
        "verdict_counts": verdict_counts,
        "step_median": statistics.median(absolute_steps) if absolute_steps else 0.0,
        "step_p95": percentile(absolute_steps, 0.95),
        "up_step_p95": percentile(up_steps, 0.95),
        "down_step_p95": percentile(down_steps, 0.95),
        "step_maximum": max(absolute_steps, default=0.0),
        "overshoot_maximum": max(overshoots, default=0.0),
        "settling_maximum_ms": max(settling_values, default=0.0),
        "round_trip_drift_maximum": round_trip_drift,
        "bidirectional_drift_maximum": bidirectional_drift,
    }


class AutorangeContinuityTestWidget(QtWidgets.QGroupBox):
    control_requested = Signal(bool, int)
    status_query_requested = Signal()
    status_message = Signal(str)
    operation_busy_changed = Signal(bool)

    def __init__(self, sensor_sample_provider, operation_available_provider=None, parent=None):
        super().__init__("档位连续性自动测试", parent)
        self.setObjectName("FeaturePanel")
        self.setCheckable(True)
        self.setChecked(False)
        self._sensor_sample_provider = sensor_sample_provider
        self._operation_available_provider = operation_available_provider
        self._connected = False
        self._external_busy = False
        self._device_status = None
        self._test_state = None
        self._last_sequence = None
        self._last_report = None
        self._last_export_path = None
        self._build_ui()
        self.toggled.connect(self._set_expanded)
        self._set_expanded(False)
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(8)
        self._timer.timeout.connect(self._poll_sample)
        self._timer.start()
        self._refresh_controls()

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 12, 8, 8)
        outer.setSpacing(6)
        self.content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        controls = QtWidgets.QGridLayout()
        controls.setHorizontalSpacing(8)
        controls.setVerticalSpacing(5)
        controls.addWidget(QtWidgets.QLabel("压力/备注"), 0, 0)
        self.label_edit = QtWidgets.QLineEdit()
        self.label_edit.setPlaceholderText("例如：500 g，保持静止")
        self.label_edit.setMinimumWidth(150)
        controls.addWidget(self.label_edit, 0, 1, 1, 5)
        controls.addWidget(QtWidgets.QLabel("循环"), 1, 0)
        self.repeat_spin = QtWidgets.QSpinBox()
        self.repeat_spin.setRange(1, 20)
        self.repeat_spin.setValue(3)
        repeat_line = QtWidgets.QHBoxLayout()
        repeat_line.setContentsMargins(0, 0, 0, 0)
        repeat_line.setSpacing(5)
        repeat_line.addWidget(self.repeat_spin, 1)
        repeat_line.addWidget(QtWidgets.QLabel("次"))
        controls.addLayout(repeat_line, 1, 1)
        controls.addWidget(QtWidgets.QLabel("稳定帧"), 1, 2)
        self.stable_frames_spin = QtWidgets.QSpinBox()
        self.stable_frames_spin.setRange(8, 128)
        self.stable_frames_spin.setValue(24)
        stable_frames_line = QtWidgets.QHBoxLayout()
        stable_frames_line.setContentsMargins(0, 0, 0, 0)
        stable_frames_line.setSpacing(5)
        stable_frames_line.addWidget(self.stable_frames_spin, 1)
        stable_frames_line.addWidget(QtWidgets.QLabel("帧"))
        controls.addLayout(stable_frames_line, 1, 3)
        controls.addWidget(QtWidgets.QLabel("台阶限值"), 1, 4)
        self.absolute_limit_spin = QtWidgets.QSpinBox()
        self.absolute_limit_spin.setRange(10, 5000)
        self.absolute_limit_spin.setValue(200)
        absolute_limit_line = QtWidgets.QHBoxLayout()
        absolute_limit_line.setContentsMargins(0, 0, 0, 0)
        absolute_limit_line.setSpacing(5)
        absolute_limit_line.addWidget(self.absolute_limit_spin, 1)
        absolute_limit_line.addWidget(QtWidgets.QLabel("counts"))
        controls.addLayout(absolute_limit_line, 1, 5)
        controls.setColumnStretch(1, 1)
        controls.setColumnStretch(3, 1)
        controls.setColumnStretch(5, 2)
        layout.addLayout(controls)

        actions = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("开始双向自动测试")
        self.start_button.clicked.connect(self.start_test)
        actions.addWidget(self.start_button)
        self.stop_button = QtWidgets.QPushButton("停止测试")
        self.stop_button.clicked.connect(self.stop_test)
        actions.addWidget(self.stop_button)
        self.export_button = QtWidgets.QPushButton("导出CSV报告")
        self.export_button.clicked.connect(self.export_report)
        actions.addWidget(self.export_button)
        actions.addStretch(1)
        layout.addLayout(actions)

        self.status_label = QtWidgets.QLabel(
            "等待开始：固定压力后自动执行12→18→27→40→60→90→135→…→12。"
        )
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.summary_label = QtWidgets.QLabel("尚无测试报告")
        self.summary_label.setWordWrap(True)
        layout.addWidget(self.summary_label)

        self.result_table = QtWidgets.QTableWidget(0, 17)
        self.result_table.setHorizontalHeaderLabels([
            "轮次", "方向", "Ref", "CH", "前Raw", "后Raw", "ΔRaw", "|Δ|",
            "%FS", "合并σ", "峰偏差", "过冲", "稳定ms", "前HW", "后HW", "饱和", "判定",
        ])
        self.result_table.verticalHeader().setVisible(False)
        no_edit = getattr(
            QtWidgets.QAbstractItemView,
            "NoEditTriggers",
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers,
        )
        self.result_table.setEditTriggers(no_edit)
        self.result_table.setMinimumHeight(250)
        self.result_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.result_table)
        outer.addWidget(self.content)

    @Slot(bool)
    def _set_expanded(self, expanded):
        self.content.setVisible(bool(expanded))
        self.setMinimumHeight(0)
        self.setMaximumHeight(16777215 if expanded else 32)

    def is_running(self):
        return self._test_state is not None

    def set_connected(self, connected):
        self._connected = bool(connected)
        if not self._connected and self._test_state is not None:
            self._abort_test("串口断开，连续性测试已经中止", restore_automatic=False)
        if not self._connected:
            self._device_status = None
        self._refresh_controls()

    def set_external_busy(self, busy):
        self._external_busy = bool(busy)
        self._refresh_controls()

    def on_device_status(self, frame):
        self._device_status = dict(frame)
        if self._test_state is not None and not bool(int(frame["flags"]) & AFFINE_ACTIVE_FLAG):
            self._abort_test("固件已退出仿射应用，测试结果无效")
            return
        self._refresh_controls()

    def control_command_failed(self, message):
        if self._test_state is not None:
            self._abort_test(f"档位控制失败：{message}", restore_automatic=False)

    def _operation_is_available(self):
        if self._operation_available_provider is None:
            return True
        result = self._operation_available_provider()
        if isinstance(result, tuple):
            return bool(result[0]), str(result[1])
        return bool(result), "仿射校准区正在执行其他操作"

    @Slot()
    def start_test(self):
        if not self._connected:
            self._set_status("串口未连接，无法开始连续性测试")
            return
        available = self._operation_is_available()
        if isinstance(available, tuple):
            allowed, reason = available
        else:
            allowed, reason = bool(available), "仿射校准区正在执行其他操作"
        if not allowed:
            self._set_status(reason)
            return
        if self._device_status is None or not bool(int(self._device_status["flags"]) & AFFINE_ACTIVE_FLAG):
            self._set_status("当前没有应用Flash仿射参数，请先完成参数写入与重启校验")
            return
        repeat_count = int(self.repeat_spin.value())
        targets = []
        for cycle in range(1, repeat_count + 1):
            sequence = TEST_SEQUENCE if cycle == 1 else TEST_SEQUENCE[1:]
            targets.extend({"cycle": cycle, "ref_cdac": ref_cdac} for ref_cdac in sequence)
        now = time.monotonic()
        self._test_state = {
            "test_id": uuid4().hex,
            "started_at": datetime.now().isoformat(timespec="milliseconds"),
            "started_monotonic": now,
            "label": self.label_edit.text().strip(),
            "repeat_count": repeat_count,
            "stable_frames": int(self.stable_frames_spin.value()),
            "absolute_limit": float(self.absolute_limit_spin.value()),
            "targets": targets,
            "target_index": 0,
            "phase": "starting",
            "phase_started": now,
            "last_status_query": 0.0,
            "capture": [],
            "transition": [],
            "stations": [],
            "rows": [],
            "samples": [],
        }
        self._last_sequence = None
        self._last_report = None
        self._last_export_path = None
        self.result_table.setRowCount(0)
        self.summary_label.setText("测试进行中，报告将在双向循环完成后生成。")
        self.operation_busy_changed.emit(True)
        self._refresh_controls()
        self._request_target(0)

    def _request_target(self, target_index):
        state = self._test_state
        if state is None:
            return
        target = state["targets"][target_index]
        now = time.monotonic()
        target["command_time"] = now
        state["target_index"] = target_index
        state["phase"] = "await_stable"
        state["phase_started"] = now
        state["capture"] = []
        state["transition"] = []
        # 控制命令发出后必须等待固件状态回读确认，不能按请求值直接采样。
        self.control_requested.emit(True, int(target["ref_cdac"]))
        if self._test_state is not state:
            return
        self._set_status(
            f"循环{target['cycle']}/{state['repeat_count']}，步骤{target_index + 1}/{len(state['targets'])}："
            f"正在切换到Ref{target['ref_cdac']}"
        )

    def _target_is_stable(self, target_ref):
        if self._device_status is None:
            return False
        flags = int(self._device_status["flags"])
        return (
            bool(flags & MANUAL_MODE_FLAG)
            and bool(flags & AFFINE_ACTIVE_FLAG)
            and not bool(flags & PENDING_FLAG)
            and int(self._device_status.get("blank_frames", -1)) == 0
            and int(self._device_status.get("actual_ref_cdac", -1)) == int(target_ref)
            and int(self._device_status.get("requested_ref_cdac", -2)) == int(target_ref)
        )

    def _make_observation(self, sample, sample_time):
        return {
            "time": float(sample_time),
            "hardware_raw": tuple(int(value) for value in sample["hardware_raw"][4:8]),
        }

    @Slot()
    def _poll_sample(self):
        state = self._test_state
        if state is None:
            return
        now = time.monotonic()
        if now - state["phase_started"] > STEP_TIMEOUT_SECONDS:
            target = state["targets"][state["target_index"]]
            self._abort_test(f"Ref{target['ref_cdac']}在{STEP_TIMEOUT_SECONDS:.0f}秒内未完成稳定采样")
            return
        if now - state["last_status_query"] >= STATUS_QUERY_INTERVAL_SECONDS:
            state["last_status_query"] = now
            self.status_query_requested.emit()
        sample = self._sensor_sample_provider()
        if sample is None or not sample.get("has_raw") or sample.get("age_seconds", 1.0) > 0.5:
            return
        sequence = int(sample["sequence"])
        if sequence == self._last_sequence:
            return
        self._last_sequence = sequence
        observation = self._make_observation(sample, now)
        target = state["targets"][state["target_index"]]
        stable = self._target_is_stable(target["ref_cdac"])
        device_status = self._device_status or {}
        state["samples"].append({
            "sample_index": len(state["samples"]),
            "elapsed_ms": (now - state["started_monotonic"]) * 1000.0,
            "phase": state["phase"],
            "cycle": int(target["cycle"]),
            "target_ref": int(target["ref_cdac"]),
            "actual_ref": int(device_status.get("actual_ref_cdac", -1)),
            "requested_ref": int(device_status.get("requested_ref_cdac", -1)),
            "blank_frames": int(device_status.get("blank_frames", -1)),
            "flags": int(device_status.get("flags", 0)),
            "stable": bool(stable),
            "hardware_raw": observation["hardware_raw"],
        })

        # 切档阶段记录过渡帧；状态稳定后才把新帧计入该档稳态窗口。
        if state["phase"] == "await_stable":
            if not stable:
                if state["stations"]:
                    state["transition"].append(observation)
                return
            state["phase"] = "capture"
            state["phase_started"] = now
            state["capture"] = []
        elif not stable:
            if state["stations"]:
                state["transition"].append(observation)
            state["phase"] = "await_stable"
            state["phase_started"] = now
            state["capture"] = []
            return

        state["capture"].append(observation)
        if len(state["capture"]) < state["stable_frames"]:
            self._set_status(
                f"循环{target['cycle']}/{state['repeat_count']}，Ref{target['ref_cdac']}已稳定："
                f"采集{len(state['capture'])}/{state['stable_frames']}帧"
            )
            return
        self._complete_station()

    def _complete_station(self):
        state = self._test_state
        if state is None:
            return
        target = state["targets"][state["target_index"]]
        station = make_station(target, state["capture"])
        if state["stations"]:
            rows = analyze_transition(
                state["stations"][-1],
                station,
                state["transition"],
                state["absolute_limit"],
            )
            state["rows"].extend(rows)
        state["stations"].append(station)
        next_index = state["target_index"] + 1
        if next_index >= len(state["targets"]):
            self._complete_test()
            return
        self._request_target(next_index)

    def _complete_test(self):
        state = self._test_state
        if state is None:
            return
        finished_at = datetime.now().isoformat(timespec="milliseconds")
        summary = summarize_results(state["rows"], state["stations"])
        self._last_report = {
            "test_id": state["test_id"],
            "started_at": state["started_at"],
            "finished_at": finished_at,
            "label": state["label"],
            "repeat_count": state["repeat_count"],
            "stable_frames": state["stable_frames"],
            "absolute_limit": state["absolute_limit"],
            "rows": list(state["rows"]),
            "samples": list(state["samples"]),
            "summary": summary,
        }
        self._test_state = None
        # 全部循环结束后主动退出手动档，恢复固件自动量程。
        self.control_requested.emit(False, 0)
        self.operation_busy_changed.emit(False)
        self._populate_report()
        try:
            self._last_export_path = self._automatic_export_path()
            self._write_report_csv(self._last_export_path)
            export_text = f"；报告已保存：{self._last_export_path.name}"
        except OSError as exc:
            export_text = f"；自动保存失败：{exc}"
        counts = summary["verdict_counts"]
        message = (
            f"连续性测试完成：合格{counts['合格']}、警告{counts['警告']}、"
            f"失败{counts['失败']}、无效{counts['无效']}；"
            f"|Δ| P95={summary['step_p95']:.1f}，最大={summary['step_maximum']:.1f} counts"
            f"{export_text}"
        )
        self._set_status(message)
        self._refresh_controls()

    @Slot()
    def stop_test(self):
        if self._test_state is not None:
            self._abort_test("用户停止测试")

    def _abort_test(self, reason, restore_automatic=True):
        if self._test_state is None:
            return
        self._test_state = None
        if restore_automatic and self._connected:
            self.control_requested.emit(False, 0)
        self.operation_busy_changed.emit(False)
        self._set_status(str(reason))
        self._refresh_controls()

    def _automatic_export_path(self):
        directory = continuity_export_directory()
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return directory / f"autorange_continuity_{timestamp}.csv"

    def _write_report_csv(self, path):
        report = self._last_report
        if report is None:
            raise OSError("没有可导出的测试报告")
        fields = [
            "record_type", "test_id", "started_at", "finished_at", "label", "repeat_count",
            "stable_frames", "absolute_limit", "summary_step_median", "summary_step_p95",
            "summary_up_step_p95", "summary_down_step_p95", "summary_step_maximum",
            "summary_overshoot_maximum", "summary_settling_maximum_ms",
            "summary_bidirectional_drift_maximum", "summary_ref12_round_trip_drift_maximum",
            "summary_pass_count", "summary_warning_count", "summary_failure_count",
            "summary_invalid_count", "cycle", "direction", "from_ref",
            "to_ref", "channel", "pre_raw_mean", "post_raw_mean", "step_counts",
            "absolute_step", "step_full_scale_percent", "combined_noise", "peak_deviation",
            "overshoot", "settling_ms", "pre_hardware_mean", "post_hardware_mean",
            "pre_hardware_maximum", "post_hardware_maximum", "saturated", "verdict", "reason",
            "sample_index", "elapsed_ms", "phase", "target_ref", "actual_ref", "requested_ref",
            "blank_frames", "flags", "stable", "ch4_hardware_raw", "ch5_hardware_raw",
            "ch6_hardware_raw", "ch7_hardware_raw",
        ]
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            # 单文件依次保存汇总、逐切档指标和完整帧轨迹，便于后续复盘。
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            common = {
                "test_id": report["test_id"],
                "started_at": report["started_at"],
                "finished_at": report["finished_at"],
                "label": report["label"],
                "repeat_count": report["repeat_count"],
                "stable_frames": report["stable_frames"],
                "absolute_limit": report["absolute_limit"],
            }
            summary = report["summary"]
            counts = summary["verdict_counts"]
            writer.writerow({
                **common,
                "record_type": "summary",
                "summary_step_median": summary["step_median"],
                "summary_step_p95": summary["step_p95"],
                "summary_up_step_p95": summary["up_step_p95"],
                "summary_down_step_p95": summary["down_step_p95"],
                "summary_step_maximum": summary["step_maximum"],
                "summary_overshoot_maximum": summary["overshoot_maximum"],
                "summary_settling_maximum_ms": summary["settling_maximum_ms"],
                "summary_bidirectional_drift_maximum": summary["bidirectional_drift_maximum"],
                "summary_ref12_round_trip_drift_maximum": summary["round_trip_drift_maximum"],
                "summary_pass_count": counts["合格"],
                "summary_warning_count": counts["警告"],
                "summary_failure_count": counts["失败"],
                "summary_invalid_count": counts["无效"],
            })
            for row in report["rows"]:
                writer.writerow({**common, "record_type": "transition", **row})
            for sample in report["samples"]:
                writer.writerow({
                    **common,
                    "record_type": "sample",
                    "cycle": sample["cycle"],
                    "sample_index": sample["sample_index"],
                    "elapsed_ms": sample["elapsed_ms"],
                    "phase": sample["phase"],
                    "target_ref": sample["target_ref"],
                    "actual_ref": sample["actual_ref"],
                    "requested_ref": sample["requested_ref"],
                    "blank_frames": sample["blank_frames"],
                    "flags": sample["flags"],
                    "stable": sample["stable"],
                    "ch4_hardware_raw": sample["hardware_raw"][0],
                    "ch5_hardware_raw": sample["hardware_raw"][1],
                    "ch6_hardware_raw": sample["hardware_raw"][2],
                    "ch7_hardware_raw": sample["hardware_raw"][3],
                })

    @Slot()
    def export_report(self):
        if self._last_report is None:
            self._set_status("尚无可导出的连续性测试报告")
            return
        suggested = self._last_export_path or self._automatic_export_path()
        selected_path, _filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "导出档位连续性测试报告",
            str(suggested),
            "CSV files (*.csv)",
        )
        if not selected_path:
            return
        path = Path(selected_path)
        if path.suffix.lower() != ".csv":
            path = path.with_suffix(".csv")
        try:
            self._write_report_csv(path)
        except OSError as exc:
            self._set_status(f"CSV报告导出失败：{exc}")
            return
        self._last_export_path = path
        self._set_status(f"CSV报告已导出：{path}")

    def _populate_report(self):
        report = self._last_report
        if report is None:
            return
        rows = report["rows"]
        self.result_table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            values = (
                row["cycle"], row["direction"], f"{row['from_ref']}→{row['to_ref']}",
                f"CH{row['channel']}", f"{row['pre_raw_mean']:.1f}",
                f"{row['post_raw_mean']:.1f}", f"{row['step_counts']:+.1f}",
                f"{row['absolute_step']:.1f}", f"{row['step_full_scale_percent']:.3f}",
                f"{row['combined_noise']:.2f}", f"{row['peak_deviation']:.1f}",
                f"{row['overshoot']:.1f}",
                "--" if row["settling_ms"] is None else f"{row['settling_ms']:.1f}",
                f"{row['pre_hardware_mean']:.1f}", f"{row['post_hardware_mean']:.1f}",
                "是" if row["saturated"] else "否", row["verdict"],
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(value))
                item.setToolTip(row["reason"])
                self.result_table.setItem(row_index, column, item)
        self.result_table.resizeColumnsToContents()
        summary = report["summary"]
        counts = summary["verdict_counts"]
        self.summary_label.setText(
            f"稳态台阶：中位数{summary['step_median']:.1f}，P95={summary['step_p95']:.1f}，"
            f"升/降档P95={summary['up_step_p95']:.1f}/{summary['down_step_p95']:.1f}，"
            f"最大{summary['step_maximum']:.1f} counts；最大过冲{summary['overshoot_maximum']:.1f}；"
            f"最长稳定{summary['settling_maximum_ms']:.1f} ms；"
            f"同Ref双向最大漂移{summary['bidirectional_drift_maximum']:.1f}，"
            f"Ref12往返最大漂移{summary['round_trip_drift_maximum']:.1f} counts；"
            f"合格{counts['合格']} / 警告{counts['警告']} / 失败{counts['失败']} / 无效{counts['无效']}。"
        )

    def _set_status(self, message):
        self.status_label.setText(str(message))
        self.status_message.emit(str(message))

    def _refresh_controls(self):
        running = self._test_state is not None
        self.start_button.setEnabled(self._connected and not running and not self._external_busy)
        self.stop_button.setEnabled(running)
        self.export_button.setEnabled(self._last_report is not None and not running)
        self.label_edit.setEnabled(not running)
        self.repeat_spin.setEnabled(not running)
        self.stable_frames_spin.setEnabled(not running)
        self.absolute_limit_spin.setEnabled(not running)
