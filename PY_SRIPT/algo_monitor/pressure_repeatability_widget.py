"""Pressure-meter force repeatability test for the PD42S1 motor."""

from __future__ import annotations

import csv
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from qt_compat import QtCore, QtWidgets, Signal, Slot
from step_moto_can_driver import PD42S1Motor


POSITION_COUNT_UNIT = 4096
TELEMETRY_INTERVAL_SECONDS = 0.1
GRAMS_PER_NEWTON = 1000.0 / 9.80665
MAX_FORCE_G = 30.0 * GRAMS_PER_NEWTON
DEFAULT_TARGET_FORCE_G = 10.0 * GRAMS_PER_NEWTON
DEFAULT_RELEASE_FORCE_G = 0.03 * GRAMS_PER_NEWTON
DEFAULT_PRESSURE_TIMEOUT_SECONDS = 1.0
MOTOR_UI_MAX_RPM = 400

RUNNING_STATE_NAMES = {
    0: "停止",
    1: "任务完成",
    2: "正在运行",
    3: "过载",
    4: "堵转",
    5: "欠压",
}

STOP_REASON_NAMES = {
    "target_force_reached": "达到目标压力",
    "release_pressure_reached": "压力已释放",
    "force_safety_limit": f"达到 {MAX_FORCE_G:.0f} g 安全上限",
    "position_reached": "完成设定行程",
    "motion_timeout": "运动到位超时",
    "ocr_invalid": "压力计读数无效",
    "ocr_timeout": "压力计读数超时",
    "user_stop": "用户紧急停止",
}


def speed_level_to_counts_per_second(speed_level):
    return int(speed_level) * POSITION_COUNT_UNIT


def counts_per_second_to_rpm(speed_counts_per_second):
    rpm = round(
        int(speed_counts_per_second) * 60.0 / PD42S1Motor.COUNTS_PER_TURN
    )
    return max(1, min(PD42S1Motor.MAX_SPEED_RPM, rpm))


def maximum_speed_level():
    counts_per_second = round(
        MOTOR_UI_MAX_RPM * PD42S1Motor.COUNTS_PER_TURN / 60.0
    )
    return max(1, round(counts_per_second / POSITION_COUNT_UNIT))


def pressure_test_directory():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "pressure_repeatability_tests"
    return Path(__file__).resolve().with_name("pressure_repeatability_tests")


class PressureCsvRecorder:
    """Write each 100 ms pressure/motor sample directly to a recoverable CSV."""

    FIELD_NAMES = (
        "timestamp",
        "session_elapsed_ms",
        "sequence",
        "test_id",
        "control_mode",
        "phase",
        "event",
        "target_force_g",
        "force_g",
        "force_age_ms",
        "position_counts",
        "relative_position_counts",
        "position_deg",
        "measured_speed_rpm",
        "position_speed_counts_s",
        "estimated_acceleration_r_s2",
        "sample_interval_ms",
        "phase_current_ma",
        "running_state",
        "running_state_name",
        "command_speed_level",
        "command_speed_counts_s",
        "command_speed_rpm",
        "command_acceleration_r_s2",
        "max_travel_level",
        "max_travel_counts",
        "command_travel_counts",
        "trigger_force_g",
        "peak_force_g",
        "end_force_g",
        "overshoot_g",
        "final_position_counts",
        "stop_reason",
        "ocr_status",
    )

    def __init__(self, output_dir=None):
        self.output_dir = Path(output_dir) if output_dir else pressure_test_directory()
        self._lock = threading.Lock()
        self._handle = None
        self._writer = None
        self._path = None
        self._started_at = 0.0
        self._row_count = 0

    @property
    def is_recording(self):
        with self._lock:
            return self._handle is not None

    @property
    def path(self):
        with self._lock:
            return self._path

    def start(self):
        with self._lock:
            if self._handle is not None:
                return None
            self.output_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self._path = self.output_dir / f"pressure_repeatability_{stamp}.csv"
            self._handle = self._path.open("w", encoding="utf-8-sig", newline="")
            self._writer = csv.DictWriter(
                self._handle,
                fieldnames=self.FIELD_NAMES,
                extrasaction="ignore",
            )
            self._writer.writeheader()
            self._handle.flush()
            self._started_at = time.monotonic()
            self._row_count = 0
            return self._path

    def record(self, values):
        with self._lock:
            if self._handle is None:
                return False
            row = {name: "" for name in self.FIELD_NAMES}
            row.update(values)
            row["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
            row["session_elapsed_ms"] = round(
                (time.monotonic() - self._started_at) * 1000.0,
                3,
            )
            row["sequence"] = self._row_count
            self._writer.writerow(row)
            self._handle.flush()
            self._row_count += 1
            return True

    def stop(self):
        with self._lock:
            if self._handle is None:
                return None, 0
            self._handle.flush()
            self._handle.close()
            path = self._path
            row_count = self._row_count
            self._handle = None
            self._writer = None
            self._path = None
            return path, row_count


class PressureRepeatabilityWidget(QtWidgets.QWidget):
    """使用压力计串口值记录相对位置重复运动的压力结果。"""

    status_message = Signal(str)
    operation_finished = Signal(object)

    def __init__(self, motor_control_widget, parent=None):
        super().__init__(parent)
        self._motor_control = motor_control_widget
        self._pressure_lock = threading.Lock()
        self._pressure_value = None
        self._pressure_status = "等待压力计串口"
        self._pressure_updated_at = 0.0
        self._pressure_sequence = 0
        self._operation_kind = None
        self._manual_next_kind = "press"
        self._repeat_running = False
        self._repeat_total = 0
        self._repeat_completed = 0
        self._repeat_phase = None
        self._repeat_press_result = None
        self._test_id = 0
        self._press_results = []
        self._motion_results = []
        self._closing = False
        self._recorder = PressureCsvRecorder()
        self._repeat_hold_timer = QtCore.QTimer(self)
        self._repeat_hold_timer.setSingleShot(True)
        self._repeat_hold_timer.timeout.connect(self._capture_repeat_pressure)
        self._build_ui()
        self.operation_finished.connect(self._finish_operation)
        self._motor_control.motor_feedback.connect(self._on_shared_motor_feedback)
        self._motor_control.control_state_changed.connect(self._on_motor_state_changed)
        self._motor_control.worker_position_zeroed.connect(self._on_position_zeroed)
        latest_feedback = self._motor_control.latest_motor_feedback()
        if latest_feedback is not None:
            self._show_telemetry(latest_feedback)
        self._on_motor_state_changed(
            self._motor_control.is_can_connected(),
            self._motor_control.is_operation_busy(),
        )

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        explanation = QtWidgets.QLabel(
            "当前为闭环相对位置测试：主运动按钮在下压和抬起之间自动切换，运动未到位时保持灰色。"
            "重复运动会依次执行下压、到位等待并采集压力计读数、抬起，再进入下一轮。"
            "不要求先保存数据，也不使用目标压力自动停止。保存数据可在运动前、运动中或运动后独立启停。"
            f"下压时若压力计读到有效的{MAX_FORCE_G:.0f} g压力仍会安全刹车。"
        )
        explanation.setWordWrap(True)
        explanation.setStyleSheet("color:#8bd5ff;")
        outer.addWidget(explanation)

        self.force_label = QtWidgets.QLabel("压力计 --.-- g")
        self.force_label.setObjectName("PressureLiveValue")
        outer.addWidget(self.force_label)

        self.motor_label = QtWidgets.QLabel(
            "电机反馈  位置 -- counts · 速度 -- RPM · "
            "估算加速度 -- r/s² · 瞬时相电流 -- mA"
        )
        self.motor_label.setObjectName("PressureMotorValue")
        self.motor_label.setWordWrap(True)
        motor_feedback_row = QtWidgets.QHBoxLayout()
        motor_feedback_row.setContentsMargins(0, 0, 0, 0)
        motor_feedback_row.setSpacing(8)
        motor_feedback_row.addWidget(self.motor_label, 1)
        self.zero_position_button = QtWidgets.QPushButton("当前位置清零")
        self.zero_position_button.setObjectName("PressureZeroButton")
        self.zero_position_button.setFixedWidth(120)
        self.zero_position_button.setToolTip(
            "向驱动板发送 0xF8，将当前角度、位置误差和累计脉冲数清零"
        )
        self.zero_position_button.clicked.connect(
            self._motor_control.zero_motor_position
        )
        motor_feedback_row.addWidget(self.zero_position_button)
        outer.addLayout(motor_feedback_row)

        parameter_group = QtWidgets.QGroupBox("测试参数")
        grid = QtWidgets.QGridLayout(parameter_group)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(7)

        self.target_force_input = QtWidgets.QDoubleSpinBox()
        self.target_force_input.setRange(1.0, MAX_FORCE_G)
        self.target_force_input.setDecimals(2)
        self.target_force_input.setSingleStep(10.0)
        self.target_force_input.setValue(DEFAULT_TARGET_FORCE_G)
        self.target_force_input.setEnabled(False)
        self.target_force_input.setToolTip("自动压力控制预留参数，当前手动测试不参与停止判断")
        self._add_value_with_unit(grid, "目标压力（预留）", self.target_force_input, "g", 0, 0)

        self.speed_level_input = QtWidgets.QSpinBox()
        self.speed_level_input.setRange(1, maximum_speed_level())
        self.speed_level_input.setValue(self._motor_control.speed_input.value())
        self._add_value_with_unit(grid, "速度档位", self.speed_level_input, "档", 0, 2)

        self.acceleration_input = QtWidgets.QSpinBox()
        self.acceleration_input.setRange(0, 200)
        self.acceleration_input.setValue(self._motor_control.acceleration_input.value())
        self._add_value_with_unit(grid, "加减速度", self.acceleration_input, "r/s²", 1, 0)

        self.max_travel_input = QtWidgets.QSpinBox()
        self.max_travel_input.setRange(1, 100)
        self.max_travel_input.setValue(max(1, self._motor_control.move_count_units_input.value()))
        self.max_travel_input.setToolTip("每次下压或抬起实际执行的闭环相对位置行程")
        self._add_value_with_unit(grid, "运动行程", self.max_travel_input, "档", 1, 2)

        self.release_force_input = QtWidgets.QDoubleSpinBox()
        self.release_force_input.setRange(0.0, 100.0)
        self.release_force_input.setDecimals(2)
        self.release_force_input.setSingleStep(1.0)
        self.release_force_input.setValue(DEFAULT_RELEASE_FORCE_G)
        self.release_force_input.setEnabled(False)
        self.release_force_input.setToolTip("自动压力控制预留参数，当前抬起只按运动行程执行")
        self._add_value_with_unit(grid, "释放压力（预留）", self.release_force_input, "g", 2, 0)

        self.ocr_timeout_input = QtWidgets.QDoubleSpinBox()
        self.ocr_timeout_input.setRange(0.3, 5.0)
        self.ocr_timeout_input.setDecimals(1)
        self.ocr_timeout_input.setSingleStep(0.1)
        self.ocr_timeout_input.setValue(DEFAULT_PRESSURE_TIMEOUT_SECONDS)
        self.ocr_timeout_input.setEnabled(False)
        self.ocr_timeout_input.setToolTip("自动压力控制预留参数，当前压力计失联不阻止定行程运动")
        self._add_value_with_unit(grid, "压力计超时（预留）", self.ocr_timeout_input, "s", 2, 2)

        self.settle_time_input = QtWidgets.QDoubleSpinBox()
        self.settle_time_input.setRange(0.2, 5.0)
        self.settle_time_input.setDecimals(1)
        self.settle_time_input.setSingleStep(0.1)
        self.settle_time_input.setValue(1.0)
        self.settle_time_input.setToolTip("重复运动下压到位后，等待这段时间再采集压力计读数")
        self._add_value_with_unit(grid, "压力采样等待", self.settle_time_input, "s", 3, 0)

        self.repeat_count_input = QtWidgets.QSpinBox()
        self.repeat_count_input.setRange(1, 1000)
        self.repeat_count_input.setValue(5)
        self.repeat_count_input.setToolTip("一次重复包含一次下压、压力采样和一次抬起")
        self._add_value_with_unit(grid, "重复次数", self.repeat_count_input, "次", 3, 2)

        safety_label = QtWidgets.QLabel(f"安全上限 {MAX_FORCE_G:.0f} g（固定）")
        safety_label.setStyleSheet("color:#ff9f43;font-weight:700;")
        grid.addWidget(safety_label, 4, 0, 1, 4)
        outer.addWidget(parameter_group)

        record_row = QtWidgets.QHBoxLayout()
        self.start_record_button = QtWidgets.QPushButton("开始保存数据")
        self.start_record_button.setObjectName("PressureRecordButton")
        self.start_record_button.clicked.connect(self.start_recording)
        record_row.addWidget(self.start_record_button)
        self.stop_record_button = QtWidgets.QPushButton("停止保存数据")
        self.stop_record_button.clicked.connect(self.stop_recording)
        record_row.addWidget(self.stop_record_button)
        self.record_status_label = QtWidgets.QLabel("尚未保存；保存状态不影响下压和抬起")
        self.record_status_label.setWordWrap(True)
        record_row.addWidget(self.record_status_label, 1)
        outer.addLayout(record_row)

        action_row = QtWidgets.QHBoxLayout()
        self.motion_button = QtWidgets.QPushButton("开始下压")
        self.motion_button.setObjectName("PressureMotionButton")
        self.motion_button.clicked.connect(self.start_next_motion)
        action_row.addWidget(self.motion_button)
        self.repeat_button = QtWidgets.QPushButton("开始重复运动")
        self.repeat_button.setObjectName("PressureRepeatButton")
        self.repeat_button.clicked.connect(self.start_repeat_motion)
        action_row.addWidget(self.repeat_button)
        self.stop_button = QtWidgets.QPushButton("紧急停止")
        self.stop_button.setObjectName("PressureStopButton")
        self.stop_button.clicked.connect(self.request_stop)
        action_row.addWidget(self.stop_button)
        outer.addLayout(action_row)

        self.status_label = QtWidgets.QLabel("等待开始压力重复精度测试")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color:#f4d35e;font-weight:600;")
        outer.addWidget(self.status_label)

        self.summary_label = QtWidgets.QLabel("重复精度：尚无有效下压结果")
        self.summary_label.setWordWrap(True)
        summary_row = QtWidgets.QHBoxLayout()
        summary_row.setContentsMargins(0, 0, 0, 0)
        summary_row.setSpacing(8)
        summary_row.addWidget(self.summary_label, 1)
        self.clear_results_button = QtWidgets.QPushButton("清空下方结果")
        self.clear_results_button.setObjectName("PressureClearResultsButton")
        self.clear_results_button.setFixedWidth(120)
        self.clear_results_button.setToolTip(
            "仅清空下方结果表格和本次统计，不删除已保存的CSV，也不清零电机位置"
        )
        self.clear_results_button.clicked.connect(self.clear_results)
        summary_row.addWidget(self.clear_results_button)
        outer.addLayout(summary_row)

        self.result_table = QtWidgets.QTableWidget(0, 9)
        self.result_table.setHorizontalHeaderLabels(
            (
                "序号",
                "动作",
                "指令档位",
                "指令/counts",
                "实际位移/counts",
                "结束压力/g",
                "等待后压力/g",
                "峰值/g",
                "结果",
            )
        )
        self.result_table.setEditTriggers(
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.result_table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.result_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeMode.Stretch
        )
        self.result_table.setMinimumHeight(220)
        outer.addWidget(self.result_table, 1)

        self.setStyleSheet(
            "QLabel#PressureLiveValue{color:#57d5ff;font-size:22px;font-weight:700;}"
            "QLabel#PressureMotorValue{color:#b9e6ff;font-family:Consolas,'Microsoft YaHei';font-size:18px;font-weight:700;}"
            "QPushButton{min-height:30px;font-weight:700;}"
            "QPushButton#PressureMotionButton{background:#8fd3a8;color:#17212b;}"
            "QPushButton#PressureMotionButton:hover{background:#a9e2bc;}"
            "QPushButton#PressureMotionButton:disabled{background:#65717c;color:#c2c9cf;}"
            "QPushButton#PressureRepeatButton{background:#bdd6e8;color:#17212b;}"
            "QPushButton#PressureRepeatButton:disabled{background:#65717c;color:#c2c9cf;}"
            "QPushButton#PressureStopButton{background:#edbbb6;color:#17212b;}"
            "QPushButton#PressureRecordButton{background:#c9d6e9;color:#17212b;}"
            "QPushButton#PressureZeroButton{background:#ead7a1;color:#17212b;}"
            "QPushButton#PressureClearResultsButton{background:#d6dce5;color:#17212b;}"
        )
        self._refresh_controls()

    @staticmethod
    def _add_value_with_unit(grid, label_text, input_widget, unit_text, row, column):
        grid.addWidget(QtWidgets.QLabel(label_text), row, column)
        value_layout = QtWidgets.QHBoxLayout()
        value_layout.setContentsMargins(0, 0, 0, 0)
        value_layout.setSpacing(5)
        input_widget.setFixedWidth(105)
        value_layout.addWidget(input_widget)
        value_layout.addWidget(QtWidgets.QLabel(unit_text))
        value_layout.addStretch(1)
        grid.addLayout(value_layout, row, column + 1)

    @Slot(object, str)
    def set_pressure_meter_pressure(self, value, status):
        now = time.monotonic()
        with self._pressure_lock:
            self._pressure_value = None if value is None else float(value)
            self._pressure_status = str(status)
            self._pressure_updated_at = now
            self._pressure_sequence += 1
            current = self._pressure_value
        if current is None:
            self.force_label.setText("压力计 --.-- g")
            self.force_label.setStyleSheet("color:#ff7b72;font-size:22px;font-weight:700;")
        else:
            self.force_label.setText(f"压力计 {current:.2f} g")
            color = "#ff7b72" if current >= MAX_FORCE_G else "#57d5ff"
            self.force_label.setStyleSheet(
                f"color:{color};font-size:22px;font-weight:700;"
            )

    def _pressure_snapshot(self):
        with self._pressure_lock:
            return (
                self._pressure_value,
                self._pressure_status,
                self._pressure_updated_at,
                self._pressure_sequence,
            )

    @Slot()
    def start_recording(self):
        try:
            path = self._recorder.start()
        except OSError as exc:
            self._set_status(f"开始保存失败：{exc}")
            return
        if path is None:
            self._set_status("数据保存已经开始")
            return
        self.record_status_label.setText(f"正在保存：{path}")
        self._set_status("已经开始保存共享 CAN 反馈与压力数据")
        self._refresh_controls()

    @Slot()
    def stop_recording(self):
        try:
            path, rows = self._recorder.stop()
        except OSError as exc:
            self._set_status(f"停止保存失败：{exc}")
            return
        if path is None:
            self._set_status("当前没有正在保存的数据")
            return
        self.record_status_label.setText(f"已保存 {rows} 行：{path}")
        self._set_status(f"CSV 保存完成：{path}")
        self._refresh_controls()

    @Slot(object)
    def _on_shared_motor_feedback(self, telemetry):
        """显示并按需保存 CAN 服务线程广播的同一份反馈快照。"""
        self._show_telemetry(telemetry)
        if not self._recorder.is_recording or self._operation_kind is not None:
            return
        sampled_at = float(telemetry.get("sampled_monotonic", time.monotonic()))
        pressure, ocr_status, pressure_updated_at, _ = self._pressure_snapshot()
        row = dict(telemetry)
        row.update(
            {
                "test_id": self._test_id,
                "control_mode": "independent_recording",
                "phase": "idle",
                "event": "idle_sample",
                "target_force_g": "",
                "force_g": pressure,
                "force_age_ms": (
                    math.inf
                    if pressure_updated_at <= 0
                    else round((sampled_at - pressure_updated_at) * 1000.0, 3)
                ),
                "ocr_status": ocr_status,
            }
        )
        self._recorder.record(row)

    @Slot()
    def start_press(self):
        return self._start_operation("press")

    @Slot()
    def start_lift(self):
        return self._start_operation("lift")

    @Slot()
    def start_next_motion(self):
        """按最近一次完成的动作，在下压和抬起之间切换。"""
        if self._repeat_running:
            self._set_status("重复运动正在执行")
            return
        self._start_operation(self._manual_next_kind)

    @Slot()
    def start_repeat_motion(self):
        """启动由独立闭环下压、压力采样和闭环抬起组成的重复运动。"""
        if self._repeat_running:
            self._set_status("重复运动已经开始")
            return
        if self._operation_kind is not None or self._motor_control.is_operation_busy():
            self._set_status("当前电机运动尚未完成，不能开始重复运动")
            return
        if not self._motor_control.is_can_connected():
            self._set_status("CAN 未连接，不能开始重复运动")
            return
        if self._manual_next_kind != "press":
            self._set_status("当前位置等待抬起，请先完成抬起再开始重复运动")
            return

        self._repeat_running = True
        self._repeat_total = self.repeat_count_input.value()
        self._repeat_completed = 0
        self._repeat_phase = "starting_press"
        self._repeat_press_result = None
        self._set_status(f"重复运动开始：共 {self._repeat_total} 次")
        self._refresh_controls()
        self._start_repeat_operation("press")

    def _start_repeat_operation(self, kind):
        cycle = self._repeat_completed + 1
        self._repeat_phase = f"starting_{kind}"
        if self._start_operation(kind, control_mode="automatic_repeat"):
            self._repeat_phase = f"running_{kind}"
            self._set_status(
                f"重复运动 {cycle}/{self._repeat_total}："
                f"正在{'下压' if kind == 'press' else '抬起'}"
            )
            return
        self._abort_repeat(f"重复运动无法启动：{self.status_label.text()}")

    def _start_operation(self, kind, control_mode="manual_relative_position"):
        if self._operation_kind is not None:
            self._set_status("已有压力测试运动正在执行")
            return False
        pressure, _, updated_at, _ = self._pressure_snapshot()
        pressure_is_fresh = (
            updated_at > 0
            and time.monotonic() - updated_at <= self.ocr_timeout_input.value()
        )
        if (
            kind == "press"
            and pressure_is_fresh
            and pressure is not None
            and pressure >= MAX_FORCE_G
        ):
            self._manual_next_kind = "lift"
            self._set_status(f"当前压力已达到安全上限 {MAX_FORCE_G:.0f} g，禁止运动")
            self._refresh_controls()
            return False

        speed_level = self.speed_level_input.value()
        speed_counts_s = speed_level_to_counts_per_second(speed_level)
        parameters = {
            "kind": kind,
            "test_id": self._test_id + 1 if kind == "press" else self._test_id,
            "speed_level": speed_level,
            "speed_counts_s": speed_counts_s,
            "speed_rpm": counts_per_second_to_rpm(speed_counts_s),
            "acceleration": self.acceleration_input.value(),
            "max_travel_level": self.max_travel_input.value(),
            "max_travel_counts": self.max_travel_input.value() * POSITION_COUNT_UNIT,
            "ocr_timeout_seconds": self.ocr_timeout_input.value(),
            "motion_timeout_seconds": max(
                8.0,
                self.max_travel_input.value()
                * POSITION_COUNT_UNIT
                / max(1, speed_counts_s)
                * 3.0
                + 2.0,
            ),
            "control_mode": control_mode,
        }
        if kind == "press":
            self._test_id += 1
            parameters["test_id"] = self._test_id

        action = "下压" if kind == "press" else "抬起"
        self._operation_kind = kind
        accepted, reason = self._motor_control.start_external_operation(
            lambda motor, stop_event: self._run_motion(motor, stop_event, parameters),
            f"压力重复精度测试：正在{action}",
        )
        if not accepted:
            self._operation_kind = None
            self._set_status(f"无法开始{action}：{reason}")
            self._refresh_controls()
            return False
        self._set_status(
            f"测试 {parameters['test_id']} 正在{action}：运动行程 "
            f"{parameters['max_travel_level']} 档，速度档位 {speed_level}，"
            f"加减速度 {parameters['acceleration']} r/s²"
        )
        self._refresh_controls()
        return True

    def _run_motion(self, motor, stop_event, parameters):
        kind = parameters["kind"]
        direction = -1 if kind == "press" else 1
        operation_started_at = time.monotonic()
        start_position = None
        target_position = None
        final_position = None
        trigger_force = None
        peak_force = None
        end_force = None
        stop_reason = "unknown"
        command_sent = False
        last_position = None
        last_sample_at = None
        last_position_speed = None
        last_telemetry = None

        try:
            motor.clear_state()
            start_position = motor.read_position_counts()
            target_position = (
                start_position + direction * parameters["max_travel_counts"]
            )
            self._motor_control.record_motion_diagnostic(
                "command_requested",
                test_id=parameters["test_id"],
                action=kind,
                control_mode=parameters["control_mode"],
                direction_byte=1 if direction < 0 else 0,
                start_position_counts=start_position,
                target_position_counts=target_position,
                command_travel_counts=(
                    direction * parameters["max_travel_counts"]
                ),
                command_speed_level=parameters["speed_level"],
                command_speed_counts_s=parameters["speed_counts_s"],
                command_speed_rpm=parameters["speed_rpm"],
                command_acceleration_r_s2=parameters["acceleration"],
            )
            motor.move_relative_counts(
                direction * parameters["max_travel_counts"],
                speed_rpm=parameters["speed_rpm"],
                accel=parameters["acceleration"],
                retry_cancelled=stop_event.is_set,
            )
            command_sent = True
            self._motor_control.record_motion_diagnostic(
                "command_accepted",
                test_id=parameters["test_id"],
                action=kind,
                control_mode=parameters["control_mode"],
                target_position_counts=target_position,
            )
            motion_deadline = time.monotonic() + parameters["motion_timeout_seconds"]
            self._record_event(parameters, kind, "motion_started", start_position)

            while command_sent and stop_reason == "unknown":
                sample_started = time.monotonic()
                telemetry = self._read_telemetry(
                    motor,
                    parameters,
                    kind,
                    start_position,
                    last_position,
                    last_sample_at,
                    last_position_speed,
                )
                last_telemetry = telemetry
                last_position = telemetry["position_counts"]
                last_sample_at = sample_started
                last_position_speed = telemetry["position_speed_counts_s"]
                final_position = last_position
                pressure = telemetry["force_g"]
                end_force = pressure
                if pressure is not None:
                    peak_force = pressure if peak_force is None else max(peak_force, pressure)

                if stop_event.is_set():
                    stop_reason = "user_stop"
                elif (
                    kind == "press"
                    and pressure is not None
                    and telemetry["force_age_ms"]
                    <= parameters["ocr_timeout_seconds"] * 1000.0
                    and pressure >= MAX_FORCE_G
                ):
                    stop_reason = "force_safety_limit"
                elif telemetry["running_state"] in (3, 4, 5):
                    stop_reason = f"motor_{RUNNING_STATE_NAMES[telemetry['running_state']]}"
                elif telemetry["running_state"] in (0, 1):
                    if abs(telemetry["position_counts"] - target_position) <= 50:
                        stop_reason = "position_reached"
                if stop_reason == "unknown" and time.monotonic() >= motion_deadline:
                    stop_reason = "motion_timeout"

                telemetry["event"] = "sample" if stop_reason == "unknown" else stop_reason
                telemetry["stop_reason"] = "" if stop_reason == "unknown" else stop_reason
                self._recorder.record(telemetry)
                self._motor_control.record_motion_diagnostic(
                    "feedback",
                    test_id=parameters["test_id"],
                    action=kind,
                    elapsed_ms=round(
                        (time.monotonic() - operation_started_at) * 1000.0,
                        3,
                    ),
                    position_counts=telemetry["position_counts"],
                    target_position_counts=target_position,
                    actual_minus_target_counts=(
                        telemetry["position_counts"] - target_position
                    ),
                    measured_speed_rpm=telemetry["measured_speed_rpm"],
                    position_speed_counts_s=telemetry["position_speed_counts_s"],
                    estimated_acceleration_r_s2=telemetry[
                        "estimated_acceleration_r_s2"
                    ],
                    phase_current_ma=telemetry["phase_current_ma"],
                    running_state=telemetry["running_state"],
                    running_state_name=telemetry["running_state_name"],
                    force_g=telemetry["force_g"],
                    decision=telemetry["event"],
                )
                if stop_reason != "unknown":
                    break
                elapsed = time.monotonic() - sample_started
                stop_event.wait(max(0.0, TELEMETRY_INTERVAL_SECONDS - elapsed))

            if command_sent and stop_reason != "position_reached":
                motor.brake_stop(auto_clear=False)
                if last_telemetry is not None:
                    final_position = last_telemetry["position_counts"]

            if final_position is None:
                final_position = motor.read_position_counts()
            ok = stop_reason == "position_reached"
            result = {
                "ok": ok,
                "kind": kind,
                "test_id": parameters["test_id"],
                "control_mode": parameters["control_mode"],
                "trigger_force_g": trigger_force,
                "peak_force_g": peak_force,
                "end_force_g": end_force,
                "overshoot_g": None,
                "command_travel_level": parameters["max_travel_level"],
                "command_travel_counts": direction * parameters["max_travel_counts"],
                "start_position_counts": start_position,
                "final_position_counts": final_position,
                "relative_position_counts": final_position - start_position,
                "stop_reason": stop_reason,
            }
            self._record_event(
                parameters,
                kind,
                "operation_complete",
                final_position,
                result,
            )
            self._motor_control.record_motion_diagnostic(
                "operation_complete",
                test_id=parameters["test_id"],
                action=kind,
                control_mode=parameters["control_mode"],
                elapsed_ms=round(
                    (time.monotonic() - operation_started_at) * 1000.0,
                    3,
                ),
                target_position_counts=target_position,
                final_position_counts=final_position,
                actual_minus_target_counts=(
                    None
                    if target_position is None or final_position is None
                    else final_position - target_position
                ),
                stop_reason=stop_reason,
                peak_force_g=peak_force,
                end_force_g=end_force,
            )
            self.operation_finished.emit(result)
        except Exception as exc:
            if command_sent:
                try:
                    motor.brake_stop(auto_clear=False)
                except Exception:
                    pass
            result = {
                "ok": False,
                "kind": kind,
                "test_id": parameters["test_id"],
                "control_mode": parameters["control_mode"],
                "trigger_force_g": trigger_force,
                "peak_force_g": peak_force,
                "end_force_g": end_force,
                "overshoot_g": None,
                "command_travel_level": parameters["max_travel_level"],
                "command_travel_counts": direction * parameters["max_travel_counts"],
                "start_position_counts": start_position,
                "final_position_counts": final_position,
                "relative_position_counts": (
                    None
                    if start_position is None or final_position is None
                    else final_position - start_position
                ),
                "stop_reason": f"error: {exc}",
            }
            self._record_event(parameters, kind, "operation_error", final_position, result)
            self._motor_control.record_motion_diagnostic(
                "operation_error",
                test_id=parameters["test_id"],
                action=kind,
                elapsed_ms=round(
                    (time.monotonic() - operation_started_at) * 1000.0,
                    3,
                ),
                target_position_counts=target_position,
                final_position_counts=final_position,
                error=str(exc),
            )
            self.operation_finished.emit(result)

    def _read_telemetry(
        self,
        motor,
        parameters,
        phase,
        start_position,
        last_position,
        last_sample_at,
        last_position_speed,
    ):
        sampled_at = time.monotonic()
        position = motor.read_position_counts()
        speed_rpm = motor.read_speed_rpm()
        current_ma = motor.read_phase_current_ma()
        running_state = motor.read_running_state()
        pressure, ocr_status, pressure_updated_at, pressure_sequence = self._pressure_snapshot()
        interval = None if last_sample_at is None else sampled_at - last_sample_at
        position_speed = None
        acceleration = None
        if interval is not None and interval > 0 and last_position is not None:
            position_speed = (position - last_position) / interval
            if last_position_speed is not None:
                acceleration = (
                    (position_speed - last_position_speed)
                    / interval
                    / PD42S1Motor.COUNTS_PER_TURN
                )
        telemetry = {
            "test_id": parameters["test_id"],
            "control_mode": parameters["control_mode"],
            "phase": phase,
            "event": "sample",
            "target_force_g": "",
            "force_g": pressure,
            "force_age_ms": (
                math.inf
                if pressure_updated_at <= 0
                else round((time.monotonic() - pressure_updated_at) * 1000.0, 3)
            ),
            "pressure_sequence": pressure_sequence,
            "position_counts": position,
            "relative_position_counts": position - start_position,
            "position_deg": round(position / PD42S1Motor.COUNTS_PER_TURN * 360.0, 6),
            "measured_speed_rpm": speed_rpm,
            "position_speed_counts_s": (
                None if position_speed is None else round(position_speed, 3)
            ),
            "estimated_acceleration_r_s2": (
                None if acceleration is None else round(acceleration, 6)
            ),
            "sample_interval_ms": None if interval is None else round(interval * 1000.0, 3),
            "phase_current_ma": current_ma,
            "running_state": running_state,
            "running_state_name": RUNNING_STATE_NAMES.get(running_state, "未知"),
            "command_speed_level": parameters["speed_level"],
            "command_speed_counts_s": parameters["speed_counts_s"],
            "command_speed_rpm": parameters["speed_rpm"],
            "command_acceleration_r_s2": parameters["acceleration"],
            "max_travel_level": parameters["max_travel_level"],
            "max_travel_counts": parameters["max_travel_counts"],
            "command_travel_counts": (
                -parameters["max_travel_counts"]
                if phase == "press"
                else parameters["max_travel_counts"]
            ),
            "stop_reason": "",
            "ocr_status": ocr_status,
        }
        self._motor_control.publish_motor_feedback(telemetry)
        return telemetry

    def _record_event(self, parameters, phase, event, position, result=None):
        result = result or {}
        pressure, status, updated_at, _ = self._pressure_snapshot()
        self._recorder.record(
            {
                "test_id": parameters["test_id"],
                "control_mode": parameters["control_mode"],
                "phase": phase,
                "event": event,
                "target_force_g": "",
                "force_g": pressure,
                "force_age_ms": (
                    ""
                    if updated_at <= 0
                    else round((time.monotonic() - updated_at) * 1000.0, 3)
                ),
                "position_counts": "" if position is None else position,
                "command_speed_level": parameters["speed_level"],
                "command_speed_counts_s": parameters["speed_counts_s"],
                "command_speed_rpm": parameters["speed_rpm"],
                "command_acceleration_r_s2": parameters["acceleration"],
                "max_travel_level": parameters["max_travel_level"],
                "max_travel_counts": parameters["max_travel_counts"],
                "command_travel_counts": result.get("command_travel_counts", ""),
                "trigger_force_g": result.get("trigger_force_g", ""),
                "peak_force_g": result.get("peak_force_g", ""),
                "end_force_g": result.get("end_force_g", ""),
                "overshoot_g": result.get("overshoot_g", ""),
                "final_position_counts": result.get("final_position_counts", ""),
                "stop_reason": result.get("stop_reason", ""),
                "ocr_status": status,
            }
        )

    @Slot(object)
    def _show_telemetry(self, telemetry):
        speed = telemetry.get("measured_speed_rpm")
        acceleration = telemetry.get("estimated_acceleration_r_s2")
        current = telemetry.get("phase_current_ma")
        acceleration_text = "--" if acceleration in (None, "") else f"{float(acceleration):.2f}"
        current_text = "--" if current in (None, "") else str(int(current))
        self.motor_label.setText(
            f"电机反馈  位置 {telemetry['position_counts']} counts · "
            f"速度 {speed} RPM · 估算加速度 {acceleration_text} r/s² · "
            f"瞬时相电流 {current_text} mA"
        )

    @Slot()
    def _on_position_zeroed(self):
        """硬件位置清零后开启新的压力重复精度统计批次。"""
        latest = self._motor_control.latest_motor_feedback() or {}
        pressure, ocr_status, pressure_updated_at, _ = self._pressure_snapshot()
        self._recorder.record(
            {
                "test_id": self._test_id,
                "control_mode": "manual_relative_position",
                "phase": "idle",
                "event": "position_zeroed",
                "force_g": pressure,
                "force_age_ms": (
                    ""
                    if pressure_updated_at <= 0
                    else round(
                        (time.monotonic() - pressure_updated_at) * 1000.0,
                        3,
                    )
                ),
                "position_counts": latest.get("position_counts", 0),
                "position_deg": latest.get("position_deg", 0.0),
                "measured_speed_rpm": latest.get("measured_speed_rpm", ""),
                "phase_current_ma": latest.get("phase_current_ma", ""),
                "running_state": latest.get("running_state", ""),
                "running_state_name": latest.get("running_state_name", ""),
                "ocr_status": ocr_status,
            }
        )
        self._test_id = 0
        self._manual_next_kind = "press"
        self._press_results.clear()
        self._motion_results.clear()
        self.result_table.setRowCount(0)
        self.summary_label.setText("重复精度：尚无有效下压结果")
        self._set_status("驱动器当前位置已清零；重复精度统计已重新开始")

    @Slot(object)
    def _finish_operation(self, result):
        self._operation_kind = None
        kind = result["kind"]
        self._append_motion_result(result)
        action = "下压" if kind == "press" else "抬起"
        if kind == "press":
            self._manual_next_kind = "lift"
        elif result["ok"]:
            self._manual_next_kind = "press"
        else:
            self._manual_next_kind = "lift"

        automatic_repeat = (
            self._repeat_running
            and result.get("control_mode") == "automatic_repeat"
        )
        if automatic_repeat and result["ok"] and kind == "press":
            self._repeat_press_result = result
            self._repeat_phase = "waiting_press_idle"
            self._set_status(
                f"重复运动 {self._repeat_completed + 1}/{self._repeat_total}："
                "下压到位，等待电机任务结束"
            )
        elif automatic_repeat and result["ok"] and kind == "lift":
            self._repeat_phase = "waiting_lift_idle"
            self._set_status(
                f"重复运动 {self._repeat_completed + 1}/{self._repeat_total}："
                "抬起到位，等待电机任务结束"
            )
        elif automatic_repeat:
            self._abort_repeat(
                f"重复运动中止：{action}{self._stop_reason_text(result['stop_reason'])}"
            )
        elif result["ok"]:
            self._set_status(
                f"{action}完成：指令 {result['command_travel_counts']:+d} counts，"
                f"实际 {result['relative_position_counts']:+d} counts"
            )
        else:
            self._set_status(
                f"{action}未正常完成：{self._stop_reason_text(result['stop_reason'])}"
            )
        self._motor_control.worker_status.emit(self.status_label.text())
        self._refresh_controls()
        QtCore.QTimer.singleShot(0, self._advance_repeat_sequence)

    def _advance_repeat_sequence(self):
        """只在前一个独立CAN任务完全释放后推进重复运动。"""
        if self._closing or not self._repeat_running:
            return
        if self._operation_kind is not None or self._motor_control.is_operation_busy():
            return

        if self._repeat_phase == "waiting_press_idle":
            wait_seconds = self.settle_time_input.value()
            self._repeat_phase = "waiting_pressure_sample"
            self._set_status(
                f"重复运动 {self._repeat_completed + 1}/{self._repeat_total}："
                f"下压已到位，等待 {wait_seconds:.1f} s 后采集压力计读数"
            )
            self._repeat_hold_timer.start(max(1, round(wait_seconds * 1000.0)))
            self._refresh_controls()
            return

        if self._repeat_phase == "waiting_lift_idle":
            self._repeat_completed += 1
            if self._repeat_completed >= self._repeat_total:
                total = self._repeat_total
                self._repeat_running = False
                self._repeat_phase = None
                self._repeat_press_result = None
                self._manual_next_kind = "press"
                self._set_status(f"重复运动完成：共 {total} 次")
                self._refresh_controls()
                return
            self._repeat_press_result = None
            self._repeat_phase = "starting_press"
            self._start_repeat_operation("press")

    @Slot()
    def _capture_repeat_pressure(self):
        """下压稳定等待结束后保存当时的压力计读数，再启动抬起。"""
        if (
            self._closing
            or not self._repeat_running
            or self._repeat_phase != "waiting_pressure_sample"
        ):
            return
        pressure, ocr_status, updated_at, pressure_sequence = self._pressure_snapshot()
        sampled_at = time.monotonic()
        force_age_ms = (
            math.inf
            if updated_at <= 0
            else round((sampled_at - updated_at) * 1000.0, 3)
        )
        result = self._repeat_press_result
        if result is not None:
            result["settled_force_g"] = pressure
            result["settled_force_age_ms"] = force_age_ms
            result["settled_force_status"] = ocr_status
            row = result.get("_table_row")
            if row is not None:
                value = "--" if pressure is None else f"{pressure:.2f}"
                self.result_table.setItem(
                    int(row),
                    6,
                    QtWidgets.QTableWidgetItem(value),
                )
            self._update_summary()

        latest = self._motor_control.latest_motor_feedback() or {}
        record = dict(latest)
        record.update(
            {
                "test_id": self._test_id,
                "control_mode": "automatic_repeat",
                "phase": "press_hold",
                "event": "settled_pressure_sample",
                "force_g": pressure,
                "force_age_ms": force_age_ms,
                "ocr_status": ocr_status,
            }
        )
        self._recorder.record(record)
        self._motor_control.record_motion_diagnostic(
            "settled_pressure_sample",
            test_id=self._test_id,
            repeat_cycle=self._repeat_completed + 1,
            force_g=pressure,
            force_age_ms=force_age_ms,
            pressure_sequence=pressure_sequence,
            ocr_status=ocr_status,
            position_counts=latest.get("position_counts"),
        )
        pressure_text = "--" if pressure is None else f"{pressure:.2f} g"
        self._set_status(
            f"重复运动 {self._repeat_completed + 1}/{self._repeat_total}："
            f"已记录压力计读数 {pressure_text}，开始抬起"
        )
        self._repeat_phase = "starting_lift"
        self._start_repeat_operation("lift")

    def _abort_repeat(self, message):
        self._repeat_hold_timer.stop()
        self._repeat_running = False
        self._repeat_phase = None
        self._repeat_press_result = None
        self._set_status(message)
        self._refresh_controls()

    @staticmethod
    def _stop_reason_text(reason):
        reason = str(reason)
        if reason.startswith("motor_"):
            return f"电机{reason.removeprefix('motor_')}"
        if reason.startswith("error: "):
            return f"运行错误：{reason.removeprefix('error: ')}"
        return STOP_REASON_NAMES.get(reason, reason)

    def _append_motion_result(self, result):
        self._motion_results.append(result)
        if result["kind"] == "press":
            self._press_results.append(result)
        row = self.result_table.rowCount()
        self.result_table.insertRow(row)
        result["_table_row"] = row
        values = (
            result["test_id"],
            "下压" if result["kind"] == "press" else "抬起",
            result["command_travel_level"],
            result["command_travel_counts"],
            "--" if result["relative_position_counts"] is None else result["relative_position_counts"],
            "--" if result["end_force_g"] is None else f"{result['end_force_g']:.2f}",
            "--" if result.get("settled_force_g") is None else f"{result['settled_force_g']:.2f}",
            "--" if result["peak_force_g"] is None else f"{result['peak_force_g']:.2f}",
            "通过" if result["ok"] else self._stop_reason_text(result["stop_reason"]),
        )
        for column, value in enumerate(values):
            self.result_table.setItem(row, column, QtWidgets.QTableWidgetItem(str(value)))
        self.result_table.scrollToBottom()
        self._update_summary()

    def _update_summary(self):
        valid = [item for item in self._press_results if item["ok"]]
        if not valid:
            self.summary_label.setText("重复精度：尚无有效下压结果")
            return
        positions = [item["final_position_counts"] for item in valid]
        peaks = [item["peak_force_g"] for item in valid if item["peak_force_g"] is not None]
        settled = [
            item["settled_force_g"]
            for item in valid
            if item.get("settled_force_g") is not None
        ]
        position_span = max(positions) - min(positions)
        position_span_deg = position_span / PD42S1Motor.COUNTS_PER_TURN * 360.0
        position_std = statistics.stdev(positions) if len(positions) >= 2 else 0.0
        peak_text = ""
        if peaks:
            peak_text = f"；下压期间压力范围 {min(peaks):.2f}～{max(peaks):.2f} g"
        if settled:
            peak_text += (
                f"；等待后压力计范围 {min(settled):.2f}～{max(settled):.2f} g"
            )
        self.summary_label.setText(
            f"重复精度：有效 {len(valid)} 次；停止位置极差 {position_span} counts "
            f"({position_span_deg:.4f}°)，样本标准差 {position_std:.2f} counts{peak_text}"
        )

    @Slot()
    def clear_results(self):
        """清空界面结果和本次统计，但保留CSV文件及电机当前位置。"""
        if self._operation_kind is not None or self._repeat_running:
            self._set_status("运动执行期间不能清空结果")
            return
        self._press_results.clear()
        self._motion_results.clear()
        self.result_table.setRowCount(0)
        self.summary_label.setText("重复精度：尚无有效下压结果")
        self._set_status("已清空下方结果和本次统计；CSV文件与电机位置未改变")
        self._refresh_controls()

    @Slot()
    def request_stop(self):
        if self._repeat_running:
            self._repeat_hold_timer.stop()
            self._repeat_running = False
            self._repeat_phase = None
            self._repeat_press_result = None
            if self._operation_kind is not None:
                self._motor_control.request_external_stop()
                self._set_status("已停止重复流程，等待当前 CAN 运动刹车")
            else:
                self._set_status("重复流程已停止")
            self._refresh_controls()
            return
        if self._operation_kind is None:
            self._set_status("当前没有正在执行的压力测试运动")
            return
        self._motor_control.request_external_stop()
        self._set_status("已请求紧急停止，等待当前 CAN 应答后刹车")

    @Slot(bool, bool)
    def _on_motor_state_changed(self, connected, _busy):
        if connected:
            if self._motor_control.latest_motor_feedback() is None:
                self.motor_label.setText(
                    "电机反馈  CAN 已连接，等待首次位置、速度和电流反馈..."
                )
        else:
            self.motor_label.setText(
                "电机反馈  CAN 未连接 · 位置 -- counts · 速度 -- RPM · "
                "估算加速度 -- r/s² · 瞬时相电流 -- mA"
            )
            if self._repeat_running:
                self._abort_repeat("CAN 已断开，重复运动已中止")
                return
        self._refresh_controls()
        QtCore.QTimer.singleShot(0, self._advance_repeat_sequence)

    def _set_status(self, message):
        self.status_label.setText(message)
        self.status_message.emit(message)

    def _refresh_controls(self):
        connected = self._motor_control.is_can_connected()
        motor_busy = self._motor_control.is_operation_busy()
        running_here = self._operation_kind is not None
        recording = self._recorder.is_recording
        sequence_active = running_here or self._repeat_running
        if motor_busy and self._operation_kind is not None:
            action = "下压" if self._operation_kind == "press" else "抬起"
            self.motion_button.setText(f"{action}运动中...")
        elif self._manual_next_kind == "lift":
            self.motion_button.setText("返回 / 抬起")
        else:
            self.motion_button.setText("开始下压")
        self.motion_button.setEnabled(
            connected and not motor_busy and not self._repeat_running
        )
        self.repeat_button.setText(
            "重复运动执行中..." if self._repeat_running else "开始重复运动"
        )
        self.repeat_button.setEnabled(
            connected
            and not motor_busy
            and not self._repeat_running
            and self._manual_next_kind == "press"
        )
        self.stop_button.setEnabled(connected and sequence_active)
        self.zero_position_button.setEnabled(
            connected and not motor_busy and not self._repeat_running
        )
        self.start_record_button.setEnabled(not recording)
        self.stop_record_button.setEnabled(recording)
        self.clear_results_button.setEnabled(not sequence_active)
        for widget in (
            self.speed_level_input,
            self.acceleration_input,
            self.max_travel_input,
            self.settle_time_input,
            self.repeat_count_input,
        ):
            widget.setEnabled(not sequence_active)
        for widget in (
            self.target_force_input,
            self.release_force_input,
            self.ocr_timeout_input,
        ):
            widget.setEnabled(False)

    def shutdown(self):
        self._closing = True
        self._repeat_hold_timer.stop()
        self._repeat_running = False
        if self._operation_kind is not None:
            self._motor_control.request_external_stop()
        try:
            self._recorder.stop()
        except OSError:
            pass
