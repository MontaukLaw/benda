"""通过压力计重复扫描建立压力与固定释放点相对下压行程的查找表。"""

from __future__ import annotations

import csv
import json
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from force_position_lookup import (
    ForcePositionLookupError,
    MAP_FORCE_STEP_N,
    latest_mapping_path,
    mapping_directory,
    resolve_force_position_target_n,
)
from pressure_meter_serial_widget import (
    GF_TO_NEWTON,
)
from qt_compat import QtCore, QtGui, QtWidgets, Signal, Slot
from motor_mapping_interface import LEGACY_PROFILE, LegacyMappingControl


DEFAULT_MAX_FORCE_N = 20.0
ABSOLUTE_SAFETY_FORCE_N = 35.0
DEFAULT_REPEAT_COUNT = 5
DEFAULT_PRESS_SPEED_LEVEL = 2
DEFAULT_PRESS_STEP_COUNTS = 256
DEFAULT_RELEASE_SPEED_LEVEL = 5
DEFAULT_POSITION_SPAN_LIMIT_COUNTS = 128
DEFAULT_MAX_TRAVEL_COUNTS = 24 * 4096
DEFAULT_MAX_CURRENT_MA = 500
DEFAULT_ACCELERATION_R_S2 = 100
PRESSURE_FRESH_SECONDS = 1.0
PRESSURE_SAMPLE_WINDOW_SECONDS = 0.20
PRESSURE_SAMPLE_MIN_COUNT = 3
MOTOR_POSITION_TOLERANCE_COUNTS = 50
RELEASE_POSITION_TOLERANCE_COUNTS = 256
MOTOR_STABLE_DELTA_COUNTS = 10
MOTOR_STABLE_SAMPLE_COUNT = 3
SCAN_MIN_SETTLED_PROGRESS_COUNTS = 16
SCAN_MAX_SETTLED_PROGRESS_RATIO = 2.0
POLL_SECONDS = 0.05
# 本页标定允许释放点附近的轻微压力漂移，并限制轮间等待时间。
MAPPING_ZERO_TOLERANCE_N = 0.10
MAPPING_ZERO_WAIT_SECONDS = 10.0


def pressure_is_near_mapping_zero(force):
    """标定零点使用双向容差，异常负值与非有限值不能作为零点。"""
    return math.isfinite(force) and abs(force) <= MAPPING_ZERO_TOLERANCE_N


def mapping_settings_path():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "force_position_mapping_config.json"
    return Path(__file__).resolve().with_name("force_position_mapping_config.json")


DEFAULT_SETTINGS = {
    "max_force_n": DEFAULT_MAX_FORCE_N,
    "repeat_count": DEFAULT_REPEAT_COUNT,
    "press_speed_level": DEFAULT_PRESS_SPEED_LEVEL,
    "press_step_counts": DEFAULT_PRESS_STEP_COUNTS,
    "release_speed_level": DEFAULT_RELEASE_SPEED_LEVEL,
    "position_span_limit_counts": DEFAULT_POSITION_SPAN_LIMIT_COUNTS,
    "max_travel_counts": DEFAULT_MAX_TRAVEL_COUNTS,
    "max_current_ma": DEFAULT_MAX_CURRENT_MA,
    "acceleration_r_s2": DEFAULT_ACCELERATION_R_S2,
    "lookup_target_force_n": 3.0,
}


class ForcePositionMappingError(RuntimeError):
    pass


def _monotonic_force_envelope(points):
    """仅保留压力创新高的扫描点，供单调插值使用。"""
    envelope = []
    highest_force = -math.inf
    for point in points:
        force = float(point["force_n"])
        position = float(point["position_counts"])
        if not math.isfinite(force) or not math.isfinite(position):
            continue
        if force > highest_force:
            envelope.append((force, position))
            highest_force = force
        elif envelope and force == highest_force:
            # 相同压力连续出现时保留最后位置，避免从扫描起点的0g跨越整段空行程插值。
            envelope[-1] = (force, position)
    return envelope


def interpolate_position_for_force(points, target_force_n):
    """在单轮单调压力包络上插值得到目标压力对应的绝对位置。"""
    envelope = _monotonic_force_envelope(points)
    target = float(target_force_n)
    if not envelope or target < envelope[0][0] or target > envelope[-1][0]:
        return None
    for index in range(1, len(envelope)):
        low_force, low_position = envelope[index - 1]
        high_force, high_position = envelope[index]
        if target > high_force:
            continue
        force_delta = high_force - low_force
        if force_delta <= 0:
            return high_position
        ratio = (target - low_force) / force_delta
        return low_position + ratio * (high_position - low_position)
    return envelope[-1][1]


def build_force_position_map(rounds, max_force_n, span_limit_counts):
    """将多轮扫描统一插值为每 0.10 N 一档，并计算相对释放点的行程重复性。"""
    mapping = []
    step_count = int(math.floor(float(max_force_n) / MAP_FORCE_STEP_N + 1e-9))
    for step_index in range(1, step_count + 1):
        force_n = round(step_index * MAP_FORCE_STEP_N, 2)
        positions = []
        press_travels = []
        for round_result in rounds:
            position = interpolate_position_for_force(
                round_result.get("points", ()),
                force_n,
            )
            if position is not None and math.isfinite(position):
                positions.append(float(position))
                release_reference = round_result.get(
                    "release_reference_position_counts",
                    round_result.get(
                        "release_position_counts",
                        round_result.get("start_position_counts"),
                    ),
                )
                try:
                    press_travel = float(release_reference) - float(position)
                except (TypeError, ValueError):
                    continue
                if math.isfinite(press_travel):
                    press_travels.append(press_travel)
        if not positions or not press_travels:
            continue
        minimum_position = min(positions)
        maximum_position = max(positions)
        minimum_travel = min(press_travels)
        maximum_travel = max(press_travels)
        travel_span = maximum_travel - minimum_travel
        mapping.append(
            {
                "force_n": force_n,
                "position_counts": int(round(statistics.median(positions))),
                "median_position_counts": statistics.median(positions),
                "minimum_position_counts": minimum_position,
                "maximum_position_counts": maximum_position,
                "press_travel_counts": int(
                    round(statistics.median(press_travels))
                ),
                "median_press_travel_counts": statistics.median(press_travels),
                "minimum_press_travel_counts": minimum_travel,
                "maximum_press_travel_counts": maximum_travel,
                "position_span_counts": travel_span,
                "position_standard_deviation_counts": (
                    statistics.stdev(press_travels)
                    if len(press_travels) >= 2
                    else 0.0
                ),
                "sample_count": len(press_travels),
                "stable": (
                    len(press_travels) == len(rounds)
                    and len(press_travels) >= 2
                    and travel_span <= float(span_limit_counts)
                ),
            }
        )
    return mapping


class ForcePositionMappingWidget(QtWidgets.QWidget):
    """自动扫描压力－位置关系并使用稳定查找表执行一次性定位。"""

    status_message = Signal(str)
    mapping_changed = Signal()
    progress_changed = Signal(object)
    operation_finished = Signal(object)

    def __init__(self, motor_control_widget, parent=None):
        super().__init__(parent)
        self._motor_control = (motor_control_widget if hasattr(motor_control_widget, "profile")
                               else LegacyMappingControl(motor_control_widget))
        self._profile = getattr(motor_control_widget, "profile", LEGACY_PROFILE)
        self._pressure_lock = threading.Lock()
        self._pressure_value = None
        self._pressure_status = "等待压力计串口"
        self._pressure_updated_at = 0.0
        self._pressure_sequence = 0
        self._running = False
        self._operation_kind = None
        self._closing = False
        self._round_results = []
        self._mapping = []
        self._mapping_metadata = {}
        self._settings_path = mapping_settings_path()
        if self._profile.namespace:
            self._settings_path = self._settings_path.with_name(f"force_position_mapping_{self._profile.namespace}.json")
        settings = self._load_settings()
        self._build_ui(settings)
        self.progress_changed.connect(self._on_progress)
        self.operation_finished.connect(self._on_operation_finished)
        self._motor_control.motor_feedback.connect(self._on_motor_feedback)
        self._motor_control.control_state_changed.connect(self._on_motor_state_changed)
        self._motor_control.worker_position_zeroed.connect(self._on_position_zeroed)
        self._motor_control.release_position_changed.connect(
            self._on_release_position_changed
        )
        if hasattr(self._motor_control, "status_message"):
            self._motor_control.status_message.connect(self._set_status)
        self._load_latest_mapping()
        latest = self._motor_control.latest_motor_feedback()
        if latest is not None:
            self._show_motor_feedback(latest)
        self._refresh_controls()

    def _mapping_directory(self):
        directory = mapping_directory()
        return directory / self._profile.namespace if self._profile.namespace else directory

    def _latest_mapping_path(self):
        if not self._profile.namespace:
            return latest_mapping_path()
        return self._mapping_directory() / "force_position_map_latest.json"

    def _build_ui(self, settings):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        explanation = QtWidgets.QLabel(
            "使用压力计而不是FLOATING Signal建立位置前馈表。"
            "每轮从已设置的固定释放位置开始，以固定速度和固定step步长慢慢下压；"
            "每一步到位后采集压力，再插值生成每0.10 N对应的相对下压行程。"
            "查表定位使用JSON记录的标定出发点减去该行程，运动中仍保留压力上限保护。"
        )
        explanation.setWordWrap(True)
        explanation.setStyleSheet("color:#8bd5ff;")
        outer.addWidget(explanation)
        driver_hint = QtWidgets.QLabel(
            f"当前驱动：{self._profile.model} · {self._profile.steps_per_turn} step/圈；"
            "驱动由 motor_connection_config.json 选择。"
        )
        driver_hint.setWordWrap(True)
        outer.addWidget(driver_hint)

        live_group = QtWidgets.QGroupBox("实时信息")
        live_layout = QtWidgets.QHBoxLayout(live_group)
        self._live_layout = live_layout
        self.pressure_label = QtWidgets.QLabel("压力计 --.-- N")
        self.pressure_label.setObjectName("ForcePositionPressure")
        live_layout.addWidget(self.pressure_label, 1)
        self.position_label = QtWidgets.QLabel(
            "电机位置 -- step · 角度 --° · 速度 -- RPM · 瞬时相电流 -- mA"
        )
        self.position_label.setObjectName("ForcePositionMotor")
        live_layout.addWidget(self.position_label, 1)
        outer.addWidget(live_group)

        parameter_group = QtWidgets.QGroupBox("扫描参数")
        grid = QtWidgets.QGridLayout(parameter_group)
        self._parameter_grid = grid
        self._parameter_fields = []
        grid.setHorizontalSpacing(9)
        grid.setVerticalSpacing(7)

        self.max_force_input = QtWidgets.QDoubleSpinBox()
        self.max_force_input.setDecimals(2)
        self.max_force_input.setRange(MAP_FORCE_STEP_N, 30.0)
        self.max_force_input.setSingleStep(MAP_FORCE_STEP_N)
        self.max_force_input.setValue(float(settings["max_force_n"]))
        self._add_value(grid, "扫描上限", self.max_force_input, "N", 0, 0)

        self.repeat_count_input = QtWidgets.QSpinBox()
        self.repeat_count_input.setRange(2, 30)
        self.repeat_count_input.setValue(int(settings["repeat_count"]))
        self._add_value(grid, "重复次数", self.repeat_count_input, "轮", 0, 2)

        self.press_speed_input = QtWidgets.QSpinBox()
        self.press_speed_input.setRange(1, self._profile.speed_max)
        self.press_speed_input.setValue(int(settings["press_speed_level"]))
        self._add_value(grid, "下压速度", self.press_speed_input, self._profile.speed_unit, 0, 4)

        self.press_step_input = QtWidgets.QSpinBox()
        self.press_step_input.setRange(1, 1000000)
        self.press_step_input.setSingleStep(1)
        self.press_step_input.setValue(int(settings["press_step_counts"]))
        self._add_value(grid, "下压步长", self.press_step_input, "step", 1, 0)

        self.release_speed_input = QtWidgets.QSpinBox()
        self.release_speed_input.setRange(1, self._profile.speed_max)
        self.release_speed_input.setValue(int(settings["release_speed_level"]))
        self._add_value(grid, "释放速度", self.release_speed_input, self._profile.speed_unit, 1, 2)

        self.span_limit_input = QtWidgets.QSpinBox()
        self.span_limit_input.setRange(1, 4096)
        self.span_limit_input.setValue(int(settings["position_span_limit_counts"]))
        self.span_limit_input.setToolTip("同一压力档位跨轮下压行程极差不超过该值才允许查表定位")
        self._add_value(grid, "稳定行程极差", self.span_limit_input, "step", 1, 4)

        self.max_travel_input = QtWidgets.QSpinBox()
        self.max_travel_input.setRange(1, 200000)
        self.max_travel_input.setSingleStep(self._profile.steps_per_turn)
        self.max_travel_input.setValue(int(settings["max_travel_counts"]))
        self._add_value(grid, "单轮最大行程", self.max_travel_input, "step", 2, 0)

        self.max_current_input = QtWidgets.QSpinBox()
        self.max_current_input.setRange(10, 3000)
        self.max_current_input.setSingleStep(50)
        self.max_current_input.setValue(int(settings["max_current_ma"]))
        self._add_value(grid, "最大电流", self.max_current_input, "mA", 2, 2)
        self.max_current_input.setToolTip(
            "ZDT Emm：设置驱动器堵转最大电流，正常运行相电流可能超过此值；参数不保存到驱动器。"
            if self._profile.namespace else "设置驱动器最大力矩电流"
        )

        self.acceleration_input = QtWidgets.QSpinBox()
        self.acceleration_input.setRange(0, self._profile.acceleration_max)
        self.acceleration_input.setValue(int(settings["acceleration_r_s2"]))
        self._add_value(grid, "加减速度", self.acceleration_input, self._profile.acceleration_unit, 2, 4)

        safety = QtWidgets.QLabel(
            f"每步到位后取{PRESSURE_SAMPLE_WINDOW_SECONDS:.1f}秒压力中位值；"
            "中位值达到扫描上限后结束本轮并释放；运动中仅在绝对保护上限"
            f"{ABSOLUTE_SAFETY_FORCE_N:.2f} N。"
        )
        safety.setStyleSheet("color:#ff9f43;font-weight:700;")
        safety.setWordWrap(True)
        grid.addWidget(safety, 3, 0, 1, 6)
        zero_hint = QtWidgets.QLabel(
            f"释放零点容差：±{MAPPING_ZERO_TOLERANCE_N:.2f} N；"
            f"回到释放位置后最多等待{MAPPING_ZERO_WAIT_SECONDS:g}秒，"
            "压力进入容差后自动继续，可随时紧急停止。"
        )
        zero_hint.setWordWrap(True)
        grid.addWidget(zero_hint, 4, 0, 1, 6)
        self._parameter_notes = (safety, zero_hint)
        outer.addWidget(parameter_group)

        zero_row = QtWidgets.QHBoxLayout()
        self._zero_layout = zero_row
        self.zero_position_button = QtWidgets.QPushButton("当前位置设为 0（角度清零）")
        self.zero_position_button.clicked.connect(self._motor_control.zero_motor_position)
        self.zero_position_button.setToolTip("电机静止时清零当前位置；旧释放位置及查找表作废，需重新设置和标定。")
        zero_row.addWidget(self.zero_position_button)
        zero_note = QtWidgets.QLabel("只改变坐标原点；清零后请重新设置释放位置并标定。")
        zero_note.setWordWrap(True)
        zero_row.addWidget(zero_note, 1)
        outer.addLayout(zero_row)

        action_row = QtWidgets.QHBoxLayout()
        self._action_layout = action_row
        self.set_release_position_button = QtWidgets.QPushButton("设置释放位置")
        self.set_release_position_button.clicked.connect(
            self._motor_control.capture_release_position
        )
        action_row.addWidget(self.set_release_position_button)
        self.start_button = QtWidgets.QPushButton("开始重复扫描并生成查找表")
        self.start_button.setObjectName("ForcePositionStart")
        self.start_button.clicked.connect(self.start_calibration)
        action_row.addWidget(self.start_button)
        self.release_button = QtWidgets.QPushButton("释放 / 抬起")
        self.release_button.clicked.connect(self.start_release)
        action_row.addWidget(self.release_button)
        self.stop_button = QtWidgets.QPushButton("紧急停止")
        self.stop_button.setObjectName("ForcePositionStop")
        self.stop_button.clicked.connect(self.request_stop)
        action_row.addWidget(self.stop_button)
        outer.addLayout(action_row)
        self.release_position_label = QtWidgets.QLabel()
        self.release_position_label.setWordWrap(True)
        outer.addWidget(self.release_position_label)
        self._on_release_position_changed(
            self._motor_control.release_position_snapshot()
        )

        lookup_group = QtWidgets.QGroupBox("查表一次性定位")
        lookup_layout = QtWidgets.QHBoxLayout(lookup_group)
        self._lookup_layout = lookup_layout
        lookup_input = QtWidgets.QHBoxLayout()
        lookup_layout.addLayout(lookup_input)
        lookup_input.addWidget(QtWidgets.QLabel("需要压力"))
        self.lookup_force_input = QtWidgets.QDoubleSpinBox()
        self.lookup_force_input.setDecimals(2)
        self.lookup_force_input.setRange(MAP_FORCE_STEP_N, 30.0)
        self.lookup_force_input.setSingleStep(MAP_FORCE_STEP_N)
        self.lookup_force_input.setValue(float(settings["lookup_target_force_n"]))
        lookup_input.addWidget(self.lookup_force_input)
        lookup_input.addWidget(QtWidgets.QLabel("N"))
        self.lookup_button = QtWidgets.QPushButton("查表并一次性运动到对应位置")
        self.lookup_button.setObjectName("ForcePositionLookup")
        self.lookup_button.clicked.connect(self.start_lookup_move)
        lookup_layout.addWidget(self.lookup_button)
        self.lookup_result_label = QtWidgets.QLabel("尚无有效查找表")
        self.lookup_result_label.setWordWrap(True)
        lookup_layout.addWidget(self.lookup_result_label, 1)
        outer.addWidget(lookup_group)

        self.status_label = QtWidgets.QLabel("等待开始位置关系扫描")
        self.status_label.setObjectName("ForcePositionStatus")
        self.status_label.setWordWrap(True)
        outer.addWidget(self.status_label)
        self.summary_label = QtWidgets.QLabel("尚无压力－位置查找表")
        self.summary_label.setWordWrap(True)
        outer.addWidget(self.summary_label)

        round_selection_row = QtWidgets.QHBoxLayout()
        self._round_layout = round_selection_row
        round_selection_row.setSpacing(8)
        round_selection_row.addWidget(QtWidgets.QLabel("参与统计轮次"))
        self.round_selector_scroll = QtWidgets.QScrollArea()
        self.round_selector_scroll.setWidgetResizable(True)
        self.round_selector_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.round_selector_scroll.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded
        )
        self.round_selector_scroll.setVerticalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        self.round_selector_scroll.setFixedHeight(43)
        self.round_selector_widget = QtWidgets.QWidget()
        self.round_selector_layout = QtWidgets.QHBoxLayout(
            self.round_selector_widget
        )
        self.round_selector_layout.setContentsMargins(2, 0, 2, 0)
        self.round_selector_layout.setSpacing(12)
        self.round_selector_scroll.setWidget(self.round_selector_widget)
        round_selection_row.addWidget(self.round_selector_scroll, 1)
        self.recalculate_button = QtWidgets.QPushButton("按勾选轮次重新统计并保存")
        self.recalculate_button.setObjectName("ForcePositionRecalculate")
        self.recalculate_button.setToolTip(
            "只用勾选轮次重算各压力档位的中位下压行程、极差和标准差；原始扫描数据不会删除"
        )
        self.recalculate_button.clicked.connect(
            self._recalculate_selected_rounds
        )
        round_selection_row.addWidget(self.recalculate_button)
        outer.addLayout(round_selection_row)
        self._round_checkboxes = []
        self._rebuild_round_selector()

        self.table = QtWidgets.QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(
            (
                "压力/N",
                "中位下压行程/step",
                "最小行程",
                "最大行程",
                "极差/step",
                "标准差",
                "有效轮数",
                "可用于定位",
            )
        )
        self.table.setEditTriggers(
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeMode.Interactive
        )
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMinimumHeight(280)
        outer.addWidget(self.table, 1)

        self.setStyleSheet(
            "QLabel#ForcePositionPressure{color:#57d5ff;font-size:22px;font-weight:700;}"
            "QLabel#ForcePositionMotor{color:#b9e6ff;font-size:18px;font-weight:700;}"
            "QLabel#ForcePositionStatus{color:#f4d35e;font-size:15px;font-weight:700;}"
            "QPushButton{min-height:30px;font-weight:700;}"
            "QPushButton#ForcePositionStart{background:#6fd5a5;color:#102019;}"
            "QPushButton#ForcePositionLookup{background:#bdd6e8;color:#17212b;}"
            "QPushButton#ForcePositionRecalculate{background:#d8c5ff;color:#211934;}"
            "QPushButton#ForcePositionStop{background:#edbbb6;color:#17212b;}"
        )

        for widget in (
            self.max_force_input,
            self.repeat_count_input,
            self.press_speed_input,
            self.press_step_input,
            self.release_speed_input,
            self.span_limit_input,
            self.max_travel_input,
            self.max_current_input,
            self.acceleration_input,
            self.lookup_force_input,
        ):
            widget.valueChanged.connect(self._save_settings_if_idle)

    def _add_value(self, grid, label, widget, unit, row, column):
        label_widget = QtWidgets.QLabel(label)
        grid.addWidget(label_widget, row, column)
        value_layout = QtWidgets.QHBoxLayout()
        value_layout.setContentsMargins(0, 0, 0, 0)
        value_layout.setSpacing(5)
        widget.setFixedWidth(105)
        value_layout.addWidget(widget)
        value_layout.addWidget(QtWidgets.QLabel(unit))
        value_layout.addStretch(1)
        grid.addLayout(value_layout, row, column + 1)
        self._parameter_fields.append((row * 3 + column // 2, label_widget, value_layout))

    def resizeEvent(self, event):
        """窄面板改为纵向排列，防止参数、停止按钮和状态文字被截断。"""
        width = event.size().width()
        columns = 3 if width >= 930 else 2 if width >= 650 else 1
        if columns != getattr(self, "_parameter_columns", None):
            self._parameter_columns = columns
            grid = self._parameter_grid
            for _index, label, value in self._parameter_fields:
                grid.removeWidget(label)
                grid.removeItem(value)
            for note in self._parameter_notes:
                grid.removeWidget(note)
            for index, (_order, label, value) in enumerate(sorted(self._parameter_fields)):
                row, column = divmod(index, columns)
                grid.addWidget(label, row, column * 2)
                grid.addLayout(value, row, column * 2 + 1)
            rows = math.ceil(len(self._parameter_fields) / columns)
            for index, note in enumerate(self._parameter_notes):
                grid.addWidget(note, rows + index, 0, 1, columns * 2)
        direction = (QtWidgets.QBoxLayout.Direction.TopToBottom if width < 930
                     else QtWidgets.QBoxLayout.Direction.LeftToRight)
        for layout in (self._live_layout, self._zero_layout, self._action_layout,
                       self._lookup_layout, self._round_layout):
            layout.setDirection(direction)
        self.position_label.setWordWrap(True)
        super().resizeEvent(event)

    def _load_settings(self):
        settings = dict(DEFAULT_SETTINGS)
        settings.update(self._profile.default_settings)
        try:
            loaded = json.loads(self._settings_path.read_text(encoding="utf-8"))
            loaded = self._profile.storage_fields(loaded, read=True)
        except (OSError, ValueError, TypeError):
            return settings
        if isinstance(loaded, dict):
            for key in settings:
                if key in loaded:
                    settings[key] = loaded[key]
            # 旧配置的克力数值只迁移为等值牛顿，不做简单改名。
            if "max_force_n" not in loaded and "max_force_g" in loaded:
                try:
                    settings["max_force_n"] = min(
                        30.0, float(loaded["max_force_g"]) * GF_TO_NEWTON
                    )
                except (TypeError, ValueError, OverflowError):
                    pass
            if (
                "lookup_target_force_n" not in loaded
                and "lookup_target_force_g" in loaded
            ):
                try:
                    settings["lookup_target_force_n"] = min(
                        30.0,
                        float(loaded["lookup_target_force_g"]) * GF_TO_NEWTON,
                    )
                except (TypeError, ValueError, OverflowError):
                    pass
        return settings

    def _settings(self):
        return {
            "max_force_n": round(self.max_force_input.value(), 2),
            "repeat_count": self.repeat_count_input.value(),
            "press_speed_level": self.press_speed_input.value(),
            "press_step_counts": self.press_step_input.value(),
            "release_speed_level": self.release_speed_input.value(),
            "position_span_limit_counts": self.span_limit_input.value(),
            "max_travel_counts": self.max_travel_input.value(),
            "max_current_ma": self.max_current_input.value(),
            "acceleration_r_s2": self.acceleration_input.value(),
            "lookup_target_force_n": round(self.lookup_force_input.value(), 2),
        }

    @Slot()
    def _save_settings_if_idle(self, *_args):
        if self._running:
            return
        try:
            self._settings_path.parent.mkdir(parents=True, exist_ok=True)
            self._settings_path.write_text(
                json.dumps(self._profile.storage_fields(self._settings()), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except OSError:
            pass

    @Slot(object, str)
    def set_pressure_meter_pressure(self, value, status):
        now = time.monotonic()
        with self._pressure_lock:
            self._pressure_value = None if value is None else float(value)
            self._pressure_status = str(status)
            self._pressure_updated_at = now
            self._pressure_sequence += 1
            pressure = self._pressure_value
        if pressure is None:
            self.pressure_label.setText("压力计 --.-- N")
            self.pressure_label.setStyleSheet("color:#ff7b72;font-size:22px;font-weight:700;")
        else:
            self.pressure_label.setText(f"压力计 {pressure:.2f} N")
            color = "#ff7b72" if pressure >= ABSOLUTE_SAFETY_FORCE_N else "#57d5ff"
            self.pressure_label.setStyleSheet(
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

    def _fresh_pressure(self):
        value, status, updated_at, sequence = self._pressure_snapshot()
        age = math.inf if updated_at <= 0 else time.monotonic() - updated_at
        if value is None or not math.isfinite(value) or age > PRESSURE_FRESH_SECONDS:
            raise ForcePositionMappingError(
                f"压力计无有效新鲜数据：{status}，数据年龄{age:.2f}秒"
            )
        return float(value), int(sequence)

    def _wait_for_mapping_zero(self, stop_event, context, after_sequence=None):
        """在工作线程内等待有效零点；新帧确认、超时与停止都有明确出口。"""
        deadline = time.monotonic() + MAPPING_ZERO_WAIT_SECONDS
        next_progress = 0.0
        last_force = None
        while time.monotonic() < deadline:
            if stop_event.is_set():
                raise ForcePositionMappingError("用户停止操作")
            force, sequence = self._fresh_pressure()
            last_force = force
            if sequence != after_sequence and pressure_is_near_mapping_zero(force):
                self.progress_changed.emit({
                    "message": f"{context}：压力{force:.2f} N，处于"
                    f"±{MAPPING_ZERO_TOLERANCE_N:.2f} N零点附近，确认通过"
                })
                return force
            now = time.monotonic()
            if now >= next_progress:
                self.progress_changed.emit({
                    "message": f"{context}：压力{force:.2f} N，等待新读数进入"
                    f"±{MAPPING_ZERO_TOLERANCE_N:.2f} N；"
                    f"剩余{max(0.0, deadline-now):.1f}秒，可紧急停止"
                })
                next_progress = now + 0.5
            stop_event.wait(POLL_SECONDS)
        force_text = "--" if last_force is None else f"{last_force:.2f}"
        raise ForcePositionMappingError(
            f"{context}：零点确认超时（{MAPPING_ZERO_WAIT_SECONDS:g}秒），"
            f"压力{force_text} N，需进入±{MAPPING_ZERO_TOLERANCE_N:.2f} N；"
            "本次操作已结束，请检查压力计零点后重新开始"
        )

    def _confirmed_release_position(self):
        snapshot = self._motor_control.release_position_snapshot()
        position = snapshot.get("position_counts")
        if not snapshot.get("confirmed") or position is None:
            return None
        return int(position)

    @Slot(object)
    def _on_release_position_changed(self, snapshot):
        position = snapshot.get("position_counts")
        if position is None:
            text = "释放位置：尚未设置，请在确认完全无压力后点击“设置释放位置”"
            color = "#ff7b72"
        elif snapshot.get("confirmed"):
            text = f"释放位置：{int(position)} step（当前CAN连接已确认）"
            color = "#5be39b"
        else:
            text = (
                f"释放位置：PC已保存 {int(position)} step，但当前连接待重新设置"
            )
            color = "#ffcc66"
        self.release_position_label.setText(text)
        self.release_position_label.setStyleSheet(
            f"color:{color};font-weight:700;"
        )
        if hasattr(self, "recalculate_button"):
            self._refresh_controls()

    @Slot()
    def start_calibration(self):
        if not self._validate_start(require_zero=True, require_release_start=True):
            return
        parameters = self._settings()
        parameters["release_position_counts"] = self._confirmed_release_position()
        if not self._confirm_start_calibration(parameters):
            self._set_status("已取消重复扫描，原查找表保持不变")
            return
        accepted = self._start_external_operation(
            "calibration",
            lambda motor, stop_event: self._run_calibration(
                motor,
                stop_event,
                parameters,
            ),
            "压力－位置关系扫描已开始",
        )
        if accepted:
            self._invalidate_mapping_for_new_scan(parameters)

    def _confirm_start_calibration(self, parameters):
        previous_round_count = len(self._round_results)
        previous_mapping_count = len(self._mapping)
        if previous_round_count or previous_mapping_count:
            previous_text = (
                f"当前界面已有 {previous_round_count} 轮原始数据、"
                f"{previous_mapping_count} 个压力档位。\n"
                "新扫描启动后，这些数据会立即从界面清空并停止用于定位。"
            )
        else:
            previous_text = "当前界面没有有效的旧查找表。"
        standard_button = getattr(
            QtWidgets.QMessageBox,
            "StandardButton",
            QtWidgets.QMessageBox,
        )
        yes_button = getattr(QtWidgets.QMessageBox, "Yes", standard_button.Yes)
        no_button = getattr(QtWidgets.QMessageBox, "No", standard_button.No)
        answer = QtWidgets.QMessageBox.question(
            self.window(),
            "确认开始重复扫描",
            "即将开始生成一套新的压力－位置查找表。\n\n"
            f"扫描上限：{parameters['max_force_n']:.2f} N\n"
            f"重复次数：{parameters['repeat_count']} 轮\n"
            f"下压步长：{parameters['press_step_counts']} step\n"
            f"下压速度：{parameters['press_speed_level']} {self._profile.speed_unit}\n\n"
            f"{previous_text}\n\n"
            "如果扫描中途停止或失败，旧表也不会自动恢复；"
            "带时间戳的历史 CSV/JSON 文件仍会保留。\n\n"
            "是否确认开始？",
            yes_button | no_button,
            no_button,
        )
        return answer == yes_button

    def _invalidate_mapping_for_new_scan(self, parameters):
        previous_metadata = dict(self._mapping_metadata)
        previous_round_count = len(self._round_results)
        previous_mapping_count = len(self._mapping)
        invalidated_at = datetime.now().astimezone().isoformat(timespec="seconds")
        self._round_results = []
        self._mapping = []
        self._mapping_metadata = {}
        self._rebuild_round_selector()
        self.table.setRowCount(0)
        self.summary_label.setText(
            "正在生成新的压力－位置查找表；旧GUI数据已作废并清空"
        )
        self.lookup_result_label.setText("新查找表尚未生成，暂不可查表定位")
        try:
            self._write_mapping_json(
                self._latest_mapping_path(),
                {
                    "metadata": {
                        "invalidated_at": invalidated_at,
                        "invalidated_reason": "new_scan_started",
                        "previous_created_at": previous_metadata.get("created_at"),
                        "previous_csv_path": previous_metadata.get("csv_path"),
                        "previous_round_count": previous_round_count,
                        "previous_mapping_count": previous_mapping_count,
                        "force_unit": "N",
                        "new_scan_max_force_n": parameters["max_force_n"],
                        "new_scan_repeat_count": parameters["repeat_count"],
                    },
                    "rounds": [],
                    "mapping": [],
                },
            )
        except OSError as exc:
            self._motor_control.record_motion_diagnostic(
                "force_position_latest_invalidation_failed",
                error=repr(exc),
            )
        self.mapping_changed.emit()
        self._refresh_controls()

    @Slot()
    def start_release(self):
        if not self._validate_start(require_zero=False):
            return
        parameters = self._settings()
        parameters["release_position_counts"] = self._confirmed_release_position()
        self._start_external_operation(
            "release",
            lambda motor, stop_event: self._run_release_operation(
                motor,
                stop_event,
                parameters,
            ),
            "正在释放抬起",
        )

    @Slot()
    def start_lookup_move(self):
        if not self._validate_start(require_zero=True, require_release_start=True):
            return
        requested_force = self.lookup_force_input.value()
        target_force = round(
            round(requested_force / MAP_FORCE_STEP_N) * MAP_FORCE_STEP_N,
            2,
        )
        try:
            lookup = resolve_force_position_target_n(
                {
                    "metadata": self._mapping_metadata,
                    "mapping": self._mapping,
                },
                target_force,
            )
        except ForcePositionLookupError as exc:
            self._set_status(str(exc))
            return
        release_position_counts = self._confirmed_release_position()
        mapping_release_position = int(
            lookup["release_reference_position_counts"]
        )
        release_reference_error = int(release_position_counts) - mapping_release_position
        if abs(release_reference_error) > self._profile.tolerance(RELEASE_POSITION_TOLERANCE_COUNTS):
            self._set_status(
                f"当前共享释放位置{release_position_counts} step与JSON标定出发点"
                f"{mapping_release_position} step相差{release_reference_error:+d} step，"
                "拒绝查表定位"
            )
            return
        parameters = self._settings()
        parameters.update(
            {
                "lookup_force_n": target_force,
                "release_position_counts": mapping_release_position,
                "lookup_press_travel_counts": int(lookup["press_travel_counts"]),
                "lookup_position_counts": int(lookup["target_position_counts"]),
                "lookup_span_counts": float(lookup["position_span_counts"]),
            }
        )
        self._start_external_operation(
            "lookup",
            lambda motor, stop_event: self._run_lookup_move(
                motor,
                stop_event,
                parameters,
            ),
            f"正在查表定位到 {target_force:.2f} N对应位置",
        )

    def _validate_start(self, require_zero, require_release_start=False):
        if self._running or self._motor_control.is_operation_busy():
            self._set_status("已有电机操作正在运行")
            return False
        if not self._motor_control.is_can_connected():
            self._set_status("CAN未连接")
            return False
        release_position = self._confirmed_release_position()
        if release_position is None:
            self._set_status(
                "当前CAN连接尚未设置释放位置；请确认完全无压力后点击“设置释放位置”"
            )
            return False
        if require_release_start:
            feedback = self._motor_control.latest_motor_feedback()
            position = None if feedback is None else feedback.get("position_counts")
            if position is None:
                self._set_status("尚未获得电机实时位置，请稍后重试")
                return False
            error = int(position) - release_position
            if abs(error) > self._profile.tolerance(RELEASE_POSITION_TOLERANCE_COUNTS):
                self._set_status(
                    f"当前位置{int(position)} step不在释放起点"
                    f"{release_position}±{self._profile.tolerance(RELEASE_POSITION_TOLERANCE_COUNTS)} step内；"
                    "请先点击释放"
                )
                return False
        if require_zero:
            try:
                pressure, _sequence = self._fresh_pressure()
            except ForcePositionMappingError as exc:
                self._set_status(str(exc))
                return False
            if not pressure_is_near_mapping_zero(pressure):
                self._set_status(
                    f"开始前压力为 {pressure:.2f} N，须先释放到"
                    f"±{MAPPING_ZERO_TOLERANCE_N:.2f} N"
                )
                return False
        return True

    def _start_external_operation(self, kind, operation, initial_status):
        self._running = True
        self._operation_kind = kind

        def task(motor, stop_event):
            try:
                if stop_event.is_set():
                    raise ForcePositionMappingError("用户停止操作")
                motor.prepare_motion()
                result = operation(motor, stop_event)
            except Exception as exc:
                result = {"ok": False, "kind": kind, "message": str(exc)}
            finally:
                try:
                    motor.stop()
                except Exception as exc:
                    result = {"ok": False, "kind": kind, "message": f"停止未确认，请检查电机：{exc}"}
            self.operation_finished.emit(result)

        accepted, reason = self._motor_control.start_external_operation(
            task,
            initial_status,
        )
        if not accepted:
            self._running = False
            self._operation_kind = None
            self._set_status(f"无法启动：{reason}")
            return False
        self._set_status(initial_status)
        self._refresh_controls()
        return True

    def _run_calibration(self, motor, stop_event, parameters):
        motor.set_current_limit_ma(parameters["max_current_ma"])
        rounds = []
        output_dir = self._mapping_directory()
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        csv_path = output_dir / f"force_position_scan_{stamp}.csv"
        with csv_path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=(
                    "timestamp",
                    "round_index",
                    "point_index",
                    "force_n",
                    "position_counts",
                    "relative_travel_counts",
                    "commanded_step_counts",
                    "actual_step_counts",
                    "target_error_counts",
                    "pressure_sample_count",
                ),
            )
            writer.fieldnames = list(self._profile.storage_fields(dict.fromkeys(writer.fieldnames)))
            def write_row(row):
                writer.writerow(self._profile.storage_fields(row))
            writer.writeheader()
            for round_index in range(1, parameters["repeat_count"] + 1):
                if stop_event.is_set():
                    raise ForcePositionMappingError("用户停止扫描")
                start_force = self._wait_for_mapping_zero(
                    stop_event, f"第{round_index}轮开始前"
                )
                motor.prepare_motion()
                start_position = int(motor.read_position_steps())
                points = [
                    {
                        "force_n": max(0.0, start_force),
                        "position_counts": start_position,
                        "pressure_sample_count": 1,
                    }
                ]
                travel = 0
                point_index = 0
                write_row(
                    {
                        "timestamp": datetime.now().astimezone().isoformat(
                            timespec="milliseconds"
                        ),
                        "round_index": round_index,
                        "point_index": point_index,
                        "force_n": points[0]["force_n"],
                        "position_counts": start_position,
                        "relative_travel_counts": 0,
                        "commanded_step_counts": 0,
                        "actual_step_counts": 0,
                        "target_error_counts": 0,
                        "pressure_sample_count": 1,
                    }
                )
                handle.flush()
                self.progress_changed.emit(
                    {
                        "message": (
                            f"第 {round_index}/{parameters['repeat_count']} 轮："
                            f"从位置 {start_position} step开始慢速下压"
                        )
                    }
                )
                while points[-1]["force_n"] < parameters["max_force_n"]:
                    if travel + parameters["press_step_counts"] > parameters["max_travel_counts"]:
                        raise ForcePositionMappingError(
                            f"第{round_index}轮达到最大行程{parameters['max_travel_counts']} step，"
                            f"压力仅{points[-1]['force_n']:.2f} N"
                        )
                    feedback = self._move_relative_checked(
                        motor,
                        stop_event,
                        -parameters["press_step_counts"],
                        parameters["press_speed_level"],
                        parameters["acceleration_r_s2"],
                        parameters["max_current_ma"],
                        accept_settled_progress=True,
                        clear_state_before_move=False,
                    )
                    travel = abs(int(feedback["position_counts"]) - start_position)
                    force, sample_count = self._sample_pressure_median(stop_event)
                    point_index += 1
                    point = {
                        "force_n": force,
                        "position_counts": int(feedback["position_counts"]),
                        "pressure_sample_count": sample_count,
                    }
                    points.append(point)
                    write_row(
                        {
                            "timestamp": datetime.now().astimezone().isoformat(
                                timespec="milliseconds"
                            ),
                            "round_index": round_index,
                            "point_index": point_index,
                            "force_n": force,
                            "position_counts": point["position_counts"],
                            "relative_travel_counts": travel,
                            "commanded_step_counts": feedback[
                                "commanded_delta_counts"
                            ],
                            "actual_step_counts": feedback["actual_delta_counts"],
                            "target_error_counts": feedback[
                                "target_error_counts"
                            ],
                            "pressure_sample_count": sample_count,
                        }
                    )
                    handle.flush()
                    self.progress_changed.emit(
                        {
                            "message": (
                                f"第 {round_index}/{parameters['repeat_count']} 轮，"
                                f"第{point_index}步：{force:.2f} N，"
                                f"位置 {point['position_counts']} step，"
                                f"本步实际 {abs(feedback['actual_delta_counts'])} step，"
                                f"累计下压 {travel} step"
                            )
                        }
                    )
                rounds.append(
                    {
                        "round_index": round_index,
                        "release_reference_position_counts": parameters[
                            "release_position_counts"
                        ],
                        "start_position_counts": start_position,
                        "end_position_counts": points[-1]["position_counts"],
                        "end_force_n": points[-1]["force_n"],
                        "points": points,
                    }
                )
                release_result = self._release_to_clearance(
                    motor,
                    stop_event,
                    parameters,
                )
                rounds[-1]["release_position_counts"] = release_result[
                    "final_position_counts"
                ]

        mapping = build_force_position_map(
            rounds,
            parameters["max_force_n"],
            parameters["position_span_limit_counts"],
        )
        metadata = {
            **self._profile.metadata(),
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "schema_version": 2,
            "force_unit": "N",
            "max_force_n": parameters["max_force_n"],
            "release_zero_tolerance_n": MAPPING_ZERO_TOLERANCE_N,
            "release_zero_wait_seconds": MAPPING_ZERO_WAIT_SECONDS,
            "force_step_n": MAP_FORCE_STEP_N,
            "repeat_count": parameters["repeat_count"],
            "mapping_mode": "release_relative_travel",
            "release_reference_position_counts": parameters[
                "release_position_counts"
            ],
            "selected_round_indices": [
                int(round_result["round_index"]) for round_result in rounds
            ],
            "selected_round_count": len(rounds),
            "available_round_count": len(rounds),
            "press_speed_level": parameters["press_speed_level"],
            "press_step_counts": parameters["press_step_counts"],
            "position_span_limit_counts": parameters["position_span_limit_counts"],
            "csv_path": str(csv_path),
        }
        archive_path = output_dir / f"force_position_map_{stamp}.json"
        payload = {"metadata": metadata, "rounds": rounds, "mapping": mapping}
        self._write_mapping_json(archive_path, payload)
        self._write_mapping_json(self._latest_mapping_path(), payload)
        return {
            "ok": True,
            "kind": "calibration",
            "message": f"完成{len(rounds)}轮扫描，生成{len(mapping)}个0.10 N档位",
            "rounds": rounds,
            "mapping": mapping,
            "metadata": metadata,
            "archive_path": str(archive_path),
        }

    def _write_mapping_json(self, path, payload):
        payload = dict(payload)
        payload["metadata"] = dict(payload.get("metadata", {}), **self._profile.metadata())
        if self._profile.namespace:
            payload["metadata"]["schema_version"] = 3
        payload = self._profile.storage_fields(payload)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary.replace(path)

    def _run_lookup_move(self, motor, stop_event, parameters):
        motor.set_current_limit_ma(parameters["max_current_ma"])
        current_position = int(motor.read_position_steps())
        target_position = int(parameters["lookup_position_counts"])
        delta_counts = target_position - current_position
        if delta_counts >= 0:
            raise ForcePositionMappingError(
                f"当前位于{current_position} step，查表目标{target_position} step不在下压方向；"
                "请重新设置释放位置或检查压力－行程查找表"
            )
        if abs(delta_counts) > int(parameters["max_travel_counts"]):
            raise ForcePositionMappingError(
                f"查表需要移动{abs(delta_counts)} step，超过允许的单次最大行程"
                f"{parameters['max_travel_counts']} step；压力－行程查找表可能已经失效"
            )
        self.progress_changed.emit(
            {
                "message": (
                    f"释放位置{parameters['release_position_counts']} - 下压行程"
                    f"{parameters['lookup_press_travel_counts']} = "
                    f"{target_position} step；"
                    f"从{current_position}一次性下压{delta_counts} step"
                )
            }
        )
        feedback = self._move_relative_checked(
            motor,
            stop_event,
            delta_counts,
            parameters["press_speed_level"],
            parameters["acceleration_r_s2"],
            parameters["max_current_ma"],
        )
        if (
            abs(int(feedback["position_counts"]) - target_position)
            > self._profile.tolerance(MOTOR_POSITION_TOLERANCE_COUNTS)
        ):
            raise ForcePositionMappingError(
                f"压力保护提前停止：查表目标{target_position}，"
                f"实际{feedback['position_counts']} step"
            )
        actual_force, sample_count = self._sample_pressure_median(
            stop_event,
            duration_seconds=1.0,
        )
        return {
            "ok": True,
            "kind": "lookup",
            "message": (
                f"查表目标{parameters['lookup_force_n']:.2f} N，下压行程"
                f"{parameters['lookup_press_travel_counts']} step，"
                f"目标位置{target_position} step；"
                f"实际停止{feedback['position_counts']} step，1秒压力中位值{actual_force:.2f} N"
            ),
            "target_force_n": parameters["lookup_force_n"],
            "target_position_counts": target_position,
            "actual_position_counts": feedback["position_counts"],
            "actual_force_n": actual_force,
            "pressure_sample_count": sample_count,
        }

    def _run_release_operation(self, motor, stop_event, parameters):
        motor.set_current_limit_ma(parameters["max_current_ma"])
        result = self._release_to_clearance(motor, stop_event, parameters)
        return {
            "ok": True,
            "kind": "release",
            "message": result["message"],
            **result,
        }

    def _release_to_clearance(self, motor, stop_event, parameters):
        start_position = int(motor.read_position_steps())
        target_position = int(parameters["release_position_counts"])
        delta_counts = target_position - start_position
        if delta_counts < -self._profile.tolerance(RELEASE_POSITION_TOLERANCE_COUNTS):
            raise ForcePositionMappingError(
                f"当前位置{start_position} step已高于释放位置{target_position} step；"
                "为避免反向下压，未自动退回，请在无压力位置重新设置释放位置"
            )
        if abs(delta_counts) > int(parameters["max_travel_counts"]):
            raise ForcePositionMappingError(
                f"到固定释放位置需要移动{abs(delta_counts)} step，超过安全行程"
                f"{parameters['max_travel_counts']} step"
            )
        if abs(delta_counts) <= self._profile.tolerance(RELEASE_POSITION_TOLERANCE_COUNTS):
            feedback = {
                "position_counts": start_position,
                "actual_delta_counts": 0,
            }
        else:
            feedback = self._move_relative_checked(
                motor,
                stop_event,
                delta_counts,
                parameters["release_speed_level"],
                parameters["acceleration_r_s2"],
                parameters["max_current_ma"],
                position_tolerance_counts=self._profile.tolerance(RELEASE_POSITION_TOLERANCE_COUNTS),
            )
        # 电机到位后等待新的压力帧，避免拿运动结束前的缓存读数判断回零。
        final_force = self._wait_for_mapping_zero(
            stop_event, "已回到固定释放位置",
            after_sequence=self._pressure_snapshot()[3],
        )
        actual_position = int(feedback["position_counts"])
        position_error = actual_position - target_position
        force_text = "--" if final_force is None else f"{final_force:.2f}"
        self.progress_changed.emit(
            {
                "message": (
                    f"固定位置释放完成：{start_position} → {actual_position} step；"
                    f"目标{target_position}，误差{position_error:+d} step，"
                    f"压力{force_text} N"
                )
            }
        )
        return {
            "start_position_counts": start_position,
            "final_position_counts": actual_position,
            "final_force_n": final_force,
            "message": (
                f"已一次性运动到固定释放位置附近：目标{target_position}，"
                f"实际{actual_position} step，误差{position_error:+d} step，"
                f"压力{force_text} N"
            ),
        }

    def _move_relative_checked(
        self,
        motor,
        stop_event,
        delta_counts,
        speed_level,
        acceleration,
        max_current_ma,
        accept_settled_progress=False,
        clear_state_before_move=True,
        position_tolerance_counts=None,
    ):
        if position_tolerance_counts is None:
            position_tolerance_counts = self._profile.tolerance(MOTOR_POSITION_TOLERANCE_COUNTS)
        if stop_event.is_set():
            raise ForcePositionMappingError("用户停止操作")
        if clear_state_before_move:
            motor.prepare_motion()
        start_position = int(motor.read_position_steps())
        target_position = start_position + int(delta_counts)
        speed_rpm = self._profile.speed_rpm(speed_level)
        force, _sequence = self._fresh_pressure()
        if delta_counts < 0 and force >= ABSOLUTE_SAFETY_FORCE_N:
            raise ForcePositionMappingError("压力已达到绝对保护上限，禁止继续下压")
        motor.move_relative_steps(
            int(delta_counts),
            speed_rpm=speed_rpm,
            accel=int(acceleration),
            retry_cancelled=stop_event.is_set,
        )
        deadline = time.monotonic() + self._profile.motion_timeout(delta_counts, speed_rpm, acceleration)
        last_position = None
        stable_samples = 0
        current_warning_recorded = False
        while time.monotonic() < deadline:
            if stop_event.is_set():
                raise ForcePositionMappingError("用户停止操作")
            sample = motor.read_feedback()
            position = int(sample["position_counts"])
            speed = float(sample["measured_speed_rpm"])
            phase_current_ma = int(sample["phase_current_ma"])
            current = abs(phase_current_ma)
            running_state = int(sample["running_state"])
            force, _sequence = self._fresh_pressure()
            feedback = {
                "position_counts": position,
                "position_deg": sample.get("position_deg", position / self._profile.steps_per_turn * 360.0),
                "measured_speed_rpm": speed,
                "phase_current_ma": phase_current_ma,
                "running_state": running_state,
                "sampled_monotonic": time.monotonic(),
                "command_start_position_counts": start_position,
                "command_target_position_counts": target_position,
                "commanded_delta_counts": int(delta_counts),
                "actual_delta_counts": position - start_position,
                "target_error_counts": position - target_position,
            }
            self._motor_control.publish_motor_feedback(feedback)
            if delta_counts < 0 and force >= ABSOLUTE_SAFETY_FORCE_N:
                motor.stop()
                raise ForcePositionMappingError(
                    f"压力{force:.2f} N达到绝对保护上限{ABSOLUTE_SAFETY_FORCE_N:.2f} N"
                )
            if current > max_current_ma and not current_warning_recorded:
                current_warning_recorded = True
                self._motor_control.record_motion_diagnostic(
                    "force_position_current_limit_observed",
                    phase_current_ma=phase_current_ma,
                    max_current_ma=max_current_ma,
                    position_counts=position,
                    target_position_counts=target_position,
                )
            if running_state in (3, 4, 5):
                raise ForcePositionMappingError(f"电机异常状态{running_state}")
            if last_position is not None and abs(position - last_position) <= self._profile.tolerance(MOTOR_STABLE_DELTA_COUNTS):
                stable_samples += 1
            else:
                stable_samples = 0
            if (
                abs(position - target_position) <= int(position_tolerance_counts)
                and (position - start_position) * delta_counts > 0
                and abs(speed) <= 1
                and stable_samples >= MOTOR_STABLE_SAMPLE_COUNT
            ):
                return feedback
            if (
                accept_settled_progress
                and abs(speed) <= 1
                and stable_samples >= MOTOR_STABLE_SAMPLE_COUNT
            ):
                # 标定记录的是电机真实位置；负载下单步未走满时，只要方向和位移合理就继续采点。
                direction = 1 if delta_counts >= 0 else -1
                commanded_distance = abs(int(delta_counts))
                actual_progress = (position - start_position) * direction
                minimum_progress = min(
                    commanded_distance,
                    self._profile.tolerance(SCAN_MIN_SETTLED_PROGRESS_COUNTS),
                )
                maximum_progress = max(
                    commanded_distance + int(position_tolerance_counts),
                    int(math.ceil(commanded_distance * SCAN_MAX_SETTLED_PROGRESS_RATIO)),
                )
                if minimum_progress <= actual_progress <= maximum_progress:
                    return feedback
                if actual_progress > maximum_progress:
                    raise ForcePositionMappingError(
                        f"单步异常超程：指令{delta_counts} step，"
                        f"实际{position - start_position} step"
                    )
            last_position = position
            stop_event.wait(POLL_SECONDS)
        raise ForcePositionMappingError(
            f"运动到位超时：起点{start_position}，目标{target_position}，"
            f"实际{last_position} step；本步实际移动"
            f"{None if last_position is None else last_position - start_position} step"
        )

    def _sample_pressure_median(self, stop_event, duration_seconds=None):
        duration = (
            PRESSURE_SAMPLE_WINDOW_SECONDS
            if duration_seconds is None
            else float(duration_seconds)
        )
        deadline = time.monotonic() + duration
        values = []
        _force, last_sequence = self._fresh_pressure()
        while time.monotonic() < deadline or len(values) < PRESSURE_SAMPLE_MIN_COUNT:
            if stop_event.is_set():
                raise ForcePositionMappingError("用户停止操作")
            force, sequence = self._fresh_pressure()
            if sequence != last_sequence:
                values.append(force)
                last_sequence = sequence
            if time.monotonic() > deadline + PRESSURE_FRESH_SECONDS:
                break
            stop_event.wait(0.02)
        if len(values) < PRESSURE_SAMPLE_MIN_COUNT:
            raise ForcePositionMappingError(
                f"压力采样不足：仅{len(values)}/{PRESSURE_SAMPLE_MIN_COUNT}帧"
            )
        return float(statistics.median(values)), len(values)

    @Slot(object)
    def _on_progress(self, progress):
        self._set_status(progress.get("message", "运行中"))

    @Slot(object)
    def _on_operation_finished(self, result):
        self._running = False
        self._operation_kind = None
        if result.get("ok") and result.get("kind") == "calibration":
            self._round_results = list(result.get("rounds") or ())
            self._mapping = list(result.get("mapping") or ())
            self._mapping_metadata = dict(result.get("metadata") or {})
            self._rebuild_round_selector(
                self._mapping_metadata.get("selected_round_indices")
            )
            self._populate_mapping_table()
            self._set_status(result.get("message", "扫描完成"))
            self.mapping_changed.emit()
        elif result.get("ok") and result.get("kind") == "lookup":
            self.lookup_result_label.setText(result.get("message", "查表定位完成"))
            self._set_status(result.get("message", "查表定位完成"))
        else:
            self._set_status(result.get("message", "操作完成"))
        self._refresh_controls()

    @staticmethod
    def _round_index(round_result, fallback):
        try:
            return int(round_result.get("round_index", fallback))
        except (AttributeError, TypeError, ValueError):
            return int(fallback)

    def _rebuild_round_selector(self, selected_round_indices=None):
        while self.round_selector_layout.count():
            item = self.round_selector_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self._round_checkboxes = []
        if not self._round_results:
            empty_label = QtWidgets.QLabel("暂无完整扫描轮次")
            empty_label.setStyleSheet("color:#8996a8;")
            self.round_selector_layout.addWidget(empty_label)
            self.round_selector_layout.addStretch(1)
            return

        selected = None
        if isinstance(selected_round_indices, (list, tuple, set)):
            selected = set()
            for value in selected_round_indices:
                try:
                    selected.add(int(value))
                except (TypeError, ValueError):
                    continue
        for fallback, round_result in enumerate(self._round_results, start=1):
            round_index = self._round_index(round_result, fallback)
            checkbox = QtWidgets.QCheckBox(f"第{round_index}轮")
            checkbox.setChecked(selected is None or round_index in selected)
            start_position = round_result.get("start_position_counts", "--")
            end_position = round_result.get("end_position_counts", "--")
            end_force = round_result.get("end_force_n")
            force_text = "--" if end_force is None else f"{float(end_force):.2f} N"
            checkbox.setToolTip(
                f"起点 {start_position} step；终点 {end_position} step；终点压力 {force_text}"
            )
            self.round_selector_layout.addWidget(checkbox)
            self._round_checkboxes.append((round_index, checkbox))
        self.round_selector_layout.addStretch(1)

    def _selected_round_indices(self):
        return [
            round_index
            for round_index, checkbox in self._round_checkboxes
            if checkbox.isChecked()
        ]

    @Slot()
    def _recalculate_selected_rounds(self):
        selected_indices = self._selected_round_indices()
        if len(selected_indices) < 2:
            self._set_status("至少勾选2轮才能重新统计稳定的位置－压力查找表")
            return
        selected_set = set(selected_indices)
        selected_rounds = [
            round_result
            for fallback, round_result in enumerate(self._round_results, start=1)
            if self._round_index(round_result, fallback) in selected_set
        ]
        if len(selected_rounds) < 2:
            self._set_status("所选轮次缺少完整原始数据，无法重新统计")
            return

        try:
            max_force_n = float(
                self._mapping_metadata.get(
                    "max_force_n", self.max_force_input.value()
                )
            )
        except (TypeError, ValueError):
            max_force_n = self.max_force_input.value()
        span_limit_counts = self.span_limit_input.value()
        mapping = build_force_position_map(
            selected_rounds,
            max_force_n,
            span_limit_counts,
        )
        now = datetime.now().astimezone()
        metadata = dict(self._mapping_metadata)
        source_created_at = metadata.get("source_created_at") or metadata.get(
            "created_at"
        )
        metadata.update(
            {
                "schema_version": 2,
                "force_unit": "N",
                "created_at": now.isoformat(timespec="seconds"),
                "source_created_at": source_created_at,
                "selection_recalculated": True,
                "mapping_mode": "release_relative_travel",
                "selected_round_indices": selected_indices,
                "selected_round_count": len(selected_rounds),
                "available_round_count": len(self._round_results),
                "position_span_limit_counts": span_limit_counts,
            }
        )
        payload = {
            "metadata": metadata,
            "rounds": self._round_results,
            "mapping": mapping,
        }
        stamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        archive_path = (
            self._mapping_directory() / f"force_position_map_selected_{stamp}.json"
        )
        try:
            self._write_mapping_json(archive_path, payload)
            self._write_mapping_json(self._latest_mapping_path(), payload)
        except OSError as exc:
            self._set_status(f"统计完成，但保存查找表失败：{exc}")
        else:
            selected_text = "、".join(str(index) for index in selected_indices)
            self._set_status(
                f"已按第{selected_text}轮重新统计并保存为最新查找表"
            )
        self._mapping = mapping
        self._mapping_metadata = metadata
        self._populate_mapping_table()
        self.mapping_changed.emit()
        self._refresh_controls()

    def _populate_mapping_table(self):
        # 批量填充后只测量一次列宽，避免每个单元格触发全表测量而阻塞界面。
        updates_enabled = self.table.updatesEnabled()
        self.table.setUpdatesEnabled(False)
        try:
            self.table.setRowCount(len(self._mapping))
            stable_count = 0
            spans = []
            for row, entry in enumerate(self._mapping):
                stable = bool(entry.get("stable"))
                if stable:
                    stable_count += 1
                spans.append(float(entry["position_span_counts"]))
                median_travel = entry.get(
                    "median_press_travel_counts", entry.get("press_travel_counts")
                )
                minimum_travel = entry.get(
                    "minimum_press_travel_counts", entry.get("minimum_position_counts")
                )
                maximum_travel = entry.get(
                    "maximum_press_travel_counts", entry.get("maximum_position_counts")
                )
                values = (
                    f"{float(entry['force_n']):.2f}",
                    int(round(float(median_travel))),
                    f"{float(minimum_travel):.1f}",
                    f"{float(maximum_travel):.1f}",
                    f"{entry['position_span_counts']:.1f}",
                    f"{entry['position_standard_deviation_counts']:.1f}",
                    entry["sample_count"],
                    "是" if stable else "否",
                )
                for column, value in enumerate(values):
                    item = QtWidgets.QTableWidgetItem(str(value))
                    if column == 7:
                        item.setForeground(
                            QtGui.QColor("#55c878" if stable else "#ff6b6b")
                        )
                    self.table.setItem(row, column, item)
            self.table.resizeColumnsToContents()
        finally:
            self.table.setUpdatesEnabled(updates_enabled)
        if self._mapping:
            self.lookup_force_input.setMaximum(
                float(max(item["force_n"] for item in self._mapping))
            )
        maximum_span = max(spans) if spans else 0.0
        selected_indices = self._mapping_metadata.get("selected_round_indices")
        if not isinstance(selected_indices, list):
            selected_indices = [
                self._round_index(round_result, fallback)
                for fallback, round_result in enumerate(
                    self._round_results, start=1
                )
            ]
        selected_text = "、".join(str(index) for index in selected_indices) or "无"
        self.summary_label.setText(
            f"查找表共{len(self._mapping)}档，稳定{stable_count}档；"
            f"最大下压行程极差{maximum_span:.1f} step；"
            f"判定上限{self._mapping_metadata.get('position_span_limit_counts', self.span_limit_input.value())} step；"
            f"统计轮次{len(selected_indices)}/{len(self._round_results)}（第{selected_text}轮）；"
            f"生成时间{self._mapping_metadata.get('created_at', '--')}"
        )
        self.lookup_result_label.setText(
            "请选择绿色稳定档位；定位前压力必须处于零点容差内"
        )

    def _load_latest_mapping(self):
        path = self._latest_mapping_path()
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload = self._profile.storage_fields(payload, read=True)
        except (OSError, ValueError, TypeError):
            return
        mapping = payload.get("mapping") if isinstance(payload, dict) else None
        if not isinstance(mapping, list):
            return
        self._mapping_metadata = dict(payload.get("metadata") or {})
        expected = self._profile.metadata()
        if self._profile.namespace and any(self._mapping_metadata.get(k) != v for k, v in expected.items()):
            self._set_status("查找表电机型号、step单位或每圈步数不匹配，请重新标定")
            return
        if self._mapping_metadata.get("force_unit") != "N":
            self._mapping = []
            self._round_results = []
            self._rebuild_round_selector()
            self.table.setRowCount(0)
            message = (
                "检测到旧的 g 单位位置查找表；为防止单位误用，已禁止定位，"
                "请重新执行 N 单位标定"
            )
            self.summary_label.setText(message)
            self.lookup_result_label.setText("旧 g 表不可用于 N 单位定位")
            self._set_status(message)
            return
        self._mapping = mapping
        self._round_results = list(payload.get("rounds") or ())
        invalidated_reason = self._mapping_metadata.get("invalidated_reason")
        if not self._mapping and invalidated_reason:
            self._rebuild_round_selector()
            self.table.setRowCount(0)
            if invalidated_reason == "new_scan_started":
                message = (
                    "旧查找表已因开始新扫描而作废；"
                    "上次新扫描未生成可用查找表，请重新扫描"
                )
            elif invalidated_reason == "motor_position_zeroed":
                message = "电机位置已清零，旧查找表已经失效，请重新扫描"
            else:
                message = "旧查找表已失效，请重新扫描"
            self.summary_label.setText(message)
            self.lookup_result_label.setText("当前没有可用于定位的查找表")
            self._set_status(message)
            return
        if self._round_results and any(
            "press_travel_counts" not in entry for entry in self._mapping
        ):
            selected_indices = self._mapping_metadata.get(
                "selected_round_indices"
            )
            if not isinstance(selected_indices, list):
                selected_indices = [
                    self._round_index(round_result, fallback)
                    for fallback, round_result in enumerate(
                        self._round_results, start=1
                    )
                ]
            selected_set = set(int(index) for index in selected_indices)
            selected_rounds = [
                round_result
                for fallback, round_result in enumerate(
                    self._round_results, start=1
                )
                if self._round_index(round_result, fallback) in selected_set
            ]
            self._mapping = build_force_position_map(
                selected_rounds,
                float(
                    self._mapping_metadata.get(
                        "max_force_n", self.max_force_input.value()
                    )
                ),
                int(
                    self._mapping_metadata.get(
                        "position_span_limit_counts",
                        self.span_limit_input.value(),
                    )
                ),
            )
            self._mapping_metadata["mapping_mode"] = (
                "release_relative_travel"
            )
            self._mapping_metadata["relative_mapping_migrated_at"] = (
                datetime.now().astimezone().isoformat(timespec="seconds")
            )
            payload = {
                "metadata": self._mapping_metadata,
                "rounds": self._round_results,
                "mapping": self._mapping,
            }
            try:
                self._write_mapping_json(path, payload)
            except OSError:
                pass
        self._rebuild_round_selector(
            self._mapping_metadata.get("selected_round_indices")
        )
        self._populate_mapping_table()
        self._set_status(f"已加载上次查找表：{path}")

    @Slot(object)
    def _on_motor_feedback(self, feedback):
        self._show_motor_feedback(feedback)

    def _show_motor_feedback(self, feedback):
        position = feedback.get("position_counts")
        speed = feedback.get("measured_speed_rpm")
        current = feedback.get("phase_current_ma")
        if position is None:
            return
        # 优先显示驱动器反馈角度；旧驱动未提供时按每圈步数换算。
        angle = feedback.get("position_deg")
        if angle is None:
            angle = float(position) / self._profile.steps_per_turn * 360.0
        speed_text = "--" if speed is None else str(int(speed))
        current_text = "--" if current is None else str(int(current))
        self.position_label.setText(
            f"电机位置 {int(position)} step · 角度 {angle:.2f}° · 速度 {speed_text} RPM · "
            f"瞬时相电流 {current_text} mA"
        )

    @Slot()
    def _on_position_zeroed(self):
        self._mapping = []
        self._round_results = []
        self._mapping_metadata = {}
        self._rebuild_round_selector()
        self.table.setRowCount(0)
        self.summary_label.setText("电机位置已清零，旧查找表已经失效，请重新扫描")
        self.lookup_result_label.setText("旧查找表已失效")
        try:
            self._write_mapping_json(
                self._latest_mapping_path(),
                {
                    "metadata": {
                        "force_unit": "N",
                        "invalidated_at": datetime.now().astimezone().isoformat(
                            timespec="seconds"
                        ),
                        "invalidated_reason": "motor_position_zeroed",
                    },
                    "rounds": [],
                    "mapping": [],
                },
            )
        except OSError:
            pass
        self.mapping_changed.emit()
        self._refresh_controls()

    @Slot(bool, bool)
    def _on_motor_state_changed(self, _connected, _busy):
        self._refresh_controls()

    @Slot()
    def request_stop(self):
        self._motor_control.request_external_stop()
        self._set_status("已请求停止，等待当前CAN操作安全刹车")

    def _set_status(self, message):
        message = str(message).replace("counts", "step")
        self.status_label.setText(str(message))
        self.status_message.emit(str(message))

    def _refresh_controls(self):
        connected = self._motor_control.is_can_connected()
        busy = self._motor_control.is_operation_busy()
        available = connected and not busy and not self._running
        release_confirmed = self._confirmed_release_position() is not None
        self.set_release_position_button.setEnabled(available)
        self.zero_position_button.setEnabled(available)
        self.start_button.setEnabled(available and release_confirmed)
        self.release_button.setEnabled(available and release_confirmed)
        self.lookup_button.setEnabled(
            available and release_confirmed and bool(self._mapping)
        )
        can_recalculate = not busy and not self._running and len(self._round_results) >= 2
        self.recalculate_button.setEnabled(can_recalculate)
        for _round_index, checkbox in self._round_checkboxes:
            checkbox.setEnabled(not busy and not self._running)
        self.stop_button.setEnabled(connected and (busy or self._running))
        for widget in (
            self.max_force_input,
            self.repeat_count_input,
            self.press_speed_input,
            self.press_step_input,
            self.release_speed_input,
            self.span_limit_input,
            self.max_travel_input,
            self.max_current_input,
            self.acceleration_input,
            self.lookup_force_input,
        ):
            widget.setEnabled(not self._running and not busy)

    def shutdown(self):
        self._closing = True
        if self._running:
            self._motor_control.request_external_stop()
        self._save_settings_if_idle()

    def is_running(self):
        return self._running
