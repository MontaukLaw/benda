"""Compact Qt motor controls embedded in the algorithm monitor."""

from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

from qt_compat import QtCore, QtWidgets, Signal, Slot
from motor_auto_test import (
    MotorAutoTestCancelled,
    MotorAutoTestConfig,
    MotorAutoTester,
    RandomPressTestRunner,
    RepeatabilityTestRunner,
)
from step_moto_can_driver import PD42S1Config, PD42S1Motor


DEFAULT_SETTINGS = {
    "move_count_units": 1,
    "speed_level": 5,
    "acceleration": 100,
    "wait_seconds": 1.0,
    "repeat_count": 5,
    "press_first": True,
    "auto_zero_step_deg": 5.0,
    "auto_max_step_deg": 10.0,
    "auto_contact_signal_threshold": 0,
    "auto_raw_limit": 60000,
    "auto_ready_lift_deg": 5.0,
    "auto_sensor_settle_seconds": 0.30,
    "auto_sensor_frame_timeout_seconds": 1.0,
    "auto_motor_position_timeout_seconds": 8.0,
    "auto_max_search_travel_deg": 720.0,
    "auto_confirmation_frames": 3,
    "random_test_rounds": 10,
    "random_test_min_angle_deg": 10,
    "random_test_max_angle_deg": 100,
    "random_test_hold_min_seconds": 1.0,
    "random_test_hold_max_seconds": 5.0,
    "random_test_release_observe_seconds": 15.0,
    "repeatability_count_units": 1,
    "rapid_floating_channel_count": 1,
    "rapid_precondition_relative_span_percent_max": 0.5,
    "rapid_formal_rounds": 10,
    "rapid_precondition_max_rounds": 10,
    "rapid_floating_timeout_seconds": 20.0,
    "rapid_floating_hold_seconds": 2.0,
    "release_position_counts": None,
    "release_position_saved_at": None,
}

POSITION_COUNT_UNIT = 4096
MOTOR_PROTOCOL_MIN_RPM = 1
MOTOR_PROTOCOL_MAX_RPM = PD42S1Motor.MAX_SPEED_RPM
MOTOR_UI_MAX_RPM = 400
CAN_LOG_MAX_BYTES = 5 * 1024 * 1024
CAN_LOG_MAX_FILES = 10
RELEASE_POSITION_CONFIRM_TOLERANCE_COUNTS = 50
MOTOR_RUNNING_STATE_NAMES = {
    0: "停止",
    1: "任务完成",
    2: "正在运行",
    3: "过载",
    4: "堵转",
    5: "欠压",
}


def rpm_to_counts_per_second(speed_rpm: int) -> int:
    return round(int(speed_rpm) * PD42S1Motor.COUNTS_PER_TURN / 60.0)


def counts_per_second_to_speed_level(speed_counts_per_second: int) -> int:
    return max(1, round(int(speed_counts_per_second) / POSITION_COUNT_UNIT))


def speed_level_to_counts_per_second(speed_level: int) -> int:
    return int(speed_level) * POSITION_COUNT_UNIT


def counts_per_second_to_rpm(speed_counts_per_second: int) -> int:
    speed_rpm = round(
        int(speed_counts_per_second) * 60.0 / PD42S1Motor.COUNTS_PER_TURN
    )
    return max(MOTOR_PROTOCOL_MIN_RPM, min(MOTOR_PROTOCOL_MAX_RPM, speed_rpm))


def motor_settings_path() -> Path:
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "motor_control_config.json"
    return Path(__file__).with_name("motor_control_config.json")


def can_log_directory() -> Path:
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "can_logs"
    return Path(__file__).with_name("can_logs")


def automatic_test_directory() -> Path:
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "automatic_tests"
    return Path(__file__).with_name("automatic_tests")


class MotorControlWidget(QtWidgets.QGroupBox):
    """Owns the PCAN connection and all motor operations."""

    worker_status = Signal(str)
    worker_connected = Signal(bool)
    worker_finished = Signal(float)
    worker_position = Signal(int)
    worker_current = Signal(int)
    worker_position_zeroed = Signal()
    worker_release_position_captured = Signal(object)
    auto_test_result = Signal(object)
    repeatability_log = Signal(str)
    control_state_changed = Signal(bool, bool)
    motor_feedback = Signal(object)
    release_position_changed = Signal(object)

    def __init__(self, parent=None, sensor_sample_provider=None, communication_enabled=True):
        super().__init__("电机压力控制", parent)
        self._communication_enabled = communication_enabled
        self.setObjectName("MotorControl")
        self.setCheckable(True)
        self.setChecked(False)

        self.motor = None
        self._busy = False
        self._repeating = False
        self._closing = False
        self._sensor_sample_provider = sensor_sample_provider
        self._zero_position_counts = None
        self._maximum_position_counts = None
        self._maximum_press_angle_deg = None
        self._auto_test_kind = None
        self._stop_event = threading.Event()
        self._task_queue = queue.Queue()
        self._service_stop_event = threading.Event()
        self._service_thread = None
        self._feedback_lock = threading.Lock()
        self._latest_motor_feedback = None
        self._feedback_last_position = None
        self._feedback_last_speed_counts_s = None
        self._feedback_last_sample_at = None
        self._settings_path = motor_settings_path()
        self._worker_started_at = None
        self._can_log_lock = threading.Lock()
        self._can_log_handle = None
        self._can_log_path = None
        self._can_log_open_error = None
        self._can_log_size_bytes = 0
        self._save_timer = QtCore.QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.setInterval(400)
        self._save_timer.timeout.connect(self._save_settings)

        settings = self._load_settings()
        try:
            self._release_position_counts = (
                None
                if settings.get("release_position_counts") is None
                else int(settings["release_position_counts"])
            )
        except (TypeError, ValueError):
            self._release_position_counts = None
        self._release_position_saved_at = settings.get(
            "release_position_saved_at"
        )
        self._release_position_confirmed = False
        self._random_test_settings = {
            "rounds": max(1, int(settings["random_test_rounds"])),
            "min_angle_deg": max(1, int(settings["random_test_min_angle_deg"])),
            "max_angle_deg": max(1.0, float(settings["random_test_max_angle_deg"])),
            "hold_min_seconds": max(0.0, float(settings["random_test_hold_min_seconds"])),
            "hold_max_seconds": max(0.0, float(settings["random_test_hold_max_seconds"])),
            "release_observe_seconds": max(0.1, float(settings["random_test_release_observe_seconds"])),
        }
        if self._random_test_settings["hold_max_seconds"] < self._random_test_settings["hold_min_seconds"]:
            self._random_test_settings["hold_max_seconds"] = self._random_test_settings["hold_min_seconds"]
        self._auto_config_error = None
        try:
            self._auto_config = MotorAutoTestConfig.from_mapping(settings)
        except ValueError as exc:
            self._auto_config = MotorAutoTestConfig()
            self._auto_config_error = str(exc)
        self._build_ui(settings)
        self._configure_numeric_inputs()
        can_log_path = self._open_can_log()
        if can_log_path is None:
            self.can_log_status_label.setText(
                f"CAN日志创建失败：{self._can_log_open_error}"
            )
            self.can_log_status_label.setStyleSheet("color:#ff7b72;")
        else:
            self.can_log_status_label.setToolTip(str(can_log_path.parent))
            self._record_can_event(
                f"LOG realtime rolling started file={can_log_path} "
                f"max_bytes={CAN_LOG_MAX_BYTES} max_files={CAN_LOG_MAX_FILES}"
            )
        self.toggled.connect(self._set_expanded)
        self.worker_status.connect(self._set_status)
        self.worker_connected.connect(self._set_connected)
        self.worker_finished.connect(self._on_worker_finished)
        self.worker_position.connect(self._set_position)
        self.worker_current.connect(self._set_current)
        self.worker_position_zeroed.connect(self._on_position_zeroed)
        self.worker_release_position_captured.connect(
            self._on_release_position_captured
        )
        self.auto_test_result.connect(self._on_auto_test_result)
        self.repeatability_log.connect(self._append_repeatability_log)
        self.motor_feedback.connect(self._apply_motor_feedback)
        self._set_expanded(False)
        self._refresh_controls()
        if self._auto_config_error is not None:
            self._set_status(
                f"自动测试参数无效，已使用默认值：{self._auto_config_error}"
            )
        self._service_thread = threading.Thread(
            target=self._service_loop,
            name="motor-can-service",
            daemon=True,
        )
        if self._communication_enabled:
            self._service_thread.start()

    def _build_ui(self, settings):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(9, 12, 9, 8)
        outer.setSpacing(6)

        self.content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        def add_value_with_unit(grid, input_widget, unit_text, row, column):
            value_layout = QtWidgets.QHBoxLayout()
            value_layout.setContentsMargins(0, 0, 0, 0)
            value_layout.setSpacing(5)
            input_widget.setFixedWidth(105)
            value_layout.addWidget(input_widget)
            value_layout.addWidget(QtWidgets.QLabel(unit_text))
            value_layout.addStretch(1)
            grid.addLayout(value_layout, row, column)

        connection_row = QtWidgets.QHBoxLayout()
        self.can_status_label = QtWidgets.QLabel("CAN 未连接")
        self.can_status_label.setObjectName("MotorStatus")
        self.can_status_label.setMinimumWidth(0)
        self.can_status_label.setWordWrap(True)
        self.can_status_label.setToolTip("PCAN_USBBUS1 · 500 kbps · 电机节点 1")
        connection_row.addWidget(self.can_status_label, 1)
        self.can_log_status_label = QtWidgets.QLabel(
            "CAN日志：实时滚动（每个 5 MiB，最多 10 个）"
        )
        self.can_log_status_label.setStyleSheet("color:#9fc3d9;")
        connection_row.addWidget(self.can_log_status_label)
        self.connect_button = QtWidgets.QPushButton("连接 CAN")
        self.connect_button.setObjectName("MotorUtilityButton")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_row.addWidget(self.connect_button)
        layout.addLayout(connection_row)

        parameter_grid = QtWidgets.QGridLayout()
        parameter_grid.setHorizontalSpacing(6)
        parameter_grid.setVerticalSpacing(5)

        self.move_count_units_input = QtWidgets.QSpinBox()
        self.move_count_units_input.setRange(1, 100000)
        self.move_count_units_input.setValue(int(settings["move_count_units"]))
        self.move_count_units_input.setToolTip("每档对应电机轴旋转约 28.8°")
        parameter_grid.addWidget(QtWidgets.QLabel("移动档位"), 0, 0)
        add_value_with_unit(parameter_grid, self.move_count_units_input, "档", 0, 1)

        self.speed_input = QtWidgets.QSpinBox()
        self.speed_input.setRange(
            1,
            counts_per_second_to_speed_level(
                rpm_to_counts_per_second(MOTOR_UI_MAX_RPM)
            ),
        )
        self.speed_input.setValue(int(settings["speed_level"]))
        self.speed_input.setToolTip(
            "每档约等于 4.8 RPM；发送CAN命令时自动换算为驱动器要求的RPM"
        )
        parameter_grid.addWidget(QtWidgets.QLabel("速度档位"), 0, 2)
        add_value_with_unit(parameter_grid, self.speed_input, "档", 0, 3)

        self.wait_input = QtWidgets.QDoubleSpinBox()
        self.wait_input.setRange(0.0, 3600.0)
        self.wait_input.setDecimals(2)
        self.wait_input.setValue(float(settings["wait_seconds"]))
        parameter_grid.addWidget(QtWidgets.QLabel("到位等待"), 1, 0)
        add_value_with_unit(parameter_grid, self.wait_input, "s", 1, 1)

        self.repeat_input = QtWidgets.QSpinBox()
        self.repeat_input.setRange(1, 100000)
        self.repeat_input.setValue(int(settings["repeat_count"]))
        parameter_grid.addWidget(QtWidgets.QLabel("往复次数"), 1, 2)
        add_value_with_unit(parameter_grid, self.repeat_input, "次", 1, 3)

        self.acceleration_input = QtWidgets.QSpinBox()
        self.acceleration_input.setRange(0, 200)
        self.acceleration_input.setValue(int(settings["acceleration"]))
        self.acceleration_input.setToolTip(
            "数值越大加减速越快；0 表示直接启动"
        )
        parameter_grid.addWidget(QtWidgets.QLabel("加减速度"), 2, 0)
        add_value_with_unit(parameter_grid, self.acceleration_input, "r/s²", 2, 1)
        parameter_grid.addWidget(QtWidgets.QLabel("控制模式"), 2, 2)
        parameter_grid.addWidget(QtWidgets.QLabel("闭环相对位置"), 2, 3)
        layout.addLayout(parameter_grid)

        option_row = QtWidgets.QHBoxLayout()
        self.position_label = QtWidgets.QLabel("位置 --")
        self.position_label.setObjectName("MotorPosition")
        option_row.addWidget(self.position_label)
        self.current_label = QtWidgets.QLabel("瞬时相电流 -- mA")
        self.current_label.setObjectName("MotorPosition")
        option_row.addWidget(self.current_label)
        option_row.addStretch(1)
        self.read_position_button = QtWidgets.QPushButton("读取位置")
        self.read_position_button.setObjectName("MotorUtilityButton")
        self.read_position_button.setToolTip("读取实时位置和相电流")
        self.read_position_button.clicked.connect(self.read_motor_feedback)
        option_row.addWidget(self.read_position_button)
        self.zero_position_button = QtWidgets.QPushButton("当前位置置零")
        self.zero_position_button.setObjectName("MotorZeroButton")
        self.zero_position_button.setToolTip("向驱动器发送当前位置角度清零指令")
        self.zero_position_button.clicked.connect(self.zero_motor_position)
        option_row.addWidget(self.zero_position_button)
        self.set_release_position_button = QtWidgets.QPushButton("设置释放位置")
        self.set_release_position_button.setObjectName("MotorUtilityButton")
        self.set_release_position_button.setToolTip(
            "把电机当前实时位置保存为所有压力运动共用的无压力终点和起点"
        )
        self.set_release_position_button.clicked.connect(
            self.capture_release_position
        )
        option_row.addWidget(self.set_release_position_button)
        self.press_first_checkbox = QtWidgets.QCheckBox("往复时先下压")
        self.press_first_checkbox.setChecked(bool(settings["press_first"]))
        option_row.addWidget(self.press_first_checkbox)
        layout.addLayout(option_row)

        self.release_position_label = QtWidgets.QLabel()
        self.release_position_label.setWordWrap(True)
        layout.addWidget(self.release_position_label)
        self._update_release_position_label()

        action_row = QtWidgets.QHBoxLayout()
        self.press_button = QtWidgets.QPushButton("下压")
        self.press_button.setObjectName("MotorPressButton")
        self.press_button.setToolTip("按移动档位下压")
        self.press_button.clicked.connect(lambda: self.start_move(-1))
        self.lift_button = QtWidgets.QPushButton("抬升")
        self.lift_button.setObjectName("MotorLiftButton")
        self.lift_button.setToolTip("按移动档位抬升")
        self.lift_button.clicked.connect(lambda: self.start_move(1))
        self.repeat_button = QtWidgets.QPushButton("开始往复")
        self.repeat_button.setObjectName("MotorRepeatButton")
        self.repeat_button.clicked.connect(self.start_repeat)
        self.stop_button = QtWidgets.QPushButton("停止")
        self.stop_button.setObjectName("MotorStopButton")
        self.stop_button.clicked.connect(self.request_stop)
        action_row.addWidget(self.press_button)
        action_row.addWidget(self.lift_button)
        action_row.addWidget(self.repeat_button)
        action_row.addWidget(self.stop_button)
        layout.addLayout(action_row)

        self.auto_test_result_label = QtWidgets.QLabel("重复精度测试：尚未开始")
        self.auto_test_result_label.setObjectName("MotorAutoTestResult")
        self.auto_test_result_label.setWordWrap(True)
        self.auto_test_result_label.setToolTip(
            f"参数文件 {self._settings_path}"
        )
        layout.addWidget(self.auto_test_result_label)

        self.repeatability_group = QtWidgets.QGroupBox("重复精度测试")
        repeatability_layout = QtWidgets.QVBoxLayout(self.repeatability_group)
        repeatability_layout.setContentsMargins(8, 8, 8, 8)
        repeatability_layout.setSpacing(5)

        repeatability_parameter_grid = QtWidgets.QGridLayout()
        repeatability_parameter_grid.setHorizontalSpacing(6)
        repeatability_parameter_grid.setVerticalSpacing(5)
        repeatability_parameter_grid.addWidget(QtWidgets.QLabel("测试档位"), 0, 0)
        self.repeatability_count_units_input = QtWidgets.QSpinBox()
        self.repeatability_count_units_input.setRange(1, 100000)
        self.repeatability_count_units_input.setValue(
            int(settings["repeatability_count_units"])
        )
        self.repeatability_count_units_input.setToolTip(
            "每档对应电机轴旋转约 28.8°"
        )
        add_value_with_unit(
            repeatability_parameter_grid,
            self.repeatability_count_units_input,
            "档",
            0,
            1,
        )

        repeatability_parameter_grid.addWidget(
            QtWidgets.QLabel("FLOATING通道数"),
            0,
            2,
        )
        self.rapid_floating_channel_count_input = QtWidgets.QSpinBox()
        self.rapid_floating_channel_count_input.setRange(1, 4)
        self.rapid_floating_channel_count_input.setValue(
            int(settings["rapid_floating_channel_count"])
        )
        self.rapid_floating_channel_count_input.setToolTip(
            "CH4～CH7中至少有多少个通道连续进入FLOATING，才完成本轮采样"
        )
        add_value_with_unit(
            repeatability_parameter_grid,
            self.rapid_floating_channel_count_input,
            "个",
            0,
            3,
        )

        repeatability_parameter_grid.addWidget(QtWidgets.QLabel("正式轮次"), 0, 4)
        self.rapid_formal_rounds_input = QtWidgets.QSpinBox()
        self.rapid_formal_rounds_input.setRange(1, 1000)
        self.rapid_formal_rounds_input.setValue(
            int(settings["rapid_formal_rounds"])
        )
        self.rapid_formal_rounds_input.setToolTip(
            "预压稳定后需要采集的正式有效结果数量"
        )
        add_value_with_unit(
            repeatability_parameter_grid,
            self.rapid_formal_rounds_input,
            "次",
            0,
            5,
        )

        repeatability_parameter_grid.addWidget(
            QtWidgets.QLabel("预压相对极差"),
            1,
            0,
        )
        self.rapid_precondition_relative_span_input = QtWidgets.QDoubleSpinBox()
        self.rapid_precondition_relative_span_input.setRange(0.0, 100.0)
        self.rapid_precondition_relative_span_input.setDecimals(3)
        self.rapid_precondition_relative_span_input.setSingleStep(0.1)
        self.rapid_precondition_relative_span_input.setValue(
            float(settings["rapid_precondition_relative_span_percent_max"])
        )
        self.rapid_precondition_relative_span_input.setToolTip(
            "最近3个同停点有效预压Signal和的相对极差不高于此值时进入正式测试"
        )
        add_value_with_unit(
            repeatability_parameter_grid,
            self.rapid_precondition_relative_span_input,
            "%",
            1,
            1,
        )

        repeatability_parameter_grid.addWidget(
            QtWidgets.QLabel("预压最大轮次"),
            1,
            2,
        )
        self.rapid_precondition_max_rounds_input = QtWidgets.QSpinBox()
        self.rapid_precondition_max_rounds_input.setRange(3, 1000)
        self.rapid_precondition_max_rounds_input.setValue(
            int(settings["rapid_precondition_max_rounds"])
        )
        self.rapid_precondition_max_rounds_input.setToolTip(
            "正式测试前允许执行的最大预压轮次；至少为3轮"
        )
        add_value_with_unit(
            repeatability_parameter_grid,
            self.rapid_precondition_max_rounds_input,
            "次",
            1,
            3,
        )

        repeatability_parameter_grid.addWidget(
            QtWidgets.QLabel("FLOAT超时"),
            1,
            4,
        )
        self.rapid_floating_timeout_input = QtWidgets.QDoubleSpinBox()
        self.rapid_floating_timeout_input.setRange(0.1, 3600.0)
        self.rapid_floating_timeout_input.setDecimals(1)
        self.rapid_floating_timeout_input.setSingleStep(0.5)
        self.rapid_floating_timeout_input.setValue(
            float(settings["rapid_floating_timeout_seconds"])
        )
        self.rapid_floating_timeout_input.setToolTip(
            "单轮等待足够通道进入FLOATING的最长时间；超时后抬升并继续下一轮"
        )
        add_value_with_unit(
            repeatability_parameter_grid,
            self.rapid_floating_timeout_input,
            "s",
            1,
            5,
        )

        repeatability_parameter_grid.addWidget(
            QtWidgets.QLabel("FLOAT后保持"),
            2,
            0,
        )
        self.rapid_floating_hold_input = QtWidgets.QDoubleSpinBox()
        self.rapid_floating_hold_input.setRange(0.0, 3600.0)
        self.rapid_floating_hold_input.setDecimals(1)
        self.rapid_floating_hold_input.setSingleStep(0.5)
        self.rapid_floating_hold_input.setValue(
            float(settings["rapid_floating_hold_seconds"])
        )
        self.rapid_floating_hold_input.setToolTip(
            "达到设定FLOATING通道数并记录Signal后，继续保持压力多久再抬升"
        )
        add_value_with_unit(
            repeatability_parameter_grid,
            self.rapid_floating_hold_input,
            "s",
            2,
            1,
        )
        repeatability_parameter_grid.setColumnStretch(1, 1)
        repeatability_parameter_grid.setColumnStretch(3, 1)
        repeatability_parameter_grid.setColumnStretch(5, 1)
        repeatability_layout.addLayout(repeatability_parameter_grid)

        repeatability_button_row = QtWidgets.QHBoxLayout()
        self.start_repeatability_button = QtWidgets.QPushButton("快速连续测试")
        self.start_repeatability_button.setObjectName("MotorRapidButton")
        self.start_repeatability_button.setToolTip(
            "使用上方快速测试参数；达到设定的FLOATING通道数后记录四通道Signal和，"
            "抬升后只短暂等待即开始下一轮；点击时立即保存参数"
        )
        self.start_repeatability_button.clicked.connect(
            lambda: self.start_repeatability_test("rapid")
        )
        self.start_full_recovery_test_button = QtWidgets.QPushButton("完全恢复测试")
        self.start_full_recovery_test_button.setObjectName("MotorRecoveryButton")
        self.start_full_recovery_test_button.setToolTip(
            "按输入counts重复10次，每轮抬升后等待CH4～CH7全部进入IDLE并再稳定1秒"
        )
        self.start_full_recovery_test_button.clicked.connect(
            lambda: self.start_repeatability_test("full")
        )
        self.stop_repeatability_button = QtWidgets.QPushButton("停止测试")
        self.stop_repeatability_button.setObjectName("MotorStopButton")
        self.stop_repeatability_button.clicked.connect(self.request_stop)
        repeatability_button_row.addWidget(self.start_repeatability_button)
        repeatability_button_row.addWidget(self.start_full_recovery_test_button)
        repeatability_button_row.addWidget(self.stop_repeatability_button)
        repeatability_layout.addLayout(repeatability_button_row)

        self.repeatability_log_view = QtWidgets.QPlainTextEdit()
        self.repeatability_log_view.setObjectName("RepeatabilityLog")
        self.repeatability_log_view.setReadOnly(True)
        self.repeatability_log_view.setMaximumBlockCount(500)
        self.repeatability_log_view.setMinimumHeight(125)
        self.repeatability_log_view.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        self.repeatability_log_view.setPlaceholderText(
            "测试日志将在这里显示每次CH4～CH7的Signal值、总和及最终极差。"
        )
        repeatability_layout.addWidget(self.repeatability_log_view, 1)
        layout.addWidget(self.repeatability_group, 1)

        outer.addWidget(self.content)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Preferred,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        self.setStyleSheet("""
            QGroupBox#MotorControl {
                border: 1px solid #35506b;
                border-radius: 7px;
                margin-top: 7px;
                padding-top: 5px;
                font-weight: 700;
                background: #0b1119;
            }
            QGroupBox#MotorControl::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #91dff9;
            }
            QLabel#MotorStatus { color: #aab8c8; font-weight: 600; }
            QLabel#MotorPosition {
                color: #57d5ff;
                font-family: Consolas, "Microsoft YaHei";
                font-size: 18px;
                font-weight: 700;
            }
            QLabel#MotorAutoTestResult { color: #f4d35e; font-weight: 600; }
            QPlainTextEdit#RepeatabilityLog {
                background: #070b10;
                color: #b9e6ff;
                border: 1px solid #31475d;
                font-family: Consolas, "Microsoft YaHei";
                font-size: 14px;
            }
            QGroupBox#MotorControl QPushButton {
                min-height: 28px;
                border: 1px solid #9eacb8;
                border-radius: 5px;
                padding: 2px 12px;
                color: #17212b;
                font-weight: 700;
            }
            QPushButton#MotorUtilityButton { background: #d8e1eb; }
            QPushButton#MotorUtilityButton:hover { background: #e7edf3; }
            QPushButton#MotorPressButton { background: #efc8b7; border-color: #c89b88; }
            QPushButton#MotorPressButton:hover { background: #f6d8ca; }
            QPushButton#MotorLiftButton { background: #bfdcd3; border-color: #8fb8ac; }
            QPushButton#MotorLiftButton:hover { background: #d2e9e2; }
            QPushButton#MotorRepeatButton { background: #c9d6e9; border-color: #98aac5; }
            QPushButton#MotorRepeatButton:hover { background: #dce5f2; }
            QPushButton#MotorZeroButton { background: #ead9ac; border-color: #c5ae71; }
            QPushButton#MotorZeroButton:hover { background: #f3e7c7; }
            QPushButton#MotorRangeButton { background: #d6c7e5; border-color: #aa94c0; }
            QPushButton#MotorRangeButton:hover { background: #e5dcef; }
            QPushButton#MotorAutoButton { background: #c3dec0; border-color: #91b58d; }
            QPushButton#MotorAutoButton:hover { background: #d7ead4; }
            QPushButton#MotorRapidButton { background: #bddce6; border-color: #8ab4c2; }
            QPushButton#MotorRapidButton:hover { background: #d3eaf1; }
            QPushButton#MotorRecoveryButton { background: #d0cae7; border-color: #a29ac2; }
            QPushButton#MotorRecoveryButton:hover { background: #e1ddef; }
            QPushButton#MotorStopButton { background: #edbbb6; border-color: #c68780; }
            QPushButton#MotorStopButton:hover { background: #f5cfcb; }
            QGroupBox#MotorControl QPushButton:pressed {
                border-color: #62717f;
                color: #0d151c;
            }
            QGroupBox#MotorControl QPushButton:disabled {
                background: #59636d;
                border-color: #69747f;
                color: #aab2ba;
            }
        """)

        for widget in (
            self.move_count_units_input,
            self.speed_input,
            self.acceleration_input,
            self.wait_input,
            self.repeat_input,
            self.press_first_checkbox,
            self.repeatability_count_units_input,
            self.rapid_floating_channel_count_input,
            self.rapid_precondition_relative_span_input,
            self.rapid_formal_rounds_input,
            self.rapid_precondition_max_rounds_input,
            self.rapid_floating_timeout_input,
            self.rapid_floating_hold_input,
        ):
            if hasattr(widget, "editingFinished"):
                widget.editingFinished.connect(self._save_settings_if_idle)
            else:
                widget.stateChanged.connect(self._save_settings_if_idle)

    def _configure_numeric_inputs(self):
        button_symbols = getattr(QtWidgets.QAbstractSpinBox, "NoButtons", None)
        if button_symbols is None:
            button_symbols = QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons
        for widget in self.findChildren(QtWidgets.QAbstractSpinBox):
            widget.setButtonSymbols(button_symbols)
            widget.setKeyboardTracking(False)

    def _save_settings_if_idle(self, *_args):
        if self._busy:
            return
        self._save_timer.stop()
        self._save_settings()

    def _new_can_log_path_locked(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        log_directory = can_log_directory()
        path = log_directory / f"can_log_{timestamp}.log"
        suffix = 1
        while path.exists():
            path = log_directory / f"can_log_{timestamp}_{suffix:03d}.log"
            suffix += 1
        return path

    def _prune_can_logs_locked(self):
        log_directory = can_log_directory()
        try:
            candidates = list(log_directory.glob("can_log_*.log"))
            candidates.sort(key=lambda path: (path.stat().st_mtime_ns, path.name))
        except OSError as exc:
            self._can_log_open_error = exc
            return
        while len(candidates) > CAN_LOG_MAX_FILES:
            oldest = candidates.pop(0)
            if oldest == self._can_log_path:
                candidates.append(oldest)
                continue
            try:
                oldest.unlink()
            except OSError as exc:
                self._can_log_open_error = exc
                break

    def _open_can_log_locked(self):
        log_directory = can_log_directory()
        log_directory.mkdir(parents=True, exist_ok=True)
        path = self._new_can_log_path_locked()
        handle = path.open("w", encoding="utf-8", buffering=1, newline="")
        self._can_log_handle = handle
        self._can_log_path = path
        self._can_log_size_bytes = 0
        self._can_log_open_error = None
        self._prune_can_logs_locked()
        return path

    def _open_can_log(self):
        try:
            with self._can_log_lock:
                if self._can_log_handle is not None:
                    return self._can_log_path
                return self._open_can_log_locked()
        except OSError as exc:
            self._can_log_open_error = exc
            return None

    def _rotate_can_log_locked(self):
        if self._can_log_handle is not None:
            self._can_log_handle.flush()
            self._can_log_handle.close()
        self._can_log_handle = None
        return self._open_can_log_locked()

    def _close_can_log(self):
        with self._can_log_lock:
            handle = self._can_log_handle
            path = self._can_log_path
            self._can_log_handle = None
            self._can_log_size_bytes = 0
            if handle is not None:
                try:
                    handle.flush()
                    handle.close()
                except OSError:
                    pass
        return path

    def _record_can_event(self, message):
        timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        thread_name = threading.current_thread().name
        line = f"[{timestamp}] [{thread_name}] {message}\n"
        encoded_size = len(line.encode("utf-8"))
        with self._can_log_lock:
            if self._can_log_handle is not None:
                try:
                    if (
                        self._can_log_size_bytes > 0
                        and self._can_log_size_bytes + encoded_size
                        > CAN_LOG_MAX_BYTES
                    ):
                        self._rotate_can_log_locked()
                    self._can_log_handle.write(line)
                    self._can_log_handle.flush()
                    self._can_log_size_bytes += encoded_size
                except (OSError, ValueError) as exc:
                    self._can_log_open_error = exc
                    try:
                        if self._can_log_handle is not None:
                            self._can_log_handle.close()
                    except OSError:
                        pass
                    self._can_log_handle = None

    def record_motion_diagnostic(self, event, **values):
        """向CAN日志写入便于比较双向运动的结构化诊断数据。"""
        payload = {"event": str(event)}
        payload.update(values)
        self._record_can_event(
            "MOTION "
            + json.dumps(
                payload,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
        )

    def _load_settings(self):
        settings = dict(DEFAULT_SETTINGS)
        try:
            if self._settings_path.exists():
                loaded = json.loads(self._settings_path.read_text(encoding="utf-8"))
                for key in settings:
                    if key in loaded:
                        settings[key] = loaded[key]
                if "speed_level" not in loaded:
                    if "speed_counts_per_second" in loaded:
                        settings["speed_level"] = counts_per_second_to_speed_level(
                            int(loaded["speed_counts_per_second"])
                        )
                    elif "speed" in loaded:
                        settings["speed_level"] = counts_per_second_to_speed_level(
                            rpm_to_counts_per_second(int(loaded["speed"]))
                        )
        except (OSError, ValueError, TypeError):
            pass
        return settings

    @Slot()
    def _schedule_save(self, *_args):
        self._save_timer.start()

    @Slot()
    def _save_settings(self):
        if not self._communication_enabled:
            return
        settings = {
            "move_count_units": self.move_count_units_input.value(),
            "speed_level": self.speed_input.value(),
            "acceleration": self.acceleration_input.value(),
            "wait_seconds": self.wait_input.value(),
            "repeat_count": self.repeat_input.value(),
            "press_first": self.press_first_checkbox.isChecked(),
            "auto_zero_step_deg": self._auto_config.zero_step_deg,
            "auto_max_step_deg": self._auto_config.max_step_deg,
            "auto_contact_signal_threshold": self._auto_config.contact_signal_threshold,
            "auto_raw_limit": self._auto_config.raw_limit,
            "auto_ready_lift_deg": self._auto_config.ready_lift_deg,
            "auto_sensor_settle_seconds": self._auto_config.sensor_settle_seconds,
            "auto_sensor_frame_timeout_seconds": self._auto_config.sensor_frame_timeout_seconds,
            "auto_motor_position_timeout_seconds": self._auto_config.motor_position_timeout_seconds,
            "auto_max_search_travel_deg": self._auto_config.max_search_travel_deg,
            "auto_confirmation_frames": self._auto_config.confirmation_frames,
            "random_test_rounds": self._random_test_settings["rounds"],
            "random_test_min_angle_deg": self._random_test_settings["min_angle_deg"],
            "random_test_max_angle_deg": self._random_test_settings["max_angle_deg"],
            "random_test_hold_min_seconds": self._random_test_settings["hold_min_seconds"],
            "random_test_hold_max_seconds": self._random_test_settings["hold_max_seconds"],
            "random_test_release_observe_seconds": self._random_test_settings["release_observe_seconds"],
            "repeatability_count_units": self.repeatability_count_units_input.value(),
            "rapid_floating_channel_count": self.rapid_floating_channel_count_input.value(),
            "rapid_precondition_relative_span_percent_max": self.rapid_precondition_relative_span_input.value(),
            "rapid_formal_rounds": self.rapid_formal_rounds_input.value(),
            "rapid_precondition_max_rounds": self.rapid_precondition_max_rounds_input.value(),
            "rapid_floating_timeout_seconds": self.rapid_floating_timeout_input.value(),
            "rapid_floating_hold_seconds": self.rapid_floating_hold_input.value(),
            "release_position_counts": self._release_position_counts,
            "release_position_saved_at": self._release_position_saved_at,
        }
        try:
            self._settings_path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = self._settings_path.with_suffix(".json.tmp")
            temporary_path.write_text(
                json.dumps(settings, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary_path.replace(self._settings_path)
        except OSError as exc:
            self._set_status(f"参数保存失败: {exc}")

    @Slot(bool)
    def _set_expanded(self, expanded):
        self.content.setVisible(expanded)
        self.setMaximumHeight(self.content.maximumHeight() if expanded else 30)
        self.setMinimumHeight(0 if not expanded else 225)

    @Slot()
    def toggle_connection(self):
        if self._busy:
            self._record_can_event("UI connect/disconnect ignored: motor worker is busy")
            return
        if self.motor is None:
            self.connect_can()
        else:
            self._record_can_event("UI disconnect requested")
            self._start_worker(self._disconnect_motor, "正在断开 CAN...")

    @Slot()
    def connect_can(self):
        if not self._communication_enabled:
            self._set_status("当前未选择旧电机 PD42S1；请修改 motor_connection_config.json 后重启")
            return
        if self._closing:
            self._record_can_event("CAN connect ignored: application is closing")
            return
        if self.motor is not None:
            self._record_can_event("CAN connect skipped: already connected")
            return
        if self._busy:
            self._record_can_event("CAN connect ignored: motor worker is busy")
            return
        self._record_can_event("CAN connect requested")
        self._start_worker(self._connect_motor, "正在连接 CAN...")

    def _connect_motor(self):
        if not self._communication_enabled:
            raise RuntimeError("旧电机通信未启用")
        motor = PD42S1Motor(PD42S1Config(), event_logger=self._record_can_event)
        try:
            motor.open()
            motor.enable_motor()
        except Exception:
            motor.close()
            raise
        self.motor = motor
        self._reset_feedback_estimator()
        self.worker_connected.emit(True)
        self.worker_status.emit("CAN 已连接 · 电机已使能 · 闭环相对位置")
        try:
            self._poll_motor_feedback()
        except Exception as exc:
            self._record_can_event(f"Initial motor feedback read failed: {exc!r}")
            self.worker_status.emit(f"CAN 已连接，但首次读取电机反馈失败：{exc}")

    def _disconnect_motor(self):
        self._stop_event.set()
        motor = self.motor
        self.motor = None
        if motor is not None:
            motor.close()
        self._reset_feedback_estimator()
        self.worker_connected.emit(False)
        self.worker_status.emit("CAN 未连接")

    def _selected_speed_counts_per_second(self):
        return speed_level_to_counts_per_second(self.speed_input.value())

    def _selected_speed_rpm(self):
        return counts_per_second_to_rpm(
            self._selected_speed_counts_per_second()
        )

    @Slot()
    def read_motor_feedback(self):
        if self.motor is None:
            self._set_status("读取失败：CAN 未连接")
            return
        if self._busy:
            self._set_status("读取失败：当前电机操作尚未完成")
            return

        def task():
            snapshot = self._poll_motor_feedback()
            counts = snapshot["position_counts"]
            current_ma = snapshot["phase_current_ma"]
            running_state = snapshot["running_state"]
            in_position = self.motor.read_in_position()
            state_name = MOTOR_RUNNING_STATE_NAMES.get(
                running_state,
                f"未知状态 {running_state}",
            )
            in_position_text = "已到位" if in_position else "未到位"
            self.worker_status.emit(
                f"读取完成：{counts} counts · 相电流 {current_ma} mA · "
                f"{state_name} · {in_position_text}"
            )

        self._start_worker(task, "正在读取电机位置和相电流...")

    @Slot()
    def capture_release_position(self):
        """把驱动器当前实时位置保存为压力运动共用的释放基准。"""
        if self.motor is None:
            self._set_status("设置释放位置失败：CAN 未连接")
            return
        if self._busy:
            self._set_status("设置释放位置失败：当前电机操作尚未完成")
            return

        feedback = self.latest_motor_feedback()
        if feedback is None or feedback.get("position_counts") is None:
            self._set_status(
                "设置释放位置失败：尚未取得电机实时位置，请稍后再试"
            )
            return
        preview_counts = int(feedback["position_counts"])
        previous_text = (
            "尚未设置"
            if self._release_position_counts is None
            else f"{self._release_position_counts} counts"
        )
        standard_button = getattr(
            QtWidgets.QMessageBox,
            "StandardButton",
            QtWidgets.QMessageBox,
        )
        yes_button = getattr(QtWidgets.QMessageBox, "Yes", standard_button.Yes)
        no_button = getattr(QtWidgets.QMessageBox, "No", standard_button.No)
        answer = QtWidgets.QMessageBox.question(
            self.window(),
            "确认设置释放位置",
            f"即将把当前电机位置 {preview_counts} counts 设置为固定释放位置。\n\n"
            f"原释放位置：{previous_text}\n\n"
            "请确认：\n"
            "1. 压头已经完全离开被测物；\n"
            "2. 压力计当前没有受力；\n"
            "3. 当前机械位置适合作为所有自动/手动释放的最终位置，"
            "以及下一次下压的起点。\n\n"
            "继续后将覆盖原释放位置，并用于自动测试、位置扫描和查表定位。",
            yes_button | no_button,
            no_button,
        )
        if answer != yes_button:
            self._record_can_event(
                f"UI release-position capture cancelled preview={preview_counts}"
            )
            self._set_status("已取消设置释放位置，原设置保持不变")
            return
        self._record_can_event(
            f"UI release-position capture confirmed preview={preview_counts}"
        )

        def task():
            if self.motor.read_running_state() == 2:
                raise RuntimeError("电机仍在运行，不能设置释放位置")
            counts = int(self.motor.read_position_counts())
            position_change = abs(counts - preview_counts)
            if position_change > RELEASE_POSITION_CONFIRM_TOLERANCE_COUNTS:
                raise RuntimeError(
                    f"确认后电机位置变化了 {position_change} counts，"
                    "为防止误设，本次未保存；请检查位置后重新设置"
                )
            self.worker_release_position_captured.emit(
                {
                    "position_counts": counts,
                    "saved_at": datetime.now().astimezone().isoformat(
                        timespec="seconds"
                    ),
                }
            )

        self._start_worker(task, "正在读取并保存释放位置...")

    def release_position_snapshot(self):
        """返回PC端保存的释放位置及其在当前CAN连接中的确认状态。"""
        return {
            "position_counts": self._release_position_counts,
            "saved_at": self._release_position_saved_at,
            "confirmed": bool(self._release_position_confirmed),
        }

    def _update_release_position_label(self):
        if self._release_position_counts is None:
            text = "释放位置：尚未设置；压力自动运动已禁用"
            color = "#ff7b72"
        elif self._release_position_confirmed:
            text = (
                f"释放位置：{self._release_position_counts} counts（当前连接已确认）"
            )
            color = "#5be39b"
        else:
            text = (
                f"释放位置：PC已保存 {self._release_position_counts} counts，"
                "CAN重新连接后待重新设置"
            )
            color = "#ffcc66"
        self.release_position_label.setText(text)
        self.release_position_label.setStyleSheet(
            f"color:{color};font-weight:700;"
        )

    @Slot(object)
    def _on_release_position_captured(self, result):
        self._release_position_counts = int(result["position_counts"])
        self._release_position_saved_at = str(result["saved_at"])
        self._release_position_confirmed = True
        self._update_release_position_label()
        self._save_settings()
        snapshot = self.release_position_snapshot()
        self.release_position_changed.emit(snapshot)
        self._set_status(
            f"释放位置已设置为 {self._release_position_counts} counts；"
            "此位置将作为压力运动的固定终点和起点"
        )
        self._refresh_controls()

    @Slot()
    def zero_motor_position(self):
        if self.motor is None:
            self._set_status("置零失败：CAN 未连接")
            return
        if self._busy:
            self._set_status("置零失败：当前电机操作尚未完成")
            return

        def task():
            if self.motor.read_running_state() == 2:
                raise RuntimeError("电机仍在运行，不能将当前位置置零")
            self.motor.zero_position()
            snapshot = self._poll_motor_feedback()
            counts = snapshot["position_counts"]
            self.worker_position_zeroed.emit()
            self.worker_status.emit(f"驱动器当前位置已置零：{counts} counts")

        self._start_worker(task, "正在将驱动器当前位置置零...")

    @Slot()
    def start_move(self, direction):
        action = "下压" if direction < 0 else "抬升"
        if self.motor is None:
            self._record_can_event(f"UI {action} ignored: CAN is not connected")
            return
        if self._busy:
            self._record_can_event(f"UI {action} ignored: motor worker is busy")
            return
        move_counts = self.move_count_units_input.value() * POSITION_COUNT_UNIT
        signed_counts = move_counts * direction
        move_level = self.move_count_units_input.value()
        speed_level = self.speed_input.value()
        speed_counts_per_second = self._selected_speed_counts_per_second()
        speed_rpm = self._selected_speed_rpm()
        acceleration = self.acceleration_input.value()
        self._stop_event.clear()
        self._record_can_event(
            f"UI {action} accepted level={move_level} counts={move_counts} "
            f"speed_level={speed_level} speed={speed_counts_per_second} counts/s "
            f"protocol_speed={speed_rpm} RPM acceleration={acceleration}"
        )

        def task():
            start_counts = self.motor.read_position_counts()
            target_counts = start_counts + signed_counts
            self.motor.move_relative_counts(
                signed_counts,
                speed_rpm=speed_rpm,
                accel=acceleration,
                retry_cancelled=self._stop_event.is_set,
            )
            timeout_seconds = max(
                8.0,
                move_counts / max(1, speed_counts_per_second) * 3.0 + 2.0,
            )
            if not self._wait_until_position(
                start_counts,
                target_counts,
                timeout_seconds=timeout_seconds,
            ):
                self._brake_after_stop()
                return
            self._poll_motor_feedback()
            self.worker_status.emit(
                f"{action}已到位：移动档位 {move_level} · "
                f"速度档位 {speed_level} · 加减速度 {acceleration} r/s²"
            )

        self._start_worker(task, f"正在{action}...")

    @Slot()
    def start_repeat(self):
        if self.motor is None:
            self._record_can_event("UI repeat ignored: CAN is not connected")
            return
        if self._busy:
            self._record_can_event("UI repeat ignored: motor worker is busy")
            return

        move_counts = self.move_count_units_input.value() * POSITION_COUNT_UNIT
        move_level = self.move_count_units_input.value()
        speed_level = self.speed_input.value()
        speed_counts_per_second = self._selected_speed_counts_per_second()
        speed_rpm = self._selected_speed_rpm()
        acceleration = self.acceleration_input.value()
        move_timeout_seconds = max(
            8.0,
            move_counts / max(1, speed_counts_per_second) * 3.0 + 2.0,
        )
        wait_seconds = self.wait_input.value()
        repeat_count = self.repeat_input.value()
        directions = (-1, 1) if self.press_first_checkbox.isChecked() else (1, -1)
        self._stop_event.clear()
        self._repeating = True
        self._record_can_event(
            f"UI repeat accepted level={move_level} counts={move_counts} "
            f"speed_level={speed_level} speed={speed_counts_per_second} counts/s "
            f"protocol_speed={speed_rpm} RPM acceleration={acceleration} "
            f"wait={wait_seconds:g} s count={repeat_count}"
        )

        def task():
            try:
                for cycle in range(1, repeat_count + 1):
                    for direction in directions:
                        if self._stop_event.is_set():
                            self._brake_after_stop()
                            return
                        action = "下压" if direction < 0 else "抬升"
                        self.worker_status.emit(f"往复 {cycle}/{repeat_count}: 正在{action}")
                        start_counts = self.motor.read_position_counts()
                        target_counts = start_counts + direction * move_counts
                        self.motor.move_relative_counts(
                            direction * move_counts,
                            speed_rpm=speed_rpm,
                            accel=acceleration,
                            retry_cancelled=self._stop_event.is_set,
                        )
                        if not self._wait_until_position(
                            start_counts,
                            target_counts,
                            timeout_seconds=move_timeout_seconds,
                        ):
                            self._brake_after_stop()
                            return
                        if self._stop_event.wait(wait_seconds):
                            self._brake_after_stop()
                            return
                self.worker_status.emit(f"往复完成: {repeat_count} 次")
            finally:
                self._repeating = False

        self._start_worker(task, "往复运动已开始")

    @Slot()
    def start_find_zero(self):
        if not self._validate_auto_test_start("寻找零点", require_raw=False):
            return
        self._stop_event.clear()
        self._auto_test_kind = "zero"
        tester = self._make_auto_tester()
        self._record_can_event(
            f"AUTO zero start step={self._auto_config.zero_step_deg:g} deg "
            f"signal_threshold={self._auto_config.contact_signal_threshold}"
        )

        def task():
            try:
                result = tester.find_zero()
                result["ok"] = True
            except MotorAutoTestCancelled as exc:
                self._brake_after_stop()
                result = {"kind": "zero", "ok": False, "cancelled": True, "error": str(exc)}
            except Exception as exc:
                recovery_error = tester.recover_to_safe_position()
                error_text = str(exc)
                if recovery_error is not None:
                    error_text += f"；返回起始位置失败：{recovery_error}"
                result = {"kind": "zero", "ok": False, "error": error_text}
                self._record_can_event(f"AUTO zero failed: {exc!r}")
            self.auto_test_result.emit(result)

        self._start_worker(task, "自动寻找零点已开始")

    @Slot()
    def start_find_max_range(self):
        if self._zero_position_counts is None:
            self._set_status("请先完成寻找零点，再寻找最大量程")
            return
        if not self._validate_auto_test_start("寻找最大量程", require_raw=True):
            return
        self._stop_event.clear()
        self._auto_test_kind = "max_range"
        zero_position_counts = self._zero_position_counts
        tester = self._make_auto_tester()
        self._record_can_event(
            f"AUTO max-range start step={self._auto_config.max_step_deg:g} deg "
            f"raw_limit={self._auto_config.raw_limit} zero_counts={zero_position_counts}"
        )

        def task():
            try:
                result = tester.find_max_range(zero_position_counts)
                result["ok"] = True
            except MotorAutoTestCancelled as exc:
                self._brake_after_stop()
                result = {
                    "kind": "max_range",
                    "ok": False,
                    "cancelled": True,
                    "error": str(exc),
                }
            except Exception as exc:
                recovery_error = tester.recover_to_safe_position()
                error_text = str(exc)
                if recovery_error is not None:
                    error_text += f"；返回准备位失败：{recovery_error}"
                result = {"kind": "max_range", "ok": False, "error": error_text}
                self._record_can_event(f"AUTO max-range failed: {exc!r}")
            self.auto_test_result.emit(result)

        self._start_worker(task, "自动寻找最大量程已开始")

    @Slot()
    def start_random_press_test(self):
        if self._zero_position_counts is None or self._maximum_press_angle_deg is None:
            self._set_status("请先完成寻找零点和寻找最大量程，再开始随机自动测试")
            return
        if not self._validate_auto_test_start("随机自动测试", require_raw=True):
            return

        maximum_angle_deg = min(
            self._maximum_press_angle_deg,
            self._random_test_settings["max_angle_deg"],
        )
        if maximum_angle_deg < self._random_test_settings["min_angle_deg"]:
            self._set_status("可用最大量程小于随机测试最小角度，无法开始")
            return

        self._stop_event.clear()
        self._auto_test_kind = "random_press"
        tester = self._make_auto_tester()
        runner = RandomPressTestRunner(
            tester=tester,
            zero_position_counts=self._zero_position_counts,
            maximum_angle_deg=maximum_angle_deg,
            output_dir=automatic_test_directory(),
            rounds=self._random_test_settings["rounds"],
            minimum_angle_deg=self._random_test_settings["min_angle_deg"],
            hold_min_seconds=self._random_test_settings["hold_min_seconds"],
            hold_max_seconds=self._random_test_settings["hold_max_seconds"],
            release_observe_seconds=self._random_test_settings["release_observe_seconds"],
        )
        self._record_can_event(
            f"AUTO random-press start rounds={self._random_test_settings['rounds']} "
            f"angle={self._random_test_settings['min_angle_deg']}..{maximum_angle_deg:g} deg "
            f"hold={self._random_test_settings['hold_min_seconds']:g}.."
            f"{self._random_test_settings['hold_max_seconds']:g} s seed={runner.random_seed}"
        )

        def task():
            try:
                result = runner.run()
                result["ok"] = True
            except MotorAutoTestCancelled as exc:
                self._brake_after_stop()
                result = {
                    "kind": "random_press",
                    "ok": False,
                    "cancelled": True,
                    "error": str(exc),
                }
            except Exception as exc:
                recovery_error = tester.recover_to_safe_position()
                error_text = str(exc)
                if recovery_error is not None:
                    error_text += f"；返回零点失败：{recovery_error}"
                result = {"kind": "random_press", "ok": False, "error": error_text}
                self._record_can_event(f"AUTO random-press failed: {exc!r}")
            self.auto_test_result.emit(result)

        self._start_worker(task, "随机自动测试已开始")

    def start_repeatability_test(self, recovery_mode="rapid"):
        if recovery_mode == "rapid":
            self._save_timer.stop()
            self._save_settings()
        if not self._validate_auto_test_start("重复精度测试", require_raw=True):
            return

        self._stop_event.clear()
        self._auto_test_kind = "repeatability"
        press_count_units = self.repeatability_count_units_input.value()
        press_counts = press_count_units * POSITION_COUNT_UNIT
        rapid_floating_channel_count = (
            self.rapid_floating_channel_count_input.value()
        )
        precondition_relative_span_percent_max = (
            self.rapid_precondition_relative_span_input.value()
        )
        precondition_max_rounds = self.rapid_precondition_max_rounds_input.value()
        floating_timeout_seconds = self.rapid_floating_timeout_input.value()
        floating_hold_seconds = self.rapid_floating_hold_input.value()
        rounds = (
            self.rapid_formal_rounds_input.value()
            if recovery_mode == "rapid"
            else 10
        )
        mode_name = "快速连续" if recovery_mode == "rapid" else "完全恢复"
        self.repeatability_log_view.clear()
        self.repeatability_log.emit(
            f"准备获取{rounds}次固定测试档位{press_count_units}的有效重复精度结果，"
            f"模式={mode_name}。"
        )
        if recovery_mode == "rapid":
            self.repeatability_log.emit(
                f"快速连续模式以CH4～CH7中至少{rapid_floating_channel_count}个通道"
                "连续进入FLOATING作为本轮稳定条件；"
                "满足后记录当帧四通道Signal和。"
            )
            self.repeatability_log.emit(
                "快速连续模式允许预压停点在目标±20 counts内逐步收敛；"
                "最近3次实际下压停点完全相同后锁定为正式参考。"
            )
            self.repeatability_log.emit(
                f"正式测试前最多预压{precondition_max_rounds}轮；"
                "最近3个有效预压Signal和的相对极差不超过"
                f"{precondition_relative_span_percent_max:.3f}%后，"
                f"开始采集{rounds}次正式结果。"
            )
            self.repeatability_log.emit(
                f"每轮最多等待FLOATING {floating_timeout_seconds:g}秒；"
                "未满足通道数则抬升并继续下一轮；"
                f"达到条件后保持{floating_hold_seconds:g}秒再抬升。"
            )
        tester = self._make_auto_tester(
            progress_callback=self._emit_repeatability_progress,
        )
        runner = RepeatabilityTestRunner(
            tester=tester,
            output_dir=automatic_test_directory(),
            rounds=rounds,
            press_counts=press_counts,
            recovery_mode=recovery_mode,
            rapid_floating_channel_count=rapid_floating_channel_count,
            precondition_relative_span_percent_max=(
                precondition_relative_span_percent_max
            ),
            precondition_max_rounds=precondition_max_rounds,
            floating_timeout_seconds=(
                floating_timeout_seconds if recovery_mode == "rapid" else 20.0
            ),
            floating_hold_seconds=(
                floating_hold_seconds if recovery_mode == "rapid" else 0.0
            ),
        )
        self._record_can_event(
            f"AUTO repeatability start rounds={rounds} counts={press_counts} "
            f"recovery_mode={recovery_mode} "
            f"floating_channels={rapid_floating_channel_count} "
            f"precondition_relative_span="
            f"{precondition_relative_span_percent_max:.6f}% "
            f"precondition_max_rounds={precondition_max_rounds}"
            f" floating_timeout_seconds={floating_timeout_seconds:.3f}"
            f" floating_hold_seconds={floating_hold_seconds:.3f}"
        )

        def task():
            try:
                result = runner.run()
                result["ok"] = True
            except MotorAutoTestCancelled as exc:
                self._brake_after_stop()
                result = {
                    "kind": "repeatability",
                    "ok": False,
                    "cancelled": True,
                    "error": str(exc),
                }
            except Exception as exc:
                recovery_error = tester.recover_to_safe_position()
                error_text = str(exc)
                if recovery_error is not None:
                    error_text += f"；返回测试起点失败：{recovery_error}"
                result = {
                    "kind": "repeatability",
                    "ok": False,
                    "error": error_text,
                }
                self._record_can_event(f"AUTO repeatability failed: {exc!r}")
            self.auto_test_result.emit(result)

        self._start_worker(task, "重复精度测试已开始")

    def _validate_auto_test_start(self, action, require_raw):
        if self.motor is None:
            self._set_status(f"{action}失败：CAN 未连接")
            return False
        if self._busy:
            self._set_status(f"{action}失败：电机正在执行其他操作")
            return False
        if self._sensor_sample_provider is None:
            self._set_status(f"{action}失败：没有传感器数据源")
            return False
        sample = self._sensor_sample_provider()
        if sample is None:
            self._set_status(f"{action}失败：尚未收到传感器数据")
            return False
        if sample.get("channel_count", 0) < 8:
            self._set_status(f"{action}失败：当前数据不足 8 个通道")
            return False
        if sample.get("age_seconds", float("inf")) > self._auto_config.sensor_frame_timeout_seconds:
            self._set_status(f"{action}失败：传感器数据已经停止更新")
            return False
        if require_raw and not sample.get("has_raw", False):
            self._set_status(f"{action}失败：请先切换到 Debug 输出模式")
            return False
        return True

    def _make_auto_tester(self, progress_callback=None):
        return MotorAutoTester(
            motor=self.motor,
            sensor_sample_provider=self._sensor_sample_provider,
            stop_event=self._stop_event,
            config=self._auto_config,
            speed_rpm=self._selected_speed_rpm(),
            acceleration=self.acceleration_input.value(),
            progress_callback=progress_callback or self._emit_auto_test_progress,
            position_callback=self.worker_position.emit,
        )

    def _emit_auto_test_progress(self, message):
        self._record_can_event(f"AUTO progress: {message}")
        self.worker_status.emit(message)

    def _emit_repeatability_progress(self, message):
        self._record_can_event(f"AUTO repeatability progress: {message}")
        self.worker_status.emit(message)
        self.repeatability_log.emit(message)

    @Slot(str)
    def _append_repeatability_log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.repeatability_log_view.appendPlainText(f"[{timestamp}] {message}")
        scroll_bar = self.repeatability_log_view.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def _wait_until_position(
        self,
        start_counts,
        target_counts,
        timeout_seconds=None,
    ):
        move_counts = abs(target_counts - start_counts)
        minimum_movement = max(10, round(move_counts * 0.1))
        deadline = (
            None
            if timeout_seconds is None
            else time.monotonic() + float(timeout_seconds)
        )
        last_counts = None
        stable_samples = 0

        while not self._stop_event.is_set():
            feedback = self._poll_motor_feedback()
            running_state = feedback["running_state"]
            current_counts = feedback["position_counts"]
            self.record_motion_diagnostic(
                "position_wait_feedback",
                position_counts=current_counts,
                target_position_counts=target_counts,
                actual_minus_target_counts=current_counts - target_counts,
                measured_speed_rpm=feedback["measured_speed_rpm"],
                position_speed_counts_s=feedback["position_speed_counts_s"],
                estimated_acceleration_r_s2=feedback[
                    "estimated_acceleration_r_s2"
                ],
                phase_current_ma=feedback["phase_current_ma"],
                running_state=running_state,
                running_state_name=feedback["running_state_name"],
            )
            moved = abs(current_counts - start_counts) >= minimum_movement
            if moved and last_counts is not None and abs(current_counts - last_counts) <= 10:
                stable_samples += 1
            else:
                stable_samples = 0
            position_error = abs(current_counts - target_counts)
            if (
                running_state in (0, 1)
                and position_error <= 20
                and stable_samples >= 2
            ):
                return True
            if running_state in (3, 4, 5):
                state_name = MOTOR_RUNNING_STATE_NAMES[running_state]
                raise RuntimeError(
                    f"电机{state_name}：目标={target_counts}，实际={current_counts} counts"
                )
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError(
                    f"等待电机到位超时：目标={target_counts}，"
                    f"实际={current_counts}，误差={current_counts - target_counts} counts"
                )
            last_counts = current_counts
            self._stop_event.wait(0.1)
        return False

    def _brake_after_stop(self):
        if self.motor is not None:
            self.worker_status.emit("正在停止电机...")
            self.motor.brake_stop(auto_clear=True)
        self.worker_status.emit("电机已停止")

    @Slot()
    def request_stop(self):
        if self.motor is None:
            self._record_can_event("UI stop ignored: CAN is not connected")
            return
        self._record_can_event(f"UI stop requested busy={self._busy}")
        self._stop_event.set()
        if self._busy:
            self._set_status("已请求停止，等待当前 CAN 应答...")
            self._record_can_event(
                "UI stop deferred: current synchronous CAN request must finish first"
            )
            return
        self._start_worker(self._brake_after_stop, "正在停止电机...")

    def is_can_connected(self):
        return self.motor is not None

    def is_operation_busy(self):
        return self._busy

    def start_external_operation(self, operation, initial_status):
        """把其他页面提交的电机任务放入唯一 CAN 服务线程。"""
        if self._closing:
            return False, "程序正在关闭"
        if self.motor is None:
            return False, "CAN 未连接"
        if self._busy:
            return False, "已有电机操作正在运行"

        motor = self.motor
        self._stop_event.clear()

        def task():
            operation(motor, self._stop_event)

        self._start_worker(task, initial_status)
        return True, ""

    def request_external_stop(self):
        """通知外部测试任务尽快刹车退出。"""
        self._stop_event.set()

    def _start_worker(self, task, initial_status):
        if self._busy or self._closing:
            self._record_can_event(
                f"WORKER start ignored busy={self._busy} closing={self._closing}"
            )
            return
        self._busy = True
        self._worker_started_at = time.perf_counter()
        self._record_can_event(f"WORKER start status={initial_status}")
        self._set_status(initial_status)
        self._refresh_controls()
        self._record_can_event(
            "GUI motor controls locked until the current worker finishes"
        )

        def queued_task():
            try:
                task()
            except Exception as exc:
                self._record_can_event(
                    f"WORKER error: {exc!r}\n{traceback.format_exc().rstrip()}"
                )
                self.worker_status.emit(f"电机操作失败: {exc}")
                if self.motor is None:
                    self.worker_connected.emit(False)
            finally:
                finished_at = time.perf_counter()
                elapsed_ms = (finished_at - self._worker_started_at) * 1000.0
                self._record_can_event(
                    f"WORKER background finished elapsed={elapsed_ms:.3f} ms"
                )
                self.worker_finished.emit(finished_at)

        self._task_queue.put(queued_task)

    def _service_loop(self):
        """在唯一常驻线程中串行处理全部CAN任务，并在空闲时发布反馈。"""
        next_feedback_at = 0.0
        try:
            while not self._service_stop_event.is_set():
                try:
                    task = self._task_queue.get(timeout=0.02)
                except queue.Empty:
                    task = None
                if task is not None:
                    task()
                    next_feedback_at = 0.0
                    continue
                now = time.monotonic()
                if self.motor is None or self._busy or now < next_feedback_at:
                    continue
                try:
                    self._poll_motor_feedback()
                except Exception as exc:
                    self._record_can_event(f"Idle feedback poll failed: {exc!r}")
                next_feedback_at = time.monotonic() + 0.1
        finally:
            motor = self.motor
            self.motor = None
            if motor is not None:
                try:
                    motor.close()
                except Exception as exc:
                    self._record_can_event(f"Service close error: {exc!r}")
            self.worker_connected.emit(False)

    def _reset_feedback_estimator(self):
        self._feedback_last_position = None
        self._feedback_last_speed_counts_s = None
        self._feedback_last_sample_at = None
        with self._feedback_lock:
            self._latest_motor_feedback = None

    def _poll_motor_feedback(self):
        motor = self.motor
        if motor is None:
            raise RuntimeError("CAN 未连接")
        sampled_at = time.monotonic()
        position = motor.read_position_counts()
        speed_rpm = motor.read_speed_rpm()
        current_ma = motor.read_phase_current_ma()
        running_state = motor.read_running_state()
        interval = (
            None
            if self._feedback_last_sample_at is None
            else sampled_at - self._feedback_last_sample_at
        )
        position_speed = None
        acceleration = None
        if (
            interval is not None
            and interval > 0
            and self._feedback_last_position is not None
        ):
            position_speed = (
                position - self._feedback_last_position
            ) / interval
            if self._feedback_last_speed_counts_s is not None:
                acceleration = (
                    (position_speed - self._feedback_last_speed_counts_s)
                    / interval
                    / PD42S1Motor.COUNTS_PER_TURN
                )
        snapshot = {
            "sampled_monotonic": sampled_at,
            "position_counts": position,
            "position_deg": position / PD42S1Motor.COUNTS_PER_TURN * 360.0,
            "measured_speed_rpm": speed_rpm,
            "position_speed_counts_s": position_speed,
            "estimated_acceleration_r_s2": acceleration,
            "sample_interval_ms": (
                None if interval is None else interval * 1000.0
            ),
            "phase_current_ma": current_ma,
            "running_state": running_state,
            "running_state_name": MOTOR_RUNNING_STATE_NAMES.get(
                running_state,
                f"未知状态{running_state}",
            ),
            "source": "motor_can_service",
        }
        self._feedback_last_position = position
        self._feedback_last_speed_counts_s = position_speed
        self._feedback_last_sample_at = sampled_at
        self.publish_motor_feedback(snapshot)
        return snapshot

    def publish_motor_feedback(self, snapshot):
        """向所有页面广播同一份电机反馈快照。"""
        published = dict(snapshot)
        sampled_at = published.get("sampled_monotonic")
        position = published.get("position_counts")
        if sampled_at is not None and position is not None:
            self._feedback_last_position = int(position)
            self._feedback_last_speed_counts_s = published.get(
                "position_speed_counts_s"
            )
            self._feedback_last_sample_at = float(sampled_at)
        with self._feedback_lock:
            self._latest_motor_feedback = dict(published)
        self.motor_feedback.emit(published)

    def latest_motor_feedback(self):
        """返回最近一次共享快照的副本，未收到反馈时返回None。"""
        with self._feedback_lock:
            if self._latest_motor_feedback is None:
                return None
            return dict(self._latest_motor_feedback)

    @Slot(object)
    def _apply_motor_feedback(self, snapshot):
        if "position_counts" in snapshot:
            self._set_position(int(snapshot["position_counts"]))
        current = snapshot.get("phase_current_ma")
        if current is not None:
            self._set_current(int(current))

    @Slot(str)
    def _set_status(self, message):
        self.can_status_label.setText(message)
        self.can_status_label.setToolTip(message)

    @Slot(bool)
    def _set_connected(self, connected):
        self.connect_button.setText("断开 CAN" if connected else "连接 CAN")
        self._release_position_confirmed = False
        self._update_release_position_label()
        self.release_position_changed.emit(self.release_position_snapshot())
        if not connected:
            self._zero_position_counts = None
            self._maximum_position_counts = None
            self._maximum_press_angle_deg = None
            self.position_label.setText("位置 --")
            self.current_label.setText("瞬时相电流 -- mA")
            self.auto_test_result_label.setText("自动测试：尚未寻找零点")
        self._refresh_controls()

    @Slot(int)
    def _set_position(self, counts):
        degrees = int(counts) / PD42S1Motor.COUNTS_PER_TURN * 360.0
        self.position_label.setText(
            f"位置 {int(counts)} counts · {degrees:.2f}°"
        )

    @Slot(int)
    def _set_current(self, current_ma):
        self.current_label.setText(f"瞬时相电流 {int(current_ma)} mA")

    @Slot()
    def _on_position_zeroed(self):
        self._zero_position_counts = None
        self._maximum_position_counts = None
        self._maximum_press_angle_deg = None
        self._release_position_confirmed = False
        self._update_release_position_label()
        self.release_position_changed.emit(self.release_position_snapshot())
        self.auto_test_result_label.setText("自动测试：位置已手动置零，请重新寻找零点")

    @Slot(float)
    def _on_worker_finished(self, emitted_at):
        queue_delay_ms = (time.perf_counter() - emitted_at) * 1000.0
        total_elapsed_ms = (
            (time.perf_counter() - self._worker_started_at) * 1000.0
            if self._worker_started_at is not None
            else 0.0
        )
        self._busy = False
        self._worker_started_at = None
        self._refresh_controls()
        self._record_can_event(
            f"GUI worker finished processed total={total_elapsed_ms:.3f} ms "
            f"queue_delay={queue_delay_ms:.3f} ms; controls refreshed"
        )

    @Slot(object)
    def _on_auto_test_result(self, result):
        self._auto_test_kind = None
        if not result.get("ok", False):
            prefix = "自动测试已停止" if result.get("cancelled", False) else "自动测试失败"
            message = f"{prefix}：{result.get('error', '未知错误')}"
            if result.get("kind") == "repeatability":
                self.repeatability_log.emit(message)
            self.auto_test_result_label.setText(message)
            self._set_status(message)
            self._refresh_controls()
            return

        if result["kind"] == "zero":
            self._zero_position_counts = int(result["zero_position_counts"])
            self._maximum_position_counts = None
            self._maximum_press_angle_deg = None
            message = (
                f"零点已找到：{self._zero_position_counts} counts；"
                f"准备位置={int(result['ready_position_counts'])} counts"
            )
            self._record_can_event(
                f"AUTO zero success counts={self._zero_position_counts} "
                f"signal_sum={result['signal_sum']} steps={result['steps']}"
            )
        elif result["kind"] == "max_range":
            self._maximum_position_counts = int(result["maximum_position_counts"])
            self._maximum_press_angle_deg = float(result["maximum_press_angle_deg"])
            trigger_text = ", ".join(f"CH{channel}" for channel in result["trigger_channels"])
            message = (
                f"最大量程已找到：相对零点下压 "
                f"{self._zero_position_counts - self._maximum_position_counts} counts；"
                f"触发通道 {trigger_text}；已返回准备位"
            )
            self._record_can_event(
                f"AUTO max-range success counts={self._maximum_position_counts} "
                f"press_deg={self._maximum_press_angle_deg:.3f} "
                f"raw={result['raw_values']} steps={result['steps']}"
            )
        elif result["kind"] == "repeatability":
            sums_text = ", ".join(str(value) for value in result["signal_sums"])
            message = (
                f"重复精度测试完成：模式={result['recovery_mode_name']}，"
                f"有效结果={result['rounds']}次，尝试={result['attempts']}次，"
                f"丢弃={result['rejected_attempts']}次；"
                f"正式FLOATING超时={result['floating_timeout_attempts']}次；"
                f"位置重锁={result['formal_relock_count']}次，"
                f"重锁废弃旧结果={result['relocked_result_discards']}次；"
                f"平均值={result['average']:.2f}，最小={result['minimum']}，"
                f"最大={result['maximum']}，极差={result['span']}，"
                f"相对极差={result['relative_span_percent']:.3f}%，"
                f"样本标准差={result['standard_deviation']:.3f}"
            )
            if result["recovery_mode"] == "rapid":
                self.repeatability_log.emit(
                    f"预压：共{result['precondition_runs']}段判稳过程，"
                    f"累计{result['total_precondition_rounds']}轮；"
                    f"最后一段执行{result['precondition_rounds']}轮，"
                    f"有效{result['precondition_valid_rounds']}轮，"
                    f"可达下压参考={result['pressed_reference_counts']} counts，"
                    f"稳定窗口相对极差="
                    f"{result['precondition_relative_span_percent']:.3f}%，"
                    f"Signal和={result['precondition_signal_sums']}"
                )
            self.repeatability_log.emit(
                f"{result['rounds']}次Signal和：{sums_text}"
            )
            self.repeatability_log.emit(message)
            self.repeatability_log.emit(f"CSV：{result['result_path']}")
            self._record_can_event(
                f"AUTO repeatability success rounds={result['rounds']} "
                f"attempts={result['attempts']} rejected={result['rejected_attempts']} "
                f"formal_relocks={result['formal_relock_count']} "
                f"relocked_result_discards="
                f"{result['relocked_result_discards']} "
                f"precondition_runs={result['precondition_runs']} "
                f"total_precondition_rounds="
                f"{result['total_precondition_rounds']} "
                f"precondition_rounds={result['precondition_rounds']} "
                f"precondition_relative_span="
                f"{result['precondition_relative_span_percent']:.6f}% "
                f"recovery_mode={result['recovery_mode']} "
                f"average={result['average']:.6f} min={result['minimum']} "
                f"max={result['maximum']} span={result['span']} "
                f"relative_span={result['relative_span_percent']:.6f}% "
                f"sample_standard_deviation="
                f"{result['standard_deviation']:.6f} "
                f"csv={result['result_path']}"
            )
        else:
            message = (
                f"随机自动测试完成：{result['rounds']} 轮；"
                f"状态异常 {result['state_failures']} 轮；"
                f"SLOW_BACK Signal 非零 {result['slow_back_failures']} 轮；"
                f"汇总文件 {result['summary_path']}"
            )
            self._record_can_event(
                f"AUTO random-press success rounds={result['rounds']} "
                f"state_failures={result['state_failures']} "
                f"slow_back_failures={result['slow_back_failures']} "
                f"seed={result['random_seed']} detail={result['detail_path']} "
                f"summary={result['summary_path']}"
            )
        self.auto_test_result_label.setText(message)
        self._set_status(message)
        self._refresh_controls()

    def _refresh_controls(self):
        connected = self.motor is not None
        self.connect_button.setEnabled(not self._busy)
        self.press_button.setEnabled(connected and not self._busy)
        self.lift_button.setEnabled(connected and not self._busy)
        self.repeat_button.setEnabled(connected and not self._busy)
        self.read_position_button.setEnabled(connected and not self._busy)
        self.zero_position_button.setEnabled(connected and not self._busy)
        self.set_release_position_button.setEnabled(connected and not self._busy)
        self.start_repeatability_button.setEnabled(connected and not self._busy)
        self.start_full_recovery_test_button.setEnabled(connected and not self._busy)
        self.repeatability_count_units_input.setEnabled(not self._busy)
        self.rapid_floating_channel_count_input.setEnabled(not self._busy)
        self.rapid_precondition_relative_span_input.setEnabled(not self._busy)
        self.rapid_formal_rounds_input.setEnabled(not self._busy)
        self.rapid_precondition_max_rounds_input.setEnabled(not self._busy)
        self.rapid_floating_timeout_input.setEnabled(not self._busy)
        self.rapid_floating_hold_input.setEnabled(not self._busy)
        self.stop_repeatability_button.setEnabled(
            connected and self._busy and self._auto_test_kind == "repeatability"
        )
        self.stop_button.setEnabled(connected)
        for widget in (
            self.move_count_units_input,
            self.speed_input,
            self.acceleration_input,
            self.wait_input,
            self.repeat_input,
            self.press_first_checkbox,
        ):
            widget.setEnabled(not self._busy)
        self.control_state_changed.emit(connected, self._busy)

    def shutdown(self):
        self._record_can_event("SHUTDOWN begin")
        self._closing = True
        self._stop_event.set()
        self._service_stop_event.set()
        self._save_timer.stop()
        self._save_settings()
        self._task_queue.put(None)
        service = self._service_thread
        if service is not None and service.is_alive():
            service.join(timeout=1.2)
        service_alive = service is not None and service.is_alive()
        if service_alive:
            self._record_can_event("SHUTDOWN CAN service still active after 1200 ms")
        self._record_can_event("SHUTDOWN end")
        self._close_can_log()
