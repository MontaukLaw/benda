import csv
import json
import statistics
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from channel_states import STATE_FLOATING, STATE_NAMES, IDLE_LIKE_STATES

from qt_compat import QtCore, QtWidgets, Signal


SCHEMA_VERSION = 3
PROJECT_SUFFIX = ".fixedweightcal.json"
AVAILABLE_WEIGHTS_G = (10, 20, 50, 100, 200, 500, 1000, 2000, 3000)
DEFAULT_HEAD_MASS_G = 52.0
DEFAULT_ROUNDS = 3
MANUAL_MODE_FLAG = 0x40
AFFINE_ACTIVE_FLAG = 0x80
UNLOADED_CONFIRM_SECONDS = 1.0
FLOATING_CONFIRM_SECONDS = 1.0
MAX_CONFIRMATION_GAP_SECONDS = 0.2
MIN_CONFIRMATION_SAMPLES = 8
POINT_TIMEOUT_SECONDS = 45.0
DATA_STALE_ABORT_SECONDS = 3.0
RANGE_STATUS_STALE_SECONDS = 2.0
MEASUREMENT_ZERO_WINDOW_SECONDS = 2.0
MEASUREMENT_DISPLAY_WINDOW_SECONDS = 0.5
MEASUREMENT_ZERO_TIMEOUT_SECONDS = 15.0
DEFAULT_LUT_PATH = (
    Path(__file__).resolve().with_name("weight_calibration_parameters")
    / "loading_lut.json"
)


def _linear_slope(samples):
    if len(samples) < 2:
        return 0.0
    t0 = samples[0][0]
    xs = [item[0] - t0 for item in samples]
    ys = [item[1] for item in samples]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    denominator = sum((value - x_mean) ** 2 for value in xs)
    if denominator <= 0.0:
        return 0.0
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator


def _confirmation_median(window):
    """使用四路连续FLOATING确认窗口的逐通道中位数生成最终标定值。"""
    def median_tuple(field, integer):
        values = []
        for index in range(4):
            median = statistics.median(item[field][index] for item in window)
            values.append(int(round(median)) if integer else float(median))
        return tuple(values)

    raw = median_tuple("raw", True)
    return {
        "time": window[-1]["time"],
        "raw": raw,
        "sum": sum(raw),
        "hardware_raw": median_tuple("hardware_raw", True),
        "baseline": median_tuple("baseline", True),
        "signal": median_tuple("signal", True),
        "fast_k_percent": median_tuple("fast_k_percent", False),
        "slow_k_percent": median_tuple("slow_k_percent", False),
        "state": tuple(int(value) for value in window[-1]["state"]),
        "median_window_sample_count": len(window),
    }


def _weight_combination(target):
    if target is None:
        return "不放压头，不放砝码"
    if target <= 0:
        return "仅放置压头"
    if target in AVAILABLE_WEIGHTS_G:
        return f"{target} g"
    remaining = int(target)
    selected = []
    for weight in reversed(AVAILABLE_WEIGHTS_G):
        while remaining >= weight:
            selected.append(weight)
            remaining -= weight
    if remaining:
        return f"合计 {target} g"
    return " + ".join(f"{weight} g" for weight in selected)


class WeightCalibrationWidget(QtWidgets.QGroupBox):
    """在固定量程下按通道状态触发真实重量标定采集。"""

    status_message = Signal(str)
    operation_busy_changed = Signal(bool)

    def __init__(self, sensor_sample_provider, operation_available_provider=None, parent=None):
        super().__init__("固定量程真实重量标定", parent)
        self.setObjectName("FeaturePanel")
        self.setCheckable(True)
        self.setChecked(False)
        self._sensor_sample_provider = sensor_sample_provider
        self._operation_available_provider = operation_available_provider
        self._connected = False
        self._device_status = None
        self._device_status_received_at = None
        self._fixed_range_mode = False
        self._fixed_channel_config = None
        self._running = False
        self._phase = None
        self._sequence = []
        self._step_index = 0
        self._last_sensor_sequence = None
        self._stale_since = None
        self._confirmation_window = []
        self._condition_since = None
        self._last_confirmation_time = None
        self._point_started_at = None
        self._locked_range = None
        self._results = []
        self._archived_results = []
        self._session_id = uuid4().hex
        self._started_at = None
        self._project_path = None
        self._project_completed = False
        self._measurement_lut = None
        self._measurement_lut_path = None
        self._measurement_baseline = None
        self._measurement_window = deque()
        self._measurement_zero_started = None
        self._build_ui()
        self._load_lut(DEFAULT_LUT_PATH, show_error=False)

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._poll_sample)
        self._timer.start()
        self.toggled.connect(self._set_expanded)
        self._set_expanded(False)
        self._update_controls()

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 12, 8, 8)
        self.content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(7)

        settings = QtWidgets.QHBoxLayout()
        settings.addWidget(QtWidgets.QLabel("压头质量"))
        self.head_mass_input = QtWidgets.QDoubleSpinBox()
        self.head_mass_input.setRange(0.0, 10000.0)
        self.head_mass_input.setDecimals(1)
        self.head_mass_input.setValue(DEFAULT_HEAD_MASS_G)
        settings.addWidget(self.head_mass_input)
        settings.addWidget(QtWidgets.QLabel("g"))
        settings.addWidget(QtWidgets.QLabel("升序独立加载轮数"))
        self.rounds_input = QtWidgets.QSpinBox()
        self.rounds_input.setRange(1, 10)
        self.rounds_input.setValue(DEFAULT_ROUNDS)
        settings.addWidget(self.rounds_input)
        settings.addWidget(QtWidgets.QLabel("次"))
        settings.addStretch(1)
        layout.addLayout(settings)

        hint = QtWidgets.QLabel(
            "每个点都先移开压头和全部砝码。真空载由四路IDLE/WAIT/RECOVER持续1秒触发；"
            "其余点必须由CH4～CH7四路全部FLOATING持续1秒触发；"
            "最终值取这一秒确认窗口内的逐通道中位数。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.instruction_label = QtWidgets.QLabel("连接设备后可开始。")
        self.instruction_label.setWordWrap(True)
        self.instruction_label.setStyleSheet("font-size: 15px; font-weight: 600; padding: 8px;")
        layout.addWidget(self.instruction_label)

        self.live_label = QtWidgets.QLabel("等待数据")
        self.live_label.setWordWrap(True)
        layout.addWidget(self.live_label)

        buttons = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("开始新标定")
        self.start_button.clicked.connect(self._start)
        buttons.addWidget(self.start_button)
        self.open_button = QtWidgets.QPushButton("打开项目")
        self.open_button.clicked.connect(self._open_project)
        buttons.addWidget(self.open_button)
        self.continue_button = QtWidgets.QPushButton("继续未完成项目")
        self.continue_button.setToolTip(
            "保留全部已完成点，从当前未完成点的完全卸载步骤继续"
        )
        self.continue_button.clicked.connect(self._continue_project)
        buttons.addWidget(self.continue_button)
        self.ready_button = QtWidgets.QPushButton("按提示继续")
        self.ready_button.setMinimumHeight(48)
        self.ready_button.setMinimumWidth(190)
        self.ready_button.setStyleSheet(
            "QPushButton {"
            " background-color: #159447; color: white; border: 1px solid #20b85a;"
            " border-radius: 6px; padding: 8px 18px; font-size: 17px; font-weight: 700;"
            "}"
            "QPushButton:hover { background-color: #1eaa53; }"
            "QPushButton:pressed { background-color: #10783a; }"
            "QPushButton:disabled {"
            " background-color: #4a5058; color: #aeb4bc; border-color: #5b626b;"
            "}"
        )
        self.ready_button.clicked.connect(self._on_capture_button_clicked)
        buttons.addWidget(self.ready_button)
        self.stop_button = QtWidgets.QPushButton("停止")
        self.stop_button.clicked.connect(self._stop)
        buttons.addWidget(self.stop_button)
        self.export_button = QtWidgets.QPushButton("导出结果CSV")
        self.export_button.clicked.connect(self._export_csv_dialog)
        buttons.addWidget(self.export_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        layout.addWidget(self.progress)

        self.table = QtWidgets.QTableWidget(0, 16)
        self.table.setHorizontalHeaderLabels([
            "轮次", "点位", "加放砝码/g", "总质量/g",
            "CH4窗口中位值", "CH5窗口中位值", "CH6窗口中位值", "CH7窗口中位值",
            "CH4状态", "CH5状态", "CH6状态", "CH7状态",
            "FLOATING数", "确认时长/ms", "确认帧数", "固定量程",
        ])
        self.table.verticalHeader().setVisible(False)
        no_edit = getattr(
            QtWidgets.QAbstractItemView,
            "NoEditTriggers",
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers,
        )
        self.table.setEditTriggers(no_edit)
        self.table.setMinimumHeight(260)
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table)

        layout.addWidget(QtWidgets.QLabel("按目标点汇总（用于检查多轮重复性和单调性）"))
        self.summary_table = QtWidgets.QTableWidget(0, 8)
        self.summary_table.setHorizontalHeaderLabels([
            "点位", "加放砝码/g", "总质量/g", "完成轮数",
            "Raw和值均值", "最小值", "最大值", "单调性",
        ])
        self.summary_table.verticalHeader().setVisible(False)
        self.summary_table.setEditTriggers(no_edit)
        self.summary_table.setMinimumHeight(220)
        self.summary_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.summary_table)

        measurement_group = QtWidgets.QGroupBox("实际重量测量测试")
        measurement_layout = QtWidgets.QVBoxLayout(measurement_group)
        measurement_buttons = QtWidgets.QHBoxLayout()
        self.load_lut_button = QtWidgets.QPushButton("读取LUT参数")
        self.load_lut_button.clicked.connect(self._select_lut)
        measurement_buttons.addWidget(self.load_lut_button)
        self.zero_measurement_button = QtWidgets.QPushButton("仅放压头，稳态置零")
        self.zero_measurement_button.clicked.connect(self._start_measurement_zero)
        measurement_buttons.addWidget(self.zero_measurement_button)
        self.measurement_parameter_label = QtWidgets.QLabel("尚未读取LUT参数")
        self.measurement_parameter_label.setWordWrap(True)
        measurement_buttons.addWidget(self.measurement_parameter_label, 1)
        measurement_layout.addLayout(measurement_buttons)

        self.measured_weight_label = QtWidgets.QLabel("-- g")
        self.measured_weight_label.setAlignment(
            getattr(QtCore.Qt, "AlignCenter", QtCore.Qt.AlignmentFlag.AlignCenter)
        )
        self.measured_weight_label.setStyleSheet(
            "font-size:48px; font-weight:800; color:#68d391; padding:14px;"
        )
        measurement_layout.addWidget(self.measured_weight_label)
        self.measurement_status_label = QtWidgets.QLabel(
            "请先仅放置压头并点击稳态置零；显示结果为已扣除压头的砝码质量。"
        )
        self.measurement_status_label.setAlignment(
            getattr(QtCore.Qt, "AlignCenter", QtCore.Qt.AlignmentFlag.AlignCenter)
        )
        self.measurement_status_label.setWordWrap(True)
        measurement_layout.addWidget(self.measurement_status_label)
        layout.addWidget(measurement_group)

        outer.addWidget(self.content)

    def _set_expanded(self, expanded):
        self.content.setVisible(bool(expanded))
        self.setMaximumHeight(16777215 if expanded else 32)

    def set_connected(self, connected):
        self._connected = bool(connected)
        if not connected and self._running:
            self._stop("串口已断开，标定已停止；已完成的数据仍已保存。")
        if not connected:
            self._device_status = None
            self._device_status_received_at = None
            self._fixed_range_mode = False
            self._fixed_channel_config = None
            self._measurement_baseline = None
            self._measurement_zero_started = None
            self._measurement_window.clear()
            self.measured_weight_label.setText("-- g")
            self.measurement_status_label.setText(
                "连接断开，测量基线已清除；重连后请仅放压头并重新置零。"
            )
        self._update_controls()

    def set_fixed_range_mode(self, enabled):
        self._fixed_range_mode = bool(enabled)
        self._fixed_channel_config = None
        if self._fixed_range_mode:
            self._device_status = None
            self._device_status_received_at = None
            if not self._running:
                self.instruction_label.setText("正在读取固件全通道固定档位……")
        self._update_controls()

    def on_channel_config(self, frame):
        if not self._fixed_range_mode:
            return
        self._fixed_channel_config = dict(frame)
        if self._running:
            error = self._range_error(self._locked_range)
            if error is not None:
                self._stop(f"固定量程锁定失效，标定已停止：{error}")
                return
        if not self._running:
            snapshot, error = self._fixed_range_snapshot()
            if error is None:
                self.instruction_label.setText(
                    f"固件固定档位已确认：Nsub={snapshot['nsub']}，"
                    f"TxDiv={snapshot['tx_div']}，Ref={snapshot['ref_cdac']}；可开始标定。"
                )
            else:
                self.instruction_label.setText(f"固件固定档位尚未就绪：{error}")
        self._update_controls()

    def on_device_status(self, frame):
        if self._fixed_range_mode:
            return
        self._device_status = dict(frame)
        self._device_status_received_at = time.perf_counter()
        if self._running:
            error = self._range_error(self._locked_range)
            if error is not None:
                self._stop(f"固定量程锁定失效，标定已停止：{error}")
                return
        self._update_controls()

    def _select_lut(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "读取重量LUT参数",
            str(DEFAULT_LUT_PATH.parent),
            "JSON (*.json)",
        )
        if path:
            self._load_lut(Path(path), show_error=True)

    def _load_lut(self, path, show_error=True):
        try:
            document = json.loads(Path(path).read_text(encoding="utf-8"))
            if int(document.get("schema_version", 0)) != 1:
                raise ValueError("不支持的LUT版本")
            if document.get("input", {}).get("aggregation") != "ch4_ch7_delta_sum":
                raise ValueError("LUT输入不是CH4～CH7 Raw增量和")
            points = document.get("points", [])
            parsed = sorted(
                (float(point["delta_sum"]), float(point["added_mass_g"]))
                for point in points
            )
            if len(parsed) < 2 or parsed[0] != (0.0, 0.0):
                raise ValueError("LUT至少需要两个点，且首点必须为(0 counts, 0 g)")
            if any(parsed[index][0] <= parsed[index - 1][0] for index in range(1, len(parsed))):
                raise ValueError("LUT的delta_sum必须严格递增")
            self._measurement_lut = {
                "document": document,
                "points": parsed,
                "head_mass_g": float(document["head_mass_g"]),
                "valid_min_g": float(document.get("valid_added_mass_range_g", [0, parsed[-1][1]])[0]),
                "valid_max_g": float(document.get("valid_added_mass_range_g", [0, parsed[-1][1]])[1]),
            }
            self._measurement_lut_path = Path(path)
            self._measurement_baseline = None
            self.measurement_parameter_label.setText(
                f"{Path(path).name}｜压头{self._measurement_lut['head_mass_g']:.1f} g｜"
                f"有效砝码范围{self._measurement_lut['valid_min_g']:.0f}～"
                f"{self._measurement_lut['valid_max_g']:.0f} g"
            )
            self.measurement_status_label.setText(
                "参数已读取。请仅放置压头并点击“稳态置零”。"
            )
            self._update_controls()
            return True
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            self._measurement_lut = None
            self.measurement_parameter_label.setText(f"LUT读取失败：{exc}")
            if show_error:
                QtWidgets.QMessageBox.warning(self, "LUT读取失败", str(exc))
            self._update_controls()
            return False

    def _start_measurement_zero(self):
        if not self._connected or self._measurement_lut is None:
            return
        self._measurement_baseline = None
        self._measurement_window.clear()
        self._measurement_zero_started = time.perf_counter()
        self.measured_weight_label.setText("置零中…")
        self.measurement_status_label.setText(
            f"请保持仅有{self._measurement_lut['head_mass_g']:.1f} g压头，正在自动寻找2秒稳态窗口。"
        )

    def _interpolate_added_mass(self, delta_sum):
        points = self._measurement_lut["points"]
        if delta_sum <= points[0][0]:
            return 0.0, delta_sum < points[0][0]
        if delta_sum >= points[-1][0]:
            return points[-1][1], delta_sum > points[-1][0]
        for index in range(1, len(points)):
            x1, y1 = points[index]
            if delta_sum <= x1:
                x0, y0 = points[index - 1]
                ratio = (delta_sum - x0) / (x1 - x0)
                return y0 + ratio * (y1 - y0), False
        return points[-1][1], True

    def _update_measurement(self, item):
        now = item["time"]
        self._measurement_window.append(item)
        keep_seconds = max(MEASUREMENT_ZERO_WINDOW_SECONDS, MEASUREMENT_DISPLAY_WINDOW_SECONDS)
        while self._measurement_window and now - self._measurement_window[0]["time"] > keep_seconds:
            self._measurement_window.popleft()

        if self._measurement_zero_started is not None:
            zero_items = [
                value for value in self._measurement_window
                if now - value["time"] <= MEASUREMENT_ZERO_WINDOW_SECONDS
            ]
            elapsed = now - self._measurement_zero_started
            if len(zero_items) >= 8 and zero_items[-1]["time"] - zero_items[0]["time"] >= 1.8:
                sums = [value["sum"] for value in zero_items]
                slope = _linear_slope([(value["time"], value["sum"]) for value in zero_items])
                if statistics.pstdev(sums) <= 5.0 and abs(slope) <= 10.0:
                    self._measurement_baseline = [
                        statistics.median(value["raw"][index] for value in zero_items)
                        for index in range(4)
                    ]
                    self._measurement_zero_started = None
                    self.measurement_status_label.setText(
                        f"仅压头置零完成：Raw和基线={sum(self._measurement_baseline):.1f}。"
                        "当前显示为已扣除压头的砝码质量。"
                    )
            if self._measurement_zero_started is not None:
                self.measured_weight_label.setText("置零中…")
                if elapsed >= MEASUREMENT_ZERO_TIMEOUT_SECONDS:
                    self._measurement_zero_started = None
                    self.measured_weight_label.setText("-- g")
                    self.measurement_status_label.setText(
                        "15秒内未找到置零稳态，请确认只放置压头并重试。"
                    )
                return

        if self._measurement_lut is None or self._measurement_baseline is None:
            return
        display_items = [
            value for value in self._measurement_window
            if now - value["time"] <= MEASUREMENT_DISPLAY_WINDOW_SECONDS
        ]
        if not display_items:
            return
        channels = [
            statistics.median(value["raw"][index] for value in display_items)
            for index in range(4)
        ]
        delta_sum = sum(channels[index] - self._measurement_baseline[index] for index in range(4))
        added_mass, clipped = self._interpolate_added_mass(delta_sum)
        self.measured_weight_label.setText(f"{added_mass:.1f} g")
        valid_min = self._measurement_lut["valid_min_g"]
        valid_max = self._measurement_lut["valid_max_g"]
        if clipped and delta_sum > self._measurement_lut["points"][-1][0]:
            status = "超过LUT上限，显示值已限制在最大标定质量"
            color = "#fc8181"
        elif 0.0 < added_mass < valid_min:
            status = "低于已验证的有效范围，仅供观察"
            color = "#ffd166"
        else:
            status = "加载方向LUT估算；已扣除压头质量"
            color = "#68d391"
        self.measured_weight_label.setStyleSheet(
            f"font-size:48px; font-weight:800; color:{color}; padding:14px;"
        )
        self.measurement_status_label.setText(
            f"{status}｜Δ Raw和={delta_sum:.1f} counts｜"
            f"压头={self._measurement_lut['head_mass_g']:.1f} g"
        )

    def is_running(self):
        return self._running

    def _operation_is_available(self):
        if self._operation_available_provider is None:
            return True
        try:
            return bool(self._operation_available_provider())
        except Exception:
            return False

    def _fixed_range_snapshot(self):
        if self._fixed_channel_config is None:
            return None, "尚未收到全通道固定参数"
        if not self._fixed_channel_config.get("read_ok", False):
            return None, "固件未能完整读取CapSense参数"

        channels = tuple(self._fixed_channel_config.get("channels", ()))
        by_channel = {
            int(channel.get("channel", -1)): channel
            for channel in channels
        }
        if any(channel not in by_channel for channel in range(8)):
            return None, "固定量程真实重量标定仅支持8通道食指"

        fixed_values = set()
        max_raw_counts = []
        for channel_index in range(8):
            channel = by_channel[channel_index]
            configured_nsub = int(channel["configured_nsub"])
            hardware_nsub = int(channel["hardware_nsub"])
            configured_tx_div = int(channel["configured_tx_clk_div"])
            hardware_tx_div = int(channel["hardware_tx_clk_div"])
            configured_ref = int(channel["configured_ref_cdac"])
            hardware_ref = int(channel["hardware_ref_cdac"])
            if (
                configured_nsub != hardware_nsub
                or configured_tx_div != hardware_tx_div
                or configured_ref != hardware_ref
            ):
                return None, f"CH{channel_index}配置值与硬件值不一致"
            fixed_values.add((hardware_nsub, hardware_tx_div, hardware_ref))
            max_raw_counts.append(int(channel["max_raw_count"]))

        if len(fixed_values) != 1:
            return None, "CH0～CH7没有使用同一固定档位"
        nsub, tx_div, ref_cdac = fixed_values.pop()
        if nsub <= 0 or tx_div <= 0 or ref_cdac <= 0 or min(max_raw_counts) <= 0:
            return None, "固定档位参数无效"
        return {
            "device": int(self._fixed_channel_config.get("device", 0)),
            "nsub": nsub,
            "tx_div": tx_div,
            "ref_cdac": ref_cdac,
            "max_count": min(max_raw_counts),
            "affine_active": False,
        }, None

    def _range_snapshot(self):
        if self._fixed_range_mode:
            snapshot, _ = self._fixed_range_snapshot()
            return snapshot
        if self._device_status is None:
            return None
        flags = int(self._device_status["flags"])
        return {
            "device": int(self._device_status.get("device", 0)),
            "nsub": int(self._device_status["nsub"]),
            "tx_div": int(self._device_status["tx_div"]),
            "ref_cdac": int(self._device_status["actual_ref_cdac"]),
            "max_count": int(self._device_status["max_count"]),
            "affine_active": bool(flags & AFFINE_ACTIVE_FLAG),
        }

    def _range_error(self, expected=None):
        if self._fixed_range_mode:
            actual, error = self._fixed_range_snapshot()
            if error is not None:
                return error
            if expected is None:
                return None
            for key in ("device", "nsub", "tx_div", "ref_cdac", "max_count", "affine_active"):
                if actual.get(key) != expected.get(key):
                    return (
                        f"{key}不一致，项目={expected.get(key)}，设备={actual.get(key)}"
                    )
            return None

        if self._device_status is None or self._device_status_received_at is None:
            return "尚未收到固件量程状态"
        if time.perf_counter() - self._device_status_received_at > RANGE_STATUS_STALE_SECONDS:
            return "固件量程状态超过2秒未更新"
        flags = int(self._device_status["flags"])
        if not flags & MANUAL_MODE_FLAG:
            return "固件当前不是手动量程"
        if flags & 0x01 or int(self._device_status.get("blank_frames", 0)) > 0:
            return "量程参数仍在应用或硬件稳定阶段"
        if expected is None:
            return None
        actual = self._range_snapshot()
        for key in ("device", "nsub", "tx_div", "ref_cdac", "max_count", "affine_active"):
            if actual.get(key) != expected.get(key):
                return (
                    f"{key}不一致，项目={expected.get(key)}，设备={actual.get(key)}"
                )
        return None

    def _point_definition(self, point_key):
        if point_key == "empty":
            return "真空载", None
        if point_key == "head":
            return "仅压头", 0
        mass = int(point_key.split("_", 1)[1])
        return f"压头+{mass}g", mass

    def _build_sequence(self):
        sequence = []
        point_keys = ["empty", "head"] + [f"weight_{mass}" for mass in AVAILABLE_WEIGHTS_G]
        for round_index in range(1, self.rounds_input.value() + 1):
            for point_key in point_keys:
                point_name, added_mass = self._point_definition(point_key)
                total_mass = (
                    0.0
                    if added_mass is None
                    else self.head_mass_input.value() + float(added_mass)
                )
                sequence.append({
                    "round": round_index,
                    "point_key": point_key,
                    "point_name": point_name,
                    "added_mass_g": added_mass,
                    "total_mass_g": total_mass,
                })
        return sequence

    def _start(self):
        if not self._connected:
            QtWidgets.QMessageBox.warning(self, "无法开始", "请先连接设备并保持Debug数据输出。")
            return
        if not self._operation_is_available():
            QtWidgets.QMessageBox.warning(self, "无法开始", "其他标定或自动测试正在运行。")
            return
        range_error = self._range_error()
        if range_error is not None:
            ready_hint = (
                "请确认上方全通道参数的配置值与硬件值一致"
                if self._fixed_range_mode
                else "请先在上方切换到手动控制并确认参数稳定"
            )
            QtWidgets.QMessageBox.warning(
                self,
                "固定量程尚未就绪",
                f"{ready_hint}：{range_error}",
            )
            return
        sample = self._sensor_sample_provider()
        if sample is None or not sample.get("has_raw") or sample.get("channel_count", 0) < 8:
            QtWidgets.QMessageBox.warning(self, "无法开始", "请把输出模式切换为Debug并确认收到8通道数据。")
            return
        if self._results:
            answer = QtWidgets.QMessageBox.question(
                self,
                "开始新标定",
                "当前结果将结束并保留在已有项目文件中。确认开始新标定吗？",
            )
            yes = getattr(QtWidgets.QMessageBox, "Yes", QtWidgets.QMessageBox.StandardButton.Yes)
            if answer != yes:
                return
        self._session_id = uuid4().hex
        self._started_at = datetime.now().isoformat(timespec="seconds")
        self._project_path = self._new_project_path()
        self._results = []
        self._archived_results = []
        self._locked_range = self._range_snapshot()
        self._project_completed = False
        self._sequence = self._build_sequence()
        self._step_index = 0
        self._begin_run()

    def _open_project(self):
        if self._running:
            return
        start = Path(__file__).resolve().with_name("weight_calibration_projects")
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "打开重量标定项目",
            str(start),
            f"固定量程重量标定项目 (*{PROJECT_SUFFIX});;JSON (*.json)",
        )
        if not path:
            return
        try:
            document = json.loads(Path(path).read_text(encoding="utf-8"))
            if int(document.get("schema_version", 0)) not in (2, SCHEMA_VERSION):
                raise ValueError("不支持的项目版本")
            rounds = int(document.get("rounds", 0))
            if rounds < 1 or rounds > 10:
                raise ValueError("项目轮数无效")
            results = document.get("results", [])
            if not isinstance(results, list):
                raise ValueError("项目结果格式无效")
            self.head_mass_input.setValue(float(document.get("head_mass_g", DEFAULT_HEAD_MASS_G)))
            self.rounds_input.setValue(rounds)
            self._session_id = str(document.get("session_id") or uuid4().hex)
            self._started_at = document.get("started_at")
            self._project_path = Path(path)
            self._results = results
            self._archived_results = list(document.get("archived_results", []))
            self._locked_range = dict(document.get("locked_range") or {})
            if not self._locked_range:
                raise ValueError("项目缺少固定量程参数")
            self._project_completed = bool(document.get("completed", False))
            self._sequence = self._build_sequence()
            self._step_index = len(self._results)
            self._validate_result_prefix()
            self.progress.setRange(0, len(self._sequence))
            self.progress.setValue(min(self._step_index, len(self._sequence)))
            self._refresh_table()
            self.instruction_label.setText(
                f"已读取项目：{Path(path).name}，有效点{len(self._results)}个，"
                f"归档点{len(self._archived_results)}个。连接设备后可继续未完成点。"
            )
            self.live_label.setText(
                "继续前会核对项目中的Nsub、TxDiv、Ref和仿射状态，然后从完全卸载开始。"
            )
            self._update_controls()
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            QtWidgets.QMessageBox.warning(self, "项目读取失败", str(exc))

    def _continue_project(self):
        if self._running or not self._connected or self._project_path is None:
            return
        if not self._operation_is_available():
            QtWidgets.QMessageBox.warning(self, "无法继续", "其他标定或自动测试正在运行。")
            return
        self._sequence = self._build_sequence()
        if len(self._results) >= len(self._sequence):
            QtWidgets.QMessageBox.information(self, "无需继续", "项目已经包含全部标定点。")
            return
        try:
            self._validate_result_prefix()
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "无法继续", str(exc))
            return
        range_error = self._range_error(self._locked_range)
        if range_error is not None:
            recovery_hint = (
                "请连接使用项目记录固定参数的固件"
                if self._fixed_range_mode
                else "请先恢复项目记录的固定手动量程"
            )
            QtWidgets.QMessageBox.warning(
                self,
                "项目量程不一致",
                f"{recovery_hint}后再继续：{range_error}",
            )
            return
        self._step_index = len(self._results)
        self._project_completed = False
        self._begin_run()

    def _validate_result_prefix(self):
        if len(self._results) > len(self._sequence):
            raise ValueError("项目结果数量超过当前采集序列")
        for index, result in enumerate(self._results):
            expected = self._sequence[index]
            if (
                int(result.get("round", -1)) != expected["round"]
                or result.get("point_key") != expected["point_key"]
            ):
                raise ValueError(f"第{index + 1}个结果与当前采集顺序不一致")

    def _begin_run(self):
        self._running = True
        self._phase = "await_unload"
        self._clear_capture_state()
        self._refresh_table()
        self.progress.setRange(0, len(self._sequence))
        self.progress.setValue(self._step_index)
        self.head_mass_input.setEnabled(False)
        self.rounds_input.setEnabled(False)
        self._show_unload_instruction()
        self._save_project(completed=False)
        self.operation_busy_changed.emit(True)
        self._update_controls()

    def _show_unload_instruction(self, prefix=""):
        if not self._running or self._step_index >= len(self._sequence):
            return
        step = self._sequence[self._step_index]
        self._phase = "await_unload"
        self.instruction_label.setText(
            f"{prefix}第 {step['round']} 轮｜步骤 {self._step_index + 1}/{len(self._sequence)}｜"
            f"目标：{step['point_name']}<br>"
            "<span style='font-size:24px; font-weight:800; color:#ff7b72;'>"
            f"请先移开{self.head_mass_input.value():.1f}g压头和当前全部砝码</span><br>"
            "确认台面完全空载后点击按钮，程序将检查四路IDLE/WAIT/RECOVER持续1秒。"
        )
        self.live_label.setText("等待人工确认已经完全卸载。")
        self._update_capture_button_text()

    def _on_capture_button_clicked(self):
        if not self._running:
            return
        if self._phase == "await_unload":
            self._begin_confirmation("verify_unload")
        elif self._phase == "await_load":
            self._begin_confirmation("verify_load")

    def _begin_confirmation(self, phase):
        if not self._running:
            return
        self._phase = phase
        self._clear_capture_state()
        self._point_started_at = time.perf_counter()
        if phase == "verify_unload":
            self.instruction_label.setText("正在确认CH4～CH7全部IDLE/WAIT/RECOVER并持续1秒，请保持完全空载……")
        else:
            self.instruction_label.setText("正在确认CH4～CH7四路全部FLOATING并持续1秒，请勿触碰载荷……")
        self._update_controls()

    def _clear_capture_state(self):
        self._confirmation_window = []
        self._condition_since = None
        self._last_confirmation_time = None
        self._point_started_at = None
        self._last_sensor_sequence = None
        self._stale_since = None

    def _poll_sample(self):
        if not self._connected:
            return
        sample = self._sensor_sample_provider()
        if sample is None or not sample.get("has_raw") or sample.get("channel_count", 0) < 8:
            if self._running:
                self.live_label.setText("未收到8通道Debug Raw数据。")
            return
        sequence = sample.get("sequence")
        if sample.get("age_seconds", 99.0) > 0.5:
            if self._running and self._phase in ("verify_unload", "verify_load"):
                self.live_label.setText("传感器数据已停止更新。")
                if self._stale_since is None:
                    self._stale_since = time.perf_counter()
                elif time.perf_counter() - self._stale_since >= DATA_STALE_ABORT_SECONDS:
                    self._stop(
                        "传感器Debug数据连续3秒未更新，标定已安全停止并保存。"
                        "固件恢复后请恢复项目固定量程并继续，当前未完成点将从完全卸载重做。"
                    )
            if self._measurement_baseline is not None:
                self.measurement_status_label.setText("传感器Debug数据已停止更新。")
            return
        self._stale_since = None
        if sequence == self._last_sensor_sequence:
            return
        self._last_sensor_sequence = sequence

        now = time.perf_counter()
        raw = tuple(int(value) for value in sample["hardware_raw"][4:8])
        item = {
            "time": now,
            "raw": raw,
            "sum": sum(raw),
            "hardware_raw": tuple(int(value) for value in sample["hardware_raw"][4:8]),
            "baseline": tuple(int(value) for value in sample["baseline"][4:8]),
            "signal": tuple(int(value) for value in sample["signal"][4:8]),
            "fast_k_percent": tuple(float(value) for value in sample["fast_k_percent"][4:8]),
            "slow_k_percent": tuple(float(value) for value in sample["slow_k_percent"][4:8]),
            "state": tuple(int(value) for value in sample["state"][4:8]),
        }
        self._update_measurement(item)

        if not self._running or self._phase not in ("verify_unload", "verify_load"):
            return

        range_error = self._range_error(self._locked_range)
        if range_error is not None:
            self._stop(f"固定量程锁定失效，标定已停止：{range_error}")
            return

        if self._point_started_at is not None and now - self._point_started_at > POINT_TIMEOUT_SECONDS:
            if self._phase == "verify_unload":
                self._show_unload_instruction("45秒内未确认完全空载。请检查装置后重试。<br>")
            else:
                self._show_load_instruction("45秒内未达到四路全部FLOATING。请重新放置。<br>")
            self.live_label.setText("本次没有写入结果。")
            self._update_controls()
            return

        floating_count = sum(state == STATE_FLOATING for state in item["state"])
        idle_count = sum(state in IDLE_LIKE_STATES for state in item["state"])
        required_seconds = (
            UNLOADED_CONFIRM_SECONDS
            if self._phase == "verify_unload"
            else FLOATING_CONFIRM_SECONDS
        )
        condition_met = idle_count == 4 if self._phase == "verify_unload" else floating_count == 4
        if (
            self._last_confirmation_time is not None
            and now - self._last_confirmation_time > MAX_CONFIRMATION_GAP_SECONDS
        ):
            self._condition_since = None
            self._confirmation_window = []
        self._last_confirmation_time = now
        if condition_met:
            if self._condition_since is None:
                self._condition_since = now
                self._confirmation_window = [item]
            else:
                self._confirmation_window.append(item)
        else:
            self._condition_since = None
            self._confirmation_window = []
        confirmed_seconds = 0.0 if self._condition_since is None else now - self._condition_since
        self.live_label.setText(
            f"状态={list(item['state'])}；IDLE/WAIT/RECOVER={idle_count}/4；FLOATING={floating_count}/4；"
            f"连续确认={confirmed_seconds:.2f}/{required_seconds:.2f}秒，"
            f"帧数={len(self._confirmation_window)}；Raw={list(raw)}"
        )
        if (
            confirmed_seconds < required_seconds
            or len(self._confirmation_window) < MIN_CONFIRMATION_SAMPLES
        ):
            return
        if self._phase == "verify_unload" and self._sequence[self._step_index]["point_key"] != "empty":
            self._show_load_instruction()
            return
        representative_item = _confirmation_median(self._confirmation_window)
        self._finish_point(
            representative_item,
            floating_count,
            confirmed_seconds,
        )

    def _show_load_instruction(self, prefix=""):
        step = self._sequence[self._step_index]
        action = _weight_combination(step["added_mass_g"])
        self._phase = "await_load"
        self._clear_capture_state()
        self.instruction_label.setText(
            f"{prefix}第 {step['round']} 轮｜步骤 {self._step_index + 1}/{len(self._sequence)}<br>"
            "空载已经确认。现在请放置 "
            f"<span style='font-size:24px; font-weight:800; color:#ffd166;'>{action}</span>"
            f"&nbsp;&nbsp;（总质量 {step['total_mass_g']:.1f} g）<br>"
            "放稳后点击按钮，程序将检查四路全部FLOATING持续1秒并取窗口中位数。"
        )
        self.live_label.setText("等待人工确认目标载荷已经放好。")
        self._update_controls()

    def _serializable_window(self, trigger_time):
        rows = []
        for item in self._confirmation_window:
            rows.append({
                "offset_from_trigger_ms": round((item["time"] - trigger_time) * 1000.0, 3),
                "hardware_raw": list(item["hardware_raw"]),
                "baseline": list(item["baseline"]),
                "signal": list(item["signal"]),
                "fast_k_percent": list(item["fast_k_percent"]),
                "slow_k_percent": list(item["slow_k_percent"]),
                "state": list(item["state"]),
            })
        return rows

    def _reference_raw(self, round_index, point_key):
        for result in self._results:
            if result["round"] == round_index and result["point_key"] == point_key:
                return result["trigger"]["hardware_raw"]
        return None

    def _finish_point(
        self,
        trigger_item,
        floating_count,
        confirmed_seconds,
    ):
        step = dict(self._sequence[self._step_index])
        raw = list(trigger_item["raw"])
        empty_reference = raw if step["point_key"] == "empty" else self._reference_raw(step["round"], "empty")
        head_reference = raw if step["point_key"] == "head" else self._reference_raw(step["round"], "head")
        delta_empty = None if empty_reference is None else [raw[i] - empty_reference[i] for i in range(4)]
        delta_head = None if head_reference is None else [raw[i] - head_reference[i] for i in range(4)]
        result = {
            **step,
            "head_mass_g": self.head_mass_input.value(),
            "captured_at": datetime.now().isoformat(timespec="milliseconds"),
            "condition": "all_4_idle" if step["point_key"] == "empty" else "all_4_floating",
            "confirmation_duration_ms": round(confirmed_seconds * 1000.0, 3),
            "confirmation_sample_count": len(self._confirmation_window),
            "primary_value_method": "median_of_confirmation_window",
            "median_window_sample_count": trigger_item["median_window_sample_count"],
            "floating_channel_count": int(floating_count),
            "locked_range": dict(self._locked_range),
            "trigger": {
                "hardware_raw": list(trigger_item["hardware_raw"]),
                "baseline": list(trigger_item["baseline"]),
                "signal": list(trigger_item["signal"]),
                "fast_k_percent": list(trigger_item["fast_k_percent"]),
                "slow_k_percent": list(trigger_item["slow_k_percent"]),
                "state": list(trigger_item["state"]),
            },
            "delta_from_empty_raw": delta_empty,
            "delta_from_head_raw": delta_head,
            "confirmation_window": self._serializable_window(trigger_item["time"]),
        }
        max_count = max(1, int(self._locked_range["max_count"]))
        saturated_channels = [
            channel
            for channel, value in zip(range(4, 8), trigger_item["hardware_raw"])
            if value >= round(max_count * 0.98)
        ]
        firmware_raw_limited = (
            self._device_status is not None
            and bool(int(self._device_status.get("flags", 0)) & 0x20)
        )
        if saturated_channels or firmware_raw_limited:
            result["archived_at"] = datetime.now().isoformat(timespec="seconds")
            channel_text = (
                "；接近满量程通道=CH" + ",CH".join(str(channel) for channel in saturated_channels)
                if saturated_channels
                else ""
            )
            result["archive_reason"] = "固定量程接近满量程或固件报告Raw限幅" + channel_text
            self._archived_results.append(result)
            self._save_project(completed=False)
            self._show_unload_instruction("本次触发数据已归档但未采用，因为固定量程发生饱和或限幅。<br>")
            self._update_controls()
            return
        self._results.append(result)
        self._step_index += 1
        self.progress.setValue(self._step_index)
        self._refresh_table()
        self._save_project()

        if self._step_index >= len(self._sequence):
            self._complete()
            return
        self._clear_capture_state()
        self._show_unload_instruction()
        self._update_controls()

    def _complete(self):
        self._running = False
        self._phase = None
        self._project_completed = True
        self.head_mass_input.setEnabled(True)
        self.rounds_input.setEnabled(True)
        self.instruction_label.setText(
            f"采集完成：共 {len(self._results)} 个固定量程触发点。项目、触发点CSV和确认窗口CSV均已保存。"
        )
        self._save_project(completed=True)
        csv_path = self._export_csv(self._project_path.with_suffix(".csv"))
        frames_path = self._export_frames_csv(csv_path.with_name(csv_path.stem + "_frames.csv"))
        summary_path = self._export_summary_csv(csv_path.with_name(csv_path.stem + "_summary.csv"))
        self.live_label.setText(
            f"项目：{self._project_path.name}；触发点：{csv_path.name}；"
            f"确认窗口：{frames_path.name}；汇总：{summary_path.name}"
        )
        self.status_message.emit(f"固定量程真实重量标定完成：{csv_path}")
        self.operation_busy_changed.emit(False)
        self._update_controls()

    def _stop(self, message=None):
        if not self._running:
            return
        self._running = False
        self._phase = None
        self._project_completed = False
        self.head_mass_input.setEnabled(True)
        self.rounds_input.setEnabled(True)
        self._save_project(completed=False)
        text = message or "标定已停止；已完成的数据已保存，可导出CSV。"
        self.instruction_label.setText(text)
        self.status_message.emit(text)
        self.operation_busy_changed.emit(False)
        self._update_controls()

    def _new_project_path(self):
        directory = Path(__file__).resolve().with_name("weight_calibration_projects")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return directory / f"fixed_range_weight_calibration_{timestamp}{PROJECT_SUFFIX}"

    def _project_document(self, completed=False):
        return {
            "schema_version": SCHEMA_VERSION,
            "session_id": self._session_id,
            "started_at": self._started_at,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "completed": bool(completed),
            "head_mass_g": self.head_mass_input.value(),
            "available_weights_g": list(AVAILABLE_WEIGHTS_G),
            "rounds": self.rounds_input.value(),
            "sequence": self._sequence,
            "next_step_index": self._step_index,
            "locked_range": self._locked_range,
            "settings": {
                "empty_condition": "all_4_idle",
                "empty_confirm_seconds": UNLOADED_CONFIRM_SECONDS,
                "loaded_condition": "all_4_floating",
                "loaded_confirm_seconds": FLOATING_CONFIRM_SECONDS,
                "max_confirmation_gap_seconds": MAX_CONFIRMATION_GAP_SECONDS,
                "minimum_confirmation_samples": MIN_CONFIRMATION_SAMPLES,
                "primary_value": "median_of_confirmation_window",
                "point_timeout_seconds": POINT_TIMEOUT_SECONDS,
                "data_stale_abort_seconds": DATA_STALE_ABORT_SECONDS,
            },
            "results": self._results,
            "archived_results": self._archived_results,
        }

    def _save_project(self, completed=False):
        if self._project_path is None:
            return
        self._project_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._project_path.with_suffix(self._project_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self._project_document(completed), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary.replace(self._project_path)

    def _export_csv_dialog(self):
        if not self._results:
            return
        default = (
            self._project_path.with_suffix(".csv")
            if self._project_path is not None
            else self._new_project_path().with_suffix(".csv")
        )
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出重量标定结果", str(default), "CSV (*.csv)"
        )
        if path:
            exported = self._export_csv(Path(path))
            frames = self._export_frames_csv(
                exported.with_name(exported.stem + "_frames.csv")
            )
            summary = self._export_summary_csv(
                exported.with_name(exported.stem + "_summary.csv")
            )
            self.status_message.emit(
                f"重量标定结果已导出：{exported}；确认窗口：{frames}；汇总：{summary}"
            )

    def _export_csv(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            "round", "point_key", "point_name", "added_mass_g", "head_mass_g",
            "total_mass_g", "captured_at", "condition", "confirmation_duration_ms",
            "confirmation_sample_count", "primary_value_method",
            "median_window_sample_count", "floating_channel_count",
            "nsub", "tx_div",
            "ref_cdac", "max_count", "affine_active",
            "ch4_hw_raw", "ch5_hw_raw", "ch6_hw_raw",
            "ch7_hw_raw", "ch4_state", "ch5_state", "ch6_state", "ch7_state",
            "raw_sum", "delta_from_empty_sum", "delta_from_head_sum",
        ]
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for result in self._results:
                row = {key: result.get(key) for key in fields}
                trigger = result["trigger"]
                locked_range = result["locked_range"]
                for key in ("nsub", "tx_div", "ref_cdac", "max_count", "affine_active"):
                    row[key] = locked_range[key]
                for index, channel in enumerate(range(4, 8)):
                    row[f"ch{channel}_hw_raw"] = trigger["hardware_raw"][index]
                    row[f"ch{channel}_state"] = trigger["state"][index]
                row["raw_sum"] = sum(trigger["hardware_raw"])
                empty_delta = result.get("delta_from_empty_raw")
                head_delta = result.get("delta_from_head_raw")
                row["delta_from_empty_sum"] = None if empty_delta is None else sum(empty_delta)
                row["delta_from_head_sum"] = None if head_delta is None else sum(head_delta)
                writer.writerow(row)
        return path

    def _export_frames_csv(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            "round", "point_key", "point_name", "offset_from_trigger_ms",

            "ch4_hw_raw", "ch5_hw_raw", "ch6_hw_raw", "ch7_hw_raw",
            "ch4_state", "ch5_state", "ch6_state", "ch7_state",
            "ch4_baseline", "ch5_baseline", "ch6_baseline", "ch7_baseline",
            "ch4_signal", "ch5_signal", "ch6_signal", "ch7_signal",
            "ch4_fast_k", "ch5_fast_k", "ch6_fast_k", "ch7_fast_k",
            "ch4_slow_k", "ch5_slow_k", "ch6_slow_k", "ch7_slow_k",
        ]
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for result in self._results:
                for frame in result["confirmation_window"]:
                    row = {
                        "round": result["round"],
                        "point_key": result["point_key"],
                        "point_name": result["point_name"],
                        "offset_from_trigger_ms": frame["offset_from_trigger_ms"],
                    }
                    mappings = (
                        ("hw_raw", "hardware_raw"),
                        ("state", "state"),
                        ("baseline", "baseline"),
                        ("signal", "signal"),
                        ("fast_k", "fast_k_percent"),
                        ("slow_k", "slow_k_percent"),
                    )
                    for output_name, source_name in mappings:
                        for index, channel in enumerate(range(4, 8)):
                            row[f"ch{channel}_{output_name}"] = frame[source_name][index]
                    writer.writerow(row)
        return path

    def _summary_rows(self):
        grouped = {}
        previous_mean = None
        rows = []
        point_keys = ["empty", "head"] + [f"weight_{mass}" for mass in AVAILABLE_WEIGHTS_G]
        for point_key in point_keys:
            points = grouped.setdefault(point_key, [
                result for result in self._results if result["point_key"] == point_key
            ])
            if not points:
                continue
            values = [sum(point["trigger"]["hardware_raw"]) for point in points]
            mean = statistics.mean(values)
            monotonic = previous_mean is None or mean >= previous_mean
            rows.append({
                "point_key": point_key,
                "point_name": points[0]["point_name"],
                "added_mass_g": points[0]["added_mass_g"],
                "total_mass_g": points[0]["total_mass_g"],
                "completed_rounds": len(points),
                "raw_sum_mean": mean,
                "raw_sum_min": min(values),
                "raw_sum_max": max(values),
                "monotonic": monotonic,
            })
            previous_mean = mean
        return rows

    def _export_summary_csv(self, path):
        fields = [
            "point_key", "point_name", "added_mass_g", "total_mass_g", "completed_rounds",
            "raw_sum_mean", "raw_sum_min", "raw_sum_max", "monotonic",
        ]
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self._summary_rows())
        return path

    def _refresh_table(self):
        self.table.setRowCount(len(self._results))
        for row_index, result in enumerate(self._results):
            trigger = result["trigger"]
            range_config = result["locked_range"]
            values = [
                result["round"], result["point_name"],
                "--" if result["added_mass_g"] is None else result["added_mass_g"],
                f"{result['total_mass_g']:.1f}",
                *trigger["hardware_raw"],
                *[STATE_NAMES.get(int(state), str(state)) for state in trigger["state"]],
                result["floating_channel_count"],
                f"{result['confirmation_duration_ms']:.0f}",
                result["confirmation_sample_count"],
                f"N{range_config['nsub']}/T{range_config['tx_div']}/R{range_config['ref_cdac']}",
            ]
            for column, value in enumerate(values):
                self.table.setItem(row_index, column, QtWidgets.QTableWidgetItem(str(value)))
        if self._results:
            self.table.scrollToBottom()
        summary = self._summary_rows()
        self.summary_table.setRowCount(len(summary))
        for row_index, result in enumerate(summary):
            values = [
                result["point_name"],
                "--" if result["added_mass_g"] is None else result["added_mass_g"],
                f"{result['total_mass_g']:.1f}", result["completed_rounds"],
                f"{result['raw_sum_mean']:.1f}", f"{result['raw_sum_min']:.1f}",
                f"{result['raw_sum_max']:.1f}",
                "通过" if result["monotonic"] else "失败",
            ]
            for column, value in enumerate(values):
                self.summary_table.setItem(
                    row_index, column, QtWidgets.QTableWidgetItem(str(value))
                )

    def _update_controls(self):
        self.start_button.setEnabled(self._connected and not self._running)
        self.open_button.setEnabled(not self._running)
        self.continue_button.setEnabled(
            self._connected
            and not self._running
            and self._project_path is not None
            and not self._project_completed
            and len(self._results) < len(self._sequence)
        )
        self.ready_button.setEnabled(
            self._connected and self._running and self._phase in ("await_unload", "await_load")
        )
        self._update_capture_button_text()
        self.stop_button.setEnabled(self._running)
        self.export_button.setEnabled(bool(self._results))
        self.load_lut_button.setEnabled(not self._running)
        self.zero_measurement_button.setEnabled(
            self._connected and self._measurement_lut is not None and not self._running
        )

    def _update_capture_button_text(self):
        if self._phase == "await_unload":
            text = "已全部移开，确认空载"
        elif self._phase == "await_load":
            text = "已按要求放好，监测FLOATING"
        elif self._phase == "verify_unload":
            text = "正在确认空载…"
        elif self._phase == "verify_load":
            text = "正在监测FLOATING…"
        else:
            text = "按提示继续"
        self.ready_button.setText(text)
