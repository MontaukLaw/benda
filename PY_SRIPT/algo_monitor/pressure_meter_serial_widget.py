"""压力计 RS-232 Modbus RTU 独立通信测试页。"""

from __future__ import annotations

import collections
import json
import math
import os
import statistics
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from qt_compat import QtCore, QtWidgets, Signal, Slot
from serial_ports import SerialPortScanner


PRESSURE_METER_BAUD_RATE = 9600
PRESSURE_METER_SLAVE_ADDRESS = 1
PRESSURE_METER_REALTIME_REGISTER = 20
PRESSURE_METER_POLL_INTERVAL_SECONDS = 0.05
PRESSURE_METER_RESPONSE_TIMEOUT_SECONDS = 0.20
PRESSURE_METER_START_ZERO_TOLERANCE_G = 5.0
KGF_TO_NEWTON = 9.80665
GF_TO_NEWTON = KGF_TO_NEWTON / 1000.0
PRESSURE_METER_START_ZERO_TOLERANCE_N = 0.05


def pressure_is_within_start_zero_tolerance(value):
    """判断兼容旧模块的克力值能否按启动零点处理。"""
    value = float(value)
    return math.isfinite(value) and abs(value) <= PRESSURE_METER_START_ZERO_TOLERANCE_G


def pressure_n_is_within_start_zero_tolerance(value):
    """判断压力计的 N 值能否按启动零点处理。"""
    value = float(value)
    return math.isfinite(value) and abs(value) <= PRESSURE_METER_START_ZERO_TOLERANCE_N


def pressure_meter_settings_path():
    """返回压力计串口设置文件路径。"""
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "pressure_meter_serial_config.json"
    return Path(__file__).resolve().with_name("pressure_meter_serial_config.json")


class PressureMeterProtocolError(RuntimeError):
    """表示压力计返回了不完整或无效的 Modbus RTU 响应。"""


def modbus_crc16(data):
    """计算 Modbus RTU CRC16，返回未拆分的 16 位整数。"""
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_realtime_force_request():
    """生成读取地址 20 起始两个寄存器的 Modbus RTU 请求帧。"""
    payload = bytes(
        (
            PRESSURE_METER_SLAVE_ADDRESS,
            0x03,
            (PRESSURE_METER_REALTIME_REGISTER >> 8) & 0xFF,
            PRESSURE_METER_REALTIME_REGISTER & 0xFF,
            0x00,
            0x02,
        )
    )
    return payload + modbus_crc16(payload).to_bytes(2, byteorder="little")


PRESSURE_METER_REALTIME_REQUEST = build_realtime_force_request()


def parse_realtime_force_response(frame):
    """按文档规定的 16 位字交换顺序解析实时力值浮点数。"""
    frame = bytes(frame)
    if len(frame) != 9:
        raise PressureMeterProtocolError(
            f"响应长度应为 9 字节，实际为 {len(frame)} 字节"
        )

    expected_crc = modbus_crc16(frame[:-2])
    received_crc = int.from_bytes(frame[-2:], byteorder="little")
    if received_crc != expected_crc:
        raise PressureMeterProtocolError(
            f"CRC错误：收到 {received_crc:04X}，应为 {expected_crc:04X}"
        )
    if frame[0] != PRESSURE_METER_SLAVE_ADDRESS:
        raise PressureMeterProtocolError(f"从机地址不是 1：{frame[0]}")
    if frame[1] != 0x03:
        raise PressureMeterProtocolError(f"功能码不是 03：{frame[1]:02X}")
    if frame[2] != 0x04:
        raise PressureMeterProtocolError(f"数据长度不是 4：{frame[2]}")

    register_bytes = frame[3:7]
    ieee754_bytes = register_bytes[2:4] + register_bytes[0:2]
    value = struct.unpack(">f", ieee754_bytes)[0]
    if not math.isfinite(value):
        raise PressureMeterProtocolError(f"浮点值无效：{value}")
    return float(value)


def classify_force_unit(raw_delta, reference_mass_g):
    """将已知重量引起的原始变化量与 g、kgf、N 三种单位进行比较。"""
    raw_delta = abs(float(raw_delta))
    reference_mass_g = float(reference_mass_g)
    if reference_mass_g <= 0.0:
        raise ValueError("参考重量必须大于 0 g")

    expected_values = {
        "g": reference_mass_g,
        "kgf": reference_mass_g / 1000.0,
        "N": reference_mass_g * KGF_TO_NEWTON / 1000.0,
    }
    candidates = []
    for unit, expected in expected_values.items():
        relative_error = abs(raw_delta - expected) / expected
        candidates.append((relative_error, unit, expected))
    candidates.sort(key=lambda item: item[0])
    best_error, best_unit, expected = candidates[0]
    return {
        "unit": best_unit,
        "relative_error": best_error,
        "expected": expected,
        "raw_delta": raw_delta,
        "candidates": tuple(candidates),
    }


class PressureMeterReaderThread(QtCore.QThread):
    """在独立线程中以 20 Hz 请求压力计实时值。"""

    port_opened = Signal(str)
    sample_received = Signal(object)
    transaction_failed = Signal(str)
    fatal_error = Signal(str)

    def __init__(self, port_name, parent=None):
        super().__init__(parent)
        self.port_name = str(port_name)
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    @staticmethod
    def _extract_response(buffer):
        while buffer and buffer[0] != PRESSURE_METER_SLAVE_ADDRESS:
            del buffer[0]
        if len(buffer) < 2:
            return None

        function_code = buffer[1]
        if function_code == (0x03 | 0x80):
            frame_length = 5
        elif function_code == 0x03:
            if len(buffer) < 3:
                return None
            frame_length = 3 + buffer[2] + 2
            if frame_length > 64:
                del buffer[0]
                return None
        else:
            del buffer[0]
            return None

        if len(buffer) < frame_length:
            return None
        frame = bytes(buffer[:frame_length])
        del buffer[:frame_length]
        return frame

    def _read_response(self, serial_port):
        deadline = time.monotonic() + PRESSURE_METER_RESPONSE_TIMEOUT_SECONDS
        buffer = bytearray()
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            waiting = serial_port.in_waiting
            chunk = serial_port.read(waiting if waiting else 1)
            if not chunk:
                continue
            buffer.extend(chunk)
            frame = self._extract_response(buffer)
            if frame is None:
                continue
            if frame[1] & 0x80:
                if modbus_crc16(frame[:-2]) != int.from_bytes(
                    frame[-2:], byteorder="little"
                ):
                    raise PressureMeterProtocolError("异常响应 CRC 错误")
                raise PressureMeterProtocolError(
                    f"Modbus异常响应，异常码 0x{frame[2]:02X}"
                )
            return frame

        partial = " ".join(f"{byte:02X}" for byte in buffer) or "无数据"
        raise TimeoutError(f"响应超时，已收到：{partial}")

    def run(self):
        try:
            import serial
        except ImportError:
            self.fatal_error.emit("pyserial 未安装，无法打开压力计串口")
            return

        serial_port = None
        try:
            serial_port = serial.Serial(
                port=self.port_name,
                baudrate=PRESSURE_METER_BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
                write_timeout=0.20,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            serial_port.reset_input_buffer()
            serial_port.reset_output_buffer()
            self.port_opened.emit(self.port_name)

            next_poll_at = time.monotonic()
            while not self._stop_event.is_set():
                wait_seconds = next_poll_at - time.monotonic()
                if wait_seconds > 0.0 and self._stop_event.wait(wait_seconds):
                    break

                started_at = time.monotonic()
                try:
                    serial_port.write(PRESSURE_METER_REALTIME_REQUEST)
                    serial_port.flush()
                    response = self._read_response(serial_port)
                    value = parse_realtime_force_response(response)
                    completed_at = time.monotonic()
                    self.sample_received.emit(
                        {
                            "received_at": completed_at,
                            "wall_time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                            "latency_ms": (completed_at - started_at) * 1000.0,
                            "raw_value": value,
                            "request": PRESSURE_METER_REALTIME_REQUEST,
                            "response": response,
                        }
                    )
                except (OSError, TimeoutError, PressureMeterProtocolError) as exc:
                    if not self._stop_event.is_set():
                        self.transaction_failed.emit(str(exc))

                next_poll_at += PRESSURE_METER_POLL_INTERVAL_SECONDS
                now = time.monotonic()
                if next_poll_at < now:
                    next_poll_at = now
        except (OSError, ValueError) as exc:
            if not self._stop_event.is_set():
                self.fatal_error.emit(f"打开或访问 {self.port_name} 失败：{exc}")
        finally:
            if serial_port is not None:
                try:
                    serial_port.close()
                except OSError:
                    pass


class PressureMeterSerialWidget(QtWidgets.QWidget):
    """提供端口控制、实时帧诊断和压力单位实测判定。"""

    status_message = Signal(str)
    pressure_n_changed = Signal(object, str)
    # 旧模块尚以克力工作，此信号在迁移期间发送 N 换算后的克力值。
    pressure_changed = Signal(object, str)
    force_sampled = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scanner = SerialPortScanner()
        self._settings_path = pressure_meter_settings_path()
        self._last_working_port = self._load_last_working_port()
        self._saved_port_confirmed_this_run = False
        self._worker = None
        self._closing_requested = False
        self._last_fatal_error = None
        self._success_count = 0
        self._failure_count = 0
        self._force_sequence = 0
        self._sample_times = collections.deque(maxlen=200)
        self._latencies_ms = collections.deque(maxlen=100)
        self._zero_baseline = None
        self._capture_kind = None
        self._capture_values = []

        self._capture_timer = QtCore.QTimer(self)
        self._capture_timer.setSingleShot(True)
        self._capture_timer.setInterval(1000)
        self._capture_timer.timeout.connect(self._finish_capture)

        self._build_ui()
        self.refresh_ports()
        self._refresh_connection_controls()
        QtCore.QTimer.singleShot(100, self._auto_connect_last_working_port)

    def _load_last_working_port(self):
        try:
            payload = json.loads(self._settings_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            return ""
        port_name = payload.get("last_working_port", "")
        return str(port_name).strip() if port_name is not None else ""

    def _save_last_working_port(self, port_name):
        port_name = str(port_name).strip()
        if not port_name:
            return
        self._saved_port_confirmed_this_run = True
        try:
            self._settings_path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = self._settings_path.with_suffix(".json.tmp")
            temporary_path.write_text(
                json.dumps(
                    {"last_working_port": port_name},
                    ensure_ascii=False,
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            temporary_path.replace(self._settings_path)
        except OSError as exc:
            self._append_log(f"保存好用COM口失败：{exc}")
            return
        self._last_working_port = port_name
        self._append_log(f"已确认并记住好用COM口：{port_name}")

    def remembered_port(self):
        """返回上次成功读取到压力值的 COM 口。"""
        return self._last_working_port

    @staticmethod
    def _hex_text(data):
        return " ".join(f"{byte:02X}" for byte in bytes(data))

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        no_frame = getattr(QtWidgets.QFrame, "NoFrame", QtWidgets.QFrame.Shape.NoFrame)
        scroll.setFrameShape(no_frame)
        outer.addWidget(scroll)

        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        scroll.setWidget(content)

        connection_group = QtWidgets.QGroupBox("RS-232 / Modbus RTU 连接")
        connection_layout = QtWidgets.QVBoxLayout(connection_group)
        port_row = QtWidgets.QHBoxLayout()
        port_row.addWidget(QtWidgets.QLabel("COM口"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumContentsLength(16)
        port_row.addWidget(self.port_combo, 1)
        self.refresh_button = QtWidgets.QPushButton("刷新")
        self.refresh_button.clicked.connect(self.refresh_ports)
        port_row.addWidget(self.refresh_button)
        self.connect_button = QtWidgets.QPushButton("打开")
        self.connect_button.clicked.connect(self.toggle_connection)
        port_row.addWidget(self.connect_button)
        connection_layout.addLayout(port_row)

        protocol_label = QtWidgets.QLabel(
            "固定参数：9600 bps · 8N1 · 从机 1 · 功能码 03 · "
            "寄存器 20～21 · 20 Hz轮询"
        )
        protocol_label.setWordWrap(True)
        connection_layout.addWidget(protocol_label)
        request_label = QtWidgets.QLabel(
            f"只读请求：{self._hex_text(PRESSURE_METER_REALTIME_REQUEST)}"
        )
        request_label.setTextInteractionFlags(
            getattr(
                QtCore.Qt,
                "TextSelectableByMouse",
                QtCore.Qt.TextInteractionFlag.TextSelectableByMouse,
            )
        )
        connection_layout.addWidget(request_label)
        self.connection_status_label = QtWidgets.QLabel("串口未打开")
        self.connection_status_label.setWordWrap(True)
        connection_layout.addWidget(self.connection_status_label)
        layout.addWidget(connection_group)

        value_group = QtWidgets.QGroupBox("实时值")
        value_layout = QtWidgets.QGridLayout(value_group)
        value_layout.addWidget(QtWidgets.QLabel("串口实时压力（控制使用）"), 0, 0)
        self.raw_value_label = QtWidgets.QLabel("--")
        self.raw_value_label.setStyleSheet(
            "font-size:24px; font-weight:700; color:#57d5ff;"
        )
        value_layout.addWidget(self.raw_value_label, 0, 1)
        self.zero_warning_label = QtWidgets.QLabel()
        self.zero_warning_label.setWordWrap(True)
        self.zero_warning_label.setStyleSheet(
            "color:#ffffff; background:#8b1e1e; border:2px solid #ff6b6b;"
            "border-radius:5px; padding:7px 9px; font-size:15px; font-weight:800;"
        )
        self.zero_warning_label.setVisible(False)
        value_layout.addWidget(self.zero_warning_label, 1, 0, 1, 2)
        value_layout.addWidget(QtWidgets.QLabel("换算克力（兼容旧模块）"), 2, 0)
        self.kgf_mass_label = QtWidgets.QLabel("-- g")
        value_layout.addWidget(self.kgf_mass_label, 2, 1)
        value_layout.addWidget(QtWidgets.QLabel("换算千克力"), 3, 0)
        self.direct_mass_label = QtWidgets.QLabel("-- kgf")
        value_layout.addWidget(self.direct_mass_label, 3, 1)
        value_layout.addWidget(QtWidgets.QLabel("确认的压力单位"), 4, 0)
        self.newton_mass_label = QtWidgets.QLabel("N")
        value_layout.addWidget(self.newton_mass_label, 4, 1)
        value_layout.addWidget(QtWidgets.QLabel("实测接收频率"), 5, 0)
        self.frequency_label = QtWidgets.QLabel("0.0 Hz")
        value_layout.addWidget(self.frequency_label, 5, 1)
        value_layout.addWidget(QtWidgets.QLabel("平均响应延迟"), 6, 0)
        self.latency_label = QtWidgets.QLabel("-- ms")
        value_layout.addWidget(self.latency_label, 6, 1)
        value_layout.addWidget(QtWidgets.QLabel("成功 / 失败"), 7, 0)
        self.count_label = QtWidgets.QLabel("0 / 0")
        value_layout.addWidget(self.count_label, 7, 1)
        value_layout.addWidget(QtWidgets.QLabel("最后响应"), 8, 0)
        self.response_label = QtWidgets.QLabel("--")
        self.response_label.setWordWrap(True)
        self.response_label.setTextInteractionFlags(
            getattr(
                QtCore.Qt,
                "TextSelectableByMouse",
                QtCore.Qt.TextInteractionFlag.TextSelectableByMouse,
            )
        )
        value_layout.addWidget(self.response_label, 8, 1)
        layout.addWidget(value_group)

        unit_group = QtWidgets.QGroupBox("N 单位校验")
        unit_layout = QtWidgets.QVBoxLayout(unit_group)
        unit_note = QtWidgets.QLabel(
            "先在空载状态记录 1 秒基线，再放上已知重量并采集 1 秒。"
            "当前按 N 使用；校验工具仍会比较 g、kgf 和 N 三种解释的误差。"
        )
        unit_note.setWordWrap(True)
        unit_layout.addWidget(unit_note)

        reference_row = QtWidgets.QHBoxLayout()
        reference_row.addWidget(QtWidgets.QLabel("已知重量（g）"))
        self.reference_mass_input = QtWidgets.QDoubleSpinBox()
        self.reference_mass_input.setRange(0.01, 50000.0)
        self.reference_mass_input.setDecimals(2)
        self.reference_mass_input.setValue(100.0)
        reference_row.addWidget(self.reference_mass_input)
        self.capture_zero_button = QtWidgets.QPushButton("1. 记录空载")
        self.capture_zero_button.clicked.connect(self.capture_zero_baseline)
        reference_row.addWidget(self.capture_zero_button)
        self.classify_unit_button = QtWidgets.QPushButton("2. 放砝码并判断")
        self.classify_unit_button.clicked.connect(self.capture_loaded_value)
        reference_row.addWidget(self.classify_unit_button)
        reference_row.addStretch(1)
        unit_layout.addLayout(reference_row)

        self.unit_result_label = QtWidgets.QLabel(
            "当前配置：本机 Modbus 实时值直接以 N 为单位。"
        )
        self.unit_result_label.setWordWrap(True)
        unit_layout.addWidget(self.unit_result_label)
        layout.addWidget(unit_group)

        log_group = QtWidgets.QGroupBox("通信记录")
        log_layout = QtWidgets.QVBoxLayout(log_group)
        log_button_row = QtWidgets.QHBoxLayout()
        log_button_row.addStretch(1)
        clear_button = QtWidgets.QPushButton("清空")
        clear_button.clicked.connect(self._clear_log)
        log_button_row.addWidget(clear_button)
        log_layout.addLayout(log_button_row)
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.document().setMaximumBlockCount(500)
        self.log_text.setMinimumHeight(180)
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group)
        layout.addStretch(1)

    def _append_log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.appendPlainText(f"[{timestamp}] {message}")

    def _clear_log(self):
        self.log_text.clear()

    def _show_invalid_pressure(self, reason):
        self.raw_value_label.setText("--.-- N")
        self.raw_value_label.setStyleSheet(
            "font-size:24px; font-weight:700; color:#ff7b72;"
        )
        self.zero_warning_label.setText(
            f"⚠ 当前没有可用于控制的压力值：{reason}"
        )
        self.zero_warning_label.setVisible(True)

    def _emit_force_sample(
        self,
        raw_force_n,
        accepted,
        reason="",
        response=b"",
        sampled_monotonic=None,
    ):
        """发送 N 原始样本，并为尚未迁移的模块提供克力兼容字段。"""
        self._force_sequence += 1
        sampled_at = (
            time.monotonic()
            if sampled_monotonic is None
            else float(sampled_monotonic)
        )
        value_n = None if raw_force_n is None else round(float(raw_force_n), 2)
        value_g = None if value_n is None else value_n / GF_TO_NEWTON
        response_text = self._hex_text(response) if response else ""
        self.force_sampled.emit(
            {
                "source": "pressure_meter_rs232",
                "sequence": self._force_sequence,
                "sampled_monotonic": sampled_at,
                "force_unit": "N",
                "raw_force_n": value_n,
                "control_raw_force_n": value_n,
                "force_n": value_n,
                # 旧字段只用于尚未迁移的按力控制模块，数值已由 N 换算为克力。
                "raw_force_g": value_g,
                "control_raw_force_g": value_g,
                "force_g": value_g,
                "filtered_force_g": value_g,
                "accepted": bool(accepted),
                "invalidate": not bool(accepted),
                "rejection_reason": str(reason),
                "failure_image_path": "",
                "raw_text": response_text,
            }
        )
        if accepted and not pressure_n_is_within_start_zero_tolerance(value_n):
            status = (
                f"压力计当前为 {value_n:.2f} N，超出启动零点容差 "
                f"±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N；开始下压前请先清零"
            )
        elif accepted:
            status = (
                f"压力计串口读取正常，当前 {value_n:.2f} N，"
                f"处于启动零点容差 ±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N 内"
            )
        else:
            status = str(reason)
        self.pressure_n_changed.emit(value_n if accepted else None, status)
        self.pressure_changed.emit(value_g if accepted else None, status)

    @Slot()
    def refresh_ports(self):
        selected_port = self.port_combo.currentData()
        preferred_port = selected_port or self._last_working_port
        ports, message = self._scanner.list_ports()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for device, description in ports:
            self.port_combo.addItem(description, device)
        if preferred_port:
            index = self.port_combo.findData(preferred_port)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        self.port_combo.blockSignals(False)
        self.connection_status_label.setText(message)
        self._refresh_connection_controls()

    @Slot()
    def _auto_connect_last_working_port(self):
        if self._worker is not None or not self._last_working_port:
            return
        index = self.port_combo.findData(self._last_working_port)
        if index < 0:
            self.connection_status_label.setText(
                f"已记住 {self._last_working_port}，当前未检测到该端口"
            )
            return
        self.port_combo.setCurrentIndex(index)
        self._append_log(f"自动连接上次好用端口：{self._last_working_port}")
        self.toggle_connection()

    def _refresh_connection_controls(self):
        running = self._worker is not None and self._worker.isRunning()
        stopping = running and self._closing_requested
        self.port_combo.setEnabled(not running)
        self.refresh_button.setEnabled(not running)
        self.connect_button.setEnabled(running or self.port_combo.currentData() is not None)
        self.connect_button.setText("正在关闭…" if stopping else ("关闭" if running else "打开"))

        capture_enabled = running and not stopping and self._capture_kind is None
        self.capture_zero_button.setEnabled(capture_enabled)
        self.classify_unit_button.setEnabled(
            capture_enabled and self._zero_baseline is not None
        )
        self.reference_mass_input.setEnabled(self._capture_kind is None)

    @Slot()
    def toggle_connection(self):
        if self._worker is not None and self._worker.isRunning():
            self._closing_requested = True
            self._worker.stop()
            self._capture_timer.stop()
            self._capture_kind = None
            self._capture_values = []
            self.connection_status_label.setText("正在关闭串口…")
            self._refresh_connection_controls()
            return

        port_name = self.port_combo.currentData()
        if not port_name:
            self.connection_status_label.setText("请先选择 COM 口")
            return

        self._success_count = 0
        self._failure_count = 0
        self._sample_times.clear()
        self._latencies_ms.clear()
        self._last_fatal_error = None
        self._closing_requested = False
        self._saved_port_confirmed_this_run = False
        self._zero_baseline = None
        self.count_label.setText("0 / 0")
        self.frequency_label.setText("0.0 Hz")
        self.latency_label.setText("-- ms")
        self.connection_status_label.setText(f"正在打开 {port_name}…")
        self.connection_status_label.setStyleSheet("")
        self.unit_result_label.setText("当前配置：本机 Modbus 实时值直接以 N 为单位。")
        self.unit_result_label.setStyleSheet("")
        self._append_log(
            f"打开 {port_name}，9600 8N1；TX {self._hex_text(PRESSURE_METER_REALTIME_REQUEST)}"
        )

        worker = PressureMeterReaderThread(port_name, self)
        worker.port_opened.connect(self._on_port_opened)
        worker.sample_received.connect(self._on_sample_received)
        worker.transaction_failed.connect(self._on_transaction_failed)
        worker.fatal_error.connect(self._on_fatal_error)
        worker.finished.connect(self._on_worker_finished)
        self._worker = worker
        worker.start()
        self._refresh_connection_controls()

    @Slot(str)
    def _on_port_opened(self, port_name):
        message = f"{port_name} 已打开，正在以 20 Hz 读取实时力值"
        self.connection_status_label.setText(message)
        self.connection_status_label.setStyleSheet("color:#76e36f;")
        self.status_message.emit(message)
        self._append_log(message)
        self._refresh_connection_controls()

    @Slot(object)
    def _on_sample_received(self, sample):
        raw_value = float(sample["raw_value"])
        received_at = float(sample["received_at"])
        latency_ms = float(sample["latency_ms"])
        response = bytes(sample["response"])

        self._success_count += 1
        if (
            self._worker is not None
            and not self._saved_port_confirmed_this_run
        ):
            self._save_last_working_port(self._worker.port_name)
        self._sample_times.append(received_at)
        while self._sample_times and received_at - self._sample_times[0] > 1.0:
            self._sample_times.popleft()
        self._latencies_ms.append(latency_ms)
        if self._capture_kind is not None:
            self._capture_values.append(raw_value)

        frequency = 0.0
        if len(self._sample_times) >= 2:
            span = self._sample_times[-1] - self._sample_times[0]
            if span > 0.0:
                frequency = (len(self._sample_times) - 1) / span

        self.raw_value_label.setText(f"{raw_value:.2f} N")
        self.raw_value_label.setStyleSheet(
            "font-size:24px; font-weight:700; "
            f"color:{'#ff7b72' if raw_value < -PRESSURE_METER_START_ZERO_TOLERANCE_N else '#57d5ff'};"
        )
        if not pressure_n_is_within_start_zero_tolerance(raw_value):
            qualifier = "负压力" if raw_value < 0.0 else "非零压力"
            self.zero_warning_label.setText(
                f"⚠ 检测到{qualifier} {raw_value:.2f} N。"
                f"已超出启动零点容差 ±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N，"
                "若准备开始下压，请先在压力计上执行清零。"
                "程序不会把负数截断为0，也不会取绝对值。"
            )
            self.zero_warning_label.setVisible(True)
        else:
            self.zero_warning_label.clear()
            self.zero_warning_label.setVisible(False)
        self.kgf_mass_label.setText(f"{raw_value / GF_TO_NEWTON:.2f} g")
        self.direct_mass_label.setText(f"{raw_value / KGF_TO_NEWTON:.2f} kgf")
        self.newton_mass_label.setText("N")
        self.frequency_label.setText(f"{frequency:.1f} Hz")
        self.latency_label.setText(
            f"{statistics.fmean(self._latencies_ms):.1f} ms"
        )
        self.count_label.setText(f"{self._success_count} / {self._failure_count}")
        response_text = self._hex_text(response)
        self.response_label.setText(response_text)
        self._emit_force_sample(
            raw_value,
            True,
            response=response,
            sampled_monotonic=received_at,
        )
        self._append_log(
            f"RX {response_text} · pressure={raw_value:.2f} N · latency={latency_ms:.1f} ms"
        )
        if self._worker is not None:
            self.connection_status_label.setText(
                f"{self._worker.port_name} 已打开，正在以 20 Hz 读取实时力值"
            )
            self.connection_status_label.setStyleSheet("color:#76e36f;")

    @Slot(str)
    def _on_transaction_failed(self, message):
        self._failure_count += 1
        self.count_label.setText(f"{self._success_count} / {self._failure_count}")
        self.connection_status_label.setText(f"串口已打开，但本次读取失败：{message}")
        self.connection_status_label.setStyleSheet("color:#ffc857;")
        self._append_log(f"读取失败：{message}")
        self._show_invalid_pressure(message)
        self._emit_force_sample(None, False, f"压力计串口读取失败：{message}")

    @Slot(str)
    def _on_fatal_error(self, message):
        self._last_fatal_error = str(message)
        self.connection_status_label.setText(message)
        self.connection_status_label.setStyleSheet("color:#ff7b72;")
        self.status_message.emit(message)
        self._append_log(message)
        self._show_invalid_pressure(message)
        self._emit_force_sample(None, False, message)

    @Slot()
    def _on_worker_finished(self):
        worker = self._worker
        self._worker = None
        self._capture_timer.stop()
        self._capture_kind = None
        self._capture_values = []
        if self._last_fatal_error is None:
            self.connection_status_label.setText("串口已关闭")
            self.connection_status_label.setStyleSheet("")
            self._append_log("串口已关闭")
            self._show_invalid_pressure("压力计串口已关闭")
            self._emit_force_sample(None, False, "压力计串口已关闭")
        self._closing_requested = False
        self._refresh_connection_controls()
        if worker is not None:
            worker.deleteLater()

    @Slot()
    def capture_zero_baseline(self):
        self._start_capture("zero")

    @Slot()
    def capture_loaded_value(self):
        self._start_capture("loaded")

    def _start_capture(self, kind):
        if self._worker is None or not self._worker.isRunning():
            self.unit_result_label.setText("请先打开压力计串口。")
            return
        if self._capture_kind is not None:
            return
        if kind == "loaded" and self._zero_baseline is None:
            self.unit_result_label.setText("请先在空载状态执行第 1 步。")
            return
        self._capture_kind = str(kind)
        self._capture_values = []
        self.unit_result_label.setStyleSheet("")
        if kind == "zero":
            self.unit_result_label.setText("正在采集 1 秒空载基线…")
        else:
            self.unit_result_label.setText("正在采集 1 秒加载值并判断单位…")
        self._capture_timer.start()
        self._refresh_connection_controls()

    @Slot()
    def _finish_capture(self):
        kind = self._capture_kind
        values = tuple(self._capture_values)
        self._capture_kind = None
        self._capture_values = []
        self._refresh_connection_controls()

        if len(values) < 5:
            self.unit_result_label.setText(
                f"采集失败：1 秒内只有 {len(values)} 个有效值，请检查通信。"
            )
            return

        median_value = float(statistics.median(values))
        spread = max(values) - min(values)
        if kind == "zero":
            self._zero_baseline = median_value
            self._refresh_connection_controls()
            self.unit_result_label.setText(
                f"空载基线已记录：{median_value:.2f} N，"
                f"1 秒波动范围={spread:.2f} N。现在放上已知重量后点击第 2 步。"
            )
            self._append_log(
                f"空载基线={median_value:.2f} N samples={len(values)} "
                f"spread={spread:.2f} N"
            )
            return

        reference_mass_g = float(self.reference_mass_input.value())
        result = classify_force_unit(
            median_value - self._zero_baseline,
            reference_mass_g,
        )
        error_percent = result["relative_error"] * 100.0
        if error_percent <= 15.0:
            confidence_text = "可以判定"
            color = "#76e36f"
        else:
            confidence_text = "暂时无法可靠判定"
            color = "#ffc857"
        candidate_text = "；".join(
            f"{unit}误差{relative_error * 100.0:.1f}%"
            for relative_error, unit, _expected in result["candidates"]
        )
        self.unit_result_label.setText(
            f"{confidence_text}：最接近 {result['unit']}。"
            f"空载={self._zero_baseline:.2f} N，加载中值={median_value:.2f} N，"
            f"变化量={result['raw_delta']:.2f} N；{candidate_text}。"
        )
        self.unit_result_label.setStyleSheet(f"color:{color};")
        self._append_log(
            f"单位判定 reference={reference_mass_g:.2f}g result={result['unit']} "
            f"error={error_percent:.1f}% samples={len(values)}"
        )

    def shutdown(self):
        """关闭窗口前停止压力计串口线程并等待串口句柄释放。"""
        self._capture_timer.stop()
        worker = self._worker
        if worker is None:
            return
        worker.stop()
        if not worker.wait(1500):
            worker.wait()
        self._worker = None
