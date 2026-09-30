"""新电机独立测试页；所有CAN访问在唯一后台线程执行。"""

from collections import deque
import queue
import threading
import time

from qt_compat import QtCore, QtWidgets, Signal
from zdt_x42s_can_driver import ZDTX42SMotor


class ZDTMotorTestWidget(QtWidgets.QWidget):
    status_received = Signal(object)
    link_changed = Signal(bool)
    command_finished = Signal(str, str)
    communication_failed = Signal(str)
    homing_parameters_received = Signal(object)
    homing_phase_changed = Signal(str)
    control_state_changed = Signal(bool, bool)
    coordinate_invalidated = Signal()
    POLL_INTERVAL = 2.0
    MOTION_POLL_INTERVAL = 0.05
    IDLE_STALE_SECONDS = 2.5
    STALE_SECONDS = 0.5

    def __init__(self, config, parent=None, motor_factory=ZDTX42SMotor):
        super().__init__(parent)
        self.config = config
        self._motor_factory = motor_factory
        self._tasks = queue.PriorityQueue()
        self._task_sequence = 0
        self._pending_commands = 0
        self._stop_queued = False
        self._cancel_motion = threading.Event()
        self._stop = threading.Event()
        self._connected = False
        self._busy = False
        self._last_sample = None
        self._sample_times = deque(maxlen=30)
        self._closing = False
        self._homing_parameters = None
        self._home_phase = "idle"
        self._home_seen_active = False
        self._home_may_be_active = threading.Event()
        self._motion_polling = False
        self._external_polling = False
        self._motion_poll_not_before = 0.0
        self._confirming_stop = False
        self._build_ui()
        self.status_received.connect(self._show_status)
        self.link_changed.connect(self._show_connection)
        self.command_finished.connect(self._finish_command)
        self.communication_failed.connect(self._show_error)
        self.homing_parameters_received.connect(self._show_homing_parameters)
        self.homing_phase_changed.connect(self._set_home_phase)
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._refresh_freshness)
        self._timer.start()
        self._thread = threading.Thread(target=self._service_loop, name="zdt-x42s-can", daemon=True)
        self._thread.start()
        self._refresh_controls()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        self.value_labels = {}
        self.header_layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.Direction.LeftToRight)
        self.header_layout.setSpacing(18)
        info = QtWidgets.QWidget()
        info_layout = QtWidgets.QVBoxLayout(info)
        info_layout.setContentsMargins(0, 0, 0, 0)
        self.header_layout.addWidget(info, 1)
        self._build_summary_ui()
        self.header_layout.addWidget(self.summary_panel, 1)
        layout.addLayout(self.header_layout)
        title = QtWidgets.QLabel("ZDT X42S 新电机 · 通信与状态测试")
        title.setWordWrap(True)
        title.setStyleSheet("font-size:18px; font-weight:600;")
        info_layout.addWidget(title)
        adapter_text = (
            f"candleLight USB CAN / 设备 {self.config.channel}（gs_usb）"
            if self.config.interface == "gs_usb"
            else f"{self.config.interface} / {self.config.channel}"
        )
        config_text = QtWidgets.QLabel(
            f"{adapter_text}    CAN {self.config.bitrate / 1000:g} kbit/s    "
            f"地址 {self.config.node_id}    扩展帧起始 ID 0x{self.config.node_id << 8:04X}\n"
            "自由协议 / 固定校验 0x6B    空闲每2秒查询 · 运动期间快速检测    固件格式自动识别 Emm / X"
        )
        config_text.setWordWrap(True)
        info_layout.addWidget(config_text)
        setup = QtWidgets.QLabel(
            "电机设置：端口复用选择 CAN1_MAP，Checksum 选择 0x6B，Response 选择 Receive 或 Both。\n"
            "连接仅开始读取状态。使能会锁轴，失能会松轴；位置测试需先使能电机。"
        )
        setup.setWordWrap(True)
        info_layout.addWidget(setup)
        row = QtWidgets.QHBoxLayout()
        self.connect_button = QtWidgets.QPushButton("连接 CAN")
        self.connect_button.clicked.connect(self._toggle_connection)
        self.enable_button = QtWidgets.QPushButton("使能电机（锁轴）")
        self.enable_button.clicked.connect(lambda: self._submit("enable"))
        self.disable_button = QtWidgets.QPushButton("失能电机（松轴）")
        self.disable_button.clicked.connect(lambda: self._submit("disable"))
        for button in (self.connect_button, self.enable_button, self.disable_button):
            row.addWidget(button)
        row.addStretch()
        info_layout.addLayout(row)
        info_layout.addStretch()
        self._build_motion_ui(layout)
        self._build_homing_ui(layout)
        calibration = QtWidgets.QGroupBox("编码器校准")
        calibration_layout = QtWidgets.QVBoxLayout(calibration)
        note = QtWidgets.QLabel("校准会缓慢正转一圈、反转一圈。必须空载，并确保转动范围内无机械阻挡。")
        note.setWordWrap(True)
        calibration_layout.addWidget(note)
        self.unloaded_checkbox = QtWidgets.QCheckBox("已确认电机空载，正反转一圈的行程安全")
        self.unloaded_checkbox.toggled.connect(self._refresh_controls)
        calibration_layout.addWidget(self.unloaded_checkbox)
        self.calibrate_button = QtWidgets.QPushButton("触发编码器校准")
        self.calibrate_button.clicked.connect(self._calibrate)
        calibration_layout.addWidget(self.calibrate_button)
        layout.addWidget(calibration)
        self.connection_label = QtWidgets.QLabel("CAN 未连接")
        self.freshness_label = QtWidgets.QLabel("尚未收到状态")
        self.command_label = QtWidgets.QLabel("等待操作")
        self.command_label.setWordWrap(True)
        layout.addWidget(self.connection_label)
        layout.addWidget(self.freshness_label)
        layout.addWidget(self.command_label)
        group = QtWidgets.QGroupBox("实时反馈（全部来自电机）")
        grid = QtWidgets.QGridLayout(group)
        fields = (
            ("firmware", "固件格式"), ("position_raw", "位置原始值"),
            ("voltage_mv", "总线电压"), ("bus_current_ma", "总线电流"),
            ("target_deg", "目标位置"), ("error_deg", "位置误差"),
            ("encoder", "单圈编码器值"), ("temperature_raw", "温度原始字节"),
            ("enabled", "使能状态"), ("in_position", "到位状态"),
            ("encoder_ready", "编码器就绪"), ("calibration_ready", "校准表就绪"),
            ("stall", "堵转"), ("stall_protection", "堵转保护"),
            ("overtemperature", "过温保护"), ("overcurrent", "过流保护"),
            ("state_flags", "电机状态位"), ("home_flags", "回零/保护状态位"),
            ("homing", "正在回零"), ("home_failed", "回零失败标志"),
        )
        for index, (key, label) in enumerate(fields):
            row, column = divmod(index, 2)
            grid.addWidget(QtWidgets.QLabel(label), row, column * 2)
            value = QtWidgets.QLabel("—")
            value.setMinimumWidth(90)
            grid.addWidget(value, row, column * 2 + 1)
            self.value_labels[key] = value
        layout.addWidget(group)
        footnote = QtWidgets.QLabel(
            "校准命令应答只表示已接收，校准表就绪不等于本次校准完成；请同时观察电机。\n"
            f"Emm 温度符号：{self.config.temperature_positive_sign:02X}正、"
            f"{1 - self.config.temperature_positive_sign:02X}负；X 温度符号：00正、01负。\n"
            "切换旧电机：修改 motor_connection_config.json 的 motor_model 为 pd42s1，然后重启 App。"
        )
        footnote.setWordWrap(True)
        layout.addWidget(footnote)
        layout.addStretch()

    def _build_summary_ui(self):
        self.summary_panel = QtWidgets.QFrame()
        self.summary_panel.setObjectName("motorSummary")
        self.summary_panel.setStyleSheet(
            "QFrame#motorSummary { background:#101f2d; border:1px solid #34536b; border-radius:10px; }"
            "QFrame#motorSummary QLabel { background:transparent; border:none; color:#b9cede; }"
        )
        grid = QtWidgets.QGridLayout(self.summary_panel)
        grid.setContentsMargins(16, 12, 16, 12)
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(4)
        self.summary_state_label = QtWidgets.QLabel("CAN 未连接")
        grid.addWidget(self.summary_state_label, 0, 0, 1, 3)
        fields = (
            ("position_deg", "实时位置（°）", 1, 0, 3, 36, "#65d9ff"),
            ("phase_current_ma", "相电流（mA）", 3, 0, 1, 24, "#ffd075"),
            ("temperature_c", "驱动板温度（℃）", 3, 1, 1, 24, "#74e2ac"),
            ("speed_rpm", "实时转速（RPM）", 3, 2, 1, 24, "#b4c9ff"),
        )
        for key, title, row, column, span, size, color in fields:
            caption = QtWidgets.QLabel(title)
            caption.setStyleSheet("font-size:12px;")
            grid.addWidget(caption, row, column, 1, span)
            value = QtWidgets.QLabel("—")
            value.setStyleSheet(
                f"QLabel {{ font-size:{size}px; font-weight:700; color:{color}; }}"
                "QLabel:disabled { color:#718293; }"
            )
            grid.addWidget(value, row + 1, column, 1, span)
            self.value_labels[key] = value
        self._summary_keys = {field[0] for field in fields}

    def resizeEvent(self, event):
        # 宽窗口靠右显示；窄面板自动换行，保留数字的可读性。
        narrow = event.size().width() < 900
        self.header_layout.setDirection(
            QtWidgets.QBoxLayout.Direction.TopToBottom if narrow
            else QtWidgets.QBoxLayout.Direction.LeftToRight
        )
        self.summary_panel.setMaximumWidth(16777215 if narrow else 440)
        super().resizeEvent(event)

    def _build_motion_ui(self, layout):
        group = QtWidgets.QGroupBox("Emm 普通位置测试")
        form = QtWidgets.QGridLayout(group)
        self.pulses_input = QtWidgets.QSpinBox()
        self.pulses_input.setRange(1, 2147483647)
        self.pulses_input.setValue(16)
        self.speed_input = QtWidgets.QSpinBox()
        self.speed_input.setRange(1, 3000)
        self.speed_input.setValue(10)
        self.acceleration_input = QtWidgets.QSpinBox()
        self.acceleration_input.setRange(0, 255)
        self.acceleration_input.setValue(100)
        self.pulses_per_turn_input = QtWidgets.QSpinBox()
        self.pulses_per_turn_input.setRange(1, 1000000)
        self.pulses_per_turn_input.setValue(self.config.steps_per_turn)
        fields = (
            ("移动脉冲数（相对）", self.pulses_input, "脉冲",
             "输入正整数，按原值发送；从当前实际位置起算，方向由下方按钮决定。1脉冲是细分后的指令步，不是1.8°整步。"),
            ("运动速度", self.speed_input, "RPM",
             "单位RPM（转/分钟）；要求电机的速度缩小10倍功能关闭。首次建议低速、小角度。"),
            ("加速度档位", self.acceleration_input, "档",
             "0～255，无物理单位。0=直接按设定速度起动；1～255使用加减速，数值越大加减速越快。"),
            ("每圈脉冲数（角度参考）", self.pulses_per_turn_input, "脉冲/圈",
             "仅用于显示参考角度，不改变发送脉冲数或电机细分。按360÷步距角×细分填写；1.8°、16细分=3200，与反馈65536 counts/圈不同。"),
        )
        for index, (title, widget, unit, note) in enumerate(fields):
            form.addWidget(QtWidgets.QLabel(title), index * 2, 0)
            input_row = QtWidgets.QHBoxLayout()
            input_row.addWidget(widget, 1)
            unit_label = QtWidgets.QLabel(unit)
            unit_label.setMinimumWidth(55)
            input_row.addWidget(unit_label)
            form.addLayout(input_row, index * 2, 1)
            widget.setToolTip(note)
            widget.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
            widget.setKeyboardTracking(False)
            # 页面独立使用时也禁止滚轮改值，覆盖已聚焦的内部编辑器。
            widget.installEventFilter(self)
            widget.lineEdit().installEventFilter(self)
            widget.valueChanged.connect(self._update_motion_preview)
            label = QtWidgets.QLabel(note)
            label.setWordWrap(True)
            form.addWidget(label, index * 2 + 1, 0, 1, 2)
        self.motion_preview_label = QtWidgets.QLabel()
        self.motion_preview_label.setWordWrap(True)
        form.addWidget(self.motion_preview_label, 8, 0, 1, 2)
        row = QtWidgets.QHBoxLayout()
        self.press_button = QtWidgets.QPushButton("下压（逆时针）")
        self.press_button.clicked.connect(lambda: self._move(True))
        self.lift_button = QtWidgets.QPushButton("抬起（顺时针）")
        self.lift_button.clicked.connect(lambda: self._move(False))
        self.stop_button = QtWidgets.QPushButton("立即停止")
        self.stop_button.clicked.connect(lambda: self._submit("stop"))
        self.stop_button.setToolTip("发送立即停止命令，保持使能；通信失联时不能保证远程停止成功。")
        for button in (self.press_button, self.lift_button, self.stop_button):
            row.addWidget(button)
        form.addLayout(row, 9, 0, 1, 2)
        self.motion_hint_label = QtWidgets.QLabel("连接Emm电机并使能后可测试。")
        self.motion_hint_label.setWordWrap(True)
        form.addWidget(self.motion_hint_label, 10, 0, 1, 2)
        note = QtWidgets.QLabel("下压=CCW，抬起=CW，方向按电机手册定义；首次小角度核对机构方向。本页不按压力自动停止，也不设置机械行程限位。")
        note.setWordWrap(True)
        form.addWidget(note, 11, 0, 1, 2)
        layout.addWidget(group)
        self._update_motion_preview()

    def eventFilter(self, watched, event):
        if event.type() == QtCore.QEvent.Type.Wheel:
            event.ignore()
            return True
        return super().eventFilter(watched, event)

    def _build_homing_ui(self, layout):
        group = QtWidgets.QGroupBox("多圈无限位碰撞回零（Emm）")
        box = QtWidgets.QVBoxLayout(group)
        note = QtWidgets.QLabel(
            "沿电机保存的回零方向寻找机械挡块，通过低转速、高相电流及持续时间判断碰撞。"
            "方向不使用下压/抬起按钮设置；碰撞后会退让，最终位置以反馈为准，不自动清零。"
        )
        note.setWordWrap(True)
        box.addWidget(note)
        self.home_parameters_label = QtWidgets.QLabel("请先读取并核对电机回零参数。")
        self.home_parameters_label.setWordWrap(True)
        box.addWidget(self.home_parameters_label)
        self.home_confirm_checkbox = QtWidgets.QCheckBox("已核对回零方向、参数和机械挡块，机构允许碰撞及退让")
        self.home_confirm_checkbox.toggled.connect(self._refresh_controls)
        box.addWidget(self.home_confirm_checkbox)
        row = QtWidgets.QHBoxLayout()
        self.read_home_button = QtWidgets.QPushButton("读取回零参数")
        self.read_home_button.clicked.connect(lambda: self._submit("read_home"))
        self.home_button = QtWidgets.QPushButton("多圈碰撞回零")
        self.home_button.clicked.connect(self._home)
        self.abort_home_button = QtWidgets.QPushButton("中断回零")
        self.abort_home_button.clicked.connect(lambda: self._submit("abort_home"))
        for button in (self.read_home_button, self.home_button, self.abort_home_button):
            row.addWidget(button)
        box.addLayout(row)
        self.home_status_label = QtWidgets.QLabel("尚未发起回零；空闲标志不代表本次回零成功。")
        self.home_status_label.setWordWrap(True)
        box.addWidget(self.home_status_label)
        layout.addWidget(group)

    def _show_homing_parameters(self, parameters):
        self._homing_parameters = parameters
        self.home_confirm_checkbox.setChecked(False)
        if parameters is None:
            text = "请先读取并核对电机回零参数。"
        else:
            p = parameters
            direction = "CW（顺时针）" if p["direction"] == 0 else "CCW（逆时针）"
            backoff = "按电流退让（BaseOnMa）" if p["backoff_raw"] == 0 else f'{p["backoff_raw"] / 10:g}°'
            text = (f'回零方向：{direction}；速度：{p["speed_rpm"]} RPM；超时：{p["timeout_ms"]} ms\n'
                    f'碰撞检测：转速 < {p["detect_speed_rpm"]} RPM，相电流 > {p["detect_current_ma"]} mA，'
                    f'持续 > {p["detect_time_ms"]} ms；退让：{backoff}\n'
                    "本按钮固定触发模式02（多圈无限位碰撞），其余参数使用上述电机设置，不修改保存值。")
        self.home_parameters_label.setText(text)
        self._refresh_controls()

    def _home_unresolved(self):
        return self._home_phase in ("preparing", "awaiting", "running", "uncertain", "aborting")

    def _set_home_phase(self, phase):
        self._home_phase = phase
        if phase != "running":
            self._home_seen_active = False
        messages = {
            "idle": "尚未发起回零；空闲标志不代表本次回零成功。",
            "preparing": "正在核对状态和电机回零参数…",
            "awaiting": "回零命令已发送，等待正在回零标志；尚不能确认成功。",
            "running": "正在回零（含碰撞后退让），保持观察。",
            "uncertain": "本次回零结果未确认，请检查实际状态；必要时点击中断回零。",
            "aborting": "正在请求中断回零…",
            "interrupted": "已发送中断回零，不作为回零成功；请确认电机停止。",
            "failed": "电机回零失败、失能或触发保护；请检查状态和机构。",
            "rejected": "回零未发出，请检查操作提示。",
        }
        self.home_status_label.setText(messages.get(phase, phase))
        self._refresh_controls()

    def _home(self):
        if self._homing_parameters is None or not self.home_confirm_checkbox.isChecked():
            return
        self._submit("home", dict(self._homing_parameters))
        self.home_confirm_checkbox.setChecked(False)

    def _observe_homing(self, snapshot):
        if not self._is_fresh() or self._home_phase not in ("awaiting", "running", "uncertain"):
            return
        flags = snapshot["home_flags"]
        if flags & 0x38 or snapshot["stall_protection"] or not snapshot["enabled"]:
            self._set_home_phase("failed")
        elif flags & 4:
            self._home_seen_active = True
            self._set_home_phase("running")
        elif self._home_seen_active and snapshot["speed_rpm"] == 0:
            self._home_phase = "completed"
            self._home_may_be_active.clear()
            self.home_status_label.setText(
                f'本次回零已结束，未报告失败；实时位置 {snapshot["position_deg"]:.3f}°。'
                "请核对退让和零点位置。"
            )

    def motion_settings_snapshot(self):
        """GUI线程读取一次本页实际参数，供共享电机任务冻结使用。"""
        return dict(speed_rpm=self.speed_input.value(), acceleration=self.acceleration_input.value(),
                    fine_step_max=self.pulses_input.value(), steps_per_turn=self.config.steps_per_turn)

    def _motion_parameters(self):
        return {
            "pulses": self.pulses_input.value(), "speed_rpm": self.speed_input.value(),
            "acceleration": self.acceleration_input.value(),
        }

    def _update_motion_preview(self, *_):
        parameters = self._motion_parameters()
        pulses = parameters["pulses"]
        angle = pulses * 360 / self.pulses_per_turn_input.value()
        acc = parameters["acceleration"]
        ramp = "直接起动，无加减速" if acc == 0 else f"每 {(256 - acc) * 0.05:g} ms 变化 1 RPM"
        self.motion_preview_label.setText(f"发送指令：{pulses} 脉冲；参考转角 ≈ {angle:.4f}°；{ramp}。")

    @staticmethod
    def _motion_block_reason(snapshot, max_age=STALE_SECONDS):
        if snapshot is None or not 0 <= time.monotonic() - snapshot["sampled_at"] <= max_age:
            return "状态无效或已过期"
        if snapshot["firmware"] != "Emm":
            return "位置测试仅支持Emm固件"
        if not snapshot["enabled"]:
            return "请先使能电机"
        if not snapshot["encoder_ready"] or not snapshot["calibration_ready"]:
            return "编码器或校准表未就绪"
        if any(snapshot[key] for key in ("stall", "stall_protection", "overtemperature", "overcurrent")):
            return "电机有堵转或保护标志"
        if snapshot["home_flags"] & 4 or abs(snapshot["speed_rpm"]) > 0:
            return "电机正在运动，请等待停止或点击立即停止"
        return ""

    def _move(self, ccw):
        parameters = self._motion_parameters()
        parameters["ccw"] = ccw
        self._submit("move", parameters)

    def _toggle_connection(self):
        self._submit("disconnect" if self._connected else "connect")

    def connect_can(self):
        if not self._connected:
            self._submit("connect")

    def start_external_operation(self, operation, initial_status):
        """外部页面复用唯一CAN线程及停止事件，运行期间锁定本页操作。"""
        reason = self._motion_block_reason(self._last_sample, self.feedback_max_age_s)
        if self._closing or self._busy or self._home_unresolved() or not self._connected or reason:
            return False, reason or "电机未连接或已有操作正在运行"
        self._submit("external", operation)
        self.command_label.setText(initial_status)
        return True, ""

    def _submit(self, operation, parameters=None):
        urgent = operation in ("stop", "abort_home")
        if self._closing or (self._busy and not urgent):
            return
        if operation not in ("connect", "disconnect") and not self._connected:
            return
        if operation in ("enable", "calibrate") and not self._is_fresh():
            return
        if operation in ("move", "home", "zero", "external"):
            if self._home_unresolved():
                return
            reason = self._motion_block_reason(self._last_sample, self.feedback_max_age_s)
            if reason:
                self.command_label.setText(reason)
                return
            self._cancel_motion.clear()
            if operation == "home":
                self._set_home_phase("preparing")
        if operation in ("enable", "calibrate", "read_home") and self._home_unresolved():
            return
        if operation == "read_home":
            self._show_homing_parameters(None)
        if urgent:
            if self._stop_queued:
                return
            self._stop_queued = True
            self._cancel_motion.set()
            if (operation == "abort_home" or self._home_may_be_active.is_set()
                    or (self._last_sample and self._last_sample["home_flags"] & 4)):
                self._set_home_phase("aborting")
        self._pending_commands += 1
        self._busy = True
        names = {"move": "位置运动", "stop": "立即停止", "connect": "连接",
                 "disconnect": "断开", "enable": "使能", "disable": "失能", "calibrate": "编码器校准",
                 "read_home": "读取回零参数", "home": "碰撞回零", "abort_home": "中断回零"}
        names.update(zero="当前位置设为0", external="压力位置标定")
        self.command_label.setText("正在执行：" + names[operation])
        self._refresh_controls()
        self._task_sequence += 1
        self._tasks.put((0 if urgent else 1, self._task_sequence, operation, parameters))

    def _calibrate(self):
        if not self.unloaded_checkbox.isChecked():
            return
        self._submit("calibrate")
        self.unloaded_checkbox.setChecked(False)

    def _service_loop(self):
        motor = None
        next_poll = 0.0
        try:
            while not self._stop.is_set():
                # 队列可立即唤醒；短等待只用于退出检查，不会因此发送CAN查询。
                wait = max(0, min(.1, next_poll - time.monotonic())) if motor else .1
                try:
                    _, _, operation, parameters = self._tasks.get(timeout=wait)
                except queue.Empty:
                    operation = None
                if self._stop.is_set():
                    break
                if operation is not None:
                    home_sent = False
                    abort_home = False
                    try:
                        if operation == "connect":
                            # 防止排队的重复连接覆盖尚未释放的旧电机实例。
                            if motor is not None:
                                motor.close()
                                motor = None
                            motor = self._motor_factory(self.config)
                            motor.open()
                            snapshot = motor.read_status()
                            self._update_poll_mode(snapshot)
                            self.link_changed.emit(True)
                            self.status_received.emit(snapshot)
                            message = "已连接；只读状态轮询已启动"
                        elif operation == "disconnect":
                            if motor:
                                motor.close()
                            motor = None
                            self._motion_polling = False
                            self.link_changed.emit(False)
                            message = "CAN 已断开"
                        elif operation in ("enable", "disable"):
                            motor.set_enabled(operation == "enable")
                            message = "使能命令已接收" if operation == "enable" else "失能命令已接收"
                        elif operation == "external":
                            # 外部任务负责捕获异常、停止和结果通知；不另开总线或线程。
                            self._external_polling = True
                            try:
                                # 外部控制器在任务入口自行读取并核验新反馈；不以空闲缓存驱动。
                                parameters(motor, self._cancel_motion)
                            finally:
                                self._external_polling = False
                            message = "标定页面任务已结束"
                        elif operation == "zero":
                            snapshot = motor.read_status()
                            reason = self._motion_block_reason(snapshot)
                            if reason or self._cancel_motion.is_set() or self._stop.is_set():
                                raise ValueError(reason or "用户停止操作")
                            self.coordinate_invalidated.emit()
                            motor.zero_position()
                            snapshot = motor.read_status()
                            self.status_received.emit(snapshot)
                            if abs(snapshot["position_deg"]) > 360 / self.config.steps_per_turn:
                                raise ValueError("清零应答已收到，但位置反馈未归零，请检查实机")
                            message = "当前位置已设为0，反馈已确认；请重新设置释放位置并标定"
                        elif operation == "calibrate":
                            snapshot = motor.read_status()
                            self.status_received.emit(snapshot)
                            if abs(snapshot['speed_rpm']) > 0 or snapshot['home_flags'] & 4:
                                raise ValueError('电机仍在运动，不能启动校准')
                            if any(snapshot[k] for k in ('stall', 'stall_protection', 'overtemperature', 'overcurrent')):
                                raise ValueError('电机有保护标志，不能启动校准')
                            self.coordinate_invalidated.emit()
                            self._begin_motion_polling()
                            motor.calibrate_encoder()
                            message = "校准命令已接收；请观察电机正反转，勿重复触发"
                        elif operation == "read_home":
                            self.homing_parameters_received.emit(motor.read_homing_parameters())
                            message = "已读取电机回零参数，请核对方向、阈值及退让设置"
                        elif operation == "home":
                            if self._cancel_motion.is_set():
                                raise ValueError("停止已请求，取消待发的回零命令")
                            current = motor.read_homing_parameters()
                            if current != parameters:
                                self.homing_parameters_received.emit(current)
                                raise ValueError("电机回零参数已变化，请重新核对后再触发")
                            if not (0 < current["speed_rpm"] <= 3000 and current["timeout_ms"] > 0
                                    and 0 < current["detect_speed_rpm"] <= 3000
                                    and 0 < current["detect_current_ma"] <= 5000
                                    and current["detect_time_ms"] > 0):
                                raise ValueError("回零速度、超时或碰撞检测参数无效，请先在电机端设置")
                            snapshot = motor.read_status()
                            self.status_received.emit(snapshot)
                            reason = self._motion_block_reason(snapshot)
                            if reason:
                                raise ValueError(reason)
                            if self._cancel_motion.is_set() or self._stop.is_set():
                                raise ValueError("停止已请求，取消待发的回零命令")
                            home_sent = True
                            self.coordinate_invalidated.emit()
                            self._home_may_be_active.set()
                            self.homing_phase_changed.emit("awaiting")
                            self._begin_motion_polling()
                            motor.home_sensorless()
                            message = "碰撞回零命令已接收；等待回零过程结束，不代表已经成功"
                        elif operation == "move":
                            if self._cancel_motion.is_set():
                                raise ValueError("停止已请求，取消待发的位置命令")
                            snapshot = motor.read_status()
                            self.status_received.emit(snapshot)
                            reason = self._motion_block_reason(snapshot)
                            if reason:
                                raise ValueError(reason)
                            if self._cancel_motion.is_set() or self._stop.is_set():
                                raise ValueError("停止已请求，取消待发的位置命令")
                            self._begin_motion_polling()
                            pulses = motor.move_relative_pulses(**parameters)
                            direction = "下压（逆时针）" if parameters["ccw"] else "抬起（顺时针）"
                            message = f"{direction}命令已接收：{pulses}脉冲，{parameters['speed_rpm']} RPM；完成情况以实际位置和到位状态为准"
                        elif operation in ("stop", "abort_home"):
                            self._begin_motion_polling(stopping=True)
                            snapshot = self._last_sample
                            abort_home = (operation == "abort_home" or self._home_may_be_active.is_set()
                                          or (snapshot is not None and snapshot["home_flags"] & 4))
                            if abort_home:
                                # 回零使用专用中断命令；普通停止仍尝试发送，避免超时漏掉停止。
                                try:
                                    motor.abort_homing()
                                finally:
                                    if operation == "stop":
                                        motor.stop_motion()
                                self._home_may_be_active.clear()
                                self.homing_phase_changed.emit("interrupted")
                                message = "中断回零命令已接收；请确认正在回零标志清除、实际转速归零"
                            else:
                                motor.stop_motion()
                                message = "立即停止命令已接收；请确认实际转速归零"
                        else:
                            raise ValueError("未知操作")
                        self.command_finished.emit(operation, message)
                    except Exception as exc:
                        if operation in ("connect", "disconnect"):
                            if motor is not None:
                                try:
                                    motor.close()
                                except Exception:
                                    pass
                            motor = None
                            self.link_changed.emit(False)
                        message = f"操作失败：{exc}"
                        if operation == "calibrate":
                            message += "；校准可能已触发，不自动重发，请观察电机"
                        elif operation == "move":
                            message += "；运动指令不自动重发，超时后请检查实际位置，必要时立即停止"
                        elif operation == "home":
                            self.homing_phase_changed.emit("uncertain" if home_sent else "rejected")
                            message += "；回零命令不自动重发，请检查实际状态，必要时中断回零"
                        elif operation == "zero":
                            message += "；清零命令不重发，旧坐标已作废，请核对实际位置"
                        elif operation in ("abort_home", "stop") and abort_home:
                            self.homing_phase_changed.emit("uncertain")
                        self.command_finished.emit(operation, message)
                    next_poll = time.monotonic() + (self._poll_interval() if operation == "connect" else 0)
                if (motor is not None and time.monotonic() >= next_poll
                        and not self._stop.is_set() and self._tasks.empty()):
                    started = time.monotonic()
                    try:
                        snapshot = motor.read_status()
                        self._update_poll_mode(snapshot)
                        self.status_received.emit(snapshot)
                    except Exception as exc:
                        self.communication_failed.emit(str(exc))
                    # 空闲2秒；运动50毫秒起始间隔。超时不补发积压轮询。
                    interval = self._poll_interval()
                    next_poll = (max(started+interval, time.monotonic()+.001) if self._motion_polling
                                 else time.monotonic()+interval)
        finally:
            if motor is not None:
                try:
                    motor.close()
                except Exception:
                    pass

    def _begin_motion_polling(self, stopping=False):
        # 命令接收后的最初静止反馈可能早于起动，至少快速观察半秒。
        self._motion_polling = True
        self._motion_poll_not_before = time.monotonic()+.5
        self._confirming_stop = stopping

    def _update_poll_mode(self, snapshot):
        moving = abs(snapshot['speed_rpm']) > 0 or bool(snapshot['home_flags'] & 4)
        if moving:
            self._motion_polling = True
        elif (time.monotonic() >= self._motion_poll_not_before
              and (snapshot.get('in_position') or not snapshot.get('enabled') or self._confirming_stop
                   or any(snapshot.get(k) for k in ('stall', 'stall_protection', 'overtemperature', 'overcurrent')))):
            self._motion_polling = False
            self._confirming_stop = False

    def _poll_interval(self):
        return self.MOTION_POLL_INTERVAL if self._motion_polling else self.POLL_INTERVAL

    @property
    def feedback_max_age_s(self):
        # 仅用于空闲显示/提交任务；控制器和后台发令前仍严格读取新反馈。
        return self.STALE_SECONDS if self._motion_polling or self._external_polling else self.IDLE_STALE_SECONDS

    def _show_connection(self, connected):
        self._connected = connected
        self._last_sample = None
        self._sample_times.clear()
        self._show_homing_parameters(None)
        self._home_may_be_active.clear()
        self._set_home_phase("idle")
        self.connection_label.setText("CAN 已连接" if connected else "CAN 未连接")
        for label in self.value_labels.values():
            label.setText("—")
            label.setToolTip("")
        self._refresh_freshness()

    def _finish_command(self, operation, message):
        self._pending_commands = max(0, self._pending_commands - 1)
        self._busy = self._pending_commands > 0
        if operation in ("stop", "abort_home"):
            self._stop_queued = False
        self.command_label.setText(message)
        self._refresh_controls()

    def _show_error(self, message):
        self.connection_label.setText("状态读取失败，正在重试：" + message)
        self.connection_label.setWordWrap(True)
        self._last_sample = None
        self._sample_times.clear()
        if self._home_unresolved():
            self._set_home_phase("uncertain")
        self._refresh_freshness()

    def _show_status(self, snapshot):
        self._last_sample = snapshot
        self._observe_homing(snapshot)
        self._sample_times.append(snapshot["sampled_at"])
        self.connection_label.setText("CAN 已连接，状态读取正常")
        units = {"temperature_c": " ℃", "position_deg": " °", "target_deg": " °",
                 "error_deg": " °", "phase_current_ma": " mA", "bus_current_ma": " mA",
                 "voltage_mv": " mV", "speed_rpm": " RPM"}
        for key, label in self.value_labels.items():
            value = snapshot.get(key)
            if value is None:
                text = "此固件未提供"
            elif isinstance(value, bool):
                text = "是" if value else "否"
            elif key.endswith("_flags"):
                text = f"0x{value:02X}"
            elif isinstance(value, float):
                text = f"{value:.3f}" + units.get(key, "")
            else:
                text = str(value) + units.get(key, "")
            # 顶部面板的单位在标题中固定显示，数值区保留足够空间。
            if key in self._summary_keys:
                label.setToolTip(text)
                if value is None:
                    text = "—"
                else:
                    text = f"{value:.3f}" if key == "position_deg" else f"{value:g}"
            label.setText(text)
        self._refresh_freshness()

    def _is_fresh(self):
        return self._last_sample is not None and 0 <= time.monotonic() - self._last_sample["sampled_at"] <= self.feedback_max_age_s

    def _refresh_freshness(self):
        fresh = self._is_fresh()
        cadence = '任务内快速查询' if self._external_polling else '运动快速查询' if self._motion_polling else '空闲每2秒查询'
        if not fresh and self._home_phase in ("awaiting", "running"):
            self._set_home_phase("uncertain")
        if self._last_sample is None:
            text = "尚无有效状态；"+cadence
        else:
            age = time.monotonic() - self._last_sample["sampled_at"]
            rate = (len(self._sample_times) - 1) / max(0.001, self._sample_times[-1] - self._sample_times[0]) if len(self._sample_times) > 1 else 0
            text = f"{'有效' if fresh else '数据已过期'} · 距上次更新 {age:.2f} s · 实际 {rate:.1f} Hz · {cadence}"
        self.freshness_label.setText(text)
        self.freshness_label.setStyleSheet("color:#299455;" if fresh else "color:#bd6b22;")
        if fresh:
            summary_text = f"● {cadence} · {rate:.1f} Hz"
        elif self._last_sample is not None:
            summary_text = "● 数据已过期 · 以下为上次读数"
        elif self._connected:
            summary_text = "● 等待有效数据 · 读数不可用"
        else:
            summary_text = "● CAN 未连接"
        self.summary_state_label.setText(summary_text)
        self.summary_state_label.setStyleSheet(
            f"font-size:12px; font-weight:600; color:{'#74e2ac' if fresh else '#ffbd70'};"
        )
        for label in self.value_labels.values():
            label.setEnabled(fresh)
        self._refresh_controls()

    def _refresh_controls(self, *_):
        self.control_state_changed.emit(self._connected and self._is_fresh(), self._busy or self._home_unresolved())
        available = not self._busy and not self._closing
        homing = self._home_unresolved() or bool(self._last_sample and self._last_sample["home_flags"] & 4)
        self.connect_button.setEnabled(available)
        self.connect_button.setText("断开 CAN" if self._connected else "连接 CAN")
        self.enable_button.setEnabled(available and self._connected and self._is_fresh() and not homing)
        self.disable_button.setEnabled(available and self._connected)
        self.calibrate_button.setEnabled(available and self._connected and self._is_fresh() and not homing and self.unloaded_checkbox.isChecked())
        reason = self._motion_block_reason(self._last_sample, self.feedback_max_age_s)
        can_move = available and self._connected and not reason and not homing
        self.press_button.setEnabled(can_move)
        self.lift_button.setEnabled(can_move)
        self.stop_button.setEnabled(self._connected and not self._closing and not self._stop_queued)
        self.read_home_button.setEnabled(available and self._connected and not homing)
        self.home_button.setEnabled(can_move and self._homing_parameters is not None and self.home_confirm_checkbox.isChecked())
        self.home_confirm_checkbox.setEnabled(available and self._connected and self._homing_parameters is not None and not homing)
        self.abort_home_button.setEnabled(self._connected and not self._closing and not self._stop_queued)
        if homing:
            reason = "回零进行中或结果未确认，请等待完成或中断回零。"
        self.motion_hint_label.setText(reason or "可发送相对位置命令；按压和抬起都从当前实际位置起算。")
        for widget in (self.pulses_input, self.speed_input, self.acceleration_input, self.pulses_per_turn_input):
            widget.setEnabled(available)

    def shutdown(self):
        self._closing = True
        self._timer.stop()
        self._stop.set()
        self._cancel_motion.set()
        self._tasks.put((-1, 0, None, None))
        self._thread.join(timeout=self.config.timeout * 4 + 1)
