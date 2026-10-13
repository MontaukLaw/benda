"""工程调试界面：参数即时生效，绘图50Hz，采样/记录保持逐帧。"""

from PySide6 import QtCore, QtGui, QtWidgets
from .config import AppConfig, CONFIG_PATH, NUMERIC_LIMITS, RAW_GAIN_LIMITS
from .processing_engine import ProcessingEngine
from .serial_reader import SerialReader
from .sensor_widget import SensorWidget
from .simulation import SimulationReader
from .calibration_import import CALIBRATION_DIR, latest_calibration_path, read_calibration

AUTO_CONNECT_PORTS = ("COM4", "COM3")
AUTO_CONNECT_TIMEOUT_MS = 2000

STYLE = """
QMainWindow, QWidget { background: #18212d; color: #e0e8f2; font-size: 12px; }
QGroupBox { border: 1px solid #3a4b60; border-radius: 6px; margin-top: 12px; padding: 12px 8px 8px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #82caff; }
QPushButton { background: #2d4359; border: 1px solid #4a637d; border-radius: 4px; padding: 7px 12px; }
QPushButton:hover { background: #3a5876; }
QPushButton:disabled { color: #788594; background: #243140; }
QComboBox, QSpinBox, QDoubleSpinBox { background: #101a26; border: 1px solid #46596d; border-radius: 3px; padding: 5px; min-height: 20px; }
QToolBox::tab { background: #28394b; border-radius: 3px; padding: 7px; }
QToolBox::tab:selected { background: #36536e; color: #ffffff; }
QScrollArea { border: 0; }
QCheckBox { spacing: 7px; padding: 3px; }
QStatusBar { background: #101923; }
"""


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, auto_connect=True):
        super().__init__()
        warning = ""
        self.config = AppConfig()
        if CONFIG_PATH.exists():
            try:
                self.config = AppConfig.load(CONFIG_PATH)
            except (OSError, ValueError, TypeError) as exc:
                warning = f"配置读取失败，使用默认值：{exc}"
        self.engine = ProcessingEngine(self.config)
        self.worker = None
        self._closing = False
        self._auto_connect_active = False
        self._auto_ports = []
        self._auto_failures = []
        self._auto_probe_reason = ""
        self._auto_start_timer = QtCore.QTimer(self)
        self._auto_start_timer.setSingleShot(True)
        self._auto_start_timer.timeout.connect(self._start_auto_connect)
        self._auto_probe_timer = QtCore.QTimer(self)
        self._auto_probe_timer.setSingleShot(True)
        self._auto_probe_timer.timeout.connect(self._auto_connect_timeout)
        self.controls = {}
        self.gain_controls = []
        self._calibration_draft = None
        self.debug_labels = {}
        self.serial_diagnostics = {"parse_errors": 0, "discarded_bytes": 0, "rejected_channels": 0}
        self.setWindowTitle("4×5 Touch Debugger · Raw → ROI → Centroid")
        self.resize(1280, 900)
        self.setMinimumSize(1050, 720)
        self.setStyleSheet(STYLE)
        self._build_ui()
        self._sync_controls(sync_gains=True)
        self.refresh_ports()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.refresh_view)
        self.timer.start()
        self.statusBar().showMessage(warning or ("请释放阵列；启动后依次自动尝试 COM4、COM3。" if auto_connect
                                               else "请释放阵列，再连接或启动模拟。"))
        if auto_connect:
            self._auto_start_timer.start(0)

    @staticmethod
    def _button(text, callback):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(callback)
        return button

    def _build_ui(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QHBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(16)
        left = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("4 × 5 压力阵列 / 连续位置与位移")
        title.setStyleSheet("font-size: 20px; font-weight: bold; color: #d5ebff;")
        left.addWidget(title)
        hint = QtWidgets.QLabel("黄框 MAX  ·  蓝虚线有效ROI  ·  绿框 Active  ·  白空心 Raw  ·  粉色 Filtered")
        hint.setStyleSheet("color: #aebed0;")
        left.addWidget(hint)
        self.sensor_widget = SensorWidget(self.config)
        self.sensor_widget.simulation_position.connect(self._simulation_position)
        left.addWidget(self.sensor_widget, 1)
        self.position_label = QtWidgets.QLabel("X —    Y —\nDX +0.000    DY +0.000")
        self.position_label.setFont(QtGui.QFont("Consolas", 22))
        self.position_label.setStyleSheet("color: #f3a2c6; padding: 10px; font-family: Consolas; font-size: 28px;")
        left.addWidget(self.position_label)
        self.touch_label = QtWidgets.QLabel("IDLE · 等待输入")
        self.touch_label.setStyleSheet("font-size: 16px; color: #94dabc;")
        left.addWidget(self.touch_label)
        self.baseline_label = QtWidgets.QLabel("请先释放阵列，等待稳定 baseline")
        self.baseline_label.setWordWrap(True)
        left.addWidget(self.baseline_label)
        actions = QtWidgets.QHBoxLayout()
        actions.addWidget(self._button("Reset Baseline", self.reset_baseline))
        actions.addWidget(self._button("Reset Tracker", self.reset_tracker))
        self.record_button = self._button("Start Recording", self.toggle_recording)
        actions.addWidget(self.record_button)
        left.addLayout(actions)
        markers = QtWidgets.QHBoxLayout()
        markers.addWidget(QtWidgets.QLabel("实际动作标记："))
        self.marker_buttons = []
        self.marker_shortcuts = []
        for observation, title, key in (("contact", "标记接触 (F7)", "F7"),
                                         ("released", "标记松手 (F8)", "F8")):
            callback = lambda checked=False, value=observation: self.mark_observation(value)
            button = self._button(title, callback)
            button.setEnabled(False)
            button.setToolTip("录制时在实际动作发生后立即标记；保存主机时间和最近采样时间。")
            markers.addWidget(button)
            self.marker_buttons.append(button)
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(key), self)
            shortcut.setAutoRepeat(False)
            shortcut.setEnabled(False)
            shortcut.activated.connect(callback)
            self.marker_shortcuts.append(shortcut)
        left.addLayout(markers)
        self.record_label = QtWidgets.QLabel("CSV：未记录")
        self.record_label.setWordWrap(True)
        self.record_label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        left.addWidget(self.record_label)
        layout.addLayout(left, 1)

        right = QtWidgets.QWidget()
        right.setMinimumWidth(365)
        right.setMaximumWidth(430)
        side = QtWidgets.QVBoxLayout(right)
        side.setContentsMargins(0, 0, 0, 0)
        side.addWidget(self._serial_panel())
        toolbox = QtWidgets.QToolBox()
        self.toolbox = toolbox
        toolbox.addItem(self._gain_panel(), "Raw Gain / 20通道增益")
        panels = [
            ("Signal / PC 本地基线", [
                ("noise_threshold", "Noise Threshold (counts)"),
                ("baseline_update_rate", "Baseline Update Rate / frame"),
                ("baseline_freeze_threshold", "双向 Freeze Threshold (counts)"),
                ("baseline_stability_limit", "稳定窗口允许波动 (counts)"),
            ]),
            ("Touch / 触摸滞回", [
                ("touch_on_threshold", "Touch ON Threshold (counts)"),
                ("touch_off_threshold", "Touch OFF Threshold (counts)"),
            ]),
            ("Position / 局部质心", [
                ("roi_mode", "ROI Mode"),
                ("roi_transition_enable", "ROI 平滑过渡"),
                ("roi_transition_ms", "ROI 过渡时间 (ms)"),
                ("relative_threshold_ratio", "Relative Threshold Ratio"),
                ("weight_power", "Weight Power"),
                ("derivative_weight_enable", "Derivative Weight Enable"),
                ("positive_gain", "Positive Gain"),
                ("negative_gain", "Negative Gain"),
                ("minimum_decay_weight", "Minimum Decay Weight"),
            ]),
            ("Filter / 位置滤波", [
                ("position_filter_enable", "Position Filter Enable"),
                ("position_alpha", "Position Alpha"),
                ("decay_guard_enable", "衰减坐标保护"),
                ("decay_guard_window_ms", "衰减观察窗口 (ms)"),
                ("decay_guard_drop_ratio", "明显衰减比例 (0～1)"),
            ]),
            ("Gesture / 位移输出", [
                ("dx_deadzone", "DX Deadzone (grid)"),
                ("dy_deadzone", "DY Deadzone (grid)"),
                ("x_gain", "X Gain"), ("y_gain", "Y Gain"),
                ("invert_x", "Invert X"), ("invert_y", "Invert Y"),
            ]),
            ("Visualization / 显示", [
                ("show_raw", "Show Raw"), ("show_signal", "Show Signal"),
                ("show_max", "Show MAX"), ("show_roi", "Show ROI"),
                ("show_active_sensors", "Show Active Sensors"),
                ("show_raw_centroid", "Show Raw Centroid"),
                ("show_filtered_cursor", "Show Filtered Cursor"),
                ("show_trail", "Show Trail"),
                ("trail_length", "Trail Length (points / 最近1秒)"),
                ("visualization_max_signal", "Max Signal (counts)"),
            ]),
        ]
        for name, entries in panels:
            panel = QtWidgets.QWidget()
            form = QtWidgets.QFormLayout(panel)
            for key, label in entries:
                self._add_control(form, key, label)
            if name.startswith("Signal"):
                note = QtWidgets.QLabel("初次连接/重建需50帧且至少0.5秒稳定空载。\n稳定按压也可能被误作基线，请先释放。")
                note.setWordWrap(True)
                form.addRow(note)
                form.addRow(self._button("Reset Baseline", self.reset_baseline))
            elif name.startswith("Touch"):
                note = QtWidgets.QLabel("OFF < ON；OFF=0时只在全零信号时释放。\n回弹残留 > OFF 时仍判为触摸，需要实测调节。")
                note.setWordWrap(True)
                form.addRow(note)
            elif name.startswith("Position"):
                note = QtWidgets.QLabel("过渡期间旧/新ROI贡献平滑交接；蓝虚线亮度表示参与权重。\n增大过渡时间会增加跟随延迟；关闭可对比原硬切算法。")
                note.setWordWrap(True)
                form.addRow(note)
            elif name.startswith("Filter"):
                note = QtWidgets.QLabel("信号整体衰减且位置突然转向、缺少新响应时，保持粉色圈并输出零位移。\n新的局部响应连续20ms后恢复；白圈仍显示当前质心，触摸释放阈值不变。")
                note.setWordWrap(True)
                form.addRow(note)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(panel)
            toolbox.addItem(scroll, name)
        debug = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(debug)
        for key, label in (
            ("fps", "Actual FPS"), ("state", "Touch State (PC)"),
            ("last_touch_event", "Last DOWN / UP Event"),
            ("max_index", "MAX Channel"), ("max_signal", "MAX Signal"),
            ("roi_transition", "ROI Transition"),
            ("position_confidence", "坐标可信度 / 保护状态"),
            ("active_count", "Active Sensor Count"), ("x_raw", "X Raw"), ("y_raw", "Y Raw"),
            ("x", "X Filtered"), ("y", "Y Filtered"), ("dx", "DX"), ("dy", "DY"),
            ("total_signal", "Total Signal"), ("parse_errors", "Parse Error Count"),
            ("discarded_bytes", "Discarded RX Bytes"), ("stream_gaps", "Stream Gap Count (>250ms)"),
            ("dropped_frames", "Dropped Frame Count"), ("samples", "Processed Samples"),
            ("rejected", "Rejected Samples/Channels"), ("metadata", "Protocol / Version / Device"),
        ):
            value = QtWidgets.QLabel("—")
            value.setWordWrap(True)
            value.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            form.addRow(label, value)
            self.debug_labels[key] = value
        note = QtWidgets.QLabel("协议没有序号，无法精确计丢帧；仅统计可见断流。\nCSV baseline/signal 是PC计算值。\n本页触摸事件不替换固件真实通道状态。")
        note.setWordWrap(True)
        form.addRow(note)
        debug_scroll = QtWidgets.QScrollArea()
        debug_scroll.setWidgetResizable(True)
        debug_scroll.setWidget(debug)
        toolbox.addItem(debug_scroll, "Debug / 数值与接收诊断")
        side.addWidget(toolbox, 1)
        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self._button("Save Config", self.save_config))
        buttons.addWidget(self._button("Load Config", self.load_config))
        side.addLayout(buttons)
        layout.addWidget(right)

    def _gain_panel(self):
        panel = QtWidgets.QWidget()
        panel.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Preferred)
        layout = QtWidgets.QVBoxLayout(panel)
        note = QtWidgets.QLabel("计算 Raw = 设备 Raw × 通道增益；R 显示计算 Raw。\n编辑后点击应用/保存生效；请先松手，等待新基线就绪。\n范围0～100；0可屏蔽通道。")
        note.setWordWrap(True)
        layout.addWidget(note)
        grid_widget = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(grid_widget)
        low, high, step, decimals = RAW_GAIN_LIMITS
        for index in range(20):
            control = QtWidgets.QDoubleSpinBox()
            control.setObjectName(f"raw_gain_{index}")
            control.setDecimals(decimals)
            control.setRange(low, high)
            control.setSingleStep(step)
            control.setKeyboardTracking(False)
            control.setMinimumWidth(100)
            grid.addWidget(QtWidgets.QLabel(f"CH{index}"), index % 10, (index // 10) * 2)
            grid.addWidget(control, index % 10, (index // 10) * 2 + 1)
            self.gain_controls.append(control)
        layout.addWidget(grid_widget)
        import_row = QtWidgets.QHBoxLayout()
        self.calibration_load_button = self._button('读取标定记录…', self.choose_calibration)
        self.calibration_latest_button = self._button('读取最近一次标定', self.load_latest_calibration)
        import_row.addWidget(self.calibration_load_button); import_row.addWidget(self.calibration_latest_button)
        layout.insertLayout(1, import_row)
        self.calibration_source_label = QtWidgets.QLabel('未导入标定记录；当前使用已保存或手动设置的增益。')
        self.calibration_source_label.setWordWrap(True)
        self.calibration_source_label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
        self.calibration_source_label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.insertWidget(2, self.calibration_source_label)
        self.calibration_confirm = QtWidgets.QCheckBox('确认同一阵列、接触面及三参数未变')
        self.calibration_confirm.setToolTip('本应用不自动核对标定设备的Nsub、TxDivider、RefCDAC；应用前请确认一致。')
        self.calibration_confirm.hide(); layout.insertWidget(3, self.calibration_confirm)
        self.gain_reset_button = self._button("全部恢复1.0", self.reset_gain_inputs)
        self.gain_apply_button = self._button("应用 / 保存增益并重建基线", self.apply_raw_gains)
        layout.addWidget(self.gain_reset_button)
        layout.addWidget(self.gain_apply_button)
        # 整页滚动，避免新增导入说明把20通道输入区域压缩为零高度。
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        self.gain_scroll = scroll
        return scroll

    def reset_gain_inputs(self):
        for control in self.gain_controls:
            control.setValue(1.0)
        self._calibration_draft = None
        self._refresh_calibration_source()
        self.statusBar().showMessage("增益输入已全部恢复1.0；点击应用/保存后生效。", 5000)

    def choose_calibration(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '读取algo_monitor标定结果', str(CALIBRATION_DIR), '标定记录 (*.json)')
        if path: self.load_calibration(path)

    def load_latest_calibration(self):
        try:
            return self.load_calibration(latest_calibration_path(CALIBRATION_DIR))
        except (OSError, ValueError) as exc:
            self.statusBar().showMessage(f'标定读取失败：{exc}')
            return False

    def load_calibration(self, path):
        """读取为可审阅的增益草稿；不改当前算法、基线或来源文件。"""
        try:
            source = read_calibration(path)
        except (OSError, ValueError, TypeError, UnicodeError, OverflowError) as exc:
            self.statusBar().showMessage(f'标定读取失败：{exc}；当前增益和输入未改变。')
            return False
        self._calibration_draft = source
        for control, gain in zip(self.gain_controls, source['gains']): control.setValue(gain)
        self.calibration_confirm.setChecked(False)
        self._refresh_calibration_source()
        self.statusBar().showMessage('20通道标定已读入增益输入框；确认环境一致，完全松手后点击应用/保存。')
        return True

    def _refresh_calibration_source(self):
        source = self._calibration_draft or self.config.calibration_source
        self.calibration_confirm.setVisible(self._calibration_draft is not None)
        if source:
            state = '待读入算法' if self._calibration_draft else '已应用并保存'
            from pathlib import Path
            self.calibration_source_label.setText(f"{state}：{source['mass_g']:g} g，目标Δ={source['target']:g}，20通道合格\n{Path(source['path']).name}")
            self.calibration_source_label.setToolTip(source['path']+'\n'+
                ('本应用不自动核对三参数；应用时替换20路总增益，不再乘已有增益。' if self._calibration_draft else
                 '启动恢复已保存增益；外部记录更新后需重新读取并应用。'))
        else:
            self.calibration_source_label.setText('未导入标定记录；当前使用已保存或手动设置的增益。')
            self.calibration_source_label.setToolTip('')

    def apply_raw_gains(self):
        for control in self.gain_controls:
            control.interpretText()
        config = self.config.copy()
        config.raw_gains = [control.value() for control in self.gain_controls]
        draft = self._calibration_draft
        if draft and config.raw_gains == draft['gains']:
            if not self.calibration_confirm.isChecked():
                self.statusBar().showMessage('请先确认是同一阵列、接触面及三参数未变，再完全松手应用标定增益。')
                return False
            from copy import deepcopy
            from datetime import datetime
            config.calibration_source = deepcopy(draft)
            config.calibration_source['confirmed_at'] = datetime.now().astimezone().isoformat()
        elif config.calibration_source and config.raw_gains != config.calibration_source['gains']:
            # 手动调整后不再将当前增益记作原标定结果。
            config.calibration_source = {}
        try:
            # 保存失败不改变运行中的增益或基线，保留输入以便重试。
            config.save(CONFIG_PATH)
            self.engine.set_config(config, rebuild_baseline=True)
        except (OSError, ValueError) as exc:
            self.statusBar().showMessage(f"增益应用/保存失败：{exc}")
            return False
        self.config = config
        self._calibration_draft = None
        self.calibration_confirm.setChecked(False)
        self._refresh_calibration_source()
        self.refresh_view()
        self.statusBar().showMessage("增益已应用并保存；请保持松手至少0.5秒，等待稳定基线。", 8000)
        return True

    def _serial_panel(self):
        panel = QtWidgets.QGroupBox("Serial / 数据来源")
        form = QtWidgets.QFormLayout(panel)
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Serial", "Simulation"])
        form.addRow("Input Mode", self.mode_combo)
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        port_row = QtWidgets.QHBoxLayout()
        port_row.addWidget(self.port_combo, 1)
        self.refresh_button = self._button("Refresh", self.refresh_ports)
        port_row.addWidget(self.refresh_button)
        form.addRow("COM Port", port_row)
        self.baud_spin = QtWidgets.QSpinBox()
        self.baud_spin.setRange(1200, 4000000)
        self.baud_spin.setValue(self.config.baudrate)
        form.addRow("Baudrate (baud)", self.baud_spin)
        self.protocol_combo = QtWidgets.QComboBox()
        self.protocol_combo.addItems(["DATA", "RAW20"])
        form.addRow("Protocol", self.protocol_combo)
        self.slow_rebound = QtWidgets.QCheckBox("模拟慢回弹（可选实验）")
        self.slow_rebound.toggled.connect(self._set_slow_rebound)
        form.addRow(self.slow_rebound)
        buttons = QtWidgets.QHBoxLayout()
        self.connect_button = self._button("Connect / Start", self.start_source)
        self.disconnect_button = self._button("Disconnect / Stop", self.stop_source)
        self.disconnect_button.setEnabled(False)
        buttons.addWidget(self.connect_button)
        buttons.addWidget(self.disconnect_button)
        form.addRow(buttons)
        self.serial_status = QtWidgets.QLabel("已断开")
        self.serial_status.setWordWrap(True)
        form.addRow(self.serial_status)
        self.fps_label = QtWidgets.QLabel("Actual FPS: 0.0 Hz")
        form.addRow(self.fps_label)
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self.port_combo.editTextChanged.connect(self._serial_config_changed)
        self.baud_spin.valueChanged.connect(self._serial_config_changed)
        self.protocol_combo.currentTextChanged.connect(self._serial_config_changed)
        self._mode_changed()
        return panel

    def _add_control(self, form, key, label):
        value = getattr(self.config, key)
        if isinstance(value, bool):
            control = QtWidgets.QCheckBox(label)
            control.toggled.connect(lambda v, k=key: self._parameter_changed(k, v))
            form.addRow(control)
        elif key == "roi_mode":
            control = QtWidgets.QComboBox()
            control.addItems(["3x3", "All Sensors"])
            control.currentTextChanged.connect(lambda v, k=key: self._parameter_changed(k, v))
            form.addRow(label, control)
        else:
            low, high, step, decimals = NUMERIC_LIMITS[key]
            control = QtWidgets.QSpinBox() if key == "trail_length" else QtWidgets.QDoubleSpinBox()
            if isinstance(control, QtWidgets.QDoubleSpinBox):
                control.setDecimals(decimals)
            control.setRange(low, high)
            control.setSingleStep(step)
            control.setKeyboardTracking(False)
            control.valueChanged.connect(lambda v, k=key: self._parameter_changed(k, v))
            form.addRow(label, control)
        self.controls[key] = control

    def _sync_controls(self, sync_gains=False):
        if sync_gains:
            self._calibration_draft = None
            self.calibration_confirm.setChecked(False)
            self._refresh_calibration_source()
            for control, gain in zip(self.gain_controls, self.config.raw_gains):
                control.setValue(gain)
        for key, control in self.controls.items():
            control.blockSignals(True)
            value = getattr(self.config, key)
            if isinstance(control, QtWidgets.QCheckBox):
                control.setChecked(value)
            elif isinstance(control, QtWidgets.QComboBox):
                control.setCurrentText(value)
            else:
                control.setValue(value)
            control.blockSignals(False)
        for widget, value in ((self.port_combo, self.config.port), (self.protocol_combo, self.config.protocol)):
            widget.blockSignals(True)
            widget.setCurrentText(value)
            widget.blockSignals(False)
        self.baud_spin.blockSignals(True)
        self.baud_spin.setValue(self.config.baudrate)
        self.baud_spin.blockSignals(False)
        self._mode_changed()

    def _parameter_changed(self, key, value):
        config = self.config.copy()
        setattr(config, key, value)
        if config.touch_off_threshold >= config.touch_on_threshold:
            if key == "touch_off_threshold":
                config.touch_on_threshold = config.touch_off_threshold + 1
            else:
                config.touch_off_threshold = max(0, config.touch_on_threshold - 1)
            self.statusBar().showMessage("已自动修正阈值，保持 OFF < ON", 5000)
        try:
            config.validate()
            self.engine.set_config(config)
            self.config = config
            self._sync_controls()
        except ValueError as exc:
            self.statusBar().showMessage(str(exc), 5000)
            self._sync_controls()

    def _serial_config_changed(self, *_args):
        self.config.port = self.port_combo.currentText().strip()
        self.config.baudrate = self.baud_spin.value()
        self.config.protocol = self.protocol_combo.currentText()
        self.engine.set_config(self.config)

    def _mode_changed(self, *_args):
        serial_mode = self.mode_combo.currentText() == "Serial"
        if not serial_mode:
            self._cancel_auto_connect()
        enabled = self.worker is None
        for widget in (self.port_combo, self.baud_spin, self.protocol_combo, self.refresh_button):
            widget.setEnabled(enabled and serial_mode)
        self.slow_rebound.setEnabled(not serial_mode)

    def refresh_ports(self):
        current = self.port_combo.currentText() or self.config.port
        try:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            self.port_combo.blockSignals(True)
            self.port_combo.clear()
            for port in ports:
                self.port_combo.addItem(port.device)
                self.port_combo.setItemData(self.port_combo.count() - 1, port.description,
                                           QtCore.Qt.ItemDataRole.ToolTipRole)
            self.port_combo.setCurrentText(current)
            self.port_combo.blockSignals(False)
        except Exception as exc:
            self.port_combo.blockSignals(False)
            self.statusBar().showMessage(f"端口扫描失败：{exc}")

    def start_simulation(self):
        if self.worker is None:
            self._cancel_auto_connect()
            self.mode_combo.setCurrentText("Simulation")
            self.start_source()

    def _cancel_auto_connect(self):
        self._auto_start_timer.stop()
        self._auto_probe_timer.stop()
        self._auto_connect_active = False
        self._auto_ports.clear()

    def _start_auto_connect(self):
        if self._closing or self.worker is not None or self.mode_combo.currentText() != "Serial":
            return
        self._auto_connect_active = True
        self._auto_ports = list(AUTO_CONNECT_PORTS)
        self._auto_failures = []
        self._try_next_auto_port()

    def _try_next_auto_port(self):
        if not self._auto_connect_active or self._closing or self.worker is not None:
            return
        if not self._auto_ports:
            self._cancel_auto_connect()
            details = "；".join(self._auto_failures)
            self.serial_status.setText(f"自动连接未成功：{details}。可手动选择端口后连接。")
            self.statusBar().showMessage("COM4、COM3 自动尝试结束；可手动连接。")
            return
        port = self._auto_ports.pop(0)
        self._auto_probe_reason = ""
        self.port_combo.blockSignals(True)
        self.port_combo.setCurrentText(port)
        self.port_combo.blockSignals(False)
        self.statusBar().showMessage(f"自动尝试 {port}，等待20通道Raw…")
        self.start_source(automatic=True)
        self._auto_probe_timer.start(AUTO_CONNECT_TIMEOUT_MS)

    def _accept_auto_connection(self):
        # 打开端口并不等于连接到了传感器；至少收到一个有效20通道Raw采样才接受。
        with self.engine.lock:
            ready = self.engine.source == "Serial" and self.engine.samples_processed > 0
        if self._auto_connect_active and ready and self.worker is not None and self.worker.isRunning():
            port = self.worker.port
            self._cancel_auto_connect()
            self.statusBar().showMessage(f"已自动连接 {port}；请保持空载，等待基线就绪。")
            return True
        return False

    def _auto_connect_timeout(self):
        if not self._auto_connect_active or self._accept_auto_connection():
            return
        self._auto_probe_reason = f"{AUTO_CONNECT_TIMEOUT_MS / 1000:g}秒内未收到有效20通道Raw"
        if self.worker is not None:
            self.serial_status.setText(f"{self.worker.port} 未收到有效Raw，正在关闭后尝试下一端口…")
            # 下一候选只能在当前线程结束、端口关闭后启动，不能同时占用两端口。
            self.worker.stop()

    def start_source(self, *, automatic=False):
        if self.worker is not None:
            return
        if not automatic:
            self._cancel_auto_connect()
        if self.mode_combo.currentText() == "Simulation":
            self.engine.begin_source("Simulation")
            worker = SimulationReader(self.engine, self)
            worker.set_slow_rebound(self.slow_rebound.isChecked())
            self.sensor_widget.simulation_enabled = True
            self.serial_status.setText("模拟100Hz：基线就绪后，左键按住并拖动阵列")
        else:
            self._serial_config_changed()
            if not self.config.port:
                self.statusBar().showMessage("请选择或输入 COM Port")
                return
            self.engine.begin_source("Serial")
            worker = SerialReader(self.config.port, self.config.baudrate, self.config.protocol, self.engine, self)
            worker.status_changed.connect(self._serial_status_changed)
            self.serial_status.setText("正在连接…")
        self.serial_diagnostics = {"parse_errors": 0, "discarded_bytes": 0, "rejected_channels": 0}
        self.worker = worker
        worker.finished.connect(self._worker_finished)
        self.mode_combo.setEnabled(False)
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)
        self._mode_changed()
        worker.start()

    def stop_source(self):
        self._cancel_auto_connect()
        if self.worker:
            self.worker.stop()
            self.disconnect_button.setEnabled(False)
            self.serial_status.setText("正在停止并关闭连接…")

    def _serial_status_changed(self, connected, message):
        self.serial_status.setText(message)

    def _worker_finished(self):
        worker = self.worker
        self._auto_probe_timer.stop()
        retry = self._auto_connect_active and not self._closing
        if retry and isinstance(worker, SerialReader):
            reason = worker.error_message or self._auto_probe_reason or "连接已结束，未收到有效Raw"
            self._auto_failures.append(f"{worker.port}: {reason}")
        if isinstance(worker, SerialReader):
            self.serial_diagnostics = {"parse_errors": worker.decoder.parse_errors,
                                       "discarded_bytes": worker.decoder.discarded_bytes,
                                       "rejected_channels": worker.rejected_channels}
        self.worker = None
        self.sensor_widget.simulation_enabled = False
        self.sensor_widget._pressed = False
        if isinstance(worker, SimulationReader):
            self.serial_status.setText("模拟已停止")
        if worker:
            worker.deleteLater()
        self.mode_combo.setEnabled(True)
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self._mode_changed()
        if self._closing:
            QtCore.QTimer.singleShot(0, self.close)
        elif retry:
            QtCore.QTimer.singleShot(0, self._try_next_auto_port)

    def _simulation_position(self, x, y, pressed):
        if isinstance(self.worker, SimulationReader):
            self.worker.set_position(x, y, pressed)

    def _set_slow_rebound(self, enabled):
        if isinstance(self.worker, SimulationReader):
            self.worker.set_slow_rebound(enabled)

    def reset_baseline(self):
        self.engine.reset_baseline()
        self.statusBar().showMessage("请释放阵列；稳定窗口建立完成前暂停触摸输出。")

    def reset_tracker(self):
        self.engine.reset_tracker()
        self.statusBar().showMessage("已清除位置参考、触摸状态、导数历史和轨迹；保留 baseline。", 5000)

    def save_config(self):
        for control in self.gain_controls:
            control.interpretText()
        if [control.value() for control in self.gain_controls] != self.config.raw_gains:
            self.apply_raw_gains()
            return
        try:
            self.config.save(CONFIG_PATH)
            self.statusBar().showMessage(f"配置已保存：{CONFIG_PATH}")
        except (OSError, ValueError) as exc:
            self.statusBar().showMessage(f"配置保存失败：{exc}")

    def load_config(self):
        if self.worker is not None:
            # 运行时载入算法参数；连接参数留待下一次连接采用。
            self.statusBar().showMessage("载入后算法参数立即生效；串口配置在下次连接生效。")
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Config", str(CONFIG_PATH), "JSON (*.json)")
        if not path:
            return
        try:
            config = AppConfig.load(path)
            self.engine.set_config(config)
            self.config = config
            self._sync_controls(sync_gains=True)
            self.statusBar().showMessage(f"已载入：{path}；增益变化会重建基线，请先松手；点击 Save Config 保存。")
        except (OSError, ValueError, TypeError) as exc:
            self.statusBar().showMessage(f"配置载入失败：{exc}")

    def toggle_recording(self):
        recorder = self.engine.recorder
        with self.engine.lock:
            if recorder.active:
                recorder.request_stop()
            elif not recorder.closing:
                try:
                    recorder.start(self.config, self.engine.revision)
                except (OSError, RuntimeError) as exc:
                    self.statusBar().showMessage(f"记录启动失败：{exc}")

    @staticmethod
    def _format(value, decimals=3):
        return "—" if value is None else f"{value:.{decimals}f}"

    def mark_observation(self, observation):
        if self.engine.mark_observation(observation):
            label = "实际接触" if observation == "contact" else "实际松手"
            self.statusBar().showMessage(f"已记录{label}标记", 3000)

    def refresh_view(self):
        if self._auto_connect_active:
            self._accept_auto_connection()
        snapshot = self.engine.snapshot()
        r = snapshot["result"]
        self.sensor_widget.set_snapshot(r, snapshot["trail"], self.config)
        self.fps_label.setText(f"Actual FPS: {snapshot['fps']:.1f} Hz（过去1秒）")
        self.baseline_label.setText(snapshot["baseline_status"])
        self.baseline_label.setStyleSheet("color: #ffd166;" if r and r.below_baseline_indices else "")
        direction = ""
        if r:
            if r.dx or r.dy:
                direction = ("→" if r.dx > 0 else "←") if abs(r.dx) > abs(r.dy) else ("↓" if r.dy > 0 else "↑")
            self.position_label.setText(f"X {self._format(r.x)}    Y {self._format(r.y)}\nDX {r.dx:+.3f}    DY {r.dy:+.3f}  {direction}")
        else:
            self.position_label.setText("X —    Y —\nDX +0.000    DY +0.000")
        state = r.state if r else "IDLE"
        suffix = " · 数据中断，位移参考已清除" if snapshot["stale"] else ""
        if r and not r.baseline_ready:
            suffix = " · 等待空载稳定基线"
        elif r and r.position_held:
            suffix += " · 衰减保护：保持最后坐标" if r.position_confidence == "decay_hold" else " · 衰减保护：确认新响应"
        self.touch_label.setText(state + suffix)
        values = {"fps": f"{snapshot['fps']:.1f} Hz", "state": state + suffix,
                  "last_touch_event": snapshot["last_touch_event"],
                  "stream_gaps": snapshot["stream_gaps"], "samples": snapshot["samples"],
                  "dropped_frames": "N/A（协议无序号）",
                  "rejected": snapshot["rejected_samples"] + self.serial_diagnostics["rejected_channels"],
                  "parse_errors": self.serial_diagnostics["parse_errors"],
                  "discarded_bytes": self.serial_diagnostics["discarded_bytes"],
                  "metadata": " / ".join(str(v) for v in snapshot["metadata"].values()) or "—"}
        if isinstance(self.worker, SerialReader):
            values["parse_errors"] = self.worker.decoder.parse_errors
            values["discarded_bytes"] = self.worker.decoder.discarded_bytes
            values["rejected"] = snapshot["rejected_samples"] + self.worker.rejected_channels
            if not r and self.worker.rejected_channels:
                self.touch_label.setText("收到非20通道数据，无法按4×5阵列处理")
            elif not r and self.worker.signal_only_frames:
                self.touch_label.setText("Signal-only 无 Raw；已请求 Debug，等待设备响应")
        if r:
            values.update(max_index=f"CH{r.max_index}", max_signal=self._format(r.max_signal, 1),
                          roi_transition="过渡中" if r.roi_transitioning else "稳定",
                          position_confidence={"inactive": "无有效触摸", "disabled": "保护关闭", "tracking": "正常跟随", "decay_hold": "整体衰减 / 暂停坐标", "recovering": "确认新响应 / 暂停坐标"}.get(r.position_confidence, r.position_confidence),
                          active_count=len(r.active_indices), total_signal=self._format(r.total_signal, 1))
            for key in ("x_raw", "y_raw", "x", "y", "dx", "dy"):
                values[key] = self._format(getattr(r, key))
        for key, label in self.debug_labels.items():
            label.setText(str(values.get(key, "—")))
        recorder = self.engine.recorder
        for button, shortcut in zip(self.marker_buttons, self.marker_shortcuts):
            enabled = recorder.active and not self._closing
            button.setEnabled(enabled)
            shortcut.setEnabled(enabled)
        self.record_button.setText("Stop Recording" if recorder.active else "正在落盘…" if recorder.closing else "Start Recording")
        self.record_button.setEnabled(not recorder.closing and not self._closing)
        if recorder.path:
            status = "记录中" if recorder.active else "正在落盘" if recorder.closing else "已关闭"
            self.record_label.setText(f"CSV ({status}, {recorder.rows_written} 帧)：{recorder.path}\n{recorder.error}")
        if self._closing and self.worker is None and not recorder.closing:
            QtCore.QTimer.singleShot(0, self.close)

    def closeEvent(self, event):
        # 异步等待接收线程和CSV队列完成，避免 QThread 销毁或漏写尾部。
        self._closing = True
        self.stop_source()
        with self.engine.lock:
            self.engine.recorder.request_stop()
        if self.worker is not None or self.engine.recorder.closing:
            event.ignore()
            return
        self.timer.stop()
        event.accept()
