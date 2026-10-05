"""纯传感器窗口，启动自动连接COM3，支持手动选择串口；不加载电机、压力计或CAN模块。"""
import json
import math
import time
from pathlib import Path
import pyqtgraph as pg
from qt_compat import QtCore, QtGui, QtWidgets
from config import BAUD_RATE, STARTUP_PORT, CHANNEL_COUNT, DEFAULT_HISTORY_LENGTH, GUI_REFRESH_HZ
from config import MIN_HISTORY_LENGTH, MAX_HISTORY_LENGTH, OUTPUT_MODE_DEBUG, OUTPUT_MODE_RUN
from channel_states import STATE_NAMES, IDLE_LIKE_STATES
from data_buffer import AlgoDataBuffer, RX_FPS_STALE_SECONDS
from centroid_view import CentroidView
from centroid_filter import CHANNEL_GRID, sensor_geometry
from touchpad_view import TouchpadPanel
from watch_face_view import WatchFaceCanvas, WatchFacePanel
from array_bar_view import ArrayBarView, ArrayBarPanel
from host_state_machine import HostStateMachine, HOST_STATE_NAMES, HOST_STATE_COLORS
from host_state_view import HostStatePanel, format_activation_reason
from slide_test_view import SlideTestPanel
from data_recorder import SerialDataRecorder
from rolling_serial_log import RollingSerialLog
from recent_data_export import RecentDataExportThread
from serial_reader import SerialReaderThread
from serial_ports import SerialPortScanner
from dfu_upgrade import DfuUpgradeThread, parse_cyacd2, resolve_dfu_image_path
from monitor_widgets import FlowLayout, ColumnToggleButton

CHANNEL_TABLE_COLUMNS = (
    ('channel', 'CH'), ('state', 'State'), ('hardware_raw_plot', 'Raw'),
    ('hardware_raw_value', 'Raw Val'), ('baseline_plot', 'Base'), ('signal_plot', 'Sig'),
    ('baseline_value', 'Base Val'), ('signal_value', 'Sig Val'), ('fast_k_plot', 'Fast K'),
    ('fast_k_value', 'Fast K [%/s]'), ('slow_k_plot', 'Slow K'), ('slow_k_value', 'Slow K [%/s]'),
    ('live_hardware_noise', '实时 Noise σ64'), ('state_background_select', 'Raw BG State CH'),
    ('activation_reason', '最近激活原因'))
CHANNEL_TABLE_COLUMN_INDEX = {key: i for i, (key, _) in enumerate(CHANNEL_TABLE_COLUMNS)}
STATE_COLORS = {-1:'#49324e', 0:'#204f3b', 1:'#764821', 2:'#245779', 3:'#535087',
                4:'#7c394b', 5:'#795020', 6:'#395c22', 7:'#216164', 8:'#5d5267'}
PLOT_COLUMNS = {'hardware_raw_plot':'raw', 'baseline_plot':'baseline', 'signal_plot':'signal',
                'fast_k_plot':'fast', 'slow_k_plot':'slow'}


def firmware_relative_path(finger_type='INDEX_FINGER'):
    return Path('build') / finger_type / 'UART/APP_CY8CPROTO-041TP/Debug' / f'mtb-example-psoc4-msclp-finger-{finger_type}_crc.cyacd2'


def find_firmware_path(finger_type='INDEX_FINGER'):
    for root in (Path.cwd(), *Path(__file__).resolve().parents):
        path = root / firmware_relative_path(finger_type)
        if path.is_file():
            return path
    return None


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.reader = self.dfu_thread = self._recent_export_worker = None
        self.current_port = self.current_device_info = None
        self.channel_count = CHANNEL_COUNT
        self.data_buffer = AlgoDataBuffer(DEFAULT_HISTORY_LENGTH)
        self.host_state_machine = HostStateMachine(history_length=DEFAULT_HISTORY_LENGTH)
        self.data_recorder = SerialDataRecorder()
        self.rolling_log = RollingSerialLog()
        self._last_snapshot_version = -1
        self._last_centroid_key = None
        self._centroid_signal_threshold = 100
        self._state_regions = []
        self._params_dirty = self._params_pending = self._connected = False
        self._closing = False
        self._requested_params = None
        self._params_poll_remaining = 0
        self._loading = True
        self._config_path = Path(__file__).with_name('sensor_display_config.json')
        self._save_timer = QtCore.QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.setInterval(400)
        self._save_timer.timeout.connect(self.save_display_config)
        self._build_ui()
        self._load_display_config()
        self._loading = False
        self.refresh_ports()
        self._set_connected(False)
        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.setInterval(max(1, round(1000 / GUI_REFRESH_HZ)))
        self.refresh_timer.timeout.connect(self.refresh_plots)
        self.refresh_timer.start()
        self.param_timer = QtCore.QTimer(self)
        self.param_timer.setInterval(150)
        self.param_timer.timeout.connect(self._poll_params)
        self.log_timer = QtCore.QTimer(self)
        self.log_timer.setInterval(1000)
        self.log_timer.timeout.connect(self._refresh_log_status)
        self.log_timer.start()
        self._refresh_log_status()
        self.resize(1700, 1000)
        self.setWindowTitle('20通道传感器监控 · V21')
        QtWidgets.QApplication.instance().installEventFilter(self)
        # 等界面初始化完成后连接；首次成功前允许设备稍晚插入或释放端口。
        self._startup_connect_pending = True
        self._startup_connect_timer = QtCore.QTimer(self)
        self._startup_connect_timer.setSingleShot(True)
        self._startup_connect_timer.timeout.connect(self._try_startup_connection)
        self._startup_connect_timer.start(0)

    def eventFilter(self, watched, event):
        if (event.type() == QtCore.QEvent.Type.KeyPress and event.key() == QtCore.Qt.Key.Key_Escape
                or event.type() == QtCore.QEvent.Type.ApplicationDeactivate):
            self.touchpad.stop_mouse()
            self.watch_face.cancel_gesture()
        if event.type() == QtCore.QEvent.Type.Wheel:
            target = watched
            while target is not None:
                if isinstance(target, (QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox, QtWidgets.QSlider)):
                    event.ignore()
                    return True
                target = target.parent()
        return super().eventFilter(watched, event)

    @staticmethod
    def _spin(low, high, value):
        widget = QtWidgets.QSpinBox()
        widget.setRange(low, high)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        widget.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        return widget

    @staticmethod
    def _button(text, callback, layout):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(callback)
        layout.addWidget(button)
        return button

    @staticmethod
    def _group(title, parent_layout):
        group = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QVBoxLayout(group)
        parent_layout.addWidget(group)
        return group, layout

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        connection = QtWidgets.QWidget()
        connection_layout = QtWidgets.QVBoxLayout(connection)
        top = QtWidgets.QHBoxLayout()
        connection_layout.addLayout(top)
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(240)
        top.addWidget(QtWidgets.QLabel('传感器串口'))
        top.addWidget(self.port_combo)
        self.scan_button = self._button('刷新串口', self.refresh_ports, top)
        self.connect_button = self._button('Connect', self.toggle_connection, top)
        connection_layout.addWidget(QtWidgets.QLabel(f'{BAUD_RATE} baud · 8N1'))
        mode_row = QtWidgets.QHBoxLayout()
        connection_layout.addLayout(mode_row)
        self.recal_button = self._button('重新校准基线', self.recalibrate, mode_row)
        self.output_mode_combo = QtWidgets.QComboBox()
        self.output_mode_combo.addItem('Debug · Raw / Baseline / Signal / 状态 / K', OUTPUT_MODE_DEBUG)
        self.output_mode_combo.addItem('Run · 仅Signal', OUTPUT_MODE_RUN)
        self.output_mode_combo.currentIndexChanged.connect(self.send_output_mode_command)
        mode_row.addWidget(self.output_mode_combo)
        self.mode_apply_button = self._button('重新应用', self.send_output_mode_command, mode_row)
        self.live_status = QtWidgets.QLabel('尚未收到传感器数据')
        self.main_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        root.addWidget(self.main_splitter, 1)
        graphs = QtWidgets.QWidget()
        graph_layout = QtWidgets.QVBoxLayout(graphs)
        graph_layout.setContentsMargins(0,0,0,0)
        settings = QtWidgets.QHBoxLayout()
        source_row = QtWidgets.QHBoxLayout()
        graph_layout.addLayout(source_row)
        self.waveform_source = QtWidgets.QComboBox()
        self.waveform_source.addItem('固件上报', 'firmware')
        self.waveform_source.addItem('上位机基线实验', 'host')
        source_row.addWidget(QtWidgets.QLabel('波形来源'))
        source_row.addWidget(self.waveform_source)
        self.host_observation_label = QtWidgets.QLabel('实验观察通道')
        self.host_observation_channel = QtWidgets.QComboBox()
        for channel in range(CHANNEL_COUNT):
            self.host_observation_channel.addItem(f'CH{channel}', channel)
        source_row.addWidget(self.host_observation_label)
        source_row.addWidget(self.host_observation_channel)
        self.host_observation_label.hide()
        self.host_observation_channel.hide()
        source_row.addStretch(1)
        settings.addWidget(QtWidgets.QLabel('显示帧数'))
        self.history_input = self._spin(MIN_HISTORY_LENGTH, MAX_HISTORY_LENGTH, DEFAULT_HISTORY_LENGTH)
        self.history_input.valueChanged.connect(self._history_changed)
        settings.addWidget(self.history_input)
        settings.addWidget(QtWidgets.QLabel('Raw背景状态通道'))
        self.state_background_channel_combo = QtWidgets.QComboBox()
        for i in range(CHANNEL_COUNT):
            self.state_background_channel_combo.addItem(f'CH{i}', i)
        self.state_background_channel_combo.currentIndexChanged.connect(self._background_changed)
        settings.addWidget(self.state_background_channel_combo)
        self._button('坐标自适应', self.auto_range, settings)
        self.plots = {}
        for key, title, unit in (('raw','Raw / Baseline','count'), ('signal','Signal','count'), ('k','Fast K / Slow K','%/s')):
            plot = pg.PlotWidget(title=title, background='#080d13')
            plot.setLabel('left', unit)
            plot.setLabel('bottom', '采样帧')
            plot.showGrid(x=True, y=True, alpha=.15)
            plot.addLegend(offset=(10,10), colCount=2)
            graph_layout.addWidget(plot, 1)
            self.plots[key] = plot
        self.raw_plot, self.signal_plot, self.k_plot = (self.plots[k] for k in ('raw','signal','k'))
        self.signal_plot.setXLink(self.raw_plot)
        self.k_plot.setXLink(self.raw_plot)
        self.host_state_plot = pg.PlotWidget(title='上位机实验状态 · 所选背景通道', background='#080d13')
        self.host_state_plot.setLabel('bottom', '采样帧')
        self.host_state_plot.getAxis('left').setTicks([list(HOST_STATE_NAMES.items())])
        self.host_state_plot.setYRange(-2.5, 3.5)
        self.host_state_plot.setXLink(self.raw_plot)
        self.host_state_curve = self.host_state_plot.plot(pen=pg.mkPen('#63d8ef', width=2), stepMode='left')
        graph_layout.addWidget(self.host_state_plot, 1)
        self.host_state_plot.hide()
        self.curves = {key: [] for key in PLOT_COLUMNS.values()}
        for i in range(CHANNEL_COUNT):
            color = pg.intColor(i, hues=CHANNEL_COUNT, values=1, maxValue=240, minValue=160)
            for key in self.curves:
                plot = self.raw_plot if key in ('raw','baseline') else self.signal_plot if key == 'signal' else self.k_plot
                pen = pg.mkPen(color, width=1.5, style=QtCore.Qt.PenStyle.DashLine if key in ('baseline','slow') else QtCore.Qt.PenStyle.SolidLine)
                self.curves[key].append(plot.plot(pen=pen, name=f'CH{i} {key}'))
        self.plot_tabs = QtWidgets.QTabWidget()
        self.plot_tabs.addTab(graphs, '波形图')
        self.centroid_view = CentroidView()
        self.plot_tabs.addTab(self.centroid_view, '阵列质心')
        self.plot_tabs.setCurrentWidget(self.centroid_view)
        self.main_splitter.addWidget(self.plot_tabs)
        self.control_tabs = QtWidgets.QTabWidget()
        self.control_tabs.setMinimumWidth(560)
        self.control_tabs.setUsesScrollButtons(True)
        self.main_splitter.addWidget(self.control_tabs)
        self.main_splitter.setSizes([900,800])
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        monitor = QtWidgets.QWidget()
        controls = QtWidgets.QVBoxLayout(monitor)
        scroll.setWidget(monitor)
        self.control_tabs.addTab(scroll, '监控')
        _, centroid_threshold_layout = self._group('质心显示阈值', controls)
        threshold_row = QtWidgets.QHBoxLayout()
        threshold_row.addWidget(QtWidgets.QLabel('单通道 Signal 阈值'))
        self.centroid_threshold_input = self._spin(0, 65535, 100)
        threshold_row.addWidget(self.centroid_threshold_input)
        threshold_row.addWidget(QtWidgets.QLabel('count'))
        self.centroid_threshold_button = self._button('设置并保存', self._apply_centroid_threshold, threshold_row)
        threshold_row.addStretch(1)
        centroid_threshold_layout.addLayout(threshold_row)
        threshold_note = QtWidgets.QLabel('至少一路Signal达到阈值才开始显示；信号失效后锁定原位置100毫秒，再隐藏。期间恢复则继续跟随。点击设置并保存后生效，下次启动自动加载。')
        threshold_note.setWordWrap(True)
        centroid_threshold_layout.addWidget(threshold_note)
        self.data_monitor_group, monitor_layout = self._group('20通道数据', controls)
        record_row = QtWidgets.QHBoxLayout()
        self.recent_export_buttons = {m: self._button(f'转存最近{m}分钟', lambda _=False, minutes=m: self.export_recent(minutes), record_row) for m in (1,3,5)}
        monitor_layout.addLayout(record_row)
        note = QtWidgets.QLabel('自动缓存最近5分钟；点击时转存。绿色显示、灰色隐藏；拖动数据列按钮排序。')
        note.setWordWrap(True)
        monitor_layout.addWidget(note)
        monitor_layout.addWidget(QtWidgets.QLabel('数据列'))
        columns = QtWidgets.QWidget()
        self.column_flow = FlowLayout(columns)
        monitor_layout.addWidget(columns)
        self.column_order = [key for key, _ in CHANNEL_TABLE_COLUMNS if key != 'channel']
        self.table_column_buttons = {}
        hidden = {'fast_k_plot','slow_k_plot','fast_k_value','slow_k_value','live_hardware_noise'}
        for key, title in CHANNEL_TABLE_COLUMNS:
            if key == 'channel':
                continue
            button = ColumnToggleButton(key, title)
            button.setChecked(key not in hidden)
            button.toggled.connect(self._column_visibility_changed)
            button.reorder_requested.connect(self._reorder_column)
            self.table_column_buttons[key] = button
            self.column_flow.addWidget(button)
        monitor_layout.addWidget(QtWidgets.QLabel('数据行'))
        rows = QtWidgets.QWidget()
        row_flow = FlowLayout(rows)
        self.table_row_buttons = {}
        for i in range(CHANNEL_COUNT):
            button = QtWidgets.QPushButton(f'CH{i}')
            button.setCheckable(True)
            button.setChecked(True)
            button.toggled.connect(self._row_visibility_changed)
            row_flow.addWidget(button)
            self.table_row_buttons[f'channel_{i}'] = button
        monitor_layout.addWidget(rows)
        self.table = QtWidgets.QTableWidget(CHANNEL_COUNT, len(CHANNEL_TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels([title for _, title in CHANNEL_TABLE_COLUMNS])
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        monitor_layout.addWidget(self.table)
        self.plot_checkboxes = {key: [] for key in self.curves}
        self.value_labels = {key: [] for key in ('state','hardware_raw_value','baseline_value','signal_value','fast_k_value','slow_k_value','live_hardware_noise','activation_reason')}
        self.state_background_buttons = {}
        self.state_background_button_group = QtWidgets.QButtonGroup(self)
        for i in range(CHANNEL_COUNT):
            self.table.setRowHeight(i, 30)
            for col, (key, _) in enumerate(CHANNEL_TABLE_COLUMNS):
                if key == 'channel':
                    item = QtWidgets.QTableWidgetItem(f'CH{i}')
                    item.setToolTip(f'MatrixButtons0 · RX{i//4} / TX{i%4}')
                    self.table.setItem(i, col, item)
                elif key in PLOT_COLUMNS or key == 'state_background_select':
                    control = QtWidgets.QRadioButton() if key == 'state_background_select' else QtWidgets.QCheckBox()
                    if key == 'state_background_select':
                        self.state_background_button_group.addButton(control, i)
                        self.state_background_buttons[i] = control
                        control.setChecked(i == 0)
                        control.toggled.connect(lambda checked, ch=i: self.state_background_channel_combo.setCurrentIndex(ch) if checked else None)
                    else:
                        curve_key = PLOT_COLUMNS[key]
                        control.setChecked(i == 0 and curve_key in ('raw','baseline','signal'))
                        control.toggled.connect(self._curve_visibility_changed)
                        self.plot_checkboxes[curve_key].append(control)
                    wrapper = QtWidgets.QWidget()
                    layout = QtWidgets.QHBoxLayout(wrapper)
                    layout.setContentsMargins(0,0,0,0)
                    layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                    layout.addWidget(control)
                    self.table.setCellWidget(i,col,wrapper)
                else:
                    label = QtWidgets.QLabel('--')
                    label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                    if key == 'activation_reason':
                        label.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter)
                        label.setMinimumWidth(225)
                        label.setStyleSheet('padding-left:6px;')
                    self.value_labels[key].append(label)
                    self.table.setCellWidget(i,col,label)
        controls.addStretch(1)
        config_tab = QtWidgets.QWidget()
        config_layout = QtWidgets.QVBoxLayout(config_tab)
        config_layout.addWidget(connection)
        _, waveform_layout = self._group('波形显示', config_layout)
        waveform_layout.addLayout(settings)
        _, centroid_layout = self._group('阵列质心', config_layout)
        self.centroid_raw_checkbox = QtWidgets.QCheckBox('使用Raw Val计算质心')
        self.centroid_raw_checkbox.toggled.connect(self._centroid_raw_changed)
        centroid_layout.addWidget(self.centroid_raw_checkbox)
        raw_note = QtWidgets.QLabel('原始Raw Val不扣Baseline，含静态底值，可能压缩移动范围；需Debug模式。显示阈值和释放判断仍使用Signal。')
        raw_note.setWordWrap(True)
        centroid_layout.addWidget(raw_note)
        self.centroid_raw_difference_checkbox = QtWidgets.QCheckBox('使用Raw均值差归一化计算质心')
        self.centroid_raw_difference_checkbox.toggled.connect(self._centroid_raw_difference_changed)
        centroid_layout.addWidget(self.centroid_raw_difference_checkbox)
        difference_note = QtWidgets.QLabel('与原始Raw模式互斥，两项均关闭时使用Signal。每点减去20路Raw均值，负差置零，再按正差总和归一化；全相等时无质心。需Debug模式，显示阈值和释放判断仍使用Signal。通道固有底值差异仍可能影响位置，小差异也可能被放大。')
        difference_note.setWordWrap(True)
        centroid_layout.addWidget(difference_note)
        self.centroid_local_2x2_checkbox = QtWidgets.QCheckBox('仅使用最强2×2四通道')
        self.centroid_local_2x2_checkbox.setChecked(True)
        self.centroid_local_2x2_checkbox.toggled.connect(self._centroid_local_2x2_changed)
        centroid_layout.addWidget(self.centroid_local_2x2_checkbox)
        self.centroid_smoothing_checkbox = QtWidgets.QCheckBox('启用X/Y独立平滑（One Euro）')
        self.centroid_smoothing_checkbox.setChecked(True)
        self.centroid_smoothing_checkbox.toggled.connect(self._centroid_smoothing_changed)
        centroid_layout.addWidget(self.centroid_smoothing_checkbox)
        centroid_note = QtWidgets.QLabel('勾选2×2时按当前模式的权重选取总和最大的相邻四通道，同分时沿用上一选区；取消勾选使用全部20通道。切换立即生效并重置坐标平滑。慢动时加强平滑，快动时减小延迟。释放时原位锁定100毫秒再隐藏。缩放只改变显示尺寸。')
        centroid_note.setWordWrap(True)
        centroid_layout.addWidget(centroid_note)
        scale_row = QtWidgets.QHBoxLayout()
        scale_row.addWidget(QtWidgets.QLabel('阵列缩放'))
        self.centroid_scale_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.centroid_scale_slider.setRange(30, 100)
        self.centroid_scale_slider.setValue(70)
        self.centroid_scale_slider.setSingleStep(1)
        self.centroid_scale_slider.setPageStep(10)
        self.centroid_scale_label = QtWidgets.QLabel('70%')
        self.centroid_scale_label.setMinimumWidth(45)
        self.centroid_scale_slider.valueChanged.connect(self._centroid_scale_changed)
        scale_row.addWidget(self.centroid_scale_slider, 1)
        scale_row.addWidget(self.centroid_scale_label)
        centroid_layout.addLayout(scale_row)
        config_layout.addStretch(1)
        self.control_tabs.addTab(config_tab, '波形设置')
        parameter_tab = QtWidgets.QWidget()
        parameter_layout = QtWidgets.QVBoxLayout(parameter_tab)
        self._build_parameters(parameter_layout)
        parameter_layout.addStretch(1)
        self.control_tabs.addTab(parameter_tab, '三参数')
        self._build_log_tab()
        self._build_dfu()
        self.touchpad = TouchpadPanel(self._log_touchpad)
        self.plot_tabs.addTab(self.touchpad.canvas, '触摸板光标')
        self.control_tabs.addTab(self.touchpad, '触摸板')
        for widget in self.touchpad.setting_inputs():
            widget.valueChanged.connect(self._schedule_save)
        self.touchpad.source.currentIndexChanged.connect(self._schedule_save)
        self.touchpad.position_decay_enabled.toggled.connect(self._schedule_save)
        self.slide_test = SlideTestPanel(self.rolling_log, self._slide_test_context,
                                        self._prepare_slide_test, self._cleanup_slide_test)
        self.plot_tabs.addTab(self.slide_test, '滑动测试')
        for widget in (self.slide_test.horizontal_count, self.slide_test.vertical_count,
                       self.slide_test.slide_seconds, self.slide_test.prepare_seconds,
                       self.slide_test.release_seconds, self.slide_test.horizontal_distance,
                       self.slide_test.vertical_distance):
            widget.valueChanged.connect(self._schedule_save)
        self.slide_test.pressure.currentIndexChanged.connect(self._schedule_save)
        self.slide_test.notes.textChanged.connect(self._schedule_save)
        self.watch_face = WatchFaceCanvas()
        self.watch_settings = WatchFacePanel(self.watch_face)
        self.plot_tabs.addTab(self.watch_face, '圆形表盘')
        self.control_tabs.addTab(self.watch_settings, '表盘设置')
        self.watch_settings.settings_changed.connect(self._schedule_save)
        self.watch_settings.reference_requested.connect(lambda: self.control_tabs.setCurrentWidget(self.touchpad))
        self.array_bars = ArrayBarView()
        self.array_bar_settings = ArrayBarPanel(self.array_bars)
        self.plot_tabs.addTab(self.array_bars, '3D柱状图')
        self.control_tabs.addTab(self.array_bar_settings, '柱状图设置')
        self.array_bar_settings.settings_changed.connect(self._schedule_save)
        self.array_bar_settings.settings_changed.connect(self._sync_touchpad_raw_gains)
        self.array_bar_settings.settings_changed.connect(self._refresh_array_bars)
        self.host_state_settings = HostStatePanel(self.host_state_machine, self._save_host_state_parameters)
        self.control_tabs.addTab(self.host_state_settings, '基线与状态机')
        self.host_state_settings.reset_requested.connect(self._reset_host_state)
        self.host_state_settings.waveform_requested.connect(self._show_host_waveforms)
        self.host_state_settings.observation_changed.connect(self._select_host_observation)
        self.host_observation_channel.currentIndexChanged.connect(self._select_host_observation)
        self.waveform_source.currentIndexChanged.connect(self._waveform_source_changed)
        self.touchpad.motion_sample.connect(self.watch_face.sensor_sample)
        self.touchpad.system_mouse.toggled.connect(self._watch_mouse_changed)
        self.plot_tabs.currentChanged.connect(self._watch_tab_changed)
        self.plot_tabs.setCurrentWidget(self.watch_face)
        self.status_label = QtWidgets.QLabel('选择传感器串口后连接')
        self.status_label.setWordWrap(True)
        root.addWidget(self.live_status)
        root.addWidget(self.status_label)
        self._column_visibility_changed()
        self._row_visibility_changed()
        self._curve_visibility_changed()
        self.setStyleSheet('''
            QMainWindow, QWidget { background:#080d13; color:#d9e7f0; font-family:"Segoe UI","Microsoft YaHei"; font-size:13px; }
            QGroupBox { border:1px solid #33495a; border-radius:6px; margin-top:12px; padding-top:8px; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; color:#63d8ef; }
            QPushButton { background:#253745; border:1px solid #486173; border-radius:4px; padding:5px 8px; }
            QPushButton:checked { background:#168c48; border-color:#8be277; color:white; }
            QPushButton:disabled { color:#6a7781; }
            QHeaderView::section { background:#1b2935; padding:6px; border:0; }
            QTableWidget { gridline-color:#263441; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background:#121e28; border:1px solid #456074; padding:4px; }
            QCheckBox::indicator { width:16px; height:16px; border:1px solid #617e92; border-radius:2px; background:#080d13; }
            QCheckBox::indicator:checked { background:#52d2f3; border-color:#8deaff; }
            QRadioButton::indicator { width:16px; height:16px; border:1px solid #617e92; border-radius:8px; background:#080d13; }
            QRadioButton::indicator:checked { border:4px solid #52d2f3; background:#183a48; }
            QTabBar::tab { padding:8px 18px; background:#182b39; }
            QTabBar::tab:selected { background:#28536b; }
        ''')

    def _build_parameters(self, controls):
        self.matrix_config_group, layout = self._group('三参数 · MatrixButtons0 全部20通道（仅RAM）', controls)
        row = QtWidgets.QHBoxLayout()
        self.nsub_input = self._spin(1,16383,300)
        self.tx_clk_div_input = self._spin(4,4096,100)
        self.tx_clk_div_input.setSingleStep(2)
        self.ref_cdac_input = self._spin(7,255,12)
        for title, widget in (('Nsub',self.nsub_input), ('TxDivider',self.tx_clk_div_input), ('RefCDAC',self.ref_cdac_input)):
            row.addWidget(QtWidgets.QLabel(title))
            row.addWidget(widget)
            widget.valueChanged.connect(self._parameter_edited)
        layout.addLayout(row)
        buttons = QtWidgets.QHBoxLayout()
        self.params_apply_button = self._button('应用三参数',self.apply_parameters,buttons)
        self.params_read_button = self._button('回读20通道',self.request_parameters,buttons)
        layout.addLayout(buttons)
        self.nsub_status_label = QtWidgets.QLabel('等待设备回读；开机采用Configurator参数。')
        self.nsub_status_label.setWordWrap(True)
        layout.addWidget(self.nsub_status_label)
        self.capsense_parameter_table = QtWidgets.QTableWidget(0,5)
        self.capsense_parameter_table.setHorizontalHeaderLabels(['CH','Nsub 配置/硬件','TxDivider 配置/硬件','RefCDAC 配置/硬件','Max Raw'])
        self.capsense_parameter_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.capsense_parameter_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.capsense_parameter_table.verticalHeader().hide()
        self.capsense_parameter_table.setMinimumHeight(180)
        self.capsense_parameter_table.setMaximumHeight(300)
        layout.addWidget(self.capsense_parameter_table)
        self.capsense_parameter_status_label = QtWidgets.QLabel('尚未回读')
        self.capsense_parameter_status_label.setWordWrap(True)
        layout.addWidget(self.capsense_parameter_status_label)

    def _build_log_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        note = QtWidgets.QLabel('连接后自动记录全部串口 RX/TX、解析帧和连接事件。包含20通道的 Raw、Baseline、Signal、状态、K，以及设备和参数回读。\n\n独立线程每5秒或4 MiB批量写盘，单文件32 MiB，滚动保留16个文件（约512 MiB）。正常关闭会在后台补写缓存；异常退出可能损失尚未落盘的尾部。\n\nRun模式仅记录固件实际上传的字段；查看完整基线与状态请使用Debug模式。原始收发数据可用于复核未识别帧。')
        note.setWordWrap(True)
        layout.addWidget(note)
        centroid_note = QtWidgets.QLabel('质心另记为centroid事件：包含完整20路Signal和本帧Raw（若有）、参与计算的通道及权重总和、原始/滤波/显示X/Y、采样序号与时间、数据源、质心模式和滤波参数。仅新采样或显示模式变化时记录，沿用后台批量写入。')
        centroid_note.setWordWrap(True)
        layout.addWidget(centroid_note)
        path = QtWidgets.QLineEdit(str(self.rolling_log.directory))
        path.setReadOnly(True)
        layout.addWidget(path)
        self.log_status_label = QtWidgets.QLabel()
        self.log_status_label.setWordWrap(True)
        self.log_status_label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.log_status_label)
        layout.addStretch(1)
        self.control_tabs.addTab(tab, '串口日志')

    def _refresh_log_status(self):
        status = self.rolling_log.status()
        message = (f"已写入 {status['written']} 条 · {status['batches']} 批\n"
                   f"待处理 {status['queued']} 条 · 待写入 {status['pending']} 条\n"
                   f"丢失 {status['dropped']} 条 · 最近写入 {status['last_write'] or '尚未写入'}\n"
                   f"当前文件：{status['path'] or '连接并接收数据后自动创建'}")
        if status['error']:
            message += '\n写盘失败（后台重试）：' + status['error']
        if status['retention_error']:
            message += '\n旧日志清理失败，保留容量可能超限：' + status['retention_error']
        self.log_status_label.setText(message)
        problem = bool(status['error'] or status['dropped'] or status['retention_error'])
        self.control_tabs.setTabText(self.control_tabs.indexOf(self.log_status_label.parentWidget()), '串口日志 ⚠' if problem else '串口日志')
        self.log_status_label.setStyleSheet('color:#ff8d78;' if problem else '')

    def _log_touchpad(self, frame):
        if self._connected:
            self.rolling_log.submit('touchpad', port=self.current_port, frame=frame)

    def _slide_test_context(self):
        return dict(connected=self._connected, port=self.current_port,
                    sample=self.data_buffer.latest_sensor_sample(),
                    host_ready=all(c.state != -1 for c in self.host_state_machine.channels),
                    host_generation=self.host_state_machine.generation,
                    host_parameters=self.host_state_machine.parameters.settings(),
                    raw_gains=self.host_state_machine.gains,
                    touchpad=self.touchpad.settings(), tracking_enabled=self.touchpad.enabled_box.isChecked(),
                    position_reference_ready=self.touchpad.tracker.position_reference is not None,
                    hardware_signature=getattr(self, '_touchpad_config_signature', None),
                    matrix_readback=getattr(self, '_last_matrix_readback', None),
                    channel_readback=getattr(self, '_last_channel_readback', None),
                    channel_grid=CHANNEL_GRID, sensor_geometry=sensor_geometry(),
                    coordinate_unit='cell', delta_unit='pixel')

    def _prepare_slide_test(self):
        self._slide_tracking_was_enabled = self.touchpad.enabled_box.isChecked()
        self.touchpad.stop_mouse()
        self.touchpad.enabled_box.setChecked(True)

    def _cleanup_slide_test(self):
        if getattr(self, '_slide_tracking_was_enabled', True) is False:
            self.touchpad.enabled_box.setChecked(False)
        self._slide_tracking_was_enabled = True

    def _sync_touchpad_raw_gains(self):
        # 无论柱状图当前选择Signal还是Raw，触摸板始终使用保存的逐通道Raw增益。
        self.touchpad.set_raw_gains(self.array_bar_settings.settings()['raw_gains'])
        previous_gains = self.host_state_machine.gains
        self.host_state_machine.set_gains(self.array_bar_settings.settings()['raw_gains'])
        if previous_gains != self.host_state_machine.gains:
            self._reset_host_state('Raw 增益已改变；请保持释放，重新初始化')

    def _log_host_settings(self):
        if self._connected:
            self.rolling_log.submit('host_state_config', port=self.current_port,
                                    frame=dict(parameters=self.host_state_machine.parameters.settings(),
                                               raw_gains=self.host_state_machine.gains,
                                               reason=self.host_state_machine.reason))

    def _reset_host_state(self, reason='请保持释放，重新初始化'):
        # 跳过重置前已到达 GUI 缓存的帧，初始化仅使用此后的 Raw。
        samples = self.data_buffer.pointer_samples_after(0)
        if samples:
            self.host_state_machine.last_sequence = samples[-1]['sequence']
        self.host_state_machine.reset(reason)
        if self.touchpad.tracker.source in self.touchpad.tracker.HOST_SOURCES:
            self.touchpad.invalidate(reason)
        self.host_state_settings.refresh()
        self._refresh_array_bars()
        self._last_snapshot_version = -1
        self._log_host_settings()

    def _show_host_waveforms(self):
        self.waveform_source.setCurrentIndex(1)
        self._select_host_observation(self.host_observation_channel.currentData())
        self.plot_tabs.setCurrentIndex(0)

    def _refresh_array_bars(self, sample=None):
        # 派生 Signal 仅附加到视图样本，不改写固件数据；必须与当前 Raw 同帧。
        if sample is None:
            sample = self.data_buffer.latest_sensor_sample()
        if sample is not None:
            engine = self.host_state_machine
            values = None
            message = engine.reason
            if not sample['has_raw']:
                message = '上位机计算 Signal需要Raw · 请在波形设置中选择Debug输出'
            elif (engine.last_timestamp is not None
                  and sample['sequence'] == engine.last_sequence
                  and sample['sample_timestamp'] == engine.last_timestamp
                  and sample['age_seconds'] <= engine.parameters.gap_seconds):
                values = tuple(channel.signal for channel in engine.channels)
            sample = dict(sample, host_signal=values, host_signal_message=message)
        self.array_bars.set_sample(sample)

    def _select_host_observation(self, channel):
        # 统一观察选择；只修改显示，不改变通道算法、基线或采样历史。
        for combo in (self.host_observation_channel, self.host_state_settings.observation_channel):
            combo.blockSignals(True)
            combo.setCurrentIndex(channel)
            combo.blockSignals(False)
        if self.waveform_source.currentData() == 'host':
            self.state_background_channel_combo.blockSignals(True)
            self.state_background_channel_combo.setCurrentIndex(channel)
            self.state_background_channel_combo.blockSignals(False)
            self.state_background_buttons[channel].setChecked(True)
            for key, boxes in self.plot_checkboxes.items():
                for i, box in enumerate(boxes):
                    box.blockSignals(True)
                    box.setChecked(i == channel and key != 'slow')
                    box.blockSignals(False)
            self._curve_visibility_changed()
        self._last_snapshot_version = -1
        self._schedule_save()

    def _waveform_source_changed(self, *_):
        host = self.waveform_source.currentData() == 'host'
        self.raw_plot.setTitle('实验 input / baseline · Raw × 增益 → 低通' if host else 'Raw / Baseline · 固件上报')
        self.signal_plot.setTitle('实验 Signal（仅 activate）' if host else 'Signal · 固件上报')
        self.k_plot.setTitle('实验压力变化速度 [%/s] · 仅 activate 显示' if host else 'Fast K / Slow K')
        self.host_state_plot.setVisible(host)
        self.host_observation_label.setVisible(host)
        self.host_observation_channel.setVisible(host)
        for key, title in (('state', '实验状态' if host else 'State'),
                           ('hardware_raw_plot', 'Input' if host else 'Raw'),
                           ('hardware_raw_value', 'Input Val' if host else 'Raw Val'),
                           ('fast_k_plot', '压力速度' if host else 'Fast K'),
                           ('fast_k_value', '速度 [%/s]' if host else 'Fast K [%/s]')):
            self.table.horizontalHeaderItem(CHANNEL_TABLE_COLUMN_INDEX[key]).setText(title)
        if host:
            self._select_host_observation(self.host_observation_channel.currentData())
        self._curve_visibility_changed()

    def _watch_tab_changed(self, *_):
        if self.plot_tabs.currentWidget() is self.watch_face:
            self.touchpad.stop_mouse()
            self.control_tabs.setCurrentWidget(self.watch_settings)
        elif self.plot_tabs.currentWidget() is self.array_bars:
            self.control_tabs.setCurrentWidget(self.array_bar_settings)

    def _watch_mouse_changed(self, enabled):
        if enabled and self.plot_tabs.currentWidget() is self.watch_face:
            self.touchpad.stop_mouse()
            self.status_label.setText('表盘直接接收触摸手势，无需开启系统鼠标控制')

    def _centroid_smoothing_changed(self, enabled):
        self.centroid_view.smoothing_enabled = enabled
        self._last_centroid_key = None
        self._schedule_save()

    def _centroid_local_2x2_changed(self, enabled):
        self.centroid_view.position_filter.set_local_2x2_enabled(enabled)
        self._last_centroid_key = None
        self._refresh_centroid(self.data_buffer.latest_sensor_sample())
        self._schedule_save()

    def _centroid_raw_changed(self, enabled):
        if enabled:
            self.centroid_raw_difference_checkbox.blockSignals(True)
            self.centroid_raw_difference_checkbox.setChecked(False)
            self.centroid_raw_difference_checkbox.blockSignals(False)
        self.centroid_view.position_filter.set_raw_enabled(enabled)
        self._last_centroid_key = None
        self._refresh_centroid(self.data_buffer.latest_sensor_sample())
        self._schedule_save()

    def _centroid_raw_difference_changed(self, enabled):
        if enabled:
            self.centroid_raw_checkbox.blockSignals(True)
            self.centroid_raw_checkbox.setChecked(False)
            self.centroid_raw_checkbox.blockSignals(False)
        self.centroid_view.position_filter.set_raw_difference_enabled(enabled)
        self._last_centroid_key = None
        self._refresh_centroid(self.data_buffer.latest_sensor_sample())
        self._schedule_save()

    def _apply_centroid_threshold(self):
        self.centroid_threshold_input.interpretText()
        self._centroid_signal_threshold = self.centroid_threshold_input.value()
        self.centroid_view.position_filter.set_signal_threshold(self._centroid_signal_threshold)
        self._last_centroid_key = None
        self._refresh_centroid(self.data_buffer.latest_sensor_sample())
        if self.save_display_config():
            self.status_label.setText(f'质心显示阈值已设置并保存：{self._centroid_signal_threshold}')

    def _centroid_scale_changed(self, value):
        self.centroid_view.set_scale_percent(value)
        self.centroid_scale_label.setText(f'{value}%')
        self._last_centroid_key = None
        self._schedule_save()

    def _refresh_centroid(self, sample):
        fresh = sample is not None and sample['age_seconds'] <= RX_FPS_STALE_SECONDS
        key = (sample['sequence'], fresh) if sample else None
        now = sample['sample_timestamp'] + sample['age_seconds'] if sample else None
        hold_due = now is not None and self.centroid_view.position_filter.hold_due(now)
        if key == self._last_centroid_key and not hold_due:
            return
        self._last_centroid_key = key
        signals = sample['signal'] if fresh else None
        states = sample['state'] if fresh and sample['has_raw'] else None
        raw_values = sample['raw'] if fresh and sample['has_raw'] else None
        result = self.centroid_view.set_signals(signals, sample['sample_timestamp'] if fresh else None,
                                               states, now, raw_values=raw_values)
        if self._connected:
            # 每个实际用于显示的新采样只记一次；生产者仅入队，后台统一批量落盘。
            self.rolling_log.submit('centroid', port=self.current_port, frame=dict(
                type='centroid', source_sample_sequence=sample['sequence'] if sample else None,
                sample_timestamp=sample['sample_timestamp'] if sample else None,
                source_type=('debug' if sample['has_raw'] else 'signal') if sample else None,
                signal=signals, raw=raw_values, state=states,
                sum_signal=sum(max(0, v) for v in signals) if signals else 0,
                fresh=fresh, update_reason='hold_timeout' if hold_due else 'sample_or_setting', **result))

    def _build_dfu(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)
        layout.addWidget(QtWidgets.QLabel('DFU · 沿用V21升级协议'))
        self.firmware_path_input = QtWidgets.QLineEdit()
        image = find_firmware_path()
        if image:
            self.firmware_path_input.setText(str(image))
        layout.addWidget(self.firmware_path_input)
        row = QtWidgets.QHBoxLayout()
        self.firmware_browse_button = self._button('选择 HEX / CYACD2',self.select_firmware,row)
        self.upgrade_button = self._button('Upgrade',self.start_dfu,row)
        layout.addLayout(row)
        self.dfu_progress = QtWidgets.QProgressBar()
        layout.addWidget(self.dfu_progress)
        self.dfu_log = QtWidgets.QPlainTextEdit()
        self.dfu_log.setReadOnly(True)
        layout.addWidget(self.dfu_log,1)
        self.control_tabs.addTab(tab,'固件升级')

    def refresh_ports(self):
        selected = self.port_combo.currentData() or STARTUP_PORT
        ports, _ = SerialPortScanner().list_ports()
        self.port_combo.clear()
        for name, description in ports:
            self.port_combo.addItem(description,name)
        index = self.port_combo.findData(selected)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)

    def _cancel_startup_connection(self):
        self._startup_connect_pending = False
        self._startup_connect_timer.stop()

    def _try_startup_connection(self):
        if not self._startup_connect_pending or self._closing or self.dfu_thread is not None:
            return
        if self.reader is None:
            self.refresh_ports()
            index = self.port_combo.findData(STARTUP_PORT)
            if index < 0:
                self.status_label.setText(f'等待 {STARTUP_PORT} 出现，将自动连接（每 2 秒重试）')
            else:
                self.port_combo.setCurrentIndex(index)
                self.status_label.setText(f'正在自动连接 {STARTUP_PORT}…')
                self.connect_serial(STARTUP_PORT)
        if self._startup_connect_pending:
            self._startup_connect_timer.start(2000)

    def toggle_connection(self):
        self._cancel_startup_connection()
        self.disconnect_serial() if self.reader is not None else self.connect_serial()

    def connect_serial(self, port=None):
        if self._closing or self.dfu_thread is not None or self.reader is not None:
            return
        port = port or self.port_combo.currentData()
        if not port:
            self.status_label.setText('请选择传感器串口')
            return
        self.current_port = port
        reader = SerialReaderThread(port,self.data_buffer,BAUD_RATE,self,self.data_recorder,
                                    rolling_log=self.rolling_log)
        self.reader = reader
        reader.state_changed.connect(lambda ok,message: self._serial_state(reader,ok,message))
        reader.device_info.connect(self.on_device_info)
        reader.matrix_config_status.connect(self.on_matrix_config_status)
        reader.capsense_channel_config.connect(self.on_capsense_channel_config)
        reader.live_noise.connect(self.on_live_noise)
        reader.command_ack.connect(lambda result: self.status_label.setText(f'命令被拒绝，结果码 {result}') if result else None)
        reader.finished.connect(lambda: self._reader_finished(reader))
        reader.start()

    def _serial_state(self, reader, connected, message):
        if reader is not self.reader:
            return
        self._set_connected(connected)
        if self._startup_connect_pending and not connected:
            # 线程退出会再发Disconnected，保留前一个打开失败原因。
            if message != 'Disconnected':
                self.status_label.setText(f'自动连接 {STARTUP_PORT} 失败：{message}；2 秒后重试')
        else:
            self.status_label.setText(message)
        if connected:
            self._cancel_startup_connection()
            QtCore.QTimer.singleShot(150,self.send_output_mode_command)
            QtCore.QTimer.singleShot(300,self.request_parameters)

    def _reader_finished(self, reader):
        if reader is self.reader:
            self.reader = None
            self._set_connected(False)
        reader.deleteLater()

    def disconnect_serial(self):
        self._cancel_startup_connection()
        self.param_timer.stop()
        if self.reader is not None:
            reader = self.reader
            reader.stop()
            if not reader.wait(2500):
                self.status_label.setText('串口线程尚未退出，暂不启动其他串口操作')
                return False
            self.reader = None
        self._set_connected(False)
        return True

    def _set_connected(self, connected):
        if not connected and hasattr(self, 'slide_test'):
            self.slide_test.stop('串口断开')
        if bool(connected) != self._connected:
            self.touchpad.invalidate('连接已改变，请松手重新采集触摸板参考')
            self._reset_host_state()
        self._connected = bool(connected)
        if connected:
            self._log_host_settings()
        self.connect_button.setText('Disconnect' if connected else 'Connect')
        for widget in (self.recal_button,self.mode_apply_button,self.params_apply_button,self.params_read_button):
            widget.setEnabled(connected and self.dfu_thread is None)
        self.port_combo.setEnabled(not connected and self.dfu_thread is None)

    def send_output_mode_command(self, *_):
        if self.reader is not None and self._connected:
            if not self.reader.send_output_mode(self.output_mode_combo.currentData()):
                self.status_label.setText('输出模式命令发送失败')

    def recalibrate(self):
        if self.reader is not None and self.reader.send_recalibration():
            self.touchpad.invalidate('固件基线重新校准，请松手重新采集触摸板参考')
            self._reset_host_state()
            self.status_label.setText('已请求全部通道重新校准基线，请保持释放')

    def on_device_info(self, frame):
        self.current_device_info = dict(frame)
        self.channel_count = min(CHANNEL_COUNT,int(frame.get('channel_count',CHANNEL_COUNT)))
        self._row_visibility_changed()
        self.status_label.setText(f"V{frame.get('protocol_version',21)} · {self.channel_count}通道 · {frame.get('device_name','')}")
        self.request_parameters()

    def _parameter_edited(self, *_):
        self._params_dirty = True

    def request_parameters(self):
        if self.reader is not None and self._connected:
            self.reader.get_matrix_config()
            self.reader.get_capsense_channel_config()

    def apply_parameters(self):
        if self.reader is None:
            return
        nsub,tx,ref = self.nsub_input.value(),self.tx_clk_div_input.value(),self.ref_cdac_input.value()
        if tx % 2:
            self.nsub_status_label.setText('TxDivider必须为偶数')
            return
        if self.reader.set_matrix_config(nsub,tx,ref):
            self.touchpad.invalidate('三参数已改变，请松手重新采集触摸板参考')
            self._reset_host_state('三参数改变；请保持释放，重新初始化')
            self._requested_params = (nsub,tx,ref)
            self._params_dirty = False
            self._params_pending = True
            self._params_poll_remaining = 40
            self.nsub_status_label.setText('等待扫描空闲点同步全部20通道并重新校准……')
            self.param_timer.start()

    def _poll_params(self):
        if not self._connected or self.reader is None:
            self.param_timer.stop()
            return
        if self._params_poll_remaining <= 0:
            self.param_timer.stop()
            self.nsub_status_label.setText('参数回读超时，请点击回读检查实际值')
            return
        self._params_poll_remaining -= 1
        self.reader.get_matrix_config()

    def on_matrix_config_status(self, frame):
        self._last_matrix_readback = frame
        pending, result = bool(frame['pending']),int(frame['result'])
        nsub,tx,ref = (int(frame[key]) for key in ('actual_nsub','actual_tx_clk_div','actual_ref_cdac'))
        if not self._params_dirty and not pending:
            for widget,value in ((self.nsub_input,nsub),(self.tx_clk_div_input,tx),(self.ref_cdac_input,ref)):
                widget.blockSignals(True)
                widget.setValue(value)
                widget.blockSignals(False)
        self.nsub_status_label.setText(f'实际 Nsub={nsub}，TxDivider={tx}，RefCDAC={ref}；' + ('应用中' if pending else '应用成功' if result == 0 else f'失败，结果码 {result}'))
        if self._params_pending and not pending and (result != 0 or (nsub,tx,ref) == self._requested_params):
            self._params_pending = False
            self.param_timer.stop()
            if result == 0:
                self._reset_host_state('三参数已同步；请保持释放，重新初始化')
            if self.reader is not None:
                self.reader.get_capsense_channel_config()

    def on_capsense_channel_config(self, frame):
        self._last_channel_readback = frame
        channels = frame.get('channels',())
        signature = tuple((c.get('channel'), c.get('hardware_nsub'),
                           c.get('hardware_tx_clk_div'), c.get('hardware_ref_cdac')) for c in channels)
        previous_signature = getattr(self, '_touchpad_config_signature', None)
        if previous_signature is not None and signature != previous_signature:
            self.touchpad.invalidate('硬件参数已改变，请松手重新采集触摸板参考')
            self._reset_host_state('硬件参数已改变；请保持释放，重新初始化')
        self._touchpad_config_signature = signature
        self.capsense_parameter_table.setRowCount(len(channels))
        mismatches = []
        for row,channel in enumerate(channels):
            values, bad = [f"CH{channel['channel']}"],False
            for key in ('nsub','tx_clk_div','ref_cdac'):
                configured,hardware = channel[f'configured_{key}'],channel[f'hardware_{key}']
                values.append(f'{configured} / {hardware}')
                bad |= configured != hardware
            values.append(str(channel['max_raw_count']))
            if bad:
                mismatches.append(values[0])
            for col,value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                if bad:
                    item.setForeground(QtGui.QColor('#ff6b6b'))
                self.capsense_parameter_table.setItem(row,col,item)
        self.capsense_parameter_status_label.setText('配置与硬件不一致：' + ', '.join(mismatches) if mismatches else f'{len(channels)}通道配置与硬件一致' if frame.get('read_ok') else '回读失败')

    def on_live_noise(self, frame):
        for label,value in zip(self.value_labels['live_hardware_noise'],frame['noise']):
            label.setText(f'{value:.2f}')

    def _row_visibility_changed(self, *_):
        if not hasattr(self,'table'):
            return
        for i in range(CHANNEL_COUNT):
            self.table.setRowHidden(i,i >= self.channel_count or not self.table_row_buttons[f'channel_{i}'].isChecked())
        height = sum(self.table.rowHeight(i) for i in range(CHANNEL_COUNT) if not self.table.isRowHidden(i))
        self.table.setFixedHeight(height+self.table.horizontalHeader().height()+26)
        self._schedule_save()

    def _column_visibility_changed(self, *_):
        if hasattr(self,'table'):
            for key,button in self.table_column_buttons.items():
                self.table.setColumnHidden(CHANNEL_TABLE_COLUMN_INDEX[key],not button.isChecked())
        self._schedule_save()

    def _reorder_column(self, source, target, after):
        if source != target:
            self.column_order.remove(source)
            self.column_order.insert(self.column_order.index(target)+int(after),source)
            self._apply_column_order()
            self._schedule_save()

    def _apply_column_order(self):
        header = self.table.horizontalHeader()
        for position,key in enumerate(['channel']+self.column_order):
            header.moveSection(header.visualIndex(CHANNEL_TABLE_COLUMN_INDEX[key]),position)
        self.column_flow.set_widgets([self.table_column_buttons[key] for key in self.column_order])

    def _curve_visibility_changed(self, *_):
        for plot in self.plots.values():
            plot.plotItem.legend.clear()
        for key,boxes in self.plot_checkboxes.items():
            for i,box in enumerate(boxes):
                visible = i < self.channel_count and box.isChecked()
                self.curves[key][i].setVisible(visible)
                if visible:
                    plot = self.raw_plot if key in ('raw','baseline') else self.signal_plot if key == 'signal' else self.k_plot
                    name = {'raw': 'input', 'fast': '压力速度'}.get(key, key) if self.waveform_source.currentData() == 'host' else key
                    plot.plotItem.legend.addItem(self.curves[key][i], f'CH{i} {name}')
        self._last_snapshot_version = -1
        self._schedule_save()

    def _background_changed(self, *_):
        channel = self.state_background_channel_combo.currentData()
        if channel in getattr(self,'state_background_buttons',{}):
            self.state_background_buttons[channel].setChecked(True)
        if self.waveform_source.currentData() == 'host' and hasattr(self, 'host_state_settings'):
            self._select_host_observation(channel)
        self._last_snapshot_version = -1
        self._schedule_save()

    def _history_changed(self, value):
        self.data_buffer.set_sample_count(value)
        self.host_state_machine.set_history_length(value)
        self._schedule_save()

    def auto_range(self):
        for plot in self.plots.values():
            plot.enableAutoRange()

    def _draw_state_background(self, history):
        runs = []
        for index,state in enumerate(history):
            if not runs or runs[-1][2] != state:
                runs.append([index,index+1,state])
            else:
                runs[-1][1] = index+1
        while len(self._state_regions) < len(runs):
            region = pg.LinearRegionItem(movable=False,pen=pg.mkPen(None))
            region.setZValue(-20)
            self.raw_plot.addItem(region,ignoreBounds=True)
            self._state_regions.append(region)
        for region,(start,stop,state) in zip(self._state_regions,runs):
            palette = HOST_STATE_COLORS if self.waveform_source.currentData() == 'host' else STATE_COLORS
            color = QtGui.QColor(palette.get(int(state),palette[-1]))
            color.setAlpha(90)
            region.setBrush(pg.mkBrush(color))
            region.setRegion((start-.5,stop-.5))
        while len(self._state_regions) > len(runs):
            self.raw_plot.removeItem(self._state_regions.pop())

    def refresh_plots(self):
        host_touchpad = self.touchpad.tracker.source in self.touchpad.tracker.HOST_SOURCES
        touchpad_samples = []
        sequence = min(self.host_state_machine.last_sequence, self.touchpad.last_sequence) if host_touchpad else self.host_state_machine.last_sequence
        for pending in self.data_buffer.pointer_samples_after(sequence):
            frame = self.host_state_machine.feed(pending)
            if frame is not None and self._connected:
                self.rolling_log.submit('host_state_machine', port=self.current_port, frame=frame)
            if host_touchpad and pending['sequence'] > self.touchpad.last_sequence:
                # 必须复制每一采样的派生 Signal，不能用最后一帧结果驱动整个 GUI 批次。
                touchpad_samples.append(dict(pending, host_signal=tuple(frame['signal']) if frame else None,
                    host_input_val=tuple(frame['input_val']) if frame else None,
                    host_baseline=tuple(frame['baseline']) if frame else None,
                    host_state=tuple(frame['state']) if frame else None,
                    host_generation=self.host_state_machine.generation,
                    host_ready=bool(frame and all(state != -1 for state in frame['state'])),
                    host_message=self.host_state_machine.reason))
        self.host_state_machine.expire(time.perf_counter())
        self.touchpad.process_samples(touchpad_samples if host_touchpad else
                                     self.data_buffer.pointer_samples_after(self.touchpad.last_sequence))
        self.host_state_settings.refresh()
        raw,base,sig,fast,slow,states,histories,fps,frames,count,channels,_,_,version = self.data_buffer.snapshot()
        sample = self.data_buffer.latest_sensor_sample()
        self._refresh_centroid(sample)
        self._refresh_array_bars(sample)
        temperature = '--'
        if sample and sample['die_temperature_celsius'] is not None and (sample['die_temperature_age_seconds'] or 0) < 5:
            temperature = f"{sample['die_temperature_celsius']:.1f} °C"
        mode = 'Debug' if sample and sample['has_raw'] else 'Run · 诊断未上传' if sample else '等待数据'
        self.live_status.setText(f'{self.current_port or "未连接"} · {channels}通道 · RX {fps:.1f} FPS · {frames}帧 · {mode} · 温度 {temperature}')
        host = self.waveform_source.currentData() == 'host'
        version = (version, self.host_state_machine.version, host)
        if version == self._last_snapshot_version:
            return
        self._last_snapshot_version = version
        if channels != self.channel_count:
            self.channel_count = min(channels,CHANNEL_COUNT)
            self._row_visibility_changed()
            self._curve_visibility_changed()
        values = {'raw':raw,'baseline':base,'signal':sig,'fast':fast,'slow':slow}
        names, palette = STATE_NAMES, STATE_COLORS
        if host:
            values = self.host_state_machine.snapshot()
            raw, base, sig, fast, slow = (values[k] for k in ('raw', 'baseline', 'signal', 'fast', 'slow'))
            histories = values['state']
            states = [c.state for c in self.host_state_machine.channels]
            count = len(raw[0])
            names, palette = HOST_STATE_NAMES, HOST_STATE_COLORS
        x = list(range(count))
        for key,curves in self.curves.items():
            for i in range(self.channel_count):
                if curves[i].isVisible():
                    curves[i].setData(x,values[key][i])
        for i in range(self.channel_count):
            state = int(states[i])
            label = self.value_labels['state'][i]
            label.setText(names.get(state,f'未知 {state}'))
            label.setStyleSheet(f'background:{palette.get(state,palette[-1])};border-radius:3px;padding:4px;')
            for key,data,decimals in (('hardware_raw_value',raw,0),('baseline_value',base,0),('signal_value',sig,0),('fast_k_value',fast,2),('slow_k_value',slow,2)):
                value = data[i][-1] if data[i] else math.nan
                text = f'{value:.{decimals}f}' if math.isfinite(value) else '--'
                if key == 'slow_k_value' and (host or state in IDLE_LIKE_STATES):
                    text = '-- 未计算'
                self.value_labels[key][i].setText(text)
            reason_label = self.value_labels['activation_reason'][i]
            if host:
                text, detail = format_activation_reason(self.host_state_machine.channels[i].last_activation)
            else:
                text, detail = '--', '固件协议未上报激活原因；切到上位机基线实验可查看实验模型的最近激活原因。'
            reason_label.setText(text)
            reason_label.setToolTip(detail)
        channel = self.state_background_channel_combo.currentData()
        if channel < len(histories):
            self._draw_state_background(histories[channel])
            if host:
                self.host_state_plot.setTitle(f'上位机实验状态 · CH{channel}')
                self.host_state_curve.setData(x, histories[channel])

    def export_recent(self, minutes):
        if self._recent_export_worker is not None:
            return
        try:
            snapshot = self.data_recorder.snapshot_recent(minutes)
        except Exception as exc:
            self.status_label.setText(str(exc))
            return
        worker = RecentDataExportThread(self.data_recorder,snapshot,self)
        self._recent_export_worker = worker
        for button in self.recent_export_buttons.values():
            button.setEnabled(False)
        worker.completed.connect(lambda result: self.status_label.setText(f"已转存：{result['output_path']}"))
        worker.failed.connect(self.status_label.setText)
        worker.finished.connect(self._export_finished)
        worker.start()

    def _export_finished(self):
        worker = self._recent_export_worker
        self._recent_export_worker = None
        if worker is not None:
            worker.deleteLater()
        for button in self.recent_export_buttons.values():
            button.setEnabled(True)

    def select_firmware(self):
        path,_ = QtWidgets.QFileDialog.getOpenFileName(self,'选择固件',self.firmware_path_input.text(),'Firmware (*.hex *.cyacd2)')
        if path:
            self.firmware_path_input.setText(path)

    def start_dfu(self):
        if self.dfu_thread is not None:
            return
        try:
            image = parse_cyacd2(resolve_dfu_image_path(self.firmware_path_input.text().strip()))
        except Exception as exc:
            self.status_label.setText(str(exc))
            return
        port = self.current_port if self._connected else self.port_combo.currentData()
        if not port:
            self.status_label.setText('请选择升级串口')
            return
        if not self.disconnect_serial():
            return
        self.current_port = port
        self.dfu_thread = DfuUpgradeThread(port,BAUD_RATE,image,self)
        self.dfu_thread.status_changed.connect(self.dfu_log.appendPlainText)
        self.dfu_thread.progress_changed.connect(self._dfu_progress_changed)
        self.dfu_thread.completed.connect(self._dfu_completed)
        self.dfu_thread.finished.connect(self._dfu_finished)
        for widget in (self.connect_button,self.upgrade_button,self.firmware_browse_button,self.port_combo,self.scan_button,self.firmware_path_input):
            widget.setEnabled(False)
        self.dfu_log.clear()
        self.dfu_progress.setValue(0)
        self._dfu_success = False
        self.dfu_thread.start()

    def _dfu_progress_changed(self, current, total):
        self.dfu_progress.setRange(0,total)
        self.dfu_progress.setValue(current)

    def _dfu_completed(self, success, message):
        self._dfu_success = success
        self.dfu_log.appendPlainText(message)
        self.status_label.setText(message)

    def _dfu_finished(self):
        worker = self.dfu_thread
        self.dfu_thread = None
        worker.deleteLater()
        for widget in (self.connect_button,self.upgrade_button,self.firmware_browse_button,self.port_combo,self.scan_button,self.firmware_path_input):
            widget.setEnabled(True)
        if self._dfu_success:
            QtCore.QTimer.singleShot(800,lambda: self.connect_serial(self.current_port))

    def _schedule_save(self, *_):
        if not self._loading:
            self._save_timer.start()

    def _save_host_state_parameters(self, parameters):
        # 先原子保存候选参数，成功后由实验页应用；写入失败时保留原运行参数。
        saved = self.save_display_config(host_parameters=parameters.settings())
        if saved:
            self.status_label.setText('实验参数已保存；请保持释放完成初始化。')
        return saved, str(self._config_path.resolve()) if saved else self.status_label.text()

    def save_display_config(self, *, host_parameters=None):
        self._save_timer.stop()
        config = {'history':self.history_input.value(),'background':self.state_background_channel_combo.currentData(),
                  'touchpad':self.touchpad.settings(),
                  'slide_test':self.slide_test.settings(),
                  'watch_face':self.watch_settings.settings(),
                  'array_bars':self.array_bar_settings.settings(),
                  'host_state_machine':self.host_state_settings.settings() if host_parameters is None else host_parameters,
                  'waveform_source':self.waveform_source.currentData(),
                  'host_observation_channel':self.host_observation_channel.currentData(),
                  'centroid_smoothing':self.centroid_smoothing_checkbox.isChecked(),
                  'centroid_local_2x2':self.centroid_local_2x2_checkbox.isChecked(),
                  'centroid_raw':self.centroid_raw_checkbox.isChecked(),
                  'centroid_raw_difference':self.centroid_raw_difference_checkbox.isChecked(),
                  'centroid_scale':self.centroid_scale_slider.value(),
                  'centroid_signal_threshold':self._centroid_signal_threshold,
                  'columns':{key:b.isChecked() for key,b in self.table_column_buttons.items()},
                  'rows':{key:b.isChecked() for key,b in self.table_row_buttons.items()},'order':self.column_order,
                  'curves':{key:[b.isChecked() for b in boxes] for key,boxes in self.plot_checkboxes.items()}}
        try:
            temporary = self._config_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
            temporary.replace(self._config_path)
            return True
        except OSError as exc:
            self.status_label.setText(f'显示配置保存失败：{exc}')
            return False

    def _load_display_config(self):
        try:
            config = json.loads(self._config_path.read_text(encoding='utf-8'))
            self.touchpad.load_settings(config.get('touchpad', {}))
            self.slide_test.load_settings(config.get('slide_test', {}))
            self.watch_settings.load_settings(config.get('watch_face', {}))
            self.array_bar_settings.load_settings(config.get('array_bars', {}))
            self._sync_touchpad_raw_gains()
            self.host_state_settings.load_settings(config.get('host_state_machine', {}))
            self.waveform_source.setCurrentIndex(1 if config.get('waveform_source') == 'host' else 0)
            self._centroid_signal_threshold = max(0,min(65535,int(config.get('centroid_signal_threshold',100))))
            self.centroid_threshold_input.setValue(self._centroid_signal_threshold)
            self.centroid_view.position_filter.set_signal_threshold(self._centroid_signal_threshold)
            self._last_centroid_key = None
            self.centroid_smoothing_checkbox.setChecked(config.get('centroid_smoothing',True) is not False)
            self.centroid_local_2x2_checkbox.setChecked(config.get('centroid_local_2x2',True) is not False)
            self.centroid_raw_checkbox.setChecked(config.get('centroid_raw',False) is True)
            self.centroid_raw_difference_checkbox.setChecked(config.get('centroid_raw_difference',False) is True)
            self.centroid_scale_slider.setValue(int(config.get('centroid_scale',70)))
            self.history_input.setValue(int(config.get('history',DEFAULT_HISTORY_LENGTH)))
            self.state_background_channel_combo.setCurrentIndex(max(0,min(19,int(config.get('background',0)))))
            for name,buttons in (('columns',self.table_column_buttons),('rows',self.table_row_buttons)):
                for key,checked in config.get(name,{}).items():
                    if key in buttons and isinstance(checked,bool):
                        buttons[key].setChecked(checked)
            order = config.get('order',[])
            if (isinstance(order, list) and len(order) == len(set(order))
                    and set(order) in (set(self.column_order), set(self.column_order) - {'activation_reason'})):
                # 兼容增加诊断列前的配置，保留用户已有列顺序，把新列追加到最右侧。
                self.column_order = order + [key for key in self.column_order if key not in order]
                self._apply_column_order()
            for key,values in config.get('curves',{}).items():
                if key in self.plot_checkboxes:
                    for box,checked in zip(self.plot_checkboxes[key],values):
                        if isinstance(checked,bool):
                            box.setChecked(checked)
            self._select_host_observation(max(0,min(19,int(config.get('host_observation_channel', config.get('background',0))))))
        except (OSError,ValueError,TypeError,AttributeError):
            pass

    def closeEvent(self, event):
        self.slide_test.stop('关闭窗口')
        self.touchpad.stop_mouse()
        self.watch_face.cancel_gesture()
        if self.dfu_thread is not None:
            self.status_label.setText('固件升级进行中，请等待完成后关闭')
            event.ignore()
            return
        if self._recent_export_worker is not None and not self._recent_export_worker.wait(2500):
            self.status_label.setText('正在写入历史快照，请等待完成后关闭')
            event.ignore()
            return
        if not self.disconnect_serial():
            event.ignore()
            return
        self._closing = True
        self.rolling_log.request_close()
        if self.rolling_log.is_alive():
            self.status_label.setText('正在保存滑动测试与串口日志，完成后自动关闭……')
            event.ignore()
            QtCore.QTimer.singleShot(100, self.close)
            return
        log_status = self.rolling_log.status()
        if log_status['error']:
            QtWidgets.QMessageBox.warning(self, '日志写入失败',
                f"{log_status['error']}\n仍有 {log_status['queued'] + log_status['pending']} 条日志未保存。")
        if self.slide_test.capture is not None:
            result = self.slide_test.capture.status()
            if result['error']:
                QtWidgets.QMessageBox.warning(self, '滑动测试保存失败',
                    f"{result['error']}\n已写入文件保留于：{result['directory']}")
        self.log_timer.stop()
        self.refresh_timer.stop()
        self._save_timer.stop()
        self.save_display_config()
        QtWidgets.QApplication.instance().removeEventFilter(self)
        event.accept()
