"""独立One Step学习页；复用唯一CAN工作线程，不接入旧学习表或传感器K表。"""
from dataclasses import asdict, replace
from copy import deepcopy
from datetime import datetime
import math
from pathlib import Path
import threading
import time

from qt_compat import QtCore, QtWidgets, Signal
from wrapped_label import WrappedLabel
from high_force_calibration import CombinedCancel, recover_release
from one_step_learning import (TARGETS_N, DOCUMENT_KIND, default_settings, validate_settings,
    run_learning, load_document, save_document, latest_path)
from pressure_feedback_control import FeedbackSettings
from one_step_working_release import working_position_matches
from one_step_repeatability import run_trial, save_trial, repeatability_summary, KIND as TRIAL_KIND
import one_step_relearning as cycles_learning


class OneStepLearningWidget(QtWidgets.QWidget):
    progress = Signal(object)
    point_ready = Signal(object)
    finished = Signal(object)
    status_message = Signal(str)
    arrival_ready = Signal(object)
    trial_plot = Signal(object)
    show_motor_plot = Signal()
    attempt_ready = Signal(object)

    def __init__(self, control, samples, parent=None, output_dir=None, busy_provider=lambda: False):
        super().__init__(parent)
        self.control, self.samples, self.busy_provider = control, samples, busy_provider
        self.output_dir = Path(output_dir) if output_dir else Path(__file__).with_name('one_step_learning')
        self._running, self._origin, self._document, self._archive = False, None, None, None
        self._working_release = None
        self._unsaved = self._zero_pending = False
        self._table_confirmed = self._coordinate_reset = False
        self._trial_running = False
        self._pending_target = None
        self._trial_results = []
        self._unsaved_trials = []
        self._seed_document = None
        self._cycle_running = False
        self._cycle_context = {}
        self._stop, self._emergency = threading.Event(), threading.Event()
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel('One Step · 整数压力位置学习')
        title.setStyleSheet('font-size:22px;font-weight:800;color:#66daff;')
        layout.addWidget(title)
        layout.addWidget(WrappedLabel('从接触附近的工作释放点学习1～30 N：先确认能稳定卸载，逐轮先调整压力位置，再优化速度，释放后休息5秒。'
            '使用压力计反馈，不依赖通道FLOATING；学习完成或普通停止后回起点。'
            '本页结果独立保存，不覆盖原压力学习表和传感器K表。'))
        self.live_label = WrappedLabel('压力计 — N · 电机位置 — step')
        self.live_label.setStyleSheet('font-size:20px;font-weight:700;color:#ffdf79;')
        layout.addWidget(self.live_label)
        self.origin_label = WrappedLabel('起始位置：未设置；先在无接触、零压力位置设定起点')
        layout.addWidget(self.origin_label)
        grid = QtWidgets.QGridLayout()
        defaults = asdict(default_settings())
        defaults['tolerance_n'] = .10
        self.inputs = {}
        fields = [
            ('max_step', '最大小步幅', 1, 16, 0, ' step'),
            ('speed', '学习速度', .1, 30, 1, ' RPM'),
            ('tolerance_n', '到位容差 ±', .10, .20, 2, ' N'),
            ('release_speed', '回起点速度', .1, 300, 1, ' RPM'),
            ('stable_s', '到位稳定确认', .5, 10, 1, ' 秒'),
            ('settle_s', '每步到位等待', .1, 5, 2, ' 秒'),
            ('force_limit_n', '保护压力', 31, 35, 1, ' N'),
            ('max_travel', '起点以下最大行程', 1, control.profile.steps_per_turn*12, 0, ' step'),
            ('max_current_ma', '电流上限', 1, 3000, 0, ' mA'),
            ('acceleration', '加速度', 1, control.profile.acceleration_max, 0, ' 档'),
            ('acquire_timeout_s', '每点寻压超时', 10, 600, 0, ' 秒'),
        ]
        for index, (key, label, low, high, decimals, suffix) in enumerate(fields):
            spin = QtWidgets.QDoubleSpinBox() if decimals else QtWidgets.QSpinBox()
            if decimals: spin.setDecimals(decimals)
            if key in ('speed', 'release_speed'): spin.setSingleStep(.1)
            spin.setRange(low, high)
            spin.setValue(defaults[key])
            self.inputs[key] = spin
            row, column = divmod(index, 2)
            grid.addWidget(QtWidgets.QLabel(f'{label}（{suffix.strip()}）'), row, column*2)
            grid.addWidget(spin, row, column*2+1)
        layout.addLayout(grid)
        cycle_group = QtWidgets.QGroupBox('轨迹参数学习 · 目标2秒内到位且压力不过冲')
        cycle_box = QtWidgets.QVBoxLayout(cycle_group)
        cycle_box.addWidget(WrappedLabel('① 接触位置可手动输入并勾选使用，跳过10次接触搜索；未勾选时自动搜索10次。'
            '② 首次以2 RPM接近接触位置，连续1秒确认±0.02 N卸载；未卸载向原起点方向退8 step再确认。'
            '③ 后续从已确认的工作释放点直接学习终点、受力速度及加减速，省去远处空行程。每点最多20轮；'
            '欠压与过冲时先固定运动参数、夹逼终点；位置区间搜尽后再保留前段速度、降低末段速度。'
            '达到压力要求且2秒内到位即记录该点，不强制重复同参数。每轮释放并休息5秒。'
            '到位压力按实测中位数判定，观察窗口范围仅作参考，不要求全部样本在容差内；峰值超限仍立即释放。'
            '过冲上限＝目标压力＋max(0.20 N，目标压力×1%)，独立于实测中位数容差。'
            '首次就位单独计时；2秒目标从工作释放点的运动命令计起。电机绝对坐标不归零，工作释放点为相对零点。'
            '每次回位都复核卸载；若残余压力超限则退回原起点并结束，保留最快压力合格方案。'))
        contact_row = QtWidgets.QHBoxLayout()
        contact_row.addWidget(QtWidgets.QLabel('接触位置（step）'))
        self.manual_contact_input = QtWidgets.QSpinBox()
        self.manual_contact_input.setRange(-2147483647, 2147483647)
        self.manual_contact_input.setValue(-717)
        self.manual_contact_input.setToolTip('当前电机坐标下的接触位置；须在释放点下方允许行程内。归零后须重新核对。')
        contact_row.addWidget(self.manual_contact_input)
        self.use_manual_contact = QtWidgets.QCheckBox('直接使用此接触位置，跳过10次搜索')
        self.use_manual_contact.setToolTip('仍保留空载确认和提前接触保护。重启或坐标归零后需重新勾选。')
        contact_row.addWidget(self.use_manual_contact)
        cycle_box.addLayout(contact_row)
        cycle_grid = QtWidgets.QGridLayout()
        self.cycle_inputs = {}
        for i, (key, label, low, high, decimals, value, suffix) in enumerate([
                ('contact_step', '接触搜索步幅', 1, 16, 0, 2, ' step'),
                ('contact_settle_s', '接触每步等待', .1, 2, 2, .1, ' 秒'),
                ('fast_speed', '运动速度上限', .1, min(300, control.profile.speed_max), 1, 30, ' RPM'),
                ('medium_speed', '初始受力速度（自动学习0.1～30）', .1, 30, 1, 5, ' RPM'),
                ('observation_s', '未微调观察时间', .5, 5, 1, 1, ' 秒'),
                ('relative_percent', '逐点相对容差 ±', 0, 2, 1, 1, ' %')]):
            spin = QtWidgets.QDoubleSpinBox() if decimals else QtWidgets.QSpinBox()
            if decimals: spin.setDecimals(decimals)
            spin.setRange(low, high)
            spin.setValue(value)
            self.cycle_inputs[key] = spin
            if key in ('fast_speed', 'medium_speed'):
                spin.setSingleStep(.1)
                spin.setToolTip('实际转速，精度0.1 RPM；运动命令由驱动自动换算，反馈保持厂家原始单位。')
            if key == 'contact_step':
                spin.setToolTip('默认2 step；需要精细接触位置时选1 step。增大步幅可能越过最初接触位置；压力突增会停止并释放。')
            elif key == 'contact_settle_s':
                spin.setSingleStep(.05)
                spin.setToolTip('仅用于接触搜索的到位沉降等待，独立于上方每步到位等待；之后仍须采集0.2秒新压力窗口。')
            cycle_grid.addWidget(QtWidgets.QLabel(f'{label}（{suffix.strip()}）'), i//2, (i%2)*2)
            cycle_grid.addWidget(spin, i//2, (i%2)*2+1)
        cycle_box.addLayout(cycle_grid)
        self.cycle_button = QtWidgets.QPushButton('开始轨迹学习（1～30 N）')
        self.cycle_button.setMinimumHeight(40)
        self.cycle_button.setStyleSheet('QPushButton {background:#6fd5a5;color:#102019;font-weight:800;}'
            'QPushButton:disabled {background:#59636d;color:#aab2ba;}')
        self.cycle_button.clicked.connect(self.start_cycles)
        cycle_actions = QtWidgets.QHBoxLayout()
        self.cycle_stop_button = QtWidgets.QPushButton('停止并回释放点')
        self.cycle_stop_button.clicked.connect(self.stop)
        self.return_zero_button = QtWidgets.QPushButton('回到0度位置')
        self.return_zero_button.setToolTip('向释放方向回到电机绝对0 step；保留坐标、工作释放点和学习结果。运动任务结束后可用。')
        self.return_zero_button.clicked.connect(lambda: self.start(return_zero=True))
        self.cycle_emergency_button = QtWidgets.QPushButton('紧急停止')
        self.cycle_emergency_button.clicked.connect(self.emergency_stop)
        self.cycle_emergency_button.setStyleSheet('QPushButton {background:#b94141;color:white;font-weight:800;}')
        for button in (self.cycle_button, self.cycle_stop_button, self.return_zero_button, self.cycle_emergency_button):
            button.setMinimumHeight(40)
            cycle_actions.addWidget(button)
        cycle_box.addLayout(cycle_actions)
        self.learning_notice = WrappedLabel('')
        self.learning_notice.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.learning_notice.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        self.learning_notice.setAccessibleName('学习停止原因和回位状态')
        self.learning_notice.hide()
        cycle_box.addWidget(self.learning_notice)
        self.return_zero_hint = WrappedLabel('')
        cycle_box.addWidget(self.return_zero_hint)
        self.contact_label = WrappedLabel('接触中位位置：— · 快速段终点：— · 实际空行程速度：—')
        self.contact_label.setStyleSheet('font-size:17px;font-weight:700;color:#ffdf79;')
        cycle_box.addWidget(self.contact_label)
        self.cycle_live_label = WrappedLabel('每轮显示实际压力、峰值、到位时间、所用轨迹和下一轮调整原因')
        self.cycle_live_label.setStyleSheet('font-size:18px;font-weight:700;color:#66daff;')
        cycle_box.addWidget(self.cycle_live_label)
        self.attempt_table = QtWidgets.QTableWidget(0, 17)
        self.attempt_table.setHorizontalHeaderLabels(['阶段/目标', '轮次', '终点 step', '实际 step', '实测 N', '峰值 N', '误差 N', '判定', '结果', '末段 RPM',
            '前段 RPM', '减速点 step', '加速档', '减速档', '到位 秒', '稳定确认 秒', '本轮结论 / 下一轮调整'])
        self.attempt_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.attempt_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.attempt_table.setMinimumHeight(210)
        cycle_box.addWidget(self.attempt_table)
        self.cycle_log = QtWidgets.QPlainTextEdit()
        self.cycle_log.setReadOnly(True)
        self.cycle_log.setMaximumBlockCount(3000)
        self.cycle_log.setMaximumHeight(130)
        cycle_box.addWidget(self.cycle_log)
        layout.addWidget(cycle_group)
        buttons = QtWidgets.QGridLayout()
        self.capture_button = QtWidgets.QPushButton('设定当前为起始位置')
        self.zero_button = QtWidgets.QPushButton('当前位置归零（电机指令）')
        self.return_button = QtWidgets.QPushButton('回到起始位置')
        self.learn_button = QtWidgets.QPushButton('旧版连续加载学习 0～30 N')
        self.stop_button = QtWidgets.QPushButton('停止并回起点')
        self.emergency_button = QtWidgets.QPushButton('紧急停止')
        self.zero_button.setToolTip('给电机发送当前位置清零指令，不是机械寻原点；所有页面的旧坐标参考会失效。完成后重新设定起点。')
        self.emergency_button.setToolTip('立即停止并取消自动回位；之后不会自行恢复运动。')
        self.learn_button.setStyleSheet('QPushButton {background:#6fd5a5;color:#102019;font-weight:800;}'
            'QPushButton:disabled {background:#59636d;color:#aab2ba;}')
        self.emergency_button.setStyleSheet('QPushButton {background:#b94141;color:white;font-weight:800;}'
            'QPushButton:disabled {background:#59636d;color:#aab2ba;}')
        for index, button in enumerate((self.capture_button, self.zero_button, self.return_button,
                                         self.learn_button, self.stop_button, self.emergency_button)):
            button.setMinimumHeight(36)
            buttons.addWidget(button, index//3, index%3)
        layout.addLayout(buttons)
        self.status_label = WrappedLabel('尚未学习；重启只显示历史表，起始位置须重新确认')
        layout.addWidget(self.status_label)
        self.table = QtWidgets.QTableWidget(31, 5)
        self.table.setHorizontalHeaderLabels(['目标压力 / N', '实测压力 / N', '绝对位置 / step', '相对释放点行程 / step', '误差 / N'])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setMinimumHeight(420)
        layout.addWidget(self.table)
        self.save_button = QtWidgets.QPushButton('保存 JSON / 重试保存')
        layout.addWidget(self.save_button)
        self.path_label = WrappedLabel('独立保存目录：'+str(self.output_dir))
        self.path_label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.path_label)
        self._build_trial_ui(layout)
        self._clear_table()
        self.capture_button.clicked.connect(self.capture_origin)
        self.zero_button.clicked.connect(self.zero_position)
        self.return_button.clicked.connect(lambda: self.start(release_only=True))
        self.learn_button.clicked.connect(self.start)
        self.stop_button.clicked.connect(self.stop)
        self.emergency_button.clicked.connect(self.emergency_stop)
        self.save_button.clicked.connect(self.save)
        self.progress.connect(self._progress)
        self.point_ready.connect(self._show_point)
        self.finished.connect(self._finished)
        self.arrival_ready.connect(self._show_arrival)
        self.attempt_ready.connect(self._show_attempt)
        self.control.worker_position_zeroed.connect(self._coordinates_invalidated)
        self.control.control_state_changed.connect(self._link_state)
        self.control.status_message.connect(self._command_status)
        try:
            self._document = load_document(self.output_dir, self.control.profile)
            self._seed_document = deepcopy(self._document)
            if self._document:
                for row in self._document['nodes']: self._show_point(row)
                for key, spin in self.inputs.items(): spin.setValue(self._document['settings'][key])
                self.status_label.setText('已读取上次完整学习表（历史记录）；起点未启用，请核对装夹与坐标后重新设定')
                self.path_label.setText('已读取：'+str(latest_path(self.output_dir, self.control.profile)))
            cycle_doc = cycles_learning.load_document(self.output_dir.parent/'one_step_cycles', self.control.profile)
            if cycle_doc:
                self._document = cycle_doc
                for row in cycle_doc['nodes']: self._show_point(row)
                for key, spin in self.inputs.items(): spin.setValue(cycle_doc['settings'][key])
                for key, spin in self.cycle_inputs.items():
                    spin.setValue(cycle_doc['cycle_settings']['relative_tolerance']*100 if key == 'relative_percent'
                                  else cycle_doc['cycle_settings'].get(key, getattr(cycles_learning.CycleSettings(), key)))
                self.control_mode.setCurrentIndex(self.control_mode.findData('cycles'))
                self._show_contact(cycle_doc)
                self.path_label.setText('已读取独立重压表：'+str(cycles_learning.latest_path(self.output_dir.parent/'one_step_cycles', self.control.profile)))
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            self._document = None
            self.status_label.setText(f'独立学习表读取失败：{exc}')
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(150)
        self._refresh()

    def is_running(self):
        return self._running or self._pending_target is not None

    def _tick(self):
        self._refresh()
        if (self._pending_target is not None and not self._running and not self.control.is_operation_busy()
                and not self.busy_provider() and self.pressure_buttons[self._pending_target].isEnabled()):
            target, self._pending_target = self._pending_target, None
            self.start_trial(target)

    def has_unsaved_result(self):
        return bool(self._unsaved_trials) or self._unsaved and self._document is not None

    def _build_trial_ui(self, layout):
        group = QtWidgets.QGroupBox('快速到压与位置重复性测试（1～30 N）')
        box = QtWidgets.QVBoxLayout(group)
        box.addWidget(WrappedLabel('默认压力优先：按学习表保守接近 → 提前制动、按压力寻压 → 小步保持 → 自动释放。'
            '完成新学习后自动改用“独立重压学习表”，沿用已验证轨迹，先记录未微调压力再保持。'
            '位置实验使用旧连续加载表。点击另一个压力先释放再执行，不强制休息。'))
        self.control_mode = QtWidgets.QComboBox()
        self.control_mode.addItem('实际压力优先（推荐）：提前减速，反馈到压后保持', 'pressure')
        self.control_mode.addItem('固定位置实验：直达学习位置，观察后再调压', 'position')
        self.control_mode.addItem('独立轨迹学习表：按记录的释放点与轨迹直达 → 保持', 'cycles')
        self.control_mode.currentIndexChanged.connect(lambda _: self._refresh())
        box.addWidget(self.control_mode)
        self.confirm_table_button = QtWidgets.QPushButton('确认使用历史学习表（装夹、材料与电机坐标未变）')
        self.confirm_table_button.clicked.connect(self.confirm_table)
        box.addWidget(self.confirm_table_button)
        self.table_use_label = WrappedLabel('直达未启用：先学习，或在原起点确认历史表')
        box.addWidget(self.table_use_label)
        grid = QtWidgets.QGridLayout()
        self.trial_inputs = {}
        fields = [('fast_speed', '快速接近速度', .1, min(300, self.control.profile.speed_max), 1, 30, ' RPM'),
                  ('hold_speed', '保持速度', .1, 30, 1, 5, ' RPM'),
                  ('hold_step', '保持最大小步幅', 1, 16, 0, 4, ' step'),
                  ('hold_s', '保持时间', 1, 3600, 1, 30, ' 秒'),
                  ('observation_s', '未微调观察窗', .2, 5, 1, .5, ' 秒'),
                  ('seek_max_step', '未到压时最大步幅', 1, 128, 0, 64, ' step')]
        for i, (key, title, low, high, decimals, value, suffix) in enumerate(fields):
            spin = QtWidgets.QDoubleSpinBox() if decimals else QtWidgets.QSpinBox()
            if decimals: spin.setDecimals(decimals)
            spin.setRange(low, high)
            spin.setValue(value)
            self.trial_inputs[key] = spin
            if key in ('fast_speed', 'hold_speed'): spin.setSingleStep(.1)
            row, col = divmod(i, 2)
            grid.addWidget(QtWidgets.QLabel(f'{title}（{suffix.strip()}）'), row, col*2)
            grid.addWidget(spin, row, col*2+1)
        box.addLayout(grid)
        self.mode_hint_label = WrappedLabel('压力优先快速段最高30 RPM，终点附近自动减速；观察窗只用于固定位置实验。')
        box.addWidget(self.mode_hint_label)
        box.addWidget(WrappedLabel('到位容差、保护压力、电流、行程、加速度和释放速度使用上方设置。'))
        buttons = QtWidgets.QGridLayout()
        self.pressure_buttons = {}
        for target in range(1, 31):
            button = QtWidgets.QPushButton(f'{target} N')
            button.setMinimumHeight(34)
            button.clicked.connect(lambda checked=False, n=target: self.start_trial(n))
            self.pressure_buttons[target] = button
            buttons.addWidget(button, (target-1)//5, (target-1)%5)
        box.addLayout(buttons)
        actions = QtWidgets.QHBoxLayout()
        self.trial_stop_button = QtWidgets.QPushButton('停止 / 取消下一点并释放')
        self.trial_stop_button.clicked.connect(self.stop)
        self.trial_emergency_button = QtWidgets.QPushButton('紧急停止')
        self.trial_emergency_button.setStyleSheet(self.emergency_button.styleSheet())
        self.trial_emergency_button.clicked.connect(self.emergency_stop)
        actions.addWidget(self.trial_stop_button)
        actions.addWidget(self.trial_emergency_button)
        box.addLayout(actions)
        self.arrival_label = WrappedLabel('尚未进行位置直达测试')
        self.arrival_label.setStyleSheet('font-size:19px;font-weight:800;color:#ffdf79;')
        box.addWidget(self.arrival_label)
        self.trial_status_label = WrappedLabel('保持与释放状态将在这里显示')
        self.trial_status_label.setStyleSheet('font-size:16px;color:#66daff;')
        box.addWidget(self.trial_status_label)
        self.repeat_summary_label = WrappedLabel('重复性统计：同表、同目标、同速度/加速度、同观察窗；至少2次才显示标准差。')
        box.addWidget(self.repeat_summary_label)
        self.trial_table = QtWidgets.QTableWidget(0, 8)
        self.trial_table.setHorizontalHeaderLabels(['目标 N', '学习 N', '到位实测 N', '偏差 N', '到位 step', '保持末值 N', '修正指令 step', '模式'])
        self.trial_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.trial_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.trial_table.setMinimumHeight(200)
        box.addWidget(self.trial_table)
        self.trial_save_button = QtWidgets.QPushButton('重试保存测试记录')
        self.trial_save_button.clicked.connect(self.save_trials)
        box.addWidget(self.trial_save_button)
        self.trial_path_label = WrappedLabel('测试结果独立保存至：'+str(self.output_dir.parent/'one_step_trials'))
        box.addWidget(self.trial_path_label)
        layout.addWidget(group)

    def confirm_table(self):
        if not self.confirm_table_button.isEnabled(): return
        try:
            position = self._unloaded_position()
            origin = self._document['origin_position_steps']
            working = self._document.get('working_release_position_steps')
            expected = working if working is not None else origin
            if (not working_position_matches(position, expected, self.control.profile) if working is not None
                    else abs(position-expected) > self.control.profile.tolerance(256)):
                raise ValueError(f'请先核对并回到学习释放点 {expected} step 附近，再确认使用；禁止平移旧表')
            self._origin, self._table_confirmed = origin, True
            self._working_release = working
            self._stop, self._emergency = threading.Event(), threading.Event()
            self.origin_label.setText(f'原起点：{origin} step · 工作释放点：{expected} step（相对工作零点0；绝对坐标未改变）')
            self.status_label.setText('历史学习表已确认，可使用下方1～30 N按钮进行直达测试')
        except Exception as exc:
            self.status_label.setText(str(exc))
        self._refresh()

    def start_trial(self, target):
        if not self.pressure_buttons[target].isEnabled(): return
        if self._trial_running:
            self._pending_target = target
            self._stop.set()
            self.status_label.setText(f'先停止并释放，确认卸载后立即按压 {target} N；可点击停止取消下一点')
            self.trial_status_label.setText(self.status_label.text())
            return
        try:
            settings = self.settings()
            for spin in self.trial_inputs.values(): spin.interpretText()
            values = {key: spin.value() for key, spin in self.trial_inputs.items()}
            mode = self.control_mode.currentData()
            if (mode == 'cycles') != (self._document.get('kind') == cycles_learning.KIND):
                raise ValueError('当前表与模式不匹配；独立重压表请选择“独立重压学习表”模式')
            settings = replace(settings, target_n=target, speed=values['hold_speed'],
                               max_step=values['hold_step'], hold_s=values['hold_s'])
            if mode == 'cycles':
                # 历史轨迹测试沿用保存的容差，图中的合格区间必须与实际控制一致。
                settings = replace(settings, tolerance_n=self._document['nodes'][target]['tolerance_n'])
            settings.validate(self.control.profile)
            document = deepcopy(self._document)
            if not self._table_confirmed or self._origin != document['origin_position_steps']:
                raise ValueError('学习坐标未确认')
        except Exception as exc:
            self.status_label.setText(str(exc))
            return
        origin = self._origin
        self._stop, self._emergency = threading.Event(), threading.Event()
        stop, urgent = self._stop, self._emergency
        self._running = self._trial_running = True
        def task(motor, worker_cancel):
            emergency = CombinedCancel(urgent, worker_cancel)
            try:
                result = run_trial(motor, self.control.profile, self.samples, stop, emergency,
                    settings, origin, document, target, mode=mode, fast_speed=values['fast_speed'],
                    observation_s=values['observation_s'], seek_max_step=values['seek_max_step'],
                    progress=self.progress.emit, on_arrival=self.arrival_ready.emit,
                    on_motor=self.control.publish_motor_feedback)
            except Exception as exc:
                released = recover_release(motor, self.control.profile, self.samples, settings, origin,
                    emergency, progress=self.progress.emit, on_motor=self.control.publish_motor_feedback)
                result = dict(kind=TRIAL_KIND, ok=False, target_n=target, control_mode=mode, arrival=None,
                    message=f'{exc}；{released["message"]}', emergency_stopped=emergency.is_set(), **{k:v for k,v in released.items() if k != 'message'})
            self.finished.emit(result)
        accepted, reason = self.control.start_external_operation(task, f'One Step直达 {target} N')
        if not accepted:
            self._running = self._trial_running = False
            self.status_label.setText(reason)
        else:
            node = document['nodes'][target]
            self.arrival_label.setText(f'目标 {target} N · 学习实测 {node["measured_force_n"]:.3f} N\n'+
                ('压力优先：学习位置只用于接近，等待真实压力到位' if mode == 'pressure' else
                 f'直达记录位置 {node["position_steps"]} step；等待本次未微调采样'))
            self.trial_plot.emit(dict(target_n=target, tolerance_n=settings.tolerance_n))
            self.trial_status_label.setText(f'准备测试 {target} N：确认起点与卸载后立即按压')
            self.show_motor_plot.emit()
        self._refresh()

    def _show_arrival(self, row):
        if row.get('measurement_kind') == 'pressure_arrival':
            self.arrival_label.setText(f'压力优先：目标 {row["target_n"]:g} N · 到压实测 {row["median_force_n"]:.3f} N\n'
                f'实际位置 {row["actual_position_steps"]:g} step · 稳定到压 {row["pressure_arrival_s"]:.2f} 秒\n'
                '此结果已执行压力调节，不作为固定位置未微调重复性结果')
            return
        self.arrival_label.setText(f'目标 {row["target_n"]:g} N · 学习实测 {row["learning_force_n"]:.3f} N\n'
            f'未微调压力 {row["median_force_n"]:.3f} N · 相比学习 {row["error_vs_learning_n"]:+.3f} N\n'
            f'实际位置 {row["actual_position_steps"]:g} step · 位置误差 {row["position_error_steps"]:+g} step'
            f'（允许±{row["position_tolerance_steps"]}）'
            f' · 直达到位 {row["position_arrival_s"]:.2f} 秒\n'
            f'观察窗范围 {row["min_force_n"]:.3f}～{row["max_force_n"]:.3f} N；此窗口未做压力微调')

    def _finish_trial(self, result):
        if result.get('working_release_invalidated'):
            self._table_confirmed = False
            self._working_release = None
        self._trial_results.append(result)
        self._unsaved_trials.append([result, None])
        arrival = result.get('arrival') or {}
        values = (result['target_n'], result.get('learning_node', {}).get('measured_force_n'),
            arrival.get('median_force_n'), arrival.get('error_vs_learning_n'), arrival.get('actual_position_steps'),
            result.get('final_force_n'), result.get('correction_command_steps'),
            '独立重压表' if result.get('control_mode') == 'cycles' else
            '压力优先' if result.get('control_mode') == 'pressure' else '位置实验')
        index = self.trial_table.rowCount()
        self.trial_table.insertRow(index)
        for col, value in enumerate(values):
            item = QtWidgets.QTableWidgetItem('—' if value is None else f'{value:.3f}' if col in (1, 2, 3, 5) else str(value))
            item.setToolTip(result['message'])
            self.trial_table.setItem(index, col, item)
        self.trial_table.scrollToBottom()
        summary = repeatability_summary(self._trial_results, result)
        if summary:
            sd = f'{summary["stdev_n"]:.3f} N' if summary['stdev_n'] is not None else '至少需要2次'
            label = '闭环到压' if result.get('control_mode') == 'pressure' else '固定位置未微调'
            self.repeat_summary_label.setText(f'本次会话同条件 {result["target_n"]} N：{summary["count"]} 次 · '
                f'{label}均值 {summary["mean_n"]:.3f} N · 范围 {summary["min_n"]:.3f}～{summary["max_n"]:.3f} N · 标准差 {sd}')
        if (not result.get('release_position_confirmed') or result.get('emergency_stopped')
                or not result.get('ok') and not result.get('cancelled')):
            self._pending_target = None
        peak, final = result.get('peak_n'), result.get('final_force_n')
        metrics = (f'\n采样峰值 {peak:.3f} N' if peak is not None else '')
        if final is not None: metrics += f' · 闭环末值 {final:.3f} N'
        metrics += f' · 累计调压指令 {result.get("correction_command_steps", 0)} step'
        self.trial_status_label.setText(result['message']+metrics)
        self.save_trials()

    def save_trials(self):
        if self._running: return
        for entry in list(self._unsaved_trials):
            try:
                entry[1] = save_trial(self.output_dir.parent/'one_step_trials', entry[0], entry[1])
                self._unsaved_trials.remove(entry)
                self.trial_path_label.setText('已保存：'+str(entry[1]))
            except Exception as exc:
                self.trial_path_label.setText(f'测试记录尚未保存，已保留内存并暂停下一点；点击重试：{exc}')
                break
        self._refresh()

    def settings(self):
        values = asdict(default_settings())
        for key, spin in self.inputs.items():
            spin.interpretText()
            values[key] = spin.value()
        settings = FeedbackSettings(**values)
        validate_settings(settings, self.control.profile)
        return settings

    def _unloaded_position(self):
        force, _, _ = self.samples.read(time.monotonic(), allow_negative=True)
        sample = self.control.latest_motor_feedback() or {}
        position, speed, current, stamp = (float(sample[k]) for k in
            ('position_counts', 'measured_speed_rpm', 'phase_current_ma', 'sampled_monotonic'))
        if (not all(math.isfinite(v) for v in (position, speed, current, stamp))
                or not 0 <= time.monotonic()-stamp <= getattr(self.control, 'feedback_max_age_s', .5) or abs(force) > .02
                or abs(speed) > 1 or not sample.get('in_position')
                or sample.get('running_state') not in (0, 1)
                or abs(current) > self.inputs['max_current_ma'].value()):
            raise ValueError('需要新鲜停止反馈，且压力在±0.02 N内')
        return int(position)

    def _return_zero_block_reason(self, idle, connected, sample):
        # 手动回绝对零点不依赖学习起点或压力采样，只核验当前电机坐标和运动互锁。
        if not idle or self._pending_target is not None:
            return '正在执行运动任务，请先停止并等待释放完成'
        if not connected:
            return '电机未连接或反馈已过期'
        if self._zero_pending or self._coordinate_reset and self._origin is None:
            return '坐标已失效，请核对坐标并重新设定起始位置'
        try:
            position, speed, current, stamp = (float(sample[k]) for k in
                ('position_counts', 'measured_speed_rpm', 'phase_current_ma', 'sampled_monotonic'))
            if (not all(math.isfinite(v) for v in (position, speed, current, stamp))
                    or not 0 <= time.monotonic()-stamp <= getattr(self.control, 'feedback_max_age_s', .5)):
                return '等待新鲜有效的电机位置反馈'
            if abs(current) > self.inputs['max_current_ma'].value() or sample.get('running_state') in (3, 4, 5):
                return '电机电流超限或处于故障状态'
            if abs(speed) > 1 or not sample.get('in_position') or sample.get('running_state') not in (0, 1):
                return '等待电机停止并确认位置'
            if position > 0:
                return '当前位置高于0度位置，返回需要向下运动，请先核对坐标'
            lower = (self._origin if self._origin is not None else 0)-self.inputs['max_travel'].value()
            if position < lower:
                return '当前位置超出允许回位范围，请核对坐标和行程设置'
        except (KeyError, TypeError, ValueError, OverflowError):
            return '等待有效的电机反馈'
        return ''

    def _refresh(self):
        idle = not self._running and not self.control.is_operation_busy() and not self.busy_provider()
        connected = self.control.is_can_connected()
        try:
            self._unloaded_position()
            unloaded = True
        except (ValueError, RuntimeError, KeyError, TypeError, OverflowError):
            unloaded = False
        try:
            force, _, _ = self.samples.read(time.monotonic(), allow_negative=True)
            force_text = f'{force:.3f}'
        except RuntimeError:
            force_text = '—'
        sample = self.control.latest_motor_feedback() or {}
        position = sample.get('position_counts')
        self.live_label.setText(f'压力计 {force_text} N · 电机位置 {position if position is not None else "—"} step')
        self.capture_button.setEnabled(idle and connected and unloaded)
        self.zero_button.setEnabled(idle and connected and unloaded)
        self.learn_button.setEnabled(idle and connected and unloaded and self._origin is not None and not self._unsaved)
        self.cycle_button.setEnabled(idle and connected and unloaded and self._origin is not None
            and self._seed_document is not None and not self.has_unsaved_result())
        self.cycle_stop_button.setEnabled(self._running)
        self.cycle_emergency_button.setEnabled(self._running or connected)
        for spin in self.cycle_inputs.values(): spin.setEnabled(not self._running)
        self.manual_contact_input.setEnabled(not self._running)
        self.use_manual_contact.setEnabled(not self._running)
        self.return_button.setEnabled(idle and connected and self._origin is not None)
        zero_reason = self._return_zero_block_reason(idle, connected, sample)
        self.return_zero_button.setEnabled(not zero_reason)
        self.return_zero_hint.setText('回0度：'+zero_reason if zero_reason else
            '回0度：返回电机绝对0 step，无需先设定学习起点')
        self.stop_button.setEnabled(self._running)
        self.emergency_button.setEnabled(self._running or connected)
        self.save_button.setEnabled(not self._running and self._document is not None)
        for spin in self.inputs.values(): spin.setEnabled(not self._running)
        valid = bool(self._document and self._document.get('complete'))
        usable = valid and self._table_confirmed and self._origin == self._document['origin_position_steps']
        self.confirm_table_button.setEnabled(idle and connected and unloaded and valid and not self._coordinate_reset)
        self.table_use_label.setText('学习坐标已确认，可以测试；每次先回起点并确认卸载' if usable else
            '测试未启用：坐标归零后须重新学习' if self._coordinate_reset else
            '测试未启用：先完成学习，或在原起点核对装夹与坐标后确认历史表')
        for target, button in self.pressure_buttons.items():
            button.setEnabled(connected and usable and not self._emergency.is_set()
                and not self.has_unsaved_result() and (idle or self._trial_running))
            if valid:
                node = self._document['nodes'][target]
                button.setToolTip(f'学习实测 {node["measured_force_n"]:.3f} N → 绝对位置 {node["position_steps"]} step')
        for spin in self.trial_inputs.values(): spin.setEnabled(not self._running)
        self.control_mode.setEnabled(not self._running)
        self.trial_inputs['observation_s'].setEnabled(not self._running and self.control_mode.currentData() == 'position')
        if self.control_mode.currentData() == 'cycles':
            for key in ('fast_speed', 'observation_s', 'seek_max_step'):
                self.trial_inputs[key].setEnabled(False)
            self.mode_hint_label.setText('沿用学习表的释放点、轨迹及观察时间；新版从接触附近工作释放点直达，到位采样后进入保持。')
        else:
            self.mode_hint_label.setText('压力优先快速段最高30 RPM，终点附近自动减速；观察窗只用于固定位置实验。')
        self.trial_stop_button.setEnabled(self._running or self._pending_target is not None)
        self.trial_emergency_button.setEnabled(self._running or connected)
        self.trial_save_button.setEnabled(not self._running and bool(self._unsaved_trials))

    def capture_origin(self):
        self._refresh()
        if not self.capture_button.isEnabled(): return
        try:
            self._origin = self._unloaded_position()
            self._working_release = None
            self._table_confirmed = False
            self.origin_label.setText(f'本页起始位置：{self._origin} step（回位目标；不改写旧页面释放点）')
            self.status_label.setText('起点已设置，可以开始独立学习；旧表仍作为历史记录显示')
        except Exception as exc:
            self.status_label.setText(str(exc))
        self._refresh()

    def zero_position(self):
        self._refresh()
        if not self.zero_button.isEnabled(): return
        self._origin = None
        self._working_release = None
        self._table_confirmed = False
        self._coordinate_reset = True
        self._pending_target = None
        self._zero_pending = True
        self.origin_label.setText('坐标正在归零，原起点已撤销；完成后重新设定起点')
        self.status_label.setText('已提交电机坐标清零指令；等待电机反馈确认')
        self.control.zero_motor_position()
        self._refresh()

    def _command_status(self, message):
        if self._zero_pending:
            self._zero_pending = False
            self.status_label.setText(message)

    def _coordinates_invalidated(self):
        self.use_manual_contact.setChecked(False)
        self._origin = None
        self._working_release = None
        self._table_confirmed = False
        self._coordinate_reset = True
        self._pending_target = None
        self.origin_label.setText('电机坐标已失效，请重新设定起点；表格仅保留历史数据')
        if self._running: self.emergency_stop()
        self._refresh()

    def _link_state(self, connected, _busy):
        if not connected and not self._running:
            self.use_manual_contact.setChecked(False)
            self._origin = None
            self._working_release = None
            self._table_confirmed = False
            self._pending_target = None
            self.origin_label.setText('连接已变化，起点待重新确认；表格仅保留历史数据')
        self._refresh()

    def start(self, _checked=False, *, release_only=False, return_zero=False):
        release_only = release_only or return_zero
        self._refresh()
        button = self.return_zero_button if return_zero else self.return_button if release_only else self.learn_button
        if not button.isEnabled():
            if return_zero: self.status_label.setText(self.return_zero_hint.text())
            return
        try:
            settings = self.settings()
            if not release_only and abs(self._unloaded_position()-self._origin) > 1:
                raise ValueError('当前位置与起点不一致，请先回起点或重新设定')
        except Exception as exc:
            self.status_label.setText(str(exc))
            return
        origin = self._working_release if release_only and self._working_release is not None else self._origin
        if return_zero:
            # 回绝对零点只改变本次回位目标，机械下压边界仍由已确认的原起点限定。
            origin = 0
            settings = replace(settings, max_travel=settings.max_travel-(self._origin if self._origin is not None else 0))
        if release_only:
            feedback = self.control.latest_motor_feedback() or {}
            return_margin = (self.control.profile.tolerance(256) if self._working_release is not None else 1)
            if feedback.get('position_counts', origin) > origin+(0 if return_zero else return_margin):
                self.status_label.setText('当前位置高于0度位置，返回需要向下运动，请先核对坐标' if return_zero else
                    '当前位置高于工作释放点；释放不会向下运动，请重新学习确认工作点')
                return
            if not return_zero and self._working_release is not None:
                from one_step_working_release import working_settings
                settings = working_settings(settings, self._origin, self._working_release)
        self._stop, self._emergency = threading.Event(), threading.Event()
        stop, urgent = self._stop, self._emergency
        self._running = True
        def task(motor, worker_cancel):
            emergency = CombinedCancel(urgent, worker_cancel)
            nodes = []
            def point(row):
                nodes.append(dict(row))
                self.point_ready.emit(row)
            try:
                if release_only:
                    result = recover_release(motor, self.control.profile, self.samples, settings, origin,
                        emergency, progress=self.progress.emit, on_motor=self.control.publish_motor_feedback,
                        return_even_within_tolerance=return_zero)
                    result['return_only'] = True
                    if return_zero:
                        result['message'] = ('已回到0度位置并确认静止' if result['release_position_confirmed'] else
                                             '回到0度位置已中止；尚未确认到位')
                else:
                    result = run_learning(motor, self.control.profile, self.samples, stop, emergency,
                        settings, origin, progress=self.progress.emit, on_point=point,
                        on_motor=self.control.publish_motor_feedback)
            except Exception as exc:
                released = recover_release(motor, self.control.profile, self.samples, settings, origin,
                    emergency, progress=self.progress.emit, on_motor=self.control.publish_motor_feedback)
                result = dict(kind=DOCUMENT_KIND, format_version=1, created_at=datetime.now().astimezone().isoformat(),
                    profile=self.control.profile.metadata(), namespace=self.control.profile.namespace,
                    origin_position_steps=origin, settings=asdict(settings), complete=False, nodes=nodes,
                    message=f'{exc}；{released["message"]}', release_position_confirmed=released['release_position_confirmed'],
                    pressure_unloaded=released['pressure_unloaded'], return_only=release_only)
            self.finished.emit(result)
        accepted, reason = self.control.start_external_operation(task,
            'One Step回到0度位置' if return_zero else 'One Step回起点' if release_only else 'One Step整数压力学习')
        if not accepted:
            self._running = False
            self.status_label.setText(reason)
        else:
            if not release_only:
                self.learning_notice.hide()
                self._cycle_context = {}
                self._clear_table()
                self._document, self._archive = None, None
                self._table_confirmed = False
            self.status_label.setText('正在回到0度位置' if return_zero else '正在回起点' if release_only else
                                     '正在核验零压力，从0到30 N逐整数学习')
        self._refresh()

    def start_cycles(self):
        self._refresh()
        if not self.cycle_button.isEnabled(): return
        try:
            settings = self.settings()
            values = {}
            for key, spin in self.cycle_inputs.items():
                spin.interpretText()
                values[key] = spin.value()
            values['relative_tolerance'] = values.pop('relative_percent')/100
            self.manual_contact_input.interpretText()
            values['trajectory_learning'] = True
            values['require_full_observation_band'] = False
            values['overshoot_absolute_n'] = .20
            values['overshoot_relative_tolerance'] = .01
            values['arrival_target_s'] = cycles_learning.NEW_ARRIVAL_TARGET_S
            values['speed_step_rpm'] = .1
            values['near_contact_release'] = True
            values['manual_contact_position'] = self.manual_contact_input.value() if self.use_manual_contact.isChecked() else None
            cycles = cycles_learning.CycleSettings(**values)
            cycles.validate(self.control.profile)
            seed = deepcopy(self._seed_document)
            origin = self._origin
            if self._coordinate_reset or origin != seed['origin_position_steps']:
                raise ValueError('旧表起点与当前坐标不一致；请先按当前坐标完成旧版连续学习，禁止平移旧表')
            expected = self._working_release if self._working_release is not None else origin
            position = self._unloaded_position()
            at_working_release = (self._working_release is not None
                                  and working_position_matches(position, expected, self.control.profile))
            if not at_working_release and abs(position-origin) > self.control.profile.tolerance(256):
                raise ValueError('请先回到已确认的释放起点')
            if cycles.manual_contact_position is not None and not origin-settings.max_travel < cycles.manual_contact_position < origin:
                raise ValueError('手动接触位置必须在释放点下方的允许行程内')
        except Exception as exc:
            self.status_label.setText(str(exc))
            return
        self._stop, self._emergency = threading.Event(), threading.Event()
        stop, urgent = self._stop, self._emergency
        self._running = self._cycle_running = True
        archive = self.output_dir.parent/'one_step_cycles'/('one_step_cycles_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
        def task(motor, worker_cancel):
            emergency = CombinedCancel(urgent, worker_cancel)
            try:
                result = cycles_learning.run_relearning(motor, self.control.profile, self.samples,
                    stop, emergency, settings, origin, seed, cycles=cycles, progress=self.progress.emit,
                    on_attempt=self.attempt_ready.emit, on_point=self.point_ready.emit,
                    on_motor=self.control.publish_motor_feedback,
                    checkpoint=lambda doc: cycles_learning.save_document(archive.parent, doc, self.control.profile, archive))
            except Exception as exc:
                released = recover_release(motor, self.control.profile, self.samples, settings, origin, emergency,
                    progress=self.progress.emit, on_motor=self.control.publish_motor_feedback)
                result = dict(released, return_only=True, complete=False,
                              cancelled=stop.is_set(), emergency_stopped=emergency.is_set(),
                              last_release_position_steps=origin, message=f'{exc}；{released["message"]}')
            self.finished.emit(result)
        accepted, reason = self.control.start_external_operation(task, 'One Step连续轨迹参数学习')
        if not accepted:
            self._running = self._cycle_running = False
            self.status_label.setText(reason)
        else:
            self.learning_notice.hide()
            self._cycle_context = {}
            self._archive = archive
            self._table_confirmed = False
            self._clear_table()
            self.attempt_table.setRowCount(0)
            self.cycle_log.clear()
            manual = cycles.manual_contact_position
            self.contact_label.setText(f'使用手动接触位置 {manual} step，跳过10次搜索' if manual is not None else
                                       '正在学习接触点：完成10次后开始轨迹学习')
            self.cycle_live_label.setText('先确认接触附近的工作释放点；首次就位单列，后续目标2秒内到位且压力合格')
            self.status_label.setText(self.contact_label.text())
            self.show_motor_plot.emit()
        self._refresh()

    def _show_contact(self, document):
        contact = document.get('contact')
        if contact:
            if document.get('position_learning_version', 0) >= 3 or contact.get('source') == 'manual' or self._cycle_running:
                self.contact_label.setText(f"{'手动' if contact.get('source') == 'manual' else '实测'}接触位置 "
                    f"{contact['median_position_steps']:g} step · 接触前边界 {contact['last_zero_boundary_steps']} step")
                if document.get('working_release_position_steps') is not None:
                    self.contact_label.setText(self.contact_label.text()+f" · 工作释放点 {document['working_release_position_steps']} step · 首次就位 {document['initial_positioning_s']:.2f} 秒")
                if not self._running:
                    self.manual_contact_input.setValue(round(contact['median_position_steps']))
                return
            self.contact_label.setText(f"接触中位位置 {contact['median_position_steps']:g} step · "
                f"快速段终点 {contact['fast_position_steps']} step · 实际空行程速度 {contact['effective_fast_speed_rpm']} RPM")

    def _show_attempt(self, row):
        self._cycle_context.update({k: row[k] for k in ('target_n', 'attempt', 'stage') if k in row})
        contact = row['stage'] == 'contact'
        values = ('接触' if contact else f"{row['target_n']} N", row['attempt'],
            row.get('candidate_position_steps'), row.get('position_steps') if contact else row.get('actual_position_steps'),
            row.get('measured_force_n'), row.get('peak_n'), row.get('error_n'),
            '—' if contact else f"{min(3, row.get('streak', 0))}/3",
            '合格' if row['ok'] else '等待重试' if row.get('retrying') else '未通过', row.get('medium_speed_rpm'))
        if row.get('trajectory'):
            plan = row['trajectory']
            arrival_target = row.get('arrival_target_s', 1.0)
            values = values[:7]+(f'压力+{arrival_target:g}秒',)+values[8:]+(plan['free_speed_rpm'], plan['decel_position_steps'],
                plan['acceleration'], plan['deceleration'], row.get('position_arrival_s'), row.get('pressure_stable_s'), row['adjustment_reason'])
            if row.get('require_full_observation_band') is False:
                values = values[:7]+(f'中位数+峰值+{arrival_target:g}秒',)+values[8:]
            if row.get('record_kind') == 'communication_interruption':
                values = values[:7]+('通信中断', '等待重试' if row.get('retry_scheduled') else '已结束')+values[9:]
        else:
            values += (None,)*7
        index = self.attempt_table.rowCount()
        self.attempt_table.insertRow(index)
        for column, value in enumerate(values):
            text = '—' if value is None else f'{value:.3f}' if column in (4, 5, 6, 14, 15) else str(value)
            item = QtWidgets.QTableWidgetItem(text)
            item.setToolTip(row['message'])
            self.attempt_table.setItem(index, column, item)
        self.attempt_table.scrollToBottom()
        if contact:
            position_text = f"位置{values[3]} step" if values[3] is not None else '未取得接触位置'
            self.cycle_live_label.setText(f"接触第{row['attempt']}/10次 · 总尝试{row.get('contact_attempt', row['attempt'])}次 · "
                f"{position_text} · 实测{self.attempt_table.item(index, 4).text()} N · {values[8]}\n{row['message']}")
        else:
            self.cycle_live_label.setText(f"目标{row['target_n']} N · 第{row['attempt']}/20次 · "
                f"实测{self.attempt_table.item(index, 4).text()} N · 峰值{self.attempt_table.item(index, 5).text()} N\n"
                f"候选{values[2]} step · 实际{values[3] if values[3] is not None else '—'} step · "
                f"误差{self.attempt_table.item(index, 6).text()} N · 连续合格{values[7]} · {values[8]}；正在释放")
            if not row['ok']: self.cycle_live_label.setText(self.cycle_live_label.text()+'\n'+row['message'])
            if row.get('trajectory'):
                arrival_text = self.attempt_table.item(index, 14).text()
                self.cycle_live_label.setText(f"目标{row['target_n']} N · 第{row['attempt']}/20轮 · 实测{self.attempt_table.item(index, 4).text()} N · "
                    f"峰值{self.attempt_table.item(index, 5).text()} N · 到位{arrival_text}秒（目标≤{arrival_target:g}秒）\n"
                    f"终点{values[2]} step · 前段/末段{values[10]}/{values[9]} RPM · 减速点{values[11]} step · "
                    f"加速/减速{values[12]}/{values[13]}档 · {'已释放' if row.get('release_position_confirmed') else '回位未确认'}\n{row['adjustment_reason']}")

    def stop(self):
        self._pending_target = None
        if self._running:
            self._stop.set()
            self.status_label.setText('停止后续任务，正在停止并回本次起点；等待确认回位')
            if self._cycle_running:
                self._set_learning_notice('正在停止学习\n原因：已点击“停止并回释放点”\n回位状态：正在释放，尚未确认回位。', 'warning')
            if self._trial_running: self.trial_status_label.setText(self.status_label.text())

    def emergency_stop(self):
        self._pending_target = None
        self._stop.set()
        self._emergency.set()
        self.control.request_external_stop()
        self.status_label.setText('紧急停止：已取消全部运动和自动回位，不会自行恢复')
        if self._cycle_running:
            self._set_learning_notice('学习紧急停止\n原因：已触发紧急停止\n回位状态：自动回位已取消，不会自行恢复运动。', 'error')
        if self._trial_running: self.trial_status_label.setText(self.status_label.text())

    def _clear_table(self):
        for target in TARGETS_N:
            for column, value in enumerate((str(target), '—', '—', '—', '—')):
                self.table.setItem(target, column, QtWidgets.QTableWidgetItem(value))

    def _show_point(self, row):
        target = row['target_force_n']
        for column, value in enumerate((str(target), f"{row['measured_force_n']:.3f}", str(row['position_steps']),
                                         str(row['travel_steps']), f"{row['error_n']:+.3f}")):
            self.table.setItem(target, column, QtWidgets.QTableWidgetItem(value))
        self.table.scrollToItem(self.table.item(target, 0))

    def _progress(self, row):
        if self._cycle_running:
            self._cycle_context.update({k: row[k] for k in ('target_n', 'attempt', 'stage') if k in row})
        if row.get('phase') == 'working_release_ready':
            self._working_release = row['working_release_position_steps']
            self.origin_label.setText(f'原起点：{self._origin} step · 工作释放点：{self._working_release} step（相对工作零点0；绝对坐标未改变）')
        if self._emergency.is_set(): return
        if self._stop.is_set() and row.get('phase') not in ('recovery', 'recovery_release', 'return'): return
        if row.get('contact'): self._show_contact(row)
        if row.get('message'):
            self.status_label.setText(row['message'])
            if self._cycle_running:
                self.cycle_log.appendPlainText(row['message'])
                if (row.get('phase') in ('contact_baseline_wait', 'contact_retry', 'speed_change', 'trajectory_learning')
                        or row.get('stage') == 'contact' and row.get('phase') not in
                        ('rest', 'return', 'recovery', 'recovery_release')):
                    self.cycle_live_label.setText(row['message'])
        elif 'target_n' in row:
            self.status_label.setText(f"{row['phase']} · 目标 {row['target_n']:g} N · 实测 {row['force_n']:.3f} N"
                f" · 位置 {row['position_steps']:g} step")
        if self._trial_running:
            text = self.status_label.text()
            if row.get('hold_remaining_s') is not None:
                text += f' · 保持剩余 {row["hold_remaining_s"]:.1f} 秒'
            self.trial_status_label.setText(text)

    def _set_learning_notice(self, text, tone='error'):
        background, border, foreground = {
            'error': ('#4b171b', '#ff6b75', '#fff0f0'),
            'warning': ('#483510', '#ffc857', '#fff3cc'),
            'success': ('#153b2b', '#6fd5a5', '#e5fff0'),
        }[tone]
        self.learning_notice.setStyleSheet(f'font-size:20px;font-weight:700;color:{foreground};'
            f'background:{background};border:2px solid {border};border-radius:6px;padding:12px;')
        self.learning_notice.setText(text)
        self.learning_notice.show()

    def _show_learning_outcome(self, result):
        context = dict(self._cycle_context)
        points = result.get('points') or []
        if points:
            context['target_n'] = points[-1]['target_n']
            attempts = points[-1].get('attempts') or []
            if attempts:
                context['attempt'] = attempts[-1]['attempt']
        urgent = result.get('emergency_stopped') or self._emergency.is_set()
        cancelled = result.get('cancelled') or self._stop.is_set()
        complete = result.get('complete')
        title = '学习紧急停止' if urgent else '学习已完成' if complete else '学习已手动停止' if cancelled else '学习已停止'
        where = []
        if context.get('target_n') is not None:
            where.append(f'目标 {context["target_n"]:g} N' if context['target_n'] else '接触点学习')
        if context.get('attempt') is not None:
            where.append(f'第 {context["attempt"]} 轮')
        if where: title += ' · '+' · '.join(where)
        reasons = []
        if urgent: reasons.append('触发紧急停止，自动回位已取消，不会自行恢复运动')
        elif cancelled: reasons.append('收到停止指令，结束后续学习')
        for key in ('working_release_check_error', 'working_release_failure', 'message'):
            value = result.get(key)
            if value and value not in '\n'.join(reasons): reasons.append(value)
        reason = '\n'.join(reasons) or '未提供具体原因，请查看学习记录'
        position = result.get('actual_release_position_steps', result.get('last_release_position_steps'))
        released = '已确认回位' if result.get('release_position_confirmed') else '尚未确认回位'
        if position is not None: released += f'（{position:g} step）'
        unloaded = '压力卸载已确认' if result.get('pressure_unloaded') else '压力卸载未确认'
        self._set_learning_notice(f'{title}\n原因：{reason}\n回位状态：{released}；{unloaded}',
                                  'error' if urgent else 'success' if complete else 'warning' if cancelled else 'error')

    def _finished(self, result):
        learning_finished = self._cycle_running or (result.get('kind') in (cycles_learning.KIND, DOCUMENT_KIND)
                                                    and not result.get('return_only'))
        self._running = False
        self._trial_running = False
        self._cycle_running = False
        self.status_label.setText(result['message'])
        if result.get('kind') == TRIAL_KIND:
            self.trial_plot.emit(None)
            self._finish_trial(result)
        elif not result.get('return_only'):
            self._document = result
            if result.get('kind') == cycles_learning.KIND:
                if result.get('cycle_settings', {}).get('near_contact_release'):
                    self._working_release = result.get('working_release_position_steps') if result.get('working_release_valid') else None
                    self.origin_label.setText(f'原起点：{self._origin} step · 工作释放点：{self._working_release if self._working_release is not None else "未确认或已失效"} step（绝对坐标未改变）')
                self._show_contact(result)
                if result.get('release_position_confirmed'):
                    self.cycle_live_label.setText(self.cycle_live_label.text().replace('正在释放', '已释放'))
                if result.get('points'):
                    point = result['points'][-1]
                    best = point.get('best_pressure_candidate')
                    if best:
                        arrival_target = result['cycle_settings'].get('arrival_target_s', 1.0)
                        self.cycle_live_label.setText(self.cycle_live_label.text()+
                            f"\n本点{point['target_n']} N最快压力合格方案：第{best['attempt']}轮，"
                            f"{best['position_arrival_s']:.3f}秒，终点{best['trajectory']['position_steps']} step；"
                            + (f'{arrival_target:g}秒目标已达成' if best['time_ok'] else f'{arrival_target:g}秒目标未达成，方案已保留'))
                self.control_mode.setCurrentIndex(self.control_mode.findData('cycles'))
            elif result.get('complete'):
                self._seed_document = deepcopy(result)
                if self.control_mode.currentData() == 'cycles': self.control_mode.setCurrentIndex(0)
            self._table_confirmed = bool(result.get('complete'))
            if self._table_confirmed: self._coordinate_reset = False
            self._unsaved = True
            self.save()
        if learning_finished:
            self._show_learning_outcome(result)
        self.status_message.emit(self.status_label.text())
        self._refresh()

    def save(self):
        if self._unsaved_trials: self.save_trials()
        if self._running or self._document is None: return
        try:
            cycle = self._document.get('kind') == cycles_learning.KIND
            directory = self.output_dir.parent/'one_step_cycles' if cycle else self.output_dir
            saver = cycles_learning.save_document if cycle else save_document
            latest = cycles_learning.latest_path if cycle else latest_path
            self._archive = saver(directory, self._document, self.control.profile, self._archive)
            self._unsaved = False
            suffix = '\n有效表：'+str(latest(directory, self.control.profile)) if self._document['complete'] else '\n未完成记录，未更新有效表'
            self.path_label.setText('独立JSON：'+str(self._archive)+suffix)
        except Exception as exc:
            self._unsaved = True
            self.path_label.setText(f'JSON尚未保存成功，结果保留在本次内存中，请点击重试保存：{exc}')
        self._refresh()

    def shutdown(self):
        self.timer.stop()
        self._pending_target = None
        if self._running: self.emergency_stop()
