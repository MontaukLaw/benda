"""按提示计时标记横向、纵向滑动；不以算法接触状态推断人工动作次数。"""
import json
import math
from pathlib import Path
import time
from qt_compat import QtCore, QtGui, QtWidgets


class SlideTestPanel(QtWidgets.QWidget):
    def __init__(self, log, context, prepare, cleanup, directory=None):
        super().__init__()
        self.log, self.context, self.prepare, self.cleanup = log, context, prepare, cleanup
        self.directory = Path(directory) if directory else Path(__file__).with_name('recordings') / 'slide_tests'
        self.running = False
        self.capture = None
        self.plan = []
        self.index = 0
        self._signature = None
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel('横向 / 纵向滑动一致性测试')
        title.setStyleSheet('font-size:24px;font-weight:bold')
        layout.addWidget(title)
        instructions = QtWidgets.QLabel('选定固定起终点，每次滑动使用相同距离、速度和力度。横向从左到右，纵向从上到下。\n'
            '每次只滑一次，结束后抬起手指；不要贴着表面返回起点。按倒计时提示操作，不按接触状态自动计次。\n'
            '可以分别以轻压、正常、重压重复整套测试。已知实际距离时填写；未知填0。测试时停止控制系统鼠标。')
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        self.settings_group = QtWidgets.QGroupBox('测试设置')
        form = QtWidgets.QFormLayout(self.settings_group)
        self.horizontal_count = self._spin(1, 30, 5)
        self.vertical_count = self._spin(1, 30, 5)
        self.slide_seconds = self._double(.5, 10, 2)
        self.prepare_seconds = self._double(1, 15, 3)
        self.release_seconds = self._double(1, 30, 3)
        self.horizontal_distance = self._double(0, 1000, 0)
        self.vertical_distance = self._double(0, 1000, 0)
        self.pressure = QtWidgets.QComboBox()
        self.pressure.addItems(['正常', '轻压', '重压'])
        self.notes = QtWidgets.QLineEdit()
        self.notes.setPlaceholderText('起终点位置、手指、力度或异常情况')
        for label, widget in (('横向次数', self.horizontal_count), ('纵向次数', self.vertical_count),
            ('每次滑动窗口（秒）', self.slide_seconds), ('每次准备时间（秒）', self.prepare_seconds),
            ('每次松手观察时间（秒）', self.release_seconds), ('横向实际距离（毫米；0表示未知）', self.horizontal_distance),
            ('纵向实际距离（毫米；0表示未知）', self.vertical_distance), ('力度', self.pressure), ('备注', self.notes)):
            form.addRow(label, widget)
        layout.addWidget(self.settings_group)
        row = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton('开始测试')
        self.stop_button = QtWidgets.QPushButton('停止并保存已采集数据')
        self.stop_button.setEnabled(False)
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        layout.addLayout(row)
        self.prompt = QtWidgets.QLabel('点击开始后，先保持松手3秒，再按提示滑动。')
        self.prompt.setWordWrap(True)
        self.prompt.setMinimumHeight(110)
        self.prompt.setStyleSheet('font-size:28px;font-weight:bold;color:#63d8ef')
        layout.addWidget(self.prompt)
        self.countdown = QtWidgets.QLabel()
        self.countdown.setStyleSheet('font-size:22px')
        layout.addWidget(self.countdown)
        self.progress = QtWidgets.QProgressBar()
        layout.addWidget(self.progress)
        self.status = QtWidgets.QLabel('测试结束后自动保存 ZIP，包含阶段标记、20通道数据、参数、质心和位移。')
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.status)
        self.open_button = QtWidgets.QPushButton('打开数据目录')
        self.open_button.clicked.connect(self.open_directory)
        layout.addWidget(self.open_button)
        layout.addStretch()
        self.start_button.clicked.connect(self.start)
        self.stop_button.clicked.connect(lambda: self.stop('用户停止'))
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    @staticmethod
    def _spin(low, high, value):
        widget = QtWidgets.QSpinBox()
        widget.setRange(low, high)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    @staticmethod
    def _double(low, high, value):
        widget = QtWidgets.QDoubleSpinBox()
        widget.setRange(low, high)
        widget.setDecimals(1)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    def settings(self):
        return dict(horizontal_count=self.horizontal_count.value(), vertical_count=self.vertical_count.value(),
                    slide_seconds=self.slide_seconds.value(), prepare_seconds=self.prepare_seconds.value(),
                    release_seconds=self.release_seconds.value(), horizontal_distance_mm=self.horizontal_distance.value(),
                    vertical_distance_mm=self.vertical_distance.value(), pressure=self.pressure.currentText(), notes=self.notes.text())

    def load_settings(self, settings):
        for key in ('horizontal_count', 'vertical_count', 'slide_seconds', 'prepare_seconds', 'release_seconds'):
            if isinstance(settings.get(key), (int, float)) and math.isfinite(settings[key]):
                value = int(settings[key]) if key.endswith('_count') else settings[key]
                getattr(self, key).setValue(value)
        for key in ('horizontal', 'vertical'):
            value = settings.get(key + '_distance_mm')
            if isinstance(value, (int, float)) and math.isfinite(value):
                getattr(self, key + '_distance').setValue(value)
        if settings.get('pressure') in ('正常', '轻压', '重压'):
            self.pressure.setCurrentText(settings['pressure'])
        if isinstance(settings.get('notes'), str):
            self.notes.setText(settings['notes'])

    @staticmethod
    def signature(context):
        return json.dumps({k: context.get(k) for k in ('port', 'host_parameters', 'host_generation', 'raw_gains',
            'touchpad', 'hardware_signature', 'tracking_enabled')}, sort_keys=True)

    def start(self):
        if self.running or self.capture and not self.capture.status()['done']:
            return
        context = self.context()
        sample = context.get('sample') or {}
        if not context.get('connected') or not sample.get('has_raw') or sample.get('channel_count') != 20:
            self.status.setText('请连接20通道传感器，并使用含Raw的Debug输出。')
            return
        if time.perf_counter() - sample.get('sample_timestamp', 0) > .5 or not context.get('host_ready'):
            self.status.setText('请等待有效采样及状态机初始化完成，再开始。')
            return
        if context['touchpad']['source'] not in ('host_signal','host_continuous'):
            self.status.setText('请先在触摸板设置中选择连续位置权重或上位机 Signal，再开始测试。')
            return
        try:
            self.prepare()
            context = self.context()
            context['protocol'] = self.settings()
            context['rolling_session'] = self.log.session
            self.capture = self.log.start_capture(self.directory, context)
        except (OSError, ValueError) as exc:
            self.cleanup()
            self.status.setText(f'无法开始测试：{exc}')
            return
        self._signature = self.signature(context)
        p = context['protocol']
        self.plan = [dict(phase='baseline', axis='none', trial=0, seconds=3.)]
        for axis in ('horizontal', 'vertical'):
            for trial in range(1, p[axis + '_count'] + 1):
                for phase in ('prepare', 'slide', 'release'):
                    self.plan.append(dict(phase=phase, axis=axis, trial=trial, seconds=p[phase + '_seconds']))
        self.index = 0
        self.running = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.settings_group.setEnabled(False)
        self.progress.setRange(0, len(self.plan))
        self.progress.setValue(0)
        self.status.setText('采集进行中；请保持参数不变。按提示完成后自动保存。')
        self.begin_phase(time.perf_counter())

    def mark(self, event, now):
        self.log.submit('slide_test', frame=dict(self.plan[self.index], event=event,
            sample_timestamp=now, phase_index=self.index,
            direction={'horizontal': 'left_to_right', 'vertical': 'top_to_bottom', 'none': 'released'}[self.plan[self.index]['axis']]), _control=True)

    def begin_phase(self, now):
        self.phase_started_at = now
        self.deadline = now + self.plan[self.index]['seconds']
        self.mark('phase_begin', now)
        self.update_prompt(now)

    def update_prompt(self, now):
        phase = self.plan[self.index]
        axis = '横向' if phase['axis'] == 'horizontal' else '纵向'
        suffix = f"{axis} 第 {phase['trial']}/{self.settings()[phase['axis'] + '_count']} 次" if phase['axis'] != 'none' else ''
        text = {'baseline': '完全松手，记录释放状态', 'prepare': '保持抬指，准备放到固定起点',
                'slide': '现在滑动：左 → 右' if phase['axis'] == 'horizontal' else '现在滑动：上 ↓ 下',
                'release': '立即抬指，保持松手；不要贴着返回'}[phase['phase']]
        self.prompt.setText(suffix + '\n' + text)
        self.countdown.setText(f"剩余 {max(0, self.deadline - now):.1f} 秒")

    def tick(self):
        now = time.perf_counter()
        if self.running:
            context = self.context()
            sample = context.get('sample') or {}
            error = self.capture.status()['error']
            if error:
                self.stop('保存异常：' + error)
            elif not context.get('connected') or now - sample.get('sample_timestamp', 0) > .5:
                self.stop('串口断开或采样中断')
            elif self.signature(context) != self._signature or not sample.get('has_raw') or sample.get('channel_count') != 20:
                self.stop('测试期间参数、输入模式或状态机发生变化')
            elif now - self.deadline > 1:
                self.stop('提示计时中断；当前测试不完整')
            elif now >= self.deadline:
                # 连续模式首段必须等自动参考就绪，避免按固定倒计时漏记第一趟滑动。
                if (self.plan[self.index]['phase'] == 'baseline'
                        and context['touchpad']['source'] == 'host_continuous'
                        and not context.get('position_reference_ready')):
                    if now-self.phase_started_at >= 30:
                        self.stop('30秒内未建立稳定松手参考；请检查非idle通道及释放波形')
                    else:
                        self.deadline = now + .2
                        self.prompt.setText('继续完全松手：等待所有通道idle及位置参考稳定')
                        self.countdown.setText(f'参考等待 {now-self.phase_started_at:.1f} 秒 / 30 秒')
                    return
                self.mark('phase_end', now)
                self.index += 1
                self.progress.setValue(self.index)
                if self.index == len(self.plan):
                    self.finish('completed', '全部提示完成')
                else:
                    self.begin_phase(now)
            else:
                self.update_prompt(now)
        elif self.capture and not self.start_button.isEnabled():
            result = self.capture.status()
            if result['done']:
                self.start_button.setEnabled(True)
                self.settings_group.setEnabled(True)
                if result['error']:
                    self.status.setText(f"保存失败：{result['error']}\n已写入的文件保留于：{result['directory']}")
                else:
                    outcome = {'completed': '测试完成', 'completed_with_gaps': '测试结束，但存在缺失数据', 'aborted': '测试中止，已保存部分数据'}[result['outcome']]
                    self.status.setText(f"{outcome}。请将这个 ZIP 提供给我分析：\n{result['archive']}")

    def stop(self, reason='用户停止'):
        if self.running:
            self.mark('phase_aborted', time.perf_counter())
            self.finish('aborted', reason)

    def finish(self, outcome, reason):
        self.running = False
        self.log.finish_capture(outcome, reason)
        self.cleanup()
        self.stop_button.setEnabled(False)
        self.prompt.setText('测试结束，正在自动保存数据' if outcome == 'completed' else '测试已停止，正在保存已有数据')
        self.countdown.setText(reason)
        self.status.setText('后台生成 ZIP；请等待保存结果。')

    def open_directory(self):
        directory = self.capture.directory.parent if self.capture else self.directory
        directory.mkdir(parents=True, exist_ok=True)
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(directory.resolve())))
