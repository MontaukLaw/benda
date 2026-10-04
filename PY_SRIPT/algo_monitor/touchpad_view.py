"""相对触摸板的试验画布和控制面板；系统鼠标仅在用户勾选后移动。"""
import time
from collections import deque

from qt_compat import QtCore, QtGui, QtWidgets, Signal
from touchpad import TouchpadTracker, FractionalMotion


class PointerCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 300)
        self.cursor = [960.0, 540.0]
        self.trail = deque(maxlen=150)
        self.message = '在右侧“触摸板”中启用并采集松手参考'

    def center(self):
        self.cursor = [960.0, 540.0]
        self.trail.clear()
        self.update()

    def advance(self, dx, dy):
        self.cursor = [max(0.0, min(limit, value + delta))
                       for value, delta, limit in zip(self.cursor, (dx, dy), (1920, 1080))]
        if dx or dy:
            self.trail.append(tuple(self.cursor))

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#080d13'))
        p.setPen(QtGui.QColor('#d9e7f0'))
        p.drawText(QtCore.QRectF(16, 12, self.width()-32, 30),
                   QtCore.Qt.AlignmentFlag.AlignCenter, '相对触摸板 · 1920 × 1080 试验画布')
        scale = min((self.width()-48)/1920, (self.height()-130)/1080)
        left, top = (self.width()-1920*scale)/2, 52+(self.height()-130-1080*scale)/2
        area = QtCore.QRectF(left, top, 1920*scale, 1080*scale)
        p.fillRect(area, QtGui.QColor('#101d29'))
        p.setPen(QtGui.QPen(QtGui.QColor('#496476'), 1))
        p.drawRect(area)
        for i in range(1, 8):
            x, y = left + i*240*scale, top + i*135*scale
            p.drawLine(QtCore.QPointF(x, top), QtCore.QPointF(x, area.bottom()))
            p.drawLine(QtCore.QPointF(left, y), QtCore.QPointF(area.right(), y))
        p.setPen(QtGui.QPen(QtGui.QColor('#438bb0'), 2))
        points = [QtCore.QPointF(left+x*scale, top+y*scale) for x,y in self.trail]
        for a,b in zip(points, points[1:]):
            p.drawLine(a,b)
        point = QtCore.QPointF(left+self.cursor[0]*scale, top+self.cursor[1]*scale)
        p.setPen(QtGui.QPen(QtGui.QColor('#def8ff'), 2))
        p.setBrush(QtGui.QColor('#38c8ed'))
        p.drawEllipse(point, 7, 7)
        p.drawText(QtCore.QRectF(12, self.height()-65, self.width()-24, 55),
                   QtCore.Qt.AlignmentFlag.AlignCenter | QtCore.Qt.TextFlag.TextWordWrap,
                   f'{self.message}\nX {self.cursor[0]:.1f}  Y {self.cursor[1]:.1f}')


class TouchpadPanel(QtWidgets.QWidget):
    motion_sample = Signal(object)

    def __init__(self, log_callback=None, parent=None):
        super().__init__(parent)
        self.tracker = TouchpadTracker()
        self.motion = FractionalMotion()
        self.canvas = PointerCanvas()
        self.last_sequence = 0
        self.not_before = 0.0
        self._reference_last_received = 0.0
        self.log_callback = log_callback
        layout = QtWidgets.QVBoxLayout(self)
        self.enabled_box = QtWidgets.QCheckBox('启用相对触摸板试验')
        layout.addWidget(self.enabled_box)
        note = QtWidgets.QLabel('数据源：每路Raw减去该通道的松手参考。需要20路Debug数据。\n'
                               '先完全松手，点击采集参考；稳定1秒后开始滑动。抬起后光标保持原位，再落指可继续划动。')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.reference_button = QtWidgets.QPushButton('采集松手参考')
        layout.addWidget(self.reference_button)
        self.reference_button.setEnabled(False)
        self.reference_group = QtWidgets.QGroupBox('松手参考采集')
        reference_layout = QtWidgets.QVBoxLayout(self.reference_group)
        self.reference_status_label = QtWidgets.QLabel('尚未采集')
        self.reference_status_label.setWordWrap(True)
        reference_layout.addWidget(self.reference_status_label)
        self.reference_progress = QtWidgets.QProgressBar()
        self.reference_progress.setRange(0, 100)
        self.reference_progress.setValue(0)
        self.reference_progress.setMinimumHeight(24)
        reference_layout.addWidget(self.reference_progress)
        self.reference_detail = QtWidgets.QLabel('启用后，请完全松手，再点击采集参考')
        self.reference_detail.setWordWrap(True)
        self.reference_detail.setMinimumHeight(44)
        reference_layout.addWidget(self.reference_detail)
        self.reference_cancel_button = QtWidgets.QPushButton('取消采集')
        self.reference_cancel_button.setEnabled(False)
        self.reference_cancel_button.clicked.connect(self.cancel_reference)
        reference_layout.addWidget(self.reference_cancel_button)
        layout.addWidget(self.reference_group)
        self.sensitivity = self._spin(10, 2000, 300)
        self.threshold = self._spin(5, 5000, 100)
        self.cutoff = QtWidgets.QDoubleSpinBox()
        self.cutoff.setRange(.5, 15)
        self.cutoff.setDecimals(1)
        self.cutoff.setSingleStep(.5)
        self.cutoff.setValue(2.5)
        self.cutoff.setKeyboardTracking(False)
        form = QtWidgets.QFormLayout()
        form.addRow('灵敏度（像素/格）', self.sensitivity)
        form.addRow('接触阈值（count）', self.threshold)
        form.addRow('静止滤波截止频率（Hz）', self.cutoff)
        layout.addLayout(form)
        help_text = QtWidgets.QLabel('截止频率越低越平稳，也会增加延迟；快速运动时自动减弱滤波。\n'
                                    '第一版使用固定灵敏度、全部20路质心；尚未加入各通道增益标定。')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        self.system_mouse = QtWidgets.QCheckBox('控制系统鼠标（窗口激活时；Esc停止）')
        layout.addWidget(self.system_mouse)
        self.center_button = QtWidgets.QPushButton('试验光标回到中心')
        layout.addWidget(self.center_button)
        self.status = QtWidgets.QLabel(self.tracker.message)
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch(1)
        self.enabled_box.toggled.connect(self._enabled_changed)
        self.reference_button.clicked.connect(self.start_reference)
        self.center_button.clicked.connect(self.center)
        self.system_mouse.toggled.connect(self._mouse_changed)
        for widget in (self.sensitivity, self.threshold, self.cutoff):
            widget.valueChanged.connect(self._configure)

    @staticmethod
    def _spin(low, high, value):
        widget = QtWidgets.QSpinBox()
        widget.setRange(low, high)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    def settings(self):
        return dict(sensitivity=self.sensitivity.value(), threshold=self.threshold.value(),
                    min_cutoff=self.cutoff.value())

    def load_settings(self, settings):
        if not isinstance(settings, dict):
            return
        for key, widget in (('sensitivity',self.sensitivity), ('threshold',self.threshold),
                            ('min_cutoff',self.cutoff)):
            if key in settings:
                try:
                    value = float(settings[key]) if key == 'min_cutoff' else int(settings[key])
                    widget.setValue(value)
                except (ValueError, TypeError, OverflowError):
                    pass

    def _configure(self, *_):
        self.cancel_motion()
        self.tracker.configure(**self._tracker_settings())
        self.motion.reset()

    def _tracker_settings(self):
        settings = self.settings()
        return dict(sensitivity=settings['sensitivity'], threshold=settings['threshold'],
                    min_cutoff=settings['min_cutoff'])

    def _enabled_changed(self, enabled):
        self.cancel_motion()
        self.system_mouse.setChecked(False)
        self.tracker.set_enabled(enabled)
        self.reference_button.setEnabled(enabled)
        self.not_before = time.perf_counter()
        self._display()

    def _mouse_changed(self, *_):
        self.tracker.end_contact()
        self.motion.reset()
        if not self.tracker.enabled and self.system_mouse.isChecked():
            self.system_mouse.setChecked(False)

    def center(self):
        self.canvas.center()
        self.tracker.end_contact()
        self.motion.reset()

    def start_reference(self):
        self.cancel_motion()
        self.system_mouse.setChecked(False)
        self.motion.reset()
        self.not_before = time.perf_counter()
        self._reference_last_received = self.not_before
        self.tracker.start_reference()
        self._display()

    def cancel_reference(self):
        self.tracker.cancel_reference()
        self.motion.reset()
        self._display()

    def invalidate(self, message):
        self.cancel_motion()
        self.system_mouse.setChecked(False)
        self.motion.reset()
        self.not_before = time.perf_counter()
        self.tracker.invalidate(message)
        self._display()

    def stop_mouse(self):
        self.system_mouse.setChecked(False)

    def cancel_motion(self):
        self.motion_sample.emit(dict(timestamp=time.perf_counter(), contact=False,
                                     dx=0.0, dy=0.0, cancel=True))

    def process_samples(self, samples, now=None):
        now = time.perf_counter() if now is None else now
        for sample in samples:
            self.last_sequence = sample['sequence']
            if sample['sample_timestamp'] <= self.not_before or now-sample['sample_timestamp'] > self.tracker.MAX_GAP:
                self.tracker.end_contact()
                self.cancel_motion()
                continue
            was_ready = self.tracker.reference is not None
            if self.tracker.calibrating:
                self._reference_last_received = sample['sample_timestamp']
            previous_time,previous_sequence = self.tracker.timestamp,self.tracker.sequence
            dx,dy = self.tracker.feed(sample)
            interrupted = (not self.tracker.enabled or self.tracker.reference is None or self.tracker.calibrating
                or not sample.get('has_raw') or sample.get('channel_count') != 20
                or (previous_time is not None and sample['sample_timestamp']-previous_time>self.tracker.MAX_GAP)
                or (previous_sequence is not None and sample['sequence']!=previous_sequence+1)
                or self.tracker.motion_block_reason in ('position_jump', 'invalid_sample'))
            self.motion_sample.emit(dict(timestamp=sample['sample_timestamp'], contact=self.tracker.contact,
                                         dx=dx/self.tracker.sensitivity,dy=dy/self.tracker.sensitivity,cancel=interrupted))
            self.canvas.advance(dx,dy)
            if not self.tracker.contact:
                self.motion.reset()
            if self.system_mouse.isChecked() and self.window().isActiveWindow():
                ix,iy = self.motion.consume(dx,dy)
                if ix or iy:
                    position = QtGui.QCursor.pos()
                    QtGui.QCursor.setPos(position.x()+ix, position.y()+iy)
            if self.log_callback is not None and self.tracker.enabled:
                self.log_callback(dict(type='touchpad', source_sample_sequence=sample['sequence'],
                    sample_timestamp=sample['sample_timestamp'], dx=dx, dy=dy,
                    raw_xy=self.tracker.raw_position, filtered_xy=self.tracker.position,
                    contact=self.tracker.contact, cursor=tuple(self.canvas.cursor),
                    source='raw_per_channel_reference', settings=self.settings(),
                    reference=self.tracker.reference if not was_ready else None,
                    noise=self.tracker.noise if not was_ready else None,
                    reference_status=dict(self.tracker.reference_status),
                    active_weight_share=self.tracker.active_weight_share,
                    motion_block_reason=self.tracker.motion_block_reason,
                    message=self.tracker.message))
        was_contact = self.tracker.contact
        self.tracker.expire(now)
        if was_contact and not self.tracker.contact:
            self.cancel_motion()
        if self.tracker.calibrating and now - self._reference_last_received > 1:
            self.tracker.wait_for_reference(
                '未收到新的采样数据，请检查串口连接并选择20路Debug输出；恢复后自动继续')
        self._display()

    def _display(self):
        capture = self.tracker.reference_status
        stage = capture['stage']
        titles = dict(idle='尚未就绪', waiting='采集已开始 · 等待条件满足',
                      collecting='正在采集 · 请保持松手', unstable='等待Raw稳定 · 采集尚未完成',
                      complete='采集完成', cancelled='采集已取消或中止')
        color = '#70e3a0' if stage == 'complete' else '#ffd479' if self.tracker.calibrating else '#9bb7ca'
        self.reference_status_label.setText(titles[stage])
        self.reference_status_label.setStyleSheet(f'color:{color};font-weight:bold;font-size:15px;')
        self.reference_progress.setValue(capture['progress'])
        self.reference_progress.setFormat('采集完成 100%' if stage == 'complete' else '%p%')
        detail = capture['detail']
        if stage in ('collecting', 'unstable', 'complete'):
            detail += f"\n有效采样 {capture['elapsed']:.2f} / 1.00 秒 · {capture['frames']} 帧（至少32帧）"
        self.reference_detail.setText(detail)
        self.reference_button.setEnabled(self.tracker.enabled and not self.tracker.calibrating)
        self.reference_button.setText('正在采集…' if self.tracker.calibrating else
                                      '重新采集松手参考' if self.tracker.reference is not None else '采集松手参考')
        self.reference_cancel_button.setEnabled(self.tracker.calibrating)
        self.status.setText(self.tracker.message)
        self.canvas.message = self.tracker.message
        self.canvas.update()
