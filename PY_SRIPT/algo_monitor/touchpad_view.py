"""相对触摸板的试验画布和控制面板；系统鼠标仅在用户勾选后移动。"""
import time
from collections import deque

from qt_compat import QtCore, QtGui, QtWidgets, Signal
from touchpad import TouchpadTracker, FractionalMotion
from centroid_filter import centroid_mm, SENSOR_PITCH_MM, SENSOR_DIAMETER_MM


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
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0,0,0,0)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        scroll.setWidget(content)
        outer.addWidget(scroll)
        self.enabled_box = QtWidgets.QCheckBox('启用相对触摸板试验')
        layout.addWidget(self.enabled_box)
        self.source = QtWidgets.QComboBox()
        self.source.addItem('上位机 Signal 质心', 'host_signal')
        self.source.addItem('连续位置权重（推荐）', 'host_continuous')
        self.source.addItem('Raw 减松手参考', 'raw_reference')
        self.tracker.set_source('host_signal')
        source_form = QtWidgets.QFormLayout()
        source_form.addRow('触摸板输入来源', self.source)
        layout.addLayout(source_form)
        note = QtWidgets.QLabel('连续位置权重：已增益、低通的input_val减独立松手参考；通道退出activate时权重不突归零。\n'
                               '旧Signal模式保留用于对照。阵列4行5列，中心间距8毫米，点直径5毫米。\n'
                               '需至少一对上下左右相邻的activate通道；不足两路或彼此不相邻时保持质心、位移为0。\n'
                               'dx/dy = 连续滤波质心之差 × 灵敏度；首次落指仅锚定，整体卸载时停止位移。\n'
                               '需要20路Raw及基线初始化；Raw参考模式仍需另行采集松手参考。')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.reference_button = QtWidgets.QPushButton('采集松手参考')
        self.force_reference_button = QtWidgets.QPushButton('强制采集')
        self.force_reference_button.setToolTip('忽略通道状态和Signal释放门槛，仍需连续1秒、至少32帧的稳定Raw；请先松手')
        reference_buttons = QtWidgets.QHBoxLayout()
        reference_buttons.addWidget(self.reference_button)
        reference_buttons.addWidget(self.force_reference_button)
        layout.addLayout(reference_buttons)
        self.reference_button.setEnabled(False)
        self.force_reference_button.setEnabled(False)
        self.reference_group = QtWidgets.QGroupBox('松手参考采集（仅Raw参考模式）')
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
        self.continuous_group = QtWidgets.QGroupBox('连续位置权重 · 松手参考与残留抑制')
        continuous_form = QtWidgets.QFormLayout(self.continuous_group)
        self.position_reference_seconds = QtWidgets.QDoubleSpinBox()
        self.position_reference_seconds.setRange(.2, 3.)
        self.position_reference_seconds.setDecimals(2)
        self.position_reference_seconds.setSingleStep(.1)
        self.position_reference_seconds.setValue(.5)
        self.position_reference_seconds.setKeyboardTracking(False)
        self.position_noise_floor = self._spin(2, 200, 20)
        continuous_form.addRow('释放稳定窗口（秒）', self.position_reference_seconds)
        continuous_form.addRow('权重噪声下限（count）', self.position_noise_floor)
        self.position_decay_enabled = QtWidgets.QCheckBox('平滑抑制非activate通道的残留权重')
        self.position_decay_enabled.setChecked(True)
        self.position_decay_enabled.setToolTip('仅影响连续模式的位置权重；通道状态和整体卸载判定保留原始口径。关闭可对照上一版。')
        continuous_form.addRow(self.position_decay_enabled)
        self.position_decay_ms = self._spin(20,500,80)
        self.position_restore_ms = self._spin(10,200,20)
        self.position_decay_percent = self._spin(0,100,60)
        continuous_form.addRow('残留衰减时间常数（毫秒）', self.position_decay_ms)
        continuous_form.addRow('重新加载恢复时间常数（毫秒）', self.position_restore_ms)
        continuous_form.addRow('残留最大衰减比例（%）', self.position_decay_percent)
        self.position_decay_percent.setToolTip('默认60：非activate通道最终保留40%权重。100趋近完全抑制；0保持上一版权重。')
        self.position_decay_ms.setToolTip('经过1个时间常数残留约37%，3个约5%；调小抑制更快，调大过渡更柔和。')
        self.position_restore_ms.setToolTip('activate时位置权重平滑恢复。调小可更快跟随新的触摸区域。')
        self.position_reference_status = QtWidgets.QLabel()
        self.position_reference_status.setWordWrap(True)
        continuous_form.addRow(self.position_reference_status)
        layout.addWidget(self.continuous_group)
        # 百分号只放标签，输入框内保持纯数字；两项阈值用于当前来源的卸载门控。
        release_group = QtWidgets.QGroupBox('整体卸载检测（当前输入来源）')
        release_form = QtWidgets.QFormLayout(release_group)
        self.release_drop = self._spin(5, 80, TouchpadTracker.RAW_RELEASE_DROP_DEFAULT)
        self.release_rise = self._spin(0, 20, TouchpadTracker.RAW_RELEASE_RISE_DEFAULT)
        release_form.addRow('卸载下降阈值（%）', self.release_drop)
        release_form.addRow('单路回升容差（下降量 %）', self.release_rise)
        self.release_drop.setToolTip('最近80毫秒下降量相对于此前总Signal、连续压力或Raw差值的比例。调低更早停；过低可能中断正常滑动。')
        self.release_rise.setToolTip('允许单路回升的幅度，占整体下降量的比例；同时保留6倍噪声容差。调高可容许释放时少量回弹，过高可能误停正常滑动。')
        release_note = QtWidgets.QLabel('下降阈值调低可更早停止；回升容差调高可容许少量回弹。\n'
                                       '默认25、0；修改立即生效并重新锚定。上位机Signal模式不需要触摸板参考。')
        release_note.setWordWrap(True)
        release_form.addRow(release_note)
        layout.addWidget(release_group)
        help_text = QtWidgets.QLabel('截止频率越低越平稳，也会增加延迟；快速运动时自动减弱滤波。\n'
                                    '使用固定灵敏度、全部20路质心；逐通道Raw增益在来源计算阶段应用。')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        self.system_mouse = QtWidgets.QCheckBox('控制系统鼠标（窗口激活时；Esc停止）')
        layout.addWidget(self.system_mouse)
        self.center_button = QtWidgets.QPushButton('试验光标回到中心')
        layout.addWidget(self.center_button)
        self.status = QtWidgets.QLabel(self.tracker.message)
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.coordinates = QtWidgets.QLabel('质心 -- · dx 0.000 / dy 0.000 格 · 位移 0.000 / 0.000 像素')
        self.coordinates.setWordWrap(True)
        layout.addWidget(self.coordinates)
        layout.addStretch(1)
        self.enabled_box.toggled.connect(self._enabled_changed)
        self.reference_button.clicked.connect(lambda: self.start_reference())
        self.force_reference_button.clicked.connect(lambda: self.start_reference(force=True))
        self.center_button.clicked.connect(self.center)
        self.system_mouse.toggled.connect(self._mouse_changed)
        self.source.currentIndexChanged.connect(self._source_changed)
        self.position_decay_enabled.toggled.connect(self._configure)
        for widget in self.setting_inputs():
            widget.valueChanged.connect(self._configure)
        self._display()

    @staticmethod
    def _spin(low, high, value):
        widget = QtWidgets.QSpinBox()
        widget.setRange(low, high)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    def settings(self):
        return dict(source=self.source.currentData(), sensitivity=self.sensitivity.value(), threshold=self.threshold.value(),
                    min_cutoff=self.cutoff.value(), raw_release_drop_percent=self.release_drop.value(),
                    raw_release_rise_percent=self.release_rise.value(),
                    position_reference_seconds=self.position_reference_seconds.value(),
                    position_noise_floor=self.position_noise_floor.value(),
                    position_decay_enabled=self.position_decay_enabled.isChecked(),
                    position_decay_ms=self.position_decay_ms.value(), position_restore_ms=self.position_restore_ms.value(),
                    position_decay_percent=self.position_decay_percent.value())

    def setting_inputs(self):
        # 设置变更与自动保存共用清单，新增输入不会漏掉保存连接。
        return (self.sensitivity, self.threshold, self.cutoff, self.release_drop, self.release_rise,
                self.position_reference_seconds, self.position_noise_floor, self.position_decay_ms, self.position_restore_ms,
                self.position_decay_percent)

    def load_settings(self, settings):
        if not isinstance(settings, dict):
            return
        source = settings.get('source', 'host_signal')
        index = self.source.findData(source)
        if index >= 0:
            self.source.setCurrentIndex(index)
        if isinstance(settings.get('position_decay_enabled'),bool):
            self.position_decay_enabled.setChecked(settings['position_decay_enabled'])
        for key, widget in (('sensitivity',self.sensitivity), ('threshold',self.threshold),
                            ('min_cutoff',self.cutoff), ('raw_release_drop_percent',self.release_drop),
                            ('raw_release_rise_percent',self.release_rise),
                            ('position_reference_seconds',self.position_reference_seconds),
                            ('position_noise_floor',self.position_noise_floor),
                            ('position_decay_ms',self.position_decay_ms), ('position_restore_ms',self.position_restore_ms),
                            ('position_decay_percent',self.position_decay_percent)):
            if key in settings:
                try:
                    value = float(settings[key]) if key in ('min_cutoff','position_reference_seconds') else int(settings[key])
                    widget.setValue(value)
                except (ValueError, TypeError, OverflowError):
                    pass

    def _configure(self, *_):
        self.cancel_motion()
        self.tracker.configure(**self._tracker_settings())
        self.motion.reset()

    def _tracker_settings(self):
        return {key: value for key, value in self.settings().items() if key != 'source'}

    def _source_changed(self, *_):
        self.cancel_motion()
        self.system_mouse.setChecked(False)
        self.motion.reset()
        self.not_before = time.perf_counter()
        self.tracker.set_source(self.source.currentData())
        self._display()

    def set_raw_gains(self, gains):
        # 柱状图增益改变时中止手势、采集和鼠标输出，并丢弃修改前的排队帧。
        if self.tracker.set_raw_gains(gains):
            self.cancel_motion()
            self.system_mouse.setChecked(False)
            self.motion.reset()
            self.not_before = time.perf_counter()
            self._display()

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

    def start_reference(self, force=False):
        if self.tracker.source in self.tracker.HOST_SOURCES:
            return
        self.cancel_motion()
        self.system_mouse.setChecked(False)
        self.motion.reset()
        self.not_before = time.perf_counter()
        self._reference_last_received = self.not_before
        self.tracker.start_reference(force=force)
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
        if self.tracker.source in self.tracker.HOST_SOURCES:
            message = message.split('，')[0] + ('；请松手重新建立位置参考' if self.tracker.source == 'host_continuous'
                                              else '；等待有效上位机Signal，重新锚定')
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
                self.motion.reset()
                self.cancel_motion()
                continue
            was_ready = self.tracker.reference is not None
            if self.tracker.calibrating:
                self._reference_last_received = sample['sample_timestamp']
            previous_time,previous_sequence = self.tracker.timestamp,self.tracker.sequence
            dx,dy = self.tracker.feed(sample)
            host = self.tracker.source in self.tracker.HOST_SOURCES
            continuous = self.tracker.source == 'host_continuous'
            interrupted = (not self.tracker.enabled or (not host and self.tracker.reference is None) or self.tracker.calibrating
                or not sample.get('has_raw') or sample.get('channel_count') != 20
                or (previous_time is not None and sample['sample_timestamp']-previous_time>self.tracker.MAX_GAP)
                or (previous_sequence is not None and sample['sequence']!=previous_sequence+1)
                or self.tracker.motion_block_reason in ('position_jump', 'invalid_sample', 'source_mode_changed', 'host_generation_changed', 'position_reference_waiting'))
            # 相邻激活条件失效停止传感器位移，并让表盘直接吸附；异常中断仍只取消手势。
            adjacency_lost = self.tracker.motion_block_reason == 'no_adjacent_activate_pair'
            self.motion_sample.emit(dict(timestamp=sample['sample_timestamp'], contact=self.tracker.contact,
                                         dx=dx/self.tracker.sensitivity,dy=dy/self.tracker.sensitivity,
                                         cancel=interrupted or adjacency_lost,
                                         snap_on_cancel=adjacency_lost and not interrupted))
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
                    source='host_continuous_centroid' if continuous else 'host_signal_centroid' if host else 'raw_per_channel_reference', settings=self.settings(),
                    algorithm_input='host_input_val_minus_position_reference' if continuous else 'host_signal' if host else 'raw_times_channel_gain', raw_gains=self.tracker.raw_gains,
                    raw_xy_mm=centroid_mm(self.tracker.raw_position), filtered_xy_mm=centroid_mm(self.tracker.position),
                    centroid_dx_mm=self.tracker.centroid_delta[0]*SENSOR_PITCH_MM,
                    centroid_dy_mm=self.tracker.centroid_delta[1]*SENSOR_PITCH_MM,
                    sensor_pitch_mm=SENSOR_PITCH_MM, sensor_diameter_mm=SENSOR_DIAMETER_MM,
                    host_input_val=sample.get('host_input_val') if host else None,
                    host_baseline=sample.get('host_baseline') if host else None,
                    position_reference=self.tracker.position_reference if continuous else None,
                    position_noise=self.tracker.position_noise if continuous else None,
                    position_weights=self.tracker.position_weights if continuous else None,
                    position_unattenuated_weights=self.tracker.position_unattenuated_weights if continuous else None,
                    position_factors=self.tracker.position_factors if continuous else None,
                    position_weight_model=('state_smoothed_v1' if self.tracker.position_decay_enabled and self.tracker.position_decay_percent > 0
                                           else 'continuous_v1') if continuous else None,
                    centroid_dx=self.tracker.centroid_delta[0], centroid_dy=self.tracker.centroid_delta[1],
                    host_generation=sample.get('host_generation') if host else None,
                    host_signal=sample.get('host_signal') if host else None,
                    host_state=sample.get('host_state') if host else None,
                    host_active_channels=self.tracker.host_active_channels if host else None,
                    host_adjacent_pairs=self.tracker.host_adjacent_pairs if host else None,
                    centroid_held=self.tracker.centroid_held,
                    raw_only=bool(sample.get('raw_only', False)),
                    reference=self.tracker.reference if not was_ready else None,
                    noise=self.tracker.noise if not was_ready else None,
                    reference_status=dict(self.tracker.reference_status),
                    active_weight_share=self.tracker.active_weight_share,
                    motion_block_reason=self.tracker.motion_block_reason,
                    message=self.tracker.message))
        was_contact = self.tracker.contact
        self.tracker.expire(now)
        if was_contact and not self.tracker.contact:
            self.motion.reset()
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
        if capture['forced']:
            titles.update(waiting='强制采集已开始 · 等待有效Raw', collecting='强制采集中 · 请保持松手',
                          unstable='强制采集 · 等待Raw稳定', complete='强制采集完成')
        color = '#70e3a0' if stage == 'complete' else '#ffd479' if self.tracker.calibrating else '#9bb7ca'
        self.reference_status_label.setText(titles[stage])
        self.reference_status_label.setStyleSheet(f'color:{color};font-weight:bold;font-size:15px;')
        self.reference_progress.setValue(capture['progress'])
        self.reference_progress.setFormat('采集完成 100%' if stage == 'complete' else '%p%')
        detail = capture['detail']
        if stage in ('collecting', 'unstable', 'complete'):
            detail += f"\n有效采样 {capture['elapsed']:.2f} / 1.00 秒 · {capture['frames']} 帧（至少32帧）"
        self.reference_detail.setText(detail)
        host = self.tracker.source in self.tracker.HOST_SOURCES
        self.continuous_group.setVisible(self.tracker.source == 'host_continuous')
        self.position_reference_status.setText('参考已就绪：全阵列idle且稳定时更新；按压、slowback、recover期间冻结。'
                                              if self.tracker.position_reference is not None else
                                              '等待参考：完全松手，全部通道idle且输入稳定后自动建立；无需点击采集。')
        self.reference_group.setVisible(not host)
        self.reference_button.setVisible(not host)
        self.force_reference_button.setVisible(not host)
        self.reference_button.setEnabled(not host and self.tracker.enabled and not self.tracker.calibrating)
        self.reference_button.setText('正在采集…' if self.tracker.calibrating else
                                      '重新采集松手参考' if self.tracker.reference is not None else '采集松手参考')
        self.force_reference_button.setEnabled(not host and self.tracker.enabled and not (self.tracker.calibrating and self.tracker.reference_forced))
        self.force_reference_button.setText('强制采集中…' if self.tracker.calibrating and self.tracker.reference_forced else '强制采集')
        self.reference_cancel_button.setEnabled(self.tracker.calibrating)
        self.status.setText(self.tracker.message)
        xy = self.tracker.position
        position = '--' if xy is None else f'X {xy[0]:.4f} / Y {xy[1]:.4f} 格'
        if self.tracker.centroid_held:
            position += '（保持）'
        if xy is not None:
            physical = centroid_mm(xy)
            position += f' · X {physical[0]:.2f} / Y {physical[1]:.2f} 毫米（CH0原点）'
        dx, dy = self.tracker.centroid_delta
        pixels = self.tracker.delta
        self.coordinates.setText(f'质心 {position} · dx {dx:.4f} / dy {dy:.4f} 格\n'
                                 f'触摸板位移 dx {pixels[0]:.3f} / dy {pixels[1]:.3f} 像素')
        self.canvas.message = self.tracker.message
        self.canvas.update()
