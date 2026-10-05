"""4行5列传感阵列的三维柱体视图，支持固件及上位机 Signal 和增益 Raw。"""
import math

from qt_compat import QtCore, QtGui, QtWidgets, Signal
from centroid_filter import CHANNEL_GRID
from data_buffer import RX_FPS_STALE_SECONDS
from raw_gain import RAW_GAIN_MAX, RAW_DISPLAY_MAX, normalize_raw_gains, apply_raw_gains


SOURCE_NAMES = {'signal': 'Signal', 'raw': 'Raw Val × 增益', 'host_signal': '上位机计算 Signal'}
SOURCE_LIMITS = {'signal': 65535, 'raw': RAW_DISPLAY_MAX, 'host_signal': RAW_DISPLAY_MAX}
DEFAULTS = dict(source='signal', auto_range=False, signal_max=20000, raw_max=65535, host_signal_max=20000,
                azimuth=35, elevation=35, show_values=True, raw_gains=(1.0,) * 20)


def normalize_settings(values):
    settings = dict(DEFAULTS, raw_gains=list(DEFAULTS['raw_gains']))
    if not isinstance(values, dict):
        return settings
    if values.get('source') in tuple(SOURCE_NAMES):
        settings['source'] = values['source']
    for key in ('auto_range', 'show_values'):
        if isinstance(values.get(key), bool):
            settings[key] = values[key]
    settings['raw_gains'] = list(normalize_raw_gains(values.get('raw_gains')))
    for key, limits in (('signal_max', (1, 65535)), ('raw_max', (1, RAW_DISPLAY_MAX)),
                        ('host_signal_max', (1, RAW_DISPLAY_MAX)),
                        ('azimuth', (-180, 180)), ('elevation', (15, 80))):
        try:
            value = float(values.get(key, settings[key]))
            if math.isfinite(value):
                settings[key] = max(limits[0], min(limits[1], round(value)))
        except (ValueError, TypeError, OverflowError):
            pass
    return settings


class ArrayBarView(QtWidgets.QWidget):
    camera_changed = Signal(int, int)
    MAX_HEIGHT = 2.6

    def __init__(self, parent=None):
        super().__init__(parent)
        self.settings = normalize_settings({})
        self.values = None
        self._sample = None
        self._message = '等待20通道数据'
        self._mouse_origin = None
        self._hit_faces = []
        self._auto_max = DEFAULTS['signal_max']
        self.setMinimumSize(360, 360)
        self.setMouseTracking(True)
        self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)

    def configure(self, values):
        settings = normalize_settings(values)
        if settings['source'] != self.settings['source']:
            self._auto_max = settings[settings['source'] + '_max']
        self.settings = settings
        self.set_sample(self._sample)

    def set_sample(self, sample):
        self._sample = sample
        source = self.settings['source']
        values = None
        if sample is None:
            message = '等待20通道数据'
        elif sample.get('age_seconds', float('inf')) > RX_FPS_STALE_SECONDS:
            message = '数据已中断 · 等待新采样'
        elif sample.get('channel_count') != 20:
            message = '需要完整20通道数据'
        elif source == 'raw' and not sample.get('has_raw'):
            message = 'Raw Val未上传 · 请在波形设置中选择Debug输出'
        elif source == 'host_signal' and sample.get('host_signal') is None:
            message = sample.get('host_signal_message', '等待上位机计算 Signal · 请保持释放完成初始化')
        else:
            candidate = sample.get(source, ())
            if len(candidate) != 20 or not all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0 for v in candidate):
                message = '本帧数值无效 · 等待有效20通道采样'
            else:
                # 与触摸板使用同一Raw增益，保留接收样本的原始读数。
                values = apply_raw_gains(candidate, self.settings['raw_gains']) if source == 'raw' else tuple(candidate)
                peak = max(values)
                channel = values.index(peak)
                name = SOURCE_NAMES[source]
                message = f'{name} · 最大值 CH{channel}: {peak:g} count' if peak else f'{name} · 全部通道为0'
                if source == 'host_signal' and sample.get('host_signal_message'):
                    message += ' · ' + sample['host_signal_message']
                if self.settings['auto_range'] and peak:
                    # 按整档向上取量程，全零时保持量程，避免零点附近反复缩放。
                    magnitude = 10 ** math.floor(math.log10(peak))
                    self._auto_max = next(step * magnitude for step in (1, 2, 5, 10) if step * magnitude >= peak)
        if values != self.values or message != self._message:
            self.values, self._message = values, message
            self.update()
        elif self.isVisible():
            self.update()

    def height_limit(self):
        return self._auto_max if self.settings['auto_range'] else self.settings[self.settings['source'] + '_max']

    def bars(self):
        # 与二维阵列共用按列编号的映射；隐藏波形或数据行不影响这20个位置。
        limit = self.height_limit()
        return [dict(channel=channel, row=row, col=col,
                     value=None if self.values is None else self.values[channel],
                     height=0 if self.values is None else min(1, self.values[channel] / limit) * self.MAX_HEIGHT)
                for row, channels in enumerate(CHANNEL_GRID) for col, channel in enumerate(channels)]

    def camera_point(self, x, y, z):
        yaw = math.radians(self.settings['azimuth'])
        pitch = math.radians(self.settings['elevation'])
        across = math.cos(yaw) * x - math.sin(yaw) * y
        forward = math.sin(yaw) * x + math.cos(yaw) * y
        return across, math.sin(pitch) * forward - math.cos(pitch) * z, math.cos(pitch) * forward + math.sin(pitch) * z

    def scene_geometry(self):
        bounds = [self.camera_point(x, y, z) for x in (-3.3, 2.9) for y in (-2.8, 2.5) for z in (0, 3)]
        lo_x, hi_x = min(p[0] for p in bounds), max(p[0] for p in bounds)
        lo_y, hi_y = min(p[1] for p in bounds), max(p[1] for p in bounds)
        scale = max(1, min((self.width() - 100) / (hi_x - lo_x), (self.height() - 190) / (hi_y - lo_y)))
        origin = QtCore.QPointF(self.width()/2 - (lo_x+hi_x)*scale/2,
                               100 + (self.height()-190)/2 - (lo_y+hi_y)*scale/2)
        return origin, scale

    def project(self, point, origin, scale):
        x, y, _ = self.camera_point(*point)
        return origin + QtCore.QPointF(x*scale, y*scale)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            point = event.position() if hasattr(event, 'position') else event.localPos()
            self._mouse_origin = (point, self.settings['azimuth'], self.settings['elevation'])
            self.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
            event.accept()

    def mouseMoveEvent(self, event):
        point = event.position() if hasattr(event, 'position') else event.localPos()
        if self._mouse_origin is not None:
            previous, yaw, pitch = self._mouse_origin
            yaw = round((yaw + (point.x()-previous.x())*.4 + 180) % 360 - 180)
            pitch = max(15, min(80, round(pitch - (point.y()-previous.y())*.3)))
            self.camera_changed.emit(yaw, pitch)
            self.update()
            event.accept()
        else:
            channel = next((channel for polygon, channel in reversed(self._hit_faces)
                            if polygon.containsPoint(point, QtCore.Qt.FillRule.OddEvenFill)), None)
            if channel is None or self.values is None:
                self.setToolTip('按住左键拖动可旋转视角')
            else:
                name = SOURCE_NAMES[self.settings['source']]
                detail = f'\n原始Raw: {self._sample["raw"][channel]:g} count · 增益: {self.settings["raw_gains"][channel]:g}' if self.settings['source'] == 'raw' else ''
                self.setToolTip(f'CH{channel} · 第{channel%4+1}行 / 第{channel//4+1}列\n{name}: {self.values[channel]:g} count{detail}')

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._mouse_origin = None
            self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
            event.accept()

    def hideEvent(self, event):
        self._mouse_origin = None
        self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
        super().hideEvent(event)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#080d13'))
        font = QtGui.QFont(self.font());font.setPixelSize(22);font.setBold(True)
        painter.setFont(font);painter.setPen(QtGui.QColor('#dceaf5'))
        name = SOURCE_NAMES[self.settings['source']]
        painter.drawText(QtCore.QRectF(16, 18, self.width()-32, 32), QtCore.Qt.AlignmentFlag.AlignCenter,
                         f'4 × 5 三维柱状图 · {name}')
        font.setPixelSize(12);font.setBold(False);painter.setFont(font)
        painter.setPen(QtGui.QColor('#9bb7ca'))
        painter.drawText(QtCore.QRectF(16, 53, self.width()-32, 24), QtCore.Qt.AlignmentFlag.AlignCenter,
                         '柱高和颜色表示同一量程下的数值 · 按住左键拖动旋转')
        origin, scale = self.scene_geometry()
        project = lambda point: self.project(point, origin, scale)
        polygon = lambda points: QtGui.QPolygonF([project(point) for point in points])
        floor = polygon([(-2.5,-2,0), (2.5,-2,0), (2.5,2,0), (-2.5,2,0)])
        painter.setPen(QtGui.QPen(QtGui.QColor('#496174'), 1.2))
        painter.setBrush(QtGui.QColor('#12212e'));painter.drawPolygon(floor)
        painter.setPen(QtGui.QPen(QtGui.QColor('#354a5b'), 1))
        for col in range(6):
            painter.drawLine(project((col-2.5,-2,0)), project((col-2.5,2,0)))
        for row in range(5):
            painter.drawLine(project((-2.5,row-2,0)), project((2.5,row-2,0)))

        # 所有可见面按相机深度从远到近绘制，旋转后仍保留空间遮挡关系。
        faces = []
        yaw = math.radians(self.settings['azimuth'])
        normals = (math.sin(yaw), math.cos(yaw))
        bars = self.bars()
        for bar in bars:
            x, y, height = bar['col']-2, bar['row']-1.5, bar['height']
            low_x, high_x, low_y, high_y = x-.35, x+.35, y-.35, y+.35
            color = QtGui.QColor.fromHsvF(.55*(1-height/self.MAX_HEIGHT), .72, .8) if self.values is not None else QtGui.QColor('#314555')
            top = [(low_x,low_y,height), (high_x,low_y,height), (high_x,high_y,height), (low_x,high_y,height)]
            visible = [(top, color.lighter(125))]
            if height > 0:
                side_x = high_x if normals[0] >= 0 else low_x
                side_y = high_y if normals[1] >= 0 else low_y
                visible.extend([([(side_x,low_y,0), (side_x,high_y,0), (side_x,high_y,height), (side_x,low_y,height)],color.darker(135)),
                                ([(low_x,side_y,0), (high_x,side_y,0), (high_x,side_y,height), (low_x,side_y,height)],color.darker(110))])
            for points, shade in visible:
                depth = sum(self.camera_point(*point)[2] for point in points)/4
                faces.append((depth, polygon(points), shade, bar['channel']))
        self._hit_faces = []
        for _, shape, shade, channel in sorted(faces, key=lambda face: face[0]):
            painter.setPen(QtGui.QPen(shade.lighter(135), .8));painter.setBrush(shade)
            painter.drawPolygon(shape)
            self._hit_faces.append((shape, channel))

        # 数字保留实际值；标签错开避免重叠，用引线标出所属柱体。
        font.setPixelSize(max(9, min(12, round(scale*.16))));painter.setFont(font)
        occupied = []
        labels = []
        for bar in sorted(bars, key=lambda bar:bar['height'], reverse=True):
            anchor = project((bar['col']-2, bar['row']-1.5, bar['height']))
            text = f"CH{bar['channel']}"
            if self.settings['show_values'] and bar['value'] is not None:
                text += f"\n{bar['value']:g}"
            height = 32 if '\n' in text else 20
            rect = QtCore.QRectF(anchor.x()-29, anchor.y()-height-5, 58, height)
            for shift in range(12):
                candidate = rect.translated(0,-shift*(height+3))
                if not any(candidate.adjusted(-2,-2,2,2).intersects(other) for other in occupied):
                    rect = candidate
                    break
            occupied.append(rect)
            labels.append((rect,anchor,text))
        for rect,anchor,text in labels:
            painter.setPen(QtGui.QPen(QtGui.QColor('#7993a8'),.8))
            painter.drawLine(anchor,QtCore.QPointF(rect.center().x(),rect.bottom()))
            painter.fillRect(rect, QtGui.QColor(8,13,19,210))
            painter.setPen(QtGui.QColor('#edf5fb'))
            painter.drawText(rect, QtCore.Qt.AlignmentFlag.AlignCenter, text)

        painter.setFont(self.font());painter.setPen(QtGui.QColor('#a7c0d3'))
        axis_origin = (-2.7,2.2,0)
        painter.drawLine(project(axis_origin),project((-2.7,2.2,self.MAX_HEIGHT)))
        for i in range(5):
            anchor = project((-2.7,2.2,self.MAX_HEIGHT*i/4))
            painter.drawLine(anchor,anchor+QtCore.QPointF(-5,0))
            painter.drawText(QtCore.QRectF(anchor.x()-65,anchor.y()-10,55,20),
                             QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignVCenter,
                             f'{self.height_limit()*i/4:g}')
        status = self._message
        if self.values is not None:
            clipped = sum(value > self.height_limit() for value in self.values)
            status += f' · {clipped}路超量程，柱高已裁剪' if clipped else ''
        painter.drawText(QtCore.QRectF(16,self.height()-67,self.width()-32,28),QtCore.Qt.AlignmentFlag.AlignCenter,status)
        painter.setPen(QtGui.QColor('#8297a9'))
        painter.drawText(QtCore.QRectF(16,self.height()-38,self.width()-32,24),QtCore.Qt.AlignmentFlag.AlignCenter,
                         f'高度范围 0—{self.height_limit():g} count · '+('自动量程' if self.settings['auto_range'] else '固定量程'))


class ArrayBarPanel(QtWidgets.QScrollArea):
    settings_changed = Signal()

    def __init__(self, view, parent=None):
        super().__init__(parent)
        self.view = view
        self._loading = False
        self._settings = normalize_settings({})
        self.setWidgetResizable(True)
        page = QtWidgets.QWidget();layout = QtWidgets.QVBoxLayout(page);self.setWidget(page)
        group = QtWidgets.QGroupBox('柱状图数据');form = QtWidgets.QFormLayout(group);layout.addWidget(group)
        self.source = QtWidgets.QComboBox()
        self.source.addItem('Signal', 'signal');self.source.addItem('Raw Val', 'raw')
        self.source.addItem('上位机计算 Signal', 'host_signal')
        form.addRow('数据来源',self.source)
        self.auto_range = QtWidgets.QCheckBox('自动调整高度量程')
        form.addRow(self.auto_range)
        self.maximum = self._spin(1,65535,DEFAULTS['signal_max'])
        form.addRow('高度上限（count）',self.maximum)
        self.show_values = QtWidgets.QCheckBox('显示每路实际数值');self.show_values.setChecked(True)
        form.addRow(self.show_values)
        note = QtWidgets.QLabel('Signal为固件上报值；Raw Val按下表增益显示，包含各通道静态底值。\n'
                               '上位机计算 Signal使用“基线与状态机”的输出，仅activate有信号，已含Raw增益，不再重复乘增益。需要完整20通道Raw采样，初始化时请保持释放。\n'
                               '数值超过高度上限时只裁剪柱高，可提高上限或使用自动量程。三种来源的高度上限分别保存，选择独立于波形来源。')
        note.setWordWrap(True);layout.addWidget(note)
        group = QtWidgets.QGroupBox('Raw Val逐通道增益');gain_layout = QtWidgets.QVBoxLayout(group);layout.addWidget(group)
        gain_note = QtWidgets.QLabel('输入值 = 原始Raw Val × 增益。柱状图与触摸板（含松手参考采集）共用；0停用该路Raw权重。修改增益后请重新采集触摸板参考。')
        gain_note.setWordWrap(True);gain_layout.addWidget(gain_note)
        self.gain_table = QtWidgets.QTableWidget(20, 2)
        self.gain_table.setHorizontalHeaderLabels(['通道', '增益'])
        self.gain_table.verticalHeader().hide()
        self.gain_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.gain_table.verticalHeader().setDefaultSectionSize(max(26, self.fontMetrics().height() + 10))
        self.gain_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.gain_table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.raw_gain_inputs = []
        for channel in range(20):
            item = QtWidgets.QTableWidgetItem(f'CH{channel}')
            item.setFlags(QtCore.Qt.ItemFlag.ItemIsEnabled | QtCore.Qt.ItemFlag.ItemIsSelectable)
            self.gain_table.setItem(channel, 0, item)
            gain = QtWidgets.QDoubleSpinBox()
            gain.setDecimals(4);gain.setRange(0, RAW_GAIN_MAX);gain.setSingleStep(0.1);gain.setValue(1.0)
            gain.setKeyboardTracking(False)
            gain.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
            gain.valueChanged.connect(self._changed)
            self.raw_gain_inputs.append(gain)
            self.gain_table.setCellWidget(channel, 1, gain)
        # 按全部20行及表头计算高度，避免通道隐藏在表格内部滚动区。
        self.gain_table.setFixedHeight(self.gain_table.horizontalHeader().sizeHint().height()
                                       + sum(self.gain_table.rowHeight(row) for row in range(20))
                                       + self.gain_table.frameWidth() * 2)
        gain_layout.addWidget(self.gain_table)
        reset_gains = QtWidgets.QPushButton('全部增益恢复为1.0')
        reset_gains.clicked.connect(self._reset_raw_gains);gain_layout.addWidget(reset_gains)
        mapping = QtWidgets.QLabel('阵列按列编号，与“阵列质心”保持一致：\n'
                                  'CH0    CH4    CH8    CH12    CH16\n'
                                  'CH1    CH5    CH9    CH13    CH17\n'
                                  'CH2    CH6    CH10   CH14    CH18\n'
                                  'CH3    CH7    CH11   CH15    CH19\n\n'
                                  '按住图中的鼠标左键拖动可旋转，悬停查看通道与数值。全部20路独立显示，不受波形或表格隐藏设置影响。设置自动保存，下次启动恢复。')
        mapping.setWordWrap(True);layout.addWidget(mapping);layout.addStretch(1)
        self.source.currentIndexChanged.connect(self._source_changed)
        self.auto_range.toggled.connect(self._changed)
        self.show_values.toggled.connect(self._changed)
        self.maximum.valueChanged.connect(self._changed)
        view.camera_changed.connect(self._set_camera)

    @staticmethod
    def _spin(low, high, value):
        widget = QtWidgets.QSpinBox();widget.setRange(low,high);widget.setValue(value)
        widget.setKeyboardTracking(False)
        widget.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        return widget

    def showEvent(self, event):
        super().showEvent(event)
        # 窗口样式会改变表头和行高，显示面板时按最终尺寸展开全部通道。
        table = self.gain_table
        table.setFixedHeight(table.horizontalHeader().sizeHint().height()
                             + sum(table.rowHeight(row) for row in range(table.rowCount()))
                             + table.frameWidth() * 2)

    def settings(self):
        return dict(self._settings, raw_gains=list(self._settings['raw_gains']))

    def load_settings(self, values):
        self._settings = normalize_settings(values)
        self._loading = True
        self.source.setCurrentIndex(self.source.findData(self._settings['source']))
        self.auto_range.setChecked(self._settings['auto_range'])
        self.maximum.setMaximum(SOURCE_LIMITS[self._settings['source']])
        self.maximum.setValue(self._settings[self._settings['source']+'_max'])
        self.maximum.setEnabled(not self._settings['auto_range'])
        self.show_values.setChecked(self._settings['show_values'])
        for gain, value in zip(self.raw_gain_inputs, self._settings['raw_gains']):
            gain.setValue(value)
        self._loading = False
        self.view.configure(self._settings)

    def _source_changed(self, *_):
        if self._loading:return
        self._settings['source'] = self.source.currentData()
        self.load_settings(self._settings)
        self.settings_changed.emit()

    def _changed(self, *_):
        if self._loading:return
        self._settings.update(auto_range=self.auto_range.isChecked(),show_values=self.show_values.isChecked(),
                              raw_gains=[gain.value() for gain in self.raw_gain_inputs])
        self._settings[self._settings['source']+'_max'] = self.maximum.value()
        self.load_settings(self._settings)
        self.settings_changed.emit()

    def _reset_raw_gains(self):
        self._settings['raw_gains'] = [1.0] * 20
        self.load_settings(self._settings)
        self.settings_changed.emit()

    def _set_camera(self, yaw, pitch):
        self._settings.update(azimuth=yaw,elevation=pitch)
        self.load_settings(self._settings)
        self.settings_changed.emit()
