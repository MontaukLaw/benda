"""圆形蜂窝应用表盘：矢量图标、直接拖动、惯性和自动居中。"""
import math
import time

from qt_compat import QtCore, QtGui, QtWidgets, Signal
from watch_face import WatchFaceModel, DEFAULTS, LIMITS, RADIUS, normalize_settings


APP_NAMES = ('运动','音乐','天气','相机','日历','时钟','信息','健康',
             '地图','设置','邮件','视频','指南针','照片','呼吸','钱包')
COLORS = ('#ed4264','#f75283','#25a7ef','#678599','#f17939','#5c6fe7','#40c674','#ff5363',
          '#43a8ba','#8793ad','#4084ef','#a363ed','#ef6545','#d771c0','#27bdb0','#b39852')


def draw_app(p, app_id, x, y, diameter, numbers):
    p.save()
    p.translate(x,y)
    p.scale(diameter/64,diameter/64)
    color = QtGui.QColor(COLORS[app_id%16]).lighter(100+(app_id//16)*3)
    gradient = QtGui.QLinearGradient(-22,-30,24,30)
    gradient.setColorAt(0,color.lighter(125))
    gradient.setColorAt(1,color.darker(135))
    p.setPen(QtGui.QPen(QtGui.QColor(255,255,255,35),.8))
    p.setBrush(gradient)
    p.drawEllipse(QtCore.QPointF(0,0),31,31)
    p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
    p.setPen(QtGui.QPen(QtGui.QColor('#ffffff'),2.7,QtCore.Qt.PenStyle.SolidLine,
                       QtCore.Qt.PenCapStyle.RoundCap,QtCore.Qt.PenJoinStyle.RoundJoin))
    motif = app_id%16
    if motif in (0,14):
        for radius,start,span in ((18,30,285),(12,115,270),(6,210,270)):
            p.drawArc(QtCore.QRectF(-radius,-radius,2*radius,2*radius),start*16,span*16)
    elif motif == 1:
        p.drawLine(QtCore.QPointF(-7,12),QtCore.QPointF(-7,-12))
        p.drawLine(QtCore.QPointF(11,8),QtCore.QPointF(11,-16))
        p.drawLine(QtCore.QPointF(-7,-12),QtCore.QPointF(11,-16))
        p.setBrush(QtGui.QColor('white'))
        p.drawEllipse(QtCore.QPointF(-12,12),5,4)
        p.drawEllipse(QtCore.QPointF(6,8),5,4)
    elif motif in (2,13):
        p.drawEllipse(QtCore.QPointF(0,-2),9,9)
        for i in range(8):
            angle=i*math.pi/4
            p.drawLine(QtCore.QPointF(14*math.cos(angle),-2+14*math.sin(angle)),
                       QtCore.QPointF(19*math.cos(angle),-2+19*math.sin(angle)))
    elif motif == 3:
        p.drawRoundedRect(QtCore.QRectF(-19,-12,38,27),5,5)
        p.drawEllipse(QtCore.QPointF(0,1),8,8)
        p.drawLine(QtCore.QPointF(-9,-16),QtCore.QPointF(4,-16))
    elif motif == 4:
        p.drawRoundedRect(QtCore.QRectF(-16,-17,32,33),4,4)
        p.drawLine(QtCore.QPointF(-16,-7),QtCore.QPointF(16,-7))
        for x0 in (-8,8):p.drawLine(QtCore.QPointF(x0,-20),QtCore.QPointF(x0,-13))
        font=QtGui.QFont('Segoe UI');font.setPixelSize(18);font.setBold(True);p.setFont(font)
        p.drawText(QtCore.QRectF(-16,-8,32,25),QtCore.Qt.AlignmentFlag.AlignCenter,str(app_id%28+1))
    elif motif == 5:
        p.drawEllipse(QtCore.QPointF(0,-1),18,18)
        p.drawLine(QtCore.QPointF(0,-1),QtCore.QPointF(0,-13))
        p.drawLine(QtCore.QPointF(0,-1),QtCore.QPointF(10,5))
    elif motif == 6:
        p.drawRoundedRect(QtCore.QRectF(-19,-16,38,27),9,9)
        p.drawLine(QtCore.QPointF(-10,11),QtCore.QPointF(-15,18))
        for x0 in (-9,0,9):p.drawEllipse(QtCore.QPointF(x0,-2),1,1)
    elif motif == 7:
        path=QtGui.QPainterPath(QtCore.QPointF(0,17))
        path.cubicTo(-32,-1,-12,-28,0,-12)
        path.cubicTo(12,-28,32,-1,0,17)
        p.setBrush(QtGui.QColor('white'));p.drawPath(path)
    elif motif == 8:
        path=QtGui.QPainterPath(QtCore.QPointF(0,20))
        path.cubicTo(-35,-14,0,-33,13,-13)
        path.cubicTo(21,-2,7,12,0,20)
        p.drawPath(path);p.drawEllipse(QtCore.QPointF(0,-8),5,5)
    elif motif == 9:
        p.drawEllipse(QtCore.QPointF(0,0),13,13);p.drawEllipse(QtCore.QPointF(0,0),5,5)
        for i in range(8):
            a=i*math.pi/4
            p.drawLine(QtCore.QPointF(14*math.cos(a),14*math.sin(a)),QtCore.QPointF(19*math.cos(a),19*math.sin(a)))
    elif motif in (10,15):
        p.drawRoundedRect(QtCore.QRectF(-19,-13,38,27),4,4)
        if motif==10:
            p.drawLine(QtCore.QPointF(-17,-10),QtCore.QPointF(0,3))
            p.drawLine(QtCore.QPointF(17,-10),QtCore.QPointF(0,3))
        else:
            p.drawLine(QtCore.QPointF(-18,-5),QtCore.QPointF(18,-5))
            p.drawLine(QtCore.QPointF(5,6),QtCore.QPointF(12,6))
    else:
        triangle=QtGui.QPolygonF([QtCore.QPointF(-10,-17),QtCore.QPointF(18,0),QtCore.QPointF(-10,17)])
        p.setBrush(QtGui.QColor('white'));p.drawPolygon(triangle)
    if numbers:
        font=QtGui.QFont('Segoe UI');font.setPixelSize(8);font.setBold(True);p.setFont(font)
        p.setPen(QtGui.QColor(255,255,255,200))
        p.drawText(QtCore.QRectF(-18,20,36,10),QtCore.Qt.AlignmentFlag.AlignCenter,f'{app_id+1:03}')
    p.restore()


class WatchFaceCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.model=WatchFaceModel()
        self.owner=None
        self._input_source=None
        self._mouse_position=None
        self._tap_target=None
        self._sizes={}
        self._focus={}
        self._frame_time=time.perf_counter()
        self.setMinimumSize(360,420)
        self.setMouseTracking(True)
        self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
        self.timer=QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.timer.setInterval(8)
        self.timer.timeout.connect(self.animate)
        self.timer.start()

    def geometry_info(self):
        scale=max(.1,min((self.width()-48)/620,(self.height()-160)/620))
        return QtCore.QPointF(self.width()/2,self.height()/2+4),scale

    def local_point(self, position):
        center,scale=self.geometry_info()
        return QtCore.QPointF((position.x()-center.x())/scale,(position.y()-center.y())/scale)

    def animate(self, now=None):
        now=time.perf_counter() if now is None else now
        dt=max(0,min(.1,now-self._frame_time));self._frame_time=now
        self.model.advance(now)
        tau=self.model.settings['size_transition_ms']/1000
        alpha=1 if tau<=0 else 1-math.exp(-dt*4/tau)
        focus_tau=self.model.settings['focus_transition_ms']/1000
        focus_alpha=1 if focus_tau<=0 else 1-math.exp(-dt*4/focus_tau)
        for icon in self.model.icons():
            old=self._sizes.get(icon['id'],icon['size'])
            size=old+(icon['size']-old)*alpha
            # 非选中项立即恢复小尺寸，只有当前选中项使用放大过渡。
            self._sizes[icon['id']]=icon['size'] if not icon['selected'] or abs(size-icon['size'])<.02 else size
            target=1.0 if icon['selected'] else 0.0
            old_focus=self._focus.get(icon['id'],target)
            focus=old_focus+(target-old_focus)*focus_alpha
            self._focus[icon['id']]=target if abs(focus-target)<.002 else focus
        if self.isVisible():self.update()

    def icon_diameter(self, icon):
        # 手势更新可能先于动画帧，绘制与命中均即时遵守单个放大项。
        if not icon['selected']:
            return icon['size']
        return max(self.model.settings['small_size'],min(icon['size'],self._sizes.get(icon['id'],icon['size'])))

    def configure(self, settings):
        self.owner=None
        self._input_source=None
        self.model.configure(settings,time.perf_counter())
        self.update()

    def cancel_gesture(self):
        self.owner=None
        self._input_source=None
        self.model.cancel()
        self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)

    def sensor_sample(self, sample):
        if not self.isVisible() or not self.model.settings['sensor_enabled'] or self.owner=='mouse':
            return
        now=sample['timestamp']
        if sample.get('cancel'):
            if self._input_source=='sensor':
                self.cancel_gesture()
                # 条件失效时仅吸附最近项，不沿用拖动速度；清除来源后连续释放帧不会重启动画。
                if sample.get('snap_on_cancel'):
                    self.model.snap(now)
                    self.update()
            return
        if sample['contact']:
            if self.owner!='sensor':
                self.model.begin(now);self.owner='sensor';self._input_source='sensor'
            gain=self.model.settings['drag_speed']
            dx=sample['dx']*gain*(-1 if self.model.settings['reverse_x'] else 1)
            dy=sample['dy']*gain*(-1 if self.model.settings['reverse_y'] else 1)
            self.model.drag(dx,dy,now)
        elif self.owner=='sensor':
            self.model.release(now)
            self.owner=None
        self.update()

    def mousePressEvent(self,event):
        if event.button()!=QtCore.Qt.MouseButton.LeftButton:return
        point=self.local_point(event.position() if hasattr(event,'position') else event.localPos())
        if math.hypot(point.x(),point.y())>RADIUS:return
        self.model.begin(time.perf_counter());self.owner='mouse';self._input_source='mouse'
        self._mouse_position=point
        hit=[icon for icon in self.model.icons() if math.hypot(icon['x']-point.x(),icon['y']-point.y())<=self.icon_diameter(icon)/2]
        self._tap_target=min(hit,key=lambda icon:math.hypot(icon['x']-point.x(),icon['y']-point.y()))['id'] if hit else None
        self.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
        event.accept()

    def mouseMoveEvent(self,event):
        if self.owner!='mouse':return
        point=self.local_point(event.position() if hasattr(event,'position') else event.localPos());delta=point-self._mouse_position
        self._mouse_position=point
        gain=self.model.settings['drag_speed']/300
        self.model.drag(delta.x()*gain*(-1 if self.model.settings['reverse_x'] else 1),
                        delta.y()*gain*(-1 if self.model.settings['reverse_y'] else 1),time.perf_counter())
        self.update();event.accept()

    def mouseReleaseEvent(self,event):
        if self.owner=='mouse' and event.button()==QtCore.Qt.MouseButton.LeftButton:
            self.model.release(time.perf_counter(),self._tap_target)
            self.owner=None
            self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
            event.accept()

    def hideEvent(self,event):
        self.cancel_gesture()
        super().hideEvent(event)

    def paintEvent(self,event):
        p=QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(),QtGui.QColor('#080d13'))
        font=QtGui.QFont(self.font());font.setPixelSize(24);font.setBold(True);p.setFont(font)
        p.setPen(QtGui.QColor('#e8eef6'))
        p.drawText(QtCore.QRectF(20,18,self.width()-40,38),QtCore.Qt.AlignmentFlag.AlignCenter,'表盘应用')
        font.setPixelSize(12);font.setBold(False);p.setFont(font);p.setPen(QtGui.QColor('#8197ad'))
        p.drawText(QtCore.QRectF(20,56,self.width()-40,24),QtCore.Qt.AlignmentFlag.AlignCenter,'100 个应用  ·  循环蜂窝布局  ·  拖动探索')
        center,scale=self.geometry_info()
        p.save();p.translate(center);p.scale(scale,scale)
        rim=QtGui.QLinearGradient(-240,-300,240,300)
        rim.setColorAt(0,QtGui.QColor('#526273'));rim.setColorAt(.35,QtGui.QColor('#202b38'));rim.setColorAt(1,QtGui.QColor('#0d1620'))
        p.setPen(QtGui.QPen(QtGui.QColor('#647180'),1));p.setBrush(rim)
        p.drawEllipse(QtCore.QPointF(0,0),300,300)
        p.setPen(QtGui.QPen(QtGui.QColor('#070c12'),6));p.setBrush(QtGui.QColor('#020508'))
        p.drawEllipse(QtCore.QPointF(0,0),287,287)
        clip=QtGui.QPainterPath();clip.addEllipse(QtCore.QPointF(0,0),RADIUS,RADIUS);p.setClipPath(clip)
        background=QtGui.QRadialGradient(0,-110,470)
        background.setColorAt(0,QtGui.QColor('#15212e'));background.setColorAt(1,QtGui.QColor('#020508'))
        p.fillRect(QtCore.QRectF(-RADIUS,-RADIUS,2*RADIUS,2*RADIUS),background)
        icons=self.model.icons()
        nearest=self.model.nearest()
        focused=next(icon for icon in icons if icon['selected'])
        inactive_opacity=self.model.settings['inactive_opacity']/100
        glow_strength=self.model.settings['center_glow_strength']/100
        for icon in sorted(icons,key=lambda i:i['size']):
            diameter=self.icon_diameter(icon)
            if icon['distance']>RADIUS+diameter/2:continue
            focus=self._focus.get(icon['id'],1.0 if icon['selected'] else 0.0)
            p.save()
            p.setOpacity(inactive_opacity+(1-inactive_opacity)*focus)
            draw_app(p,icon['id'],icon['x'],icon['y'],diameter,self.model.settings['show_numbers'])
            p.restore()
            # 多层柔光与细亮边跟随选中图标，焦点切换时交叉淡入淡出。
            intensity=focus*glow_strength
            if intensity>.002:
                p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
                radius=diameter*31/64+3
                position=QtCore.QPointF(icon['x'],icon['y'])
                for width,opacity in ((14,12),(9,22),(5,40),(2,245)):
                    p.setPen(QtGui.QPen(QtGui.QColor(152,224,255,round(opacity*intensity)),width))
                    p.drawEllipse(position,radius,radius)
        p.restore()
        name=APP_NAMES[focused['id']%16]
        font.setPixelSize(17);font.setBold(True);p.setFont(font);p.setPen(QtGui.QColor('#e0eaf5'))
        p.drawText(QtCore.QRectF(20,self.height()-78,self.width()-40,28),QtCore.Qt.AlignmentFlag.AlignCenter,f"{name}  /  {focused['id']+1:03}")
        mode=dict(idle='已居中' if nearest['distance']<1 else '等待拖动',drag='正在拖动',inertia='惯性滑行',snap='自动居中')[self.model.phase]
        font.setPixelSize(12);font.setBold(False);p.setFont(font);p.setPen(QtGui.QColor('#8197ad'))
        p.drawText(QtCore.QRectF(20,self.height()-46,self.width()-40,24),QtCore.Qt.AlignmentFlag.AlignCenter,mode+'  ·  再次触摸可立即接管')


class WatchFacePanel(QtWidgets.QScrollArea):
    settings_changed = Signal()
    reference_requested = Signal()

    def __init__(self,canvas,parent=None):
        super().__init__(parent)
        self.canvas=canvas
        self.controls={}
        self._loading=False
        self.setWidgetResizable(True)
        page=QtWidgets.QWidget();layout=QtWidgets.QVBoxLayout(page);self.setWidget(page)
        note=QtWidgets.QLabel('手指或鼠标拖动整片图标；松手先惯性滑行，再将最近的应用吸附到中心。\n传感器使用前，请在触摸板页启用并采集松手参考。')
        note.setWordWrap(True);layout.addWidget(note)
        reference=QtWidgets.QPushButton('前往触摸板采集参考');reference.clicked.connect(lambda: self.reference_requested.emit());layout.addWidget(reference)
        groups=(('滑动与方向',(('sensor_enabled','传感器控制表盘'),('drag_speed','滑动速度（像素/格）'),('reverse_x','反转水平滑动'),('reverse_y','反转垂直滑动'),('tap_threshold','点击容差（像素）'))),
                ('惯性',(('inertia_enabled','启用惯性'),('inertia_strength','惯性强度（倍率）'),('inertia_decay_ms','衰减时间（毫秒）'),('inertia_max_distance','最大滑行距离（像素）'),('inertia_max_speed','最高滑行速度（像素/秒）'),('inertia_max_duration_ms','最长滑行时间（毫秒）'),('inertia_stop_speed','停止速度（像素/秒）'))),
                ('自动居中',(('snap_duration_ms','吸附动画时长（毫秒）'),)),
                ('中心高亮',(('center_glow_strength','边缘高亮强度（%）'),('inactive_opacity','非中心图标不透明度（%）'),('focus_transition_ms','高亮过渡时间（毫秒）'))),
                ('图标布局',(('spacing','中心间距（像素）'),('large_size','选中图标直径（像素）'),('small_size','未选中图标直径（像素）'),('size_transition_ms','选中放大过渡时间（毫秒）'),('show_numbers','显示应用编号'))))
        for title,fields in groups:
            group=QtWidgets.QGroupBox(title);form=QtWidgets.QFormLayout(group);layout.addWidget(group)
            for key,label in fields:
                default=DEFAULTS[key]
                if isinstance(default,bool):
                    control=QtWidgets.QCheckBox(label);control.setChecked(default);form.addRow(control)
                    control.toggled.connect(self._changed)
                else:
                    control=QtWidgets.QDoubleSpinBox() if isinstance(default,float) else QtWidgets.QSpinBox()
                    control.setRange(*LIMITS[key]);control.setValue(default);control.setKeyboardTracking(False)
                    if isinstance(default,float):control.setDecimals(2);control.setSingleStep(.05)
                    control.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
                    form.addRow(label,control);control.valueChanged.connect(self._changed)
                self.controls[key]=control
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        center=QtWidgets.QPushButton('回到应用001');row.addWidget(center)
        center.clicked.connect(self.center)
        reset=QtWidgets.QPushButton('恢复默认参数');row.addWidget(reset);reset.clicked.connect(self.reset_defaults)
        note=QtWidgets.QLabel('修改后自动保存，下次启动恢复。仅选中图标放大，其余统一为小尺寸；选中直径自动保持大于未选中直径，并限制在间距以内。惯性结束后再开始吸附计时。')
        note.setWordWrap(True);layout.addWidget(note);layout.addStretch(1)

    def settings(self):
        return normalize_settings({key:control.isChecked() if isinstance(control,QtWidgets.QCheckBox) else control.value()
                                   for key,control in self.controls.items()})

    def load_settings(self,values):
        values=normalize_settings(values)
        self._loading=True
        for key,control in self.controls.items():
            control.setChecked(values[key]) if isinstance(control,QtWidgets.QCheckBox) else control.setValue(values[key])
        self._loading=False
        self.canvas.configure(values)

    def _changed(self,*_):
        if self._loading:return
        self.load_settings(self.settings())
        self.settings_changed.emit()

    def center(self):
        self.canvas.cancel_gesture()
        self.canvas.model.snap(time.perf_counter(),0)
        self.canvas.update()

    def reset_defaults(self):
        self.load_settings(DEFAULTS)
        self.settings_changed.emit()
