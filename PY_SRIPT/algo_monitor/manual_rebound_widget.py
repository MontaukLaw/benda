"""手动施压、释放及逐通道回弹观察页；复用现有电机互斥与安全校验。"""

import json
import math
import os
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

from qt_compat import QtCore, QtWidgets
from wrapped_label import WrappedLabel
from channel_states import STATE_NAMES
from force_press_control_widget import TARGET_FORCES_N
from manual_rebound_observation import ReboundObservation, CHANNELS, MAX_SAMPLE_AGE, sensor_values


class ManualReboundWidget(QtWidgets.QWidget):
    def __init__(self, force_controller, sensor_sample_provider, parent=None, output_dir=None):
        super().__init__(parent)
        self.setStyleSheet('QPushButton:disabled { background:#59636d; color:#aab9c4; }')
        self.controller = force_controller
        self.sample_provider = sensor_sample_provider
        self._armed = None
        self._press_operation = self._release_operation = None
        self._closed = False
        self.log_path = None
        self._log_file = None
        root = Path(os.environ.get('LOCALAPPDATA', Path.home()))/'AlgoMonitor' if getattr(sys,'frozen',False) else Path(__file__).parent
        self.output_dir = Path(output_dir) if output_dir is not None else root/'manual_rebound_observations'
        self.observation = ReboundObservation(self._append_log)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(6,6,6,6)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(6,6,6,6)
        note = WrappedLabel('手动选择压力，保持所需时间后点击释放。目标压力沿用当前查表定位，实测力可能不同。'
                            '施压时长从压力计首次超过0.05 N起算；回弹时间从释放发令起算。')
        note.setStyleSheet('color:#8bd5ff;')
        layout.addWidget(note)
        group = QtWidgets.QGroupBox('目标压力')
        grid = QtWidgets.QGridLayout(group)
        self.target_buttons = {}
        for n,value in enumerate(TARGET_FORCES_N):
            button = QtWidgets.QPushButton(f'{value:.2f} N')
            button.setObjectName('ForceTargetButton')
            button.setMinimumHeight(34)
            button.clicked.connect(lambda checked=False,v=value: self._press(v))
            grid.addWidget(button,n//5,n%5)
            self.target_buttons[value] = button
        layout.addWidget(group)
        self.lookup_label = WrappedLabel('读取压力－位置查找表…')
        layout.addWidget(self.lookup_label)
        actions = QtWidgets.QHBoxLayout()
        self.release_button = QtWidgets.QPushButton('释放 / 回到标定出发点')
        self.release_button.setMinimumHeight(38)
        self.release_button.clicked.connect(self._release)
        self.stop_button = QtWidgets.QPushButton('紧急停止电机')
        self.stop_button.clicked.connect(self._stop_motor)
        self.stop_button.setStyleSheet('background:#a83232;color:white;font-weight:bold;')
        actions.addWidget(self.release_button,1)
        actions.addWidget(self.stop_button)
        layout.addLayout(actions)
        options = QtWidgets.QHBoxLayout()
        options.addWidget(QtWidgets.QLabel('Raw返回容差 ±'))
        self.tolerance_input = QtWidgets.QSpinBox()
        self.tolerance_input.setRange(0,100)
        self.tolerance_input.setValue(2)
        options.addWidget(self.tolerance_input)
        options.addWidget(QtWidgets.QLabel('counts，连续确认（s）'))
        self.stable_input = QtWidgets.QDoubleSpinBox()
        self.stable_input.setRange(.1,10)
        self.stable_input.setSingleStep(.1)
        self.stable_input.setValue(.5)
        options.addWidget(self.stable_input)
        options.addStretch(1)
        layout.addLayout(options)
        self.force_label = QtWidgets.QLabel('目标 -- N ｜ 实测 -- N')
        self.force_label.setStyleSheet('font-size:18px;font-weight:bold;color:#8bd5ff;')
        self.force_label.setWordWrap(True)
        self.timer_label = WrappedLabel('等待手动施压')
        self.timer_label.setStyleSheet('font-size:16px;')
        layout.addWidget(self.force_label)
        layout.addWidget(self.timer_label)
        self.table = QtWidgets.QTableWidget(4,7)
        self.table.setHorizontalHeaderLabels(['通道','真实状态','冻结基线','Raw','距原基线','回IDLE / s','Raw返回 / s'])
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.table.setMinimumWidth(0)
        self.table.setMinimumHeight(170)
        self.table.setMaximumHeight(180)
        for row,c in enumerate(CHANNELS):
            for col in range(7):
                self.table.setItem(row,col,QtWidgets.QTableWidgetItem(f'CH{c}' if col==0 else '--'))
        layout.addWidget(self.table)
        explanation = WrappedLabel('两个终点分别记录：①固件首次进入IDLE；②Raw回到本轮冻结基线附近。'
                                   'IDLE时基线可能已移动，Raw也可能始终回不到原值。未返回会持续计时；再次施压或结束观察时保留未完成结果。')
        layout.addWidget(explanation)
        self.status_label = WrappedLabel('')
        layout.addWidget(self.status_label)
        log_actions = QtWidgets.QHBoxLayout()
        log_actions.addWidget(QtWidgets.QLabel('观察日志（时间单位：秒）'),1)
        self.finish_button = QtWidgets.QPushButton('结束本轮观察')
        self.finish_button.clicked.connect(lambda: self.observation.interrupt(time.monotonic(),'用户结束本轮观察'))
        log_actions.addWidget(self.finish_button)
        self.save_button = QtWidgets.QPushButton('另存日志')
        self.save_button.clicked.connect(self.save_log)
        log_actions.addWidget(self.save_button)
        layout.addLayout(log_actions)
        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMinimumHeight(190)
        self.log_edit.document().setMaximumBlockCount(5000)
        layout.addWidget(self.log_edit,1)
        self.path_label = WrappedLabel('每轮时间和结果会自动保存到观察日志。')
        layout.addWidget(self.path_label)
        scroll.setWidget(content)
        outer.addWidget(scroll)
        self.controller.manual_observation_event.connect(self._on_event)
        self.controller.status_message.connect(self.status_label.setText)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._tick)
        self.timer.start()
        self._tick()

    def _append_log(self, entry):
        at = float(entry.get('at',time.monotonic()))
        stamp = datetime.now().astimezone()-timedelta(seconds=max(0,time.monotonic()-at))
        record = dict(entry,timestamp=stamp.isoformat(timespec='milliseconds'))
        text = f'[{stamp.strftime("%H:%M:%S.%f")[:-3]}] 第{entry.get("trial",0)}轮 {entry["message"]}'
        self.log_edit.appendPlainText(text)
        try:
            if self._log_file is None:
                self.output_dir.mkdir(parents=True,exist_ok=True)
                self.log_path = self.output_dir/f'manual_rebound_{stamp.strftime("%Y%m%d_%H%M%S_%f")}.jsonl'
                self._log_file = self.log_path.open('x',encoding='utf-8')
                self.path_label.setText(f'自动保存：{self.log_path}')
            self._log_file.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
            self._log_file.flush()
        except (OSError,ValueError) as exc:
            self.path_label.setText(f'自动保存失败：{exc}；可另存当前屏幕日志。')

    def _fresh_sample(self):
        sample = self.sample_provider() if callable(self.sample_provider) else None
        if sensor_values(sample) is None or float(sample.get('age_seconds',math.inf)) > MAX_SAMPLE_AGE:
            return None
        return sample

    def _press(self, target):
        if self._fresh_sample() is None:
            self.status_label.setText('需要新鲜的CH4～CH7 Debug数据才能开始观察。')
            return
        self._armed = ('press',target,self.tolerance_input.value(),self.stable_input.value())
        if not self.controller.start_press(target):
            self._armed = None
            self.status_label.setText(self.controller.manual_control_state()['status'])

    def _release(self):
        if self.observation.phase in ('pressing','holding'):
            self._armed = ('release',)
        if not self.controller.start_release():
            self._armed = None
            self.status_label.setText(self.controller.manual_control_state()['status'])

    def _stop_motor(self):
        self.controller.request_stop()
        self.interrupt('用户紧急停止电机，观察中断')

    def _on_event(self, event):
        if self._closed:
            return
        at,mode = event['at'],event.get('mode')
        op = event.get('operation_id')
        if event['event']=='motion_dispatched':
            if mode=='press' and self._armed and self._armed[0]=='press':
                _,target,tolerance,stable = self._armed
                self._armed = None
                sample = event.get('sensor')
                if sensor_values(sample) is None or float(sample.get('age_seconds',math.inf))>MAX_SAMPLE_AGE:
                    self.interrupt('下压发令时传感器数据无效，无法开始回弹计时')
                    self.status_label.setText('电机已接受下压，但数据无效，未建立观察记录。')
                    return
                self._press_operation = op
                self._release_operation = None
                self.observation.begin(at,target,sample,tolerance,stable)
            elif mode=='release' and self.observation.phase in ('pressing','holding'):
                self._armed = None
                self._release_operation = op
                p = event.get('pressure') or {}
                force = p.get('accepted_force_n') if p.get('accepted_age_seconds',math.inf)<=1 else None
                self.observation.release(at,event.get('sensor'),force)
            elif self.observation.active and op not in (self._press_operation,self._release_operation):
                self.interrupt('其他电机操作改变了本轮条件，观察中断')
        elif event['event']=='operation_complete':
            result = event.get('result',{})
            if op==self._press_operation:
                if result.get('ok'):
                    self.observation.arrived(at,result.get('final_force_n'))
                else:
                    self.interrupt('下压未完成：'+str(result.get('message','未知错误')))
            elif op==self._release_operation:
                if result.get('ok'):
                    self.observation.log(at,'release_arrived','电机已回到释放位置；继续观察通道自然恢复。')
                else:
                    self.interrupt('释放未完成：'+str(result.get('message','未知错误')))
            if self._armed and mode==self._armed[0]:
                self._armed = None
                self.interrupt('本次操作未产生已确认的运动：'+str(result.get('message','')))

    def interrupt(self, reason):
        self._armed = None
        self.observation.interrupt(time.monotonic(),reason)

    def _tick(self):
        now = time.monotonic()
        state = self.controller.manual_control_state()
        for value,button in self.target_buttons.items():
            button.setEnabled(bool(state['targets'].get(value)) and self._armed is None)
        self.release_button.setEnabled(bool(state['release']) and self._armed is None)
        self.stop_button.setEnabled(bool(state['stop']))
        self.lookup_label.setText(state['lookup'])
        p = self.controller._ocr_snapshot()
        force = p.get('accepted_force_n') if p.get('accepted_age_seconds',math.inf)<=1 else None
        o = self.observation
        if force is not None:
            o.pressure(float(p.get('accepted_at',now)),float(force))
        sample = self.sample_provider() if callable(self.sample_provider) else None
        o.sample(now,sample)
        target = '--' if o.target is None else f'{o.target:.2f}'
        force_text = '--' if force is None else f'{force:.2f}'
        self.force_label.setText(f'目标 {target} N ｜ 压力计实测 {force_text} N')
        if o.phase in ('pressing','holding'):
            contact = '等待接触' if o.contact_at is None else f'接触持续 {now-o.contact_at:.2f} s'
            hold = '尚未到位' if o.arrival_at is None else f'到位保持 {now-o.arrival_at:.2f} s'
            self.timer_label.setText(f'第{o.trial}轮：下压发令后 {now-o.press_at:.2f} s ｜ {contact} ｜ {hold}')
        elif o.phase=='released':
            self.timer_label.setText(f'第{o.trial}轮：释放后 {now-o.release_at:.2f} s，持续观察中')
        elif o.phase in ('complete','interrupted'):
            self.timer_label.setText(f'第{o.trial}轮：'+('观察完成' if o.phase=='complete' else '观察已结束，未完成结果保留'))
        values = sensor_values(sample) if sample and sample.get('age_seconds',math.inf)<=MAX_SAMPLE_AGE else None
        for i,c in enumerate(CHANNELS):
            row = o.channels.get(c,{})
            raw = values[c][0] if values else None
            reference = row.get('reference')
            contents = [f'CH{c}',STATE_NAMES.get(values[c][2],'未知') if values else '无新数据',
                        '--' if reference is None else f'{reference:.1f}', '--' if raw is None else f'{raw:.1f}',
                        '--' if raw is None or reference is None else f'{raw-reference:+.1f}']
            for key in ('idle_s','raw_s'):
                seconds = row.get(key)
                contents.append(f'{seconds:.3f}' if seconds is not None else ('未返回' if o.release_at is not None else '--'))
            for col,text in enumerate(contents):
                self.table.item(i,col).setText(text)
                self.table.item(i,col).setToolTip(text)
        self.tolerance_input.setEnabled(not o.active and self._armed is None)
        self.stable_input.setEnabled(not o.active and self._armed is None)
        self.finish_button.setEnabled(o.phase=='released')
        self.save_button.setEnabled(not self.log_edit.document().isEmpty())

    def save_log(self):
        filename,_ = QtWidgets.QFileDialog.getSaveFileName(self,'保存手动回弹观察日志','manual_rebound.txt','文本日志 (*.txt)')
        if filename:
            try:
                Path(filename).write_text(self.log_edit.toPlainText(),encoding='utf-8')
            except OSError as exc:
                self.status_label.setText(f'日志保存失败：{exc}')

    def shutdown(self):
        self.timer.stop()
        self.interrupt('窗口关闭，观察结束')
        self._closed = True
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
