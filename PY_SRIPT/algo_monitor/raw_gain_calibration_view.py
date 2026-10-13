"""右侧100g逐通道标定页，人工确认放置位置，保存原始增量及独立验证。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
from uuid import uuid4

from qt_compat import QtCore, QtWidgets, Signal
from raw_gain_calibration import CalibrationSettings, RawGainCalibration


class RawGainCalibrationPanel(QtWidgets.QScrollArea):
    settings_changed = Signal()

    def __init__(self, apply_gains, get_sample, get_context, parent=None, output_dir=None):
        super().__init__(parent)
        self.apply_gains = apply_gains
        self.get_sample = get_sample
        self.get_context = get_context
        self.output_dir = Path(output_dir) if output_dir else Path(__file__).with_name('raw_gain_calibrations')
        self.engine = RawGainCalibration()
        self.context = None
        self.record_context = None
        self.report_path = None
        self.loaded_path = None
        self.restored_from = None
        self._needs_environment_check = False
        self.applied = None
        self._loading = False
        self._pending_apply = False
        self.setWidgetResizable(True)
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page); self.setWidget(page)
        note = QtWidgets.QLabel('使用同一个砝码、相同接触面逐点放置，不滑动、不额外用手压。每次先完全移开砝码采集空载，再放在指定通道中心采集按压；质量和次数以下面的设置为准。\n'
                               '使用原始Raw−本次空载，排除已有增益和动态基线影响。砝码重量为人工声明，不是实测力值。')
        note.setWordWrap(True);layout.addWidget(note)
        group=QtWidgets.QGroupBox('采集设置'); form=QtWidgets.QFormLayout(group);layout.addWidget(group)
        self.inputs={}
        for name,label,low,high,step,decimals in (
            ('mass_g','砝码总质量（g）',1,10000,10,1),
            ('repeats','每通道标定次数',3,20,1,0),
            ('target','目标变化量（count，0＝自动取20通道中位数）',0,1000000,100,1),
            ('tolerance','空载稳定容差（count）',1,10000,10,1),
            ('settle_seconds','开始采集后的等待时间（秒）',.2,10,.1,1),
            ('window_seconds','稳定采样窗口（秒）',.3,5,.1,1),
            ('max_cv_percent','重复响应CV上限（%）',1,50,1,1)):
            w=QtWidgets.QSpinBox() if decimals==0 else QtWidgets.QDoubleSpinBox()
            if decimals:w.setDecimals(decimals)
            w.setRange(low,high);w.setSingleStep(step);w.setValue(getattr(self.engine.settings,name))
            w.setKeyboardTracking(False);w.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
            w.valueChanged.connect(self._settings_changed);self.inputs[name]=w;form.addRow(label,w)
        self.new_button=QtWidgets.QPushButton('新建标定（已采集记录保留在文件中）')
        self.new_button.clicked.connect(self.new_session);layout.addWidget(self.new_button)
        history_row=QtWidgets.QHBoxLayout()
        self.load_button=QtWidgets.QPushButton('读取标定记录…')
        self.latest_button=QtWidgets.QPushButton('读取最近一次记录')
        self.load_button.clicked.connect(self.choose_report)
        self.latest_button.clicked.connect(lambda:self.restore_last_record())
        history_row.addWidget(self.load_button);history_row.addWidget(self.latest_button);layout.addLayout(history_row)
        self.confirm_environment_button=QtWidgets.QPushButton('确认同一阵列且三参数未变，继续使用记录')
        self.confirm_environment_button.clicked.connect(self.confirm_environment);layout.addWidget(self.confirm_environment_button)
        self.environment_status=QtWidgets.QLabel('已应用的增益会随应用配置恢复；历史采集记录可读取后继续验证或重测。')
        self.environment_status.setWordWrap(True);layout.addWidget(self.environment_status)
        self.mode=QtWidgets.QComboBox();self.mode.addItem('逐点标定','calibrate');self.mode.addItem('独立验证（重新按压，不参与增益计算）','validate')
        self.mode.currentIndexChanged.connect(self._selection_changed);layout.addWidget(self.mode)
        grid=QtWidgets.QGridLayout();self.channel_buttons=[]
        for ch in range(20):
            b=QtWidgets.QPushButton(f'CH{ch}');b.setCheckable(True)
            b.clicked.connect(lambda checked=False,c=ch:self.select_channel(c))
            grid.addWidget(b,ch%4,ch//4);self.channel_buttons.append(b)
        layout.addLayout(grid)
        self.status=QtWidgets.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
        row=QtWidgets.QHBoxLayout();self.empty_button=QtWidgets.QPushButton('已移开砝码：采集空载')
        self.loaded_button=QtWidgets.QPushButton('已放在点中心：采集按压')
        self.empty_button.clicked.connect(self.capture_empty);self.loaded_button.clicked.connect(self.capture_loaded)
        row.addWidget(self.empty_button);row.addWidget(self.loaded_button);layout.addLayout(row)
        self.cancel_button=QtWidgets.QPushButton('取消本次采集');self.cancel_button.clicked.connect(self.cancel_capture);layout.addWidget(self.cancel_button)
        self.undo_button=QtWidgets.QPushButton('撤销本通道最近一次（保留原记录）')
        self.undo_button.clicked.connect(self.undo_last);layout.addWidget(self.undo_button)
        self.table=QtWidgets.QTableWidget(20,7)
        self.table.setHorizontalHeaderLabels(['通道','次数','Δ中位数','CV %','增益','验证次数','验证误差 %'])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMinimumHeight(360)
        self.table.cellClicked.connect(lambda row,col:self.select_channel(row));layout.addWidget(self.table)
        self.summary=QtWidgets.QLabel();self.summary.setWordWrap(True);layout.addWidget(self.summary)
        self.apply_button=QtWidgets.QPushButton('已移开砝码：应用/保存20通道增益并重建参考')
        self.apply_button.clicked.connect(self.request_apply);layout.addWidget(self.apply_button)
        self.save_button=QtWidgets.QPushButton('保存/重试保存标定记录')
        self.save_button.clicked.connect(self.save_report);layout.addWidget(self.save_button)
        self.file_status=QtWidgets.QLabel('完成每次按压后自动保存记录。');self.file_status.setWordWrap(True);layout.addWidget(self.file_status)
        note=QtWidgets.QLabel('彩色选中点为当前通道。CV过大或响应不足应重测；目标通道不是最大响应通道时会提示核对放置位置。\n'
                             '独立验证误差＝新增按压响应×增益÷目标−1；修改标定后旧验证失效，需重新验证。\n'
                             '标定完成后仍需滑动验证；单点增益不能消除串扰、慢回弹或运动判定暂停。')
        note.setWordWrap(True);layout.addWidget(note);layout.addStretch(1)
        self.select_channel(0)

    def settings(self):
        return {name:w.value() for name,w in self.inputs.items()}

    def resume_path(self):
        return str(self.report_path or self.loaded_path or '')

    def choose_report(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'读取20通道标定记录',str(self.output_dir),'标定记录 (*.json)')
        if path:self.load_report(path)

    def restore_last_record(self, path=None):
        """首次升级找最近记录；以后按保存的项目路径恢复，不跳到别的项目。"""
        if path=='':return False
        try:
            if path is None:
                files=list(self.output_dir.glob('raw_gain_*.json'))
                if not files:return False
                path=max(files,key=lambda p:p.stat().st_mtime_ns)
            return self.load_report(path)
        except OSError as exc:
            self.file_status.setText(f'历史记录读取失败：{exc}；当前采集记录和运行增益未改变。')
            return False

    def load_report(self, path):
        """先完整校验再替换界面；继续采集时另存副本，保留来源文件。"""
        if self.engine.capturing or self.engine.phase=='await_load':
            self.file_status.setText('请先取消当前采集，再读取历史记录。');return False
        try:
            path=Path(path).resolve()
            if path.stat().st_size>20*1024*1024:raise ValueError('标定文件超过20 MiB')
            data=path.read_bytes()
            def reject_constant(value):raise ValueError(f'记录含非有限数值：{value}')
            report=json.loads(data.decode('utf-8-sig'),parse_constant=reject_constant)
            model=RawGainCalibration.from_report(report)
            context=report.get('context')
            if context is not None and not isinstance(context,dict):raise ValueError('记录设备环境格式无效')
            selection=report.get('selection',{})
            if not isinstance(selection,dict):raise ValueError('记录所选通道无效')
            channel=selection.get('channel',0)
            if type(channel) is not int or channel not in range(20):raise ValueError('记录所选通道无效')
            mode=selection.get('mode','validate' if model.results()['gains'] else 'calibrate')
            if mode not in ('calibrate','validate'):raise ValueError('记录采集模式无效')
        except (OSError,ValueError,UnicodeError,TypeError,OverflowError) as exc:
            self.file_status.setText(f'历史记录读取失败：{exc}；当前采集记录和运行增益未改变。');return False
        if not self.save_report():return False
        # 若重新读取当前工作副本，使用刚保存的最新数据，不能回退到保存前的内容。
        if self.report_path is not None and self.report_path.resolve()==path:
            try:
                data=path.read_bytes();report=json.loads(data.decode('utf-8-sig'),parse_constant=reject_constant)
                model=RawGainCalibration.from_report(report);context=report.get('context')
                channel=report.get('selection',{}).get('channel',0)
                mode=report.get('selection',{}).get('mode','calibrate')
            except (OSError,ValueError,UnicodeError,TypeError) as exc:
                self.file_status.setText(f'重新读取工作副本失败：{exc}；当前窗口的数据仍保留。');return False
        self.engine=model;self.context=None;self.record_context=context
        self.report_path=None;self.loaded_path=path;self.applied=report.get('applied')
        self.restored_from=dict(path=str(path),sha256=hashlib.sha256(data).hexdigest(),
                                saved_at=report.get('saved_at'),invalid_reason=report.get('invalid_reason',''),context=context)
        self._pending_apply=False;self._needs_environment_check=True
        self.load_settings(report['settings'])
        self.mode.blockSignals(True);self.mode.setCurrentIndex(1 if mode=='validate' else 0);self.mode.blockSignals(False)
        self.select_channel(channel)
        self.file_status.setText(f'已读取：{path}\n历史记录读取不会替换当前运行增益；继续保存将生成副本，保留原文件。')
        self._check_history_environment()
        self.settings_changed.emit();self.refresh();return True

    @staticmethod
    def _canonical(value):
        return json.dumps(value,sort_keys=True,ensure_ascii=False)

    def _environment_check(self, current):
        """仅完整环境可自动匹配；缺失历史信息须人工确认，已知差异不能覆盖。"""
        old=self.record_context or {}
        missing=False
        def value(context,key):
            result=context.get(key)
            if key=='hardware_signature' and result:
                if (not isinstance(result,(list,tuple)) or len(result)!=20
                        or any(not isinstance(row,(list,tuple)) or len(row)!=4
                               or any(type(v) is not int for v in row) for row in result)
                        or {row[0] for row in result}!=set(range(20))):return None
            return result
        for key,label in (('port','串口'),('device_info','设备信息'),('hardware_signature','三参数')):
            before,after=value(old,key),value(current,key)
            if before in (None,[],{},''):
                missing=True;continue
            if after in (None,[],{},''):return 'waiting',f'等待当前{label}回读，历史数据已保留。'
            if self._canonical(before)!=self._canonical(after):
                if key=='port':missing=True;continue
                return 'mismatch',f'{label}与历史记录不一致；请使用匹配的设备环境或新建标定。'
        if value(old,'hardware_signature') and old.get('raw_limits') is not None and self._canonical(old['raw_limits'])!=self._canonical(current.get('raw_limits')):
            return 'mismatch','Raw上限与历史记录不一致，请新建标定。'
        if missing:return 'confirm','历史设备信息不完整或串口已变化；请确认仍是同一阵列，且Nsub、TxDivider、RefCDAC未变。'
        return 'match','设备与三参数已核对一致；每次按压及应用仍需重新采集空载。'

    def _current_environment(self, now):
        sample=self.get_sample();context=self.get_context()
        if (not sample or not sample.get('has_raw') or sample.get('channel_count')!=20
                or not 0<=now-sample['sample_timestamp']<=.2 or context.get('connected') is False):
            raise ValueError('请连接传感器并等待新的20通道Debug Raw；历史记录可先查看。')
        if context.get('parameters_pending'):raise ValueError('三参数正在同步，请等待完成。')
        return sample,context

    def _bind_environment(self, context, message):
        self.context=context;self.record_context=context;self._needs_environment_check=False
        self.engine.invalid_reason='';self.engine.cancel(message)
        self.environment_status.setText(message)

    def _check_history_environment(self, now=None):
        if not self._needs_environment_check:return
        import time
        try:
            _,context=self._current_environment(time.perf_counter() if now is None else now)
            status,message=self._environment_check(context)
            if status=='match':self._bind_environment(context,message)
            else:
                self.environment_status.setText(message);self.engine.message=message
        except (ValueError,KeyError,TypeError) as exc:
            self.environment_status.setText(str(exc));self.engine.message=str(exc)

    def confirm_environment(self):
        import time
        if not self._needs_environment_check:return
        try:
            _,context=self._current_environment(time.perf_counter())
            status,message=self._environment_check(context)
            if status not in ('confirm','match'):raise ValueError(message)
            self._bind_environment(context,'已人工确认同一阵列及三参数；历史数据保留，请重新采集空载后继续。')
            self.save_report()
        except (ValueError,KeyError,TypeError) as exc:self.environment_status.setText(str(exc))
        self.refresh()

    def load_settings(self, values):
        self._loading=True
        try:
            settings=CalibrationSettings(**{k:v for k,v in values.items() if k in self.inputs})
        except (TypeError,ValueError,AttributeError):settings=CalibrationSettings()
        for name,w in self.inputs.items():w.setValue(getattr(settings,name))
        self.engine.settings=settings;self._loading=False;self.refresh()

    def _settings_changed(self):
        if self._loading:return
        self.engine.settings=CalibrationSettings(**self.settings())
        if any(self.engine.trials):self.save_report()
        self.settings_changed.emit();self.refresh()

    def _selection_changed(self):
        self.cancel_capture();self.refresh()

    def select_channel(self, channel):
        if self.engine.channel!=channel:self.cancel_capture()
        self.engine.channel=channel
        for ch,b in enumerate(self.channel_buttons):
            b.setChecked(ch==channel);b.setStyleSheet('background: #246e87; color: white;' if ch==channel else '')
        self.refresh()

    def new_session(self):
        if any(self.engine.trials) or any(self.engine.validations) or self.engine.discarded:
            if not self.save_report():return
        self._pending_apply=False
        self.engine=RawGainCalibration(CalibrationSettings(**self.settings()))
        self.context=None;self.record_context=None;self.report_path=None;self.loaded_path=None;self.restored_from=None
        self._needs_environment_check=False;self.applied=None;self.mode.setCurrentIndex(0);self.select_channel(0)
        self.file_status.setText('新标定已建立，首次完成按压后自动保存。')
        self.environment_status.setText('新标定；首次采集时记录当前设备与三参数。')
        self.settings_changed.emit()
        self.refresh()

    def _fresh_sample(self, now):
        self._check_history_environment(now)
        if self._needs_environment_check:raise ValueError(self.environment_status.text())
        sample,context=self._current_environment(now)
        if self.context is None:self.context=self.record_context=context
        elif context!=self.context:
            self.engine.invalidate('连接或硬件参数已改变');raise ValueError('采集环境已改变，请新建标定。')
        self.engine.raw_limits=tuple(context.get('raw_limits') or (65535,)*20)
        self.engine.last_sequence=sample['sequence'];self.engine.last_timestamp=sample['sample_timestamp']

    def capture_empty(self):
        import time
        try:
            now=time.perf_counter();self._fresh_sample(now)
            self._pending_apply=False;self.engine.start_empty(self.engine.channel,now,self.mode.currentData())
        except ValueError as exc:self.engine.message=str(exc)
        self.refresh()

    def capture_loaded(self):
        import time
        try:self.engine.start_loaded(time.perf_counter())
        except ValueError as exc:self.engine.message=str(exc)
        self.refresh()

    def request_apply(self):
        import time
        try:
            now=time.perf_counter();self._fresh_sample(now)
            self.engine.start_empty(self.engine.channel,now,'apply');self._pending_apply=True
        except ValueError as exc:self.engine.message=str(exc)
        self.refresh()

    def cancel_capture(self):
        self._pending_apply=False;self.engine.cancel();self.refresh()

    def undo_last(self):
        if not self.save_report():return
        self._pending_apply=False;self.engine.undo_last(self.engine.channel,self.mode.currentData())
        self.save_report();self.refresh()

    def invalidate(self, reason):
        self._pending_apply=False
        if any(self.engine.trials) or any(self.engine.validations):
            if self.record_context is None:self.record_context=self.context
            self.context=None;self._needs_environment_check=True
            self.engine.invalidate(reason)
            self.engine.message=reason+'；已采集记录保留，连接后重新核对环境。'
            self.save_report()
        else:self.engine.cancel(reason)
        self.refresh()

    def process_samples(self, samples, now):
        self._check_history_environment(now)
        if self.context is not None and self.context!=self.get_context():
            self.invalidate('连接或硬件参数已改变')
            self.context=None
        changed=False
        for sample in samples:
            stamp=sample.get('sample_timestamp')
            if self.engine.capturing and isinstance(stamp,(int,float)) and now-stamp>self.engine.GAP_SECONDS:
                self._pending_apply=False
                self.engine.cancel('采集帧已过期，本次作废；请重新采集空载。')
            event=self.engine.feed(sample)
            if event=='trial_ready':
                changed=True;self.save_report()
                result=self.engine.results();ch=self.engine.channel
                if self.mode.currentData()=='calibrate' and result['channels'][ch]['qualified']:
                    next_ch=next((i for i in range(ch+1,20) if not result['channels'][i]['qualified']),None)
                    if next_ch is not None:self.select_channel(next_ch)
            elif event=='empty_ready' and self._pending_apply:
                self._pending_apply=False
                gains=self.engine.results()['gains']
                empty_capture=self.engine.empty_capture
                ok,message=self.apply_gains(gains)
                self.engine.cancel(message)
                if ok:
                    self.applied=dict(timestamp=datetime.now().astimezone().isoformat(),
                                      calibration_revision=self.engine.revision,gains=gains,unloaded=empty_capture)
                    self.save_report()
                changed=True
        self.engine.expire(now)
        if not self.engine.capturing:self._pending_apply=False
        self.refresh(update_table=changed)

    def save_report(self):
        if not any(self.engine.trials) and not any(self.engine.validations) and not self.engine.discarded:return True
        try:
            self.output_dir.mkdir(parents=True,exist_ok=True)
            report_path=self.report_path
            if report_path is None:
                stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                report_path=self.output_dir/f'raw_gain_{stamp}_{uuid4().hex[:8]}.json'
            report=self.engine.report()
            report.update(saved_at=datetime.now().astimezone().isoformat(),context=self.record_context or self.context,
                          applied=self.applied,restored_from=self.restored_from,
                          selection=dict(channel=self.engine.channel,mode=self.mode.currentData()))
            temporary=report_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
            temporary.replace(report_path)
            self.report_path=report_path
            self.file_status.setText(f'记录已保存：{self.report_path}')
            self.settings_changed.emit()
            return True
        except (OSError,ValueError) as exc:
            self.file_status.setText(f'记录保存失败：{exc}；数据仍保留在当前窗口，请重试保存。')
            return False

    def refresh(self, update_table=True):
        e=self.engine;r=e.results();ch=e.channel
        count=len(e.trials[ch]) if self.mode.currentData()=='calibrate' else r['channels'][ch]['validation_count']
        self.status.setText(f'当前 CH{ch} · {self.mode.currentText()} · 已采集 {count} 次\n{e.message}')
        capturing=e.capturing
        for name,w in self.inputs.items():w.setEnabled(not capturing and (name=='target' or not any(e.trials)))
        self.mode.setEnabled(not capturing)
        for b in self.channel_buttons:b.setEnabled(not capturing)
        self.new_button.setEnabled(not capturing)
        self.load_button.setEnabled(not capturing and e.phase!='await_load')
        self.latest_button.setEnabled(not capturing and e.phase!='await_load')
        self.confirm_environment_button.setVisible(self._needs_environment_check)
        try:
            import time
            _,current=self._current_environment(time.perf_counter())
            can_confirm=self._environment_check(current)[0] in ('confirm','match')
        except (ValueError,KeyError,TypeError):can_confirm=False
        self.confirm_environment_button.setEnabled(self._needs_environment_check and can_confirm and not capturing)
        self.empty_button.setEnabled(not capturing and not e.invalid_reason)
        self.loaded_button.setEnabled(e.phase=='await_load' and not e.invalid_reason)
        self.cancel_button.setEnabled(capturing or e.phase=='await_load')
        source=e.validations if self.mode.currentData()=='validate' else e.trials
        self.undo_button.setEnabled(not capturing and bool(source[ch]))
        self.apply_button.setEnabled(r['ready'] and not capturing)
        if update_table:
            for c in r['channels']:
                vals=[f"CH{c['channel']}",str(c['count']),self._number(c['response'],1),self._number(c['cv_percent'],1),
                      self._number(c['gain'],4),str(c['validation_count']),self._number(c['validation_error_percent'],1)]
                for col,value in enumerate(vals):self.table.setItem(c['channel'],col,QtWidgets.QTableWidgetItem(value))
                warning=''
                if e.trials[c['channel']] and e.trials[c['channel']][-1]['dominant_channel']!=c['channel']:
                    warning=f"最近一次最大响应为CH{e.trials[c['channel']][-1]['dominant_channel']}，请核对砝码位置。"
                self.table.item(c['channel'],0).setToolTip(warning)
            qualified=sum(c['qualified'] for c in r['channels'])
            questionable=[f"CH{i}→CH{trials[-1]['dominant_channel']}" for i,trials in enumerate(e.trials)
                          if trials and trials[-1]['dominant_channel']!=i]
            self.summary.setText(f"合格通道 {qualified}/20 · 目标变化量 {self._number(r['target'],1)} count\n"+
                                 ('20通道可应用；先移开砝码，再点击应用按钮。' if r['ready'] else
                                  r['error'] or e.invalid_reason or '需每通道达到次数要求，且CV不超过设置上限；响应不足时不能用放大增益代替重测。')+
                                 ('\n位置待核对（所选→最大响应）：'+', '.join(questionable) if questionable else ''))

    @staticmethod
    def _number(value, decimals):
        return '--' if value is None else f'{value:.{decimals}f}'
