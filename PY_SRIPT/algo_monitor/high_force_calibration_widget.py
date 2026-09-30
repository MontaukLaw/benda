"""监控页1～30 N自动标定入口，复用唯一电机线程和现有K表页。"""
from datetime import datetime
from copy import deepcopy
from pathlib import Path
import json
import math
import threading
import time

from high_force_calibration import (HighForceCalibrationRunner, calibration_targets,
    DEFAULT_REST_SECONDS, MAX_REST_SECONDS, RestWaitControl, validate_rest_seconds, CombinedCancel, recover_release)
from high_force_calibration_quality import DEFAULT_ACCURACY_PERCENT, validate_accuracy, historical_focus
from pressure_feedback_control import FeedbackSettings, pressure_tolerance_n
from force_signal_pressure_calibration import (
    _write_json_atomic, save_signal_pressure_mapping_document, ForceSignalPressureCalibrationError)
from calibration_record_store import CalibrationRecordStore
from high_force_calibration_resume import (discover_resume, read_resume, learning_fingerprint,
                                           check_resume_context, validate_resume)
from qt_compat import QtCore, QtWidgets, Signal
from wrapped_label import WrappedLabel


def duration_text(seconds):
    minutes, seconds = divmod(round(seconds, 1), 60)
    return f'{int(minutes)}分{seconds:04.1f}秒'


class HighForceCalibrationWidget(QtWidgets.QGroupBox):
    progress = Signal(object)
    finished = Signal(object)
    status_message = Signal(str)

    def __init__(self, control, feedback, sensor_provider, k_widget, busy_provider=lambda: False,
                 parent=None, output_dir=None):
        super().__init__('自动压力标定（1～30 N）', parent)
        self.setObjectName('HighForceCalibration')
        self.setCheckable(True)
        self.setChecked(True)
        self.setStyleSheet('''
            QGroupBox#HighForceCalibration {
                border: 1px solid #365b6f;
                border-radius: 7px;
                margin-top: 7px;
                padding-top: 5px;
                font-weight: 700;
                background: #0b1119;
            }
            QGroupBox#HighForceCalibration::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #91dff9;
            }
            QPushButton#HighForceStart { background:#6fd5a5;color:#102019;font-weight:800; }
            QPushButton#HighForceStop { background:#edbbb6;color:#17212b;font-weight:700; }
            QPushButton#HighForceSkip { background:#f3cf68;color:#17212b;font-weight:700; }
            QGroupBox#HighForceCalibration QPushButton:disabled {
                background:#59636d;color:#aab2ba;
            }
        ''')
        self.control, self.feedback, self.sensor_provider, self.k_widget = control, feedback, sensor_provider, k_widget
        self.busy_provider = busy_provider
        self.output_dir = Path(output_dir) if output_dir else Path(__file__).with_name('force_press_tests')/'high_force_calibration'
        self._running = False
        self._result = None
        self._pending_record_document = None
        self._resume_candidate = None
        self._resume_scanned_at = None
        self._context = {}
        self._rest_control = RestWaitControl()
        self._rest_id = None
        self._stopping = False
        self._stop_requested = threading.Event()
        self._emergency_requested = threading.Event()
        self._config_path = self.output_dir/f'settings_{self.control.profile.namespace}.json'
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(9, 12, 9, 8)
        outer.setSpacing(7)
        self.content = QtWidgets.QWidget()
        outer.addWidget(self.content)
        layout = QtWidgets.QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(7)
        description = WrappedLabel('① 热身：30 N到位后恒力保持5秒，再采样确认，释放后休息10秒；至少5次，稳定后进入正式测试，最多15次。'
            '② 正式标定：按设定点数、轮数采集。③ 重点复测：历史/本次误差大的点各补测3次，再独立逐点验收，最多3批。'
            '验收通过才更新K表；1 N以下手动参数独立。')
        description.setStyleSheet('color:#8bd5ff;')
        layout.addWidget(description)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('标定点数'))
        self.point_count = QtWidgets.QComboBox()
        for value in (10, 20): self.point_count.addItem(f'{value} 点', value)
        row.addWidget(self.point_count)
        row.addWidget(QtWidgets.QLabel('轮数'))
        self.round_count = QtWidgets.QSpinBox()
        self.round_count.setRange(1, 100)
        self.round_count.setValue(5)
        row.addWidget(self.round_count)
        self.start_button = QtWidgets.QPushButton('开始自动标定')
        self.resume_button = QtWidgets.QPushButton('继续标定')
        self.resume_button.setObjectName('HighForceStart')
        self.resume_button.setToolTip('读取最近未完成记录，保留完成点；装夹、材料和三参数应与中断前一致。')
        self.stop_button = QtWidgets.QPushButton('停止并释放')
        self.start_button.setObjectName('HighForceStart')
        self.stop_button.setObjectName('HighForceStop')
        self.stop_button.setToolTip('停止后续按压，自动回到原释放位置后结束；通信异常时继续尝试回位。')
        row.addWidget(self.start_button)
        row.addWidget(self.resume_button)
        row.addWidget(self.stop_button)
        layout.addLayout(row)
        rest_row = QtWidgets.QHBoxLayout()
        rest_row.addWidget(QtWidgets.QLabel('点间休息（秒）'))
        self.rest_seconds = QtWidgets.QDoubleSpinBox()
        self.rest_seconds.setRange(0, MAX_REST_SECONDS)
        self.rest_seconds.setDecimals(1)
        self.rest_seconds.setValue(DEFAULT_REST_SECONDS)
        self.rest_seconds.setToolTip('默认30秒，0表示不额外休息；启动时固定本次设置，跨轮同样适用。')
        rest_row.addWidget(self.rest_seconds)
        rest_row.addWidget(QtWidgets.QLabel('验收误差 ±（%）'))
        self.accuracy_percent = QtWidgets.QDoubleSpinBox()
        self.accuracy_percent.setRange(.5, 10)
        self.accuracy_percent.setDecimals(1)
        self.accuracy_percent.setValue(DEFAULT_ACCURACY_PERCENT)
        self.accuracy_percent.setToolTip('按压力计实测值验收，低力点至少允许±0.05 N；与电机寻压容差独立。')
        rest_row.addWidget(self.accuracy_percent)
        self.skip_button = QtWidgets.QPushButton('跳过剩余等待')
        self.skip_button.setObjectName('HighForceSkip')
        self.skip_button.setToolTip('仅跳过正式标定/复测的当前休息；热身的10秒不可跳过。仍须确认卸载及通道恢复。')
        rest_row.addWidget(self.skip_button)
        self.emergency_button = QtWidgets.QPushButton('紧急停止')
        self.emergency_button.setToolTip('立即取消全部运动及自动回位；用于必须停止电机的紧急情况。')
        self.emergency_button.setStyleSheet('background:#b94141;color:white;font-weight:700;')
        rest_row.addWidget(self.emergency_button)
        rest_row.addStretch(1)
        layout.addLayout(rest_row)
        self.plan_label = WrappedLabel()
        self.status_label = WrappedLabel('尚未开始')
        layout.addWidget(self.plan_label)
        self.resume_label = WrappedLabel()
        self.resume_label.setStyleSheet('color:#8bd5ff;')
        layout.addWidget(self.resume_label)
        layout.addWidget(self.status_label)
        self.stage_label = WrappedLabel('阶段：尚未开始')
        self.stage_label.setStyleSheet('color:#ffda72;font-weight:700;')
        layout.addWidget(self.stage_label)
        self.sample_result_panel = QtWidgets.QFrame()
        self.sample_result_panel.setObjectName('HighForceSampleResult')
        self.sample_result_panel.setStyleSheet('''
            QFrame#HighForceSampleResult {
                background:#10241f; border:2px solid #6fd5a5; border-radius:7px;
            }
        ''')
        sample_layout = QtWidgets.QVBoxLayout(self.sample_result_panel)
        sample_layout.setContentsMargins(12, 9, 12, 9)
        sample_layout.setSpacing(4)
        self.sample_result_title = WrappedLabel('最近一次采样 · 等待采集')
        self.sample_result_title.setStyleSheet('color:#bcebd9;font-size:13px;font-weight:700;')
        self.sample_result_values = WrappedLabel('目标压力 — N    实测压力 — N\n四通道 Signal 合计 — counts')
        self.sample_result_values.setStyleSheet('color:#ffdf79;font-size:24px;font-weight:800;')
        sample_layout.addWidget(self.sample_result_title)
        sample_layout.addWidget(self.sample_result_values)
        layout.addWidget(self.sample_result_panel)
        self.timing_table = QtWidgets.QTableWidget(0, 3)
        self.timing_table.setHorizontalHeaderLabels(['轮次', '耗时（含休息）', '完成情况'])
        self.timing_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.timing_table.setMaximumHeight(180)
        self.timing_table.setToolTip('每轮包含该轮各点按压前的实际休息，跨轮休息计入下一轮；跳过后按实际耗时记录，首轮另含启动检查。')
        self.timing_table.hide()
        layout.addWidget(self.timing_table)
        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['目标 N', '实测 N', '四路Signal和', '点K（N/count）'])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setMaximumHeight(240)
        self.table.hide()
        layout.addWidget(self.table)
        self.quality_table = QtWidgets.QTableWidget(0, 4)
        self.quality_table.setHorizontalHeaderLabels(['目标 N', '验收实测 N', '误差 / 允许± N', '验收结果'])
        self.quality_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.quality_table.setMaximumHeight(220)
        self.quality_table.hide()
        layout.addWidget(self.quality_table)
        self.point_count.currentIndexChanged.connect(self._plan)
        self.round_count.valueChanged.connect(self._plan)
        self._load_rest_setting()
        self.rest_seconds.valueChanged.connect(self._plan)
        self.rest_seconds.valueChanged.connect(self._save_rest_setting)
        self.accuracy_percent.valueChanged.connect(self._save_rest_setting)
        self.start_button.clicked.connect(self.start)
        self.resume_button.clicked.connect(self.resume)
        self.stop_button.clicked.connect(self.stop)
        self.skip_button.clicked.connect(self.skip_rest)
        self.emergency_button.clicked.connect(self.emergency_stop)
        self.progress.connect(self._progress)
        self.finished.connect(self._finished)
        self.control.worker_position_zeroed.connect(self._coordinates_changed)
        self.control.release_position_changed.connect(self._reference_changed)
        self.toggled.connect(self._set_expanded)
        self._set_expanded(self.isChecked())
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        self._plan()
        self._refresh()

    def is_running(self):
        return self._running

    def _load_rest_setting(self):
        try:
            data = json.loads(self._config_path.read_text(encoding='utf-8'))
            self.rest_seconds.setValue(validate_rest_seconds(data['rest_seconds']))
            self.accuracy_percent.setValue(validate_accuracy(data.get('accuracy_percent', DEFAULT_ACCURACY_PERCENT)))
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.status_label.setText(f'休息配置读取失败，使用默认30秒：{exc}')

    def _save_rest_setting(self, *_):
        try:
            _write_json_atomic(self._config_path, dict(rest_seconds=self.rest_seconds.value(),
                accuracy_percent=self.accuracy_percent.value()))
        except Exception as exc:
            self.status_label.setText(f'休息时间尚未保存：{exc}')

    def _set_expanded(self, expanded):
        self.content.setVisible(bool(expanded))
        self.setMinimumHeight(0)
        self.setMaximumHeight(16777215 if expanded else 32)
        self._refresh()

    def _plan(self, *_):
        points = calibration_targets(self.point_count.currentData())
        presses = len(points)*self.round_count.value()
        self.plan_label.setText('整数点位：'+ '、'.join(f'{p:g}' for p in points)+
                               f' N；共 {presses} 次按压，每次休息 {self.rest_seconds.value():g} 秒，'
                               f'计划休息合计 {duration_text((presses-1)*self.rest_seconds.value())}（未跳过时），另加寻压、采样及释放时间。'
                               '以上为正式轮内计划，另加热身、阶段间休息和针对性复测/验收。通过后同步至“传感器标定K值表”。')

    def _refresh(self):
        ready = (not self._running and not self.control.is_operation_busy() and not self.busy_provider()
                 and self.control.is_can_connected() and self.control.release_position_snapshot().get('confirmed')
                 and not self.feedback.learning_recovery_pending())
        self.start_button.setEnabled(bool(ready and self._pending_record_document is None))
        if not self._running and (self._resume_scanned_at is None or time.monotonic()-self._resume_scanned_at >= 3):
            self._resume_scanned_at = time.monotonic()
            self._resume_candidate = discover_resume(self.output_dir, self.control.profile.namespace)
            candidate = self._resume_candidate
            self.resume_label.setText(('可继续：'+candidate['summary']+
                '。装夹、材料和三参数须保持不变；恢复原任务设置。') if candidate else '没有可继续的未完成标定记录')
            self.resume_label.setToolTip(str(candidate['path']) if candidate else '')
        if not self._running and self._pending_record_document is not None:
            self.resume_label.setText('有尚未落盘的最新进度；请保留程序打开，点击“继续标定”先重试保存。')
        self.resume_button.setEnabled(bool(ready and (self._resume_candidate or self._pending_record_document is not None)))
        self.stop_button.setEnabled(self._running)
        self.emergency_button.setEnabled(self._running)
        self.point_count.setEnabled(not self._running)
        self.round_count.setEnabled(not self._running)
        self.rest_seconds.setEnabled(not self._running)
        self.accuracy_percent.setEnabled(not self._running)
        self.skip_button.setEnabled(self._running and not self._stopping and self._rest_control.can_skip(self._rest_id))

    def resume(self):
        self._resume_scanned_at = None
        self._refresh()
        if not self.resume_button.isEnabled():
            return
        try:
            if self._pending_record_document is not None:
                # 存储失败后停止时，内存比磁盘更新；不得用旧文件覆盖这些采样。
                pending = self._pending_record_document
                if not pending.get('complete'):
                    validate_resume(pending)
                store = CalibrationRecordStore(pending['record_storage']['primary_path'])
                store.save(pending)
                self._pending_record_document = None
                if pending.get('complete'):
                    completed = deepcopy(pending)
                    completed.update(ok=True, path=str(store.saved_path),
                                     message='完整标定记录已补存，恢复已通过验收的结果')
                    completed['document']['metadata']['source_path'] = str(store.saved_path)
                    self._finished(completed)
                    return
                self._resume_candidate = read_resume(store.saved_path)
            # 点击时重新读取并校验，不能使用几秒前的缓存启动运动。
            candidate = read_resume(self._resume_candidate['path'])
        except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
            self.status_label.setText(f'无法继续该记录：{exc}')
            self._resume_scanned_at = None
            return
        self.start(resume_candidate=candidate)

    def start(self, _checked=False, *, resume_candidate=None):
        self._refresh()
        if not self.start_button.isEnabled(): return
        try:
            snapshot = dict(self.feedback.control_settings_snapshot())
            ratio = snapshot.pop('tolerance_percent')
            snapshot.update(target_n=30, tolerance_n=pressure_tolerance_n(30, ratio))
            settings = FeedbackSettings(**snapshot)
            settings.validate(self.control.profile)
            pressure, _, _ = self.feedback.samples.read(time.monotonic(), allow_negative=True)
            if abs(pressure) > .02: raise ValueError('请先释放并确认压力在±0.02 N内')
            sensor = self.sensor_provider() or {}
            if not 0 <= sensor.get('age_seconds', 99) <= .5 or len(sensor.get('state', ())) < 8:
                raise ValueError('请先连接传感器并确认CH4～CH7实时数据有效')
            self._release = self.control.release_position_snapshot()['position_counts']
            learned = self.feedback.learned_map_snapshot()
            if learned is None: raise ValueError('请先完成学习或等待上次学习表核验恢复，再进行快速自动标定')
            if resume_candidate:
                saved = resume_candidate['document']['metadata']
                check_resume_context(resume_candidate['document'], self.control.profile,
                                     self._release, learned, snapshot)
                snapshot = deepcopy(saved['motion_settings'])
                ratio = saved['tolerance_percent']
                settings = FeedbackSettings(**snapshot)
                settings.validate(self.control.profile)
                self.point_count.setCurrentIndex(self.point_count.findData(saved['point_count']))
                self.round_count.setValue(saved['rounds'])
                self.rest_seconds.setValue(validate_rest_seconds(saved['rest_seconds']))
                self.accuracy_percent.setValue(validate_accuracy(saved['accuracy_percent']))
            count, rounds = self.point_count.currentData(), self.round_count.value()
            self.rest_seconds.interpretText()
            rest_seconds = validate_rest_seconds(self.rest_seconds.value())
            self.accuracy_percent.interpretText()
            accuracy_percent = validate_accuracy(self.accuracy_percent.value())
            historical_targets, historical_path = [], ''
            for history in sorted(self.output_dir.glob('high_force_*.json'), reverse=True):
                try:
                    previous = json.loads(history.read_text(encoding='utf-8'))
                    if (previous.get('complete') and previous.get('document') and
                            previous.get('metadata', {}).get('namespace') == self.control.profile.namespace):
                        historical_targets = historical_focus(previous['document'], previous['records'],
                                                              calibration_targets(count), accuracy_percent)
                        historical_path = str(history)
                        break
                except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError):
                    continue
            if resume_candidate:
                historical_targets = saved.get('historical_targets', [])
                historical_path = saved.get('historical_source', '')
            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir/('high_force_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
        except Exception as exc:
            self.status_label.setText(str(exc))
            return
        self._running = True
        self.feedback.begin_learning_task_notice(learned)
        self._context = {}
        self._rest_control = RestWaitControl()
        rest_control = self._rest_control
        self._rest_id, self._stopping = None, False
        self._stop_requested = threading.Event()
        self._emergency_requested = threading.Event()
        stop_requested, emergency_requested = self._stop_requested, self._emergency_requested
        self._path = path
        self._result = None
        self.stage_label.setText('阶段①：30 N热身，至少5次；10秒休息不可跳过')
        self.sample_result_title.setText('最近一次采样 · 等待本次任务采集')
        self.sample_result_values.setText('目标压力 — N    实测压力 — N\n四通道 Signal 合计 — counts')
        self.table.hide()
        self.quality_table.hide()
        self.timing_table.setRowCount(0)
        self.timing_table.hide()
        metadata = dict(point_count=count, rounds=rounds, release_position=self._release,
                        profile=self.control.profile.metadata(), namespace=self.control.profile.namespace,
                        motion_settings=snapshot, tolerance_percent=ratio, rest_seconds=rest_seconds,
                        accuracy_percent=accuracy_percent, historical_targets=historical_targets,
                        historical_source=historical_path,
                        round_timing_basis='includes_rest_before_each_press_and_initial_check')

        metadata.update(learning_fingerprint=learning_fingerprint(learned), resume_root_id=path.stem,
                        resume_generation=0, timing_excludes_offline_pause=True)
        if resume_candidate:
            metadata.update(resume_root_id=resume_candidate['root_id'],
                resume_generation=saved.get('resume_generation', 0)+1,
                resume_source_path=str(resume_candidate['path']), resume_source_sha256=resume_candidate['sha256'],
                legacy_learning_identity_unavailable=not bool(saved.get('learning_fingerprint')))
            self._show_round_timings(resume_candidate['document'].get('round_timings', []))
            self.stage_label.setText('继续标定：'+resume_candidate['summary'])

        def task(motor, cancel):
            records = []
            runner = None
            store = CalibrationRecordStore(path)
            emergency = CombinedCancel(emergency_requested, cancel)
            operation_cancel = CombinedCancel(stop_requested, emergency)
            revision = 0
            def save_record(document):
                nonlocal revision
                revision += 1
                document['record_revision'] = revision
                # 检查点只在卸载阶段写入；全盘不可写时先确认释放，保留内存数据等待恢复。
                document['record_storage'] = dict(primary_path=str(path), fallback_events=list(store.events))
                waiting = False
                while True:
                    try:
                        notice = store.save(document)
                        self._pending_record_document = None
                        if notice:
                            self.progress.emit(dict(phase='storage_warning', message=notice))
                        elif waiting:
                            self.progress.emit(dict(phase='storage_recovered', message='记录保存已恢复，保留进度继续'))
                        return True
                    except ForceSignalPressureCalibrationError as exc:
                        self._pending_record_document = document
                        if not waiting:
                            if runner is not None:
                                runner._recover(f'记录尚未落盘：{exc}')
                            else:
                                released = recover_release(motor, self.control.profile, self.feedback.samples,
                                    settings, self._release, emergency, progress=self.progress.emit,
                                    on_motor=self.control.publish_motor_feedback)
                                if not released['release_position_confirmed']:
                                    return False
                            waiting = True
                        self.progress.emit(dict(phase='storage_wait',
                            message=f'已释放，记录暂无法落盘；保留进度等待保存恢复：{exc}'))
                        if operation_cancel.is_set():
                            return False
                        operation_cancel.wait(.5)
            def checkpoint(rows):
                records[:] = rows
                save_record(dict(metadata=metadata, complete=False, records=rows,
                    elapsed_s=0 if runner is None else runner.elapsed_seconds(),
                    round_timings=[] if runner is None else runner.round_timings,
                    rest_periods=[] if runner is None else runner.rest_periods,
                    recoveries=[] if runner is None else runner.recoveries,
                    **({} if runner is None else runner.audit())))
            try:
                runner = HighForceCalibrationRunner(motor, self.control.profile, self.feedback.samples,
                    self.sensor_provider, operation_cancel, settings, self._release, learned, count, rounds, ratio,
                    progress=self.progress.emit, checkpoint=checkpoint, on_motor=self.control.publish_motor_feedback,
                    rest_seconds=rest_seconds, rest_control=rest_control, recovery_cancel=emergency,
                    accuracy_percent=accuracy_percent, historical_targets=historical_targets,
                    resume_document=None if resume_candidate is None else resume_candidate['document'])
                checkpoint(list(runner.records))
                result = runner.run()
                result['path'] = str(path)
                if result.get('document'):
                    result['document']['metadata']['source_path'] = str(path)
                persisted = save_record(dict(metadata=metadata, complete=result['ok'], records=result['records'],
                    message=result['message'], document=result.get('document'),
                    round_timings=result['round_timings'], elapsed_s=result['elapsed_s'], rest_periods=result['rest_periods'],
                    recoveries=result['recoveries'], release_position_confirmed=result.get('release_position_confirmed'),
                    pressure_unloaded=result.get('pressure_unloaded'), **runner.audit()))
                result['path'] = str(store.saved_path or path)
                result['record_storage'] = dict(primary_path=str(path), fallback_events=list(store.events))
                if result.get('document'):
                    result['document']['metadata']['source_path'] = result['path']
                if not persisted:
                    result.update(ok=False, document=None, storage_pending=True,
                        message=result['message']+'；最新记录尚未落盘，已保留在本次程序内存中，请勿关闭程序')
                elif store.saved_path != path:
                    result['message'] += '；主文件无法替换，完整记录已保存独立备份（见记录路径）'
            except Exception as exc:
                result = dict(ok=False, message=str(exc), records=records, path=str(path),
                              round_timings=[] if runner is None else runner.round_timings)
                released = recover_release(motor, self.control.profile, self.feedback.samples, settings, self._release,
                    emergency, progress=self.progress.emit, on_motor=self.control.publish_motor_feedback)
                result.update({key: value for key, value in released.items() if key != 'message'})
                result['message'] += '；'+released['message']
            self.finished.emit(result)
        accepted, reason = self.control.start_external_operation(task, '1～30 N自动标定')
        if not accepted:
            self._running = False
            self.feedback.end_learning_task_notice()
            self.status_label.setText(reason)
        else:
            self.status_label.setText(('继续标定已开始：'+resume_candidate['summary']+'；' if resume_candidate else '自动标定已开始，')+
                                     '正在检查释放状态')
        self._refresh()

    def stop(self):
        if self._running:
            self._stopping = True
            self._stop_requested.set()
            self.feedback.update_learning_task_notice('stopping')
            self.status_label.setText('停止后续按压，正在自动回原释放点；确认回位后结束')
            self._refresh()

    def emergency_stop(self):
        if self._running:
            self._stopping = True
            self._stop_requested.set()
            self._emergency_requested.set()
            self.feedback.update_learning_task_notice('emergency')
            self.control.request_external_stop()
            self.status_label.setText('紧急停止：已取消自动回位，正在停止电机')
            self._refresh()

    def skip_rest(self):
        if self._running and not self._stopping and self._rest_control.request_skip(self._rest_id):
            self.status_label.setText('已跳过本次剩余休息；确认卸载及新鲜反馈后按压下一点')
        self._refresh()

    def _coordinates_changed(self):
        if self._running: self.emergency_stop()

    def _reference_changed(self, reference):
        if self._running and reference.get('confirmed') and reference.get('position_counts') != self._release:
            self.emergency_stop()

    def _progress(self, row):
        if row.get('phase') == 'sample_result':
            sample = row['sample']
            stage = {'warmup':'热身', 'formal':'正式标定', 'retest':'重点复测',
                     'validation':'独立验收'}.get(row.get('stage'), '标定')
            state = {'captured':'已采样，正在释放', 'released':'已释放，本点采集完成',
                     'discarded':'本次未完成，不计入标定；等待回位或重试'}[row['sample_state']]
            self.sample_result_title.setText(
                f"最近一次采样 · {stage} · 第{sample['round']}轮 / 第{sample['point']}点 · {state}")
            self.sample_result_values.setText(
                f"目标压力 {sample['target_force_n']:.2f} N    实测压力 {sample['final_force_n']:.3f} N\n"
                f"四通道 Signal 合计 {sample['signal_sum']:,.1f} counts")
            return
        if not self._stopping:
            self.feedback.update_learning_task_notice(row.get('phase', 'starting'))
        self._rest_id = row.get('rest_id') if row.get('phase') == 'rest' and row.get('rest_remaining_s', 0) > 0 else None
        self._refresh()
        if self._stopping and row.get('phase') not in ('recovery', 'recovery_release'):
            return
        if row.get('stage'):
            self.stage_label.setText({'warmup':'阶段①：30 N热身（至少5次，最多15次）',
                'formal':'阶段②：正式标定', 'retest':'阶段③：针对性复测',
                'validation':'阶段③：冻结候选表，独立逐点验收'}.get(row['stage'], row['stage']))
        if row.get('phase') in ('stage', 'quality'):
            self.status_label.setText(row.get('message', self.stage_label.text()))
            return
        if row.get('phase') in ('storage_warning', 'storage_wait', 'storage_recovered'):
            self.status_label.setText(row['message'])
            return
        if row.get('phase') in ('recovery', 'retry', 'retry_wait'):
            self.status_label.setText(f"第{self._context.get('round', 0)}轮 · 点{self._context.get('point', 0)} · "
                                     + row.get('message', '正在恢复释放'))
            return
        if row.get('phase') == 'round_complete':
            self._show_round_timings(row['round_timings'])
            return
        if row.get('phase') == 'point': self._context = dict(row)
        context = self._context
        phase = {'point':'开始寻压', 'press':'快速接近＋压力微调', 'capture':'等待FLOATING并采集',
                 'release':'释放', 'release_wait':'等待通道卸载恢复', 'rest':'材料休息',
                 'rest_finished':'休息结束，核验卸载', 'recovery_release':'异常恢复：正在回原释放点'}.get(row.get('phase'), '')
        if row.get('phase') == 'warmup_hold':
            phase = f"30 N恒力热身 · 已保持{row.get('hold_elapsed_s', 0):.1f}/5秒"
        suffix = f" · 压力 {row['force_n']:.2f} N" if 'force_n' in row else ''
        if row.get('phase') == 'capture': suffix += f" · FLOATING {row['floating_count']}/4 · 采样 {row['stable_s']:.1f}/1秒"
        if row.get('phase') == 'rest': suffix += f" · 剩余 {math.ceil(row['rest_remaining_s'])} 秒后按压下一点"
        self.status_label.setText(f"第{context.get('round', 0)}/{context.get('rounds', self.round_count.value())}轮 · "
            f"点{context.get('point', 0)}/{context.get('points', self.point_count.currentData())} · "
            f"目标{context.get('target_n', 0):g} N · {phase}{suffix}")

    def _show_round_timings(self, rows):
        self.timing_table.setRowCount(len(rows))
        for index, row in enumerate(rows):
            state = '已完成' if row['completed'] else f"未完成（{row['completed_points']}点）"
            for column, value in enumerate((f"第{row['round']}轮", duration_text(row['elapsed_s']), state)):
                self.timing_table.setItem(index, column, QtWidgets.QTableWidgetItem(value))
        self.timing_table.setVisible(bool(rows))

    def _finished(self, result):
        self._running = False
        self._resume_scanned_at = None
        self.feedback.end_learning_task_notice()
        self._rest_id, self._stopping = None, False
        self._result = result
        self.feedback.handle_learning_operation_result(result)
        message = result['message']
        if result.get('ok'):
            try:
                document = result['document']
                saved = save_signal_pressure_mapping_document(document, 'auto_high')
                self.k_widget._show_document(document, result['path'], saved['archive_path'])
                self.k_widget.status_label.setText('三阶段标定及独立精度验收通过，可检查K表并使用原有写入MCU入口')
                self.k_widget.mapping_changed.emit()
                self.table.setRowCount(len(document['nodes']))
                for index, node in enumerate(document['nodes']):
                    values = (f"{node['nominal_target_force_n']:g}", f"{node['measured_pressure_median_n']:.3f}",
                              f"{node['signal_sum_median']:.1f}", f"{node['k_n_per_signal_count']:.9f}")
                    for column, value in enumerate(values): self.table.setItem(index, column, QtWidgets.QTableWidgetItem(value))
                self.table.show()
                message += f"；已生成{len(document['nodes'])}点K值及{len(document['segments'])}个线性分段，尚未写入MCU"
                message += f"；实测节点范围{document['nodes'][0]['measured_pressure_median_n']:.2f}～{document['nodes'][-1]['measured_pressure_median_n']:.2f} N，端点外精度未验证"
            except Exception as exc:
                message += f'；K表发布失败，原始记录已保留：{exc}'
        else:
            message += f"；保留{len(result.get('records', []))}次完整采集，不覆盖原K表。"
            message += ('已确认回到原释放位置。' if result.get('release_position_confirmed')
                        else '未确认释放位置，自动回位已被紧急停止。')
        self._show_round_timings(result.get('round_timings', []))
        batches = result.get('quality', {}).get('passes', [])
        checks = []
        if batches:
            batch = batches[-1]
            checks = batch.get('validation', batch.get('internal', batch.get('initial', {}))).get('checks', [])
        worst = {}
        for check in checks:
            target = check['target_force_n']
            if target not in worst or abs(check['error_n']) > abs(worst[target]['error_n']):
                worst[target] = check
        self.quality_table.setRowCount(len(worst))
        independent = bool(batches and 'validation' in batches[-1])
        for index, (target, check) in enumerate(sorted(worst.items())):
            values = (f'{target:g}', f"{check['measured_n']:.3f}",
                f"{check['error_n']:+.3f} / {check['allowed_error_n']:.3f}",
                ('独立验收' if independent else '内部检查')+('通过' if check['passed'] else '未通过'))
            for column, value in enumerate(values):
                self.quality_table.setItem(index, column, QtWidgets.QTableWidgetItem(value))
        self.quality_table.setVisible(bool(worst))
        if 'elapsed_s' in result:
            message += f"；总耗时 {duration_text(result['elapsed_s'])}，每轮耗时见下表（含休息）"
        names = {'warmup':'热身', 'formal':'正式标定', 'retest':'针对性复测/验收'}
        self.stage_label.setText(('验收通过' if result.get('ok') else '未通过/已停止，原K表保留')+' · '+
            '；'.join(f"{names[t['stage']]} {duration_text(t['elapsed_s'])}" for t in result.get('stage_timings', [])))
        self.status_label.setText(message+f"\n记录：{result.get('path', '')}")
        self.status_message.emit(message)
        self._refresh()

    def shutdown(self):
        self.timer.stop()
        self.stop()
