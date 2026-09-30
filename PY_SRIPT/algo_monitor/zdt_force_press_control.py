"""监控页按力下压：共用压力反馈页参数、学习表和寻压过程。"""

import json
import time

from force_press_control_widget import (
    ForcePressControlWidget, ForceControlError, TARGET_FORCES_N,
    force_sweep_checkpoint_path, force_press_settings_path,
)
from force_position_lookup import (
    mapping_directory, resolve_force_position_target_n, resolve_mapping_release_reference,
    ForcePositionLookupError,
)
from pressure_feedback_control import CONTROL_STRATEGY, FeedbackSettings, PressureFeedbackController, PressureSamples, pressure_tolerance_n
from qt_compat import QtWidgets
from wrapped_label import WrappedLabel
from creep_test_safety import CreepDeadlineEvent, CREEP_RELEASE_TIMEOUT_SECONDS
import threading


class ZDTForcePressControlWidget(ForcePressControlWidget):
    def __init__(self, motor_control_widget, sensor_sample_provider=None, parent=None):
        self._profile = motor_control_widget.profile
        self._release_tolerance = self._profile.tolerance(256)
        self._feedback_samples = PressureSamples()
        self._feedback_source = None
        self._learned_map_provider = lambda: None
        self._native_history = {}
        super().__init__(motor_control_widget, sensor_sample_provider, parent)

    def set_feedback_source(self, source):
        """绑定压力反馈页；参数读取仅发生在界面线程，任务使用独立快照。"""
        self._feedback_source = source
        self.learning_notice_label.setStyleSheet(source.learning_notice_label.styleSheet())
        source.learning_notice_changed.connect(self._set_learning_notice)
        self.operation_finished.connect(source.handle_learning_operation_result)
        self._set_learning_notice(source.learning_notice_text())
        self.set_learned_map_provider(source.learned_map_snapshot)
        for widget in source.inputs.values():
            widget.valueChanged.connect(self._sync_native_settings)
        self._sync_native_settings()

    def set_learned_map_provider(self, provider):
        self._learned_map_provider = provider
        self.reload_lookup_mapping()

    def _set_learning_notice(self, message):
        if self._feedback_source is not None:
            self.learning_notice_label.setStyleSheet(self._feedback_source.learning_notice_label.styleSheet())
        self.learning_notice_label.setText(message)
        self.learning_notice_label.setVisible(bool(message))

    def _build_ui(self, settings):
        settings = dict(settings, max_current_ma=3000)
        super()._build_ui(settings)
        self.setTitle("按力下压（新电机 · 快速接近＋压力微调）")
        self.workflow_note.setText(
            "与压力反馈控制页共用当前运动参数、压力容差、保护参数及有效学习结果。"
            "有学习表时按同一规则快速接近，否则从释放点自适应寻压。"
            "到位并完成稳定确认后交给原有采集流程；释放一次返回固定释放点。")
        self.motion_parameter_group.hide()
        self.native_motion_label = WrappedLabel()
        self.content.layout().insertWidget(3, self.native_motion_label)
        # 监控页保留直接按压与释放操作；后台复用的参数控件不再占用页面空间。
        for section in (self.lookup_map_label, self.native_motion_label,
                        self.auto_test_group, self.sensor_calibration_group,
                        self.set_release_position_button):
            section.hide()
        self.learning_notice_label = WrappedLabel()
        self.learning_notice_label.hide()
        self.content.layout().insertWidget(0, self.learning_notice_label)
        self.press_speed_input.setRange(1, self._profile.speed_max)
        self.release_speed_input.setRange(1, self._profile.speed_max)
        self.acceleration_input.setRange(0, self._profile.acceleration_max)
        for label in self.findChildren(QtWidgets.QLabel):
            text = label.text()
            if "查表一次性到位" in text:
                label.setText(text.replace("查表一次性到位", "快速接近＋压力微调到位"))

    def _load_settings(self):
        destination = self._settings_path
        try:
            if not destination.exists():
                self._settings_path = force_press_settings_path()
            settings = super()._load_settings()
        finally:
            self._settings_path = destination
        # 重复次数等任务设置独立保存，运动参数由压力反馈页提供。
        settings["max_current_ma"] = 3000
        return settings

    def _sync_native_settings(self, *_):
        if self._running or self._feedback_source is None:
            return
        snapshot = self._feedback_source.control_settings_snapshot()
        for widget, value in ((self.press_speed_input, snapshot['speed']),
                              (self.release_speed_input, snapshot['release_speed']),
                              (self.acceleration_input, snapshot['acceleration'])):
            widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(False)
        self._update_settings_snapshot()

    def _operation_motion_parameters(self, commit=True):
        if self._feedback_source is None:
            raise ForceControlError('feedback_settings_missing', '压力反馈控制页尚未初始化，不能启动按力控制')
        snapshot = self._feedback_source.control_settings_snapshot(commit=commit)
        return dict(press_speed_level=snapshot['speed'], release_speed_level=snapshot['release_speed'],
                    acceleration=snapshot['acceleration'], fine_step_max=snapshot['max_step'],
                    max_current_ma=snapshot['max_current_ma'], max_travel_counts=snapshot['max_travel'],
                    force_limit_n=snapshot['force_limit_n'], feedback_settings=snapshot,
                    approach_speed=snapshot['approach_speed'],
                    learned_map=self._learned_map_provider())

    def _start_operation(self, *args, **kwargs):
        if self._feedback_source is None:
            self._set_status('压力反馈控制页尚未初始化，不能启动按力控制')
            return
        mode = args[0] if args else kwargs.get('mode')
        if mode != 'release' and self._feedback_source.learning_recovery_pending():
            self._set_status('学习表已保留；请先确认停止并回原释放点卸载，恢复检查通过后再下压')
            return
        return super()._start_operation(*args, **kwargs)

    def _update_settings_snapshot(self, *_):
        super()._update_settings_snapshot()
        if self._feedback_source is None:
            return
        motion = self._operation_motion_parameters(commit=False)
        settings = motion['feedback_settings']
        with self._state_lock:
            self._settings_snapshot.update({k: v for k, v in motion.items() if k not in ('feedback_settings', 'learned_map')},
                                           **self._profile.metadata(), speed_unit="RPM",
                                           acceleration_unit="档", control_strategy=CONTROL_STRATEGY)
        self.native_motion_label.setText(
            f"同步压力反馈控制：快速 {motion['approach_speed']} RPM / 微调 {motion['press_speed_level']} RPM / 释放 {motion['release_speed_level']} RPM · 加速度 {motion['acceleration']} 档 · "
            f"微调每次最多 {motion['fine_step_max']} step\n"
            f"容差 ±{settings['tolerance_percent']:g}%（≤0.2 N时至少±0.01 N） · 到位等待 {settings['settle_s']:g}秒 · 稳定确认 {settings['stable_s']:g}秒\n"
            "0.1～0.2 N：释放点底数稳定且与学习时一致才快速走空行程；允许±0.02 N底数，接触及受力段≤2 RPM。\n"
            f"保护：压力 {motion['force_limit_n']:.2f} N · 电流 {motion['max_current_ma']} mA · "
            f"释放点以下最大行程 {motion['max_travel_counts']} step；请到压力反馈控制页修改参数。")

    def _sweep_checkpoint_path(self):
        path = force_sweep_checkpoint_path()
        return path.with_name(f"{path.stem}_{self._profile.namespace}{path.suffix}")

    def _load_lookup_document(self, show_error=True):
        release = self._confirmed_release_position()
        reason = "尚无有效的新电机位置表"
        document = None
        path = mapping_directory() / self._profile.namespace / 'force_position_map_latest.json'
        try:
            stored = json.loads(path.read_text(encoding='utf8'))
            if not isinstance(stored, dict) or not isinstance(stored.get('metadata'), dict):
                raise ForcePositionLookupError("新电机位置表格式无效")
            metadata = stored.get('metadata', {})
            if any(metadata.get(k) != v for k, v in self._profile.metadata().items()):
                raise ForcePositionLookupError("位置表电机型号或步数单位不匹配")
            candidate = self._profile.storage_fields(stored, read=True)
            reference = resolve_mapping_release_reference(candidate)
            if release is None or reference != release:
                raise ForcePositionLookupError("位置表释放基准与当前确认的释放点不一致")
            document = candidate
        except (OSError, ValueError, TypeError) as exc:
            reason = str(exc)
        if document is None:
            # 这是明确标记的运行计划，不伪造稳定位置表，也不写回历史文件。
            document = dict(feedback_only=True, mapping=[], metadata=dict(
                force_unit='N', mapping_mode='release_relative_travel',
                release_reference_position_counts=release or 0, created_at='压力闭环寻压',
                **self._profile.metadata()))
        self._lookup_document = document
        self._lookup_targets = {force: self._resolve_lookup_target(document, force) for force in TARGET_FORCES_N}
        self.lookup_map_label.setText(
            "寻压与压力反馈控制页一致：使用当前已核验学习表快速接近；位置表用于采集位置参考。"
            + (f"\n{reason}；可在压力反馈控制页“一键学习到30 N”。" if document.get('feedback_only') else ""))
        self.lookup_map_label.setStyleSheet("color:#8bd5ff;")
        return document

    def _resolve_lookup_target(self, document, target_force_n):
        release = resolve_mapping_release_reference(document)
        if not document.get('feedback_only'):
            try:
                resolved = resolve_force_position_target_n(document, target_force_n)
                if resolved['press_travel_counts'] <= self._profile.default_settings['max_travel_counts']:
                    return resolved
            except ForcePositionLookupError:
                pass
        travel = None
        table = self._learned_map_provider()
        if table is not None and table.matches(self._profile, release):
            travel = table.travel_at(target_force_n)
        if travel is None:
            with self._state_lock:
                travel = self._native_history.get((release, round(float(target_force_n), 2)))
        return dict(force_n=target_force_n, release_reference_position_counts=release,
                    press_travel_counts=0 if travel is None else round(travel),
                    target_position_counts=release if travel is None else release-round(travel),
                    position_span_counts=0, created_at=document['metadata'].get('created_at', '--'))

    def _on_release_position_changed(self, snapshot):
        with self._state_lock:
            self._native_history.clear()
        super()._on_release_position_changed(snapshot)
        self.release_position_label.setText(self.release_position_label.text().replace("counts", "step"))
        self.reload_lookup_mapping()

    def set_pressure_sample(self, sample):
        self._feedback_samples.update(sample or {})
        super().set_pressure_sample(sample)

    def _prepare_motor(self, motor, tracker, stop_event, parameters):
        if stop_event.is_set():
            raise ForceControlError('user_stop', '用户请求停止')
        releasing = parameters['mode'] in ('release', 'creep_test')
        self._controller_settings(parameters).validate(self._profile, release_only=releasing)
        motor.prepare_motion()
        motor.set_current_limit_ma(parameters['max_current_ma'])
        self._poll_checked(motor, tracker, stop_event, parameters['max_current_ma'],
                           None if releasing else parameters['force_limit_n'],
                           'release' if releasing else 'press')

    def _read_feedback(self, motor, tracker):
        feedback = dict(motor.read_feedback())
        now, position = feedback['sampled_monotonic'], feedback['position_counts']
        previous = tracker.get('sampled_at')
        interval = None if previous is None else now-previous
        velocity = ((position-tracker['position'])/interval if interval and interval > 0 else None)
        feedback.update(position_speed_counts_s=velocity,
                        sample_interval_ms=None if interval is None else interval*1000,
                        running_state_name='运动中' if feedback['running_state'] == 2 else '已停止',
                        source='zdt_force_press_control')
        tracker.update(sampled_at=now, position=position, position_speed=velocity)
        self._motor_control.publish_motor_feedback(feedback)
        return feedback

    def _show_motor_feedback(self, feedback):
        feedback = feedback or {}
        self.position_value_label.setText(f"电机位置 {feedback.get('position_counts', '--')} step")
        self.speed_value_label.setText(f"实时转速 {feedback.get('measured_speed_rpm', '--')} RPM")
        with self._state_lock:
            limit = self._settings_snapshot.get('max_current_ma', 3000)
        self.current_value_label.setText(f"相电流 {feedback.get('phase_current_ma', '--')} / {limit} mA")
        self.state_value_label.setText('电机状态 ' + ('运动中' if feedback.get('running_state') == 2 else '已停止'))
        self.acceleration_value_label.setText(f"设置加速度 {self.acceleration_input.value()} 档")

    def _controller_settings(self, parameters):
        """只替换本轮目标压力，容差和全部控制参数沿用界面线程冻结的快照。"""
        target = float(parameters.get('target_force_n') or 1)
        values = dict(parameters['feedback_settings'])
        values['target_n'] = target
        values['tolerance_n'] = pressure_tolerance_n(target, values.pop('tolerance_percent'))
        return FeedbackSettings(**values)

    def _controller(self, motor, stop_event, parameters):
        settings = self._controller_settings(parameters)
        def record(row):
            self._set_motion_context(row['command_steps'], row['travel_steps'])
            self.operation_progress.emit(dict(phase='native_force_control', current_force_n=row['force_n'],
                target_force_n=row['target_n'], command_counts=row['command_steps'],
                total_travel_counts=row['travel_steps'],
                message=f"{row['phase']}：{row['force_n']:.2f} / {row['target_n']:.2f} N · "
                f"位置{row['position_steps']:.0f} step"))
            self._record_event('native_force_control', final_force_n=row['force_n'],
                               message=f"{row['phase']}；误差{row['error_n']:+.3f} N")
        return PressureFeedbackController(motor, self._profile, self._feedback_samples, stop_event,
            settings, parameters['release_position_counts'], on_record=record,
            on_motor=self._motor_control.publish_motor_feedback, learned_map=parameters['learned_map'])

    def _run_press(self, motor, tracker, stop_event, parameters, require_zero_start=True):
        controller = self._controller(motor, stop_event, parameters)
        release, target = parameters['release_position_counts'], float(parameters['target_force_n'])
        started = time.monotonic()
        result = controller.run(acquire_only=True, require_zero_start=require_zero_start)
        if not result['ok']:
            raise ForceControlError('user_stop' if result.get('cancelled') else 'pressure_feedback_failed', result['message'])
        position = int(result['actual_position_steps'])
        force = float(result['actual_force_n'])
        with self._state_lock:
            self._native_history[(release, round(target, 2))] = release-position
        # 无事先位置表时，以实测到位位置作为后续采样漂移的参照，不能拿释放点当目标。
        if parameters['lookup_press_travel_counts'] <= 0:
            parameters['lookup_target_position_counts'] = position
        self._read_feedback(motor, tracker)
        timing = f"首次进入容差 {result['first_in_band_s']:.2f}秒 · 稳定到位 {result['stable_arrival_s']:.2f}秒"
        self._record_event('pressure_arrival_timing', message=timing,
                           first_in_band_s=result['first_in_band_s'], stable_arrival_s=result['stable_arrival_s'])
        return dict(ok=True, stop_reason='pressure_target_reached', final_force_n=force,
            first_in_band_s=result['first_in_band_s'], stable_arrival_s=result['stable_arrival_s'],
            final_position_counts=position, target_position_counts=parameters['lookup_target_position_counts'],
            lookup_target_position_counts=parameters['lookup_target_position_counts'],
            position_error_counts=position-parameters['lookup_target_position_counts'],
            total_travel_counts=release-position, motion_started_monotonic=started,
            arrived_monotonic=time.monotonic(), lookup_created_at=parameters['lookup_created_at'],
            message=f"新电机到位：{force:.2f} / {target:.2f} N，位置{position} step；{timing}")

    def _run_release(self, motor, tracker, stop_event, parameters):
        controller = self._controller(motor, stop_event, parameters)
        result = controller.run(release_only=True)
        if not result['ok']:
            raise ForceControlError('user_stop' if result.get('cancelled') else 'release_failed', result['message'])
        self._read_feedback(motor, tracker)
        force, _, _ = self._feedback_samples.read(time.monotonic(), allow_negative=True)
        result.update(stop_reason='released_to_fixed_position', final_force_n=force,
                      final_position_counts=result['actual_position_steps'],
                      release_position_counts=parameters['release_position_counts'])
        if parameters.get('wait_for_sensor_idle_after_release'):
            idle = self._wait_for_sensor_idle_after_release(motor, tracker, stop_event, parameters)
            result.update(release_idle_recovered=idle['recovered'], release_idle_wait_seconds=idle['wait_seconds'],
                          release_idle_stable_seconds=idle['stable_seconds'], release_idle_timeout_seconds=idle['timeout_seconds'])
        return result

    def _run_rebound_press(self, motor, tracker, stop_event, parameters):
        return self._run_press(motor, tracker, stop_event, parameters)

    def _creep_emergency_release(self, motor, tracker, parameters):
        # 失联命令不重发；仍通过同一驱动接口执行一次有界释放。
        self._safe_brake(motor)
        bounded = CreepDeadlineEvent(threading.Event(), CREEP_RELEASE_TIMEOUT_SECONDS, ForceControlError)
        return self._run_creep_release(motor, tracker, bounded, parameters)

    @staticmethod
    def _safe_brake(motor):
        try:
            motor.stop()
        except Exception:
            pass

    def _move_relative(self, *args, **kwargs):
        raise ForceControlError('legacy_motion_blocked', '新电机禁止调用旧驱动位移接口')

    def request_stop(self):
        # 即使实时状态已过期，也必须取消正在运行的新电机任务并请求停止。
        self._motor_control.request_external_stop()
        self._set_status('已请求新电机紧急停止，等待停止确认')

    def _on_operation_progress(self, progress):
        super()._on_operation_progress(progress)
        self.command_value_label.setText(self.command_value_label.text().replace('counts', 'step'))

    def _set_status(self, message, summary=None, force_summary=True):
        message = str(message).replace('counts', 'step').replace('JSON标定出发点', '释放点')
        super()._set_status(message, summary, force_summary)

    def _force_mapping_export_row(self, entry):
        row = super()._force_mapping_export_row(entry)
        # 保持下游传感器标定所需列名，同时明确本记录的真实电机和单位。
        row.update(self._profile.metadata(), speed_unit='RPM', acceleration_unit='档',
                   control_strategy=CONTROL_STRATEGY)
        return row

    def _refresh_controls(self):
        super()._refresh_controls()
        for widget in (self.press_speed_input, self.release_speed_input, self.acceleration_input):
            widget.setEnabled(False)
