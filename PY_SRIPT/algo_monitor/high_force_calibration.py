"""1～30 N自动标定：复用压力闭环，采集真实FLOATING下的四通道Signal。"""
from dataclasses import replace
from copy import deepcopy
from collections import deque
from datetime import datetime
import math
import statistics
import threading
import time

from channel_states import STATE_FLOATING, RELEASE_READY_STATES
from pressure_feedback_control import PressureFeedbackController, FeedbackControlError, pressure_tolerance_n
from force_signal_pressure_calibration import high_force_calibration_targets as calibration_targets
from high_force_calibration_quality import (paired_mapping, assess_mapping, warmup_convergence,
    validate_accuracy, DEFAULT_ACCURACY_PERCENT, WARMUP_MAX, WARMUP_HOLD_S,
    WARMUP_REST_S, MAX_RETEST_PASSES, RETEST_REPEATS)

from high_force_calibration_resume import validate_resume

HIGH_FORCE_PROFILE = 'auto_1_to_30_n'
CHANNELS = (4, 5, 6, 7)
DEFAULT_REST_SECONDS = 30.0
MAX_REST_SECONDS = 3600.0
RETRY_COOLDOWN_AFTER = 3
RETRY_COOLDOWN_SECONDS = 30.0


class CombinedCancel:
    """普通停止仅取消采集；硬件急停还会取消释放恢复。"""
    def __init__(self, stop, emergency):
        self.stop, self.emergency = stop, emergency

    def is_set(self):
        return self.stop.is_set() or self.emergency.is_set()

    def wait(self, seconds):
        self.stop.wait(seconds)
        return self.is_set()


def recover_release(motor, profile, pressure, settings, release, emergency,
                    progress=lambda row: None, on_motor=lambda row: None,
                    clock=time.monotonic, wait=None, controller_factory=PressureFeedbackController,
                    return_even_within_tolerance=False, position_tolerance_steps=None, release_side_tolerance_steps=None):
    """异常后持续回位；仅依赖电机反馈放行抬升，不让压力/传感器断流阻止释放。"""
    wait = wait or emergency.wait
    tolerance = profile.tolerance(256) if position_tolerance_steps is None else position_tolerance_steps
    if type(tolerance) is not int or not 1 <= tolerance <= profile.tolerance(256):
        raise ValueError('释放位置容差须为允许范围内的正整数')
    release_side_tolerance = tolerance if release_side_tolerance_steps is None else release_side_tolerance_steps
    if type(release_side_tolerance) is not int or not tolerance <= release_side_tolerance <= profile.tolerance(256):
        raise ValueError('释放侧位置容差须在精确容差与驱动回位窗口之间')
    attempts, last_error = 0, ''
    return_command_attempted = False
    while not emergency.is_set():
        attempts += 1
        progress(dict(phase='recovery', recovery_attempt=attempts, message=last_error or '正在停止并回到原释放点'))
        try:
            # 停止ACK丢失不等于电机未停，后续必须以多组新鲜反馈确认。
            try:
                motor.stop()
            except Exception as exc:
                last_error = f'停止应答异常：{exc}'
            started, stable_since, last_stamp = clock(), None, None
            stopped = None
            while clock()-started < 3:
                if emergency.is_set():
                    raise FeedbackControlError('紧急停止释放恢复')
                sample = motor.read_feedback()
                now = clock()
                position, speed, current, stamp = (float(sample[k]) for k in
                    ('position_counts', 'measured_speed_rpm', 'phase_current_ma', 'sampled_monotonic'))
                if not all(math.isfinite(v) for v in (position, speed, current, stamp)) or not 0 <= now-stamp <= .5:
                    raise FeedbackControlError('等待新鲜有效电机反馈后回位')
                on_motor(sample)
                if abs(current) > settings.max_current_ma or int(sample['running_state']) in (3, 4, 5):
                    raise FeedbackControlError('等待电机过流/故障解除后回位')
                stationary = abs(speed) <= 1 and int(sample['running_state']) in (0, 1)
                if not stationary:
                    stable_since = None
                elif stamp >= started and (last_stamp is None or stamp > last_stamp):
                    if stable_since is None:
                        stable_since = stamp
                    elif stamp-stable_since >= .1-1e-9:
                        stopped = sample
                        break
                last_stamp = stamp
                wait(.05)
            if stopped is None:
                raise FeedbackControlError('停止反馈尚未稳定，继续尝试停止与回位')
            position = float(stopped['position_counts'])
            # 接触学习的加载行程可能小于16 step；仍先发回原点指令，不能直接把接触位置当释放点。
            if (-tolerance <= position-release <= release_side_tolerance
                    and (not return_even_within_tolerance or position >= release
                         or return_command_attempted and abs(position-release) <= profile.tolerance(16))):
                unloaded = False
                try:
                    force, _, _ = pressure.read(clock(), allow_negative=True)
                    unloaded = math.isfinite(force) and abs(force) <= .1
                except Exception:
                    pass
                return dict(release_position_confirmed=True, stop_confirmed=True,
                            actual_release_position_steps=position, release_position_error_steps=position-release,
                            pressure_unloaded=unloaded, release_recovery_attempts=attempts,
                            message='已回到原释放位置并确认静止' + ('，压力已卸载' if unloaded else '；压力卸载尚未确认'))
            if position_tolerance_steps is not None and position > release+release_side_tolerance:
                # 已在释放侧但超出精确窗口时不能反向下压纠偏，由任务层退回原始安全释放点。
                return dict(release_position_confirmed=False, stop_confirmed=True, pressure_unloaded=False,
                            actual_release_position_steps=position, release_position_error_steps=position-release,
                            release_position_outside_tolerance=True, release_recovery_attempts=attempts,
                            message=f'实际位置{position:g} step超出释放点{release} step的-{tolerance}～+{release_side_tolerance}步窗口；未反向下压')
            if not release-settings.max_travel <= position < release:
                raise FeedbackControlError('坐标超出已确认释放范围，等待核对；不盲目移动')
            if emergency.is_set():
                raise FeedbackControlError('紧急停止释放恢复')
            controller = controller_factory(motor, profile, pressure, emergency, settings, release,
                on_motor=on_motor, clock=clock,
                on_record=lambda row: progress(dict(row, phase='recovery_release')))
            controller.release_tolerance = tolerance
            outcome = controller.run(release_only=True)
            return_command_attempted = bool(outcome.get('move_count'))
            # 即使运动ACK或压力诊断失败，也回到顶部重新读真实位置，不重放旧相对位移。
            last_error = outcome.get('message', '释放反馈待确认')
        except Exception as exc:
            last_error = str(exc)
        progress(dict(phase='recovery', recovery_attempt=attempts,
                      message=f'{last_error}；自动重试回位，尚未退出恢复'))
        wait(.5)
    try:
        motor.stop()
    except Exception:
        pass
    return dict(release_position_confirmed=False, stop_confirmed=False, pressure_unloaded=False,
                release_recovery_attempts=attempts, message='释放恢复被紧急停止，未确认回位' + (f'：{last_error}' if last_error else ''))


def validate_rest_seconds(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= MAX_REST_SECONDS:
        raise ValueError('点间休息须为0～3600秒')
    return float(value)


class RestWaitControl:
    """按等待编号接收跳过请求，迟到或重复点击不会影响下一次休息。"""
    def __init__(self):
        self._lock = threading.Lock()
        self._generation = 0
        self._active = None
        self._skip = False

    def begin(self):
        with self._lock:
            self._generation += 1
            self._active, self._skip = self._generation, False
            return self._active

    def can_skip(self, wait_id):
        with self._lock:
            return wait_id is not None and self._active == wait_id and not self._skip

    def request_skip(self, wait_id):
        with self._lock:
            if wait_id is None or self._active != wait_id or self._skip:
                return False
            self._skip = True
            return True

    def skip_requested(self, wait_id):
        with self._lock:
            return self._active == wait_id and self._skip

    def end(self, wait_id):
        with self._lock:
            if self._active == wait_id:
                self._active, self._skip = None, False


def build_high_force_mapping(records, count, rounds, source_path=''):
    targets = calibration_targets(count)
    if type(rounds) is not int or not 1 <= rounds <= 100 or len(records) != count*rounds:
        raise ValueError('标定尚未完成全部轮次，未生成有效K表')
    grouped = {target: [] for target in targets}
    seen = set()
    for entry in records:
        key = (entry['round'], entry['target_force_n'])
        if key in seen or key[1] not in targets or type(key[0]) is not int or not 1 <= key[0] <= rounds:
            raise ValueError('标定轮次或压力点重复/不完整')
        if entry['floating_count'] != 4:
            raise ValueError('仅接受四通道全部FLOATING的采样')
        for name in ('signal_sum', 'final_force_n'):
            if not math.isfinite(entry[name]) or entry[name] <= 0:
                raise ValueError('标定值无效')
        seen.add(key)
        grouped[key[1]].append(entry)
    metadata = dict(format_version=2, calibration_profile=HIGH_FORCE_PROFILE,
        force_unit='N', feature='ch4567_signal_sum', pressure_reference='final_force_n',
        created_at=datetime.now().astimezone().isoformat(), rounds=rounds,
        point_count=count, source_path=str(source_path), source_row_count=len(records),
        required_sensor_state='CH4_CH7_ALL_FLOATING', interpolation='piecewise_linear')
    return paired_mapping(grouped, metadata)


class HighForceCalibrationRunner:
    def __init__(self, motor, profile, pressure_samples, sensor_provider, cancel,
                 settings, release, learned_map, count, rounds, tolerance_percent,
                 progress=lambda row: None, checkpoint=lambda rows: None,
                 on_motor=lambda row: None, clock=time.monotonic, controller_factory=PressureFeedbackController,
                 rest_seconds=DEFAULT_REST_SECONDS, rest_control=None, recovery_cancel=None, recovery_wait=None,
                 accuracy_percent=DEFAULT_ACCURACY_PERCENT, historical_targets=(), resume_document=None):
        self.motor, self.profile, self.pressure = motor, profile, pressure_samples
        self.sensor, self.cancel, self.clock = sensor_provider, cancel, clock
        self.settings, self.release, self.learned_map = settings, release, learned_map
        self.targets, self.rounds = calibration_targets(count), rounds
        if type(rounds) is not int or not 1 <= rounds <= 100:
            raise ValueError('轮数须为1～100')
        self.tolerance_percent = tolerance_percent
        self.progress, self.checkpoint, self.on_motor = progress, checkpoint, on_motor
        self.factory, self.records = controller_factory, []
        self.round_timings = []
        self.rest_seconds = validate_rest_seconds(rest_seconds)
        self.rest_control = rest_control if rest_control is not None else RestWaitControl()
        self.rest_periods = []
        self.recovery_cancel = recovery_cancel if recovery_cancel is not None else threading.Event()
        self.recovery_wait = recovery_wait
        self.recoveries = []
        self.release_status = {}
        self.accuracy_percent = validate_accuracy(accuracy_percent)
        self.historical_targets = sorted(set(historical_targets) & set(self.targets))
        self.stage = 'warmup'
        self.stage_timings = []
        self.warmup_records, self.retest_records, self.validation_records = [], [], []
        self.quality = {}
        self.candidate_document = None
        self._press_count = 0
        self.point_failures, self.retry_waits, self.motion_diagnostics = [], [], []
        self.point_context = {}
        self._motor_trace = deque(maxlen=24)
        self._stage_started = self._run_started = None
        self._stage_base = self._elapsed_before = 0.0
        self.resume_generation = 0
        self.resume_sessions = []
        if resume_document is not None:
            validate_resume(resume_document)
            saved = deepcopy(resume_document)
            if saved['metadata']['point_count'] != count or saved['metadata']['rounds'] != rounds:
                raise ValueError('续跑计划与记录不一致')
            for name in ('records', 'round_timings', 'rest_periods', 'recoveries', 'stage_timings',
                         'warmup_records', 'retest_records', 'validation_records', 'point_failures',
                         'retry_waits', 'motion_diagnostics', 'resume_sessions'):
                setattr(self, name, saved.get(name, []))
            self.quality = saved.get('quality', {})
            self.stage = saved.get('stage', 'warmup')
            self.candidate_document = saved.get('candidate_document')
            self.resume_generation = saved['metadata'].get('resume_generation', 0)+1
            self._elapsed_before = saved.get('elapsed_s', sum(t['elapsed_s'] for t in self.stage_timings))
            self._press_count = sum(len(getattr(self, name)) for name in
                ('records', 'warmup_records', 'retest_records', 'validation_records'))
        # 诊断只消费真实反馈，不改变硬件状态或上报内容。
        def observe_motor(sample):
            row = {key: sample.get(key) for key in ('position_counts', 'measured_speed_rpm',
                'phase_current_ma', 'running_state', 'in_position', 'sampled_monotonic')}
            row = {key: None if isinstance(value, float) and not math.isfinite(value) else value
                   for key, value in row.items()}
            stamp = row['sampled_monotonic']
            row['feedback_age_s'] = self.clock()-stamp if isinstance(stamp, (int, float)) else None
            row['observed_at'] = datetime.now().astimezone().isoformat()
            self._motor_trace.append(row)
            on_motor(sample)
        self.on_motor = observe_motor

    def _stage(self, name):
        self._update_stage_time()
        if self._stage_started is not None:
            self.stage_timings[-1]['completed'] = True
        self.stage, self._stage_started = name, self.clock()
        if not self.stage_timings or self.stage_timings[-1]['stage'] != name:
            self.stage_timings.append(dict(stage=name, elapsed_s=0, completed=False))
        self._stage_base = self.stage_timings[-1]['elapsed_s']
        self.stage_timings[-1]['completed'] = False
        self.progress(dict(phase='stage', stage=name))

    def _update_stage_time(self):
        if self._stage_started is not None:
            self.stage_timings[-1]['elapsed_s'] = self._stage_base+self.clock()-self._stage_started

    def audit(self):
        self._update_stage_time()
        if self.resume_generation and self._run_started is not None:
            self.resume_sessions[-1]['elapsed_s'] = self.clock()-self._run_started
        return dict(stage=self.stage, stage_timings=self.stage_timings, warmup_records=self.warmup_records,
            retest_records=self.retest_records, validation_records=self.validation_records,
            quality=self.quality, candidate_document=self.candidate_document,
            point_failures=self.point_failures, retry_waits=self.retry_waits,
            motion_diagnostics=self.motion_diagnostics, point_context=self.point_context,
            historical_targets=self.historical_targets, accuracy_percent=self.accuracy_percent,
            resume_sessions=self.resume_sessions)

    def elapsed_seconds(self):
        return self._elapsed_before+(0 if self._run_started is None else self.clock()-self._run_started)

    def _warmup(self):
        if self.warmup_records and warmup_convergence(self.warmup_records)['passed']:
            return
        self._stage('warmup')
        for index in range(len(self.warmup_records)+1, WARMUP_MAX+1):
            self.progress(dict(phase='point', stage=self.stage, round=1, rounds=1,
                point=index, points=WARMUP_MAX, target_n=30))
            entry = self._point(1, index, 30, hold_s=WARMUP_HOLD_S, rest_seconds=WARMUP_REST_S, allow_skip=False)
            self.warmup_records.append(entry)
            check = warmup_convergence(self.warmup_records)
            self.quality['warmup'] = check
            self.checkpoint(list(self.records))
            self.progress(dict(phase='quality', stage=self.stage, message=f'热身{index}次：'+check['reason']))
            if check['passed']:
                return
        raise FeedbackControlError(f'热身{WARMUP_MAX}次仍未收敛，已释放；保留原K表，请检查材料恢复条件')

    def _targeted_retest(self):
        self._stage('retest')
        grouped = {t: [e for e in self.records if e['target_force_n'] == t] for t in self.targets}
        metadata = dict(format_version=2, calibration_profile=HIGH_FORCE_PROFILE, force_unit='N',
            feature='ch4567_signal_sum', pressure_reference='final_force_n', rounds=self.rounds,
            point_count=len(self.targets), source_path='', created_at=datetime.now().astimezone().isoformat(),
            required_sensor_state='CH4_CH7_ALL_FLOATING', interpolation='piecewise_linear')
        forced = set(self.historical_targets)
        saved_passes = deepcopy(self.quality.get('passes', []))
        self.quality['passes'] = deepcopy(saved_passes)
        self.quality.pop('passed', None)
        for pass_index in range(1, MAX_RETEST_PASSES+1):
            try:
                candidate = paired_mapping(grouped, metadata)
                initial = assess_mapping(candidate, [e for rows in grouped.values() for e in rows], self.accuracy_percent)
                pending = set(initial['failed_targets']) | forced
            except ValueError as exc:
                initial = dict(passed=False, failed_targets=list(self.targets), message=str(exc))
                pending = set(self.targets)
            audit = dict(pass_index=pass_index, initial=initial, retest_targets=sorted(pending))
            if pass_index <= len(saved_passes):
                audit = deepcopy(saved_passes[pass_index-1])
                pending = set(audit['retest_targets'])
            if pass_index <= len(self.quality['passes']):
                self.quality['passes'][pass_index-1] = audit
            else:
                self.quality['passes'].append(audit)
            self.progress(dict(phase='quality', stage=self.stage, message=f'第{pass_index}批重点复测：'+
                ('、'.join(f'{t:g} N' for t in sorted(pending)) if pending else '内部误差合格，开始独立验收')))
            for index, target in enumerate(sorted(pending), 1):
                replacements = []
                for repeat in range(1, RETEST_REPEATS+1):
                    previous = next((e for e in self.retest_records if e['pass_index'] == pass_index
                        and e['target_force_n'] == target and e['round'] == repeat), None)
                    if previous is not None:
                        replacements.append(previous)
                        continue
                    self.progress(dict(phase='point', stage=self.stage, round=repeat, rounds=RETEST_REPEATS,
                        point=index, points=len(pending), target_n=target, pass_index=pass_index))
                    entry = dict(self._point(repeat, index, target, pass_index=pass_index), pass_index=pass_index)
                    replacements.append(entry)
                    self.retest_records.append(entry)
                    self.checkpoint(list(self.records))
                grouped[target] = replacements
            forced = set()
            try:
                candidate = paired_mapping(grouped, metadata)
                self.candidate_document = candidate
                internal = assess_mapping(candidate, [e for rows in grouped.values() for e in rows], self.accuracy_percent)
            except ValueError as exc:
                audit['fit_error'] = str(exc)
                forced = set(self.targets)
                continue
            audit['internal'] = internal
            audit.pop('fit_error', None)
            if not internal['passed']:
                forced = set(internal['failed_targets'])
                self.checkpoint(list(self.records))
                continue
            # 冻结候选表后逐点独立验收，验收样本不回填到该表，避免自证准确。
            # 已失败的完整验收不重复，继续下一批；中断批次改用本次完整的新样本。
            if audit.get('validation', {}).get('passed') is False:
                forced = set(audit['validation']['failed_targets'])
                continue
            audit.pop('validation', None)
            audit['validation_session'] = self.resume_generation
            validation = []
            for index, target in enumerate(self.targets, 1):
                self.progress(dict(phase='point', stage='validation', round=pass_index, rounds=MAX_RETEST_PASSES,
                    point=index, points=len(self.targets), target_n=target))
                entry = dict(self._point(pass_index, index, target, phase_stage='validation',
                                        pass_index=pass_index), stage='validation', pass_index=pass_index,
                             validation_session=self.resume_generation)
                validation.append(entry)
                self.validation_records.append(entry)
                self.checkpoint(list(self.records))
            assessment = assess_mapping(candidate, validation, self.accuracy_percent)
            audit['validation'] = assessment
            self.progress(dict(phase='quality', stage=self.stage,
                message=f'独立验收最大误差{assessment["max_error_n"]:.3f} N，'+('通过' if assessment['passed'] else '未通过，先释放后针对失败点补测')))
            if assessment['passed']:
                self.quality.update(passed=True, accuracy_percent=self.accuracy_percent, minimum_error_n=.05)
                candidate['metadata'].update(quality=self.quality, independent_validation=True,
                    acquisition_row_count=len(self.records), retest_row_count=len(self.retest_records),
                    validation_row_count=len(self.validation_records))
                return candidate
            forced = set(assessment['failed_targets'])
            self.checkpoint(list(self.records))
        self.quality['passed'] = False
        raise FeedbackControlError(f'完成{MAX_RETEST_PASSES}批针对性复测仍未通过精度验收，已释放，保留原K表')

    def _recover(self, reason):
        self.progress(dict(phase='recovery', message=f'{reason}；先回原释放点'))
        started = self.clock()
        self.release_status = recover_release(self.motor, self.profile, self.pressure, self.settings, self.release,
            self.recovery_cancel, progress=self.progress, on_motor=self.on_motor, clock=self.clock,
            wait=self.recovery_wait, controller_factory=self.factory)
        entry = dict(self.point_context, reason=str(reason), elapsed_s=self.clock()-started,
                     **self.release_status)
        previous = self.recoveries[-1] if self.recoveries else None
        if (previous and previous['reason'] == entry['reason']
                and all(previous.get(k) == v for k, v in self.point_context.items())):
            # 同一点等待期间的持续断流合并计数，避免无限等待积累重复大记录。
            entry['elapsed_s'] += previous['elapsed_s']
            entry['repeat_count'] = previous.get('repeat_count', 1)+1
            self.recoveries[-1] = entry
        else:
            self.recoveries.append(entry)
        if not self.release_status['release_position_confirmed']:
            raise FeedbackControlError(self.release_status['message'])

    def _wait_retry_ready(self, attempt):
        """已释放后保留进度等待恢复；连续失败增加不可跳过的冷却，不结束任务。"""
        started = self.clock()
        cooldown = RETRY_COOLDOWN_SECONDS if attempt >= RETRY_COOLDOWN_AFTER else 1.0
        entry = dict(self.point_context, requested_s=cooldown, elapsed_s=0, completed=False)
        self.retry_waits.append(entry)
        self.checkpoint(list(self.records))
        stable_since, last_keys, count = None, None, 0
        try:
            while True:
                if self.cancel.is_set() or self.recovery_cancel.is_set():
                    raise FeedbackControlError('已停止标定，保留已完成进度')
                remaining = max(0, started+cooldown-self.clock())
                self.progress(dict(phase='retry_wait', **self.point_context,
                    message=f'已释放，当前点已失败{attempt}次；保留进度等待恢复'
                            + (f'，冷却剩余{math.ceil(remaining)}秒（不可跳过）' if remaining else '，等待连续卸载反馈')))
                try:
                    feedback, force, _, pressure_sequence, sensor, selected = self._check()
                    if abs(float(feedback['position_counts'])-self.release) > self.profile.tolerance(256):
                        raise FeedbackControlError('恢复等待期间位置偏离释放点')
                    ready = (abs(force) <= .02 and max(selected) == 0
                             and all(sensor['state'][c] in RELEASE_READY_STATES for c in CHANNELS))
                    keys = (float(feedback['sampled_monotonic']), pressure_sequence, sensor['sequence'])
                    fresh = last_keys is None or (keys[0] > last_keys[0]
                        and keys[1] != last_keys[1] and keys[2] != last_keys[2])
                    if not ready or not fresh:
                        stable_since, count = None, 0
                    else:
                        if stable_since is None:
                            stable_since = self.clock()
                        count += 1
                        if remaining <= 0 and count >= 3 and self.clock()-stable_since >= .5-1e-9:
                            entry['completed'] = True
                            return
                    last_keys = keys
                except RuntimeError as exc:
                    if self.cancel.is_set() or self.recovery_cancel.is_set():
                        raise
                    # 缺反馈不放行下一次下压；位置/电机异常仍先执行同一安全回位流程。
                    self._recover(str(exc))
                    stable_since, last_keys, count = None, None, 0
                self.cancel.wait(.1)
        finally:
            entry['elapsed_s'] = self.clock()-started
            self.checkpoint(list(self.records))

    def _point(self, cycle, index, target, hold_s=0, rest_seconds=None, allow_skip=True,
               phase_stage=None, pass_index=None):
        attempt = 0
        while True:
            attempt += 1
            self.point_context = dict(stage=phase_stage or self.stage, round=cycle, point=index,
                                      target_n=target, attempt=attempt, pass_index=pass_index)
            entry = None
            try:
                if self.cancel.is_set() or self.recovery_cancel.is_set():
                    raise FeedbackControlError('用户停止标定')
                if self._press_count or attempt > 1:
                    self._rest(cycle, index, seconds=rest_seconds, allow_skip=allow_skip)
                else:
                    self._collect()
                self.release_status = {}
                self._press_count += 1
                self._control(target, hold_s=hold_s)
                entry = dict(self._collect(target), round=cycle, point=index, target_force_n=target,
                             attempt=attempt, stage=self.stage, hold_s=hold_s)
                # 显示完整采样窗口的结果；释放期间压力归零不应覆盖刚采得的数值。
                self.progress(dict(phase='sample_result', sample=dict(entry), sample_state='captured',
                                   **self.point_context))
                self._control(target, release_only=True)
                self._collect()
                self.release_status = dict(release_position_confirmed=True, pressure_unloaded=True, stop_confirmed=True)
                self.progress(dict(phase='sample_result', sample=dict(entry), sample_state='released',
                                   **self.point_context))
                return entry
            except Exception as exc:
                if entry is not None:
                    self.progress(dict(phase='sample_result', sample=dict(entry), sample_state='discarded',
                                       **self.point_context))
                self.point_failures.append(dict(self.point_context, reason=str(exc),
                    occurred_at=datetime.now().astimezone().isoformat(),
                    feedback=[dict(row) for row in self._motor_trace]))
                self._recover(str(exc))
                self.checkpoint(list(self.records))
                if self.cancel.is_set() or self.recovery_cancel.is_set():
                    raise FeedbackControlError('已停止标定，已回原释放点') from exc
                # 编程/数据结构错误不能无限重试；运行时硬件/采样异常在释放后等待恢复。
                if not isinstance(exc, RuntimeError):
                    raise
                self._wait_retry_ready(attempt)
                self.progress(dict(phase='retry', attempt=attempt+1,
                    message=f'已确认恢复，准备重试当前点（第{attempt+1}次，已完成点保留）：{exc}'))

    def _rest(self, cycle, point, seconds=None, allow_skip=True):
        """仅跳过当前休息计时，下一次按压前仍重新核验卸载和新鲜反馈。"""
        started = self.clock()
        seconds = self.rest_seconds if seconds is None else seconds
        deadline = started+seconds
        wait_id = self.rest_control.begin() if allow_skip else None
        entry = dict(round=cycle, point=point, requested_s=seconds, stage=self.stage,
                     elapsed_s=0.0, skipped=False, completed=False)
        try:
            while True:
                feedback, force, _, _, _, _ = self._check()
                if (abs(float(feedback['position_counts'])-self.release) > self.profile.tolerance(256)
                        or abs(force) > .02):
                    raise FeedbackControlError('材料休息期间未保持释放位置或卸载状态')
                remaining = min(seconds, max(0.0, deadline-self.clock()))
                self.progress(dict(phase='rest', rest_id=wait_id, rest_remaining_s=remaining, force_n=force))
                if remaining <= 0 or allow_skip and self.rest_control.skip_requested(wait_id):
                    entry.update(skipped=remaining > 0, completed=True)
                    break
                if self.cancel.wait(min(.1, remaining)):
                    raise FeedbackControlError('用户停止标定')
        finally:
            if allow_skip:
                self.rest_control.end(wait_id)
            entry['elapsed_s'] = self.clock()-started
            self.rest_periods.append(entry)
            self.progress(dict(phase='rest_finished', rest_id=wait_id, skipped=entry['skipped']))
        self._collect()

    def _check(self):
        if self.cancel.is_set():
            raise FeedbackControlError('用户停止标定')
        feedback = self.motor.read_feedback()
        self.on_motor(feedback)
        now = self.clock()
        if not 0 <= now-float(feedback['sampled_monotonic']) <= .5:
            raise FeedbackControlError('电机反馈过期')
        current = float(feedback['phase_current_ma'])
        if not math.isfinite(current) or abs(current) > self.settings.max_current_ma:
            raise FeedbackControlError('电流反馈无效或超限')
        position = float(feedback['position_counts'])
        if (not math.isfinite(position) or position < self.release-self.settings.max_travel
                or position > self.release+self.profile.tolerance(256)):
            raise FeedbackControlError('电机位置超出允许行程')
        force, stamp, sequence = self.pressure.read(now, allow_negative=True)
        if force > self.settings.force_limit_n:
            raise FeedbackControlError('压力超过保护上限')
        speed = float(feedback['measured_speed_rpm'])
        if (not math.isfinite(speed) or abs(speed) > 1 or not feedback.get('in_position')
                or int(feedback['running_state']) not in (0, 1)):
            raise FeedbackControlError('采集期间电机未停止到位')
        sample = self.sensor() or {}
        if (not 0 <= float(sample.get('age_seconds', math.inf)) <= .5
                or len(sample.get('signal', ())) < 8 or len(sample.get('state', ())) < 8):
            raise FeedbackControlError('传感器反馈失效或不足四个目标通道')
        selected = [float(sample['signal'][c]) for c in CHANNELS]
        if any(not math.isfinite(v) or v < 0 for v in selected):
            raise FeedbackControlError('Signal采样无效')
        return feedback, force, stamp, sequence, sample, selected

    def _collect(self, target=None):
        started, stable_since, last_sensor, last_pressure = self.clock(), None, None, None
        signals, forces = [], []
        last_stamp = None
        while self.clock()-started < 30:
            feedback, force, pressure_stamp, pressure_sequence, sample, selected = self._check()
            now = self.clock()
            states = [sample['state'][c] for c in CHANNELS]
            ready = (all(s == STATE_FLOATING for s in states) if target is not None else
                     all(s in RELEASE_READY_STATES for s in states) and max(selected) == 0 and abs(force) <= .02)
            if target is None and abs(float(feedback['position_counts'])-self.release) > self.profile.tolerance(256):
                raise FeedbackControlError('未回到原释放点')
            if target is not None and abs(force-target) > max(.05, target*.10):
                # 寻压到位后的材料松弛允许10%窗口，K始终使用采集时实测压力。
                ready = False
            sequence = sample['sequence']
            sensor_stamp = now-float(sample['age_seconds'])
            ready = ready and sensor_stamp >= started and pressure_stamp >= started
            fresh = last_sensor is None or sequence > last_sensor
            if (not ready or last_sensor is not None and sequence < last_sensor
                    or last_stamp is not None and sensor_stamp-last_stamp > .2):
                stable_since, signals, forces = None, [], []
            if fresh:
                if ready:
                    if stable_since is None:
                        stable_since = sensor_stamp
                    signals.append(selected)
                last_sensor, last_stamp = sequence, sensor_stamp
            if ready and pressure_sequence != last_pressure:
                forces.append(force)
                last_pressure = pressure_sequence
            duration = 0 if stable_since is None else sensor_stamp-stable_since
            self.progress(dict(phase='capture' if target is not None else 'release_wait',
                               force_n=force, floating_count=states.count(STATE_FLOATING), stable_s=duration))
            if ready and len(signals) >= 3 and len(forces) >= 3 and duration >= (1 if target is not None else .2):
                medians = [statistics.median(frame[i] for frame in signals) for i in range(4)]
                if target is not None and sum(medians) <= 0:
                    raise FeedbackControlError('四通道Signal合力为零，不能计算K值')
                # FLOATING不是材料稳定证明；压力在窗口内仍明显变化时重新计时。
                pressure_span = max(forces)-min(forces)
                if target is not None and pressure_span > max(.03, target*.01):
                    stable_since, signals, forces = None, [], []
                    continue
                return dict(signals=medians, signal_sum=sum(medians), final_force_n=statistics.median(forces),
                            floating_count=4, sample_count=len(signals), capture_seconds=duration,
                            pressure_sample_count=len(forces), pressure_span_n=pressure_span,
                            pressure_min_n=min(forces), pressure_max_n=max(forces),
                            captured_at=datetime.now().astimezone().isoformat())
            if self.cancel.wait(.05):
                raise FeedbackControlError('用户停止标定')
        raise FeedbackControlError('等待四通道FLOATING或释放恢复超时')

    def _control(self, target, release_only=False, hold_s=0):
        settings = replace(self.settings, target_n=target,
                           tolerance_n=pressure_tolerance_n(target, self.tolerance_percent),
                           hold_s=hold_s or self.settings.hold_s)
        controller = self.factory(self.motor, self.profile, self.pressure, self.cancel, settings, self.release,
            learned_map=self.learned_map, on_motor=self.on_motor,
            on_record=lambda row: self.progress(dict(row, phase='release' if release_only else
                'warmup_hold' if hold_s and row.get('hold_elapsed_s') is not None else 'press')), clock=self.clock)
        result = controller.run(release_only=release_only, acquire_only=not release_only and not hold_s)
        if result.get('motion_diagnostics'):
            self.motion_diagnostics.append(dict(self.point_context,
                operation='release' if release_only else 'press',
                recorded_at=datetime.now().astimezone().isoformat(), events=result['motion_diagnostics']))
            # 在当前点释放或异常回位后随检查点保存，不能承压等待文件系统恢复。
        if not result.get('ok'):
            raise FeedbackControlError(result.get('message', '压力控制失败'))

    def run(self):
        started = self.clock()
        self._run_started = started
        round_started = None
        if self.resume_generation:
            self.resume_sessions.append(dict(generation=self.resume_generation,
                started_at=datetime.now().astimezone().isoformat(), elapsed_s=0))
        try:
            self._warmup()
            if len(self.records) < len(self.targets)*self.rounds:
                self._stage('formal')
            for cycle in range(1, self.rounds+1):
                done = sum(e['round'] == cycle for e in self.records)
                if done == len(self.targets):
                    continue
                if self.cancel.is_set():
                    raise FeedbackControlError('用户停止标定')
                timing = next((t for t in self.round_timings if t['round'] == cycle), None)
                if timing is None:
                    timing = dict(round=cycle, started_at=datetime.now().astimezone().isoformat(),
                                  elapsed_s=0.0, completed=False, completed_points=done)
                    self.round_timings.append(timing)
                round_started = self.clock()-timing['elapsed_s']
                for index, target in enumerate(self.targets, 1):
                    if index <= done:
                        continue
                    self.progress(dict(phase='point', stage=self.stage, round=cycle, rounds=self.rounds,
                                       point=index, points=len(self.targets), target_n=target))
                    entry = self._point(cycle, index, target)
                    timing.update(elapsed_s=self.clock()-round_started, completed_points=index,
                                  completed=index == len(self.targets))
                    entry['round_elapsed_s'] = timing['elapsed_s']
                    if timing['completed']:
                        timing['ended_at'] = datetime.now().astimezone().isoformat()
                        self.progress(dict(phase='round_complete', round_timings=[dict(t) for t in self.round_timings]))
                    self.records.append(entry)
                    self.checkpoint(list(self.records))
                round_started = None
            if self.cancel.is_set():
                raise FeedbackControlError('用户停止标定')
            document = self._targeted_retest()
            if self.cancel.is_set():
                raise FeedbackControlError('用户停止标定')
            result = dict(ok=True, records=self.records, document=document,
                        message='热身、正式标定及独立精度验收通过，已回到释放点')
        except Exception as exc:
            result = dict(ok=False, cancelled=self.cancel.is_set(), records=self.records, message=str(exc))
            # 记录写入、汇总等外层异常也必须走回位；已急停则不恢复运动。
            if not self.release_status.get('release_position_confirmed') and not self.recovery_cancel.is_set():
                try:
                    self._recover(str(exc))
                except Exception as recovery_exc:
                    result['message'] += f'；{recovery_exc}'
            result['message'] += '；' + self.release_status.get('message',
                '已确认原释放位置及停止' if self.release_status.get('release_position_confirmed') else '未确认回位')
            if round_started is not None and self.round_timings and not self.round_timings[-1]['completed']:
                self.round_timings[-1].update(elapsed_s=self.clock()-round_started,
                    ended_at=datetime.now().astimezone().isoformat())
        result['round_timings'] = [dict(t) for t in self.round_timings]
        result['elapsed_s'] = self.elapsed_seconds()
        if self.resume_generation:
            self.resume_sessions[-1].update(elapsed_s=self.clock()-started,
                ended_at=datetime.now().astimezone().isoformat(), completed=result['ok'])
        result['rest_seconds'] = self.rest_seconds
        result['rest_periods'] = [dict(period) for period in self.rest_periods]
        result['recoveries'] = list(self.recoveries)
        self._update_stage_time()
        if self._stage_started is not None:
            self.stage_timings[-1]['completed'] = result['ok']
        result.update(self.audit())
        result.update({key: value for key, value in self.release_status.items() if key != 'message'})
        result.setdefault('stop_confirmed', False)
        if result.get('document'):
            result['document']['metadata'].update(round_timings=result['round_timings'],
                elapsed_s=result['elapsed_s'], rest_seconds=self.rest_seconds, rest_periods=result['rest_periods'],
                stage_timings=result['stage_timings'], recoveries=result['recoveries'])
        return result
