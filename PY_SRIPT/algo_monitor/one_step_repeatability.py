"""独立One Step测试：默认压力优先接近与保持，保留固定位置重复性实验。"""
from dataclasses import asdict, replace
from collections import deque
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

from high_force_calibration import CombinedCancel, recover_release
from one_step_learning import validate_document
from pressure_feedback_control import PressureFeedbackController, FeedbackControlError
from motor_speed_units import valid_rpm
from force_signal_pressure_calibration import _write_json_atomic

KIND = 'one_step_position_repeatability'


def learning_identity(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, ensure_ascii=False,
                                    allow_nan=False).encode('utf-8')).hexdigest()


class PositionTrialController(PressureFeedbackController):
    def __init__(self, *args, node, fast_speed, observation_s, seek_max_step=64, on_arrival=lambda row: None, **kwargs):
        super().__init__(*args, **kwargs)
        self.node, self.fast_speed, self.observation_s = node, fast_speed, observation_s
        self.on_arrival = on_arrival
        self.seek_max_step = seek_max_step
        self.arrival = None
        self.correction_steps = 0
        self.correction_net_steps = 0
        self.last_force = self.last_position = None
        self.position_arrival_s = None
        # 与共享电机回位反馈精度一致；指令目标不变，实测偏差单独记录。
        self.position_tolerance = self.profile.tolerance(256)
        self._micro_confirm_since = self._micro_confirm_last = None

    def _poll(self):
        result = super()._poll()
        self.last_position, self.last_force = result[0]['position_counts'], result[1]
        return result

    def _move(self, delta, *args, **kwargs):
        # 只统计进入闭环后的修正指令，直达与释放不混入微调步数。
        count = self.move_count
        self._micro_confirm_since = self._micro_confirm_last = None
        try:
            return super()._move(delta, *args, **kwargs)
        finally:
            if self.move_count > count:
                self.correction_steps += abs(int(delta))
                self.correction_net_steps += int(delta)

    def _position_move_complete(self, sample, origin, target, delta, stable):
        if super()._position_move_complete(sample, origin, target, delta, stable):
            return True
        # 1～2 step指令可能落在整数位置量化区间内；必须连续确认静止，不能等虚假的位移。
        stationary = (abs(delta) <= 2 and sample['position_counts'] == origin
                      and abs(sample['measured_speed_rpm']) <= 1 and sample.get('in_position')
                      and int(sample['running_state']) in (0, 1))
        if not stationary:
            self._micro_confirm_since = self._micro_confirm_last = None
            return False
        stamp = sample['sampled_monotonic']
        if self._micro_confirm_last is None or stamp > self._micro_confirm_last:
            self._micro_confirm_since = stamp if self._micro_confirm_since is None else self._micro_confirm_since
            self._micro_confirm_last = stamp
            if stamp-self._micro_confirm_since >= .15-1e-9:
                self._motion_event('微步指令后反馈位置未跨整数格，连续确认静止；按新压力重新判断，不重发原指令')
                return True
        return False

    def _adjustment_steps(self, error, force):
        if self.hold_started is not None or abs(error) <= max(.1, 2*self.settings.tolerance_n):
            return super()._adjustment_steps(error, force)
        # 尚未到压且误差明显时，用独立寻压上限；进入目标附近后恢复保持小步幅。
        original = self.settings
        try:
            self.settings = replace(original, max_step=self.seek_max_step)
            size = super()._adjustment_steps(error, force)
        finally:
            self.settings = original
        if self._response_slopes:
            pressure_budget = min(abs(error)*.65, max(.1, min(1.0, original.target_n*.25)))
            size = min(size, max(1, math.floor(pressure_budget/max(self._response_slopes))))
        self._last_adjustment_size = size
        return size

    def _fast_approach(self):
        """按学习记录发一次位置命令；不套用旧接近阶段的80%压力提前停止。"""
        self.phase = '位置直达'
        sample, _, stamp, _ = self._poll()
        origin, target = int(sample['position_counts']), self.node['position_steps']
        if (abs(float(sample['measured_speed_rpm'])) > 1 or not sample.get('in_position')
                or int(sample['running_state']) not in (0, 1)):
            raise FeedbackControlError('直达前电机未静止')
        if not self.release-self.settings.max_travel <= target < origin <= self.release+self.release_tolerance:
            raise FeedbackControlError('学习位置不在本次允许的下压范围内')
        if self.clock()-stamp > .15:
            raise FeedbackControlError('直达要求压力反馈不超过0.15秒')
        self.motor.prepare_motion()
        sample, _, stamp, _ = self._poll()
        if (int(sample['position_counts']) != origin or abs(sample['measured_speed_rpm']) > 1
                or self.clock()-stamp > .15 or not sample.get('in_position')):
            raise FeedbackControlError('发令前位置、静止状态或压力新鲜度发生变化')
        self.command_steps = target-origin
        expiry = self.clock()+self.profile.motion_timeout(self.command_steps, self.fast_speed, self.settings.acceleration)
        self.motor.move_relative_steps(self.command_steps, speed_rpm=self.fast_speed,
            accel=self.settings.acceleration, retry_cancelled=self.cancel.is_set)
        self.move_count += 1
        self._stationary_samples.clear()
        self._stationary_ready = None
        stopped_since, last_stamp, stopped_position = None, None, None
        while self.clock() < expiry:
            sample, _, pressure_stamp, _ = self._poll()
            position = float(sample['position_counts'])
            if self.clock()-pressure_stamp > .15:
                raise FeedbackControlError('直达中压力反馈超过0.15秒')
            if position > origin+self.position_tolerance:
                raise FeedbackControlError(f'直达运动方向异常：起点{origin} step，实际{position:g} step')
            if position < target-self.position_tolerance:
                raise FeedbackControlError(f'直达超出允许位置偏差：目标{target} step，实际{position:g} step，'
                                           f'允许±{self.position_tolerance} step')
            stationary = (abs(position-target) <= self.position_tolerance and abs(sample['measured_speed_rpm']) <= 1
                          and sample.get('in_position') and int(sample['running_state']) in (0, 1))
            motor_stamp = sample['sampled_monotonic']
            if not stationary:
                stopped_since, stopped_position = None, None
            elif motor_stamp != last_stamp:
                if stopped_since is None or abs(position-stopped_position) > 1:
                    stopped_since, stopped_position = motor_stamp, position
                if motor_stamp-stopped_since >= .2-1e-9:
                    self.command_steps = 0
                    self.position_arrival_s = self.clock()-self.started
                    break
            last_stamp = motor_stamp
            self._wait()
        else:
            raise FeedbackControlError('记录位置到位超时；未重发位置命令')

        self.phase = '未微调采样'
        ready = self.clock()
        stationary_position = position
        window = []
        window_resets = 0
        deadline = ready + self.observation_s + 2
        while self.clock() < deadline:
            sample, force, stamp, sequence = self._poll()
            if abs(sample['position_counts']-target) > self.position_tolerance:
                raise FeedbackControlError('未微调观察时位置超出允许范围')
            if (abs(sample['position_counts']-stationary_position) > 2
                    or abs(sample['measured_speed_rpm']) > 1
                    or int(sample['running_state']) not in (0, 1) or not sample.get('in_position')):
                # 终点收敛期间丢弃未稳定窗口，在原有有限等待时间内重新采集，不因单帧立即中止。
                if not window_resets:
                    self._motion_event('未微调观察仍有终点收敛，清空窗口并等待真实静止')
                window_resets += 1
                stationary_position, ready = sample['position_counts'], self.clock()
                window.clear()
                self._wait()
                continue
            if stamp >= ready and (not window or sequence != window[-1]['sequence']):
                if window and (stamp <= window[-1]['stamp'] or stamp-window[-1]['stamp'] > .2+1e-9):
                    raise FeedbackControlError('未微调采样的压力数据不连续')
                window.append(dict(force_n=force, position_steps=sample['position_counts'], stamp=stamp, sequence=sequence))
            if len(window) >= 3 and window[-1]['stamp']-window[0]['stamp'] >= self.observation_s-1e-9:
                forces = [row['force_n'] for row in window]
                median = statistics.median(forces)
                self.arrival = dict(target_n=self.settings.target_n,
                    measurement_kind='position_observation',
                    learning_force_n=self.node['measured_force_n'],
                    recorded_position_steps=target, actual_position_steps=sample['position_counts'],
                    position_error_steps=sample['position_counts']-target,
                    position_tolerance_steps=self.position_tolerance,
                    observation_window_resets=window_resets,
                    first_force_n=forces[0], median_force_n=median,
                    min_force_n=min(forces), max_force_n=max(forces),
                    error_vs_learning_n=median-self.node['measured_force_n'],
                    error_vs_target_n=median-self.settings.target_n,
                    position_arrival_s=self.position_arrival_s, samples=window)
                self.on_arrival(dict(self.arrival))
                # 未微调窗口独立保存，保持阶段重新建立静止采样窗口。
                self._settle_until = self.clock()
                self._stationary_ready = None
                self._stationary_samples.clear()
                return
            self._wait()
        raise FeedbackControlError('未微调压力采样不足，未进入保持')


class PressurePriorityController(PositionTrialController):
    """学习表只决定保守接近点；压力预测制动、误差寻压和小步保持独立处理。"""
    def __init__(self, *args, learning_nodes, **kwargs):
        super().__init__(*args, **kwargs)
        self.learning_nodes = learning_nodes
        self._pressure_history = deque(maxlen=2)
        self._fast_segment = False
        self.braking_events = []

    def _poll(self):
        value = super()._poll()
        sample, force, stamp, sequence = value
        if self._fast_segment and self.clock()-stamp > .15:
            raise FeedbackControlError('快速接近压力反馈超过0.15秒')
        if not self._pressure_history or sequence != self._pressure_history[-1][2]:
            self._pressure_history.append((force, stamp, sequence))
        return value

    def _pressure_move_should_stop(self, delta, force, approach_force_n, stop_band, sample):
        if super()._pressure_move_should_stop(delta, force, approach_force_n, stop_band, sample):
            return True
        if delta >= 0 or len(self._pressure_history) < 2:
            return False
        previous, current = self._pressure_history
        elapsed = current[1]-previous[1]
        if not .02 <= elapsed <= .2:
            return False
        rise = max(0, (current[0]-previous[0])/elapsed)
        rpm = abs(sample['measured_speed_rpm'])
        brake_time = rpm*(256-self.settings.acceleration)*.000025
        horizon = min(.3, .08+max(0, self.clock()-current[1])+brake_time)
        limit = min(self.settings.target_n-stop_band,
                    approach_force_n if approach_force_n is not None else math.inf)
        predicted = force+rise*horizon
        if predicted >= limit:
            self.braking_events.append(dict(elapsed_s=self.clock()-self.started, force_n=force,
                predicted_force_n=predicted, stop_limit_n=limit, rise_n_per_s=rise,
                horizon_s=horizon, position_steps=sample['position_counts'], speed_rpm=sample['measured_speed_rpm']))
            return True
        return False

    def _fast_approach(self):
        self.phase = '压力优先：快速接近'
        sample, _, _, _ = self._poll()
        # 只走到约75%目标的学习位置；不足1 N时用首段受压斜率退让，不把空行程当线性区。
        approach_force = self.settings.target_n*.75
        nodes = self.learning_nodes[1:]
        if approach_force < nodes[0]['measured_force_n']:
            a, b = nodes[:2]
            slope = (b['measured_force_n']-a['measured_force_n'])/(a['position_steps']-b['position_steps'])
            position = a['position_steps']+(a['measured_force_n']-approach_force)/slope
        else:
            position = nodes[-1]['position_steps']
            for a, b in zip(nodes, nodes[1:]):
                if a['measured_force_n'] <= approach_force <= b['measured_force_n']:
                    fraction = (approach_force-a['measured_force_n'])/(b['measured_force_n']-a['measured_force_n'])
                    position = a['position_steps']+(b['position_steps']-a['position_steps'])*fraction
                    break
        position = max(self.release-self.settings.max_travel, min(self.release, math.ceil(position)))
        delta = position-int(sample['position_counts'])
        if delta < 0:
            self._fast_segment = True
            try:
                # 接近段上限30 RPM；接触力可提前结束这条命令，不强行走完学习位置。
                PressureFeedbackController._move(self, delta, approach_force_n=approach_force,
                    speed_override=min(30, self.fast_speed))
            finally:
                self._fast_segment = False
        self.position_arrival_s = self.clock()-self.started

    def _adjustment_steps(self, error, force):
        if self.hold_started is not None or abs(error) <= max(.1, 2*self.settings.tolerance_n):
            return super()._adjustment_steps(error, force)
        original = self.settings
        try:
            self.settings = replace(original, max_step=self.seek_max_step)
            size = PressureFeedbackController._adjustment_steps(self, error, force)
        finally:
            self.settings = original
        # 远离目标时加快追赶；接近目标时按剩余误差收缩，不能照最大步数固定补压。
        if self._response_slopes:
            budget = min(abs(error)*.65, max(.1, min(3.0, original.target_n*.2)))
            size = min(size, max(1, math.floor(budget/max(self._response_slopes))))
        self._last_adjustment_size = size
        return size

    def _move(self, delta, *args, **kwargs):
        # 目标附近按刚度和剩余压力限速，压力高的刚性区更早降速。
        force = self.last_force if self.last_force is not None else 0
        error = abs(self.settings.target_n-force)
        slope = max(self._response_slopes, default=.04)
        safe_rpm = max(1, math.floor(60*max(.02, error*.4)/(self.profile.steps_per_turn*slope*.15)))
        kwargs['speed_override'] = min(self.settings.speed, safe_rpm, 10)
        return super()._move(delta, *args, **kwargs)

    def _regulate(self, stop_on_stable=False):
        acquired = PressureFeedbackController._regulate(self, stop_on_stable=True)
        force, position = acquired['actual_force_n'], acquired['actual_position_steps']
        frames = list(self._stationary_samples)
        values = [r[0] for r in frames] or [force]
        self.arrival = dict(measurement_kind='pressure_arrival', target_n=self.settings.target_n,
            learning_force_n=self.node['measured_force_n'], recorded_position_steps=self.node['position_steps'],
            actual_position_steps=position, position_error_steps=position-self.node['position_steps'],
            median_force_n=force, min_force_n=min(values), max_force_n=max(values), first_force_n=values[0],
            error_vs_learning_n=force-self.node['measured_force_n'], error_vs_target_n=force-self.settings.target_n,
            pressure_arrival_s=self.stable_arrival_s, position_arrival_s=self.position_arrival_s,
            samples=[dict(force_n=f, stamp=t, sequence=s) for f,t,s in frames])
        self.on_arrival(dict(self.arrival))
        if stop_on_stable:
            return acquired
        self.hold_started = self.clock()
        self.deadline = self.hold_started+self.settings.hold_s+5
        return PressureFeedbackController._regulate(self)


def run_trial(motor, profile, samples, stop, emergency, settings, origin, document, target,
              *, mode='pressure', fast_speed=30, observation_s=.5, seek_max_step=64, progress=lambda row: None,
              on_arrival=lambda row: None, on_motor=lambda row: None, clock=time.monotonic):
    """普通停止不取消恢复；任何阶段异常都先回固定起点，急停除外。"""
    started = clock()
    if mode not in ('pressure', 'position', 'cycles'):
        raise ValueError('未知One Step控制模式')
    if mode == 'cycles':
        from one_step_relearning import validate_document as validate_cycles, CycleSettings, CyclePositionController
        validate_cycles(document, profile)
        cycle_settings = CycleSettings(**document['cycle_settings'])
        if not cycle_settings.trajectory_learning and settings.acceleration != document['settings']['acceleration']:
            raise ValueError('独立重压位置仅适用于学习时的加速度，请恢复设置或重新学习')
        fast_speed, observation_s = cycle_settings.fast_speed, cycle_settings.observation_s
    else:
        validate_document(document, profile)
    if not document['complete'] or type(target) is not int or not 1 <= target <= 30:
        raise ValueError('直达需要完整有效学习表及1～30 N整数目标')
    if origin != document['origin_position_steps']:
        raise ValueError('当前起点与学习表坐标不一致，请核对坐标')
    speed_step = cycle_settings.speed_step_rpm if mode == 'cycles' else (.1 if profile.model == 'zdt_x42s' else 1)
    if (not valid_rpm(fast_speed, minimum=speed_step, maximum=min(400, profile.speed_max), step=speed_step)
            or not math.isfinite(observation_s) or not .2 <= observation_s <= 5):
        raise ValueError('直达速度或观察时间超出范围')
    settings = replace(settings, target_n=target)
    settings.validate(profile)
    if not 1 <= settings.max_step <= 16:
        raise ValueError('保持阶段每次最多1～16 step')
    if type(seek_max_step) is not int or not 1 <= seek_max_step <= 128:
        raise ValueError('寻压阶段每次最多1～128 step')
    node = dict(document['nodes'][target])
    if mode == 'cycles':
        cycle_settings = replace(cycle_settings, medium_speed=node.get('medium_speed_rpm', cycle_settings.medium_speed))
        if cycle_settings.trajectory_learning:
            settings = replace(settings, tolerance_n=node['tolerance_n'])
    trace, controller = [], None
    def record(row):
        trace.append(dict(row))
        progress(row)
    result = dict(ok=False, message='未开始直达')
    cancelled = CombinedCancel(stop, emergency)
    reference_origin, reference_settings = origin, settings
    near_release = mode == 'cycles' and cycle_settings.near_contact_release
    recovery_origin = origin
    initial_positioning_s = 0.0
    startup_unload_checks = []
    working_release_invalidated = False
    if near_release:
        from one_step_working_release import (working_settings, establish_working_release, confirm_unloaded,
                                             WORKING_POSITION_TOLERANCE_STEPS, WorkingReleaseSettlingTimeout)
        origin = document['working_release_position_steps']
        settings = working_settings(settings, reference_origin, origin)
    try:
        if near_release:
            checker = PressureFeedbackController(motor, profile, samples, cancelled, reference_settings, reference_origin,
                on_motor=on_motor, clock=clock)
            sample = checker._poll()[0]
            if sample['position_counts'] > origin+profile.tolerance(256):
                # 历史表首次使用时单独低速就位；不能用“释放”命令向下移动。
                positioning_started = clock()
                initial = recover_release(motor, profile, samples, reference_settings, reference_origin, emergency,
                    progress=record, on_motor=on_motor, clock=clock, return_even_within_tolerance=True)
                if not initial['release_position_confirmed']:
                    raise FeedbackControlError(initial['message'])
                check = establish_working_release(motor, profile, samples, cancelled, reference_settings, reference_origin,
                    document['contact'], [], progress=record, on_motor=on_motor, clock=clock)
                if check['position_steps'] != origin:
                    raise FeedbackControlError('工作释放点与历史表不一致，需要重新学习')
                initial_positioning_s = clock()-positioning_started
            recovery_origin = origin
        release = recover_release(motor, profile, samples, settings, origin, emergency,
            progress=record, on_motor=on_motor, clock=clock, return_even_within_tolerance=near_release,
            position_tolerance_steps=WORKING_POSITION_TOLERANCE_STEPS if near_release else None,
            release_side_tolerance_steps=profile.tolerance(256) if near_release else None)
        if release.get('release_position_outside_tolerance'):
            working_release_invalidated, recovery_origin = True, reference_origin
        if not release['release_position_confirmed']:
            raise FeedbackControlError(release['message'])
        # 仅核验起点和卸载，确认后立即按压，不插入材料休息计时。
        checker = PressureFeedbackController(motor, profile, samples, cancelled, settings, origin,
            on_motor=on_motor, clock=clock)
        sample, force, _, _ = checker._poll()
        if (abs(sample['position_counts']-origin) > checker.release_tolerance
                or abs(sample['measured_speed_rpm']) > 1 or not sample.get('in_position')
                or int(sample['running_state']) not in (0, 1) or not near_release and abs(force) > .02+1e-9):
            raise FeedbackControlError('起点未静止或压力未卸载至±0.02 N，取消本次按压')
        if near_release:
            check = confirm_unloaded(checker, origin, startup_unload_checks, phase='直达启动前卸载复核')
            if not check['unloaded']:
                working_release_invalidated, recovery_origin = True, reference_origin
                raise FeedbackControlError('直达启动前连续复核仍有残余压力，回原释放点，需要重新学习')
        controller_type = PressurePriorityController if mode == 'pressure' else PositionTrialController
        extra = dict(learning_nodes=document['nodes']) if mode == 'pressure' else {}
        if mode == 'cycles':
            controller_type = CyclePositionController
            extra = dict(contact=document['contact'], cycle_settings=cycle_settings, measure_only=False)
            if cycle_settings.trajectory_learning:
                from one_step_trajectory import TrajectoryController, Trajectory
                controller_type = TrajectoryController
                extra['trajectory'] = Trajectory(**node['trajectory'])
        approach_args = {} if mode == 'cycles' else dict(fast_speed=fast_speed, observation_s=observation_s)
        controller = controller_type(motor, profile, samples, cancelled, settings, origin,
            node=node, seek_max_step=seek_max_step, **approach_args,
            on_arrival=on_arrival,
            on_record=record, on_motor=on_motor, clock=clock, **extra)
        # 使用学习表局部刚度作为保持初值，不参与直达位置计算。
        left, right = document['nodes'][max(1, target-1)], document['nodes'][min(30, target+1)]
        if left['position_steps'] != right['position_steps']:
            controller._response_slopes.append((right['measured_force_n']-left['measured_force_n']) /
                                               (left['position_steps']-right['position_steps']))
        result = controller.run()
        if near_release and controller.guard_reason == 'working_release_not_unloaded':
            working_release_invalidated, recovery_origin = True, reference_origin
    except Exception as exc:
        result = dict(ok=False, message=str(exc))
    finally:
        release_settings = reference_settings if recovery_origin == reference_origin else settings
        release = recover_release(motor, profile, samples, release_settings, recovery_origin, emergency,
            progress=record, on_motor=on_motor, clock=clock, return_even_within_tolerance=near_release,
            position_tolerance_steps=WORKING_POSITION_TOLERANCE_STEPS if near_release and recovery_origin == origin else None,
            release_side_tolerance_steps=profile.tolerance(256) if near_release and recovery_origin == origin else None)
        if release.get('release_position_outside_tolerance'):
            result.update(ok=False, message=release['message'])
            working_release_invalidated, recovery_origin = True, reference_origin
            release = recover_release(motor, profile, samples, reference_settings, reference_origin, emergency,
                progress=record, on_motor=on_motor, clock=clock, return_even_within_tolerance=True)
        if near_release and recovery_origin == origin and release['release_position_confirmed'] and not emergency.is_set():
            checker = PressureFeedbackController(motor, profile, samples, emergency, settings, origin,
                on_motor=on_motor, on_record=record, clock=clock)
            checker.releasing = True
            try:
                checks = result.setdefault('working_release_checks', [])
                check = confirm_unloaded(checker, origin, checks)
                result['working_release_check'] = check
                release['pressure_unloaded'] = check['unloaded']
                if not check['unloaded']:
                    working_release_invalidated = True
                    result.update(ok=False, working_release_invalidated=True,
                                  message='工作释放点残余压力超出±0.02 N，回原释放点并停用此表，需要重新学习')
                    recovery_origin = reference_origin
                    release = recover_release(motor, profile, samples, reference_settings, reference_origin, emergency,
                        progress=record, on_motor=on_motor, clock=clock, return_even_within_tolerance=True)
            except Exception as exc:
                result.update(ok=False, message=f'已按位置回位，卸载尚未确认：{exc}')
                release['pressure_unloaded'] = False
                if isinstance(exc, WorkingReleaseSettlingTimeout):
                    working_release_invalidated, recovery_origin = True, reference_origin
                    release = recover_release(motor, profile, samples, reference_settings, reference_origin, emergency,
                        progress=record, on_motor=on_motor, clock=clock, return_even_within_tolerance=True)
    result.update(kind=KIND, format_version=1, created_at=datetime.now().astimezone().isoformat(),
        profile=profile.metadata(), namespace=profile.namespace, origin_position_steps=reference_origin,
        working_release_position_steps=origin if near_release else None, last_release_position_steps=recovery_origin,
        initial_positioning_s=initial_positioning_s,
        working_release_invalidated=working_release_invalidated,
        startup_unload_checks=startup_unload_checks,
        trajectory_startup_unload_checks=getattr(controller, 'startup_unload_checks', []),
        learning_id=learning_identity(document), learning_created_at=document['created_at'],
        learning_node=node, target_n=target, settings=asdict(settings), fast_speed_rpm=fast_speed,
        effective_fast_speed_rpm=(node['trajectory']['free_speed_rpm'] if node.get('trajectory') and mode == 'cycles' else
                                 document['contact']['effective_fast_speed_rpm'] if mode == 'cycles' else
                                 min(fast_speed, 30) if mode == 'pressure' else fast_speed),
        medium_speed_rpm=cycle_settings.medium_speed if mode == 'cycles' else None,
        trajectory=node.get('trajectory'),
        braking_events=getattr(controller, 'braking_events', []),
        observation_s=observation_s, rest_s=0, seek_max_step=seek_max_step, control_mode=mode,
        arrival=controller.arrival if controller else None,
        correction_command_steps=controller.correction_steps if controller else 0,
        correction_net_command_steps=controller.correction_net_steps if controller else 0,
        final_force_n=controller.last_force if controller and controller.hold_started is not None else None,
        last_observed_force_n=controller.last_force if controller else None,
        final_position_steps=controller.last_position if controller else None,
        cancelled=stop.is_set(), emergency_stopped=emergency.is_set(),
        elapsed_s=clock()-started, trace=trace)
    outcome_message = result.get('message', '')
    result.update(release)
    result['ok'] = bool(result['ok'] and release['release_position_confirmed'] and not cancelled.is_set())
    result['message'] = ('保持完成，已回起点' if result['ok'] else '本次测试结束；'+outcome_message+'；'+release['message'])
    return result


def save_trial(directory, result, path=None):
    if result.get('kind') != KIND:
        raise ValueError('不是One Step重复性测试记录')
    path = Path(path) if path else Path(directory)/('one_step_trial_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    _write_json_atomic(path, result)
    return path


def repeatability_summary(results, result):
    """隔离两种模式，只比较同表、同测试条件且回位确认的有效到位记录。"""
    keys = ('learning_id', 'target_n', 'fast_speed_rpm', 'observation_s', 'rest_s')
    matched = [r for r in results if r.get('arrival') and r.get('release_position_confirmed')
               and r.get('control_mode', 'position') == result.get('control_mode', 'position')
               and not r.get('emergency_stopped') and all(r.get(k) == result.get(k) for k in keys)
               and r.get('settings', {}).get('acceleration') == result.get('settings', {}).get('acceleration')]
    values = [r['arrival']['median_force_n'] for r in matched]
    if not values: return None
    return dict(count=len(values), mean_n=statistics.mean(values), min_n=min(values), max_n=max(values),
                stdev_n=statistics.stdev(values) if len(values) > 1 else None)
