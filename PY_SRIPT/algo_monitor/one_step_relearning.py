"""独立释放重压学习：可手动接触坐标、逐轮连续轨迹学习，兼容历史位置验证表。"""
from copy import deepcopy
from collections import deque
from dataclasses import asdict, dataclass, replace
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import time

from force_signal_pressure_calibration import _write_json_atomic
from high_force_calibration import CombinedCancel, recover_release
from one_step_learning import validate_document as validate_seed
from one_step_repeatability import PositionTrialController, learning_identity
from pressure_feedback_control import PressureFeedbackController, FeedbackControlError, FeedbackSettings
from zdt_x42s_can_driver import ZDTTimeoutError
from motor_speed_units import valid_rpm, quantize_rpm

KIND = 'one_step_independent_cycle_learning'
NEW_ARRIVAL_TARGET_S = 2.0


def observation_pressure_ok(values, target, tolerance, require_full_window=True):
    """旧表校验整窗，新任务按实测中位数；峰值和数据时效保护另行执行。"""
    if not values or not all(math.isfinite(v) for v in values):
        return False
    judged = values if require_full_window else [statistics.median(values)]
    return all(abs(v-target) <= tolerance+1e-9 for v in judged)


@dataclass(frozen=True)
class CycleSettings:
    contact_trials: int = 10
    min_trials: int | None = None
    max_trials: int = 20
    consecutive: int | None = None
    rest_s: float = 5.0
    fast_speed: float = 30
    medium_speed: float = 5
    # 历史任务缺省保持整数策略；新页面显式采用0.1 RPM细调。
    speed_step_rpm: float = 1.0
    observation_s: float = 1.0
    # 缺字段的旧记录仍校验整窗；新页面只以实测中位数判定到位压力。
    require_full_observation_band: bool = True
    # 旧记录沿用实测压力容差作为峰值上限；新任务独立保存过冲允许量。
    overshoot_absolute_n: float | None = None
    overshoot_relative_tolerance: float = .01
    relative_tolerance: float = .01
    contact_delta_n: float = .01
    contact_step: int = 2
    contact_settle_s: float = .1
    # 历史表缺少此字段时保留原轨迹解释；页面新任务显式启用逐轮轨迹学习。
    trajectory_learning: bool = False
    # 旧记录缺省仍从原释放点出发；页面新任务确认接触附近的独立工作释放点。
    near_contact_release: bool = False
    manual_contact_position: int | None = None
    # 缺少字段的历史记录仍按1秒解释；页面新任务显式使用当前目标。
    arrival_target_s: float = 1.0

    def __post_init__(self):
        if self.min_trials is None:
            object.__setattr__(self, 'min_trials', 1 if self.trajectory_learning else 5)
        if self.consecutive is None:
            object.__setattr__(self, 'consecutive', 1 if self.trajectory_learning else 3)

    def overshoot_limit(self, target, pressure_tolerance):
        allowance = (pressure_tolerance if self.overshoot_absolute_n is None else
                     max(self.overshoot_absolute_n, target*self.overshoot_relative_tolerance))
        return target+allowance

    def validate(self, profile):
        if (self.overshoot_absolute_n is not None and
                (type(self.overshoot_absolute_n) not in (int, float) or not math.isfinite(self.overshoot_absolute_n)
                 or not 0 < self.overshoot_absolute_n <= 1)):
            raise ValueError('过冲绝对允许量须为0～1 N内的有限正数')
        if (type(self.overshoot_relative_tolerance) not in (int, float)
                or not math.isfinite(self.overshoot_relative_tolerance) or not 0 <= self.overshoot_relative_tolerance <= .02):
            raise ValueError('过冲相对允许量须为0～2%')
        if type(self.require_full_observation_band) is not bool:
            raise ValueError('观察窗口判定设置须为布尔值')
        if (type(self.speed_step_rpm) not in (int, float) or self.speed_step_rpm not in (.1, 1.0)
                or self.speed_step_rpm == .1 and (not self.trajectory_learning or profile.model != 'zdt_x42s')):
            raise ValueError('0.1 RPM速度学习仅用于ZDT连续轨迹；历史任务沿用整数速度')
        if type(self.near_contact_release) is not bool or self.near_contact_release and not self.trajectory_learning:
            raise ValueError('近接触工作释放点只用于连续轨迹学习')
        if (type(self.trajectory_learning) is not bool or self.arrival_target_s not in (1.0, 2.0)
                or self.manual_contact_position is not None and
                (type(self.manual_contact_position) is not int or not self.trajectory_learning)):
            raise ValueError('轨迹到位目标须为1或2秒；手动接触坐标须为整数且只用于新轨迹学习')
        if (type(self.contact_step) is not int or not 1 <= self.contact_step <= 16
                or not math.isfinite(self.contact_settle_s) or not .1 <= self.contact_settle_s <= 2):
            raise ValueError('接触搜索步幅须为1～16 step，每步等待须为0.10～2.00秒')
        if (self.contact_trials != 10 or self.min_trials != (1 if self.trajectory_learning else 5) or self.max_trials != 20
                or self.consecutive != (1 if self.trajectory_learning else 3) or self.rest_s != 5 or self.contact_delta_n != .01):
            raise ValueError('接触10次，释放间隔5秒；轨迹学习每点1～20轮，历史位置学习5～20次/连续3次')
        speed_cap = min(300 if self.speed_step_rpm == .1 else 400, profile.speed_max)
        if (not valid_rpm(self.fast_speed, minimum=self.speed_step_rpm, maximum=speed_cap, step=self.speed_step_rpm)
                or not valid_rpm(self.medium_speed, minimum=self.speed_step_rpm, maximum=30, step=self.speed_step_rpm)
                or self.medium_speed > self.fast_speed
                or not math.isfinite(self.observation_s) or not .5 <= self.observation_s <= 5
                or not math.isfinite(self.relative_tolerance) or not 0 <= self.relative_tolerance <= .02):
            raise ValueError('请核对空行程速度、中速（不高于空行程）、观察时间及相对容差')


class ContactController(PositionTrialController):
    def __init__(self, *args, cycle_settings=CycleSettings(), **kwargs):
        super().__init__(*args, node={}, fast_speed=1, observation_s=.5, **kwargs)
        cycle_settings.validate(self.profile)
        self.settings = replace(self.settings, settle_s=cycle_settings.contact_settle_s)
        self.contact_step = cycle_settings.contact_step
        self._baseline_force = None
        self.contact = None
        self.contact_diagnostics = dict(rejected_candidates=[], baseline_wait_s=0)
        self.contact_fatal = False

    def run(self, **kwargs):
        # 本控制器自己在释放点连续确认底数；不能被基类单窗口的低力启动检查提前终止。
        return super().run(require_zero_start=False, **kwargs)

    def _poll(self):
        try:
            result = super()._poll()
            # 较大搜索步幅也逐帧检查接触增压，运动和沉降期间不能漏掉峰值。
            if self._baseline_force is not None and result[1]-self._baseline_force > .15:
                self.contact_fatal = True
                raise FeedbackControlError('接触搜索压力增长过大，先释放检查；请减小接触搜索步幅')
            return result
        except FeedbackControlError:
            try:
                force, _, _ = self.samples.read(self.clock(), allow_negative=True)
                if force >= self.settings.force_limit_n:
                    self.contact_fatal = True
            except FeedbackControlError:
                pass
            raise

    def _baseline(self):
        frames = deque()
        started, last_notice = self.clock(), -math.inf
        self.phase = '在释放点等待空载底数稳定（不下压）'
        while True:
            # 等待稳定不占搜索超时，取消、反馈、电流与行程检查仍由_poll执行。
            self.deadline = self.clock()+self.settings.acquire_timeout_s
            sample, force, stamp, seq = self._poll()
            if (abs(sample['position_counts']-self.release) > self.release_tolerance
                    or abs(sample['measured_speed_rpm']) > 1 or not sample.get('in_position')
                    or sample['running_state'] not in (0, 1)):
                raise FeedbackControlError('空载确认时未停在释放点，先回位后重试')
            if not frames or seq != frames[-1]['sequence']:
                if frames and (stamp <= frames[-1]['stamp'] or stamp-frames[-1]['stamp'] > .2+1e-9):
                    frames.clear()
                frames.append(dict(force_n=force, stamp=stamp, sequence=seq))
                while len(frames) > 3 and frames[1]['stamp'] <= stamp-1:
                    frames.popleft()
            values = [r['force_n'] for r in frames]
            low, high = min(values), max(values)
            split = max(1, len(values)//2)
            shift = abs(statistics.median(values[:split])-statistics.median(values[split:] or values))
            stable = (len(frames) >= 10 and frames[-1]['stamp']-frames[0]['stamp'] >= 1-1e-9
                      and max(abs(low), abs(high)) <= .02+1e-9
                      and high-low <= .01+1e-9 and shift <= .005+1e-9)
            self.contact_diagnostics.update(baseline_samples=list(frames), baseline_wait_s=self.clock()-started,
                baseline_min_n=low, baseline_max_n=high)
            if stable:
                return sample, statistics.median(values), low, high
            if self.clock()-last_notice >= .5:
                self.on_record(dict(phase='contact_baseline_wait', message=
                    f'在释放点等待空载稳定，未下压：当前范围{low:.3f}～{high:.3f} N；'
                    f'连续观察1秒，允许0.01 N量化波动；已等待{self.clock()-started:.1f}秒，稳定后自动继续'))
                last_notice = self.clock()
            self._wait()

    def _fast_approach(self):
        sample, baseline, baseline_low, baseline_high = self._baseline()
        threshold = baseline_high+.01
        self._baseline_force = baseline
        self.deadline = self.clock()+self.settings.acquire_timeout_s
        zero_position = int(sample['position_counts'])
        while True:
            remaining = self.settings.max_travel-(self.release-int(sample['position_counts']))
            if remaining <= 0:
                self.contact_fatal = True
                raise FeedbackControlError('接触搜索已到最大行程，释放后结束；不继续盲目下压')
            # 每轮及候选被拒后均沿用用户步幅，仅在最大行程末端裁剪。
            size = min(self.contact_step, remaining)
            self.phase = f'{size} step搜索接触，每步等待{self.settings.settle_s:.2f}秒，持续门槛{threshold:.3f} N'
            self._move(-size, speed_override=min(2, self.settings.speed))
            sample, force = self._window()
            low, high = self._window_force_range
            if high-baseline > .15:
                self.contact_fatal = True
                raise FeedbackControlError('接触搜索压力增长过大，先释放检查')
            if low >= baseline_low-.005 and high <= baseline_high+.005:
                zero_position = int(sample['position_counts'])
            if low < threshold-1e-9:
                continue
            candidate = int(sample['position_counts'])
            self.phase = f'接触候选{candidate} step：停止下压，连续确认0.6秒'
            confirm = []
            while True:
                sample, force, stamp, sequence = self._poll()
                if (abs(sample['position_counts']-candidate) > 1 or abs(sample['measured_speed_rpm']) > 1
                        or not sample.get('in_position') or sample['running_state'] not in (0, 1)):
                    raise FeedbackControlError('接触确认时位置变化，释放后重新尝试')
                if force-baseline > .15:
                    self.contact_fatal = True
                    raise FeedbackControlError('接触确认压力增长过大，先释放检查')
                if not confirm or sequence != confirm[-1]['sequence']:
                    if force < threshold-1e-9 or (confirm and stamp-confirm[-1]['stamp'] > .2+1e-9):
                        rejected = self.contact_diagnostics['rejected_candidates']
                        rejected.append(dict(position_steps=candidate, force_n=force, reason='未持续达到接触门槛'))
                        del rejected[:-32]
                        break
                    confirm.append(dict(force_n=force, stamp=stamp, sequence=sequence))
                if len(confirm) >= 6 and confirm[-1]['stamp']-confirm[0]['stamp'] >= .6-1e-9:
                    if candidate >= zero_position:
                        self.contact_diagnostics['rejected_candidates'].append(dict(position_steps=candidate,
                            reason='未确认相对最后空载位置的真实下压位移'))
                        break
                    measured = statistics.median(r['force_n'] for r in confirm)
                    self.contact = dict(position_steps=candidate, last_zero_position_steps=zero_position,
                        baseline_n=baseline, baseline_min_n=baseline_low, baseline_max_n=baseline_high,
                        threshold_n=threshold, measured_force_n=measured, delta_n=measured-baseline,
                        confirmation_samples=confirm)
                    return
                self._wait()

    def _regulate(self, stop_on_stable=False):
        return dict(ok=True, message='接触点已确认，准备释放')


class CyclePositionController(PositionTrialController):
    """空行程一条命令、受力段一条命令；学习时绝不补压，误差留到下一次修正。"""
    def __init__(self, *args, contact, cycle_settings, measure_only=True, **kwargs):
        record = kwargs.pop('on_record', lambda row: None)
        super().__init__(*args, fast_speed=cycle_settings.medium_speed,
                         observation_s=cycle_settings.observation_s,
                         on_record=lambda row: record(dict(row, motion_segment=self.segment)), **kwargs)
        self.contact_map, self.cycle_settings = contact, cycle_settings
        self.measure_only = measure_only
        self.segment = None
        self.baseline = 0.0
        self.free_pressure_limit = .01
        self.guard_reason = None
        self.predicted_peak_n = None
        self.guard_position = None
        self._previous_pressure = None
        self.observed_trace = []

    def _poll(self):
        try:
            value = super()._poll()
        except FeedbackControlError:
            try:
                force, _, _ = self.samples.read(self.clock(), allow_negative=True)
                if force >= self.settings.force_limit_n:
                    self.peak_n = max(self.peak_n, force)
            except FeedbackControlError:
                pass
            raise
        sample, force, stamp, sequence = value
        if self.segment:
            if self.clock()-stamp > .15:
                raise FeedbackControlError('位置学习运动/观察要求压力反馈不超过0.15秒')
            if not self.observed_trace or sequence != self.observed_trace[-1]['sequence']:
                self.observed_trace.append(dict(force_n=force, position_steps=sample['position_counts'],
                    stamp=stamp, sequence=sequence, segment=self.segment,
                    speed_rpm=sample['measured_speed_rpm'], in_position=sample.get('in_position'),
                    pressure_age_s=self.clock()-stamp,
                    remaining_steps=max(0, sample['position_counts']-self.node['position_steps'])))
            if self.segment == 'free' and force >= self.free_pressure_limit-1e-9:
                self.guard_reason = 'early_contact'
                raise FeedbackControlError('快速段提前接触：本次边界失效，释放后需重新学习接触点')
            if self.segment == 'loaded':
                self._check_loaded_pressure(sample, force, stamp, sequence)
        return value

    def _check_loaded_pressure(self, sample, force, stamp, sequence):
        """真实超压始终停止；预测仅适用于仍有足够剩余行程的下压运动，不外推到终点之外。"""
        limit = self.settings.target_n+self.settings.tolerance_n
        if force > limit+1e-9:
            self.guard_reason = 'overshoot'
            self.guard_position = int(sample['position_counts'])
            raise FeedbackControlError(f'实测压力{force:.3f} N超过{limit:.3f} N，先释放并缩短下一次行程')
        previous = self._previous_pressure
        if previous is not None and sequence == previous[2]:
            return
        self._previous_pressure = (force, stamp, sequence)
        remaining = float(sample['position_counts'])-self.node['position_steps']
        rpm = abs(float(sample['measured_speed_rpm']))
        if (previous is None or remaining <= self.position_tolerance or rpm <= 1
                or sample.get('in_position') or force < self.settings.target_n-max(.1*self.settings.target_n, 2*self.settings.tolerance_n)):
            return
        dt = stamp-previous[1]
        if not .02 <= dt <= .2:
            return
        rate = max(0, (force-previous[0])/dt)
        brake_s = rpm*(256-self.settings.acceleration)*.000025
        remaining_s = remaining/(self.profile.steps_per_turn*rpm/60)
        horizon = min(.3, .08+max(0, self.clock()-stamp)+brake_s, remaining_s)
        predicted = force+rate*horizon
        if predicted > limit+1e-9:
            self.guard_reason = 'predictive_brake'
            self.predicted_peak_n = predicted
            self.guard_position = int(sample['position_counts'])
            raise FeedbackControlError(f'预测提前制动：实测{force:.3f} N，预测{predicted:.3f} N；'
                '先释放，下次验证停止处位置；预测值不记作真实过冲')

    def _fast_approach(self):
        sample, self.baseline = self._window()
        if max(abs(v) for v in self._window_force_range) > .02:
            raise FeedbackControlError('每次按压前必须确认空载底数在±0.02 N内')
        base_low = self.contact_map.get('baseline_min_n', self.contact_map['baseline_median_n'])
        base_high = self.contact_map.get('baseline_max_n', self.contact_map['baseline_median_n'])
        if not base_low-.005-1e-9 <= self.baseline <= base_high+.005+1e-9:
            raise FeedbackControlError('空载底数与接触学习不一致，先释放检查压力计零点')
        self.free_pressure_limit = max(base_high, self._window_force_range[1])+.01
        fast_position = self.contact_map['fast_position_steps']
        if int(sample['position_counts']) > fast_position:
            # 借用已确认的独立接触边界，不伪造旧闭环学习表；仍逐帧保护、只发一条位置命令。
            self.segment = 'free'
            node, speed, observe, callback = self.node, self.fast_speed, self.observation_s, self.on_arrival
            try:
                self.node = dict(position_steps=fast_position, measured_force_n=0)
                self.fast_speed = self.contact_map['effective_fast_speed_rpm']
                self.observation_s, self.on_arrival = .2, lambda row: None
                PositionTrialController._fast_approach(self)
            finally:
                self.node, self.fast_speed, self.observation_s, self.on_arrival = node, speed, observe, callback
                self.arrival = None
        self.segment = 'loaded'
        self.phase = '中速一次定位并采集未微调压力'
        try:
            PositionTrialController._fast_approach(self)
        finally:
            self.segment = None

    def _regulate(self, stop_on_stable=False):
        if self.measure_only:
            return dict(ok=True, message='单次位置按压已采集，未做压力微调')
        return super()._regulate(stop_on_stable=stop_on_stable)


def contact_summary(rows, origin, profile, settings, cycles):
    # 预留反馈延迟、刹车距离及编码器容差。短空行程自动降低最高速度，避免高速撞入材料。
    zero_travel = min(origin-r['last_zero_position_steps'] for r in rows)
    reserve = profile.tolerance(256)*2
    speed = cycles.fast_speed
    def margin(rpm):
        steps_s = profile.steps_per_turn*rpm/60
        brake_s = rpm*(256-settings.acceleration)*.000025
        return reserve+math.ceil(steps_s*(.15+brake_s/2))
    while speed > cycles.speed_step_rpm and margin(speed) > zero_travel*.5:
        speed = quantize_rpm(max(cycles.speed_step_rpm, speed-cycles.speed_step_rpm), cycles.speed_step_rpm)
    fast_travel = max(0, zero_travel-margin(speed))
    summary = dict(median_position_steps=statistics.median(r['position_steps'] for r in rows),
        earliest_position_steps=max(r['position_steps'] for r in rows),
        last_zero_boundary_steps=origin-zero_travel,
        fast_position_steps=origin-fast_travel, reserve_steps=margin(speed),
        requested_fast_speed_rpm=cycles.fast_speed, effective_fast_speed_rpm=speed,
        baseline_median_n=statistics.median(r['baseline_n'] for r in rows))
    if all('baseline_min_n' in r and 'baseline_max_n' in r for r in rows):
        summary.update(baseline_min_n=min(r['baseline_min_n'] for r in rows),
                       baseline_max_n=max(r['baseline_max_n'] for r in rows))
    return summary


def next_candidate(candidate, target, measured, slope, origin, contact, maximum, attempts):
    travel = origin-candidate
    good_pairs = [(origin-r['candidate_position_steps'], r['measured_force_n']) for r in attempts
                  if r.get('measured_force_n') is not None and not r.get('guard_reason')]
    local = [(f1-f0)/(x1-x0) for x0, f0 in good_pairs for x1, f1 in good_pairs
             if x1-x0 >= 2 and f1 > f0]
    slope = max(.0001, statistics.median(local[-10:]) if local else slope)
    change = round(.8*(target-measured)/slope)
    if change == 0: change = 1 if target > measured else -1
    change = max(-128, min(128, change))
    lower = origin-contact['fast_position_steps']+1
    new_travel = max(lower, min(maximum, travel+change))
    # 只有真实压力证据建立行程上界，历史记录中的预测误报也不能成为永久禁区。
    upper = min((origin-(r.get('guard_position_steps') if r.get('guard_position_steps') is not None
                         else r['candidate_position_steps']) for r in attempts
                 if ((r.get('peak_n') is not None and r['peak_n'] > target+r['tolerance_n']+1e-9)
                     or (r.get('measured_force_n') is not None
                         and r['measured_force_n'] > target+r['tolerance_n']+1e-9))), default=maximum+1)
    if change > 0 and upper > lower:
        new_travel = min(new_travel, upper-1)
    return origin-new_travel


def run_relearning(motor, profile, samples, stop, emergency, settings, origin, seed, *,
                   cycles=CycleSettings(), progress=lambda row: None, on_attempt=lambda row: None,
                   on_point=lambda row: None, checkpoint=lambda doc: None,
                   on_motor=lambda row: None, clock=time.monotonic):
    validate_seed(seed, profile)
    cycles.validate(profile)
    settings.validate(profile)
    if not seed['complete'] or origin != seed['origin_position_steps']:
        raise ValueError('首次参考表必须完整，且释放起点与原表完全一致；禁止平移旧表')
    if cycles.manual_contact_position is not None and not origin-settings.max_travel < cycles.manual_contact_position < origin:
        raise ValueError('手动接触位置必须在释放点下方的允许行程内')
    if cycles.trajectory_learning and not callable(getattr(motor, 'move_absolute_steps', None)):
        raise ValueError('当前接口不支持绝对位置在线变速')
    started = clock()
    release_origin = origin
    working_release_valid = False
    cancel = CombinedCancel(stop, emergency)
    document = dict(kind=KIND, format_version=1, created_at=datetime.now().astimezone().isoformat(),
        profile=profile.metadata(), namespace=profile.namespace, origin_position_steps=origin,
        settings=asdict(settings), cycle_settings=asdict(cycles), seed_id=learning_identity(seed),
        seed_created_at=seed['created_at'], contacts=[], contact_attempts=[], contact_detection_version=2,
        position_learning_version=4 if cycles.near_contact_release else 3 if cycles.trajectory_learning else 2,
        contact=None, points=[], nodes=[],
        complete=False, message='尚未完成', checkpoint_errors=[])
    if cycles.near_contact_release:
        document.update(working_release_position_steps=None, working_release_valid=False,
                        working_release_checks=[], initial_positioning_s=None)
    def active_settings():
        from one_step_working_release import working_settings
        return working_settings(settings, origin, release_origin)
    context = dict(stage='contact', target_n=0, attempt=0)
    def emit(row):
        if row.get('phase') == 'trajectory_learning':
            context.update(attempt=row['attempt'], target_n=row['target_n'], stage='pressure')
        item = dict(context, **{k:v for k,v in row.items() if k not in context})
        if 'message' not in item:
            item['message'] = (f"{'接触学习' if context['stage']=='contact' else str(context['target_n'])+' N位置学习'}"
                f" · 第{context['attempt']}/{'10' if context['stage']=='contact' else '20'}次 · {row.get('phase', '')}"
                f" · {dict(free='快速空行程', loaded='中速定位/未微调观察').get(row.get('motion_segment'), '')}"
                f" · 实测{row.get('force_n', 0):.3f} N · 位置{row.get('position_steps', '—')} step")
            if context.get('candidate_position_steps') is not None:
                item['message'] += f" · 候选{context['candidate_position_steps']} step · 连续合格{context['streak']}/3"
        elif row.get('phase') in ('recovery', 'recovery_release', 'contact_baseline_wait'):
            prefix = (f"接触第{context['attempt']}/10次" if context['stage'] == 'contact' else
                      f"目标{context['target_n']:g} N · 第{context['attempt']}次")
            item['message'] = prefix+' · '+item['message']
        progress(item)
    def save():
        document['elapsed_s'] = clock()-started
        try:
            checkpoint(deepcopy(document))
        except Exception as exc:
            message = f'逐次存盘失败，结果仍保留内存，将继续重试保存：{exc}'
            document['checkpoint_errors'].append(message)
            emit(dict(message=message))
    def invalidate_working_release(reason):
        nonlocal release_origin, working_release_valid
        working_release_valid = False
        document['working_release_valid'] = False
        document['working_release_failure'] = reason+'；改回原释放点，需重新学习'
        release_origin = origin
        emit(dict(phase='return', message=document['working_release_failure']))
    def release(invalidate_working=False):
        from one_step_working_release import WORKING_POSITION_TOLERANCE_STEPS, WorkingReleaseSettlingTimeout
        if invalidate_working:
            invalidate_working_release('轨迹启动前连续复核仍有残余压力')
        emit(dict(phase='return', message=f"目标{context['target_n']:g} N · 第{context['attempt']}次：停止并回释放点{release_origin} step"))
        result = recover_release(motor, profile, samples, active_settings(), release_origin, emergency,
            progress=emit, on_motor=on_motor, clock=clock, return_even_within_tolerance=True,
            position_tolerance_steps=WORKING_POSITION_TOLERANCE_STEPS if working_release_valid else None,
            release_side_tolerance_steps=profile.tolerance(256) if working_release_valid else None)
        revoked = invalidate_working
        if working_release_valid and result.get('release_position_outside_tolerance'):
            document['working_release_position_error'] = dict(result)
            revoked = True
            invalidate_working_release(result['message'])
            result = recover_release(motor, profile, samples, settings, origin, emergency,
                progress=emit, on_motor=on_motor, clock=clock, return_even_within_tolerance=True)
        if working_release_valid and result['release_position_confirmed'] and not emergency.is_set():
            from one_step_working_release import confirm_unloaded
            checker = PressureFeedbackController(motor, profile, samples, emergency, active_settings(), release_origin,
                on_record=emit, on_motor=on_motor, clock=clock)
            checker.releasing = True
            try:
                # 回位后的读取超时同样保留恢复机会；重新确认位置再重建完整卸载窗口。
                for check_attempt in range(3):
                    try:
                        check = confirm_unloaded(checker, release_origin, document.setdefault('working_release_return_checks', []))
                        break
                    except ZDTTimeoutError:
                        if check_attempt == 2 or emergency.is_set():
                            raise
                        emit(dict(phase='recovery', message='回位后卸载复核通信超时，重新确认位置及完整卸载窗口'))
                        result = recover_release(motor, profile, samples, active_settings(), release_origin, emergency,
                            progress=emit, on_motor=on_motor, clock=clock, return_even_within_tolerance=True,
                            position_tolerance_steps=WORKING_POSITION_TOLERANCE_STEPS,
                            release_side_tolerance_steps=profile.tolerance(256))
                        if not result['release_position_confirmed'] or emergency.is_set():
                            raise
                    finally:
                        document['working_release_return_checks'][-1].update(
                            target_n=context['target_n'], attempt=context['attempt'])
                result['pressure_unloaded'] = check['unloaded']
                if not check['unloaded']:
                    # 承压后不能卸载说明工作起点已失效；不更改历史轨迹坐标，退回原释放点并结束。
                    revoked = True
                    invalidate_working_release('承压后工作释放点连续复核仍有残余压力')
                    result = recover_release(motor, profile, samples, settings, origin, emergency,
                        progress=emit, on_motor=on_motor, clock=clock, return_even_within_tolerance=True)
            except Exception as exc:
                result['pressure_unloaded'] = False
                document['working_release_check_error'] = str(exc)
                if isinstance(exc, WorkingReleaseSettlingTimeout) or result.get('release_position_outside_tolerance'):
                    revoked = True
                    invalidate_working_release(result['message'] if result.get('release_position_outside_tolerance')
                                               else '三次原位复核仍不能确认静止卸载：'+str(exc))
                    result = recover_release(motor, profile, samples, settings, origin, emergency,
                        progress=emit, on_motor=on_motor, clock=clock, return_even_within_tolerance=True)
        document['last_release_position_steps'] = release_origin
        document.update({k:v for k,v in result.items() if k != 'message'})
        return dict(result, release_position_confirmed=False, working_release_invalidated=True,
                    message=document['working_release_failure']) if revoked else result
    def prepare(rest, contact_only=False):
        if document.get('working_release_failure'):
            raise FeedbackControlError(document['working_release_failure'])
        checker = PressureFeedbackController(motor, profile, samples, cancel, active_settings(), release_origin,
            on_record=emit, on_motor=on_motor, clock=clock)
        # 接触搜索先在释放点等待底数；压力断流或暂时偏高不能抢先结束整个任务。
        checker.releasing = contact_only
        until = clock()+(cycles.rest_s if rest else 0)
        while clock() < until:
            sample, _, _, _ = checker._poll()
            if abs(sample['position_counts']-release_origin) > checker.release_tolerance:
                raise FeedbackControlError('休息期间偏离释放位置')
            emit(dict(phase='rest', message=f"目标{context['target_n']:g} N · 下一次第{context['attempt']}次：已释放，恢复剩余{until-clock():.1f}秒"))
            cancel.wait(min(.1, max(0, until-clock())))
        if working_release_valid and not contact_only:
            from one_step_working_release import confirm_unloaded
            checks = document.setdefault('working_release_prepare_checks', [])
            try:
                check = confirm_unloaded(checker, release_origin, checks, phase='休息后工作释放点卸载复核')
            finally:
                if checks:
                    checks[-1].update(target_n=context['target_n'], attempt=context['attempt'], after_rest=rest)
            if not check['unloaded']:
                invalidate_working_release(f'启动{context["target_n"]:g} N前连续复核仍有残余压力'
                    f'（{check["min_force_n"]:.3f}～{check["max_force_n"]:.3f} N）')
                raise FeedbackControlError(document['working_release_failure'])
            return
        sample = checker._poll()[0] if contact_only else checker._window()[0]
        if not contact_only and max(abs(v) for v in checker._window_force_range) > .02+1e-9:
            raise FeedbackControlError('释放后仍未卸载至±0.02 N，保留结果并停在释放点')
        if abs(sample['position_counts']-release_origin) > (1 if working_release_valid else checker.release_tolerance):
            raise FeedbackControlError('实际位置不在本次释放点')
    released = False
    try:
        save()
        released = release()['release_position_confirmed']
        if not released: raise FeedbackControlError('未确认释放起点')
        total_attempts = 0
        if cycles.manual_contact_position is not None:
            prepare(False, contact_only=True)
            checker = ContactController(motor, profile, samples, cancel, settings, origin,
                cycle_settings=cycles, on_record=emit, on_motor=on_motor, clock=clock)
            _, baseline, low, high = checker._baseline()
            contact_pos = cycles.manual_contact_position
            # 手动坐标不伪装成十次测量；接触前保留位置不确定度，仍逐帧保护。
            boundary = min(origin, contact_pos+max(cycles.contact_step, profile.tolerance(256)))
            document['contact'] = dict(source='manual', median_position_steps=contact_pos,
                earliest_position_steps=contact_pos, last_zero_boundary_steps=boundary,
                fast_position_steps=origin, reserve_steps=origin-boundary,
                requested_fast_speed_rpm=cycles.fast_speed, effective_fast_speed_rpm=cycles.fast_speed,
                baseline_median_n=baseline, baseline_min_n=low, baseline_max_n=high,
                baseline_samples=checker.contact_diagnostics['baseline_samples'])
        while cycles.manual_contact_position is None and len(document['contacts']) < cycles.contact_trials:
            attempt = len(document['contacts'])+1
            total_attempts += 1
            context.update(stage='contact', target_n=0, attempt=attempt, contact_attempt=total_attempts)
            controller = ContactController(motor, profile, samples, cancel,
                replace(settings, target_n=.15, tolerance_n=.005, acquire_timeout_s=600), origin,
                cycle_settings=cycles, on_record=emit, on_motor=on_motor, clock=clock)
            try:
                prepare(total_attempts > 1, contact_only=True)
                result = controller.run()
            except Exception as exc:
                result = dict(ok=False, message=f'接触准备未完成：{exc}；先回释放点再重试')
            released = release()['release_position_confirmed']
            verified, release_frames = False, []
            if result['ok'] and released and controller.contact and not cancel.is_set():
                try:
                    checker = PressureFeedbackController(motor, profile, samples, cancel, settings, origin,
                        on_motor=on_motor, clock=clock)
                    expiry = clock()+2
                    while clock() < expiry:
                        sample, force, stamp, sequence = checker._poll()
                        if (abs(sample['position_counts']-origin) > checker.release_tolerance
                                or abs(sample['measured_speed_rpm']) > 1 or not sample.get('in_position')
                                or sample['running_state'] not in (0, 1)):
                            raise FeedbackControlError('接触释放验证时电机未停在释放点')
                        if not release_frames or sequence != release_frames[-1]['sequence']:
                            if release_frames and stamp-release_frames[-1]['stamp'] > .2+1e-9:
                                release_frames.clear()
                            release_frames.append(dict(force_n=force, stamp=stamp, sequence=sequence))
                        if len(release_frames) >= 6 and release_frames[-1]['stamp']-release_frames[0]['stamp'] >= .6-1e-9:
                            forces = [r['force_n'] for r in release_frames]
                            verified = (max(abs(f) for f in forces) <= .02+1e-9
                                and max(forces) <= controller.contact['baseline_max_n']+.005+1e-9
                                and controller.contact['measured_force_n']-max(forces) >= .01-1e-9)
                            break
                        cancel.wait(.05)
                    if not verified:
                        result['message'] = '释放后未确认压力降回原底数，可能是底数漂移；不计入接触次数，等待后自动重试'
                except Exception as exc:
                    result['message'] = f'接触释放验证未完成：{exc}；已释放，稍后重试'
                    released = release()['release_position_confirmed']
            row = dict(stage='contact', attempt=attempt, target_n=0, ok=bool(result['ok'] and released),
                       contact_attempt=total_attempts, message=result['message'],
                       diagnostics=controller.contact_diagnostics, release_samples=release_frames,
                       release_position_confirmed=released, **(controller.contact or {}))
            row['ok'] = bool(row['ok'] and verified and not cancel.is_set())
            row['retrying'] = bool(not row['ok'] and released and not cancel.is_set() and not controller.contact_fatal)
            if row['ok']: row['message'] = '持续接触及释放降压均已确认，本次计入有效接触结果'
            document['contact_attempts'].append(row)
            if row['ok']:
                document['contacts'].append(row)
            on_attempt(deepcopy(row))
            save()
            if not released or cancel.is_set() or controller.contact_fatal:
                raise FeedbackControlError(result['message'])
            if not row['ok']:
                emit(dict(phase='contact_retry', message=f'接触第{attempt}/10次尚未取得有效结果：{row["message"]}；'
                    '已回释放点，休息5秒后自动重试本次；可随时停止'))
        if cycles.manual_contact_position is None:
            document['contact'] = contact_summary(document['contacts'], origin, profile, settings, cycles)
        contact = document['contact']
        emit(dict(contact=contact, message=f"{'使用手动接触位置' if cycles.manual_contact_position is not None else '10次接触学习完成'}："
            f"{contact['median_position_steps']:g} step；" + ('各压力点分别学习减速点和连续速度轨迹' if cycles.trajectory_learning else
            f"快速段终点{contact['fast_position_steps']} step，速度{contact['effective_fast_speed_rpm']} RPM")))
        if cycles.near_contact_release:
            from one_step_working_release import establish_working_release
            positioning_started = clock()
            check = establish_working_release(motor, profile, samples, cancel, settings, origin, contact,
                document['working_release_checks'], progress=emit, on_motor=on_motor, clock=clock)
            release_origin, working_release_valid = check['position_steps'], True
            document.update(working_release_position_steps=release_origin, working_release_valid=True,
                            initial_positioning_s=clock()-positioning_started)
            emit(dict(phase='working_release_ready', working_release_position_steps=release_origin,
                message=f'工作释放点已确认：绝对位置{release_origin} step，相对工作零点0；后续直接学习压力点，首次就位单独计时'))
        zero_force = (statistics.median(r['force_n'] for r in check['samples']) if cycles.near_contact_release
                      else contact['baseline_median_n'])
        zero = dict(target_force_n=0, measured_force_n=zero_force, position_steps=release_origin,
                    travel_steps=0, error_n=zero_force)
        document['nodes'].append(zero)
        on_point(zero)
        save()
        for target in range(1, 31):
            tolerance = max(settings.tolerance_n, target*cycles.relative_tolerance)
            left, right = seed['nodes'][max(1, target-1)], seed['nodes'][min(30, target+1)]
            slope = (right['measured_force_n']-left['measured_force_n'])/(left['position_steps']-right['position_steps'])
            correction = 0
            if len(document['nodes']) > 1:
                previous = document['nodes'][-1]
                correction = previous['position_steps']-seed['nodes'][target-1]['position_steps']
            if len(document['nodes']) > 2:
                a, b = document['nodes'][-2:]
                if a['position_steps'] > b['position_steps']:
                    slope = (b['measured_force_n']-a['measured_force_n'])/(a['position_steps']-b['position_steps'])
            # 首次取旧表略浅位置，之后只用本点的真实独立按压反馈修正。
            candidate = seed['nodes'][target]['position_steps']+correction+max(1, math.ceil(max(tolerance, target*.03)/slope))
            candidate = max(origin-settings.max_travel, candidate)
            candidate = min(candidate, contact['fast_position_steps']-1)
            if cycles.trajectory_learning:
                from one_step_trajectory import learn_point
                point = dict(target_n=target, converged=False, attempts=[], speed_changes=[])
                document['points'].append(point)
                context.update(stage='pressure', target_n=target, attempt=1)
                node = learn_point(motor, profile, samples, cancel,
                    replace(active_settings(), target_n=target, tolerance_n=tolerance), release_origin, target, slope, candidate,
                    contact, cycles, document['nodes'][-1], point, prepare, release, save, emit, on_attempt, on_motor, clock)
                document['nodes'].append(node)
                on_point(deepcopy(node))
                save()
                continue
            attempts, streak = [], 0
            medium_speed = cycles.medium_speed
            point = dict(target_n=target, converged=False, attempts=attempts, speed_changes=[])
            document['points'].append(point)
            for attempt in range(1, cycles.max_trials+1):
                context.update(stage='pressure', target_n=target, attempt=attempt,
                               candidate_position_steps=candidate, streak=streak)
                prepare(True)
                emit(dict(message=f"目标{target} N（第{target}/30点）· 第{attempt}/20次 · 连续合格{streak}/3"
                    f" · 释放点{origin} → 空行程终点{contact['fast_position_steps']} → 候选位置{candidate} step"
                    f" · 中速{medium_speed} RPM · 允许±{tolerance:.3f} N"))
                node = dict(position_steps=candidate, measured_force_n=target)
                controller = CyclePositionController(motor, profile, samples, cancel,
                    replace(settings, target_n=target, tolerance_n=tolerance), origin,
                    node=node, contact=contact, cycle_settings=replace(cycles, medium_speed=medium_speed),
                    on_record=emit, on_motor=on_motor, clock=clock)
                result = controller.run()
                arrival = controller.arrival
                forces = [r['force_n'] for r in (arrival or {}).get('samples', [])]
                peak = result.get('peak_n')
                passed = bool(result['ok'] and forces and all(abs(f-target) <= tolerance+1e-9 for f in forces)
                              and peak <= target+tolerance+1e-9)
                streak = streak+1 if passed else 0
                row = dict(stage='pressure', target_n=target, attempt=attempt,
                    candidate_position_steps=candidate, actual_position_steps=(arrival or {}).get('actual_position_steps', controller.last_position),
                    measured_force_n=(arrival or {}).get('median_force_n'), peak_n=peak, tolerance_n=tolerance,
                    medium_speed_rpm=medium_speed,
                    last_observed_force_n=controller.last_force,
                    error_n=(arrival['median_force_n']-target) if arrival else None, ok=passed,
                    streak=streak, guard_reason=controller.guard_reason, predicted_peak_n=controller.predicted_peak_n,
                    guard_position_steps=controller.guard_position,
                    message=result['message'], arrival=arrival, trace=controller.observed_trace)
                if result['ok'] and not passed:
                    row['message'] = (f'本次未通过：观察压力{min(forces):.3f}～{max(forces):.3f} N，'
                        f'中位值{arrival["median_force_n"]:.3f} N，实测峰值{peak:.3f} N；'
                        f'要求{target-tolerance:.3f}～{target+tolerance:.3f} N，释放后修正下一次位置')
                attempts.append(row)
                on_attempt(deepcopy(row))
                released = release()['release_position_confirmed']
                save()
                if not released or cancel.is_set(): raise FeedbackControlError('已取消后续学习')
                if peak is not None and peak >= settings.force_limit_n:
                    raise FeedbackControlError('本次触及总保护压力，已释放；保留数据，不重复可能危险的按压')
                if controller.guard_reason == 'early_contact':
                    raise FeedbackControlError('快速接触边界变化，本轮学习已释放；需要重学接触点')
                if not result['ok'] and controller.guard_reason not in ('overshoot', 'predictive_brake'):
                    # 运动/通信异常已回位；同一点可重新开始，不重复旧相对命令。
                    if attempt < cycles.max_trials:
                        emit(dict(message=f'目标{target} N · 第{attempt}次异常已释放，下一次重新尝试：'+result['message']))
                        continue
                if attempt >= cycles.min_trials and streak >= cycles.consecutive:
                    point['converged'] = True
                    good = attempts[-cycles.consecutive:]
                    force = statistics.median(r['measured_force_n'] for r in good)
                    node = dict(target_force_n=target, measured_force_n=force, position_steps=candidate,
                        travel_steps=origin-candidate, error_n=force-target, tolerance_n=tolerance,
                        trials=attempt, confirmed_repeats=cycles.consecutive, medium_speed_rpm=medium_speed)
                    document['nodes'].append(node)
                    on_point(deepcopy(node))
                    save()
                    break
                if not passed:
                    if controller.guard_reason == 'predictive_brake':
                        # 下次从释放点独立验证刚才的停止处，不拿预测压力反算位置或永久封住更深位置。
                        candidate = min(contact['fast_position_steps']-1, max(origin-settings.max_travel, controller.guard_position))
                        continue
                    observed = peak if controller.guard_reason == 'overshoot' else row['measured_force_n']
                    if observed is not None:
                        base = controller.guard_position if controller.guard_position is not None else candidate
                        same_speed = [r for r in attempts if r['medium_speed_rpm'] == medium_speed]
                        proposed = next_candidate(base, target, observed, slope, origin, contact, settings.max_travel, same_speed)
                        real_upper = any(r.get('peak_n') is not None and r['peak_n'] > target+tolerance+1e-9 for r in same_speed)
                        window_low = min(forces) if forces else observed
                        peak_window_conflict = bool(forces and peak-min(forces) > 2*tolerance+.005
                                                    and abs(observed-target) <= 2*tolerance)
                        if (medium_speed > 1 and (peak_window_conflict or
                                proposed == candidate and window_low < target-tolerance and real_upper)):
                            # 同速下真实峰值约束与欠压冲突：下一轮降速验证，旧速度的界限不能锁死新轨迹。
                            lower_speed = max(1, medium_speed//2)
                            point['speed_changes'].append(dict(after_attempt=attempt, from_rpm=medium_speed,
                                to_rpm=lower_speed, position_steps=candidate,
                                reason='当前速度下实际峰值与观察窗压力无法同时合格，降低速度重新验证同一位置'))
                            emit(dict(phase='speed_change', message=f'目标{target} N：当前速度下实际峰值与观察窗压力难以同时满足容差；'
                                f'已释放，下一次由{medium_speed}降至{lower_speed} RPM验证同一位置，不放宽容差'))
                            medium_speed = lower_speed
                            streak = 0
                        else:
                            candidate = proposed
            if not point['converged']:
                raise FeedbackControlError(f'{target} N已尝试20次仍未收敛；已释放，保留全部尝试，未更新有效表')
        document['complete'] = True
        document['message'] = (f'1～30 N轨迹学习完成，全部点满足压力容差和{cycles.arrival_target_s:g}秒到位目标；已释放'
            if cycles.trajectory_learning else '接触10次及1～30 N独立重压学习完成，全部点通过至少5次/连续3次验证；已释放')
    except Exception as exc:
        document['message'] = str(exc)
    finally:
        released = release()['release_position_confirmed']
        document['cancelled'], document['emergency_stopped'] = stop.is_set(), emergency.is_set()
        document['complete'] = bool(document['complete'] and released and not cancel.is_set())
        if document['complete']:
            try:
                prepare(False)
                validate_document(document, profile)
            except Exception as exc:
                document['complete'] = False
                document['message'] = f'完整性复核未通过，保留逐次数据：{exc}'
        if not document['complete']: document['message'] += '；已回释放点' if released else '；尚未确认回位'
        if document.get('working_release_failure'):
            document['message'] += '；'+document['working_release_failure']
        save()
    return document


def validate_document(doc, profile):
    if (doc.get('kind') != KIND or doc.get('format_version') != 1 or doc.get('profile') != profile.metadata()
            or doc.get('namespace') != profile.namespace or type(doc.get('origin_position_steps')) is not int):
        raise ValueError('不是当前电机的独立重压学习记录')
    cycles = CycleSettings(**doc['cycle_settings'])
    cycles.validate(profile)
    FeedbackSettings(**doc['settings']).validate(profile)
    trajectory = doc.get('position_learning_version', 1) in (3, 4)
    if trajectory != cycles.trajectory_learning:
        raise ValueError('学习版本与轨迹设置不一致')
    if (doc.get('position_learning_version', 1) == 4) != cycles.near_contact_release:
        raise ValueError('工作释放点版本与设置不一致')
    if doc.get('complete'):
        origin = doc['origin_position_steps']
        manual = cycles.manual_contact_position is not None
        if (len(doc['contacts']) != (0 if manual else 10) or len(doc['points']) != 30 or len(doc['nodes']) != 31
                or not doc.get('release_position_confirmed') or not doc.get('pressure_unloaded')):
            raise ValueError('独立重压学习未完成接触、全部压力点或回位卸载确认')
        if any(not r['ok'] or r['delta_n'] < .01-1e-9 or not math.isfinite(r['baseline_n'])
               or not origin-doc['settings']['max_travel'] <= r['position_steps'] < r['last_zero_position_steps'] <= origin
               for r in doc['contacts']):
            raise ValueError('接触原始记录无效')
        if doc.get('contact_detection_version', 1) >= 2:
            for row in doc['contacts']:
                loaded, unloaded = row['confirmation_samples'], row['release_samples']
                if (not all(math.isfinite(row[k]) for k in
                            ('baseline_min_n', 'baseline_max_n', 'threshold_n', 'measured_force_n'))
                        or not -.02-1e-9 <= row['baseline_min_n'] <= row['baseline_max_n'] <= .02+1e-9
                        or row['baseline_max_n']-row['baseline_min_n'] > .01+1e-9
                        or abs(row['threshold_n']-row['baseline_max_n']-.01) > 1e-9
                        or not all(math.isfinite(r['force_n']) and math.isfinite(r['stamp']) for r in loaded+unloaded)
                        or len(loaded) < 6 or len(unloaded) < 6
                        or loaded[-1]['stamp']-loaded[0]['stamp'] < .6-1e-9
                        or unloaded[-1]['stamp']-unloaded[0]['stamp'] < .6-1e-9
                        or min(r['force_n'] for r in loaded) < row['threshold_n']-1e-9
                        or abs(row['measured_force_n']-statistics.median(r['force_n'] for r in loaded)) > 1e-9
                        or max(abs(r['force_n']) for r in unloaded) > .02+1e-9
                        or max(r['force_n'] for r in unloaded) > row['baseline_max_n']+.005+1e-9
                        or row['measured_force_n']-max(r['force_n'] for r in unloaded) < .01-1e-9):
                    raise ValueError('接触结果缺少持续增压或释放降压证据')
        if manual:
            c = doc['contact']
            frames = c.get('baseline_samples', [])
            if (c.get('source') != 'manual' or c['median_position_steps'] != cycles.manual_contact_position
                    or not origin-doc['settings']['max_travel'] < cycles.manual_contact_position < origin
                    or c['last_zero_boundary_steps'] != min(origin, cycles.manual_contact_position+max(cycles.contact_step, profile.tolerance(256)))
                    or len(frames) < 10 or frames[-1]['stamp']-frames[0]['stamp'] < 1-1e-9
                    or any(not math.isfinite(r['force_n']) or abs(r['force_n']) > .02+1e-9 for r in frames)):
                raise ValueError('手动接触来源、坐标或空载确认无效')
        elif doc['contact'] != contact_summary(doc['contacts'], origin, profile,
                FeedbackSettings(**doc['settings']), cycles):
            raise ValueError('接触边界与原始10次记录不一致')
        working = origin
        if cycles.near_contact_release:
            from one_step_working_release import working_position_matches
            working = doc.get('working_release_position_steps')
            checks = doc.get('working_release_checks', [])
            if (type(working) is not int or not doc.get('working_release_valid') or not checks
                    or not doc['contact']['median_position_steps'] <= working <= origin
                    or doc.get('last_release_position_steps') != working
                    or not isinstance(doc.get('initial_positioning_s'), (int, float))
                    or not math.isfinite(doc['initial_positioning_s']) or doc['initial_positioning_s'] < 0):
                raise ValueError('工作释放点未确认或已失效')
            release_checks = [checks[-1]]+doc.get('working_release_return_checks', [])+doc.get('working_release_prepare_checks', [])
            for index, check in enumerate(release_checks):
                if check.get('status') == 'error' and check.get('error_type') == 'ZDTTimeoutError':
                    # 中断证据不能充当卸载成功；必须存在同阶段、同压力点同轮的后续完整合格检查。
                    recovered = any(later.get('unloaded') and later.get('status') in ('passed', 'recovered')
                        and all(later.get(k) == check.get(k) for k in ('phase', 'position_steps', 'target_n', 'attempt'))
                        and later.get('started_monotonic', -1) >= check['started_monotonic']+check['elapsed_s']-1e-9
                        for later in release_checks[index+1:])
                    if not recovered:
                        raise ValueError('卸载检查通信中断后缺少同轮恢复证据')
                    continue
                frames = check.get('samples', [])
                if (not check.get('unloaded') or check.get('position_steps') != working or len(frames) < 10
                        or frames[-1]['stamp']-frames[0]['stamp'] < 1-1e-9
                        or any(not math.isfinite(r['force_n']) or abs(r['force_n']) > .02+1e-9
                               or not working_position_matches(r['position_steps'], working, profile) for r in frames)
                        or max(r['position_steps'] for r in frames)-min(r['position_steps'] for r in frames) > 2
                        or any(not 0 < b['stamp']-a['stamp'] <= .2+1e-9 or b['sequence'] == a['sequence']
                               for a, b in zip(frames, frames[1:]))):
                    raise ValueError('工作释放点缺少连续卸载证据')
        previous = working+1
        for target, node in enumerate(doc['nodes']):
            pos = node['position_steps']
            if (node['target_force_n'] != target or type(pos) is not int or pos >= previous
                    or not origin-doc['settings']['max_travel'] <= pos <= origin
                    or node['travel_steps'] != working-pos or not math.isfinite(node['measured_force_n'])):
                raise ValueError('压力位置表无效或未保持单调')
            previous = pos
            if target:
                point = doc['points'][target-1]
                rows = point['attempts']
                tol = max(doc['settings']['tolerance_n'], target*cycles.relative_tolerance)
                if trajectory:
                    from one_step_trajectory import Trajectory, validate_plan
                    if not rows or not 1 <= len(rows) <= 20:
                        raise ValueError('轨迹学习轮次无效')
                    final = rows[-1]
                    seen = []
                    for r in rows:
                        plan = Trajectory(**r['trajectory'])
                        from one_step_working_release import working_settings
                        validate_plan(plan, working, doc['contact'], working_settings(FeedbackSettings(**doc['settings']), origin, working), cycles, profile)
                        if r['trajectory'] in seen:
                            raise ValueError('轨迹学习重复了已尝试参数')
                        seen.append(r['trajectory'])
                    window = (final.get('arrival') or {}).get('samples', [])
                    window_valid = (len(window) >= 3
                        and all(math.isfinite(r['force_n']) and math.isfinite(r['stamp']) for r in window)
                        and window[-1]['stamp']-window[0]['stamp'] >= cycles.observation_s-1e-9
                        and all(0 < b['stamp']-a['stamp'] <= .2+1e-9 and a['sequence'] != b['sequence']
                                for a, b in zip(window, window[1:])))
                    if (point['target_n'] != target or not point['converged'] or not final['ok']
                            or not final.get('release_position_confirmed') or final['trajectory'] != node['trajectory']
                            or final['candidate_position_steps'] != pos or node['trajectory']['position_steps'] != pos
                            or not 0 <= final['position_arrival_s'] <= cycles.arrival_target_s+1e-9
                            or node['position_arrival_s'] != final['position_arrival_s']
                            or abs(node['measured_force_n']-final['measured_force_n']) > 1e-9
                            or abs(node['error_n']-(node['measured_force_n']-target)) > 1e-9
                            or not window_valid or not math.isfinite(final['peak_n'])
                            or final['peak_n'] > cycles.overshoot_limit(target, tol)+1e-9
                            or final['peak_n'] >= doc['settings']['force_limit_n']
                            or any(r['force_n'] > final['peak_n']+1e-9 for r in window)
                            or abs(final['measured_force_n']-statistics.median(r['force_n'] for r in window)) > 1e-9
                            or not observation_pressure_ok([r['force_n'] for r in window], target, tol,
                                                           cycles.require_full_observation_band)):
                        raise ValueError(f'轨迹缺少压力合格及{cycles.arrival_target_s:g}秒到位的真实记录')
                    continue
                if (point['target_n'] != target or not point['converged'] or not 5 <= len(rows) <= 20
                        or abs(node['measured_force_n']-statistics.median(r['measured_force_n'] for r in rows[-3:])) > 1e-9
                        or abs(node['error_n']-(node['measured_force_n']-target)) > 1e-9
                        or any(not r['ok'] or r['candidate_position_steps'] != pos or
                            abs(r['measured_force_n']-target) > tol+1e-9 or r['peak_n'] > target+tol+1e-9
                            for r in rows[-3:])):
                    raise ValueError('同一候选位置未连续三次通过')
                if doc.get('position_learning_version', 1) >= 2:
                    rpm = node['medium_speed_rpm']
                    if (not valid_rpm(rpm, minimum=1, maximum=cycles.medium_speed, step=1)
                            or any(r['medium_speed_rpm'] != rpm for r in rows[-3:])):
                        raise ValueError('最终位置未按同一中速连续验证三次')
    return doc


def latest_path(directory, profile):
    return Path(directory)/f'one_step_cycles_latest_{profile.namespace}.json'


def save_document(directory, document, profile, archive_path=None):
    validate_document(document, profile)
    path = Path(archive_path) if archive_path else Path(directory)/('one_step_cycles_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    _write_json_atomic(path, document)
    if document['complete']:
        _write_json_atomic(latest_path(directory, profile), document)
    return path


def load_document(directory, profile):
    path = latest_path(directory, profile)
    if not path.exists(): return None
    doc = validate_document(json.loads(path.read_text(encoding='utf-8')), profile)
    if not doc['complete']: raise ValueError('latest不是完整独立重压学习表')
    return doc
