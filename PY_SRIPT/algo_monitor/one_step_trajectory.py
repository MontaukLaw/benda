"""逐轮学习绝对位置轨迹；运动中切换速度，失败按实测差距调整，保留每轮决策证据。"""
from dataclasses import asdict, dataclass, replace
import math
import statistics

from pressure_feedback_control import FeedbackControlError
from one_step_relearning import CyclePositionController, observation_pressure_ok
from zdt_x42s_can_driver import ZDTTimeoutError
from motor_speed_units import valid_rpm, quantize_rpm


MAX_COMMUNICATION_RETRIES = 2


@dataclass(frozen=True)
class Trajectory:
    position_steps: int
    free_speed_rpm: float
    decel_position_steps: int
    loaded_speed_rpm: float
    acceleration: int
    deceleration: int


def braking_distance(profile, fast, slow, level):
    # 厂家Emm公式为每(256-acc)*50us改变1RPM；包含一次反馈/发令延迟和位置裕量。
    ramp_s = (fast-slow)*(256-level)*.00005
    return math.ceil(profile.steps_per_turn/60*(fast*.15+(fast+slow)*ramp_s/2)) + profile.tolerance(256)*2


def bounded(plan, origin, contact, settings, cycles, profile):
    """旧空行程在接触前降速；工作点末段曲线在终点前保留反馈及制动距离。"""
    step = cycles.speed_step_rpm
    loaded = quantize_rpm(max(step, min(30, cycles.fast_speed, plan.loaded_speed_rpm)), step)
    fast = quantize_rpm(max(loaded, min(cycles.fast_speed, plan.free_speed_rpm)), step)
    acceleration, deceleration = max(1, min(255, plan.acceleration)), max(1, min(255, plan.deceleration))
    endpoint = max(origin-settings.max_travel, min(plan.position_steps, math.floor(contact['median_position_steps'])-1))
    if cycles.near_contact_release:
        # 只有显式位于工作点下方的减速起点才启用末段曲线，历史单速表保持原义。
        fast = min(30, fast)
        safe = endpoint+braking_distance(profile, fast, loaded, deceleration)
        decel = min(origin, max(safe, plan.decel_position_steps))
        if fast == loaded or decel >= origin:
            fast, decel = loaded, origin
    else:
        boundary = contact['last_zero_boundary_steps']
        while fast > loaded and boundary+braking_distance(profile, fast, loaded, deceleration) >= origin:
            fast = quantize_rpm(max(loaded, fast-step), step)
        safe = boundary+braking_distance(profile, fast, loaded, deceleration)
        decel = min(origin, max(boundary+1, safe, plan.decel_position_steps))
    return Trajectory(int(endpoint), fast, int(decel), loaded, acceleration, deceleration)


def validate_plan(plan, origin, contact, settings, cycles, profile):
    if (any(type(v) is not int for k, v in asdict(plan).items() if k not in ('free_speed_rpm', 'loaded_speed_rpm'))
            or any(not valid_rpm(v, minimum=cycles.speed_step_rpm, maximum=cycles.fast_speed, step=cycles.speed_step_rpm)
                   for v in (plan.free_speed_rpm, plan.loaded_speed_rpm))):
        raise ValueError('轨迹位置/加减速须为整数，速度须符合本次RPM精度')
    if bounded(plan, origin, contact, settings, cycles, profile) != plan:
        raise ValueError('轨迹位置、速度或减速距离超出安全边界')
    contact_valid = (contact['median_position_steps'] <= origin if cycles.near_contact_release
                     else contact['median_position_steps'] < origin)
    if not origin-settings.max_travel <= plan.position_steps < contact['median_position_steps'] or not contact_valid:
        raise ValueError('轨迹终点或接触坐标不在释放点下方允许行程内')


class TrajectoryController(CyclePositionController):
    """固定绝对终点的连续两速轨迹；按所学起点发降速命令，不插入停车观察。"""
    def __init__(self, *args, trajectory, **kwargs):
        super().__init__(*args, **kwargs)
        self.trajectory = trajectory
        self.motion_started = None
        self.commands = []
        self.position_arrival_s = None
        self.pressure_stable_s = None
        self.startup_unload_checks = []

    def run(self, **kwargs):
        if self.cycle_settings.near_contact_release:
            # 新流程在发令前统一连续复核，避免基类短窗口先行终止而丢失证据。
            kwargs['require_zero_start'] = False
        return super().run(**kwargs)

    def _check_loaded_pressure(self, sample, force, stamp, sequence):
        # 连续轨迹始终保留实测超压保护；终点/速度由下一轮学习，不将预测值冒充实测上界。
        limit = self.cycle_settings.overshoot_limit(self.settings.target_n, self.settings.tolerance_n)
        if force > limit+1e-9:
            self.guard_reason, self.guard_position = 'overshoot', int(sample['position_counts'])
            raise FeedbackControlError(f'实测{force:.3f} N超出过冲上限{limit:.3f} N，先释放再调整轨迹')

    def _command(self, speed, accel):
        self._check()
        entry = dict(position_steps=self.trajectory.position_steps, speed_rpm=speed, acceleration=accel,
                     elapsed_s=self.clock()-self.motion_started, acknowledged=False)
        self.commands.append(entry)
        self.motor.move_absolute_steps(self.trajectory.position_steps, speed_rpm=speed, accel=accel,
                                       retry_cancelled=self.cancel.is_set)
        entry['acknowledged'] = True
        self.move_count += 1

    def _fast_approach(self):
        plan = self.trajectory
        validate_plan(plan, self.release, self.contact_map, self.settings, self.cycle_settings, self.profile)
        if not callable(getattr(self.motor, 'move_absolute_steps', None)):
            raise FeedbackControlError('当前电机接口不支持绝对位置在线变速，未启动轨迹')
        if self.cycle_settings.near_contact_release:
            from one_step_working_release import confirm_unloaded
            check = confirm_unloaded(self, self.release, self.startup_unload_checks, phase='轨迹启动前卸载复核')
            if not check['unloaded']:
                self.guard_reason = 'working_release_not_unloaded'
                raise FeedbackControlError('轨迹启动前连续复核仍有残余压力，退回原释放点并结束')
            values = [r['force_n'] for r in check['samples']]
            sample, self.baseline = self._unloaded_last_sample, statistics.median(values)
            low, high = min(values), max(values)
        else:
            sample, self.baseline = self._window()
            low, high = self._window_force_range
        if max(abs(low), abs(high)) > .02+1e-9:
            raise FeedbackControlError('轨迹启动前未确认空载底数')
        if not self.cycle_settings.near_contact_release and not (self.contact_map.get('baseline_min_n', 0)-.005-1e-9 <= self.baseline
                <= self.contact_map.get('baseline_max_n', 0)+.005+1e-9):
            raise FeedbackControlError('空载底数与接触记录不一致')
        from one_step_working_release import working_position_matches
        start_valid = (working_position_matches(sample['position_counts'], self.release, self.profile)
                       if self.cycle_settings.near_contact_release else abs(sample['position_counts']-self.release) <= 1)
        if (not start_valid or abs(sample['measured_speed_rpm']) > 1
                or not sample.get('in_position')):
            raise FeedbackControlError('轨迹必须从已确认的静止释放点启动')
        self.free_pressure_limit = max(high, self.contact_map.get('baseline_max_n', high))+.01
        self.motion_started = self.clock()
        self.phase = '连续轨迹寻压'
        switched = plan.decel_position_steps >= sample['position_counts'] or plan.free_speed_rpm == plan.loaded_speed_rpm
        # 工作点出发的前段也属于受力区，全程执行目标压力保护，不使用空行程接触判据。
        self.segment = 'loaded' if switched or self.cycle_settings.near_contact_release else 'free'
        self._command(plan.loaded_speed_rpm if switched else plan.free_speed_rpm,
                      plan.deceleration if switched or self.cycle_settings.near_contact_release else plan.acceleration)
        expiry = self.clock()+self.profile.motion_timeout(self.release-plan.position_steps,
                                                         plan.loaded_speed_rpm, plan.deceleration)
        stationary_since, last_motor_stamp, stationary_pos = None, None, None
        window, last_sequence, last_pressure_stamp = [], None, None
        while self.clock() < expiry+self.observation_s+2:
            sample, force, stamp, sequence = self._poll()
            now, pos, rpm = self.clock(), sample['position_counts'], abs(sample['measured_speed_rpm'])
            if now-sample['sampled_monotonic'] > .15:
                raise FeedbackControlError('连续轨迹电机反馈超过0.15秒，停止并释放')
            if now-stamp > .15:
                raise FeedbackControlError('连续轨迹压力反馈超过0.15秒，停止并释放')
            if sequence != last_sequence:
                if last_pressure_stamp is not None and not 0 < stamp-last_pressure_stamp <= .2+1e-9:
                    raise FeedbackControlError('连续轨迹压力采样间断，停止并释放')
                last_sequence, last_pressure_stamp = sequence, stamp
            if pos < plan.position_steps-self.position_tolerance:
                self.guard_reason = 'position_overshoot'
                raise FeedbackControlError('实际位置越过轨迹终点容差，先释放')
            projected_position = pos-rpm*self.profile.steps_per_turn/60*(now-sample['sampled_monotonic']+self.POLL_S)
            if not switched and projected_position <= plan.decel_position_steps:
                # 保留同一个绝对终点，直接更新速度；不调用要求静止的prepare_motion。
                self._command(plan.loaded_speed_rpm, plan.deceleration)
                switched, self.segment = True, 'loaded'
            slow_boundary = (plan.position_steps+self.position_tolerance if self.cycle_settings.near_contact_release
                             else self.contact_map['last_zero_boundary_steps'])
            if pos <= slow_boundary and rpm > plan.loaded_speed_rpm+1:
                self.guard_reason = 'late_deceleration'
                raise FeedbackControlError('减速保护边界前尚未降至末段速度，先释放并提前减速')
            # 到位计时取首次真实静止反馈，连续0.2秒确认后回填该时刻；不把观察时间算入运动时间。
            stationary = (abs(pos-plan.position_steps) <= 2 and rpm <= 1 and sample.get('in_position')
                          and sample['running_state'] in (0, 1))
            motor_stamp = sample['sampled_monotonic']
            if not stationary or stationary_pos is not None and abs(pos-stationary_pos) > 1:
                # 重新收敛时必须同时清除旧位置锚点，否则两步回弹后会永久重置观察窗。
                stationary_since, stationary_pos, self.position_arrival_s = None, None, None
                window.clear()
            elif motor_stamp != last_motor_stamp:
                if stationary_since is None:
                    stationary_since, stationary_pos = motor_stamp, pos
                if motor_stamp-stationary_since >= .2-1e-9:
                    self.position_arrival_s = max(0, stationary_since-self.motion_started)
                    self.phase = '轨迹到位观察'
                    if stamp >= stationary_since and (not window or sequence != window[-1]['sequence']):
                        window.append(dict(force_n=force, position_steps=pos, stamp=stamp, sequence=sequence))
                    if len(window) >= 3 and window[-1]['stamp']-window[0]['stamp'] >= self.observation_s-1e-9:
                        values = [r['force_n'] for r in window]
                        if all(abs(f-self.settings.target_n) <= self.settings.tolerance_n+1e-9 for f in values):
                            self.pressure_stable_s = window[-1]['stamp']-self.motion_started
                        self.arrival = dict(target_n=self.settings.target_n, measurement_kind='trajectory_observation',
                            learning_force_n=self.node['measured_force_n'], recorded_position_steps=plan.position_steps,
                            actual_position_steps=pos, position_error_steps=pos-plan.position_steps,
                            position_tolerance_steps=2, position_arrival_s=self.position_arrival_s,
                            pressure_stable_s=self.pressure_stable_s, first_force_n=values[0],
                            median_force_n=statistics.median(values), min_force_n=min(values), max_force_n=max(values),
                            error_vs_learning_n=statistics.median(values)-self.node['measured_force_n'],
                            error_vs_target_n=statistics.median(values)-self.settings.target_n, samples=window)
                        self.on_arrival(dict(self.arrival))
                        self.segment = None
                        return
            last_motor_stamp = motor_stamp
            if self.position_arrival_s is None and now > expiry:
                self.guard_reason = 'position_timeout'
                raise FeedbackControlError('轨迹到位超时，先释放并调整参数')
            self._wait()
        self.guard_reason = 'settling_timeout'
        raise FeedbackControlError('终点静止或观察超时，先释放并调整参数')

    def _regulate(self, stop_on_stable=False):
        if self.measure_only:
            return dict(ok=True, message='连续轨迹已完成未微调观察')
        return super()._regulate(stop_on_stable=stop_on_stable)


def pressure_bracket(plan, row, history, require_full_window=True, peak_limit=None):
    """只用同目标、容差、释放点和运动参数的有效欠压/超压证据建立位置区间。"""
    target, tolerance = row['target_n'], row['tolerance_n']
    peak_limit = target+tolerance if peak_limit is None else peak_limit
    lower, upper, seen = [], [], set()
    for record in history:
        trajectory = record.get('trajectory', {})
        if (record.get('target_n') != target or record.get('tolerance_n') != tolerance
                or abs(record.get('overshoot_limit_n', target+tolerance)-peak_limit) > 1e-9
                or record.get('release_position_steps') != row.get('release_position_steps')
                or record.get('learning_evidence_valid') is False
                or record.get('error_type') is not None and record.get('guard_reason') != 'overshoot'
                or any(trajectory.get(k) != v for k, v in asdict(plan).items() if k != 'position_steps')):
            continue
        pos = trajectory['position_steps']
        seen.add(pos)
        peak, arrival, guard = record.get('peak_n'), record.get('arrival'), record.get('guard_reason')
        if guard == 'overshoot' and peak is not None and peak > peak_limit+1e-9:
            upper.append(pos)
        elif (guard is None and arrival and record.get('measured_force_n') is not None
              and peak is not None and peak <= peak_limit+1e-9):
            low = arrival['min_force_n'] if require_full_window else record['measured_force_n']
            high = arrival['max_force_n'] if require_full_window else record['measured_force_n']
            if low < target-tolerance-1e-9 and high <= target+tolerance+1e-9:
                lower.append(pos)
    if not lower or not upper:
        return None
    shallow, deep = min(lower), max(upper)
    # 位置越负加载越深；交叉证据不能当成单调夹逼的依据。
    if shallow <= deep:
        return dict(conflict=True)
    candidates = [p for p in range(deep+1, shallow) if p not in seen]
    midpoint = (deep+shallow)/2
    candidate = min(candidates, key=lambda p: (abs(p-midpoint), -p)) if candidates else None
    return dict(conflict=False, shallow=shallow, deep=deep, candidate=candidate)


def adjust(plan, row, history, slope, origin, contact, settings, cycles, profile):
    """先学习达压终点，再优化用时；超压保护优先，无可信到位压力时禁止盲目提速。"""
    changes, proposed = [], plan
    speed_step = cycles.speed_step_rpm
    target, tolerance = row['target_n'], row['tolerance_n']
    peak_limit = cycles.overshoot_limit(target, tolerance)
    force, peak, guard = row['measured_force_n'], row['peak_n'], row['guard_reason']
    slope = max(.0001, slope)
    # 用同一速度/加减速下的实测点估计局部刚度，避免把速度造成的变化误算成位移响应。
    motion_keys = ('free_speed_rpm', 'decel_position_steps', 'loaded_speed_rpm', 'acceleration', 'deceleration')
    pairs = [r for r in history if r.get('measured_force_n') is not None and r.get('arrival')
             and all(r['trajectory'][k] == getattr(plan, k) for k in motion_keys)]
    estimates = []
    for a, b in zip(pairs, pairs[1:]):
        dx = a['candidate_position_steps']-b['candidate_position_steps']
        df = b['measured_force_n']-a['measured_force_n']
        if abs(dx) >= 2 and df/dx > 0: estimates.append(df/dx)
    if estimates: slope = max(.0001, statistics.median(estimates[-6:]))
    def change(**kwargs):
        nonlocal proposed
        proposed = replace(proposed, **kwargs)
    if guard == 'early_contact':
        return None, '快速段已出现接触，接触坐标可能失效；已释放，请核对输入或重学接触点'
    if guard in ('position_timeout', 'settling_timeout'):
        return None, '未取得可信到位观察压力：已释放，检查位置到位/静止反馈；不以超时为依据提高速度'
    observed = row.get('arrival') or {}
    low = observed.get('min_force_n') if cycles.require_full_observation_band else force
    high = observed.get('max_force_n') if cycles.require_full_observation_band else force
    under = (guard is None and observed and force is not None
             and peak is not None and peak <= peak_limit+1e-9
             and low < target-tolerance-1e-9 and high <= target+tolerance+1e-9)
    bracket = (pressure_bracket(plan, row, history, cycles.require_full_observation_band, peak_limit)
               if cycles.near_contact_release and (under or guard == 'overshoot') else None)
    if bracket and bracket['conflict']:
        return None, '同一运动参数下欠压与超压位置证据交叉，不能可靠夹逼；已释放，请检查压力重复性及装夹'
    if bracket and bracket['candidate'] is not None:
        change(position_steps=bracket['candidate'])
        changes.append(f'先学习压力位置：欠压端{bracket["shallow"]}与超压端{bracket["deep"]} step之间夹逼，'
                       f'尝试{bracket["candidate"]} step；速度、减速起点及加减速保持不变')
    elif bracket:
        if plan.loaded_speed_rpm <= speed_step:
            return None, '位置区间已无未试整数步且末段速度达到下限，保留最佳方案并结束；已释放，不放宽压力标准'
        # 位置已搜尽才开始末端降速，不反复跳回旧区间两端，不把全程速度一起降低。
        slow = quantize_rpm(plan.loaded_speed_rpm-speed_step, speed_step)
        endpoint = bracket['shallow']
        reserve = max(32, math.ceil((origin-endpoint)*.2),
                      braking_distance(profile, plan.free_speed_rpm, slow, plan.deceleration))
        switch = min(origin, max(endpoint+reserve,
                     plan.decel_position_steps if plan.free_speed_rpm > plan.loaded_speed_rpm else endpoint+reserve))
        change(position_steps=endpoint, loaded_speed_rpm=slow, decel_position_steps=switch)
        changes.append(f'位置夹逼区间{bracket["deep"]}～{endpoint} step已无未试整数步：'
                       f'从欠压端重新学习，末段速度降至{slow:g} RPM；保留前段速度及加减速，按终点前制动距离设置减速起点')
    elif guard in ('late_deceleration', 'position_overshoot'):
        change(decel_position_steps=plan.decel_position_steps+max(8, math.ceil(plan.free_speed_rpm*profile.steps_per_turn/60*.08)),
               free_speed_rpm=max(plan.loaded_speed_rpm, plan.free_speed_rpm-speed_step if speed_step == .1 else int(plan.free_speed_rpm*.8)),
               loaded_speed_rpm=max(speed_step, plan.loaded_speed_rpm-speed_step), deceleration=max(1, plan.deceleration-15))
        changes.append('接触速度/位置收敛异常：降低速度并柔化末段加减速'
                       + ('，调整末段曲线减速起点' if plan.free_speed_rpm > plan.loaded_speed_rpm else ''))
    elif guard == 'overshoot' or peak is not None and peak > peak_limit+1e-9:
        lower_speed = max(speed_step, plan.loaded_speed_rpm-speed_step) if speed_step == .1 else max(1, int(plan.loaded_speed_rpm*.7))
        change(loaded_speed_rpm=lower_speed, deceleration=max(1, plan.deceleration-15),
               decel_position_steps=plan.decel_position_steps+8)
        if force is None or force > target+tolerance:
            # 超压中断无稳态样本时只小幅退浅；不把整段峰值直接作为静态刚度证据。
            change(position_steps=plan.position_steps+min(32, max(1, math.ceil((peak-target)/slope*.5))))
        changes.append('实测过冲且没有同参数欠压边界：降低受力速度；无稳态样本时小幅退浅'
                       + ('，调整减速起点' if not cycles.near_contact_release or plan.free_speed_rpm > plan.loaded_speed_rpm else ''))
    elif force is not None and row.get('arrival'):
        if low < target-tolerance-1e-9 or high > target+tolerance+1e-9:
            if low < target-tolerance-1e-9 and high > target+tolerance+1e-9:
                return None, '到位观察压力同时越过上下限，无法仅靠终点平移收敛；已释放，检查压力波动'
            # 未达压时冻结速度与加减速，只修正终点，避免混淆位移和速度对压力的影响。
            error = target-force
            delta = max(-64, min(64, round(.7*error/slope)))
            if not delta: delta = 1 if error >= 0 else -1
            change(position_steps=plan.position_steps-delta)
            changes.append(f'先学习压力位置：观察压力偏差{force-target:+.3f} N，修正终点{-delta:+d} step；速度及加减速保持不变')
        elif row.get('pressure_ok') and row.get('position_arrival_s') is not None and row['position_arrival_s'] > cycles.arrival_target_s:
            # 达压而运动慢：先缩短保守空行程，再提受力速度；由安全距离约束重新计算减速点。
            if speed_step == .1:
                # 小数速度按一格探索，保持合格终点及加减速，避免多项同时改变导致反复过冲。
                change(free_speed_rpm=min(cycles.fast_speed, plan.free_speed_rpm+speed_step),
                       loaded_speed_rpm=min(30, cycles.fast_speed, plan.loaded_speed_rpm+speed_step))
                changes.append(f'压力已合格，到位{row["position_arrival_s"]:.3f}秒超过{cycles.arrival_target_s:g}秒：'
                               '速度增加0.1 RPM，保持终点、减速起点及加减速档位')
            else:
                change(free_speed_rpm=min(cycles.fast_speed, max(plan.free_speed_rpm+1, math.ceil(plan.free_speed_rpm*1.2))),
                       acceleration=min(255, plan.acceleration+20),
                       deceleration=min(255, plan.deceleration+20),
                       decel_position_steps=contact['last_zero_boundary_steps']+1,
                       loaded_speed_rpm=min(30, cycles.fast_speed, plan.loaded_speed_rpm+1))
                changes.append(f'压力已合格，开始优化速度：到位{row["position_arrival_s"]:.3f}秒超过{cycles.arrival_target_s:g}秒，保持终点并提高速度/加速、压缩多余提前量')
        else:
            return None, '未满足压力合格及可信到位计时条件，不进入速度优化；已释放'
    else:
        return None, '本轮通信、采样或运动异常，缺少可信学习依据；已执行回位恢复，结束本轮任务'
    proposed = bounded(proposed, origin, contact, settings, cycles, profile)
    seen = [r['trajectory'] for r in history]
    if asdict(proposed) in seen or proposed == plan:
        # 位置学习遇到重复点，只探索本次修正区间内未尝试的终点，不用任意降速凑新参数。
        position_only = (proposed.position_steps != plan.position_steps
                         and all(getattr(proposed, k) == getattr(plan, k) for k in motion_keys))
        direction = 1 if proposed.position_steps > plan.position_steps else -1
        alternatives = ([replace(plan, position_steps=pos)
                         for pos in range(proposed.position_steps, plan.position_steps, -direction)]
                        if position_only else [])
        proposed = next((p for raw in alternatives if asdict(p := bounded(raw, origin, contact, settings, cycles, profile)) not in seen), None)
        if proposed is None:
            return None, '当前误差修正范围内无未尝试的有效参数，已释放；不重复失败参数，不为凑新参数改变速度'
        changes.append(f'避免重复失败参数：保持速度，改试修正区间内终点{proposed.position_steps} step')
    return proposed, '；'.join(changes)


def learn_point(motor, profile, samples, cancel, settings, origin, target, slope, candidate, contact,
                cycles, previous, point, prepare, release, save, emit, on_attempt, on_motor, clock):
    prior = (previous or {}).get('trajectory')
    loaded = prior['loaded_speed_rpm'] if prior else cycles.medium_speed
    plan = bounded(Trajectory(int(candidate), loaded if cycles.near_contact_release else min(30, cycles.fast_speed), contact['last_zero_boundary_steps']+1,
                              loaded, max(1, settings.acceleration), prior['deceleration'] if prior else max(1, settings.acceleration)),
                   origin, contact, settings, cycles, profile)
    attempt, communication_retries = 1, 0
    while attempt <= cycles.max_trials:
        learning_stage = 'speed_optimization' if point['attempts'] and point['attempts'][-1]['pressure_ok'] else 'pressure_position'
        stage_label = '压力已合格，优化速度' if learning_stage == 'speed_optimization' else '先学习达压终点'
        emit(dict(phase='trajectory_learning', attempt=attempt, target_n=target, learning_stage=learning_stage,
                  message=f'{target} N轨迹学习第{attempt}/20轮 · 终点{plan.position_steps} step · '
                  f'前段{plan.free_speed_rpm}/末段{plan.loaded_speed_rpm} RPM · 减速点{plan.decel_position_steps} step · '
                  f'加速/减速{plan.acceleration}/{plan.deceleration}档 · {stage_label}'))
        result = None
        try:
            prepare(True)
        except ZDTTimeoutError as exc:
            # 休息或启动检查超时也先回位，不把尚未发令的一轮判成参数失败。
            result = dict(ok=False, message=str(exc), error_type=type(exc).__name__,
                          failure_phase='轮前准备', communication_timeout=True)
        controller = TrajectoryController(motor, profile, samples, cancel, settings, origin,
            node=dict(position_steps=plan.position_steps, measured_force_n=target), contact=contact,
            cycle_settings=cycles, trajectory=plan, on_record=emit, on_motor=on_motor, clock=clock)
        if result is None:
            result = controller.run()
        arrival, peak = controller.arrival, result.get('peak_n')
        force = arrival['median_force_n'] if arrival else None
        peak_limit = cycles.overshoot_limit(target, settings.tolerance_n)
        pressure_ok = bool(result['ok'] and arrival and peak is not None and peak <= peak_limit+1e-9
                           and observation_pressure_ok([r['force_n'] for r in arrival['samples']], target,
                               settings.tolerance_n, cycles.require_full_observation_band))
        time_ok = controller.position_arrival_s is not None and controller.position_arrival_s <= cycles.arrival_target_s+1e-9
        row = dict(stage='pressure', learning_stage=learning_stage, target_n=target, attempt=attempt, trajectory=asdict(plan),
            release_position_steps=origin,
            candidate_position_steps=plan.position_steps, actual_position_steps=controller.last_position,
            medium_speed_rpm=plan.loaded_speed_rpm, measured_force_n=force, peak_n=peak,
            tolerance_n=settings.tolerance_n, error_n=force-target if force is not None else None,
            overshoot_limit_n=peak_limit,
            position_arrival_s=controller.position_arrival_s, pressure_stable_s=controller.pressure_stable_s,
            arrival_target_s=cycles.arrival_target_s,
            require_full_observation_band=cycles.require_full_observation_band,
            motion_started_monotonic=controller.motion_started,
            pressure_ok=pressure_ok, time_ok=time_ok, ok=pressure_ok and time_ok, streak=0,
            guard_reason=controller.guard_reason, arrival=arrival, trace=controller.observed_trace,
            startup_unload_checks=controller.startup_unload_checks,
            commands=controller.commands, execution_message=result['message'], motion_diagnostics=result.get('motion_diagnostics', []),
            error_type=result.get('error_type'), failure_phase=result.get('failure_phase'),
            last_observed_force_n=controller.last_force, message=result['message'], next_trajectory=None, parameter_changes={}, adjustment_reason='')
        release_result = (release(invalidate_working=True)
                          if controller.guard_reason == 'working_release_not_unloaded' else release())
        released = release_result['release_position_confirmed']
        row['release_position_confirmed'] = released
        row['release_pressure_unloaded'] = release_result.get('pressure_unloaded', False)
        if result.get('communication_timeout') and not controller.guard_reason:
            # 不在运动途中重发不确定的命令；只有回位及卸载成功后，才重新执行整轮。
            safe_retry = (released and row['release_pressure_unloaded'] and not cancel.is_set()
                          and (peak is None or peak <= peak_limit+1e-9)
                          and communication_retries < MAX_COMMUNICATION_RETRIES)
            row.update(record_kind='communication_interruption', learning_evidence_valid=False,
                       ok=False, pressure_ok=False, time_ok=False, retry_scheduled=safe_retry,
                       communication_retry=communication_retries+1,
                       next_trajectory=asdict(plan) if safe_retry else None)
            reason = (f'通信超时；已确认回位及卸载，重试当前第{attempt}轮'
                      f'（本压力点自动重试{communication_retries+1}/{MAX_COMMUNICATION_RETRIES}）；'
                      '中断单独保存，不占学习轮次，不据此调整参数' if safe_retry else
                      '通信超时；重试次数已用完或安全恢复条件未满足，保留已学结果并结束')
            row['message'] = row['adjustment_reason'] = reason+'；原始原因：'+result['message']
            point.setdefault('communication_interruptions', []).append(row)
            on_attempt(dict(row))
            emit(dict(phase='communication_retry', message=f'{target} N：'+row['message']))
            save()
            if not safe_retry:
                raise FeedbackControlError(f'{target} N轨迹学习中断：'+row['message'])
            communication_retries += 1
            continue
        point['attempts'].append(row)
        # 最快压力合格方案单独保存；后续试探失败不能覆盖，超出本次时间目标仍明确标为未达标。
        best = point.get('best_pressure_candidate')
        if (released and pressure_ok and (not cycles.near_contact_release or row['release_pressure_unloaded'])
                and (best is None or row['position_arrival_s'] < best['position_arrival_s'])):
            point['best_pressure_candidate'] = dict(attempt=attempt, trajectory=asdict(plan),
                release_position_steps=origin, measured_force_n=force, peak_n=peak,
                position_arrival_s=row['position_arrival_s'], time_ok=time_ok)
        if release_result.get('working_release_invalidated'):
            row['ok'] = False
            row['adjustment_reason'] = '工作释放点未通过复核，已结束当前学习：'+release_result['message']
        elif not released or cancel.is_set():
            row['ok'] = False
            row['adjustment_reason'] = '取消或未确认回位；不执行下一轮'
        elif cycles.near_contact_release and not row['release_pressure_unloaded']:
            row['ok'] = False
            row['adjustment_reason'] = '已按位置回工作释放点，但未取得连续卸载证据；结束任务，不启动下一轮'
        elif peak is not None and peak >= settings.force_limit_n:
            row['ok'] = False
            row['adjustment_reason'] = '触及总压力保护，已释放，结束任务'
        elif row['ok']:
            row['adjustment_reason'] = (f'压力和{cycles.arrival_target_s:g}秒到位目标均达成，记录所学轨迹；不重复同参数校验'
                if cycles.require_full_observation_band else
                f'实测压力中位数合格、峰值未超限且{cycles.arrival_target_s:g}秒内到位，记录所学轨迹；整窗压力范围仅作参考')
        else:
            next_plan, reason = adjust(plan, row, point['attempts'], slope, origin, contact, settings, cycles, profile)
            if not result['ok']:
                reason += '；原始原因：'+result['message']
            row['adjustment_reason'] = reason
            row['next_trajectory'] = asdict(next_plan) if next_plan and attempt < cycles.max_trials else None
            if row['next_trajectory']:
                row['parameter_changes'] = {key: dict(before=value, after=row['next_trajectory'][key])
                    for key, value in row['trajectory'].items() if value != row['next_trajectory'][key]}
        row['message'] = row['adjustment_reason']
        if attempt == cycles.max_trials and not row['ok']:
            row['adjustment_reason'] += '；已达到20轮上限，本点未达标'
            row['message'] = row['adjustment_reason']
        on_attempt(dict(row))
        emit(dict(phase='trajectory_adjustment', message=f'{target} N第{attempt}轮：'+row['adjustment_reason']))
        save()
        if row['ok']:
            point['converged'] = True
            return dict(target_force_n=target, measured_force_n=force, position_steps=plan.position_steps,
                overshoot_limit_n=peak_limit,
                travel_steps=origin-plan.position_steps, error_n=force-target, tolerance_n=settings.tolerance_n,
                trials=attempt, confirmed_repeats=1, medium_speed_rpm=plan.loaded_speed_rpm,
                trajectory=asdict(plan), position_arrival_s=controller.position_arrival_s,
                pressure_stable_s=controller.pressure_stable_s)
        if not row['next_trajectory']:
            raise FeedbackControlError(f'{target} N轨迹学习未达标：'+row['adjustment_reason'])
        plan = Trajectory(**row['next_trajectory'])
        attempt += 1
    raise FeedbackControlError(f'{target} N已学习20轮仍未满足压力及{cycles.arrival_target_s:g}秒到位目标；已释放')
