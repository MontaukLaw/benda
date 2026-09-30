"""压力计闭环：串行小步运动、到位后重新采样，独立于传感器通道状态。"""

from dataclasses import dataclass, asdict, replace
from collections import deque
import math
import statistics
import threading
import time
from motor_speed_units import valid_rpm

# 与位置标定页的释放容差一致；精细调节仍使用50 counts的等效容差。
RELEASE_REFERENCE_TOLERANCE_COUNTS = 256
CONTROL_STRATEGY = "continuous_sampling_v8"
LEARNING_TARGETS_N = (0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 20, 25, 30)
LOW_FORCE_MAX_N = 0.2
LOW_FORCE_TOLERANCE_N = 0.01
LOW_FORCE_ZERO_N = 0.02
CONTACT_ZERO_N = 0.005


def pressure_tolerance_n(target_n, percent):
    """两页共用有效容差；低压力按当前压力计0.01 N读数分辨率设置下限。"""
    requested = target_n * percent / 100
    return max(LOW_FORCE_TOLERANCE_N, requested) if target_n <= LOW_FORCE_MAX_N else requested


class FeedbackControlError(RuntimeError):
    pass


class FeedbackCancelled(FeedbackControlError):
    pass


class PressureSamples:
    """保留压力计原始采样时间，重复帧和GUI延迟不能刷新数据有效期。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._sample = None

    def update(self, sample):
        with self._lock:
            previous = self._sample
            if (previous and sample.get("accepted") and previous.get("accepted")
                    and sample.get("sequence", -1) <= previous.get("sequence", -1)):
                return
            self._sample = dict(sample)

    def read(self, now, max_age=0.5, *, allow_negative=False):
        with self._lock:
            sample = dict(self._sample or {})
        try:
            force = float(sample["force_n"])
            sampled = float(sample["sampled_monotonic"])
            sequence = int(sample["sequence"])
        except (KeyError, ValueError, TypeError, OverflowError):
            raise FeedbackControlError("压力计没有有效样本") from None
        if (not sample.get("accepted") or sample.get("invalidate")
                or sample.get("force_unit") != "N"
                or not math.isfinite(force) or not math.isfinite(sampled)):
            raise FeedbackControlError("压力计数据无效：" + str(sample.get("rejection_reason", "")))
        if not 0 <= now - sampled <= max_age:
            raise FeedbackControlError("压力计数据过期或时间异常，停止调节")
        if force < -0.1 and not allow_negative:
            raise FeedbackControlError("压力计负值超过 -0.10 N，请卸载后检查零点")
        return force, sampled, sequence


@dataclass(frozen=True)
class FeedbackSettings:
    target_n: float = 1.0
    tolerance_n: float = 0.05
    force_limit_n: float = 5.0
    max_step: int = 16
    speed: float = 5
    approach_speed: float = 5
    release_speed: float = 24
    acceleration: int = 100
    max_current_ma: int = 3000
    max_travel: int = 6144
    settle_s: float = 0.2
    stable_s: float = 1.0
    hold_s: float = 30.0
    acquire_timeout_s: float = 180.0
    use_native_motion_limits: bool = False

    def validate(self, profile, release_only=False):
        if any(not math.isfinite(float(value)) for value in asdict(self).values()):
            raise FeedbackControlError("控制参数必须为有限数值")
        if not release_only and not 0 < self.tolerance_n < self.target_n <= 30:
            raise FeedbackControlError("目标压力须在 0～30 N 内，容差须大于0且小于目标")
        if not release_only and not self.target_n + self.tolerance_n < self.force_limit_n <= 35:
            raise FeedbackControlError("保护压力须高于目标加容差，且不超过35 N")
        for key in ("max_step", "acceleration", "max_current_ma", "max_travel"):
            value = getattr(self, key)
            if isinstance(value, bool) or int(value) != value:
                raise FeedbackControlError("步数、电流和加速度必须为整数")
        minimum_speed = .1 if profile.model == 'zdt_x42s' else 1
        for key in ('speed', 'approach_speed', 'release_speed'):
            value = getattr(self, key)
            resolution = 1 if value > 300 else minimum_speed
            if not valid_rpm(value, minimum=minimum_speed, maximum=profile.speed_max, step=resolution):
                raise FeedbackControlError('ZDT速度须为0.1 RPM整数倍；其他电机速度档位须为整数')
        if not (1 <= self.max_step
                and minimum_speed <= self.speed <= (profile.speed_max if self.use_native_motion_limits else min(30, profile.speed_max))
                and minimum_speed <= self.approach_speed <= min(30, profile.speed_max)
                and minimum_speed <= self.release_speed <= (profile.speed_max if self.use_native_motion_limits else min(400, profile.speed_max))
                and (0 if self.use_native_motion_limits else 1) <= self.acceleration <= profile.acceleration_max
                and 1 <= self.max_current_ma <= 3000
                and 1 <= self.max_travel <= profile.steps_per_turn * 12):
            raise FeedbackControlError("运动参数超过允许范围")
        if not (0.1 <= self.settle_s <= 5 and 0.5 <= self.stable_s <= 10
                and 1 <= self.hold_s <= 3600 and 10 <= self.acquire_timeout_s <= 600):
            raise FeedbackControlError("等待、稳定、保持或寻压超时时间超出范围")


@dataclass(frozen=True)
class LearnedPressureMap:
    """本次装夹的加载曲线，仅供保守接近；点为实测压力N和相对释放点的行程。"""

    motor_model: str
    namespace: str
    steps_per_turn: int
    release_position: int
    points: tuple
    contact_zero_travel: float | None = None
    contact_travel: float | None = None
    contact_force_n: float | None = None
    contact_reference_n: float = 0.0

    def __post_init__(self):
        if len(self.points) < 3:
            raise FeedbackControlError("学习点不足，未建立快速接近表")
        previous_force, previous_travel = 0.0, 0.0
        for force, travel in self.points:
            if (not math.isfinite(force) or not math.isfinite(travel)
                    or force <= previous_force or travel <= previous_travel):
                raise FeedbackControlError("学习压力或位置未单调增加，请检查装夹和反馈")
            previous_force, previous_travel = force, travel
        contact = (self.contact_zero_travel, self.contact_travel, self.contact_force_n)
        if not math.isfinite(self.contact_reference_n) or abs(self.contact_reference_n) > LOW_FORCE_ZERO_N+1e-9:
            raise FeedbackControlError("接触点底数必须在±0.02 N内")
        if any(value is not None for value in contact):
            if (not all(value is not None and math.isfinite(value) for value in contact)
                    or not 0 <= self.contact_zero_travel < self.contact_travel < self.points[0][1]
                    or not .01-1e-9 <= self.contact_force_n-self.contact_reference_n <= .02+1e-9
                    or self.contact_force_n >= self.points[0][0]):
                raise FeedbackControlError("接触点必须由稳定底数及相对增加0.01～0.02 N的样本确认，并位于首个加载点之前")

    def loading_points(self):
        """接触实测点可用于加载段插值；零压力边界只用于空行程保护。"""
        return (((self.contact_force_n, self.contact_travel),) + self.points
                if self.contact_travel is not None else self.points)

    def matches(self, profile, release_position):
        return (self.motor_model == profile.model and self.namespace == profile.namespace
                and self.steps_per_turn == profile.steps_per_turn and self.release_position == release_position)

    def travel_at(self, force):
        # 只在实测加载范围内插值，不向未知压力外推。
        points = self.loading_points()
        if not points[0][0] <= force <= points[-1][0]:
            return None
        for (f0, x0), (f1, x1) in zip(points, points[1:]):
            if f0 <= force <= f1:
                return x0+(force-f0)*(x1-x0)/(f1-f0)
        return None

    def approach_travel(self, target_n, margin_steps):
        travel = self.travel_at(target_n*0.8)
        if travel is None:
            return None
        return max(0, math.floor(travel-max(margin_steps, travel*0.02)))


class PressureFeedbackController:
    """只在所属电机工作线程运行；任何退出都停止，不自动重发或反向卸载。"""

    POLL_S = 0.05

    def __init__(self, motor, profile, samples, cancel, settings, release_position,
                 on_record=lambda record: None, on_motor=lambda sample: None,
                 clock=time.monotonic, learned_map=None):
        self.motor, self.profile, self.samples, self.cancel = motor, profile, samples, cancel
        self.settings, self.release = settings, int(release_position)
        self.release_tolerance = profile.tolerance(RELEASE_REFERENCE_TOLERANCE_COUNTS)
        self.on_record, self.on_motor, self.clock = on_record, on_motor, clock
        self.phase = "准备"
        self.started = clock()
        self.deadline = self.started + settings.acquire_timeout_s
        self.hold_started = None
        self.last_record = -math.inf
        self.last_record_sequence = None
        self.command_steps = 0
        self.move_count = 0
        self.hold_errors = []
        self.peak_n = -math.inf
        self.releasing = False
        self._response_slopes = deque(maxlen=3)
        self._previous_observation = None
        self._low_force_anchor = None
        self._last_adjustment_error = None
        self._adjustment_gain = 0.75
        self._last_adjustment_size = None
        self._reversal_cap = None
        self.learned_map = learned_map
        self.learning = False
        self.learning_points = []
        self.used_learned_map = False
        self._noise_windows = deque(maxlen=3)
        self._centering_direction = 0
        self._settle_until = self.started
        self._window_in_band = False
        self._window_band_since = None
        self.first_in_band_s = None
        self.stable_arrival_s = None
        self._window_force_range = None
        self._learning_contact = False
        self._contact_zero_travel = None
        self._contact_travel = None
        self._contact_force_n = None
        self.used_contact_approach = False
        self._contact_fast_aborted = False
        self._contact_reference_n = 0.0
        self._contact_reference_confirmed = False
        self._stationary_samples = deque()
        self._stationary_ready = None
        self._motor_trace = deque(maxlen=24)
        self.motion_diagnostics = deque(maxlen=32)

    def _trace_motor(self, sample):
        """保留异常前后的真实电机反馈；无效数值写null，不伪造停止读数。"""
        row = dict(elapsed_s=self.clock()-self.started, phase=self.phase)
        for key in ('position_counts', 'measured_speed_rpm', 'phase_current_ma',
                    'running_state', 'in_position', 'sampled_monotonic'):
            value = sample.get(key)
            row[key] = None if isinstance(value, float) and not math.isfinite(value) else value
        stamp = row['sampled_monotonic']
        row['feedback_age_s'] = self.clock()-stamp if isinstance(stamp, (int, float)) else None
        self._motor_trace.append(row)

    def _motion_event(self, reason):
        self.motion_diagnostics.append(dict(reason=str(reason), elapsed_s=self.clock()-self.started,
            phase=self.phase, target_n=self.settings.target_n, command_steps=self.command_steps,
            feedback=[dict(row) for row in self._motor_trace]))

    def _stop_and_confirm(self, *, monitor_operation=True):
        """停止后以至少3组新反馈确认速度、状态和位置连续稳定，不用单帧判失败。"""
        started = self.clock()
        ack_error = None
        try:
            self.motor.stop()
        except Exception as exc:
            ack_error = str(exc)
            self._motion_event(f'停止应答异常，继续核验反馈：{exc}')
        stable_since, last_stamp, anchor, count = None, None, None, 0
        confirmed = None
        had_unsettled_feedback = False
        last_error = '电机停止状态尚未连续确认'
        while self.clock()-started < 2:
            try:
                if monitor_operation:
                    sample = self._poll()[0]
                else:
                    # 收尾停止不依赖压力采样或普通取消标志，且绝不追加位移。
                    sample = self.motor.read_feedback()
                    self._trace_motor(sample)
                    self.on_motor(sample)
                now = self.clock()
                position, speed, current, stamp = (float(sample[k]) for k in
                    ('position_counts', 'measured_speed_rpm', 'phase_current_ma', 'sampled_monotonic'))
                valid = (all(math.isfinite(v) for v in (position, speed, current, stamp))
                         and 0 <= now-stamp <= .5 and stamp >= started
                         and abs(current) <= self.settings.max_current_ma)
                stationary = (valid and abs(speed) <= 1 and int(sample['running_state']) in (0, 1)
                              and sample.get('in_position', True))
                fresh = last_stamp is None or stamp > last_stamp
                if not stationary or not fresh or last_stamp is not None and stamp-last_stamp > .5:
                    if not had_unsettled_feedback:
                        self._motion_event('停止后反馈尚未稳定，继续等待连续新反馈')
                        had_unsettled_feedback = True
                    stable_since, anchor, count = None, None, 0
                elif anchor is None or abs(position-anchor) > self.profile.tolerance(16):
                    stable_since, anchor, count = stamp, position, 1
                else:
                    count += 1
                    if count >= 3 and stamp-stable_since >= .1-1e-9:
                        self.command_steps = 0
                        confirmed = sample
                        break
                last_stamp = stamp
            except FeedbackCancelled:
                raise
            except Exception as exc:
                # 调节期间的压力/电流等保护仍立即上抛，由外层先释放。
                if monitor_operation:
                    raise
                stable_since, last_stamp, anchor, count = None, None, None, 0
                last_error = str(exc)
            if monitor_operation:
                self._wait()
            else:
                before = self.clock()
                self.cancel.wait(self.POLL_S)
                # Event已取消时wait立即返回，避免停止确认忙轮询；兼容虚拟时钟测试。
                remaining = self.POLL_S-(self.clock()-before)
                if remaining > .001:
                    time.sleep(remaining)
        if confirmed is not None:
            if had_unsettled_feedback:
                self._motion_event('停止后的延迟反馈已恢复，连续确认静止')
            if ack_error:
                self._motion_event('停止应答异常，但连续真实反馈已确认静止；交外层释放恢复')
                raise FeedbackControlError(f'停止应答异常（连续反馈已确认静止）：{ack_error}')
            return confirmed
        self._motion_event(f'停止连续确认超时：{last_error}')
        raise FeedbackControlError(f'未取得连续新鲜的停止状态反馈：{last_error}')

    def _raw_target_n(self):
        """仅接触学习使用相对底数的增量；普通目标与保护始终使用原始压力。"""
        return self.settings.target_n + (self._contact_reference_n if self._learning_contact else 0.0)

    def _control_bands(self):
        """合格范围保持不变，使用更窄的中心触发/停止范围，并让出噪声及半步压力。"""
        tolerance = self.settings.tolerance_n
        if self.learning or self.releasing:
            return tolerance, tolerance
        noise = max(0.01, max(self._noise_windows, default=0.0))
        step_force = max(self._response_slopes, default=0.0)
        floor = max(noise, step_force*0.55)
        trigger = min(tolerance, max(tolerance*0.25, floor))
        stop = min(trigger, max(tolerance*0.1, floor))
        return trigger, stop

    def _observe_response(self, position, force):
        """只用到位后的真实位移和压力变化学习接触刚度，排除空行程及静止蠕变。"""
        previous = self._previous_observation
        self._previous_observation = (position, force)
        if previous is None:
            return
        old_position, old_force = previous
        travel, change = old_position-position, force-old_force
        if self.settings.target_n <= LOW_FORCE_MAX_N:
            # 0.01 N量化会让相邻两步没有可见增量，累计多次真实位移后再估计刚度。
            if min(old_force, force) <= LOW_FORCE_ZERO_N:
                self._low_force_anchor = (position, force)
                return
            if abs(travel) < 1:
                # 静止时的压力变化不作为位移响应，防止把松弛或零漂混入斜率。
                if abs(change) >= .01-1e-9:
                    self._low_force_anchor = (position, force)
                return
            anchor_position, anchor_force = self._low_force_anchor or previous
            if (anchor_position-position)*travel <= 0:
                anchor_position, anchor_force = previous
            span, rise = anchor_position-position, force-anchor_force
            if abs(rise) >= .02-1e-9 and abs(span) >= 1 and span*rise > 0:
                self._response_slopes.append(abs(rise/span))
                self._low_force_anchor = (position, force)
            else:
                self._low_force_anchor = (anchor_position, anchor_force)
            return
        if (min(old_force, force) > 0.05 and abs(travel) >= 1
                and abs(change) >= 0.02 and travel*change > 0):
            self._response_slopes.append(abs(change/travel))

    def _adjustment_steps(self, error, force):
        """最大步数仅作上限；接触后按实测刚度估算步数，反复过冲时自动减弱。"""
        if (self._last_adjustment_error is not None
                and error*self._last_adjustment_error < 0):
            self._adjustment_gain = max(0.1, self._adjustment_gain*0.5)
            self._reversal_cap = max(1, self._last_adjustment_size//2)
        scale = self.profile.steps_per_turn/3200
        if not self._response_slopes and force <= 0.05 and error > 0:
            # 未接触时不知道材料刚度；低目标压力限制探测步长，避免盲走大段。
            size = math.floor(128*scale*max(0.1, self.settings.target_n))
        else:
            # 初始先保守微调；取最近几次刚度的较大值，避免松弛造成步数放大。
            slope = max(self._response_slopes) if self._response_slopes else 0.04/scale
            _, stop_band = self._control_bands()
            size = math.floor(self._adjustment_gain*(abs(error)-stop_band*0.5)/slope)
            if not self.learning and abs(error) <= self.settings.tolerance_n+1e-9:
                # 已在合格范围内时只作小幅中心修正，避免一次跨过整个中心区。
                size = min(size, max(1, math.floor(self.settings.tolerance_n*0.25/slope)))
        size = max(1, min(self.settings.max_step, size))
        # 尚无实测或核验过的学习刚度时，不能因提高步数上限而扩大盲探距离。
        if not self._response_slopes:
            size = min(size, max(1, round(256*scale)))
        if self.settings.target_n <= LOW_FORCE_MAX_N:
            if self._response_slopes and abs(error) > max(.02, self.settings.tolerance_n*2)+1e-9:
                # 已有实测/核验刚度时按压力预算补压，不能仍被固定2步截断。
                budget = min(.08, self.settings.target_n*.5, abs(error)*.75)
                cap = min(max(1, math.floor(32*scale)),
                          max(1, math.floor(budget/max(self._response_slopes))))
            else:
                # 未知刚度和目标附近仍保留低速小步，避免一次跨过容差带。
                cap = max(1, math.floor((2 if force > LOW_FORCE_ZERO_N else 4)*scale))
            size = min(size, cap)
        if self._reversal_cap is not None:
            size = min(size, self._reversal_cap)
        self._last_adjustment_error = error
        self._last_adjustment_size = size
        return size

    def _check(self):
        if self.cancel.is_set():
            raise FeedbackCancelled("已停止调节；压力可能仍在，请按需点击释放")
        if self.clock() >= self.deadline:
            raise FeedbackControlError("操作超时，停止调节；未自动释放")

    def _read_pressure(self):
        # 释放只由位置决定终点；压力异常仅用于到位后的卸载诊断，不中断向上回位。
        try:
            return self.samples.read(self.clock(), allow_negative=self.releasing)
        except FeedbackControlError:
            if not self.releasing:
                raise
            return math.nan, math.nan, None

    def _poll(self):
        self._check()
        sample = self.motor.read_feedback()
        self._trace_motor(sample)
        now = self.clock()
        force, stamp, sequence = self._read_pressure()
        position = float(sample["position_counts"])
        speed = float(sample["measured_speed_rpm"])
        current = float(sample["phase_current_ma"])
        age = now - float(sample["sampled_monotonic"])
        if not all(math.isfinite(v) for v in (position, speed, current, age)) or not 0 <= age <= 0.5:
            raise FeedbackControlError("电机反馈过期或无效")
        self.on_motor(sample)
        if int(sample["running_state"]) in (3, 4, 5):
            raise FeedbackControlError("电机运行状态异常")
        if abs(current) > self.settings.max_current_ma:
            raise FeedbackControlError("电机电流超过设置上限")
        if not self.releasing and force >= self.settings.force_limit_n:
            raise FeedbackControlError(f"压力 {force:.2f} N 达到保护上限")
        if position < self.release - self.settings.max_travel:
            raise FeedbackControlError(
                f"实际位置{position:g} step超过最大下压行程{self.settings.max_travel} step（释放点{self.release}）")
        if position > self.release + self.release_tolerance:
            raise FeedbackControlError(
                f"实际位置{position:g} step高于释放点{self.release}，偏差{position-self.release:+g} step，"
                f"超过回位容差{self.release_tolerance} step；请核对机械位置")
        self.peak_n = max(self.peak_n, force)
        # 首次进入容差只记录瞬时读数；稳定到位另行确认，不能混为同一个时间。
        if (not self.releasing and not self.learning and self.first_in_band_s is None
                and abs(self.settings.target_n-force) <= self.settings.tolerance_n+1e-9):
            self.first_in_band_s = now-self.started
        if now - self.last_record >= 0.1 and (self.releasing or sequence != self.last_record_sequence):
            self.last_record = now
            self.last_record_sequence = sequence
            error = self._raw_target_n() - force
            trigger_band, stop_band = self._control_bands()
            if self.hold_started is not None and not self.releasing:
                self.hold_errors.append(error)
            self.on_record(dict(elapsed_s=now-self.started, phase=self.phase,
                                force_n=force, target_n=self._raw_target_n(),
                                tolerance_n=self.settings.tolerance_n, error_n=error,
                                control_trigger_n=trigger_band, control_stop_n=stop_band,
                                pressure_noise_n=max(0.01, max(self._noise_windows, default=0.0)),
                                position_steps=position, travel_steps=self.release-position,
                                current_ma=current, pressure_age_s=now-stamp,
                                pressure_sequence=sequence, command_steps=self.command_steps,
                                response_n_per_step=max(self._response_slopes) if self._response_slopes else None,
                                adjustment_gain=self._adjustment_gain,
                                learning_point_count=len(self.learning_points),
                                using_learned_map=self.used_learned_map,
                                using_contact_approach=self.used_contact_approach,
                                contact_zero_travel=self._contact_zero_travel,
                                contact_travel=self._contact_travel,
                                contact_force_n=self._contact_force_n,
                                contact_reference_n=(self._contact_reference_n if self._contact_reference_confirmed else None),
                                first_in_band_s=self.first_in_band_s,
                                stable_arrival_s=self.stable_arrival_s,
                                hold_elapsed_s=None if self.hold_started is None else now-self.hold_started,
                                hold_remaining_s=None if self.hold_started is None else max(0, self.settings.hold_s-(now-self.hold_started))))
        self._check()
        return sample, force, stamp, sequence

    def _wait(self):
        self.cancel.wait(self.POLL_S)
        self._check()

    def _window(self):
        """运动后重新采样；连续静止时滚动更新同一新鲜窗口，不重复等整窗。"""
        if self._stationary_ready is None:
            self._stationary_ready = max(self.clock(), self._settle_until)
        ready = self._stationary_ready
        timeout = max(self.clock(), ready) + 1.2
        frames = self._stationary_samples
        entry_sequence = frames[-1][2] if frames else None
        corrections = 0
        while True:
            sample, force, stamp, sequence = self._poll()
            if (abs(float(sample["measured_speed_rpm"])) > 1
                    or int(sample['running_state']) not in (0, 1) or not sample.get('in_position', True)):
                self._motion_event('静止采样窗口收到运动反馈，先停止并连续确认')
                frames.clear()
                self._window_in_band, self._window_band_since = False, None
                if corrections >= 2:
                    raise FeedbackControlError('采样期间反复出现运动反馈，需释放后重新寻压')
                corrections += 1
                self._stop_and_confirm()
                self._settle_until = self.clock()+self.settings.settle_s
                self._stationary_ready = ready = self._settle_until
                timeout, entry_sequence = ready+1.2, None
                self._motion_event('连续反馈确认静止，重新累计完整压力窗口')
                continue
            if math.isfinite(force) and stamp >= ready and (not frames or sequence != frames[-1][2]):
                # 压力采样间断后重新累计窗口，不用间断前的样本凑跨度。
                if frames and (stamp <= frames[-1][1] or stamp-frames[-1][1] > .2+1e-9):
                    frames.clear()
                    self._stationary_ready = ready = stamp
                frames.append((force, stamp, sequence))
                # 保留覆盖最近0.2秒的边界样本，并至少保留3帧。
                while len(frames) > 3 and frames[1][1] <= stamp-.2+1e-9:
                    frames.popleft()
            if (len(frames) >= 3 and sequence != entry_sequence
                    and stamp-frames[0][1] >= .2-1e-9):
                values = [frame[0] for frame in frames]
                first_stamp = frames[0][1]
                median = statistics.median(values)
                self._window_force_range = (min(values), max(values))
                if self._learning_contact and max(abs(value-self._contact_reference_n) for value in values) <= CONTACT_ZERO_N:
                    self._contact_zero_travel = max(0, self.release-float(sample["position_counts"]))
                self._window_in_band = all(abs(value-self._raw_target_n()) <= self.settings.tolerance_n+1e-9
                                           for value in values)
                self._window_band_since = first_stamp if self._window_in_band else None
                self._noise_windows.append(3*statistics.median(abs(value-median) for value in values))
                return sample, median
            if self.clock() >= timeout:
                raise FeedbackControlError("到位后的新压力样本不足")
            self._wait()

    def _move(self, delta, approach_force_n=None, speed_override=None, *, free_travel=False):
        """一次仅发一个有界命令，核对真实位置及停止状态后才允许下一次动作。"""
        sample, force, _, _ = self._poll()
        _, stop_band = self._control_bands()
        if abs(float(sample["measured_speed_rpm"])) > 1:
            raise FeedbackControlError("电机未停止，禁止叠加位置命令")
        if not self.releasing:
            error = self._raw_target_n() - force
            if (delta < 0 and error <= stop_band
                    or delta > 0 and error >= -stop_band
                    or approach_force_n is not None and force >= approach_force_n):
                if free_travel:
                    self._contact_fast_aborted = True
                return
        origin = int(sample["position_counts"])
        if self.releasing:
            # 释放始终以最新反馈计算到固定释放点的距离，禁止坐标变化后反向下压。
            delta = self.release-origin
            if delta < 0:
                if abs(delta) <= self.release_tolerance:
                    return
                raise FeedbackControlError("当前位置高于释放点，禁止用下压方式释放")
            if delta == 0:
                return
        target = origin + int(delta)
        # 从释放点上方的容差带开始时，允许小步向下接近；不向上扩大运动边界。
        upper_target = max(self.release, origin) if delta < 0 else self.release
        if not self.release - self.settings.max_travel <= target <= upper_target:
            raise FeedbackControlError("下一步将超过行程限制，停止调节")
        self.motor.prepare_motion()
        self._check()
        # prepare_motion可能耗时，发令前再次确认压力有效及保护上限。
        force, pressure_stamp, _ = self._read_pressure()
        if free_travel and self.clock()-pressure_stamp > .15:
            raise FeedbackControlError("快速空行程要求压力反馈不超过0.15秒，请检查压力计刷新")
        if not self.releasing and force >= self.settings.force_limit_n:
            raise FeedbackControlError("发令前压力已达到保护上限")
        if not self.releasing:
            error = self._raw_target_n()-force
            if (delta < 0 and error <= stop_band
                    or delta > 0 and error >= -stop_band
                    or approach_force_n is not None and force >= approach_force_n):
                if free_travel:
                    self._contact_fast_aborted = True
                return
        self.command_steps = int(delta)
        speed = self.settings.release_speed if self.releasing else (
            self.settings.speed if speed_override is None else speed_override)
        rpm = self.profile.speed_rpm(speed)
        if not self.releasing and self.settings.target_n <= LOW_FORCE_MAX_N and not free_travel:
            # 学习位置接近最多5 RPM，低压力寻压和保持最多2 RPM；释放速度独立。
            rpm = min(rpm, 5 if approach_force_n is not None else 2)
        motion_timeout = self.profile.motion_timeout(delta, rpm, self.settings.acceleration)
        if free_travel:
            table = self.learned_map
            if (self.releasing or table is None or table.contact_zero_travel is None
                    or not table.matches(self.profile, self.release)
                    or self.release-target > table.contact_zero_travel-self.release_tolerance*2
                    or delta >= 0 or approach_force_n != self._contact_reference_n+.01
                    or not self._contact_reference_confirmed):
                raise FeedbackControlError("快速空行程缺少有效接触边界或越过预留距离")
        if self.releasing:
            # 回位耗时按实际距离和释放速度计算，不使用寻压超时截断低速长行程释放。
            self.deadline = self.clock() + motion_timeout + self.settings.settle_s + 2
        # 任何实际运动都会作废静止采样，下一窗口仍须等待材料响应并采集完整新窗。
        self._stationary_samples.clear()
        self._stationary_ready = None
        self.motor.move_relative_steps(int(delta), speed_rpm=rpm,
                                       accel=self.settings.acceleration,
                                       retry_cancelled=self.cancel.is_set)
        self.move_count += 1
        expiry = self.clock() + motion_timeout
        last_position, stable = None, 0
        while self.clock() < expiry:
            sample, moving_force, pressure_stamp, _ = self._poll()
            if free_travel and self.clock()-pressure_stamp > .15:
                raise FeedbackControlError("快速空行程压力反馈超过0.15秒，停止运动")
            position = int(sample["position_counts"])
            stable = stable + 1 if position == last_position else 0
            last_position = position
            if ((position-origin)*delta < 0
                    and abs(position-origin) > self.profile.tolerance(50)):
                raise FeedbackControlError("电机实际运动方向与命令不符")
            if (not self.releasing and abs(position-target) > self.profile.tolerance(50)
                    and self._pressure_move_should_stop(delta, moving_force, approach_force_n, stop_band, sample)):
                # 运动途中已进入目标范围则提前刹停，不再等整段走完；仍确认新鲜停止反馈。
                try:
                    self._stop_and_confirm()
                except FeedbackCancelled:
                    raise
                except FeedbackControlError as exc:
                    raise FeedbackControlError(f'压力到达目标后停止未确认：{exc}') from exc
                if free_travel:
                    self._contact_fast_aborted = True
                self._settle_until = self.clock()+self.settings.settle_s
                return
            if self._position_move_complete(sample, origin, target, delta, stable):
                self.command_steps = 0
                self._settle_until = self.clock()+self.settings.settle_s
                return
            self._wait()
        raise FeedbackControlError("电机到位超时；运动命令未重发")

    def _position_move_complete(self, sample, origin, target, delta, stable):
        """默认沿用真实同向位移确认；独立实验控制器可细化编码器量化处理。"""
        tolerance = self.release_tolerance if self.releasing else self.profile.tolerance(50)
        position = int(sample['position_counts'])
        return (abs(position-target) <= tolerance and (position-origin)*delta > 0 and stable >= 2
                and abs(float(sample['measured_speed_rpm'])) <= 1 and sample.get('in_position', True))

    def _pressure_move_should_stop(self, delta, force, approach_force_n, stop_band, sample):
        """默认压力停止阈值保持不变，独立控制器可增加制动提前量。"""
        return ((delta < 0 and force >= min(self._raw_target_n()-stop_band,
                                            approach_force_n if approach_force_n is not None else math.inf))
                or (delta > 0 and force <= self._raw_target_n()+stop_band))

    def run(self, release_only=False, learning=False, *, acquire_only=False,
            require_zero_start=True, approach_position=None):
        result = {"ok": False}
        stop_confirmed = False
        try:
            if learning and (release_only or self.settings.target_n != 30):
                raise FeedbackControlError("学习模式必须以30 N为目标，且不能同时请求释放模式")
            self.learning = learning
            self.settings.validate(self.profile, release_only)
            self.releasing = release_only
            self.motor.prepare_motion()
            sample, force, _, _ = self._poll()
            if not release_only and require_zero_start:
                if abs(int(sample["position_counts"])-self.release) > self.release_tolerance:
                    raise FeedbackControlError(f"请先回到释放点{self.release} step附近（允许±{self.release_tolerance} step）")
                contact_mode = (self.settings.target_n <= LOW_FORCE_MAX_N and self.learned_map is not None
                                and self.learned_map.contact_travel is not None
                                and self.learned_map.matches(self.profile, self.release))
                zero_limit = (LOW_FORCE_ZERO_N if learning or self.settings.target_n <= LOW_FORCE_MAX_N else .1)
                if abs(force) > zero_limit:
                    raise FeedbackControlError(f"启动前压力须在 ±{zero_limit:.2f} N 内，请先卸载并将压力计校零")
                if zero_limit <= LOW_FORCE_ZERO_N:
                    self.phase = "低压力零点确认"
                    _, reference = self._window()
                    if max(abs(value) for value in self._window_force_range) > zero_limit:
                        raise FeedbackControlError("低压力底数超出±0.02 N；请先卸载并校零")
                    if learning or contact_mode:
                        if self._window_force_range[1]-self._window_force_range[0] > CONTACT_ZERO_N:
                            raise FeedbackControlError("接触点底数不稳定；请等待压力稳定后再开始")
                        self._contact_reference_n = reference
                        self._contact_reference_confirmed = True
            self.motor.set_current_limit_ma(self.settings.max_current_ma)
            if release_only:
                result = self._release()
            elif learning:
                result = self._learn()
            else:
                if approach_position is not None:
                    position = int(sample["position_counts"])
                    delta = int(approach_position)-position
                    if delta < 0:
                        self.phase = "查表快速接近"
                        self._move(delta, approach_force_n=self.settings.target_n*0.8)
                else:
                    self._fast_approach()
                result = self._regulate(stop_on_stable=acquire_only)
        except FeedbackCancelled as exc:
            result = dict(ok=False, cancelled=True, message=str(exc))
        except Exception as exc:
            self._motion_event(exc)
            # 保留原始异常类别；学习恢复只能识别明确的通信超时，不能按报错文字重试保护故障。
            from zdt_x42s_can_driver import ZDTTimeoutError
            result = dict(ok=False, message=f"已终止：{exc}；请检查压力和实际位置",
                          error_type=type(exc).__name__, failure_phase=self.phase,
                          communication_timeout=isinstance(exc, ZDTTimeoutError))
        finally:
            try:
                self._stop_and_confirm(monitor_operation=False)
                stop_confirmed = True
            except Exception as exc:
                result.update(ok=False, message=f"{result.get('message', '')}；停止命令未确认：{exc}；请立即检查电机")
        errors = self.hold_errors
        result.update(stop_confirmed=stop_confirmed,
                      motion_diagnostics=list(self.motion_diagnostics),
                      first_in_band_s=self.first_in_band_s, stable_arrival_s=self.stable_arrival_s,
                      used_learned_map=self.used_learned_map,
                      used_contact_approach=self.used_contact_approach,
                      move_count=self.move_count, peak_n=self.peak_n if math.isfinite(self.peak_n) else None,
                      hold_sample_count=len(errors),
                      hold_mean_error_n=statistics.fmean(errors) if errors else None,
                      hold_rmse_n=math.sqrt(statistics.fmean(e*e for e in errors)) if errors else None,
                      hold_in_band_fraction=sum(abs(e)<=self.settings.tolerance_n+1e-9 for e in errors)/len(errors) if errors else None)
        return result

    def _fast_approach(self):
        """一次接近学习位置的保守前方，全程监测实时压力，接近后交回闭环。"""
        table = self.learned_map
        if table is None or not table.matches(self.profile, self.release):
            return
        if self.settings.target_n <= LOW_FORCE_MAX_N and table.contact_travel is not None:
            self._contact_approach(table)
            return
        travel = table.approach_travel(self.settings.target_n, self.release_tolerance*2)
        if travel is None or travel <= 0 or travel > self.settings.max_travel:
            return
        sample, _, _, _ = self._poll()
        delta = self.release-travel-int(sample["position_counts"])
        if delta < 0:
            self.used_learned_map = True
            self.phase = "学习位置快速接近"
            self._move(delta, approach_force_n=self.settings.target_n*0.8,
                       speed_override=self.settings.approach_speed)

    def _contact_approach(self, table):
        """只在实测空行程加速，之后用单段有界低速运动接触并初步加载。"""
        goal = table.travel_at(self.settings.target_n*.8)
        if goal is None or goal > self.settings.max_travel:
            return
        free_goal = max(0, math.floor(table.contact_zero_travel-self.release_tolerance*2))
        sample, force = self._window()
        # 中途承压调用只允许普通反馈调节；不能再次使用空行程高速。
        if (not self._contact_reference_confirmed
                or abs(self._contact_reference_n-table.contact_reference_n) > CONTACT_ZERO_N
                or max(abs(value-self._contact_reference_n) for value in self._window_force_range) > CONTACT_ZERO_N):
            return
        self.used_learned_map = self.used_contact_approach = True
        self._contact_zero_travel = table.contact_zero_travel
        self._contact_travel = table.contact_travel
        self._contact_force_n = table.contact_force_n
        delta = self.release-free_goal-int(sample['position_counts'])
        if delta < 0:
            self.phase = "接触点前快速空行程"
            self._move(delta, approach_force_n=self._contact_reference_n+.01,
                       speed_override=self.settings.approach_speed, free_travel=True)
        sample, force = self._window()
        # 接触提前说明位置参考发生变化，交回小步闭环，不能继续按旧表盲走受力段。
        if self._contact_fast_aborted or max(abs(value-self._contact_reference_n) for value in self._window_force_range) > CONTACT_ZERO_N:
            return
        contact_goal = max(free_goal, math.floor(goal-self.profile.tolerance(50)))
        delta = self.release-contact_goal-int(sample['position_counts'])
        if delta < 0:
            self.phase = "低速接触并初步加载"
            self._move(delta, approach_force_n=self.settings.target_n*.8,
                       speed_override=min(self.settings.speed, 2))

    def _seed_learned_response(self, position, force):
        """位置符合学习曲线时，用目标区间插值估算首轮补压，小力仍取较大刚度。"""
        contact_floor = LOW_FORCE_ZERO_N if self.settings.target_n <= LOW_FORCE_MAX_N else .05
        if not self.used_learned_map or self._response_slopes or force <= contact_floor:
            return
        table = self.learned_map
        expected = table.travel_at(force)
        target = table.travel_at(self.settings.target_n)
        if expected is None or target is None or force >= self.settings.target_n:
            return
        if abs((self.release-position)-expected) > max(self.release_tolerance*2, expected*.05):
            return
        points = table.loading_points()
        slopes = [(f1-f0)/(x1-x0) for (f0, x0), (f1, x1) in zip(points, points[1:])
                  if f1 >= force and f0 < self.settings.target_n]
        if slopes:
            # 跨越分段拐点时，远端一小段较陡曲线不应拖慢整段补压。
            # 仅在已实测范围内使用两点插值距离；小力保留原压力增量预算。
            slope = (self.settings.target_n-force)/(target-expected)
            self._response_slopes.append(max(slopes) if self.settings.target_n <= LOW_FORCE_MAX_N else slope)
            self._adjustment_gain = 0.95

    def _learn(self):
        """一次加载到30 N，在关键压力处确认后记录；全部完成并卸载后才返回有效表。"""
        original = self.settings
        ratio = original.tolerance_n/original.target_n
        if not self._contact_reference_confirmed:
            raise FeedbackControlError("学习前必须在释放点确认稳定底数")
        self._learning_contact = True
        self._contact_zero_travel = 0.0
        self.settings = replace(original, target_n=.015, tolerance_n=.005)
        try:
            contact = self._regulate(stop_on_stable=True)
            self._contact_travel = self.release-contact['actual_position_steps']
            self._contact_force_n = contact['actual_force_n']
            if self._contact_travel <= self._contact_zero_travel:
                raise FeedbackControlError("未确认真实下压后的接触位置，请检查零点波动与释放点")
        finally:
            self._learning_contact = False
            self.settings = original
        for target in LEARNING_TARGETS_N:
            tolerance = LOW_FORCE_TOLERANCE_N if target <= LOW_FORCE_MAX_N else max(0.02, target*ratio)
            # 首个加载点必须高于接触实测值，避免正底数下直接复用接触位置。
            if not self.learning_points:
                tolerance = min(tolerance, max(.005, target-self._contact_force_n-.005))
            self.settings = replace(original, target_n=target, tolerance_n=tolerance)
            # 阶段切换不应被当作压力过冲，局部响应估计继续保留。
            self._last_adjustment_error = None
            self._reversal_cap = None
            self._adjustment_gain = 0.75
            result = self._regulate(stop_on_stable=True)
            self.learning_points.append((result["actual_force_n"], self.release-result["actual_position_steps"]))
        self.settings = original
        table = LearnedPressureMap(self.profile.model, self.profile.namespace, self.profile.steps_per_turn,
                                   self.release, tuple(self.learning_points), self._contact_zero_travel,
                                   self._contact_travel, self._contact_force_n, self._contact_reference_n)
        self.releasing = True
        result = self._release()
        result.update(learned_map=table, message=f"学习完成：已确认{table.contact_force_n:.2f} N接触点及{len(table.points)}个加载点至约30 N，并回到释放点")
        return result

    def _regulate(self, stop_on_stable=False):
        stable_since = None
        while True:
            self.phase = (f"学习接触点（底数{self._contact_reference_n:+.2f} N，增量0.01～0.02 N）" if self._learning_contact else
                          f"学习 {self.settings.target_n:g} N" if self.learning else "寻压" if self.hold_started is None else "恒力保持")
            sample, force = self._window()
            self._seed_learned_response(float(sample["position_counts"]), force)
            self._observe_response(float(sample["position_counts"]), force)
            now = self.clock()
            error = self._raw_target_n()-force
            if self.hold_started is not None and now-self.hold_started >= self.settings.hold_s:
                return dict(ok=True, message="保持时间结束，已停止调节并保持位置；按需点击“停止并释放”")
            if abs(error) <= self.settings.tolerance_n+1e-9 and self._window_in_band:
                if stable_since is None:
                    stable_since = self._window_band_since
                else:
                    # 压力采样有间断时，稳定时间也必须从间断后的新样本重新累计。
                    stable_since = max(stable_since, self._stationary_ready)
                if self.hold_started is None and now-stable_since >= self.settings.stable_s:
                    if not self.learning:
                        self.stable_arrival_s = now-self.started
                    if stop_on_stable:
                        return dict(ok=True, actual_force_n=force, actual_position_steps=float(sample["position_counts"]))
                    self.hold_started = now
                    self.deadline = now + self.settings.hold_s + 5
            else:
                stable_since = None
            # 首次已进入合格范围时先静止确认，避免中心微调反复打断到位；保持阶段继续追踪中心。
            if self.hold_started is None and abs(error) <= self.settings.tolerance_n+1e-9:
                continue
            # 合格计时仍按用户容差，调节独立追踪目标中心；跨过中心后重新判断，防止追逐噪声。
            trigger_band, stop_band = self._control_bands()
            if abs(error) <= stop_band or error*self._centering_direction < 0:
                self._centering_direction = 0
            if abs(error) > trigger_band:
                self._centering_direction = 1 if error > 0 else -1
            if self._centering_direction:
                size = self._adjustment_steps(error, force)
                self.phase = "偏低，补压" if error > 0 else "偏高，回退"
                self._move(-size if error > 0 else size)
                stable_since = None

    def _release(self):
        """按独立释放速度发一次完整回位命令，沿途检查保护，到位后确认卸载。"""
        self.phase = "释放"
        sample, _, _, _ = self._poll()
        distance = self.release-int(sample["position_counts"])
        if distance < -self.release_tolerance:
            raise FeedbackControlError("当前位置高于释放点，禁止用下压方式释放")
        if distance > 0:
            self._move(distance)
        try:
            sample, force = self._window()
        except FeedbackControlError as exc:
            if isinstance(exc, FeedbackCancelled):
                raise
            raise FeedbackControlError(f"回位命令已完成，但释放后检查失败：{exc}") from exc
        actual_position = int(sample["position_counts"])
        if abs(actual_position-self.release) > self.release_tolerance:
            raise FeedbackControlError("释放后位置偏离释放点，未追加运动")
        if abs(force) > 0.1:
            raise FeedbackControlError("已到释放位置但压力未归零，未继续运动")
        return dict(ok=True, actual_position_steps=actual_position,
                    release_error_steps=actual_position-self.release,
                    message=f"释放完成：目标{self.release} step，实际{actual_position} step，"
                    f"偏差{actual_position-self.release:+d} step（允许±{self.release_tolerance}）；"
                    "新压力样本确认卸载，保留原释放点")
