"""可见的上位机实验状态机；不读写或替代固件通道状态。"""
from collections import deque
from dataclasses import dataclass, field
import math

from raw_gain import normalize_raw_gains, apply_raw_gains

HOST_STATE_IDLE_NEGATIVE = 4
HOST_STATE_RELEASE_UNDERSHOOT = 5
HOST_NEGATIVE_STATES = (HOST_STATE_IDLE_NEGATIVE, HOST_STATE_RELEASE_UNDERSHOOT)
HOST_RECOVERY_STATES = (2, 3, *HOST_NEGATIVE_STATES)
HOST_STATE_ACTIVATE = 1
HOST_STATE_NAMES = {-2: 'disabled', -1: '初始化', 0: 'idle', HOST_STATE_ACTIVATE: 'activate',
                    2: 'slowback', 3: 'recover', 4: '空闲负向扰动', 5: '释放下冲'}
HOST_STATE_COLORS = {-2: '#444444', -1: '#49324e', 0: '#204f3b',
                     1: '#764821', 2: '#245779', 3: '#535087', 4: '#426575', 5: '#75538a'}

# 键、标签、默认值、最小值、最大值、小数位。全部单位放在标签中。
PARAMETERS = (
    ('input_alpha', '输入低通 alpha', .2, .000001, 1., 6),
    ('signal_alpha', 'Signal 低通 alpha（仅显示）', .1, .000001, 1., 6),
    ('initial_seconds', '释放初始化时长（秒，期间取最大值）', 3., .01, 60., 2),
    ('window', '速度回归窗口（采样帧）', 20, 2, 500, 0),
    ('activate_threshold', '激活差值 input − baseline（count）', 200., 0., 65535000., 2),
    ('activate_frames', '激活连续确认（帧）', 3, 1, 10000, 0),
    ('drop_rate', '快速卸载下降速度（%/s，填正数）', 20., .001, 100000., 3),
    ('drop_frames', '快速卸载连续确认（帧）', 2, 1, 10000, 0),
    ('release_threshold', '慢速释放差值（count）', 50., 0., 65535000., 2),
    ('release_frames', '慢速释放连续确认（帧）', 5, 1, 10000, 0),
    ('slowback_frames', 'slowback 最长保持（采样帧）', 300, 1, 100000, 0),
    ('slowback_tolerance_percent', 'slowback 提前恢复相对基线差值（%）', 10., 0., 100., 3),
    ('slowback_close_frames', 'slowback 提前恢复连续确认（帧）', 3, 1, 10000, 0),
    ('negative_threshold', '负向下冲进入差值（count）', 200., .001, 65535000., 2),
    ('negative_frames', '负向下冲进入连续确认（帧）', 3, 1, 10000, 0),
    ('negative_rebase_seconds', 'recover 残余平台持续稳定时间（秒）', 5., .1, 600., 2),
    ('negative_press_margin', '负向下冲再按超出原零点（count）', 300., 0., 65535000., 2),
    ('negative_press_rate', '负向下冲再按上升速度（%/s）', 20., .001, 100000., 3),
    ('negative_press_frames', '负向下冲再按连续确认（帧）', 3, 1, 10000, 0),
    ('idle_alpha', 'idle 基线跟踪 alpha', .002, .000001, 1., 6),
    ('recover_alpha', 'recover 基线跟踪 alpha', .1, .000001, 1., 6),
    ('recover_tolerance', '恢复允许基线差值（count）', 20., 0., 65535000., 2),
    ('stable_rate', '恢复允许变化速度绝对值（%/s）', 2., 0., 100000., 3),
    ('recover_frames', '恢复稳定连续确认（帧）', 5, 1, 10000, 0),
    ('reload_rate', '重新加载上升速度（%/s）', 20., .001, 100000., 3),
    ('reload_threshold', '重新加载较谷值增加量（count）', 200., 0., 65535000., 2),
    ('reload_reference_margin', '重新加载超出卸载前基线（count，0关闭）', 300., 0., 65535000., 2),
    ('reload_frames', '重新加载连续确认（帧）', 3, 1, 10000, 0),
    ('normalization_floor', '速度归一化最小 input（count）', 1., .000001, 65535000., 6),
    ('gap_seconds', '采样中断重新初始化间隔（秒）', .2, .01, 60., 3),
)


@dataclass(frozen=True)
class HostParameters:
    input_alpha: float = .2
    signal_alpha: float = .1
    initial_seconds: float = 3.
    window: int = 20
    activate_threshold: float = 200.
    activate_frames: int = 3
    drop_rate: float = 20.
    drop_frames: int = 2
    release_threshold: float = 50.
    release_frames: int = 5
    slowback_frames: int = 300
    slowback_tolerance_percent: float = 10.
    slowback_close_frames: int = 3
    negative_threshold: float = 200.
    negative_frames: int = 3
    negative_rebase_seconds: float = 5.
    negative_press_margin: float = 300.
    negative_press_rate: float = 20.
    negative_press_frames: int = 3
    idle_alpha: float = .002
    recover_alpha: float = .1
    recover_tolerance: float = 20.
    stable_rate: float = 2.
    recover_frames: int = 5
    reload_rate: float = 20.
    reload_threshold: float = 200.
    reload_reference_margin: float = 300.
    reload_frames: int = 3
    normalization_floor: float = 1.
    gap_seconds: float = .2

    channel_repress: dict = field(default_factory=dict)

    def __post_init__(self):
        for key, _, _, low, high, decimals in PARAMETERS:
            value = getattr(self, key)
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{key} 超出范围')
            if decimals == 0 and int(value) != value:
                raise ValueError(f'{key} 必须为整数')
            if decimals == 0:
                object.__setattr__(self, key, int(value))
        if not isinstance(self.channel_repress, dict):
            raise ValueError('通道独立配置须为对象')
        normalized = {}
        for channel, values in self.channel_repress.items():
            index = int(channel)
            if str(index) != str(channel) or not 0 <= index < 20:
                raise ValueError('通道编号必须为0至19')
            if not isinstance(values, dict) or set(values) != {'margin', 'confirm_frames'}:
                raise ValueError('通道独立配置需要增量与确认时长')
            margin, duration = float(values['margin']), float(values['confirm_frames'])
            if not math.isfinite(margin) or not 0 <= margin <= 65535000:
                raise ValueError('再按压增量超出范围')
            if not math.isfinite(duration) or not 1 <= duration <= 10000 or duration != int(duration):
                raise ValueError('连续确认须为1至10000整数帧')
            normalized[str(index)] = dict(margin=margin, confirm_frames=duration)
        object.__setattr__(self, 'channel_repress', normalized)
        if self.release_threshold >= self.activate_threshold:
            raise ValueError('慢速释放差值必须小于激活差值')
        if self.recover_alpha < self.idle_alpha:
            raise ValueError('recover alpha 必须不小于 idle alpha')

    def settings(self):
        return dict({key: getattr(self, key) for key, *_ in PARAMETERS},
                    channel_repress={key: dict(value) for key, value in self.channel_repress.items()})

    @classmethod
    def from_settings(cls, settings):
        return cls(**{key: settings[key] for key in (*[row[0] for row in PARAMETERS], 'channel_repress') if key in settings})


class _Channel:
    def __init__(self, window):
        self.state = -1
        self.input = self.baseline = self.signal = 0.
        self.signal_filtered = 0.
        self.rate = math.nan
        self.decision_rate = math.nan
        self.started = None
        self.history = deque(maxlen=window)
        self.activate_count = self.drop_count = self.release_count = 0
        self.reload_count = self.recover_count = self.hold_count = 0
        self.close_count = 0
        self.negative_count = 0
        self.negative_stable_since = None
        self.negative_min = self.negative_max = 0.
        self.trough = 0.
        self.release_reference = None
        self.activation_number = 0
        self.last_activation = None
        self.repress_override = None
        # 恢复来源仅用于选择真实下冲状态，原参考与局部下降参考各自独立。
        self.recovery_source = None
        self.had_negative = False
        self.recovery_high = 0.


    def commit_state(self, state, activation=None):
        # 运行时状态仅由此入口提交，同时集中复位确认计数。
        if state == self.state:
            return
        # 每轮首次按压锁定可信零点，再次加载不得继承更低的恢复基线。
        previous_state = self.state
        if state == HOST_STATE_ACTIVATE:
            self.recovery_source = 'release'
            self.had_negative = False
            self.baseline = (self.release_reference if self.release_reference is not None
                             else self.baseline)
            self.release_reference = self.baseline
        if state == 1 and activation is not None:
            self.activation_number += 1
            self.last_activation = dict(activation, from_state=self.state,
                                        activation_number=self.activation_number)
        # 本轮参考只在首次进入时建立，恢复中的高低平台都不能改写它。
        if state in (-2, 0):
            self.release_reference = None
            self.recovery_source = None
            self.had_negative = False
        # 负向形变保存原零点，重新按压后继续用于卸载保护。
        if state in HOST_NEGATIVE_STATES:
            self.had_negative = True
            self.recovery_source = 'idle' if state == HOST_STATE_IDLE_NEGATIVE else 'release'
            if self.release_reference is None:
                self.release_reference = self.baseline
            self.baseline = self.release_reference
        self.negative_count = 0
        self.negative_stable_since = None
        self.state = state
        self.activate_count = self.drop_count = self.release_count = 0
        self.reload_count = self.recover_count = self.hold_count = 0
        self.close_count = 0
        self.trough = self.input
        # 下冲结束后的新下降必须相对本地平台重新积累，不能再次使用旧零点残差。
        self.recovery_high = (max(self.input, self.baseline)
                              if state == 2 and previous_state == HOST_STATE_ACTIVATE else self.input)

    def activation_evidence(self, timestamp, rate, p):
        # 在提交状态、清空计数及谷值前保存本次真正通过的判据。
        reloading = self.state in (2, 3)
        negative_press = self.state in HOST_NEGATIVE_STATES or (reloading and self.had_negative)
        baseline = self.release_reference if self.release_reference is not None else self.baseline
        evidence = dict(reason=('negative_recovery_press' if negative_press
                                else 'reload' if reloading else 'idle_threshold'),
                        sample_timestamp=timestamp, input_val=self.input, baseline=baseline,
                        difference=self.input - baseline,
                        activate_threshold=p.activate_threshold, pressure_rate=rate,
                        confirmed_frames=self.reload_count if reloading else self.activate_count,
                        required_frames=p.reload_frames if reloading else p.activate_frames)
        if negative_press:
            evidence.update(negative_source=self.recovery_source, negative_reference=self.release_reference,
                            reference_difference=self.input - self.release_reference,
                            negative_press_margin=p.negative_press_margin,
                            negative_press_rate=p.negative_press_rate,
                            negative_press_frames=p.negative_press_frames,
                            required_frames=p.negative_press_frames)
        if reloading:
            reference = self.release_reference if self.release_reference is not None else self.baseline
            evidence.update(trough=self.trough, increase=self.input - self.trough,
                            reload_threshold=p.reload_threshold,
                            reload_rate=p.negative_press_rate if negative_press else p.reload_rate,
                            release_reference=reference, reference_difference=self.input-reference,
                            reload_reference_margin=p.negative_press_margin if negative_press else p.reload_reference_margin)
        if self.repress_override is not None and self.state in HOST_RECOVERY_STATES:
            if negative_press:
                evidence['negative_press_margin'] = self.repress_override['margin']
            if reloading:
                evidence['reload_reference_margin'] = self.repress_override['margin']
            else:
                evidence['negative_press_margin'] = self.repress_override['margin']
            evidence.update(repress_margin=self.repress_override['margin'],
                            required_frames=self.repress_override['confirm_frames'],
                            parameter_source='channel_override')
        return evidence

    def confirm_repress(self, candidate, timestamp, count, frames):
        # 沿用各入口连续帧计数；任一条件失效时调用方将计数清零。
        required = self.repress_override['confirm_frames'] if self.repress_override else frames
        return candidate and count >= required

    def slope_rate(self, p):
        if len(self.history) < p.window or self.input < p.normalization_floor:
            return math.nan
        origin = self.history[0][0]
        times = [t - origin for t, _ in self.history]
        mean_t = sum(times) / len(times)
        mean_y = sum(v for _, v in self.history) / len(times)
        variance = sum((t - mean_t)**2 for t in times)
        if variance <= 0:
            return math.nan
        slope = sum((t - mean_t) * (v - mean_y)
                    for t, (_, v) in zip(times, self.history)) / variance
        return 100. * slope / self.input

    def feed(self, raw, timestamp, enabled, p, repress_override=None):
        self.repress_override = repress_override
        self.decision_rate = math.nan
        if not enabled:
            self.commit_state(-2)
            self.input = self.baseline = self.signal = 0.
            self.rate = math.nan
            return
        if self.started is None:
            self.started = timestamp
            self.input = self.baseline = raw
        else:
            self.input += p.input_alpha * (raw - self.input)
        self.history.append((timestamp, self.input))
        # 各状态都逐采样计算观察速度；窗口不足或分母过小时返回无效值。
        self.rate = self.slope_rate(p)
        self.signal = 0.
        if self.state == -1:
            self.baseline = max(self.baseline, self.input)
            if timestamp - self.started >= p.initial_seconds:
                self.commit_state(0)
            return
        # idle 激活只检查差值；负向恢复还使用实时速度确认稳定。
        rate = self.rate if self.state in (1, *HOST_RECOVERY_STATES) else math.nan
        difference = self.input - self.baseline
        if self.state == 0:
            # 从首个明显下沉候选起冻结基线，确认期间保住原零点。
            if difference < -p.negative_threshold:
                self.negative_count += 1
                self.activate_count = 0
                if self.negative_count >= p.negative_frames:
                    self.commit_state(HOST_STATE_IDLE_NEGATIVE)
                return
            self.negative_count = 0
            self.activate_count = self.activate_count + 1 if difference > p.activate_threshold else 0
            if self.activate_count >= p.activate_frames:
                rate = self.rate
                self.commit_state(1, self.activation_evidence(timestamp, rate, p))
            elif not self.activate_count:
                self.baseline += p.idle_alpha * difference
        elif self.state in HOST_NEGATIVE_STATES:
            # 原零点继续保护再按压；负向状态只描述下降阶段，不负责长期零点恢复。
            reference = self.release_reference
            difference = self.input - reference
            pressing = (difference > max(p.activate_threshold, repress_override['margin'] if repress_override else p.negative_press_margin)
                        and math.isfinite(rate) and rate >= p.negative_press_rate)
            self.activate_count = self.activate_count + 1 if pressing else 0
            if self.confirm_repress(pressing, timestamp, self.activate_count, p.negative_press_frames):
                self.commit_state(1, self.activation_evidence(timestamp, rate, p))
            else:
                # 快速再按候选优先；下降减弱或开始回升后连续确认退出。
                ended = math.isfinite(rate) and rate >= -p.stable_rate and not pressing
                self.recover_count = self.recover_count + 1 if ended else 0
                if self.recover_count >= p.recover_frames:
                    if abs(difference) <= p.recover_tolerance and abs(rate) <= p.stable_rate:
                        self.baseline = reference
                        self.commit_state(0)
                    elif self.state == HOST_STATE_RELEASE_UNDERSHOOT and rate > p.stable_rate:
                        self.commit_state(2)
                    else:
                        self.commit_state(3)
        elif self.state == 1:
            self.drop_count = self.drop_count + 1 if rate <= -p.drop_rate else 0
            self.release_count = self.release_count + 1 if difference < p.release_threshold else 0
            if self.drop_count >= p.drop_frames or self.release_count >= p.release_frames:
                self.commit_state(2)
        elif self.state in (2, 3):
            self.trough = min(self.trough, self.input)
            reference = self.release_reference if self.release_reference is not None else self.baseline
            # 低于原参考的经历保留再按保护，但不因此强行切回下冲状态。
            if reference - self.input > p.negative_threshold:
                self.had_negative = True
            # 仅新的明显下降才重新进入负向状态；处于旧零点下方本身不构成下冲。
            falling = math.isfinite(rate) and rate < -p.stable_rate
            new_drop = (falling and self.recovery_high - self.input > p.negative_threshold
                        and reference - self.input > p.negative_threshold)
            self.negative_count = self.negative_count + 1 if new_drop else 0
            if new_drop:
                self.negative_stable_since = None
                self.reload_count = self.recover_count = self.close_count = 0
                if self.negative_count >= p.negative_frames:
                    self.commit_state(HOST_STATE_IDLE_NEGATIVE if self.recovery_source == 'idle'
                                      else HOST_STATE_RELEASE_UNDERSHOOT)
                self.decision_rate = rate
                return
            # 平稳平台推进局部参考，缓慢漂移不会沿用历史高点重复触发。
            if math.isfinite(rate) and abs(rate) <= p.stable_rate:
                self.recovery_high = self.input
            elif self.input > self.recovery_high:
                self.recovery_high = self.input
            # 回到原释放基线以下的快速回弹继续恢复；越过基线后才冻结新加载候选。
            negative_cycle = self.had_negative
            margin = repress_override['margin'] if repress_override else (p.negative_press_margin if negative_cycle else p.reload_reference_margin)
            reload_rate = p.negative_press_rate if negative_cycle else p.reload_rate
            reload_frames = p.negative_press_frames if negative_cycle else p.reload_frames
            guard_enabled = margin > 0
            rising_candidate = (rate >= reload_rate
                                and (not guard_enabled or self.input > reference))
            reloading = (rising_candidate
                         and (negative_cycle or self.input - self.trough > p.reload_threshold)
                         and self.input - reference > p.activate_threshold
                         and (not guard_enabled or self.input > reference + margin))
            self.reload_count = self.reload_count + 1 if reloading else 0
            if self.confirm_repress(reloading, timestamp, self.reload_count, reload_frames):
                self.commit_state(1, self.activation_evidence(timestamp, rate, p))
            elif self.state == 2:
                self.hold_count += 1
                # 基线两侧都检查相对差值；乘法判定避免基线为零时除零。
                # 快速上升候选暂停提前恢复，给重新加载的连续确认留出机会。
                close = (abs(difference) <= abs(self.baseline) * p.slowback_tolerance_percent / 100.
                         and not rate >= p.reload_rate)
                self.close_count = self.close_count + 1 if close else 0
                if (self.hold_count >= p.slowback_frames
                        or self.close_count >= p.slowback_close_frames):
                    self.commit_state(3)
            elif not rising_candidate:
                # 回弹未越过卸载前基线时继续快速跟踪，避免低基线因正速度而一直冻结。
                self.baseline += p.recover_alpha * difference
                stable = (abs(self.input - self.baseline) <= p.recover_tolerance
                          and abs(rate) <= p.stable_rate)
                if stable and self.input < reference - p.recover_tolerance:
                    self.had_negative = True
                near_reference = abs(self.input - reference) <= p.recover_tolerance
                # 普通释放保留原有快速恢复；经历负向形变后按原零点保护完成确认。
                reference_stable = (abs(rate) <= p.stable_rate and near_reference
                                    if self.had_negative else stable and self.input >= reference - p.recover_tolerance)
                self.recover_count = self.recover_count + 1 if reference_stable else 0
                if self.recover_count >= p.recover_frames:
                    if self.had_negative:
                        self.baseline = reference
                    self.commit_state(0)
                elif abs(rate) <= p.stable_rate and not near_reference:
                    # 原参考保留到平台连续稳定完成，工作基线跟随不代表零点已经提交。
                    if self.negative_stable_since is None:
                        self.negative_stable_since = timestamp
                        self.negative_min = self.negative_max = self.input
                    self.negative_min = min(self.negative_min, self.input)
                    self.negative_max = max(self.negative_max, self.input)
                    if self.negative_max - self.negative_min > p.recover_tolerance:
                        self.negative_stable_since = timestamp
                        self.negative_min = self.negative_max = self.input
                    elif timestamp - self.negative_stable_since >= p.negative_rebase_seconds:
                        self.baseline = self.input
                        self.commit_state(0)
                else:
                    self.negative_stable_since = None
            else:
                self.recover_count = 0
                self.negative_stable_since = None
        # 单独保存本帧参与判定的速度；公开速度持续输出，不受真实状态切换遮蔽。
        self.decision_rate = rate
        if self.state == 1:
            self.signal = max(self.input - self.baseline, 0.)


class HostStateMachine:
    CHANNELS = 20

    def __init__(self, parameters=None, history_length=1000):
        self.parameters = parameters or HostParameters()
        self.gains = normalize_raw_gains(None)
        self.history_length = history_length
        self.last_sequence = 0
        self.last_timestamp = None
        self.version = 0
        self.generation = 0
        self.reset('等待 Raw 数据；请保持释放')

    def reset(self, reason='请保持释放，重新初始化'):
        # 保留已消费的序号，防止重新初始化后回放旧的缓存帧。
        self.channels = [_Channel(self.parameters.window) for _ in range(self.CHANNELS)]
        self.histories = {key: [deque(maxlen=self.history_length) for _ in self.channels]
                          for key in ('raw', 'baseline', 'signal', 'signal_filtered', 'fast', 'slow', 'state')}
        self.last_timestamp = None
        self.reason = reason
        self.reset_reason = reason
        self.generation += 1
        self.version += 1

    def configure(self, parameters):
        if parameters != self.parameters:
            self.parameters = parameters
            self.reset('参数已改变；请保持释放，重新初始化')

    def set_gains(self, gains):
        normalized = normalize_raw_gains(gains)
        if normalized != self.gains:
            self.gains = normalized
            self.reset('Raw 增益已改变；请保持释放，重新初始化')

    def set_history_length(self, length):
        self.history_length = length
        for key in self.histories:
            self.histories[key] = [deque(values, maxlen=length) for values in self.histories[key]]
        self.version += 1

    def expire(self, now):
        if self.last_timestamp is not None and now - self.last_timestamp > self.parameters.gap_seconds:
            self.reset('采样中断；等待 Raw 数据，恢复后请保持释放')

    def feed(self, sample):
        sequence = sample['sequence']
        if sequence <= self.last_sequence:
            return None
        previous_sequence = self.last_sequence
        self.last_sequence = sequence
        raw = sample.get('raw', ())
        timestamp = sample['sample_timestamp']
        if (not sample.get('has_raw') or sample.get('channel_count') != self.CHANNELS
                or len(raw) != self.CHANNELS or not all(math.isfinite(v) and v >= 0 for v in raw)
                or not math.isfinite(timestamp)):
            self.reset('需要完整的 20 通道 Raw；Run/无效采样不参与计算')
            return None
        if self.last_timestamp is not None:
            if timestamp <= self.last_timestamp:
                self.reset('采样时间异常；重新初始化')
            elif sequence != previous_sequence + 1 or timestamp - self.last_timestamp > self.parameters.gap_seconds:
                self.reset('采样缺口；请保持释放，重新初始化')
        values = apply_raw_gains(raw, self.gains)
        activation_events = []
        for index, (channel, value, gain) in enumerate(zip(self.channels, values, self.gains)):
            activation_number = channel.activation_number
            channel.feed(value, timestamp, gain > 0, self.parameters,
                         self.parameters.channel_repress.get(str(index)))
            # 显示低通逐采样更新，包含释放后的零值；不反馈到基线、速度或状态判定。
            if gain > 0:
                channel.signal_filtered += self.parameters.signal_alpha * (channel.signal - channel.signal_filtered)
            else:
                channel.signal_filtered = 0.
            if channel.activation_number != activation_number:
                activation_events.append(dict(channel=index, **channel.last_activation))
        self.last_timestamp = timestamp
        self.reason = '运行中' if all(c.state != -1 for c in self.channels) else f'初始化中，请保持释放 · {self.reset_reason}'
        frame = dict(sequence=sequence, sample_timestamp=timestamp,
                     generation=self.generation, reset_reason=self.reset_reason,
                     activation_events=activation_events,
                     input_val=[c.input for c in self.channels], baseline=[c.baseline for c in self.channels],
                     signal=[c.signal for c in self.channels],
                     signal_filtered=[c.signal_filtered for c in self.channels],
                     signal_alpha=self.parameters.signal_alpha,
                     pressure_rate=[c.rate for c in self.channels],
                     decision_rate=[c.decision_rate for c in self.channels],
                     release_reference=[c.release_reference for c in self.channels],
                     state=[c.state for c in self.channels],
                     recovery_source=[c.recovery_source for c in self.channels],
                     state_schema_version=2)
        for key, source in (('raw', 'input_val'), ('baseline', 'baseline'), ('signal', 'signal'),
                            ('signal_filtered', 'signal_filtered'),
                            ('fast', 'pressure_rate'), ('state', 'state')):
            for history, value in zip(self.histories[key], frame[source]):
                history.append(value)
        for history in self.histories['slow']:
            history.append(math.nan)
        self.version += 1
        return frame

    def snapshot(self):
        return {key: [list(values) for values in channels] for key, channels in self.histories.items()}
