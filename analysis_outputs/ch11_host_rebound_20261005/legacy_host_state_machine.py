"""可见的上位机实验状态机；不读写或替代固件通道状态。"""
from collections import deque
from dataclasses import dataclass
import math

from raw_gain import normalize_raw_gains, apply_raw_gains

HOST_STATE_ACTIVATE = 1
HOST_STATE_NAMES = {-2: 'disabled', -1: '初始化', 0: 'idle', HOST_STATE_ACTIVATE: 'activate',
                    2: 'slowback', 3: 'recover'}
HOST_STATE_COLORS = {-2: '#444444', -1: '#49324e', 0: '#204f3b',
                     1: '#764821', 2: '#245779', 3: '#535087'}

# 键、标签、默认值、最小值、最大值、小数位。全部单位放在标签中。
PARAMETERS = (
    ('input_alpha', '输入低通 alpha', .2, .000001, 1., 6),
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
    ('idle_alpha', 'idle 基线跟踪 alpha', .002, .000001, 1., 6),
    ('recover_alpha', 'recover 基线跟踪 alpha', .1, .000001, 1., 6),
    ('recover_tolerance', '恢复允许基线差值（count）', 20., 0., 65535000., 2),
    ('stable_rate', '恢复允许变化速度绝对值（%/s）', 2., 0., 100000., 3),
    ('recover_frames', '恢复稳定连续确认（帧）', 5, 1, 10000, 0),
    ('reload_rate', '重新加载上升速度（%/s）', 20., .001, 100000., 3),
    ('reload_threshold', '重新加载较谷值增加量（count）', 200., 0., 65535000., 2),
    ('reload_frames', '重新加载连续确认（帧）', 3, 1, 10000, 0),
    ('normalization_floor', '速度归一化最小 input（count）', 1., .000001, 65535000., 6),
    ('gap_seconds', '采样中断重新初始化间隔（秒）', .2, .01, 60., 3),
)


@dataclass(frozen=True)
class HostParameters:
    input_alpha: float = .2
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
    idle_alpha: float = .002
    recover_alpha: float = .1
    recover_tolerance: float = 20.
    stable_rate: float = 2.
    recover_frames: int = 5
    reload_rate: float = 20.
    reload_threshold: float = 200.
    reload_frames: int = 3
    normalization_floor: float = 1.
    gap_seconds: float = .2

    def __post_init__(self):
        for key, _, _, low, high, decimals in PARAMETERS:
            value = getattr(self, key)
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{key} 超出范围')
            if decimals == 0 and int(value) != value:
                raise ValueError(f'{key} 必须为整数')
            if decimals == 0:
                object.__setattr__(self, key, int(value))
        if self.release_threshold >= self.activate_threshold:
            raise ValueError('慢速释放差值必须小于激活差值')
        if self.recover_alpha < self.idle_alpha:
            raise ValueError('recover alpha 必须不小于 idle alpha')

    def settings(self):
        return {key: getattr(self, key) for key, *_ in PARAMETERS}

    @classmethod
    def from_settings(cls, settings):
        return cls(**{key: settings[key] for key, *_ in PARAMETERS if key in settings})


class _Channel:
    def __init__(self, window):
        self.state = -1
        self.input = self.baseline = self.signal = 0.
        self.rate = math.nan
        self.decision_rate = math.nan
        self.started = None
        self.history = deque(maxlen=window)
        self.activate_count = self.drop_count = self.release_count = 0
        self.reload_count = self.recover_count = self.hold_count = 0
        self.close_count = 0
        self.trough = 0.
        self.activation_number = 0
        self.last_activation = None

    def commit_state(self, state, activation=None):
        # 运行时状态仅由此入口提交，同时集中复位确认计数。
        if state == self.state:
            return
        if state == 1 and activation is not None:
            self.activation_number += 1
            self.last_activation = dict(activation, from_state=self.state,
                                        activation_number=self.activation_number)
        self.state = state
        self.activate_count = self.drop_count = self.release_count = 0
        self.reload_count = self.recover_count = self.hold_count = 0
        self.close_count = 0
        self.trough = self.input

    def activation_evidence(self, timestamp, rate, p):
        # 在提交状态、清空计数及谷值前保存本次真正通过的判据。
        reloading = self.state in (2, 3)
        evidence = dict(reason='reload' if reloading else 'idle_threshold',
                        sample_timestamp=timestamp, input_val=self.input, baseline=self.baseline,
                        difference=self.input - self.baseline,
                        activate_threshold=p.activate_threshold, pressure_rate=rate,
                        confirmed_frames=self.reload_count if reloading else self.activate_count,
                        required_frames=p.reload_frames if reloading else p.activate_frames)
        if reloading:
            evidence.update(trough=self.trough, increase=self.input - self.trough,
                            reload_threshold=p.reload_threshold, reload_rate=p.reload_rate)
        return evidence

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

    def feed(self, raw, timestamp, enabled, p):
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
        self.signal = 0.
        if self.state == -1:
            self.baseline = max(self.baseline, self.input)
            if timestamp - self.started >= p.initial_seconds:
                self.commit_state(0)
            return
        # idle 不计算速度；窗口仍保留，以便激活当帧立即具备有效历史。
        rate = self.slope_rate(p) if self.state in (1, 2, 3) else math.nan
        difference = self.input - self.baseline
        if self.state == 0:
            self.activate_count = self.activate_count + 1 if difference > p.activate_threshold else 0
            if self.activate_count >= p.activate_frames:
                rate = self.slope_rate(p)
                self.commit_state(1, self.activation_evidence(timestamp, rate, p))
            elif not self.activate_count:
                self.baseline += p.idle_alpha * difference
        elif self.state == 1:
            self.drop_count = self.drop_count + 1 if rate <= -p.drop_rate else 0
            self.release_count = self.release_count + 1 if difference < p.release_threshold else 0
            if self.drop_count >= p.drop_frames or self.release_count >= p.release_frames:
                self.commit_state(2)
        elif self.state in (2, 3):
            self.trough = min(self.trough, self.input)
            reloading = (rate >= p.reload_rate
                         and self.input - self.trough > p.reload_threshold
                         and difference > p.activate_threshold)
            self.reload_count = self.reload_count + 1 if reloading else 0
            if self.reload_count >= p.reload_frames:
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
            elif not rate >= p.reload_rate:
                # 新加载候选出现的第一帧即暂停跟踪，避免确认期间吞掉新压力。
                self.baseline += p.recover_alpha * difference
                stable = (abs(self.input - self.baseline) <= p.recover_tolerance
                          and abs(rate) <= p.stable_rate)
                self.recover_count = self.recover_count + 1 if stable else 0
                if self.recover_count >= p.recover_frames:
                    self.commit_state(0)
            else:
                self.recover_count = 0
        # 等待与恢复保留内部判定速度，公开速度仅在 activate 输出。
        self.decision_rate = rate
        self.rate = rate if self.state == 1 else math.nan
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
                          for key in ('raw', 'baseline', 'signal', 'fast', 'slow', 'state')}
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
            channel.feed(value, timestamp, gain > 0, self.parameters)
            if channel.activation_number != activation_number:
                activation_events.append(dict(channel=index, **channel.last_activation))
        self.last_timestamp = timestamp
        self.reason = '运行中' if all(c.state != -1 for c in self.channels) else f'初始化中，请保持释放 · {self.reset_reason}'
        frame = dict(sequence=sequence, sample_timestamp=timestamp,
                     generation=self.generation, reset_reason=self.reset_reason,
                     activation_events=activation_events,
                     input_val=[c.input for c in self.channels], baseline=[c.baseline for c in self.channels],
                     signal=[c.signal for c in self.channels], pressure_rate=[c.rate for c in self.channels],
                     decision_rate=[c.decision_rate for c in self.channels],
                     state=[c.state for c in self.channels])
        for key, source in (('raw', 'input_val'), ('baseline', 'baseline'), ('signal', 'signal'),
                            ('fast', 'pressure_rate'), ('state', 'state')):
            for history, value in zip(self.histories[key], frame[source]):
                history.append(value)
        for history in self.histories['slow']:
            history.append(math.nan)
        self.version += 1
        return frame

    def snapshot(self):
        return {key: [list(values) for values in channels] for key, channels in self.histories.items()}
