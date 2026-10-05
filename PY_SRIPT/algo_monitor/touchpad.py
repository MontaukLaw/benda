"""20路 Raw 参考或显式上位机 Signal 质心的相对触摸板；不改写通道状态。"""
import math
import statistics
from collections import deque

from centroid_filter import OneEuroAxis, signal_centroid, active_adjacent_pairs
from host_state_machine import HOST_STATE_ACTIVATE, HOST_STATE_NAMES
from raw_gain import normalize_raw_gains, apply_raw_gains
from channel_states import STATE_IDLE, STATE_RISING, STATE_HOLD, STATE_FALLING, STATE_FLOATING, STATE_NAMES


class TouchpadTracker:
    """从 Raw 参考或上位机 Signal 定位，以连续质心差输出浮点位移。"""
    MAX_GAP = .15
    CALIBRATION_SECONDS = 1.0
    CONTACT_SECONDS = .03
    # 恢复权重占主导且Raw整体一致回落时，停止释放尾段的鼠标输出。
    MIN_ACTIVE_WEIGHT_SHARE = .5
    RAW_RELEASE_DROP_DEFAULT = 25
    RAW_RELEASE_RISE_DEFAULT = 0
    HOST_SOURCES = ('host_signal', 'host_continuous')

    def __init__(self, sensitivity=300, threshold=100, min_cutoff=2.5,
                 raw_release_drop_percent=RAW_RELEASE_DROP_DEFAULT,
                 raw_release_rise_percent=RAW_RELEASE_RISE_DEFAULT,
                 position_reference_seconds=.5, position_noise_floor=20.,
                 position_decay_enabled=True, position_decay_ms=80., position_restore_ms=20., position_decay_percent=60.):
        self.sensitivity = float(sensitivity)
        self.threshold = float(threshold)
        self.min_cutoff = float(min_cutoff)
        self.raw_release_drop_percent = self._bounded_percent(raw_release_drop_percent, 5, 80)
        self.raw_release_rise_percent = self._bounded_percent(raw_release_rise_percent, 0, 20)
        self.position_reference_seconds = self._bounded_percent(position_reference_seconds, .2, 3.)
        self.position_noise_floor = self._bounded_percent(position_noise_floor, 2., 200.)
        self.position_decay_enabled = bool(position_decay_enabled)
        self.position_decay_ms = self._bounded_percent(position_decay_ms, 20., 500.)
        self.position_restore_ms = self._bounded_percent(position_restore_ms, 10., 200.)
        self.position_decay_percent = self._bounded_percent(position_decay_percent, 0., 100.)
        self.position_factors = self.position_unattenuated_weights = None
        self.position_reference = self.position_noise = None
        self.position_weights = None
        self._position_reference_samples = deque(maxlen=2048)
        self._position_reference_at = None
        self.raw_gains = (1.0,) * 20
        self.source = 'raw_reference'
        self.host_generation = None
        self.centroid_delta = (0.0, 0.0)
        self.centroid_held = False
        self.host_active_channels = self.host_adjacent_pairs = ()
        self.enabled = False
        self.reference = self.noise = None
        self.calibrating = False
        self.reference_forced = False
        self._calibration = deque(maxlen=2048)
        self.timestamp = self.sequence = None
        self._raw_only_mode = None
        self.message = '启用后，请松手采集参考'
        self._reference_status('idle', '尚未采集松手参考')
        self.end_contact()

    def _reference_status(self, stage, detail, progress=0, elapsed=0.0, frames=0):
        # 独立保存采集操作的进度和结果，不被下一帧的光标运行状态覆盖。
        self.reference_status = dict(stage=stage, detail=detail, progress=progress,
                                     elapsed=elapsed, frames=frames, forced=self.reference_forced)

    def end_contact(self, clear_release=True):
        self.contact = False
        self._candidate_since = None
        self._history = deque(maxlen=3)
        self.axes = (OneEuroAxis(self.min_cutoff, .8), OneEuroAxis(self.min_cutoff, .8))
        self.position = self.raw_position = None
        self.delta = (0.0, 0.0)
        self.centroid_delta = (0.0, 0.0)
        self.centroid_held = False
        self.active_weight_share = None
        self.motion_block_reason = None
        if clear_release:
            self._release_samples = deque()
            self.release_blocked = False

    def invalidate(self, message='参考已失效，请松手重新采集'):
        was_calibrating = self.calibrating
        self.reference = self.noise = None
        self.calibrating = False
        self.reference_forced = False
        self._calibration.clear()
        self.timestamp = self.sequence = None
        self._raw_only_mode = None
        self.host_generation = None
        self._clear_position_reference()
        self.end_contact()
        self.message = message
        self._reference_status('cancelled' if was_calibrating else 'idle',
                               '采集已中止：' + message if was_calibrating else message)

    def start_reference(self, force=False):
        self.invalidate('请完全松手，等待连续1秒稳定的20路Debug数据')
        self.reference_forced = bool(force)
        self.calibrating = True
        self._reference_status('waiting', '强制采集已开始，忽略通道状态和Signal释放门槛；请保持松手，等待稳定Raw'
                               if force else '已开始采集，请完全松手；等待20路Debug数据')

    def cancel_reference(self):
        self.invalidate('已取消松手参考采集，可重新开始')
        self._reference_status('cancelled', self.message)

    def wait_for_reference(self, message):
        if self.calibrating:
            self._calibration.clear()
            self._reference_status('waiting', message)

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        self.invalidate(('等待上位机输入；请保持释放完成参考初始化' if self.source in self.HOST_SOURCES
                         else '请松手采集参考') if enabled else '触摸板已关闭')

    def set_source(self, source):
        if source not in ('raw_reference', *self.HOST_SOURCES):
            raise ValueError('未知触摸板数据来源')
        if source == self.source:
            return False
        self.source = source
        self.invalidate('已切到上位机位置计算，请保持释放等待有效采样' if source in self.HOST_SOURCES
                        else '已切到 Raw 参考，请松手采集参考')
        return True

    @staticmethod
    def _bounded_percent(value, low, high):
        # 算法入口与输入框使用相同边界，不接受非有限阈值。
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('释放阈值必须为有限数值')
        return max(low, min(high, value))

    def configure(self, sensitivity, threshold, min_cutoff,
                  raw_release_drop_percent=None, raw_release_rise_percent=None,
                  position_reference_seconds=None, position_noise_floor=None,
                  position_decay_enabled=None, position_decay_ms=None, position_restore_ms=None, position_decay_percent=None):
        self.sensitivity = float(sensitivity)
        self.threshold = float(threshold)
        self.min_cutoff = float(min_cutoff)
        if raw_release_drop_percent is not None:
            self.raw_release_drop_percent = self._bounded_percent(raw_release_drop_percent, 5, 80)
        if raw_release_rise_percent is not None:
            self.raw_release_rise_percent = self._bounded_percent(raw_release_rise_percent, 0, 20)
        if position_reference_seconds is not None:
            self.position_reference_seconds = self._bounded_percent(position_reference_seconds, .2, 3.)
        if position_noise_floor is not None:
            self.position_noise_floor = self._bounded_percent(position_noise_floor, 2., 200.)
        if position_decay_enabled is not None:
            self.position_decay_enabled = bool(position_decay_enabled)
        if position_decay_ms is not None:
            self.position_decay_ms = self._bounded_percent(position_decay_ms, 20., 500.)
        if position_restore_ms is not None:
            self.position_restore_ms = self._bounded_percent(position_restore_ms, 10., 200.)
        if position_decay_percent is not None:
            self.position_decay_percent = self._bounded_percent(position_decay_percent, 0., 100.)
        if self.source == 'host_continuous':
            self._clear_position_reference()
        # 已确认卸载时调参仍保留停止锁存，不让原释放尾段重新成为接触。
        self.end_contact(clear_release=not self.release_blocked)

    def set_raw_gains(self, gains):
        # 增益变化会改变参考及噪声量纲，禁止混用旧参考或续接旧轨迹。
        gains = normalize_raw_gains(gains)
        if gains == self.raw_gains:
            return False
        self.raw_gains = gains
        self.invalidate('Raw增益已改变，等待上位机基线重新初始化' if self.source in self.HOST_SOURCES
                        else 'Raw增益已改变，请松手重新采集参考')
        return True

    @staticmethod
    def _valid(values):
        return values is not None and len(values) == 20 and all(math.isfinite(v) for v in values)

    def _collect_reference(self, raw, signals, states, timestamp):
        # 只接受固件确认空闲且Signal低的帧；有触摸、恢复或不稳定时重新计时。
        if not self.reference_forced and (any(state != STATE_IDLE for state in states) or max(signals) >= self.threshold * .5):
            self._calibration.clear()
            self.message = '等待完全释放，20路均为空闲后采集参考'
            busy = [f'CH{i}({STATE_NAMES.get(state, str(state))})'
                    for i, state in enumerate(states) if state != STATE_IDLE]
            reasons = []
            if busy:
                reasons.append(f'{len(busy)}路尚未空闲：' + '、'.join(busy[:6]) + ('等' if len(busy)>6 else ''))
            if max(signals) >= self.threshold * .5:
                reasons.append(f'Signal最大值 {max(signals):.1f}，需低于 {self.threshold*.5:g} count')
            self._reference_status('waiting', '请继续松手。' + '；'.join(reasons))
            return
        self._calibration.append((timestamp, tuple(raw)))
        while len(self._calibration) > 2 and self._calibration[1][0] <= timestamp - self.CALIBRATION_SECONDS:
            self._calibration.popleft()
        elapsed = timestamp - self._calibration[0][0]
        frames = len(self._calibration)
        progress = min(99, int(100 * min(elapsed/self.CALIBRATION_SECONDS, frames/32)))
        self._reference_status('collecting', '正在强制采集稳定Raw，不检查通道状态，请保持松手'
                               if self.reference_forced else '正在采集稳定参考，请保持松手', progress, elapsed, frames)
        self.message = f'松手参考采集中：{min(100, int(elapsed * 100))}%'
        if elapsed < self.CALIBRATION_SECONDS or len(self._calibration) < 32:
            return
        columns = list(zip(*(row for _, row in self._calibration)))
        # 检查整段变化幅度，避免把仍在回弹的材料当成稳定背景。
        if any(max(col) - min(col) > max(20, self.threshold * .4) for col in columns):
            self.message = 'Raw仍在变化，请继续松手等待稳定'
            self._reference_status('unstable', self.message, progress, elapsed, frames)
            return
        self.reference = tuple(statistics.mean(col) for col in columns)
        self.noise = tuple(max(2.0, 3 * statistics.pstdev(col)) for col in columns)
        self.calibrating = False
        self._calibration.clear()
        self.message = '参考就绪，滑动可移动光标'
        self._reference_status('complete', '20路Raw参考已强制保存，采集未检查通道状态'
                               if self.reference_forced else '20路参考已保存，可以开始滑动', 100, elapsed, frames)

    def feed(self, sample):
        self.delta = (0.0, 0.0)
        self.centroid_delta = (0.0, 0.0)
        self.active_weight_share = None
        self.motion_block_reason = None
        self.host_active_channels = self.host_adjacent_pairs = ()
        if not self.enabled:
            return self.delta
        if self.source in self.HOST_SOURCES:
            return self._feed_host_signal(sample)
        timestamp, sequence = sample.get('sample_timestamp'), sample.get('sequence')
        raw, signals, states = (sample.get(key) for key in ('raw', 'signal', 'state'))
        raw_only = bool(sample.get('raw_only', False))
        # Raw观察帧没有Signal和状态，直接消费原始差值；完整Debug仍校验原诊断字段。
        raw_required = (self.calibrating and self.reference_forced) or (raw_only and not self.calibrating)
        required = (raw,) if raw_required else (raw, signals, states)
        if (not sample.get('has_raw') or sample.get('channel_count') != 20
                or not all(self._valid(v) for v in required)
                or timestamp is None or not math.isfinite(timestamp)):
            self.end_contact()
            self._calibration.clear()
            self.message = '需要20路Debug数据，当前不输出位移'
            self.motion_block_reason = 'invalid_sample'
            if self.calibrating:
                self._reference_status('waiting', '采集暂停：需要有效的20路Debug数据，请检查连接与输出模式')
            return self.delta
        if self.sequence is not None and sequence <= self.sequence:
            return self.delta
        # 采集参考、测量噪声与运行定位均消费增益后的Raw，不修改来源样本。
        raw = apply_raw_gains(raw, self.raw_gains)
        dt = timestamp - self.timestamp if self.timestamp is not None else 0
        if self.timestamp is not None and (dt <= 0 or dt > self.MAX_GAP or sequence != self.sequence + 1):
            self.end_contact()
            self._calibration.clear()
        self.timestamp, self.sequence = timestamp, sequence
        # 数据来源模式变化时断开轨迹并重新锚定，避免把两种接触判定跨帧拼接。
        mode_changed = self._raw_only_mode is not None and self._raw_only_mode != raw_only
        self._raw_only_mode = raw_only
        if mode_changed:
            self.end_contact()
            self._calibration.clear()
            self.motion_block_reason = 'source_mode_changed'
            self.message = '数据模式已改变，重新确认接触'
            return self.delta
        if self.calibrating:
            self._collect_reference(raw, signals, states, timestamp)
            return self.delta
        if self.reference is None:
            self.message = '请完全松手，再点击采集参考'
            return self.delta

        # 接触只控制触摸板位移，不写入或伪造任何通道状态。
        active = (STATE_RISING, STATE_HOLD, STATE_FALLING, STATE_FLOATING)
        threshold = self.threshold * (.6 if self.contact else 1.0)
        live_differences = [max(0.0, value - reference) for value, reference in zip(raw, self.reference)]
        valid_touch = (max(live_differences) >= threshold if raw_only else
                       any(state in active and signal >= threshold for state, signal in zip(states, signals)))
        if not valid_touch and not raw_only:
            self.end_contact()
            self.message = '已松手或低于接触阈值，光标保持原位'
            return self.delta
        # 抬指门控直接检查本帧Raw，不等待中值窗或位置滤波追上释放过程。
        live_weights = self._position_weights(live_differences)
        # Raw模式只检查整体卸载，不依赖缺失的Signal或通道状态。
        release_blocked, active_share = self._release_guard(
            live_differences, live_weights, states, active, timestamp, raw_only=raw_only)
        self.active_weight_share = active_share
        if release_blocked:
            # 这是鼠标输出门控，不改写固件状态；丢弃滤波尾量，恢复后重新落指锚定。
            self.end_contact(clear_release=False)
            self.active_weight_share = active_share
            self.motion_block_reason = 'coherent_release_tail'
            self.message = '检测到整体释放，停止位移并清除滤波尾量'
            return self.delta
        if not valid_touch:
            # 低于接触阈值时保留Raw卸载历史，回弹尾段不能重置释放锁存。
            self.end_contact(clear_release=False)
            self.message = f'Raw差值最大 {max(live_differences):.1f} count，低于接触阈值 {threshold:g}，光标保持原位'
            return self.delta
        self._history.append(tuple(raw))
        filtered_raw = [statistics.median(col) for col in zip(*self._history)]
        differences = [max(0.0, value - reference)
                       for value, reference in zip(filtered_raw, self.reference)]
        # 平滑压制接近噪声的权重，较强响应近似线性；不以全阵列均值代替背景。
        weights = self._position_weights(differences)
        xy = signal_centroid(weights, local_2x2_enabled=False)
        if xy is None or max(differences) < threshold:
            self.end_contact()
            self.message = '有效Raw差值不足，光标保持原位'
            return self.delta
        if self._candidate_since is None:
            self._candidate_since = timestamp
        if timestamp - self._candidate_since < self.CONTACT_SECONDS:
            self.message = '确认接触中'
            return self.delta
        return self._track_centroid(xy, timestamp, dt,
                                   '正在跟踪 · Raw观察模式（Raw参考差值判定接触）' if raw_only
                                   else '正在跟踪 · Raw逐通道差值')

    def _feed_host_signal(self, sample):
        # 独立派生字段与固件字段区分；直接使用已增益、低通、扣基线后的 Signal。
        timestamp, sequence = sample.get('sample_timestamp'), sample.get('sequence')
        signals = sample.get('host_signal')
        states = sample.get('host_state')
        generation = sample.get('host_generation')
        continuous = self.source == 'host_continuous'
        inputs = sample.get('host_input_val')
        if (not sample.get('has_raw') or sample.get('channel_count') != 20
                or not sample.get('host_ready') or not self._valid(signals)
                or not self._valid(states) or any(state not in HOST_STATE_NAMES for state in states)
                or any(v < 0 for v in signals) or generation is None
                or continuous and (not self._valid(inputs) or any(v < 0 for v in inputs))
                or sequence is None or timestamp is None or not math.isfinite(timestamp)):
            self.end_contact()
            if continuous:
                self._clear_position_reference()
            self.motion_block_reason = 'invalid_sample'
            self.message = sample.get('host_message', '等待有效的上位机 Signal；请保持释放完成初始化')
            return self.delta
        if self.sequence is not None and sequence <= self.sequence:
            return self.delta
        dt = timestamp - self.timestamp if self.timestamp is not None else 0.
        generation_changed = self.host_generation is not None and generation != self.host_generation
        if generation_changed or (self.timestamp is not None and
                (dt <= 0 or dt > self.MAX_GAP or sequence != self.sequence + 1)):
            self.end_contact()
            self._clear_position_reference()
            self.motion_block_reason = 'host_generation_changed' if generation_changed else 'source_mode_changed'
        self.host_generation = generation
        self.timestamp, self.sequence = timestamp, sequence
        self.host_active_channels = tuple(ch for ch, state in enumerate(states) if state == HOST_STATE_ACTIVATE)
        self.host_adjacent_pairs = active_adjacent_pairs(states, HOST_STATE_ACTIVATE)
        if continuous:
            return self._feed_continuous_position(inputs, states, timestamp, dt)
        threshold = self.threshold * (.6 if self.contact else 1.)
        # 直接检查当前 Signal 的整体卸载，不能等位置滤波追上已经消失的信号。
        blocked, _ = self._release_guard(signals, signals, (), (), timestamp, raw_only=True)
        if not self.host_adjacent_pairs:
            # 少于两路激活或无上下左右相邻对时，不计算质心；显示位置冻结，运动历史清空。
            self._hold_centroid()
            self.motion_block_reason = 'no_adjacent_activate_pair'
            self.message = f'activate {len(self.host_active_channels)}路，无物理相邻激活对；保持质心，位移为0'
            return self.delta
        if blocked:
            self._hold_centroid()
            self.motion_block_reason = 'coherent_release_tail'
            self.message = '上位机 Signal整体卸载，停止位移并清除滤波尾量'
            return self.delta
        if max(signals) < threshold or sum(signals) <= 0:
            self._hold_centroid()
            self.message = '上位机 Signal低于接触阈值，光标保持原位'
            return self.delta
        xy = signal_centroid(signals, local_2x2_enabled=False)
        if self._candidate_since is None:
            self._candidate_since = timestamp
        if timestamp - self._candidate_since < self.CONTACT_SECONDS:
            self.message = '上位机 Signal确认接触中'
            return self.delta
        return self._track_centroid(xy, timestamp, dt, '正在跟踪 · 上位机 Signal全部20路质心')

    def _clear_position_reference(self):
        self.position_reference = self.position_noise = self.position_weights = None
        self.position_factors = self.position_unattenuated_weights = None
        self._position_reference_samples.clear()
        self._position_reference_at = None

    def _update_position_reference(self, inputs, states, timestamp):
        # 仅消费现有状态。全阵列空闲且输入稳定时更新位置参考，任何承压或恢复均冻结。
        if any(state not in (0, -2) for state in states):
            self._position_reference_samples.clear()
            return False
        self._position_reference_samples.append((timestamp, tuple(inputs)))
        window = self._position_reference_samples
        while len(window) > 2 and window[1][0] <= timestamp - self.position_reference_seconds:
            window.popleft()
        if timestamp-window[0][0] < self.position_reference_seconds or len(window) < 4:
            return False
        if self._position_reference_at is not None and timestamp-self._position_reference_at < .1:
            return False
        columns = list(zip(*(values for _, values in window)))
        if any(max(c)-min(c) > max(40., self.threshold*.4) for c in columns):
            return False
        self.position_reference = tuple(statistics.mean(c) for c in columns)
        self.position_noise = tuple(max(2., 3*statistics.pstdev(c)) for c in columns)
        self._position_reference_at = timestamp
        # 确认松手后开启新参考周期，不把旧卸载历史与新参考混合比较。
        self._release_samples.clear()
        self.release_blocked = False
        return True

    def _feed_continuous_position(self, inputs, states, timestamp, dt):
        self._update_position_reference(inputs, states, timestamp)
        if self.position_reference is None:
            self._hold_centroid()
            self.motion_block_reason = 'position_reference_waiting'
            self.message = f'请完全松手：等待全阵列idle及连续{self.position_reference_seconds:g}秒稳定输入，建立位置参考'
            return self.delta
        # input_val已乘增益并低通。位置参考与通道baseline分开，通道切换不把位置权重归零。
        differences = [max(0., value-base) if state != -2 else 0.
                       for value, base, state in zip(inputs, self.position_reference, states)]
        self.position_unattenuated_weights = tuple(value * (1-math.exp(-(value/max(noise,self.position_noise_floor))**2))
                                                   for value, noise in zip(differences,self.position_noise))
        self.position_weights = self._attenuate_position_weights(self.position_unattenuated_weights, states, dt)
        # 整体释放仍检测真实连续输入，不能把人为权重衰减误当作卸载。
        blocked, _ = self._release_guard(differences, self.position_unattenuated_weights, (), (), timestamp,
                                         raw_only=True, noise_values=self.position_noise)
        if not self.host_adjacent_pairs:
            self._hold_centroid()
            self.motion_block_reason = 'no_adjacent_activate_pair'
            self.message = '无物理相邻activate对；保持质心及位置参考，位移为0'
            return self.delta
        if blocked:
            self._hold_centroid()
            self.motion_block_reason = 'coherent_release_tail'
            self.message = '连续压力整体卸载，停止位移并清除滤波尾量'
            return self.delta
        threshold = self.threshold * (.6 if self.contact else 1.)
        if max(differences) < threshold or sum(self.position_weights) <= 0:
            self._hold_centroid()
            self.message = '连续位置权重低于接触阈值，保持位置'
            return self.delta
        xy = signal_centroid(self.position_weights, local_2x2_enabled=False)
        if self._candidate_since is None:
            self._candidate_since = timestamp
        if timestamp-self._candidate_since < self.CONTACT_SECONDS:
            self.message = '连续位置权重确认接触中'
            return self.delta
        return self._track_centroid(xy,timestamp,dt,'正在跟踪 · 连续位置权重 / 8毫米等间距阵列')

    def _attenuate_position_weights(self, weights, states, dt):
        # 只滤波位置可信度，不改写通道状态。非activate趋向保留比例，activate平滑恢复到1。
        if not self.position_decay_enabled or self.position_decay_percent <= 0:
            self.position_factors = (1.,)*20
            return weights
        retained = 1. - self.position_decay_percent / 100
        targets = tuple(1. if state == HOST_STATE_ACTIVATE else retained for state in states)
        if self.position_factors is None:
            self.position_factors = targets
        else:
            self.position_factors = tuple(
                target + (old-target)*math.exp(-max(0.,dt)*1000 / (
                    self.position_restore_ms if target > old else self.position_decay_ms))
                for old,target in zip(self.position_factors,targets))
        return tuple(weight*factor for weight,factor in zip(weights,self.position_factors))

    def _hold_centroid(self):
        raw_position, position = self.raw_position, self.position
        self.end_contact(clear_release=False)
        self.raw_position, self.position = raw_position, position
        self.centroid_held = position is not None

    def _track_centroid(self, xy, timestamp, dt, message):
        # 落指、丢帧恢复或离群跳变只重建锚点，不输出跨断点位移。
        if (self.contact and self.raw_position is not None and
                math.dist(xy, self.raw_position) > max(.75, 40 * max(dt, .001))):
            self.end_contact()
            self.message = '位置突变，重新确认接触'
            self.motion_block_reason = 'position_jump'
            return self.delta
        self.raw_position = xy
        position = tuple(axis.update(value, max(dt, .001)) for axis, value in zip(self.axes, xy))
        if self.contact:
            self.centroid_delta = tuple(new - old for new, old in zip(position, self.position))
            self.delta = tuple(value * self.sensitivity for value in self.centroid_delta)
        self.position = position
        self.centroid_held = False
        self.contact = True
        self.message = message
        return self.delta

    def _position_weights(self, differences):
        return [value * (1 - math.exp(-(value / noise) ** 2))
                for value, noise in zip(differences, self.noise)]

    def _release_guard(self, differences, weights, states, active, timestamp, raw_only=False, noise_values=None):
        # Signal 已由上位机低通，未采集 Raw 噪声时沿用最小 2 count 门限。
        noise = noise_values if noise_values is not None else self.noise if self.noise is not None else (2.,) * 20
        total = sum(weights)
        active_share = None if raw_only else (
            sum(w for w, state in zip(weights, states) if state in active) / total if total > 0 else 0.0)
        # 只比较最近80毫秒；正常滑动有通道升、有通道降，不能仅凭某路FALLING停光标。
        while self._release_samples and timestamp-self._release_samples[0][0] > .08:
            self._release_samples.popleft()
        if self._release_samples:
            then, previous = self._release_samples[0]
            gains = sum(max(0, new-old) for new,old in zip(differences, previous))
            losses = sum(max(0, old-new) for new,old in zip(differences, previous))
            noise_floor = 6 * sum(noise)
            # 增益不一致时空间转移可能总量下降；有通道明显上升则不视为整体卸载。
            # 可允许单路小幅回升，但总回升仍必须低于下降量的20%。
            rise_tolerance = losses * self.raw_release_rise_percent / 100
            coherent_raw = not any(new-old > max(6*noise, rise_tolerance)
                                   for new,old,noise in zip(differences, previous, noise))
            drop_ratio = self.raw_release_drop_percent / 100 if raw_only else .25
            if self.release_blocked:
                # 只有新的明显加载或空间转移才重新锚定，材料慢回弹不补发历史位移。
                fresh_motion = ((raw_only or active_share >= .2) and gains > losses*.8
                                and gains > max(noise_floor, self.threshold*.5, sum(previous)*.08))
                if fresh_motion:
                    self.release_blocked = False
            elif (timestamp-then >= .025 and (coherent_raw if raw_only else active_share < self.MIN_ACTIVE_WEIGHT_SHARE)
                  and losses > max(noise_floor, sum(previous)*drop_ratio) and gains < losses*.2):
                self.release_blocked = True
        self._release_samples.append((timestamp, tuple(differences)))
        return self.release_blocked, active_share

    def expire(self, now):
        if self.timestamp is not None and now - self.timestamp > self.MAX_GAP:
            self.end_contact()
            if self.source == 'host_continuous':
                self._clear_position_reference()
            self._calibration.clear()
            self.message = '数据中断，光标保持原位'
            if self.calibrating and self.reference_status['stage'] in ('collecting', 'unstable'):
                self._reference_status('waiting', '采样中断，进度已重置；数据恢复后自动重新采集')


class FractionalMotion:
    """仅在发送像素位移时取整，小数余量保留到后续帧。"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.remainder = [0.0, 0.0]

    def consume(self, dx, dy):
        self.remainder[0] += dx
        self.remainder[1] += dy
        whole = tuple(math.trunc(value) for value in self.remainder)
        self.remainder = [value - pixel for value, pixel in zip(self.remainder, whole)]
        return whole
