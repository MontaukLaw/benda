"""20路Raw的相对触摸板试验；只消费固件状态，不维护或改写通道状态。"""
import math
import statistics
from collections import deque

from centroid_filter import OneEuroAxis, signal_centroid
from channel_states import STATE_IDLE, STATE_RISING, STATE_HOLD, STATE_FALLING, STATE_FLOATING, STATE_NAMES


class TouchpadTracker:
    """松手采集逐通道参考；触摸期间固定参考，以连续质心差输出浮点位移。"""
    MAX_GAP = .15
    CALIBRATION_SECONDS = 1.0
    CONTACT_SECONDS = .03
    # 恢复权重占主导且Raw整体一致回落时，停止释放尾段的鼠标输出。
    MIN_ACTIVE_WEIGHT_SHARE = .5

    def __init__(self, sensitivity=300, threshold=100, min_cutoff=2.5):
        self.sensitivity = float(sensitivity)
        self.threshold = float(threshold)
        self.min_cutoff = float(min_cutoff)
        self.enabled = False
        self.reference = self.noise = None
        self.calibrating = False
        self._calibration = deque(maxlen=2048)
        self.timestamp = self.sequence = None
        self.message = '启用后，请松手采集参考'
        self._reference_status('idle', '尚未采集松手参考')
        self.end_contact()

    def _reference_status(self, stage, detail, progress=0, elapsed=0.0, frames=0):
        # 独立保存采集操作的进度和结果，不被下一帧的光标运行状态覆盖。
        self.reference_status = dict(stage=stage, detail=detail, progress=progress,
                                     elapsed=elapsed, frames=frames)

    def end_contact(self, clear_release=True):
        self.contact = False
        self._candidate_since = None
        self._history = deque(maxlen=3)
        self.axes = (OneEuroAxis(self.min_cutoff, .8), OneEuroAxis(self.min_cutoff, .8))
        self.position = self.raw_position = None
        self.delta = (0.0, 0.0)
        self.active_weight_share = None
        self.motion_block_reason = None
        if clear_release:
            self._release_samples = deque()
            self.release_blocked = False

    def invalidate(self, message='参考已失效，请松手重新采集'):
        was_calibrating = self.calibrating
        self.reference = self.noise = None
        self.calibrating = False
        self._calibration.clear()
        self.timestamp = self.sequence = None
        self.end_contact()
        self.message = message
        self._reference_status('cancelled' if was_calibrating else 'idle',
                               '采集已中止：' + message if was_calibrating else message)

    def start_reference(self):
        self.invalidate('请完全松手，等待连续1秒稳定的20路Debug数据')
        self.calibrating = True
        self._reference_status('waiting', '已开始采集，请完全松手；等待20路Debug数据')

    def cancel_reference(self):
        self.invalidate('已取消松手参考采集，可重新开始')
        self._reference_status('cancelled', self.message)

    def wait_for_reference(self, message):
        if self.calibrating:
            self._calibration.clear()
            self._reference_status('waiting', message)

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        self.invalidate('请松手采集参考' if enabled else '触摸板已关闭')

    def configure(self, sensitivity, threshold, min_cutoff):
        self.sensitivity = float(sensitivity)
        self.threshold = float(threshold)
        self.min_cutoff = float(min_cutoff)
        self.end_contact()

    @staticmethod
    def _valid(values):
        return values is not None and len(values) == 20 and all(math.isfinite(v) for v in values)

    def _collect_reference(self, raw, signals, states, timestamp):
        # 只接受固件确认空闲且Signal低的帧；有触摸、恢复或不稳定时重新计时。
        if any(state != STATE_IDLE for state in states) or max(signals) >= self.threshold * .5:
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
        self._reference_status('collecting', '正在采集稳定参考，请保持松手', progress, elapsed, frames)
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
        self._reference_status('complete', '20路参考已保存，可以开始滑动', 100, elapsed, frames)

    def feed(self, sample):
        self.delta = (0.0, 0.0)
        self.active_weight_share = None
        self.motion_block_reason = None
        if not self.enabled:
            return self.delta
        timestamp, sequence = sample.get('sample_timestamp'), sample.get('sequence')
        raw, signals, states = (sample.get(key) for key in ('raw', 'signal', 'state'))
        if (not sample.get('has_raw') or sample.get('channel_count') != 20
                or not all(self._valid(v) for v in (raw, signals, states))
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
        dt = timestamp - self.timestamp if self.timestamp is not None else 0
        if self.timestamp is not None and (dt <= 0 or dt > self.MAX_GAP or sequence != self.sequence + 1):
            self.end_contact()
            self._calibration.clear()
        self.timestamp, self.sequence = timestamp, sequence
        if self.calibrating:
            self._collect_reference(raw, signals, states, timestamp)
            return self.delta
        if self.reference is None:
            self.message = '请完全松手，再点击采集参考'
            return self.delta

        # 固件状态和Signal仅作接触门控；全部位置权重来自逐通道Raw差值。
        active = (STATE_RISING, STATE_HOLD, STATE_FALLING, STATE_FLOATING)
        threshold = self.threshold * (.6 if self.contact else 1.0)
        valid_touch = any(state in active and signal >= threshold
                          for state, signal in zip(states, signals))
        if not valid_touch:
            self.end_contact()
            self.message = '已松手或低于接触阈值，光标保持原位'
            return self.delta
        # 抬指门控直接检查本帧Raw，不等待中值窗或位置滤波追上释放过程。
        live_differences = [max(0.0, value - reference) for value, reference in zip(raw, self.reference)]
        live_weights = self._position_weights(live_differences)
        release_blocked, active_share = self._release_guard(live_differences, live_weights, states, active, timestamp)
        self.active_weight_share = active_share
        if release_blocked:
            # 这是鼠标输出门控，不改写固件状态；丢弃滤波尾量，恢复后重新落指锚定。
            self.end_contact(clear_release=False)
            self.active_weight_share = active_share
            self.motion_block_reason = 'coherent_release_tail'
            self.message = '检测到整体释放，停止位移并清除滤波尾量'
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
        # 落指、丢帧恢复或离群跳变只重建参考，不输出跨断点位移。
        if (self.raw_position is not None and
                math.dist(xy, self.raw_position) > max(.75, 40 * max(dt, .001))):
            self.end_contact()
            self.message = '位置突变，重新确认接触'
            self.motion_block_reason = 'position_jump'
            return self.delta
        self.raw_position = xy
        position = tuple(axis.update(value, max(dt, .001)) for axis, value in zip(self.axes, xy))
        if self.contact:
            self.delta = tuple((new - old) * self.sensitivity for new, old in zip(position, self.position))
        self.position = position
        self.contact = True
        self.message = '正在跟踪 · Raw逐通道差值'
        return self.delta

    def _position_weights(self, differences):
        return [value * (1 - math.exp(-(value / noise) ** 2))
                for value, noise in zip(differences, self.noise)]

    def _release_guard(self, differences, weights, states, active, timestamp):
        total = sum(weights)
        active_share = (sum(w for w, state in zip(weights, states) if state in active) / total
                        if total > 0 else 0.0)
        # 只比较最近80毫秒；正常滑动有通道升、有通道降，不能仅凭某路FALLING停光标。
        while self._release_samples and timestamp-self._release_samples[0][0] > .08:
            self._release_samples.popleft()
        if self._release_samples:
            then, previous = self._release_samples[0]
            gains = sum(max(0, new-old) for new,old in zip(differences, previous))
            losses = sum(max(0, old-new) for new,old in zip(differences, previous))
            noise_floor = 6 * sum(self.noise)
            if self.release_blocked:
                # 只有新的明显加载或空间转移才重新锚定，材料慢回弹不补发历史位移。
                fresh_motion = (active_share >= .2 and gains > losses*.8
                                and gains > max(noise_floor, self.threshold*.5, sum(previous)*.08))
                if fresh_motion:
                    self.release_blocked = False
            elif (timestamp-then >= .025 and active_share < self.MIN_ACTIVE_WEIGHT_SHARE
                  and losses > max(noise_floor, sum(previous)*.25) and gains < losses*.2):
                self.release_blocked = True
        self._release_samples.append((timestamp, tuple(differences)))
        return self.release_blocked, active_share

    def expire(self, now):
        if self.timestamp is not None and now - self.timestamp > self.MAX_GAP:
            self.end_contact()
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
