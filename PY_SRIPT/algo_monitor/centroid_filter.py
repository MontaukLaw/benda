"""仅处理显示坐标的One Euro滤波；不改写Signal或任何通道状态。

参考算法与参数含义：https://gery.casiez.net/1euro/
坐标单位为格，时间单位为秒，X/Y分别维护滤波历史。
"""
import math
from channel_states import STATE_FALLING, STATE_RISING, STATE_HOLD


MIN_CUTOFF_HZ = 1.5
SPEED_BETA = 0.25
DERIVATIVE_CUTOFF_HZ = 1.0
RESET_GAP_SECONDS = 0.5
# 以本次连续有效触摸峰值为参照，2%配合释放证据隐藏；4%提供回显滞回。
RELEASE_PEAK_RATIO = 0.02
REAPPEAR_PEAK_RATIO = 0.04
DEFAULT_SIGNAL_THRESHOLD = 100
# 有效信号短暂消失时只保持原位置，绝不把残留信号送入位置滤波。
DISAPPEAR_HOLD_SECONDS = 0.1

# 通道按列编号；候选区域必须是空间相邻的两行两列。
CHANNEL_GRID = tuple(tuple(row + 4 * col for col in range(5)) for row in range(4))
CENTROID_BLOCKS = tuple((ch, ch + 1, ch + 4, ch + 5)
                        for col in range(4) for row in range(3) for ch in (4 * col + row,))


def _local_centroid(weights, previous_channels=()):
    """比较12个区域的非负信号总和，同分时沿用上一选区。输入须已校验。"""
    best_channels, best_total = (), 0.0
    for channels in CENTROID_BLOCKS:
        a, b, c, d = channels
        total = weights[a] + weights[b] + weights[c] + weights[d]
        if total > best_total or (total > 0 and total == best_total and channels == previous_channels):
            best_channels, best_total = channels, total
    if not best_channels:
        return None, (), 0.0
    a, b, c, d = best_channels
    # 单位间距下用右列、下行的权重占比直接求坐标，省去逐通道坐标乘法。
    xy = (a // 4 + 0.5 + (weights[c] + weights[d]) / best_total,
          a % 4 + 0.5 + (weights[b] + weights[d]) / best_total)
    return xy, best_channels, best_total


def _all_channel_centroid(weights):
    """恢复原20通道加权质心，零权重通道不影响坐标。"""
    total = sum(weights)
    if total <= 0:
        return None, (), 0.0
    xy = (sum((ch // 4 + .5) * weight for ch, weight in enumerate(weights)) / total,
          sum((ch % 4 + .5) * weight for ch, weight in enumerate(weights)) / total)
    return xy, tuple(range(20)), total


def raw_mean_difference_weights(raw_values):
    """全阵列减均值后只保留正差，按总和归一化；输入须为20路有效Raw。"""
    mean = math.fsum(raw_values) / len(raw_values)
    differences = [max(0.0, value - mean) for value in raw_values]
    total = math.fsum(differences)
    return ([value / total for value in differences] if total > 0 else [0.0] * 20), mean


def signal_centroid(signals, local_2x2_enabled=True):
    """以左上角为原点，按所选模式计算局部或全部20通道质心。"""
    if signals is None or len(signals) != 20 or not all(math.isfinite(v) for v in signals):
        return None
    weights = [max(0.0, value) for value in signals]
    return (_local_centroid(weights) if local_2x2_enabled else _all_channel_centroid(weights))[0]


class OneEuroAxis:
    def __init__(self, min_cutoff=MIN_CUTOFF_HZ, beta=SPEED_BETA,
                 derivative_cutoff=DERIVATIVE_CUTOFF_HZ):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.derivative_cutoff = derivative_cutoff
        self.reset()

    def reset(self):
        self.previous_raw = self.filtered = None
        self.velocity = 0.0

    @staticmethod
    def alpha(cutoff, dt):
        return 1.0 / (1.0 + 1.0 / (2.0 * math.pi * cutoff * dt))

    def update(self, value, dt):
        if self.filtered is None:
            self.previous_raw = self.filtered = value
            return value
        derivative = (value - self.previous_raw) / dt
        a = self.alpha(self.derivative_cutoff, dt)
        self.velocity += a * (derivative - self.velocity)
        cutoff = self.min_cutoff + self.beta * abs(self.velocity)
        a = self.alpha(cutoff, dt)
        self.filtered += a * (value - self.filtered)
        self.previous_raw = value
        return self.filtered


class CentroidFilter:
    def __init__(self, signal_threshold=DEFAULT_SIGNAL_THRESHOLD, local_2x2_enabled=True,
                 raw_enabled=False, raw_difference_enabled=False):
        self.signal_threshold = max(0, min(65535, int(signal_threshold)))
        self.local_2x2_enabled = bool(local_2x2_enabled)
        self.raw_difference_enabled = bool(raw_difference_enabled)
        self.raw_enabled = bool(raw_enabled) and not self.raw_difference_enabled
        self.axes = (OneEuroAxis(), OneEuroAxis())
        self.reset()

    def set_signal_threshold(self, value):
        value = max(0, min(65535, int(value)))
        if value != self.signal_threshold:
            self.signal_threshold = value
            self.reset()

    def set_local_2x2_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled != self.local_2x2_enabled:
            self.local_2x2_enabled = enabled
            self._reset_coordinate_history()

    def set_raw_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled != self.raw_enabled:
            self.raw_enabled = enabled
            if enabled:
                self.raw_difference_enabled = False
            self._reset_coordinate_history()

    def set_raw_difference_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled != self.raw_difference_enabled:
            self.raw_difference_enabled = enabled
            if enabled:
                self.raw_enabled = False
            self._reset_coordinate_history()

    def _reset_coordinate_history(self):
        # 重算同一采样并清除旧坐标平滑历史，同时保留Signal释放判据，避免尾段重新显点。
        self._reset_position()
        self.timestamp = None
        self.raw_xy = None
        self.selected_channels = ()
        self.selected_sum_signal = self.selected_sum_weight = 0.0
        self.raw_mean = None
        self.normalized_weights = ()
        self.source_missing = False
        self.holding_position = False

    def reset(self):
        self._reset_position()
        self.timestamp = None
        self.raw_xy = None
        self.selected_channels = ()
        self.selected_sum_signal = 0.0
        self.selected_sum_weight = 0.0
        self.raw_mean = None
        self.normalized_weights = ()
        self.source_missing = False
        self.peak_signal = self.previous_total = self.hidden_min_total = 0.0
        self.release_hidden = False
        self.threshold_hidden = False
        self.max_signal = 0.0
        self.loss_since = None
        self.holding_position = False

    def _reset_position(self):
        for axis in self.axes:
            axis.reset()
        self.filtered_xy = None

    def hold_due(self, now):
        return (self.holding_position and self.loss_since is not None
                and now - self.loss_since >= DISAPPEAR_HOLD_SECONDS)

    def expire_hold(self, now):
        # 即使没有下一帧，也允许界面时钟让冻结点准时消失。
        if self.hold_due(now):
            self._reset_position()
            self.holding_position = False

    def _hold_or_hide(self, timestamp):
        if self.loss_since is None:
            self.loss_since = timestamp
            for axis in self.axes:
                axis.velocity = 0.0
        self.holding_position = self.filtered_xy is not None
        self.expire_hold(timestamp)
        return self.filtered_xy

    def update(self, signals, timestamp, states=None, raw_values=None):
        if (signals is None or len(signals) != 20 or not all(math.isfinite(v) for v in signals)
                or not math.isfinite(timestamp)):
            self.reset()
            return None
        if (self.raw_enabled or self.raw_difference_enabled) and (
                raw_values is None or len(raw_values) != 20
                or not all(math.isfinite(v) for v in raw_values)):
            # Run帧不含Raw，必须隐藏并提示，不能沿用旧Raw或偷偷改回Signal。
            self.reset()
            self.source_missing = True
            return None
        if self.timestamp is not None and timestamp <= self.timestamp:
            return self.filtered_xy
        dt = timestamp - self.timestamp if self.timestamp is not None else 0.0
        if dt > RESET_GAP_SECONDS:
            self.reset()
        self.source_missing = False
        signal_weights = [max(0.0, value) for value in signals]
        # Raw模式直接使用原始硬件计数；选区与加权使用同一来源，不扣Baseline。
        weights = [max(0.0, value) for value in raw_values] if self.raw_enabled else signal_weights
        self.raw_mean = None
        self.normalized_weights = ()
        if self.raw_difference_enabled:
            # 先在全部20路上统一去底值，再用同一组归一化权重选区和计算坐标。
            weights, self.raw_mean = raw_mean_difference_weights(raw_values)
            self.normalized_weights = tuple(weights)
        self.raw_xy, self.selected_channels, self.selected_sum_weight = (
            _local_centroid(weights, self.selected_channels) if self.local_2x2_enabled
            else _all_channel_centroid(weights))
        self.selected_sum_signal = sum(signal_weights[ch] for ch in self.selected_channels)
        raw = self.raw_xy
        self.max_signal = max(signals)
        self.timestamp = timestamp
        # 只检查最大单通道Signal，不能把多个低于阈值的通道相加后视为达标。
        if raw is None or self.max_signal <= 0 or self.max_signal < self.signal_threshold:
            self.threshold_hidden = raw is not None
            self.release_hidden = False
            self.peak_signal = self.previous_total = self.hidden_min_total = 0.0
            return self._hold_or_hide(timestamp)
        self.threshold_hidden = False
        # 可见性和释放峰值始终使用Signal，Raw的静态底值不作为触摸证据。
        total = sum(signal_weights)
        has_states = states is not None and len(states) == 20 and all(s >= 0 for s in states)
        positive_states = [state for signal, state in zip(signals, states) if signal > 0] if has_states else []
        # 只消费固件真实状态，以下历史仅决定质心是否可显示，不改变通道状态或Signal。
        releasing = (all(state == STATE_FALLING for state in positive_states) if has_states
                     else total < self.previous_total)
        if self.release_hidden:
            new_press = (any(state in (STATE_RISING, STATE_HOLD) for state in positive_states)
                         if has_states else total >= max(self.hidden_min_total * 1.5, self.hidden_min_total + 1))
            if new_press or total >= self.peak_signal * REAPPEAR_PEAK_RATIO:
                self.release_hidden = False
                self.peak_signal = total
            else:
                self.hidden_min_total = min(self.hidden_min_total, total)
        self.peak_signal = max(self.peak_signal, total)
        if not self.release_hidden and releasing and total <= self.peak_signal * RELEASE_PEAK_RATIO:
            self.release_hidden = True
            self.hidden_min_total = total
        self.previous_total = total
        if self.release_hidden:
            return self._hold_or_hide(timestamp)
        if self.loss_since is not None:
            if timestamp - self.loss_since >= DISAPPEAR_HOLD_SECONDS:
                self._reset_position()
            else:
                # 恢复时沿用冻结位置，不把缺口内的坐标变化误当成瞬时高速运动。
                for axis, value in zip(self.axes, raw):
                    axis.previous_raw = value
                    axis.velocity = 0.0
            self.loss_since = None
        self.holding_position = False
        self.filtered_xy = tuple(axis.update(value, dt) for axis, value in zip(self.axes, raw))
        return self.filtered_xy

    def visibility(self):
        return dict(release_hidden=self.release_hidden, touch_peak_signal=self.peak_signal,
                    release_threshold=self.peak_signal * RELEASE_PEAK_RATIO,
                    threshold_hidden=self.threshold_hidden, signal_threshold=self.signal_threshold,
                    max_signal=self.max_signal, holding_position=self.holding_position,
                    hold_started_at=self.loss_since,
                    selected_channels=self.selected_channels, selected_sum_signal=self.selected_sum_signal,
                    selected_sum_weight=self.selected_sum_weight, source_missing=self.source_missing,
                    raw_mean=self.raw_mean, normalized_weights=self.normalized_weights,
                    signal_valid=self.raw_xy is not None and not self.threshold_hidden and not self.release_hidden)

    def parameters(self):
        return dict(algorithm='one_euro', min_cutoff_hz=MIN_CUTOFF_HZ,
                    beta=SPEED_BETA, derivative_cutoff_hz=DERIVATIVE_CUTOFF_HZ,
                    reset_gap_seconds=RESET_GAP_SECONDS, coordinate_unit='cell',
                    centroid_algorithm='max_sum_2x2' if self.local_2x2_enabled else 'all_20',
                    centroid_source='raw' if self.raw_enabled or self.raw_difference_enabled else 'signal',
                    centroid_weighting='positive_mean_difference_normalized' if self.raw_difference_enabled else 'identity',
                    visibility_source='signal',
                    centroid_tie_break='previous_then_grid_order' if self.local_2x2_enabled else None,
                    release_peak_ratio=RELEASE_PEAK_RATIO, reappear_peak_ratio=REAPPEAR_PEAK_RATIO,
                    disappear_hold_seconds=DISAPPEAR_HOLD_SECONDS)
