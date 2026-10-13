"""MAX → 物理 ROI → 相对阈值 → 非线性质心；无 Qt/串口依赖。"""

from dataclasses import dataclass
import math
from .config import AppConfig, SENSOR_POSITIONS
from .sensor_processor import SensorProcessor, validate_raw
from .coordinate_guard import CoordinateGuard


@dataclass(frozen=True)
class TouchResult:
    timestamp: float
    raw: tuple
    baseline: tuple
    signal: tuple
    baseline_ready: bool
    touched: bool = False
    state: str = "IDLE"
    max_index: int = 0
    max_signal: float = 0.0
    x_raw: float | None = None
    y_raw: float | None = None
    x: float | None = None
    y: float | None = None
    dx: float = 0.0
    dy: float = 0.0
    roi_indices: tuple = ()
    active_indices: tuple = ()
    total_signal: float = 0.0
    position_valid: bool = False
    below_baseline_indices: tuple = ()
    roi_target_indices: tuple = ()
    roi_weights: tuple = ()
    roi_transitioning: bool = False
    adjusted_raw: tuple = ()
    raw_gains: tuple = ()
    position_held: bool = False
    position_confidence: str = "inactive"


class TouchTracker:
    """update(raw_values, 单调秒时间戳) 返回 TouchResult。

    TOUCH_* 为整个触摸板的事件，不是固件逐通道状态。
    hooks 可以替换为后续 UI 回调，默认不输出逐帧文本。
    """

    def __init__(self, config=None, positions=None):
        self.config = (config or AppConfig()).copy().validate()
        self.positions = dict(SENSOR_POSITIONS if positions is None else positions)
        if set(self.positions) != set(range(20)) or set(self.positions.values()) != {
            (x, y) for x in range(5) for y in range(4)
        }:
            raise ValueError("坐标映射必须完整且唯一覆盖4行×5列的20通道")
        self.processor = SensorProcessor()
        self._last_timestamp = None
        self.reset_tracker()

    def set_config(self, config):
        updated = config.copy().validate()
        gains_changed = updated.raw_gains != self.config.raw_gains
        guard_changed = any(getattr(updated, key) != getattr(self.config, key) for key in
                            ("decay_guard_enable", "decay_guard_window_ms", "decay_guard_drop_ratio"))
        self.config = updated
        if gains_changed:
            self.reset_baseline()
            # 新增益属于新数值尺度，旧基线不能用于过渡显示或计算。
            self.processor.baseline = None
        elif guard_changed:
            self._coordinate_guard.reset()

    def reset_tracker(self):
        self._touched = False
        self._position = None
        self._raw_position = None
        self._previous_signal = None
        self._derivative_timestamp = None
        self._roi_weights = None
        self._roi_timestamp = None
        self._coordinate_guard = CoordinateGuard(self.positions)

    def _blend_roi(self, target, timestamp, touched, was_touched):
        """按真实经过时间限制ROI成员权重变化，最后一次切换后在设定时间内完成。"""
        goal = tuple(1.0 if i in target else 0.0 for i in range(20))
        c = self.config
        if not touched or not c.roi_transition_enable or c.roi_transition_ms == 0 or c.roi_mode == "All Sensors":
            self._roi_weights = None
            self._roi_timestamp = None
            return goal
        if not was_touched or self._roi_weights is None:
            weights = goal
        else:
            elapsed = max(0.0, timestamp - self._roi_timestamp)
            step = min(1.0, elapsed * 1000.0 / c.roi_transition_ms)
            weights = tuple(old + max(-step, min(step, new - old))
                            for old, new in zip(self._roi_weights, goal))
        self._roi_weights = weights
        self._roi_timestamp = timestamp
        return weights

    def reset_baseline(self):
        self.reset_tracker()
        self.processor.reset_baseline()

    def interrupt(self):
        """断流/模式切换后清除位移参考，保持已有基线。"""
        self.reset_tracker()
        self.processor.break_capture()

    def on_touch_down(self, x, y):
        pass

    def on_touch_move(self, dx, dy, x, y):
        pass

    def on_touch_up(self, x, y):
        pass

    def update(self, raw_values, timestamp):
        values = validate_raw(raw_values)
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise ValueError("timestamp 必须为有限的单调秒数")
        if self._last_timestamp is not None and timestamp <= self._last_timestamp:
            raise ValueError("timestamp 必须严格递增")
        if self._last_timestamp is not None and timestamp - self._last_timestamp > 0.25:
            self.interrupt()
        self._last_timestamp = timestamp
        c = self.config
        adjusted_raw = tuple(value * gain for value, gain in zip(values, c.raw_gains))
        baseline, signals, ready = self.processor.update(adjusted_raw, timestamp, c)
        maximum = max(range(20), key=signals.__getitem__)
        max_signal = signals[maximum]
        mx, my = self.positions[maximum]
        target_roi = tuple(index for index, (sx, sy) in self.positions.items()
                    if c.roi_mode == "All Sensors" or abs(sx - mx) <= 1 and abs(sy - my) <= 1)
        was_touched = self._touched
        touched = ready and (max_signal >= c.touch_off_threshold if was_touched
                             else max_signal > c.touch_on_threshold)
        # OFF=0 时全零数据仍必须释放。
        touched = touched and max_signal > 0
        roi_weights = self._blend_roi(target_roi, timestamp, touched, was_touched)
        roi = tuple(i for i in range(20) if roi_weights[i] > 0)
        active = tuple(index for index in roi
                       if signals[index] > 0 and signals[index] >= max_signal * c.relative_threshold_ratio)
        transitioning = any(abs(roi_weights[i] - (1.0 if i in target_roi else 0.0)) > 1e-9
                            for i in range(20))
        state = "MOVING" if touched and was_touched else "TOUCH_DOWN" if touched else "TOUCH_UP" if was_touched else "IDLE"
        x_raw, y_raw = self._raw_position or (None, None)
        x, y = self._position or (None, None)
        dx = dy = 0.0
        valid = False
        confidence = "inactive"
        position_held = False
        if touched:
            weights = []
            dt = timestamp - self._derivative_timestamp if self._derivative_timestamp is not None else 0.01
            for index in active:
                # 归一化后取幂与 signal**power 的质心等价，方便移植且避免大数。
                weight = (signals[index] / max_signal) ** c.weight_power * roi_weights[index]
                if c.derivative_weight_enable and self._previous_signal is not None:
                    # 以100Hz差分为参考；批量到达的近零dt不得放大导数。
                    ds = (signals[index] - self._previous_signal[index]) / max_signal
                    normalized_ds = max(-1.0, min(1.0, ds * 0.01 / max(dt, 0.01)))
                    factor = (1.0 + c.positive_gain * normalized_ds if normalized_ds >= 0
                              else max(c.minimum_decay_weight, 1.0 + c.negative_gain * normalized_ds))
                    weight *= factor
                weights.append((index, weight))
            total = math.fsum(weight for _, weight in weights)
            if total > 1e-12:
                x_raw = math.fsum(self.positions[i][0] * w for i, w in weights) / total
                y_raw = math.fsum(self.positions[i][1] * w for i, w in weights) / total
                valid = True
                confidence = self._coordinate_guard.update(signals, (x_raw, y_raw), timestamp, c)
                position_held = self._coordinate_guard.held
                if not was_touched or self._position is None:
                    x, y = x_raw, y_raw
                elif not position_held:
                    alpha = c.position_alpha if c.position_filter_enable else 1.0
                    x = self._position[0] + alpha * (x_raw - self._position[0])
                    y = self._position[1] + alpha * (y_raw - self._position[1])
                    delta_x, delta_y = x - self._position[0], y - self._position[1]
                    dx = (delta_x if abs(delta_x) >= c.dx_deadzone else 0.0) * c.x_gain
                    dy = (delta_y if abs(delta_y) >= c.dy_deadzone else 0.0) * c.y_gain
                    if c.invert_x:
                        dx = -dx
                    if c.invert_y:
                        dy = -dy
                else:
                    # 低可信阶段保留最后输出坐标，原始质心继续计算，位移明确为零。
                    x, y = self._position
                self._position = (x, y)
                self._raw_position = (x_raw, y_raw)
            # 无有效权重时保持上一位置，禁止产生位移。
        if not touched or not valid:
            self._coordinate_guard.reset()
        self._touched = touched
        self._previous_signal = signals
        self._derivative_timestamp = timestamp
        result = TouchResult(timestamp, values, baseline, signals, ready, touched, state,
                             maximum, max_signal, x_raw, y_raw, x, y, dx, dy, roi, active,
                             math.fsum(signals), valid,
                             below_baseline_indices=self.processor.below_baseline_indices,
                             roi_target_indices=target_roi, roi_weights=roi_weights,
                             roi_transitioning=transitioning,
                             adjusted_raw=adjusted_raw, raw_gains=tuple(c.raw_gains),
                             position_held=position_held, position_confidence=confidence)
        if state == "TOUCH_DOWN":
            self.on_touch_down(x, y)
        elif state == "MOVING" and valid:
            self.on_touch_move(dx, dy, x, y)
        elif state == "TOUCH_UP":
            self.on_touch_up(x, y)
            self.reset_tracker()
        return result
