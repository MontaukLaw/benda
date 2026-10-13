"""衰减时仅暂停坐标，不改信号、基线或触摸判定；按单调时间评估新位置支持。"""

from collections import deque
import math


class CoordinateGuard:
    def __init__(self, positions):
        self.positions = positions
        self.reset()

    def reset(self):
        self.history = deque(maxlen=512)
        self.held = False
        self.floor = None
        self.recovery_since = None
        self.motion = None
        self.motion_timestamp = None

    def _local_growth(self, signals, reference, candidate):
        x, y = candidate
        return sum(max(0.0, signals[i] - reference[i])
                   for i, (sx, sy) in self.positions.items()
                   if abs(sx - x) <= 1 and abs(sy - y) <= 1)

    def _uncertain_move(self, candidate, timestamp):
        previous = self.history[-1][4]
        step = (candidate[0] - previous[0], candidate[1] - previous[1])
        length = math.hypot(*step)
        if length <= 0.01:
            return False
        # 近期几乎静止时，大幅漂移不可信；运动时保护超过60度的突然转向。
        if self.motion is None or timestamp - self.motion_timestamp > 0.2:
            return length > 0.03
        return sum(a * b for a, b in zip(step, self.motion)) < 0.5 * length

    def update(self, signals, candidate, timestamp, config):
        if not config.decay_guard_enable:
            self.reset()
            return "disabled"
        window = config.decay_guard_window_ms / 1000.0
        while self.history and self.history[0][0] < timestamp - window:
            self.history.popleft()
        maximum, total = max(signals), sum(signals)
        support = max(config.noise_threshold, maximum * 0.10, 1.0)
        status = "tracking"
        if self.held:
            self.floor = tuple(min(old, value) for old, value in zip(self.floor, signals))
            supported = self._local_growth(signals, self.floor, candidate) >= support
            if supported:
                if self.recovery_since is None:
                    self.recovery_since = timestamp
                # 连续20ms出现新的局部响应后恢复，单帧波动不能解除保护。
                if timestamp - self.recovery_since >= 0.020 - 1e-9:
                    self.held = False
                    self.floor = None
                    self.recovery_since = None
                    self.history.clear()
                    self.motion = None
                    self.motion_timestamp = None
                else:
                    status = "recovering"
            else:
                self.recovery_since = None
            if self.held and status == "tracking":
                status = "decay_hold"
        elif self.history:
            # 用同一个近期强响应样本比较MAX与总量，要求两者同时明显下降。
            reference = max(self.history, key=lambda item: item[2])
            elapsed = timestamp - reference[0]
            fraction = 1 - config.decay_guard_drop_ratio
            falling = (elapsed >= 0.005 and maximum <= reference[3] * fraction
                       and total <= reference[2] * fraction)
            growth = self._local_growth(signals, reference[1], candidate)
            if (falling and growth < support
                    and self._uncertain_move(candidate, timestamp)):
                self.held = True
                self.floor = tuple(signals)
                self.recovery_since = None
                status = "decay_hold"
            elif (maximum >= reference[3] * 0.90 and total >= reference[2] * 0.90) or growth >= support:
                # 衰减初期不把尾部漂移学成新的运动方向；新通道增长仍允许真实转弯。
                direction = (candidate[0] - self.history[0][4][0],
                             candidate[1] - self.history[0][4][1])
                length = math.hypot(*direction)
                if length >= 0.03:
                    self.motion = tuple(v / length for v in direction)
                    self.motion_timestamp = timestamp
        self.history.append((timestamp, tuple(signals), total, maximum, tuple(candidate)))
        return status
