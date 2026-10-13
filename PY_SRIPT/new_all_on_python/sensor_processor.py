"""只消费真实 Raw；本地 baseline/signal 不替换固件上报数据或状态。"""

from collections import deque
from statistics import median


def validate_raw(raw_values):
    values = tuple(raw_values)
    if len(values) != 20:
        raise ValueError("Raw 必须包含20通道")
    if any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 65535 for v in values):
        raise ValueError("Raw 必须为0～65535的整数")
    return values


class SensorProcessor:
    """冻结明显受压通道；重新建基线需连续稳定的空载窗口。"""

    def __init__(self):
        self.baseline = None
        self._capture = deque(maxlen=1000)
        self.calibrating = True
        self._below_since = [None] * 20
        self.below_baseline_indices = ()
        self.status = "请释放阵列，等待50帧且至少0.5秒稳定数据"

    def reset_baseline(self):
        # 原基线暂存，但重建期间不输出触摸；不把单帧按压值直接设为基线。
        self._capture.clear()
        self._below_since = [None] * 20
        self.below_baseline_indices = ()
        self.calibrating = True
        self.status = "请保持空载，重新采集稳定基线"

    def break_capture(self):
        self._capture.clear()
        self._below_since = [None] * 20
        self.below_baseline_indices = ()
        if not self.calibrating:
            self.status = "基线已建立（PC本地）"

    def update(self, values, timestamp, config):
        if self.calibrating:
            self._capture.append((timestamp, values))
            # 以时间窗口为准，避免实际帧率高于100Hz时50帧永远不足0.5秒。
            while len(self._capture) > 50 and timestamp - self._capture[1][0] >= 0.5:
                self._capture.popleft()
            ready = len(self._capture) >= 50 and timestamp - self._capture[0][0] >= 0.5
            if ready:
                channels = list(zip(*(item[1] for item in self._capture)))
                stable = all(max(ch) - min(ch) <= config.baseline_stability_limit for ch in channels)
                if stable:
                    self.baseline = [float(median(ch)) for ch in channels]
                    self.calibrating = False
                    self._capture.clear()
                    self.status = "基线就绪（PC本地）"
                else:
                    self.status = "Raw 波动超限；请保持空载或调整稳定窗口允许波动"
            else:
                self.status = f"基线采集中 {len(self._capture)} 帧 / 至少50帧；请保持空载至少0.5秒"
            return tuple(self.baseline or values), (0.0,) * 20, False

        signals = []
        for index, raw in enumerate(values):
            difference = float(raw) - self.baseline[index]
            # 使用带符号差值保护两侧：负向下冲不能因 Signal 截零而拉低基线。
            # 阈值内的小幅空载漂移仍可双向缓慢跟踪；大幅漂移需空载重建。
            if abs(difference) < config.baseline_freeze_threshold:
                self.baseline[index] += config.baseline_update_rate * difference
            signal = max(float(raw) - self.baseline[index], 0.0)
            signals.append(signal if signal >= config.noise_threshold else 0.0)
        # 持续负差只作诊断：可能是负向耦合/回弹或基线偏高，不据此改写触摸状态。
        negative_limit = max(config.baseline_freeze_threshold, config.noise_threshold,
                             config.baseline_stability_limit, 1.0)
        below = []
        for index, raw in enumerate(values):
            if self.baseline[index] - raw > negative_limit:
                if self._below_since[index] is None:
                    self._below_since[index] = timestamp
                if timestamp - self._below_since[index] >= 0.5:
                    below.append(index)
            else:
                self._below_since[index] = None
        self.below_baseline_indices = tuple(below)
        if below:
            largest = sorted(below, key=lambda i: self.baseline[i] - values[i], reverse=True)
            channels = ", ".join(f"CH{i}" for i in largest[:5])
            if len(largest) > 5:
                channels += f" 等{len(largest)}通道"
            worst = values[largest[0]] - self.baseline[largest[0]]
            self.status = (f"持续低于基线 ≥0.5秒：{channels}；最大负差 {worst:.0f} counts。"
                           "Signal 截零可能漏触，请确认释放稳定后评估基线。")
        else:
            self.status = "基线已建立（PC本地）"
        return tuple(self.baseline), tuple(signals), True
