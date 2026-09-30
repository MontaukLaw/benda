"""固定实验参考下的四通道Hardware Raw响应；不依赖固件baseline和状态。"""
from datetime import datetime
import math


class RawResponse:
    CAPTURE_SECONDS = 2.0
    MAX_REFERENCE_STD = 3.0
    MAX_REFERENCE_DRIFT = 3.0

    def __init__(self):
        self.last_time = None
        self.device = None
        self.reset()

    def reset(self, message="请保持当前预紧力不变，采集施力前参考"):
        self.reference = None
        self.noise = None
        self.reference_id = ""
        self.capture_start = None
        self.samples = []
        self.result = None
        self.message = message

    def start(self, now):
        self.reset("正在采集2秒参考，请保持负载不变")
        self.capture_start = now

    def check_stale(self, now):
        if self.last_time is not None and now - self.last_time > 0.5:
            if self.reference is not None or self.capture_start is not None:
                self.reset("数据中断，需重新采集参考")

    def feed(self, raw, device, now):
        self.check_stale(now)
        if self.device is not None and device != self.device:
            self.reset("设备改变，需重新采集参考")
        self.device = device
        self.last_time = now
        if raw is None or len(raw) < 8:
            self.reset("需要八通道Debug Hardware Raw，请切换Debug后采集参考")
            return
        values = tuple(float(x) for x in raw[4:8])
        if not all(math.isfinite(x) and 0 <= x <= 65535 for x in values):
            self.reset("Hardware Raw无效，需重新采集参考")
            return
        if self.capture_start is not None:
            self.samples.append((now, values))
            if now - self.capture_start < self.CAPTURE_SECONDS:
                return
            samples = self.samples
            self.capture_start = None
            self.samples = []
            if len(samples) < 30 or samples[-1][0] - samples[0][0] < 1.8:
                self.reset("有效参考样本不足，请重新采集")
                return
            columns = list(zip(*(v for _, v in samples)))
            means = tuple(sum(v) / len(v) for v in columns)
            noise = tuple(math.sqrt(sum((x - m)**2 for x in v) / len(v)) for v, m in zip(columns, means))
            half = len(samples) // 2
            drift = max(abs(sum(v[:half]) / half - sum(v[half:]) / (len(v) - half)) for v in columns)
            if max(noise) > self.MAX_REFERENCE_STD or drift > self.MAX_REFERENCE_DRIFT:
                self.reset("参考仍在变化，请保持负载稳定后重新采集")
                return
            self.reference = means
            self.noise = noise
            self.reference_id = datetime.now().astimezone().isoformat(timespec="microseconds")
            self.message = f"参考已固定（{len(samples)}帧）；再施力观察响应"
        if self.reference is None:
            return
        delta = tuple(x - b for x, b in zip(values, self.reference))
        mean = sum(delta) / 4.0
        std = math.sqrt(sum((x - mean)**2 for x in delta) / 4.0)
        # 保守地按最噪通道设置低响应门槛，避免小分母放大噪声。
        minimum = max(2.0, 3.0 * max(self.noise))
        valid = mean > minimum and min(delta) >= 0.0
        status = "有效" if valid else ("存在负增量，不解释为受力占比" if min(delta) < 0 else "增量太小，百分比暂不计算")
        self.result = {
            "reference_id": self.reference_id, "reference": self.reference,
            "delta": delta, "mean": mean, "stddev": std, "minimum_mean": minimum,
            "cv_percent": std / mean * 100.0 if valid else None,
            "shares_percent": tuple(x / (4.0 * mean) * 100.0 for x in delta) if valid else None,
            "status": status,
        }

    def snapshot(self, now):
        self.check_stale(now)
        return {
            "message": self.message,
            "capturing": self.capture_start is not None,
            "result": dict(self.result) if self.result is not None else None,
        }
