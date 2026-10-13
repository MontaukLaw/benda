"""工作线程逐帧更新，GUI 只读取快照，不堆积逐帧 Qt 信号。"""

from collections import deque
from dataclasses import asdict, replace
import threading
from .touch_tracker import TouchTracker
from .data_recorder import CsvRecorder
from .sample_clock import now as sample_now


class ProcessingEngine:
    def __init__(self, config):
        self.lock = threading.RLock()
        self.tracker = TouchTracker(config)
        self.recorder = CsvRecorder()
        self.result = None
        self.trail = deque(maxlen=1000)
        self.arrivals = deque(maxlen=2000)
        self.last_arrival = None
        self.stream_stale = False
        self.stream_gaps = 0
        self.rejected_samples = 0
        self.samples_processed = 0
        self.revision = 0
        self.source = "None"
        self.metadata = {}
        self.last_touch_event = "—"

    def set_config(self, config, rebuild_baseline=False):
        with self.lock:
            gains_changed = config.raw_gains != self.tracker.config.raw_gains
            self.tracker.set_config(config)
            self.revision += 1
            self.recorder.event("config_changed", config_revision=self.revision, config=asdict(config))
            if gains_changed or rebuild_baseline:
                if not gains_changed:
                    self.tracker.reset_baseline()
                self.trail.clear()
                self.result = None
                self.last_touch_event = "—"
                self.recorder.event("reset_baseline", reason="raw_gains_applied", config_revision=self.revision)

    def reset_tracker(self):
        with self.lock:
            self.tracker.reset_tracker()
            self.trail.clear()
            if self.result:
                self.result = replace(self.result, touched=False, state="IDLE", x=None, y=None,
                                      x_raw=None, y_raw=None, dx=0, dy=0, position_valid=False,
                                      position_held=False, position_confidence="inactive")
            self.recorder.event("reset_tracker")

    def reset_baseline(self):
        with self.lock:
            self.reset_tracker()
            self.tracker.reset_baseline()
            if self.result:
                self.result = replace(self.result, baseline_ready=False)
            self.recorder.event("reset_baseline")

    def mark_observation(self, observation):
        """人为动作标记独立于算法/固件状态，使用同一高精度采样时钟。"""
        if observation not in ("contact", "released"):
            raise ValueError("动作标记必须为 contact 或 released")
        with self.lock:
            if not self.recorder.active:
                return False
            stamp = sample_now()
            result = self.result
            self.recorder.event("user_observation", observation=observation,
                                monotonic_timestamp=stamp, source=self.source,
                                processed_sample_count=self.samples_processed,
                                latest_sample_timestamp=result.timestamp if result else None,
                                latest_sample_age_seconds=stamp - result.timestamp if result else None,
                                algorithm_state=result.state if result else None)
            return True

    def begin_source(self, source):
        with self.lock:
            self.source = source
            # 新连接不复用上个设备或模拟器的 baseline。
            self.tracker = TouchTracker(self.tracker.config)
            self.trail.clear()
            self.arrivals.clear()
            self.result = None
            self.last_arrival = None
            self.stream_stale = False
            self.stream_gaps = self.samples_processed = self.rejected_samples = 0
            self.metadata = {}
            self.last_touch_event = "—"
            self.recorder.event("source_started", source=source)

    def end_source(self):
        with self.lock:
            self._interrupt("source_stopped")
            self.source = "None"
            self.arrivals.clear()

    def _interrupt(self, reason):
        self.tracker.interrupt()
        self.trail.clear()
        if self.result:
            self.result = replace(self.result, touched=False, state="IDLE", dx=0, dy=0, position_valid=False,
                                  position_held=False, position_confidence="inactive")
        self.recorder.event(reason)

    def update(self, raw_values, timestamp, frame=None):
        """真实 Raw 进入 TouchTracker 的唯一入口；模拟输入也复用此路径。"""
        frame = frame or {}
        with self.lock:
            if self.last_arrival is not None and timestamp - self.last_arrival > 0.25:
                if not self.stream_stale:
                    self.stream_gaps += 1
                    self._interrupt("stream_gap")
            try:
                result = self.tracker.update(raw_values, timestamp)
            except (ValueError, TypeError) as exc:
                self.rejected_samples += 1
                self.recorder.event("sample_rejected", message=str(exc))
                return None
            self.stream_stale = False
            self.last_arrival = timestamp
            self.arrivals.append(timestamp)
            self.samples_processed += 1
            self.result = result
            if result.state in ("TOUCH_DOWN", "TOUCH_UP"):
                self.last_touch_event = result.state
            self.metadata = {key: frame.get(key, "—") for key in ("protocol", "protocol_version", "device")}
            if result.state == "TOUCH_DOWN":
                self.trail.clear()
            if result.touched and result.position_valid:
                self.trail.append((timestamp, result.x, result.y))
            self.recorder.record(result, frame, self.source, self.revision)
            return result

    def snapshot(self, now=None):
        now = sample_now() if now is None else now
        with self.lock:
            if self.source != "None" and self.last_arrival is not None and now - self.last_arrival > 0.25:
                if not self.stream_stale:
                    self.stream_gaps += 1
                    self._interrupt("stream_gap")
                self.stream_stale = True
            while self.arrivals and now - self.arrivals[0] > 1.0:
                self.arrivals.popleft()
            # 显示过去1秒收到的采样帧数；不把温度/状态/参数应答算入FPS。
            fps = float(len(self.arrivals)) if self.source != "None" and not self.stream_stale else 0.0
            length = self.tracker.config.trail_length
            trail = tuple(point for point in self.trail if now - point[0] <= 1.0)[-length:]
            return {"result": self.result, "trail": trail, "fps": fps, "stale": self.stream_stale,
                    "stream_gaps": self.stream_gaps, "samples": self.samples_processed,
                    "rejected_samples": self.rejected_samples, "baseline_status": self.tracker.processor.status,
                    "source": self.source, "metadata": dict(self.metadata),
                    "last_touch_event": self.last_touch_event}
