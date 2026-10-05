import threading
import time
from collections import deque

from config import CHANNEL_COUNT, DEFAULT_HISTORY_LENGTH
from raw_response import RawResponse


RX_FPS_WINDOW_SECONDS = 1.0
RX_FPS_STALE_SECONDS = 0.5


def _make_run_force_vector(frame, region, now):
    normal_force, tangential_force, theta = frame["vectors"][region]
    component_mask = int(frame.get("component_calibration_mask", 0))
    component_shift = region * 3
    return {
        "source": "run",
        "device": frame["device"],
        "channel_count": frame["channel_count"],
        "normal_force": normal_force,
        "optimized_normal_force": frame.get("optimized_normal_force") if region == 0 else None,
        "low_force_simulated": region == 0 and bool(frame.get("low_force_simulated", False)),
        "measured_normal_force": frame.get("measured_normal_force") if region == 0 else None,
        "measured_optimized_normal_force": frame.get("measured_optimized_normal_force") if region == 0 else None,
        "low_force_mode": region == 0 and int(frame.get("protocol_version", 0)) >= 17 and bool(component_mask & 0x40),
        "low_force_over_range": region == 0 and int(frame.get("protocol_version", 0)) >= 17 and bool(component_mask & 0x80),
        "low_force_calibration_valid": region == 0 and int(frame.get("protocol_version", 0)) >= 17 and bool(component_mask & 0x100),
        "tangential_force": tangential_force,
        "theta": theta,
        "calibration_valid": bool(frame["calibration_mask"] & (1 << region)),
        "normal_calibration_valid": bool(
            component_mask & (1 << component_shift)
        ),
        "tangential_calibration_valid": bool(
            component_mask & (1 << (component_shift + 1))
        ),
        "direction_calibration_valid": bool(
            component_mask & (1 << (component_shift + 2))
        ),
        "timestamp": now,
    }


class AlgoDataBuffer:
    def __init__(self, sample_count=DEFAULT_HISTORY_LENGTH):
        self._lock = threading.Lock()
        self._raw_response = RawResponse()
        self._raw_config_signature = None
        self._sample_count = sample_count
        self._channel_count = CHANNEL_COUNT
        self._raw = self._make_channels(self._channel_count, sample_count, float("nan"))
        self._baseline = self._make_channels(self._channel_count, sample_count, float("nan"))
        self._signal = self._make_channels(self._channel_count, sample_count)
        self._fast_k_percent = self._make_channels(self._channel_count, sample_count, float("nan"))
        self._slow_k_percent = self._make_channels(self._channel_count, sample_count, float("nan"))
        self._state_history = self._make_channels(self._channel_count, sample_count, -1)
        self._states = [-1] * self._channel_count
        self._run_force_vector = None
        self._die_temperature_celsius = None
        self._die_temperature_frame_time = None
        self._force_vector = None
        self._finger_pad_force_vector = None
        self._frame_times = deque(maxlen=300)
        self._latest_sensor_frame_time = None
        self._latest_sensor_has_raw = False
        self._latest_sensor_raw_only = False
        self._total_frames = 0
        # 保存短队列供相对位移逐帧消费，避免GUI刷新只取末帧而漏掉轨迹。
        self._pointer_samples = deque(maxlen=512)
        self._pending_channel_statistics = None
        self._channel_statistics = None
        self._version = 0

    def append_frame(self, frame):
        now = time.perf_counter()

        with self._lock:
            # 旧固件慢基线帧仅在解析层识别，运行时不维护或展示。
            if frame["type"] == "slow_baseline":
                return

            if frame["type"] == "channel_4567_statistics":
                self._pending_channel_statistics = dict(frame, timestamp=now)
                return

            if frame["type"] == "channel_4567_signal_sum":
                # Legacy V13 frame: V14 calculates selectable Signal sums in the host UI.
                return

            if frame["type"] == "run_force":
                self._run_force_vector = _make_run_force_vector(frame, 0, now)
                self._force_vector = dict(self._run_force_vector)
                if frame["vector_count"] > 1:
                    self._finger_pad_force_vector = _make_run_force_vector(frame, 1, now)
                else:
                    self._finger_pad_force_vector = None
                self._version += 1
                return

            if frame["type"] == "temperature":
                self._die_temperature_celsius = float(frame["temperature_celsius"])
                self._die_temperature_frame_time = now
                self._version += 1
                return

            if frame["type"] in ("force_vector", "finger_pad_force_vector"):
                target = "_force_vector" if frame["type"] == "force_vector" else "_finger_pad_force_vector"
                vector = dict(frame)
                vector["source"] = "debug"
                vector["timestamp"] = now
                setattr(self, target, vector)
                self._version += 1
                return

            channel_count = int(frame["channel_count"])
            # 统计帧先于同一扫描的通道帧；只消费一次，缺帧和旧固件均不沿用旧值。
            pending = self._pending_channel_statistics
            self._pending_channel_statistics = None
            self._channel_statistics = (
                pending if pending is not None and channel_count >= 8
                and pending["device"] == frame["device"]
                and now - pending["timestamp"] <= RX_FPS_STALE_SECONDS else None
            )
            self._set_channel_count_locked(channel_count)
            self._frame_times.append(now)
            self._latest_sensor_frame_time = now
            self._latest_sensor_has_raw = frame["type"] == "debug"
            # 保留Raw帧的真实来源标记，触摸板不能把补零Signal当成实际测量。
            self._latest_sensor_raw_only = self._latest_sensor_has_raw and bool(frame.get('raw_only', False))
            self._raw_response.feed(frame.get("raw") if self._latest_sensor_has_raw else None, frame["device"], now)

            if frame["type"] == "debug":
                raw_values = frame["raw"]
                baseline_values = frame["baseline"]
                signal_values = frame["signal"]
                fast_k_percent_values = frame["fast_k_percent"]
                slow_k_percent_values = frame["slow_k_percent"]
                self._states = list(frame["state"])
            else:
                signal_values = frame["signal"]
                # Run帧没有诊断字段；NaN让曲线断开，-1表示状态未上传而非IDLE。
                raw_values = [float("nan")] * channel_count
                baseline_values = [float("nan")] * channel_count
                fast_k_percent_values = [float("nan")] * channel_count
                slow_k_percent_values = [float("nan")] * channel_count
                self._states = [-1] * channel_count

            for channel in range(channel_count):
                self._raw[channel].append(raw_values[channel])
                self._baseline[channel].append(baseline_values[channel])
                self._signal[channel].append(signal_values[channel])
                self._fast_k_percent[channel].append(fast_k_percent_values[channel])
                self._slow_k_percent[channel].append(slow_k_percent_values[channel])
                self._state_history[channel].append(self._states[channel])
            self._total_frames += 1
            self._pointer_samples.append(dict(
                sequence=self._total_frames, sample_timestamp=now,
                has_raw=self._latest_sensor_has_raw, channel_count=channel_count,
                raw_only=self._latest_sensor_raw_only,
                raw=tuple(raw_values) if self._latest_sensor_has_raw else (),
                signal=tuple(signal_values),
                state=tuple(self._states) if self._latest_sensor_has_raw else ()))
            self._version += 1

    def pointer_samples_after(self, sequence):
        """返回独立快照；队列溢出由消费者通过序号缺口识别并断开轨迹。"""
        with self._lock:
            return [dict(sample) for sample in self._pointer_samples
                    if sample['sequence'] > sequence]

    def start_raw_reference(self):
        with self._lock:
            now = time.perf_counter()
            if (not self._latest_sensor_has_raw or self._channel_count < 8
                    or self._latest_sensor_frame_time is None
                    or now - self._latest_sensor_frame_time > 0.5):
                self._raw_response.reset("需要实时八通道Debug Hardware Raw，请连接并切换Debug")
                return
            self._raw_response.start(now)

    def clear_raw_reference(self, message="参考已清除，请重新采集"):
        with self._lock:
            self._raw_response.reset(message)

    def update_raw_reference_config(self, channels):
        signature = tuple(
            (c.get("channel"), c.get("hardware_nsub"), c.get("hardware_tx_clk_div"), c.get("hardware_ref_cdac"))
            for c in channels
        )
        with self._lock:
            if self._raw_config_signature is not None and signature != self._raw_config_signature:
                self._raw_response.reset("三参数改变，需重新采集参考")
            self._raw_config_signature = signature

    def snapshot(self):
        with self._lock:
            channel_count = self._channel_count
            raw = [list(channel) for channel in self._raw]
            baseline = [list(channel) for channel in self._baseline]
            signal = [list(channel) for channel in self._signal]
            fast_k_percent = [list(channel) for channel in self._fast_k_percent]
            slow_k_percent = [list(channel) for channel in self._slow_k_percent]
            states = list(self._states)
            state_history = [list(channel) for channel in self._state_history]
            fps = self._calculate_fps_locked()
            total_frames = self._total_frames
            sample_count = self._sample_count
            force_vector = dict(self._force_vector) if self._force_vector is not None else None
            finger_pad_force_vector = dict(self._finger_pad_force_vector) if self._finger_pad_force_vector is not None else None
            version = self._version
        return raw, baseline, signal, fast_k_percent, slow_k_percent, states, state_history, fps, total_frames, sample_count, channel_count, force_vector, finger_pad_force_vector, version

    def latest_sensor_sample(self):
        """返回自动测试需要的最新通道数据，避免复制整段绘图历史。"""
        with self._lock:
            if self._latest_sensor_frame_time is None:
                return None
            run_force_vector = (
                None
                if self._run_force_vector is None
                else dict(self._run_force_vector)
            )
            return {
                "sequence": self._total_frames,
                "sample_timestamp": self._latest_sensor_frame_time,
                "raw_response": self._raw_response.snapshot(time.perf_counter()),
                "channel_count": self._channel_count,
                # 未上传的诊断值返回空元组，避免记录器或测试把占位值当成测量值。
                "raw": tuple(channel[-1] for channel in self._raw) if self._latest_sensor_has_raw else (),
                "hardware_raw": tuple(channel[-1] for channel in self._raw) if self._latest_sensor_has_raw else (),
                "baseline": tuple(channel[-1] for channel in self._baseline) if self._latest_sensor_has_raw else (),
                "signal": tuple(channel[-1] for channel in self._signal),
                "fast_k_percent": tuple(channel[-1] for channel in self._fast_k_percent) if self._latest_sensor_has_raw else (),
                "slow_k_percent": tuple(channel[-1] for channel in self._slow_k_percent) if self._latest_sensor_has_raw else (),
                "state": tuple(self._states) if self._latest_sensor_has_raw else (),
                "mcu_force_vector": run_force_vector,
                "channel_4567_statistics": (
                    dict(self._channel_statistics) if self._channel_statistics is not None else None
                ),
                "mcu_force_age_seconds": (
                    None
                    if run_force_vector is None
                    else max(0.0, time.perf_counter() - run_force_vector["timestamp"])
                ),
                "die_temperature_celsius": self._die_temperature_celsius,
                "die_temperature_age_seconds": (
                    None
                    if self._die_temperature_frame_time is None
                    else max(0.0, time.perf_counter() - self._die_temperature_frame_time)
                ),
                "has_raw": self._latest_sensor_has_raw,
                "raw_only": self._latest_sensor_raw_only,
                "age_seconds": time.perf_counter() - self._latest_sensor_frame_time,
            }

    def set_sample_count(self, sample_count):
        with self._lock:
            if sample_count == self._sample_count:
                return
            self._sample_count = sample_count
            self._raw = [self._resize_channel(channel, sample_count, float("nan")) for channel in self._raw]
            self._baseline = [self._resize_channel(channel, sample_count, float("nan")) for channel in self._baseline]
            self._signal = [self._resize_channel(channel, sample_count) for channel in self._signal]
            self._fast_k_percent = [self._resize_channel(channel, sample_count, float("nan")) for channel in self._fast_k_percent]
            self._slow_k_percent = [self._resize_channel(channel, sample_count, float("nan")) for channel in self._slow_k_percent]
            self._state_history = [self._resize_channel(channel, sample_count, -1) for channel in self._state_history]
            self._version += 1

    def _set_channel_count_locked(self, channel_count):
        if channel_count == self._channel_count:
            return
        self._channel_count = channel_count
        self._raw = self._make_channels(channel_count, self._sample_count, float("nan"))
        self._baseline = self._make_channels(channel_count, self._sample_count, float("nan"))
        self._signal = self._make_channels(channel_count, self._sample_count)
        self._fast_k_percent = self._make_channels(channel_count, self._sample_count, float("nan"))
        self._slow_k_percent = self._make_channels(channel_count, self._sample_count, float("nan"))
        self._state_history = self._make_channels(channel_count, self._sample_count, -1)
        self._states = [-1] * channel_count

    def _make_channels(self, channel_count, sample_count, fill=0):
        return [deque([fill] * sample_count, maxlen=sample_count) for _ in range(channel_count)]

    def _resize_channel(self, channel, sample_count, fill=0):
        values = list(channel)[-sample_count:]
        if len(values) < sample_count:
            values = ([fill] * (sample_count - len(values))) + values
        return deque(values, maxlen=sample_count)

    def _calculate_fps_locked(self):
        now = time.perf_counter()
        cutoff = now - RX_FPS_WINDOW_SECONDS
        while len(self._frame_times) > 2 and self._frame_times[1] < cutoff:
            self._frame_times.popleft()

        if len(self._frame_times) < 2:
            return 0.0
        if (now - self._frame_times[-1]) > RX_FPS_STALE_SECONDS:
            return 0.0
        elapsed = self._frame_times[-1] - self._frame_times[0]
        if elapsed <= 0:
            return 0.0
        return (len(self._frame_times) - 1) / elapsed
