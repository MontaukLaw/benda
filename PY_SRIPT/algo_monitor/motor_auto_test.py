"""基于电机位置和实时传感器数据执行自动零点与量程搜索。"""

from __future__ import annotations

import math
import csv
import random
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Mapping, Optional

from channel_states import (
    STATE_IDLE as CHANNEL_STATE_IDLE,
    STATE_RISING as CHANNEL_STATE_RISING,
    STATE_FALLING as CHANNEL_STATE_FALLING,
    STATE_SLOW_BACK as CHANNEL_STATE_SLOW_BACK,
    STATE_FLOATING as CHANNEL_STATE_FLOATING,
    STATE_NAMES as CHANNEL_STATE_NAMES, IDLE_LIKE_STATES,
)

CONTACT_CHANNELS = (4, 5, 6, 7)
MOTOR_STATE_STOPPED = 0
MOTOR_STATE_TASK_COMPLETE = 1
MOTOR_STATE_RUNNING = 2
MOTOR_STATE_OVERLOAD = 3
MOTOR_STATE_STALLED = 4
MOTOR_STATE_UNDERVOLTAGE = 5
MOTOR_FINISHED_STATES = (MOTOR_STATE_STOPPED, MOTOR_STATE_TASK_COMPLETE)
MOTOR_FAULT_NAMES = {
    MOTOR_STATE_OVERLOAD: "过载",
    MOTOR_STATE_STALLED: "堵转",
    MOTOR_STATE_UNDERVOLTAGE: "欠压",
}
# 快速连续重复精度测试最多允许尝试目标有效次数的十倍，防止电机持续定位异常时无限运行。
REPEATABILITY_MAX_ATTEMPT_MULTIPLIER = 10
# 快速连续测试正式统计前最多执行十轮材料预压。
REPEATABILITY_PRECONDITION_MAX_ROUNDS = 10
# 使用最近三个有效预压结果判断材料是否已经稳定。
REPEATABILITY_PRECONDITION_WINDOW = 3
# 最近预压窗口相对极差不超过百分之零点五时允许开始正式测试。
REPEATABILITY_PRECONDITION_RELATIVE_SPAN_PERCENT_MAX = 0.5
# 快速连续测试只要求四个受力通道中至少一个进入 FLOATING。
RAPID_REPEATABILITY_FLOATING_CHANNEL_MIN = 1
# 正式测试中同一个非零下压停点连续出现三次后，重新锁定实际可达位置。
REPEATABILITY_FORMAL_RELOCK_CONFIRMATION_ATTEMPTS = 3
# 限制一次测试最多重新锁定三次，避免机械位置持续漂移时无限循环。
REPEATABILITY_FORMAL_MAX_RELOCKS = 3


class MotorAutoTestError(RuntimeError):
    """自动测试无法继续时抛出的可读错误。"""


class MotorAutoTestCancelled(MotorAutoTestError):
    """用户主动停止自动测试。"""


@dataclass(frozen=True)
class MotorAutoTestConfig:
    """自动测试参数；所有角度单位均为度。"""

    zero_step_deg: float = 5.0
    max_step_deg: float = 10.0
    contact_signal_threshold: int = 0
    raw_limit: int = 60000
    ready_lift_deg: float = 5.0
    sensor_settle_seconds: float = 0.30
    sensor_frame_timeout_seconds: float = 1.0
    motor_position_timeout_seconds: float = 8.0
    max_search_travel_deg: float = 720.0
    confirmation_frames: int = 3
    position_tolerance_counts: int = 20
    position_stable_samples: int = 3
    poll_interval_seconds: float = 0.01

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "MotorAutoTestConfig":
        """从 JSON 字典读取参数，并拒绝危险或不可用的数值。"""

        config = cls(
            zero_step_deg=_positive_float(values, "auto_zero_step_deg", cls.zero_step_deg),
            max_step_deg=_positive_float(values, "auto_max_step_deg", cls.max_step_deg),
            contact_signal_threshold=_nonnegative_int(
                values,
                "auto_contact_signal_threshold",
                cls.contact_signal_threshold,
            ),
            raw_limit=_positive_int(values, "auto_raw_limit", cls.raw_limit),
            ready_lift_deg=_nonnegative_float(
                values,
                "auto_ready_lift_deg",
                cls.ready_lift_deg,
            ),
            sensor_settle_seconds=_nonnegative_float(
                values,
                "auto_sensor_settle_seconds",
                cls.sensor_settle_seconds,
            ),
            sensor_frame_timeout_seconds=_positive_float(
                values,
                "auto_sensor_frame_timeout_seconds",
                cls.sensor_frame_timeout_seconds,
            ),
            motor_position_timeout_seconds=_positive_float(
                values,
                "auto_motor_position_timeout_seconds",
                cls.motor_position_timeout_seconds,
            ),
            max_search_travel_deg=_positive_float(
                values,
                "auto_max_search_travel_deg",
                cls.max_search_travel_deg,
            ),
            confirmation_frames=_positive_int(
                values,
                "auto_confirmation_frames",
                cls.confirmation_frames,
            ),
        )
        if config.raw_limit > 65535:
            raise ValueError("auto_raw_limit 必须在 1～65535 之间")
        if config.zero_step_deg > config.max_search_travel_deg:
            raise ValueError("auto_zero_step_deg 不能大于 auto_max_search_travel_deg")
        if config.max_step_deg > config.max_search_travel_deg:
            raise ValueError("auto_max_step_deg 不能大于 auto_max_search_travel_deg")
        return config


def _number(values: Mapping[str, object], key: str, default: float) -> float:
    value = values.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} 必须是数字")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{key} 必须是有限数字")
    return value


def _positive_float(values: Mapping[str, object], key: str, default: float) -> float:
    value = _number(values, key, default)
    if value <= 0.0:
        raise ValueError(f"{key} 必须大于 0")
    return value


def _nonnegative_float(values: Mapping[str, object], key: str, default: float) -> float:
    value = _number(values, key, default)
    if value < 0.0:
        raise ValueError(f"{key} 不能小于 0")
    return value


def _positive_int(values: Mapping[str, object], key: str, default: int) -> int:
    value = _number(values, key, default)
    if value <= 0.0 or not value.is_integer():
        raise ValueError(f"{key} 必须是正整数")
    return int(value)


def _nonnegative_int(values: Mapping[str, object], key: str, default: int) -> int:
    value = _number(values, key, default)
    if value < 0.0 or not value.is_integer():
        raise ValueError(f"{key} 必须是非负整数")
    return int(value)


class MotorAutoTester:
    """在工作线程中按步进方式寻找接触零点和原始值上限。"""

    def __init__(
        self,
        motor,
        sensor_sample_provider: Callable[[], Optional[dict]],
        stop_event: threading.Event,
        config: MotorAutoTestConfig,
        speed_rpm: int,
        acceleration: int = 100,
        progress_callback: Optional[Callable[[str], None]] = None,
        position_callback: Optional[Callable[[float], None]] = None,
    ):
        self.motor = motor
        self.sensor_sample_provider = sensor_sample_provider
        self.stop_event = stop_event
        self.config = config
        self.speed_rpm = int(speed_rpm)
        self.acceleration = int(acceleration)
        self.progress_callback = progress_callback
        self.position_callback = position_callback
        self.safe_return_counts = None

    def find_zero(self) -> dict:
        """每次下压固定角度，直到 CH4～CH7 Signal 和连续越过阈值。"""

        self._check_cancelled()
        start_counts = self.motor.read_position_counts()
        self.safe_return_counts = start_counts
        current_counts = start_counts
        step_counts = self._degrees_to_counts(self.config.zero_step_deg)
        maximum_travel_counts = self._degrees_to_counts(self.config.max_search_travel_deg)
        step_index = 0

        while True:
            contacted, sample, signal_sum = self._read_confirmed_condition(
                lambda current: sum(current["signal"][channel] for channel in CONTACT_CHANNELS)
                > self.config.contact_signal_threshold,
                require_raw=False,
            )
            if contacted:
                zero_counts = self.motor.read_position_counts()
                self._emit_position(zero_counts)
                ready_counts = zero_counts + self._degrees_to_counts(
                    self.config.ready_lift_deg
                )
                if ready_counts != zero_counts:
                    self._emit_progress(
                        f"找到零点，Signal 和={signal_sum}；正在抬升 "
                        f"{ready_counts - zero_counts} counts 到准备位"
                    )
                    self._move_to_counts(ready_counts)
                return {
                    "kind": "zero",
                    "zero_position_counts": zero_counts,
                    "zero_position_deg": self._counts_to_degrees(zero_counts),
                    "ready_position_counts": ready_counts,
                    "signal_sum": signal_sum,
                    "steps": step_index,
                    "sample_sequence": sample["sequence"],
                }

            if abs(current_counts - start_counts) + step_counts > maximum_travel_counts:
                raise MotorAutoTestError(
                    f"下压已达到安全搜索范围 {maximum_travel_counts} counts，"
                    "仍未找到接触零点"
                )

            step_index += 1
            self._emit_progress(
                f"寻找零点：第 {step_index} 步，下压 {step_counts} counts；"
                f"当前 Signal 和={signal_sum}"
            )
            target_counts = current_counts - step_counts
            current_counts = self._move_to_counts(target_counts)

    def find_max_range(self, zero_position_counts: int) -> dict:
        """每次下压固定角度，直到 CH4～CH7 任一 Raw 达到量程阈值。"""

        self._check_cancelled()
        current_counts = self.motor.read_position_counts()
        self._emit_position(current_counts)
        step_counts = self._degrees_to_counts(self.config.max_step_deg)
        maximum_travel_counts = self._degrees_to_counts(self.config.max_search_travel_deg)
        ready_counts = zero_position_counts + self._degrees_to_counts(
            self.config.ready_lift_deg
        )
        self.safe_return_counts = ready_counts
        step_index = 0

        while True:
            reached, sample, _unused = self._read_confirmed_condition(
                lambda current: any(
                    current["raw"][channel] >= self.config.raw_limit
                    for channel in CONTACT_CHANNELS
                ),
                require_raw=True,
            )
            monitored_raw = [sample["raw"][channel] for channel in CONTACT_CHANNELS]
            if reached:
                maximum_counts = self.motor.read_position_counts()
                self._emit_position(maximum_counts)
                maximum_press_deg = max(
                    0.0,
                    self._counts_to_degrees(zero_position_counts - maximum_counts),
                )
                trigger_channels = [
                    channel
                    for channel in CONTACT_CHANNELS
                    if sample["raw"][channel] >= self.config.raw_limit
                ]
                self._emit_progress(
                    f"找到最大量程，CH{','.join(str(channel) for channel in trigger_channels)} "
                    f"Raw 达到 {self.config.raw_limit}；正在返回准备位"
                )
                if maximum_counts != ready_counts:
                    self._move_to_counts(ready_counts)
                return {
                    "kind": "max_range",
                    "maximum_position_counts": maximum_counts,
                    "maximum_position_deg": self._counts_to_degrees(maximum_counts),
                    "maximum_press_angle_deg": maximum_press_deg,
                    "ready_position_counts": ready_counts,
                    "raw_values": monitored_raw,
                    "trigger_channels": trigger_channels,
                    "steps": step_index,
                    "sample_sequence": sample["sequence"],
                }

            next_counts = current_counts - step_counts
            down_travel_counts = zero_position_counts - next_counts
            if down_travel_counts > maximum_travel_counts:
                raise MotorAutoTestError(
                    f"相对零点下压将超过安全范围 {maximum_travel_counts} counts，"
                    "仍未达到 Raw 上限"
                )

            step_index += 1
            self._emit_progress(
                f"寻找最大量程：第 {step_index} 步，下压 {step_counts} counts；"
                f"CH4～CH7 Raw={monitored_raw}"
            )
            current_counts = self._move_to_counts(next_counts)

    def recover_to_safe_position(self) -> Optional[str]:
        """搜索异常时尽量卸载压力；返回 None 表示恢复成功或无需恢复。"""

        if self.safe_return_counts is None or self.stop_event.is_set():
            return None
        try:
            self._emit_progress("自动测试异常，正在返回安全位置")
            self._move_to_counts(self.safe_return_counts)
        except Exception as exc:
            return str(exc)
        return None

    def _read_confirmed_condition(self, predicate, require_raw):
        if self.stop_event.wait(self.config.sensor_settle_seconds):
            raise MotorAutoTestCancelled("自动测试已停止")

        deadline = time.monotonic() + self.config.sensor_frame_timeout_seconds
        last_sequence = None
        samples = []
        decisions = []
        while len(samples) < self.config.confirmation_frames:
            self._check_cancelled()
            sample = self.sensor_sample_provider()
            self._validate_sensor_sample(sample, require_raw)
            if sample["sequence"] != last_sequence:
                last_sequence = sample["sequence"]
                samples.append(sample)
                decisions.append(bool(predicate(sample)))
            if len(samples) >= self.config.confirmation_frames:
                break
            if time.monotonic() >= deadline:
                raise MotorAutoTestError("等待新的传感器数据帧超时，请检查串口和 Debug 输出模式")
            self.stop_event.wait(self.config.poll_interval_seconds)

        last_sample = samples[-1]
        signal_sum = sum(last_sample["signal"][channel] for channel in CONTACT_CHANNELS)
        return all(decisions), last_sample, signal_sum

    def _validate_sensor_sample(self, sample, require_raw):
        if sample is None:
            raise MotorAutoTestError("尚未收到传感器数据")
        if sample.get("channel_count", 0) <= CONTACT_CHANNELS[-1]:
            raise MotorAutoTestError("传感器数据不足 8 个通道，无法读取 CH4～CH7")
        if sample.get("age_seconds", float("inf")) > self.config.sensor_frame_timeout_seconds:
            raise MotorAutoTestError("传感器数据已经停止更新")
        if require_raw and not sample.get("has_raw", False):
            raise MotorAutoTestError("寻找最大量程需要串口处于 Debug 输出模式")

    def _move_to_counts(self, target_counts):
        self._check_cancelled()
        start_counts = self.motor.read_position_counts()
        delta_counts = int(target_counts) - start_counts
        if abs(delta_counts) <= self.config.position_tolerance_counts:
            self._emit_progress(
                f"当前位置已在目标容差内：目标={int(target_counts)}，实际={start_counts}，"
                f"误差={start_counts - int(target_counts)} counts"
            )
            return start_counts
        self._send_relative_move_with_rejected_retry(start_counts, delta_counts)
        return self._wait_until_position(start_counts, int(target_counts))

    def _send_relative_move_with_rejected_retry(self, start_counts, delta_counts):
        self.motor.move_relative_counts(
            delta_counts,
            speed_rpm=self.speed_rpm,
            accel=self.acceleration,
            retry_cancelled=self.stop_event.is_set,
        )

    def _wait_until_position(self, start_counts, target_counts):
        deadline = time.monotonic() + self.config.motor_position_timeout_seconds
        requested_movement = abs(target_counts - start_counts)
        minimum_movement = max(10, round(requested_movement * 0.1))
        moved = requested_movement == 0
        stable_samples = 0
        last_counts = None
        current_counts = start_counts
        running_state = MOTOR_STATE_RUNNING
        while True:
            self._check_cancelled()
            running_state = self.motor.read_running_state()
            current_counts = self.motor.read_position_counts()
            self._emit_position(current_counts)
            if abs(current_counts - start_counts) >= minimum_movement:
                moved = True
            stable = (
                last_counts is not None
                and abs(current_counts - last_counts)
                <= self.config.position_tolerance_counts
            )
            motor_finished = running_state in MOTOR_FINISHED_STATES
            stable_samples = stable_samples + 1 if moved and motor_finished and stable else 0
            if stable_samples >= self.config.position_stable_samples:
                position_error = current_counts - target_counts
                self._emit_progress(
                    f"电机运动完成：起点={start_counts}，目标={target_counts}，"
                    f"实际={current_counts}，误差={position_error} counts，"
                    f"运行状态={running_state}"
                )
                return current_counts
            if running_state in MOTOR_FAULT_NAMES:
                raise MotorAutoTestError(
                    f"电机报告{MOTOR_FAULT_NAMES[running_state]}，"
                    f"起点={start_counts}，目标={target_counts}，实际={current_counts}"
                )
            if time.monotonic() >= deadline:
                raise MotorAutoTestError(
                    f"等待电机停止超时：起点={start_counts}，目标={target_counts}，"
                    f"实际={current_counts}，误差={current_counts - target_counts} counts，"
                    f"运行状态={running_state}"
                )
            last_counts = current_counts
            self.stop_event.wait(0.1)

    def _check_cancelled(self):
        if self.stop_event.is_set():
            raise MotorAutoTestCancelled("自动测试已停止")

    def _degrees_to_counts(self, degrees):
        return max(1, round(float(degrees) / 360.0 * self.motor.COUNTS_PER_TURN))

    def _counts_to_degrees(self, counts):
        return float(counts) / self.motor.COUNTS_PER_TURN * 360.0

    def _emit_progress(self, message):
        if self.progress_callback is not None:
            self.progress_callback(message)

    def _emit_position(self, counts):
        if self.position_callback is not None:
            self.position_callback(int(counts))


class RepeatabilityTestRunner:
    """以固定行程重复按压，并在达到模式对应的 FLOATING 条件后统计信号和。"""

    def __init__(
        self,
        tester: MotorAutoTester,
        output_dir: Path,
        rounds: int = 10,
        press_counts: int = 4096,
        floating_timeout_seconds: float = 20.0,
        recovery_mode: str = "rapid",
        idle_timeout_seconds: float = 45.0,
        idle_settle_seconds: float = 1.0,
        rapid_floating_channel_count: int = RAPID_REPEATABILITY_FLOATING_CHANNEL_MIN,
        precondition_relative_span_percent_max: float = REPEATABILITY_PRECONDITION_RELATIVE_SPAN_PERCENT_MAX,
        precondition_max_rounds: int = REPEATABILITY_PRECONDITION_MAX_ROUNDS,
        floating_hold_seconds: float = 2.0,
    ):
        self.tester = tester
        self.output_dir = Path(output_dir)
        self.rounds = int(rounds)
        self.press_counts = int(press_counts)
        self.floating_timeout_seconds = float(floating_timeout_seconds)
        self.recovery_mode = str(recovery_mode)
        self.idle_timeout_seconds = float(idle_timeout_seconds)
        self.idle_settle_seconds = float(idle_settle_seconds)
        self.rapid_floating_channel_count = int(rapid_floating_channel_count)
        self.precondition_relative_span_percent_max = float(
            precondition_relative_span_percent_max
        )
        self.precondition_max_rounds = int(precondition_max_rounds)
        self.floating_hold_seconds = float(floating_hold_seconds)
        self._last_sequence = None

    def run(self) -> dict:
        if self.rounds <= 0:
            raise MotorAutoTestError("重复精度测试次数必须大于零")
        if self.press_counts <= 0:
            raise MotorAutoTestError("重复精度测试下压counts必须大于零")
        if self.press_counts % 4096 != 0:
            raise MotorAutoTestError("重复精度测试下压counts必须是4096的整数倍")
        if self.recovery_mode not in ("rapid", "full"):
            raise MotorAutoTestError("未知的重复精度测试恢复模式")
        if not 1 <= self.rapid_floating_channel_count <= len(CONTACT_CHANNELS):
            raise MotorAutoTestError("快速测试FLOATING通道数必须在1～4之间")
        if self.precondition_relative_span_percent_max < 0.0:
            raise MotorAutoTestError("预压相对极差阈值不能小于0%")
        if self.precondition_max_rounds < REPEATABILITY_PRECONDITION_WINDOW:
            raise MotorAutoTestError(
                f"预压最大轮次不能少于稳定窗口的"
                f"{REPEATABILITY_PRECONDITION_WINDOW}轮"
            )
        if self.floating_timeout_seconds <= 0.0:
            raise MotorAutoTestError("等待FLOATING的单轮超时必须大于0秒")
        if self.floating_hold_seconds < 0.0:
            raise MotorAutoTestError("达到FLOATING后的保持时间不能小于0秒")

        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        result_path = self.output_dir / f"repeatability_{self.recovery_mode}_{stamp}.csv"
        origin_counts = self.tester.motor.read_position_counts()
        target_counts = origin_counts - self.press_counts
        self.tester.safe_return_counts = origin_counts
        results = []
        attempt_index = 0
        rejected_attempts = 0
        floating_timeout_attempts = 0
        maximum_attempts = self.rounds * REPEATABILITY_MAX_ATTEMPT_MULTIPLIER
        formal_relock_count = 0
        relocked_result_discards = 0
        precondition_runs = 0
        total_precondition_rounds = 0
        relock_candidate_counts = None
        relock_candidate_consecutive_attempts = 0
        require_exact_motion = self.recovery_mode == "rapid"
        precondition_result = {
            "rounds": 0,
            "valid_rounds": 0,
            "signal_sums": [],
            "relative_span_percent": 0.0,
        }

        self.tester._emit_progress(
            f"重复精度测试起点 {origin_counts} counts；模式={self._mode_name()}；"
            f"需要 {self.rounds} 次有效结果，每次下压 {self.press_counts} counts"
        )
        if require_exact_motion:
            self.tester._emit_progress(
                "快速连续模式在预压阶段跟踪±20 counts内的实际下压停点；"
                "同一停点连续形成稳定窗口后，正式轮次要求与该参考完全一致"
            )
        try:
            if require_exact_motion:
                precondition_result = self._run_rapid_preconditioning(
                    origin_counts,
                    target_counts,
                )
                precondition_runs += 1
                total_precondition_rounds += precondition_result["rounds"]
                origin_counts = precondition_result["return_reference_counts"]
                target_counts = precondition_result["pressed_reference_counts"]
                self.tester.safe_return_counts = origin_counts
            while len(results) < self.rounds:
                self.tester._check_cancelled()
                attempt_index += 1
                if attempt_index > maximum_attempts:
                    raise MotorAutoTestError(
                        f"重复精度测试已尝试 {maximum_attempts} 轮，"
                        f"仅获得 {len(results)} 次有效结果"
                    )
                round_index = len(results) + 1
                self.tester._emit_progress(
                    f"重复精度测试有效结果 {round_index}/{self.rounds}，"
                    f"尝试 {attempt_index}：正在下压 {self.press_counts} counts"
                )
                pressed_counts = self.tester._move_to_counts(target_counts)
                press_error_counts = pressed_counts - target_counts
                result = None
                floating_timed_out = False
                if not require_exact_motion or press_error_counts == 0:
                    sample = self._wait_for_floating_threshold(round_index)
                    if sample is None:
                        floating_timed_out = True
                        floating_timeout_attempts += 1
                    else:
                        signal_values = [
                            int(sample["signal"][channel])
                            for channel in CONTACT_CHANNELS
                        ]
                        signal_sum = sum(signal_values)
                        result = {
                            "round": round_index,
                            "attempt": attempt_index,
                            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                            "motor_origin_counts": origin_counts,
                            "motor_pressed_target_counts": target_counts,
                            "motor_pressed_counts": pressed_counts,
                            "press_error_counts": press_error_counts,
                            "press_counts": self.press_counts,
                            "recovery_mode": self.recovery_mode,
                            "sensor_sequence": int(sample["sequence"]),
                            "ch4_signal": signal_values[0],
                            "ch5_signal": signal_values[1],
                            "ch6_signal": signal_values[2],
                            "ch7_signal": signal_values[3],
                            "signal_sum": signal_sum,
                        }
                        self.tester._emit_progress(
                            f"尝试 {attempt_index} FLOATING：CH4～CH7={signal_values}，"
                            f"Signal 和={signal_sum}"
                        )
                        self._wait_after_floating(f"尝试 {attempt_index}")
                else:
                    self.tester._emit_progress(
                        f"尝试 {attempt_index} 下压误差={press_error_counts} counts，"
                        "不读取本轮结果，正在抬升后重试"
                    )
                self.tester._emit_progress(
                    f"尝试 {attempt_index}：正在抬升 {self.press_counts} counts"
                )
                returned_counts = self.tester._move_to_counts(origin_counts)
                return_error_counts = returned_counts - origin_counts
                if result is not None:
                    result["motor_returned_counts"] = returned_counts
                    result["return_error_counts"] = return_error_counts

                if floating_timed_out:
                    rejected_attempts += 1
                    relock_candidate_counts = None
                    relock_candidate_consecutive_attempts = 0
                    self.tester._emit_progress(
                        f"尝试 {attempt_index} 已因等待FLOATING超时而丢弃；"
                        f"电机已抬升，当前仍有 {len(results)}/{self.rounds} 次有效结果"
                    )
                    self._wait_for_next_round(f"尝试 {attempt_index}")
                    continue

                press_is_exact = press_error_counts == 0
                return_is_within_tolerance = (
                    abs(return_error_counts)
                    <= self.tester.config.position_tolerance_counts
                )
                if require_exact_motion and not (
                    press_is_exact and return_is_within_tolerance
                ):
                    rejected_attempts += 1
                    press_is_within_tolerance = (
                        abs(press_error_counts)
                        <= self.tester.config.position_tolerance_counts
                    )
                    can_relock_position = (
                        not press_is_exact
                        and press_is_within_tolerance
                        and return_is_within_tolerance
                    )
                    if can_relock_position:
                        if pressed_counts == relock_candidate_counts:
                            relock_candidate_consecutive_attempts += 1
                        else:
                            relock_candidate_counts = pressed_counts
                            relock_candidate_consecutive_attempts = 1
                    else:
                        relock_candidate_counts = None
                        relock_candidate_consecutive_attempts = 0
                    relock_candidate_text = ""
                    if relock_candidate_counts is not None:
                        relock_candidate_text = (
                            f"；重锁候选={relock_candidate_counts} counts，连续"
                            f"{relock_candidate_consecutive_attempts}/"
                            f"{REPEATABILITY_FORMAL_RELOCK_CONFIRMATION_ATTEMPTS}次"
                        )
                    self.tester._emit_progress(
                        f"尝试 {attempt_index} 已丢弃：下压误差={press_error_counts} counts，"
                        f"抬升误差={return_error_counts} counts"
                        "（正式采样要求下压误差为0；"
                        f"重锁候选容差±{self.tester.config.position_tolerance_counts}）；"
                        f"当前仍有 {len(results)}/{self.rounds} 次有效结果"
                        f"{relock_candidate_text}"
                    )
                    self._wait_for_next_round(f"尝试 {attempt_index}")
                    if (
                        relock_candidate_counts is not None
                        and relock_candidate_consecutive_attempts
                        >= REPEATABILITY_FORMAL_RELOCK_CONFIRMATION_ATTEMPTS
                    ):
                        if formal_relock_count >= REPEATABILITY_FORMAL_MAX_RELOCKS:
                            raise MotorAutoTestError(
                                f"正式测试已经重新锁定位置 {formal_relock_count} 次，"
                                f"下压停点仍持续变化；最后候选位置="
                                f"{relock_candidate_counts} counts"
                            )
                        formal_relock_count += 1
                        discarded_result_count = len(results)
                        relocked_result_discards += discarded_result_count
                        previous_target_counts = target_counts
                        candidate_target_counts = relock_candidate_counts
                        results.clear()
                        self.tester._emit_progress(
                            f"同一非零下压停点 {candidate_target_counts} counts 已连续出现"
                            f"{REPEATABILITY_FORMAL_RELOCK_CONFIRMATION_ATTEMPTS}次；"
                            f"正式参考将从 {previous_target_counts} counts 重新锁定。"
                            f"已废弃此前 {discarded_result_count} 个正式结果，"
                            "现在重新执行预压判稳"
                        )
                        precondition_result = self._run_rapid_preconditioning(
                            origin_counts,
                            candidate_target_counts,
                        )
                        precondition_runs += 1
                        total_precondition_rounds += precondition_result["rounds"]
                        origin_counts = precondition_result[
                            "return_reference_counts"
                        ]
                        target_counts = precondition_result[
                            "pressed_reference_counts"
                        ]
                        self.tester.safe_return_counts = origin_counts
                        relock_candidate_counts = None
                        relock_candidate_consecutive_attempts = 0
                        maximum_attempts = max(
                            maximum_attempts,
                            attempt_index
                            + self.rounds
                            * REPEATABILITY_MAX_ATTEMPT_MULTIPLIER,
                        )
                        self.tester._emit_progress(
                            f"正式位置第 {formal_relock_count} 次重锁完成："
                            f"回程参考={origin_counts} counts，"
                            f"下压参考={target_counts} counts；"
                            f"正式结果从 1/{self.rounds} 重新采集"
                        )
                    continue

                relock_candidate_counts = None
                relock_candidate_consecutive_attempts = 0
                results.append(result)
                self.tester._emit_progress(
                    f"有效结果 {len(results)}/{self.rounds} 已接收："
                    f"Signal 和={result['signal_sum']}，下压位置与可达参考完全一致，"
                    f"抬升误差={return_error_counts} counts"
                    if require_exact_motion
                    else f"第 {len(results)} 次结果已接收：Signal 和={result['signal_sum']}"
                )
                self._wait_for_next_round(len(results))
        finally:
            current_counts = self.tester.motor.read_position_counts()
            if abs(current_counts - origin_counts) > self.tester.config.position_tolerance_counts:
                self.tester._emit_progress("重复精度测试结束，正在返回测试起点")
                self.tester._move_to_counts(origin_counts)

        self._write_results(result_path, results)
        sums = [item["signal_sum"] for item in results]
        minimum = min(sums)
        maximum = max(sums)
        average = sum(sums) / len(sums)
        span = maximum - minimum
        relative_span_percent = (span / average * 100.0) if average != 0.0 else 0.0
        standard_deviation = (
            math.sqrt(
                sum((value - average) ** 2 for value in sums)
                / (len(sums) - 1)
            )
            if len(sums) > 1
            else 0.0
        )
        return {
            "kind": "repeatability",
            "rounds": len(results),
            "press_counts": self.press_counts,
            "recovery_mode": self.recovery_mode,
            "recovery_mode_name": self._mode_name(),
            "signal_sums": sums,
            "minimum": minimum,
            "maximum": maximum,
            "average": average,
            "span": span,
            "relative_span_percent": relative_span_percent,
            "standard_deviation": standard_deviation,
            "attempts": attempt_index,
            "rejected_attempts": rejected_attempts,
            "floating_timeout_attempts": floating_timeout_attempts,
            "formal_relock_count": formal_relock_count,
            "relocked_result_discards": relocked_result_discards,
            "precondition_runs": precondition_runs,
            "total_precondition_rounds": total_precondition_rounds,
            "precondition_rounds": precondition_result["rounds"],
            "precondition_valid_rounds": precondition_result["valid_rounds"],
            "precondition_signal_sums": precondition_result["signal_sums"],
            "precondition_relative_span_percent": precondition_result["relative_span_percent"],
            "pressed_reference_counts": precondition_result.get(
                "pressed_reference_counts",
                target_counts,
            ),
            "return_reference_counts": precondition_result.get(
                "return_reference_counts",
                origin_counts,
            ),
            "result_path": str(result_path),
        }

    def _run_rapid_preconditioning(self, origin_counts, target_counts):
        """预压材料直到最近三个零误差结果的相对极差满足稳定条件。"""

        valid_signal_sums = []
        latest_relative_span_percent = 0.0
        pressed_reference_counts = None
        return_reference_counts = None
        self.tester._emit_progress(
            f"开始快速连续测试预压：最多 {self.precondition_max_rounds} 轮；"
            f"最近 {REPEATABILITY_PRECONDITION_WINDOW} 个有效结果相对极差不超过 "
            f"{self.precondition_relative_span_percent_max:.3f}% 后进入正式测试"
        )

        for precondition_round in range(1, self.precondition_max_rounds + 1):
            self.tester._check_cancelled()
            self.tester._emit_progress(
                f"预压 {precondition_round}/{self.precondition_max_rounds}：正在下压"
            )
            requested_target_counts = (
                return_reference_counts - self.press_counts
                if return_reference_counts is not None
                else target_counts
            )
            pressed_counts = self.tester._move_to_counts(requested_target_counts)
            press_error_counts = pressed_counts - requested_target_counts
            signal_sum = None
            signal_values = None
            press_is_within_tolerance = (
                abs(press_error_counts)
                <= self.tester.config.position_tolerance_counts
            )
            if press_is_within_tolerance:
                sample = self._wait_for_floating_threshold(f"预压 {precondition_round}")
                if sample is not None:
                    signal_values = [
                        int(sample["signal"][channel])
                        for channel in CONTACT_CHANNELS
                    ]
                    signal_sum = sum(signal_values)
                    self.tester._emit_progress(
                        f"预压 {precondition_round} FLOATING：CH4～CH7={signal_values}，"
                        f"Signal 和={signal_sum}"
                    )
                    self._wait_after_floating(f"预压 {precondition_round}")
            else:
                self.tester._emit_progress(
                    f"预压 {precondition_round} 下压误差={press_error_counts} counts，"
                    f"超过容差±{self.tester.config.position_tolerance_counts}，"
                    "本轮不参与稳定判断"
                )

            self.tester._emit_progress(f"预压 {precondition_round}：正在抬升")
            requested_return_counts = (
                return_reference_counts
                if return_reference_counts is not None
                else origin_counts
            )
            returned_counts = self.tester._move_to_counts(requested_return_counts)
            initial_return_error_counts = returned_counts - origin_counts
            if return_reference_counts is None and (
                abs(initial_return_error_counts)
                <= self.tester.config.position_tolerance_counts
            ):
                return_reference_counts = returned_counts
                self.tester.safe_return_counts = return_reference_counts
                self.tester._emit_progress(
                    f"预压 {precondition_round} 已校准回程端点："
                    f"回程参考={return_reference_counts}，"
                    f"理论下压目标={return_reference_counts - self.press_counts}，"
                    f"输入行程={self.press_counts} counts；"
                    f"初始起点偏差={initial_return_error_counts} counts"
                )

            return_error_counts = (
                returned_counts - return_reference_counts
                if return_reference_counts is not None
                else initial_return_error_counts
            )
            return_is_within_tolerance = (
                abs(return_error_counts)
                <= self.tester.config.position_tolerance_counts
            )
            if press_is_within_tolerance and return_is_within_tolerance and signal_sum is not None:
                if pressed_counts != pressed_reference_counts:
                    pressed_reference_counts = pressed_counts
                    valid_signal_sums.clear()
                    latest_relative_span_percent = 0.0
                    self.tester._emit_progress(
                        f"预压 {precondition_round} 更新实际可达下压停点="
                        f"{pressed_reference_counts} counts；"
                        "停点发生变化，重新累计最近3次稳定窗口"
                    )
                valid_signal_sums.append(signal_sum)
            else:
                self.tester._emit_progress(
                    f"预压 {precondition_round} 不参与稳定判断："
                    f"下压误差={press_error_counts} counts，"
                    f"抬升误差={return_error_counts} counts"
                )

            self._wait_for_next_round(f"预压 {precondition_round}")
            if len(valid_signal_sums) < REPEATABILITY_PRECONDITION_WINDOW:
                self.tester._emit_progress(
                    f"预压稳定窗口尚未填满：当前下压停点="
                    f"{pressed_reference_counts}，同停点有效结果 "
                    f"{len(valid_signal_sums)}/{REPEATABILITY_PRECONDITION_WINDOW}"
                )
                continue

            recent_sums = valid_signal_sums[-REPEATABILITY_PRECONDITION_WINDOW:]
            recent_average = sum(recent_sums) / len(recent_sums)
            recent_span = max(recent_sums) - min(recent_sums)
            latest_relative_span_percent = (
                recent_span / recent_average * 100.0
                if recent_average != 0.0
                else 0.0
            )
            self.tester._emit_progress(
                f"预压 {precondition_round} 稳定判断：最近结果={recent_sums}，"
                f"相对极差={latest_relative_span_percent:.3f}%"
            )
            if (
                latest_relative_span_percent
                <= self.precondition_relative_span_percent_max
            ):
                self.tester._emit_progress(
                    f"预压稳定条件已满足，共执行 {precondition_round} 轮；"
                    f"现在开始采集{self.rounds}次正式有效结果"
                )
                return {
                    "rounds": precondition_round,
                    "valid_rounds": len(valid_signal_sums),
                    "signal_sums": valid_signal_sums,
                    "relative_span_percent": latest_relative_span_percent,
                    "pressed_reference_counts": pressed_reference_counts,
                    "return_reference_counts": return_reference_counts,
                }

        if len(valid_signal_sums) < REPEATABILITY_PRECONDITION_WINDOW:
            raise MotorAutoTestError(
                f"完成 {self.precondition_max_rounds} 轮预压后，"
                f"只有 {len(valid_signal_sums)} 个有效位置样本，"
                f"不足 {REPEATABILITY_PRECONDITION_WINDOW} 个，无法计算稳定性；"
                f"理论目标={target_counts}，下压参考={pressed_reference_counts}，"
                f"回程参考={return_reference_counts}"
            )
        raise MotorAutoTestError(
            f"完成 {self.precondition_max_rounds} 轮预压后仍未稳定："
            f"最近 {REPEATABILITY_PRECONDITION_WINDOW} 个有效结果相对极差="
            f"{latest_relative_span_percent:.3f}%，要求不超过 "
            f"{self.precondition_relative_span_percent_max:.3f}%"
        )

    def _required_floating_channel_count(self):
        if self.recovery_mode == "rapid":
            return self.rapid_floating_channel_count
        return len(CONTACT_CHANNELS)

    def _wait_for_floating_threshold(self, round_index):
        deadline = time.monotonic() + self.floating_timeout_seconds
        required_count = self._required_floating_channel_count()
        confirmed_frames = 0
        latest_sample = None
        while time.monotonic() < deadline:
            self.tester._check_cancelled()
            sample = self.tester.sensor_sample_provider()
            self.tester._validate_sensor_sample(sample, require_raw=True)
            if sample["sequence"] != self._last_sequence:
                self._last_sequence = sample["sequence"]
                latest_sample = sample
                states = [sample["state"][channel] for channel in CONTACT_CHANNELS]
                floating_count = sum(
                    state == CHANNEL_STATE_FLOATING for state in states
                )
                if floating_count >= required_count:
                    confirmed_frames += 1
                    if confirmed_frames >= self.tester.config.confirmation_frames:
                        return latest_sample
                else:
                    confirmed_frames = 0
            self.tester.stop_event.wait(self.tester.config.poll_interval_seconds)

        state_text = "无有效数据"
        if latest_sample is not None:
            state_text = ", ".join(
                f"CH{channel}={CHANNEL_STATE_NAMES.get(latest_sample['state'][channel], latest_sample['state'][channel])}"
                for channel in CONTACT_CHANNELS
            )
        round_label = (
            f"第 {round_index} 次"
            if isinstance(round_index, int)
            else str(round_index)
        )
        self.tester._emit_progress(
            f"{round_label}等待 CH4～CH7 中至少 {required_count} 个通道"
            f"进入 FLOATING 已超过 {self.floating_timeout_seconds:g} 秒：{state_text}；"
            "本轮不记录结果，立即抬升后继续下一轮"
        )
        return None

    def _wait_after_floating(self, round_index):
        if self.floating_hold_seconds <= 0.0:
            return
        self.tester._emit_progress(
            f"{round_index} 已达到FLOATING条件，保持 "
            f"{self.floating_hold_seconds:g} 秒后抬升"
        )
        if self.tester.stop_event.wait(self.floating_hold_seconds):
            self.tester._check_cancelled()

    def _wait_for_next_round(self, round_index):
        if self.recovery_mode == "rapid":
            if isinstance(round_index, int):
                round_label = f"第 {round_index} 次"
            else:
                round_label = str(round_index)
            self.tester._emit_progress(
                f"{round_label}已回到起点，短暂等待后继续快速连续测试"
            )
            self.tester.stop_event.wait(self.tester.config.sensor_settle_seconds)
            return

        self.tester._emit_progress(
            f"第 {round_index} 次已回到起点，等待 CH4～CH7 全部进入 IDLE/WAIT/RECOVER"
        )
        deadline = time.monotonic() + self.idle_timeout_seconds
        confirmed_frames = 0
        latest_states = None
        while time.monotonic() < deadline:
            self.tester._check_cancelled()
            sample = self.tester.sensor_sample_provider()
            self.tester._validate_sensor_sample(sample, require_raw=True)
            if sample["sequence"] != self._last_sequence:
                self._last_sequence = sample["sequence"]
                latest_states = [sample["state"][channel] for channel in CONTACT_CHANNELS]
                if all(state in IDLE_LIKE_STATES for state in latest_states):
                    confirmed_frames += 1
                    if confirmed_frames >= self.tester.config.confirmation_frames:
                        self.tester._emit_progress(
                            f"第 {round_index} 次 CH4～CH7 已全部进入 IDLE/WAIT/RECOVER，再稳定等待 {self.idle_settle_seconds:g} 秒"
                        )
                        if self.tester.stop_event.wait(self.idle_settle_seconds):
                            self.tester._check_cancelled()
                        return
                else:
                    confirmed_frames = 0
            self.tester.stop_event.wait(self.tester.config.poll_interval_seconds)

        state_text = "无有效数据"
        if latest_states is not None:
            state_text = ", ".join(
                f"CH{channel}={CHANNEL_STATE_NAMES.get(state, state)}"
                for channel, state in zip(CONTACT_CHANNELS, latest_states)
            )
        raise MotorAutoTestError(
            f"第 {round_index} 次等待 CH4～CH7 全部恢复 IDLE/WAIT/RECOVER 超时：{state_text}"
        )

    def _mode_name(self):
        return "快速连续" if self.recovery_mode == "rapid" else "完全恢复"

    @staticmethod
    def _write_results(result_path, results):
        fieldnames = [
            "round",
            "attempt",
            "timestamp",
            "motor_origin_counts",
            "motor_pressed_target_counts",
            "motor_pressed_counts",
            "press_error_counts",
            "motor_returned_counts",
            "return_error_counts",
            "press_counts",
            "recovery_mode",
            "sensor_sequence",
            "ch4_signal",
            "ch5_signal",
            "ch6_signal",
            "ch7_signal",
            "signal_sum",
        ]
        with result_path.open("w", encoding="utf-8-sig", newline="") as result_file:
            writer = csv.DictWriter(result_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)


class RandomPressTestRunner:
    """执行随机角度按压，并把每一帧传感器数据和每轮结论写入 CSV。"""

    def __init__(
        self,
        tester: MotorAutoTester,
        zero_position_counts: int,
        maximum_angle_deg: float,
        output_dir: Path,
        rounds: int = 10,
        minimum_angle_deg: int = 10,
        hold_min_seconds: float = 1.0,
        hold_max_seconds: float = 5.0,
        release_observe_seconds: float = 15.0,
        random_seed: Optional[int] = None,
    ):
        self.tester = tester
        self.zero_position_counts = int(zero_position_counts)
        self.maximum_angle_deg = float(maximum_angle_deg)
        self.output_dir = Path(output_dir)
        self.rounds = int(rounds)
        self.minimum_angle_deg = int(minimum_angle_deg)
        self.hold_min_seconds = float(hold_min_seconds)
        self.hold_max_seconds = float(hold_max_seconds)
        self.release_observe_seconds = float(release_observe_seconds)
        self.random_seed = int(random_seed if random_seed is not None else time.time_ns())
        self._rng = random.Random(self.random_seed)
        self._started_monotonic = None
        self._detail_writer = None
        self._detail_file = None
        self._last_sequence = None
        self._round_states = None
        self._round_state_sequences = None
        self._slow_back_nonzero_frames = 0
        self._slow_back_frames = 0

    def run(self) -> dict:
        if self.maximum_angle_deg < self.minimum_angle_deg:
            raise MotorAutoTestError("最大测试角度小于最小随机角度，无法开始自动测试")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        detail_path = self.output_dir / f"random_press_detail_{stamp}.csv"
        summary_path = self.output_dir / f"random_press_summary_{stamp}.csv"
        self._started_monotonic = time.monotonic()
        summaries = []
        self.tester.safe_return_counts = self.zero_position_counts

        try:
            self._detail_file = detail_path.open("w", encoding="utf-8-sig", newline="")
            self._detail_writer = csv.DictWriter(
                self._detail_file,
                fieldnames=self._detail_field_names(),
            )
            self._detail_writer.writeheader()
            self._move_and_capture(self.zero_position_counts, 0, 0.0, "回到测试零点")

            for round_index in range(1, self.rounds + 1):
                self.tester._check_cancelled()
                angle_deg = self._rng.randint(
                    self.minimum_angle_deg,
                    max(self.minimum_angle_deg, int(self.maximum_angle_deg)),
                )
                hold_seconds = self._rng.uniform(
                    self.hold_min_seconds,
                    self.hold_max_seconds,
                )
                target_counts = self.zero_position_counts - self.tester._degrees_to_counts(angle_deg)
                self._round_states = [set() for _ in CONTACT_CHANNELS]
                self._round_state_sequences = [[] for _ in CONTACT_CHANNELS]
                self._slow_back_nonzero_frames = 0
                self._slow_back_frames = 0
                self.tester._emit_progress(
                    f"随机测试 {round_index}/{self.rounds}：下压 "
                    f"{self.zero_position_counts - target_counts} counts，"
                    f"保持 {hold_seconds:.2f}s"
                )
                self._move_and_capture(target_counts, round_index, angle_deg, "下压")
                self._capture_for(hold_seconds, round_index, angle_deg, "保持")
                self._move_and_capture(self.zero_position_counts, round_index, angle_deg, "回零")
                returned_idle = self._observe_release(round_index, angle_deg)
                summaries.append(
                    self._make_round_summary(
                        round_index,
                        angle_deg,
                        hold_seconds,
                        returned_idle,
                    )
                )
                self.tester._emit_progress(
                    f"随机测试 {round_index}/{self.rounds} 完成："
                    f"SLOW_BACK 非零帧 {self._slow_back_nonzero_frames}/"
                    f"{self._slow_back_frames}"
                )
        finally:
            if self._detail_file is not None:
                self._detail_file.flush()
                self._detail_file.close()
                self._detail_file = None

        self._write_summary(summary_path, summaries)
        return {
            "kind": "random_press",
            "detail_path": str(detail_path),
            "summary_path": str(summary_path),
            "rounds": len(summaries),
            "random_seed": self.random_seed,
            "slow_back_failures": sum(
                1 for item in summaries if item["slow_back_signal_zero"] == "FAIL"
            ),
            "state_failures": sum(
                1 for item in summaries if item["state_sequence"] == "FAIL"
            ),
        }

    def _move_and_capture(self, target_counts, round_index, angle_deg, phase):
        completed = threading.Event()
        result = {}

        def move():
            try:
                result["counts"] = self.tester._move_to_counts(target_counts)
            except Exception as exc:
                result["error"] = exc
            finally:
                completed.set()

        thread = threading.Thread(target=move, name="random-test-move", daemon=True)
        thread.start()
        while not completed.wait(self.tester.config.poll_interval_seconds):
            if not self.tester.stop_event.is_set():
                self._capture_latest(round_index, angle_deg, phase)
        thread.join()
        if "error" in result:
            raise result["error"]
        self._capture_latest(round_index, angle_deg, phase, motor_counts=result.get("counts"))
        return result.get("counts")

    def _capture_for(self, duration, round_index, angle_deg, phase):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.tester._check_cancelled()
            self._capture_latest(round_index, angle_deg, phase)
            self.tester.stop_event.wait(self.tester.config.poll_interval_seconds)

    def _observe_release(self, round_index, angle_deg):
        deadline = time.monotonic() + self.release_observe_seconds
        saw_non_idle = False
        idle_frames = 0
        while time.monotonic() < deadline:
            self.tester._check_cancelled()
            sample = self._capture_latest(round_index, angle_deg, "释放观察")
            if sample is not None:
                states = [sample["state"][channel] for channel in CONTACT_CHANNELS]
                saw_non_idle = saw_non_idle or any(state not in IDLE_LIKE_STATES for state in states)
                idle_frames = idle_frames + 1 if all(state in IDLE_LIKE_STATES for state in states) else 0
                if saw_non_idle and idle_frames >= self.tester.config.confirmation_frames:
                    return True
            self.tester.stop_event.wait(self.tester.config.poll_interval_seconds)
        return False

    def _capture_latest(self, round_index, angle_deg, phase, motor_counts=None):
        sample = self.tester.sensor_sample_provider()
        self.tester._validate_sensor_sample(sample, require_raw=True)
        if sample["sequence"] == self._last_sequence:
            return None
        self._last_sequence = sample["sequence"]
        if motor_counts is None:
            motor_counts = ""
        row = {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "elapsed_s": round(time.monotonic() - self._started_monotonic, 6),
            "random_seed": self.random_seed,
            "round": round_index,
            "target_angle_deg": angle_deg,
            "phase": phase,
            "sensor_sequence": sample["sequence"],
            "motor_position_counts": motor_counts,
        }
        for channel in range(sample["channel_count"]):
            row[f"ch{channel}_raw"] = sample["raw"][channel]
            row[f"ch{channel}_baseline"] = sample["baseline"][channel]
            row[f"ch{channel}_signal"] = sample["signal"][channel]
            row[f"ch{channel}_state"] = sample["state"][channel]
            row[f"ch{channel}_state_name"] = CHANNEL_STATE_NAMES.get(
                sample["state"][channel], str(sample["state"][channel])
            )
            row[f"ch{channel}_fast_k_percent"] = sample["fast_k_percent"][channel]
            row[f"ch{channel}_slow_k_percent"] = sample["slow_k_percent"][channel]
        self._detail_writer.writerow(row)
        self._update_round_analysis(sample)
        return sample

    def _update_round_analysis(self, sample):
        if self._round_states is None:
            return
        for index, channel in enumerate(CONTACT_CHANNELS):
            state = sample["state"][channel]
            self._round_states[index].add(state)
            sequence = self._round_state_sequences[index]
            if not sequence or sequence[-1] != state:
                sequence.append(state)
            if state == CHANNEL_STATE_SLOW_BACK:
                self._slow_back_frames += 1
                if sample["signal"][channel] != 0:
                    self._slow_back_nonzero_frames += 1

    def _make_round_summary(self, round_index, angle_deg, hold_seconds, returned_idle):
        required = {CHANNEL_STATE_RISING, CHANNEL_STATE_FALLING, CHANNEL_STATE_SLOW_BACK}
        state_ok = all(
            required.issubset(states)
            and self._contains_ordered_states(
                sequence,
                (CHANNEL_STATE_RISING, CHANNEL_STATE_FALLING, CHANNEL_STATE_SLOW_BACK),
            )
            for states, sequence in zip(self._round_states, self._round_state_sequences)
        )
        return {
            "round": round_index,
            "target_angle_deg": angle_deg,
            "hold_seconds": round(hold_seconds, 6),
            "state_sequence": "PASS" if state_ok else "FAIL",
            "returned_idle": "PASS" if returned_idle else "TIMEOUT",
            "slow_back_frames": self._slow_back_frames,
            "slow_back_nonzero_signal_frames": self._slow_back_nonzero_frames,
            "slow_back_signal_zero": "PASS" if self._slow_back_frames > 0 and self._slow_back_nonzero_frames == 0 else "FAIL",
            "states_by_channel": " | ".join(
                f"CH{channel}:" + "->".join(
                    CHANNEL_STATE_NAMES.get(state, str(state)) for state in sequence
                )
                for channel, sequence in zip(CONTACT_CHANNELS, self._round_state_sequences)
            ),
        }

    @staticmethod
    def _contains_ordered_states(sequence, required):
        required_index = 0
        for state in sequence:
            if state == required[required_index]:
                required_index += 1
                if required_index == len(required):
                    return True
        return False

    @staticmethod
    def _write_summary(path, summaries):
        if not summaries:
            return
        with path.open("w", encoding="utf-8-sig", newline="") as target:
            writer = csv.DictWriter(target, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)

    @staticmethod
    def _detail_field_names():
        fields = [
            "timestamp", "elapsed_s", "random_seed", "round", "target_angle_deg",
            "phase", "sensor_sequence", "motor_position_counts",
        ]
        for channel in range(8):
            fields.extend(
                [
                    f"ch{channel}_raw", f"ch{channel}_baseline", f"ch{channel}_signal",
                    f"ch{channel}_state", f"ch{channel}_state_name",
                    f"ch{channel}_fast_k_percent", f"ch{channel}_slow_k_percent",
                ]
            )
        return fields
