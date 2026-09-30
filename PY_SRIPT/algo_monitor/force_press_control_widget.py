"""按压力－位置查找表执行一次性下压、释放、安全监控与数据记录。"""

from __future__ import annotations

import csv
import io
import json
import math
import os
import queue
import statistics
import sys
import threading
import time
from rebound_test_protocol import (
    REBOUND_PROTOCOL, REBOUND_ZERO_TOLERANCE_N, REBOUND_ZERO_WAIT_SECONDS,
    REBOUND_POSITION_TOLERANCE_COUNTS, near_rebound_zero,
    REBOUND_MOTION_CONFIRM_SAMPLES, REBOUND_MOTION_CONFIRM_SECONDS,
)
from datetime import datetime
from pathlib import Path

from channel_states import (
    STATE_IDLE as SENSOR_STATE_IDLE,
    STATE_FLOATING as SENSOR_STATE_FLOATING,
    RELEASE_READY_STATES, STATE_NAMES,
)
from low_force_calibration import low_force_display
from motion_log_sampling import MotionLogSampling

from creep_test_safety import (
    CreepDeadlineEvent, RECOVERABLE_CREEP_ERRORS, CREEP_MAX_ATTEMPTS,
    CREEP_MAX_CONSECUTIVE_FAILURES, CREEP_PRESS_TIMEOUT_SECONDS,
    CREEP_RELEASE_TIMEOUT_SECONDS, CREEP_RECOVERY_SECONDS,
    prevent_idle_sleep,
)
from force_position_lookup import (
    ForcePositionLookupError,
    MAP_FORCE_STEP_N,
    load_latest_mapping_document,
    resolve_force_position_target_n,
    resolve_mapping_release_reference,
)
from force_signal_pressure_calibration import (
    FORCE_CALIBRATION_TARGETS_N,
    QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT,
    QUICK_CORRECTION_TARGETS_N,
)
from pressure_meter_serial_widget import (
    GF_TO_NEWTON,
    PRESSURE_METER_START_ZERO_TOLERANCE_N,
    pressure_n_is_within_start_zero_tolerance,
)
from qt_compat import QtCore, QtWidgets, Signal, Slot
from step_moto_can_driver import PD42S1CommunicationError, PD42S1Motor
from wrapped_label import WrappedLabel


POSITION_COUNT_UNIT = 4096
MOTOR_UI_MAX_RPM = 400
# 目标测试按钮与快速修正共用覆盖0.1～30 N的十九个代表档位。
TARGET_FORCES_N = QUICK_CORRECTION_TARGETS_N
WARMUP_FORCE_N = 10.0
WARMUP_ROUNDS = 5
RELEASE_BUTTON_IDLE_TEXT = "释放 / 回到标定出发点"
RELEASE_BUTTON_RUNNING_TEXT = "电机仍在运行"
TARGET_TOLERANCE_N = 0.10
# 高压力档按2%判断“明显低于目标”，低压力档仍至少保留0.10 N容差。
PRESS_POSITION_CORRECTION_FORCE_TOLERANCE_RATIO = 0.02
# 压力计零点附近的小负漂移不构成运动风险，连续越过该独立下限才停止。
PRESSURE_METER_NEGATIVE_LIMIT_N = 0.10
PRESSURE_METER_NEGATIVE_CONFIRMATION_COUNT = 3
STABLE_SAMPLE_COUNT = 3
STABLE_SAMPLE_SPAN_N = 0.10
PRESSURE_FRESH_TIMEOUT_SECONDS = 1.0
PRESSURE_STABILITY_TIMEOUT_SECONDS = 5.0
CONTROL_SUMMARY_REFRESH_SECONDS = 0.6
MAX_OPERATION_FORCE_N = 30.40
MAX_SWEEP_FORCE_N = 30.0
MAX_OPERATION_TRAVEL_COUNTS = 12 * POSITION_COUNT_UNIT
MAX_PRESS_COMMAND_COUNTS = 4096
AUTO_FLOATING_STABLE_SECONDS = 1.0
AUTO_FLOATING_MIN_FRAMES = 3
AUTO_FLOATING_TIMEOUT_SECONDS = 20.0
RELEASE_ZERO_CONFIRM_SECONDS = 1.0
RELEASE_ZERO_MIN_SAMPLES = 3
RELEASE_ZERO_SAMPLE_MAX_GAP_SECONDS = 0.25
AUTO_SENSOR_MAX_AGE_SECONDS = 0.5
AUTO_SENSOR_POLL_SECONDS = 0.05
SWEEP_MOTION_TIMEOUT_MAX_RETRIES = 3
SWEEP_START_ZERO_RETRY_SECONDS = 10.0
MIN_STEP_COUNTS = 64
MOTOR_MAX_TORQUE_CURRENT_MA = 3000
STARTUP_CURRENT_SAMPLE_COUNT = 3
CURRENT_LIMIT_CONFIRMATION_COUNT = 3
CURRENT_LIMIT_IMMEDIATE_RATIO = 1.5
CURRENT_LIMIT_IMMEDIATE_MARGIN_MA = 200
MOTOR_STATE_CLEAR_DELAY_SECONDS = 0.08
MOTOR_COMMUNICATION_RETRY_DELAY_SECONDS = 0.25
MOTOR_COMMUNICATION_RETRY_MAX_DELAY_SECONDS = 1.0
MOTION_POSITION_TOLERANCE_COUNTS = 50
# 查表下压的一次位置修正恢复使用约 ±50 counts 的允许误差。
PRESS_POSITION_CORRECTION_TOLERANCE_COUNTS = 50
# 按10 N对应约128 counts线性放大一次修正的准入范围，高压力允许更大的受载停点误差。
PRESS_POSITION_CORRECTION_COUNTS_PER_N = 12.8
RELEASE_POSITION_TOLERANCE_COUNTS = 256
SWEEP_SEQUENCE_LEGACY = "ascending_then_descending"
SWEEP_SEQUENCE_ALTERNATING = "alternating_bidirectional"
MOTION_STABLE_DELTA_COUNTS = 10
MOTION_STABLE_SAMPLE_COUNT = 3
MOTION_STOP_SPEED_RPM = 1
# 状态2但任务线程已结束时，仅在位置差分也接近静止后识别为受力保持。
DRIVER_HOLDING_MAX_POSITION_SPEED_COUNTS_S = 100.0
SENSOR_CHANNELS = (4, 5, 6, 7)
SENSOR_RELEASE_RAW_SPAN_COUNTS = 8.0
SENSOR_RECORD_POLL_INTERVAL_MS = 10
MOVEMENT_LOG_MAX_BYTES = 5 * 1024 * 1024
MOVEMENT_LOG_MAX_FILES = 10
MOVEMENT_LOG_QUEUE_MAX_ROWS = 20000
MOVEMENT_LOG_FLUSH_INTERVAL_SECONDS = 0.5
MOVEMENT_LOG_FLUSH_MAX_ROWS = 128
MOVEMENT_LOG_STOP_TIMEOUT_SECONDS = 5.0
SENSOR_VECTOR_FIELDS = (
    "hardware_raw",
    "baseline",
    "signal",
    "fast_k_percent",
    "slow_k_percent",
    "state",
)
SENSOR_LOG_FIELD_NAMES = (
    "sensor_sequence",
    "sensor_sequence_gap",
    "sensor_age_ms",
    "sensor_channel_count",
    "sensor_has_raw",
    "sensor_four_channel_valid",
) + tuple(
    f"ch{channel}_{field_name}"
    for channel in SENSOR_CHANNELS
    for field_name in SENSOR_VECTOR_FIELDS
) + (
    "ch4567_hardware_raw_sum",
    "ch4567_mcu_hardware_raw_mean",
    "ch4567_mcu_hardware_raw_stddev",
    "ch4567_mcu_signal_sum",
    "raw_response_reference_id",
    "raw_response_status",
    "ch4567_delta_raw_mean",
    "ch4567_delta_raw_stddev",
    "ch4567_delta_raw_cv_percent",
    "ch4567_delta_raw_minimum_mean",
    "ch4_reference_hardware_raw", "ch5_reference_hardware_raw", "ch6_reference_hardware_raw", "ch7_reference_hardware_raw",
    "ch4_delta_hardware_raw", "ch5_delta_hardware_raw", "ch6_delta_hardware_raw", "ch7_delta_hardware_raw",
    "ch4_delta_share_percent", "ch5_delta_share_percent", "ch6_delta_share_percent", "ch7_delta_share_percent",
    "ch4567_baseline_sum",
    "ch4567_signal_sum",
)
AUTO_RAW_MEDIAN_FIELD_NAMES = (
    "raw_median_window_seconds",
    "raw_median_sample_count",
) + tuple(
    f"ch{channel}_raw_1s_median" for channel in SENSOR_CHANNELS
) + (
    "ch4567_raw_1s_median_sum",
)
AUTO_RESULT_FIELD_NAMES = (
    "motor_model", "position_unit", "steps_per_turn", "speed_unit", "acceleration_unit", "control_strategy",
    "captured_at",
    "operation_id",
    "control_mode",
    "force_unit",
    "target_force_n",
    "lookup_created_at",
    "release_reference_position_counts",
    "lookup_target_position_counts",
    "press_arrival_position_counts",
    "press_arrival_position_error_counts",
    "press_position_correction_counts",
    "press_position_error_before_correction_counts",
    "press_position_error_after_correction_counts",
    "floating_capture_position_counts",
    "floating_position_error_counts",
    "floating_running_state",
    "press_speed_level",
    "release_speed_level",
    "acceleration_r_s2",
    "max_current_ma",
    "pressure_level_index",
    "pressure_level_total",
    "calibration_cycle_index",
    "calibration_cycle_total",
    "sweep_direction",
    "direction_step_index",
    "direction_step_total",
    "repeat_index",
    "repeat_total",
    "attempt_index",
    "auto_round_index",
    "auto_round_total",
    "press_arrival_force_n",
    "final_force_n",
    "release_force_n",
    "release_idle_recovered",
    "release_idle_wait_seconds",
    "release_idle_stable_seconds",
    "release_idle_timeout_seconds",
) + tuple(
    f"ch{channel}_signal" for channel in SENSOR_CHANNELS
) + (
    "ch4567_signal_sum",
    "floating_count",
    "raw_median_window_seconds",
    "raw_median_sample_count",
) + tuple(
    f"ch{channel}_raw_1s_median" for channel in SENSOR_CHANNELS
) + (
    "ch4567_raw_1s_median_sum",
)

PRESSURE_N_CSV_FIELDS = (
    "target_force_n",
    "press_arrival_force_n",
    "final_force_n",
    "release_force_n",
    "raw_pressure_n",
    "filtered_force_n",
    "approach_pressure_threshold_n",
)


MOTOR_RUNNING_STATE_NAMES = {
    0: "停止",
    1: "任务完成",
    2: "正在运行",
    3: "过载",
    4: "堵转",
    5: "欠压",
}


def label_soft_wrap_text(value):
    """为QLabel纯文本加入不可见断行点，避免连续中文横向截断。"""
    return "\u200b".join(str(value))


def speed_level_to_counts_per_second(speed_level):
    return int(speed_level) * POSITION_COUNT_UNIT


def counts_per_second_to_rpm(speed_counts_per_second):
    rpm = round(
        int(speed_counts_per_second) * 60.0 / PD42S1Motor.COUNTS_PER_TURN
    )
    return max(1, min(PD42S1Motor.MAX_SPEED_RPM, rpm))


def maximum_speed_level():
    counts_per_second = round(
        MOTOR_UI_MAX_RPM * PD42S1Motor.COUNTS_PER_TURN / 60.0
    )
    return max(1, round(counts_per_second / POSITION_COUNT_UNIT))


def pressure_n_is_within_press_start_tolerance(value):
    """允许零点小幅负漂移，同时继续阻止正压力带载启动。"""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(value)
        and -PRESSURE_METER_NEGATIVE_LIMIT_N <= value
        <= PRESSURE_METER_START_ZERO_TOLERANCE_N
    )


def update_negative_pressure_confirmation(
    tracker,
    accepted_force_n,
    accepted_sequence,
):
    """只按新的压力样本累计连续负压力次数，重复轮询同一帧不重复计数。"""
    accepted_sequence = int(accepted_sequence)
    if accepted_sequence == tracker.get("negative_pressure_sequence"):
        return int(tracker.get("negative_pressure_samples", 0))
    tracker["negative_pressure_sequence"] = accepted_sequence
    if float(accepted_force_n) < -PRESSURE_METER_NEGATIVE_LIMIT_N:
        tracker["negative_pressure_samples"] = int(
            tracker.get("negative_pressure_samples", 0)
        ) + 1
    else:
        tracker["negative_pressure_samples"] = 0
    return int(tracker["negative_pressure_samples"])


def press_position_correction_force_tolerance_n(target_force_n):
    """按目标压力比例计算是否需要补位置的压力容差。"""
    return max(
        TARGET_TOLERANCE_N,
        abs(float(target_force_n))
        * PRESS_POSITION_CORRECTION_FORCE_TOLERANCE_RATIO,
    )


def press_position_correction_required(
    final_force_n,
    target_force_n,
    position_error_counts,
):
    """仅在压力明显不足且下压位置确实少走时请求一次位置修正。"""
    force_tolerance_n = press_position_correction_force_tolerance_n(
        target_force_n
    )
    pressure_clearly_low = (
        float(final_force_n) < float(target_force_n) - force_tolerance_n
    )
    position_clearly_short = (
        int(position_error_counts) > PRESS_POSITION_CORRECTION_TOLERANCE_COUNTS
    )
    return pressure_clearly_low and position_clearly_short


def press_position_correction_max_counts(target_force_n):
    """按目标压力线性计算一次位置修正的最大准入误差。"""
    proportional_counts = round(
        abs(float(target_force_n)) * PRESS_POSITION_CORRECTION_COUNTS_PER_N
    )
    return max(
        MOTION_POSITION_TOLERANCE_COUNTS,
        proportional_counts,
    )


def driver_feedback_is_settled_holding(feedback):
    """识别驱动器状态2下已经停止转动、仅维持负载的位置保持状态。"""
    try:
        running_state = int(feedback.get("running_state"))
        speed_rpm = abs(float(feedback.get("measured_speed_rpm")))
        position_speed = feedback.get("position_speed_counts_s")
        if position_speed is None:
            return False
        position_speed = abs(float(position_speed))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    return (
        running_state == 2
        and math.isfinite(speed_rpm)
        and math.isfinite(position_speed)
        and speed_rpm <= MOTION_STOP_SPEED_RPM
        and position_speed <= DRIVER_HOLDING_MAX_POSITION_SPEED_COUNTS_S
    )


def sweep_direction_plans(targets, cycle_index, sequence_mode):
    """按标定轮次生成升降方向计划，并兼容旧版固定升降顺序。"""
    ascending = ("ascending", "升序", tuple(targets))
    descending = ("descending", "降序", tuple(reversed(targets)))
    if sequence_mode == SWEEP_SEQUENCE_LEGACY:
        return ascending, descending
    if sequence_mode != SWEEP_SEQUENCE_ALTERNATING:
        raise ValueError(f"未知的传感器标定顺序：{sequence_mode}")
    if int(cycle_index) % 2:
        return ascending, descending
    return descending, ascending


def sweep_sequence_description(sequence_mode):
    """返回便于界面和日志显示的双向标定顺序说明。"""
    if sequence_mode == SWEEP_SEQUENCE_ALTERNATING:
        return "奇数轮升序→降序，偶数轮降序→升序"
    return "每轮升序→降序"


def force_press_test_directory():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "force_press_tests"
    return Path(__file__).resolve().with_name("force_press_tests")


def force_press_settings_path():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "force_press_control_config.json"
    return Path(__file__).resolve().with_name("force_press_control_config.json")


def force_sweep_checkpoint_path():
    return force_press_test_directory() / "force_sweep_checkpoint_latest.json"


DEFAULT_FORCE_PRESS_SETTINGS = {
    "press_speed_level": 2,
    "contact_before_step_counts": 1024,
    "contact_after_step_counts": 256,
    "approach_step_counts": 64,
    "approach_pressure_threshold_n": 0.30,
    "release_speed_level": 5,
    "acceleration_r_s2": 100,
    "max_current_ma": 500,
    "auto_test_target_n": 2.0,
    "auto_test_rounds": 10,
    "auto_test_interval_seconds": 0.0,
    "sweep_max_force_n": 30.0,
    "sweep_force_interval_n": 1.0,
    "sweep_cycle_count": 10,
    "sweep_idle_stable_seconds": 1.0,
    "sweep_idle_timeout_seconds": 15.0,
}


class ForceControlError(RuntimeError):
    def __init__(self, reason, message, brake_required=True):
        super().__init__(message)
        self.reason = str(reason)
        self.brake_required = bool(brake_required)


class ForcePressCsvRecorder:
    """线程安全地实时保存压力计、运动和传感器反馈。"""

    FIELD_NAMES = (
        "timestamp",
        "session_elapsed_ms",
        "sequence",
        "event",
        "operation_id",
        "control_mode",
        "motor_model", "position_unit", "steps_per_turn", "speed_unit", "acceleration_unit",
        "control_strategy", "fine_step_max", "approach_speed",
        "first_in_band_s", "stable_arrival_s",
        "force_unit",
        "auto_round_index",
        "auto_round_total",
        "pressure_level_index",
        "pressure_level_total",
        "calibration_cycle_index",
        "calibration_cycle_total",
        "sweep_direction",
        "direction_step_index",
        "direction_step_total",
        "repeat_index",
        "repeat_total",
        "attempt_index",
        "target_force_n",
        "lookup_created_at",
        "release_reference_position_counts",
        "lookup_target_position_counts",
        "press_arrival_position_counts",
        "press_arrival_position_error_counts",
        "press_position_correction_counts",
        "press_position_error_before_correction_counts",
        "press_position_error_after_correction_counts",
        "floating_capture_position_counts",
        "floating_position_error_counts",
        "floating_running_state",
        "final_force_n",
        "raw_pressure_n",
        "filtered_force_n",
        "ocr_accepted",
        "ocr_rejection_reason",
        "ocr_failure_image_path",
        "ocr_age_ms",
    ) + SENSOR_LOG_FIELD_NAMES + AUTO_RAW_MEDIAN_FIELD_NAMES + (
        "position_counts",
        "position_deg",
        "measured_speed_rpm",
        "position_speed_counts_s",
        "estimated_acceleration_r_s2",
        "sample_interval_ms",
        "phase_current_ma",
        "max_current_ma",
        "running_state",
        "running_state_name",
        "command_counts",
        "total_travel_counts",
        "press_speed_level",
        "contact_before_step_counts",
        "contact_after_step_counts",
        "approach_step_counts",
        "approach_pressure_threshold_n",
        "release_speed_level",
        "command_speed_rpm",
        "command_acceleration_r_s2",
        "max_travel_counts",
        "sweep_idle_stable_seconds",
        "sweep_idle_timeout_seconds",
        "release_idle_recovered",
        "release_idle_wait_seconds",
        "release_idle_stable_seconds",
        "release_idle_timeout_seconds",
        "stop_reason",
        "message",
        "ocr_text",
        "sensor_record_interval_ms",
        "sensor_record_reason",
        "sensor_states_json",
        "sensor_previous_states_json",
        "sensor_snapshot_fresh",
        "sensor_record_source_event",
        "auto_test_interval_seconds", "auto_interval_elapsed_seconds",
    )

    def __init__(
        self,
        output_dir=None,
        max_file_size_bytes=MOVEMENT_LOG_MAX_BYTES,
        max_files=MOVEMENT_LOG_MAX_FILES,
    ):
        self.output_dir = Path(output_dir) if output_dir else force_press_test_directory()
        self.max_file_size_bytes = max(1, int(max_file_size_bytes))
        self.max_files = max(1, int(max_files))
        self._lock = threading.Lock()
        self._handle = None
        self._path = None
        self._started_at = 0.0
        self._row_count = 0
        self._file_row_count = 0
        self._size_bytes = 0
        self._last_error = None
        self._recording = False
        self._write_queue = None
        self._writer_thread = None
        self._stop_token = object()

    @property
    def is_recording(self):
        with self._lock:
            return self._recording

    @property
    def current_path(self):
        with self._lock:
            return self._path

    @property
    def last_error(self):
        with self._lock:
            return self._last_error

    @classmethod
    def _csv_text(cls, row=None, include_header=False):
        buffer = io.StringIO(newline="")
        writer = csv.DictWriter(
            buffer,
            fieldnames=cls.FIELD_NAMES,
            extrasaction="ignore",
        )
        if include_header:
            writer.writeheader()
        if row is not None:
            writer.writerow(row)
        return buffer.getvalue()

    def _new_path_locked(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = self.output_dir / f"force_press_{stamp}.csv"
        suffix = 1
        while path.exists():
            path = self.output_dir / f"force_press_{stamp}_{suffix:03d}.csv"
            suffix += 1
        return path

    def _prune_old_files_locked(self):
        try:
            candidates = list(self.output_dir.glob("force_press_*.csv"))
            candidates.sort(key=lambda path: (path.stat().st_mtime_ns, path.name))
        except OSError as exc:
            self._last_error = exc
            return
        while len(candidates) > self.max_files:
            oldest = candidates.pop(0)
            if oldest == self._path:
                candidates.append(oldest)
                continue
            try:
                oldest.unlink()
            except OSError as exc:
                self._last_error = exc
                break

    def _open_new_file_locked(self):
        path = self._new_path_locked()
        handle = path.open("w", encoding="utf-8-sig", newline="")
        header = self._csv_text(include_header=True)
        handle.write(header)
        handle.flush()
        self._handle = handle
        self._path = path
        self._file_row_count = 0
        self._size_bytes = path.stat().st_size
        self._last_error = None
        self._prune_old_files_locked()
        return path

    def _rotate_locked(self):
        if self._handle is not None:
            self._handle.flush()
            self._handle.close()
        self._handle = None
        return self._open_new_file_locked()

    def start(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            if self._recording:
                return self._path
            self._started_at = time.monotonic()
            self._row_count = 0
            path = self._open_new_file_locked()
            self._write_queue = queue.Queue(
                maxsize=MOVEMENT_LOG_QUEUE_MAX_ROWS
            )
            self._recording = True
            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                name="force-press-csv-writer",
                daemon=True,
            )
            self._writer_thread.start()
            return path

    def record(self, values):
        with self._lock:
            if not self._recording or self._write_queue is None:
                return False
            now = datetime.now().astimezone()
            elapsed_ms = round(
                (time.monotonic() - self._started_at) * 1000.0,
                3,
            )
            sequence = self._row_count
            item = (
                dict(values),
                now.isoformat(timespec="milliseconds"),
                elapsed_ms,
                sequence,
            )
            try:
                self._write_queue.put_nowait(item)
            except queue.Full:
                self._last_error = RuntimeError(
                    "运动日志后台队列已满，本行记录被丢弃"
                )
                return False
            self._row_count += 1
        return True

    def _write_item(self, item):
        values, timestamp, elapsed_ms, sequence = item
        row = {name: values.get(name, "") for name in self.FIELD_NAMES}
        for field_name in PRESSURE_N_CSV_FIELDS:
            value = row.get(field_name)
            if value not in (None, ""):
                row[field_name] = f"{float(value):.2f}"
        row["timestamp"] = timestamp
        row["session_elapsed_ms"] = elapsed_ms
        row["sequence"] = sequence
        text = self._csv_text(row=row)
        encoded_size = len(text.encode("utf-8"))
        if (
            self._file_row_count > 0
            and self._size_bytes + encoded_size > self.max_file_size_bytes
        ):
            self._rotate_locked()
        self._handle.write(text)
        self._size_bytes += encoded_size
        self._file_row_count += 1

    def _writer_loop(self):
        write_queue = self._write_queue
        pending_flush_rows = 0
        last_flush_at = time.monotonic()
        try:
            while True:
                try:
                    item = write_queue.get(
                        timeout=MOVEMENT_LOG_FLUSH_INTERVAL_SECONDS
                    )
                except queue.Empty:
                    item = None
                if item is self._stop_token:
                    break
                if item is not None:
                    self._write_item(item)
                    pending_flush_rows += 1
                now = time.monotonic()
                if pending_flush_rows and (
                    pending_flush_rows >= MOVEMENT_LOG_FLUSH_MAX_ROWS
                    or now - last_flush_at
                    >= MOVEMENT_LOG_FLUSH_INTERVAL_SECONDS
                ):
                    self._handle.flush()
                    pending_flush_rows = 0
                    last_flush_at = now
        except (OSError, ValueError, TypeError, OverflowError) as exc:
            with self._lock:
                self._last_error = exc
                self._recording = False
        finally:
            try:
                if self._handle is not None:
                    self._handle.flush()
                    self._handle.close()
            except OSError as exc:
                with self._lock:
                    self._last_error = exc
            finally:
                with self._lock:
                    self._handle = None
                    self._recording = False

    def stop(self):
        with self._lock:
            path = self._path
            row_count = self._row_count
            self._recording = False
            write_queue = self._write_queue
            writer_thread = self._writer_thread
        if write_queue is not None and writer_thread is not None:
            try:
                write_queue.put(
                    self._stop_token,
                    timeout=MOVEMENT_LOG_STOP_TIMEOUT_SECONDS,
                )
            except queue.Full:
                with self._lock:
                    self._last_error = RuntimeError(
                        "关闭时运动日志后台队列仍为满载"
                    )
            writer_thread.join(MOVEMENT_LOG_STOP_TIMEOUT_SECONDS)
            if writer_thread.is_alive():
                with self._lock:
                    self._last_error = RuntimeError(
                        "关闭时运动日志后台写入未能及时结束"
                    )
        with self._lock:
            self._write_queue = None
            self._writer_thread = None
            self._path = None
            return path, row_count


class ForcePressControlWidget(QtWidgets.QGroupBox):
    """在监控页中按压力－位置查找表执行一次性下压和安全释放。"""

    status_message = Signal(str)
    operation_progress = Signal(object)
    operation_finished = Signal(object)
    # 只发布已接受的运动及结束事件，供手动观察页计时，不另建电机控制入口。
    manual_observation_event = Signal(object)
    force_mapping_captured = Signal(object)
    quick_correction_completed = Signal(object)

    def __init__(
        self,
        motor_control_widget,
        sensor_sample_provider=None,
        parent=None,
    ):
        super().__init__("按力下压（压力－位置查表）", parent)
        self.setObjectName("ForcePressControl")
        self.setCheckable(True)
        self.setChecked(False)
        self._motor_control = motor_control_widget
        self._sensor_sample_provider = sensor_sample_provider
        self._last_recorded_sensor_sequence = None
        self._state_lock = threading.Lock()
        self._latest_ocr_event = None
        self._accepted_force_n = None
        self._accepted_control_force_n = None
        self._accepted_force_at = 0.0
        self._accepted_force_sequence = 0
        self._rejected_ocr_count = 0
        self._consecutive_rejected_ocr_count = 0
        self._operation_rejected_ocr_count = 0
        self._running = False
        self._operation_id = 0
        self._control_mode = "idle"
        self._target_force_n = None
        self._auto_round_index = 0
        self._auto_round_total = 0
        self._pressure_level_index = 0
        self._pressure_level_total = 0
        self._calibration_cycle_index = 0
        self._calibration_cycle_total = 0
        self._sweep_direction = ""
        self._direction_step_index = 0
        self._direction_step_total = 0
        self._repeat_index = 0
        self._repeat_total = 0
        self._attempt_index = 0
        self._command_counts = 0
        self._total_travel_counts = 0
        self._settings_snapshot = {}
        self._closing = False
        self._last_status_message = ""
        self._last_summary_update_at = 0.0
        self._last_progress_stage = None
        self._communication_recovery_generation = 0
        self._force_mapping_log_count = 0
        self._force_mapping_entries = []
        self._sweep_entries = []
        self._validation_entries = []
        self._lookup_document = None
        self._lookup_targets = {}
        self._quick_correction_ram_trial_active = False
        self._mcu_force_display_state = None
        self._last_motor_running_state = None
        self._settings_path = force_press_settings_path()
        namespace = getattr(getattr(self, "_profile", None), "namespace", "")
        if namespace:
            self._settings_path = self._settings_path.with_name(f"force_press_control_{namespace}.json")
        settings = self._load_settings()
        self._recorder = ForcePressCsvRecorder(
            force_press_test_directory() / namespace if namespace else None)
        self._log_sampling = MotionLogSampling()
        self._log_sampling_lock = threading.Lock()
        self._target_buttons = []
        self._build_ui(settings)
        self._configure_numeric_inputs()
        self._sensor_record_timer = QtCore.QTimer(self)
        self._sensor_record_timer.setInterval(SENSOR_RECORD_POLL_INTERVAL_MS)
        self._sensor_record_timer.timeout.connect(self._record_latest_sensor_sample)
        if callable(self._sensor_sample_provider):
            self._sensor_record_timer.start()
        self.toggled.connect(self._set_expanded)
        self.toggled.connect(self._reload_lookup_when_expanded)
        self._set_expanded(False)
        self._update_settings_snapshot()
        for widget in (
            self.press_speed_input,
            self.release_speed_input,
            self.acceleration_input,
            self.max_current_input,
            self.auto_test_rounds_input,
            self.auto_test_interval_input,
            self.sweep_max_force_input,
            self.sweep_force_interval_input,
            self.sweep_repeats_input,
            self.sweep_idle_stable_input,
            self.sweep_idle_timeout_input,
        ):
            widget.valueChanged.connect(self._update_settings_snapshot)
            widget.valueChanged.connect(self._save_settings_if_idle)
        self.auto_test_target_input.currentIndexChanged.connect(
            self._save_settings_if_idle
        )
        self.auto_test_target_input.currentIndexChanged.connect(
            self._refresh_controls
        )
        for widget in (
            self.sweep_max_force_input,
            self.sweep_force_interval_input,
            self.sweep_repeats_input,
            self.sweep_idle_stable_input,
            self.sweep_idle_timeout_input,
        ):
            widget.valueChanged.connect(self._update_sweep_plan_label)
        self._update_sweep_plan_label()
        self.operation_progress.connect(self._on_operation_progress)
        self.operation_finished.connect(self._on_operation_finished)
        self.force_mapping_captured.connect(self._append_force_mapping_entry)
        self._motor_control.motor_feedback.connect(self._on_motor_feedback)
        self._motor_control.control_state_changed.connect(self._on_motor_state_changed)
        self._motor_control.release_position_changed.connect(
            self._on_release_position_changed
        )
        self._on_release_position_changed(
            self._motor_control.release_position_snapshot()
        )
        self._load_lookup_document(show_error=False)
        latest = self._motor_control.latest_motor_feedback()
        if latest is not None:
            try:
                self._last_motor_running_state = int(latest.get("running_state"))
            except (TypeError, ValueError):
                self._last_motor_running_state = None
            self._show_motor_feedback(latest)
        self._start_realtime_recording()
        self._refresh_controls()

    def _build_ui(self, settings):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(9, 12, 9, 8)
        outer.setSpacing(7)
        self.content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(7)

        note = WrappedLabel(
            f"每次开始下压前压力计须处于 "
            f"-{PRESSURE_METER_NEGATIVE_LIMIT_N:.2f}～"
            f"+{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N，"
            "容差内残余值按零点处理，超出则拒绝启动。"
            "目标按钮按照最新压力－位置JSON中的稳定档位计算绝对位置，"
            "并按设置速度只发送一次下压位移命令。"
            "压力计仅用于启动校验、运动中超压保护和到位结果显示，不再分段步进或补压。"
            "释放时一次性返回该JSON记录的标定出发点。"
        )
        self.workflow_note = note
        note.setStyleSheet("color:#8bd5ff;")
        layout.addWidget(note)

        target_group = QtWidgets.QGroupBox("目标压力")
        target_grid = QtWidgets.QGridLayout(target_group)
        target_grid.setHorizontalSpacing(6)
        target_grid.setVerticalSpacing(6)
        for index, target_n in enumerate(TARGET_FORCES_N):
            button = QtWidgets.QPushButton(f"{target_n:.2f} N")
            button.setObjectName("ForceTargetButton")
            button.clicked.connect(
                lambda _checked=False, value=target_n: self.start_press(value)
            )
            target_grid.addWidget(button, index // 5, index % 5)
            self._target_buttons.append(button)
        layout.addWidget(target_group)

        self.lookup_map_label = WrappedLabel("压力－位置查找表：正在读取")
        layout.addWidget(self.lookup_map_label)

        parameter_group = QtWidgets.QGroupBox("运动与安全参数")
        self.motion_parameter_group = parameter_group
        parameter_grid = QtWidgets.QGridLayout(parameter_group)
        parameter_grid.setHorizontalSpacing(8)
        parameter_grid.setVerticalSpacing(6)

        self.press_speed_input = QtWidgets.QSpinBox()
        self.press_speed_input.setRange(1, maximum_speed_level())
        self.press_speed_input.setValue(int(settings["press_speed_level"]))
        self.press_speed_input.setToolTip("所有下压阶段统一使用该运动速度")
        self._add_parameter(
            parameter_grid,
            "下压运动速度",
            self.press_speed_input,
            "档",
            0,
            0,
        )

        self.contact_before_step_input = QtWidgets.QSpinBox()
        self.contact_before_step_input.setRange(MIN_STEP_COUNTS, MAX_PRESS_COMMAND_COUNTS)
        self.contact_before_step_input.setSingleStep(MIN_STEP_COUNTS)
        self.contact_before_step_input.setValue(int(settings["contact_before_step_counts"]))
        self.contact_before_step_input.setToolTip("压力计仍处于±0.05 N零点容差时的单步下压距离")
        self.contact_before_step_input.hide()

        self.contact_after_step_input = QtWidgets.QSpinBox()
        self.contact_after_step_input.setRange(MIN_STEP_COUNTS, MAX_PRESS_COMMAND_COUNTS)
        self.contact_after_step_input.setSingleStep(MIN_STEP_COUNTS)
        self.contact_after_step_input.setValue(int(settings["contact_after_step_counts"]))
        self.contact_after_step_input.setToolTip("接触后、进入接近压力阈值前的单步下压距离")
        self.contact_after_step_input.hide()

        self.approach_step_input = QtWidgets.QSpinBox()
        self.approach_step_input.setRange(MIN_STEP_COUNTS, MAX_PRESS_COMMAND_COUNTS)
        self.approach_step_input.setSingleStep(MIN_STEP_COUNTS)
        self.approach_step_input.setValue(int(settings["approach_step_counts"]))
        self.approach_step_input.setToolTip("进入接近压力阈值后及自动补压时的单步下压距离")
        self.approach_step_input.hide()

        self.approach_pressure_threshold_input = QtWidgets.QDoubleSpinBox()
        self.approach_pressure_threshold_input.setDecimals(2)
        self.approach_pressure_threshold_input.setRange(
            TARGET_TOLERANCE_N + 0.01,
            MAX_SWEEP_FORCE_N,
        )
        self.approach_pressure_threshold_input.setSingleStep(0.10)
        self.approach_pressure_threshold_input.setValue(
            float(settings["approach_pressure_threshold_n"])
        )
        self.approach_pressure_threshold_input.setToolTip(
            "当前压力距离目标压力不超过该值时切换为接近目标步长；须大于目标±0.10 N容差"
        )
        self.approach_pressure_threshold_input.hide()

        self.release_speed_input = QtWidgets.QSpinBox()
        self.release_speed_input.setRange(1, maximum_speed_level())
        self.release_speed_input.setValue(int(settings["release_speed_level"]))
        self._add_parameter(parameter_grid, "抬起速度", self.release_speed_input, "档", 0, 2)

        self.acceleration_input = QtWidgets.QSpinBox()
        self.acceleration_input.setRange(0, 200)
        self.acceleration_input.setValue(int(settings["acceleration_r_s2"]))
        self._add_parameter(parameter_grid, "加减速度", self.acceleration_input, "r/s²", 0, 4)

        self.max_current_input = QtWidgets.QSpinBox()
        self.max_current_input.setRange(10, MOTOR_MAX_TORQUE_CURRENT_MA)
        self.max_current_input.setSingleStep(50)
        self.max_current_input.setValue(int(settings["max_current_ma"]))
        self.max_current_input.setToolTip(
            "操作开始时通过0x64写入驱动器硬件力矩电流上限；"
            "上位机同时执行连续过流和严重过流监控"
        )
        self._add_parameter(parameter_grid, "最大电流", self.max_current_input, "mA", 0, 6)

        protection = WrappedLabel(
            f"压力保护：固定上限 {MAX_OPERATION_FORCE_N:.2f} N；"
            f"内部安全行程 {MAX_OPERATION_TRAVEL_COUNTS} counts；"
            "电流使用驱动器硬件限制"
        )
        protection.setStyleSheet("color:#ff9f43;font-weight:700;")
        parameter_grid.addWidget(protection, 1, 0, 1, 8)
        layout.addWidget(parameter_group)

        auto_group = QtWidgets.QGroupBox("自动重复测试", self.content)
        self.auto_test_group = auto_group
        auto_layout = QtWidgets.QVBoxLayout(auto_group)
        auto_layout.setContentsMargins(8, 7, 8, 7)
        auto_layout.setSpacing(5)
        auto_row = QtWidgets.QHBoxLayout()
        auto_layout.addLayout(auto_row)
        auto_row.setSpacing(7)
        auto_row.addWidget(QtWidgets.QLabel("目标压力"))
        self.auto_test_target_input = QtWidgets.QComboBox()
        for target_n in TARGET_FORCES_N:
            self.auto_test_target_input.addItem(f"{target_n:.2f}", target_n)
        self.auto_test_target_input.setCurrentText(
            f"{float(settings['auto_test_target_n']):.2f}"
        )
        self.auto_test_target_input.setFixedWidth(100)
        auto_row.addWidget(self.auto_test_target_input)
        auto_row.addWidget(QtWidgets.QLabel("N"))
        auto_row.addWidget(QtWidgets.QLabel("重复次数"))
        self.auto_test_rounds_input = QtWidgets.QSpinBox()
        self.auto_test_rounds_input.setRange(1, 100)
        self.auto_test_rounds_input.setValue(int(settings["auto_test_rounds"]))
        self.auto_test_rounds_input.setFixedWidth(90)
        auto_row.addWidget(self.auto_test_rounds_input)
        auto_row.addWidget(QtWidgets.QLabel("次"))
        auto_row.addWidget(QtWidgets.QLabel("释放后间隔"))
        self.auto_test_interval_input = QtWidgets.QDoubleSpinBox()
        self.auto_test_interval_input.setRange(0.0, 3600.0)
        self.auto_test_interval_input.setDecimals(2)
        self.auto_test_interval_input.setSingleStep(1.0)
        self.auto_test_interval_input.setValue(float(settings['auto_test_interval_seconds']))
        self.auto_test_interval_input.setFixedWidth(90)
        self.auto_test_interval_input.setToolTip(
            '确认释放、压力接近零后开始计时；到时重复相同目标压力，不等通道状态或Raw恢复。0表示不额外等待，末次不等待。')
        auto_row.addWidget(self.auto_test_interval_input)
        auto_row.addWidget(QtWidgets.QLabel("秒"))
        auto_hint = WrappedLabel(
            "每轮：查表一次性到位 → CH4～CH7全部FLOATING后连续采集1秒 → "
            "记录Signal和Raw → 释放 → 等设定秒数再按压（不等通道恢复）"
        )
        auto_hint.setStyleSheet("color:#8bd5ff;")
        auto_row.addStretch(1)
        self.start_auto_test_button = QtWidgets.QPushButton("开始自动测试")
        self.start_auto_test_button.setObjectName("ForceAutoTestButton")
        self.start_auto_test_button.clicked.connect(self.start_auto_test)
        auto_row.addWidget(self.start_auto_test_button)
        auto_layout.addWidget(auto_hint)
        layout.addWidget(auto_group)

        sweep_group = QtWidgets.QGroupBox("传感器标定（多压力）", self.content)
        self.sensor_calibration_group = sweep_group
        sweep_layout = QtWidgets.QVBoxLayout(sweep_group)
        sweep_layout.setContentsMargins(8, 7, 8, 7)
        sweep_layout.setSpacing(6)
        quick_correction_row = QtWidgets.QHBoxLayout()
        quick_correction_row.setSpacing(7)
        quick_correction_hint = WrappedLabel(
            "快速修正：低力0.1/0.5/1/1.5/2 N、高力4～30 N每2 N各测一次，Raw节点累计最多"
            f"{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}，"
            "完成后先在MCU RAM试用"
        )
        quick_correction_hint.setStyleSheet("color:#f4d35e;font-weight:700;")
        quick_correction_row.addWidget(quick_correction_hint, 1)
        self.start_quick_correction_button = QtWidgets.QPushButton(
            "两段快速K修正"
        )
        self.start_quick_correction_button.setObjectName(
            "ForceQuickCorrectionButton"
        )
        self.start_quick_correction_button.setToolTip(
            "依次从零点执行两段共19个代表档位，采集压力计与FLOATING Signal；"
            f"完成后只把{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}修正表暂存到MCU RAM，"
            "验证满意后再单独保存Flash"
        )
        self.start_quick_correction_button.clicked.connect(
            self.start_quick_correction_test
        )
        quick_correction_row.addWidget(self.start_quick_correction_button)
        sweep_layout.addLayout(quick_correction_row)
        self.quick_correction_ram_status_label = WrappedLabel(
            "MCU RAM写入状态：本次连接尚未执行快速K修正"
        )
        self.quick_correction_ram_status_label.setObjectName(
            "ForceQuickCorrectionRamStatus"
        )
        self.quick_correction_ram_status_label.setStyleSheet(
            "QLabel#ForceQuickCorrectionRamStatus {"
            "color:#b9d7e8;background:#101923;border:1px solid #365b6f;"
            "border-radius:5px;padding:7px;font-weight:700;"
            "}"
        )
        sweep_layout.addWidget(self.quick_correction_ram_status_label)
        sweep_parameter_row = QtWidgets.QHBoxLayout()
        sweep_parameter_row.setSpacing(7)
        sweep_parameter_row.addWidget(QtWidgets.QLabel("最大压力"))
        self.sweep_max_force_input = QtWidgets.QDoubleSpinBox()
        self.sweep_max_force_input.setDecimals(2)
        self.sweep_max_force_input.setRange(
            max(FORCE_CALIBRATION_TARGETS_N),
            max(FORCE_CALIBRATION_TARGETS_N),
        )
        self.sweep_max_force_input.setValue(max(FORCE_CALIBRATION_TARGETS_N))
        self.sweep_max_force_input.setEnabled(False)
        self.sweep_max_force_input.setToolTip(
            "两段力标定：0.1～2 N每0.1 N，3～30 N每1 N，共48个实体节点"
        )
        self.sweep_max_force_input.setFixedWidth(90)
        sweep_parameter_row.addWidget(self.sweep_max_force_input)
        sweep_parameter_row.addWidget(QtWidgets.QLabel("N"))
        sweep_parameter_row.addWidget(QtWidgets.QLabel("高力段间隔"))
        self.sweep_force_interval_input = QtWidgets.QDoubleSpinBox()
        self.sweep_force_interval_input.setDecimals(2)
        self.sweep_force_interval_input.setRange(1.0, 1.0)
        self.sweep_force_interval_input.setValue(1.0)
        self.sweep_force_interval_input.setEnabled(False)
        self.sweep_force_interval_input.setToolTip(
            "高力段2～30 N每1 N；低力段0.1～2 N每0.1 N，2 N共用一个节点"
        )
        self.sweep_force_interval_input.setFixedWidth(90)
        sweep_parameter_row.addWidget(self.sweep_force_interval_input)
        sweep_parameter_row.addWidget(QtWidgets.QLabel("N"))
        sweep_parameter_row.addWidget(QtWidgets.QLabel("往返轮数"))
        self.sweep_repeats_input = QtWidgets.QSpinBox()
        self.sweep_repeats_input.setRange(1, 100)
        self.sweep_repeats_input.setValue(
            int(settings["sweep_cycle_count"])
        )
        self.sweep_repeats_input.setFixedWidth(80)
        sweep_parameter_row.addWidget(self.sweep_repeats_input)
        sweep_parameter_row.addWidget(QtWidgets.QLabel("轮"))
        self.sweep_plan_label = WrappedLabel()
        self.sweep_plan_label.setStyleSheet("color:#8bd5ff;")
        sweep_parameter_row.addStretch(1)
        sweep_layout.addLayout(sweep_parameter_row)
        sweep_layout.addWidget(self.sweep_plan_label)

        sweep_recovery_row = QtWidgets.QHBoxLayout()
        sweep_recovery_row.setSpacing(7)
        sweep_recovery_row.addWidget(QtWidgets.QLabel("回零后传感器稳定"))
        self.sweep_idle_stable_input = QtWidgets.QDoubleSpinBox()
        self.sweep_idle_stable_input.setDecimals(1)
        self.sweep_idle_stable_input.setRange(0.1, 10.0)
        self.sweep_idle_stable_input.setSingleStep(0.5)
        self.sweep_idle_stable_input.setValue(
            float(settings["sweep_idle_stable_seconds"])
        )
        self.sweep_idle_stable_input.setFixedWidth(90)
        sweep_recovery_row.addWidget(self.sweep_idle_stable_input)
        sweep_recovery_row.addWidget(QtWidgets.QLabel("秒"))
        sweep_recovery_row.addWidget(QtWidgets.QLabel("最长等待"))
        self.sweep_idle_timeout_input = QtWidgets.QDoubleSpinBox()
        self.sweep_idle_timeout_input.setDecimals(1)
        self.sweep_idle_timeout_input.setRange(1.0, 120.0)
        self.sweep_idle_timeout_input.setSingleStep(1.0)
        self.sweep_idle_timeout_input.setValue(
            float(settings["sweep_idle_timeout_seconds"])
        )
        self.sweep_idle_timeout_input.setFixedWidth(90)
        sweep_recovery_row.addWidget(self.sweep_idle_timeout_input)
        sweep_recovery_row.addWidget(QtWidgets.QLabel("秒"))
        recovery_hint = WrappedLabel(
            "四通道均为IDLE/WAIT/RECOVER/SLOW_BACK、Signal为0且Raw稳定后继续；超时则记录警告"
        )
        recovery_hint.setStyleSheet("color:#8bd5ff;")
        sweep_recovery_row.addWidget(recovery_hint, 1)
        sweep_layout.addLayout(sweep_recovery_row)

        sweep_action_row = QtWidgets.QHBoxLayout()
        sweep_action_row.setSpacing(7)
        self.start_sweep_test_button = QtWidgets.QPushButton("开始传感器标定")
        self.start_sweep_test_button.setObjectName("ForceSweepTestButton")
        self.start_sweep_test_button.clicked.connect(self.start_sweep_test)
        sweep_action_row.addWidget(self.start_sweep_test_button, 1)
        self.start_validation_test_button = QtWidgets.QPushButton(
            "仅验证（不标定）"
        )
        self.start_validation_test_button.setObjectName(
            "ForceValidationTestButton"
        )
        self.start_validation_test_button.setToolTip(
            "执行相同的多压力重复运动并记录压力计、位置和FLOATING Signal，"
            "结果不会参与K值表生成"
        )
        self.start_validation_test_button.clicked.connect(
            self.start_validation_test
        )
        sweep_action_row.addWidget(self.start_validation_test_button, 1)
        self.resume_sweep_test_button = QtWidgets.QPushButton("读取上次标定并继续")
        self.resume_sweep_test_button.clicked.connect(self.resume_sweep_test)
        sweep_action_row.addWidget(self.resume_sweep_test_button, 1)
        self.save_sweep_log_button = QtWidgets.QPushButton("保存传感器标定数据")
        self.save_sweep_log_button.setEnabled(False)
        self.save_sweep_log_button.clicked.connect(self.save_sweep_log)
        sweep_action_row.addWidget(self.save_sweep_log_button, 1)
        self.save_validation_log_button = QtWidgets.QPushButton("保存验证数据")
        self.save_validation_log_button.setEnabled(False)
        self.save_validation_log_button.clicked.connect(
            self.save_validation_log
        )
        sweep_action_row.addWidget(self.save_validation_log_button, 1)
        sweep_layout.addLayout(sweep_action_row)
        sweep_hint = WrappedLabel(
            f"传感器标定启动后先按本次最大压力自动执行 {WARMUP_ROUNDS} 次热身，"
            "热身不写入标定结果或K值表。正式标定奇数轮由小到大再由大到小，"
            "偶数轮由大到小再由小到大；"
            "每个压力点都从零点出发，FLOATING采集1秒后返回零点，并等待传感器恢复稳定。"
            f"下压到位超时会返回出发点并重试当前轮，连续最多"
            f"{SWEEP_MOTION_TIMEOUT_MAX_RETRIES}次；出发压力未归零时每"
            f"{SWEEP_START_ZERO_RETRY_SECONDS:g}秒自动重试；"
            "标定每轮成功后自动更新恢复记录；验证数据单独保存且不参与K值表。"
        )
        sweep_hint.setStyleSheet("color:#9fc3d9;")
        sweep_layout.addWidget(sweep_hint)
        self.sweep_stage_label = WrappedLabel(
            f"传感器标定阶段：空闲；启动后先按本次最大压力进行{WARMUP_ROUNDS}次热身"
        )
        self.sweep_stage_label.setObjectName("ForceSweepStageIndicator")
        self.sweep_stage_label.setStyleSheet(
            "QLabel#ForceSweepStageIndicator {"
            "color:#9fc3d9;background:#101923;border:1px solid #365b6f;"
            "border-radius:5px;padding:7px;font-weight:700;"
            "}"
        )
        sweep_layout.addWidget(self.sweep_stage_label)
        layout.addWidget(sweep_group)

        action_row = QtWidgets.QHBoxLayout()
        self.set_release_position_button = QtWidgets.QPushButton("设置释放位置")
        self.set_release_position_button.setObjectName(
            "ForceSetReleasePositionButton"
        )
        self.set_release_position_button.clicked.connect(
            self._motor_control.capture_release_position
        )
        self.set_release_position_button.setToolTip(
            "把当前电机位置设为共享释放位置和后续下压的标定出发点"
        )
        action_row.addWidget(self.set_release_position_button, 1)
        self.release_button = QtWidgets.QPushButton(RELEASE_BUTTON_IDLE_TEXT)
        self.release_button.setObjectName("ForceReleaseButton")
        self.release_button.clicked.connect(self.start_release)
        action_row.addWidget(self.release_button, 1)
        self.warmup_button = QtWidgets.QPushButton("热身")
        self.warmup_button.setObjectName("ForceWarmupButton")
        self.warmup_button.setToolTip(
            f"按查找表自动执行{WARMUP_FORCE_N:.2f} N下压和释放，共{WARMUP_ROUNDS}次"
        )
        self.warmup_button.clicked.connect(self.start_warmup)
        action_row.addWidget(self.warmup_button, 1)
        self.stop_button = QtWidgets.QPushButton("紧急停止")
        self.stop_button.setObjectName("ForceStopButton")
        self.stop_button.clicked.connect(self.request_stop)
        action_row.addWidget(self.stop_button, 1)
        layout.addLayout(action_row)
        self.release_position_label = WrappedLabel()
        layout.addWidget(self.release_position_label)

        mcu_force_group = QtWidgets.QGroupBox("MCU 实时法向合力")
        mcu_force_group.setObjectName("McuCalibratedForceCard")
        mcu_force_layout = QtWidgets.QVBoxLayout(mcu_force_group)
        mcu_force_layout.setContentsMargins(14, 12, 14, 12)
        mcu_force_layout.setSpacing(5)
        mcu_force_header = QtWidgets.QHBoxLayout()
        self.mcu_force_value_label = QtWidgets.QLabel("--.-- N")
        self.mcu_force_value_label.setObjectName("McuCalibratedForceValue")
        mcu_force_header.addWidget(self.mcu_force_value_label)
        mcu_force_header.addStretch(1)
        self.mcu_force_state_label = QtWidgets.QLabel("等待 MCU Type 0x02 力值")
        self.mcu_force_state_label.setObjectName("McuCalibratedForceState")
        mcu_force_header.addWidget(self.mcu_force_state_label)
        mcu_force_layout.addLayout(mcu_force_header)
        self.mcu_force_path_label = WrappedLabel(
            "CH4～CH7 Signal和 → MCU标定换算 → 原始力值；V18检测线0.09 N、量程小于1 N，吸附前后对比见低力标定页"
        )
        self.mcu_force_path_label.setObjectName("McuCalibratedForcePath")
        mcu_force_layout.addWidget(self.mcu_force_path_label)
        self.floating_force_label = WrappedLabel(
            "通道诊断：等待 CH4～CH7 Signal 与状态数据"
        )
        self.floating_force_label.setObjectName("FloatingForceCurrentValue")
        mcu_force_layout.addWidget(self.floating_force_label)
        layout.addWidget(mcu_force_group)

        telemetry_group = QtWidgets.QGroupBox("压力计与运动安全")
        telemetry_grid = QtWidgets.QGridLayout(telemetry_group)
        telemetry_grid.setHorizontalSpacing(14)
        telemetry_grid.setVerticalSpacing(5)
        self.force_value_label = QtWidgets.QLabel(
            "当前压力计：--.-- N"
        )
        self.force_value_label.setObjectName("ForceLiveValue")
        self.force_value_label.setWordWrap(False)
        telemetry_grid.addWidget(self.force_value_label, 0, 0, 1, 4)
        self.ocr_warning_label = WrappedLabel()
        self.ocr_warning_label.setObjectName("ForcePressureWarning")
        self.ocr_warning_label.setVisible(False)
        telemetry_grid.addWidget(self.ocr_warning_label, 1, 0, 1, 4)
        self.position_value_label = QtWidgets.QLabel("位置 -- counts · --°")
        self.speed_value_label = QtWidgets.QLabel("速度 -- RPM · -- counts/s")
        self.acceleration_value_label = QtWidgets.QLabel("估算加速度 -- r/s²")
        self.current_value_label = QtWidgets.QLabel("瞬时相电流 -- mA / 限制 500 mA")
        self.state_value_label = QtWidgets.QLabel("电机状态 --")
        self.command_value_label = QtWidgets.QLabel("本步 -- counts · 累计 -- counts")
        telemetry_grid.addWidget(self.position_value_label, 2, 0, 1, 2)
        telemetry_grid.addWidget(self.speed_value_label, 2, 2, 1, 2)
        telemetry_grid.addWidget(self.acceleration_value_label, 3, 0, 1, 2)
        telemetry_grid.addWidget(self.current_value_label, 3, 2, 1, 2)
        telemetry_grid.addWidget(self.state_value_label, 4, 0, 1, 2)
        telemetry_grid.addWidget(self.command_value_label, 4, 2, 1, 2)
        layout.addWidget(telemetry_group)

        record_row = QtWidgets.QHBoxLayout()
        self.record_status_label = WrappedLabel(
            "运动日志：实时滚动保存，包含自动测试每轮Signal与Raw中位值"
            "（每个 5 MiB，最多 10 个）"
        )
        record_row.addWidget(self.record_status_label, 1)
        layout.addLayout(record_row)

        self.status_label = WrappedLabel("")
        self.status_label.setObjectName("ForceControlStatus")
        layout.addWidget(self.status_label)
        self.status_label.hide()
        self.status_detail_label = WrappedLabel("详细状态：--")
        self.status_detail_label.setObjectName("ForceControlDetail")
        layout.addWidget(self.status_detail_label)
        self.last_result_label = WrappedLabel("上次结果：--")
        self.last_result_label.setObjectName("ForceControlLastResult")
        layout.addWidget(self.last_result_label)

        self.force_mapping_group = QtWidgets.QGroupBox(
            "采样记录（0 条，点击展开）"
        )
        self.force_mapping_group.setCheckable(True)
        self.force_mapping_group.setChecked(False)
        mapping_layout = QtWidgets.QVBoxLayout(self.force_mapping_group)
        mapping_layout.setContentsMargins(8, 8, 8, 8)
        mapping_layout.setSpacing(6)
        self.force_mapping_details = QtWidgets.QWidget()
        force_mapping_details_layout = QtWidgets.QVBoxLayout(
            self.force_mapping_details
        )
        force_mapping_details_layout.setContentsMargins(0, 0, 0, 0)
        force_mapping_details_layout.setSpacing(6)
        mapping_toolbar = QtWidgets.QHBoxLayout()
        self.force_mapping_log_count_label = QtWidgets.QLabel("共 0 条")
        self.force_mapping_log_count_label.setStyleSheet("color:#9fc3d9;")
        mapping_toolbar.addWidget(self.force_mapping_log_count_label)
        mapping_toolbar.addStretch(1)
        self.save_force_mapping_log_button = QtWidgets.QPushButton("保存记录CSV")
        self.save_force_mapping_log_button.setObjectName("ForceMappingLogSaveButton")
        self.save_force_mapping_log_button.setEnabled(False)
        self.save_force_mapping_log_button.setToolTip(
            "将下方当前累计记录单独保存到force_press_tests目录"
        )
        self.save_force_mapping_log_button.clicked.connect(
            self.save_force_mapping_log
        )
        mapping_toolbar.addWidget(self.save_force_mapping_log_button)
        self.clear_force_mapping_log_button = QtWidgets.QPushButton("清空记录")
        self.clear_force_mapping_log_button.setObjectName("ForceMappingLogClearButton")
        self.clear_force_mapping_log_button.setEnabled(False)
        self.clear_force_mapping_log_button.clicked.connect(
            self.clear_force_mapping_log
        )
        mapping_toolbar.addWidget(self.clear_force_mapping_log_button)
        force_mapping_details_layout.addLayout(mapping_toolbar)
        self.force_mapping_log = QtWidgets.QPlainTextEdit()
        self.force_mapping_log.setObjectName("ForceMappingLog")
        self.force_mapping_log.setReadOnly(True)
        self.force_mapping_log.setPlaceholderText(
            "达到目标压力后自动记录Signal，并统计同一秒内CH4～CH7 Raw中位值"
        )
        self.force_mapping_log.setMaximumHeight(150)
        self.force_mapping_log.document().setMaximumBlockCount(1000)
        force_mapping_details_layout.addWidget(self.force_mapping_log)
        mapping_layout.addWidget(self.force_mapping_details)
        self.force_mapping_group.toggled.connect(
            self.force_mapping_details.setVisible
        )
        self.force_mapping_details.setVisible(False)
        layout.addWidget(self.force_mapping_group)
        outer.addWidget(self.content)

        self.setStyleSheet("""
            QGroupBox#ForcePressControl {
                border: 1px solid #365b6f;
                border-radius: 7px;
                margin-top: 7px;
                padding-top: 5px;
                font-weight: 700;
                background: #0b1119;
            }
            QGroupBox#ForcePressControl::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #91dff9;
            }
            QLabel#ForceLiveValue {
                color:#57d5ff;
                font-family:Consolas,"Microsoft YaHei";
                font-size:18px;
                font-weight:700;
            }
            QLabel#ForcePressureWarning {
                color:#ffffff;
                background:#8b1e1e;
                border:2px solid #ff6b6b;
                border-radius:4px;
                padding:4px 7px;
                font-size:15px;
                font-weight:800;
            }
            QGroupBox#McuCalibratedForceCard {
                color:#8fe9ff;
                background:#082735;
                border:2px solid #35cfee;
                border-radius:8px;
                margin-top:9px;
                padding-top:8px;
                font-weight:800;
            }
            QGroupBox#McuCalibratedForceCard::title {
                subcontrol-origin:margin;
                left:12px;
                padding:0 6px;
            }
            QLabel#McuCalibratedForceValue {
                color:#eaff72;
                font-family:Consolas,"Microsoft YaHei";
                font-size:38px;
                font-weight:900;
            }
            QLabel#McuCalibratedForceState {
                color:#8fe9ff;
                font-size:15px;
                font-weight:800;
            }
            QLabel#McuCalibratedForcePath {
                color:#ffffff;
                font-size:14px;
                font-weight:700;
            }
            QLabel#FloatingForceCurrentValue {
                color:#9fc3d9;
                background:#0b1b25;
                border:1px solid #285167;
                border-radius:4px;
                padding:5px 7px;
                font-family:Consolas,"Microsoft YaHei";
                font-size:12px;
                font-weight:500;
            }
            QLabel#ForceControlStatus {
                color:#f4d35e;
                background:#111b25;
                border:1px solid #3d5265;
                border-radius:5px;
                padding:7px 9px;
                font-size:20px;
                font-weight:700;
            }
            QLabel#ForceControlDetail {
                color:#9fc3d9;
                font-size:12px;
                font-weight:400;
            }
            QLabel#ForceControlLastResult {
                color:#ff9f43;
                font-size:14px;
                font-weight:700;
            }
            QPushButton#ForceTargetButton { background:#bddce6;color:#17212b;font-weight:700; }
            QPushButton#ForceReleaseButton { background:#bfdcd3;color:#17212b;font-weight:700; }
            QPushButton#ForceStopButton { background:#edbbb6;color:#17212b;font-weight:700; }
            QPushButton#ForceRecordButton { background:#c9d6e9;color:#17212b;font-weight:700; }
            QPushButton#ForceAutoTestButton { background:#6fd5a5;color:#102019;font-weight:800; }
            QPushButton#ForceQuickCorrectionButton { background:#f4d35e;color:#211b00;font-weight:800; }
            QPushButton#ForceMappingLogClearButton { background:#536473;color:#ffffff;font-weight:700; }
            QPlainTextEdit#ForceMappingLog {
                color:#d8ff74;
                background:#071018;
                border:1px solid #365b6f;
                border-radius:4px;
                padding:5px;
                font-family:Consolas,"Microsoft YaHei";
                font-size:14px;
            }
            QGroupBox#ForcePressControl QPushButton:disabled {
                background:#59636d;color:#aab2ba;
            }
        """)

    @Slot(bool)
    def _set_expanded(self, expanded):
        self.content.setVisible(bool(expanded))
        self.setMinimumHeight(0)
        self.setMaximumHeight(16777215 if expanded else 32)

    def _configure_numeric_inputs(self):
        button_symbols = getattr(QtWidgets.QAbstractSpinBox, "NoButtons", None)
        if button_symbols is None:
            button_symbols = QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons
        for widget in self.findChildren(QtWidgets.QAbstractSpinBox):
            widget.setButtonSymbols(button_symbols)
            widget.setKeyboardTracking(False)

    def _load_settings(self):
        settings = dict(DEFAULT_FORCE_PRESS_SETTINGS)
        try:
            loaded = json.loads(self._settings_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                for key in settings:
                    if key in loaded:
                        settings[key] = loaded[key]
                # 旧版三段速度配置迁移为统一下压速度，新步长使用当前默认值。
                if "press_speed_level" not in loaded:
                    settings["press_speed_level"] = loaded.get(
                        "contact_after_fast_speed_level",
                        loaded.get(
                            "pre_contact_press_speed_level",
                            settings["press_speed_level"],
                        ),
                    )
                # 旧 g 配置只迁移数值，不复用旧字段名；档位对齐到 0.10 N。
                if (
                    "approach_pressure_threshold_n" not in loaded
                    and "approach_pressure_threshold_g" in loaded
                ):
                    settings["approach_pressure_threshold_n"] = round(
                        float(loaded["approach_pressure_threshold_g"])
                        * GF_TO_NEWTON,
                        2,
                    )
                if "auto_test_target_n" not in loaded and "auto_test_target_g" in loaded:
                    converted_target = (
                        float(loaded["auto_test_target_g"]) * GF_TO_NEWTON
                    )
                    settings["auto_test_target_n"] = min(
                        TARGET_FORCES_N,
                        key=lambda value: abs(value - converted_target),
                    )
                if (
                    "sweep_force_interval_n" not in loaded
                    and "sweep_force_interval_g" in loaded
                ):
                    settings["sweep_force_interval_n"] = max(
                        MAP_FORCE_STEP_N,
                        round(
                            float(loaded["sweep_force_interval_g"])
                            * GF_TO_NEWTON
                            / MAP_FORCE_STEP_N
                        )
                        * MAP_FORCE_STEP_N,
                    )
                if "sweep_max_force_n" not in loaded and "sweep_max_force_g" in loaded:
                    interval_n = float(settings["sweep_force_interval_n"])
                    converted_maximum = (
                        float(loaded["sweep_max_force_g"]) * GF_TO_NEWTON
                    )
                    settings["sweep_max_force_n"] = min(
                        MAX_SWEEP_FORCE_N,
                        max(
                            interval_n,
                            round(converted_maximum / interval_n) * interval_n,
                        ),
                    )
                # 旧版“每档重复”数量迁移为新版完整升降往返轮数。
                if (
                    "sweep_cycle_count" not in loaded
                    and "sweep_repeats_per_level" in loaded
                ):
                    settings["sweep_cycle_count"] = int(
                        loaded["sweep_repeats_per_level"]
                    )
                if not any(
                    key in loaded
                    for key in (
                        "contact_before_step_counts",
                        "contact_after_step_counts",
                        "approach_step_counts",
                    )
                ) and float(settings["approach_pressure_threshold_n"]) <= float(
                    TARGET_TOLERANCE_N
                ):
                    settings["approach_pressure_threshold_n"] = 0.30
        except (OSError, ValueError, TypeError, OverflowError):
            pass
        if float(settings["auto_test_target_n"]) not in TARGET_FORCES_N:
            settings["auto_test_target_n"] = TARGET_FORCES_N[0]
        try:
            interval = float(settings['auto_test_interval_seconds'])
            settings['auto_test_interval_seconds'] = min(3600.0, max(0.0, interval)) if math.isfinite(interval) else 0.0
        except (ValueError, TypeError, OverflowError):
            settings['auto_test_interval_seconds'] = 0.0
        return settings

    def _save_settings_if_idle(self, *_args):
        if self._running or self._motor_control.is_operation_busy():
            return
        self._update_settings_snapshot()
        settings = {
            "press_speed_level": self.press_speed_input.value(),
            "release_speed_level": self.release_speed_input.value(),
            "acceleration_r_s2": self.acceleration_input.value(),
            "max_current_ma": self.max_current_input.value(),
            "auto_test_target_n": round(
                float(self.auto_test_target_input.currentData()), 2
            ),
            "auto_test_rounds": self.auto_test_rounds_input.value(),
            "auto_test_interval_seconds": self.auto_test_interval_input.value(),
            "sweep_max_force_n": self.sweep_max_force_input.value(),
            "sweep_force_interval_n": self.sweep_force_interval_input.value(),
            "sweep_cycle_count": self.sweep_repeats_input.value(),
            "sweep_idle_stable_seconds": self.sweep_idle_stable_input.value(),
            "sweep_idle_timeout_seconds": self.sweep_idle_timeout_input.value(),
        }
        try:
            self._settings_path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = self._settings_path.with_suffix(".json.tmp")
            temporary_path.write_text(
                json.dumps(settings, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary_path.replace(self._settings_path)
        except OSError as exc:
            self._set_status(f"按力参数保存失败：{exc}")

    @staticmethod
    def _add_parameter(grid, label, widget, unit, row, column):
        grid.addWidget(QtWidgets.QLabel(label), row, column)
        line = QtWidgets.QHBoxLayout()
        line.setContentsMargins(0, 0, 0, 0)
        line.setSpacing(5)
        widget.setFixedWidth(90)
        line.addWidget(widget)
        line.addWidget(QtWidgets.QLabel(unit))
        line.addStretch(1)
        grid.addLayout(line, row, column + 1)

    @Slot(object)
    def set_pressure_sample(self, sample):
        """接收压力计串口样本；无效样本立即撤销闭环可用值。"""
        sample = dict(sample or {})
        with self._state_lock:
            self._latest_ocr_event = sample
            if sample.get("invalidate"):
                self._accepted_force_n = None
                self._accepted_control_force_n = None
                self._accepted_force_at = 0.0
            direct_force = sample.get("force_n", sample.get("filtered_force_n"))
            if sample.get("accepted") and direct_force is not None:
                self._accepted_force_n = float(direct_force)
                control_force = sample.get(
                    "control_raw_force_n",
                    sample.get("raw_force_n"),
                )
                if control_force is not None:
                    self._accepted_control_force_n = float(control_force)
                self._accepted_force_at = float(
                    sample.get("sampled_monotonic", time.monotonic())
                )
                self._accepted_force_sequence = int(
                    sample.get("sequence", self._accepted_force_sequence + 1)
                )
            else:
                self._rejected_ocr_count += 1
                self._consecutive_rejected_ocr_count += 1
                if self._running:
                    self._operation_rejected_ocr_count += 1
            if sample.get("accepted"):
                self._consecutive_rejected_ocr_count = 0
            accepted_force = self._accepted_force_n
            accepted_at = self._accepted_force_at
            consecutive_rejected = self._consecutive_rejected_ocr_count
            operation_rejected = self._operation_rejected_ocr_count
            total_rejected = self._rejected_ocr_count
        raw = sample.get("raw_force_n")
        raw_text = "--" if raw is None else f"{float(raw):.2f}"
        filtered_text = "--" if accepted_force is None else f"{accepted_force:.2f}"
        age_text = "--" if accepted_at <= 0 else f"{(time.monotonic() - accepted_at) * 1000.0:.0f}"
        self.force_value_label.setText(f"当前压力计：{filtered_text} N｜原始 {raw_text} N")
        accepted = bool(sample.get("accepted"))
        if accepted:
            if (
                raw is not None
                and not pressure_n_is_within_press_start_tolerance(raw)
                and not self._running
            ):
                qualifier = "负压力" if float(raw) < 0.0 else "非零压力"
                self.ocr_warning_label.setText(
                    f"⚠ 压力计检测到{qualifier} {float(raw):.2f} N。"
                    f"已超出启动范围 -{PRESSURE_METER_NEGATIVE_LIMIT_N:.2f}～"
                    f"+{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N；"
                    "开始下压或自动测试前，请先在压力计上执行清零。"
                )
                self.ocr_warning_label.setVisible(True)
            else:
                self.ocr_warning_label.clear()
                self.ocr_warning_label.setVisible(False)
        else:
            reason = sample.get("rejection_reason") or "没有有效的串口压力值"
            self.ocr_warning_label.setText(
                f"压力计异常：{reason}｜距上次有效值 {age_text} ms｜"
                f"连续丢弃 {consecutive_rejected}｜本次 {operation_rejected}｜"
                f"累计 {total_rejected}"
            )
            self.ocr_warning_label.setVisible(True)
        self._record_event(
            "pressure_meter_sample",
            raw_pressure_n="" if raw is None else raw,
            filtered_force_n=("" if accepted_force is None else accepted_force),
            ocr_accepted=accepted,
            ocr_rejection_reason=sample.get("rejection_reason", ""),
            ocr_failure_image_path=sample.get("failure_image_path", ""),
            ocr_text=sample.get("raw_text", ""),
        )

    def _ocr_snapshot(self):
        with self._state_lock:
            event = dict(self._latest_ocr_event or {})
            accepted_control_force = self._accepted_control_force_n
            filtered_force = self._accepted_force_n
            accepted_at = self._accepted_force_at
            accepted_sequence = self._accepted_force_sequence
        event.update(
            {
                "accepted_force_n": accepted_control_force,
                "accepted_filtered_force_n": filtered_force,
                "accepted_at": accepted_at,
                "accepted_sequence": accepted_sequence,
                "accepted_age_seconds": (
                    math.inf
                    if accepted_at <= 0
                    else max(0.0, time.monotonic() - accepted_at)
                ),
            }
        )
        return event

    @Slot(object)
    def _on_motor_feedback(self, feedback):
        try:
            running_state = int((feedback or {}).get("running_state"))
        except (TypeError, ValueError):
            running_state = None
        state_changed = running_state != self._last_motor_running_state
        self._last_motor_running_state = running_state
        self._show_motor_feedback(feedback)
        self._record_event("motor_sample", **dict(feedback or {}))
        if state_changed:
            self._refresh_controls()

    def _show_motor_feedback(self, feedback):
        position = feedback.get("position_counts")
        position_deg = feedback.get("position_deg")
        speed_rpm = feedback.get("measured_speed_rpm")
        speed_counts = feedback.get("position_speed_counts_s")
        acceleration = feedback.get("estimated_acceleration_r_s2")
        current = feedback.get("phase_current_ma")
        state = feedback.get("running_state_name", "--")
        position_text = "--" if position is None else str(int(position))
        degree_text = "--" if position_deg is None else f"{float(position_deg):.3f}"
        rpm_text = "--" if speed_rpm is None else str(int(speed_rpm))
        speed_text = "--" if speed_counts is None else f"{float(speed_counts):.1f}"
        acceleration_text = "--" if acceleration is None else f"{float(acceleration):.3f}"
        current_text = "--" if current is None else str(int(current))
        self.position_value_label.setText(f"位置 {position_text} counts · {degree_text}°")
        self.speed_value_label.setText(f"速度 {rpm_text} RPM · {speed_text} counts/s")
        self.acceleration_value_label.setText(f"估算加速度 {acceleration_text} r/s²")
        self.current_value_label.setText(
            f"瞬时相电流 {current_text} mA / 限制 {self.max_current_input.value()} mA"
        )
        self.state_value_label.setText(f"电机状态 {state}")

    def _operation_context(self):
        with self._state_lock:
            context = {
                "operation_id": self._operation_id,
                "control_mode": self._control_mode,
                "force_unit": "N",
                "auto_round_index": self._auto_round_index or "",
                "auto_round_total": self._auto_round_total or "",
                "pressure_level_index": self._pressure_level_index or "",
                "pressure_level_total": self._pressure_level_total or "",
                "calibration_cycle_index": self._calibration_cycle_index or "",
                "calibration_cycle_total": self._calibration_cycle_total or "",
                "sweep_direction": self._sweep_direction,
                "direction_step_index": self._direction_step_index or "",
                "direction_step_total": self._direction_step_total or "",
                "repeat_index": self._repeat_index or "",
                "repeat_total": self._repeat_total or "",
                "attempt_index": self._attempt_index or "",
                "target_force_n": "" if self._target_force_n is None else self._target_force_n,
                "command_counts": self._command_counts,
                "total_travel_counts": self._total_travel_counts,
            }
            context.update(self._settings_snapshot)
            return context

    @Slot()
    def _update_settings_snapshot(self, *_args):
        settings = {
            "auto_test_interval_seconds": self.auto_test_interval_input.value(),
            "max_current_ma": self.max_current_input.value(),
            "press_speed_level": self.press_speed_input.value(),
            "release_speed_level": self.release_speed_input.value(),
            "command_acceleration_r_s2": self.acceleration_input.value(),
            "max_travel_counts": MAX_OPERATION_TRAVEL_COUNTS,
            "sweep_idle_stable_seconds": self.sweep_idle_stable_input.value(),
            "sweep_idle_timeout_seconds": self.sweep_idle_timeout_input.value(),
        }
        with self._state_lock:
            self._settings_snapshot = settings
        latest = self._motor_control.latest_motor_feedback()
        if latest is not None:
            self._show_motor_feedback(latest)
        else:
            self.current_value_label.setText(
                f"瞬时相电流 -- mA / 限制 {settings['max_current_ma']} mA"
            )

    def _set_motion_context(self, command_counts=None, total_travel_counts=None):
        with self._state_lock:
            if command_counts is not None:
                self._command_counts = int(command_counts)
            if total_travel_counts is not None:
                self._total_travel_counts = int(total_travel_counts)

    def _record_event(self, event, **values):
        # 控制线程与界面定时器共用一个采样节奏，避免三类周期事件重复写入快照。
        with self._log_sampling_lock:
            self._record_sampled_event(event, **values)

    def _record_sampled_event(self, event, **values):
        if not self._recorder.is_recording:
            return
        sensor_sample = values.pop("_sensor_sample", None)
        if sensor_sample is None:
            sensor_sample = self._latest_sensor_sample()
        row = self._operation_context()
        feedback = self._motor_control.latest_motor_feedback() or {}
        ocr = self._ocr_snapshot()
        busy = self._running or self._motor_control.is_operation_busy()
        sampling = self._log_sampling.select(event, sensor_sample, feedback, ocr, busy, time.monotonic())
        if sampling is None:
            return
        if not sampling['sensor_snapshot_fresh']:
            sensor_sample = None
        elif event in MotionLogSampling.PERIODIC:
            sequence = int(sensor_sample['sequence'])
            values['sensor_sequence_gap'] = ('' if self._last_recorded_sensor_sequence is None else
                                            max(0, sequence-self._last_recorded_sensor_sequence-1))
            self._last_recorded_sensor_sequence = sequence
        if event in MotionLogSampling.PERIODIC and 'sensor_state_changed' in sampling['sensor_record_reason']:
            event = 'sensor_state_change'
        row.update(feedback)
        row.update(self._sensor_log_values(sensor_sample))
        row.update(
            {
                "event": event,
                "raw_pressure_n": (
                    "" if ocr.get("raw_force_n") is None else ocr.get("raw_force_n")
                ),
                "filtered_force_n": (
                    ""
                    if ocr.get("accepted_filtered_force_n") is None
                    else ocr.get("accepted_filtered_force_n")
                ),
                "ocr_accepted": ocr.get("accepted", ""),
                "ocr_rejection_reason": ocr.get("rejection_reason", ""),
                "ocr_failure_image_path": ocr.get("failure_image_path", ""),
                "ocr_age_ms": (
                    ""
                    if not math.isfinite(ocr["accepted_age_seconds"])
                    else round(ocr["accepted_age_seconds"] * 1000.0, 3)
                ),
                "ocr_text": ocr.get("raw_text", ""),
            }
        )
        row.update(values)
        row.update(sampling)
        self._recorder.record(row)

    def _latest_sensor_sample(self):
        if not callable(self._sensor_sample_provider):
            return None
        try:
            sample = self._sensor_sample_provider()
        except Exception:
            return None
        return None if sample is None else dict(sample)

    @Slot()
    def reload_signal_pressure_mapping(self):
        """标定数据变化时刷新界面；主力值始终采用MCU上传结果。"""
        sample = self._latest_sensor_sample()
        if sample is not None:
            self._update_sensor_force_display(sample)

    def _update_mcu_force_value(self, sample):
        """显示MCU持续换算并通过Type 0x02上传的当前法向力。"""
        vector = dict((sample or {}).get("mcu_force_vector") or {})
        age_seconds = (sample or {}).get("mcu_force_age_seconds")
        if not vector or age_seconds is None:
            self.mcu_force_value_label.setText("--.-- N")
            self.mcu_force_state_label.setText("等待 MCU Type 0x02 力值")
            self._set_mcu_force_display_state("waiting")
            return False
        if float(age_seconds) > AUTO_SENSOR_MAX_AGE_SECONDS:
            self.mcu_force_value_label.setText("--.-- N")
            self.mcu_force_state_label.setText(
                f"MCU 力值已超时（{float(age_seconds) * 1000.0:.0f} ms）"
            )
            self._set_mcu_force_display_state("stale")
            return False
        low_display = low_force_display(vector)
        if low_display is not None:
            text, reason, state = low_display
            self.mcu_force_value_label.setText(text)
            self.mcu_force_state_label.setText(reason)
            self._set_mcu_force_display_state(state)
            return state == "valid"
        if not vector.get("normal_calibration_valid"):
            self.mcu_force_value_label.setText("--.-- N")
            self.mcu_force_state_label.setText("Flash K表无效，禁止按 N 使用")
            self._set_mcu_force_display_state("invalid")
            return False
        try:
            normal_force_n = float(vector["normal_force"])
        except (KeyError, TypeError, ValueError):
            self.mcu_force_value_label.setText("--.-- N")
            self.mcu_force_state_label.setText("Type 0x02 法向力字段无效")
            self._set_mcu_force_display_state("invalid")
            return False
        self.mcu_force_value_label.setText(f"{max(0.0, normal_force_n):.2f} N")
        self.mcu_force_state_label.setText(
            f"Flash K表有效 · MCU连续换算 · {float(age_seconds) * 1000.0:.0f} ms"
        )
        self._set_mcu_force_display_state("valid")
        return True

    def _set_mcu_force_display_state(self, state):
        """只在状态切换时更新颜色，避免10毫秒刷新周期反复触发Qt样式计算。"""
        if state == self._mcu_force_display_state:
            return
        colors = {
            "waiting": ("#ffc857", "#ffc857"),
            "stale": ("#ff9f43", "#ff9f43"),
            "invalid": ("#ff6b6b", "#ff6b6b"),
            "over_range": ("#ffb454", "#ffb454"),
            "valid": ("#eaff72", "#69f0ae"),
        }
        value_color, state_color = colors[state]
        self.mcu_force_value_label.setStyleSheet(f"color:{value_color};")
        self.mcu_force_state_label.setStyleSheet(f"color:{state_color};")
        self._mcu_force_display_state = state

    def _update_sensor_force_display(self, sample):
        self._update_mcu_force_value(sample)
        force_values = self._sensor_force_values(sample)
        if force_values is None:
            self.floating_force_label.setText(
                "通道诊断：CH4～CH7 Signal 或状态数据不完整"
            )
            return False
        selected_signals = force_values["signals"]
        signal_sum = force_values["signal_sum"]
        floating_count = force_values["floating_count"]
        state_text = (
            "FLOATING 4/4"
            if floating_count == len(SENSOR_CHANNELS)
            else f"FLOATING {floating_count}/{len(SENSOR_CHANNELS)}"
        )
        channel_text = " · ".join(
            f"CH{channel} {value}"
            for channel, value in zip(SENSOR_CHANNELS, selected_signals)
        )
        self.floating_force_label.setText(
            f"通道诊断：Signal和 {signal_sum} · {state_text} · {channel_text}"
        )
        return True

    @staticmethod
    def _sensor_channel_states(sample):
        if not sample:
            return None
        states = tuple(sample.get("state") or ())
        if max(SENSOR_CHANNELS) >= len(states):
            return None
        try:
            return tuple(int(states[channel]) for channel in SENSOR_CHANNELS)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _sensor_force_values(cls, sample):
        selected_states = cls._sensor_channel_states(sample)
        if selected_states is None:
            return None
        signals = tuple(sample.get("signal") or ())
        if max(SENSOR_CHANNELS) >= len(signals):
            return None
        try:
            selected_signals = tuple(
                int(signals[channel]) for channel in SENSOR_CHANNELS
            )
        except (TypeError, ValueError):
            return None
        return {
            "signals": selected_signals,
            "signal_sum": sum(selected_signals),
            "floating_count": sum(
                state == SENSOR_STATE_FLOATING for state in selected_states
            ),
        }

    @staticmethod
    def _signal_sum_statistics(round_results):
        values = []
        for entry in round_results or ():
            try:
                value = float(entry["signal_sum"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
        if not values:
            return None
        minimum = min(values)
        maximum = max(values)
        average = statistics.fmean(values)
        span = maximum - minimum
        return {
            "count": len(values),
            "average": average,
            "standard_deviation": (
                statistics.stdev(values) if len(values) >= 2 else 0.0
            ),
            "minimum": minimum,
            "maximum": maximum,
            "span": span,
            "span_percent": (
                None if math.isclose(average, 0.0, abs_tol=1e-12)
                else span / abs(average) * 100.0
            ),
        }

    @staticmethod
    def _format_signal_sum_statistics(signal_statistics):
        if not signal_statistics:
            return ""
        span_percent = signal_statistics.get("span_percent")
        span_percent_text = (
            "--" if span_percent is None else f"{float(span_percent):.2f}%"
        )
        return (
            f"Signal合力统计（{signal_statistics['count']}轮）｜"
            f"均值 {signal_statistics['average']:.2f}｜"
            f"标准差 {signal_statistics['standard_deviation']:.2f}｜"
            f"极差 {signal_statistics['span']:.0f} "
            f"（{signal_statistics['minimum']:.0f}～"
            f"{signal_statistics['maximum']:.0f}，"
            f"相对{span_percent_text}）"
        )

    @staticmethod
    def _numeric_statistics(values):
        finite_values = []
        for value in values:
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(number):
                finite_values.append(number)
        if not finite_values:
            return None
        minimum = min(finite_values)
        maximum = max(finite_values)
        average = statistics.fmean(finite_values)
        span = maximum - minimum
        return {
            "count": len(finite_values),
            "average": average,
            "standard_deviation": (
                statistics.stdev(finite_values)
                if len(finite_values) >= 2
                else 0.0
            ),
            "minimum": minimum,
            "maximum": maximum,
            "span": span,
            "span_percent": (
                None if math.isclose(average, 0.0, abs_tol=1e-12)
                else span / abs(average) * 100.0
            ),
        }

    @classmethod
    def _raw_median_statistics(cls, round_results):
        channel_statistics = {}
        for channel_index, channel in enumerate(SENSOR_CHANNELS):
            channel_statistics[channel] = cls._numeric_statistics(
                entry.get("raw_medians", ())[channel_index]
                for entry in round_results or ()
                if len(entry.get("raw_medians", ())) == len(SENSOR_CHANNELS)
            )
        sum_statistics = cls._numeric_statistics(
            entry.get("raw_median_sum") for entry in round_results or ()
        )
        sample_counts = [
            int(entry.get("raw_median_sample_count", 0) or 0)
            for entry in round_results or ()
            if int(entry.get("raw_median_sample_count", 0) or 0) > 0
        ]
        if not sum_statistics or any(
            value is None for value in channel_statistics.values()
        ):
            return None
        return {
            "count": sum_statistics["count"],
            "channels": channel_statistics,
            "sum": sum_statistics,
            "minimum_sample_count": min(sample_counts) if sample_counts else 0,
            "maximum_sample_count": max(sample_counts) if sample_counts else 0,
        }

    @staticmethod
    def _format_raw_median_statistics(norm_statistics):
        if not norm_statistics:
            return ""
        channel_parts = []
        for channel in SENSOR_CHANNELS:
            values = norm_statistics["channels"][channel]
            span_percent = values.get("span_percent")
            span_percent_text = (
                "--" if span_percent is None else f"{float(span_percent):.2f}%"
            )
            channel_parts.append(
                f"CH{channel}极差 {values['span']:.1f}"
                f"（{values['minimum']:.1f}～{values['maximum']:.1f}，"
                f"相对{span_percent_text}）"
            )
        sample_min = norm_statistics["minimum_sample_count"]
        sample_max = norm_statistics["maximum_sample_count"]
        sample_text = (
            f"每轮{sample_min}帧"
            if sample_min == sample_max
            else f"每轮{sample_min}～{sample_max}帧"
        )
        sum_values = norm_statistics["sum"]
        sum_span_percent = sum_values.get("span_percent")
        sum_span_percent_text = (
            "--"
            if sum_span_percent is None
            else f"{float(sum_span_percent):.2f}%"
        )
        return (
            f"Raw 1秒中位值统计（{norm_statistics['count']}轮，{sample_text}）｜"
            + "｜".join(channel_parts)
            + f"｜四路和极差 {sum_values['span']:.1f}"
            f"（{sum_values['minimum']:.1f}～{sum_values['maximum']:.1f}，"
            f"相对{sum_span_percent_text}）"
        )

    @staticmethod
    def _raw_median_log_values(window_result):
        medians = tuple(window_result.get("raw_medians") or ())
        values = {
            "raw_median_window_seconds": window_result.get(
                "raw_median_window_seconds",
                "",
            ),
            "raw_median_sample_count": window_result.get(
                "raw_median_sample_count",
                "",
            ),
            "ch4567_raw_1s_median_sum": window_result.get(
                "raw_median_sum",
                "",
            ),
        }
        for channel_index, channel in enumerate(SENSOR_CHANNELS):
            values[f"ch{channel}_raw_1s_median"] = (
                medians[channel_index]
                if channel_index < len(medians)
                else ""
            )
        return values

    @classmethod
    def _sensor_sample_is_current(cls, sample):
        if cls._sensor_force_values(sample) is None:
            return False
        if int(sample.get("channel_count", 0) or 0) < 8:
            return False
        age_seconds = sample.get("age_seconds")
        return (
            sample.get("sequence") is not None
            and age_seconds is not None
            and float(age_seconds) <= AUTO_SENSOR_MAX_AGE_SECONDS
        )

    def _append_force_mapping_log(self, result):
        sample = self._latest_sensor_sample()
        force_values = self._sensor_force_values(sample)
        if force_values is None:
            return False
        self._update_sensor_force_display(sample)
        entry = dict(result)
        entry.update(force_values)
        self._append_force_mapping_entry(entry)
        return True

    @Slot(object)
    def _append_force_mapping_entry(self, entry):
        entry = dict(entry)
        entry["force_unit"] = "N"
        entry["captured_at"] = str(
            entry.get("captured_at")
            or datetime.now().astimezone().isoformat(timespec="milliseconds")
        )
        entry["signals"] = tuple(entry.get("signals") or ())
        entry["raw_medians"] = tuple(entry.get("raw_medians") or ())
        entry["control_mode"] = str(
            entry.get("control_mode") or entry.get("mode") or ""
        )
        mode_text = {
            "press": "单次下压",
            "auto": "自动重复测试",
            "sweep": "传感器标定",
            "validation": "仅验证（不标定）",
            "quick_correction": "单轮快速K修正",
        }.get(entry["control_mode"], entry["control_mode"] or "记录")
        target_force = float(entry["target_force_n"])
        final_force = float(entry["final_force_n"])
        round_index = int(entry.get("auto_round_index", 0) or 0)
        round_total = int(entry.get("auto_round_total", 0) or 0)
        level_index = int(entry.get("pressure_level_index", 0) or 0)
        level_total = int(entry.get("pressure_level_total", 0) or 0)
        cycle_index = int(entry.get("calibration_cycle_index", 0) or 0)
        cycle_total = int(entry.get("calibration_cycle_total", 0) or 0)
        sweep_direction = str(entry.get("sweep_direction", ""))
        direction_text = {
            "ascending": "升序",
            "descending": "降序",
        }.get(sweep_direction, sweep_direction)
        direction_step_index = int(entry.get("direction_step_index", 0) or 0)
        direction_step_total = int(entry.get("direction_step_total", 0) or 0)
        repeat_index = int(entry.get("repeat_index", 0) or 0)
        repeat_total = int(entry.get("repeat_total", 0) or 0)
        if (
            level_index
            and level_total
            and cycle_index
            and cycle_total
            and direction_text
        ):
            round_text = (
                f"往返轮 {cycle_index}/{cycle_total} · {direction_text} "
                f"{direction_step_index}/{direction_step_total} · "
                f"档位 {level_index}/{level_total}  |  "
            )
        elif level_index and level_total and repeat_index and repeat_total:
            round_text = (
                f"档位 {level_index}/{level_total} · "
                f"重复 {repeat_index}/{repeat_total}  |  "
            )
        elif round_index and round_total:
            round_text = f"第 {round_index}/{round_total} 轮  |  "
        else:
            round_text = ""
        channel_text = " · ".join(
            f"CH{channel} {value}"
            for channel, value in zip(SENSOR_CHANNELS, entry["signals"])
        )
        record_text = (
            f"{datetime.now():%H:%M:%S}  |  [{mode_text}]  |  "
            f"{round_text}目标 {target_force:.2f} N"
            f" → 最终压力 {final_force:.2f} N"
            f"  |  Signal合力 {entry['signal_sum']}"
            f"  |  {channel_text}"
            f"  |  FLOATING {entry['floating_count']}/{len(SENSOR_CHANNELS)}"
        )
        arrival_position = entry.get("press_arrival_position_counts")
        floating_position = entry.get("floating_capture_position_counts")
        target_position = entry.get("lookup_target_position_counts")
        if (
            arrival_position not in (None, "")
            and floating_position not in (None, "")
            and target_position not in (None, "")
        ):
            record_text += (
                f"\n    电机位置：目标 {int(target_position)} counts"
                f"  |  下压判定 {int(arrival_position)}"
                f"（误差 {int(entry['press_arrival_position_error_counts']):+d}）"
                f"  |  FLOATING采样 {int(floating_position)}"
                f"（误差 {int(entry['floating_position_error_counts']):+d}）"
                f"  |  驱动器状态 {int(entry['floating_running_state'])}"
            )
        raw_medians = tuple(entry.get("raw_medians") or ())
        if len(raw_medians) == len(SENSOR_CHANNELS):
            norm_text = " · ".join(
                f"CH{channel} {value:g}"
                for channel, value in zip(SENSOR_CHANNELS, raw_medians)
            )
            record_text += (
                f"\n    Raw 1秒中位值：{norm_text}"
                f"  |  四路和 {entry['raw_median_sum']:g}"
                f"  |  {entry['raw_median_sample_count']}帧"
            )
        self.force_mapping_log.appendPlainText(record_text)
        self._force_mapping_entries.append(entry)
        if entry["control_mode"] == "sweep":
            self._sweep_entries.append(entry)
            self.save_sweep_log_button.setEnabled(True)
        elif entry["control_mode"] == "validation":
            self._validation_entries.append(entry)
            self.save_validation_log_button.setEnabled(True)
        self._force_mapping_log_count += 1
        self.force_mapping_log_count_label.setText(
            f"共 {self._force_mapping_log_count} 条"
        )
        self.force_mapping_group.setTitle(
            f"采样记录（{self._force_mapping_log_count} 条）"
        )
        self.save_force_mapping_log_button.setEnabled(True)
        self.clear_force_mapping_log_button.setEnabled(True)
        scrollbar = self.force_mapping_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    @staticmethod
    def _force_mapping_export_row(entry):
        signals = tuple(entry.get("signals") or ())
        medians = tuple(entry.get("raw_medians") or ())
        row = {
            "captured_at": entry.get("captured_at", ""),
            "operation_id": entry.get("operation_id", ""),
            "control_mode": entry.get("control_mode", entry.get("mode", "")),
            "force_unit": "N",
            "target_force_n": entry.get("target_force_n", ""),
            "lookup_created_at": entry.get("lookup_created_at", ""),
            "release_reference_position_counts": entry.get(
                "release_reference_position_counts",
                "",
            ),
            "lookup_target_position_counts": entry.get(
                "lookup_target_position_counts",
                "",
            ),
            "press_arrival_position_counts": entry.get(
                "press_arrival_position_counts",
                "",
            ),
            "press_arrival_position_error_counts": entry.get(
                "press_arrival_position_error_counts",
                "",
            ),
            "press_position_correction_counts": entry.get(
                "press_position_correction_counts",
                "",
            ),
            "press_position_error_before_correction_counts": entry.get(
                "press_position_error_before_correction_counts",
                "",
            ),
            "press_position_error_after_correction_counts": entry.get(
                "press_position_error_after_correction_counts",
                "",
            ),
            "floating_capture_position_counts": entry.get(
                "floating_capture_position_counts",
                "",
            ),
            "floating_position_error_counts": entry.get(
                "floating_position_error_counts",
                "",
            ),
            "floating_running_state": entry.get(
                "floating_running_state",
                "",
            ),
            "press_speed_level": entry.get("press_speed_level", ""),
            "release_speed_level": entry.get("release_speed_level", ""),
            "acceleration_r_s2": entry.get("acceleration_r_s2", ""),
            "max_current_ma": entry.get("max_current_ma", ""),
            "pressure_level_index": entry.get("pressure_level_index", ""),
            "pressure_level_total": entry.get("pressure_level_total", ""),
            "calibration_cycle_index": entry.get(
                "calibration_cycle_index",
                "",
            ),
            "calibration_cycle_total": entry.get(
                "calibration_cycle_total",
                "",
            ),
            "sweep_direction": entry.get("sweep_direction", ""),
            "direction_step_index": entry.get("direction_step_index", ""),
            "direction_step_total": entry.get("direction_step_total", ""),
            "repeat_index": entry.get("repeat_index", ""),
            "repeat_total": entry.get("repeat_total", ""),
            "attempt_index": entry.get("attempt_index", ""),
            "auto_round_index": entry.get("auto_round_index", ""),
            "auto_round_total": entry.get("auto_round_total", ""),
            "press_arrival_force_n": entry.get("press_arrival_force_n", ""),
            "final_force_n": entry.get("final_force_n", ""),
            "release_force_n": entry.get("release_force_n", ""),
            "release_idle_recovered": entry.get(
                "release_idle_recovered",
                "",
            ),
            "release_idle_wait_seconds": entry.get(
                "release_idle_wait_seconds",
                "",
            ),
            "release_idle_stable_seconds": entry.get(
                "release_idle_stable_seconds",
                "",
            ),
            "release_idle_timeout_seconds": entry.get(
                "release_idle_timeout_seconds",
                "",
            ),
            "ch4567_signal_sum": entry.get("signal_sum", ""),
            "floating_count": entry.get("floating_count", ""),
            "raw_median_window_seconds": entry.get(
                "raw_median_window_seconds",
                "",
            ),
            "raw_median_sample_count": entry.get(
                "raw_median_sample_count",
                "",
            ),
            "ch4567_raw_1s_median_sum": entry.get(
                "raw_median_sum",
                "",
            ),
        }
        for channel_index, channel in enumerate(SENSOR_CHANNELS):
            row[f"ch{channel}_signal"] = (
                signals[channel_index] if channel_index < len(signals) else ""
            )
            row[f"ch{channel}_raw_1s_median"] = (
                medians[channel_index] if channel_index < len(medians) else ""
            )
        for field_name in PRESSURE_N_CSV_FIELDS:
            value = row.get(field_name)
            if value not in (None, ""):
                row[field_name] = f"{float(value):.2f}"
        return row

    def _save_force_entries(self, entries, filename_prefix, description):
        entries = tuple(entries)
        if not entries:
            self._set_status(f"当前没有可保存的{description}")
            return None
        output_dir = force_press_test_directory()
        namespace = getattr(getattr(self, "_profile", None), "namespace", "")
        if namespace:
            output_dir = output_dir / namespace
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = output_dir / f"{filename_prefix}_{stamp}.csv"
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8-sig", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=AUTO_RESULT_FIELD_NAMES)
                writer.writeheader()
                for entry in entries:
                    writer.writerow(self._force_mapping_export_row(entry))
        except OSError as exc:
            self._set_status(f"{description}保存失败：{exc}")
            return None
        self._record_event(
            "force_sensor_results_saved",
            message=f"已单独保存{len(entries)}条{description}：{path}",
        )
        self._set_status(f"已单独保存 {len(entries)} 条{description}：{path}")
        return path

    @Slot()
    def save_force_mapping_log(self):
        self._save_force_entries(
            self._force_mapping_entries,
            "force_sensor_results",
            "压力－传感器记录",
        )

    @Slot()
    def save_sweep_log(self):
        self._save_force_entries(
            self._sweep_entries,
            "force_sensor_calibration_results",
            "传感器标定记录",
        )

    @Slot()
    def save_validation_log(self):
        self._save_force_entries(
            self._validation_entries,
            "force_sensor_validation_results",
            "多压力验证记录",
        )

    @staticmethod
    def _sweep_entry_identity(entry):
        return (
            str(entry.get("captured_at", "")),
            int(entry.get("calibration_cycle_index", 0) or 0),
            str(entry.get("sweep_direction", "")),
            int(entry.get("pressure_level_index", 0) or 0),
            round(float(entry.get("target_force_n", 0) or 0), 2),
        )

    @staticmethod
    def _retain_completed_sweep_cycles(
        entries,
        cycle_count,
        samples_per_cycle,
    ):
        completed_cycle_count = 0
        for cycle_index in range(1, int(cycle_count) + 1):
            cycle_entry_count = sum(
                int(entry["calibration_cycle_index"]) == cycle_index
                for entry in entries
            )
            if cycle_entry_count != int(samples_per_cycle):
                break
            completed_cycle_count = cycle_index
        retained_entries = [
            entry
            for entry in entries
            if int(entry["calibration_cycle_index"]) <= completed_cycle_count
        ]
        discarded_entries = [
            entry
            for entry in entries
            if int(entry["calibration_cycle_index"]) > completed_cycle_count
        ]
        return completed_cycle_count, retained_entries, discarded_entries

    def _write_sweep_checkpoint(
        self,
        parameters,
        entries,
        status,
        last_error="",
    ):
        plan = dict(parameters["sweep_plan"])
        plan["targets_n"] = [round(float(value), 2) for value in plan["targets_n"]]
        lookup_targets = tuple(parameters.get("sweep_lookup_targets") or ())
        lookup_created_at = (
            str(lookup_targets[0].get("created_at", "--"))
            if lookup_targets
            else str(parameters.get("lookup_created_at", "--"))
        )
        document = {
            "format_version": 4,
            "force_unit": "N",
            "updated_at": datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "status": str(status),
            "last_error": str(last_error),
            "lookup_created_at": lookup_created_at,
            "release_reference_position_counts": int(
                parameters["release_position_counts"]
            ),
            "plan": plan,
            "completed_rounds": len(entries),
            "entries": [dict(entry) for entry in entries],
        }
        path = self._sweep_checkpoint_path()
        temporary_path = Path(f"{path}.tmp")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path.write_text(
                json.dumps(document, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            temporary_path.replace(path)
        except OSError as exc:
            raise ForceControlError(
                "sweep_checkpoint_write_failed",
                f"保存传感器标定恢复记录失败：{exc}",
                brake_required=False,
            ) from exc
        return path

    def _load_sweep_checkpoint(self):
        path = self._sweep_checkpoint_path()
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError("没有找到上次传感器标定恢复记录") from exc
        except (OSError, ValueError, TypeError) as exc:
            raise ValueError(f"读取上次传感器标定恢复记录失败：{exc}") from exc
        if (
            not isinstance(document, dict)
            or document.get("format_version") != 4
            or document.get("force_unit") != "N"
        ):
            raise ValueError(
                "上次记录不是当前48节点两段标定格式；旧记录保留，不能与新计划混合续测"
            )
        plan = document.get("plan")
        entries = document.get("entries")
        if not isinstance(plan, dict) or not isinstance(entries, list):
            raise ValueError("上次传感器标定恢复记录缺少测试计划或结果")
        try:
            maximum_force = round(float(plan["maximum_force_n"]), 2)
            force_interval = round(float(plan["force_interval_n"]), 2)
            cycle_count = int(plan["cycle_count"])
            targets = tuple(round(float(value), 2) for value in plan["targets_n"])
            warmup_force_n = round(
                float(plan.get("warmup_force_n", maximum_force)),
                2,
            )
            idle_stable_seconds = round(
                float(plan.get("sensor_idle_stable_seconds", 1.0)),
                1,
            )
            idle_timeout_seconds = round(
                float(plan.get("sensor_idle_timeout_seconds", 15.0)),
                1,
            )
            release_reference = int(
                document["release_reference_position_counts"]
            )
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError("上次传感器标定恢复记录中的参数无效") from exc
        maximum_steps = int(round(maximum_force / MAP_FORCE_STEP_N))
        interval_steps = int(round(force_interval / MAP_FORCE_STEP_N))
        if maximum_steps <= 0 or interval_steps <= 0:
            raise ValueError("上次传感器标定恢复记录中的压力计划无效")
        expected_targets = FORCE_CALIBRATION_TARGETS_N
        if (
            maximum_force < force_interval
            or maximum_force != 30.0
            or force_interval != 1.0
            or plan.get("low_force_interval_n") != 0.1
            or plan.get("range_boundary_n") != 2.0
            or maximum_force > MAX_SWEEP_FORCE_N
            or force_interval <= 0
            or not math.isclose(
                maximum_force, maximum_steps * MAP_FORCE_STEP_N, abs_tol=1e-9
            )
            or not math.isclose(
                force_interval, interval_steps * MAP_FORCE_STEP_N, abs_tol=1e-9
            )
            or maximum_steps % interval_steps
            or cycle_count <= 0
            or cycle_count > 100
            or warmup_force_n <= 0.0
            or warmup_force_n > MAX_SWEEP_FORCE_N
            or idle_stable_seconds < 0.1
            or idle_stable_seconds > 10.0
            or idle_timeout_seconds < 1.0
            or idle_timeout_seconds < idle_stable_seconds
            or idle_timeout_seconds > 120.0
            or targets != expected_targets
            or targets != FORCE_CALIBRATION_TARGETS_N
            or plan.get("sequence_mode")
            not in (SWEEP_SEQUENCE_LEGACY, SWEEP_SEQUENCE_ALTERNATING)
        ):
            raise ValueError("上次传感器标定恢复记录中的压力计划无效")
        total_rounds = len(targets) * 2 * cycle_count
        if int(plan.get("total_rounds", 0) or 0) != total_rounds:
            raise ValueError("上次传感器标定恢复记录中的采样总数无效")
        normalized_entries = []
        completed_keys = set()
        for raw_entry in entries:
            if not isinstance(raw_entry, dict):
                raise ValueError("上次传感器标定恢复记录中存在无效轮次")
            entry = dict(raw_entry)
            try:
                level_index = int(entry["pressure_level_index"])
                cycle_index = int(entry["calibration_cycle_index"])
                cycle_total = int(entry["calibration_cycle_total"])
                sweep_direction = str(entry["sweep_direction"])
                direction_step_index = int(entry["direction_step_index"])
                direction_step_total = int(entry["direction_step_total"])
                repeat_index = int(entry["repeat_index"])
                repeat_total = int(entry["repeat_total"])
                auto_round_index = int(entry["auto_round_index"])
                auto_round_total = int(entry["auto_round_total"])
                target_force = round(float(entry["target_force_n"]), 2)
                float(entry["final_force_n"])
                float(entry["signal_sum"])
                signals = tuple(entry["signals"])
                raw_medians = tuple(entry["raw_medians"])
                int(entry["floating_count"])
                float(entry["raw_median_sum"])
                int(entry["raw_median_sample_count"])
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                raise ValueError("上次传感器标定恢复记录中存在不完整轮次") from exc
            if (
                not 1 <= level_index <= len(targets)
                or not 1 <= cycle_index <= cycle_count
                or cycle_total != cycle_count
                or sweep_direction not in ("ascending", "descending")
                or not 1 <= direction_step_index <= len(targets)
                or direction_step_total != len(targets)
                or repeat_index != cycle_index
                or repeat_total != cycle_count
                or not 1 <= auto_round_index <= total_rounds
                or auto_round_total != total_rounds
                or target_force != targets[level_index - 1]
                or len(signals) != len(SENSOR_CHANNELS)
                or len(raw_medians) != len(SENSOR_CHANNELS)
            ):
                raise ValueError("上次传感器标定恢复记录中的轮次与计划不一致")
            expected_step_index = (
                level_index
                if sweep_direction == "ascending"
                else len(targets) - level_index + 1
            )
            direction_plans = sweep_direction_plans(
                targets,
                cycle_index,
                str(plan["sequence_mode"]),
            )
            expected_direction_index = next(
                index
                for index, direction_plan in enumerate(direction_plans)
                if direction_plan[0] == sweep_direction
            )
            expected_auto_round_index = (
                (cycle_index - 1) * len(targets) * 2
                + expected_direction_index * len(targets)
                + direction_step_index
            )
            if (
                direction_step_index != expected_step_index
                or auto_round_index != expected_auto_round_index
            ):
                raise ValueError("上次传感器标定恢复记录中的升降顺序无效")
            key = (cycle_index, sweep_direction, level_index)
            if key in completed_keys:
                raise ValueError("上次传感器标定恢复记录中存在重复轮次")
            completed_keys.add(key)
            normalized_entries.append(entry)
        normalized_entries.sort(
            key=lambda entry: int(entry["auto_round_index"])
        )
        samples_per_cycle = len(targets) * 2
        (
            completed_cycle_count,
            retained_entries,
            discarded_entries,
        ) = self._retain_completed_sweep_cycles(
            normalized_entries,
            cycle_count,
            samples_per_cycle,
        )
        retained_keys = {
            (
                int(entry["calibration_cycle_index"]),
                str(entry["sweep_direction"]),
                int(entry["pressure_level_index"]),
            )
            for entry in retained_entries
        }
        return {
            "path": path,
            "status": str(document.get("status", "running")),
            "lookup_created_at": str(document.get("lookup_created_at", "--")),
            "release_reference_position_counts": release_reference,
            "plan": {
                "maximum_force_n": maximum_force,
                "force_interval_n": force_interval,
                "low_force_interval_n": 0.1,
                "range_boundary_n": 2.0,
                "cycle_count": cycle_count,
                "sequence_mode": str(plan["sequence_mode"]),
                "targets_n": targets,
                "level_count": len(targets),
                "passes_per_cycle": 2,
                "samples_per_cycle": len(targets) * 2,
                "total_rounds": total_rounds,
                "warmup_force_n": warmup_force_n,
                "warmup_rounds": WARMUP_ROUNDS,
                "warmup_records_data": False,
                "sensor_idle_stable_seconds": idle_stable_seconds,
                "sensor_idle_timeout_seconds": idle_timeout_seconds,
            },
            "entries": retained_entries,
            "completed_keys": retained_keys,
            "discarded_entries": discarded_entries,
            "original_completed_rounds": len(normalized_entries),
            "completed_cycle_count": completed_cycle_count,
        }

    @Slot()
    def resume_sweep_test(self):
        if self._running or self._motor_control.is_operation_busy():
            self._set_status("电机正在执行操作，不能读取恢复记录")
            return
        try:
            checkpoint = self._load_sweep_checkpoint()
        except ValueError as exc:
            self._set_status(str(exc))
            return
        plan = checkpoint["plan"]
        if len(checkpoint["completed_keys"]) >= plan["total_rounds"]:
            self._set_status("上次传感器标定已经完成，没有未完成轮次")
            return
        document = self._load_lookup_document(show_error=True)
        if document is None:
            return
        metadata = document.get("metadata", {})
        current_created_at = str(metadata.get("created_at", "--"))
        try:
            current_release_reference = resolve_mapping_release_reference(document)
            for target_force in plan["targets_n"]:
                self._resolve_lookup_target(document, target_force)
        except ForcePositionLookupError as exc:
            self._set_status(f"不能继续上次测试：{exc}")
            return
        if (
            current_created_at != checkpoint["lookup_created_at"]
            or current_release_reference
            != checkpoint["release_reference_position_counts"]
        ):
            self._set_status(
                "不能继续上次测试：当前压力－位置查找表或标定出发点"
                "与恢复记录不一致"
            )
            return
        sample = self._latest_sensor_sample()
        if not self._sensor_sample_is_current(sample):
            self._set_status(
                "没有0.5秒内更新的完整CH4～CH7传感器数据，不能继续传感器标定"
            )
            return
        # 恢复任务直接使用历史计划，不覆盖界面中的日常参数及其持久化配置。
        discarded_entries = checkpoint.get("discarded_entries", ())
        discarded_identities = {
            self._sweep_entry_identity(entry) for entry in discarded_entries
        }
        if discarded_identities:
            self._sweep_entries = [
                entry
                for entry in self._sweep_entries
                if self._sweep_entry_identity(entry) not in discarded_identities
            ]
            self._force_mapping_entries = [
                entry
                for entry in self._force_mapping_entries
                if not (
                    str(entry.get("control_mode", "")) == "sweep"
                    and self._sweep_entry_identity(entry)
                    in discarded_identities
                )
            ]
            self._force_mapping_log_count = len(self._force_mapping_entries)
            self.force_mapping_log_count_label.setText(
                f"共 {self._force_mapping_log_count} 条"
            )
            self.force_mapping_group.setTitle(
                f"采样记录（{self._force_mapping_log_count} 条）"
            )
            self.force_mapping_log.appendPlainText(
                "恢复处理：已从后续保存数据中排除未完成往返轮的"
                f"{len(discarded_entries)}条旧记录；界面上方历史文字仅供诊断。"
            )
        existing_identities = {
            self._sweep_entry_identity(entry) for entry in self._sweep_entries
        }
        for entry in checkpoint["entries"]:
            identity = self._sweep_entry_identity(entry)
            if identity not in existing_identities:
                self._append_force_mapping_entry(entry)
                existing_identities.add(identity)
        remaining_rounds = plan["total_rounds"] - len(checkpoint["completed_keys"])
        restart_cycle = checkpoint["completed_cycle_count"] + 1
        restart_direction_plan = sweep_direction_plans(
            plan["targets_n"],
            restart_cycle,
            plan["sequence_mode"],
        )[0]
        restart_direction_text = restart_direction_plan[1]
        restart_force_n = float(restart_direction_plan[2][0])
        self._record_event(
            "sweep_checkpoint_resumed",
            message=(
                f"已读取恢复记录 {checkpoint['path']}，仅本次续测使用历史计划，"
                "界面日常参数保持不变；保留前"
                f"{checkpoint['completed_cycle_count']}个完整往返轮、"
                f"共{len(checkpoint['entries'])}次有效采样；"
                f"丢弃未完成轮{len(discarded_entries)}次旧采样；"
                f"重新完成不记录数据的标定前热身后，从第{restart_cycle}轮"
                f"{restart_direction_text}{restart_force_n:.2f} N重新开始，"
                f"剩余{remaining_rounds}次"
            ),
        )
        self._start_operation(
            "sweep",
            float(plan["targets_n"][0]),
            auto_rounds=plan["total_rounds"],
            sweep_plan=plan,
            sweep_resume_entries=checkpoint["entries"],
        )

    @Slot()
    def clear_force_mapping_log(self):
        self.force_mapping_log.clear()
        self._force_mapping_entries.clear()
        self._sweep_entries.clear()
        self._validation_entries.clear()
        self._force_mapping_log_count = 0
        self.force_mapping_log_count_label.setText("共 0 条")
        self.force_mapping_group.setTitle("采样记录（0 条）")
        self.save_force_mapping_log_button.setEnabled(False)
        self.save_sweep_log_button.setEnabled(False)
        self.save_validation_log_button.setEnabled(False)
        self.clear_force_mapping_log_button.setEnabled(False)

    @staticmethod
    def _sensor_log_values(sample):
        values = {name: "" for name in SENSOR_LOG_FIELD_NAMES}
        if not sample:
            return values

        channel_count = int(sample.get("channel_count", 0) or 0)
        response = sample.get("raw_response") or {}
        result = response.get("result")
        if result and float(sample.get("age_seconds", float("inf"))) <= 0.5:
            values["raw_response_reference_id"] = result["reference_id"]
            values["raw_response_status"] = result["status"]
            for key, field in (("mean", "ch4567_delta_raw_mean"), ("stddev", "ch4567_delta_raw_stddev"),
                               ("cv_percent", "ch4567_delta_raw_cv_percent"), ("minimum_mean", "ch4567_delta_raw_minimum_mean")):
                values[field] = "" if result[key] is None else round(result[key], 6)
            for key, field in (("reference", "reference_hardware_raw"), ("delta", "delta_hardware_raw"), ("shares_percent", "delta_share_percent")):
                for i, channel in enumerate(SENSOR_CHANNELS):
                    values[f"ch{channel}_{field}"] = "" if result[key] is None else round(result[key][i], 6)
        else:
            values["raw_response_status"] = response.get("message", "")
        stats = sample.get("channel_4567_statistics")
        if stats and float(sample.get("age_seconds", float("inf"))) <= 0.5:
            values["ch4567_mcu_hardware_raw_mean"] = stats["hardware_raw_mean"]
            values["ch4567_mcu_hardware_raw_stddev"] = stats["hardware_raw_stddev"]
            values["ch4567_mcu_signal_sum"] = stats["signal_sum"]
        values.update(
            {
                "sensor_sequence": sample.get("sequence", ""),
                "sensor_age_ms": (
                    ""
                    if sample.get("age_seconds") is None
                    else round(float(sample["age_seconds"]) * 1000.0, 3)
                ),
                "sensor_channel_count": channel_count,
                "sensor_has_raw": bool(sample.get("has_raw")),
                "sensor_four_channel_valid": bool(sample.get("has_raw")) and channel_count > max(SENSOR_CHANNELS),
            }
        )

        for field_name in SENSOR_VECTOR_FIELDS:
            # Signal-only帧的其他字段没有上传，兼容旧缓存时也不能记录其补零值。
            if field_name != "signal" and not sample.get("has_raw"):
                continue
            field_values = tuple(sample.get(field_name) or ())
            for channel in SENSOR_CHANNELS:
                if channel < len(field_values):
                    values[f"ch{channel}_{field_name}"] = field_values[channel]

        for field_name in (
            "hardware_raw",
            "baseline",
            "signal",
        ):
            selected_values = [
                values[f"ch{channel}_{field_name}"] for channel in SENSOR_CHANNELS
            ]
            if all(value != "" for value in selected_values):
                values[f"ch4567_{field_name}_sum"] = sum(selected_values)
        return values

    @Slot()
    def _record_latest_sensor_sample(self):
        sample = self._latest_sensor_sample()
        if sample:
            self._update_sensor_force_display(sample)
        if not self._recorder.is_recording:
            return
        self._record_event(
            "sensor_sample",
            _sensor_sample=sample,
        )

    def _start_realtime_recording(self):
        self._last_recorded_sensor_sequence = None
        self._log_sampling = MotionLogSampling()
        try:
            path = self._recorder.start()
        except OSError as exc:
            self.record_status_label.setText(f"运动日志创建失败：{exc}")
            self.record_status_label.setStyleSheet("color:#ff7b72;")
            return
        self.record_status_label.setText(
            "运动日志：通常100 ms一次；八通道全部IDLE且无电机操作满60秒后，每秒一次。"
            "状态变化和关键事件立即保存（每个5 MiB，最多10个）"
        )
        self.record_status_label.setToolTip(str(path.parent))
        self._record_event("recording_started", message="应用启动，实时运动日志已开启")

    def start_press(self, target_force_n):
        return self._start_operation("press", float(target_force_n))

    def manual_control_state(self):
        """共享原按钮的显式可用条件，不受原页面折叠或父组禁用影响。"""
        enabled = lambda button: button.isEnabledTo(button.parentWidget())
        return {
            "targets": {value: enabled(button) for value, button in zip(TARGET_FORCES_N, self._target_buttons)},
            "release": enabled(self.release_button),
            "stop": enabled(self.stop_button),
            "lookup": self.lookup_map_label.text(),
            "status": self._last_status_message,
        }

    @Slot()
    def start_release(self):
        return self._start_operation("release", None)

    def start_creep_test(self, plan):
        """把完整蠕变实验作为一个独占电机任务提交。"""
        plan = dict(plan or {})
        trials = tuple(dict(item) for item in plan.get("trials", ()))
        if not trials:
            self._set_status("长时蠕变测试计划为空")
            return False
        first_trial = trials[0]
        first_force = float(
            first_trial.get("high_force_n")
            or first_trial.get("probe_force_n")
            or 0.0
        )
        plan["trials"] = trials
        return self._start_operation(
            "creep_test",
            first_force,
            auto_rounds=len(trials),
            sweep_plan=plan,
        )

    def last_status_message(self):
        """返回底层最近一次状态详情，供组合测试页面展示准确拒绝原因。"""
        return self._last_status_message

    @Slot()
    def start_warmup(self):
        self._start_operation(
            "warmup",
            float(WARMUP_FORCE_N),
            auto_rounds=WARMUP_ROUNDS,
        )

    def _confirmed_release_position(self):
        snapshot = self._motor_control.release_position_snapshot()
        position = snapshot.get("position_counts")
        if not snapshot.get("confirmed") or position is None:
            return None
        return int(position)

    def _sweep_checkpoint_path(self):
        return force_sweep_checkpoint_path()

    def _resolve_lookup_target(self, document, target_force_n):
        return resolve_force_position_target_n(document, target_force_n)

    def _operation_motion_parameters(self):
        return dict(press_speed_level=self.press_speed_input.value(),
                    release_speed_level=self.release_speed_input.value(),
                    acceleration=self.acceleration_input.value(),
                    max_travel_counts=MAX_OPERATION_TRAVEL_COUNTS,
                    max_current_ma=self.max_current_input.value())

    def _load_lookup_document(self, show_error=True):
        try:
            document = load_latest_mapping_document()
            reference = resolve_mapping_release_reference(document)
        except ForcePositionLookupError as exc:
            self._lookup_document = None
            self._lookup_targets = {}
            if hasattr(self, "lookup_map_label"):
                self.lookup_map_label.setText(f"压力－位置查找表：不可用（{exc}）")
                self.lookup_map_label.setStyleSheet("color:#ff7b72;font-weight:700;")
            if show_error:
                self._set_status(str(exc))
            return None

        resolved_targets = {}
        for target_force in TARGET_FORCES_N:
            try:
                resolved_targets[target_force] = self._resolve_lookup_target(
                    document,
                    target_force,
                )
            except ForcePositionLookupError:
                continue
        self._lookup_document = document
        self._lookup_targets = resolved_targets
        metadata = document.get("metadata", {})
        created_at = metadata.get("created_at", "--")
        self.lookup_map_label.setText(
            f"压力－位置查找表：目标按钮稳定档 {len(resolved_targets)}/"
            f"{len(TARGET_FORCES_N)}；"
            f"标定出发点 {reference} counts；生成时间 {created_at}"
        )
        color = "#5be39b" if len(resolved_targets) == len(TARGET_FORCES_N) else "#ffcc66"
        self.lookup_map_label.setStyleSheet(f"color:{color};font-weight:700;")
        return document

    @Slot(bool)
    def _reload_lookup_when_expanded(self, expanded):
        if expanded:
            self.reload_lookup_mapping()

    @Slot()
    def reload_lookup_mapping(self):
        self._load_lookup_document(show_error=False)
        self._refresh_controls()

    @Slot(bool)
    def set_quick_correction_ram_trial_active(self, active):
        """RAM中有待确认的快调表时，禁止再次启动快调。"""
        self._quick_correction_ram_trial_active = bool(active)
        self.start_quick_correction_button.setText(
            "RAM快调待验证"
            if self._quick_correction_ram_trial_active
            else "两段快速K修正"
        )
        self._refresh_controls()

    @Slot(object)
    def set_quick_correction_ram_result(self, payload):
        """在快调入口旁明确显示MCU RAM写入与回读确认结果。"""
        result = dict(payload or {})
        state = str(result.get("state", "failed"))
        message = str(result.get("message", "未提供结果说明"))
        presentations = {
            "waiting": (
                "MCU RAM写入状态：等待电机任务完全结束，尚未写入",
                "#f4d35e",
            ),
            "writing": (
                "MCU RAM写入状态：正在写入，并等待逐点回读确认",
                "#f4d35e",
            ),
            "verified": (
                "MCU RAM写入状态：成功，逐点回读一致；Flash尚未写入",
                "#5be39b",
            ),
            "flash_saved": (
                "MCU Flash保存状态：成功，复位后回读一致",
                "#5be39b",
            ),
            "discarded": (
                "MCU RAM写入状态：已撤销，当前已恢复Flash原参数",
                "#8bd5ff",
            ),
            "lost": (
                "MCU RAM写入状态：串口已断开，RAM试用结果不可确认",
                "#ff9f43",
            ),
            "failed": (
                "MCU RAM写入状态：失败或未确认，当前不能视为已生效",
                "#ff7b72",
            ),
        }
        summary, color = presentations.get(state, presentations["failed"])
        self.quick_correction_ram_status_label.setText(
            label_soft_wrap_text(f"{summary}\n{message}")
        )
        self.quick_correction_ram_status_label.setStyleSheet(
            "QLabel#ForceQuickCorrectionRamStatus {"
            f"color:{color};background:#101923;border:2px solid {color};"
            "border-radius:5px;padding:7px;font-weight:700;"
            "}"
        )
        self._record_event(
            "quick_correction_ram_transfer_result",
            ram_transfer_state=state,
            message=message,
        )

    def _emit_quick_correction_after_motor_idle(self, payload, attempt=0):
        """确认电机工作线程真正退出后再启动MCU RAM传输。"""
        if self._closing:
            return
        if not self._motor_control.is_operation_busy():
            self.quick_correction_completed.emit(payload)
            return
        if attempt >= 50:
            self.set_quick_correction_ram_result(
                {
                    "state": "failed",
                    "message": (
                        "测试数据已保存，但等待电机线程退出超过5秒；"
                        "为避免运动与参数写入并发，本次未向MCU发送RAM快调表"
                    ),
                }
            )
            return
        QtCore.QTimer.singleShot(
            100,
            lambda data=payload, next_attempt=attempt + 1: (
                self._emit_quick_correction_after_motor_idle(
                    data,
                    next_attempt,
                )
            ),
        )

    @Slot(object)
    def _on_release_position_changed(self, snapshot):
        position = snapshot.get("position_counts")
        if position is None:
            text = "共享标定出发点：尚未设置；请先在力与位置标定页设置"
            color = "#ff7b72"
        elif snapshot.get("confirmed"):
            text = f"共享标定出发点：{int(position)} counts（当前CAN连接已确认）"
            color = "#5be39b"
        else:
            text = (
                f"共享标定出发点：PC已保存 {int(position)} counts，"
                "但当前连接待在标定页重新确认"
            )
            color = "#ffcc66"
        self.release_position_label.setText(text)
        self.release_position_label.setStyleSheet(
            f"color:{color};font-weight:700;"
        )
        self._refresh_controls()

    @Slot()
    def start_auto_test(self):
        target_force_n = float(self.auto_test_target_input.currentData())
        rounds = int(self.auto_test_rounds_input.value())
        sample = self._latest_sensor_sample()
        if not self._sensor_sample_is_current(sample):
            self._set_status("没有0.5秒内更新的完整CH4～CH7传感器数据，不能启动自动测试")
            return
        self._start_operation("auto", target_force_n, auto_rounds=rounds)

    def _sweep_test_plan(self):
        maximum_force = round(float(self.sweep_max_force_input.value()), 2)
        force_interval = round(float(self.sweep_force_interval_input.value()), 2)
        cycle_count = int(self.sweep_repeats_input.value())
        idle_stable_seconds = round(
            float(self.sweep_idle_stable_input.value()),
            1,
        )
        idle_timeout_seconds = round(
            float(self.sweep_idle_timeout_input.value()),
            1,
        )
        if idle_timeout_seconds < idle_stable_seconds:
            return None, "最长等待时间不能小于回零后传感器连续稳定时间"
        maximum_steps = int(round(maximum_force / MAP_FORCE_STEP_N))
        interval_steps = int(round(force_interval / MAP_FORCE_STEP_N))
        if (
            maximum_steps <= 0
            or interval_steps <= 0
            or not math.isclose(
                maximum_force,
                maximum_steps * MAP_FORCE_STEP_N,
                abs_tol=1e-9,
            )
            or not math.isclose(
                force_interval,
                interval_steps * MAP_FORCE_STEP_N,
                abs_tol=1e-9,
            )
        ):
            return None, f"最大压力和压力间隔必须是 {MAP_FORCE_STEP_N:.2f} N的整数倍"
        if maximum_steps % interval_steps:
            return None, "最大压力必须能被压力间隔整除"
        targets = FORCE_CALIBRATION_TARGETS_N
        if not targets:
            return None, "没有可执行的压力档位"
        if len(targets) < 2:
            return None, "传感器分段标定至少需要两个压力档位"
        return {
            "maximum_force_n": maximum_force,
            "force_interval_n": force_interval,
            "low_force_interval_n": 0.1,
            "range_boundary_n": 2.0,
            "cycle_count": cycle_count,
            "sequence_mode": SWEEP_SEQUENCE_ALTERNATING,
            "targets_n": targets,
            "level_count": len(targets),
            "passes_per_cycle": 2,
            "samples_per_cycle": len(targets) * 2,
            "total_rounds": len(targets) * 2 * cycle_count,
            "warmup_force_n": maximum_force,
            "warmup_rounds": WARMUP_ROUNDS,
            "warmup_records_data": False,
            "sensor_idle_stable_seconds": idle_stable_seconds,
            "sensor_idle_timeout_seconds": idle_timeout_seconds,
        }, ""

    @Slot()
    def _update_sweep_plan_label(self, *_args):
        plan, error = self._sweep_test_plan()
        if plan is None:
            self.sweep_plan_label.setText(f"计划无效：{error}")
            self.sweep_plan_label.setStyleSheet("color:#ff7b72;font-weight:700;")
        else:
            self.sweep_plan_label.setText(
                "低力0.1～2 N每0.1 N；高力2～30 N每1 N（2 N共用）。"
                f"共{plan['level_count']}档 × 升降2遍 × {plan['cycle_count']}轮"
                f" = {plan['total_rounds']}次有效采样；"
                f"先以{plan['warmup_force_n']:.2f} N热身{WARMUP_ROUNDS}次；"
                f"{sweep_sequence_description(plan['sequence_mode'])}"
            )
            self.sweep_plan_label.setStyleSheet("color:#8bd5ff;")
        if hasattr(self, "start_sweep_test_button"):
            self._refresh_controls()

    @Slot()
    def start_sweep_test(self):
        plan, error = self._sweep_test_plan()
        if plan is None:
            self._set_status(f"传感器标定计划无效：{error}")
            return
        sample = self._latest_sensor_sample()
        if not self._sensor_sample_is_current(sample):
            self._set_status("没有0.5秒内更新的完整CH4～CH7传感器数据，不能启动传感器标定")
            return
        self._start_operation(
            "sweep",
            float(plan["targets_n"][0]),
            auto_rounds=plan["total_rounds"],
            sweep_plan=plan,
        )

    @Slot()
    def start_validation_test(self):
        plan, error = self._sweep_test_plan()
        if plan is None:
            self._set_status(f"多压力验证计划无效：{error}")
            return
        sample = self._latest_sensor_sample()
        if not self._sensor_sample_is_current(sample):
            self._set_status(
                "没有0.5秒内更新的完整CH4～CH7传感器数据，"
                "不能启动多压力验证"
            )
            return
        self._start_operation(
            "validation",
            float(plan["targets_n"][0]),
            auto_rounds=plan["total_rounds"],
            sweep_plan=plan,
        )

    @Slot()
    def start_quick_correction_test(self):
        """执行两段十九个代表档位的短流程，为RAM快速修正采集数据。"""
        sample = self._latest_sensor_sample()
        if not self._sensor_sample_is_current(sample):
            self._set_status(
                "没有0.5秒内更新的完整CH4～CH7传感器数据，"
                "不能启动单轮快速K修正"
            )
            return
        targets = tuple(QUICK_CORRECTION_TARGETS_N)
        plan = {
            "maximum_force_n": max(targets),
            "force_interval_n": 0.0,
            "cycle_count": 1,
            "sequence_mode": "ascending_single_pass",
            "targets_n": targets,
            "level_count": len(targets),
            "passes_per_cycle": 1,
            "samples_per_cycle": len(targets),
            "total_rounds": len(targets),
            "warmup_rounds": 0,
            "warmup_records_data": False,
            "sensor_idle_stable_seconds": float(
                self.sweep_idle_stable_input.value()
            ),
            "sensor_idle_timeout_seconds": float(
                self.sweep_idle_timeout_input.value()
            ),
        }
        self._start_operation(
            "quick_correction",
            targets[0],
            auto_rounds=len(targets),
            sweep_plan=plan,
        )

    def _start_operation(
        self,
        mode,
        target_force_n,
        auto_rounds=0,
        sweep_plan=None,
        sweep_resume_entries=None,
    ):
        if self._running:
            self._set_status("按力控制正在运行")
            return
        if not self._motor_control.is_can_connected():
            self._set_status("CAN未连接，不能启动按力控制")
            return
        if self._motor_control.is_operation_busy():
            self._set_status("电机正在执行其他操作")
            return
        release_position = self._confirmed_release_position()
        if release_position is None:
            self._set_status(
                "当前CAN连接尚未确认标定出发点；请先在力与位置标定页设置"
            )
            return
        document = self._load_lookup_document(show_error=True)
        if document is None:
            return
        try:
            mapping_release_position = resolve_mapping_release_reference(document)
        except ForcePositionLookupError as exc:
            self._set_status(str(exc))
            return
        lookup_target = None
        if target_force_n is not None:
            try:
                lookup_target = self._resolve_lookup_target(
                    document,
                    target_force_n,
                )
            except ForcePositionLookupError as exc:
                self._set_status(str(exc))
                return
        sweep_lookup_targets = []
        sweep_warmup_lookup = None
        creep_trials = []
        if mode in ("sweep", "validation", "quick_correction"):
            multi_pressure_name = {
                "sweep": "传感器标定",
                "validation": "多压力验证",
                "quick_correction": "单轮快速K修正",
            }[mode]
            if not isinstance(sweep_plan, dict) or not sweep_plan.get("targets_n"):
                self._set_status(f"{multi_pressure_name}计划为空")
                return
            missing_targets = []
            for sweep_force in sweep_plan["targets_n"]:
                try:
                    sweep_lookup_targets.append(
                        self._resolve_lookup_target(document, sweep_force)
                    )
                except ForcePositionLookupError:
                    missing_targets.append(float(sweep_force))
            if missing_targets:
                missing_text = "、".join(f"{value:.2f} N" for value in missing_targets)
                self._set_status(
                    f"{multi_pressure_name}无法启动："
                    f"当前查找表缺少稳定档位 {missing_text}"
                )
                return
            if mode == "sweep":
                warmup_force_n = round(
                    float(
                        sweep_plan.get(
                            "warmup_force_n",
                            sweep_plan["maximum_force_n"],
                        )
                    ),
                    2,
                )
                try:
                    sweep_warmup_lookup = self._resolve_lookup_target(
                        document,
                        warmup_force_n,
                    )
                except ForcePositionLookupError:
                    self._set_status(
                        "传感器标定无法启动：当前查找表缺少热身所需的稳定档位 "
                        f"{warmup_force_n:.2f} N"
                    )
                    return
        elif mode == "creep_test":
            source_trials = tuple((sweep_plan or {}).get("trials", ()))
            if not source_trials:
                self._set_status("长时蠕变测试计划为空")
                return False
            missing_targets = set()
            for source_trial in source_trials:
                trial = dict(source_trial)
                for force_key, lookup_key in (
                    ("high_force_n", "high_lookup"),
                    ("probe_force_n", "probe_lookup"),
                ):
                    force_n = float(trial.get(force_key, 0.0) or 0.0)
                    if force_n <= 0.0:
                        trial[lookup_key] = None
                        continue
                    try:
                        trial[lookup_key] = self._resolve_lookup_target(
                            document,
                            force_n,
                        )
                    except ForcePositionLookupError:
                        missing_targets.add(force_n)
                creep_trials.append(trial)
            if missing_targets:
                missing_text = "、".join(
                    f"{value:g} N" for value in sorted(missing_targets)
                )
                self._set_status(
                    f"长时蠕变测试无法启动：当前位置表缺少 {missing_text}"
                )
                return False
        release_reference_error = release_position - mapping_release_position
        if abs(release_reference_error) > getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS):
            self._set_status(
                f"当前共享释放位置 {release_position} counts与JSON标定出发点"
                f" {mapping_release_position} counts相差{release_reference_error:+d} counts，"
                "查找表与当前电机基准不一致，已拒绝运动"
            )
            return
        if mode in (
            "press",
            "auto",
            "warmup",
            "sweep",
            "validation",
            "quick_correction",
        ):
            feedback = self._motor_control.latest_motor_feedback()
            position = None if feedback is None else feedback.get("position_counts")
            if position is None:
                self._set_status("尚未获得电机实时位置，请稍后重试")
                return
            error = int(position) - mapping_release_position
            if abs(error) > getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS):
                self._set_status(
                    f"当前位置{int(position)} counts不在JSON标定出发点"
                    f"{mapping_release_position}±{getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS)} counts内；"
                    "请先点击“释放 / 回到标定出发点”"
                )
                return
        ocr = self._ocr_snapshot()
        current_force = ocr["accepted_force_n"]
        if mode != "release" and (
            current_force is None
            or ocr["accepted_age_seconds"] > PRESSURE_FRESH_TIMEOUT_SECONDS
        ):
            self._set_status("没有1秒内更新的有效压力计串口值")
            return
        if current_force is not None:
            current_force = float(current_force)
        if (
            mode in ("press", "auto", "warmup")
            and not pressure_n_is_within_press_start_tolerance(current_force)
        ):
            self.ocr_warning_label.setText(
                f"⚠ 压力计当前为 {current_force:.2f} N，禁止启动下压。"
                f"启动范围为 -{PRESSURE_METER_NEGATIVE_LIMIT_N:.2f}～"
                f"+{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N，"
                "请先在压力计上执行清零。"
            )
            self.ocr_warning_label.setVisible(True)
            self._set_status(
                f"当前压力计读数为 {current_force:.2f} N，"
                f"超出 -{PRESSURE_METER_NEGATIVE_LIMIT_N:.2f}～"
                f"+{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N，"
                "不能启动下压；请先在压力计上清零"
            )
            return
        with self._state_lock:
            self._operation_id += 1
            operation_id = self._operation_id
            self._control_mode = mode
            self._target_force_n = target_force_n
            self._auto_round_index = 0
            self._auto_round_total = (
                int(auto_rounds)
                if mode in (
                    "auto",
                    "warmup",
                    "sweep",
                    "validation",
                    "quick_correction",
                    "creep_test",
                )
                else 0
            )
            self._pressure_level_index = 0
            self._pressure_level_total = 0
            self._calibration_cycle_index = 0
            self._calibration_cycle_total = 0
            self._sweep_direction = ""
            self._direction_step_index = 0
            self._direction_step_total = 0
            self._repeat_index = 0
            self._repeat_total = 0
            self._attempt_index = 0
            self._command_counts = 0
            self._total_travel_counts = 0
            self._operation_rejected_ocr_count = 0
            self._last_progress_stage = None
        parameters = {
            "operation_id": operation_id,
            "mode": mode,
            "target_force_n": target_force_n,
            "auto_rounds": int(auto_rounds),
            "auto_test_interval_seconds": self.auto_test_interval_input.value() if mode == 'auto' else 0.0,
            **self._operation_motion_parameters(),
            "release_position_counts": mapping_release_position,
            "lookup_press_travel_counts": (
                0 if lookup_target is None else int(lookup_target["press_travel_counts"])
            ),
            "lookup_target_position_counts": (
                mapping_release_position
                if lookup_target is None
                else int(lookup_target["target_position_counts"])
            ),
            "lookup_position_span_counts": (
                0.0
                if lookup_target is None
                else float(lookup_target["position_span_counts"])
            ),
            "lookup_created_at": (
                "--" if lookup_target is None else lookup_target["created_at"]
            ),
            "sweep_plan": dict(sweep_plan or {}),
            "sweep_lookup_targets": tuple(sweep_lookup_targets),
            "sweep_warmup_lookup": sweep_warmup_lookup,
            "sweep_resume_entries": tuple(sweep_resume_entries or ()),
            "creep_trials": tuple(creep_trials),
            "wait_for_sensor_idle_after_release": mode in (
                "sweep",
                "validation",
                "quick_correction",
            ),
            "sensor_idle_stable_seconds": float(
                (sweep_plan or {}).get(
                    "sensor_idle_stable_seconds",
                    self.sweep_idle_stable_input.value(),
                )
            ),
            "sensor_idle_timeout_seconds": float(
                (sweep_plan or {}).get(
                    "sensor_idle_timeout_seconds",
                    self.sweep_idle_timeout_input.value(),
                )
            ),
        }
        parameters["_sweep_checkpoint_entries"] = list(
            parameters["sweep_resume_entries"]
        )

        def operation(motor, stop_event):
            result = {
                "ok": False,
                "operation_id": operation_id,
                "mode": mode,
                "target_force_n": target_force_n,
                "stop_reason": "unknown",
            }
            tracker = {}
            self._creep_active = mode == "creep_test"
            self._creep_rebound_active = self._creep_active and (parameters.get("sweep_plan") or {}).get("protocol") == REBOUND_PROTOCOL
            try:
                if mode == "creep_test":
                    self._creep_current_trial = dict(parameters["creep_trials"][0], trial_index=1)
                    if not prevent_idle_sleep(True):
                        self._record_event("sleep_prevention_failed", message="阻止空闲休眠失败，请检查电源设置")
                self._prepare_motor(
                    motor,
                    tracker,
                    stop_event,
                    parameters,
                )
                if mode == "press":
                    result.update(
                        self._run_press(
                            motor,
                            tracker,
                            stop_event,
                            parameters,
                        )
                    )
                elif mode == "release":
                    result.update(
                        self._run_release(
                            motor,
                            tracker,
                            stop_event,
                            parameters,
                        )
                    )
                elif mode == "warmup":
                    result.update(
                        self._run_warmup(
                            motor,
                            tracker,
                            stop_event,
                            parameters,
                        )
                    )
                elif mode in ("sweep", "validation", "quick_correction"):
                    result.update(
                        self._run_sweep_test(
                            motor,
                            tracker,
                            stop_event,
                            parameters,
                        )
                    )
                elif mode == "creep_test":
                    result.update(
                        self._run_creep_test(
                            motor,
                            tracker,
                            stop_event,
                            parameters,
                        )
                    )
                else:
                    result.update(
                        self._run_auto_test(
                            motor,
                            tracker,
                            stop_event,
                            parameters,
                        )
                    )
            except ForceControlError as exc:
                if exc.brake_required:
                    self._safe_brake(motor)
                message = str(exc)
                if mode == "sweep":
                    checkpoint_entries = parameters.get(
                        "_sweep_checkpoint_entries",
                        (),
                    )
                    try:
                        checkpoint_path = self._write_sweep_checkpoint(
                            parameters,
                            checkpoint_entries,
                            "stopped",
                            str(exc),
                        )
                    except ForceControlError as checkpoint_exc:
                        message = f"{message}；{checkpoint_exc}"
                    else:
                        message = (
                            f"{message}；已保留{len(checkpoint_entries)}轮结果至"
                            f" {checkpoint_path}，可点击“读取上次标定并继续”"
                        )
                result.update(stop_reason=exc.reason, message=message)
            except Exception as exc:
                self._safe_brake(motor)
                message = str(exc)
                if mode == "sweep":
                    checkpoint_entries = parameters.get(
                        "_sweep_checkpoint_entries",
                        (),
                    )
                    try:
                        checkpoint_path = self._write_sweep_checkpoint(
                            parameters,
                            checkpoint_entries,
                            "stopped",
                            str(exc),
                        )
                    except ForceControlError as checkpoint_exc:
                        message = f"{message}；{checkpoint_exc}"
                    else:
                        message = (
                            f"{message}；已保留{len(checkpoint_entries)}轮结果至"
                            f" {checkpoint_path}，可点击“读取上次标定并继续”"
                        )
                result.update(stop_reason="error", message=message)
            finally:
                if mode == "creep_test" and not result.get("ok"):
                    self._finish_creep_failure(motor, tracker, parameters, result)
                self._creep_active = False
                self._creep_rebound_active = False
                if mode == "creep_test":
                    prevent_idle_sleep(False)
                self._record_event(
                    "operation_complete",
                    stop_reason=result.get("stop_reason", "unknown"),
                    message=result.get("message", ""),
                )
                self.manual_observation_event.emit({
                    "event": "operation_complete", "at": time.monotonic(),
                    "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    "operation_id": operation_id, "mode": mode, "result": dict(result),
                })
                self.operation_finished.emit(result)

        action = {
            "press": "按力下压",
            "release": "释放抬起",
            "warmup": f"{WARMUP_FORCE_N:.2f} N热身",
            "auto": "自动重复测试",
            "sweep": "传感器标定",
            "validation": "多压力验证（不标定）",
            "quick_correction": "单轮快速K修正",
            "creep_test": "长时蠕变测试",
        }[mode]
        self._running = True
        accepted, reason = self._motor_control.start_external_operation(
            operation,
            f"{action}已开始",
        )
        if not accepted:
            self._running = False
            self._set_status(f"{action}无法启动：{reason}")
            self._refresh_controls()
            return False
        self._record_event("operation_started", message=f"{action}已开始")
        if mode == "release":
            summary = "抬起"
        else:
            summary = "下压"
        self._set_status(f"{action}运行中", summary=summary)
        self._refresh_controls()
        return True

    def _call_with_motor_communication_retry(
        self,
        action,
        callback,
        stop_event,
        motion_mode=None,
    ):
        retry_count = 0
        while True:
            if stop_event.is_set():
                raise ForceControlError("user_stop", "用户请求停止电机通信重试")
            try:
                result = callback()
            except PD42S1CommunicationError as exc:
                retry_count += 1
                if getattr(self, "_creep_active", False) and retry_count >= 3:
                    raise ForceControlError("motor_communication_timeout", f"{action}连续通信失败，转入卸载：{exc}") from exc
                phase = (
                    f"{motion_mode}_communication_retry"
                    if motion_mode in ("press", "release")
                    else "communication_retry"
                )
                message = (
                    f"电机通信暂时失败，正在重试{action}（第{retry_count}次）：{exc}"
                )
                self._record_event(
                    "motor_communication_retry",
                    message=message,
                )
                self.operation_progress.emit(
                    {
                        "phase": phase,
                        "current_force_n": self._ocr_snapshot().get(
                            "accepted_force_n"
                        ),
                        "communication_retry_count": retry_count,
                        "message": message,
                    }
                )
                retry_delay = min(
                    MOTOR_COMMUNICATION_RETRY_MAX_DELAY_SECONDS,
                    MOTOR_COMMUNICATION_RETRY_DELAY_SECONDS * retry_count,
                )
                if stop_event.wait(retry_delay):
                    raise ForceControlError(
                        "user_stop",
                        "用户请求停止电机通信重试",
                    ) from exc
                continue
            if retry_count:
                self._communication_recovery_generation += 1
                message = f"电机通信已恢复，{action}继续执行"
                self._record_event(
                    "motor_communication_recovered",
                    message=message,
                )
                self.operation_progress.emit(
                    {
                        "phase": (
                            f"{motion_mode}_communication_recovered"
                            if motion_mode in ("press", "release")
                            else "communication_recovered"
                        ),
                        "current_force_n": self._ocr_snapshot().get(
                            "accepted_force_n"
                        ),
                        "message": message,
                    }
                )
            return result

    def _prepare_motor(self, motor, tracker, stop_event, parameters):
        max_current_ma = int(parameters["max_current_ma"])
        mode = str(parameters.get("mode", ""))
        if mode == "creep_test":
            mode = "release"
        applied_current_ma = self._call_with_motor_communication_retry(
            "设置电流限制",
            lambda: motor.set_max_torque_current_ma(max_current_ma),
            stop_event,
        )
        running_state = int(
            self._call_with_motor_communication_retry(
                "读取运行状态",
                motor.read_running_state,
                stop_event,
            )
        )
        release_from_holding = running_state == 2 and mode == "release"
        if running_state == 2 and not release_from_holding:
            raise ForceControlError(
                "motor_already_running",
                "按力控制启动前检测到电机仍在运行",
            )
        if release_from_holding:
            holding_positions = []
            for sample_index in range(MOTION_STABLE_SAMPLE_COUNT):
                feedback = self._poll_checked(
                    motor,
                    tracker,
                    stop_event,
                    max_current_ma,
                    None,
                    "release",
                )
                if int(feedback["running_state"]) != 2:
                    break
                holding_positions.append(int(feedback["position_counts"]))
                if (
                    abs(int(feedback["measured_speed_rpm"]))
                    > MOTION_STOP_SPEED_RPM
                    or (
                        len(holding_positions) >= 2
                        and abs(holding_positions[-1] - holding_positions[-2])
                        > MOTION_STABLE_DELTA_COUNTS
                    )
                ):
                    raise ForceControlError(
                        "motor_already_running",
                        "释放启动前检测到状态2下电机位置仍在变化，已拒绝接管",
                        brake_required=False,
                    )
                if sample_index + 1 < MOTION_STABLE_SAMPLE_COUNT:
                    if stop_event.wait(0.05):
                        raise ForceControlError(
                            "user_stop",
                            "用户请求紧急停止",
                        )
            if len(holding_positions) == MOTION_STABLE_SAMPLE_COUNT:
                self._record_event(
                    "release_from_settled_holding",
                    message=(
                        "驱动器状态仍为2，但转速为0且位置连续稳定；"
                        "直接交给释放运动路径清状态并发送回零位移"
                    ),
                )
                # 高负载位置保持不会因0xFB清状态立即退出状态2。
                # 保持当前力矩直到释放路径在0xF3回零命令前紧邻执行0xFB，
                # 与自动标定每档采样后的可靠释放顺序保持一致。
                tracker["overcurrent_samples"] = 0
                self._record_event(
                    "motor_prepared",
                    message=(
                        "已确认状态2为静止受力保持并设置0x64硬件力矩限制 "
                        f"{applied_current_ma} mA；等待立即发送释放位移"
                    ),
                )
                return
        self._call_with_motor_communication_retry(
            "清除驱动器状态",
            motor.clear_state,
            stop_event,
        )
        if stop_event.wait(MOTOR_STATE_CLEAR_DELAY_SECONDS):
            raise ForceControlError("user_stop", "用户请求紧急停止")

        tracker.clear()
        sampled_currents = []
        for sample_index in range(STARTUP_CURRENT_SAMPLE_COUNT):
            feedback = self._call_with_motor_communication_retry(
                "读取启动反馈",
                lambda: self._read_feedback(motor, tracker),
                stop_event,
            )
            running_state = int(feedback["running_state"])
            if running_state == 2:
                raise ForceControlError(
                    "motor_already_running",
                    "清除历史状态后电机仍在运行",
                )
            if running_state in (3, 4, 5):
                raise ForceControlError(
                    "motor_fault",
                    f"启动前电机状态异常：{feedback['running_state_name']}",
                    brake_required=False,
                )
            sampled_currents.append(abs(int(feedback["phase_current_ma"])))
            if sample_index + 1 < STARTUP_CURRENT_SAMPLE_COUNT:
                if stop_event.wait(0.05):
                    raise ForceControlError("user_stop", "用户请求紧急停止")

        if all(current > max_current_ma for current in sampled_currents):
            raise ForceControlError(
                "startup_current_limit",
                "启动前静态相电流持续超过限制："
                f"{sampled_currents} mA，限制 {max_current_ma} mA；"
                "已拒绝运动且不会重复刹车",
                brake_required=False,
            )
        tracker["overcurrent_samples"] = 0
        self._record_event(
            "motor_prepared",
            max_current_ma=applied_current_ma,
            message=(
                f"已清除历史状态并设置0x64硬件力矩限制 {applied_current_ma} mA；"
                f"启动电流 {sampled_currents} mA"
            ),
        )

    def _run_press(
        self,
        motor,
        tracker,
        stop_event,
        parameters,
        require_zero_start=True,
    ):
        max_current = parameters["max_current_ma"]
        target_force = parameters["target_force_n"]
        force_limit = MAX_OPERATION_FORCE_N
        feedback = self._poll_checked(
            motor,
            tracker,
            stop_event,
            max_current,
            force_limit,
            "press",
        )
        origin_counts = int(feedback["position_counts"])
        initial_sequence = self._ocr_snapshot()["accepted_sequence"]
        current_force = self._wait_for_stable_force(
            motor,
            tracker,
            stop_event,
            max_current,
            force_limit,
            initial_sequence,
            None,
            "press",
        )
        # 一次性查表下压发令前再次确认压力计零点，避免带载从标定起点运动。
        if require_zero_start and not pressure_n_is_within_press_start_tolerance(
            current_force
        ):
            raise ForceControlError(
                "press_start_force_not_zero",
                f"下压启动前压力计读数为 {current_force:.2f} N，"
                f"超出 -{PRESSURE_METER_NEGATIVE_LIMIT_N:.2f}～"
                f"+{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N，已拒绝运动",
                brake_required=False,
            )
        target_position = int(parameters["lookup_target_position_counts"])
        press_travel = int(parameters["lookup_press_travel_counts"])
        delta_counts = target_position - origin_counts
        if delta_counts >= 0:
            raise ForceControlError(
                "lookup_direction_invalid",
                f"当前位置 {origin_counts} counts，查表目标 {target_position} counts"
                "不在下压方向；请检查标定出发点",
                brake_required=False,
            )
        if abs(delta_counts) > int(parameters["max_travel_counts"]):
            raise ForceControlError(
                "travel_limit",
                f"查表需要移动 {abs(delta_counts)} counts，超过内部安全行程"
                f" {parameters['max_travel_counts']} counts",
                brake_required=False,
        )

        speed_level = parameters["press_speed_level"]
        # 高负载下的位置修正沿用本轮下压速度，避免固定1档无法克服静态负载而超时。
        correction_speed_level = speed_level
        correction_max_counts = press_position_correction_max_counts(
            target_force
        )
        correction_force_tolerance_n = (
            press_position_correction_force_tolerance_n(target_force)
        )
        correction_force_lower_limit_n = (
            target_force - correction_force_tolerance_n
        )
        self.operation_progress.emit(
            {
                "phase": "press_lookup_prepare",
                "current_force_n": current_force,
                "target_force_n": target_force,
                "command_counts": delta_counts,
                "total_travel_counts": abs(delta_counts),
                "speed_level": speed_level,
                "target_position_counts": target_position,
                "message": (
                    f"JSON目标 {target_force:.2f} N：标定行程 {press_travel} counts，"
                    f"从 {origin_counts} 一次性运动到 {target_position} counts"
                ),
            }
        )
        feedback = self._move_relative(
            motor,
            tracker,
            stop_event,
            delta_counts,
            speed_level,
            parameters["acceleration"],
            max_current,
            force_limit,
            "press",
            abs(delta_counts),
            require_zero_force_start=True,
            motion_speed_name="查表一次性下压",
            position_tolerance_counts=correction_max_counts,
            fixed_target_counts=target_position,
        )
        actual_position = int(feedback["position_counts"])
        position_error = actual_position - target_position
        position_error_before_correction = position_error
        correction_counts = 0
        arrival_sequence = self._ocr_snapshot()["accepted_sequence"]
        final_force = self._wait_for_stable_force(
            motor,
            tracker,
            stop_event,
            max_current,
            force_limit,
            arrival_sequence,
            None,
            "press",
        )
        correction_required = press_position_correction_required(
            final_force,
            target_force,
            position_error,
        )
        if (
            position_error > PRESS_POSITION_CORRECTION_TOLERANCE_COUNTS
            and not correction_required
        ):
            self._record_event(
                "press_position_correction_skipped_force_reached",
                final_force_n=final_force,
                press_position_correction_counts=0,
                press_position_error_before_correction_counts=position_error,
                press_position_error_after_correction_counts=position_error,
                message=(
                    f"查表下压停止后少走{position_error} counts，但稳定压力"
                    f"{final_force:.2f} N已达到目标{target_force:.2f} N的"
                    f"{correction_force_tolerance_n:.2f} N比例容差下限"
                    f"{correction_force_lower_limit_n:.2f} N；"
                    "接受当前停点，不再补位置"
                ),
            )
        elif correction_required:
            if position_error > correction_max_counts:
                raise ForceControlError(
                    "lookup_position_error",
                    f"一次性下压未到达可修正范围：目标 {target_position}，"
                    f"实际 {actual_position} counts，少走 {position_error} counts，"
                    f"当前{target_force:.2f} N允许最多 {correction_max_counts} counts",
                )
            correction_counts = -position_error
            self._record_event(
                "press_position_correction_started",
                final_force_n=final_force,
                press_position_correction_counts=correction_counts,
                press_position_error_before_correction_counts=position_error,
                message=(
                    f"稳定压力{final_force:.2f} N低于目标允许下限"
                    f"{correction_force_lower_limit_n:.2f} N"
                    f"（容差{correction_force_tolerance_n:.2f} N），且查表下压"
                    f"少走{position_error} counts；"
                    f"当前压力档允许修正范围为{correction_max_counts} counts；"
                    f"以本轮下压速度{correction_speed_level}档补一次"
                    f"{correction_counts:+d} counts纯位置修正"
                ),
            )
            self.operation_progress.emit(
                {
                    "phase": "press_position_correction",
                    "target_force_n": target_force,
                    "command_counts": correction_counts,
                    "total_travel_counts": abs(delta_counts) + abs(correction_counts),
                    "speed_level": correction_speed_level,
                    "target_position_counts": target_position,
                    "message": (
                        f"当前压力{final_force:.2f} N仍低于目标，"
                        f"目标位置还差{position_error} counts，"
                        f"未超过本压力档{correction_max_counts} counts准入范围，"
                        "正在执行一次纯位置修正"
                    ),
                }
            )
            feedback = self._move_relative(
                motor,
                tracker,
                stop_event,
                correction_counts,
                correction_speed_level,
                parameters["acceleration"],
                max_current,
                force_limit,
                "press",
                abs(delta_counts) + abs(correction_counts),
                motion_speed_name="查表位置一次修正",
                position_tolerance_counts=(
                    PRESS_POSITION_CORRECTION_TOLERANCE_COUNTS
                ),
                fixed_target_counts=target_position,
            )
            actual_position = int(feedback["position_counts"])
            position_error = actual_position - target_position
            self._record_event(
                "press_position_correction_completed",
                press_position_correction_counts=correction_counts,
                press_position_error_before_correction_counts=(
                    position_error_before_correction
                ),
                press_position_error_after_correction_counts=position_error,
                message=(
                    f"一次纯位置修正完成：目标 {target_position}，"
                    f"实际 {actual_position} counts，误差 {position_error:+d} counts"
                ),
            )
            arrival_sequence = self._ocr_snapshot()["accepted_sequence"]
            final_force = self._wait_for_stable_force(
                motor,
                tracker,
                stop_event,
                max_current,
                force_limit,
                arrival_sequence,
                None,
                "press",
            )
        pressure_target_reached = final_force >= correction_force_lower_limit_n
        if pressure_target_reached and (
            position_error_before_correction
            > PRESS_POSITION_CORRECTION_TOLERANCE_COUNTS
        ):
            completion_text = "稳定压力已达目标，接受当前停点且未补位置"
        elif correction_counts:
            completion_text = "压力不足且位置少走，已完成一次位置修正"
        else:
            completion_text = (
                f"位置误差在±{MOTION_POSITION_TOLERANCE_COUNTS} counts内，无需修正"
            )
        if (
            not pressure_target_reached
            and abs(position_error) > MOTION_POSITION_TOLERANCE_COUNTS
        ):
            raise ForceControlError(
                "lookup_position_error",
                f"一次性下压未到达查表目标：目标 {target_position}，"
                f"实际 {actual_position} counts",
            )
        total_travel_counts = abs(delta_counts) + abs(correction_counts)
        self._set_motion_context(delta_counts, total_travel_counts)
        return {
            "ok": True,
            "stop_reason": "lookup_position_reached",
            "message": (
                f"查表目标 {target_force:.2f} N；目标位置 {target_position}，"
                f"实际 {actual_position} counts，误差 {position_error:+d} counts；"
                f"到位压力 {final_force:.2f} N；{completion_text}"
            ),
            "final_force_n": final_force,
            "target_position_counts": target_position,
            "final_position_counts": actual_position,
            "press_position_correction_counts": correction_counts,
            "press_position_error_before_correction_counts": (
                position_error_before_correction
            ),
            "press_position_error_after_correction_counts": position_error,
            "total_travel_counts": total_travel_counts,
        }

    def _run_warmup(self, motor, tracker, stop_event, parameters):
        """按指定查表压力自动完成固定次数的下压和释放。"""
        total_rounds = int(parameters["auto_rounds"])
        target_force = float(parameters["target_force_n"])
        calibration_warmup = bool(parameters.get("calibration_warmup"))
        phase_prefix = "sweep_warmup" if calibration_warmup else "warmup"
        no_record_text = "；当前为标定前热身，不记录数据" if calibration_warmup else ""
        completed_rounds = 0
        last_press_result = None
        last_release_result = None
        for round_index in range(1, total_rounds + 1):
            if stop_event.is_set():
                raise ForceControlError("user_stop", "用户停止热身")
            with self._state_lock:
                self._auto_round_index = round_index
                self._auto_round_total = total_rounds
                self._attempt_index = 1
            self.operation_progress.emit(
                {
                    "phase": f"{phase_prefix}_round_start",
                    "target_force_n": target_force,
                    "warmup_round_index": round_index,
                    "warmup_round_total": total_rounds,
                    "message": (
                        f"热身第 {round_index}/{total_rounds} 次："
                        f"一次性下压到 {target_force:.2f} N查表位置"
                        f"{no_record_text}"
                    ),
                }
            )
            attempt_index = 0
            motion_retries = 0
            while True:
                attempt_index += 1
                with self._state_lock:
                    self._attempt_index = attempt_index
                try:
                    last_press_result = self._run_press(
                        motor,
                        tracker,
                        stop_event,
                        parameters,
                    )
                except ForceControlError as exc:
                    if not calibration_warmup:
                        raise
                    round_context = {
                        "auto_round_index": round_index,
                        "auto_round_total": total_rounds,
                        "pressure_level_index": 0,
                        "pressure_level_total": 0,
                        "calibration_cycle_index": 0,
                        "calibration_cycle_total": 0,
                        "sweep_direction": "warmup",
                        "direction_step_index": round_index,
                        "direction_step_total": total_rounds,
                        "repeat_index": 0,
                        "repeat_total": 0,
                        "attempt_index": attempt_index,
                        "round_name": f"标定前热身 {round_index}/{total_rounds}",
                    }
                    if exc.reason in (
                        "motion_timeout",
                        "motion_position_error",
                        "lookup_position_error",
                    ):
                        self._safe_brake(motor)
                        self.operation_progress.emit(
                            {
                                "phase": "sweep_timeout_recovery",
                                "target_force_n": target_force,
                                "message": (
                                    f"标定前热身第 {round_index}/{total_rounds} 次"
                                    f"到位失败：{exc}；返回零点后重试，不记录数据"
                                ),
                            }
                        )
                        self._record_event(
                            "sweep_warmup_motion_retry",
                            stop_reason=exc.reason,
                            message=(
                                f"标定前热身第 {round_index}/{total_rounds} 次作废："
                                f"{exc}；返回零点后重试，不记录数据"
                            ),
                        )
                        self._run_release(
                            motor,
                            tracker,
                            stop_event,
                            parameters,
                        )
                        motion_retries += 1
                        if motion_retries > SWEEP_MOTION_TIMEOUT_MAX_RETRIES:
                            raise ForceControlError(
                                "sweep_warmup_retry_exhausted",
                                f"标定前热身第 {round_index}/{total_rounds} 次连续重试"
                                f"{SWEEP_MOTION_TIMEOUT_MAX_RETRIES}次仍到位失败；"
                                "已返回零点并停止传感器标定",
                                brake_required=False,
                            ) from exc
                        continue
                    if exc.reason != "press_start_force_not_zero":
                        raise
                    self.operation_progress.emit(
                        {
                            "phase": "sweep_start_zero_wait",
                            "target_force_n": target_force,
                            "message": (
                                f"标定前热身第 {round_index}/{total_rounds} 次尚未运动："
                                f"{exc}；等待{SWEEP_START_ZERO_RETRY_SECONDS:g}秒后重试，"
                                "不记录数据"
                            ),
                        }
                    )
                    self._wait_before_sweep_zero_retry(
                        motor,
                        tracker,
                        stop_event,
                        parameters,
                        round_context,
                    )
                    continue
                break
            self.operation_progress.emit(
                {
                    "phase": f"{phase_prefix}_release",
                    "current_force_n": last_press_result.get("final_force_n"),
                    "target_force_n": target_force,
                    "warmup_round_index": round_index,
                    "warmup_round_total": total_rounds,
                    "message": (
                        f"热身第 {round_index}/{total_rounds} 次下压完成，"
                        f"正在返回标定出发点{no_record_text}"
                    ),
                }
            )
            last_release_result = self._run_release(
                motor,
                tracker,
                stop_event,
                parameters,
            )
            completed_rounds = round_index
            self._record_event(
                f"{phase_prefix}_round_completed",
                final_force_n=last_press_result.get("final_force_n"),
                auto_round_index=round_index,
                auto_round_total=total_rounds,
                message=(
                    f"{target_force:.2f} N热身第 {round_index}/{total_rounds} 次完成并已释放"
                    f"{no_record_text}"
                ),
            )

        return {
            "ok": True,
            "stop_reason": "warmup_completed",
            "message": (
                f"热身完成：{target_force:.2f} N × {completed_rounds} 次，"
                "电机已回到标定出发点"
            ),
            "final_force_n": (
                None
                if last_release_result is None
                else last_release_result.get("final_force_n")
            ),
            "warmup_rounds_completed": completed_rounds,
            "last_press_force_n": (
                None
                if last_press_result is None
                else last_press_result.get("final_force_n")
            ),
        }

    def _run_auto_test(self, motor, tracker, stop_event, parameters):
        target_force = float(parameters["target_force_n"])
        total_rounds = int(parameters["auto_rounds"])
        interval = float(parameters.get('auto_test_interval_seconds', 0.0))
        if not math.isfinite(interval) or not 0 <= interval <= 3600:
            raise ForceControlError('auto_interval_invalid', '释放后间隔须在0～3600秒之间')
        release_parameters = dict(parameters, wait_for_sensor_idle_after_release=False)
        force_limit = MAX_OPERATION_FORCE_N
        round_results = []
        release_result = None
        for round_index in range(1, total_rounds + 1):
            with self._state_lock:
                self._auto_round_index = round_index
            self.operation_progress.emit(
                {
                    "phase": "auto_round_start",
                    "target_force_n": target_force,
                    "message": (
                        f"自动测试第 {round_index}/{total_rounds} 轮："
                        f"正在下压到 {target_force:.2f} N"
                    ),
                }
            )
            press_result = self._run_press(
                motor,
                tracker,
                stop_event,
                parameters,
            )
            try:
                floating_window = self._wait_for_all_floating_stable(
                    motor,
                    tracker,
                    stop_event,
                    parameters["max_current_ma"],
                    force_limit,
                    round_index,
                    total_rounds,
                )
                sample = floating_window["sample"]
                ocr = self._ocr_snapshot()
                final_force = ocr.get("accepted_force_n")
                if (
                    final_force is None
                    or ocr.get("accepted_age_seconds", math.inf)
                    > PRESSURE_FRESH_TIMEOUT_SECONDS
                ):
                    raise ForceControlError(
                        "auto_ocr_stale",
                        f"第 {round_index} 轮FLOATING采集完成时没有新鲜的压力计读数",
                    )
                final_force = float(final_force)
            except ForceControlError as exc:
                if exc.reason in (
                    "floating_timeout",
                    "sensor_unavailable",
                    "auto_ocr_stale",
                ):
                    self.operation_progress.emit(
                        {
                            "phase": "auto_safe_release",
                            "message": f"{exc}；正在先返回固定释放位置再停止自动测试",
                        }
                    )
                    self._run_release(
                        motor,
                        tracker,
                        stop_event,
                        parameters,
                    )
                raise

            force_values = self._sensor_force_values(sample)
            entry = {
                "captured_at": datetime.now().astimezone().isoformat(
                    timespec="milliseconds"
                ),
                "operation_id": parameters["operation_id"],
                "control_mode": "auto",
                "target_force_n": target_force,
                "final_force_n": final_force,
                "auto_round_index": round_index,
                "auto_round_total": total_rounds,
                "press_arrival_force_n": press_result.get("final_force_n"),
                "press_position_correction_counts": press_result.get(
                    "press_position_correction_counts",
                    0,
                ),
                "press_position_error_before_correction_counts": press_result.get(
                    "press_position_error_before_correction_counts",
                    "",
                ),
                "press_position_error_after_correction_counts": press_result.get(
                    "press_position_error_after_correction_counts",
                    "",
                ),
                "raw_medians": floating_window["raw_medians"],
                "raw_median_sum": floating_window["raw_median_sum"],
                "raw_median_sample_count": floating_window[
                    "raw_median_sample_count"
                ],
                "raw_median_window_seconds": floating_window[
                    "raw_median_window_seconds"
                ],
                **force_values,
            }
            round_results.append(entry)
            self.force_mapping_captured.emit(entry)
            self._record_event(
                "auto_round_captured",
                _sensor_sample=sample,
                final_force_n=final_force,
                auto_round_index=round_index,
                auto_round_total=total_rounds,
                **self._raw_median_log_values(floating_window),
                message=(
                    f"第 {round_index}/{total_rounds} 轮记录："
                    f"{final_force:.2f} N -> Signal合力 {force_values['signal_sum']}；"
                    f"Raw 1秒中位值 {floating_window['raw_medians']}，"
                    f"四路和 {floating_window['raw_median_sum']:g}"
                ),
            )
            self.operation_progress.emit(
                {
                    "phase": "auto_release",
                    "current_force_n": final_force,
                    "message": (
                        f"自动测试第 {round_index}/{total_rounds} 轮已记录"
                        f"Signal合力 {force_values['signal_sum']}和Raw 1秒中位值，"
                        f"正在返回标定出发点"
                    ),
                }
            )
            release_result = self._run_release(
                motor,
                tracker,
                stop_event,
                release_parameters,
            )
            round_results[-1]["release_force_n"] = release_result.get(
                "final_force_n",
                0.0,
            )
            if round_index < total_rounds and interval > 0:
                self._wait_auto_repeat_interval(motor, tracker, stop_event, parameters,
                                                interval, round_index, total_rounds)

        signal_statistics = self._signal_sum_statistics(round_results)
        raw_statistics = self._raw_median_statistics(round_results)
        statistics_parts = [
            self._format_signal_sum_statistics(signal_statistics),
            self._format_raw_median_statistics(raw_statistics),
        ]
        statistics_text = "；".join(
            part for part in statistics_parts if part
        )
        self._record_event(
            "auto_statistics",
            auto_round_index=total_rounds,
            auto_round_total=total_rounds,
            message=statistics_text,
        )
        return {
            "ok": True,
            "stop_reason": "auto_completed",
            "message": f"自动测试完成：{target_force:.2f} N × {total_rounds} 轮",
            "final_force_n": (
                None if release_result is None else release_result.get("final_force_n")
            ),
            "auto_round_results": round_results,
            "auto_signal_statistics": signal_statistics,
            "auto_raw_median_statistics": raw_statistics,
        }

    def _wait_auto_repeat_interval(self, motor, tracker, stop_event, parameters, seconds, round_index, total_rounds):
        """卸载确认后按秒等待；只核对压力与电机保护，不读取通道状态。"""
        zero_deadline = time.monotonic() + REBOUND_ZERO_WAIT_SECONDS
        started = None
        last_progress = -math.inf
        while True:
            self._poll_checked(motor, tracker, stop_event, parameters['max_current_ma'],
                               MAX_OPERATION_FORCE_N, 'observe')
            now = time.monotonic()
            if started is None:
                pressure = self._ocr_snapshot()
                if (float(pressure.get('accepted_age_seconds', math.inf)) <= PRESSURE_FRESH_TIMEOUT_SECONDS
                        and near_rebound_zero(pressure.get('accepted_force_n'))):
                    started = now
                    self._record_event('auto_interval_started', auto_round_index=round_index,
                        auto_test_interval_seconds=seconds,
                        message=f'第{round_index}次已释放，间隔{seconds:g}秒后重复按压，不等待通道恢复')
                elif now >= zero_deadline:
                    raise ForceControlError('auto_release_not_zero', '释放后压力仍未接近零，无法开始下一次按压间隔')
            remaining = seconds if started is None else max(0.0, seconds-(now-started))
            if now-last_progress >= 0.2 or remaining == 0:
                self.operation_progress.emit(dict(
                    phase='auto_interval_zero_wait' if started is None else 'auto_interval_wait',
                    remaining_seconds=remaining, interval_seconds=seconds,
                    message=('已到释放位置，等待压力接近零' if started is None else
                        f'释放后间隔：还剩{remaining:.1f}秒，随后进行第{round_index+1}/{total_rounds}次按压；不等待通道恢复')))
                last_progress = now
            if started is not None and remaining == 0:
                self._record_event('auto_interval_completed', auto_round_index=round_index,
                    auto_test_interval_seconds=seconds, auto_interval_elapsed_seconds=now-started,
                    message=f'间隔完成，实际{now-started:.3f}秒，开始第{round_index+1}/{total_rounds}次')
                return
            if stop_event.wait(min(0.05, remaining) if started is not None else 0.05):
                raise ForceControlError('user_stop', '用户停止自动重复测试')

    def _run_sweep_capture_round(
        self,
        motor,
        tracker,
        stop_event,
        parameters,
        round_context,
    ):
        mode = str(parameters["mode"])
        mode_name = {
            "sweep": "传感器标定",
            "validation": "多压力验证",
            "quick_correction": "单轮快速K修正",
        }.get(mode, "多压力测试")
        event_prefix = {
            "sweep": "sweep",
            "validation": "validation",
            "quick_correction": "quick_correction",
        }.get(mode, "multi_pressure")
        press_result = self._run_press(
            motor,
            tracker,
            stop_event,
            parameters,
        )
        floating_window = self._wait_for_all_floating_stable(
            motor,
            tracker,
            stop_event,
            parameters["max_current_ma"],
            MAX_OPERATION_FORCE_N,
            round_context["auto_round_index"],
            round_context["auto_round_total"],
        )
        sample = floating_window["sample"]
        ocr = self._ocr_snapshot()
        final_force = ocr.get("accepted_force_n")
        if (
            final_force is None
            or ocr.get("accepted_age_seconds", math.inf)
            > PRESSURE_FRESH_TIMEOUT_SECONDS
        ):
            raise ForceControlError(
                "auto_ocr_stale",
                f"{round_context['round_name']}采集完成时没有新鲜的压力计读数",
            )
        final_force = float(final_force)
        force_values = self._sensor_force_values(sample)
        if force_values is None:
            raise ForceControlError(
                "sensor_unavailable",
                f"{round_context['round_name']}采集完成时CH4～CH7数据不完整",
            )
        target_position = int(parameters["lookup_target_position_counts"])
        press_arrival_position = int(press_result["final_position_counts"])
        floating_feedback = dict(floating_window["motor_feedback"])
        floating_position = int(floating_feedback["position_counts"])
        entry = {
            "captured_at": datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "operation_id": parameters["operation_id"],
            "control_mode": mode,
            "force_unit": "N",
            "target_force_n": parameters["target_force_n"],
            "lookup_created_at": parameters["lookup_created_at"],
            "release_reference_position_counts": parameters[
                "release_position_counts"
            ],
            "lookup_target_position_counts": parameters[
                "lookup_target_position_counts"
            ],
            "press_arrival_position_counts": press_arrival_position,
            "press_arrival_position_error_counts": (
                press_arrival_position - target_position
            ),
            "press_position_correction_counts": press_result.get(
                "press_position_correction_counts",
                0,
            ),
            "press_position_error_before_correction_counts": press_result.get(
                "press_position_error_before_correction_counts",
                press_arrival_position - target_position,
            ),
            "press_position_error_after_correction_counts": press_result.get(
                "press_position_error_after_correction_counts",
                press_arrival_position - target_position,
            ),
            "floating_capture_position_counts": floating_position,
            "floating_position_error_counts": floating_position - target_position,
            "floating_running_state": int(floating_feedback["running_state"]),
            "press_speed_level": parameters["press_speed_level"],
            "release_speed_level": parameters["release_speed_level"],
            "acceleration_r_s2": parameters["acceleration"],
            "max_current_ma": parameters["max_current_ma"],
            "final_force_n": final_force,
            "press_arrival_force_n": press_result.get("final_force_n"),
            "raw_medians": floating_window["raw_medians"],
            "raw_median_sum": floating_window["raw_median_sum"],
            "raw_median_sample_count": floating_window[
                "raw_median_sample_count"
            ],
            "raw_median_window_seconds": floating_window[
                "raw_median_window_seconds"
            ],
            **round_context,
            **force_values,
        }
        self.operation_progress.emit(
            {
                "phase": "sweep_release",
                "current_force_n": final_force,
                "message": (
                    f"{mode_name}：{round_context['round_name']}采集完成，"
                    "正在返回标定出发点"
                ),
            }
        )
        release_result = self._run_release(
            motor,
            tracker,
            stop_event,
            parameters,
        )
        entry["release_force_n"] = release_result.get("final_force_n", 0.0)
        entry["release_idle_recovered"] = release_result.get(
            "release_idle_recovered",
            "",
        )
        entry["release_idle_wait_seconds"] = release_result.get(
            "release_idle_wait_seconds",
            "",
        )
        entry["release_idle_stable_seconds"] = release_result.get(
            "release_idle_stable_seconds",
            "",
        )
        entry["release_idle_timeout_seconds"] = release_result.get(
            "release_idle_timeout_seconds",
            "",
        )
        self._record_event(
            f"{event_prefix}_round_captured",
            _sensor_sample=sample,
            final_force_n=final_force,
            lookup_created_at=parameters["lookup_created_at"],
            release_reference_position_counts=parameters[
                "release_position_counts"
            ],
            lookup_target_position_counts=parameters[
                "lookup_target_position_counts"
            ],
            press_arrival_position_counts=press_arrival_position,
            press_arrival_position_error_counts=(
                press_arrival_position - target_position
            ),
            press_position_correction_counts=press_result.get(
                "press_position_correction_counts",
                0,
            ),
            press_position_error_before_correction_counts=press_result.get(
                "press_position_error_before_correction_counts",
                press_arrival_position - target_position,
            ),
            press_position_error_after_correction_counts=press_result.get(
                "press_position_error_after_correction_counts",
                press_arrival_position - target_position,
            ),
            floating_capture_position_counts=floating_position,
            floating_position_error_counts=floating_position - target_position,
            floating_running_state=int(floating_feedback["running_state"]),
            **{
                name: round_context[name]
                for name in (
                    "auto_round_index",
                    "auto_round_total",
                    "pressure_level_index",
                    "pressure_level_total",
                    "calibration_cycle_index",
                    "calibration_cycle_total",
                    "sweep_direction",
                    "direction_step_index",
                    "direction_step_total",
                    "repeat_index",
                    "repeat_total",
                    "attempt_index",
                )
            },
            **self._raw_median_log_values(floating_window),
            message=(
                f"{mode_name}：{round_context['round_name']}记录成功："
                f"压力计 {final_force:.2f} N，目标/到位/FLOATING位置 "
                f"{target_position}/{press_arrival_position}/{floating_position} counts，"
                f"FLOATING Signal合力 {force_values['signal_sum']}，"
                f"Raw 1秒中位值 {floating_window['raw_medians']}"
            ),
        )
        self.force_mapping_captured.emit(entry)
        return entry, release_result

    def _wait_before_sweep_zero_retry(
        self,
        motor,
        tracker,
        stop_event,
        parameters,
        round_context,
    ):
        deadline = time.monotonic() + SWEEP_START_ZERO_RETRY_SECONDS
        last_progress_at = 0.0
        while True:
            now = time.monotonic()
            remaining = max(0.0, deadline - now)
            if remaining <= 0:
                break
            self._poll_checked(
                motor,
                tracker,
                stop_event,
                parameters["max_current_ma"],
                None,
                "release",
            )
            if now - last_progress_at >= 0.5:
                current_force = self._ocr_snapshot().get("accepted_force_n")
                self.operation_progress.emit(
                    {
                        "phase": "sweep_start_zero_wait",
                        "current_force_n": current_force,
                        "retry_wait_remaining_seconds": remaining,
                        "message": (
                            f"{round_context['round_name']}出发前压力未回零；"
                            f"保持停止，等待{remaining:.1f}秒后重试"
                        ),
                    }
                )
                last_progress_at = now
            if stop_event.wait(min(0.25, remaining)):
                raise ForceControlError("user_stop", "用户请求紧急停止")
        self._record_event(
            "sweep_start_zero_wait_completed",
            **{
                name: round_context[name]
                for name in (
                    "auto_round_index",
                    "auto_round_total",
                    "pressure_level_index",
                    "pressure_level_total",
                    "calibration_cycle_index",
                    "calibration_cycle_total",
                    "sweep_direction",
                    "direction_step_index",
                    "direction_step_total",
                    "repeat_index",
                    "repeat_total",
                    "attempt_index",
                )
            },
            message=(
                f"{round_context['round_name']}已等待"
                f"{SWEEP_START_ZERO_RETRY_SECONDS:g}秒，重新检查出发压力"
            ),
        )

    def _run_sweep_test(self, motor, tracker, stop_event, parameters):
        mode = str(parameters["mode"])
        is_calibration = mode == "sweep"
        is_quick_correction = mode == "quick_correction"
        mode_name = {
            "sweep": "传感器标定",
            "validation": "多压力验证",
            "quick_correction": "单轮快速K修正",
        }.get(mode, "多压力测试")
        event_prefix = {
            "sweep": "sweep",
            "validation": "validation",
            "quick_correction": "quick_correction",
        }.get(mode, "multi_pressure")
        plan = parameters["sweep_plan"]
        lookups = {
            round(float(item["force_n"]), 2): item
            for item in parameters["sweep_lookup_targets"]
        }
        targets = tuple(round(float(value), 2) for value in plan["targets_n"])
        cycle_count = int(plan["cycle_count"])
        passes_per_cycle = int(plan.get("passes_per_cycle", 2))
        total_rounds = int(plan["total_rounds"])
        round_results = []
        if is_calibration:
            round_results = [
                dict(entry) for entry in parameters.get("sweep_resume_entries", ())
            ]
        completed_keys = {
            (
                int(entry["calibration_cycle_index"]),
                str(entry["sweep_direction"]),
                int(entry["pressure_level_index"]),
            )
            for entry in round_results
        }
        if is_calibration:
            parameters["_sweep_checkpoint_entries"] = round_results
        failed_attempts = 0
        last_release_result = None
        checkpoint_text = "不写入标定恢复记录"
        if is_calibration:
            checkpoint_path = self._write_sweep_checkpoint(
                parameters,
                round_results,
                "running",
            )
            checkpoint_text = f"恢复记录 {checkpoint_path}"

        if is_calibration:
            warmup_force_n = round(float(plan["warmup_force_n"]), 2)
            warmup_lookup = parameters.get("sweep_warmup_lookup")
            if not isinstance(warmup_lookup, dict):
                raise ForceControlError(
                    "warmup_lookup_missing",
                    f"压力－位置查找表缺少 {warmup_force_n:.2f} N 热身档位",
                    brake_required=False,
                )
            warmup_parameters = dict(parameters)
            warmup_parameters.update(
                {
                    "target_force_n": warmup_force_n,
                    "auto_rounds": WARMUP_ROUNDS,
                    "lookup_press_travel_counts": int(
                        warmup_lookup["press_travel_counts"]
                    ),
                    "lookup_target_position_counts": int(
                        warmup_lookup["target_position_counts"]
                    ),
                    "lookup_position_span_counts": float(
                        warmup_lookup["position_span_counts"]
                    ),
                    "lookup_created_at": warmup_lookup["created_at"],
                    "calibration_warmup": True,
                }
            )
            with self._state_lock:
                self._target_force_n = warmup_force_n
                self._auto_round_index = 0
                self._auto_round_total = WARMUP_ROUNDS
                self._pressure_level_index = 0
                self._pressure_level_total = 0
                self._calibration_cycle_index = 0
                self._calibration_cycle_total = cycle_count
                self._sweep_direction = "warmup"
                self._direction_step_index = 0
                self._direction_step_total = WARMUP_ROUNDS
                self._repeat_index = 0
                self._repeat_total = 0
                self._attempt_index = 0
            warmup_message = (
                f"传感器标定正在热身：{warmup_force_n:.2f} N × {WARMUP_ROUNDS} 次；"
                "热身数据不写入标定结果或K值表"
            )
            self.operation_progress.emit(
                {
                    "phase": "sweep_warmup_started",
                    "target_force_n": warmup_force_n,
                    "message": warmup_message,
                }
            )
            self._record_event("sweep_warmup_started", message=warmup_message)
            self._run_warmup(
                motor,
                tracker,
                stop_event,
                warmup_parameters,
            )
            warmup_complete_message = (
                f"传感器标定热身完成：{warmup_force_n:.2f} N × {WARMUP_ROUNDS} 次；"
                "未记录热身数据，开始正式双向标定"
            )
            self.operation_progress.emit(
                {
                    "phase": "sweep_warmup_completed",
                    "target_force_n": targets[0],
                    "message": warmup_complete_message,
                }
            )
            self._record_event(
                "sweep_warmup_completed",
                message=warmup_complete_message,
            )

        self._record_event(
            f"{event_prefix}_plan_started",
            message=(
                f"{mode_name}开始：{len(targets)}档 × {passes_per_cycle}遍 × {cycle_count}轮"
                f" = {total_rounds}次有效采样；已完成{len(round_results)}次；"
                + (
                    "低力0.1/0.5/1/1.5/2 N，高力4～30 N每2 N，共19档单向短流程"
                    if is_quick_correction
                    else sweep_sequence_description(plan["sequence_mode"])
                )
                + f"；{checkpoint_text}"
            ),
        )

        for cycle_index in range(1, cycle_count + 1):
            if is_quick_correction:
                direction_plans = (("ascending", "升序", targets),)
            else:
                direction_plans = sweep_direction_plans(
                    targets,
                    cycle_index,
                    plan["sequence_mode"],
                )
            for direction_index, (
                sweep_direction,
                direction_text,
                ordered_targets,
            ) in enumerate(direction_plans):
                for direction_step_index, target_force in enumerate(
                    ordered_targets,
                    start=1,
                ):
                    level_index = targets.index(target_force) + 1
                    completed_key = (
                        cycle_index,
                        sweep_direction,
                        level_index,
                    )
                    if completed_key in completed_keys:
                        continue
                    lookup = lookups[target_force]
                    level_parameters = dict(parameters)
                    level_parameters.update(
                        {
                            "target_force_n": float(target_force),
                            "lookup_press_travel_counts": int(
                                lookup["press_travel_counts"]
                            ),
                            "lookup_target_position_counts": int(
                                lookup["target_position_counts"]
                            ),
                            "lookup_position_span_counts": float(
                                lookup["position_span_counts"]
                            ),
                            "lookup_created_at": lookup["created_at"],
                        }
                    )
                    completed_round_index = (
                        (cycle_index - 1) * len(targets) * passes_per_cycle
                        + direction_index * len(targets)
                        + direction_step_index
                    )
                    timeout_retries = 0
                    attempt_index = 0
                    while True:
                        attempt_index += 1
                        if is_quick_correction:
                            round_name = (
                                f"快速修正 {direction_step_index}/{len(targets)}，"
                                f"{target_force:.2f} N"
                            )
                        else:
                            round_name = (
                                f"往返轮 {cycle_index}/{cycle_count}，{direction_text} "
                                f"{direction_step_index}/{len(targets)}，"
                                f"{target_force:.2f} N档 {level_index}/{len(targets)}"
                            )
                        round_context = {
                            "auto_round_index": completed_round_index,
                            "auto_round_total": total_rounds,
                            "pressure_level_index": level_index,
                            "pressure_level_total": len(targets),
                            "calibration_cycle_index": cycle_index,
                            "calibration_cycle_total": cycle_count,
                            "sweep_direction": sweep_direction,
                            "direction_step_index": direction_step_index,
                            "direction_step_total": len(targets),
                            "repeat_index": cycle_index,
                            "repeat_total": cycle_count,
                            "attempt_index": attempt_index,
                            "round_name": round_name,
                        }
                        with self._state_lock:
                            self._target_force_n = float(target_force)
                            self._auto_round_index = completed_round_index
                            self._auto_round_total = total_rounds
                            self._pressure_level_index = level_index
                            self._pressure_level_total = len(targets)
                            self._calibration_cycle_index = cycle_index
                            self._calibration_cycle_total = cycle_count
                            self._sweep_direction = sweep_direction
                            self._direction_step_index = direction_step_index
                            self._direction_step_total = len(targets)
                            self._repeat_index = cycle_index
                            self._repeat_total = cycle_count
                            self._attempt_index = attempt_index
                        self.operation_progress.emit(
                            {
                                "phase": "sweep_round_start",
                                "target_force_n": float(target_force),
                                "message": (
                                    f"{mode_name}：{round_name}，"
                                    f"第 {attempt_index} 次尝试；从零点出发"
                                ),
                            }
                        )
                        event_context = {
                            name: round_context[name]
                            for name in (
                                "auto_round_index",
                                "auto_round_total",
                                "pressure_level_index",
                                "pressure_level_total",
                                "calibration_cycle_index",
                                "calibration_cycle_total",
                                "sweep_direction",
                                "direction_step_index",
                                "direction_step_total",
                                "repeat_index",
                                "repeat_total",
                                "attempt_index",
                            )
                        }
                        self._record_event(
                            f"{event_prefix}_round_started",
                            **event_context,
                            message=(
                                f"{round_name}第 {attempt_index} 次尝试开始；"
                                "要求从零点出发并在采集后回到零点"
                            ),
                        )
                        try:
                            entry, last_release_result = self._run_sweep_capture_round(
                                motor,
                                tracker,
                                stop_event,
                                level_parameters,
                                round_context,
                            )
                        except ForceControlError as exc:
                            if exc.reason == "press_start_force_not_zero":
                                self.operation_progress.emit(
                                    {
                                        "phase": "sweep_start_zero_wait",
                                        "target_force_n": float(target_force),
                                        "retry_wait_remaining_seconds": (
                                            SWEEP_START_ZERO_RETRY_SECONDS
                                        ),
                                        "message": (
                                            f"{round_name}尚未发出下压命令：{exc}；"
                                            f"等待{SWEEP_START_ZERO_RETRY_SECONDS:g}秒后"
                                            "重试本次采样"
                                        ),
                                    }
                                )
                                self._record_event(
                                    f"{event_prefix}_start_force_wait",
                                    stop_reason=exc.reason,
                                    **event_context,
                                    message=(
                                        f"{round_name}尚未运动：{exc}；"
                                        f"等待{SWEEP_START_ZERO_RETRY_SECONDS:g}秒后重试"
                                    ),
                                )
                                self._wait_before_sweep_zero_retry(
                                    motor,
                                    tracker,
                                    stop_event,
                                    level_parameters,
                                    round_context,
                                )
                                continue
                            if exc.reason in (
                                "motion_timeout",
                                "motion_position_error",
                                "lookup_position_error",
                            ):
                                failed_attempts += 1
                                self._safe_brake(motor)
                                self.operation_progress.emit(
                                    {
                                        "phase": "sweep_timeout_recovery",
                                        "target_force_n": float(target_force),
                                        "message": (
                                            f"{round_name}到位失败：{exc}；"
                                            "正在返回标定出发点"
                                        ),
                                    }
                                )
                                self._record_event(
                                    f"{event_prefix}_motion_timeout_retry",
                                    stop_reason=exc.reason,
                                    **event_context,
                                    message=(
                                        f"{round_name}本次作废：{exc}；"
                                        "刹停后返回标定出发点再重试"
                                    ),
                                )
                                last_release_result = self._run_release(
                                    motor,
                                    tracker,
                                    stop_event,
                                    level_parameters,
                                )
                                timeout_retries += 1
                                if timeout_retries > SWEEP_MOTION_TIMEOUT_MAX_RETRIES:
                                    raise ForceControlError(
                                        "sweep_motion_retry_exhausted",
                                        f"{round_name}连续重试"
                                        f"{SWEEP_MOTION_TIMEOUT_MAX_RETRIES}次仍到位超时，"
                                        f"已返回标定出发点并停止{mode_name}",
                                        brake_required=False,
                                    ) from exc
                                continue
                            if exc.reason in (
                                "floating_timeout",
                                "sensor_unavailable",
                                "auto_ocr_stale",
                            ):
                                self.operation_progress.emit(
                                    {
                                        "phase": "sweep_safe_release",
                                        "message": f"{exc}；正在返回标定出发点后停止",
                                    }
                                )
                                self._run_release(
                                    motor,
                                    tracker,
                                    stop_event,
                                    level_parameters,
                                )
                            raise
                        round_results.append(entry)
                        completed_keys.add(completed_key)
                        if is_calibration:
                            parameters["_sweep_checkpoint_entries"] = round_results
                            self._write_sweep_checkpoint(
                                parameters,
                                round_results,
                                "running",
                            )
                        break

        if is_calibration:
            self._write_sweep_checkpoint(
                parameters,
                round_results,
                "completed",
            )
        self._record_event(
            f"{event_prefix}_plan_completed",
            auto_round_index=total_rounds,
            auto_round_total=total_rounds,
            message=(
                f"{mode_name}完成：{len(targets)}档，{passes_per_cycle}遍，"
                f"成功{len(round_results)}次有效采样，"
                f"到位失败重试{failed_attempts}次"
            ),
        )
        results_key = {
            "sweep": "sweep_round_results",
            "validation": "validation_round_results",
            "quick_correction": "quick_correction_results",
        }.get(mode, "round_results")
        return {
            "ok": True,
            "stop_reason": f"{event_prefix}_completed",
            "message": (
                f"{mode_name}完成：{len(targets)}档 × {passes_per_cycle}遍 × {cycle_count}轮"
                f" = {len(round_results)}次有效采样；"
                f"到位失败重试{failed_attempts}次"
            ),
            "final_force_n": (
                None
                if last_release_result is None
                else last_release_result.get("final_force_n")
            ),
            results_key: round_results,
            "sweep_failed_attempts": failed_attempts,
        }

    def _wait_for_all_floating_stable(
        self,
        motor,
        tracker,
        stop_event,
        max_current_ma,
        force_limit_n,
        round_index,
        total_rounds,
    ):
        deadline = time.monotonic() + AUTO_FLOATING_TIMEOUT_SECONDS
        recovery_generation = self._communication_recovery_generation
        stable_since = None
        confirmed_frames = 0
        raw_frames = []
        last_sequence = None
        latest_sample = None
        last_progress_at = 0.0
        while time.monotonic() < deadline:
            feedback = self._poll_checked(
                motor,
                tracker,
                stop_event,
                max_current_ma,
                force_limit_n,
                "press",
            )
            if recovery_generation != self._communication_recovery_generation:
                recovery_generation = self._communication_recovery_generation
                deadline = time.monotonic() + AUTO_FLOATING_TIMEOUT_SECONDS
                stable_since = None
                confirmed_frames = 0
                raw_frames = []
            now = time.monotonic()
            sample = self._latest_sensor_sample()
            sample_valid = self._sensor_sample_is_current(sample)
            if sample_valid and sample["sequence"] != last_sequence:
                last_sequence = sample["sequence"]
                latest_sample = sample
                force_values = self._sensor_force_values(sample)
                all_floating = (
                    force_values["floating_count"] == len(SENSOR_CHANNELS)
                )
                raw_values = tuple(sample.get("hardware_raw", sample.get("raw")) or ())
                try:
                    selected_raw = tuple(
                        float(raw_values[channel])
                        for channel in SENSOR_CHANNELS
                    )
                except (IndexError, TypeError, ValueError):
                    selected_raw = ()
                raw_valid = (
                    len(selected_raw) == len(SENSOR_CHANNELS)
                    and all(math.isfinite(value) for value in selected_raw)
                )
                if all_floating and raw_valid:
                    if stable_since is None:
                        stable_since = now
                        confirmed_frames = 1
                        raw_frames = []
                    else:
                        confirmed_frames += 1
                    raw_frames.append(selected_raw)
                else:
                    stable_since = None
                    confirmed_frames = 0
                    raw_frames = []
                stable_seconds = 0.0 if stable_since is None else now - stable_since
                if (
                    confirmed_frames >= AUTO_FLOATING_MIN_FRAMES
                    and stable_seconds >= AUTO_FLOATING_STABLE_SECONDS
                ):
                    medians = tuple(
                        float(statistics.median(
                            frame[channel_index] for frame in raw_frames
                        ))
                        for channel_index in range(len(SENSOR_CHANNELS))
                    )
                    return {
                        "sample": latest_sample,
                        "motor_feedback": dict(feedback),
                        "raw_medians": medians,
                        "raw_median_sum": sum(medians),
                        "raw_median_sample_count": len(raw_frames),
                        "raw_median_window_seconds": stable_seconds,
                    }
                if now - last_progress_at >= CONTROL_SUMMARY_REFRESH_SECONDS:
                    last_progress_at = now
                    self.operation_progress.emit(
                        {
                            "phase": "auto_floating_wait",
                            "current_force_n": self._ocr_snapshot().get(
                                "accepted_force_n"
                            ),
                            "floating_count": force_values["floating_count"],
                            "floating_required": len(SENSOR_CHANNELS),
                            "stable_seconds": stable_seconds,
                            "stable_required_seconds": AUTO_FLOATING_STABLE_SECONDS,
                            "raw_sample_count": len(raw_frames),
                            "message": (
                                f"采集任务第 {round_index}/{total_rounds} 轮："
                                f"FLOATING {force_values['floating_count']}/4，"
                                f"连续稳定 {stable_seconds:.1f}/"
                                f"{AUTO_FLOATING_STABLE_SECONDS:.1f} 秒"
                            ),
                        }
                    )
            elif not sample_valid:
                stable_since = None
                confirmed_frames = 0
                raw_frames = []
            if stop_event.wait(AUTO_SENSOR_POLL_SECONDS):
                raise ForceControlError("user_stop", "用户停止采集任务")

        state_text = "无有效传感器数据"
        if latest_sample is not None:
            states = tuple(latest_sample.get("state") or ())
            state_text = "、".join(
                f"CH{channel}={states[channel]}" for channel in SENSOR_CHANNELS
            )
        raise ForceControlError(
            "floating_timeout",
            f"第 {round_index} 轮等待CH4～CH7全部FLOATING连续稳定"
            f"{AUTO_FLOATING_STABLE_SECONDS:g}秒超时：{state_text}",
        )

    def _wait_for_continuous_zero(
        self,
        motor,
        tracker,
        stop_event,
        max_current_ma,
    ):
        """电机停止期间逐帧确认压力计连续1秒处于释放零点容差内。"""
        deadline = time.monotonic() + PRESSURE_STABILITY_TIMEOUT_SECONDS
        recovery_generation = self._communication_recovery_generation
        first_snapshot = self._ocr_snapshot()
        seen_event_sequence = int(first_snapshot.get("sequence", 0))
        zero_since = None
        last_zero_sample_at = None
        zero_samples = 0
        last_progress_at = 0.0
        last_reason = (
            f"等待新的压力计 ±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N 读数"
        )
        confirmed_force = None
        while time.monotonic() < deadline:
            self._poll_checked(
                motor,
                tracker,
                stop_event,
                max_current_ma,
                None,
                "release",
            )
            if recovery_generation != self._communication_recovery_generation:
                recovery_generation = self._communication_recovery_generation
                if not getattr(self, "_creep_active", False):
                    deadline = time.monotonic() + PRESSURE_STABILITY_TIMEOUT_SECONDS
                zero_since = None
                last_zero_sample_at = None
                zero_samples = 0
                last_reason = (
                    "通信恢复后重新确认压力计处于 "
                    f"±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N"
                )

            now = time.monotonic()
            ocr = self._ocr_snapshot()
            event_sequence = int(ocr.get("sequence", 0))
            if event_sequence > seen_event_sequence:
                seen_event_sequence = event_sequence
                accepted = bool(ocr.get("accepted"))
                value = ocr.get("accepted_force_n")
                fresh = (
                    float(ocr.get("accepted_age_seconds", math.inf))
                    <= PRESSURE_FRESH_TIMEOUT_SECONDS
                )
                if accepted and fresh and value is not None:
                    value = float(value)
                    if not pressure_n_is_within_start_zero_tolerance(value):
                        self._record_event(
                            "release_zero_interrupted",
                            raw_pressure_n=value,
                            message=(
                                f"释放零点确认被 {value:.2f} N中断，继续释放"
                            ),
                        )
                        return value
                    confirmed_force = value
                    if zero_since is None:
                        zero_since = now
                        zero_samples = 1
                    else:
                        zero_samples += 1
                    last_zero_sample_at = now
                    last_reason = (
                        f"压力计 {value:.2f} N 保持在 "
                        f"±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N 内"
                    )
                else:
                    zero_since = None
                    last_zero_sample_at = None
                    zero_samples = 0
                    last_reason = (
                        ocr.get("rejection_reason") or "压力计读数无效，重新计时"
                    )

            if (
                zero_since is not None
                and last_zero_sample_at is not None
                and now - last_zero_sample_at
                > RELEASE_ZERO_SAMPLE_MAX_GAP_SECONDS
            ):
                zero_since = None
                last_zero_sample_at = None
                zero_samples = 0
                last_reason = "压力计释放零点读数中断，重新计时"
            stable_seconds = 0.0 if zero_since is None else now - zero_since
            if (
                zero_samples >= RELEASE_ZERO_MIN_SAMPLES
                and stable_seconds >= RELEASE_ZERO_CONFIRM_SECONDS
            ):
                self._record_event(
                    "release_zero_confirmed",
                    raw_pressure_n=confirmed_force,
                    message=(
                        f"压力计连续{stable_seconds:.2f}秒处于 "
                        f"±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N，释放确认成功"
                    ),
                )
                return confirmed_force
            if now - last_progress_at >= 0.1:
                last_progress_at = now
                self.operation_progress.emit(
                    {
                        "phase": "release_zero_confirm",
                        "current_force_n": ocr.get("accepted_force_n"),
                        "stable_samples": zero_samples,
                        "stable_seconds": stable_seconds,
                        "stable_required_seconds": RELEASE_ZERO_CONFIRM_SECONDS,
                        "message": (
                            "电机已停止，确认压力计处于 "
                            f"±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N："
                            f"{stable_seconds:.1f}/{RELEASE_ZERO_CONFIRM_SECONDS:g}秒；"
                            f"{last_reason}"
                        ),
                    }
                )
            if stop_event.wait(AUTO_SENSOR_POLL_SECONDS):
                raise ForceControlError("user_stop", "用户请求停止释放")

        raise ForceControlError(
            "ocr_zero_confirmation_timeout",
            f"等待压力计在 ±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N 内连续"
            f"{RELEASE_ZERO_CONFIRM_SECONDS:g}秒超时：{last_reason}",
            brake_required=False,
        )

    def _wait_for_sensor_idle_after_release(
        self,
        motor,
        tracker,
        stop_event,
        parameters,
    ):
        """回零后确认状态、零Signal与Raw稳定；超时只告警。"""
        required_seconds = max(
            0.1,
            float(parameters["sensor_idle_stable_seconds"]),
        )
        timeout_seconds = max(
            required_seconds,
            float(parameters["sensor_idle_timeout_seconds"]),
        )
        started_at = time.monotonic()
        deadline = started_at + timeout_seconds
        stable_since = None
        confirmed_frames = 0
        last_sequence = None
        latest_states = None
        latest_signals = None
        raw_frames = []
        latest_raw_span = 0.0
        last_progress_at = 0.0
        while time.monotonic() < deadline:
            self._poll_checked(
                motor,
                tracker,
                stop_event,
                parameters["max_current_ma"],
                None,
                "release",
            )
            now = time.monotonic()
            sample = self._latest_sensor_sample()
            sample_valid = self._sensor_sample_is_current(sample)
            if sample_valid and sample["sequence"] != last_sequence:
                last_sequence = sample["sequence"]
                latest_states = self._sensor_channel_states(sample)
                force_values = self._sensor_force_values(sample)
                latest_signals = (
                    None if force_values is None else force_values["signals"]
                )
                raw_values = tuple(sample.get("hardware_raw", sample.get("raw")) or ())
                try:
                    selected_raw = tuple(
                        float(raw_values[channel])
                        for channel in SENSOR_CHANNELS
                    )
                except (IndexError, TypeError, ValueError):
                    selected_raw = ()
                raw_valid = (
                    len(selected_raw) == len(SENSOR_CHANNELS)
                    and all(math.isfinite(value) for value in selected_raw)
                )
                states_ready = (
                    latest_states is not None
                    and all(
                        state in RELEASE_READY_STATES
                        for state in latest_states
                    )
                )
                signals_zero = (
                    latest_signals is not None
                    and all(value == 0 for value in latest_signals)
                )
                if states_ready and signals_zero and raw_valid:
                    if stable_since is None:
                        stable_since = now
                        confirmed_frames = 1
                        raw_frames = [selected_raw]
                    else:
                        confirmed_frames += 1
                        raw_frames.append(selected_raw)
                    channel_spans = tuple(
                        max(frame[index] for frame in raw_frames)
                        - min(frame[index] for frame in raw_frames)
                        for index in range(len(SENSOR_CHANNELS))
                    )
                    latest_raw_span = max(channel_spans, default=0.0)
                    if (
                        latest_raw_span
                        > SENSOR_RELEASE_RAW_SPAN_COUNTS
                    ):
                        stable_since = now
                        confirmed_frames = 1
                        raw_frames = [selected_raw]
                        latest_raw_span = 0.0
                else:
                    stable_since = None
                    confirmed_frames = 0
                    raw_frames = []
                    latest_raw_span = 0.0
                stable_seconds = (
                    0.0 if stable_since is None else now - stable_since
                )
                if (
                    confirmed_frames >= AUTO_FLOATING_MIN_FRAMES
                    and stable_seconds >= required_seconds
                ):
                    waited_seconds = now - started_at
                    state_names = tuple(
                        STATE_NAMES[state]
                        for state in latest_states
                    )
                    state_text = "、".join(
                        f"CH{channel}={state_name}"
                        for channel, state_name in zip(
                            SENSOR_CHANNELS,
                            state_names,
                        )
                    )
                    self._record_event(
                        "release_sensor_ready_confirmed",
                        release_idle_recovered=True,
                        release_idle_wait_seconds=waited_seconds,
                        release_idle_stable_seconds=required_seconds,
                        release_idle_timeout_seconds=timeout_seconds,
                        message=(
                            f"回零后{state_text}，四路Signal为0，Raw最大极差"
                            f"{latest_raw_span:.1f} counts；连续稳定"
                            f"{stable_seconds:.2f}秒，"
                            f"等待共{waited_seconds:.2f}秒，允许开始下一轮"
                        ),
                    )
                    self.operation_progress.emit(
                        {
                            "phase": "release_sensor_idle_ready",
                            "idle_count": len(SENSOR_CHANNELS),
                            "idle_required": len(SENSOR_CHANNELS),
                            "stable_seconds": stable_seconds,
                            "stable_required_seconds": required_seconds,
                            "idle_wait_elapsed_seconds": waited_seconds,
                            "raw_span_counts": latest_raw_span,
                            "message": (
                                "CH4～CH7状态、零Signal与Raw均已稳定，"
                                "准备开始下一轮"
                            ),
                        }
                    )
                    return {
                        "recovered": True,
                        "wait_seconds": waited_seconds,
                        "stable_seconds": stable_seconds,
                        "timeout_seconds": timeout_seconds,
                        "states": latest_states,
                    }
            elif not sample_valid:
                stable_since = None
                confirmed_frames = 0
                raw_frames = []
                latest_raw_span = 0.0

            stable_seconds = 0.0 if stable_since is None else now - stable_since
            if now - last_progress_at >= CONTROL_SUMMARY_REFRESH_SECONDS:
                last_progress_at = now
                ready_count = sum(
                    state in RELEASE_READY_STATES
                    for state in (latest_states or ())
                )
                remaining = max(0.0, deadline - now)
                self.operation_progress.emit(
                    {
                        "phase": "release_sensor_idle_wait",
                        "idle_count": ready_count,
                        "idle_required": len(SENSOR_CHANNELS),
                        "stable_seconds": stable_seconds,
                        "stable_required_seconds": required_seconds,
                        "idle_wait_remaining_seconds": remaining,
                        "raw_span_counts": latest_raw_span,
                        "message": (
                            "回零后等待CH4～CH7恢复："
                            f"IDLE/WAIT/RECOVER/SLOW_BACK {ready_count}/4，"
                            f"零Signal={'是' if latest_signals is not None and all(value == 0 for value in latest_signals) else '否'}，"
                            f"Raw极差{latest_raw_span:.1f}/"
                            f"{SENSOR_RELEASE_RAW_SPAN_COUNTS:.1f} counts，"
                            f"稳定{stable_seconds:.1f}/{required_seconds:.1f}秒；"
                            f"最长等待剩余{remaining:.1f}秒"
                        ),
                    }
                )
            if stop_event.wait(AUTO_SENSOR_POLL_SECONDS):
                raise ForceControlError("user_stop", "用户请求紧急停止")

        waited_seconds = time.monotonic() - started_at
        state_text = (
            "无有效传感器数据"
            if latest_states is None
            else "、".join(
                f"CH{channel}={state}"
                for channel, state in zip(SENSOR_CHANNELS, latest_states)
            )
        )
        message = (
            f"回零后等待状态/零Signal/Raw稳定达到{timeout_seconds:.1f}秒上限："
            f"{state_text}；已记录警告并继续下一轮"
        )
        self._record_event(
            "release_sensor_recovery_timeout_continue",
            release_idle_recovered=False,
            release_idle_wait_seconds=waited_seconds,
            release_idle_stable_seconds=required_seconds,
            release_idle_timeout_seconds=timeout_seconds,
            message=message,
        )
        self.operation_progress.emit(
            {
                "phase": "release_sensor_idle_timeout_continue",
                "idle_count": sum(
                    state in RELEASE_READY_STATES
                    for state in (latest_states or ())
                ),
                "idle_required": len(SENSOR_CHANNELS),
                "stable_seconds": 0.0,
                "stable_required_seconds": required_seconds,
                "idle_wait_remaining_seconds": 0.0,
                "raw_span_counts": latest_raw_span,
                "message": message,
            }
        )
        return {
            "recovered": False,
            "wait_seconds": waited_seconds,
            "stable_seconds": 0.0,
            "timeout_seconds": timeout_seconds,
            "states": latest_states,
        }

    def _creep_progress(self, phase, trial, trial_total, **values):
        """向长时测试页发送带完整试验条件的阶段事件。"""
        payload = {
            "phase": str(phase),
            "controller_monotonic": time.monotonic(),
            "trial_index": int(trial["trial_index"]),
            "planned_trial_index": int(trial.get("planned_trial_index", trial["trial_index"])),
            "attempt_index": int(trial.get("attempt_index", 0)),
            "history_interrupted": bool(trial.get("history_interrupted", False)),
            "trial_total": int(trial_total),
            "repeat_index": int(trial.get("repeat_index", 0)),
            "category": str(trial.get("category", "response")),
            "high_force_n": float(trial.get("high_force_n", 0.0)),
            "high_hold_seconds": float(trial.get("high_hold_seconds", 0.0)),
            "pre_observation_seconds": float(trial.get("pre_observation_seconds", 0.0)),
            "recovery_seconds": float(trial.get("recovery_seconds", 0.0)),
            "probe_force_n": float(trial.get("probe_force_n", 0.0)),
            "probe_hold_seconds": float(trial.get("probe_hold_seconds", 0.0)),
            "process_index": int(trial.get("process_index", 0)),
        }
        payload.update(values)
        self.operation_progress.emit(payload)

    def _run_creep_wait(
        self,
        motor,
        tracker,
        stop_event,
        parameters,
        trial,
        trial_total,
        phase,
        duration_seconds,
    ):
        """等待期间持续检查压力、电流和驱动状态，并按固定节拍报告进度。"""
        duration_seconds = max(0.0, float(duration_seconds))
        started_at = time.monotonic()
        deadline = started_at + duration_seconds
        last_progress_at = 0.0
        outside_since = None
        outside_samples = 0
        last_feedback_at = None
        motion_status = "normal"
        while True:
            now = time.monotonic()
            elapsed = min(duration_seconds, max(0.0, now - started_at))
            remaining = max(0.0, deadline - now)
            if now - last_progress_at >= 0.5 or remaining <= 0.0:
                last_progress_at = now
                self._creep_progress(
                    phase,
                    trial,
                    trial_total,
                    phase_elapsed_seconds=elapsed,
                    phase_duration_seconds=duration_seconds,
                    phase_remaining_seconds=remaining,
                    current_force_n=self._ocr_snapshot().get("accepted_force_n"),
                    message=(
                        f"长时测试 {trial['trial_index']}/{trial_total}："
                        f"{phase} {elapsed:.1f}/{duration_seconds:.1f}秒"
                    ),
                )
            if remaining <= 0.0:
                return
            observation = (getattr(self, "_creep_rebound_active", False) and phase in (
                "creep_pre_observation", "creep_recovery_wait", "creep_retry_recovery", "creep_inter_trial_wait"))
            feedback = self._poll_checked(
                motor,
                tracker,
                stop_event,
                parameters["max_current_ma"],
                MAX_OPERATION_FORCE_N,
                "observe" if observation else "press",
            )
            if observation:
                feedback_at = float(feedback.get("sampled_monotonic", time.monotonic()))
                if last_feedback_at is None or feedback_at > last_feedback_at:
                    last_feedback_at = feedback_at
                    position = int(feedback["position_counts"])
                    speed = int(feedback["measured_speed_rpm"])
                    offset = position - int(parameters["release_position_counts"])
                    outside = abs(offset) > getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS)
                    if outside:
                        if outside_since is None:
                            outside_since = feedback_at
                        outside_samples += 1
                    else:
                        outside_since = None
                        outside_samples = 0
                    status = "position_outside" if outside else (
                        "speed_only" if abs(speed) > MOTION_STOP_SPEED_RPM else "normal")
                    if status != motion_status:
                        motion_status = status
                        self.manual_observation_event.emit({
                            "event": "observation_motion_check", "mode": "creep_test",
                            "at": feedback_at, "status": status, "position_counts": position,
                            "release_position_counts": int(parameters["release_position_counts"]),
                            "position_offset_counts": offset, "measured_speed_rpm": speed,
                        })
                    # 位置留在释放容差内时，单独的速度异常不能证明电机离位。
                    if (outside_samples >= REBOUND_MOTION_CONFIRM_SAMPLES
                            and feedback_at - outside_since >= REBOUND_MOTION_CONFIRM_SECONDS):
                        raise ForceControlError(
                            "creep_unexpected_motion",
                            f"空载位置连续{outside_samples}次越界、持续{feedback_at - outside_since:.2f}秒："
                            f"实际{position}，相对释放位置偏差{offset:+d} counts"
                            f"（容差±{getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS)}），速度{speed} rpm，停止并卸载")
            if stop_event.wait(min(0.1, remaining)):
                raise ForceControlError("user_stop", "用户停止长时蠕变测试")

    @staticmethod
    def _creep_force_parameters(parameters, lookup, target_force_n):
        """为一次高压或探针下压生成独立查表参数。"""
        target = dict(parameters)
        target.update(
            target_force_n=float(target_force_n),
            lookup_press_travel_counts=int(lookup["press_travel_counts"]),
            lookup_target_position_counts=int(lookup["target_position_counts"]),
            lookup_position_span_counts=float(lookup["position_span_counts"]),
            lookup_created_at=lookup["created_at"],
            wait_for_sensor_idle_after_release=False,
        )
        return target

    def _run_creep_press(self, motor, tracker, stop_event, parameters):
        bounded = CreepDeadlineEvent(stop_event, CREEP_PRESS_TIMEOUT_SECONDS, ForceControlError)
        if (parameters.get("sweep_plan") or {}).get("protocol") == REBOUND_PROTOCOL:
            return self._run_rebound_press(motor, tracker, bounded, parameters)
        return self._run_press(motor, tracker, bounded, parameters)

    def _run_rebound_press(self, motor, tracker, stop_event, parameters):
        """回弹辨识只按查表位置加载，到位即返回，不等待力稳定或补压。"""
        zero_deadline = time.monotonic() + REBOUND_ZERO_WAIT_SECONDS
        while True:
            self._poll_checked(motor, tracker, stop_event, parameters['max_current_ma'], MAX_OPERATION_FORCE_N, 'observe')
            sample = self._ocr_snapshot()
            if near_rebound_zero(sample.get('accepted_force_n')) and float(sample.get('accepted_age_seconds', math.inf)) <= PRESSURE_FRESH_TIMEOUT_SECONDS:
                break
            if time.monotonic() >= zero_deadline:
                raise ForceControlError('press_start_force_not_zero', '下压前等待10秒仍未获得±0.10 N内的新鲜零点读数')
            stop_event.wait(0.1)
        feedback = self._poll_checked(motor, tracker, stop_event, parameters['max_current_ma'], MAX_OPERATION_FORCE_N, 'press')
        target = int(parameters['lookup_target_position_counts'])
        origin = int(feedback['position_counts'])
        if abs(origin-int(parameters['release_position_counts'])) > getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS):
            raise ForceControlError('release_position_error', '回弹测试下压前不在标定释放位置')
        delta = target-origin
        if delta >= 0 or abs(delta) > parameters['max_travel_counts']:
            raise ForceControlError('lookup_direction_invalid', '回弹测试查表方向或行程无效')
        started = time.monotonic()
        feedback = self._move_relative(
            motor, tracker, stop_event, delta, parameters['press_speed_level'],
            parameters['acceleration'], parameters['max_current_ma'], MAX_OPERATION_FORCE_N,
            'press', abs(delta), require_zero_force_start=True,
            motion_speed_name='回弹测试查表下压',
            position_tolerance_counts=REBOUND_POSITION_TOLERANCE_COUNTS,
            fixed_target_counts=target, zero_force_tolerance_n=REBOUND_ZERO_TOLERANCE_N,
        )
        return dict(final_position_counts=feedback['position_counts'],
                    final_force_n=self._ocr_snapshot().get('accepted_force_n'),
                    motion_started_monotonic=started, arrived_monotonic=time.monotonic(),
                    lookup_target_position_counts=target,
                    lookup_created_at=parameters.get('lookup_created_at'),
                    position_error_counts=int(feedback['position_counts'])-target)

    def _run_creep_release(self, motor, tracker, stop_event, parameters):
        """回标定释放位置并确认压力归零，不重设电机坐标。"""
        bounded = CreepDeadlineEvent(stop_event, CREEP_RELEASE_TIMEOUT_SECONDS, ForceControlError)
        rebound = (parameters.get("sweep_plan") or {}).get("protocol") == REBOUND_PROTOCOL
        release = dict(parameters, wait_for_sensor_idle_after_release=False)
        command_at = time.monotonic()
        result = self._run_release(motor, tracker, bounded, release)
        result['release_command_monotonic'] = command_at
        result['release_position_arrived_monotonic'] = time.monotonic()
        deadline = time.monotonic() + (REBOUND_ZERO_WAIT_SECONDS if rebound else 3.0)
        seen = self._ocr_snapshot().get("accepted_sequence")
        zero_samples = 0
        while time.monotonic() < deadline:
            feedback = self._poll_checked(motor, tracker, bounded, parameters["max_current_ma"], None, "release")
            sample = self._ocr_snapshot()
            sequence = sample.get("accepted_sequence")
            if sequence != seen:
                seen = sequence
                force = sample.get("accepted_force_n")
                valid = (force is not None
                         and float(sample.get("accepted_age_seconds", math.inf)) <= PRESSURE_FRESH_TIMEOUT_SECONDS
                         and (near_rebound_zero(force) if rebound else pressure_n_is_within_press_start_tolerance(force))
                         and abs(int(feedback["position_counts"]) - int(parameters["release_position_counts"])) <= getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS)
                         and abs(int(feedback["measured_speed_rpm"])) <= MOTION_STOP_SPEED_RPM)
                zero_samples = zero_samples + 1 if valid else 0
                if zero_samples >= 3:
                    result["final_force_n"] = float(force)
                    result['release_zero_confirmed_monotonic'] = time.monotonic()
                    return result
            bounded.wait(0.08)
        raise ForceControlError("creep_release_unconfirmed", "回零后未确认位置静止且压力计连续归零，禁止继续下压")

    def _creep_emergency_release(self, motor, tracker, parameters):
        """异常后独立卸载；用户停止事件不能取消必要的回零。"""
        trial = getattr(self, "_creep_current_trial", {"trial_index": 0})
        total = len(parameters.get("creep_trials", ()))
        last_error = None
        for attempt in range(1, 3):
            self._creep_progress("creep_safety_release", trial, total,
                                 message=f"异常卸载：第{attempt}/2次回到标定释放位置")
            self._safe_brake(motor)
            try:
                bounded = CreepDeadlineEvent(threading.Event(), CREEP_RELEASE_TIMEOUT_SECONDS, ForceControlError)
                self._call_with_motor_communication_retry("卸载前清除故障状态", motor.clear_state, bounded, motion_mode="release")
                tracker.clear()
                result = self._run_creep_release(motor, tracker, bounded, parameters)
                self._creep_progress("creep_safety_released", trial, total,
                                     release_result=result, message="已确认回到释放位置且压力计归零")
                return result
            except Exception as exc:
                last_error = exc
        self._safe_brake(motor)
        raise ForceControlError("creep_release_failed",
                                f"危险：两次自动卸载均未确认成功，可能仍受压，请立即人工卸载；{last_error}")

    def _finish_creep_failure(self, motor, tracker, parameters, result):
        """准备阶段、未知异常和用户停止均在退出前尝试卸载。"""
        if result.get("stop_reason") == "creep_release_failed":
            result["safety_released"] = False
            return
        try:
            self._creep_emergency_release(motor, tracker, parameters)
            result["safety_released"] = True
            result["message"] = result.get("message", "") + "；已确认卸载，数据和断点已保留"
        except Exception as exc:
            result.update(safety_released=False, stop_reason="creep_release_failed",
                          message=result.get("message", "") + "；" + str(exc))

    def _run_creep_test(self, motor, tracker, stop_event, parameters):
        """失败尝试留档，先卸载再重试；成功与待补测条件分别记录。"""
        trials = tuple(parameters.get("creep_trials", ()))
        if not trials:
            raise ForceControlError("creep_plan_empty", "长时测试计划为空", brake_required=False)
        trial_total = len(trials)
        plan = parameters.get("sweep_plan") or {}
        completed, skipped = [], []
        consecutive_failures = 0
        history_interrupted = bool(plan.get("resumed_from"))
        release_parameters = dict(parameters, wait_for_sensor_idle_after_release=False)
        self._creep_current_trial = dict(trials[0], trial_index=1,
                                        history_interrupted=history_interrupted)
        self._creep_progress("creep_initial_release", self._creep_current_trial, trial_total,
                             message="启动前自动回到标定释放位置并确认卸载")
        self._run_creep_release(motor, tracker, stop_event, release_parameters)
        if plan.get("resumed_from"):
            self._run_creep_wait(motor, tracker, stop_event, parameters,
                                 self._creep_current_trial, trial_total, "creep_retry_recovery",
                                 CREEP_RECOVERY_SECONDS)
        for index, source in enumerate(trials, 1):
            trial = dict(source, trial_index=index)
            self._creep_current_trial = trial
            for attempt in range(1, CREEP_MAX_ATTEMPTS + 1):
                trial["attempt_index"] = attempt
                trial["history_interrupted"] = history_interrupted
                try:
                    result = self._run_creep_trial(motor, tracker, stop_event, parameters,
                                                  trial, trial_total, release_parameters)
                except ForceControlError as exc:
                    history_interrupted = True
                    self._creep_progress("creep_trial_failed", trial, trial_total,
                                         reason=exc.reason, data_valid=False, message=str(exc))
                    self._creep_emergency_release(motor, tracker, parameters)
                    if stop_event.is_set() or exc.reason not in RECOVERABLE_CREEP_ERRORS:
                        raise
                    # 异常加载改变了压力历史，记录恢复段后从本条件开头重做。
                    self._run_creep_wait(motor, tracker, stop_event, parameters, trial,
                                         trial_total, "creep_retry_recovery", CREEP_RECOVERY_SECONDS)
                    if attempt == CREEP_MAX_ATTEMPTS:
                        skipped.append(dict(trial))
                        consecutive_failures += 1
                        self._creep_progress("creep_trial_skipped", trial, trial_total,
                                             reason=exc.reason, data_valid=False,
                                             message="本条件三次失败，保留待补测并继续下一条件")
                        if consecutive_failures >= CREEP_MAX_CONSECUTIVE_FAILURES:
                            raise ForceControlError("creep_repeated_failure", "连续三个条件失败，已卸载并保留断点，请检查电机或标定表")
                    continue
                completed.append(result)
                history_interrupted = False
                consecutive_failures = 0
                self._creep_progress("creep_trial_complete", trial, trial_total,
                                     trial_result=result, data_valid=True,
                                     phase_elapsed_seconds=0.0,
                                     message=f"条件 {index}/{trial_total} 已完成")
                break
            interval = float(plan.get("inter_trial_seconds", 0))
            if trial.get('category') == 'paired_rebound':
                # 组合测试在小压力释放后记录完整休息段，不在这里重复等待。
                interval = 0.0
            if index < trial_total and interval > 0:
                self._run_creep_wait(motor, tracker, stop_event, parameters, trial,
                                     trial_total, "creep_inter_trial_wait", interval)
        with self._state_lock:
            self._target_force_n = None
        return {"ok": True, "stop_reason": "creep_test_completed",
                "message": f"长时测试完成{len(completed)}/{trial_total}项，待补测{len(skipped)}项，已卸载",
                "creep_completed_trials": completed, "creep_skipped_trials": skipped}

    def _run_creep_trial(self, motor, tracker, stop_event, parameters, trial, trial_total, release_parameters):
        """执行一个完整条件；失败时由外层卸载并重试。"""
        if trial.get('category') == 'paired_rebound':
            return self._run_paired_rebound_trial(motor, tracker, stop_event, parameters, trial, trial_total, release_parameters)
        if trial.get('category') == 'raw_rebound':
            return self._run_rebound_trial(motor, tracker, stop_event, parameters, trial, trial_total, release_parameters)
        index = trial["trial_index"]
        trial_started_at = time.monotonic()
        with self._state_lock:
            self._auto_round_index = index
            self._auto_round_total = trial_total
            self._repeat_index = int(trial.get("repeat_index", 0))
            self._repeat_total = int(
                (parameters.get("sweep_plan") or {}).get("repeat_count", 0)
            )
        self._creep_progress(
            "creep_trial_start",
            trial,
            trial_total,
            phase_elapsed_seconds=0.0,
            message=(
                f"开始条件 {index}/{trial_total}：高压"
                f"{float(trial.get('high_force_n', 0.0)):g} N，额外保持"
                f"{float(trial.get('high_hold_seconds', 0.0)):g}秒，释放后等待"
                f"{float(trial.get('recovery_seconds', 0.0)):g}秒，"
                f"探针{float(trial.get('probe_force_n', 0.0)):g} N"
            ),
        )

        high_result = {}
        high_force_n = float(trial.get("high_force_n", 0.0))
        if high_force_n > 0.0:
            with self._state_lock:
                self._target_force_n = high_force_n
            self._creep_progress(
                "creep_high_press",
                trial,
                trial_total,
                message=f"正在下压到高压 {high_force_n:g} N",
            )
            high_parameters = self._creep_force_parameters(
                parameters,
                trial["high_lookup"],
                high_force_n,
            )
            high_result = self._run_creep_press(
                motor,
                tracker,
                stop_event,
                high_parameters,
            )
            self._run_creep_wait(
                motor,
                tracker,
                stop_event,
                parameters,
                trial,
                trial_total,
                "creep_high_hold",
                trial.get("high_hold_seconds", 0.0),
            )
            self._creep_progress(
                "creep_high_release",
                trial,
                trial_total,
                message="高压保持结束，正在回到固定释放位置",
            )
            self._run_creep_release(
                motor,
                tracker,
                stop_event,
                release_parameters,
            )
            with self._state_lock:
                self._target_force_n = None

        recovery_seconds = float(trial.get("recovery_seconds", 0.0))
        self._run_creep_wait(
            motor,
            tracker,
            stop_event,
            parameters,
            trial,
            trial_total,
            "creep_recovery_wait",
            recovery_seconds,
        )

        probe_force_n = float(trial["probe_force_n"])
        with self._state_lock:
            self._target_force_n = probe_force_n
        self._creep_progress(
            "creep_probe_press",
            trial,
            trial_total,
            message=f"恢复等待结束，正在执行 {probe_force_n:g} N探针测量",
        )
        probe_parameters = self._creep_force_parameters(
            parameters,
            trial["probe_lookup"],
            probe_force_n,
        )
        probe_result = self._run_creep_press(
            motor,
            tracker,
            stop_event,
            probe_parameters,
        )
        self._run_creep_wait(
            motor,
            tracker,
            stop_event,
            parameters,
            trial,
            trial_total,
            "creep_probe_hold",
            trial.get("probe_hold_seconds", 0.0),
        )
        self._creep_progress(
            "creep_probe_release",
            trial,
            trial_total,
            message="探针保持结束，正在回到固定释放位置",
        )
        release_result = self._run_creep_release(
            motor,
            tracker,
            stop_event,
            release_parameters,
        )
        with self._state_lock:
            self._target_force_n = None
        trial_result = {
            key: value
            for key, value in trial.items()
            if key not in ("high_lookup", "probe_lookup")
        }
        trial_result.update(
            trial_elapsed_seconds=time.monotonic() - trial_started_at,
            high_measured_force_n=high_result.get("final_force_n"),
            probe_measured_force_n=probe_result.get("final_force_n"),
            probe_position_counts=probe_result.get(
                "floating_capture_position_counts",
                probe_result.get("final_position_counts"),
            ),
            release_measured_force_n=release_result.get("final_force_n"),
        )
        return trial_result

    def _run_paired_rebound_trial(self, motor, tracker, stop_event, parameters, trial, trial_total, release_parameters):
        """大压力、指定空载间隔、小压力组成不可拆分的过程，失败时整段重做。"""
        started = time.monotonic()
        self._creep_progress('creep_trial_start', trial, trial_total, message='开始大压力后测小压力')
        with self._state_lock:
            self._auto_round_index = trial['trial_index']
            self._auto_round_total = trial_total
            self._target_force_n = None
        self._run_creep_wait(motor, tracker, stop_event, parameters, trial, trial_total,
                             'creep_pre_observation', trial['pre_observation_seconds'])
        parts = {}
        for role in ('high', 'probe'):
            force = float(trial[f'{role}_force_n'])
            before = self._latest_sensor_sample() or {}
            with self._state_lock:
                self._target_force_n = force
            self._creep_progress(f'creep_{role}_press', trial, trial_total,
                                 message=f'目标{force:g} N，查表下压一次', pre_press_sensor=before)
            press = self._run_creep_press(motor, tracker, stop_event,
                self._creep_force_parameters(parameters, trial[f'{role}_lookup'], force))
            self._run_creep_wait(motor, tracker, stop_event, parameters, trial, trial_total,
                                 f'creep_{role}_hold', trial[f'{role}_hold_seconds'])
            self._creep_progress(f'creep_{role}_release', trial, trial_total,
                message=f'{force:g} N保持结束，松开并确认卸载', press_result=press,
                release_sensor=self._latest_sensor_sample(), release_pressure=self._ocr_snapshot())
            release = self._run_creep_release(motor, tracker, stop_event, release_parameters)
            parts[role] = dict(press_result=press, release_result=release, pre_press_sensor=before)
            with self._state_lock:
                self._target_force_n = None
            self._run_creep_wait(motor, tracker, stop_event, parameters, trial, trial_total,
                'creep_recovery_wait' if role == 'high' else 'creep_inter_trial_wait',
                trial['recovery_seconds'] if role == 'high' else trial['post_observation_seconds'])
        return dict({k:v for k,v in trial.items() if k not in ('high_lookup','probe_lookup')},
            trial_elapsed_seconds=time.monotonic()-started, paired_results=parts,
            high_measured_force_n=parts['high']['press_result'].get('final_force_n'),
            probe_measured_force_n=parts['probe']['press_result'].get('final_force_n'),
            release_measured_force_n=parts['probe']['release_result'].get('final_force_n'),
            end_sensor=self._latest_sensor_sample(), completion_basis='complete_large_small_sequence',
            fully_recovered=None)

    def _run_rebound_trial(self, motor, tracker, stop_event, parameters, trial, trial_total, release_parameters):
        """每项只有一次加载；固定观察窗口不会被IDLE、Raw或压力稳定判据缩短。"""
        started = time.monotonic()
        self._creep_progress('creep_trial_start', trial, trial_total, message='开始单次按压回弹采集')
        with self._state_lock:
            self._auto_round_index = trial['trial_index']
            self._auto_round_total = trial_total
            self._target_force_n = None
        self._run_creep_wait(motor, tracker, stop_event, parameters, trial, trial_total,
                             'creep_pre_observation', trial['pre_observation_seconds'])
        before = self._latest_sensor_sample() or {}
        force = float(trial['high_force_n'])
        with self._state_lock:
            self._target_force_n = force
        self._creep_progress('creep_high_press', trial, trial_total, message=f'目标{force:g} N，查表下压一次',
                             pre_press_sensor=before)
        press_params = self._creep_force_parameters(parameters, trial['high_lookup'], force)
        press = self._run_creep_press(motor, tracker, stop_event, press_params)
        self._run_creep_wait(motor, tracker, stop_event, parameters, trial, trial_total,
                             'creep_high_hold', trial['high_hold_seconds'])
        self._creep_progress('creep_high_release', trial, trial_total, message='保持结束，释放并开始记录回弹',
                             press_result=press, release_sensor=self._latest_sensor_sample(),
                             release_pressure=self._ocr_snapshot())
        release = self._run_creep_release(motor, tracker, stop_event, release_parameters)
        with self._state_lock:
            self._target_force_n = None
        self._run_creep_wait(motor, tracker, stop_event, parameters, trial, trial_total,
                             'creep_recovery_wait', trial['recovery_seconds'])
        return dict({k:v for k,v in trial.items() if k not in ('high_lookup','probe_lookup')},
                    trial_elapsed_seconds=time.monotonic()-started,
                    high_measured_force_n=press.get('final_force_n'),
                    release_measured_force_n=release.get('final_force_n'),
                    press_result=press, release_result=release,
                    pre_press_sensor=before, end_sensor=self._latest_sensor_sample(),
                    completion_basis='fixed_observation_window', fully_recovered=None)

    def _run_release(self, motor, tracker, stop_event, parameters):
        max_current = parameters["max_current_ma"]
        feedback = self._poll_checked(
            motor,
            tracker,
            stop_event,
            max_current,
            None,
            "release",
        )
        start_position = int(feedback["position_counts"])
        target_position = int(parameters["release_position_counts"])
        delta_counts = target_position - start_position
        if delta_counts < -getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS):
            raise ForceControlError(
                "release_target_below_current",
                f"当前位置{start_position} counts已高于释放位置{target_position} counts；"
                "为避免反向下压，未自动退回，请在无压力位置重新设置释放位置",
                brake_required=False,
            )
        if abs(delta_counts) > int(parameters["max_travel_counts"]):
            raise ForceControlError(
                "release_position_travel_limit",
                f"到固定释放位置需要移动{abs(delta_counts)} counts，超过内部安全行程"
                f"{parameters['max_travel_counts']} counts",
                brake_required=False,
            )
        self._record_event(
            "release_position_started",
            command_counts=delta_counts,
            total_travel_counts=abs(delta_counts),
            message=(
                f"固定位置释放：{start_position} → {target_position} counts，"
                f"一次性相对移动{delta_counts:+d} counts"
            ),
        )
        if abs(delta_counts) > getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS):
            feedback = self._move_relative(
                motor,
                tracker,
                stop_event,
                delta_counts,
                parameters["release_speed_level"],
                parameters["acceleration"],
                max_current,
                None,
                "release",
                abs(delta_counts),
                motion_speed_name="固定释放位置",
                position_tolerance_counts=getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS),
                fixed_target_counts=target_position,
            )
        final_position = int(feedback["position_counts"])
        position_error = final_position - target_position
        latest_force = self._ocr_snapshot().get("accepted_force_n")
        final_force = None if latest_force is None else float(latest_force)
        force_text = "--" if final_force is None else f"{final_force:.2f}"
        total_travel = abs(final_position - start_position)
        self._set_motion_context(delta_counts, total_travel)
        self._record_event(
            "release_position_completed",
            raw_pressure_n="" if final_force is None else final_force,
            command_counts=delta_counts,
            total_travel_counts=total_travel,
            message=(
                f"固定位置释放完成：目标{target_position}，实际{final_position} counts，"
                f"误差{position_error:+d} counts，压力{force_text} N"
            ),
        )
        result = {
            "ok": True,
            "stop_reason": "released_to_fixed_position",
            "message": (
                f"已一次性返回固定释放位置附近：目标{target_position}，"
                f"实际{final_position} counts，误差{position_error:+d} counts，"
                f"压力{force_text} N"
            ),
            "final_force_n": final_force,
            "final_position_counts": final_position,
            "release_position_counts": target_position,
            "total_travel_counts": total_travel,
        }
        if parameters.get("wait_for_sensor_idle_after_release"):
            idle_result = self._wait_for_sensor_idle_after_release(
                motor,
                tracker,
                stop_event,
                parameters,
            )
            result.update(
                release_idle_recovered=bool(idle_result["recovered"]),
                release_idle_wait_seconds=float(idle_result["wait_seconds"]),
                release_idle_stable_seconds=float(idle_result["stable_seconds"]),
                release_idle_timeout_seconds=float(idle_result["timeout_seconds"]),
            )
        return result

    def _move_relative(
        self,
        motor,
        tracker,
        stop_event,
        delta_counts,
        speed_level,
        acceleration,
        max_current_ma,
        force_limit_n,
        mode,
        total_travel_counts,
        require_zero_force_start=False,
        motion_speed_name="",
        position_tolerance_counts=MOTION_POSITION_TOLERANCE_COUNTS,
        fixed_target_counts=None,
        zero_force_tolerance_n=None,
    ):
        feedback = self._poll_checked(
            motor,
            tracker,
            stop_event,
            max_current_ma,
            force_limit_n,
            mode,
        )
        start_counts = int(feedback["position_counts"])
        # 绝对查表目标应以发令前的最新位置计算位移，避免两次读位置间的漂移平移目标。
        if fixed_target_counts is not None:
            delta_counts = int(fixed_target_counts) - start_counts
            if mode == "release" and abs(delta_counts) <= getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS):
                return feedback
            if mode == "release" and delta_counts < -getattr(self, '_release_tolerance', RELEASE_POSITION_TOLERANCE_COUNTS):
                raise ForceControlError("release_target_below_current", "释放目标已在下压方向，拒绝反向运动")
            if mode == "press" and delta_counts >= 0:
                raise ForceControlError("lookup_direction_invalid", "下压目标已不在下压方向，拒绝运动")
            if abs(delta_counts) > MAX_OPERATION_TRAVEL_COUNTS:
                raise ForceControlError("travel_limit", "重新计算的位移超过安全行程")
        target_counts = start_counts + int(delta_counts)
        speed_counts_s = speed_level_to_counts_per_second(speed_level)
        speed_rpm = counts_per_second_to_rpm(speed_counts_s)
        timeout_seconds = max(
            3.0,
            abs(delta_counts) / max(1, speed_counts_s) * 4.0 + 2.0,
        )
        self._set_motion_context(delta_counts, total_travel_counts)
        direction_text = "下压" if mode == "press" else "抬起"
        self._record_event(
            "motion_command",
            command_counts=delta_counts,
            total_travel_counts=total_travel_counts,
            command_speed_rpm=speed_rpm,
            message=f"{motion_speed_name or direction_text}：目标位置 {target_counts} counts",
        )
        self.operation_progress.emit(
            {
                "phase": f"{mode}_moving",
                "motion_direction": mode,
                "current_force_n": self._ocr_snapshot().get("accepted_force_n"),
                "command_counts": delta_counts,
                "total_travel_counts": total_travel_counts,
                "speed_level": speed_level,
                "speed_name": motion_speed_name,
                "target_position_counts": target_counts,
                "message": (
                    f"电机正在{direction_text}：本步 {delta_counts:+d} counts，"
                    f"目标位置 {target_counts} counts"
                ),
            }
        )
        self._call_with_motor_communication_retry(
            "清除驱动器状态",
            motor.clear_state,
            stop_event,
            motion_mode=mode,
        )
        if require_zero_force_start:
            # 一次性下压位移发出前再次核对压力计零点。
            start_ocr = self._ocr_snapshot()
            start_force = start_ocr.get("accepted_force_n")
            if (
                start_force is None
                or float(start_ocr.get("accepted_age_seconds", math.inf))
                > PRESSURE_FRESH_TIMEOUT_SECONDS
                or not (pressure_n_is_within_press_start_tolerance(start_force)
                        if zero_force_tolerance_n is None else
                        math.isfinite(float(start_force)) and abs(float(start_force)) <= zero_force_tolerance_n)
            ):
                force_text = "无有效值" if start_force is None else f"{float(start_force):.2f} N"
                zero_range = (f"-{PRESSURE_METER_NEGATIVE_LIMIT_N:.2f}～+{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f}"
                              if zero_force_tolerance_n is None else f"±{zero_force_tolerance_n:.2f}")
                raise ForceControlError(
                    "press_start_force_not_zero",
                    f"下压命令发出前压力计为{force_text}，"
                    f"超出零点容差 {zero_range} N，已拒绝运动",
                    brake_required=False,
                )
        # 保存发令前的真实通道和压力，收到驱动器接受结果后再通知观察页。
        observation = {
            "event": "motion_dispatched", "at": time.monotonic(),
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "operation_id": self._operation_id, "mode": self._control_mode,
            "direction": mode, "target_force_n": self._target_force_n,
            "sensor": self._latest_sensor_sample(), "pressure": self._ocr_snapshot(),
        }
        move_result = self._call_with_motor_communication_retry(
            f"发送{direction_text}位移",
            lambda: motor.move_relative_counts(
                int(delta_counts),
                speed_rpm=speed_rpm,
                accel=acceleration,
                retry_cancelled=stop_event.is_set,
            ),
            stop_event,
            motion_mode=mode,
        )
        observation["ack_at"] = time.monotonic()
        self.manual_observation_event.emit(observation)
        response_recovered = bool(
            isinstance(move_result, dict)
            and move_result.get("response_recovered")
        )
        if response_recovered:
            self._communication_recovery_generation += 1
            self._record_event(
                "motion_response_recovered",
                command_counts=delta_counts,
                total_travel_counts=total_travel_counts,
                message=(
                    "运动命令应答异常，但已通过位置复核恢复；"
                    "未重复发送已经开始执行的位移"
                ),
        )
        deadline = time.monotonic() + timeout_seconds
        recovery_generation = self._communication_recovery_generation
        stable_positions = 0
        last_position = None
        minimum_movement = min(10, max(1, abs(delta_counts) // 4))
        while True:
            if stop_event.is_set():
                raise ForceControlError("user_stop", "用户请求紧急停止")
            feedback = self._poll_checked(
                motor,
                tracker,
                stop_event,
                max_current_ma,
                force_limit_n,
                mode,
            )
            if recovery_generation != self._communication_recovery_generation:
                recovery_generation = self._communication_recovery_generation
                if not getattr(self, "_creep_active", False):
                    deadline = time.monotonic() + timeout_seconds
            position = int(feedback["position_counts"])
            moved = abs(position - start_counts) >= minimum_movement
            if (
                last_position is not None
                and abs(position - last_position) <= MOTION_STABLE_DELTA_COUNTS
            ):
                stable_positions += 1
            else:
                stable_positions = 0
            if (
                abs(position - target_counts) <= int(position_tolerance_counts)
                and abs(int(feedback["measured_speed_rpm"]))
                <= MOTION_STOP_SPEED_RPM
                and stable_positions >= MOTION_STABLE_SAMPLE_COUNT
            ):
                if feedback["running_state"] == 2:
                    self._record_event(
                        "motion_position_settled",
                        message=(
                            "位置和速度已连续稳定，驱动器运行状态仍为2，"
                            "按实际到位完成本段运动"
                        ),
                    )
                return feedback
            if (
                response_recovered
                and moved
                and abs(int(feedback["measured_speed_rpm"]))
                <= MOTION_STOP_SPEED_RPM
                and stable_positions >= (
                    MOTION_STABLE_SAMPLE_COUNT
                    if feedback["running_state"] != 2
                    else MOTION_STABLE_SAMPLE_COUNT + 2
                )
            ):
                self._record_event(
                    "motion_partial_response_recovered",
                    message=(
                        "应答异常后的位移已经停止但未完整到达目标；"
                        f"目标 {target_counts}，实际 {position} counts"
                    ),
                )
                raise ForceControlError(
                    "motion_position_error",
                    f"一次性运动未完整到位：目标 {target_counts}，实际 {position} counts",
                )
            if time.monotonic() >= deadline:
                raise ForceControlError(
                    "motion_timeout",
                    f"等待到位超时：目标 {target_counts}，实际 {position} counts，"
                    f"容差±{position_tolerance_counts}，速度{feedback['measured_speed_rpm']} rpm，"
                    f"稳定样本{stable_positions}/{MOTION_STABLE_SAMPLE_COUNT}",
                )
            last_position = position
            stop_event.wait(0.08)

    def _wait_for_stable_force(
        self,
        motor,
        tracker,
        stop_event,
        max_current_ma,
        force_limit_n,
        after_sequence,
        previous_force,
        mode,
        required_samples=STABLE_SAMPLE_COUNT,
    ):
        required_samples = max(1, min(STABLE_SAMPLE_COUNT, int(required_samples)))
        deadline = time.monotonic() + PRESSURE_STABILITY_TIMEOUT_SECONDS
        recovery_generation = self._communication_recovery_generation
        accepted_values = []
        seen_sequence = int(after_sequence)
        first_snapshot = self._ocr_snapshot()
        seen_event_sequence = int(first_snapshot.get("sequence", 0))
        last_rejection_reason = "尚未收到新的压力计读数"
        wait_phase = (
            f"{mode}_ocr_initial"
            if previous_force is None
            else f"{mode}_ocr_wait"
        )
        self.operation_progress.emit(
            {
                "phase": wait_phase,
                "current_force_n": first_snapshot.get("accepted_force_n"),
                "stable_samples": 0,
                "stable_required": required_samples,
                "message": f"等待压力计稳定：已收集 0/{required_samples}",
            }
        )
        while time.monotonic() < deadline:
            if stop_event.is_set():
                raise ForceControlError("user_stop", "用户请求紧急停止")
            self._poll_checked(
                motor,
                tracker,
                stop_event,
                max_current_ma,
                force_limit_n,
                mode,
            )
            if recovery_generation != self._communication_recovery_generation:
                recovery_generation = self._communication_recovery_generation
                if not getattr(self, "_creep_active", False):
                    deadline = time.monotonic() + PRESSURE_STABILITY_TIMEOUT_SECONDS
                accepted_values.clear()
            ocr = self._ocr_snapshot()
            sequence = int(ocr["accepted_sequence"])
            event_sequence = int(ocr.get("sequence", 0))
            value = ocr["accepted_force_n"]
            if event_sequence > seen_event_sequence:
                seen_event_sequence = event_sequence
                if not ocr.get("accepted"):
                    last_rejection_reason = (
                        ocr.get("rejection_reason") or "没有有效的串口压力值"
                    )
                    self._record_event(
                        "control_ocr_rejected",
                        raw_pressure_n=(
                            ""
                            if ocr.get("raw_force_n") is None
                            else ocr.get("raw_force_n")
                        ),
                        ocr_accepted=False,
                        ocr_rejection_reason=last_rejection_reason,
                        ocr_text=ocr.get("raw_text", ""),
                        message=(
                            f"等待压力计稳定：已收集 {len(accepted_values)}/"
                            f"{required_samples}；{last_rejection_reason}"
                        ),
                    )
                    self.operation_progress.emit(
                        {
                            "phase": f"{wait_phase}_rejected",
                            "stable_samples": len(accepted_values),
                            "stable_required": required_samples,
                            "message": (
                                f"等待压力计稳定：已收集 {len(accepted_values)}/"
                                f"{required_samples}；最近拒绝："
                                f"{last_rejection_reason}"
                            )
                        }
                    )
            if sequence > seen_sequence and value is not None:
                seen_sequence = sequence
                direction_valid = True
                if previous_force is not None and mode == "press":
                    direction_valid = value >= previous_force - 20.0
                elif previous_force is not None and mode == "release":
                    direction_valid = value <= previous_force + 20.0
                if direction_valid:
                    accepted_values.append(float(value))
                    accepted_values = accepted_values[-required_samples:]
                    value_span = max(accepted_values) - min(accepted_values)
                    if (
                        len(accepted_values) >= required_samples
                        and value_span > STABLE_SAMPLE_SPAN_N
                    ):
                        last_rejection_reason = (
                            f"最近{required_samples}个读数跨度 "
                            f"{value_span:.2f} N，超过 {STABLE_SAMPLE_SPAN_N:.2f} N"
                        )
                    self.operation_progress.emit(
                        {
                            "phase": wait_phase,
                            "current_force_n": float(value),
                            "stable_samples": len(accepted_values),
                            "stable_required": required_samples,
                            "message": (
                                f"等待压力计稳定：已收集 {len(accepted_values)}/"
                                f"{required_samples}，最近 {float(value):.2f} N，"
                                f"跨度 {value_span:.2f} N"
                            )
                        }
                    )
                    if (
                        len(accepted_values) >= required_samples
                        and value_span <= STABLE_SAMPLE_SPAN_N
                    ):
                        stable_force = float(accepted_values[-1])
                        self._record_event(
                            "stable_force",
                            raw_pressure_n=stable_force,
                            filtered_force_n=stable_force,
                            message=(
                                f"获得{required_samples}帧连续稳定的原始读数，"
                                "控制采用最新一帧，不做滤波"
                            ),
                        )
                        return stable_force
                else:
                    last_rejection_reason = "与运动方向不一致"
                    self._record_event(
                        "control_ocr_rejected",
                        filtered_force_n=value,
                        ocr_accepted=False,
                        ocr_rejection_reason=last_rejection_reason,
                        ocr_text=ocr.get("raw_text", ""),
                        message=(
                            f"等待压力计稳定：已收集 {len(accepted_values)}/"
                            f"{required_samples}；{last_rejection_reason}"
                        ),
                    )
                    self.operation_progress.emit(
                        {
                            "phase": f"{wait_phase}_direction_rejected",
                            "stable_samples": len(accepted_values),
                            "stable_required": required_samples,
                            "message": (
                                f"等待压力计稳定：已收集 {len(accepted_values)}/"
                                f"{required_samples}；最近拒绝："
                                f"{last_rejection_reason}"
                            )
                        }
                    )
            stop_event.wait(0.08)
        raise ForceControlError(
            "ocr_unstable",
            f"{PRESSURE_STABILITY_TIMEOUT_SECONDS:.0f}秒内未获得连续稳定压力计读数："
            f"已收集 {len(accepted_values)}/{required_samples}；"
            f"最近拒绝：{last_rejection_reason}",
        )

    def _poll_checked(
        self,
        motor,
        tracker,
        stop_event,
        max_current_ma,
        force_limit_n,
        mode,
    ):
        if stop_event.is_set():
            raise ForceControlError("user_stop", "用户请求紧急停止")
        driver_recovery_before = int(
            getattr(motor, "communication_recovery_generation", 0)
        )
        feedback = self._call_with_motor_communication_retry(
            "读取电机反馈",
            lambda: self._read_feedback(motor, tracker),
            stop_event,
            motion_mode=mode,
        )
        driver_recovery_after = int(
            getattr(motor, "communication_recovery_generation", 0)
        )
        if driver_recovery_after != driver_recovery_before:
            self._communication_recovery_generation += 1
        current_ma = abs(int(feedback["phase_current_ma"]))
        running_state = int(feedback["running_state"])
        if current_ma > max_current_ma:
            tracker["overcurrent_samples"] = (
                int(tracker.get("overcurrent_samples", 0)) + 1
            )
        else:
            tracker["overcurrent_samples"] = 0
        immediate_limit_ma = max(
            max_current_ma + CURRENT_LIMIT_IMMEDIATE_MARGIN_MA,
            round(max_current_ma * CURRENT_LIMIT_IMMEDIATE_RATIO),
        )
        severe_overcurrent = current_ma >= immediate_limit_ma
        confirmed_overcurrent = (
            tracker["overcurrent_samples"] >= CURRENT_LIMIT_CONFIRMATION_COUNT
        )
        if severe_overcurrent or confirmed_overcurrent:
            qualifier = "严重" if severe_overcurrent else "连续"
            raise ForceControlError(
                "current_limit",
                f"{qualifier}相电流超限：{current_ma} mA，"
                f"硬件限制 {max_current_ma} mA",
                brake_required=running_state == 2,
            )
        if running_state in (3, 4, 5):
            raise ForceControlError(
                "motor_fault",
                f"电机状态异常：{feedback['running_state_name']}",
            )
        ocr = self._ocr_snapshot()
        accepted_force = ocr.get("accepted_force_n")
        accepted_age = float(ocr.get("accepted_age_seconds", math.inf))
        # 由电机线程检查原始帧失联，避免依赖界面定时器发现无效测试。
        if getattr(self, "_creep_active", False) and not getattr(self, "_creep_rebound_active", False) and mode != "release":
            sample = self._latest_sensor_sample() or {}
            raw_valid = (sample.get("has_raw") and int(sample.get("channel_count", 0)) >= 8
                         and float(sample.get("age_seconds", math.inf)) <= 0.5)
            if raw_valid:
                tracker.pop("creep_raw_missing_since", None)
            elif time.monotonic() - tracker.setdefault("creep_raw_missing_since", time.monotonic()) >= 2.0:
                raise ForceControlError("creep_raw_unavailable", "连续2秒缺少新鲜8通道原始帧，卸载并保留断点")
        if mode != "release" and (
            accepted_force is None
            or accepted_age > PRESSURE_FRESH_TIMEOUT_SECONDS
        ):
            # 空载观察保留压力缺帧；受载阶段允许最多两秒的短暂更新延迟。
            tolerate_gap = getattr(self, '_creep_rebound_active', False) and (
                mode == 'observe' or (accepted_force is not None and accepted_age <= 2.0))
            if not tolerate_gap:
                raise ForceControlError(
                    "pressure_meter_unavailable",
                    "压力计串口值无效或已经超时，已停止运动",
                    brake_required=running_state == 2,
                )
        if mode in ("release", "observe"):
            tracker["negative_pressure_samples"] = 0
            tracker["negative_pressure_sequence"] = None
        else:
            negative_pressure_samples = update_negative_pressure_confirmation(
                tracker,
                accepted_force,
                ocr.get("accepted_sequence", 0),
            )
            if negative_pressure_samples >= PRESSURE_METER_NEGATIVE_CONFIRMATION_COUNT:
                raise ForceControlError(
                    "pressure_meter_negative",
                    f"压力计连续{negative_pressure_samples}个新样本低于"
                    f"-{PRESSURE_METER_NEGATIVE_LIMIT_N:.2f} N，"
                    f"当前为 {float(accepted_force):.2f} N，已停止运动；"
                    "请检查压力计零点",
                    brake_required=running_state == 2,
                )
        raw_force = ocr.get("raw_force_n")
        raw_at = float(ocr.get("sampled_monotonic", 0.0) or 0.0)
        if (
            mode in ("press", "observe")
            and force_limit_n is not None
            and raw_force is not None
            and raw_at > 0
            and time.monotonic() - raw_at <= PRESSURE_FRESH_TIMEOUT_SECONDS
            and float(raw_force) >= force_limit_n
        ):
            raise ForceControlError(
                "force_limit",
                f"压力计原始值 {float(raw_force):.2f} N 达到保护值 {force_limit_n:.2f} N",
                brake_required=running_state == 2,
            )
        return feedback

    def _read_feedback(self, motor, tracker):
        sampled_at = time.monotonic()
        position = motor.read_position_counts()
        speed_rpm = motor.read_speed_rpm()
        current_ma = motor.read_phase_current_ma()
        running_state = motor.read_running_state()
        previous_at = tracker.get("sampled_at")
        previous_position = tracker.get("position")
        previous_speed = tracker.get("position_speed")
        interval = None if previous_at is None else sampled_at - previous_at
        position_speed = None
        acceleration = None
        if interval is not None and interval > 0 and previous_position is not None:
            position_speed = (position - previous_position) / interval
            if previous_speed is not None:
                acceleration = (
                    (position_speed - previous_speed)
                    / interval
                    / PD42S1Motor.COUNTS_PER_TURN
                )
        feedback = {
            "sampled_monotonic": sampled_at,
            "position_counts": position,
            "position_deg": position / PD42S1Motor.COUNTS_PER_TURN * 360.0,
            "measured_speed_rpm": speed_rpm,
            "position_speed_counts_s": position_speed,
            "estimated_acceleration_r_s2": acceleration,
            "sample_interval_ms": None if interval is None else interval * 1000.0,
            "phase_current_ma": current_ma,
            "running_state": running_state,
            "running_state_name": MOTOR_RUNNING_STATE_NAMES.get(
                running_state,
                f"未知状态{running_state}",
            ),
            "source": "force_press_control",
        }
        tracker["sampled_at"] = sampled_at
        tracker["position"] = position
        tracker["position_speed"] = position_speed
        self._motor_control.publish_motor_feedback(feedback)
        return feedback

    @staticmethod
    def _safe_brake(motor):
        try:
            motor.brake_stop(auto_clear=True)
        except Exception:
            try:
                motor.clear_state()
            except Exception:
                pass

    def _set_sweep_stage_indicator(self, text, state="idle"):
        colors = {
            "idle": ("#9fc3d9", "#101923", "#365b6f"),
            "warmup": ("#ffd166", "#2a210d", "#d79b24"),
            "calibrating": ("#70e6a4", "#10251a", "#2fa866"),
            "stopped": ("#ff9b93", "#2a1515", "#c94d47"),
        }
        foreground, background, border = colors.get(state, colors["idle"])
        self.sweep_stage_label.setText(str(text))
        self.sweep_stage_label.setStyleSheet(
            "QLabel#ForceSweepStageIndicator {"
            f"color:{foreground};background:{background};border:2px solid {border};"
            "border-radius:5px;padding:7px;font-weight:700;"
            "}"
        )

    def _update_sweep_stage_indicator(self, progress):
        with self._state_lock:
            mode = self._control_mode
            target_force = self._target_force_n
            warmup_or_sample_index = self._auto_round_index
            warmup_or_sample_total = self._auto_round_total
            cycle_index = self._calibration_cycle_index
            cycle_total = self._calibration_cycle_total
            sweep_direction = self._sweep_direction
            direction_step_index = self._direction_step_index
            direction_step_total = self._direction_step_total
        if mode != "sweep":
            return
        phase = str(progress.get("phase", ""))
        if phase == "release_sensor_idle_wait":
            self._set_sweep_stage_indicator(
                "当前阶段：已回零，等待传感器恢复｜"
                f"IDLE/WAIT/RECOVER/SLOW_BACK {int(progress.get('idle_count', 0))}/4｜"
                f"连续稳定 {float(progress.get('stable_seconds', 0.0)):.1f}/"
                f"{float(progress.get('stable_required_seconds', 0.0)):.1f}秒｜"
                f"最长等待剩余 {float(progress.get('idle_wait_remaining_seconds', 0.0)):.1f}秒",
                "warmup",
            )
            return
        if phase == "release_sensor_idle_timeout_continue":
            self._set_sweep_stage_indicator(
                "当前阶段：传感器恢复等待超时；已写入警告并继续下一轮",
                "warmup",
            )
            return
        if sweep_direction == "warmup":
            warmup_force = (
                float(target_force)
                if target_force is not None
                else float(self.sweep_max_force_input.value())
            )
            self._set_sweep_stage_indicator(
                "当前阶段：标定前热身（不记录数据）｜"
                f"{warmup_or_sample_index}/{warmup_or_sample_total}次｜"
                f"目标 {warmup_force:.2f} N｜完成热身后才开始采集标定数据",
                "warmup",
            )
            return
        if cycle_index and cycle_total and sweep_direction:
            direction_text = {
                "ascending": "由小到大",
                "descending": "由大到小",
            }.get(sweep_direction, sweep_direction)
            target_text = (
                "--.--"
                if target_force is None
                else f"{float(target_force):.2f}"
            )
            self._set_sweep_stage_indicator(
                "当前阶段：正式传感器标定（正在记录数据）｜"
                f"往返轮 {cycle_index}/{cycle_total}｜{direction_text} "
                f"{direction_step_index}/{direction_step_total}｜"
                f"目标 {target_text} N｜有效采样 "
                f"{warmup_or_sample_index}/{warmup_or_sample_total}",
                "calibrating",
            )
            return
        self._set_sweep_stage_indicator(
            "当前阶段：准备标定前热身｜热身数据不写入标定结果或K值表",
            "warmup",
        )

    @Slot(object)
    def _on_operation_progress(self, progress):
        if "command_counts" in progress or "total_travel_counts" in progress:
            command_counts = int(progress.get("command_counts", 0))
            total_counts = int(progress.get("total_travel_counts", 0))
            self.command_value_label.setText(
                f"本步 {command_counts:+d} counts · 累计 {total_counts} counts"
            )
        self._update_sweep_stage_indicator(progress)
        progress_stage, _action_text = self._progress_stage(progress)
        stage_changed = progress_stage != self._last_progress_stage
        self._last_progress_stage = progress_stage
        detail = self._progress_summary(progress)
        raw_message = str(progress.get("message", "")).strip()
        if raw_message and raw_message not in detail:
            detail = (
                f"{detail}\n过程信息：{raw_message}"
                if detail
                else f"\n过程信息：{raw_message}"
            )
        self._set_status(
            detail or "按力控制运行中",
            summary=self._progress_headline(progress),
            force_summary=stage_changed,
        )

    def _progress_headline(self, progress):
        progress_stage, _action_text = self._progress_stage(progress)
        creep_phase = str(progress.get("phase", ""))
        if creep_phase == "creep_high_hold":
            return "高压保压"
        if creep_phase == "creep_recovery_wait":
            return "空载恢复"
        if creep_phase == "creep_probe_hold":
            return "探针保压"
        release_stages = {
            "release_moving",
            "release_prepare",
            "release_ocr_initial",
            "release_ocr_wait",
            "release_zero_confirm",
            "release_sensor_idle_wait",
            "release_sensor_idle_ready",
            "release_sensor_idle_timeout_continue",
            "release_communication_retry",
            "release_communication_recovered",
        }
        if progress_stage in release_stages:
            return "抬起"
        with self._state_lock:
            return "抬起" if self._control_mode == "release" else "下压"

    @staticmethod
    def _progress_stage(progress):
        phase = str(progress.get("phase", ""))
        if phase in ("creep_high_press", "creep_probe_press"):
            return "press_prepare", "长时测试准备下压"
        if phase in ("creep_high_release", "creep_probe_release", "creep_initial_release"):
            return "release_prepare", "长时测试准备释放"
        if phase == "creep_high_hold":
            return "creep_high_hold", "长时测试高压保持"
        if phase == "creep_recovery_wait":
            return "creep_recovery_wait", "长时测试空载恢复"
        if phase == "creep_probe_hold":
            return "creep_probe_hold", "长时测试探针保持"
        if phase == "creep_inter_trial_wait":
            return "creep_inter_trial_wait", "长时测试条件间隔"
        if phase == "press_moving":
            return "press_moving", "正在下压"
        if phase == "press_lookup_prepare":
            return "press_prepare", "准备查表一次性下压"
        if phase == "press_position_correction":
            return "press_moving", "正在执行一次纯位置修正"
        if phase == "warmup_round_start":
            return "press_prepare", f"准备{WARMUP_FORCE_N:.2f} N热身下压"
        if phase == "warmup_release":
            return "release_prepare", "热身下压完成，准备释放"
        if phase == "press_fast_approach":
            return "press_moving", "接触前下压（压力计仍为0 N）"
        if phase == "press_contact_detected":
            return "press_prepare", "压力计出现正值，切换接触后快速下压"
        if phase == "release_moving":
            return "release_moving", "正在抬起"
        if phase == "press_communication_retry":
            return "press_communication_retry", "通信重试，暂停下压"
        if phase == "release_communication_retry":
            return "release_communication_retry", "通信重试，暂停抬起"
        if phase == "press_communication_recovered":
            return "press_communication_recovered", "通信已恢复，继续下压流程"
        if phase == "release_communication_recovered":
            return "release_communication_recovered", "通信已恢复，继续抬起流程"
        if phase == "communication_retry":
            return "communication_retry", "电机通信重试"
        if phase == "communication_recovered":
            return "communication_recovered", "电机通信已恢复"
        if phase.startswith("press_ocr_initial"):
            return "press_ocr_initial", "读取压力计，准备下压"
        if phase.startswith("release_ocr_initial"):
            return "release_ocr_initial", "读取压力计，准备抬起"
        if phase.startswith("press_ocr_"):
            return "press_ocr_wait", "等待压力计（下压后）"
        if phase.startswith("release_ocr_"):
            return "release_ocr_wait", "等待压力计（抬起后）"
        if phase == "release_zero_confirm":
            return (
                "release_zero_confirm",
                f"电机已停止，确认压力计处于 ±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N",
            )
        if phase == "release_sensor_idle_wait":
            return "release_sensor_idle_wait", "回零后等待四通道恢复稳定"
        if phase == "release_sensor_idle_ready":
            return "release_sensor_idle_ready", "四通道状态与基线已稳定"
        if phase == "release_sensor_idle_timeout_continue":
            return "release_sensor_idle_timeout_continue", "恢复等待超时，记录后继续"
        if phase == "auto_floating_wait":
            return "floating_wait", "保压等待FLOATING稳定"
        if phase in ("sweep_warmup_started", "sweep_warmup_round_start"):
            return "calibration_warmup", "标定前热身（不记录数据）"
        if phase == "sweep_warmup_release":
            return "calibration_warmup", "标定前热身释放（不记录数据）"
        if phase == "sweep_warmup_completed":
            return "calibration_warmup", "热身完成，准备正式标定"
        if phase in ("auto_release", "auto_safe_release"):
            return "release_prepare", "准备返回固定释放位置"
        if phase in ("sweep_release", "sweep_safe_release"):
            return "release_prepare", "多压力任务正在返回标定出发点"
        if phase == "sweep_timeout_recovery":
            return "release_prepare", "到位超时，返回出发点后重试"
        if phase == "sweep_start_zero_wait":
            return "sweep_start_zero_wait", "出发压力未回零，等待后重试"
        if phase == "sweep_round_start":
            return "press_prepare", "准备多压力查表下压"
        if phase == "auto_round_start":
            return "press_prepare", "准备下压"
        if phase == 'auto_interval_zero_wait':
            return 'auto_interval_zero_wait', '已释放，等待压力接近零'
        if phase == 'auto_interval_wait':
            return 'auto_interval_wait', f"释放后间隔，剩余{float(progress.get('remaining_seconds', 0)):.1f}秒"
        if phase == "press":
            return "press_prepare", "准备继续下压"
        if phase == "release":
            return "release_prepare", "准备返回固定释放位置"
        return "processing", "处理中"

    def _progress_summary(self, progress):
        current_force = progress.get("current_force_n")
        if current_force is None:
            current_force = self._ocr_snapshot().get("accepted_force_n")
        if current_force is not None:
            current_force = float(current_force)
        with self._state_lock:
            mode = self._control_mode
            target_force = self._target_force_n
            auto_round_index = self._auto_round_index
            auto_round_total = self._auto_round_total
            pressure_level_index = self._pressure_level_index
            pressure_level_total = self._pressure_level_total
            calibration_cycle_index = self._calibration_cycle_index
            calibration_cycle_total = self._calibration_cycle_total
            sweep_direction = self._sweep_direction
            direction_step_index = self._direction_step_index
            direction_step_total = self._direction_step_total
            repeat_index = self._repeat_index
            repeat_total = self._repeat_total
            attempt_index = self._attempt_index
        progress_stage, action_text = self._progress_stage(progress)
        parts = []
        if (
            mode == "sweep"
            and sweep_direction == "warmup"
            and auto_round_total
        ):
            warmup_index = auto_round_index or 0
            parts.append(
                f"标定前热身 {warmup_index}/{auto_round_total}（不记录数据）"
            )
        elif (
            mode in ("sweep", "validation", "quick_correction")
            and pressure_level_index
            and pressure_level_total
            and calibration_cycle_index
            and calibration_cycle_total
            and sweep_direction
        ):
            direction_text = {
                "ascending": "升序",
                "descending": "降序",
            }.get(sweep_direction, sweep_direction)
            parts.append(
                f"往返轮 {calibration_cycle_index}/{calibration_cycle_total}"
                f" · {direction_text} {direction_step_index}/{direction_step_total}"
                f" · 档位 {pressure_level_index}/{pressure_level_total}"
                f" · 有效采样 {auto_round_index}/{auto_round_total}"
                f" · 尝试 {attempt_index}"
            )
        elif mode in ("auto", "warmup") and auto_round_index and auto_round_total:
            parts.append(f"第 {auto_round_index}/{auto_round_total} 轮")
        parts.append(f"当前动作：{action_text}")
        if current_force is not None:
            parts.append(f"当前压力 {current_force:.2f} N")
        if progress_stage in (
            "press_ocr_initial",
            "press_ocr_wait",
            "release_ocr_initial",
            "release_ocr_wait",
        ):
            stable_samples = progress.get("stable_samples")
            stable_required = progress.get("stable_required")
            if stable_samples is not None and stable_required is not None:
                parts.append(f"压力 {int(stable_samples)}/{int(stable_required)}帧")
        if progress_stage == "floating_wait":
            floating_count = progress.get("floating_count")
            floating_required = progress.get("floating_required")
            stable_seconds = progress.get("stable_seconds")
            stable_required_seconds = progress.get("stable_required_seconds")
            if floating_count is not None and floating_required is not None:
                parts.append(
                    f"FLOATING {int(floating_count)}/{int(floating_required)}"
                )
            if stable_seconds is not None and stable_required_seconds is not None:
                parts.append(
                    f"稳定 {float(stable_seconds):.1f}/"
                    f"{float(stable_required_seconds):.1f} s"
                )
            raw_sample_count = progress.get("raw_sample_count")
            if raw_sample_count is not None:
                parts.append(f"Raw {int(raw_sample_count)}帧")
        if progress_stage == "release_zero_confirm":
            stable_seconds = progress.get("stable_seconds", 0.0)
            stable_required_seconds = progress.get(
                "stable_required_seconds",
                RELEASE_ZERO_CONFIRM_SECONDS,
            )
            parts.append(
                f"释放零点 ±{PRESSURE_METER_START_ZERO_TOLERANCE_N:.2f} N "
                f"{float(stable_seconds):.1f}/"
                f"{float(stable_required_seconds):.1f} s"
            )
        if progress_stage in (
            "release_sensor_idle_wait",
            "release_sensor_idle_ready",
            "release_sensor_idle_timeout_continue",
        ):
            idle_count = progress.get("idle_count", 0)
            idle_required = progress.get("idle_required", len(SENSOR_CHANNELS))
            stable_seconds = progress.get("stable_seconds", 0.0)
            stable_required_seconds = progress.get(
                "stable_required_seconds",
                0.0,
            )
            parts.append(
                f"IDLE/WAIT/RECOVER/SLOW_BACK {int(idle_count)}/{int(idle_required)}"
            )
            parts.append(
                f"稳定 {float(stable_seconds):.1f}/"
                f"{float(stable_required_seconds):.1f} s"
            )
            remaining = progress.get("idle_wait_remaining_seconds")
            if remaining is not None:
                parts.append(f"最长等待剩余 {float(remaining):.1f} s")
            raw_span = progress.get("raw_span_counts")
            if raw_span is not None:
                parts.append(
                    f"Raw极差 {float(raw_span):.1f}/"
                    f"{SENSOR_RELEASE_RAW_SPAN_COUNTS:.1f} counts"
                )
        communication_retry_count = progress.get("communication_retry_count")
        if communication_retry_count is not None:
            parts.append(f"第{int(communication_retry_count)}次")
        retry_wait_remaining = progress.get("retry_wait_remaining_seconds")
        if retry_wait_remaining is not None:
            parts.append(f"等待剩余 {float(retry_wait_remaining):.1f} s")

        motion_details = []
        if progress_stage in ("press_moving", "release_moving"):
            speed_level = progress.get("speed_level")
            command_counts = progress.get("command_counts")
            if speed_level is not None:
                speed_name = str(progress.get("speed_name", "")).strip()
                motion_details.append(f"{speed_name or '速度'} {int(speed_level)}档")
            if command_counts is not None:
                motion_details.append(f"本步 {int(command_counts):+d} counts")

        release_stage = progress_stage in (
            "release_moving",
            "release_prepare",
            "release_ocr_wait",
            "release_zero_confirm",
            "release_sensor_idle_wait",
            "release_sensor_idle_ready",
            "release_sensor_idle_timeout_continue",
            "release_communication_retry",
            "release_communication_recovered",
        )
        if mode == "release" or release_stage:
            target_position = progress.get("target_position_counts")
            if target_position is not None:
                parts.append(f"固定释放位置 {int(target_position)} counts")
            parts.extend(motion_details)
            return "｜".join(parts)

        press_stage = progress_stage in (
            "press_moving",
            "press_prepare",
            "press_ocr_wait",
            "press_communication_retry",
            "press_communication_recovered",
        )
        if (
            (
                mode in (
                    "press",
                    "auto",
                    "sweep",
                    "validation",
                    "quick_correction",
                )
                or press_stage
            )
            and target_force is not None
            and current_force is not None
        ):
            target_force = float(target_force)
            remaining = target_force - current_force
            target_tolerance_n = press_position_correction_force_tolerance_n(
                target_force
            )
            if remaining > target_tolerance_n:
                parts.append(f"距离目标还差 {remaining:.2f} N")
            elif remaining >= -target_tolerance_n:
                parts.append(f"已进入 {target_force:.2f} N 目标范围")
            else:
                parts.append(f"超过目标 {abs(remaining):.2f} N")
        elif press_stage and target_force is not None:
            parts.append(f"目标 {float(target_force):.2f} N")
        parts.extend(motion_details)
        return "｜".join(parts)

    @Slot(object)
    def _on_operation_finished(self, result):
        self._running = False
        finished_mode = str(result.get("mode", ""))
        if (
            result.get("ok")
            and result.get("mode") == "press"
            and result.get("target_force_n") is not None
            and result.get("final_force_n") is not None
        ):
            self._append_force_mapping_log(result)
        with self._state_lock:
            self._control_mode = "idle"
            self._target_force_n = None
            self._auto_round_index = 0
            self._auto_round_total = 0
            self._pressure_level_index = 0
            self._pressure_level_total = 0
            self._calibration_cycle_index = 0
            self._calibration_cycle_total = 0
            self._sweep_direction = ""
            self._direction_step_index = 0
            self._direction_step_total = 0
            self._repeat_index = 0
            self._repeat_total = 0
            self._attempt_index = 0
            self._command_counts = 0
            self._last_progress_stage = None
        if result.get("ok"):
            message = result.get("message", "按力控制完成")
            if finished_mode == "quick_correction":
                quick_entries = tuple(
                    dict(entry)
                    for entry in result.get("quick_correction_results", ())
                )
                quick_path = self._save_force_entries(
                    quick_entries,
                    "force_sensor_quick_correction_results",
                    "单轮快速K修正记录",
                )
                quick_payload = {
                    "entries": quick_entries,
                    "source_path": "" if quick_path is None else str(quick_path),
                }
                self.set_quick_correction_ram_result(
                    {
                        "state": "waiting",
                        "message": (
                            "十个压力点采集完成；等待电机线程释放后，"
                            "将自动发送RAM快调表"
                        ),
                    }
                )
                self._emit_quick_correction_after_motor_idle(quick_payload)
            statistics_text = self._format_signal_sum_statistics(
                result.get("auto_signal_statistics")
            )
            raw_statistics_text = self._format_raw_median_statistics(
                result.get("auto_raw_median_statistics")
            )
            result_text = f"上次结果：成功 · {message}"
            if statistics_text:
                result_text = f"{result_text}\n{statistics_text}"
            if raw_statistics_text:
                result_text = f"{result_text}\n{raw_statistics_text}"
            self.last_result_label.setText(result_text)
            self._set_status("空闲，等待下一次操作")
            if finished_mode == "sweep":
                self._set_sweep_stage_indicator(
                    "传感器标定阶段：已完成；热身数据未记录，"
                    "正式升降往返数据已保存到当前记录",
                    "calibrating",
                )
        else:
            reason = result.get("message") or result.get("stop_reason", "未知原因")
            if finished_mode == "quick_correction":
                self.set_quick_correction_ram_result(
                    {
                        "state": "failed",
                        "message": f"快速K修正采集未完成，因此没有写入MCU RAM：{reason}",
                    }
                )
            self.last_result_label.setText(f"上次结果：停止 · {reason}")
            self._set_status(f"已停止：{reason}")
            if finished_mode == "sweep":
                self._set_sweep_stage_indicator(
                    f"传感器标定阶段：已停止｜{reason}",
                    "stopped",
                )
        self.status_label.hide()
        self._refresh_controls()

    @Slot()
    def request_stop(self):
        if not self._motor_control.is_can_connected():
            self._set_status("CAN未连接")
            return
        self._motor_control.request_stop()
        self._set_status("已请求紧急停止，等待当前CAN指令结束后刹车")

    @Slot(bool, bool)
    def _on_motor_state_changed(self, _connected, _busy):
        self._refresh_controls()

    def _set_status(self, message, summary=None, force_summary=True):
        message = str(message)
        self._last_status_message = message
        self.status_detail_label.setText(f"详细状态：{message}")
        now = time.monotonic()
        if summary is not None and (
            force_summary
            or now - self._last_summary_update_at
            >= CONTROL_SUMMARY_REFRESH_SECONDS
        ):
            self.status_label.setText(str(summary))
            self.status_label.show()
            self._last_summary_update_at = now
        self.status_message.emit(str(message))

    def _refresh_controls(self):
        connected = self._motor_control.is_can_connected()
        motor_busy = self._motor_control.is_operation_busy()
        latest_feedback = self._motor_control.latest_motor_feedback() or {}
        try:
            driver_reports_running = (
                connected and int(latest_feedback.get("running_state")) == 2
            )
        except (TypeError, ValueError):
            driver_reports_running = False
        base_available = connected and not motor_busy and not self._running
        driver_holding = (
            driver_reports_running
            and base_available
            and driver_feedback_is_settled_holding(latest_feedback)
        )
        driver_actively_moving = driver_reports_running and not driver_holding
        target_available = base_available and not driver_reports_running
        release_available = base_available and (
            not driver_reports_running or driver_holding
        )
        release_confirmed = self._confirmed_release_position() is not None
        self.set_release_position_button.setEnabled(target_available)
        for target_force, button in zip(TARGET_FORCES_N, self._target_buttons):
            button.setEnabled(
                target_available
                and release_confirmed
                and target_force in self._lookup_targets
            )
        self.release_button.setText(
            RELEASE_BUTTON_RUNNING_TEXT
            if driver_actively_moving
            else RELEASE_BUTTON_IDLE_TEXT
        )
        self.release_button.setEnabled(
            release_available
            and release_confirmed
            and self._lookup_document is not None
        )
        self.warmup_button.setEnabled(
            target_available
            and release_confirmed
            and WARMUP_FORCE_N in self._lookup_targets
        )
        auto_target = self.auto_test_target_input.currentData()
        auto_target_available = (
            auto_target is not None
            and round(float(auto_target), 2) in self._lookup_targets
        )
        self.start_auto_test_button.setEnabled(
            target_available
            and release_confirmed
            and self._lookup_document is not None
            and auto_target_available
        )
        sweep_plan, _sweep_error = self._sweep_test_plan()
        sweep_targets_available = False
        sweep_warmup_available = False
        if sweep_plan is not None and self._lookup_document is not None:
            try:
                for sweep_force in sweep_plan["targets_n"]:
                    self._resolve_lookup_target(
                        self._lookup_document,
                        sweep_force,
                    )
            except ForcePositionLookupError:
                pass
            else:
                sweep_targets_available = True
            try:
                self._resolve_lookup_target(
                    self._lookup_document,
                    sweep_plan["warmup_force_n"],
                )
            except ForcePositionLookupError:
                pass
            else:
                sweep_warmup_available = True
        self.start_sweep_test_button.setEnabled(
            target_available
            and release_confirmed
            and sweep_targets_available
            and sweep_warmup_available
        )
        self.start_validation_test_button.setEnabled(
            target_available
            and release_confirmed
            and sweep_targets_available
        )
        self.start_quick_correction_button.setEnabled(
            target_available
            and release_confirmed
            and not self._quick_correction_ram_trial_active
            and self._lookup_document is not None
            and all(
                target_force in self._lookup_targets
                for target_force in QUICK_CORRECTION_TARGETS_N
            )
        )
        self.resume_sweep_test_button.setEnabled(
            target_available
            and release_confirmed
            and self._lookup_document is not None
            and self._sweep_checkpoint_path().is_file()
        )
        self.stop_button.setEnabled(
            connected and (motor_busy or self._running or driver_reports_running)
        )
        for widget in (
            self.press_speed_input,
            self.release_speed_input,
            self.acceleration_input,
            self.max_current_input,
            self.auto_test_target_input,
            self.auto_test_rounds_input,
            self.auto_test_interval_input,
            self.sweep_max_force_input,
            self.sweep_force_interval_input,
            self.sweep_repeats_input,
            self.sweep_idle_stable_input,
            self.sweep_idle_timeout_input,
        ):
            widget.setEnabled(not self._running and not motor_busy)

    def shutdown(self):
        self._closing = True
        self._sensor_record_timer.stop()
        if self._running:
            self._motor_control.request_external_stop()
        self._save_settings_if_idle()
        try:
            self._record_event("recording_stopped", message="应用关闭，实时运动日志结束")
            self._recorder.stop()
        except OSError:
            pass
