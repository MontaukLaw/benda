"""压力标定使用的电机接口；单位换算和型号差异只在此模块处理。

历史算法的 *_counts 字段保留兼容；接口传入这些字段的值已统一为 step，
新表以 position_unit、motor_model、steps_per_turn 明确标注，且按驱动隔离。
"""

from dataclasses import dataclass, field
import logging
import math
import time
from typing import Protocol

from qt_compat import QtCore, Signal
from motor_speed_units import valid_rpm


class MappingMotor(Protocol):
    def read_position_steps(self) -> int: ...
    def read_feedback(self) -> dict: ...
    def move_relative_steps(self, steps, *, speed_rpm, accel, retry_cancelled): ...
    def set_current_limit_ma(self, current): ...
    def prepare_motion(self): ...
    def stop(self): ...


@dataclass(frozen=True)
class MappingProfile:
    model: str
    steps_per_turn: int
    acceleration_unit: str
    acceleration_max: int
    speed_unit: str
    speed_max: int
    namespace: str = ""
    default_settings: dict = field(default_factory=dict)

    def tolerance(self, legacy_counts):
        return max(1, round(legacy_counts * self.steps_per_turn / 51200))

    def speed_rpm(self, value):
        if self.speed_unit == "RPM":
            if not valid_rpm(value, maximum=self.speed_max):
                raise ValueError('速度须为有效的0.1 RPM整数倍')
            return value
        return max(1, round(value * 4096 * 60 / self.steps_per_turn))

    def metadata(self):
        return dict(motor_model=self.model, position_unit="step", steps_per_turn=self.steps_per_turn)

    def motion_timeout(self, steps, rpm, acceleration):
        travel_seconds = abs(steps) / max(1, rpm * self.steps_per_turn / 60)
        ramp_seconds = 0
        if acceleration:
            ramp_seconds = (rpm * (256 - acceleration) * 0.00005
                            if self.acceleration_unit == "档" else rpm / 60 / acceleration)
        return max(3.0, travel_seconds * 4 + ramp_seconds * 2 + 2)

    def storage_fields(self, value, *, read=False):
        """新驱动JSON/CSV使用step和真实速度单位；旧文件结构保持兼容。"""
        if not self.namespace:
            return value
        names = {"press_speed_level": "press_speed_rpm", "release_speed_level": "release_speed_rpm",
                 "acceleration_r_s2": "acceleration_level", "press_step_counts": "press_step",
                 "commanded_step_counts": "commanded_steps", "actual_step_counts": "actual_steps"}
        if read:
            names = {v: k for k, v in names.items()}
        def key_name(key):
            if key in names:
                return names[key]
            source, target = ("_steps", "_counts") if read else ("_counts", "_steps")
            return key[:-len(source)] + target if key.endswith(source) else key
        if isinstance(value, dict):
            return {key_name(k): self.storage_fields(v, read=read) for k, v in value.items()}
        if isinstance(value, list):
            return [self.storage_fields(v, read=read) for v in value]
        return value


LEGACY_PROFILE = MappingProfile("pd42s1", 51200, "r/s²", 200, "档", 20)


class PD42S1MappingMotor:
    def __init__(self, motor):
        self.motor = motor

    def read_position_steps(self):
        return self.motor.read_position_counts()

    def read_feedback(self):
        return dict(position_counts=self.read_position_steps(),
                    measured_speed_rpm=self.motor.read_speed_rpm(),
                    phase_current_ma=self.motor.read_phase_current_ma(),
                    running_state=self.motor.read_running_state(), sampled_monotonic=time.monotonic())

    def move_relative_steps(self, steps, **kwargs):
        return self.motor.move_relative_counts(steps, **kwargs)

    def set_current_limit_ma(self, current):
        self.motor.set_max_torque_current_ma(current)

    def prepare_motion(self):
        self.motor.clear_state()

    def stop(self):
        self.motor.brake_stop(auto_clear=True)


class LegacyMappingControl:
    profile = LEGACY_PROFILE

    def __init__(self, control):
        self.control = control

    def __getattr__(self, name):
        return getattr(self.control, name)

    def start_external_operation(self, operation, initial_status):
        return self.control.start_external_operation(
            lambda motor, cancel: operation(PD42S1MappingMotor(motor), cancel), initial_status)


class ZDTMappingMotor:
    def __init__(self, motor, control):
        self.motor, self.control = motor, control

    def read_feedback(self):
        sample = self.motor.read_status()
        self.control.backend.status_received.emit(sample)
        if sample["firmware"] != "Emm":
            raise RuntimeError("当前位置扫描接口支持 ZDT Emm 固件，请核对电机固件")
        if not sample["enabled"] or sample["homing"] or not sample["encoder_ready"] or not sample["calibration_ready"]:
            raise RuntimeError("电机未使能、正在回零或编码器未就绪")
        if any(sample[key] for key in ("stall", "stall_protection", "overtemperature", "overcurrent")):
            raise RuntimeError("电机有堵转、过流或过温保护，已停止扫描")
        return self.control.convert_feedback(sample)

    def read_position_steps(self):
        return self.read_feedback()["position_counts"]

    def prepare_motion(self):
        sample = self.read_feedback()
        if abs(sample["measured_speed_rpm"]) > 0:
            raise RuntimeError("电机仍在运动，请等待停止")

    def move_relative_steps(self, steps, *, speed_rpm, accel, retry_cancelled):
        if retry_cancelled():
            raise RuntimeError("用户停止操作")
        self.motor.move_relative_pulses(pulses=abs(int(steps)), speed_rpm=speed_rpm,
                                        acceleration=accel, ccw=steps < 0)

    def set_current_limit_ma(self, current):
        self.motor.set_current_limit_ma(int(current))

    def move_absolute_steps(self, position, *, speed_rpm, accel, retry_cancelled):
        """同一CAN工作线程内更新绝对轨迹；调用者逐帧核对坐标、保护和反馈，不等待静止。"""
        if retry_cancelled():
            raise RuntimeError("用户停止操作")
        self.motor.move_absolute_pulses(position, speed_rpm, accel)

    def stop(self):
        self.motor.stop_motion()


class ZDTMappingControl(QtCore.QObject):
    motor_feedback = Signal(object)
    control_state_changed = Signal(bool, bool)
    worker_position_zeroed = Signal()
    release_position_changed = Signal(object)
    status_message = Signal(str)

    def __init__(self, backend, parent=None):
        super().__init__(parent)
        self.backend = backend
        steps = backend.config.steps_per_turn
        self.profile = MappingProfile("zdt_x42s", steps, "档", 255, "RPM", 3000,
                                      f"zdt_x42s_{backend.config.node_id}_{steps}",
                                      dict(press_step_counts=max(1, round(steps / 100)),
                                           position_span_limit_counts=max(1, round(steps / 400)),
                                           max_travel_counts=max(1, round(steps * 1.92)),
                                           press_speed_level=10, release_speed_level=24,
                                           max_current_ma=3000, acceleration_r_s2=100))
        self._release = None
        self._feedback = None
        backend.status_received.connect(self._on_status)
        backend.link_changed.connect(self._on_link)
        backend.communication_failed.connect(self._on_link_error)
        backend.control_state_changed.connect(self.control_state_changed.emit)
        backend.command_finished.connect(self._on_command_finished)
        backend.coordinate_invalidated.connect(self._invalidate_coordinates)

    def convert_feedback(self, sample):
        steps = round(sample["position_deg"] * self.profile.steps_per_turn / 360)
        return dict(position_counts=steps, position_deg=sample["position_deg"],
                    measured_speed_rpm=sample["speed_rpm"], phase_current_ma=sample["phase_current_ma"],
                    running_state=2 if abs(sample["speed_rpm"]) > 0 else 0,
                    in_position=sample["in_position"], state_flags=sample["state_flags"],
                    sampled_monotonic=sample["sampled_at"])

    def _on_status(self, sample):
        self._feedback = self.convert_feedback(sample)
        self.motor_feedback.emit(self._feedback)

    def _on_link(self, connected):
        self._feedback = None
        self._release = None
        self.release_position_changed.emit(self.release_position_snapshot())
        self.control_state_changed.emit(connected, self.is_operation_busy())

    def _on_link_error(self, _message):
        self._feedback = None
        self.control_state_changed.emit(False, self.is_operation_busy())

    def _on_command_finished(self, _operation, message):
        if _operation != "external":
            self.status_message.emit(message)

    def _invalidate_coordinates(self):
        # 命令超时也可能已被执行，发送前就撤销旧坐标的使用资格。
        self._release = None
        self._feedback = None
        self.release_position_changed.emit(self.release_position_snapshot())
        self.worker_position_zeroed.emit()

    def is_can_connected(self):
        return self.backend._connected and self.backend._is_fresh()

    @property
    def feedback_max_age_s(self):
        return self.backend.feedback_max_age_s

    def is_operation_busy(self):
        return self.backend._busy or self.backend._home_unresolved()

    def latest_motor_feedback(self):
        return self._feedback if self.is_can_connected() else None

    def publish_motor_feedback(self, feedback):
        self.motor_feedback.emit(feedback)

    def record_motion_diagnostic(self, event, **fields):
        logging.getLogger(__name__).info("%s %s", event, fields)

    def start_external_operation(self, operation, initial_status):
        def task(motor, cancel):
            operation(ZDTMappingMotor(motor, self), cancel)
        return self.backend.start_external_operation(task, initial_status)

    def request_external_stop(self):
        self.backend._submit("stop")

    def release_position_snapshot(self):
        return dict(position_counts=self._release, confirmed=self._release is not None)

    def restore_release_position(self, position):
        """压力页已连续确认卸载后恢复原坐标；只核验反馈，不移动或重置电机。"""
        if (isinstance(position, bool) or not isinstance(position, int)
                or not self.is_can_connected() or self.is_operation_busy()
                or self._release not in (None, position)):
            return False
        sample = self.backend._last_sample
        feedback = self.latest_motor_feedback() or {}
        try:
            if self.backend._motion_block_reason(sample, self.feedback_max_age_s):
                return False
            age = time.monotonic()-float(feedback['sampled_monotonic'])
            actual = float(feedback['position_counts'])
            speed = float(feedback['measured_speed_rpm'])
            if (not all(math.isfinite(v) for v in (age, actual, speed))
                    or not 0 <= age <= self.feedback_max_age_s or abs(speed) > 1
                    or not feedback.get('in_position')
                    or abs(actual-position) > self.profile.tolerance(256)):
                return False
        except (KeyError, ValueError, TypeError, OverflowError):
            return False
        self._release = position
        self.release_position_changed.emit(self.release_position_snapshot())
        return True

    def capture_release_position(self):
        reason = self.backend._motion_block_reason(self.backend._last_sample, self.feedback_max_age_s)
        if reason or self.is_operation_busy():
            self.status_message.emit(reason or "电机操作正在进行")
            return
        self._release = self.convert_feedback(self.backend._last_sample)["position_counts"]
        self.release_position_changed.emit(self.release_position_snapshot())

    def zero_motor_position(self):
        self.backend._submit("zero")


def create_mapping_control(model, legacy_control, zdt_control=None):
    """由已校验的 JSON 型号选择适配器，不创建第二条 CAN 连接。"""
    factories = {
        "pd42s1": lambda: LegacyMappingControl(legacy_control),
        "zdt_x42s": lambda: ZDTMappingControl(zdt_control, zdt_control),
    }
    if model not in factories:
        raise ValueError("未配置有效的电机标定驱动")
    return factories[model]()
