"""PD42S1 stepper motor CAN driver used by the integrated monitor UI."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

try:
    import can
except ModuleNotFoundError:
    can = None


class PD42S1Error(Exception):
    """PD42S1 base exception."""


class PD42S1CommunicationError(PD42S1Error):
    """Raised for a recoverable CAN/protocol communication failure."""


class PD42S1TimeoutError(PD42S1CommunicationError):
    """Raised when no complete response is received before the timeout."""


class PD42S1ProtocolError(PD42S1CommunicationError):
    """Raised when a private-protocol packet is malformed."""


class PD42S1MoveCommandUnconfirmedError(PD42S1CommunicationError):
    """Raised only after confirming that a rejected move produced no motion."""


DEVICE_RESULT_CODE_NAMES = {
    0xE1: "帧长度不足",
    0xE2: "帧头错误",
    0xE3: "下行命令帧尾错误",
    0xE4: "校验和错误",
    0xE5: "不支持的功能码",
    0xE6: "命令数据不合法",
}


class PD42S1DeviceError(PD42S1Error):
    """Raised when the motor controller explicitly rejects a command."""

    def __init__(self, function_code: int, result_code: int, response: bytes):
        self.function_code = int(function_code)
        self.result_code = int(result_code)
        self.response = bytes(response)
        description = DEVICE_RESULT_CODE_NAMES.get(self.result_code, "未知设备错误")
        super().__init__(
            f"电机返回错误 0x{self.result_code:02X}（{description}）："
            f"{self.response.hex(' ')}"
        )


@dataclass(frozen=True)
class PD42S1Config:
    interface: str = "pcan"
    channel: str = "PCAN_USBBUS1"
    bitrate: int = 500000
    can_id_tx: int = 0x1001
    can_id_rx: Optional[int] = 0x00000001
    node_id: int = 0x01
    is_extended_id: bool = True
    inter_frame_delay: float = 0.0
    command_interval: float = 0.002
    rx_poll_interval: float = 0.01
    default_timeout: float = 0.8
    debug: bool = False


class PD42S1Motor:
    """PD42S1 private protocol transported over extended CAN frames."""

    FRAME_HEAD = 0xC5
    FRAME_TAIL = 0x5C
    COUNTS_PER_TURN = 51200

    CMD_READ_PHASE_CURRENT = 0x23
    CMD_READ_SPEED = 0x29
    CMD_READ_POSITION = 0x2A
    CMD_READ_RUNNING_STATE = 0x2C
    CMD_READ_IN_POSITION = 0x30
    CMD_SET_MAX_TORQUE_CURRENT = 0x64
    CMD_ZERO_POSITION = 0xF8
    CMD_ENABLE = 0xFA
    CMD_CLEAR_STATE = 0xFB
    CMD_BRAKE_STOP = 0xFC
    CMD_MOVE_REL = 0xF3
    MAX_SPEED_RPM = 6000

    # 只读和幂等控制命令允许承受偶发丢帧或E1～E4通信帧错误。
    READ_QUERY_MAX_ATTEMPTS = 5
    READ_QUERY_RETRY_DELAY_SECONDS = 0.05
    IDEMPOTENT_COMMAND_MAX_ATTEMPTS = 3
    # F3长包通信失败后，通过位置容差、运动方向和稳定状态确认是否可安全重发。
    MOVE_REJECTED_MAX_RETRIES = 2
    MOVE_REJECTED_RETRY_DELAY_SECONDS = 0.20
    MOVE_REJECTED_POSITION_CONFIRM_SAMPLES = 3
    MOVE_REJECTED_POSITION_CONFIRM_INTERVAL_SECONDS = 0.05
    MOVE_REJECTED_POSITION_TOLERANCE_COUNTS = 10
    MOVE_REJECTED_SETTLED_DELTA_COUNTS = 10

    def __init__(
        self,
        config: PD42S1Config,
        event_logger: Optional[Callable[[str], None]] = None,
    ):
        self.config = config
        self.bus: Any = None
        self.can_id_rx = config.can_id_tx if config.can_id_rx is None else config.can_id_rx
        self._request_lock = threading.Lock()
        self._event_logger = event_logger
        self._request_sequence = 0
        self._last_command_sent_at = 0.0
        self._communication_recovery_generation = 0

    @property
    def communication_recovery_generation(self):
        return self._communication_recovery_generation

    def _log_event(self, message: str) -> None:
        if self._event_logger is None:
            return
        try:
            self._event_logger(message)
        except Exception:
            # Diagnostic logging must never break motor control.
            pass

    def open(self) -> None:
        if self.bus is not None:
            self._log_event("OPEN skipped: CAN is already connected")
            return
        if can is None:
            raise PD42S1Error(
                "缺少 python-can，无法连接 PCAN。请先执行: py -3 -m pip install python-can"
            )
        started = time.perf_counter()
        self._log_event(
            f"OPEN begin interface={self.config.interface} "
            f"channel={self.config.channel} bitrate={self.config.bitrate}"
        )
        try:
            self.bus = can.Bus(
                interface=self.config.interface,
                channel=self.config.channel,
                bitrate=self.config.bitrate,
            )
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self._log_event(f"OPEN error elapsed={elapsed_ms:.3f} ms: {exc!r}")
            raise
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._log_event(f"OPEN ok elapsed={elapsed_ms:.3f} ms")

    def close(self) -> None:
        bus = self.bus
        self.bus = None
        if bus is not None:
            started = time.perf_counter()
            self._log_event("CLOSE begin")
            try:
                bus.shutdown()
            except Exception as exc:
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                self._log_event(f"CLOSE error elapsed={elapsed_ms:.3f} ms: {exc!r}")
                raise
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self._log_event(f"CLOSE ok elapsed={elapsed_ms:.3f} ms")

    @staticmethod
    def _u16_be(value: int) -> bytes:
        return int(value).to_bytes(2, byteorder="big", signed=False)

    @staticmethod
    def _u32_be(value: int) -> bytes:
        return int(value).to_bytes(4, byteorder="big", signed=False)

    @staticmethod
    def _be_to_i16(data: bytes) -> int:
        return int.from_bytes(data, byteorder="big", signed=True)

    @staticmethod
    def _be_to_i32(data: bytes) -> int:
        return int.from_bytes(data, byteorder="big", signed=True)

    @staticmethod
    def _checksum(body: bytes) -> int:
        return sum(body) & 0xFF

    def _build_packet(self, function_code: int, data: bytes = b"") -> bytes:
        if not 0 <= self.config.node_id <= 0xFF:
            raise ValueError("node_id must be in the range 0..255")
        body = bytes((self.FRAME_HEAD, self.config.node_id, function_code)) + data
        return body + bytes((self._checksum(body), self.FRAME_TAIL))

    def _validate_packet(self, packet: bytes) -> None:
        if len(packet) < 5:
            raise PD42S1ProtocolError(f"packet too short: {packet.hex(' ')}")
        if packet[0] != self.FRAME_HEAD or packet[-1] != self.FRAME_TAIL:
            raise PD42S1ProtocolError(f"invalid packet boundary: {packet.hex(' ')}")
        if self._checksum(packet[:-2]) != packet[-2]:
            raise PD42S1ProtocolError(f"checksum error: {packet.hex(' ')}")

    def _require_bus(self):
        if self.bus is None:
            raise PD42S1Error("CAN 尚未连接")
        return self.bus

    def _send_packet(self, packet: bytes) -> None:
        bus = self._require_bus()
        for offset in range(0, len(packet), 8):
            chunk = packet[offset:offset + 8]
            message = can.Message(
                arbitration_id=self.config.can_id_tx,
                is_extended_id=self.config.is_extended_id,
                data=chunk,
            )
            frame_number = (offset // 8) + 1
            started = time.perf_counter()
            self._log_event(
                f"TX begin frame={frame_number} id=0x{self.config.can_id_tx:08X} "
                f"dlc={len(chunk)} data={chunk.hex(' ').upper()}"
            )
            try:
                bus.send(message)
            except Exception as exc:
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                self._log_event(
                    f"TX error frame={frame_number} elapsed={elapsed_ms:.3f} ms: {exc!r}"
                )
                raise
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self._log_event(f"TX ok frame={frame_number} elapsed={elapsed_ms:.3f} ms")
            if self.config.debug:
                print(f"PD42S1 TX {chunk.hex(' ').upper()}")
            if offset + 8 < len(packet) and self.config.inter_frame_delay > 0:
                time.sleep(self.config.inter_frame_delay)

    def _wait_command_interval(self) -> None:
        interval = max(0.0, float(self.config.command_interval))
        if interval <= 0.0 or self._last_command_sent_at <= 0.0:
            return
        remaining = interval - (time.monotonic() - self._last_command_sent_at)
        if remaining > 0.0:
            time.sleep(remaining)

    def _recv_packet(self, expected_function: int, timeout: Optional[float] = None) -> bytes:
        bus = self._require_bus()
        effective_timeout = self.config.default_timeout if timeout is None else timeout
        started = time.perf_counter()
        deadline = time.monotonic() + effective_timeout
        buffer = bytearray()
        ignored_id_count = 0

        self._log_event(
            f"RX wait func=0x{expected_function:02X} timeout={effective_timeout:.3f} s "
            f"expected_id=0x{self.can_id_rx:08X}"
        )

        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                message = bus.recv(timeout=min(self.config.rx_poll_interval, remaining))
            except Exception as exc:
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                self._log_event(f"RX error elapsed={elapsed_ms:.3f} ms: {exc!r}")
                raise
            if message is None:
                continue
            if self.can_id_rx is not None and message.arbitration_id != self.can_id_rx:
                ignored_id_count += 1
                continue

            chunk = bytes(message.data)
            buffer.extend(chunk)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            frame_type = "EXT" if getattr(message, "is_extended_id", False) else "STD"
            self._log_event(
                f"RX frame elapsed={elapsed_ms:.3f} ms id=0x{message.arbitration_id:08X} "
                f"type={frame_type} dlc={len(chunk)} data={chunk.hex(' ').upper()}"
            )
            if self.config.debug:
                print(f"PD42S1 RX {chunk.hex(' ').upper()}")

            while buffer and buffer[0] != self.FRAME_HEAD:
                del buffer[0]

            for end_index, value in enumerate(buffer):
                if value != self.FRAME_TAIL:
                    continue
                candidate = bytes(buffer[:end_index + 1])
                try:
                    self._validate_packet(candidate)
                except PD42S1ProtocolError:
                    continue
                del buffer[:end_index + 1]
                if candidate[2] == expected_function:
                    elapsed_ms = (time.perf_counter() - started) * 1000.0
                    self._log_event(
                        f"RX packet ok func=0x{expected_function:02X} "
                        f"elapsed={elapsed_ms:.3f} ms ignored_ids={ignored_id_count} "
                        f"data={candidate.hex(' ').upper()}"
                    )
                    return candidate
                self._log_event(
                    f"RX packet skipped expected_func=0x{expected_function:02X} "
                    f"actual_func=0x{candidate[2]:02X} data={candidate.hex(' ').upper()}"
                )
                break

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._log_event(
            f"RX timeout func=0x{expected_function:02X} elapsed={elapsed_ms:.3f} ms "
            f"ignored_ids={ignored_id_count} partial={bytes(buffer).hex(' ').upper() or '-'}"
        )
        raise PD42S1TimeoutError(
            f"等待电机响应超时，功能码 0x{expected_function:02X}，"
            f"已等待 {elapsed_ms:.1f} ms"
        )

    def _request(self, function_code: int, data: bytes = b"") -> bytes:
        with self._request_lock:
            self._request_sequence += 1
            request_id = self._request_sequence
            packet = self._build_packet(function_code, data)
            started = time.perf_counter()
            self._log_event(
                f"REQ #{request_id} begin func=0x{function_code:02X} "
                f"payload={data.hex(' ').upper() or '-'}"
            )
            try:
                self._wait_command_interval()
                try:
                    self._send_packet(packet)
                finally:
                    self._last_command_sent_at = time.monotonic()
                response = self._recv_packet(function_code)
            except Exception as exc:
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                self._log_event(
                    f"REQ #{request_id} error func=0x{function_code:02X} "
                    f"elapsed={elapsed_ms:.3f} ms: {exc!r}"
                )
                raise
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self._log_event(
                f"REQ #{request_id} ok func=0x{function_code:02X} "
                f"elapsed={elapsed_ms:.3f} ms"
            )
            return response

    @staticmethod
    def _is_retryable_communication_error(exc, function_code=None):
        if isinstance(exc, PD42S1DeviceError):
            return (
                (function_code is None or exc.function_code == int(function_code))
                and exc.result_code in (0xE1, 0xE2, 0xE3, 0xE4)
            )
        if isinstance(exc, PD42S1CommunicationError):
            return True
        can_error_type = None if can is None else getattr(can, "CanError", None)
        return can_error_type is not None and isinstance(exc, can_error_type)

    def _request_ok_with_retry(
        self,
        function_code,
        data=b"",
        max_attempts=None,
        retry_label="COMMAND",
    ):
        """重试不会产生累积副作用的查询或控制命令。"""
        max_attempts = int(max_attempts or self.IDEMPOTENT_COMMAND_MAX_ATTEMPTS)
        last_error = None
        for attempt in range(1, max_attempts + 1):
            try:
                response = self._request(function_code, data)
                self._ensure_ok(response)
                if attempt > 1:
                    self._communication_recovery_generation += 1
                    self._log_event(
                        f"{retry_label} communication recovered "
                        f"func=0x{function_code:02X} attempt={attempt}/{max_attempts}"
                    )
                return response
            except Exception as exc:
                if not self._is_retryable_communication_error(exc, function_code):
                    raise
                last_error = exc
                if attempt >= max_attempts:
                    self._log_event(
                        f"{retry_label} retry exhausted func=0x{function_code:02X} "
                        f"attempts={max_attempts}: {exc}"
                    )
                    break

                self._log_event(
                    f"{retry_label} retry func=0x{function_code:02X} "
                    f"next_attempt={attempt + 1}/{max_attempts} "
                    f"delay={self.READ_QUERY_RETRY_DELAY_SECONDS * 1000.0:.0f} ms"
                )
                time.sleep(self.READ_QUERY_RETRY_DELAY_SECONDS)

        raise PD42S1CommunicationError(
            f"电机通信连续失败，功能码0x{function_code:02X}，"
            f"已尝试{max_attempts}次：{last_error}"
        ) from last_error

    def _request_read_with_retry(self, function_code: int) -> bytes:
        """只读查询发生超时或E1～E4帧错误时自动重发。"""
        return self._request_ok_with_retry(
            function_code,
            max_attempts=self.READ_QUERY_MAX_ATTEMPTS,
            retry_label="READ",
        )

    def _ensure_ok(self, response: bytes) -> None:
        if len(response) < 4 or response[3] != 0x01:
            result = response[3] if len(response) > 3 else -1
            function_code = response[2] if len(response) > 2 else -1
            self._log_event(
                f"DEVICE rejected func=0x{function_code:02X} result=0x{result:02X} "
                f"data={response.hex(' ').upper()}"
            )
            raise PD42S1DeviceError(function_code, result, response)
        self._log_event(
            f"DEVICE accepted func=0x{response[2]:02X} result=0x{response[3]:02X}"
        )

    def enable_motor(self) -> int:
        response = self._request_ok_with_retry(self.CMD_ENABLE, bytes((0x00,)))
        return response[4]

    def disable_motor(self) -> int:
        response = self._request_ok_with_retry(self.CMD_ENABLE, bytes((0x01,)))
        return response[4]

    def clear_state(self) -> None:
        self._request_ok_with_retry(self.CMD_CLEAR_STATE)

    def brake_stop(self, auto_clear: bool = False, clear_delay: float = 0.05) -> None:
        self._request_ok_with_retry(self.CMD_BRAKE_STOP)
        if auto_clear:
            time.sleep(clear_delay)
            self.clear_state()

    def read_phase_current_ma(self) -> int:
        response = self._request_read_with_retry(self.CMD_READ_PHASE_CURRENT)
        if len(response) < 8:
            raise PD42S1ProtocolError(
                f"phase-current response too short: {response.hex(' ')}"
            )
        return self._be_to_i16(response[4:6])

    def read_position_counts(self) -> int:
        response = self._request_read_with_retry(self.CMD_READ_POSITION)
        if len(response) < 10:
            raise PD42S1ProtocolError(f"position response too short: {response.hex(' ')}")
        return self._be_to_i32(response[4:8])

    def read_speed_rpm(self) -> int:
        response = self._request_read_with_retry(self.CMD_READ_SPEED)
        if len(response) < 8:
            raise PD42S1ProtocolError(
                f"speed response too short: {response.hex(' ')}"
            )
        return self._be_to_i16(response[4:6])

    def read_running_state(self) -> int:
        response = self._request_read_with_retry(self.CMD_READ_RUNNING_STATE)
        if len(response) < 7:
            raise PD42S1ProtocolError(
                f"running-state response too short: {response.hex(' ')}"
            )
        return response[4]

    def read_in_position(self) -> bool:
        response = self._request_read_with_retry(self.CMD_READ_IN_POSITION)
        if len(response) < 7:
            raise PD42S1ProtocolError(
                f"in-position response too short: {response.hex(' ')}"
            )
        return response[4] == 0x01

    def set_max_torque_current_ma(self, current_ma: int) -> int:
        """设置速度/位置环硬件最大力矩电流，并核对应答回读值。"""
        current_ma = int(current_ma)
        if not 0 <= current_ma <= 3000:
            raise ValueError("current_ma must be in the range 0..3000")
        response = self._request_ok_with_retry(
            self.CMD_SET_MAX_TORQUE_CURRENT,
            self._u16_be(current_ma),
        )
        if len(response) < 8:
            raise PD42S1ProtocolError(
                f"max-torque-current response too short: {response.hex(' ')}"
            )
        applied_current_ma = int.from_bytes(
            response[4:6],
            byteorder="big",
            signed=False,
        )
        if applied_current_ma != current_ma:
            raise PD42S1ProtocolError(
                "max-torque-current readback mismatch: "
                f"requested={current_ma}, applied={applied_current_ma}"
            )
        return applied_current_ma

    def zero_position(self) -> None:
        response = self._request(self.CMD_ZERO_POSITION)
        self._ensure_ok(response)

    def move_relative_deg(self, delta_deg: float, speed_rpm: int, accel: int = 100) -> None:
        counts = round(delta_deg / 360.0 * self.COUNTS_PER_TURN)
        self.move_relative_counts(counts, speed_rpm, accel)

    def _wait_move_retry_delay(self, delay_seconds, retry_cancelled, cause):
        deadline = time.monotonic() + float(delay_seconds)
        while True:
            if retry_cancelled is not None and retry_cancelled():
                raise PD42S1Error("F3/E3安全重试已由停止请求取消") from cause
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.02, remaining))

    def _assess_rejected_move(
        self,
        start_counts,
        delta_counts,
        retry_cancelled,
        cause,
    ):
        def read_recovery_position():
            while True:
                if retry_cancelled is not None and retry_cancelled():
                    raise PD42S1Error("F3运动状态复核已由停止请求取消") from cause
                try:
                    return self.read_position_counts()
                except PD42S1CommunicationError as exc:
                    self._log_event(
                        "MOVE recovery position read failed; waiting before retry: "
                        f"{exc}"
                    )
                    self._wait_move_retry_delay(
                        self.MOVE_REJECTED_RETRY_DELAY_SECONDS,
                        retry_cancelled,
                        exc,
                    )

        direction = 1 if int(delta_counts) >= 0 else -1
        positions = []
        confirmation_not_before = (
            time.monotonic() + self.MOVE_REJECTED_RETRY_DELAY_SECONDS
        )
        while True:
            position = read_recovery_position()
            positions.append(position)
            positions = positions[-20:]
            commanded_progress = (position - start_counts) * direction
            if commanded_progress > self.MOVE_REJECTED_POSITION_TOLERANCE_COUNTS:
                return True, positions
            stable_window = positions[-self.MOVE_REJECTED_POSITION_CONFIRM_SAMPLES:]
            stable = (
                len(stable_window) >= self.MOVE_REJECTED_POSITION_CONFIRM_SAMPLES
                and max(stable_window) - min(stable_window)
                <= self.MOVE_REJECTED_SETTLED_DELTA_COUNTS
            )
            if time.monotonic() >= confirmation_not_before and stable:
                return False, positions
            self._wait_move_retry_delay(
                self.MOVE_REJECTED_POSITION_CONFIRM_INTERVAL_SECONDS,
                retry_cancelled,
                cause,
            )

    def move_relative_counts(
        self,
        delta_counts: int,
        speed_rpm: int,
        accel: int = 100,
        retry_cancelled: Optional[Callable[[], bool]] = None,
    ) -> dict:
        if not 1 <= speed_rpm <= self.MAX_SPEED_RPM:
            raise ValueError(
                f"speed_rpm must be in the range 1..{self.MAX_SPEED_RPM}"
            )
        if not 0 <= accel <= 200:
            raise ValueError("accel must be in the range 0..200")

        direction = 0x00 if delta_counts >= 0 else 0x01
        payload = (
            bytes((direction, accel))
            + self._u16_be(speed_rpm)
            + self._u32_be(abs(int(delta_counts)))
        )
        start_counts = self.read_position_counts()
        for attempt in range(self.MOVE_REJECTED_MAX_RETRIES + 1):
            try:
                response = self._request(self.CMD_MOVE_REL, payload)
                self._ensure_ok(response)
                if attempt:
                    self._communication_recovery_generation += 1
                    self._log_event(
                        f"MOVE F3/E3 safe retry accepted attempt={attempt}/"
                        f"{self.MOVE_REJECTED_MAX_RETRIES}"
                    )
                return {
                    "accepted": True,
                    "response_recovered": bool(attempt),
                    "attempts": attempt + 1,
                    "positions": (),
                }
            except Exception as exc:
                if not self._is_retryable_communication_error(
                    exc,
                    self.CMD_MOVE_REL,
                ):
                    raise
                command_started, positions = self._assess_rejected_move(
                    start_counts,
                    delta_counts,
                    retry_cancelled,
                    exc,
                )
                if command_started:
                    self._communication_recovery_generation += 1
                    self._log_event(
                        "MOVE response error recovered: commanded movement observed; "
                        f"same command will not be resent samples={positions} cause={exc}"
                    )
                    return {
                        "accepted": False,
                        "response_recovered": True,
                        "movement_observed": True,
                        "attempts": attempt + 1,
                        "positions": tuple(positions),
                    }
                if attempt >= self.MOVE_REJECTED_MAX_RETRIES:
                    raise PD42S1MoveCommandUnconfirmedError(
                        "F3运动命令通信重试后仍未收到成功应答；"
                        f"已确认位置未沿指令方向移动，发送前={start_counts}，"
                        f"复核={positions} counts"
                    ) from exc
                self._log_event(
                    "MOVE response error; no commanded movement confirmed "
                    f"samples={positions}; safe retry attempt={attempt + 1}/"
                    f"{self.MOVE_REJECTED_MAX_RETRIES}"
                )

        raise AssertionError("unreachable move retry path")
