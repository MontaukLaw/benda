import struct
import threading
import time
import math
from collections import deque

from config import (
    BAUD_RATE,
    COMMAND_TYPE_RECALIBRATE,
    COMMAND_TYPE_READ_FORCE_CALIBRATION,
    COMMAND_TYPE_SET_FORCE_CALIBRATION,
    COMMAND_TYPE_SAVE_FORCE_CALIBRATION,
    COMMAND_TYPE_DISCARD_FORCE_CALIBRATION,
    COMMAND_TYPE_GET_CAPSENSE_NSUB_TIER,
    COMMAND_TYPE_GET_CAPSENSE_MATRIX_CONFIG,
    COMMAND_TYPE_SET_CAPSENSE_MATRIX_CONFIG,
    COMMAND_TYPE_GET_CAPSENSE_AUTORANGE_STATUS,
    COMMAND_TYPE_SET_CAPSENSE_AUTORANGE_CALIBRATION,
    COMMAND_TYPE_STAGE_CAPSENSE_AFFINE_CALIBRATION,
    COMMAND_TYPE_SAVE_CAPSENSE_AFFINE_CALIBRATION,
    COMMAND_TYPE_GET_CAPSENSE_AFFINE_CALIBRATION,
    COMMAND_TYPE_SOFTWARE_RESET,
    COMMAND_TYPE_GET_DEVICE_INFO,
    COMMAND_TYPE_GET_CAPSENSE_CHANNEL_CONFIG,
    COMMAND_TYPE_SET_OUTPUT_MODE,
    DEVICE_TYPE,
    FORCE_CALIBRATION_MAX_POINTS,
    FORCE_CALIBRATION_MIN_PROTOCOL_VERSION,
    AFFINE_CALIBRATION_ENABLED,
    FRAME_MAGIC,
    FRAME_VERSION,
)
from frame_parser import AlgoFrameParser
from low_force_calibration import validate_points, validate_detection_counts
from qt_compat import QtCore, Signal


LIVE_NOISE_WINDOW_FRAMES = 64


class SerialReaderThread(QtCore.QThread):
    state_changed = Signal(bool, str)
    stream_started = Signal()
    command_ack = Signal(int)
    nsub_status = Signal(object)
    matrix_config_status = Signal(object)
    autorange_status = Signal(object)
    affine_calibration_status = Signal(object)
    affine_operation_status = Signal(object)
    live_noise = Signal(object)
    device_info = Signal(object)
    capsense_channel_config = Signal(object)
    force_calibration_parameters = Signal(object)
    low_force_calibration = Signal(object)

    def __init__(self, port_name, data_buffer, baud_rate=BAUD_RATE, parent=None, data_recorder=None, rolling_log=None):
        super().__init__(parent)
        self.port_name = port_name
        self.data_buffer = data_buffer
        self.baud_rate = baud_rate
        self._protocol_version = FRAME_VERSION
        self._protocol_version_confirmed = False
        self._channel_count = 0
        self._running = False
        self._stop_requested = threading.Event()
        self._serial = None
        self._write_lock = threading.Lock()
        self._live_noise_lock = threading.Lock()
        self._live_noise_windows = []
        self._live_noise_sums = []
        self._live_noise_sum_squares = []
        self.data_recorder = data_recorder
        self.rolling_log = rolling_log

    def _log(self, kind, **fields):
        if self.rolling_log is not None:
            self.rolling_log.submit(kind, port=self.port_name, **fields)

    def stop(self):
        self._stop_requested.set()
        self._running = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass

    def send_command(self, command):
        with self._write_lock:
            if self._serial is None or not self._serial.is_open:
                return False
            try:
                written = self._serial.write(command)
                self._log('tx', data=command[:written])
                self._serial.flush()
                return True
            except Exception as exc:
                self._log('tx_error', message=str(exc))
                return False

    def send_output_mode(self, output_mode):
        command = struct.pack(
            "<4sBBBBHB",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_SET_OUTPUT_MODE,
            DEVICE_TYPE,
            0,
            1,
            output_mode,
        )
        sent = self.send_command(command + command)
        if sent:
            self.reset_live_noise()
        return sent

    def send_recalibration(self):
        return self.send_command(
            struct.pack(
                "<4sBBBBH",
                FRAME_MAGIC,
                self._protocol_version,
                COMMAND_TYPE_RECALIBRATE,
                DEVICE_TYPE,
                0,
                0,
            )
        )

    def get_nsub_tier_status(self):
        command = struct.pack(
            "<4sBBBBH",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_GET_CAPSENSE_NSUB_TIER,
            DEVICE_TYPE,
            0,
            0,
        )
        return self.send_command(command)

    def get_matrix_config(self):
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, self._protocol_version,
            COMMAND_TYPE_GET_CAPSENSE_MATRIX_CONFIG, DEVICE_TYPE, 0, 0))

    def set_matrix_config(self, nsub, tx_clk_div, ref_cdac):
        sent = self.send_command(struct.pack(
            "<4sBBBBHHHB",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_SET_CAPSENSE_MATRIX_CONFIG,
            DEVICE_TYPE,
            0,
            5,
            int(nsub),
            int(tx_clk_div),
            int(ref_cdac),
        ))
        if sent:
            self.reset_live_noise()
        return sent

    def get_autorange_status(self):
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, self._protocol_version,
            COMMAND_TYPE_GET_CAPSENSE_AUTORANGE_STATUS, DEVICE_TYPE, 0, 0))

    def set_autorange_calibration_mode(self, enabled, ref_cdac=0):
        mode = 1 if enabled else 0
        ref_value = int(ref_cdac) if enabled else 0
        command = struct.pack(
            "<4sBBBBHBB",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_SET_CAPSENSE_AUTORANGE_CALIBRATION,
            DEVICE_TYPE,
            0,
            2,
            mode,
            ref_value,
        )
        return self.send_command(command)

    def get_force_calibration(self, region, force_type):
        payload = struct.pack("<BB", int(region), int(force_type))
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, self._protocol_version,
            COMMAND_TYPE_READ_FORCE_CALIBRATION, DEVICE_TYPE, 0, len(payload)
        ) + payload)

    def set_force_calibration(
        self, region, force_type, raw_points, calibrated_points
    ):
        if not self._protocol_version_confirmed or self._protocol_version < FORCE_CALIBRATION_MIN_PROTOCOL_VERSION:
            return False
        raw_points = tuple(float(value) for value in raw_points)
        calibrated_points = tuple(float(value) for value in calibrated_points)
        if (
            len(raw_points) != len(calibrated_points)
            or len(raw_points) < 2
            or len(raw_points) > FORCE_CALIBRATION_MAX_POINTS
            or int(region) not in (0, 1)
            or int(force_type) not in (0, 1, 2)
            or (int(region) == 0 and int(force_type) == 0 and
                len(raw_points) not in ((10, 20, FORCE_CALIBRATION_MAX_POINTS) if self._protocol_version >= 21 else (FORCE_CALIBRATION_MAX_POINTS,)))
            or (int(force_type) == 1 and len(raw_points) > 20)
            or (int(force_type) == 2 and len(raw_points) != 2)
        ):
            return False
        payload = struct.pack(
            "<BBBBBBBB",
            int(region),
            int(force_type),
            2,
            len(raw_points),
            0,
            0,
            0,
            0,
        )
        payload += b"".join(
            struct.pack("<ff", raw_value, calibrated_value)
            for raw_value, calibrated_value in zip(
                raw_points, calibrated_points
            )
        )
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, self._protocol_version,
            COMMAND_TYPE_SET_FORCE_CALIBRATION, DEVICE_TYPE, 0, len(payload)
        ) + payload)

    def save_force_calibration(self):
        if not self._protocol_version_confirmed or self._protocol_version < FORCE_CALIBRATION_MIN_PROTOCOL_VERSION:
            return False
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, self._protocol_version,
            COMMAND_TYPE_SAVE_FORCE_CALIBRATION, DEVICE_TYPE, 0, 0
        ))

    def discard_force_calibration(self):
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, self._protocol_version,
            COMMAND_TYPE_DISCARD_FORCE_CALIBRATION, DEVICE_TYPE, 0, 0
        ))

    def stage_affine_calibration_channel(self, calibration_id, channel, coefficients):
        if not AFFINE_CALIBRATION_ENABLED:
            return False
        if len(coefficients) != 7:
            return False
        payload = struct.pack(
            "<IBBBB",
            int(calibration_id) & 0xFFFFFFFF,
            int(channel),
            1,
            7,
            0,
        )
        payload += b"".join(
            struct.pack("<ii", int(gain_q16), int(offset))
            for gain_q16, offset in coefficients
        )
        command = struct.pack(
            "<4sBBBBH",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_STAGE_CAPSENSE_AFFINE_CALIBRATION,
            DEVICE_TYPE,
            0,
            len(payload),
        )
        return self.send_command(command + payload)

    def save_affine_calibration(self, calibration_id):
        if not AFFINE_CALIBRATION_ENABLED:
            return False
        payload = struct.pack("<I", int(calibration_id) & 0xFFFFFFFF)
        command = struct.pack(
            "<4sBBBBH",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_SAVE_CAPSENSE_AFFINE_CALIBRATION,
            DEVICE_TYPE,
            0,
            len(payload),
        )
        return self.send_command(command + payload)

    def get_affine_calibration_channel(self, channel):
        if not AFFINE_CALIBRATION_ENABLED:
            return False
        payload = struct.pack("<B", int(channel))
        command = struct.pack(
            "<4sBBBBH",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_GET_CAPSENSE_AFFINE_CALIBRATION,
            DEVICE_TYPE,
            0,
            len(payload),
        )
        return self.send_command(command + payload)

    def software_reset(self):
        command = struct.pack(
            "<4sBBBBH",
            FRAME_MAGIC,
            self._protocol_version,
            COMMAND_TYPE_SOFTWARE_RESET,
            DEVICE_TYPE,
            0,
            0,
        )
        return self.send_command(command)

    def low_force_command(self, operation, token, points=None, detection_counts=0.0):
        if not self._protocol_version_confirmed or self._protocol_version < 17 or operation not in (0x98, 0x99):
            return False
        payload = struct.pack("<I", token)
        if operation == 0x99:
            try:
                points = validate_points(points)
                detection_counts = validate_detection_counts(detection_counts, points)
                if self._protocol_version < 19 and detection_counts != 0:
                    return False
                payload += struct.pack("<4f", *points)
                if self._protocol_version >= 19:
                    payload += struct.pack("<f", detection_counts)
            except (ValueError, TypeError, OverflowError):
                return False
        return self.send_command(struct.pack("<4sBBBBH", FRAME_MAGIC, self._protocol_version,
                                             operation, DEVICE_TYPE, 0, len(payload)) + payload)

    def get_device_info(self, protocol_version=FRAME_VERSION):
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, int(protocol_version),
            COMMAND_TYPE_GET_DEVICE_INFO, DEVICE_TYPE, 0, 0))

    def get_capsense_channel_config(self):
        return self.send_command(struct.pack(
            "<4sBBBBH", FRAME_MAGIC, self._protocol_version,
            COMMAND_TYPE_GET_CAPSENSE_CHANNEL_CONFIG, DEVICE_TYPE, 0, 0))

    def reset_live_noise(self):
        with self._live_noise_lock:
            for window in self._live_noise_windows:
                window.clear()
            self._live_noise_sums[:] = [0] * len(self._live_noise_sums)
            self._live_noise_sum_squares[:] = [0] * len(self._live_noise_sum_squares)

    def _update_live_noise(self, raw_values):
        if not raw_values:
            return

        noise_values = []
        with self._live_noise_lock:
            if len(self._live_noise_windows) != len(raw_values):
                self._live_noise_windows = [
                    deque(maxlen=LIVE_NOISE_WINDOW_FRAMES) for _ in raw_values
                ]
                self._live_noise_sums = [0] * len(raw_values)
                self._live_noise_sum_squares = [0] * len(raw_values)

            for index, value in enumerate(raw_values):
                value = int(value)
                window = self._live_noise_windows[index]
                if len(window) == LIVE_NOISE_WINDOW_FRAMES:
                    oldest = window[0]
                    self._live_noise_sums[index] -= oldest
                    self._live_noise_sum_squares[index] -= oldest * oldest

                window.append(value)
                self._live_noise_sums[index] += value
                self._live_noise_sum_squares[index] += value * value

                sample_count = len(window)
                mean = self._live_noise_sums[index] / sample_count
                variance = (self._live_noise_sum_squares[index] / sample_count) - (mean * mean)
                noise_values.append(math.sqrt(max(0.0, variance)))

            sample_count = len(self._live_noise_windows[0])

        self.live_noise.emit({
            "type": "live_noise",
            "noise": tuple(noise_values),
            "sample_count": sample_count,
            "window_size": LIVE_NOISE_WINDOW_FRAMES,
        })

    def run(self):
        if self._stop_requested.is_set():
            return
        try:
            import serial
        except ImportError:
            self.state_changed.emit(False, "pyserial is not installed")
            return

        parser = AlgoFrameParser()
        self._running = True
        stream_started = False

        try:
            self._serial = serial.Serial(self.port_name, self.baud_rate, timeout=0.02)
            self._log('connected', message=f'{self.baud_rate} baud, 8N1')
            if self._stop_requested.is_set():
                return
            self.state_changed.emit(True, "Connected: " + self.port_name)
            self.get_device_info()
            self.get_device_info(19)
            self.get_device_info(18)
            self.get_device_info(17)
            self.get_device_info(16)
            self.get_device_info(15)
            self.get_device_info(14)

            while self._running and not self._stop_requested.is_set():
                waiting = self._serial.in_waiting
                data = self._serial.read(min(waiting, 65536) if waiting else 1)
                if not data:
                    time.sleep(0.001)
                    continue

                self._log('rx', data=data)
                for frame in parser.feed(data):
                    self._log('frame', frame=frame)
                    # 首次模式命令使用实际流版本，避免V14设备收到V15命令后静默忽略。
                    self._protocol_version = int(frame.get("protocol_version", self._protocol_version))
                    self._protocol_version_confirmed = True
                    # 只有全设备帧描述总通道数；四通道统计/矩阵帧不能把八通道设备覆盖成四通道。
                    if frame["type"] in ("device_info", "signal", "debug", "run_force", "capsense_channel_config_status") and frame.get("channel_count", 0):
                        self._channel_count = frame["channel_count"]
                    if self.data_recorder is not None:
                        self.data_recorder.record(frame)
                    # 独立诊断帧保留在完整日志及历史快照中，但不计入采样显示。
                    if frame["type"] == "slow_baseline":
                        continue
                    if frame["type"] == "command_ack":
                        self.command_ack.emit(frame["result"])
                    elif frame["type"] == "capsense_nsub_tier_status":
                        self.nsub_status.emit(frame)
                    elif frame["type"] == "capsense_matrix_config_status":
                        self.matrix_config_status.emit(frame)
                    elif frame["type"] == "capsense_autorange_status":
                        self.autorange_status.emit(frame)
                    elif frame["type"] == "capsense_affine_calibration_status":
                        self.affine_calibration_status.emit(frame)
                    elif frame["type"] == "capsense_affine_operation_status":
                        self.affine_operation_status.emit(frame)
                    elif frame["type"] == "device_info":
                        self._protocol_version = int(
                            frame.get("protocol_version", FRAME_VERSION)
                        )
                        self.device_info.emit(frame)
                    elif frame["type"] == "capsense_channel_config_status":
                        self.capsense_channel_config.emit(frame)
                    elif frame["type"] == "force_calibration_parameters":
                        self.force_calibration_parameters.emit(frame)
                    elif frame["type"] == "low_force_calibration":
                        self.low_force_calibration.emit(frame)
                    else:
                        if frame["type"] == "debug":
                            self._update_live_noise(frame["raw"])
                        if not stream_started:
                            stream_started = True
                            self.stream_started.emit()
                        self.data_buffer.append_frame(frame)

        except Exception as exc:
            self._log('serial_error', message=str(exc))
            if self._running:
                self.state_changed.emit(False, str(exc))
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._running = False
            self._log('disconnected')
            self.state_changed.emit(False, "Disconnected")
