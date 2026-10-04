import struct
from config import FRAME_TYPE_LOW_FORCE_CALIBRATION
from low_force_calibration import validate_points, validate_detection_counts

from config import (
    BYTES_PER_FLOAT,
    BYTES_PER_VALUE,
    FRAME_HEADER_SIZE,
    FRAME_MAGIC,
    FORCE_CALIBRATION_MAX_POINTS,
    FRAME_TYPE_COMMAND_ACK,
    FRAME_TYPE_CAPSENSE_NSUB_TIER_STATUS,
    FRAME_TYPE_CAPSENSE_MATRIX_CONFIG_STATUS,
    FRAME_TYPE_CAPSENSE_AUTORANGE_STATUS,
    FRAME_TYPE_CAPSENSE_AFFINE_CALIBRATION_STATUS,
    FRAME_TYPE_CAPSENSE_AFFINE_OPERATION_STATUS,
    FRAME_TYPE_DEVICE_INFO,
    FRAME_TYPE_CAPSENSE_CHANNEL_CONFIG_STATUS,
    FRAME_TYPE_CHANNEL_4567_SIGNAL_SUM,
    FRAME_TYPE_CHANNEL_4567_STATISTICS,
    FRAME_TYPE_SLOW_BASELINE,
    FRAME_TYPE_DEBUG,
    FRAME_TYPE_RAW,
    FRAME_TYPE_FORCE_VECTOR,
    FRAME_TYPE_FINGER_PAD_FORCE_VECTOR,
    FRAME_TYPE_FORCE_CALIBRATION_PARAMETERS,
    FRAME_TYPE_RUN_FORCE,
    FRAME_TYPE_SIGNAL,
    FRAME_TYPE_TEMPERATURE,
    SUPPORTED_FRAME_VERSIONS,
)


class AlgoFrameParser:
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        self._buffer.extend(data)
        frames = []

        while True:
            magic_index = self._buffer.find(FRAME_MAGIC)
            if magic_index < 0:
                max_prefix = max(0, len(self._buffer) - (len(FRAME_MAGIC) - 1))
                if max_prefix:
                    del self._buffer[:max_prefix]
                break

            if magic_index > 0:
                del self._buffer[:magic_index]

            if len(self._buffer) < FRAME_HEADER_SIZE:
                break

            magic, version, frame_type, device, channel_count, payload_len = struct.unpack_from(
                "<4sBBBBH",
                self._buffer,
                0,
            )
            if (magic != FRAME_MAGIC or version not in SUPPORTED_FRAME_VERSIONS
                    or channel_count > 20 or payload_len > 1024):
                del self._buffer[0]
                continue

            frame_size = FRAME_HEADER_SIZE + payload_len
            if len(self._buffer) < frame_size:
                break

            payload = bytes(self._buffer[FRAME_HEADER_SIZE:frame_size])
            frame = self._parse_payload(
                version, frame_type, device, channel_count, payload_len, payload
            )
            if frame is not None:
                # 每一种有效帧都能证明协议版本，不依赖设备身份响应先到达。
                frame.setdefault("protocol_version", version)
                frames.append(frame)
            del self._buffer[:frame_size]

        return frames

    def _parse_payload(
        self, protocol_version, frame_type, device, channel_count, payload_len, payload
    ):
        if frame_type == FRAME_TYPE_LOW_FORCE_CALIBRATION:
            expected_length = 28 if protocol_version >= 19 else 24
            if protocol_version < 17 or channel_count != 0 or payload_len != expected_length:
                return None
            operation, result, flags, token, *points = struct.unpack_from("<BBHI4f", payload)
            detection_counts = struct.unpack_from("<f", payload, 24)[0] if protocol_version >= 19 else None
            if operation not in (0x98, 0x99) or flags & ~3:
                return None
            if flags & 2:
                try:
                    validate_points(points)
                    if detection_counts is not None:
                        validate_detection_counts(detection_counts, points)
                except ValueError:
                    return None
            return {"type": "low_force_calibration", "device": device, "channel_count": 0,
                    "operation": operation, "result": result, "token": token,
                    "flash_valid": bool(flags & 1), "ram_valid": bool(flags & 2), "points": tuple(points),
                    "detection_counts": detection_counts}
        if frame_type == FRAME_TYPE_FORCE_CALIBRATION_PARAMETERS:
            if channel_count != 0 or payload_len < 24:
                return None
            region, force_type, model, point_count, flags = payload[:5]
            if (
                region > 1
                or force_type > 2
                or model != 2
                or point_count < 2
                or point_count > FORCE_CALIBRATION_MAX_POINTS
                or payload_len != 8 + point_count * 8
                or payload[5:8] != b"\x00\x00\x00"
            ):
                return None
            values = struct.unpack_from(f"<{point_count * 2}f", payload, 8)
            return {
                "type": "force_calibration_parameters",
                "device": device,
                "channel_count": channel_count,
                "region": region,
                "force_type": force_type,
                "model": model,
                "point_count": point_count,
                "flags": flags,
                "valid": bool(flags & 0x01),
                "dirty": bool(flags & 0x02),
                "raw_points": tuple(values[0::2]),
                "calibrated_points": tuple(values[1::2]),
            }

        if frame_type == FRAME_TYPE_CAPSENSE_CHANNEL_CONFIG_STATUS:
            entry_size = 14
            expected_len = 4 + channel_count * entry_size
            if (
                channel_count == 0
                or payload_len != expected_len
                or payload[0] != 1
                or payload[1] != entry_size
                or payload[3] != 0
            ):
                return None
            channels = []
            for channel in range(channel_count):
                offset = 4 + channel * entry_size
                (
                    configured_nsub,
                    hardware_nsub,
                    configured_tx_clk_div,
                    hardware_tx_clk_div,
                    max_raw_count,
                    configured_ref_cdac,
                    hardware_ref_cdac,
                    widget_id,
                    sensor_id,
                ) = struct.unpack_from("<HHHHHBBBB", payload, offset)
                channels.append({
                    "channel": channel,
                    "configured_nsub": configured_nsub,
                    "hardware_nsub": hardware_nsub,
                    "configured_tx_clk_div": configured_tx_clk_div,
                    "hardware_tx_clk_div": hardware_tx_clk_div,
                    "max_raw_count": max_raw_count,
                    "configured_ref_cdac": configured_ref_cdac,
                    "hardware_ref_cdac": hardware_ref_cdac,
                    "widget_id": widget_id,
                    "sensor_id": sensor_id,
                })
            return {
                "type": "capsense_channel_config_status",
                "device": device,
                "channel_count": channel_count,
                "format_version": payload[0],
                "read_ok": bool(payload[2] & 0x01),
                "channels": tuple(channels),
            }

        if frame_type == FRAME_TYPE_DEVICE_INFO:
            if channel_count == 0 or payload_len < 4:
                return None
            info_version, finger_type, capability_flags, name_length = payload[:4]
            if finger_type != device or payload_len != 4 + name_length:
                return None
            try:
                device_name = payload[4:].decode("ascii")
            except UnicodeDecodeError:
                return None
            return {
                "type": "device_info",
                "protocol_version": protocol_version,
                "device": device,
                "channel_count": channel_count,
                "info_version": info_version,
                "finger_type": finger_type,
                "capability_flags": capability_flags,
                "matrix_autorange": bool(capability_flags & 0x01),
                "runtime_capsense_config": bool(capability_flags & 0x02),
                "device_name": device_name,
            }

        if frame_type == FRAME_TYPE_SLOW_BASELINE and protocol_version < 20:
            if channel_count not in (5, 8, 20) or payload_len != channel_count * 4:
                return None
            values = struct.unpack("<" + "I" * channel_count, payload)
            if any(value > (65535 << 16) for value in values):
                return None
            return {
                "type": "slow_baseline", "device": device, "channel_count": channel_count,
                "slow_baseline": tuple(value / 65536.0 for value in values),
            }

        if frame_type == FRAME_TYPE_CHANNEL_4567_STATISTICS:
            if channel_count != 4 or payload_len != 12:
                return None
            signal_sum, raw_sum, stddev_x100 = struct.unpack("<III", payload)
            if signal_sum > 262140 or raw_sum > 262140 or stddev_x100 > 3276750:
                return None
            return {
                "type": "channel_4567_statistics",
                "device": device,
                "channel_count": channel_count,
                "signal_sum": signal_sum,
                "hardware_raw_sum": raw_sum,
                "hardware_raw_mean": raw_sum / 4.0,
                "hardware_raw_stddev": stddev_x100 / 100.0,
            }

        if frame_type == FRAME_TYPE_CHANNEL_4567_SIGNAL_SUM:
            if channel_count != 4 or payload_len != 4:
                return None
            return {
                "type": "channel_4567_signal_sum",
                "device": device,
                "channel_count": channel_count,
                "signal_sum": struct.unpack("<I", payload)[0],
            }

        if frame_type == FRAME_TYPE_COMMAND_ACK:
            if channel_count != 0 or payload_len != 1:
                return None
            return {
                "type": "command_ack",
                "device": device,
                "result": payload[0],
            }

        if frame_type == FRAME_TYPE_RUN_FORCE:
            expected_length = 40 if protocol_version >= 19 else (32 if protocol_version >= 18 else 28)
            if channel_count == 0 or payload_len != expected_length:
                return None
            (
                vector_count,
                calibration_mask,
                component_calibration_mask,
                force1_normal,
                force1_tangential,
                force1_direction,
                force2_normal,
                force2_tangential,
                force2_direction,
            ) = struct.unpack_from("<BBH6f", payload)
            return {
                "type": "run_force",
                "device": device,
                "channel_count": channel_count,
                "vector_count": vector_count,
                "calibration_mask": calibration_mask,
                "component_calibration_mask": component_calibration_mask,
                "original_normal_force": force1_normal,
                "optimized_normal_force": struct.unpack_from("<f", payload, 28)[0] if protocol_version >= 18 else None,
                "low_force_simulated": protocol_version >= 19 and bool(component_calibration_mask & 0x200),
                "measured_normal_force": struct.unpack_from("<f", payload, 32)[0] if protocol_version >= 19 else None,
                "measured_optimized_normal_force": struct.unpack_from("<f", payload, 36)[0] if protocol_version >= 19 else None,
                "vectors": (
                    (force1_normal, force1_tangential, force1_direction),
                    (force2_normal, force2_tangential, force2_direction),
                ),
            }

        if frame_type in (FRAME_TYPE_FORCE_VECTOR, FRAME_TYPE_FINGER_PAD_FORCE_VECTOR):
            if channel_count == 0 or payload_len != 14 * BYTES_PER_FLOAT:
                return None
            (
                theta,
                tangential_force,
                confidence,
                normal_force,
                cop_x,
                cop_y,
                diff_x,
                diff_y,
                force_x,
                force_y,
                p00,
                p01,
                p10,
                p11,
            ) = struct.unpack("<14f", payload)
            return {
                "type": "force_vector" if frame_type == FRAME_TYPE_FORCE_VECTOR else "finger_pad_force_vector",
                "device": device,
                "channel_count": channel_count,
                "theta": theta,
                "tangential_force": tangential_force,
                "confidence": confidence,
                "normal_force": normal_force,
                "cop_x": cop_x,
                "cop_y": cop_y,
                "diff_x": diff_x,
                "diff_y": diff_y,
                "force_x": force_x,
                "force_y": force_y,
                "p00": p00,
                "p01": p01,
                "p10": p10,
                "p11": p11,
            }

        if frame_type == FRAME_TYPE_CAPSENSE_NSUB_TIER_STATUS:
            expected_len = 4 + (channel_count * 2)
            if channel_count == 0 or payload_len != expected_len:
                return None
            requested_nsub = struct.unpack_from("<H", payload, 0)[0]
            actual_nsub = struct.unpack_from("<" + ("H" * channel_count), payload, 2)
            return {
                "type": "capsense_nsub_tier_status",
                "device": device,
                "channel_count": channel_count,
                "requested_nsub": requested_nsub,
                "actual_nsub": actual_nsub,
                "pending": bool(payload[-2]),
                "result": payload[-1],
            }

        if frame_type == FRAME_TYPE_CAPSENSE_MATRIX_CONFIG_STATUS:
            if channel_count not in (4, 5, 8, 20) or payload_len != 12:
                return None
            requested_nsub, requested_tx_clk_div = struct.unpack_from("<HH", payload, 0)
            actual_nsub, actual_tx_clk_div = struct.unpack_from("<HH", payload, 5)
            return {
                "type": "capsense_matrix_config_status",
                "device": device,
                "channel_count": channel_count,
                "requested_nsub": requested_nsub,
                "requested_tx_clk_div": requested_tx_clk_div,
                "requested_ref_cdac": payload[4],
                "actual_nsub": actual_nsub,
                "actual_tx_clk_div": actual_tx_clk_div,
                "actual_ref_cdac": payload[9],
                "pending": bool(payload[10]),
                "result": payload[11],
            }

        if frame_type == FRAME_TYPE_TEMPERATURE:
            if channel_count != 0 or payload_len != 2:
                return None
            temperature_tenth_celsius = struct.unpack("<h", payload)[0]
            return {
                "type": "temperature",
                "device": device,
                "channel_count": channel_count,
                "temperature_tenth_celsius": temperature_tenth_celsius,
                "temperature_celsius": temperature_tenth_celsius / 10.0,
            }

        if frame_type == FRAME_TYPE_CAPSENSE_AUTORANGE_STATUS:
            if channel_count != 4 or payload_len != 20:
                return None
            nsub, tx_div, max_count = struct.unpack_from("<HHH", payload, 0)
            peak_raw, filtered_peak_raw = struct.unpack_from("<HH", payload, 12)
            return {
                "type": "capsense_autorange_status",
                "device": device,
                "channel_count": channel_count,
                "nsub": nsub,
                "tx_div": tx_div,
                "max_count": max_count,
                "actual_ref_cdac": payload[6],
                "tier_index": payload[7],
                "tier_count": payload[8],
                "state": payload[9],
                "flags": payload[10],
                "affine_active": bool(payload[10] & 0x80),
                "requested_ref_cdac": payload[11],
                "peak_raw": peak_raw,
                "filtered_peak_raw": filtered_peak_raw,
                "high_count": payload[16],
                "low_count": payload[17],
                "cooldown_frames": payload[18],
                "blank_frames": payload[19],
            }

        if frame_type == FRAME_TYPE_CAPSENSE_AFFINE_CALIBRATION_STATUS:
            if channel_count != 4 or payload_len != 72:
                return None
            coefficients = struct.unpack_from("<14i", payload, 16)
            return {
                "type": "capsense_affine_calibration_status",
                "device": device,
                "channel_count": channel_count,
                "format_version": payload[0],
                "channel": payload[1],
                "tier_count": payload[2],
                "flags": payload[3],
                "calibration_id": struct.unpack_from("<I", payload, 4)[0],
                "generation": struct.unpack_from("<I", payload, 8)[0],
                "nvm_status": struct.unpack_from("<I", payload, 12)[0],
                "coefficients": tuple(
                    (coefficients[index], coefficients[index + 1])
                    for index in range(0, len(coefficients), 2)
                ),
            }

        if frame_type == FRAME_TYPE_CAPSENSE_AFFINE_OPERATION_STATUS:
            if channel_count != 0 or payload_len != 8:
                return None
            return {
                "type": "capsense_affine_operation_status",
                "device": device,
                "command_type": payload[0],
                "result": payload[1],
                "channel": payload[2],
                "calibration_id": struct.unpack_from("<I", payload, 4)[0],
            }

        if frame_type == FRAME_TYPE_RAW:
            if protocol_version < 21 or channel_count == 0 or payload_len != channel_count * BYTES_PER_VALUE:
                return None
            raw = struct.unpack(f"<{channel_count}H", payload)
            # 复用现有监控采样链路；未上传状态用-1表示未知，不能伪造IDLE。
            return {
                "type": "debug", "device": device, "channel_count": channel_count,
                "raw_only": True, "raw": raw,
                "baseline": (0,) * channel_count, "signal": (0,) * channel_count,
                "fast_k_percent": (float("nan"),) * channel_count,
                "slow_k_percent": (float("nan"),) * channel_count,
                "state": (-1,) * channel_count,
            }

        if frame_type == FRAME_TYPE_DEBUG:
            return self._parse_debug_payload(device, channel_count, payload_len, payload, protocol_version)

        expected_values = self._expected_value_count(frame_type, channel_count)
        if expected_values is None or payload_len != expected_values * BYTES_PER_VALUE:
            return None

        values = struct.unpack("<" + ("H" * expected_values), payload)
        if frame_type == FRAME_TYPE_SIGNAL:
            return {
                "type": "signal",
                "device": device,
                "channel_count": channel_count,
                "signal": values[:channel_count],
            }

    def _parse_debug_payload(self, device, channel_count, payload_len, payload, protocol_version):
        expected_payload_len = channel_count * (16 if protocol_version >= 20 else 26)
        if channel_count == 0 or payload_len != expected_payload_len:
            return None

        offset = 0

        def read_u16_array():
            nonlocal offset
            values = struct.unpack_from("<" + ("H" * channel_count), payload, offset)
            offset += channel_count * BYTES_PER_VALUE
            return values

        def read_i32_array(divisor):
            nonlocal offset
            values = struct.unpack_from("<" + ("i" * channel_count), payload, offset)
            offset += channel_count * 4
            return tuple(value / divisor for value in values)

        raw = read_u16_array()
        # V14～19只跳过旧版两组Norm Raw；对外统一提供真实Raw，不伪造新字段。
        if protocol_version < 20:
            offset += channel_count * 8
        baseline = read_u16_array()
        signal = read_u16_array()
        fast_k_percent = read_i32_array(100.0)
        slow_k_percent = read_i32_array(100.0)
        if protocol_version < 20:
            read_u16_array()  # 旧版固定Noise不再进入当前数据链路。
        state = read_u16_array()

        return {
            "type": "debug",
            "device": device,
            "channel_count": channel_count,
            "raw": raw,
            "baseline": baseline,
            "signal": signal,
            "fast_k_percent": fast_k_percent,
            "slow_k_percent": slow_k_percent,
            "state": state,
        }

    def _expected_value_count(self, frame_type, channel_count):
        if channel_count == 0:
            return None
        if frame_type == FRAME_TYPE_SIGNAL:
            return channel_count
        return None
