"""按厂家报文示例验证新电机协议，并用模拟电机验证后台轮询与UI恢复。"""

from collections import deque
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import threading
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "PY_SRIPT/algo_monitor"))

import can
from motor_connection_config import load_motor_connection_settings
from qt_compat import QtWidgets
from zdt_motor_test_widget import ZDTMotorTestWidget
from zdt_x42s_can_driver import (
    ZDTConfig, ZDTProtocolError, ZDTTimeoutError, ZDTX42SMotor, parse_system_status,
    degrees_to_pulses,
)

# 手册第96和98页完整示例，保持原样作为独立协议基准。
X_PACKET = bytes.fromhex("01 43 25 0C 5C 67 00 00 00 03 30 B5 43 EB 01 00 00 0E 10 00 00 00 00 00 00 0E 10 01 00 00 00 02 00 22 03 03 6B")
EMM_PACKET = bytes.fromhex("01 43 1F 09 5C 67 00 03 43 EB 01 00 01 00 00 00 00 00 00 00 01 00 00 01 00 00 00 08 03 03 6B")
HOME_PACKET = bytes.fromhex("01 22 00 00 00 1E 00 00 27 10 01 2C 03 20 00 3C 00 6B")
HOME_PARAMETERS = dict(mode=0, direction=0, speed_rpm=30, timeout_ms=10000,
                       detect_speed_rpm=300, detect_current_ma=800, detect_time_ms=60,
                       power_on_homing=False, backoff_raw=20)


def frames(packet):
    return [can.Message(arbitration_id=(packet[0] << 8) | n, is_extended_id=True,
                        data=bytes((packet[1],)) + packet[2:][start:start + 7])
            for n, start in enumerate(range(0, len(packet) - 2, 7))]


class FakeBus:
    def __init__(self, responder):
        self.responder = responder
        self.pending = deque()
        self.sent = []
        self.closed = False

    def send(self, message, timeout=None):
        self.sent.append(message)
        self.pending.extend(self.responder(message))

    def recv(self, timeout=0):
        if self.pending:
            return self.pending.popleft()
        if timeout:
            time.sleep(timeout)
        return None

    def shutdown(self):
        self.closed = True


def driver(responder, **kwargs):
    bus = FakeBus(responder)
    motor = ZDTX42SMotor(ZDTConfig(timeout=0.01, **kwargs), bus_factory=lambda **_: bus)
    motor.open()
    return motor, bus


def scaled_driver():
    """按手册模拟Emm输入缩放；反馈仍用RPM，记录电机实际解释的速度。"""
    device = dict(scale=1, status=EMM_PACKET, config_reply=True, motion_reply=True, motions=[])
    partial = bytearray()
    def respond(msg):
        code, part = msg.data[0], msg.arbitration_id & 0xFF
        if code == 0x43: return frames(device['status'])
        if code == 0x39: return frames(bytes.fromhex('01 39 00 19 6B'))
        if code == 0x4F:
            assert bytes(msg.data[1:3]) == b'\x71\x00'
            device['scale'] = 10 if msg.data[3] else 1
            return frames(bytes.fromhex('01 4F 02 6B')) if device['config_reply'] else []
        if code == 0xFD:
            if part == 0: partial.clear()
            partial.extend(msg.data[1:])
            if part == 0: return []
            word = int.from_bytes(partial[1:3], 'big')
            device['motions'].append(dict(rpm=word/device['scale'], word=word, mode=partial[8]))
            return frames(bytes.fromhex('01 FD 02 6B')) if device['motion_reply'] else []
        return frames(bytes((1, code, 2, 0x6B)))
    motor, bus = driver(respond)
    motor.read_status()
    bus.sent.clear()
    return motor, bus, device


class ProtocolTests(unittest.TestCase):
    def test_temperature_throttled_but_position_and_flags_remain_fresh(self):
        temperature = bytes.fromhex('01 39 00 19 6B')
        motor, bus = driver(lambda msg: frames(EMM_PACKET if msg.data[0] == 0x43 else temperature))
        self.addCleanup(motor.close)
        first = motor.read_status()
        for _ in range(5):
            current = motor.read_status()
            self.assertEqual(current['temperature_sampled_at'], first['temperature_sampled_at'])
            self.assertGreaterEqual(current['sampled_at'], first['sampled_at'])
        self.assertEqual(sum(m.data[0] == 0x43 for m in bus.sent), 6)
        self.assertEqual(sum(m.data[0] == 0x39 for m in bus.sent), 1)
        motor._temperature_cache['temperature_sampled_at'] -= 2.1
        motor.read_status()
        self.assertEqual(sum(m.data[0] == 0x39 for m in bus.sent), 2)

    def test_sensorless_homing_parameters_command_and_abort(self):
        def respond(msg):
            if msg.data[0] == 0x22:
                return frames(HOME_PACKET)
            if msg.data[0] == 0x3F:
                return frames(bytes.fromhex("01 3F 00 14 6B"))
            return frames(bytes((1, msg.data[0], 2, 0x6B)))
        motor, bus = driver(respond)
        try:
            self.assertEqual(motor.read_homing_parameters(), HOME_PARAMETERS)
            self.assertEqual([bytes(m.data) for m in bus.sent], [b"\x22\x6b", b"\x3f\x6b"])
            motor._firmware = "Emm"
            motor.home_sensorless()
            motor.abort_homing()
            self.assertEqual([(m.arbitration_id, bytes(m.data)) for m in bus.sent[-2:]],
                             [(0x100, bytes.fromhex("9A 02 00 6B")),
                              (0x100, bytes.fromhex("9C 48 6B"))])
        finally:
            motor.close()
        motor, bus = driver(lambda msg: [])
        motor._firmware = "Emm"
        try:
            with self.assertRaises(ZDTTimeoutError):
                motor.home_sensorless()
            self.assertEqual(len(bus.sent), 1)
        finally:
            motor.close()

    def test_emm_relative_move_can_frames_direction_and_stop(self):
        def respond(msg):
            if msg.data[0] == 0x43:
                return frames(EMM_PACKET)
            if msg.data[0] == 0x39:
                return frames(bytes.fromhex("01 39 00 19 6B"))
            if msg.data[0] == 0xFD and msg.arbitration_id & 0xFF == 0:
                return []
            return frames(bytes((1, msg.data[0], 2, 0x6B)))
        motor, bus = driver(respond)
        try:
            motor.read_status()
            bus.sent.clear()
            self.assertEqual(motor.move_relative_pulses(801, 10, 100, True), 801)
            self.assertEqual([(m.arbitration_id, bytes(m.data)) for m in bus.sent], [
                (0x100, bytes.fromhex("43 7A 6B")),
                (0x100, bytes.fromhex("4F 71 00 01 6B")),
                (0x100, bytes.fromhex("FD 01 00 64 64 00 00 03")),
                (0x101, bytes.fromhex("FD 21 02 00 6B")),
            ])
            bus.sent.clear()
            motor.move_relative_pulses(801, 10, 100, False)
            self.assertEqual(bytes(bus.sent[1].data), bytes.fromhex("FD 00 00 64 64 00 00 03"))
            motor.stop_motion()
            self.assertEqual(bytes(bus.sent[-1].data), bytes.fromhex("FE 98 00 6B"))
        finally:
            motor.close()

    def test_motion_validation_quantization_and_no_retry(self):
        self.assertEqual(degrees_to_pulses(5, 3200), 44)
        self.assertEqual(degrees_to_pulses(360, 6400), 6400)
        for angle in (0, -1, float("nan"), float("inf"), 0.01):
            with self.assertRaises(ValueError):
                degrees_to_pulses(angle, 3200)
        motor, bus = driver(lambda msg: [])
        try:
            with self.assertRaisesRegex(Exception, "Emm"):
                motor.move_relative_degrees(5, 10, 100, 3200, True)
            motor._firmware = "Emm"
            for pulses in (0, -1, 1.5, True, "16", 0x100000000):
                with self.assertRaises(ValueError):
                    motor.move_relative_pulses(pulses, 10, 100, True)
            for speed, acc in ((0, 100), (3001, 100), (10, -1), (10, 256)):
                with self.assertRaises(ValueError):
                    motor.move_relative_degrees(5, speed, acc, 3200, True)
            self.assertEqual(bus.sent, [])
            with self.assertRaises(ZDTTimeoutError):
                motor.move_relative_degrees(5, 10, 100, 3200, True)
            self.assertEqual([m.data[0] for m in bus.sent], [0x43])
        finally:
            motor.close()

    def test_fractional_and_integer_motion_share_real_rpm_and_feedback_is_not_scaled(self):
        motor, bus, device = scaled_driver()
        self.addCleanup(motor.close)
        for rpm in (.1, 3.5, 3.6, 24, 300, 400, 3.7):
            motor.move_relative_pulses(100, rpm, 100, True)
        self.assertEqual([r['rpm'] for r in device['motions']], [.1, 3.5, 3.6, 24, 300, 400, 3.7])
        self.assertEqual([r['word'] for r in device['motions']], [1, 35, 36, 240, 3000, 400, 37])
        packet = bytearray(EMM_PACKET)
        packet[15:18] = bytes.fromhex('01 00 03')
        device['status'] = bytes(packet)
        self.assertEqual(motor.read_status()['speed_rpm'], -3)
        # 单位不变时允许在线更新，不插入停止命令；读回速度不除以10。
        bus.sent.clear()
        motor.move_absolute_pulses(-800, 3.8, 85)
        self.assertEqual([m.data[0] for m in bus.sent], [0x4F, 0xFD, 0xFD])
        self.assertEqual(device['motions'][-1]['rpm'], 3.8)
        with self.assertRaisesRegex(Exception, '须先停止'):
            motor.move_absolute_pulses(-800, 400, 85)

    def test_scaling_ack_loss_blocks_motion_then_recovers_and_reasserts_after_device_reset(self):
        motor, bus, device = scaled_driver()
        self.addCleanup(motor.close)
        device['config_reply'] = False
        with self.assertRaises(ZDTTimeoutError): motor.move_absolute_pulses(-800, 3.5, 85)
        self.assertEqual(device['motions'], [])
        self.assertIsNone(motor._emm_speed_scale)
        self.assertEqual(sum(m.data[0] == 0x4F for m in bus.sent), 1)
        device['config_reply'] = True
        motor.stop_motion()
        motor.move_absolute_pulses(-800, 3.5, 85)
        device['scale'] = 1
        motor.move_absolute_pulses(-800, 3.6, 85)
        self.assertEqual([r['rpm'] for r in device['motions']], [3.5, 3.6])
        motor.close()
        self.assertIsNone(motor._emm_speed_scale)

    def test_fractional_speed_precision_and_motion_ack_are_never_rounded_or_retried(self):
        motor, bus, device = scaled_driver()
        self.addCleanup(motor.close)
        for rpm in (True, '3.5', float('nan'), float('inf'), .05, 3.55, 300.1, 3001):
            with self.assertRaises(ValueError): motor.move_absolute_pulses(-800, rpm, 85)
        self.assertEqual(bus.sent, [])
        device['motion_reply'] = False
        with self.assertRaises(ZDTTimeoutError): motor.move_absolute_pulses(-800, 3.5, 85)
        self.assertEqual(len(device['motions']), 1)
        self.assertIsNone(motor._emm_speed_scale)

    def test_candlelight_zero_channel_open_and_pcan_compatibility(self):
        for config in (ZDTConfig(), ZDTConfig(interface="pcan", channel="PCAN_USBBUS1")):
            with self.subTest(interface=config.interface), \
                    patch("zdt_x42s_can_driver.prepare_gs_usb_backend") as backend, \
                    patch("gs_usb_transport.open_gs_usb_bus" if config.interface == "gs_usb"
                          else "zdt_x42s_can_driver.can.Bus") as factory:
                motor = ZDTX42SMotor(config)
                motor.open()
                factory.assert_called_once_with(interface=config.interface, channel=config.channel,
                                                bitrate=500000, receive_own_messages=False)
                self.assertEqual(backend.call_count, int(config.interface == "gs_usb"))
                motor.close()
                factory.return_value.shutdown.assert_called_once()
        for channel in ("0", "PCAN_USBBUS1", -1, None, True):
            with self.subTest(channel=channel), self.assertRaises(ValueError):
                ZDTConfig(channel=channel)

    def test_manual_status_examples(self):
        for packet, firmware, error in ((X_PACKET, "X", -0.02), (EMM_PACKET, "Emm", -8 * 360 / 65536)):
            with self.subTest(firmware=firmware):
                result = parse_system_status(packet)
                self.assertEqual(result["firmware"], firmware)
                self.assertEqual(result["phase_current_ma"], 3)
                self.assertEqual(result["voltage_mv"], 23655)
                self.assertEqual(result["position_deg"], 360)
                self.assertEqual(result["target_deg"], -360)
                self.assertAlmostEqual(result["error_deg"], error)
                self.assertTrue(result["enabled"])
                self.assertTrue(result["calibration_ready"])
                self.assertFalse(result["stall"])
        self.assertEqual(parse_system_status(X_PACKET)["temperature_c"], 34)
        self.assertIsNone(parse_system_status(EMM_PACKET)["bus_current_ma"])

    def test_negative_position_speed_and_protection_bits(self):
        packet = bytearray(X_PACKET)
        packet[19:22] = bytes.fromhex("01 04 D2")
        packet[22] = 1
        packet[34] = 0x30
        packet[35] = 0x0C
        result = parse_system_status(packet)
        self.assertEqual(result["position_deg"], -360)
        self.assertAlmostEqual(result["speed_rpm"], -123.4)
        self.assertTrue(result["overtemperature"])
        self.assertTrue(result["overcurrent"])
        self.assertTrue(result["stall_protection"])
        self.assertFalse(result["enabled"])

    def test_can_reassembly_and_foreign_frames(self):
        noise = [can.Message(arbitration_id=0x200, data=b"\x43\x00", is_extended_id=True),
                 can.Message(arbitration_id=0x100, data=b"\x43\x00", is_extended_id=False),
                 can.Message(arbitration_id=0x100, data=b"\x43\x7a\x6b", is_extended_id=True, is_rx=False)]
        motor, bus = driver(lambda message: noise + frames(X_PACKET))
        self.assertEqual(motor.read_status()["position_deg"], 360)
        self.assertEqual(bytes(bus.sent[0].data), bytes.fromhex("43 7A 6B"))
        self.assertEqual(bus.sent[0].arbitration_id, 0x100)
        self.assertTrue(bus.sent[0].is_extended_id)
        motor.close()
        self.assertTrue(bus.closed)

    def test_emm_temperature_and_override(self):
        for positive_sign, expected in ((1, -34), (0, 34)):
            motor, bus = driver(lambda msg: frames(EMM_PACKET if msg.data[0] == 0x43 else bytes.fromhex("01 39 00 22 6B")), temperature_positive_sign=positive_sign)
            self.assertEqual(motor.read_status()["temperature_c"], expected)
            self.assertEqual([bytes(m.data) for m in bus.sent], [b"\x43\x7a\x6b", b"\x39\x6b"])
            motor.close()

    def test_emm_temperature_default_preserves_signed_values(self):
        for raw, expected in (("00 19", 25), ("01 19", -25), ("00 00", 0)):
            with self.subTest(raw=raw):
                temperature_packet = bytes.fromhex("01 39 " + raw + " 6B")
                motor, _ = driver(lambda msg: frames(
                    EMM_PACKET if msg.data[0] == 0x43 else temperature_packet))
                try:
                    status = motor.read_status()
                    self.assertEqual(status["temperature_c"], expected)
                    self.assertEqual(status["temperature_raw"], raw)
                finally:
                    motor.close()

    def test_write_command_bytes_and_ack(self):
        motor, bus = driver(lambda msg: frames(bytes((1, msg.data[0], 2, 0x6B))))
        motor.set_enabled(True)
        motor.set_enabled(False)
        motor.calibrate_encoder()
        self.assertEqual([bytes(m.data) for m in bus.sent], [bytes.fromhex(s) for s in ("F3 AB 01 00 6B", "F3 AB 00 00 6B", "06 45 6B")])
        motor.close()

    def test_reject_missing_reordered_corrupt_frames(self):
        bad_tail = bytearray(X_PACKET)
        bad_tail[-1] = 0
        duplicate = frames(X_PACKET)
        duplicate.insert(1, duplicate[0])
        for replies in (frames(bytes.fromhex("01 43 E2 6B")), frames(bad_tail),
                        frames(X_PACKET)[1:], duplicate):
            motor, _ = driver(lambda msg: replies)
            with self.assertRaises(ZDTProtocolError):
                motor.read_status()
            motor.close()
        motor, _ = driver(lambda msg: frames(X_PACKET)[:-1])
        with self.assertRaises(ZDTTimeoutError):
            motor.read_status()
        motor.close()

    def test_calibration_timeout_does_not_retry(self):
        motor, bus = driver(lambda msg: [])
        with self.assertRaises(ZDTTimeoutError):
            motor.calibrate_encoder()
        self.assertEqual(len(bus.sent), 1)
        motor.close()

    def test_broadcast_blocked_and_configuration_defaults(self):
        with self.assertRaises(ValueError):
            ZDTConfig(node_id=0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor_connection_config.json"
            model, config = load_motor_connection_settings(path)
            self.assertEqual((model, config.bitrate, config.node_id), ("zdt_x42s", 500000, 1))
            self.assertEqual((config.interface, config.channel), ("gs_usb", 0))
            path.write_text(json.dumps({"motor_model": "pd42s1"}), encoding="utf-8")
            self.assertEqual(load_motor_connection_settings(path)[0], "pd42s1")
            path.write_text('{"motor_model":"typo"}', encoding="utf-8")
            with self.assertRaises(ValueError):
                load_motor_connection_settings(path)


class SimulatedMotor:
    def __init__(self, config):
        self.read_times = []
        self.actions = []
        self.failed = False
        self.closed = False
        self.packet = X_PACKET
        self.overrides = {}
        self.home_parameters = dict(HOME_PARAMETERS)

    def open(self):
        pass

    def read_status(self):
        self.read_times.append(time.monotonic())
        if self.failed:
            raise ZDTTimeoutError("模拟失联")
        snapshot = parse_system_status(self.packet)
        if snapshot["firmware"] == "Emm":
            snapshot.update(temperature_c=25, temperature_raw="00 19")
        snapshot.update(self.overrides)
        snapshot["sampled_at"] = time.monotonic()
        return snapshot

    def set_enabled(self, enabled):
        self.actions.append(("enable", enabled))

    def calibrate_encoder(self):
        self.actions.append(("calibrate",))

    def close(self):
        self.closed = True

    def move_relative_pulses(self, **parameters):
        self.actions.append(("move", parameters))
        return parameters["pulses"]

    def stop_motion(self):
        self.actions.append(("stop",))

    def read_homing_parameters(self):
        return dict(self.home_parameters)

    def home_sensorless(self):
        self.actions.append(("home",))
        self.overrides.update(home_flags=7, homing=True)

    def abort_homing(self):
        self.actions.append(("abort_home",))
        self.overrides.update(home_flags=3, homing=False)


class WidgetTests(unittest.TestCase):
    def test_explicit_stop_returns_to_idle_even_when_original_target_was_not_reached(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        motor.overrides.update(speed_rpm=10, in_position=False)
        def stop():
            motor.actions.append(('stop',))
            motor.overrides['speed_rpm'] = 0
        motor.stop_motion = stop
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        try:
            widget.connect_can()
            self.pump(.15)
            self.assertTrue(widget._motion_polling)
            widget.stop_button.click()
            self.pump(.7)
            self.assertFalse(widget._motion_polling)
            count = len(motor.read_times)
            self.pump(.3)
            self.assertEqual(len(motor.read_times), count)
        finally:
            widget.shutdown()
            widget.close()

    def test_motion_switches_to_fast_queries_then_returns_to_two_second_idle(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        try:
            widget.connect_can()
            self.pump(1.0)
            self.assertEqual(len(motor.read_times), 1)
            self.assertTrue(widget.press_button.isEnabled())
            before = len(motor.read_times)
            widget.press_button.click()
            self.pump(.7)
            self.assertGreaterEqual(len(motor.read_times)-before, 8)
            self.assertFalse(widget._motion_polling)
            count = len(motor.read_times)
            self.pump(.7)
            self.assertEqual(len(motor.read_times), count)
        finally:
            widget.shutdown()
            widget.close()

    def test_cached_idle_state_does_not_bypass_fresh_preflight_protection(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        try:
            widget.connect_can()
            self.pump(.8)
            self.assertTrue(widget.press_button.isEnabled())
            motor.overrides['overcurrent'] = True
            widget.press_button.click()
            self.pump(.15)
            self.assertEqual(motor.actions, [])
            self.assertIn('保护', widget.command_label.text())
        finally:
            widget.shutdown()
            widget.close()

    def test_external_loop_exclusively_owns_one_worker_and_idle_queries_do_not_interleave(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        threads = []
        original = motor.read_status
        def read():
            threads.append(threading.get_ident())
            return original()
        motor.read_status = read
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        samples = []
        def operation(device, cancel):
            for _ in range(3):
                widget.status_received.emit(device.read_status())
                samples.append(True)
            entered.set()
            release.wait(2)
            finished.set()
        try:
            widget.connect_can()
            self.pump(.8)
            self.assertTrue(widget.start_external_operation(operation, 'test')[0])
            self.assertTrue(entered.wait(1))
            count = len(motor.read_times)
            self.pump(.3)
            self.assertEqual(len(motor.read_times), count)
            self.assertTrue(widget._external_polling)
            self.assertEqual(set(threads), {widget._thread.ident})
            release.set()
            self.pump(.2)
            self.assertTrue(finished.is_set())
            self.assertFalse(widget._external_polling)
        finally:
            release.set()
            widget.shutdown()
            widget.close()

    def test_idle_and_motion_freshness_limits_are_separate(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        try:
            snapshot = motor.read_status()
            snapshot['sampled_at'] -= 1.0
            widget._last_sample = snapshot
            self.assertTrue(widget._is_fresh())
            self.assertEqual(widget._motion_block_reason(snapshot), '状态无效或已过期')
            widget._external_polling = True
            self.assertFalse(widget._is_fresh())
            widget._external_polling = False
            widget._motion_polling = True
            self.assertFalse(widget._is_fresh())
        finally:
            widget.shutdown()
            widget.close()

    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.app.processEvents()
            time.sleep(0.005)

    def test_failed_connect_then_disconnect_and_reconnect(self):
        motors = []

        def factory(config):
            if motors:
                self.assertTrue(motors[-1].closed)
            motor = SimulatedMotor(config)
            motor.failed = not motors
            motors.append(motor)
            return motor

        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=factory)
        try:
            widget.connect_can()
            self.pump(0.15)
            self.assertFalse(widget._connected)
            self.assertTrue(motors[0].closed)
            for _ in range(3):
                widget.connect_can()
                self.pump(0.15)
                self.assertTrue(widget._connected)
                self.assertFalse(motors[-1].closed)
                widget._toggle_connection()
                self.pump(0.15)
                self.assertFalse(widget._connected)
                self.assertTrue(motors[-1].closed)
            self.assertEqual(len(motors), 4)
            self.assertTrue(all(not motor.actions for motor in motors))
        finally:
            widget.shutdown()
            widget.close()

    def test_homing_lifecycle_and_abort_without_software_zero(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        try:
            widget.connect_can()
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "idle")
            self.assertFalse(widget.home_button.isEnabled())
            widget.read_home_button.click()
            self.pump(0.1)
            self.assertIn("CW", widget.home_parameters_label.text())
            self.assertIn("2°", widget.home_parameters_label.text())
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "running")
            self.assertFalse(widget.press_button.isEnabled())
            self.assertFalse(widget.home_button.isEnabled())
            self.assertEqual(motor.actions, [("home",)])
            motor.overrides.update(home_flags=3, homing=False, position_deg=2.125)
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "completed")
            self.assertEqual(widget.value_labels["position_deg"].text(), "2.125")
            self.assertIn("2.125", widget.home_status_label.text())
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.pump(0.15)
            widget.stop_button.click()
            self.pump(0.15)
            self.assertEqual(motor.actions[-2:], [("abort_home",), ("stop",)])
            self.assertEqual(widget._home_phase, "interrupted")
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.pump(0.15)
            widget.abort_home_button.click()
            self.pump(0.15)
            self.assertEqual(motor.actions[-1], ("abort_home",))
            self.assertEqual(widget._home_phase, "interrupted")
        finally:
            widget.shutdown()
            widget.close()

    def test_homing_parameter_change_failure_and_missing_active_state(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        try:
            widget.connect_can()
            self.pump(0.15)
            widget.read_home_button.click()
            self.pump(0.1)
            motor.home_parameters["direction"] = 1
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.pump(0.15)
            self.assertEqual(motor.actions, [])
            self.assertEqual(widget._home_phase, "rejected")
            self.assertFalse(widget.home_confirm_checkbox.isChecked())
            self.assertIn("CCW", widget.home_parameters_label.text())
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.pump(0.15)
            motor.overrides.update(home_flags=11, homing=False, home_failed=True)
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "failed")
            # 没观察到正在回零时，空闲标志不能当成本次成功。
            motor.overrides.update(home_flags=3, home_failed=False)
            def timeout_home():
                motor.actions.append(("home_timeout",))
                raise ZDTTimeoutError("模拟回零应答丢失")
            motor.home_sensorless = timeout_home
            self.pump(0.15)
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.pump(0.25)
            self.assertEqual(widget._home_phase, "uncertain")
            self.assertFalse(widget.press_button.isEnabled())
            self.assertEqual(motor.actions.count(("home_timeout",)), 1)
            widget.abort_home_button.click()
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "interrupted")
        finally:
            widget.shutdown()
            widget.close()

    def test_homing_cancel_preflight_and_communication_gap(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        entered, release = threading.Event(), threading.Event()
        try:
            widget.connect_can()
            self.pump(0.15)
            widget.read_home_button.click()
            self.pump(0.1)
            original = motor.read_homing_parameters
            def blocked_read():
                entered.set()
                release.wait(1)
                return original()
            motor.read_homing_parameters = blocked_read
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.assertTrue(entered.wait(0.5))
            widget.stop_button.click()
            release.set()
            self.pump(0.2)
            self.assertNotIn(("home",), motor.actions)
            self.assertEqual(motor.actions[-1], ("stop",))
            motor.read_homing_parameters = original
            widget.home_confirm_checkbox.setChecked(True)
            widget.home_button.click()
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "running")
            motor.failed = True
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "uncertain")
            motor.failed = False
            motor.overrides.update(home_flags=3, homing=False)
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "uncertain")
            def failed_abort():
                raise ZDTTimeoutError("模拟中断应答丢失")
            motor.abort_homing = failed_abort
            widget.stop_button.click()
            self.pump(0.15)
            self.assertEqual(widget._home_phase, "uncertain")
            self.assertEqual(motor.actions[-1], ("stop",))
            self.assertFalse(widget.press_button.isEnabled())
        finally:
            release.set()
            widget.shutdown()
            widget.close()

    def test_motion_inputs_directions_and_status_guards(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        # 本用例只检查状态互锁；真实2秒空闲节奏由独立用例验证。
        widget.POLL_INTERVAL = .1
        try:
            self.assertFalse(widget.press_button.isEnabled())
            widget.connect_can()
            self.pump(0.15)
            self.assertTrue(widget.press_button.isEnabled())
            widget.pulses_input.setValue(801)
            widget.speed_input.setValue(12)
            widget.acceleration_input.setValue(50)
            self.assertIn("801", widget.motion_preview_label.text())
            self.assertIn("90.1125", widget.motion_preview_label.text())
            widget.pulses_per_turn_input.setValue(6400)
            self.assertIn("45.0562", widget.motion_preview_label.text())
            widget.press_button.click()
            self.pump(0.15)
            widget.lift_button.click()
            self.pump(0.15)
            moves = [a[1] for a in motor.actions if a[0] == "move"]
            self.assertEqual([a["ccw"] for a in moves], [True, False])
            self.assertEqual(moves[0], dict(pulses=801, speed_rpm=12, acceleration=50, ccw=True))
            self.assertEqual(moves[1]["pulses"], 801)
            for changes in ({"enabled": False}, {"speed_rpm": 1}, {"stall": True},
                            {"calibration_ready": False}, {"firmware": "X"}):
                motor.overrides = changes
                self.pump(0.15)
                self.assertFalse(widget.press_button.isEnabled(), changes)
                self.assertTrue(widget.stop_button.isEnabled())
            # UI旧状态仍正常时，后台发令前也必须重新读取并拦截失能。
            motor.overrides = {}
            self.pump(0.15)
            self.assertTrue(widget.press_button.isEnabled())
            motor.overrides = {"enabled": False}
            widget.press_button.click()
            self.pump(0.15)
            self.assertEqual(len([a for a in motor.actions if a[0] == "move"]), 2)
            self.assertIn("先使能", widget.command_label.text())
            motor.failed = True
            self.pump(0.15)
            self.assertFalse(widget.press_button.isEnabled())
            widget.stop_button.click()
            self.pump(0.15)
            self.assertEqual(motor.actions[-1], ("stop",))
        finally:
            widget.shutdown()
            widget.close()

    def test_stop_cancels_move_waiting_for_preflight(self):
        motor = SimulatedMotor(ZDTConfig())
        motor.packet = EMM_PACKET
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        entered, release = threading.Event(), threading.Event()
        try:
            widget.connect_can()
            self.pump(0.15)
            original = motor.read_status
            def blocked_read():
                entered.set()
                release.wait(1)
                return original()
            motor.read_status = blocked_read
            widget.press_button.click()
            self.assertTrue(entered.wait(1))
            self.assertTrue(widget.stop_button.isEnabled())
            widget.stop_button.click()
            release.set()
            self.pump(0.2)
            self.assertEqual(motor.actions, [("stop",)])
            self.assertFalse(widget._busy)
        finally:
            release.set()
            widget.shutdown()
            widget.close()

    def test_polling_buttons_failure_recovery_and_shutdown(self):
        motor = SimulatedMotor(ZDTConfig())
        widget = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
        try:
            widget.connect_can()
            self.pump(1.2)
            self.assertTrue(widget._connected)
            self.assertEqual(motor.actions, [])
            self.assertEqual(len(motor.read_times), 1)
            self.assertTrue(widget.enable_button.isEnabled())
            self.assertIn('空闲每2秒', widget.freshness_label.text())
            self.pump(1.0)
            self.assertEqual(len(motor.read_times), 2)
            self.assertGreaterEqual(motor.read_times[1]-motor.read_times[0], 1.95)
            self.assertFalse(widget.calibrate_button.isEnabled())
            widget.unloaded_checkbox.setChecked(True)
            widget.calibrate_button.click()
            self.pump(0.1)
            self.assertEqual(motor.actions, [("calibrate",)])
            self.assertFalse(widget.unloaded_checkbox.isChecked())
            widget.enable_button.click()
            self.pump(0.1)
            widget.disable_button.click()
            self.pump(0.1)
            self.assertEqual(motor.actions[-2:], [("enable", True), ("enable", False)])
            widget._begin_motion_polling()
            motor.failed = True
            self.pump(2.1)
            self.assertFalse(widget.enable_button.isEnabled())
            self.assertFalse(widget.value_labels["position_deg"].isEnabled())
            self.assertTrue(widget.disable_button.isEnabled())
            motor.failed = False
            self.pump(2.1)
            self.assertTrue(widget.enable_button.isEnabled())
            self.assertEqual(widget.value_labels["temperature_c"].text(), "34")
            widget._last_sample["sampled_at"] -= 3
            widget._refresh_freshness()
            self.assertIn("数据已过期", widget.freshness_label.text())
            self.assertFalse(widget.enable_button.isEnabled())
            widget._toggle_connection()
            self.pump(0.1)
            self.assertFalse(widget._connected)
            self.assertEqual(widget.value_labels["position_deg"].text(), "—")
            self.assertTrue(motor.closed)
        finally:
            widget.shutdown()
            widget.close()
        self.assertFalse(widget._thread.is_alive())


if __name__ == "__main__":
    unittest.main(verbosity=2)
