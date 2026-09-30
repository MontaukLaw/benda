"""回归Signal-only帧的缺失字段、CSV空值、界面状态及协议版本同步。"""
import math
import os
from pathlib import Path
import struct
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "PY_SRIPT/algo_monitor"))
from config import OUTPUT_MODE_DEBUG, OUTPUT_MODE_RUN
from frame_parser import AlgoFrameParser
from data_buffer import AlgoDataBuffer
from force_press_control_widget import ForcePressControlWidget
from main_window import MainWindow
from serial_reader import SerialReaderThread
from qt_compat import QtWidgets


def packet(kind, payload, version=15, channels=8):
    return struct.pack("<4sBBBBH", b"DATA", version, kind, 1, channels, len(payload)) + payload


def signal_packet(version=15):
    return packet(1, struct.pack("<8H", 0, 0, 0, 0, 6157, 9411, 5273, 7244), version)


def debug_packet():
    payload = struct.pack("<8H", *range(1000, 1008))
    payload += struct.pack("<8I", *range(2000, 2008))
    payload += struct.pack("<8I", *range(3000, 3008))
    payload += struct.pack("<8H", *([900] * 8))
    payload += struct.pack("<8H", *([100] * 8))
    payload += struct.pack("<8i", *([0] * 8)) * 2
    payload += struct.pack("<8H", *([10] * 8))
    payload += struct.pack("<8H", *([6] * 8))
    return packet(0x10, payload)


class MissingDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.parser = AlgoFrameParser()
        self.buffer = AlgoDataBuffer(sample_count=5)

    def feed(self, data):
        for frame in self.parser.feed(data):
            self.buffer.append_frame(frame)

    def test_signal_only_keeps_real_signal_and_marks_diagnostics_missing(self):
        self.feed(signal_packet())
        sample = self.buffer.latest_sensor_sample()
        self.assertFalse(sample["has_raw"])
        self.assertEqual(sample["signal"][4:], (6157, 9411, 5273, 7244))
        for key in ("hardware_raw", "raw", "baseline", "state"):
            self.assertEqual(sample[key], ())
        raw, *rest = self.buffer.snapshot()
        self.assertTrue(math.isnan(raw[7][-1]))
        self.assertEqual(self.buffer.snapshot()[5][7], -1)
        log = ForcePressControlWidget._sensor_log_values(sample)
        self.assertEqual(log["ch4567_signal_sum"], 28085)
        self.assertEqual(log["ch7_hardware_raw"], "")
        self.assertEqual(log["ch7_state"], "")
        self.assertFalse(log["sensor_four_channel_valid"])
        self.assertFalse(ForcePressControlWidget._sensor_sample_is_current(sample))

    def test_debug_signal_debug_never_reuses_stale_raw_or_state(self):
        self.feed(debug_packet())
        self.assertEqual(self.buffer.latest_sensor_sample()["hardware_raw"][7], 1007)
        self.feed(signal_packet())
        self.assertEqual(self.buffer.latest_sensor_sample()["state"], ())
        self.feed(debug_packet())
        self.assertEqual(self.buffer.latest_sensor_sample()["state"][7], 6)
        raw = self.buffer.snapshot()[0][7]
        self.assertEqual(raw[-3], 1007)
        self.assertTrue(math.isnan(raw[-2]))
        self.assertEqual(raw[-1], 1007)

    def test_force_and_temperature_frames_do_not_erase_debug(self):
        self.feed(debug_packet())
        self.feed(packet(2, struct.pack("<BBH6f", 2, 3, 63, *([1.0] * 6))))
        self.feed(packet(3, struct.pack("<h", 260), channels=0))
        sample = self.buffer.latest_sensor_sample()
        self.assertTrue(sample["has_raw"])
        self.assertEqual(sample["hardware_raw"][7], 1007)
        self.assertEqual(sample["die_temperature_celsius"], 26)

    def test_missing_padding_remains_missing_after_resize(self):
        self.feed(signal_packet())
        self.buffer.set_sample_count(8)
        self.assertTrue(all(math.isnan(x) for x in self.buffer.snapshot()[0][7]))
        self.assertEqual(set(self.buffer.snapshot()[6][7]), {-1})

    def test_display_differentiates_zero_missing_and_mode_mismatch(self):
        self.assertEqual(MainWindow._diagnostic_value_text(0), "0")
        self.assertEqual(MainWindow._diagnostic_value_text(0, False), "--")
        self.assertEqual(MainWindow._diagnostic_value_text(float("nan")), "--")
        self.assertIn("未上传", MainWindow._state_display(None, -1)[0])
        self.feed(signal_packet())
        text = MainWindow._stream_data_status_text(self.buffer.latest_sensor_sample(), OUTPUT_MODE_DEBUG)
        self.assertIn("Debug尚未生效", text)
        self.assertIn("重新应用", text)
        text = MainWindow._stream_data_status_text(self.buffer.latest_sensor_sample(), OUTPUT_MODE_RUN)
        self.assertNotIn("尚未生效", text)
        self.assertIn("中断", MainWindow._stream_data_status_text({"age_seconds": 2}, OUTPUT_MODE_DEBUG))

    def test_split_v14_signal_sets_version_before_first_mode_request(self):
        reader = SerialReaderThread("FAKE", self.buffer)
        wire = signal_packet(version=14)
        chunks = [wire[:7], wire[7:]]
        writes = []
        fake = SimpleNamespace(is_open=True, in_waiting=64, write=writes.append, flush=lambda: None, close=lambda: None)
        def read(_size):
            if chunks:
                return chunks.pop(0)
            reader._running = False
            return b""
        fake.read = read
        reader.stream_started.connect(lambda: reader.send_output_mode(OUTPUT_MODE_DEBUG))
        with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=lambda *a, **k: fake)}):
            reader.run()
        mode_writes = [value for value in writes if value[5] == 0x82]
        self.assertEqual(len(mode_writes), 1)
        self.assertEqual(mode_writes[0][4], 14)

    def test_table_refresh_shows_missing_diagnostics_with_valid_signal(self):
        self.feed(signal_packet())
        flags = [SimpleNamespace(checkbox=SimpleNamespace(isChecked=lambda: True)) for _ in range(8)]
        w = SimpleNamespace(data_buffer=self.buffer, _last_snapshot_version=-1, sample_count=5,
                            _plot_x_values=list(range(5)), _matrix_slow_k_valid_after_frame=None,
                            stream_data_status_label=QtWidgets.QLabel(),
                            raw_statistics_label=QtWidgets.QLabel(),
                            raw_response_widget=Mock(),
                            _update_record_button_state=Mock(),
                            _raw_statistics_text=MainWindow._raw_statistics_text,
                            state_background_channel_combo=SimpleNamespace(currentData=lambda: None),
                            output_mode_combo=SimpleNamespace(currentData=lambda: OUTPUT_MODE_DEBUG),
                            force_vector_checkbox=SimpleNamespace(isChecked=lambda: False),
                            channel_4567_signal_sum_checkbox=flags[0], channel_4567_signal_sum_curve=Mock(),
                            channel_4567_signal_sum_value_label=QtWidgets.QLabel(),
                            table=SimpleNamespace(item=lambda *args: None),
                            _stream_data_status_text=MainWindow._stream_data_status_text,
                            _diagnostic_value_text=MainWindow._diagnostic_value_text,
                            _format_force_sum_channel_name=MainWindow._format_force_sum_channel_name)
        for name in ("_update_rx_fps_display", "_refresh_die_temperature_display", "update_window_title", "_ensure_channel_count"):
            setattr(w, name, Mock())
        for prefix in ("raw", "baseline", "signal", "fast_k", "slow_k"):
            setattr(w, prefix + "_checkboxes", flags)
            setattr(w, prefix + "_curves", [Mock() for _ in range(8)])
            setattr(w, prefix + "_value_labels", [QtWidgets.QLabel() for _ in range(8)])
        w.force_sum_checkboxes = flags
        w._set_state_label = Mock()
        MainWindow.refresh_plots(w)
        self.assertEqual(w.raw_value_labels[7].text(), "--")
        self.assertEqual(w.baseline_value_labels[7].text(), "--")
        self.assertEqual(w.signal_value_labels[7].text(), "7244")
        self.assertEqual(w.channel_4567_signal_sum_value_label.text(), "28085")
        w._set_state_label.assert_any_call(7, -1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
