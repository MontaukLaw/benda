import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pathlib import Path
import csv
import json
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from PySide6 import QtCore, QtTest, QtWidgets
import serial

from new_all_on_python.config import AppConfig, SENSOR_POSITIONS
from new_all_on_python.main_window import MainWindow
from new_all_on_python.processing_engine import ProcessingEngine
from new_all_on_python.serial_reader import SerialReader
from new_all_on_python.sample_clock import now as sample_now
from new_all_on_python.tests.test_protocol_recording import packet, raw20_packet


APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
# Windows的offscreen插件不枚举系统字体，显式加载后才能验证文本排版。
from PySide6 import QtGui
for font_path in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/consola.ttf"):
    if Path(font_path).exists():
        QtGui.QFontDatabase.addApplicationFont(font_path)
APP.setFont(QtGui.QFont("Microsoft YaHei", 9))


def wait_until(condition, timeout=2):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        QtTest.QTest.qWait(10)
        time.sleep(0.001)
    return bool(condition())


class GuiSerialTests(unittest.TestCase):
    def auto_window_case(self, behavior, stop=False):
        attempts, connections = [], []
        wire = packet(0x16, struct.pack("<20H", *([12000]*20)))

        class Device:
            def __init__(self, valid):
                self.data = wire if valid else b""
                self.closed = False

            @property
            def in_waiting(self):
                return len(self.data)

            def read(self, size):
                chunk, self.data = self.data[:size], self.data[size:]
                if not chunk:
                    time.sleep(0.005)
                return chunk

            def close(self):
                self.closed = True

        def factory(port, *args, **kwargs):
            # 回退到COM3之前必须先关闭COM4，禁止两个线程同时持有端口。
            self.assertTrue(all(c.closed for c in connections))
            attempts.append(port)
            action = behavior[port]
            if action == "error":
                raise OSError("port busy or unavailable")
            device = Device(action == "raw")
            connections.append(device)
            return device

        with tempfile.TemporaryDirectory() as folder, \
                patch("new_all_on_python.main_window.CONFIG_PATH", Path(folder)/"none.json"), \
                patch("new_all_on_python.main_window.AUTO_CONNECT_TIMEOUT_MS", 100), \
                patch("serial.Serial", side_effect=factory):
            window = MainWindow()
            try:
                window.show()
                if stop:
                    self.assertTrue(wait_until(lambda: len(attempts)==1))
                    window.stop_source()
                    self.assertTrue(wait_until(lambda: window.worker is None))
                    QtTest.QTest.qWait(150)
                    self.assertEqual(attempts,["COM4"])
                    self.assertFalse(window._auto_connect_active)
                    self.assertTrue(window.connect_button.isEnabled())
                elif "raw" in behavior.values():
                    self.assertTrue(wait_until(lambda: window.engine.samples_processed>0 and not window._auto_connect_active))
                    successful = "COM4" if behavior["COM4"]=="raw" else "COM3"
                    expected = ["COM4"] if successful=="COM4" else ["COM4","COM3"]
                    self.assertEqual(attempts,expected)
                    self.assertEqual(window.port_combo.currentText(),successful)
                    self.assertEqual(window.worker.port,successful)
                    self.assertFalse(window._auto_probe_timer.isActive())
                    # 连接成功后，普通断开不能触发另一端口的自动连接。
                    window.stop_source()
                    self.assertTrue(wait_until(lambda: window.worker is None))
                    QtTest.QTest.qWait(150)
                    self.assertEqual(attempts,expected)
                else:
                    self.assertTrue(wait_until(lambda: len(attempts)==2 and window.worker is None and not window._auto_connect_active))
                    self.assertTrue(window.connect_button.isEnabled())
                    self.assertIn("COM4",window.serial_status.text())
                    self.assertIn("COM3",window.serial_status.text())
                    self.assertIn("自动连接未成功",window.serial_status.text())
            finally:
                window.close()
                self.assertTrue(wait_until(lambda: window.worker is None))
            self.assertTrue(all(c.closed for c in connections))

    def test_startup_auto_connect_keeps_com4_after_valid_raw(self):
        self.auto_window_case({"COM4":"raw","COM3":"raw"})

    def test_startup_auto_connect_falls_back_from_busy_com4_to_com3(self):
        self.auto_window_case({"COM4":"error","COM3":"raw"})

    def test_startup_auto_connect_falls_back_from_silent_com4_to_com3(self):
        self.auto_window_case({"COM4":"silent","COM3":"raw"})

    def test_startup_auto_connect_both_fail_leaves_manual_controls(self):
        self.auto_window_case({"COM4":"error","COM3":"error"})

    def test_stop_cancels_pending_auto_fallback(self):
        self.auto_window_case({"COM4":"silent","COM3":"raw"},stop=True)

    def test_simulation_and_close_cancel_pending_auto_start(self):
        for simulate in (True,False):
            with patch("serial.Serial",side_effect=AssertionError("unexpected hardware access")) as opener:
                window = MainWindow()
                try:
                    if simulate:
                        window.start_simulation()
                        self.assertTrue(wait_until(lambda: window.engine.samples_processed>0))
                    window.close()
                    self.assertTrue(wait_until(lambda: window.worker is None))
                    APP.processEvents()
                    opener.assert_not_called()
                finally:
                    window.close()
                    wait_until(lambda: window.worker is None)

    def test_observation_shortcuts_record_timing_without_changing_touch(self):
        window = MainWindow(auto_connect=False)
        with tempfile.TemporaryDirectory() as folder:
            try:
                window.show()
                QtTest.QTest.qWait(30)
                self.assertFalse(window.engine.mark_observation("released"))
                window.engine.begin_source("Serial")
                start = sample_now() - 1.4
                for n in range(70):
                    window.engine.update([12000] * 20, start + n / 100)
                raw = [12000] * 20
                raw[0], raw[3] = 17000, 8000
                for n in range(70, 140):
                    window.engine.update(raw, start + n / 100)
                self.assertEqual(window.engine.result.below_baseline_indices, (3,))
                window.engine.recorder.start(window.config, window.engine.revision, folder)
                # 第一行同时保存完整采样及持续负差诊断。
                window.engine.update(raw, sample_now())
                window.refresh_view()
                self.assertIn("CH3", window.baseline_label.text())
                original = window.engine.result
                window.activateWindow()
                QtTest.QTest.qWait(20)
                QtTest.QTest.keyClick(window, QtCore.Qt.Key.Key_F7)
                QtTest.QTest.keyClick(window, QtCore.Qt.Key.Key_F8)
                self.assertIs(window.engine.result, original)
                artifact_dir = Path(__file__).resolve().parents[1] / "_verification"
                artifact_dir.mkdir(exist_ok=True)
                self.assertTrue(window.grab().save(str(artifact_dir / "gui_baseline_warning.png")))
                self.assertTrue(window.engine.recorder.finish())
                path = window.engine.recorder.path
                events = [json.loads(line) for line in path.with_suffix(".events.jsonl").read_text(encoding="utf-8").splitlines()]
                marks = [event for event in events if event["kind"] == "user_observation"]
                self.assertEqual([e["observation"] for e in marks], ["contact", "released"])
                for mark in marks:
                    self.assertEqual(mark["latest_sample_timestamp"], original.timestamp)
                    self.assertGreaterEqual(mark["monotonic_timestamp"], original.timestamp)
                    self.assertEqual(mark["processed_sample_count"], 141)
                    self.assertEqual(mark["algorithm_state"], "MOVING")
                with path.open(encoding="utf-8-sig", newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(rows[0]["below_baseline_indices"], "3")
                self.assertTrue(all(row["firmware_state3"] == "" for row in rows))
                window.refresh_view()
                self.assertTrue(all(not button.isEnabled() for button in window.marker_buttons))
            finally:
                window.engine.recorder.finish()
                window.close()
                APP.processEvents()

    def test_main_entry_starts_simulation_and_exits_cleanly(self):
        script = '''
import runpy
from PySide6 import QtCore, QtWidgets
OriginalApplication = QtWidgets.QApplication
class SmokeApplication(OriginalApplication):
    def __init__(self, args):
        super().__init__(args)
        QtCore.QTimer.singleShot(900, self.finish)
    def finish(self):
        windows = self.topLevelWidgets()
        main = next(w for w in windows if isinstance(w, QtWidgets.QMainWindow))
        assert main.engine.samples_processed > 50
        assert main.engine.result.baseline_ready
        main.close()
QtWidgets.QApplication = SmokeApplication
sys_module = __import__('sys')
sys_module.argv = ['main.py', '--simulate']
runpy.run_path('main.py', run_name='__main__')
'''
        result = subprocess.run([sys.executable, "-c", script],
                                cwd=Path(__file__).resolve().parents[1],
                                capture_output=True, text=True, timeout=6)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_window_close_drains_recording_tail(self):
        window = MainWindow(auto_connect=False)
        with tempfile.TemporaryDirectory() as folder:
            window.show()
            window.start_simulation()
            with window.engine.lock:
                window.engine.recorder.start(window.config, window.engine.revision, folder)
            try:
                self.assertTrue(wait_until(lambda: window.engine.samples_processed >= 65))
                window.close()
                self.assertTrue(wait_until(lambda: window.worker is None and not window.engine.recorder.closing))
                recorder = window.engine.recorder
                self.assertFalse(recorder.error)
                self.assertGreater(recorder.rows_written, 50)
                with recorder.path.open(encoding="utf-8-sig") as file:
                    self.assertEqual(sum(1 for _ in file) - 1, recorder.rows_written)
            finally:
                window.close()
                wait_until(lambda: window.worker is None and not window.engine.recorder.closing)

    def test_window_render_mapping_and_numeric_inputs(self):
        window = MainWindow(auto_connect=False)
        try:
            window.show()
            QtTest.QTest.qWait(30)
            window.engine.begin_source("Simulation")
            start = sample_now() - 0.7
            for n in range(70):
                window.engine.update([12000] * 20, start + n * 0.01)
            raw = [12000] * 20
            raw[9], raw[10], raw[13], raw[14] = 18000, 22000, 16000, 20000
            window.engine.update(raw, sample_now())
            window.refresh_view()
            widget = window.sensor_widget
            self.assertLess(widget._point(0, 0).y(), widget._point(0, 3).y())
            self.assertLess(widget._point(0, 0).x(), widget._point(4, 0).x())
            for y in range(4):
                self.assertEqual([i for i, p in SENSOR_POSITIONS.items() if p[1] == y],
                                 list(range(y, 20, 4)))
            for control in window.findChildren(QtWidgets.QAbstractSpinBox):
                self.assertEqual(control.prefix(), "")
                self.assertEqual(control.suffix(), "")
            window.controls["touch_off_threshold"].setValue(2000)
            self.assertGreater(window.config.touch_on_threshold, window.config.touch_off_threshold)
            window.controls["relative_threshold_ratio"].setValue(0.5)
            self.assertEqual(window.engine.tracker.config.relative_threshold_ratio, 0.5)
            window.controls["roi_transition_ms"].setValue(40)
            self.assertEqual(window.engine.tracker.config.roi_transition_ms, 40)
            window.controls["roi_transition_enable"].setChecked(False)
            self.assertFalse(window.engine.tracker.config.roi_transition_enable)
            window.controls["roi_transition_enable"].setChecked(True)
            artifact_dir = Path(__file__).resolve().parents[1] / "_verification"
            artifact_dir.mkdir(exist_ok=True)
            self.assertTrue(window.grab().save(str(artifact_dir / "gui.png")))
            window.toolbox.setCurrentIndex(next(i for i in range(window.toolbox.count())
                                                 if window.toolbox.itemText(i).startswith("Position")))
            QtTest.QTest.qWait(20)
            self.assertTrue(window.grab().save(str(artifact_dir / "gui_roi_transition.png")))
            window.toolbox.setCurrentIndex(next(i for i in range(window.toolbox.count())
                                                 if window.toolbox.itemText(i).startswith("Debug")))
            QtTest.QTest.qWait(20)
            self.assertTrue(window.grab().save(str(artifact_dir / "gui_debug.png")))
        finally:
            window.close()
            APP.processEvents()

    def test_mouse_simulation_100hz_and_clean_close(self):
        window = MainWindow(auto_connect=False)
        try:
            window.show()
            window.start_simulation()
            self.assertTrue(wait_until(lambda: window.engine.result is not None and window.engine.result.baseline_ready))
            position = window.sensor_widget._point(1.6, 2.1).toPoint()
            QtTest.QTest.mousePress(window.sensor_widget, QtCore.Qt.MouseButton.LeftButton, pos=position)
            self.assertTrue(wait_until(lambda: window.engine.result.touched))
            self.assertAlmostEqual(window.engine.result.x, 1.6, delta=0.2)
            self.assertAlmostEqual(window.engine.result.y, 2.1, delta=0.2)
            QtTest.QTest.mouseRelease(window.sensor_widget, QtCore.Qt.MouseButton.LeftButton, pos=position)
            self.assertTrue(wait_until(lambda: not window.engine.result.touched))
            self.assertEqual((window.engine.result.dx, window.engine.result.dy), (0, 0))
            self.assertEqual(window.engine.last_touch_event, "TOUCH_UP")
            window.stop_source()
            self.assertTrue(wait_until(lambda: window.worker is None))
            self.assertFalse(window.engine.recorder.closing)
        finally:
            window.close()
            wait_until(lambda: window.worker is None)

    def test_serial_connection_parse_and_reconnect_via_pyserial_loopback(self):
        for protocol in ("DATA", "RAW20"):
            engine = ProcessingEngine(AppConfig())
            engine.begin_source("Serial")
            connections = []

            def factory(port, baud, **kwargs):
                connection = serial.serial_for_url(port, baudrate=baud, **kwargs)
                connections.append(connection)
                return connection

            for run in range(2):
                engine.begin_source("Serial")
                worker = SerialReader("loop://", 1000000, protocol, engine, serial_factory=factory)
                worker.start()
                try:
                    self.assertTrue(wait_until(lambda: len(connections) > run), worker.error_message)
                    wire = (packet(0x16, struct.pack("<20H", *range(20))) if protocol == "DATA"
                            else raw20_packet(range(20)))
                    connections[-1].write(wire * 3)
                    self.assertTrue(wait_until(lambda: engine.samples_processed == 3))
                    self.assertEqual(engine.result.raw, tuple(range(20)))
                    self.assertEqual(worker.decoder.parse_errors, 0)
                finally:
                    worker.stop()
                    self.assertTrue(worker.wait(2000))
                self.assertFalse(connections[-1].is_open)
                self.assertEqual(engine.source, "None")

    def test_signal_only_requests_debug_using_observed_version(self):
        class FakeSerial:
            in_waiting = 100

            def __init__(self, *args, **kwargs):
                self.writes = []
                self.data = packet(0x01, struct.pack("<20H", *([2000] * 20)), version=14)
                self.closed = False

            def read(self, size):
                data, self.data = self.data, b""
                if not data:
                    time.sleep(0.01)
                return data

            def write(self, data):
                self.writes.append(data)
                return len(data)

            def close(self):
                self.closed = True

        device = FakeSerial()
        engine = ProcessingEngine(AppConfig())
        engine.begin_source("Serial")
        worker = SerialReader("fake", 1000000, "DATA", engine, serial_factory=lambda *a, **k: device)
        worker.start()
        try:
            self.assertTrue(wait_until(lambda: len(device.writes) == 1))
            self.assertEqual(device.writes[0], b"DATA\x0e\x82\x00\x00\x01\x00\x01")
            self.assertEqual(engine.samples_processed, 0)
        finally:
            worker.stop()
            self.assertTrue(worker.wait(2000))

    def test_serial_fault_is_reported_without_exiting_gui(self):
        engine = ProcessingEngine(AppConfig())
        engine.begin_source("Serial")

        def failure(*args, **kwargs):
            raise OSError("device unplugged")

        worker = SerialReader("fake", 1000000, "DATA", engine, serial_factory=failure)
        worker.start()
        self.assertTrue(worker.wait(2000))
        self.assertIn("device unplugged", worker.error_message)
        self.assertEqual(engine.source, "None")

    def test_batched_frames_survive_coarse_windows_monotonic_clock(self):
        for protocol in ("DATA", "RAW20"):
            engine = ProcessingEngine(AppConfig())
            engine.begin_source("Serial")
            wire = (packet(0x16, struct.pack("<20H", *range(20))) if protocol == "DATA"
                    else raw20_packet(range(20)))

            class BatchedSerial:
                in_waiting = len(wire) * 200

                def __init__(self):
                    self.data = wire * 200

                def read(self, size):
                    data, self.data = self.data, b""
                    if not data:
                        worker.requestInterruption()
                    return data

                def close(self):
                    pass

            connection = BatchedSerial()
            worker = SerialReader("fake", 1000000, protocol, engine,
                                  serial_factory=lambda *a, **k: connection)
            # 原接入在同一粗时钟tick内只保留首帧；高精度时钟必须完整保留。
            with patch("time.monotonic", return_value=1000.0):
                worker.start()
                self.assertTrue(worker.wait(2000))
            self.assertEqual(engine.samples_processed, 200)
            self.assertEqual(engine.rejected_samples, 0)
            self.assertEqual(engine.stream_gaps, 0)


if __name__ == "__main__":
    unittest.main()
