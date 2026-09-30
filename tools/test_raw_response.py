"""验证固定Raw参考、响应比例边界、缓冲及CSV集成。"""
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "PY_SRIPT/algo_monitor"))
from raw_response import RawResponse
from raw_response_widget import RawResponseWidget
from data_buffer import AlgoDataBuffer
from force_press_control_widget import ForcePressControlWidget
from frame_parser import AlgoFrameParser
from qt_compat import QtWidgets
from test_missing_debug_data import debug_packet


BASE = (700, 600, 450, 1000)


def capture(model, values=BASE):
    model.start(0)
    for i in range(201):
        model.feed((1, 2, 3, 4) + tuple(values), 1, i / 100)
    assert model.reference is not None


class RawResponseTests(unittest.TestCase):
    def test_equal_increments_cancel_unequal_baselines(self):
        model = RawResponse()
        capture(model)
        model.feed((0, 0, 0, 0) + tuple(x + 100 for x in BASE), 1, 2.01)
        result = model.snapshot(2.01)["result"]
        self.assertEqual(result["delta"], (100,) * 4)
        self.assertEqual(result["cv_percent"], 0)
        self.assertEqual(result["shares_percent"], (25,) * 4)
        self.assertEqual(model.reference, BASE)

    def test_unequal_increment_and_zero_channel_are_not_clamped(self):
        model = RawResponse()
        capture(model)
        model.feed((0,) * 4 + tuple(b + d for b, d in zip(BASE, (100, 0, 0, 0))), 1, 2.01)
        self.assertAlmostEqual(model.result["cv_percent"], math.sqrt(3) * 100)
        self.assertEqual(model.result["shares_percent"], (100, 0, 0, 0))
        self.assertEqual(sum(model.result["shares_percent"]), 100)

    def test_small_zero_and_negative_deltas_hide_percentages(self):
        for delta in ((0,0,0,0), (2,2,2,2), (-1,100,100,100), (-10,-10,-10,-10)):
            model = RawResponse()
            capture(model)
            model.feed((0,) * 4 + tuple(b + d for b, d in zip(BASE, delta)), 1, 2.01)
            self.assertEqual(model.result["delta"], delta)
            self.assertIsNone(model.result["cv_percent"])
            self.assertIsNone(model.result["shares_percent"])
            self.assertTrue(math.isfinite(model.result["stddev"]))

    def test_stability_checks_drift_and_noise(self):
        for noisy in (False, True):
            model = RawResponse()
            model.start(0)
            for i in range(201):
                change = (10 if i % 2 else -10) if noisy else i / 10
                model.feed((0,) * 4 + tuple(x + change for x in BASE), 1, i / 100)
            self.assertIsNone(model.reference)
            self.assertIsNone(model.capture_start)

    def test_noise_dependent_threshold(self):
        model = RawResponse()
        model.start(0)
        for i in range(201):
            model.feed((0,) * 4 + tuple(x + (2 if i % 2 else -2) for x in BASE), 1, i / 100)
        self.assertIsNotNone(model.reference)
        model.feed((0,) * 4 + tuple(x + 4 for x in BASE), 1, 2.01)
        self.assertGreater(model.result["minimum_mean"], 5.9)
        self.assertIsNone(model.result["cv_percent"])

    def test_invalidation_on_stale_run_device_and_recapture(self):
        for action in ("stale", "run", "device", "capture"):
            model = RawResponse()
            capture(model)
            if action == "stale":
                model.snapshot(3)
            elif action == "run":
                model.feed(None, 1, 2.01)
            elif action == "device":
                model.feed((0,) * 4 + BASE, 2, 2.01)
            else:
                model.start(2.01)
            self.assertIsNone(model.reference)
            self.assertIsNone(model.result)

    def test_buffer_hardware_source_state_independence_and_csv(self):
        buffer = AlgoDataBuffer(5)
        frame = AlgoFrameParser().feed(debug_packet())[0]
        frame['raw'] = (0,) * 4 + BASE
        frame['normalized_raw'] = (50000,) * 8
        frame['baseline'] = (40000,) * 8
        frame['state'] = (0,) * 8
        frame['signal'] = (0,) * 8
        with patch('data_buffer.time.perf_counter', return_value=0):
            buffer.append_frame(frame)
            buffer.start_raw_reference()
        for i in range(1, 202):
            with patch('data_buffer.time.perf_counter', return_value=i / 100):
                buffer.append_frame(frame)
        frame['raw'] = (0,) * 4 + tuple(x + 100 for x in BASE)
        with patch('data_buffer.time.perf_counter', return_value=2.02):
            buffer.append_frame(frame)
            sample = buffer.latest_sensor_sample()
        result = sample['raw_response']['result']
        self.assertEqual(result['reference'], BASE)
        self.assertEqual(result['cv_percent'], 0)
        logged = ForcePressControlWidget._sensor_log_values(sample)
        self.assertEqual(logged['ch4_delta_hardware_raw'], 100)
        self.assertEqual(logged['ch7_delta_share_percent'], 25)
        self.assertEqual(logged['ch4567_delta_raw_cv_percent'], 0)
        self.assertTrue(logged['raw_response_reference_id'])
        buffer.update_raw_reference_config([{'channel':4,'hardware_nsub':300}])
        buffer.update_raw_reference_config([{'channel':4,'hardware_nsub':400}])
        with patch('data_buffer.time.perf_counter', return_value=2.02):
            cleared = buffer.latest_sensor_sample()
        self.assertIsNone(cleared['raw_response']['result'])
        self.assertEqual(ForcePressControlWidget._sensor_log_values(cleared)['ch4567_delta_raw_cv_percent'], '')

    def test_widget_buttons_and_large_numbers(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        buffer = AlgoDataBuffer(5)
        widget = RawResponseWidget(buffer)
        model = RawResponse()
        capture(model)
        model.feed((0,) * 4 + tuple(x + 100 for x in BASE), 1, 2.01)
        widget.update_sample({'raw_response':model.snapshot(2.01)})
        self.assertEqual(widget.cells['delta'][0].text(), '+100.00')
        self.assertEqual(widget.cells['shares_percent'][3].text(), '25.00%')
        self.assertIn('0.00%', widget.metrics.text())
        self.assertIn('18px', widget.cells['delta'][0].styleSheet())
        widget.capture_button.click()
        widget.close()
        app.processEvents()


if __name__ == '__main__':
    unittest.main(verbosity=2)
