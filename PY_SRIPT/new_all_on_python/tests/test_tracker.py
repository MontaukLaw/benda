import json
import math
from pathlib import Path
import tempfile
import unittest

from new_all_on_python.config import AppConfig, SENSOR_POSITIONS
from new_all_on_python.touch_tracker import TouchTracker
from new_all_on_python.processing_engine import ProcessingEngine


class TrackerTests(unittest.TestCase):
    def make_tracker(self, **changes):
        config = AppConfig(baseline_update_rate=0, position_filter_enable=False,
                           roi_transition_enable=False,
                           dx_deadzone=0, dy_deadzone=0)
        for key, value in changes.items():
            setattr(config, key, value)
        tracker = TouchTracker(config)
        for n in range(70):
            tracker.update([12000] * 20, n / 100)
        self.assertFalse(tracker.processor.calibrating)
        return tracker

    @staticmethod
    def raw(**signals):
        values = [12000] * 20
        for key, value in signals.items():
            values[int(key[2:])] += value
        return values

    def test_mapping_and_single_channel_coordinates(self):
        self.assertEqual([[i for i, p in SENSOR_POSITIONS.items() if p[1] == y] for y in range(4)],
                         [[0, 4, 8, 12, 16], [1, 5, 9, 13, 17],
                          [2, 6, 10, 14, 18], [3, 7, 11, 15, 19]])
        for i in range(20):
            tracker = self.make_tracker()
            raw = [12000] * 20
            raw[i] += 10000
            result = tracker.update(raw, 0.70)
            self.assertEqual((result.x, result.y), SENSOR_POSITIONS[i])
            self.assertEqual((result.dx, result.dy, result.state), (0, 0, "TOUCH_DOWN"))

    def test_roi_center_and_clipped_corner(self):
        tracker = self.make_tracker()
        r = tracker.update(self.raw(CH10=10000), 0.70)
        self.assertEqual(set(r.roi_indices), {5, 6, 7, 9, 10, 11, 13, 14, 15})
        r = tracker.update(self.raw(CH0=10000), 0.71)
        self.assertEqual(set(r.roi_indices), {0, 1, 4, 5})

    def test_roi_transition_reduces_switch_jump_and_preserves_target_max(self):
        smooth = self.make_tracker(roi_transition_enable=True, roi_transition_ms=30)
        hard = self.make_tracker()
        before = self.raw(CH14=3008, CH12=2940, CH9=2298, CH11=2623, CH15=974)
        after = self.raw(CH14=2270, CH12=3150, CH9=2394, CH11=2468)
        smooth.update(before, 0.70)
        hard.update(before, 0.70)
        a = smooth.update(after, 0.71)
        b = hard.update(after, 0.71)
        self.assertEqual(a.max_index, b.max_index)
        self.assertEqual(a.roi_target_indices, b.roi_indices)
        self.assertLess(math.hypot(a.dx, a.dy), math.hypot(b.dx, b.dy) * 0.6)
        self.assertTrue(a.roi_transitioning)
        self.assertIn(14, a.roi_indices)
        for n in range(2, 5):
            a = smooth.update(after, 0.70 + n / 100)
        self.assertFalse(a.roi_transitioning)
        self.assertEqual(a.roi_indices, a.roi_target_indices)
        self.assertAlmostEqual(a.x_raw, b.x_raw)
        self.assertAlmostEqual(a.y_raw, b.y_raw)

    def test_roi_transition_uses_elapsed_time_not_frame_count(self):
        trackers = [self.make_tracker(roi_transition_enable=True, roi_transition_ms=30) for _ in range(2)]
        old, new = self.raw(CH14=3008, CH12=2940), self.raw(CH14=2270, CH12=3150)
        for tracker in trackers:
            tracker.update(old, 0.7)
        a = trackers[0].update(new, 0.71)
        trackers[0].update(new, 0.72)
        a = trackers[0].update(new, 0.73)
        trackers[1].update(new, 0.70000001)
        b = trackers[1].update(new, 0.73)
        for x, y in zip(a.roi_weights, b.roi_weights):
            self.assertAlmostEqual(x, y)
        self.assertAlmostEqual(a.x, b.x)
        self.assertAlmostEqual(a.y, b.y)

    def test_roi_transition_release_gap_and_far_new_contact(self):
        tracker = self.make_tracker(roi_transition_enable=True, roi_transition_ms=30)
        tracker.update(self.raw(CH0=30000), 0.70)
        # 旧位置已无信号，跨阵列的新接触仍应立即出现在新位置。
        result = tracker.update(self.raw(CH19=30000), 0.71)
        self.assertEqual((result.x, result.y), (4, 3))
        result = tracker.update([12000] * 20, 0.72)
        self.assertEqual((result.state, result.dx, result.dy), ("TOUCH_UP", 0, 0))
        result = tracker.update(self.raw(CH0=30000), 0.73)
        self.assertEqual((result.x, result.y, result.dx, result.dy), (0, 0, 0, 0))
        self.assertFalse(result.roi_transitioning)
        result = tracker.update(self.raw(CH19=30000), 1.03)
        self.assertEqual((result.x, result.y, result.dx, result.dy), (4, 3, 0, 0))
        self.assertFalse(result.roi_transitioning)

    def test_roi_transition_expires_remote_residue_and_keeps_long_hold(self):
        tracker = self.make_tracker(roi_transition_enable=True, roi_transition_ms=30)
        tracker.update(self.raw(CH0=30000), 0.70)
        result = tracker.update(self.raw(CH0=12000, CH18=30000), 0.71)
        self.assertEqual(result.max_index, 18)
        self.assertTrue(result.roi_transitioning)
        for n in range(2, 6):
            result = tracker.update(self.raw(CH0=12000, CH18=30000), 0.70 + n / 100)
        self.assertFalse(result.roi_transitioning)
        self.assertNotIn(0, result.roi_indices)
        self.assertEqual((result.x, result.y), (4, 2))
        for n in range(6, 306):
            result = tracker.update(self.raw(CH0=12000, CH18=30000), 0.70 + n / 100)
            self.assertEqual((result.state, result.dx, result.dy), ("MOVING", 0, 0))
    def test_roi_transition_toggle_zero_and_all_sensor_bypass(self):
        tracker = self.make_tracker(roi_transition_enable=True, roi_transition_ms=30)
        tracker.update(self.raw(CH14=3008, CH12=2940), 0.7)
        result = tracker.update(self.raw(CH14=2270, CH12=3150), 0.71)
        self.assertTrue(result.roi_transitioning)
        config = tracker.config.copy()
        config.roi_transition_enable = False
        tracker.set_config(config)
        result = tracker.update(self.raw(CH14=2270, CH12=3150), 0.72)
        self.assertFalse(result.roi_transitioning)
        self.assertEqual(result.roi_indices, result.roi_target_indices)
        for changes in ({"roi_transition_ms": 0}, {"roi_mode": "All Sensors"}):
            config.roi_transition_enable = True
            for key, value in changes.items():
                setattr(config, key, value)
            tracker.set_config(config)
            result = tracker.update(self.raw(CH14=3008, CH12=2940), result.timestamp + 0.01)
            self.assertFalse(result.roi_transitioning)

    def test_continuous_power_centroid_and_relative_threshold(self):
        tracker = self.make_tracker()
        r = tracker.update(self.raw(CH8=10000, CH12=8000, CH4=2000), 0.70)
        self.assertEqual(set(r.active_indices), {8, 12})
        self.assertAlmostEqual(r.x, (2 + 3 * 0.8 ** 2) / (1 + 0.8 ** 2))
        self.assertEqual(r.y, 0)
        self.assertTrue(2 < r.x < 3)

    def test_hysteresis_release_and_new_contact_zero_delta(self):
        tracker = self.make_tracker()
        self.assertEqual(tracker.update(self.raw(CH0=900), 0.70).state, "IDLE")
        self.assertEqual(tracker.update(self.raw(CH0=1200), 0.71).state, "TOUCH_DOWN")
        self.assertEqual(tracker.update(self.raw(CH0=800), 0.72).state, "MOVING")
        r = tracker.update(self.raw(CH16=500), 0.73)
        self.assertEqual((r.state, r.dx, r.dy), ("TOUCH_UP", 0, 0))
        r = tracker.update(self.raw(CH16=400), 0.74)
        self.assertEqual((r.state, r.dx, r.dy), ("IDLE", 0, 0))
        r = tracker.update(self.raw(CH19=5000), 0.75)
        self.assertEqual((r.state, r.dx, r.dy, r.x, r.y), ("TOUCH_DOWN", 0, 0, 4, 3))

    def test_off_zero_still_releases_zero_signal(self):
        tracker = self.make_tracker(touch_off_threshold=0)
        tracker.update(self.raw(CH0=1200), 0.7)
        self.assertEqual(tracker.update([12000] * 20, 0.71).state, "TOUCH_UP")

    def test_filter_deadzone_gain_and_inversion(self):
        tracker = self.make_tracker(position_filter_enable=True, position_alpha=0.4,
                                    x_gain=2, y_gain=3, invert_x=True)
        tracker.update(self.raw(CH0=10000), 0.70)
        r = tracker.update(self.raw(CH5=10000), 0.71)
        self.assertAlmostEqual(r.x, 0.4)
        self.assertAlmostEqual(r.y, 0.4)
        self.assertAlmostEqual(r.dx, -0.8)
        self.assertAlmostEqual(r.dy, 1.2)
        config = tracker.config.copy()
        config.dx_deadzone = config.dy_deadzone = 1
        tracker.set_config(config)
        r = tracker.update(self.raw(CH5=10000), 0.72)
        self.assertEqual((r.dx, r.dy), (0, 0))

    def test_max_switch_does_not_snap_to_sensor_center(self):
        tracker = self.make_tracker()
        a = tracker.update(self.raw(CH9=12000, CH10=11900), 0.7)
        b = tracker.update(self.raw(CH9=11800, CH10=12100), 0.71)
        self.assertEqual((a.max_index, b.max_index), (9, 10))
        self.assertLess(abs(b.y - a.y), 0.03)
        self.assertAlmostEqual(b.x, 2)

    def test_remote_and_weak_old_residue_is_excluded(self):
        tracker = self.make_tracker()
        tracker.update(self.raw(CH0=30000), 0.70)
        r = tracker.update(self.raw(CH0=12000, CH18=30000, CH14=5000), 0.71)
        self.assertEqual((r.x, r.y), (4, 2))
        self.assertNotIn(0, r.roi_indices)
        self.assertNotIn(14, r.active_indices)
        for n in range(10):
            r = tracker.update(self.raw(CH0=10000 - n * 500, CH18=30000, CH14=4000 - n * 100), 0.72 + n * 0.01)
            self.assertEqual((r.dx, r.dy), (0, 0))

    def test_derivative_weight_is_bounded_with_bursts_and_extremes(self):
        tracker = self.make_tracker(derivative_weight_enable=True, positive_gain=10, negative_gain=10,
                                    minimum_decay_weight=0, weight_power=4)
        tracker.update(self.raw(CH9=50000, CH10=2000), 0.7)
        r = tracker.update(self.raw(CH9=2000, CH10=50000), 0.70000001)
        self.assertTrue(all(math.isfinite(v) for v in (r.x, r.y, r.dx, r.dy)))
        self.assertTrue(0 <= r.x <= 4 and 0 <= r.y <= 3)

    def test_gap_reset_and_invalid_frame_does_not_mutate_tracker(self):
        tracker = self.make_tracker()
        tracker.update(self.raw(CH0=10000), 0.7)
        with self.assertRaises(ValueError):
            tracker.update([1] * 19, 0.71)
        with self.assertRaises(ValueError):
            tracker.update([1.0] * 20, 0.71)
        with self.assertRaises(ValueError):
            tracker.update([12000] * 20, float("nan"))
        r = tracker.update(self.raw(CH19=10000), 1.0)
        self.assertEqual((r.dx, r.dy, r.state), (0, 0, "TOUCH_DOWN"))
        with self.assertRaises(ValueError):
            tracker.update([12000] * 20, 1.0)

    def test_baseline_freezes_pressure_and_tracks_unloaded(self):
        tracker = self.make_tracker(baseline_update_rate=0.1)
        r = tracker.update(self.raw(CH0=5000, CH4=200), 0.70)
        self.assertEqual(r.baseline[0], 12000)
        self.assertEqual(r.signal[0], 5000)
        self.assertEqual(r.baseline[4], 12020)
        r = tracker.update([11900] * 20, 0.71)
        self.assertEqual(r.signal[0], 0)

    def test_negative_undershoot_recovery_does_not_create_phantom_contact(self):
        tracker = self.make_tracker(baseline_update_rate=0.01)
        tracker.update(self.raw(CH0=5000), 0.70)
        for n in range(200):
            result = tracker.update(self.raw(CH0=-4000), 0.71 + n / 100)
            self.assertEqual(result.baseline[0], 12000)
            self.assertEqual(result.signal[0], 0)
        # 返回原空载 Raw 时，不应因基线曾追踪下冲而重新报 DOWN。
        result = tracker.update([12000] * 20, 2.71)
        self.assertEqual((result.state, result.max_signal, result.dx, result.dy),
                         ("IDLE", 0, 0, 0))

    def test_bidirectional_protection_keeps_hold_and_small_idle_drift(self):
        tracker = self.make_tracker(baseline_update_rate=0.1)
        for n in range(200):
            result = tracker.update(self.raw(CH0=5000, CH16=-4000), 0.7 + n / 100)
            self.assertTrue(result.touched)
            self.assertEqual((result.baseline[0], result.baseline[16]), (12000, 12000))
            self.assertEqual((result.dx, result.dy), (0, 0))
        result = tracker.update(self.raw(CH0=5000, CH4=200, CH8=-100), 2.7)
        self.assertEqual((result.baseline[4], result.baseline[8]), (12020, 11990))
        # 可信空载重建仍能处理超过保护阈值的长期环境偏移。
        tracker.reset_baseline()
        for n in range(70):
            result = tracker.update([8000] * 20, 2.71 + n / 100)
        self.assertTrue(result.baseline_ready)
        self.assertEqual(result.baseline, (8000.0,) * 20)
        self.assertFalse(result.touched)

    def test_persistent_negative_difference_is_diagnostic_only_and_clears(self):
        tracker = self.make_tracker(baseline_update_rate=0.001)
        # 正常长按保持；另一个通道的短暂下冲不立即标记为持续异常。
        for n in range(30):
            result = tracker.update(self.raw(CH0=5000, CH3=-4000), 0.7 + n / 100)
            self.assertEqual(result.below_baseline_indices, ())
        for n in range(30, 60):
            result = tracker.update(self.raw(CH0=5000, CH3=-4000), 0.7 + n / 100)
        self.assertEqual(result.below_baseline_indices, (3,))
        self.assertIn("CH3", tracker.processor.status)
        self.assertEqual((result.state, result.x, result.y, result.dx, result.dy),
                         ("MOVING", 0, 0, 0, 0))
        self.assertEqual(result.baseline[3], 12000)
        result = tracker.update(self.raw(CH0=5000), 1.30)
        self.assertEqual(result.below_baseline_indices, ())
        self.assertNotIn("CH3", tracker.processor.status)
        # 断流后不能沿用断流前的异常持续时间。
        for n in range(60):
            result = tracker.update(self.raw(CH0=5000, CH3=-4000), 1.31 + n / 100)
        self.assertEqual(result.below_baseline_indices, (3,))
        result = tracker.update(self.raw(CH0=5000, CH3=-4000), 2.30)
        self.assertEqual(result.below_baseline_indices, ())
        tracker.reset_baseline()
        self.assertEqual(tracker.processor.below_baseline_indices, ())

    def test_baseline_rebuild_stability_and_high_fps(self):
        tracker = TouchTracker()
        for n in range(100):
            values = [12000 + (n % 2) * 1000] * 20
            tracker.update(values, n / 100)
        self.assertTrue(tracker.processor.calibrating)
        for n in range(100, 170):
            tracker.update([12000] * 20, n / 100)
        self.assertFalse(tracker.processor.calibrating)
        tracker.reset_baseline()
        for n in range(200):
            r = tracker.update([12500] * 20, 2 + n / 200)
            self.assertEqual((r.dx, r.dy), (0, 0))
        self.assertFalse(tracker.processor.calibrating)
        self.assertEqual(tracker.processor.baseline, [12500.0] * 20)

    def test_reset_tracker_keeps_baseline(self):
        tracker = self.make_tracker()
        tracker.update(self.raw(CH0=10000), 0.7)
        tracker.reset_tracker()
        r = tracker.update(self.raw(CH19=10000), 0.71)
        self.assertTrue(r.baseline_ready)
        self.assertEqual((r.state, r.dx, r.dy), ("TOUCH_DOWN", 0, 0))

    def test_config_roundtrip_and_rejects_nonfinite_or_wrong_types(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            AppConfig().save(path)
            self.assertEqual(AppConfig.load(path), AppConfig())
            for data in ({"position_alpha": float("nan")}, {"touch_off_threshold": 2000},
                         {"invert_x": "false"}, {"trail_length": 50.5}, {"weight_power": "2"}):
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.assertRaises(ValueError):
                    AppConfig.load(path)

    def test_engine_stale_zero_delta_and_accurate_sample_count(self):
        engine = ProcessingEngine(AppConfig())
        engine.begin_source("Serial")
        for n in range(70):
            engine.update([12000] * 20, n / 100)
        engine.update(self.raw(CH0=10000), 0.7)
        snap = engine.snapshot(1.01)
        self.assertTrue(snap["stale"])
        self.assertEqual(snap["fps"], 0)
        self.assertEqual((snap["result"].dx, snap["result"].dy), (0, 0))
        engine.update(self.raw(CH19=10000), 1.02)
        snap = engine.snapshot(1.02)
        self.assertEqual(snap["stream_gaps"], 1)
        self.assertEqual(snap["samples"], 72)
        self.assertEqual(snap["result"].state, "TOUCH_DOWN")


if __name__ == "__main__":
    unittest.main()
