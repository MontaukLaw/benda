"""验证派生 Signal 质心、逐帧相对位移及松手/重置后的锚定。"""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker
from centroid_filter import signal_centroid, active_adjacent_pairs


def spot(x=2., y=1.5, amplitude=2000):
    return [amplitude * max(0, 1 - abs(i // 4 + .5 - x))
            * max(0, 1 - abs(i % 4 + .5 - y)) for i in range(20)]


class HostSource:
    def __init__(self, hz=100):
        self.hz, self.time, self.sequence, self.generation = hz, 0., 0, 1

    def sample(self, values=None, **overrides):
        self.time += 1 / self.hz
        self.sequence += 1
        values = spot() if values is None else values
        result = dict(sequence=self.sequence, sample_timestamp=self.time,
                      channel_count=20, has_raw=True, raw_only=True,
                      raw=[60000] * 20, signal=[0] * 20, state=[-1] * 20,
                      host_signal=values, host_state=[1 if value > 0 else 0 for value in values],
                      host_generation=self.generation, host_ready=True)
        result.update(overrides)
        return result


class HostSignalTouchpadTests(unittest.TestCase):
    def ready(self, hz=100):
        tracker = TouchpadTracker()
        tracker.set_source('host_signal')
        tracker.set_enabled(True)
        source = HostSource(hz)
        for _ in range(hz):
            self.assertEqual(tracker.feed(source.sample()), (0, 0))
        self.assertTrue(tracker.contact)
        self.assertIsNone(tracker.reference)
        return tracker, source

    def test_direct_twenty_channel_weights_no_gain_or_reference_applied_twice(self):
        tracker = TouchpadTracker()
        tracker.set_source('host_signal')
        tracker.set_raw_gains([10.] * 20)
        tracker.set_enabled(True)
        source = HostSource()
        values = [0.] * 20
        values[0], values[1], values[19] = 1000., 1000., 3000.
        for _ in range(100):
            tracker.feed(source.sample(values))
        self.assertEqual(tracker.raw_position, signal_centroid(values, local_2x2_enabled=False))
        self.assertEqual(tracker.raw_position, (2.9, 2.5))
        self.assertEqual(tracker.position, tracker.raw_position)
        self.assertTrue(tracker.contact)
        self.assertIsNone(tracker.reference)
        self.assertIsNone(tracker.noise)

    def test_centroid_delta_fixed_sensitivity_multiple_rates_and_pressure_scaling(self):
        for hz in (70, 100, 200):
            with self.subTest(hz=hz):
                tracker, source = self.ready(hz)
                for index in range(hz):
                    tracker.feed(source.sample(spot(amplitude=2000 + index * 10)))
                    self.assertAlmostEqual(tracker.delta[0], 0.)
                    self.assertAlmostEqual(tracker.delta[1], 0.)
                totals = [0., 0.]
                previous = tracker.position
                for index in range(2 * hz):
                    values = spot(2. + .4 * (index + 1) / (2 * hz),
                                  1.5 + .3 * (index + 1) / (2 * hz), amplitude=2000 + (hz - 1) * 10)
                    dx, dy = tracker.feed(source.sample(values))
                    for axis in (0, 1):
                        self.assertAlmostEqual(tracker.centroid_delta[axis], tracker.position[axis] - previous[axis])
                        self.assertAlmostEqual(tracker.delta[axis], tracker.centroid_delta[axis] * tracker.sensitivity)
                    totals[0] += dx
                    totals[1] += dy
                    previous = tracker.position
                for _ in range(hz):
                    dx, dy = tracker.feed(source.sample(spot(2.4, 1.8, amplitude=2000 + (hz - 1) * 10)))
                    totals[0] += dx
                    totals[1] += dy
                self.assertAlmostEqual(totals[0], 120., delta=1.)
                self.assertAlmostEqual(totals[1], 90., delta=1.)

    def test_release_residual_no_recoil_and_new_touch_reanchors(self):
        tracker, source = self.ready()
        held = [0.] * 20
        held[4], held[5], held[19] = 2000., 2000., 300.
        for _ in range(100):
            tracker.feed(source.sample(held))
        residual = [0.] * 20
        residual[19] = 300.
        for _ in range(20):
            self.assertEqual(tracker.feed(source.sample(residual)), (0, 0))
            self.assertFalse(tracker.contact)
            self.assertTrue(tracker.release_blocked)
            self.assertTrue(all(axis.filtered is None for axis in tracker.axes))
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(3.2))), (0, 0))
        self.assertTrue(tracker.contact)
        self.assertFalse(tracker.release_blocked)
        deltas = [tracker.feed(source.sample(spot(3.2 + .2 * (i + 1) / 100))) for i in range(100)]
        self.assertGreater(sum(dx for dx, dy in deltas), 40.)

    def test_generation_gap_missing_raw_initialization_and_duplicate_breaks(self):
        tracker, source = self.ready()
        frame = source.sample(spot(2.05))
        tracker.feed(frame)
        position = tracker.position
        self.assertEqual(tracker.feed(frame), (0, 0))
        self.assertEqual(tracker.position, position)
        source.generation += 1
        self.assertEqual(tracker.feed(source.sample(spot(3.2))), (0, 0))
        self.assertFalse(tracker.contact)
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(3.2))), (0, 0))
        self.assertTrue(tracker.contact)
        source.time += .3
        self.assertEqual(tracker.feed(source.sample(spot(1.5))), (0, 0))
        self.assertFalse(tracker.contact)
        for invalid in (dict(has_raw=False), dict(host_ready=False), dict(host_signal=None),
                        dict(host_signal=[math.nan] * 20), dict(host_signal=[-1] * 20),
                        dict(host_state=None), dict(host_state=[math.nan] * 20)):
            self.assertEqual(tracker.feed(source.sample(**invalid)), (0, 0))
            self.assertFalse(tracker.contact)
            self.assertIsNone(tracker.position)

    def test_global_spatial_transfer_direction_reversal_and_source_switch(self):
        tracker, source = self.ready()
        deltas = []
        for index in range(400):
            deltas.append(tracker.feed(source.sample(spot(2. + .4 * math.sin(index * .025)))))
            self.assertFalse(tracker.release_blocked)
            self.assertTrue(tracker.contact)
        self.assertTrue(any(dx > .1 for dx, dy in deltas))
        self.assertTrue(any(dx < -.1 for dx, dy in deltas))
        self.assertTrue(tracker.set_source('raw_reference'))
        self.assertFalse(tracker.contact)
        self.assertEqual(tracker.delta, (0, 0))
        self.assertEqual(tracker.feed(source.sample()), (0, 0))
        self.assertIsNone(tracker.reference)

    def test_physical_adjacency_no_diagonal_or_numbering_wrap(self):
        for pair, allowed in (((0, 1), True), ((0, 4), True), ((18, 19), True),
                              ((15, 19), True), ((3, 4), False), ((0, 5), False),
                              ((0, 19), False)):
            with self.subTest(pair=pair):
                states = [0] * 20
                for ch in pair:
                    states[ch] = 1
                self.assertEqual(bool(active_adjacent_pairs(states, 1)), allowed)

    def test_no_centroid_before_adjacent_pair_and_only_real_activate_counts(self):
        tracker = TouchpadTracker()
        tracker.set_source('host_signal')
        tracker.set_enabled(True)
        source = HostSource()
        with patch('touchpad.signal_centroid', wraps=signal_centroid) as centroid:
            for channels in ((5,), (0, 19), (3, 4), (0, 5), ()):
                weights = [0.] * 20
                for ch in channels:
                    weights[ch] = 2000.
                for _ in range(5):
                    self.assertEqual(tracker.feed(source.sample(weights)), (0, 0))
                self.assertEqual(centroid.call_count, 0)
                self.assertIsNone(tracker.position)
                self.assertEqual(tracker.motion_block_reason, 'no_adjacent_activate_pair')
            values = [0.] * 20
            values[5], values[9] = 1000., 1000.
            states = [0] * 20
            states[5], states[9] = 1, 3
            tracker.feed(source.sample(values, host_state=states))
            self.assertEqual(centroid.call_count, 0)
            self.assertEqual(tracker.host_active_channels, (5,))
            self.assertFalse(tracker.host_adjacent_pairs)

    def test_hold_and_resume_do_not_compute_or_send_cross_gap_displacement(self):
        tracker, source = self.ready()
        before_raw, before_filtered = tracker.raw_position, tracker.position
        # 权重完全不变而真实状态失去相邻对，必须立即拦截位置计算。
        for active_channels in ((5,), (0, 19)):
            states = [0] * 20
            for ch in active_channels:
                states[ch] = 1
            with patch('touchpad.signal_centroid', wraps=signal_centroid) as centroid:
                for _ in range(10):
                    self.assertEqual(tracker.feed(source.sample(host_state=states)), (0, 0))
                    self.assertEqual(tracker.raw_position, before_raw)
                    self.assertEqual(tracker.position, before_filtered)
                    self.assertTrue(tracker.centroid_held)
                    self.assertFalse(tracker.contact)
                    self.assertTrue(all(axis.filtered is None for axis in tracker.axes))
                self.assertEqual(centroid.call_count, 0)
        # 恢复到远处的有效相邻通道，重新确认并直接锚定，不补发冻结期间的差值。
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(3.2, 2.2))), (0, 0))
        self.assertTrue(tracker.contact)
        self.assertFalse(tracker.centroid_held)
        self.assertAlmostEqual(tracker.position[0], 3.2)
        self.assertAlmostEqual(tracker.position[1], 2.2)
        outputs = [tracker.feed(source.sample(spot(3.2 + .2 * (i + 1) / 100, 2.2))) for i in range(100)]
        self.assertGreater(sum(dx for dx, dy in outputs), 40.)


if __name__ == '__main__':
    unittest.main()
