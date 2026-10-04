"""触摸板行为回归：连续位移、压力变化、噪声、抬指、断流及逐帧消费。"""
import math
from pathlib import Path
import random
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker, FractionalMotion
from data_buffer import AlgoDataBuffer


BASE = [8000 + i * 123 for i in range(20)]


def spot(x=2.0, y=1.5, amplitude=1000):
    # 双线性响应在阵列内部具有已知真实质心，便于验证微小位移和幅度缩放。
    return [amplitude * max(0, 1-abs(i//4+.5-x)) * max(0, 1-abs(i%4+.5-y)) for i in range(20)]


class Source:
    def __init__(self, hz=100):
        self.hz, self.sequence, self.time = hz, 0, 0.0

    def sample(self, weights=None, state=2, signal=None):
        self.sequence += 1
        self.time += 1/self.hz
        weights = [0]*20 if weights is None else weights
        return dict(sequence=self.sequence, sample_timestamp=self.time,
                    channel_count=20, has_raw=True,
                    raw=[a+b for a,b in zip(BASE, weights)],
                    signal=weights if signal is None else signal,
                    state=[state]*20)


class TouchpadTests(unittest.TestCase):
    def ready(self, hz=100):
        tracker, source = TouchpadTracker(), Source(hz)
        tracker.set_enabled(True)
        tracker.start_reference()
        for _ in range(2*hz):
            tracker.feed(source.sample(state=0))
        self.assertEqual(tracker.reference, tuple(BASE))
        return tracker, source

    def settle(self, tracker, source, x=2.0, y=1.5):
        return [tracker.feed(source.sample(spot(x,y))) for _ in range(source.hz)]

    def test_reference_rejects_contact_and_unstable_background(self):
        tracker, source = self.ready()
        tracker.start_reference()
        self.settle(tracker, source)
        self.assertIsNone(tracker.reference)
        for i in range(250):
            tracker.feed(source.sample([i]*20, state=0, signal=[0]*20))
        self.assertIsNone(tracker.reference)
        for _ in range(150):
            tracker.feed(source.sample(state=0))
        self.assertEqual(tracker.reference, tuple(BASE))

    def test_reference_progress_reasons_and_persistent_completion(self):
        tracker, source = self.ready()
        tracker.start_reference()
        self.assertEqual(tracker.reference_status['stage'], 'waiting')
        tracker.feed(source.sample(state=6))
        self.assertIn('CH0(FLOATING)', tracker.reference_status['detail'])
        self.assertEqual(tracker.reference_status['progress'], 0)
        for _ in range(50):
            tracker.feed(source.sample(state=0))
        status = tracker.reference_status
        self.assertEqual(status['stage'], 'collecting')
        self.assertTrue(40 <= status['progress'] <= 50)
        self.assertEqual(status['frames'], 50)
        for _ in range(60):
            tracker.feed(source.sample(state=0))
        completed = dict(tracker.reference_status)
        self.assertEqual(completed['stage'], 'complete')
        self.assertEqual(completed['progress'], 100)
        self.settle(tracker, source)
        tracker.expire(source.time+1)
        self.assertEqual(tracker.reference_status, completed)

    def test_reference_cancel_invalid_data_and_interrupt_reset_progress(self):
        tracker, source = self.ready()
        tracker.start_reference()
        for _ in range(50):
            tracker.feed(source.sample(state=0))
        tracker.expire(source.time+.2)
        self.assertEqual(tracker.reference_status['stage'], 'waiting')
        self.assertEqual(tracker.reference_status['progress'], 0)
        frame = source.sample(state=0)
        frame['has_raw'] = False
        tracker.feed(frame)
        self.assertIn('Debug', tracker.reference_status['detail'])
        tracker.cancel_reference()
        self.assertFalse(tracker.calibrating)
        self.assertEqual(tracker.reference_status['stage'], 'cancelled')
        tracker.start_reference()
        for i in range(130):
            tracker.feed(source.sample([i]*20, state=0, signal=[0]*20))
        self.assertEqual(tracker.reference_status['stage'], 'unstable')
        self.assertLess(tracker.reference_status['progress'], 100)
        tracker.set_enabled(False)
        self.assertEqual(tracker.reference_status['stage'], 'cancelled')

    def test_subpixel_motion_and_fixed_gain_at_multiple_sample_rates(self):
        for hz in (70, 100, 200):
            with self.subTest(hz=hz):
                tracker, source = self.ready(hz)
                self.settle(tracker, source)
                outputs = []
                for i in range(2*hz):
                    outputs.append(tracker.feed(source.sample(spot(2+.4*(i+1)/(2*hz)))))
                outputs += self.settle(tracker, source, 2.4)
                self.assertAlmostEqual(sum(dx for dx,dy in outputs), 120, delta=1)
                self.assertLess(max(abs(dy) for dx,dy in outputs), .001)
                self.assertGreater(sum(0 < dx < 1 for dx,dy in outputs), hz)

    def test_pressure_scaling_and_locked_signal_do_not_lock_position(self):
        tracker, source = self.ready()
        self.settle(tracker, source)
        motion = []
        for i in range(200):
            amplitude = 1200 + 700*math.sin(i*.04)
            motion.append(tracker.feed(source.sample(spot(amplitude=amplitude), state=6, signal=[200]*20)))
        self.assertLess(sum(abs(dx)+abs(dy) for dx,dy in motion), .01)
        motion = [tracker.feed(source.sample(spot(2+.5*(i+1)/100), state=6, signal=[200]*20)) for i in range(100)]
        self.assertGreater(sum(dx for dx,dy in motion), 100)
        self.assertEqual(tracker.reference, tuple(BASE))

    def test_lift_retouch_and_residual_do_not_move_cursor(self):
        tracker, source = self.ready()
        self.settle(tracker, source)
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(3.5), state=4, signal=[0]*20)), (0,0))
        self.assertFalse(tracker.contact)
        moved = self.settle(tracker, source, 3.5)
        self.assertTrue(all(delta == (0,0) for delta in moved))

    def test_noise_and_single_frame_spike(self):
        tracker, source = self.ready()
        self.settle(tracker, source)
        rng = random.Random(24)
        position, positions = [0.,0.], []
        for i in range(600):
            values = [max(0, value+rng.uniform(-1,1)) for value in spot()]
            if i == 300:
                values[19] += 20000
            dx,dy = tracker.feed(source.sample(values))
            position = [position[0]+dx, position[1]+dy]
            positions.append(tuple(position))
        self.assertLess(max(math.hypot(x,y) for x,y in positions), 1.0)
        self.assertTrue(tracker.contact)

    def test_release_tail_stops_before_small_active_channel_clears(self):
        tracker, source = self.ready()
        def sample(main, released=False):
            values = [0]*20
            values[4], values[19] = main, 300
            frame = source.sample(values)
            frame['state'] = [0]*20
            frame['state'][4], frame['state'][19] = (4 if released else 2), 2
            frame['signal'] = [0]*20
            frame['signal'][4], frame['signal'][19] = (0 if released else main), 150
            return frame
        for _ in range(100): tracker.feed(sample(4000))
        seen_guard = False
        for i in range(15):
            delta = tracker.feed(sample(max(50, 4000-350*(i+1)), released=True))
            if tracker.motion_block_reason:
                seen_guard = True
            if seen_guard:
                self.assertEqual(delta, (0,0))
                self.assertFalse(tracker.contact)
                self.assertTrue(all(axis.filtered is None for axis in tracker.axes))
        self.assertTrue(seen_guard)
        # 小通道始终保持活动且Signal达标，尾部仍不能重新产生回勾。
        for _ in range(100):
            self.assertEqual(tracker.feed(sample(50, released=True)), (0,0))
            self.assertTrue(tracker.release_blocked)
        # 同一次串口流中的新落指须重新锚定，不能补发被丢弃的尾量。
        deltas = self.settle(tracker, source, 3.5)
        self.assertTrue(tracker.contact)
        self.assertFalse(tracker.release_blocked)
        self.assertTrue(all(delta == (0,0) for delta in deltas))
        moved = [tracker.feed(source.sample(spot(3.5+.2*(i+1)/50))) for i in range(50)]
        self.assertGreater(sum(dx for dx,dy in moved), 40)

    def test_spatial_transfer_and_direction_reversal_survive_recovery_channels(self):
        tracker, source = self.ready()
        self.settle(tracker, source, 2.0)
        previous = spot(2.0)
        for i in range(400):
            values = spot(2.0+.3*math.sin(i*.025))
            frame = source.sample(values)
            # 模拟滑动中离开的通道进入恢复、接近的通道继续活动；总负载不消失。
            frame['state'] = [4 if value < old else 2 for value,old in zip(values,previous)]
            frame['signal'] = [0 if state == 4 else value for state,value in zip(frame['state'],values)]
            tracker.feed(frame)
            self.assertFalse(tracker.release_blocked)
            self.assertTrue(tracker.contact)
            previous = values

    def test_gap_duplicate_run_and_invalid_frames_break_trajectory(self):
        tracker, source = self.ready()
        self.settle(tracker, source)
        sample = source.sample(spot(2.1))
        tracker.feed(sample)
        self.assertEqual(tracker.feed(sample), (0,0))
        source.time += .3
        self.assertEqual(tracker.feed(source.sample(spot(3.5))), (0,0))
        self.assertFalse(tracker.contact)
        self.settle(tracker, source, 3.5)
        source.sequence += 20
        self.assertEqual(tracker.feed(source.sample(spot(1.5))), (0,0))
        self.assertFalse(tracker.contact)
        for invalid in (dict(has_raw=False), dict(raw=[float('nan')]*20), dict(channel_count=8)):
            self.settle(tracker, source)
            sample = source.sample(spot(3.5)); sample.update(invalid)
            self.assertEqual(tracker.feed(sample), (0,0))
            self.assertFalse(tracker.contact)
        tracker.expire(source.time+1)
        self.assertFalse(tracker.contact)

    def test_fractional_motion_preserves_slow_and_negative_motion(self):
        motion = FractionalMotion()
        self.assertEqual(sum(motion.consume(.125,0)[0] for _ in range(80)), 10)
        self.assertEqual(sum(motion.consume(-.125,0)[0] for _ in range(80)), -10)
        motion.consume(.9, .9)
        motion.reset()
        self.assertEqual(motion.consume(.2,.2), (0,0))

    def test_buffer_retains_all_sensor_frames_and_reports_overflow_by_sequence(self):
        buffer = AlgoDataBuffer(50)
        frame = dict(type='debug', device=1, channel_count=20, raw=BASE,
                     baseline=BASE, signal=[0]*20, state=[0]*20,
                     fast_k_percent=[0]*20, slow_k_percent=[0]*20)
        with patch('data_buffer.time.perf_counter', return_value=1):
            for _ in range(520):
                buffer.append_frame(frame)
            buffer.append_frame(dict(type='temperature',temperature_celsius=25))
        samples = buffer.pointer_samples_after(0)
        self.assertEqual(len(samples), 512)
        self.assertEqual(samples[0]['sequence'], 9)
        self.assertEqual(samples[-1]['sequence'], 520)
        self.assertEqual(len(buffer.pointer_samples_after(518)), 2)
        samples[-1]['raw'] = ()
        self.assertEqual(buffer.pointer_samples_after(519)[0]['raw'], tuple(BASE))


if __name__ == '__main__':
    unittest.main()
