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

    def sample(self, weights=None, state=2, signal=None, raw_only=False):
        self.sequence += 1
        self.time += 1/self.hz
        weights = [0]*20 if weights is None else weights
        return dict(sequence=self.sequence, sample_timestamp=self.time,
                    channel_count=20, has_raw=True,
                    raw=[a+b for a,b in zip(BASE, weights)],
                    signal=[0]*20 if raw_only else weights if signal is None else signal,
                    state=[-1 if raw_only else state]*20, raw_only=raw_only)


class TouchpadTests(unittest.TestCase):
    def test_raw_release_adjustable_drop_and_rise_tolerance(self):
        def response(main=4000, rise=0):
            values = [0]*20
            values[4] = values[12] = main
            values[19] = rise
            return values
        # 同样20%的下降，默认25不会停止，调至15后在本帧停止。
        for drop, should_stop in ((25, False), (15, True)):
            tracker,source = self.raw_ready()
            tracker.configure(300,1000,2.5,raw_release_drop_percent=drop)
            for _ in range(100):tracker.feed(source.sample(response(),raw_only=True))
            delta = tracker.feed(source.sample(response(3200),raw_only=True))
            self.assertEqual(tracker.release_blocked,should_stop)
            if should_stop:self.assertEqual(delta,(0,0))
        # 卸载中小路回升100超过噪声，但低于整体下降量2400的5%。
        for tolerance, should_stop in ((0,False),(5,True)):
            tracker,source = self.raw_ready()
            tracker.configure(300,1000,2.5,raw_release_rise_percent=tolerance)
            for _ in range(100):tracker.feed(source.sample(response(),raw_only=True))
            delta=tracker.feed(source.sample(response(2800,100),raw_only=True))
            self.assertEqual(tracker.release_blocked,should_stop)
            if should_stop:self.assertEqual(delta,(0,0))
        # 调高单路容差不旁路总回升限制；大量空间转移仍继续跟踪。
        tracker,source=self.raw_ready()
        tracker.configure(300,1000,2.5,raw_release_drop_percent=5,raw_release_rise_percent=20)
        for _ in range(100):tracker.feed(source.sample(response(),raw_only=True))
        tracker.feed(source.sample(response(2800,800),raw_only=True))
        self.assertFalse(tracker.release_blocked)
        # 两项Raw设置不改变普通Debug模式的真实状态门控。
        tracker,source=self.ready()
        tracker.configure(300,100,2.5,raw_release_drop_percent=5,raw_release_rise_percent=20)
        for _ in range(100):tracker.feed(source.sample(response()))
        tracker.feed(source.sample(response(2000)))
        self.assertFalse(tracker.release_blocked)

    def test_release_configuration_bounds_and_latched_tail(self):
        tracker=TouchpadTracker(raw_release_drop_percent=0,raw_release_rise_percent=100)
        self.assertEqual(tracker.raw_release_drop_percent,5)
        self.assertEqual(tracker.raw_release_rise_percent,20)
        tracker.configure(300,100,2.5,raw_release_drop_percent=100,raw_release_rise_percent=-1)
        self.assertEqual(tracker.raw_release_drop_percent,80)
        self.assertEqual(tracker.raw_release_rise_percent,0)
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):TouchpadTracker(raw_release_drop_percent=value)
        tracker,source=self.raw_ready()
        for _ in range(100):tracker.feed(source.sample(spot(amplitude=10000),raw_only=True))
        tracker.feed(source.sample(spot(amplitude=4000),raw_only=True))
        self.assertTrue(tracker.release_blocked)
        reference=tracker.reference
        tracker.configure(300,1000,2.5,raw_release_drop_percent=15,raw_release_rise_percent=5)
        self.assertEqual(tracker.reference,reference)
        self.assertTrue(tracker.release_blocked)
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(amplitude=4000),raw_only=True)),(0,0))
            self.assertTrue(tracker.release_blocked)

    def raw_ready(self, hz=100, gains=None):
        tracker, source = TouchpadTracker(threshold=1000), Source(hz)
        tracker.set_enabled(True)
        if gains is not None:
            tracker.set_raw_gains(gains)
        tracker.start_reference(force=True)
        for _ in range(2*hz):
            tracker.feed(source.sample(raw_only=True))
        return tracker, source

    def test_raw_unloading_latches_above_threshold_and_reanchors_fresh_touch(self):
        for hz in (70, 100, 200):
            with self.subTest(hz=hz):
                tracker, source = self.raw_ready(hz)
                def response(amplitude):
                    values = spot(4.1, 2.9, amplitude)
                    values[4] += 1400
                    return values
                for _ in range(hz):
                    tracker.feed(source.sample(response(10000), raw_only=True))
                self.assertTrue(tracker.contact)
                # 第一帧卸载仍远高于释放阈值，必须在中值窗、位置滤波前截断。
                frame = source.sample(response(2000), raw_only=True)
                original = list(frame['raw'])
                self.assertEqual(tracker.feed(frame), (0, 0))
                self.assertEqual(frame['raw'], original)
                self.assertEqual(frame['state'], [-1]*20)
                self.assertEqual(tracker.motion_block_reason, 'coherent_release_tail')
                self.assertFalse(tracker.contact)
                self.assertTrue(all(axis.filtered is None for axis in tracker.axes))
                self.assertIsNone(tracker.active_weight_share)
                for i in range(2*hz):
                    self.assertEqual(tracker.feed(source.sample(response(2000*math.exp(-i/hz)), raw_only=True)), (0, 0))
                    self.assertTrue(tracker.release_blocked)
                # 完全释放及低于进入阈值也不能清掉锁存，避免残余缓慢上升再度驱动。
                for _ in range(hz):
                    tracker.feed(source.sample(raw_only=True))
                for _ in range(hz):
                    self.assertEqual(tracker.feed(source.sample(response(0), raw_only=True)), (0, 0))
                for _ in range(hz):
                    self.assertEqual(tracker.feed(source.sample(spot(2.2, amplitude=8000), raw_only=True)), (0, 0))
                self.assertTrue(tracker.contact)
                self.assertFalse(tracker.release_blocked)
                moving = [tracker.feed(source.sample(spot(2.2+.3*(i+1)/hz, amplitude=8000), raw_only=True)) for i in range(hz)]
                self.assertGreater(sum(dx for dx,dy in moving), 70)

    def test_gain_scaled_reference_noise_threshold_and_centroid(self):
        for forced in (False, True):
            tracker, source = TouchpadTracker(threshold=100), Source()
            tracker.set_enabled(True)
            gains = [1.0]*20
            gains[5], gains[9], gains[19] = 2, .5, 0
            tracker.set_raw_gains(gains)
            tracker.start_reference(force=forced)
            for i in range(200):
                frame = source.sample([(-1)**i]*20, state=0, raw_only=forced)
                tracker.feed(frame)
            self.assertAlmostEqual(tracker.reference[5]/2, tracker.reference[9]/.5-4*123, delta=1)
            self.assertEqual(tracker.reference[19], 0)
            self.assertGreater(tracker.noise[5], tracker.noise[9])
            for _ in range(130):
                tracker.feed(source.sample(state=0, raw_only=forced))
            # 两通道原始响应相等，增益后4:1，质心应为(1.7,1.5)。
            values = [0]*20
            values[5] = values[9] = 1000
            values[19] = 50000
            for _ in range(100):
                frame = source.sample(values, raw_only=forced)
                tracker.feed(frame)
            self.assertAlmostEqual(tracker.raw_position[0], 1.7, delta=.01)
            self.assertAlmostEqual(tracker.raw_position[1], 1.5, delta=.01)
            self.assertEqual(frame['raw'][19], BASE[19]+50000)
        # 阈值同样检查乘增益后的Raw；原始差80乘2后能进入接触。
        tracker, source = self.raw_ready(gains=[2.0]*20)
        tracker.threshold = 100
        for _ in range(50):
            tracker.feed(source.sample([80]+[0]*19, raw_only=True))
        self.assertTrue(tracker.contact)

    def test_gain_changes_invalidate_contact_and_in_progress_reference(self):
        tracker, source = self.raw_ready()
        for _ in range(100):
            tracker.feed(source.sample(spot(amplitude=8000), raw_only=True))
        self.assertFalse(tracker.set_raw_gains([1]*20))
        self.assertTrue(tracker.contact)
        self.assertTrue(tracker.set_raw_gains([2]*20))
        self.assertIsNone(tracker.reference)
        self.assertFalse(tracker.contact)
        tracker.start_reference(force=True)
        for _ in range(50):
            tracker.feed(source.sample(raw_only=True))
        tracker.set_raw_gains([.5]*20)
        self.assertFalse(tracker.calibrating)
        self.assertEqual(len(tracker._calibration), 0)
        tracker.start_reference(force=True)
        for _ in range(150):
            tracker.feed(source.sample(raw_only=True))
        self.assertEqual(tracker.reference, tuple(v*.5 for v in BASE))
        tracker.set_raw_gains([0]*20)
        tracker.start_reference(force=True)
        for _ in range(150):
            tracker.feed(source.sample(raw_only=True))
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample([50000]*20, raw_only=True)), (0,0))
        self.assertFalse(tracker.contact)

    def test_raw_spatial_transfer_survives_unequal_gains_and_direction_reversal(self):
        gains = [8 if i//4 == 1 else .25 for i in range(20)]
        tracker, source = self.raw_ready(gains=gains)
        tracker.threshold = 100
        for _ in range(100):
            tracker.feed(source.sample(spot(2.0, amplitude=10000), raw_only=True))
        dx = []
        for i in range(400):
            delta = tracker.feed(source.sample(spot(2.0+.9*math.sin(i*.015), amplitude=10000), raw_only=True))
            dx.append(delta[0])
            self.assertFalse(tracker.release_blocked)
            self.assertTrue(tracker.contact)
        self.assertTrue(any(value > .1 for value in dx))
        self.assertTrue(any(value < -.1 for value in dx))

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

    def test_forced_reference_accepts_unknown_active_and_missing_states(self):
        for state in (-1, 2, 4, 6, None):
            tracker, source = TouchpadTracker(), Source()
            tracker.set_enabled(True)
            tracker.start_reference(force=True)
            for _ in range(130):
                frame = source.sample(state=state, signal=[500]*20)
                if state is None:
                    frame.pop('state')
                    frame.pop('signal')
                self.assertEqual(tracker.feed(frame), (0, 0))
            self.assertEqual(tracker.reference, tuple(BASE))
            self.assertEqual(tracker.reference_status['stage'], 'complete')
            self.assertTrue(tracker.reference_status['forced'])
            # 强制采集不改写固件状态，也不把后续滑动改为强制门控。
            unknown = source.sample(spot(2.5), state=-1, signal=[0]*20)
            self.assertEqual(tracker.feed(unknown), (0, 0))
            self.assertFalse(tracker.contact)

    def test_forced_reference_keeps_stability_validity_and_gap_checks(self):
        tracker, source = TouchpadTracker(), Source()
        tracker.set_enabled(True)
        tracker.start_reference(force=True)
        for i in range(140):
            tracker.feed(source.sample([i]*20, state=-1))
        self.assertIsNone(tracker.reference)
        self.assertEqual(tracker.reference_status['stage'], 'unstable')
        for invalid in (dict(has_raw=False), dict(channel_count=8), dict(raw=[float('nan')]*20)):
            frame = source.sample(state=-1)
            frame.update(invalid)
            tracker.feed(frame)
            self.assertEqual(tracker.reference_status['progress'], 0)
        for _ in range(50):
            tracker.feed(source.sample(state=-1))
        source.time += .3
        tracker.feed(source.sample(state=-1))
        self.assertEqual(tracker.reference_status['frames'], 1)
        for _ in range(80):
            tracker.feed(source.sample(state=-1))
        self.assertIsNone(tracker.reference)
        for _ in range(40):
            tracker.feed(source.sample(state=-1))
        self.assertEqual(tracker.reference, tuple(BASE))
        tracker.start_reference(force=True)
        tracker.cancel_reference()
        self.assertFalse(tracker.reference_forced)
        tracker.start_reference()
        for _ in range(130):
            tracker.feed(source.sample(state=-1))
        self.assertIsNone(tracker.reference)
        self.assertFalse(tracker.reference_status['forced'])

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

    def test_raw_only_forced_reference_moves_and_reanchors_after_release(self):
        tracker, source = TouchpadTracker(), Source()
        tracker.set_enabled(True)
        tracker.start_reference(force=True)
        for _ in range(130):
            tracker.feed(source.sample(raw_only=True))
        self.assertEqual(tracker.reference, tuple(BASE))
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(), raw_only=True)), (0, 0))
        self.assertTrue(tracker.contact)
        moving = [tracker.feed(source.sample(spot(2+.5*(i+1)/100), raw_only=True)) for i in range(100)]
        self.assertGreater(sum(dx for dx,dy in moving), 100)
        self.assertIsNone(tracker.active_weight_share)
        self.assertIn('Raw观察模式', tracker.message)
        # 原始帧没有Signal及状态，也无需伪造活动通道。
        release = source.sample(raw_only=True)
        self.assertEqual(release['state'], [-1]*20)
        self.assertEqual(tracker.feed(release), (0, 0))
        self.assertFalse(tracker.contact)
        self.assertTrue(all(axis.filtered is None for axis in tracker.axes))
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(3.5), raw_only=True)), (0, 0))
        self.assertTrue(tracker.contact)
        moving = [tracker.feed(source.sample(spot(3.5, 1.5+.5*(i+1)/100), raw_only=True)) for i in range(100)]
        self.assertGreater(sum(dy for dx,dy in moving), 100)
        self.assertEqual(tracker.reference, tuple(BASE))

    def test_raw_only_keeps_threshold_gap_and_mode_change_guards(self):
        tracker, source = self.ready()
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample([5]*20, raw_only=True)), (0, 0))
        self.assertFalse(tracker.contact)
        self.assertIn('低于接触阈值', tracker.message)
        for _ in range(100):
            tracker.feed(source.sample(spot(), raw_only=True))
        source.time += .3
        self.assertEqual(tracker.feed(source.sample(spot(3.5), raw_only=True)), (0, 0))
        self.assertFalse(tracker.contact)
        for _ in range(100):
            tracker.feed(source.sample(spot(3.5), raw_only=True))
        self.assertTrue(tracker.contact)
        self.assertEqual(tracker.feed(source.sample(spot(2), state=2)), (0, 0))
        self.assertFalse(tracker.contact)
        self.assertEqual(tracker.motion_block_reason, 'source_mode_changed')
        # 普通Debug缺少有效状态仍不得驱动，不把强制参考采集当成永久运动旁路。
        for _ in range(100):
            self.assertEqual(tracker.feed(source.sample(spot(2.5), state=-1, signal=[0]*20)), (0, 0))
        self.assertFalse(tracker.contact)

    def test_buffer_preserves_raw_only_source_and_clears_it_for_normal_samples(self):
        from frame_parser import AlgoFrameParser
        import struct
        wire = struct.pack('<4sBBBBH20H', b'DATA', 21, 0x16, 1, 20, 40, *BASE)
        frame = AlgoFrameParser().feed(wire)[0]
        buffer = AlgoDataBuffer(50)
        buffer.append_frame(frame)
        self.assertTrue(buffer.latest_sensor_sample()['raw_only'])
        self.assertTrue(buffer.pointer_samples_after(0)[-1]['raw_only'])
        self.assertEqual(buffer.pointer_samples_after(0)[-1]['state'], (-1,)*20)
        buffer.append_frame(dict(frame, raw_only=False))
        self.assertFalse(buffer.latest_sensor_sample()['raw_only'])
        self.assertFalse(buffer.pointer_samples_after(0)[-1]['raw_only'])
        buffer.append_frame(dict(type='signal', device=1, channel_count=20, signal=[0]*20))
        self.assertFalse(buffer.latest_sensor_sample()['raw_only'])

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
