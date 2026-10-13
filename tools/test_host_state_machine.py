"""按实际时间、逐采样验证上位机实验基线及释放状态。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'PY_SRIPT/algo_monitor'))
from host_state_machine import HostParameters, HostStateMachine, HOST_STATE_IDLE_NEGATIVE, HOST_STATE_RELEASE_UNDERSHOOT, HOST_NEGATIVE_STATES


class Stream:
    def __init__(self, parameters=None, hz=100):
        self.model = HostStateMachine(parameters)
        self.sequence = 0
        self.time = 0.
        self.dt = 1 / hz

    def feed(self, value=10000, **overrides):
        self.sequence += 1
        self.time += self.dt
        sample = dict(sequence=self.sequence, sample_timestamp=self.time, has_raw=True,
                      channel_count=20, raw=[value] * 20)
        sample.update(overrides)
        return self.model.feed(sample)

    def repeat(self, value, frames):
        for _ in range(frames):
            self.feed(value)

    def initialize(self):
        self.repeat(10000, math.ceil(self.model.parameters.initial_seconds / self.dt) + 3)
        assert self.model.channels[0].state == 0


class HostStateTests(unittest.TestCase):
    def test_recovery_platform_never_raises_original_reference(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2,
                           drop_frames=1, slowback_frames=1))
        s.initialize()
        s.repeat(14000, 10)
        c = s.model.channels[0]
        s.feed(11000)
        s.feed(11000)
        s.feed(11000)
        self.assertGreater(c.baseline, 10000)
        self.assertEqual(c.release_reference, 10000)
        s.repeat(8000, 4)
        self.assertEqual(c.state, 3)
        self.assertEqual(c.release_reference, 10000)
        self.assertEqual(c.release_reference, 10000)

    def test_negative_press_parameters_and_continuous_confirmation(self):
        p = replace(HostParameters(), input_alpha=1., window=2,
                    negative_press_margin=600., negative_press_rate=40., negative_press_frames=2)
        s = Stream(p)
        s.initialize()
        s.repeat(8000, 10)
        c = s.model.channels[0]
        # 快速上涨但幅度不足；幅度足够但速度不足，都不能激活。
        s.feed(10500)
        self.assertEqual(c.reload_count, 0)
        s.repeat(10500, 5)
        for v in (10610, 10620, 10630):
            s.feed(v)
            self.assertEqual(c.state, 3)
        s.feed(11500)
        self.assertEqual(c.reload_count, 1)
        s.feed(11500)
        self.assertEqual(c.reload_count, 0)
        s.feed(12500)
        f = s.feed(13500)
        self.assertEqual(c.state, 1)
        event = f['activation_events'][0]
        self.assertEqual(event['negative_press_margin'], 600)
        self.assertEqual(event['negative_press_rate'], 40)
        self.assertEqual(event['required_frames'], 2)
        self.assertEqual(event['confirmed_frames'], 2)

    def test_positive_residual_platform_can_settle_without_slow_activation(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2))
        s.initialize()
        s.repeat(8000, 10)
        for v in range(8000, 11001, 10):
            s.feed(v)
        s.repeat(11000, 600)
        c = s.model.channels[0]
        self.assertEqual(c.state, 0)
        self.assertEqual(c.baseline, 11000)
        self.assertEqual(c.activation_number, 0)

    def test_release_undershoot_then_slow_rebound_does_not_latch_active(self):
        for hz in (70, 100, 200):
            s = Stream(replace(HostParameters(), input_alpha=1., window=2,
                               drop_frames=1, slowback_frames=1), hz)
            s.initialize()
            s.repeat(14000, 10)
            c = s.model.channels[0]
            self.assertEqual(c.release_reference, 10000)
            s.repeat(7500, 20)
            self.assertEqual(c.state, 3)
            self.assertEqual(c.release_reference, 10000)
            # 下冲后缓慢回升到原零点以下，不能制造假 Signal。
            for i in range(round(hz*3)):
                frame = s.feed(7500 + 2400*i/(hz*3))
                self.assertNotEqual(c.state, 1)
                self.assertEqual(frame['signal'][0], 0)
                self.assertEqual(c.release_reference, 10000)
            s.repeat(10000, 40)
            self.assertEqual(c.state, 0)
            self.assertIsNone(c.release_reference)

    def test_small_recover_undershoot_cannot_clear_original_reference(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2,
                           drop_frames=1, slowback_frames=1))
        s.initialize()
        s.repeat(14000, 10)
        s.repeat(9900, 80)
        c = s.model.channels[0]
        self.assertEqual(c.state, 3)
        self.assertEqual(c.release_reference, 10000)
        self.assertEqual(c.release_reference, 10000)
        s.repeat(10000, 20)
        self.assertEqual(c.state, 0)

    def test_negative_recovery_rejects_slow_rise_and_keeps_fast_long_hold(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2,
                           drop_frames=1, slowback_frames=1))
        s.initialize()
        s.repeat(14000, 10)
        s.repeat(8000, 20)
        c = s.model.channels[0]
        self.assertEqual(c.state, 3)
        for i in range(300):
            s.feed(8000 + 10*i)
        self.assertEqual(c.state, 3)
        self.assertEqual(c.signal, 0)
        for value in (13000, 14000, 15000):
            s.feed(value)
        self.assertEqual(c.state, 1)
        self.assertEqual(c.release_reference, 10000)
        self.assertIn(c.last_activation['reason'], ('reload', 'negative_recovery_press'))
        s.repeat(15000, 1200)
        self.assertEqual(c.state, 1)
        self.assertEqual(c.signal, 5000)
        s.repeat(8000, 20)
        self.assertEqual(c.release_reference, 10000)
        self.assertEqual(c.release_reference, 10000)

    def test_negative_recovery_freezes_candidate_and_repeated_rebounds(self):
        s = Stream(replace(HostParameters(), input_alpha=1.))
        s.initialize()
        s.feed(8000)
        self.assertEqual(s.model.channels[0].baseline, 10000)
        self.assertEqual(s.model.channels[0].state, 0)
        s.repeat(8000, 3)
        self.assertEqual(s.model.channels[0].state, HOST_STATE_IDLE_NEGATIVE)
        for value in [8000]*30 + [9700]*40 + [7500]*20 + [9900]*40:
            f = s.feed(value)
            self.assertNotEqual(f['state'][0], 1)
            self.assertEqual(f['signal'][0], 0)
            self.assertEqual(f['release_reference'][0], 10000)
        s.repeat(10000, 40)
        self.assertEqual(s.model.channels[0].state, 0)
        self.assertIsNone(s.model.channels[0].release_reference)

    def test_negative_recovery_real_press_and_other_channel(self):
        s = Stream(replace(HostParameters(), input_alpha=1.))
        s.initialize()
        for _ in range(30):
            raw = [10000]*20
            raw[8], raw[11] = 8000, 14000
            s.feed(raw=raw)
        self.assertEqual(s.model.channels[8].state, 3)
        self.assertEqual(s.model.channels[11].state, 1)
        for _ in range(3):
            raw[8] = 10500
            f = s.feed(raw=raw)
        self.assertEqual(f['state'][8], 1)
        self.assertEqual(f['signal'][8], 500)
        event = next(e for e in f['activation_events'] if e['channel'] == 8)
        self.assertEqual(event['reason'], 'negative_recovery_press')
        self.assertEqual(event['negative_reference'], 10000)
        self.assertEqual(event['from_state'], 3)

    def test_negative_rebase_requires_continuous_narrow_platform(self):
        for hz in (70, 100, 200):
            s = Stream(replace(HostParameters(), input_alpha=1.), hz=hz)
            s.initialize()
            s.repeat(8000, round(hz*2))
            self.assertEqual(s.model.channels[0].release_reference, 10000)
            # 低速持续回升也不能被当成稳定低平台。
            for i in range(round(hz*6)):
                s.feed(8000 + 10*i/hz)
                self.assertEqual(s.model.channels[0].release_reference, 10000)
            s.repeat(8060, round(hz*6))
            self.assertEqual(s.model.channels[0].state, 0)
            self.assertAlmostEqual(s.model.channels[0].baseline, 8060)

    def test_negative_gap_and_disable_clear_reference(self):
        s = Stream(replace(HostParameters(), input_alpha=1.))
        s.initialize()
        s.repeat(8000, 30)
        self.assertEqual(s.model.channels[0].release_reference, 10000)
        s.time += .3
        f = s.feed(8000)
        self.assertEqual(f['state'][0], -1)
        self.assertIsNone(f['release_reference'][0])
        s.model.set_gains([0.] + [1.]*19)
        f = s.feed(8000)
        self.assertEqual(f['state'][0], -2)
        self.assertIsNone(f['release_reference'][0])

    def test_signal_lowpass_preserves_state_and_tracks_each_sample(self):
        streams = [Stream(replace(HostParameters(), signal_alpha=alpha)) for alpha in (.1, 1.)]
        previous = [0.] * 20
        for value in [10000] * 305 + [14000] * 50 + [8000] * 40 + [16000] * 50:
            filtered, original = [s.feed(value) for s in streams]
            for key in ('input_val', 'baseline', 'signal', 'state', 'activation_events', 'release_reference'):
                self.assertEqual(filtered[key], original[key])
            self.assertEqual(original['signal_filtered'], original['signal'])
            for index in range(20):
                expected = previous[index] + .1 * (filtered['signal'][index] - previous[index])
                self.assertAlmostEqual(filtered['signal_filtered'][index], expected)
            previous = filtered['signal_filtered']
        s = streams[0]
        self.assertEqual(s.model.histories['signal_filtered'][0][-1], previous[0])
        version = s.model.version
        duplicate = dict(sequence=s.sequence, sample_timestamp=s.time, has_raw=True,
                         channel_count=20, raw=[20000] * 20)
        self.assertIsNone(s.model.feed(duplicate))
        self.assertEqual(s.model.version, version)
        self.assertEqual(s.model.channels[0].signal_filtered, previous[0])
        s.model.reset()
        self.assertEqual(s.model.channels[0].signal_filtered, 0)
        self.assertFalse(s.model.histories['signal_filtered'][0])

    def test_signal_lowpass_release_tail_disabled_and_parameter_validation(self):
        s = Stream(replace(HostParameters(), input_alpha=1., signal_alpha=.25, release_frames=1))
        s.model.set_gains([1., 0.] + [1.] * 18)
        s.initialize()
        s.repeat(14000, 40)
        previous = s.model.channels[0].signal_filtered
        frame = s.feed(10000)
        self.assertEqual(frame['signal'][0], 0)
        self.assertNotEqual(frame['state'][0], 1)
        self.assertAlmostEqual(frame['signal_filtered'][0], previous * .75)
        self.assertEqual(frame['signal_filtered'][1], 0)
        for invalid in (0., -1., 1.01, math.nan):
            with self.assertRaises(ValueError):
                replace(HostParameters(), signal_alpha=invalid)
        self.assertEqual(HostParameters.from_settings({'input_alpha': .1}).signal_alpha, .1)
        s.time += .3
        frame = s.feed()
        self.assertEqual(frame['signal_filtered'], [0.] * 20)

    def test_gain_before_lowpass_and_initial_max(self):
        p = replace(HostParameters(), initial_seconds=.05)
        stream = Stream(p)
        stream.model.set_gains([2., 0.] + [1.] * 18)
        stream.feed(10000)
        stream.feed(11000)
        self.assertEqual(stream.model.channels[0].input, 20400.)
        self.assertEqual(stream.model.channels[0].baseline, 20400.)
        stream.repeat(9000, 8)
        self.assertEqual(stream.model.channels[0].state, HOST_STATE_IDLE_NEGATIVE)
        self.assertEqual(stream.model.channels[0].baseline, 20400.)
        stream.repeat(9000, 600)
        self.assertEqual(stream.model.channels[0].state, 0)
        self.assertLess(stream.model.channels[0].baseline, 20400.)
        self.assertEqual(stream.model.channels[1].state, -2)
        self.assertEqual(stream.model.channels[1].signal, 0)
        self.assertTrue(math.isnan(stream.model.channels[1].rate))

    def test_no_activation_on_raw_offset_and_channel_independence(self):
        s = Stream()
        s.initialize()
        self.assertTrue(all(c.state == 0 for c in s.model.channels))
        p = s.model.parameters
        for _ in range(p.activate_frames + 2):
            s.feed(raw=[13000] + [10000] * 19)
        self.assertEqual(s.model.channels[0].state, 1)
        self.assertGreater(s.model.channels[0].signal, 200)
        self.assertTrue(all(c.state == 0 and c.signal == 0 for c in s.model.channels[1:]))
        baseline = s.model.channels[0].baseline
        s.repeat(13000, 40)
        self.assertEqual(s.model.channels[0].baseline, baseline)

    def test_time_based_rate_multiple_sample_rates(self):
        for hz in (70, 100, 200):
            with self.subTest(hz=hz):
                s = Stream(replace(HostParameters(), input_alpha=1., drop_rate=1000.), hz)
                s.initialize()
                s.repeat(13000, 40)
                for _ in range(50):
                    s.feed(13000 - 1000 * (_ + 1) / hz)
                c = s.model.channels[0]
                self.assertEqual(c.state, 1)
                self.assertAlmostEqual(c.rate, -100000 / c.input, places=7)

    def test_fast_release_zero_and_exact_hold_and_recover(self):
        # 延长提前恢复确认，单独验证最长等待仍精确按采样帧兜底。
        s = Stream(replace(HostParameters(), slowback_close_frames=1000))
        s.initialize()
        s.repeat(14000, 40)
        baseline = s.model.channels[0].baseline
        while s.model.channels[0].state == 1:
            s.feed(10000)
        c = s.model.channels[0]
        self.assertEqual(c.state, 2)
        self.assertEqual(c.signal, 0)
        self.assertEqual(c.baseline, baseline)
        s.repeat(10000, 299)
        self.assertEqual(c.state, 2)
        self.assertEqual(c.baseline, baseline)
        s.feed(10000)
        self.assertEqual(c.state, 3)
        self.assertEqual(c.baseline, baseline)
        for _ in range(100):
            s.feed(10000)
            self.assertEqual(c.signal, 0)
        self.assertEqual(c.state, 0)
        self.assertLess(abs(c.baseline - c.input), s.model.parameters.recover_tolerance)

    def test_realtime_rate_once_per_sample_in_idle_active_and_recovery(self):
        s = Stream(replace(HostParameters(), slowback_frames=3))
        s.initialize()
        c = s.model.channels[0]
        with patch.object(c, 'slope_rate', wraps=c.slope_rate) as regression:
            s.repeat(10000, 30)
            self.assertEqual(regression.call_count, 30)
            self.assertEqual(len(c.history), s.model.parameters.window)
            self.assertEqual(c.rate, 0.)
            self.assertTrue(math.isnan(c.decision_rate))
            regression.reset_mock()
            for index in range(20):
                frame = s.feed(14000)
                if c.state == 1:
                    break
            self.assertEqual(c.state, 1)
            self.assertEqual(regression.call_count, index + 1)
            self.assertTrue(math.isfinite(c.rate))
            self.assertEqual(frame['pressure_rate'][0], c.decision_rate)
        s.repeat(14000, 30)
        while c.state == 1:
            frame = s.feed(8000)
        self.assertEqual(c.state, 2)
        self.assertTrue(math.isfinite(frame['pressure_rate'][0]))
        self.assertTrue(math.isfinite(frame['decision_rate'][0]))
        for _ in range(20):
            frame = s.feed(8000)
            self.assertIn(c.state, (2, 3, *HOST_NEGATIVE_STATES))
            self.assertTrue(math.isfinite(c.rate))
            self.assertEqual(s.model.histories['fast'][0][-1], c.rate)
            self.assertTrue(math.isfinite(frame['pressure_rate'][0]))
            self.assertTrue(math.isfinite(frame['decision_rate'][0]))

    def test_realtime_rate_during_initialization_and_irregular_idle_sampling(self):
        s = Stream(replace(HostParameters(), input_alpha=1., activate_threshold=10000.))
        for index in range(25):
            frame = s.feed(10000)
            self.assertEqual(frame['state'][0], -1)
            if index < 19:
                self.assertTrue(math.isnan(frame['pressure_rate'][0]))
            else:
                self.assertEqual(frame['pressure_rate'][0], 0.)
        s.initialize()
        started = s.time
        for index in range(40):
            interval = (.003, .008, .005, .011)[index % 4]
            s.time += interval - s.dt
            frame = s.feed(10000 + 250 * (s.time + s.dt - started))
        self.assertEqual(frame['state'][0], 0)
        self.assertAlmostEqual(frame['pressure_rate'][0], 100 * 250 / frame['input_val'][0], places=8)
        self.assertTrue(math.isnan(frame['decision_rate'][0]))

    def test_slowback_early_recover_relative_band_both_sides_and_gains(self):
        for gain in (1., 2.):
            for target, outside in ((11000, 11100), (9000, 12000)):
                with self.subTest(gain=gain, target=target):
                    s = Stream(replace(HostParameters(), input_alpha=1., window=2, drop_frames=1, negative_threshold=5000))
                    s.model.set_gains([gain] * 20)
                    s.initialize()
                    s.repeat(16000, 5)
                    s.feed(12000)
                    c = s.model.channels[0]
                    baseline = c.baseline
                    self.assertEqual(c.state, 2)
                    self.assertEqual(c.close_count, 0)
                    s.repeat(target, 2)
                    self.assertEqual(c.state, 2)
                    self.assertEqual(c.close_count, 2)
                    s.feed(outside)
                    self.assertEqual(c.close_count, 0)
                    s.repeat(target, 2)
                    self.assertEqual(c.state, 2)
                    s.feed(target)
                    self.assertEqual(c.state, 3)
                    self.assertEqual(c.baseline, baseline)
                    self.assertEqual(c.close_count, 0)
                    self.assertEqual(c.signal, 0)
                    s.feed(target)
                    self.assertNotEqual(c.baseline, baseline)

    def test_slowback_custom_tolerance_and_confirmation(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2, drop_frames=1,
                           slowback_tolerance_percent=2., slowback_close_frames=2))
        s.initialize()
        s.repeat(16000, 5)
        s.feed(12000)
        c = s.model.channels[0]
        s.repeat(10900, 10)
        self.assertEqual(c.state, 2)
        self.assertEqual(c.close_count, 0)
        s.feed(10100)
        self.assertEqual(c.state, 2)
        s.feed(10100)
        self.assertEqual(c.state, 3)

    def test_slowback_reload_candidates_take_priority_over_close_input(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2, drop_frames=1,
                           slowback_close_frames=1))
        s.initialize()
        s.repeat(16000, 5)
        s.feed(8000)
        c = s.model.channels[0]
        baseline = c.baseline
        for value in (10500, 10600):
            s.feed(value)
            self.assertEqual(c.state, 2)
            self.assertEqual(c.close_count, 0)
            self.assertEqual(c.baseline, baseline)
        s.feed(10700)
        self.assertEqual(c.state, 1)
        self.assertEqual(c.baseline, baseline)
        self.assertEqual(c.last_activation['from_state'], 2)

    def test_slowback_zero_baseline_needs_exact_match(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2, drop_frames=1,
                           slowback_close_frames=2))
        s.repeat(0, 310)
        s.repeat(1000, 5)
        s.feed(10)
        c = s.model.channels[0]
        self.assertEqual(c.state, 2)
        self.assertEqual(c.baseline, 0)
        s.repeat(10, 3)
        self.assertEqual(c.state, 2)
        s.feed(0)
        self.assertEqual(c.state, 2)
        s.feed(0)
        self.assertEqual(c.state, 3)

    def test_old_settings_keep_existing_parameters_and_default_new_options(self):
        old = dict(drop_rate=15., slowback_frames=120, reload_threshold=800.)
        p = HostParameters.from_settings(old)
        self.assertEqual(p.drop_rate, 15.)
        self.assertEqual(p.slowback_frames, 120)
        self.assertEqual(p.reload_threshold, 800.)
        self.assertEqual(p.slowback_tolerance_percent, 10.)
        self.assertEqual(p.slowback_close_frames, 3)
        self.assertEqual(p.reload_reference_margin, 300.)

    def test_undershoot_rebound_preserves_reference_at_multiple_rates_and_gains(self):
        for hz in (70, 100, 200):
            for gain in (1., 2.):
                with self.subTest(hz=hz, gain=gain):
                    s = Stream(replace(HostParameters(), slowback_frames=1), hz)
                    s.model.set_gains([gain]*20)
                    s.initialize()
                    s.repeat(14000, 40)
                    c = s.model.channels[0]
                    original = c.baseline
                    s.repeat(6000, 50)
                    self.assertIn(c.state, (3, HOST_STATE_RELEASE_UNDERSHOOT))
                    self.assertEqual(c.release_reference, original)
                    self.assertEqual(c.release_reference, original)
                    for _ in range(120):
                        frame = s.feed(10000)
                        self.assertIn(c.state, (0, 2, 3, HOST_STATE_RELEASE_UNDERSHOOT))
                        self.assertEqual(c.signal, 0)
                    self.assertEqual(c.state, 0)
                    self.assertIsNone(c.release_reference)

    def test_reference_margin_boundary_real_reload_and_repeated_unload(self):
        for margin in (200., 800.):
            with self.subTest(margin=margin):
                p = replace(HostParameters(), input_alpha=1., window=2, drop_frames=1,
                            negative_threshold=10000, slowback_frames=1, reload_threshold=100., reload_reference_margin=margin)
                s = Stream(p)
                s.initialize()
                s.repeat(14000, 5)
                c = s.model.channels[0]
                reference = c.baseline
                s.feed(6000)
                s.feed(6000)
                s.repeat(6000, 5)
                self.assertEqual(c.state, 3)
                frozen = c.baseline
                s.feed(reference+margin)
                self.assertEqual(c.state, 3)
                self.assertEqual(c.reload_count, 0)
                self.assertEqual(c.baseline, frozen)
                for increment in (100., 200., 300.):
                    s.feed(reference+margin+increment)
                    self.assertEqual(c.baseline, reference if c.state == 1 else frozen)
                self.assertEqual(c.state, 1)
                evidence = c.last_activation
                self.assertEqual(evidence['release_reference'], reference)
                self.assertEqual(evidence['reload_reference_margin'], margin)
                self.assertGreater(evidence['reference_difference'], margin)
                self.assertEqual(evidence['confirmed_frames'], p.reload_frames)
                # 再次卸载时新 baseline 较低，原释放参考仍须保留，不能被恢复下冲降低。
                s.feed(6000)
                self.assertEqual(c.state, 2)
                self.assertEqual(c.release_reference, reference)
                s.model.reset()
                self.assertIsNone(s.model.channels[0].release_reference)

    def test_zero_reload_margin_does_not_disable_negative_recovery(self):
        s = Stream(replace(HostParameters(), input_alpha=1., window=2, drop_frames=1,
                           slowback_frames=1, reload_threshold=100., reload_reference_margin=0.))
        s.initialize()
        s.repeat(14000, 5)
        c = s.model.channels[0]
        s.repeat(6000, 10)
        for value in (9500, 9600, 9700):
            s.feed(value)
        self.assertEqual(c.state, 3)
        self.assertEqual(c.release_reference, 10000)
        self.assertEqual(c.signal, 0)

    def test_rate_uses_irregular_timestamps(self):
        s = Stream(replace(HostParameters(), input_alpha=1., drop_rate=1000.))
        s.initialize()
        s.repeat(13000, 40)
        started = s.time
        for index in range(50):
            s.dt = (.004, .016, .007, .023)[index % 4]
            value = 13000 - 1200 * (s.time + s.dt - started)
            s.feed(value)
        c = s.model.channels[0]
        self.assertAlmostEqual(c.rate, -120000 / c.input, places=7)

    def test_slow_release_without_fast_rate(self):
        s = Stream(replace(HostParameters(), input_alpha=1.))
        s.initialize()
        s.repeat(11000, 30)
        # 每秒下降100 count，仅约1%/s，低于快速卸载门限。
        for index in range(960):
            s.feed(11000 - index)
            if s.model.channels[0].state != 1:
                break
        self.assertEqual(s.model.channels[0].state, 2)
        self.assertEqual(s.model.channels[0].signal, 0)

    def test_reload_in_both_hold_and_recover(self):
        for state in (2, 3):
            with self.subTest(state=state):
                s = Stream(replace(HostParameters(), slowback_frames=30, negative_threshold=10000))
                s.initialize()
                s.repeat(14000, 40)
                while s.model.channels[0].state == 1:
                    s.feed(8000)
                if state == 3:
                    s.repeat(8000, 30)
                c = s.model.channels[0]
                self.assertEqual(c.state, state)
                frozen = None
                for _ in range(30):
                    s.feed(16000)
                    if c.decision_rate >= s.model.parameters.reload_rate and frozen is None:
                        frozen = c.baseline
                    if frozen is not None:
                        self.assertEqual(c.baseline, max(frozen, c.release_reference) if c.state == 1 else frozen)
                    if c.state == 1:
                        break
                self.assertEqual(c.state, 1)
                self.assertGreater(c.signal, 0)
                evidence = c.last_activation
                self.assertEqual(evidence['reason'], 'reload')
                self.assertEqual(evidence['from_state'], state)
                self.assertEqual(evidence['activation_number'], 2)
                self.assertEqual(evidence['confirmed_frames'], 3)
                self.assertGreater(evidence['increase'], evidence['reload_threshold'])
                self.assertGreaterEqual(evidence['pressure_rate'], evidence['reload_rate'])
                self.assertAlmostEqual(evidence['increase'], evidence['input_val'] - evidence['trough'])
                self.assertLess(evidence['trough'], c.trough)

    def test_activation_evidence_is_trigger_snapshot_and_logged_once(self):
        s = Stream()
        s.initialize()
        events = []
        for _ in range(40):
            result = s.feed(14000)
            events.extend(result['activation_events'])
        c = s.model.channels[0]
        saved = dict(c.last_activation)
        self.assertEqual(len(events), 20)
        self.assertEqual(events[0], dict(channel=0, **saved))
        self.assertEqual(saved['reason'], 'idle_threshold')
        self.assertEqual(saved['from_state'], 0)
        self.assertEqual(saved['confirmed_frames'], saved['required_frames'])
        self.assertEqual(saved['required_frames'], 3)
        self.assertGreater(saved['difference'], saved['activate_threshold'])
        self.assertNotEqual(saved['input_val'], c.input)
        while c.state == 1:
            result = s.feed(8000)
            self.assertEqual(result['activation_events'], [])
        self.assertEqual(c.last_activation, saved)
        s.repeat(8000, 650)
        self.assertEqual(c.state, 0)
        self.assertEqual(c.last_activation, saved)
        self.assertNotEqual(c.baseline, saved['baseline'])
        s.model.reset()
        self.assertIsNone(s.model.channels[0].last_activation)
        self.assertEqual(events[0], dict(channel=0, **saved))

    def test_recover_requires_stability_not_only_proximity(self):
        s = Stream(replace(HostParameters(), recover_alpha=1., slowback_frames=1,
                           reload_rate=100000., stable_rate=.001, negative_threshold=10000))
        s.initialize()
        s.repeat(14000, 40)
        while s.model.channels[0].state == 1:
            s.feed(8000)
        s.feed(8000)
        for index in range(40):
            s.feed(8000 + index * 10)
            self.assertEqual(s.model.channels[0].state, 3)
            self.assertEqual(s.model.channels[0].baseline, s.model.channels[0].input)

    def test_duplicate_gap_time_invalid_and_run_reset(self):
        s = Stream()
        s.initialize()
        sequence = s.model.last_sequence
        version = s.model.version
        self.assertIsNone(s.model.feed(dict(sequence=sequence)))
        self.assertEqual(s.model.version, version)
        s.sequence += 2
        s.feed()
        self.assertEqual(s.model.channels[0].state, -1)
        self.assertIn('缺口', s.model.reset_reason)
        s.model.expire(s.time + 1)
        self.assertIsNone(s.model.last_timestamp)
        self.assertEqual(s.model.last_sequence, s.sequence)
        s.feed(has_raw=False, raw=[])
        self.assertIn('Run', s.model.reason)
        s.feed(raw=[math.nan] * 20)
        self.assertFalse(s.model.histories['raw'][0])
        s.feed(sample_timestamp=s.time - 2)
        s.feed(sample_timestamp=s.time - 3)
        self.assertIn('时间异常', s.model.reset_reason)

    def test_gain_and_parameter_reset_bounded_history(self):
        s = Stream()
        s.initialize()
        s.model.set_history_length(10)
        self.assertEqual(len(s.model.histories['raw'][0]), 10)
        s.model.set_gains([2.] * 20)
        self.assertIsNone(s.model.channels[0].started)
        self.assertFalse(s.model.histories['raw'][0])
        s.feed()
        self.assertEqual(s.model.channels[0].input, 20000)
        s.model.configure(replace(s.model.parameters, window=50))
        self.assertEqual(s.model.channels[0].history.maxlen, 50)

    def test_invalid_parameters(self):
        for change in (dict(input_alpha=math.nan), dict(window=1), dict(drop_frames=2.5),
                       dict(release_threshold=200), dict(recover_alpha=.001),
                       dict(slowback_tolerance_percent=-1), dict(slowback_close_frames=0),
                       dict(reload_reference_margin=-1), dict(reload_reference_margin=math.nan)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                replace(HostParameters(), **change)


class NegativeStateRoutingTests(unittest.TestCase):
    def stream(self):
        stream = Stream(replace(HostParameters(), input_alpha=1., window=4,
                                negative_frames=2, recover_frames=3, drop_frames=1))
        stream.initialize()
        return stream

    def test_idle_disturbance_is_separate_from_other_channel_press(self):
        stream = self.stream()
        for value in (9500, 9000, 8500):
            frame = stream.feed(raw=[value, 14000] + [10000]*18)
        self.assertEqual(frame['state'][0], HOST_STATE_IDLE_NEGATIVE)
        self.assertEqual(frame['state'][1], 1)
        self.assertEqual(frame['recovery_source'][0], 'idle')
        self.assertEqual(frame['state_schema_version'], 2)
        stream.repeat(8500, 20)
        self.assertEqual(stream.model.channels[0].state, 3)
        self.assertEqual(stream.model.channels[0].release_reference, 10000)

    def test_release_undershoot_bounces_to_slowback_then_recover(self):
        stream = self.stream()
        stream.repeat(14000, 20)
        c = stream.model.channels[0]
        states=[]
        for value in (9500, 9000, 8500, 8000):
            states.append(stream.feed(value)['state'][0])
        self.assertIn(HOST_STATE_RELEASE_UNDERSHOOT, states)
        self.assertNotIn(HOST_STATE_IDLE_NEGATIVE, states)
        self.assertEqual(c.recovery_source, 'release')
        for value in (8200, 8400, 8600, 8800, 9000, 9200, 9400):
            states.append(stream.feed(value)['state'][0])
            self.assertEqual(c.signal, 0)
            self.assertEqual(c.release_reference, 10000)
        self.assertIn(2, states[-5:])
        stream.repeat(9400, 40)
        self.assertEqual(c.state, 3)
        self.assertLess(abs(c.baseline-9400), 20)
        self.assertEqual(c.release_reference, 10000)

    def test_low_platform_and_gentle_drift_do_not_reenter_negative(self):
        for origin in ('idle','release'):
            stream=self.stream()
            if origin=='release':
                stream.repeat(14000, 20)
            for value in (9500, 9000, 8500, 8000):
                stream.feed(value)
            stream.repeat(8000, 50)
            c=stream.model.channels[0]
            self.assertEqual(c.state, 3)
            for index in range(150):
                stream.feed(8000-.1*index)
                self.assertEqual(c.state, 3)
                self.assertEqual(c.release_reference, 10000)
                self.assertEqual(c.signal, 0)
            # 第二次下降需要新的局部增量与下降沿，来源仍沿用本轮真实来源。
            states=[]
            for value in (7700, 7400, 7100, 6800):
                states.append(stream.feed(value)['state'][0])
            expected=HOST_STATE_IDLE_NEGATIVE if origin=='idle' else HOST_STATE_RELEASE_UNDERSHOOT
            self.assertIn(expected, states)

    def test_exit_confirmation_and_near_zero_direct_idle(self):
        stream=Stream(replace(HostParameters(), input_alpha=1., window=2,
                             negative_frames=2, recover_frames=3))
        stream.initialize()
        stream.feed(9500)
        stream.feed(9000)
        c=stream.model.channels[0]
        self.assertEqual(c.state, HOST_STATE_IDLE_NEGATIVE)
        stream.repeat(9000, 2)
        self.assertEqual(c.state, HOST_STATE_IDLE_NEGATIVE)
        stream.feed(9000)
        self.assertEqual(c.state, 3)
        stream.repeat(10000, 15)
        self.assertEqual(c.state, 0)
        self.assertIsNone(c.release_reference)
        self.assertIsNone(c.recovery_source)

class ChannelRepressTests(unittest.TestCase):
    def test_restore_and_validation(self):
        p = HostParameters(channel_repress={'4': {'margin': 450, 'confirm_frames': 10}})
        self.assertEqual(HostParameters.from_settings(p.settings()), p)
        for count in (0, 1.5, 10001):
            with self.assertRaises(ValueError):
                HostParameters(channel_repress={'4': {'margin':300, 'confirm_frames':count}})

    def test_all_entries_confirm_exact_frames_at_different_rates(self):
        for state in (2, 3, *HOST_NEGATIVE_STATES):
            for hz in (100, 200):
                with self.subTest(state=state, hz=hz):
                    p = HostParameters(input_alpha=1, window=2,
                        channel_repress={'4': {'margin':300, 'confirm_frames':10}})
                    stream = Stream(p, hz)
                    stream.initialize()
                    for c in stream.model.channels:
                        c.commit_state(state)
                        c.release_reference = c.baseline = c.trough = 10000
                    for i in range(30):
                        stream.feed(10400 + 4000*i/hz)
                        c = stream.model.channels[4]
                        if c.state == 1:
                            self.assertEqual(c.last_activation['confirmed_frames'], 10)
                            self.assertEqual(c.last_activation['required_frames'], 10)
                            break
                    else:
                        self.fail('未按连续帧数激活')
                    self.assertEqual(stream.model.channels[5].state, 1)

    def test_cancelled_candidate_resets_count(self):
        p = HostParameters(input_alpha=1, window=2,
            channel_repress={'4': {'margin':300, 'confirm_frames':10}})
        stream = Stream(p)
        stream.initialize()
        c=stream.model.channels[4]
        c.commit_state(HOST_STATE_IDLE_NEGATIVE)
        c.release_reference = c.baseline = 10000
        for i in range(6):
            stream.feed(10400 + 40*i)
        self.assertNotEqual(c.state, 1)
        stream.feed(10000)
        self.assertEqual(c.activate_count, 0)
        for i in range(6):
            stream.feed(10400 + 40*i)
        self.assertNotEqual(c.state, 1)

if __name__ == '__main__':
    unittest.main()
