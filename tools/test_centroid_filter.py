"""验证显示滤波的抖动、延迟、采样率变化和断流恢复边界。"""
import math
from pathlib import Path
import statistics
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from centroid_filter import CENTROID_BLOCKS, CentroidFilter, signal_centroid


def signals_at(x, y):
    values = [0.0]*20
    # 在相邻四通道上生成双线性权重，避免用阵列四角模拟局部触摸。
    col, row = min(3, int(x-.5)), min(2, int(y-.5))
    u, v = x-.5-col, y-.5-row
    start = 4*col+row
    for ch, value in ((start,(1-u)*(1-v)),(start+4,u*(1-v)),
                      (start+1,(1-u)*v),(start+5,u*v)):
        values[ch] = value*10000
    return values


class CentroidTests(unittest.TestCase):
    def test_independent_axes_and_jitter_reduction(self):
        filt=CentroidFilter()
        raw, displayed = [], []
        for i in range(300):
            t=i/100
            x=2.5+.2*math.sin(2*math.pi*15*t)
            point=filt.update(signals_at(x,1.5),t)
            self.assertAlmostEqual(point[1],1.5)
            if i>100:
                raw.append(x); displayed.append(point[0])
        self.assertLess(statistics.pstdev(displayed),.25*statistics.pstdev(raw))

    def test_step_bounded_and_sample_rate_independent(self):
        results=[]
        for rate in (73,200):
            filt=CentroidFilter()
            filt.update(signals_at(.5,.5),0)
            for i in range(1,round(.5*rate)+1):
                point=filt.update(signals_at(4.5,3.5),i/rate)
                self.assertTrue(.5<=point[0]<=4.5 and .5<=point[1]<=3.5)
                if i == round(.2*rate):
                    results.append(point)
            self.assertGreater(point[0],4.45)
        for axis in range(2):
            self.assertLess(abs(results[0][axis]-results[1][axis]),.08)
        self.assertGreater(results[0][0],4.1)

    def test_reset_and_duplicate_timestamps(self):
        for invalid in (None,[1]*19,[float('nan')]+[1]*19):
            filt=CentroidFilter()
            filt.update(signals_at(.5,.5),0)
            self.assertIsNone(filt.update(invalid,.01))
            self.assertEqual(filt.update(signals_at(4.5,3.5),.02),(4.5,3.5))
        filt=CentroidFilter()
        filt.update(signals_at(.5,.5),0)
        self.assertEqual(filt.update(signals_at(4.5,3.5),0),(.5,.5))
        self.assertEqual(filt.update(signals_at(4.5,3.5),1),(4.5,3.5))

    def test_release_tail_hidden_and_light_repress_without_zero(self):
        filt=CentroidFilter(signal_threshold=0)
        strong=signals_at(2.5,2)
        filt.update(strong,0,[1]*20)
        tail=[0]*20; tail[0]=49
        original=tail.copy()
        self.assertEqual(filt.update(tail,.01,[3]*20),(2.5,2.0))
        self.assertEqual(filt.raw_xy,(.5,.5))
        self.assertTrue(filt.release_hidden)
        self.assertEqual(tail,original)
        for i in range(2,10):
            self.assertEqual(filt.update(tail,i/100,[3]*20),(2.5,2.0))
        self.assertIsNone(filt.update(tail,.12,[3]*20))
        light=[0]*20; light[19]=20
        self.assertEqual(filt.update(light,.15,[1]*20),(4.5,3.5))
        self.assertFalse(filt.release_hidden)
        self.assertEqual(filt.peak_signal,20)

    def test_light_touch_and_sliding_survive(self):
        filt=CentroidFilter(signal_threshold=0)
        light=[0]*20; light[0]=5
        self.assertEqual(filt.update(light,0,[1]*20),(.5,.5))
        self.assertIsNotNone(filt.update(light,.01,[2]*20))
        # 从较强按压减到轻触，RISING/ACTIVE/FLOATING仍是有效触摸；正常滑动衰减不隐藏。
        filt.update(signals_at(2.5,2),.02,[2]*20)
        for i,state in enumerate((1,2,6)):
            self.assertIsNotNone(filt.update(light,.03+i*.01,[state]*20))
        moderate=[v*.1 for v in signals_at(4,2)]
        self.assertIsNotNone(filt.update(moderate,.06,[3]*20))

    def test_run_mode_release_repress_and_stale_reset(self):
        filt=CentroidFilter(signal_threshold=0)
        filt.update(signals_at(2.5,2),0)
        tail=[0]*20; tail[0]=50
        self.assertEqual(filt.update(tail,.01),(2.5,2.0))
        self.assertEqual(filt.update(tail,.02),(2.5,2.0))
        tail[0]=40
        self.assertEqual(filt.update(tail,.03),(2.5,2.0))
        new=[0]*20; new[19]=65
        resumed=filt.update(new,.04)
        self.assertTrue(2.5<resumed[0]<3 and 2<resumed[1]<2.5)
        tail[0]=1
        self.assertEqual(filt.update(tail,.05),resumed)
        self.assertEqual(filt.update(tail,1),(.5,.5))

    def test_single_channel_threshold_and_reappear_without_trailing(self):
        for states in (None, [1]*20, [3]*20, [6]*20):
            filt=CentroidFilter()
            signal=[0]*20; signal[0]=101
            self.assertEqual(filt.update(signal,0,states),(.5,.5))
            # 未达单通道阈值时先冻结，不得向仍有总Signal的中心漂移。
            weak=[99]*20
            self.assertEqual(filt.update(weak,.01,states),(.5,.5))
            self.assertTrue(filt.threshold_hidden)
            self.assertTrue(filt.holding_position)
            self.assertEqual(filt.raw_xy,(1.0,1.0))
            self.assertIsNone(filt.update(weak,.12,states))
            signal=[0]*20; signal[19]=100
            self.assertEqual(filt.update(signal,.13,states),(4.5,3.5))
            self.assertFalse(filt.threshold_hidden)
            self.assertEqual(filt.max_signal,100)

    def test_threshold_change_rechecks_same_sample(self):
        filt=CentroidFilter()
        signal=[0]*20; signal[0]=100
        self.assertEqual(filt.update(signal,1),(.5,.5))
        filt.set_signal_threshold(101)
        self.assertIsNone(filt.update(signal,1))
        filt.set_signal_threshold(100)
        self.assertEqual(filt.update(signal,1),(.5,.5))
        self.assertEqual(filt.visibility()['signal_threshold'],100)

    def test_zero_hold_expiry_without_next_sample(self):
        filt=CentroidFilter()
        original=filt.update(signals_at(1,2),0)
        self.assertEqual(filt.update([0]*20,.01),original)
        self.assertFalse(filt.hold_due(.109))
        filt.expire_hold(.111)
        self.assertIsNone(filt.filtered_xy)
        self.assertFalse(filt.holding_position)
        point=filt.update(signals_at(4,3),.12)
        self.assertAlmostEqual(point[0],4)
        self.assertAlmostEqual(point[1],3)

    def test_short_loss_resumes_smoothly_and_real_disconnect_clears(self):
        filt=CentroidFilter()
        original=filt.update(signals_at(1,2),0)
        for t in (.01,.03,.05,.09):
            self.assertEqual(filt.update([0]*20,t),original)
        resumed=filt.update(signals_at(4,3),.095)
        self.assertTrue(1<resumed[0]<1.3)
        self.assertTrue(2<resumed[1]<2.1)
        self.assertFalse(filt.holding_position)
        self.assertEqual(filt.update([0]*20,.1),resumed)
        self.assertIsNone(filt.update(None,.11))
        self.assertFalse(filt.holding_position)


class RawCentroidTests(unittest.TestCase):
    def test_raw_source_keeps_static_counts_and_drives_block_selection(self):
        signals = [0]*20; signals[0] = 100
        raw = [1000]*20; raw[19] = 3000
        for local, expected in ((False, (59000/22000,47000/22000)),
                                (True, (25000/6000,19000/6000))):
            filt = CentroidFilter(local_2x2_enabled=local)
            self.assertEqual(filt.update(signals, 1, raw_values=raw), (.5,.5))
            filt.set_raw_enabled(True)
            point = filt.update(signals, 1, raw_values=raw)
            for actual, target in zip(point, expected):
                self.assertAlmostEqual(actual, target)
            self.assertEqual(filt.selected_sum_weight, 6000 if local else 22000)
            self.assertEqual(filt.selected_sum_signal, 0 if local else 100)
            if local:
                self.assertEqual(filt.selected_channels, (14,15,18,19))
            self.assertEqual(filt.parameters()['centroid_source'], 'raw')
            filt.set_raw_enabled(False)
            self.assertEqual(filt.update(signals, 1, raw_values=raw), (.5,.5))
        self.assertEqual(raw, [1000]*19+[3000])
        self.assertEqual(signals, [100]+[0]*19)

    def test_raw_floor_does_not_show_touch_and_release_uses_signal(self):
        raw = [1000]*20
        filt = CentroidFilter(signal_threshold=0, local_2x2_enabled=False, raw_enabled=True)
        self.assertIsNone(filt.update([0]*20, 0, raw_values=raw))
        self.assertFalse(filt.visibility()['signal_valid'])
        strong = [0]*20; strong[0] = 10000
        self.assertEqual(filt.update(strong, .01, [1]*20, raw_values=raw), (2.5,2))
        tail = [0]*20; tail[0] = 20
        changed_raw = [1000]*19+[3000]
        self.assertEqual(filt.update(tail, .02, [3]*20, raw_values=changed_raw), (2.5,2))
        self.assertTrue(filt.release_hidden)
        self.assertIsNone(filt.update(tail, .13, [3]*20, raw_values=changed_raw))
        filt.set_raw_enabled(False)
        self.assertIsNone(filt.update(tail, .13, [3]*20))
        self.assertTrue(filt.release_hidden)

    def test_missing_raw_never_reuses_previous_sample_or_falls_back(self):
        for invalid in (None, (), [1000]*19, [float('nan')]+[1000]*19):
            filt = CentroidFilter(raw_enabled=True)
            self.assertIsNotNone(filt.update([100]*20, 0, raw_values=[1000]*20))
            self.assertIsNone(filt.update([100]*20, .01, raw_values=invalid))
            self.assertTrue(filt.source_missing)
            self.assertIsNone(filt.raw_xy)
            self.assertIsNotNone(filt.update([100]*20, .02, raw_values=[1000]*20))
            self.assertFalse(filt.source_missing)


class RawDifferenceCentroidTests(unittest.TestCase):
    def test_common_floor_removed_and_all_corners_reachable(self):
        for local in (False, True):
            for channel in (0, 3, 16, 19):
                signals = [0]*20; signals[channel] = 100
                for floor in (1000, 30000, 65000):
                    raw = [floor]*20; raw[channel] += 100
                    original = raw.copy()
                    filt = CentroidFilter(local_2x2_enabled=local, raw_difference_enabled=True)
                    self.assertEqual(filt.update(signals, 0, raw_values=raw),
                                     (channel//4+.5, channel%4+.5))
                    self.assertEqual(filt.raw_mean, floor+5)
                    self.assertEqual(filt.normalized_weights[channel], 1)
                    self.assertAlmostEqual(sum(filt.normalized_weights), 1)
                    self.assertEqual(raw, original)

    def test_relative_weights_and_offset_gain_invariance(self):
        signals = [0]*19+[100]
        raw = [1000]*20; raw[0] = 1100; raw[19] = 1300
        for gain, offset in ((1, 0), (2, 5000), (.5, 123)):
            filt = CentroidFilter(local_2x2_enabled=False, raw_difference_enabled=True)
            point = filt.update(signals, 0, raw_values=[gain*v+offset for v in raw])
            # 均值1020，两个正差为80和280；偏低通道不能通过绝对值参与定位。
            self.assertAlmostEqual(point[0], (80*.5+280*4.5)/360)
            self.assertAlmostEqual(point[1], (80*.5+280*3.5)/360)
            self.assertAlmostEqual(filt.normalized_weights[0], 2/9)
            self.assertAlmostEqual(filt.normalized_weights[19], 7/9)
            self.assertEqual(sum(w > 0 for w in filt.normalized_weights), 2)
            self.assertAlmostEqual(filt.selected_sum_weight, 1)

    def test_uniform_raw_and_missing_raw_have_no_position(self):
        filt = CentroidFilter(raw_difference_enabled=True)
        for raw in ([1000]*20, [0]*20):
            self.assertIsNone(filt.update([100]*20, 0, raw_values=raw))
            self.assertFalse(filt.visibility()['signal_valid'])
        for raw in (None, [1000]*19, [float('nan')]+[1000]*19, [float('inf')]+[1000]*19):
            self.assertIsNone(filt.update([100]*20, .01, raw_values=raw))
            self.assertTrue(filt.source_missing)
            self.assertEqual(filt.normalized_weights, ())
        self.assertEqual(filt.update([100]*20, .02, raw_values=[1000]*19+[1100]), (4.5,3.5))
        self.assertFalse(filt.source_missing)
        # 变成均匀Raw时按现有规则原位保持100毫秒，不能回到阵列中心。
        self.assertEqual(filt.update([100]*20, .03, raw_values=[1000]*20), (4.5,3.5))
        self.assertTrue(filt.holding_position)
        self.assertIsNone(filt.raw_xy)
        self.assertIsNone(filt.update([100]*20, .14, raw_values=[1000]*20))

    def test_switches_recompute_same_sample_and_preserve_release_gate(self):
        filt = CentroidFilter(local_2x2_enabled=False)
        signals = [100]+[0]*19
        raw = [1000]*19+[1100]
        self.assertEqual(filt.update(signals, 0, raw_values=raw), (.5,.5))
        filt.set_raw_enabled(True)
        self.assertNotEqual(filt.update(signals, 0, raw_values=raw), (4.5,3.5))
        filt.set_raw_difference_enabled(True)
        self.assertFalse(filt.raw_enabled)
        self.assertEqual(filt.update(signals, 0, raw_values=raw), (4.5,3.5))
        self.assertEqual(filt.parameters()['centroid_weighting'], 'positive_mean_difference_normalized')
        filt.set_raw_enabled(True)
        self.assertFalse(filt.raw_difference_enabled)
        filt.set_raw_difference_enabled(True)
        filt.set_raw_difference_enabled(False)
        self.assertEqual(filt.update(signals, 0, raw_values=raw), (.5,.5))
        filt.set_raw_difference_enabled(True)
        self.assertIsNone(filt.update([0]*20, .01, raw_values=raw))
        strong = [10000]+[0]*19
        self.assertEqual(filt.update(strong, .02, [1]*20, raw_values=raw), (4.5,3.5))
        tail = [100]+[0]*19
        filt.update(tail, .03, [3]*20, raw_values=raw)
        self.assertTrue(filt.release_hidden)
        filt.set_raw_difference_enabled(False)
        self.assertIsNone(filt.update(tail, .03, [3]*20, raw_values=raw))
        filt.set_raw_difference_enabled(True)
        self.assertIsNone(filt.update(tail, .03, [3]*20, raw_values=raw))
        self.assertTrue(filt.release_hidden)


class LocalCentroidTests(unittest.TestCase):
    def test_mode_switch_recalculates_same_sample_without_old_smoothing(self):
        values = [0]*20
        values[0], values[19] = 100, 300
        filt = CentroidFilter()
        self.assertEqual(filt.update(values, 1), (4.5,3.5))
        filt.set_local_2x2_enabled(False)
        self.assertEqual(filt.update(values, 1), (3.5,2.75))
        self.assertEqual(signal_centroid(values, False), (3.5,2.75))
        self.assertEqual(filt.selected_channels, tuple(range(20)))
        self.assertEqual(filt.selected_sum_signal, 400)
        self.assertEqual(filt.parameters()['centroid_algorithm'], 'all_20')
        filt.set_local_2x2_enabled(True)
        self.assertEqual(filt.update(values, 1), (4.5,3.5))
        self.assertEqual(filt.parameters()['centroid_algorithm'], 'max_sum_2x2')

    def test_mode_switch_keeps_release_hidden_and_zero_invalid_handling(self):
        filt = CentroidFilter(signal_threshold=0)
        filt.update(signals_at(2.5,2), 0, [1]*20)
        tail = [0]*20
        tail[0] = 20
        filt.update(tail, .01, [3]*20)
        self.assertTrue(filt.release_hidden)
        filt.set_local_2x2_enabled(False)
        self.assertIsNone(filt.update(tail, .01, [3]*20))
        self.assertTrue(filt.release_hidden)
        for values in ([0]*20, None, [1]*19, [float('nan')]+[1]*19):
            self.assertIsNone(signal_centroid(values, False))

    def test_all_twelve_blocks_and_weighted_coordinates(self):
        self.assertEqual(len(set(CENTROID_BLOCKS)), 12)
        for channels in CENTROID_BLOCKS:
            values = [0]*20
            for ch, weight in zip(channels, (100, 200, 300, 400)):
                values[ch] = weight
            filt = CentroidFilter()
            point = filt.update(values, 0)
            self.assertEqual(filt.selected_channels, channels)
            self.assertEqual(filt.selected_sum_signal, 1000)
            for axis, coordinate in enumerate((lambda ch: ch//4+.5, lambda ch: ch%4+.5)):
                expected = sum(coordinate(ch)*values[ch] for ch in channels)/1000
                self.assertAlmostEqual(point[axis], expected)
            self.assertEqual(signal_centroid(values), point)

    def test_screenshot_and_remote_noise_do_not_pull_position(self):
        values = [0,267,0,0,6019,8106,58,936,2005,7786,200,0,343,404,99,0,652,493,143,0]
        original = values.copy()
        states = [6]*20
        filt = CentroidFilter()
        point = filt.update(values, 0, states)
        self.assertEqual(filt.selected_channels, (4,5,8,9))
        self.assertEqual(filt.selected_sum_signal, 23916)
        isolated = [v if ch in (4,5,8,9) else 0 for ch,v in enumerate(values)]
        self.assertEqual(point, signal_centroid(isolated))
        self.assertEqual(values, original)
        self.assertEqual(states, [6]*20)

    def test_strongest_sum_can_exclude_largest_single_channel(self):
        values = [0]*20
        values[0] = 1500
        for ch in (14,15,18,19):
            values[ch] = 500
        filt = CentroidFilter()
        self.assertEqual(filt.update(values, 0), (4,3))
        self.assertEqual(filt.selected_channels, (14,15,18,19))

    def test_ties_keep_previous_and_strictly_stronger_block_switches(self):
        values = [0]*20
        for ch in (14,15,18,19):
            values[ch] = 1000
        filt = CentroidFilter()
        filt.update(values, 0)
        for ch in (0,1,4,5):
            values[ch] = 1000
        filt.update(values, .01)
        self.assertEqual(filt.selected_channels, (14,15,18,19))
        self.assertEqual(filt.raw_xy, (4,3))
        values[0] += 1
        filt.update(values, .02)
        self.assertEqual(filt.selected_channels, (0,1,4,5))
        filt.update(None, .03)
        self.assertEqual(filt.selected_channels, ())
        self.assertEqual(filt.selected_sum_signal, 0)
        self.assertEqual(filt.update([100]*20, .04), (1,1))

    def test_invalid_zero_and_negative_signals(self):
        for values in (None, [1]*19, [0]*20, [-10]*20,
                       [float('inf')]+[1]*19, [float('nan')]+[1]*19):
            self.assertIsNone(signal_centroid(values))
        values = [-10]*20
        values[19] = 65535
        self.assertEqual(signal_centroid(values), (4.5,3.5))
        self.assertEqual(signal_centroid([65535]*20), (1,1))

    def test_local_slide_across_every_row_and_column_boundary(self):
        for y in (.5, 1.2, 2.7, 3.5):
            filt = CentroidFilter()
            for i in range(401):
                x = .5+i/100
                filt.update(signals_at(x,y), i/100)
                self.assertAlmostEqual(filt.raw_xy[0], x)
                self.assertAlmostEqual(filt.raw_xy[1], y)
        for x in (.5, 1.2, 3.7, 4.5):
            filt = CentroidFilter()
            for i in range(301):
                y = .5+i/100
                filt.update(signals_at(x,y), i/100)
                self.assertAlmostEqual(filt.raw_xy[0], x)
                self.assertAlmostEqual(filt.raw_xy[1], y)


if __name__=='__main__':
    unittest.main(verbosity=2)
