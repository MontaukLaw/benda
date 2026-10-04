"""执行真实ARM状态机，验证回弹迟滞、实际周期确认及恢复期间真实再次按压。"""
import struct
import unittest

from test_matrix20_firmware import Firmware


class Machine:
    def __init__(self, hz=74, period=None, baseline=8000):
        self.fw = Firmware()
        self.fw.call('ChannelState_SetSampleRateHz', hz)
        if period:
            self.fw.call('ChannelState_SetFramePeriodUs', period)
        self.baseline = baseline
        self.address = 0x20101000
        self.input = 0x20102000
        self.fw.call('ChannelState_Reset', self.address, baseline, baseline)
        self.hz = hz
        self.feed(baseline, hz)

    def feed(self, raw, count=1, **flags):
        trace = []
        for _ in range(count):
            self.fw.uc.mem_write(self.input, struct.pack('<HHH4?2x', raw, self.baseline, 1,
                True, flags.get('suppress', False), flags.get('transition', False),
                flags.get('finished', False)))
            trace.append(self.fw.call('ChannelState_Update', self.address, self.input))
        return trace

    def ramp(self, start, end, count):
        return [self.feed(round(start+(end-start)*(i+1)/count))[0] for i in range(count)]

    def release(self, peak=8500, tail=7800):
        self.ramp(self.baseline, peak, self.hz//5)
        self.feed(peak, self.hz)
        trace = self.feed(tail, self.hz//5)
        assert trace[-1] == 4, trace


class MatrixRetriggerTests(unittest.TestCase):
    def test_repeated_valley_rebound_above_original_zero_is_not_a_press(self):
        for hz in (70, 95, 200):
            with self.subTest(hz=hz):
                m = Machine(hz)
                m.release()
                for _ in range(8):
                    trace = m.ramp(7800, 8180, hz//5) + m.feed(8180, hz//3)
                    trace += m.ramp(8180, 7800, hz//5)
                    self.assertTrue(all(s in (4,5,7,8,0) for s in trace), trace)
                # 稳定后能离开慢回弹；实际基线跟随由既有全算法测试覆盖。
                trace = m.feed(8180, 3*hz)
                self.assertIn(8, trace)

    def test_real_repress_in_each_recovery_state_and_bounded_peak_threshold(self):
        for hz in (70, 95, 200):
            for recovery in (4,8,7,5):
                with self.subTest(hz=hz, recovery=recovery):
                    m = Machine(hz)
                    m.release(peak=18000, tail=10000)
                    if recovery == 5:
                        # 持续变化超过八秒，走公开SLOW_BACK超时路径。
                        m.ramp(10000, 10080, 9*hz)
                    elif recovery != 4:
                        for _ in range(4*hz):
                            state = m.feed(10000)[0]
                            if state == recovery:
                                break
                        self.assertEqual(state, recovery)
                    # 释放前峰值10000，幅度门槛应封顶400，600 counts仍可再次激活。
                    raw = 10080 if recovery == 5 else 10000
                    trace = m.ramp(raw, raw+600, hz//5) + m.feed(raw+600, hz//2)
                    self.assertIn(1, trace, trace)
                    self.assertEqual(trace[-1], 2)

    def test_short_candidates_cannot_accumulate_and_actual_period_controls_confirmation(self):
        # 窗口锁定95FPS但实际70FPS，专用时间门槛使用真实微秒周期。
        m = Machine(95, period=round(1e6/70))
        m.release(peak=9000)
        for _ in range(3):
            trace = m.feed(8300, 3) + m.feed(7800, 12)
            self.assertNotIn(1, trace)
        trace = m.feed(8600, 15)
        self.assertIn(1, trace)
        self.assertGreaterEqual(trace.index(1)+1, 6)
        self.assertLessEqual(trace.index(1)+1, 8)

    def test_recalibration_clears_large_peak_hysteresis_and_keeps_first_light_press(self):
        m = Machine()
        m.release(peak=18000)
        m.fw.call('ChannelState_Reset', m.address, 8000, 8000)
        m.feed(8000, m.hz)
        trace = m.ramp(8000,8060,m.hz) + m.feed(8060,m.hz)
        self.assertIn(1,trace)
        self.assertEqual(trace[-1],2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
