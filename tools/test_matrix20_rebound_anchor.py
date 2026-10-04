"""真实ARM验证稳定平台后的下冲回升，以及再次按压零点不下降。"""
import unittest

from test_matrix20_retrigger import Machine


class MatrixReboundAnchorTests(unittest.TestCase):
    def prepare_residual_hold(self,hz):
        m=Machine(hz=hz,baseline=5765)
        m.release(peak=10000,tail=6200)
        trace=m.ramp(6200,6656,hz//5)+m.feed(6656,2*hz)
        self.assertIn(1,trace)
        self.assertEqual(trace[-1],2)
        self.assertEqual(m.fw.call('ChannelState_GetActivationBaseline',m.address),6200)
        return m

    def test_pulse_release_and_large_valley_rebound_do_not_reactivate(self):
        for hz in (70,95,200):
            with self.subTest(hz=hz):
                m=self.prepare_residual_hold(hz)
                m.ramp(6656,8794,hz//8)
                m.ramp(8794,5836,hz//5)
                trace=m.ramp(5836,6648,hz//4)+m.feed(6648,2*hz)
                self.assertTrue(all(s in (0,4,5,7,8) for s in trace),trace)
                # 已松手后的进一步真实加载仍有入口，并且提交零点保留加载前平台。
                trace=m.ramp(6648,7256,hz//5)+m.feed(7256,hz)
                self.assertIn(1,trace)
                self.assertEqual(trace[-1],2)
                self.assertGreaterEqual(m.fw.call('ChannelState_GetActivationBaseline',m.address),6656)

    def test_anchor_is_retained_even_when_no_plateau_can_be_confirmed(self):
        m=Machine(baseline=5765)
        m.release(peak=10000,tail=6200)
        m.ramp(6200,6656,m.hz//5)
        # 尚未稳定满慢窗时再次加压，只有原6200恢复锚点可依赖。
        m.feed(6656,3)
        m.ramp(6656,8794,m.hz//8)
        m.ramp(8794,5836,m.hz//5)
        trace=m.ramp(5836,7000,m.hz//4)+m.feed(7000,m.hz)
        self.assertIn(1,trace)
        self.assertGreaterEqual(m.fw.call('ChannelState_GetActivationBaseline',m.address),6200)

    def test_stable_hold_is_not_cleared_and_recalibration_clears_platform_history(self):
        m=self.prepare_residual_hold(74)
        for i in range(20*74):
            self.assertIn(m.feed(6656+i%3-1)[0],(1,2,3))
        m.fw.call('ChannelState_Reset',m.address,5765,5765)
        m.feed(5765,m.hz)
        trace=m.ramp(5765,5825,m.hz)+m.feed(5825,m.hz)
        self.assertIn(1,trace)
        self.assertEqual(trace[-1],2)


if __name__=='__main__':
    unittest.main(verbosity=2)
