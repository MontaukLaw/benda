"""执行真实ARM固件，验证残余上的再次按压、慢释放及稳定长按保护。"""
import struct
import unittest
from test_matrix20_firmware import Firmware


class MatrixReleaseReferenceTests(unittest.TestCase):
    def fresh(self, hz):
        fw = Firmware()
        fw.call('ChannelState_SetSampleRateHz', hz)
        fw.call('initialize_baseline_runtime')
        fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H', 300))
        for _ in range(3*hz): fw.sample([8000]*20)
        return fw

    def ramp(self, fw, start, end, count):
        return [fw.sample([round(start+(end-start)*(i+1)/count)]*20) for i in range(count)]

    def test_repress_uses_local_reference_in_slow_back_wait_and_recover(self):
        for hz in (70, 95, 200):
            for recovery_state in (4, 8, 7):
                with self.subTest(hz=hz, recovery_state=recovery_state):
                    fw = self.fresh(hz)
                    self.ramp(fw, 8000, 14000, hz//5)
                    for _ in range(hz): fw.sample([14000]*20)
                    for _ in range(8*hz):
                        states, outputs = fw.sample([10000]*20)
                        if states == (recovery_state,)*20: break
                    self.assertEqual(states, (recovery_state,)*20)
                    self.assertTrue(all(o[2] == 0 for o in outputs))
                    self.ramp(fw, 10000, 10600, hz//5)
                    # 首通道的公开只读接口必须返回局部零点，而非旧8000基线。
                    reference = fw.call('ChannelState_GetActivationBaseline', fw.symbols['channel_state_machines'])
                    # RECOVER在超过再次按压门槛前仍可推进参考，但最多吸收阈值内的增量。
                    self.assertTrue(10000 <= reference <= 10040, reference)
                    for _ in range(6*hz): states, outputs = fw.sample([10600]*20)
                    self.assertEqual(states, (6,)*20)
                    self.assertTrue(all(abs(o[2]-(10600-reference)) <= 3 for o in outputs), outputs)
                    released = self.ramp(fw, 10600, 10080, hz//3)
                    self.assertTrue(any(s == (4,)*20 for s, _ in released))
                    for _ in range(2*hz): states, outputs = fw.sample([10080]*20)
                    self.assertTrue(all(s in (0,4,5,7,8) for s in states))
                    self.assertTrue(all(o[2] == 0 for o in outputs))

    def test_floating_slow_release_exits_lock_and_confirms_release(self):
        for hz in (70, 95, 200):
            with self.subTest(hz=hz):
                fw = self.fresh(hz)
                self.ramp(fw, 8000, 14000, hz//5)
                for _ in range(6*hz): states, outputs = fw.sample([14000]*20)
                self.assertEqual(states, (6,)*20)
                trace = self.ramp(fw, 14000, 10000, 6*hz)
                state_trace = [s[0] for s, _ in trace]
                self.assertIn(3, state_trace[:2*hz])
                self.assertIn(4, state_trace[:5*hz])
                first_falling = state_trace.index(3)
                first_release = state_trace.index(4)
                self.assertNotIn(6, state_trace[first_falling:first_release])
                self.assertTrue(all(o[2] == 0 for o in trace[first_release][1]))
                for _ in range(5*hz): states, outputs = fw.sample([10000]*20)
                self.assertEqual(states, (0,)*20)
                self.assertTrue(all(o[2] == 0 for o in outputs))

    def test_long_hold_partial_unload_and_noise_are_not_release(self):
        for hz in (70, 200):
            with self.subTest(hz=hz):
                fw = self.fresh(hz)
                self.ramp(fw, 8000, 8200, hz//5)
                for i in range(35*hz):
                    states, outputs = fw.sample([8200 + (i%3-1)]*20)
                    self.assertTrue(all(s in (1,2,3,6) for s in states))
                    self.assertTrue(all(o[2] > 150 for o in outputs))
                trace = self.ramp(fw, 8200, 8120, 4*hz)
                for states, outputs in trace:
                    self.assertTrue(all(s in (1,2,3,6) for s in states))
                    self.assertTrue(all(o[2] > 80 for o in outputs))
                for _ in range(4*hz):
                    states, outputs = fw.sample([8120]*20)
                    self.assertEqual(states, (3,)*20)
                    self.assertTrue(all(o[2] > 80 for o in outputs))
                # 真实重新加载应结束FALLING，不能把历史下降永久锁成另一种状态。
                trace = self.ramp(fw, 8120, 8400, hz//5)
                self.assertIn((1,)*20, [s for s, _ in trace])
                fw.call('SensorAlgorithm_RequestRecalibration')
                for _ in range(3*hz): states, outputs = fw.sample([8000]*20)
                self.assertEqual(states, (0,)*20)
                self.assertTrue(all(o[2] == 0 for o in outputs))


if __name__ == '__main__':
    unittest.main(verbosity=2)
