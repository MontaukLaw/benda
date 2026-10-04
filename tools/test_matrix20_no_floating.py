"""执行真实ARM固件，验证矩阵无锁定输出、慢变化及旧变体编译边界。"""
import unittest

from test_matrix20_recovery_tuning import fresh, ramp


class MatrixNoFloatingTests(unittest.TestCase):
    def test_lock_buffers_are_absent_from_matrix_firmware(self):
        fw = fresh(74)
        for name in ('floating_locked_signal_q16','floating_compensation_values','floating_compensation_valid'):
            self.assertNotIn(name,fw.symbols)

    def test_long_hold_slow_increase_and_partial_decrease_follow_raw(self):
        for hz in (74,95,200):
            with self.subTest(hz=hz):
                fw = fresh(hz)
                ramp(fw,8000,14000,hz//5)
                for _ in range(8*hz):
                    states,outputs = fw.sample([14000]*20)
                    self.assertNotIn(6,states)
                self.assertEqual(states,(2,)*20)
                first_signal=outputs[0][2]
                # 低于原FLOATING退出门槛的小幅慢加载也必须反映到Signal。
                rise=ramp(fw,14000,14060,3*hz)
                self.assertTrue(all(6 not in s for s,o in rise))
                self.assertGreater(rise[-1][1][0][2],first_signal+50)
                for _ in range(hz):
                    states,outputs=fw.sample([14060]*20)
                self.assertTrue(all(abs(o[2]-6058)<=3 for o in outputs))
                fall=ramp(fw,14060,13500,3*hz)
                self.assertTrue(all(s in (1,2,3) for states,o in fall for s in states))
                self.assertIn(3,[s[0] for s,o in fall])
                for _ in range(hz):
                    states,outputs=fw.sample([13500]*20)
                self.assertTrue(all(abs(o[2]-5498)<=3 for o in outputs))
                # 完全释放后仍须清零并恢复，不能把连续跟随变成永久活动。
                release=ramp(fw,13500,8000,hz//5)
                self.assertTrue(all(6 not in s for s,o in release))
                for _ in range(4*hz):
                    states,outputs=fw.sample([8000]*20)
                self.assertEqual(states,(0,)*20)
                self.assertTrue(all(o[2]==0 for o in outputs))

    def test_signal_following_remains_independent_across_twenty_channels(self):
        hz=74
        fw=fresh(hz)
        for _ in range(8*hz):
            states,outputs=fw.sample([8600 if i%2==0 else 8000 for i in range(20)])
        self.assertEqual(states,tuple(2 if i%2==0 else 0 for i in range(20)))
        for step in range(1,3*hz+1):
            raw=[8600+round(60*step/(3*hz)) if i%2==0 else 8000 for i in range(20)]
            states,outputs=fw.sample(raw)
            self.assertNotIn(6,states)
            self.assertTrue(all(o[2]==0 for i,o in enumerate(outputs) if i%2))
        self.assertTrue(all(o[2]>650 for i,o in enumerate(outputs) if i%2==0))


if __name__=='__main__':
    unittest.main(verbosity=2)
