"""对真实ARM新旧固件比较慢释放、恢复耗时和实测帧周期超时。"""
import json
from pathlib import Path
import struct
import unittest

import test_matrix20_firmware as firmware

OLD_ELF = firmware.ROOT / 'build/recovery_20261003/before/mtb-example-psoc4-msclp-finger-INDEX_FINGER.elf'
RESULTS = {}


def fresh(hz, old=False):
    current = firmware.ELF
    try:
        if old:
            firmware.ELF = OLD_ELF
        fw = firmware.Firmware()
    finally:
        firmware.ELF = current
    fw.call('ChannelState_SetSampleRateHz', hz)
    fw.call('initialize_baseline_runtime')
    fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H', 300))
    for _ in range(3*hz):
        fw.sample([8000]*20)
    return fw


def ramp(fw, start, end, count):
    return [fw.sample([round(start+(end-start)*(i+1)/count)]*20) for i in range(count)]


class RecoveryTuningTests(unittest.TestCase):
    @classmethod
    def tearDownClass(cls):
        output = firmware.ROOT/'build/no_floating_20261003/tuning_results.json'
        output.write_text(json.dumps(RESULTS,indent=2),encoding='utf-8')

    def test_gradual_small_release_no_longer_stays_floating(self):
        for hz in (74, 95):
            tails = {}
            for old in (True, False):
                fw = fresh(hz, old)
                ramp(fw, 8000, 8200, hz//5)
                for _ in range(6*hz):
                    states, outputs = fw.sample([8200]*20)
                self.assertEqual(states, ((6 if old else 2),)*20)
                trace = ramp(fw, 8200, 8080, 15*hz)
                for _ in range(4*hz):
                    states, outputs = fw.sample([8080]*20)
                tails['old' if old else 'new'] = dict(states=states, signal=outputs[0][2])
                if old:
                    self.assertEqual(states, (6,)*20)
                else:
                    self.assertIn((3,)*20, [s for s,o in trace])
                    self.assertIn((4,)*20, [s for s,o in trace])
                    self.assertEqual(states, (0,)*20)
                    self.assertTrue(all(o[2] == 0 for o in outputs))
            RESULTS[f'slow_release_{hz}'] = tails

    def test_confirmed_release_recovers_while_small_tail_drift_settles(self):
        hz = 74
        times = {}
        for old in (True, False):
            fw = fresh(hz, old)
            ramp(fw, 8000, 14000, hz//5)
            for _ in range(6*hz):
                fw.sample([14000]*20)
            first_idle = None
            for i in range(8*hz):
                raw = 10000 + round(8*min(i/hz, 3))
                states, outputs = fw.sample([raw]*20)
                if first_idle is None and states == (0,)*20:
                    first_idle = i/hz
                self.assertTrue(all(s in (0,4,5,7,8) for s in states))
                self.assertTrue(all(o[2] == 0 for o in outputs))
            self.assertIsNotNone(first_idle)
            times['old' if old else 'new'] = first_idle
        self.assertLess(times['new'], times['old']-1)
        RESULTS['release_tail_idle_seconds'] = times

    def test_timeout_uses_actual_period_with_no_rebase_during_motion(self):
        fw = fresh(95)
        ramp(fw, 8000, 14000, 19)
        for _ in range(6*95):
            fw.sample([14000]*20)
        # 窗口仍按开机95FPS锁定，但真实运行74FPS，复现原30秒变38秒的问题。
        period = round(1000000/74)
        fw.call('ChannelState_SetFramePeriodUs', period)
        first_zero = None
        for i in range(9*74):
            states, outputs = fw.sample([10000 + round(60*i/74)]*20)
            if first_zero is None and states == (5,)*20:
                first_zero = i*period/1000000
            self.assertTrue(all(s in (4,5) for s in states))
            self.assertTrue(all(o[1] == 8000 and o[2] == 0 for o in outputs), (i,states,outputs[0]))
        self.assertIsNotNone(first_zero)
        self.assertTrue(7.95 <= first_zero <= 8.05, first_zero)
        RESULTS['timeout_actual_seconds'] = first_zero

    def test_tail_drift_with_short_window_noise_does_not_start_another_press(self):
        hz = 74
        fw = fresh(hz)
        ramp(fw, 8000, 14000, hz//5)
        for _ in range(6*hz):
            fw.sample([14000]*20)
        for _ in range(5*hz):
            fw.sample([10000]*20)
        for i in range(15*hz):
            raw = 10000 + round(8*i/hz) + (3 if i%4<2 else -3)
            states, outputs = fw.sample([raw]*20)
            self.assertTrue(all(s in (0,7,8) for s in states), (i,states,outputs[0]))
            self.assertTrue(all(o[2] == 0 for o in outputs))

    def test_real_small_slow_press_still_has_an_entry(self):
        for hz in (74,95):
            fw = fresh(hz)
            trace = ramp(fw,8000,8060,hz)
            self.assertIn((1,)*20,[s for s,o in trace])
            for _ in range(5*hz):
                states,outputs=fw.sample([8060]*20)
            self.assertEqual(states,(2,)*20)
            self.assertTrue(all(o[2]>40 for o in outputs))


if __name__ == '__main__':
    try:
        unittest.main(verbosity=2)
    finally:
        output = firmware.ROOT/'build/no_floating_20261003/tuning_results.json'
        output.write_text(json.dumps(RESULTS,indent=2),encoding='utf-8')
