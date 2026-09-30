"""在正式ARM固件中验证两段加压的Signal不被提前FLOATING锁定吞掉。"""
import unittest

from test_channel_state_pressure import ramp
from test_fixed_noise import Firmware


class FloatingTwoStageTests(unittest.TestCase):
    def test_signal_captures_second_loading_stage_for_each_variant(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            with self.subTest(variant=variant):
                fw = Firmware(variant)
                for _ in range(300):
                    fw.step([1000]*fw.channels)
                first = ramp(1000, 6000, round(fw.hz*.4))+[6000]*fw.hz
                for raw in first:
                    rows = fw.step([raw]*fw.channels)
                    fw.call('SensorAlgorithm_GetStates', 0x20120000)
                    self.assertNotIn(6, bytes(fw.uc.mem_read(0x20120000, fw.channels)))
                first_signals = [row['signal'] for row in rows]
                second = ramp(6000, 7000, round(fw.hz*.25))+[7000]*(4*fw.hz)
                for raw in second:
                    rows = fw.step([raw]*fw.channels)
                fw.call('SensorAlgorithm_GetStates', 0x20120000)
                self.assertEqual(bytes(fw.uc.mem_read(0x20120000, fw.channels)), bytes([6]*fw.channels))
                for previous, row in zip(first_signals, rows):
                    self.assertGreater(row['signal']-previous, 950)
                    self.assertLessEqual(abs(row['signal']-(row['raw']-row['baseline'])), 3)

    def test_late_small_corrections_remain_in_signal_before_lock(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            with self.subTest(variant=variant):
                fw = Firmware(variant)
                for _ in range(300):
                    fw.step([1000]*fw.channels)
                waveform = ramp(1000, 6000, round(fw.hz*.4))+[6000]*round(fw.hz*2.7)
                for raw in waveform:
                    rows = fw.step([raw]*fw.channels)
                previous = [r['signal'] for r in rows]
                correction = ramp(6000, 6100, round(fw.hz*.2))+[6100]*round(fw.hz*.3)
                correction += ramp(6100, 6150, round(fw.hz*.2))
                for raw in correction:
                    rows = fw.step([raw]*fw.channels)
                    fw.call('SensorAlgorithm_GetStates', 0x20120000)
                    self.assertNotIn(6, bytes(fw.uc.mem_read(0x20120000, fw.channels)))
                for _ in range(2*fw.hz):
                    rows = fw.step([6150]*fw.channels)
                fw.call('SensorAlgorithm_GetStates', 0x20120000)
                self.assertEqual(bytes(fw.uc.mem_read(0x20120000, fw.channels)), bytes([6]*fw.channels))
                for initial, row in zip(previous, rows):
                    self.assertGreaterEqual(row['signal']-initial, 147)
                    self.assertLessEqual(abs(row['signal']-(row['raw']-row['baseline'])), 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
