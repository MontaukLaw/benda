"""真实ARM回放高力10/20点协议、Flash及微力/高力输出分流。"""
import struct
from types import SimpleNamespace
import unittest

import test_flash_two_range as flash_tests
import test_low_force_calibration as low_tests
from high_force_calibration import calibration_targets
from serial_reader import SerialReaderThread
from data_buffer import _make_run_force_vector


class HighForceFirmwareTests(unittest.TestCase):
    def test_ten_twenty_point_save_reload_and_flash_boundaries(self):
        for variant in flash_tests.VARIANTS:
            for count in (10, 20):
                with self.subTest(variant=variant, count=count):
                    fw = flash_tests.CalibrationFirmware(variant)
                    fw._device = flash_tests.VARIANTS.index(variant)+1
                    untouched = bytes(fw.uc.mem_read(0x1F500, 256))+bytes(fw.uc.mem_read(0x1F900, 0x700))
                    fw.stage(targets=calibration_targets(count))
                    self.assertEqual(fw.acks[-1], 0)
                    before = fw.read_table()
                    self.assertEqual(before['point_count'], count)
                    self.assertTrue(before['valid'])
                    fw.command(0x85)
                    self.assertEqual(fw.acks[-1], 0)
                    self.assertEqual(fw.writes, [0x1F600, 0x1F700, 0x1F800])
                    fw.call('initialize_force_calibration')
                    after = fw.read_table()
                    self.assertTrue(after['valid'])
                    self.assertEqual(after['raw_points'], before['raw_points'])
                    self.assertEqual(after['calibrated_points'], before['calibrated_points'])
                    self.assertEqual(untouched, bytes(fw.uc.mem_read(0x1F500, 256))+bytes(fw.uc.mem_read(0x1F900, 0x700)))

    def test_short_legacy_range_is_rejected_without_replacing_table(self):
        fw = flash_tests.CalibrationFirmware('INDEX_FINGER')
        fw.stage(targets=calibration_targets(10))
        before = fw.read_table()
        fw.stage(targets=tuple(range(1, 21)))
        self.assertNotEqual(fw.acks[-1], 0)
        self.assertEqual(fw.read_table()['calibrated_points'], before['calibrated_points'])

    def test_high_force_runtime_and_reboot_preserve_independent_microforce(self):
        for count in (10, 20):
            with self.subTest(count=count):
                fw = low_tests.LowFirmware()
                self.assertEqual(fw.low_command(True)['result'], 0)
                low_flash = bytes(fw.uc.mem_read(0x1F500, 256))
                low_before = _make_run_force_vector(fw.run_frame(400), 0, 0)
                targets = calibration_targets(count)
                payload = bytes((0, 0, 2, count, 0, 0, 0, 0))
                payload += b''.join(struct.pack('<ff', p, p) for p in targets)
                fw.command(0x84, payload)
                self.assertEqual(fw.acks[-1], 0)
                fw.command(0x85)
                self.assertEqual(fw.acks[-1], 0)
                fw.call('SensorAlgorithm_Init')
                self.assertEqual(bytes(fw.uc.mem_read(0x1F500, 256)), low_flash)
                self.assertEqual(fw.low_command()['points'], low_tests.POINTS)
                low_after = _make_run_force_vector(fw.run_frame(400), 0, 0)
                self.assertTrue(low_after['low_force_mode'])
                self.assertAlmostEqual(low_after['normal_force'], low_before['normal_force'])
                for pressure in (1, 2, 14, 25, 30):
                    frame = _make_run_force_vector(fw.run_frame(pressure*1000), 0, 0)
                    self.assertFalse(frame['low_force_mode'])
                    self.assertFalse(frame['low_force_over_range'])
                    self.assertTrue(frame['normal_calibration_valid'])
                    self.assertAlmostEqual(frame['normal_force'], pressure, places=4)
                    self.assertAlmostEqual(frame['optimized_normal_force'], pressure, places=4)

    def test_host_requires_v21_for_short_tables_and_keeps_v20_48_nodes(self):
        for version in (20, 21):
            sent = []
            reader = SimpleNamespace(_protocol_version_confirmed=True, _protocol_version=version,
                                     send_command=lambda command: sent.append(command) or True)
            for count in (10, 20, 48):
                targets = calibration_targets(count) if count != 48 else flash_tests.FORCE_CALIBRATION_TARGETS_N
                result = SerialReaderThread.set_force_calibration(reader, 0, 0, targets, targets)
                self.assertEqual(result, version >= 21 or count == 48)
            self.assertEqual(len(sent), 3 if version >= 21 else 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
