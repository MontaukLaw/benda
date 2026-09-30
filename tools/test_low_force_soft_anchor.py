"""真实ARM软吸附连续性、同帧双值、CSV及显示方式持久化回归。"""
import csv
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest

from test_low_force_calibration import LowFirmware, POINTS
from unicorn.arm_const import UC_ARM_REG_R0
from config import FRAME_VERSION
from data_buffer import _make_run_force_vector
from data_recorder import SerialDataRecorder
from frame_parser import AlgoFrameParser
from low_force_calibration import FORCES_N, low_force_display
from low_force_calibration_widget import LowForceCalibrationWidget
from qt_compat import QtWidgets


class SoftAnchorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def optimized(self, firmware, force):
        bits = struct.unpack('<I', struct.pack('<f', force))[0]
        firmware.call('LowForce_Optimize', bits)
        return struct.unpack('<f', struct.pack('<I', firmware.uc.reg_read(UC_ARM_REG_R0)))[0]

    def test_anchors_monotonic_boundaries_and_reduced_sensitivity(self):
        fw = LowFirmware()
        for anchor in FORCES_N:
            self.assertAlmostEqual(self.optimized(fw, anchor), anchor, places=7)
            delta = 0.00001
            slope = (self.optimized(fw, anchor+delta)-self.optimized(fw, anchor-delta))/(2*delta)
            self.assertAlmostEqual(slope, 0.2, delta=0.005)
        boundaries = [0.0, *(0.5*(a+b) for a,b in zip(FORCES_N,FORCES_N[1:])), 1.0]
        for boundary in boundaries:
            self.assertAlmostEqual(self.optimized(fw, boundary), boundary, places=7)
            if 0 < boundary < 1:
                left = self.optimized(fw, boundary-0.00001)
                right = self.optimized(fw, boundary+0.00001)
                self.assertAlmostEqual((right-left)/0.00002, 1, delta=0.005)
        previous = -1.0
        for i in range(1001):
            value = self.optimized(fw, i/1000)
            self.assertGreater(value, previous)
            self.assertGreaterEqual(value, 0)
            self.assertLessEqual(value, 1)
            previous = value
        for index, anchor in enumerate(FORCES_N):
            for delta in (-0.001, 0.001):
                self.assertLess(abs(self.optimized(fw, anchor+delta)-anchor), abs(delta))

    def test_same_frame_pair_and_detection_before_optimization(self):
        fw = LowFirmware()
        fw.low_command(True)
        for total in (0, 90, 92, 100, 200, 210, 500, 510, 1000, 1019, 1020):
            frame = fw.run_frame(total)
            self.assertEqual(frame['protocol_version'], FRAME_VERSION)
            self.assertEqual(len(fw.frames[-1]), 50)
            original = frame['original_normal_force']
            optimized = frame['optimized_normal_force']
            self.assertEqual(original, frame['vectors'][0][0])
            expected = total * 0.000980665
            self.assertAlmostEqual(original, expected if 0.09 <= expected < 1 else 0, places=6)
            if expected >= 1:
                self.assertTrue(frame['component_calibration_mask'] & 0x80)
                self.assertEqual(optimized, 0)
            elif expected < 0.09:
                self.assertEqual((original, optimized), (0,0))
            else:
                self.assertAlmostEqual(optimized, self.optimized(fw, original), places=7)
        self.assertAlmostEqual(fw.run_frame(100)['optimized_normal_force'], FORCES_N[0], places=7)

    def test_parser_version_length_fragmentation_and_csv(self):
        fw = LowFirmware()
        fw.low_command(True)
        frame = fw.run_frame(210)
        packet = fw.frames[-1]
        parser = AlgoFrameParser()
        frames = []
        for byte in packet:
            frames.extend(parser.feed(bytes([byte])))
        self.assertEqual(frames, [frame])
        legacy = bytearray(packet[:38])
        legacy[4] = 17
        struct.pack_into('<H', legacy, 8, 28)
        legacy_frame = AlgoFrameParser().feed(legacy)[0]
        self.assertIsNone(legacy_frame['optimized_normal_force'])
        vector = _make_run_force_vector(legacy_frame, 0, 0)
        self.assertEqual(low_force_display(vector, optimized=True)[2], 'waiting')
        for version, payload in ((17, packet[10:]), (18, legacy[10:])):
            invalid = struct.pack('<4sBBBBH', b'DATA', version, 2, 1, 8, len(payload)) + payload
            self.assertEqual(AlgoFrameParser().feed(invalid), [])
        with tempfile.TemporaryDirectory() as directory:
            recorder = SerialDataRecorder(directory)
            recorder.record(frame)
            recorder.record({'type':'signal', 'device':1, 'channel_count':8, 'signal':(0,)*8})
            result = recorder.write_recent_snapshot(recorder.snapshot_recent(1))
            with Path(result['output_path']).open(encoding='utf-8-sig',newline='') as handle:
                row = next(row for row in csv.DictReader(handle) if row['type']=='run_force')
            self.assertAlmostEqual(float(row['original_normal_force']), frame['original_normal_force'])
            self.assertAlmostEqual(float(row['optimized_normal_force']), frame['optimized_normal_force'])
            self.assertNotEqual(row['original_normal_force'], row['optimized_normal_force'])

    def test_modes_persist_and_hide_only_display_not_data(self):
        fw = LowFirmware()
        fw.low_command(True)
        frame = fw.run_frame(210)
        vector = _make_run_force_vector(frame, 0, 0)
        sample = {'mcu_force_vector':vector, 'mcu_force_age_seconds':0}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'display.json'
            w = LowForceCalibrationWidget(lambda:None,lambda:sample,settings_path=path)
            self.assertEqual(w.display_mode_combo.currentData(), 'both')
            self.assertFalse(w.force_label.isHidden())
            self.assertFalse(w.optimized_force_label.isHidden())
            self.assertIn('21.00 g', w.force_label.text())
            self.assertIn(' g', w.optimized_force_label.text())
            original_text, optimized_text = w.force_label.text(), w.optimized_force_label.text()
            for mode in ('original','optimized','both'):
                w.display_mode_combo.setCurrentIndex(w.display_mode_combo.findData(mode))
                self.assertEqual(json.loads(path.read_text())['display_mode'], mode)
                self.assertEqual(w.force_label.isHidden(), mode=='optimized')
                self.assertEqual(w.optimized_force_label.isHidden(), mode=='original')
                self.assertEqual((w.force_label.text(), w.optimized_force_label.text()), (original_text, optimized_text))
                other = LowForceCalibrationWidget(lambda:None,lambda:sample,settings_path=path)
                self.assertEqual(other.display_mode_combo.currentData(), mode)
                other.close()
            sample['mcu_force_age_seconds']=2
            w.refresh()
            self.assertNotIn(' g', w.force_label.text())
            self.assertNotIn(' g', w.optimized_force_label.text())
            w.close()
            path.write_text('{broken',encoding='utf-8')
            w = LowForceCalibrationWidget(lambda:None,lambda:sample,settings_path=path)
            self.assertEqual(w.display_mode_combo.currentData(), 'both')
            w.close()

    def test_setting_write_failure_is_reported_and_bad_optimized_data_not_shown(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'directory'
            path.mkdir()
            w = LowForceCalibrationWidget(lambda:None,lambda:None,settings_path=path)
            w.display_mode_combo.setCurrentIndex(0)
            self.assertIn('未保存', w.display_settings_status.text())
            w.close()
        base = dict(low_force_mode=True,low_force_calibration_valid=True,normal_calibration_valid=True,normal_force=0.2)
        for value in (float('nan'),float('inf'),-1,1.1):
            self.assertEqual(low_force_display(dict(base,optimized_normal_force=value),optimized=True)[2],'invalid')


if __name__ == '__main__':
    unittest.main(verbosity=2)
