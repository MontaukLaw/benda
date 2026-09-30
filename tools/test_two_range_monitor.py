"""离屏验证两段标定计划、断点恢复、完整下发和旧固件写入禁用。"""
import json
from dataclasses import replace
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from qt_compat import QtWidgets
from test_force_status_layout import LayoutOnlyForceWidget
from serial_reader import SerialReaderThread
from dfu_upgrade import parse_cyacd2, validate_application_image, Cyacd2Row, DfuProgrammer
from force_signal_pressure_calibration_widget import ForceSignalPressureCalibrationWidget
from force_signal_pressure_calibration import (
    FORCE_CALIBRATION_TARGETS_N, QUICK_CORRECTION_TARGETS_N,
    build_signal_pressure_mapping, build_quick_corrected_mapping,
    select_firmware_calibration_document_nodes,
)


def mapping():
    entries = [dict(target_force_n=t, signal_sum=t*1000, final_force_n=t,
                    floating_count=4, captured_at='2026-09-12T12:00:00')
               for t in FORCE_CALIBRATION_TARGETS_N]
    return build_signal_pressure_mapping(entries, 'synthetic.csv', 'test')


class TwoRangeMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_plan_and_resume_preserve_two_range_fields(self):
        widget = LayoutOnlyForceWidget()
        self.addCleanup(widget.close)
        plan, error = widget._sweep_test_plan()
        self.assertEqual(error, '')
        self.assertEqual(plan['targets_n'], FORCE_CALIBRATION_TARGETS_N)
        self.assertEqual(plan['samples_per_cycle'], 96)
        self.assertEqual(plan['warmup_force_n'], 30)
        self.assertFalse(widget.sweep_max_force_input.isEnabled())
        self.assertFalse(widget.sweep_force_interval_input.isEnabled())
        parameters = {'sweep_plan': plan, 'release_position_counts': 0}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'checkpoint.json'
            with patch('force_press_control_widget.force_sweep_checkpoint_path', return_value=path):
                widget._write_sweep_checkpoint(parameters, [], 'running')
                first = widget._load_sweep_checkpoint()
                self.assertEqual(first['plan']['low_force_interval_n'], .1)
                self.assertEqual(first['plan']['range_boundary_n'], 2)
                parameters['sweep_plan'] = first['plan']
                widget._write_sweep_checkpoint(parameters, [], 'running')
                second = widget._load_sweep_checkpoint()
                self.assertEqual(second['plan'], first['plan'])
                document = json.loads(path.read_text(encoding='utf-8'))
                document['format_version'] = 3
                path.write_text(json.dumps(document), encoding='utf-8')
                with self.assertRaises(ValueError):
                    widget._load_sweep_checkpoint()

    def test_dfu_rejects_unsafe_images_before_contacting_device(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            path = ROOT / f'build/{variant}/UART/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-{variant}_crc.cyacd2'
            image = parse_cyacd2(path)
            self.assertEqual(image.verify_start + image.verify_size + 4, 0x1F500)
            unsafe = [replace(image, app_id=0), replace(image, verify_size=0x1C600-4),
                      replace(image, rows=image.rows[:-1]), replace(image, rows=image.rows+image.rows[-1:])]
            for address in (0, 0x1F500, 0x1F600, 0x1FD00, 0x1FE00, 0x1FF00):
                unsafe.append(replace(image, rows=image.rows + [Cyacd2Row(address, bytes(256))]))
            damaged = Cyacd2Row(image.rows[0].address, bytes(256))
            unsafe.append(replace(image, rows=[damaged] + image.rows[1:]))
            for bad in unsafe:
                with self.assertRaises(ValueError):
                    validate_application_image(bad)
                # 使用无client的实例，若触及通信便会失败，证明校验先于设备动作。
                with self.assertRaises(ValueError):
                    DfuProgrammer.program(object(), bad)

    def test_mapping_quick_correction_and_zero_rejection(self):
        document = mapping()
        self.assertEqual(len(document['segments']), 47)
        self.assertEqual(len(select_firmware_calibration_document_nodes(document)['nodes']), 48)
        self.assertEqual(len(QUICK_CORRECTION_TARGETS_N), 19)
        entries = [dict(target_force_n=t, final_force_n=t, signal_sum=t*1100, floating_count=4)
                   for t in QUICK_CORRECTION_TARGETS_N]
        corrected = build_quick_corrected_mapping(document, entries)['document']
        self.assertEqual(len(select_firmware_calibration_document_nodes(corrected)['nodes']), 48)
        with self.assertRaises(ValueError):
            build_quick_corrected_mapping(document, entries[1:])
        document['nodes'][0]['signal_sum_median'] = 0
        with self.assertRaises(ValueError):
            select_firmware_calibration_document_nodes(document)

    def test_actual_widget_scales_all_48_nodes_for_wire(self):
        with patch.object(ForceSignalPressureCalibrationWidget, '_load_existing_mapping'):
            widget = ForceSignalPressureCalibrationWidget()
        self.addCleanup(widget.close)
        widget._document = mapping()
        widget._connected = True
        requests = []
        widget.firmware_persist_requested.connect(requests.append)
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.StandardButton.Yes):
            widget._request_firmware_persist()
        self.assertEqual(len(requests), 1)
        request = requests[0]
        self.assertEqual(request['raw_points'], FORCE_CALIBRATION_TARGETS_N)
        self.assertEqual(request['calibrated_points'], FORCE_CALIBRATION_TARGETS_N)
        reader = SerialReaderThread('NO_DEVICE', None)
        sent = []
        reader.send_command = lambda data: (sent.append(data) or True)
        args = (0, 0, request['raw_points'], request['calibrated_points'])
        self.assertFalse(reader.set_force_calibration(*args))
        self.assertFalse(reader.save_force_calibration())
        reader._protocol_version_confirmed = True
        for version in (14, 15):
            reader._protocol_version = version
            self.assertFalse(reader.set_force_calibration(*args))
            self.assertFalse(reader.save_force_calibration())
        self.assertEqual(sent, [])
        reader._protocol_version = 16
        self.assertTrue(reader.set_force_calibration(*args))
        self.assertEqual(len(sent[-1]), 402)
        self.assertEqual(struct.unpack_from('<H', sent[-1], 8)[0], 392)
        self.assertEqual(sent[-1][4:6], bytes((16, 0x84)))
        self.assertEqual(struct.unpack_from('<ff', sent[-1], 18 + 47*8), (30, 30))
        self.assertTrue(reader.save_force_calibration())
        self.assertFalse(reader.set_force_calibration(0, 1, request['raw_points'], request['calibrated_points']))
        self.assertFalse(reader.stage_affine_calibration_channel(1, 4, [(65536, 0)]*7))
        self.assertFalse(reader.save_affine_calibration(1))


if __name__ == '__main__':
    unittest.main(verbosity=2)
