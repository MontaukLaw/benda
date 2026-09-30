"""真实ARM低力换算/Flash故障回放及上位机独立读写链路回归。"""
import os
import struct
import unittest
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from test_flash_two_range import CalibrationFirmware
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0
from frame_parser import AlgoFrameParser
from data_buffer import AlgoDataBuffer
from serial_reader import SerialReaderThread
from low_force_calibration import validate_points, low_force_display, FORCES_N
from low_force_calibration_widget import LowForceCalibrationWidget
from qt_compat import QtWidgets
from run_force_widget import RunForceWidget
from force_press_control_widget import ForcePressControlWidget

POINTS = (100.0, 200.0, 500.0, 1000.0)


class LowFirmware(CalibrationFirmware):
    def __init__(self, variant='INDEX_FINGER'):
        super().__init__(variant)
        self.uc.mem_write(0x1F500, b'\xff' * 256)
        self.call('LowForce_Load')

    def low_command(self, save=False, points=POINTS, token=456, detection_counts=0.0):
        payload = struct.pack('<I', token) + (struct.pack('<5f', *points, detection_counts) if save else b'')
        self.command(0x99 if save else 0x98, payload)
        parser = AlgoFrameParser()
        result = []
        for byte in self.frames[-1]:
            result.extend(parser.feed(bytes([byte])))
        assert len(result) == 1
        return result[0]

    def evaluate(self, signal):
        bits = struct.unpack('<I', struct.pack('<f', signal))[0]
        self.call('LowForce_Evaluate', bits, 0x20115000)
        return self.uc.reg_read(UC_ARM_REG_R0), struct.unpack('<f', self.uc.mem_read(0x20115000, 4))[0]

    def run_frame(self, total):
        address = self.symbols['send_output_frame'] & ~1
        hook = self.uc.hook_add(UC_HOOK_CODE, self.response, begin=address, end=address)
        self.uc.mem_write(self.symbols['signal_q16'], struct.pack('<8I', 0, 0, 0, 0, total << 16, 0, 0, 0))
        self.uc.mem_write(self.symbols['calibrated'], b'\x01')
        self.call('send_run_force_frame')
        self.uc.hook_del(hook)
        return AlgoFrameParser().feed(self.frames[-1])[0]


class LowForceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_unsaved_invalid_points_do_not_replace_valid_mcu_force_or_write_flash(self):
        commands=[]
        reader=SimpleNamespace(_protocol_version_confirmed=True,_protocol_version=19,_channel_count=8,
                               low_force_command=lambda *args:commands.append(args) or True)
        sample=dict(signal=[0]*8,age_seconds=0,mcu_force_age_seconds=0,
            mcu_force_vector=dict(low_force_mode=True,low_force_calibration_valid=True,
                                  normal_calibration_valid=True,normal_force=0,optimized_normal_force=0))
        widget=LowForceCalibrationWidget(lambda:reader,lambda:sample)
        try:
            for editor,value in zip(widget.inputs,(100,2000,5000,1000)):
                editor.setText(str(value));editor.textEdited.emit(str(value))
            widget.refresh();widget.save_button.click()
            self.assertFalse(commands)
            self.assertIn('严格递增',widget.input_validation_label.text())
            self.assertIn('本次未下发',widget.status_label.text())
            self.assertIn('0.000 N',widget.force_label.text())
            self.assertIn('0.000 N',widget.optimized_force_label.text())
            sample['mcu_force_vector']['low_force_calibration_valid']=False
            widget.refresh()
            self.assertIn('低力标定未保存或无效',widget.force_label.text())
            self.assertIn('低力标定未保存或无效',widget.optimized_force_label.text())
        finally:widget.close()

    def test_force_unavailable_reasons_visible_and_stale_values_are_not_reused(self):
        vector=dict(low_force_mode=True,low_force_calibration_valid=True,
                    normal_calibration_valid=True,normal_force=.2,optimized_normal_force=.196133)
        sample=dict(mcu_force_vector=vector,mcu_force_age_seconds=0)
        widget=LowForceCalibrationWidget(lambda:None,lambda:sample)
        try:
            self.assertIn('0.200 N',widget.force_label.text())
            sample['mcu_force_age_seconds']=1;widget.refresh()
            self.assertIn('已超时',widget.force_label.text())
            self.assertNotIn('0.200',widget.force_label.text())
            sample['mcu_force_age_seconds']=0
            vector['normal_calibration_valid']=False;widget.refresh()
            self.assertIn('等待MCU完成基线校准',widget.force_label.text())
            vector['normal_calibration_valid']=True
            vector['optimized_normal_force']=None;widget.refresh()
            self.assertIn('0.200 N',widget.force_label.text())
            self.assertIn('优化值未上传',widget.optimized_force_label.text())
            vector['low_force_mode']=False;widget.refresh()
            self.assertIn('未上报低力模式',widget.force_label.text())
            sample['mcu_force_vector']=None;widget.refresh()
            self.assertIn('未收到MCU力值数据',widget.force_label.text())
        finally:widget.close()

    def test_save_read_startup_preserves_other_flash_and_wear(self):
        fw = LowFirmware()
        other = bytes(fw.uc.mem_read(0x1F600, 0xA00))
        self.assertFalse(fw.low_command()['ram_valid'])
        reply = fw.low_command(True)
        self.assertEqual((reply['result'], reply['token'], reply['points']), (0, 456, POINTS))
        self.assertTrue(reply['flash_valid'] and reply['ram_valid'])
        self.assertEqual(fw.writes, [0x1F500])
        fw.low_command(True)
        self.assertEqual(fw.writes, [0x1F500])
        fw.uc.mem_write(fw.symbols['low_force_valid'], b'\0')
        fw.uc.mem_write(fw.symbols['low_force_points'], bytes(16))
        fw.call('SensorAlgorithm_Init')
        self.assertEqual(fw.low_command()['points'], POINTS)
        self.assertEqual(bytes(fw.uc.mem_read(0x1F600, 0xA00)), other)
        fw.stage()
        fw.command(0x85)
        self.assertEqual(fw.low_command()['points'], POINTS)

    def test_mass_conversion_interpolation_and_cutoffs(self):
        fw = LowFirmware()
        self.assertEqual(fw.evaluate(200), (0, 0))
        fw.low_command(True)
        for signal, expected in zip(POINTS, FORCES_N):
            state, value = fw.evaluate(signal)
            self.assertEqual(state, 1)
            self.assertAlmostEqual(value, expected, places=6)
        for expected in (0.0, 0.0899, 0.0901, 0.0999, 0.1001, 0.25, 0.75, 0.9999, 1.0001, 5.0):
            state, value = fw.evaluate(expected / 0.000980665)
            self.assertEqual(state, 2 if expected >= 1 else 1)
            self.assertAlmostEqual(value, expected if 0.09 <= expected < 1 else 0, places=6)

    def test_invalid_inputs_and_malformed_commands_never_write(self):
        fw = LowFirmware()
        for points in ((0, 2, 3, 4), (-1, 2, 3, 4), (1, 1, 2, 3), (4, 3, 2, 1),
                       (1, 2, 3, float('inf')), (1, 2, 3, float('nan')), (1, 2, 3, 262141)):
            self.assertEqual(fw.low_command(True, points)['result'], 1)
        for payload in (b'', bytes(3), bytes(19), bytes(20), bytes(23), bytes(25)):
            fw.command(0x99, payload)
            self.assertEqual(AlgoFrameParser().feed(fw.frames[-1])[0]['result'], 1)
        self.assertEqual(fw.writes, [])

    def test_write_failure_and_corruption_keep_ram_but_not_false_persistence(self):
        for failure in ('fail_address', 'corrupt_address'):
            fw = LowFirmware()
            fw.low_command(True)
            setattr(fw, failure, 0x1F500)
            reply = fw.low_command(True, (110, 220, 550, 1100))
            self.assertEqual(reply['result'], 4)
            self.assertEqual(reply['points'], POINTS)
            self.assertTrue(reply['ram_valid'])
            if failure == 'corrupt_address':
                self.assertFalse(reply['flash_valid'])
                self.assertFalse(fw.low_command()['ram_valid'])
                self.assertEqual(fw.evaluate(200), (0, 0))

    def test_wrong_device_and_incomplete_channels(self):
        fw = LowFirmware()
        fw.low_command(True)
        record = bytearray(fw.uc.mem_read(0x1F500, 256))
        record[5] = 2
        fw.uc.mem_write(0x1F500, bytes(record))
        self.assertFalse(fw.low_command()['ram_valid'])
        for variant in ('THUMB', 'PINKY'):
            fw = LowFirmware(variant)
            self.assertEqual(fw.low_command(True)['result'], 3)
            self.assertEqual(fw.writes, [])

    def test_uart_no_old_k_required_and_all_force_displays(self):
        fw = LowFirmware()
        for calibrated, total, expected in ((False, 500, '-- N'), (True, 100, '0.098 N'),
                                             (True, 200, '0.196 N'), (True, 1000, '0.981 N'),
                                             (True, 1020, '超出低力量程')):
            if calibrated:
                fw.low_command(True)
            self.assertFalse(fw.flag('normal_valid'))
            frame = fw.run_frame(total)
            buffer = AlgoDataBuffer(5)
            buffer.append_frame(frame)
            # 同正式路径将Type0x02转换为上位机法向力及有效位。
            from data_buffer import _make_run_force_vector
            vector = _make_run_force_vector(frame, 0, 0)
            self.assertEqual(low_force_display(vector)[0], expected)
            widget = RunForceWidget('test')
            widget.set_vector(vector)
            self.assertIn(expected, widget.normal_label.text())
            fake = SimpleNamespace(mcu_force_value_label=QtWidgets.QLabel(), mcu_force_state_label=QtWidgets.QLabel(),
                                   _set_mcu_force_display_state=lambda state: None)
            ForcePressControlWidget._update_mcu_force_value(fake, {'mcu_force_vector': vector, 'mcu_force_age_seconds': 0})
            self.assertEqual(fake.mcu_force_value_label.text(), expected)
            widget.close()

    def test_host_protocol_guard_and_float_encoding(self):
        reader = SerialReaderThread('FAKE', AlgoDataBuffer(5))
        sent = []
        reader.send_command = lambda packet: sent.append(packet) or True
        self.assertFalse(reader.low_force_command(0x99, 5, POINTS))
        reader._protocol_version_confirmed = True
        reader._protocol_version = 16
        self.assertFalse(reader.low_force_command(0x99, 5, POINTS))
        reader._protocol_version = 17
        self.assertTrue(reader.low_force_command(0x99, 5, POINTS))
        self.assertEqual(struct.unpack('<I4f', sent[-1][10:]), (5, *POINTS))
        self.assertEqual(sent[-1][4:6], bytes((17, 0x99)))
        self.assertFalse(reader.low_force_command(0x99, 5, (0, 1, 2, 3)))

    def test_manual_tab_save_read_timeout_stale_reply_and_input_preserved(self):
        commands = []
        reader = SimpleNamespace(_protocol_version_confirmed=True, _protocol_version=17, _channel_count=8,
                                 low_force_command=lambda *args: commands.append(args) or True)
        widget = LowForceCalibrationWidget(lambda: reader, lambda: None)
        widget.save_button.click()
        self.assertEqual(commands, [])
        for editor, value in zip(widget.inputs, POINTS):
            editor.setText(str(value))
        widget.save_button.click()
        operation, token, points = commands[-1]
        self.assertFalse(widget.save_button.isEnabled())
        reply = dict(operation=operation, token=token+1, result=0, flash_valid=True, ram_valid=True, points=points)
        widget.handle_response(reply)
        self.assertIsNotNone(widget._pending)
        reply['token'] = token
        widget.handle_response(reply)
        self.assertIn('已保存', widget.status_label.text())
        widget.read_button.click()
        operation, token, _ = commands[-1]
        widget.handle_response(dict(reply, operation=operation, token=token))
        self.assertIn('已读取', widget.status_label.text())
        widget.save_button.click()
        widget._timeout.timeout.emit()
        self.assertIn('结果未知', widget.status_label.text())
        self.assertEqual(tuple(float(e.text()) for e in widget.inputs), POINTS)
        widget.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
