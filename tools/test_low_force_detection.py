"""在真实ARM产物中验证最低检测线、模拟边界、旧Flash兼容和上位机记录。"""
import csv
import io
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_low_force_calibration import LowFirmware
from elftools.elf.elffile import ELFFile
from test_flash_two_range import ROOT, checksum
from data_buffer import _make_run_force_vector, AlgoDataBuffer
from data_recorder import SerialDataRecorder
from frame_parser import AlgoFrameParser
from low_force_calibration import low_force_display
from low_force_calibration_widget import LowForceCalibrationWidget
from serial_reader import SerialReaderThread
from qt_compat import QtWidgets
from config import FRAME_TYPE_CHANNEL_4567_SIGNAL_SUM, FRAME_TYPE_CHANNEL_4567_STATISTICS, FRAME_VERSION

POINTS = (200.0, 400.0, 1000.0, 2000.0)
N_PER_G = 0.00980665


class DetectionFirmware(LowFirmware):
    def __init__(self):
        super().__init__()
        path = ROOT / 'build/INDEX_FINGER/UART/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-INDEX_FINGER.elf'
        elf = ELFFile(io.BytesIO(path.read_bytes()))
        for cu in elf.get_dwarf_info().iter_CUs():
            for die in cu.iter_DIEs():
                name = die.attributes.get('DW_AT_name')
                if die.tag == 'DW_TAG_typedef' and name and name.value == b'channel_state_machine_t':
                    record = die.get_DIE_from_attribute('DW_AT_type')
                    self.state_stride = record.attributes['DW_AT_byte_size'].value
                    for member in record.iter_children():
                        if member.tag == 'DW_TAG_member' and member.attributes['DW_AT_name'].value == b'current_state':
                            self.state_offset = member.attributes['DW_AT_data_member_location'].value
                            dtype = member.get_DIE_from_attribute('DW_AT_type')
                            while 'DW_AT_byte_size' not in dtype.attributes:
                                dtype = dtype.get_DIE_from_attribute('DW_AT_type')
                            self.state_bytes = dtype.attributes['DW_AT_byte_size'].value
                    return
        raise AssertionError('state layout absent')

    def states(self, floating=(), other_state=0):
        # 仅向仿真内存注入已提交状态；产品代码仍只读统一状态机。
        for channel in range(8):
            address = self.symbols['channel_state_machines'] + channel * self.state_stride + self.state_offset
            self.uc.mem_write(address, (6 if channel in floating else other_state).to_bytes(self.state_bytes, 'little'))


class DetectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_threshold_continues_across_all_states_and_preserves_measurement(self):
        fw = DetectionFirmware()
        fw.low_command(True, POINTS, detection_counts=120)
        for floating, total, active in (((), 150, True), ((0, 1, 2, 3), 150, True),
                                        ((4,), 120, False), ((7,), 119, False),
                                        ((4,), 121, True), ((7,), 199, True),
                                        ((4, 5, 6, 7), 150, True), ((6,), 200, False),
                                        ((5,), 400, False), ((5,), 2040, False), ((), 150, True)):
            fw.states(floating)
            before = bytes(fw.uc.mem_read(fw.symbols['channel_state_machines'], fw.state_stride * 8))
            baseline = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 32))
            frame = fw.run_frame(total)
            self.assertEqual(frame['low_force_simulated'], active, (floating, total))
            self.assertAlmostEqual(frame['measured_normal_force'] / N_PER_G, total / 20, places=4)
            self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['channel_state_machines'], fw.state_stride * 8)), before)
            self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 32)), baseline)
            if active:
                self.assertAlmostEqual(frame['original_normal_force'] / N_PER_G, 8.88, places=5)
                self.assertGreaterEqual(frame['optimized_normal_force'] / N_PER_G, 9.99999)
                self.assertLess(frame['optimized_normal_force'] / N_PER_G, 10.995)
                if total < 184:
                    self.assertEqual(frame['measured_optimized_normal_force'], 0)
                self.assertNotIn('[模拟]', low_force_display(_make_run_force_vector(frame, 0, 0))[0])
            else:
                self.assertEqual(frame['optimized_normal_force'], frame['measured_optimized_normal_force'])
        fw.states((7,))
        self.assertGreater(len({fw.run_frame(150)['optimized_normal_force'] for _ in range(20)}), 5)
        fw.low_command(True, POINTS, detection_counts=0)
        self.assertFalse(fw.run_frame(150)['low_force_simulated'])
        for state in (0, 1, 2, 3, 4, 5, 7, 8):
            fw.low_command(True, POINTS, detection_counts=120)
            fw.states((), state)
            self.assertTrue(fw.run_frame(150)['low_force_simulated'])
        # 更改并重新加载阈值后，触发边界随输入移动，不能固定使用120。
        fw.low_command(True, POINTS, detection_counts=160)
        fw.call('SensorAlgorithm_Init')
        self.assertFalse(fw.run_frame(150)['low_force_simulated'])
        self.assertTrue(fw.run_frame(170)['low_force_simulated'])
        fw.uc.mem_write(0x1F500, b'\xff' * 256)
        fw.low_command()
        fw.states((7,))
        self.assertFalse(fw.run_frame(150)['low_force_simulated'])

    def test_persistence_old_record_validation_and_failure(self):
        fw = DetectionFirmware()
        record = struct.pack('<IBBBB4f', 0x3143464C, 1, 1, 4, 0, *POINTS)
        fw.uc.mem_write(0x1F500, record + struct.pack('<I', checksum(record)) + b'\xff' * 228)
        reply = fw.low_command()
        self.assertTrue(reply['ram_valid'])
        self.assertEqual((reply['points'], reply['detection_counts']), (POINTS, 0))
        self.assertEqual(fw.writes, [])
        for invalid in (-1, 200, 201, float('nan'), float('inf')):
            self.assertEqual(fw.low_command(True, POINTS, detection_counts=invalid)['result'], 1)
        self.assertEqual(fw.writes, [])
        reply = fw.low_command(True, POINTS, detection_counts=120)
        self.assertEqual(reply['detection_counts'], 120)
        self.assertEqual(fw.uc.mem_read(0x1F504, 1), b'\x02')
        fw.call('SensorAlgorithm_Init')
        self.assertEqual(fw.low_command()['detection_counts'], 120)
        fw.low_command(True, POINTS, detection_counts=120)
        self.assertEqual(fw.writes, [0x1F500])
        fw.fail_address = 0x1F500
        reply = fw.low_command(True, POINTS, detection_counts=110)
        self.assertEqual((reply['result'], reply['detection_counts']), (4, 120))
        fw.fail_address = None
        fw.corrupt_address = 0x1F500
        reply = fw.low_command(True, POINTS, detection_counts=110)
        self.assertEqual((reply['result'], reply['detection_counts']), (4, 120))
        self.assertFalse(reply['flash_valid'])
        reply = fw.low_command()
        self.assertFalse(reply['ram_valid'])
        self.assertEqual(reply['detection_counts'], 0)
        self.assertEqual(bytes(fw.uc.mem_read(0x1FD00, 768)), fw.protected)

    def test_host_save_read_and_legacy_guard(self):
        reader = SerialReaderThread('FAKE', AlgoDataBuffer(5))
        sent = []
        reader.send_command = lambda frame: sent.append(frame) or True
        reader._protocol_version_confirmed = True
        reader._channel_count = 8
        reader._protocol_version = 18
        self.assertFalse(reader.low_force_command(0x99, 1, POINTS, 120))
        reader._protocol_version = 19
        self.assertTrue(reader.low_force_command(0x99, 1, POINTS, 120))
        self.assertEqual(struct.unpack('<I5f', sent[-1][10:]), (1, *POINTS, 120))
        fw = DetectionFirmware()
        commands = []
        reader.low_force_command = lambda *args: commands.append(args) or True
        with tempfile.TemporaryDirectory() as directory:
            widget = LowForceCalibrationWidget(lambda: reader, lambda: None, settings_path=Path(directory)/'view.json')
            for editor, value in zip(widget.inputs, POINTS):
                editor.setText(str(value))
            widget.detection_input.setText('200')
            widget.save_button.click()
            self.assertFalse(commands)
            widget.detection_input.setText('120')
            widget.save_button.click()
            op, token, points, threshold = commands[-1]
            self.assertFalse(widget.detection_input.isEnabled())
            reply = fw.low_command(True, points, token, threshold)
            widget.handle_response(dict(reply, token=token+1))
            self.assertIsNotNone(widget._pending)
            widget.handle_response(dict(reply, detection_counts=110))
            self.assertIn('不一致', widget.status_label.text())
            self.assertEqual(widget.detection_input.text(), '120')
            widget.save_button.click()
            op, token, points, threshold = commands[-1]
            widget.handle_response(fw.low_command(True, points, token, threshold))
            self.assertIn('已保存', widget.status_label.text())
            widget.detection_input.setText('99')
            widget.read_button.click()
            widget.handle_response(fw.low_command(token=commands[-1][1]))
            self.assertEqual(widget.detection_input.text(), '120')
            widget.close()

    def test_single_click_read_after_subgroup_frames_uses_device_channel_count(self):
        fw = DetectionFirmware()
        fw.low_command(True, POINTS, detection_counts=120)
        for channel_count in (8, 5):
            with self.subTest(channel_count=channel_count), tempfile.TemporaryDirectory() as directory:
                def packet(kind, count, payload):
                    return struct.pack('<4sBBBBH', b'DATA', FRAME_VERSION, kind, 1, count, len(payload)) + payload
                # 重现整设备采样帧后紧跟四通道统计帧，然后用户立即点击读取。
                mixed = packet(1, channel_count, struct.pack('<'+'H'*channel_count, *([20]*channel_count)))
                mixed += packet(FRAME_TYPE_CHANNEL_4567_STATISTICS, 4, struct.pack('<III', 150, 4000, 20))
                mixed += packet(FRAME_TYPE_CHANNEL_4567_SIGNAL_SUM, 4, struct.pack('<I', 150))
                chunks = [mixed[:7], mixed[7:]]
                reader = SerialReaderThread('FAKE', AlgoDataBuffer(5))
                widget = LowForceCalibrationWidget(lambda: reader, lambda: None, settings_path=Path(directory)/'view.json')
                reader.low_force_calibration.connect(widget.handle_response)
                writes = []
                clicked = False

                def write(wire):
                    writes.append(wire)
                    if wire[5] == 0x98:
                        fw.command(0x98, wire[10:])
                        chunks.append(fw.frames[-1])
                    return len(wire)

                def read(_size):
                    nonlocal clicked
                    if chunks:
                        return chunks.pop(0)
                    if not clicked:
                        clicked = True
                        self.assertEqual(reader._channel_count, channel_count)
                        widget.read_button.click()
                        if chunks:
                            return chunks.pop(0)
                    reader._running = False
                    return b''

                fake = SimpleNamespace(is_open=True, in_waiting=1024, read=read, write=write, flush=lambda: None, close=lambda: None)
                with patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=lambda *a, **k: fake)}):
                    reader.run()
                reads = [wire for wire in writes if wire[5] == 0x98]
                self.assertEqual(len(reads), 1 if channel_count == 8 else 0)
                if channel_count == 8:
                    self.assertIn('已读取', widget.status_label.text())
                    self.assertEqual(tuple(float(e.text()) for e in widget.inputs), POINTS)
                    self.assertEqual(widget.detection_input.text(), '120')
                    self.assertIsNone(widget._pending)
                else:
                    self.assertIn('八通道', widget.status_label.text())
                widget.close()

    def test_tab_entry_auto_read_waits_for_connection_and_preserves_edits(self):
        commands = []
        reader = SimpleNamespace(_protocol_version_confirmed=False, _protocol_version=19, _channel_count=0,
                                 low_force_command=lambda *args: commands.append(args) or True)
        current = [None]
        busy = [False]
        with tempfile.TemporaryDirectory() as directory:
            tabs = QtWidgets.QTabWidget()
            tabs.addTab(QtWidgets.QWidget(), 'monitor')
            widget = LowForceCalibrationWidget(lambda: current[0], lambda: None, lambda: busy[0],
                                               settings_path=Path(directory)/'view.json')
            tabs.addTab(widget, 'calibration')
            tabs.show()
            self.app.processEvents()
            self.assertFalse(commands)
            tabs.setCurrentIndex(1)
            self.app.processEvents()
            self.assertFalse(commands)
            current[0] = reader
            widget.refresh()
            self.assertFalse(commands)
            reader._protocol_version_confirmed = True
            reader._channel_count = 8
            busy[0] = True
            widget.refresh()
            self.assertFalse(commands)
            busy[0] = False
            widget.refresh()
            self.assertEqual(len(commands), 1)
            self.assertEqual(commands[-1][0], 0x98)

            def respond():
                widget.handle_response(dict(operation=0x98, token=commands[-1][1], result=0,
                                            flash_valid=True, ram_valid=True, points=POINTS, detection_counts=120))

            respond()
            widget.refresh()
            self.assertEqual(len(commands), 1)
            self.assertEqual(tuple(float(e.text()) for e in widget.inputs), POINTS)
            self.assertEqual(widget.detection_input.text(), '120')
            tabs.setCurrentIndex(0)
            tabs.setCurrentIndex(1)
            self.app.processEvents()
            self.assertEqual(len(commands), 2)
            respond()
            widget.detection_input.setText('50')
            widget.detection_input.textEdited.emit('50')
            tabs.setCurrentIndex(0)
            tabs.setCurrentIndex(1)
            self.app.processEvents()
            self.assertEqual(len(commands), 2)
            self.assertEqual(widget.detection_input.text(), '50')
            self.assertIn('保留输入', widget.status_label.text())
            widget.read_button.click()
            self.assertEqual(len(commands), 3)
            respond()
            self.assertFalse(widget._inputs_dirty)
            # 新连接在当前页自动读取；超时后不会按刷新定时器反复发送。
            current[0] = SimpleNamespace(**vars(reader))
            widget.refresh()
            self.assertEqual(len(commands), 4)
            widget._timeout.timeout.emit()
            widget.refresh()
            self.assertEqual(len(commands), 4)
            self.assertIn('超时', widget.status_label.text())
            tabs.close()

    def test_wire_versions_csv_and_visible_simulation(self):
        fw = DetectionFirmware()
        fw.low_command(True, POINTS, detection_counts=120)
        fw.states((7,))
        frame = fw.run_frame(150)
        packet = fw.frames[-1]
        parser = AlgoFrameParser()
        fragments = []
        for value in packet:
            fragments.extend(parser.feed(bytes([value])))
        self.assertEqual(fragments, [frame])
        for version, length in ((17, 28), (18, 32)):
            legacy = bytearray(packet[:length + 10])
            legacy[4] = version
            struct.pack_into('<H', legacy, 8, length)
            old = AlgoFrameParser().feed(legacy)[0]
            self.assertFalse(old['low_force_simulated'])
            self.assertIsNone(old['measured_normal_force'])
        with tempfile.TemporaryDirectory() as directory:
            recorder = SerialDataRecorder(directory)
            recorder.record(frame)
            recorder.record({'type': 'signal', 'device': 1, 'channel_count': 8, 'signal': (0,)*8})
            result = recorder.write_recent_snapshot(recorder.snapshot_recent(1))
            with Path(result['output_path']).open(encoding='utf-8-sig', newline='') as handle:
                row = next(row for row in csv.DictReader(handle) if row['type'] == 'run_force')
            self.assertIn(row['low_force_simulated'], ('True', '1'))
            self.assertAlmostEqual(float(row['measured_normal_force']) / N_PER_G, 7.5, places=5)
            vector = _make_run_force_vector(frame, 0, 0)
            sample = dict(mcu_force_vector=vector, mcu_force_age_seconds=0)
            widget = LowForceCalibrationWidget(lambda: None, lambda: sample, settings_path=Path(directory)/'view.json')
            self.assertIn('8.88 g', widget.force_label.text())
            self.assertNotIn('[模拟]', widget.force_label.text())
            self.assertNotIn('[模拟]', widget.optimized_force_label.text())
            self.assertIn('7.50 g', widget.measured_force_label.text())
            sample['mcu_force_age_seconds'] = 1
            widget.refresh()
            self.assertNotIn('8.88 g', widget.force_label.text())
            self.assertTrue(widget.measured_force_label.isHidden())
            widget.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
