"""运行真实固件ELF和现有GUI，验证Baseline宏关闭后的Raw接收与3D柱体。"""
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

from test_matrix20_firmware import Firmware, ROOT

sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from data_buffer import AlgoDataBuffer
from data_recorder import SerialDataRecorder
from frame_parser import AlgoFrameParser
import test_matrix20_firmware as firmware_module


def packet(raw, *, version=21, device=1, channels=None):
    payload = struct.pack(f'<{len(raw)}H', *raw)
    return struct.pack('<4sBBBBH', b'DATA', version, 0x16, device,
                       len(raw) if channels is None else channels, len(payload)) + payload


class RawModeTests(unittest.TestCase):
    def test_every_split_and_invalid_frame_recovery(self):
        raw = (0, 65535, 0x4144, 0x4154, *range(16))
        wire = packet(raw)
        for split in range(len(wire) + 1):
            parser = AlgoFrameParser()
            frames = parser.feed(wire[:split]) + parser.feed(wire[split:])
            self.assertEqual(len(frames), 1)
            frame = frames[0]
            self.assertEqual(frame['raw'], raw)
            self.assertEqual(frame['baseline'], (0,) * 20)
            self.assertEqual(frame['signal'], (0,) * 20)
            self.assertEqual(frame['state'], (-1,) * 20)
            self.assertTrue(all(math.isnan(v) for v in frame['fast_k_percent']))
        malformed = packet(raw, version=20) + packet((), channels=0) + packet(raw, channels=19)
        self.assertEqual(AlgoFrameParser().feed(malformed + wire)[0]['raw'], raw)
        parser, frames = AlgoFrameParser(), []
        for byte in b'noiseDAT' + wire * 3:
            frames += parser.feed(bytes([byte]))
        self.assertEqual([f['raw'] for f in frames], [raw] * 3)

    def test_real_firmware_raw_outputs_all_variants_and_output_modes(self):
        original = firmware_module.ELF
        try:
            for finger, device, count in (('INDEX_FINGER', 1, 20), ('THUMB', 2, 5), ('PINKY', 3, 5)):
                firmware_module.ELF = ROOT / f'build/{finger}/UART/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-{finger}.elf'
                for mode in range(3):
                    fw = Firmware()
                    fw.call('initialize_baseline_runtime')
                    fw.uc.mem_write(fw.symbols['output_mode'], bytes([mode]))
                    # 只隔离硬件UART FIFO操作，组帧及发送队列执行真实固件代码。
                    fw.stub_success('SensorAlgorithm_UartTxPoll')
                    raw = tuple([0, 65535] + [42000 + i for i in range(count - 2)])
                    fw.uc.mem_write(0x20100000, struct.pack(f'<{count}H', *raw))
                    fw.uc.mem_write(0x20100100, b'\xff' * (count * 28))
                    fw.call('SensorAlgorithm_Process', 0x20100000, 0x20100100)
                    outputs = [struct.unpack('<HHH2xiiiiH2x', fw.uc.mem_read(0x20100100 + i * 28, 28))
                               for i in range(count)]
                    self.assertEqual(tuple(o[0] for o in outputs), raw)
                    self.assertTrue(all(o[1:7] == (0, 0, 0, 0, 0, 0) for o in outputs))
                    # 重新校准不会导致Raw静默、改变输出或启动基线处理。
                    fw.call('SensorAlgorithm_RequestRecalibration')
                    fw.call('SensorAlgorithm_ProcessAndSend', 0x20100000)
                    self.assertEqual(struct.unpack('<I', fw.uc.mem_read(fw.symbols['uart_tx_count'], 4))[0], 1)
                    size = struct.unpack('<I', fw.uc.mem_read(fw.symbols['uart_tx_sizes'], 4))[0]
                    self.assertEqual(size, 10 + count * 2)
                    wire = bytes(fw.uc.mem_read(fw.symbols['uart_tx_buffers'], size))
                    self.assertEqual(wire, packet(raw, device=device))
                    frame = AlgoFrameParser().feed(wire)[0]
                    buffer = AlgoDataBuffer(5)
                    buffer.append_frame(frame)
                    sample = buffer.latest_sensor_sample()
                    self.assertTrue(sample['has_raw'])
                    self.assertEqual(sample['raw'], raw)
                    self.assertEqual(sample['baseline'], (0,) * count)
                    self.assertEqual(sample['signal'], (0,) * count)
                    with tempfile.TemporaryDirectory() as directory:
                        recorder = SerialDataRecorder(directory)
                        recorder.record(frame)
                        exported = recorder.write_recent_snapshot(recorder.snapshot_recent(1))
                        csv = Path(exported['output_path']).read_text(encoding='utf-8-sig')
                        self.assertIn('65535', csv)
        finally:
            firmware_module.ELF = original

    def test_existing_monitor_3d_raw_without_gui_changes(self):
        script = r'''
import os, struct, sys
from pathlib import Path
from qt_compat import QtGui, QtWidgets
from serial_ports import SerialPortScanner
from sensor_window import MainWindow
from frame_parser import AlgoFrameParser
SerialPortScanner.list_ports = lambda self: ([], '')
app = QtWidgets.QApplication([])
for name in ('msyh.ttc', 'segoeui.ttf'):
    font = Path('C:/Windows/Fonts') / name
    if font.exists():
        QtGui.QFontDatabase.addApplicationFont(str(font))
window = MainWindow()
window.show()
window.refresh_timer.stop()
view, panel = window.array_bars, window.array_bar_settings
window.plot_tabs.setCurrentWidget(view)
panel.source.setCurrentIndex(panel.source.findData('raw'))
raw = tuple(24000 + i * 2000 for i in range(20))
wire = struct.pack('<4sBBBBH20H', b'DATA', 21, 0x16, 1, 20, 40, *raw)
frame = AlgoFrameParser().feed(wire)[0]
window.data_buffer.append_frame(frame)
window.refresh_plots()
app.processEvents()
assert view.values == raw
assert all(b['height'] > 0 and b['value'] == raw[b['channel']] for b in view.bars())
assert window.value_labels['baseline_value'][0].text() == '0'
assert window.value_labels['signal_value'][0].text() == '0'
assert window.data_buffer.latest_sensor_sample()['state'] == (-1,) * 20
if os.environ.get('RAW_MODE_SCREENSHOT'):
    window.grab().save(os.environ['RAW_MODE_SCREENSHOT'])
window.close()
app.processEvents()
'''
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT / 'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path, directory)
            path = Path(directory) / 'raw_mode_smoke.py'
            path.write_text(script, encoding='utf-8')
            env = dict(os.environ, QT_QPA_PLATFORM='offscreen', PYTHONIOENCODING='utf-8')
            result = subprocess.run([sys.executable, str(path)], cwd=directory, env=env,
                                    capture_output=True, text=True, encoding='utf-8', timeout=40)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_real_i2c_frames_keep_raw_value_encoding(self):
        original = firmware_module.ELF
        try:
            for finger, count in (('INDEX_FINGER', 20), ('THUMB', 5), ('PINKY', 5)):
                firmware_module.ELF = ROOT / f'build/{finger}/I2C/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-{finger}.elf'
                fw = Firmware()
                fw.call('initialize_baseline_runtime')
                raw = tuple([0, 65535] + [40000 + i for i in range(count - 2)])
                fw.uc.mem_write(0x20100000, struct.pack(f'<{count}H', *raw))
                fw.call('SensorAlgorithm_Process', 0x20100000, 0x20100100)
                fw.call('app_iic_build_live_data_frame', 0x20101000)
                wire = bytes(fw.uc.mem_read(0x20101000, 30 + count * 3))
                self.assertEqual(wire[:4], b'\x55\xaa\x03\x85')
                self.assertEqual(wire[5], count)
                encoded = [int.from_bytes(wire[6 + i * 3:9 + i * 3], 'little') for i in range(count)]
                self.assertEqual(encoded, [v << 1 for v in raw])
                self.assertEqual(int.from_bytes(wire[-2:], 'little'), sum(wire[:-2]) & 65535)
        finally:
            firmware_module.ELF = original


if __name__ == '__main__':
    unittest.main(verbosity=2)
