"""执行实际固件统计/组帧代码，验证全量程精度及上位机显示记录链路。"""
import io
import math
import os
from pathlib import Path
import random
import statistics
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tmp/state_test_deps'), str(ROOT / 'PY_SRIPT/algo_monitor')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC
from data_buffer import AlgoDataBuffer
from data_recorder import SerialDataRecorder
from frame_parser import AlgoFrameParser
from force_press_control_widget import ForcePressControlWidget
from main_window import MainWindow
from test_missing_debug_data import signal_packet, debug_packet, packet
from test_channel_state_pressure import compiler_path


def extract_function(source, name):
    import re
    match = re.search(r'static [^\n]+\b' + name + r'\([^;]+?\) \{', source)
    start = source.index('{', match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


class HardwareStatisticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (ROOT / 'sensor_algorithm.c').read_text(encoding='utf-8')
        directory = ROOT / 'build/raw_statistics'
        directory.mkdir(parents=True, exist_ok=True)
        harness = '''#include <stdint.h>
#define SENSOR_ALGO_CHANNEL_COUNT 8
#define SENSOR_ALGO_PROTOCOL_HEADER_SIZE 10
#define SENSOR_ALGO_FRAME_TYPE_CHANNEL_4567_STATISTICS 0x14
typedef struct { uint16_t signal; } sensor_algorithm_output_t;
uint16_t latest_hardware_raw[8];
sensor_algorithm_output_t outputs[8];
uint8_t captured[22];
/* 测试桩仅提供传输接口，统计及组帧执行固件原函数。 */
static void write_u32(uint8_t *p, uint32_t *o, uint32_t v) {
    for (unsigned i=0;i<4;i++) p[(*o)++]=(uint8_t)(v>>(8*i));
}
/* 生成与串口协议相同的固定头。 */
static void write_frame_header(uint8_t *p,uint32_t *o,uint8_t t,uint8_t n,uint16_t len) {
    p[0]='D';p[1]='A';p[2]='T';p[3]='A';p[4]=15;p[5]=t;p[6]=1;p[7]=n;p[8]=len;p[9]=len>>8;*o=10;
}
/* 捕获固件发送结果供Python真实解析器验证。 */
static void send_output_frame(const uint8_t *p,uint32_t n) {
    for(unsigned i=0;i<n;i++) captured[i]=p[i];
}
'''
        for name in ('isqrt_u64', 'calculate_hardware_raw_stddev_x100', 'send_channel_4567_statistics_frame'):
            harness += '\n/* 执行实际固件函数。 */\n' + extract_function(source, name)
        harness += '\n/* 执行统计和发送。 */\nvoid test_run(void) { send_channel_4567_statistics_frame(outputs); }\n'
        (directory / 'harness.c').write_text(harness, encoding='utf-8')
        elf_path = directory / 'statistics.elf'
        result = subprocess.run([compiler_path(), '-mcpu=cortex-m0plus', '-mthumb', '-Os', '-g', '-Wall', '-Wextra', '-Werror', '-std=c99', '-nostartfiles', '--specs=nosys.specs', str(directory / 'harness.c'), '-Wl,-e,test_run,-Ttext=0x10000,-Tdata=0x20000000', '-o', str(elf_path)], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        elf = ELFFile(io.BytesIO(elf_path.read_bytes()))
        cls.segments = [(s['p_vaddr'], s.data()) for s in elf.iter_segments() if s['p_type'] == 'PT_LOAD']
        cls.symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

    def firmware_frame(self, raw, signals=(0, 0, 0, 0)):
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(0x10000, 0x100000)
        uc.mem_map(0x20000000, 0x200000)
        for address, data in self.segments:
            uc.mem_write(address, data)
        # 前四通道故意使用不同值，确保只计算CH4～CH7。
        uc.mem_write(self.symbols['latest_hardware_raw'], struct.pack('<8H', 65535, 0, 333, 777, *raw))
        uc.mem_write(self.symbols['outputs'], struct.pack('<8H', 1, 2, 3, 4, *signals))
        uc.reg_write(UC_ARM_REG_SP, 0x201FF000)
        uc.reg_write(UC_ARM_REG_LR, 0xF0001)
        uc.emu_start(self.symbols['test_run'] | 1, 0xF0000, count=100000)
        self.assertEqual(uc.reg_read(UC_ARM_REG_PC), 0xF0000)
        return bytes(uc.mem_read(self.symbols['captured'], 22))

    def test_full_range_and_small_differences(self):
        rng = random.Random(4567)
        cases = [(0,)*4, (65535,)*4, (65535,65535,0,0), (65535,65534,65535,65534), (0,0,0,1), (541,396,349,767)]
        cases += [tuple(rng.randrange(65536) for _ in range(4)) for _ in range(300)]
        for raw in cases:
            with self.subTest(raw=raw):
                frame = AlgoFrameParser().feed(self.firmware_frame(raw, (65535,)*4))[0]
                self.assertEqual(frame['hardware_raw_sum'], sum(raw))
                self.assertEqual(frame['hardware_raw_mean'], statistics.mean(raw))
                self.assertLessEqual(abs(frame['hardware_raw_stddev'] - statistics.pstdev(raw)), 0.00500001)
                self.assertEqual(frame['signal_sum'], 262140)

    def test_offset_invariance_and_zero_signal(self):
        a = AlgoFrameParser().feed(self.firmware_frame((10,20,30,40)))[0]
        b = AlgoFrameParser().feed(self.firmware_frame((60010,60020,60030,60040)))[0]
        self.assertEqual(a['hardware_raw_stddev'], b['hardware_raw_stddev'])
        self.assertEqual(a['signal_sum'], 0)
        self.assertGreater(a['hardware_raw_stddev'], 0)

    def test_split_packets_buffer_ui_and_csv(self):
        data = self.firmware_frame((100,200,300,400)) + debug_packet()
        parser = AlgoFrameParser()
        frames = []
        for byte in data:
            frames.extend(parser.feed(bytes([byte])))
        buffer = AlgoDataBuffer(5)
        for frame in frames:
            buffer.append_frame(frame)
        sample = buffer.latest_sensor_sample()
        self.assertEqual(sample['channel_count'], 8)
        self.assertEqual(sample['sequence'], 1)
        self.assertEqual(sample['channel_4567_statistics']['hardware_raw_mean'], 250)
        self.assertIn('111.80', MainWindow._raw_statistics_text(sample))
        logged = ForcePressControlWidget._sensor_log_values(sample)
        self.assertEqual(logged['ch4567_mcu_hardware_raw_stddev'], 111.80)
        # Hardware/Norm调试数组刻意与测试统计值不同，确认界面使用MCU值。
        self.assertNotEqual(sample['raw'][4], 100)
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SerialDataRecorder(tmp)
            recorder.start()
            recorder.record(frames[0])
            path, count = recorder.stop()
            self.assertEqual(count, 1)
            self.assertIn('hardware_raw_stddev', path.read_text(encoding='utf-8-sig'))
        buffer.append_frame(AlgoFrameParser().feed(signal_packet())[0])
        self.assertIsNone(buffer.latest_sensor_sample()['channel_4567_statistics'])
        self.assertIn('--', MainWindow._raw_statistics_text(buffer.latest_sensor_sample()))

    def test_run_zero_missing_stale_and_wrong_channel_count(self):
        buffer = AlgoDataBuffer(5)
        frames = AlgoFrameParser().feed(self.firmware_frame((100,)*4) + signal_packet())
        for frame in frames:
            buffer.append_frame(frame)
        sample = buffer.latest_sensor_sample()
        self.assertIn('0.00', MainWindow._raw_statistics_text(sample))
        self.assertEqual(ForcePressControlWidget._sensor_log_values(sample)['ch4567_mcu_hardware_raw_stddev'], 0)
        self.assertFalse(sample['has_raw'])
        sample['age_seconds'] = 1
        self.assertIn('--', MainWindow._raw_statistics_text(sample))
        self.assertEqual(ForcePressControlWidget._sensor_log_values(sample)['ch4567_mcu_hardware_raw_stddev'], '')
        with patch('data_buffer.time.perf_counter', side_effect=[1, 2]):
            for frame in frames:
                buffer.append_frame(frame)
        self.assertIsNone(buffer.latest_sensor_sample()['channel_4567_statistics'])
        buffer.append_frame(frames[0])
        buffer.append_frame(AlgoFrameParser().feed(packet(1, struct.pack('<5H', *([0]*5)), channels=5))[0])
        self.assertIsNone(buffer.latest_sensor_sample()['channel_4567_statistics'])
        for payload, channels in [(b'\0'*8,4), (b'\0'*12,8), (struct.pack('<III',0,0,3276751),4)]:
            self.assertEqual(AlgoFrameParser().feed(packet(0x14,payload,channels=channels)), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
