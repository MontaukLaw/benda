"""执行固件真实组帧函数，覆盖V20精简布局、旧版兼容和实时数据消费者。"""
import io
import re
import statistics
import struct
import subprocess
import tempfile
import unittest

from test_hardware_raw_statistics import ROOT, extract_function
from test_channel_state_pressure import compiler_path
from test_missing_debug_data import packet, debug_packet
from frame_parser import AlgoFrameParser
from data_buffer import AlgoDataBuffer
from data_recorder import SerialDataRecorder
from serial_reader import SerialReaderThread
from force_press_control_widget import ForcePressControlWidget
from weight_calibration_widget import _confirmation_median
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC


class RawProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (ROOT / 'sensor_algorithm.c').read_text(encoding='utf8')
        cls.models = {}
        for variant in (1, 2, 3):
            directory = ROOT / 'build/raw_protocol_v20' / str(variant)
            directory.mkdir(parents=True, exist_ok=True)
            harness = '#include "sensor_algorithm.h"\n'
            # 从固件提取尺寸定义，避免测试中的常量掩盖固件布局错误。
            for name in ('DEVICE_TYPE', 'PROTOCOL_MAGIC_SIZE', 'PROTOCOL_HEADER_SIZE', 'FRAME_TYPE_DEBUG',
                         'BYTES_PER_VALUE', 'BYTES_PER_K_VALUE', 'DEBUG_U16_VALUES_PER_SENSOR',
                         'DEBUG_K_VALUES_PER_SENSOR', 'DEBUG_PAYLOAD_SIZE', 'DEBUG_FRAME_SIZE'):
                match = re.search(r'^#define SENSOR_ALGO_' + name + r'\b(?:[^\n]*\\\n)*[^\n]*', source, re.M)
                harness += match[0] + '\n'
            harness += r'''
uint16_t latest_hardware_raw[SENSOR_ALGO_CHANNEL_COUNT];
uint8_t captured[SENSOR_ALGO_DEBUG_FRAME_SIZE];
/* 传输桩只保存字节，载荷由固件原函数生成。 */
static void send_output_frame(const uint8_t *p, uint32_t n) {
    for (unsigned i=0; i<n; i++) captured[i]=p[i];
}
/* 测试状态由独立状态接口返回，不能误用输出结构中的占位状态。 */
void SensorAlgorithm_GetStates(sensor_algorithm_state_t states[SENSOR_ALGO_CHANNEL_COUNT]) {
    for (unsigned i=0; i<SENSOR_ALGO_CHANNEL_COUNT; i++) states[i]=(sensor_algorithm_state_t)(i+1);
}
'''
            for name in ('write_u16', 'write_u32', 'write_frame_header', 'send_debug_frame'):
                harness += '\n/* 执行固件原函数。 */\n' + extract_function(source, name)
            harness += r'''
/* 使用全量程Raw及正负K值验证数组顺序、符号和精度。 */
void test_send(void) {
    sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT] = {0};
    for (unsigned i=0; i<SENSOR_ALGO_CHANNEL_COUNT; i++) {
        latest_hardware_raw[i]=65535-i;
        outputs[i].baseline=100+i;
        outputs[i].signal=200+i;
        outputs[i].fast_normalized_k_x100=-12345-(int32_t)i;
        outputs[i].slow_normalized_k_x100=23456+i;
        outputs[i].noise=333;
    }
    send_debug_frame(outputs);
}
'''
            path = directory / 'harness.c'
            path.write_text(harness, encoding='utf8')
            elf_path = directory / 'debug.elf'
            command = [compiler_path(), '-mcpu=cortex-m0plus', '-mthumb', '-Os', '-g',
                       '-Wall', '-Wextra', '-Werror', '-std=c99', '-nostartfiles', '--specs=nosys.specs',
                       f'-DDEVICE_FINGER_TYPE={variant}', '-I', str(ROOT), str(path),
                       '-Wl,-e,test_send,-Ttext=0x10000,-Tdata=0x20000000', '-o', str(elf_path)]
            result = subprocess.run(command, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
            elf = ELFFile(io.BytesIO(elf_path.read_bytes()))
            cls.models[variant] = (
                [(s['p_vaddr'], s.data()) for s in elf.iter_segments() if s['p_type'] == 'PT_LOAD'],
                {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()})

    def wire(self, variant):
        segments, symbols = self.models[variant]
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(0x10000, 0x100000)
        uc.mem_map(0x20000000, 0x200000)
        for address, data in segments:
            uc.mem_write(address, data)
        uc.reg_write(UC_ARM_REG_SP, 0x201FF000)
        uc.reg_write(UC_ARM_REG_LR, 0xF0001)
        uc.emu_start(symbols['test_send'] | 1, 0xF0000, count=100000)
        self.assertEqual(uc.reg_read(UC_ARM_REG_PC), 0xF0000)
        return bytes(uc.mem_read(symbols['captured'], 330 if variant == 1 else 90))

    def test_real_firmware_all_variants_bytewise_parser_and_consumers(self):
        for variant, n in ((1, 20), (2, 5), (3, 5)):
            wire = self.wire(variant)
            self.assertEqual(wire[:10], struct.pack('<4sBBBBH', b'DATA', 21, 0x10, variant, n, 16*n))
            parser, frames = AlgoFrameParser(), []
            for byte in wire:
                frames += parser.feed(bytes([byte]))
            self.assertEqual(len(frames), 1)
            frame = frames[0]
            for name, expected in (
                ('raw', tuple(65535-i for i in range(n))),
                ('baseline', tuple(100+i for i in range(n))),
                ('signal', tuple(200+i for i in range(n))),
                ('fast_k_percent', tuple((-12345-i)/100 for i in range(n))),
                ('slow_k_percent', tuple((23456+i)/100 for i in range(n))),
                ('state', tuple(range(1, n+1)))):
                self.assertEqual(frame[name], expected)
            buffer = AlgoDataBuffer(5)
            buffer.append_frame(frame)
            sample = buffer.latest_sensor_sample()
            self.assertEqual(sample['raw'], frame['raw'])
            self.assertEqual(sample['hardware_raw'], frame['raw'])
            log = ForcePressControlWidget._sensor_log_values(sample)
            for removed in ('normalized_raw', 'candidate_normalized_raw', 'noise', 'slow_baseline'):
                self.assertNotIn(removed, frame)
                self.assertNotIn(removed, sample)
                self.assertFalse(any(removed in key for key in log))

    def test_legacy_versions_skip_old_fields_without_misaligning_state(self):
        old = debug_packet()
        for version in range(14, 20):
            wire = old[:4] + bytes([version]) + old[5:]
            frame = AlgoFrameParser().feed(wire)[0]
            self.assertEqual(frame['raw'], tuple(range(1000, 1008)))
            self.assertEqual(frame['baseline'], (900,)*8)
            self.assertEqual(frame['state'], (6,)*8)
            self.assertNotIn('normalized_raw', frame)
            self.assertNotIn('noise', frame)

    def test_wrong_version_lengths_and_retired_frame_cannot_corrupt_next_frame(self):
        valid = self.wire(1)
        old = debug_packet()
        wrong = old[:4] + bytes([20]) + old[5:]
        wrong += valid[:4] + bytes([19]) + valid[5:]
        wrong += packet(0x15, bytes(32), version=20)
        self.assertEqual(AlgoFrameParser().feed(wrong + valid), AlgoFrameParser().feed(valid))

    def test_legacy_slow_baseline_ignored_and_new_recordings_have_no_removed_columns(self):
        legacy = AlgoFrameParser().feed(packet(0x15, bytes(32), version=19))[0]
        buffer = AlgoDataBuffer(5)
        before = buffer.snapshot()
        buffer.append_frame(legacy)
        self.assertEqual(buffer.snapshot()[-1], before[-1])
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SerialDataRecorder(tmp)
            recorder.record(legacy)
            frame = AlgoFrameParser().feed(self.wire(1))[0]
            recorder.record(dict(frame, normalized_raw=[1]*8, candidate_normalized_raw=[2]*8, noise=[1]*8))
            snapshot = recorder.snapshot_recent(1)
            self.assertEqual(snapshot['metadata']['row_count'], 1)
            result = recorder.write_recent_snapshot(snapshot)
            from pathlib import Path
            columns = Path(result['output_path']).read_text(encoding='utf-8-sig').splitlines()[0]
            for removed in ('normalized_raw', 'noise', 'slow_baseline'):
                self.assertNotIn(removed, columns)

    def test_host_noise_uses_last_64_raw_samples(self):
        reader = SerialReaderThread('FAKE', AlgoDataBuffer(5))
        observed = []
        reader.live_noise.connect(observed.append)
        for i in range(80):
            reader._update_live_noise([60000+i]*8)
        self.assertAlmostEqual(observed[-1]['noise'][0], statistics.pstdev(range(16, 80)), places=6)
        reader.reset_live_noise()
        reader._update_live_noise([42]*8)
        self.assertEqual(observed[-1]['noise'], (0.0,)*8)

    def test_weight_median_needs_only_retained_fields(self):
        window = [dict(time=i, raw=(100+i,)*4, hardware_raw=(100+i,)*4,
                       baseline=(90,)*4, signal=(10+i,)*4, fast_k_percent=(.5,)*4,
                       slow_k_percent=(-.1,)*4, state=(6,)*4) for i in range(3)]
        result = _confirmation_median(window)
        self.assertEqual(result['hardware_raw'], (101,)*4)
        self.assertEqual(result['sum'], 404)
        self.assertNotIn('noise', result)


if __name__ == '__main__':
    unittest.main(verbosity=2)
