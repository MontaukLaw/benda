"""在实际编译的三变体ARM固件中验证固定噪声、校准及基线冻结。"""
import io
import struct
import unittest
from test_channel_state_pressure import ROOT
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC


class Firmware:
    def __init__(self, variant):
        self.channels = 8 if variant == 'INDEX_FINGER' else 5
        self.hz = 160 if self.channels == 8 else 250
        path = ROOT / f'build/{variant}/UART/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-{variant}.elf'
        elf = ELFFile(io.BytesIO(path.read_bytes()))
        self.symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(0, 0x100000)
        self.uc.mem_map(0x20000000, 0x200000)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                address = segment['p_vaddr']
                if address < 0x100000 or 0x20000000 <= address < 0x20200000:
                    self.uc.mem_write(address, segment.data())
        # 从正式ELF的DWARF读取结构布局，避免测试猜测ARM对齐。
        for cu in elf.get_dwarf_info().iter_CUs():
            found = False
            for die in cu.iter_DIEs():
                name = die.attributes.get('DW_AT_name')
                if die.tag == 'DW_TAG_typedef' and name and name.value == b'sensor_algorithm_output_t':
                    record = die.get_DIE_from_attribute('DW_AT_type')
                    self.stride = record.attributes['DW_AT_byte_size'].value
                    self.offsets = {m.attributes['DW_AT_name'].value.decode(): m.attributes['DW_AT_data_member_location'].value
                                    for m in record.iter_children() if m.tag == 'DW_TAG_member'}
                    found = True
                    break
            if found:
                break
        assert hasattr(self, 'stride'), 'Missing output DWARF'
        self.call('SensorAlgorithm_RestartBaselineInitialization')

    def call(self, name, *args):
        for reg, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2), args):
            self.uc.reg_write(reg, value)
        self.uc.reg_write(UC_ARM_REG_SP, 0x201FF000)
        self.uc.reg_write(UC_ARM_REG_LR, 0xF0001)
        self.uc.emu_start(self.symbols[name] | 1, 0xF0000, count=10000000)
        assert self.uc.reg_read(UC_ARM_REG_PC) == 0xF0000, name

    def step(self, raw):
        self.uc.mem_write(0x20100000, struct.pack('<' + 'H' * self.channels, *raw))
        self.call('SensorAlgorithm_Process', 0x20100000, 0x20101000)
        return self.read_outputs(0x20101000)

    def read_outputs(self, address):
        data = self.uc.mem_read(address, self.stride * self.channels)
        return [{name: struct.unpack_from('<H', data, i * self.stride + self.offsets[name])[0]
                 for name in ('raw', 'baseline', 'signal', 'noise')} for i in range(self.channels)]

    def calibrated(self):
        return self.uc.mem_read(self.symbols['calibrated'], 1)[0] != 0


class FixedNoiseTests(unittest.TestCase):
    def test_startup_recalibration_and_small_press_all_channels(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            with self.subTest(variant=variant):
                fw = Firmware(variant)
                initial = fw.read_outputs(fw.symbols['latest_snapshot'])
                self.assertEqual([x['noise'] for x in initial], [1] * fw.channels)
                # 大幅波动的校准输入也不能抬高Noise或触发噪声重试。
                for i in range(300):
                    rows = fw.step([1000 + c * 100 + (i % 2) * 100 for c in range(fw.channels)])
                    self.assertEqual([x['noise'] for x in rows], [1] * fw.channels)
                    self.assertEqual(fw.calibrated(), i == 299)
                targets = [2000 + c * 100 for c in range(fw.channels)]
                fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H', 300))
                fw.call('SensorAlgorithm_RequestRecalibration')
                self.assertFalse(fw.calibrated())
                for i in range(2 * fw.hz):
                    rows = fw.step(targets)
                    self.assertEqual([x['noise'] for x in rows], [1] * fw.channels)
                    self.assertEqual(fw.calibrated(), i == 2 * fw.hz - 1)
                self.assertEqual([x['baseline'] for x in rows], targets)
                # 用高于门槛1 count的输入验证防抖期间就冻结，避免基线抢先吞掉小压力。
                frozen = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], fw.channels * 4))
                for _ in range(fw.hz * 2):
                    rows = fw.step([v + 13 for v in targets])
                    self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], fw.channels * 4)), frozen)
                self.assertTrue(all(x['signal'] > 0 and x['noise'] == 1 for x in rows))
                # 单通道重校准路径也必须保持固定噪声。
                fw.uc.mem_write(fw.symbols['channel_recalibrating'], bytes([1] * fw.channels))
                fw.uc.mem_write(fw.symbols['channel_calibration_counts'], bytes(4 * fw.channels))
                for _ in range(2 * fw.hz):
                    rows = fw.step(targets)
                    self.assertTrue(all(x['noise'] == 1 for x in rows))
                self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['channel_recalibrating'], fw.channels)), bytes(fw.channels))


if __name__ == '__main__':
    unittest.main(verbosity=2)
