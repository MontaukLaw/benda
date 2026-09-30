"""在 Unicorn 中运行本次构建的真实固件，验证20路算法、参数映射及DFU保留RAM。

模拟器不能替代电极扫描、串口或实际Flash升级测试。
"""
import io
import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tmp/state_test_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

ELF = ROOT / 'build/INDEX_FINGER/UART/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-INDEX_FINGER.elf'


class Firmware:
    def __init__(self):
        elf = ELFFile(io.BytesIO(ELF.read_bytes()))
        self.symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for address, size in ((0, 0x200000), (0x0ffff000, 0x1000), (0x20000000, 0x200000),
                              (0x40000000, 0x1000000), (0xe0000000, 0x100000)):
            self.uc.mem_map(address, size)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                self.uc.mem_write(segment['p_vaddr'], segment.data())

    def call(self, name, *args, limit=5000000):
        for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            self.uc.reg_write(register, value)
        self.uc.reg_write(UC_ARM_REG_SP, 0x201ff000)
        self.uc.reg_write(UC_ARM_REG_LR, 0x1f0001)
        self.uc.emu_start(self.symbols[name] | 1, 0x1f0000, count=limit)
        if self.uc.reg_read(UC_ARM_REG_PC) != 0x1f0000:
            raise AssertionError(f'{name} did not return; PC={self.uc.reg_read(UC_ARM_REG_PC):x}')
        return self.uc.reg_read(UC_ARM_REG_R0)

    def stub_success(self, name):
        # 只替换依赖真实MSCLP扫描完成的硬件操作，参数和扫描槽写入仍运行原函数。
        self.uc.mem_write(self.symbols[name] & ~1, b'\x00\x20\x70\x47')

    def sample(self, values):
        self.uc.mem_write(0x20100000, struct.pack('<20H', *values))
        self.call('SensorAlgorithm_Process', 0x20100000, 0x20100100)
        self.call('SensorAlgorithm_GetStates', 0x20100400)
        states = tuple(self.uc.mem_read(0x20100400, 20))
        outputs = [struct.unpack('<HHH2xiiiiH2x', self.uc.mem_read(0x20100100 + i*28, 28)) for i in range(20)]
        return states, outputs


class Matrix20FirmwareTests(unittest.TestCase):
    def test_recovery_tolerates_quantization_noise(self):
        fw = Firmware()
        hz = 95
        fw.call('ChannelState_SetSampleRateHz', hz)
        fw.call('initialize_baseline_runtime')
        fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H',300))
        for _ in range(3*hz): fw.sample([8000]*20)
        for _ in range(hz//3): fw.sample([7800]*20)
        for i in range(7*hz):
            states, outputs = fw.sample([8010+(-3 if i%8<4 else 3)]*20)
            self.assertTrue(all(state in (0,4,5,7,8) for state in states))
        self.assertEqual(states,(0,)*20)
        self.assertTrue(all(o[2]==0 for o in outputs))

    def test_slow_release_and_recovery_rebound(self):
        for hz in (74, 95, 200):
            with self.subTest(hz=hz):
                fw = Firmware()
                fw.call('ChannelState_SetSampleRateHz', hz)
                fw.call('initialize_baseline_runtime')
                fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H',300))
                for _ in range(3*hz): fw.sample([8000]*20)
                # 大部分卸载过程斜率低于快速松手阈值，仍应在稳定后释放残余。
                for _ in range(6*hz): states, outputs = fw.sample([8500]*20)
                self.assertEqual(states,(6,)*20)
                for i in range(10*hz): fw.sample([8500-round(400*(i+1)/(10*hz))]*20)
                for _ in range(5*hz): states, outputs = fw.sample([8100]*20)
                self.assertEqual(states,(0,)*20)
                self.assertTrue(all(o[2]==0 for o in outputs))
                # 负向形变恢复时，回到原基线以下不能被局部低点差值激活。
                fw.call('SensorAlgorithm_RequestRecalibration')
                for _ in range(3*hz): fw.sample([8000]*20)
                for _ in range(3*hz): fw.sample([8500]*20)
                for _ in range(hz//5): fw.sample([7800]*20)
                for i in range(3*hz):
                    raw = 7800+round(280*(i+1)/(3*hz))
                    states, outputs = fw.sample([raw]*20)
                    self.assertTrue(all(state in (0,4,5,7,8) for state in states))
                    self.assertTrue(all(o[2]==0 for o in outputs))
                    self.assertTrue(all(o[1]==8000 for o in outputs))
                for _ in range(4*hz): states, outputs = fw.sample([8080]*20)
                self.assertEqual(states,(0,)*20)
                # 恢复过程中真实再次快速按压仍须识别，不能把所有再触摸屏蔽。
                for _ in range(3*hz): fw.sample([8500]*20)
                for _ in range(hz//5): fw.sample([7800]*20)
                for _ in range(hz//2): states, outputs = fw.sample([8600]*20)
                self.assertTrue(all(state in (1,2,3,6) for state in states))
                self.assertTrue(all(o[2]>100 for o in outputs))

    def test_matrix_release_thresholds_preserve_hold_and_reject_negative_rebound(self):
        for hz in (87, 115, 200):
            with self.subTest(hz=hz):
                fw = Firmware()
                fw.call('ChannelState_SetSampleRateHz', hz)
                fw.call('initialize_baseline_runtime')
                fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H',300))
                for _ in range(3*hz): fw.sample([8000]*20)
                # 负向形变约一秒，回弹只高出原零点20 counts，不应被当成按压。
                for _ in range(hz): fw.sample([7800]*20)
                for _ in range(3*hz): states, outputs = fw.sample([8020]*20)
                self.assertEqual(states, (0,)*20)
                self.assertTrue(all(o[2]==0 for o in outputs))
                # 重新校准后真实轻压60 counts，持续十秒不能因超时或小幅蠕变清零。
                fw.call('SensorAlgorithm_RequestRecalibration')
                for _ in range(3*hz): fw.sample([8000]*20)
                for _ in range(5*hz): states, outputs = fw.sample([8060]*20)
                self.assertEqual(states,(6,)*20)
                for _ in range(5*hz): states, outputs = fw.sample([8050]*20)
                self.assertTrue(all(s in (1,2,3,6) for s in states))
                self.assertTrue(all(o[2]>32 for o in outputs))
                # 释放后只剩20 counts偏移，应完成正常恢复并把Signal归零。
                for _ in range(5*hz): states, outputs = fw.sample([8020]*20)
                self.assertEqual(states,(0,)*20)
                self.assertTrue(all(o[2]==0 for o in outputs))
                # 200 counts按压部分卸载至150仍为接触，回落至80才满足新松手比例。
                for _ in range(5*hz): fw.sample([8220]*20)
                for _ in range(3*hz): states, outputs = fw.sample([8170]*20)
                self.assertTrue(all(s in (1,2,3,6) for s in states))
                for _ in range(hz): states, outputs = fw.sample([8100]*20)
                self.assertTrue(all(s in (4,5,7,8,0) for s in states))
                self.assertTrue(all(o[2]==0 for o in outputs))
                # 下降证据过期后不能触发未来的缓慢部分卸载，也不能跨重新校准复用。
                fw.call('SensorAlgorithm_RequestRecalibration')
                for _ in range(3*hz): fw.sample([8000]*20)
                for _ in range(5*hz): fw.sample([8200]*20)
                for _ in range(2*hz): fw.sample([8180]*20)
                for i in range(3*hz):
                    states, outputs = fw.sample([8180-round(60*(i+1)/(3*hz))]*20)
                    self.assertTrue(all(s in (1,2,3,6) for s in states))

    def test_twenty_independent_baselines_press_hold_release_and_recalibration(self):
        for hz in (200, 120):
            with self.subTest(hz=hz):
                fw = Firmware()
                fw.call('ChannelState_SetSampleRateHz', hz)
                fw.call('initialize_baseline_runtime')
                # 模拟发送函数已完成开机静默，直接执行正式算法入口。
                fw.uc.mem_write(fw.symbols['startup_quiet_frame_count'], struct.pack('<H', 300))
                base = [1000+i*100 for i in range(20)]
                for _ in range(3*hz):
                    states, outputs = fw.sample(base)
                self.assertEqual(states, (0,)*20)
                self.assertEqual([o[1] for o in outputs], base)
                # 偶数通道加载，奇数通道保持空闲，覆盖CH18和CH19。
                for step in range(1, hz//10 + 1):
                    values = [v + (round(200*step/(hz//10)) if i % 2 == 0 else 0) for i, v in enumerate(base)]
                    fw.sample(values)
                for _ in range(6*hz):
                    states, outputs = fw.sample(values)
                self.assertEqual(states, tuple(6 if i % 2 == 0 else 0 for i in range(20)))
                self.assertEqual([o[1] for o in outputs], base)
                self.assertTrue(all(o[2] > 150 if i % 2 == 0 else o[2] == 0 for i, o in enumerate(outputs)))
                for _ in range(2*hz):
                    states, outputs = fw.sample(base)
                self.assertEqual(states, (0,)*20)
                self.assertTrue(all(o[2] == 0 for o in outputs))
                # 矩阵空闲跟随降低十倍：六秒仍保留偏差，长期漂移最终可以跟随。
                drift = [v-10 for v in base]
                for _ in range(6*hz):
                    states, outputs = fw.sample(drift)
                self.assertTrue(all(3 <= o[1]-v <= 8 for o,v in zip(outputs,drift)))
                for _ in range(30*hz):
                    states, outputs = fw.sample(drift)
                self.assertTrue(all(abs(o[1]-v) <= 1 for o, v in zip(outputs, drift)))
                fw.call('SensorAlgorithm_RequestRecalibration')
                for _ in range(3*hz):
                    states, outputs = fw.sample(drift)
                self.assertEqual(states, (0,)*20)

    def test_three_parameters_cover_all_twenty_scan_slots(self):
        fw = Firmware()
        # Init填充Configurator上下文；真实硬件测量步骤在此单独隔离。
        fw.stub_success('Cy_CapSense_InitializeCic2Rate')
        fw.stub_success('Cy_CapSense_InitializeCic2Shift')
        fw.stub_success('Cy_CapSense_InitializeMaxRaw')
        fw.call('Cy_CapSense_Init', fw.symbols['cy_capsense_context'])
        fw.call('CapSenseChannelConfig_ReadAll', 0x20101000)
        boot_values = []
        for i in range(20):
            boot = struct.unpack('<5H4B', fw.uc.mem_read(0x20101000+i*14, 14))
            boot_values.append((boot[0],boot[2],boot[5]))
        # Configurator是开机参数来源，用户修改配置后不再把旧值写死在测试里。
        self.assertTrue(all(values==boot_values[0] for values in boot_values))
        self.assertTrue(all(v>0 for v in boot_values[0]))
        for nsub, tx, ref in ((300, 100, 12), (400, 120, 15), (300, 100, 12)):
            self.assertEqual(fw.call('CapSenseChannelConfig_ApplyFixedAll', nsub, tx, ref), 1)
            self.assertEqual(fw.call('CapSenseChannelConfig_ReadAll', 0x20101000), 1)
            for i in range(20):
                configured_nsub, hardware_nsub, configured_tx, hardware_tx, _, configured_ref, hardware_ref, widget, sensor = struct.unpack(
                    '<5H4B', fw.uc.mem_read(0x20101000 + i*14, 14))
                self.assertEqual((configured_nsub, hardware_nsub, configured_tx, hardware_tx, configured_ref, hardware_ref),
                                 (nsub, nsub, tx, tx, ref, ref))
                self.assertEqual((widget, sensor), (0, i))

    def test_dfu_entry_always_selects_app_zero(self):
        fw = Firmware()
        # 真实SDK设置App0后写AIRCR复位；模拟器中截获复位写入而不运行死循环。
        from unicorn import UC_HOOK_MEM_WRITE
        resets = []
        def on_write(uc, access, address, size, value, user):
            if address == 0xe000ed0c and value & 4:
                resets.append(value)
                uc.emu_stop()
        fw.uc.hook_add(UC_HOOK_MEM_WRITE, on_write)
        for _ in range(2):
            fw.uc.mem_write(0x20000000, b'\x01')
            fw.uc.reg_write(UC_ARM_REG_R0, 1)
            fw.uc.reg_write(UC_ARM_REG_SP, 0x201ff000)
            fw.uc.emu_start(fw.symbols['IapRequest_EnterBootloader'] | 1, 0x1f0000, count=10000)
            self.assertEqual(bytes(fw.uc.mem_read(0x20000000, 1)), b'\x00')
            self.assertEqual(struct.unpack('<II', fw.uc.mem_read(0x20000004, 8)), (0x49504150, 1))
        self.assertEqual(len(resets), 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
