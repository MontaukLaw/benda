"""执行真实ARM固件验证Flash边界、48节点协议和保存恢复；不连接硬件。"""
import io
import struct
import sys
import unittest

from test_fixed_noise import Firmware, ROOT
from elftools.elf.elffile import ELFFile
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_LR, UC_ARM_REG_PC

sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
from force_signal_pressure_calibration import FORCE_CALIBRATION_TARGETS_N, select_firmware_calibration_nodes
from package_iap import parse_intel_hex, build_image, APP_START, APP_END
from check_flash_layout import check_layout

VARIANTS = ('INDEX_FINGER', 'THUMB', 'PINKY')


def checksum(data):
    result = 2166136261
    for value in data:
        result = ((result ^ value) * 16777619) & 0xFFFFFFFF
    return result


class CalibrationFirmware(Firmware):
    def __init__(self, variant):
        super().__init__(variant)
        self.writes, self.acks, self.frames = [], [], []
        self.fail_address = self.corrupt_address = None
        self.uc.mem_write(0x1F500, b'\xff' * 0x800)
        self.protected = bytes(range(256)) * 3
        self.uc.mem_write(0x1FD00, self.protected)
        for name, handler in (
            ('Cy_Flash_WriteRow', self.flash_write),
            ('SensorAlgorithm_SendCommandAck', self.ack),
            ('send_command_response_frame', self.response),
        ):
            address = self.symbols[name] & ~1
            self.uc.hook_add(UC_HOOK_CODE, handler, begin=address, end=address)
        path = ROOT / f'build/{variant}/UART/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-{variant}.elf'
        elf = ELFFile(io.BytesIO(path.read_bytes()))
        for cu in elf.get_dwarf_info().iter_CUs():
            for die in cu.iter_DIEs():
                name = die.attributes.get('DW_AT_name')
                if die.tag == 'DW_TAG_typedef' and name and name.value == b'sensor_force_calibration_t':
                    record = die.get_DIE_from_attribute('DW_AT_type')
                    self.cal_size = record.attributes['DW_AT_byte_size'].value
                    self.cal_offsets = {m.attributes['DW_AT_name'].value.decode(): m.attributes['DW_AT_data_member_location'].value
                                        for m in record.iter_children() if m.tag == 'DW_TAG_member'}
        self.call('initialize_force_calibration')

    def return_from_hook(self, value=0):
        self.uc.reg_write(UC_ARM_REG_R0, value)
        self.uc.reg_write(UC_ARM_REG_PC, self.uc.reg_read(UC_ARM_REG_LR))

    def flash_write(self, uc, address, size, data):
        destination, source = uc.reg_read(UC_ARM_REG_R0), uc.reg_read(UC_ARM_REG_R1)
        assert 0x1F500 <= destination <= 0x1FC00 and destination % 256 == 0
        self.writes.append(destination)
        if destination == self.fail_address:
            self.return_from_hook(1)
            return
        record = bytearray(uc.mem_read(source, 256))
        if destination == self.corrupt_address:
            record[0] ^= 1
        uc.mem_write(destination, bytes(record))
        self.return_from_hook()

    def ack(self, uc, address, size, data):
        self.acks.append(uc.reg_read(UC_ARM_REG_R0))
        self.return_from_hook()

    def response(self, uc, address, size, data):
        self.frames.append(bytes(uc.mem_read(uc.reg_read(UC_ARM_REG_R0), uc.reg_read(UC_ARM_REG_R1))))
        self.return_from_hook()

    def command(self, command, payload=b''):
        self.uc.mem_write(0x20110000, payload or b'\0')
        self.call('SensorAlgorithm_HandleProtocolCommand', command, 0x20110000, len(payload))

    def stage(self, region=0, component=0, targets=FORCE_CALIBRATION_TARGETS_N):
        payload = bytes((region, component, 2, len(targets), 0, 0, 0, 0))
        payload += b''.join(struct.pack('<ff', target * 1000, target) for target in targets)
        self.command(0x84, payload)

    def read_table(self, region=0, component=0):
        self.command(0x83, bytes((region, component)))
        parser = AlgoFrameParser()
        frames = []
        # 逐字节模拟串口分包，验证402字节长帧不会截断。
        for value in self.frames[-1]:
            frames.extend(parser.feed(bytes((value,))))
        assert len(frames) == 1
        return frames[0]

    def flag(self, name, region=0):
        return self.uc.mem_read(self.symbols['force_calibration'] + self.cal_size * region + self.cal_offsets[name], 1)[0]

    def install_legacy(self, region=0):
        record = bytearray(b'\xff' * 512)
        record[:11] = b'FCV8' + bytes((8, self.device, 7, 20, 2, region, 2))
        for index in range(20):
            struct.pack_into('<f', record, 16 + index * 4, (index + 1) * 1000)
            struct.pack_into('<f', record, 96 + index * 4, index + 1)
            struct.pack_into('<f', record, 176 + index * 4, float(index))
            struct.pack_into('<f', record, 256 + index * 4, float(index))
        struct.pack_into('<4f', record, 336, 0, 360, 0, 360)
        struct.pack_into('<I', record, 352, checksum(record[:352]))
        self.uc.mem_write(0x1F900 + region * 512, bytes(record))

    @property
    def device(self):
        return 1 if self.channels == 8 else self._device


class FlashTwoRangeTests(unittest.TestCase):
    def firmware(self, variant='INDEX_FINGER'):
        fw = CalibrationFirmware(variant)
        fw._device = VARIANTS.index(variant) + 1
        return fw

    def test_guard_rejects_every_protected_row_and_bad_address(self):
        for variant in VARIANTS:
            fw = self.firmware(variant)
            for address in (0, 0x3600, 0x1F400, 0x1F501, 0x1F601, 0x1FD00, 0x1FE00, 0x1FF00, 0x20000, 0xFFFFFF00):
                fw.call('AppFlash_WriteRow', address, 0x20110000)
                self.assertNotEqual(fw.uc.reg_read(UC_ARM_REG_R0), 0)
            fw.call('AppFlash_WriteRow', 0x1F600, 0)
            self.assertEqual(fw.writes, [])
            for address in range(0x1F500, 0x1FD00, 256):
                fw.call('AppFlash_WriteRow', address, 0x20110000)
                self.assertEqual(fw.uc.reg_read(UC_ARM_REG_R0), 0)
            self.assertEqual(bytes(fw.uc.mem_read(0x1FD00, 768)), fw.protected)

    def test_complete_protocol_save_reload_and_interpolation_all_variants(self):
        for variant in VARIANTS:
            with self.subTest(variant=variant):
                fw = self.firmware(variant)
                fw.stage()
                self.assertEqual(fw.acks[-1], 0)
                self.assertEqual(fw.flag('normal_valid'), 1)
                self.assertEqual(fw.flag('tangential_valid'), 0)
                before = fw.read_table()
                self.assertEqual(len(fw.frames[-1]), 402)
                self.assertEqual(before['point_count'], 48)
                self.assertTrue(before['dirty'])
                fw.command(0x85)
                self.assertEqual(fw.acks[-1], 0)
                self.assertEqual(fw.writes, [0x1F600, 0x1F700, 0x1F800])
                record = bytes(fw.uc.mem_read(0x1F600, 768))
                self.assertEqual(record[:5], b'FCV9\x09')
                self.assertEqual(struct.unpack_from('<I', record, 576)[0], checksum(record[:576]))
                fw.call('initialize_force_calibration')
                after = fw.read_table()
                self.assertEqual(after['raw_points'], before['raw_points'])
                self.assertEqual(after['calibrated_points'], before['calibrated_points'])
                self.assertTrue(after['valid'])
                self.assertFalse(after['dirty'])
                self.assertFalse(fw.read_table(component=1)['valid'])
                # 食指V17仅输出独立低力表；旧K表换算由仍使用它的五通道变体验证。
                if variant != 'INDEX_FINGER':
                    for force in (0, .05, .1, .35, .9, 1.95, 2, 2.5, 29.5, 30):
                        bits = struct.unpack('<I', struct.pack('<f', force * 1000))[0]
                        fw.call('apply_normal_force_calibration', bits, fw.symbols['force_calibration'])
                        actual = struct.unpack('<f', struct.pack('<I', fw.uc.reg_read(UC_ARM_REG_R0)))[0]
                        self.assertAlmostEqual(actual, force, places=5)
                        bits = struct.unpack('<I', struct.pack('<f', force))[0]
                        fw.call('apply_force_output_soft_anchor', bits)
                        self.assertEqual(fw.uc.reg_read(UC_ARM_REG_R0), bits)
                fw.command(0x85)
                self.assertEqual(len(fw.writes), 3)
                self.assertEqual(bytes(fw.uc.mem_read(0x1FD00, 768)), fw.protected)

    def test_failed_or_corrupt_write_retains_dirty_and_can_retry(self):
        for failure_kind in ('fail_address', 'corrupt_address'):
            fw = self.firmware()
            fw.stage()
            setattr(fw, failure_kind, 0x1F700)
            fw.command(0x85)
            self.assertEqual(fw.acks[-1], 4)
            self.assertTrue(fw.flag('dirty'))
            setattr(fw, failure_kind, None)
            fw.command(0x85)
            self.assertEqual(fw.acks[-1], 0)
            self.assertFalse(fw.flag('dirty'))
            fw.uc.mem_write(0x1F620, b'\xff' * 4)
            fw.call('initialize_force_calibration')
            self.assertFalse(fw.flag('normal_valid'))

    def test_old_table_readonly_migration_preserves_other_components(self):
        fw = self.firmware()
        fw.install_legacy(0)
        fw.install_legacy(1)
        fw.call('initialize_force_calibration')
        self.assertEqual(fw.writes, [])
        self.assertFalse(fw.flag('normal_valid'))
        before = fw.read_table(component=1)
        self.assertTrue(before['valid'])
        # region 1新地址重叠旧region 0；保存前必须先迁移region 0。
        fw.stage(region=1, targets=(1, 2))
        fw.command(0x85)
        self.assertEqual(fw.acks[-1], 0)
        self.assertEqual(fw.writes, list(range(0x1F600, 0x1FC00, 256)))
        fw.call('initialize_force_calibration')
        self.assertEqual(fw.read_table(component=1), before)
        self.assertFalse(fw.flag('normal_valid'))
        fw.stage()
        fw.command(0x85)
        fw.call('initialize_force_calibration')
        self.assertTrue(fw.flag('normal_valid'))
        self.assertEqual(fw.read_table(component=1), before)

    def test_reject_old_primary_oversized_tangent_and_disabled_affine(self):
        fw = self.firmware()
        fw.stage(targets=tuple(range(1, 21)))
        self.assertEqual(fw.acks[-1], 1)
        fw.stage(component=1)
        self.assertEqual(fw.acks[-1], 1)
        for command in (0x92, 0x93, 0x94):
            fw.command(command)
            self.assertEqual(fw.acks[-1], 2)
        self.assertEqual(fw.writes, [])

    def test_node_grid_and_artifact_bounds(self):
        self.assertEqual(check_layout(), (0x1F500, 0x1FD00))
        targets = FORCE_CALIBRATION_TARGETS_N
        self.assertEqual(len(targets), 48)
        self.assertEqual(targets.count(2.0), 1)
        nodes = [dict(nominal_target_force_n=t, signal_sum_median=t*1000, measured_pressure_median_n=t) for t in targets]
        self.assertEqual(len(select_firmware_calibration_nodes(nodes)['nodes']), 48)
        for invalid in (nodes[:-1], nodes[:20], nodes[::-1]):
            with self.assertRaises(ValueError):
                select_firmware_calibration_nodes(invalid)
        for variant in VARIANTS:
            folder = ROOT / f'build/{variant}/UART/APP_CY8CPROTO-041TP/Debug'
            path = folder / f'mtb-example-psoc4-msclp-finger-{variant}.hex'
            memory = parse_intel_hex(path)
            addresses = [address for address in memory if address < 0x20000]
            self.assertEqual((min(addresses), max(addresses)), (APP_START, APP_END - 1))
            image = build_image(memory)
            self.assertEqual(len(image), 0x1BF00)
            self.assertEqual(path.with_suffix('.iap.bin').read_bytes(), image)
            cyacd = path.with_name(path.stem + '_crc.cyacd2')
            records = [bytes.fromhex(line[1:]) for line in cyacd.read_text(encoding='ascii').splitlines() if line.startswith(':')]
            row_addresses = [int.from_bytes(record[:4], 'little') for record in records]
            self.assertEqual(row_addresses, list(range(APP_START, APP_END, 256)))
            self.assertTrue(all(len(record) == 260 for record in records))
            for address in (0, 0x1F500, 0x1F600, 0x1FD00, 0x1FE00, 0x1FF00):
                with self.assertRaises(ValueError):
                    build_image({**memory, address: 0})


if __name__ == '__main__':
    unittest.main(verbosity=2)
