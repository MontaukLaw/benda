"""真实ARM固件回放RECOVER进入/退出、再次按压、UART与CSV状态一致性。"""
import csv
import io
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace

from test_fixed_noise import Firmware, ROOT
from elftools.elf.elffile import ELFFile
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_PC, UC_ARM_REG_LR

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from frame_parser import AlgoFrameParser
from data_recorder import SerialDataRecorder
from qt_compat import QtWidgets
from main_window import MainWindow


class RecoverStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def states(self, fw):
        fw.call('SensorAlgorithm_GetStates', 0x20120000)
        states = bytes(fw.uc.mem_read(0x20120000, fw.channels))
        snapshot = bytes(fw.uc.mem_read(fw.symbols['latest_snapshot'] + fw.stride*fw.channels, fw.channels))
        self.assertEqual(states, snapshot)
        # 直接读取状态机实例，防止再次出现仅替换上报状态的实现。
        if not hasattr(fw, 'machine_stride'):
            path = ROOT / f'build/{fw.variant}/UART/APP_CY8CPROTO-041TP/Debug/mtb-example-psoc4-msclp-finger-{fw.variant}.elf'
            elf = ELFFile(io.BytesIO(path.read_bytes()))
            for cu in elf.get_dwarf_info().iter_CUs():
                for die in cu.iter_DIEs():
                    name = die.attributes.get('DW_AT_name')
                    if die.tag == 'DW_TAG_typedef' and name and name.value == b'channel_state_machine_t':
                        record = die.get_DIE_from_attribute('DW_AT_type')
                        fw.machine_stride = record.attributes['DW_AT_byte_size'].value
                        fw.machine_offsets = {m.attributes['DW_AT_name'].value.decode(): m.attributes['DW_AT_data_member_location'].value
                                              for m in record.iter_children() if m.tag == 'DW_TAG_member'}
        actual = tuple(fw.uc.mem_read(fw.symbols['channel_state_machines'] + fw.machine_stride*i + fw.machine_offsets['current_state'], 1)[0]
                       for i in range(fw.channels))
        self.assertEqual(actual, tuple(states))
        return tuple(states)

    def recover_firmware(self, variant, stop_state=7):
        fw = Firmware(variant)
        fw.variant = variant
        for _ in range(300):
            fw.step([1000]*fw.channels)
        for _ in range(fw.hz+100):
            fw.step([1100]*fw.channels)
        observed = []
        for _ in range(fw.hz*4):
            rows = fw.step([1008]*fw.channels)
            states = self.states(fw)
            if not observed or observed[-1] != states[0]:
                observed.append(states[0])
            if states == (stop_state,)*fw.channels:
                self.assertIn(4, observed)
                if stop_state == 7:
                    self.assertEqual(observed[-2:], [8, 7])
                self.assertTrue(all(row['signal'] == 0 for row in rows))
                return fw
        self.fail(f'{variant} never entered RECOVER: {observed}')

    def test_first_press_stays_idle_while_baseline_is_frozen(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            with self.subTest(variant=variant):
                fw = Firmware(variant)
                fw.variant = variant
                for _ in range(300):
                    fw.step([1000]*fw.channels)
                frozen = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels))
                # 所有通道在前两帧真实保持IDLE，但基线不能吸收13 counts轻触。
                for expected in (0, 0, 1):
                    rows = fw.step([1013]*fw.channels)
                    self.assertEqual(self.states(fw), (expected,)*fw.channels)
                    self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels)), frozen)
                    if expected == 0:
                        self.assertTrue(all(row['signal'] == 0 for row in rows))
                    else:
                        self.assertTrue(all(row['signal'] > 0 for row in rows))
                # 后续保持仍应正常锁定，不能因为省去WAIT_STABLE丢失活动计时。
                for _ in range(2*fw.hz):
                    fw.step([1013]*fw.channels)
                    self.assertNotIn(self.states(fw)[0], (0, 7, 8))
                self.assertEqual(self.states(fw), (6,)*fw.channels)

    def test_unconfirmed_disturbance_resumes_idle_follow_without_recovery(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            with self.subTest(variant=variant):
                fw = Firmware(variant)
                fw.variant = variant
                for _ in range(300):
                    fw.step([1000]*fw.channels)
                # 连续两帧越界后撤销，不能拼接成有效按压或启动按压后恢复。
                for _ in range(3):
                    frozen = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels))
                    for _ in range(2):
                        fw.step([1013]*fw.channels)
                        self.assertEqual(self.states(fw), (0,)*fw.channels)
                        self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels)), frozen)
                    before = struct.unpack('<I', frozen[:4])[0]
                    rows = fw.step([1008]*fw.channels)
                    after = struct.unpack('<I', fw.uc.mem_read(fw.symbols['baseline_q16'], 4))[0]
                    self.assertEqual(after, before + (((1008<<16)-before)*327)//65536)
                    self.assertEqual(self.states(fw), (0,)*fw.channels)
                    self.assertTrue(all(row['signal'] == 0 for row in rows))
                # 扰动后继续运行超过80帧，只允许普通IDLE跟随，不能延迟进入RECOVER。
                for _ in range(2*fw.hz):
                    fw.step([1008]*fw.channels)
                    self.assertEqual(self.states(fw), (0,)*fw.channels)

    def test_idle_freeze_clears_on_recalibration(self):
        fw = Firmware('INDEX_FINGER')
        fw.variant = 'INDEX_FINGER'
        for _ in range(300):
            fw.step([1000]*fw.channels)
        fw.step([1013]*fw.channels)
        self.assertEqual(self.states(fw), (0,)*fw.channels)
        fw.call('SensorAlgorithm_RequestRecalibration')
        for _ in range(2*fw.hz):
            fw.step([2000]*fw.channels)
        before = struct.unpack('<I', fw.uc.mem_read(fw.symbols['baseline_q16'], 4))[0]
        fw.step([2002]*fw.channels)
        after = struct.unpack('<I', fw.uc.mem_read(fw.symbols['baseline_q16'], 4))[0]
        self.assertEqual(after, before + (((2002<<16)-before)*327)//65536)
        self.assertEqual(self.states(fw), (0,)*fw.channels)

    def test_wait_requires_80_consecutive_frames_and_updates_on_entry(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            fw = self.recover_firmware(variant, stop_state=8)
            frozen = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels))
            for _ in range(30):
                fw.step([1008]*fw.channels)
            fw.step([1004]*fw.channels)
            fw.step([1008]*fw.channels)
            for _ in range(79):
                fw.step([1008]*fw.channels)
                self.assertEqual(self.states(fw), (8,)*fw.channels)
                self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels)), frozen)
            fw.step([1008]*fw.channels)
            self.assertEqual(self.states(fw), (7,)*fw.channels)
            self.assertNotEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels)), frozen)

    def test_unstable_wait_has_bounded_fallback_to_recover(self):
        fw = self.recover_firmware('INDEX_FINGER', stop_state=8)
        frozen = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels))
        # 每帧4 counts跳变会持续打断80帧稳定确认，但320帧上限后必须进入RECOVER。
        for frame in range(321):
            fw.step(([1004] if frame % 2 else [1008])*fw.channels)
            state = self.states(fw)[0]
            self.assertNotIn(state, (1, 2, 3, 6))
            if state == 7:
                break
            self.assertEqual(state, 8)
            self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels)), frozen)
        else:
            self.fail('unstable WAIT_STABLE exceeded its bounded fallback')
        self.assertGreaterEqual(frame, 318)

    def test_positive_residual_above_activation_threshold_can_recover(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            with self.subTest(variant=variant):
                fw = Firmware(variant)
                fw.variant = variant
                for _ in range(300):
                    fw.step([1000]*fw.channels)
                # 建立足够大的已确认按压，再缓慢释放到仍高于12 counts的稳定残余。
                for step in range(fw.hz//5):
                    value = 1000 + round(1000*(step+1)/(fw.hz//5))
                    fw.step([value]*fw.channels)
                for _ in range(2*fw.hz):
                    fw.step([2000]*fw.channels)
                for step in range(2*fw.hz):
                    value = 2000 - round(930*(step+1)/(2*fw.hz))
                    fw.step([value]*fw.channels)
                seen_wait = False
                seen_recover = False
                for i in range(5*fw.hz):
                    # 两个counts范围内的量化波动不能形成新的有效按压。
                    value = 1070 + (i % 3)
                    rows = fw.step([value]*fw.channels)
                    state = self.states(fw)[0]
                    seen_wait |= state == 8
                    seen_recover |= state == 7
                    if seen_wait:
                        self.assertNotIn(state, (1, 2, 3, 6))
                    if state == 0:
                        break
                self.assertTrue(seen_wait)
                self.assertTrue(seen_recover)
                self.assertEqual(self.states(fw), (0,)*fw.channels)
                self.assertTrue(all(row['signal'] == 0 for row in rows))

    def test_recover_tracks_slow_upward_drift_without_false_repress(self):
        fw = Firmware('INDEX_FINGER')
        fw.variant = 'INDEX_FINGER'
        for _ in range(300):
            fw.step([1000]*fw.channels)
        for _ in range(fw.hz+100):
            fw.step([2000]*fw.channels)
        for step in range(2*fw.hz):
            value = 2000 - round(930*(step+1)/(2*fw.hz))
            fw.step([value]*fw.channels)
        for _ in range(2*fw.hz):
            fw.step([1070]*fw.channels)
            if self.states(fw) == (7,)*fw.channels:
                break
        else:
            self.fail('large residual never entered RECOVER')
        # 大残差RECOVER期间以20 counts/s缓慢上漂，历史低点必须随Baseline推进，不能冻结或误判再次按压。
        for frame in range(fw.hz):
            value = 1070 + round(20*(frame+1)/fw.hz)
            rows = fw.step([value]*fw.channels)
            self.assertNotIn(self.states(fw)[0], (1, 2, 3, 6))
        # 上漂停止后必须收敛回IDLE，不能因超过旧局部低点12 counts永久卡在RECOVER。
        for _ in range(4*fw.hz):
            rows = fw.step([1090]*fw.channels)
            if (self.states(fw) == (0,)*fw.channels) and (abs(rows[0]['baseline']-1090) <= 1):
                break
        else:
            self.fail('slow upward drift left RECOVER stuck')
        self.assertLessEqual(abs(rows[0]['baseline']-1090), 1)

    def test_idle_speed_and_recalibration_clear_recovery(self):
        for phase in (7, 8):
            fw = self.recover_firmware('INDEX_FINGER', stop_state=phase)
            fw.call('SensorAlgorithm_RequestRecalibration')
            for _ in range(2*fw.hz + 2):
                fw.step([2000]*fw.channels)
            self.assertEqual(self.states(fw), (0,)*fw.channels)
            before = struct.unpack('<I', fw.uc.mem_read(fw.symbols['baseline_q16'], 4))[0]
            fw.step([2002]*fw.channels)
            after = struct.unpack('<I', fw.uc.mem_read(fw.symbols['baseline_q16'], 4))[0]
            self.assertEqual(self.states(fw), (0,)*fw.channels)
            self.assertEqual(after, before + (((2002<<16)-before)*327)//65536)

    def test_zero_rebase_remains_separate_and_finishes_in_idle(self):
        fw = Firmware('INDEX_FINGER')
        fw.variant = 'INDEX_FINGER'
        for _ in range(300):
            fw.step([1000]*8)
        for _ in range(2*fw.hz):
            fw.step([1500]*8)
        reached_zero = False
        for _ in range(35*fw.hz):
            before = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 32))
            fw.step([1050]*8)
            state = self.states(fw)[0]
            if state == 4:
                self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 32)), before)
            if state == 5:
                reached_zero = True
            if reached_zero:
                self.assertNotIn(state, (7, 8))
                if state == 0:
                    break
        else:
            self.fail('ZERO_REBASE did not finish in IDLE')
        self.assertTrue(reached_zero)

    def test_enter_recover_follow_exit_and_repress_all_channels(self):
        for variant in ('INDEX_FINGER', 'THUMB', 'PINKY'):
            with self.subTest(variant=variant):
                fw = self.recover_firmware(variant)
                before = struct.unpack('<I', fw.uc.mem_read(fw.symbols['baseline_q16'], 4))[0]
                rows = fw.step([1008]*fw.channels)
                after = struct.unpack('<I', fw.uc.mem_read(fw.symbols['baseline_q16'], 4))[0]
                self.assertEqual(after, before + (((1008<<16)-before)*1311)//65536)
                self.assertEqual(self.states(fw), (7,)*fw.channels)
                for _ in range(fw.hz*3):
                    rows = fw.step([1008]*fw.channels)
                    if self.states(fw) == (0,)*fw.channels:
                        break
                else:
                    self.fail('RECOVER never completed')
                self.assertLessEqual(abs(rows[0]['baseline']-1008), 1)
                fw = self.recover_firmware(variant)
                frozen = bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels))
                for frame in range(10):
                    rows = fw.step([1100]*fw.channels)
                    self.assertEqual(bytes(fw.uc.mem_read(fw.symbols['baseline_q16'], 4*fw.channels)), frozen)
                    # RECOVER内冻结两帧候选，第三帧直接RISING，不借用WAIT_STABLE表示按压确认。
                    expected = 7 if frame < 2 else 1
                    self.assertEqual(self.states(fw), (expected,)*fw.channels)
                self.assertEqual(self.states(fw), (1,)*fw.channels)
                self.assertTrue(all(row['signal'] > 0 for row in rows))

    def test_wire_display_and_csv(self):
        for phase in (7, 8):
            fw = self.recover_firmware('INDEX_FINGER', stop_state=phase)
            wire = []
            def capture(uc, address, size, user):
                wire.append(bytes(uc.mem_read(uc.reg_read(UC_ARM_REG_R0), uc.reg_read(UC_ARM_REG_R1))))
                uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
            address = fw.symbols['send_output_frame'] & ~1
            fw.uc.hook_add(UC_HOOK_CODE, capture, begin=address, end=address)
            fw.call('send_debug_frame', 0x20101000)
            frame = AlgoFrameParser().feed(wire[0])[0]
            self.assertEqual(frame['state'], (phase,)*8)
            label = QtWidgets.QLabel()
            window = SimpleNamespace(state_value_labels=[label])
            window._state_display = lambda state: MainWindow._state_display(window, state)
            window._apply_state_label_style = lambda target, state: MainWindow._apply_state_label_style(window, target, state)
            MainWindow._set_state_label(window, 0, 7)
            self.assertEqual(label.text(), 'RECOVER')
            self.assertIn('0.02', label.toolTip())
            MainWindow._set_state_label(window, 0, 0)
            self.assertEqual(label.text(), 'IDLE')
            self.assertEqual(label.toolTip(), '')
            MainWindow._set_state_label(window, 0, 8)
            self.assertEqual(label.text(), 'WAIT STABLE')
            self.assertIn('80', label.toolTip())
            with tempfile.TemporaryDirectory() as directory:
                recorder = SerialDataRecorder(directory)
                recorder.record(frame)
                result = recorder.write_recent_snapshot(recorder.snapshot_recent(1))
                with Path(result['output_path']).open(encoding='utf-8-sig', newline='') as handle:
                    row = next(csv.DictReader(handle))
                self.assertEqual([row[f'state_{i}'] for i in range(8)], [str(phase)]*8)


if __name__ == '__main__':
    unittest.main(verbosity=2)
