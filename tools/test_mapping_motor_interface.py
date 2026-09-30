"""验证标定接口的步数换算、协议清零、坐标作废及同一线程扫描/停止。"""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_zdt_motor import driver, frames, SimulatedMotor, EMM_PACKET
from zdt_x42s_can_driver import ZDTConfig, ZDTTimeoutError
from zdt_motor_test_widget import ZDTMotorTestWidget
from qt_compat import QtWidgets, QtCore, QtGui
from motor_mapping_interface import create_mapping_control, ZDTMappingMotor, LEGACY_PROFILE
import force_position_mapping_widget as mapping


class MappingSimMotor(SimulatedMotor):
    def __init__(self, config):
        super().__init__(config)
        self.config = config
        self.packet = EMM_PACKET
        self.position = 0
        self.zero_timeout = False
        self.thread_ids = set()

    def read_status(self):
        self.thread_ids.add(threading.get_ident())
        sample = super().read_status()
        sample['position_deg'] = self.position * 360 / self.config.steps_per_turn
        return sample

    def move_relative_pulses(self, **params):
        super().move_relative_pulses(**params)
        self.position += (-1 if params['ccw'] else 1) * params['pulses']
        return params['pulses']

    def set_current_limit_ma(self, value):
        self.actions.append(('current', value))

    def zero_position(self):
        self.actions.append(('zero',))
        self.position = 0
        if self.zero_timeout:
            raise ZDTTimeoutError('simulated lost acknowledgement')


class ProtocolTests(unittest.TestCase):
    def test_zero_and_current_frames_and_timeout_no_retry(self):
        motor, bus = driver(lambda msg: frames(bytes((1, msg.data[0], 2, 0x6b))))
        try:
            motor.zero_position()
            motor.set_current_limit_ma(3000)
            self.assertEqual([(m.arbitration_id, bytes(m.data)) for m in bus.sent],
                             [(0x100, bytes.fromhex('0A 6D 6B')),
                              (0x100, bytes.fromhex('45 66 00 0B B8 6B'))])
        finally:
            motor.close()
        motor, bus = driver(lambda msg: [])
        try:
            with self.assertRaises(ZDTTimeoutError):
                motor.zero_position()
            self.assertEqual(len(bus.sent), 1)
        finally:
            motor.close()

    def test_invalid_step_scale_rejected(self):
        for value in (0, -1, 2.5, True, 1000001):
            with self.assertRaises(ValueError):
                ZDTConfig(steps_per_turn=value)


class MappingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def pump_until(self, condition, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.app.processEvents()
            if condition():
                return
            time.sleep(.005)
        self.fail('timed out')

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        for name, value in [('mapping_directory', lambda: self.root),
                            ('mapping_settings_path', lambda: self.root/'settings.json')]:
            patcher = patch.object(mapping, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.motor = MappingSimMotor(ZDTConfig())
        self.backend = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda config: self.motor)
        self.control = create_mapping_control('zdt_x42s', None, self.backend)
        self.page = mapping.ForcePositionMappingWidget(self.control)
        self.addCleanup(self.close_widgets)
        self.backend.connect_can()
        self.pump_until(lambda: self.control.is_can_connected() and not self.control.is_operation_busy())

    def close_widgets(self):
        self.page.shutdown()
        self.backend.shutdown()
        self.page.deleteLater()
        self.backend.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()

    def test_steps_conversion_profile_and_file_isolation(self):
        self.motor.position = -800
        self.pump_until(lambda: self.page.position_label.text().startswith('电机位置 -800 step'))
        self.assertEqual(self.page.press_step_input.value(), 32)
        self.assertEqual(self.page.acceleration_input.maximum(), 255)
        self.assertEqual(self.page.press_speed_input.value(), 10)
        self.assertEqual(self.page._profile.speed_rpm(10), 10)
        self.assertEqual(self.control.profile.tolerance(50), 3)
        self.assertIn('zdt_x42s_1_3200', str(self.page._latest_mapping_path()))
        self.assertFalse(self.page._latest_mapping_path().exists())
        self.page._save_settings_if_idle()
        settings = json.loads(self.page._settings_path.read_text(encoding='utf-8'))
        self.assertEqual(settings['press_step'], 32)
        self.assertEqual(settings['press_speed_rpm'], 10)
        self.assertEqual(settings['acceleration_level'], 100)
        self.assertNotIn('press_step_counts', settings)

    def test_zero_invalidates_table_and_release_even_on_ack_timeout(self):
        for lost_ack in (False, True):
            with self.subTest(lost_ack=lost_ack):
                self.motor.position = 800
                self.pump_until(lambda: self.control.latest_motor_feedback()['position_counts'] == 800)
                self.control.capture_release_position()
                self.assertTrue(self.control.release_position_snapshot()['confirmed'])
                self.page._mapping = [{'force_n': 1}]
                self.motor.zero_timeout = lost_ack
                self.page.zero_position_button.click()
                self.pump_until(lambda: not self.control.is_operation_busy())
                self.assertFalse(self.page._mapping)
                self.assertFalse(self.control.release_position_snapshot()['confirmed'])
                payload = json.loads(self.page._latest_mapping_path().read_text(encoding='utf-8'))
                self.assertEqual(payload['metadata']['invalidated_reason'], 'motor_position_zeroed')
                self.assertEqual(self.control.latest_motor_feedback()['position_counts'], 0)
                self.assertIn('不重发' if lost_ack else '反馈已确认', self.page.status_label.text())
        self.assertEqual(self.motor.actions.count(('zero',)), 2)

    def test_real_scan_loop_uses_step_commands_and_saves_step_schema(self):
        self.control.capture_release_position()
        self.page.repeat_count_input.setValue(2)
        self.page.max_force_input.setValue(.1)
        self.page._confirm_start_calibration = lambda _: True
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: self.page.set_pressure_meter_pressure(abs(self.motor.position)/32, 'sim'))
        timer.start(10)
        try:
            self.page.set_pressure_meter_pressure(0, 'sim')
            self.page.start_calibration()
            self.assertTrue(self.page.is_running())
            self.assertFalse(self.backend.press_button.isEnabled())
            self.pump_until(lambda: not self.page.is_running() and not self.control.is_operation_busy(), 8)
            self.assertEqual(len(self.page._round_results), 2, self.page.status_label.text())
            self.assertEqual(self.motor.position, 0)
            moves = [a[1] for a in self.motor.actions if a[0] == 'move']
            self.assertEqual([x['pulses'] for x in moves], [32, 32, 32, 32])
            self.assertEqual([x['ccw'] for x in moves], [True, False, True, False])
            self.assertEqual([x['speed_rpm'] for x in moves], [10, 24, 10, 24])
            self.assertEqual(self.motor.thread_ids, {self.backend._thread.ident})
            payload = json.loads(self.page._latest_mapping_path().read_text(encoding='utf-8'))
            self.assertEqual(payload['metadata']['position_unit'], 'step')
            self.assertEqual(payload['metadata']['steps_per_turn'], 3200)
            self.assertIn('press_travel_steps', payload['mapping'][0])
            self.assertNotIn('press_travel_counts', payload['mapping'][0])
            self.page._mapping = []
            self.page._load_latest_mapping()
            self.assertTrue(self.page._mapping)
        finally:
            timer.stop()

    def test_stop_cancels_external_wait_and_releases_controls(self):
        def operation(motor, cancel):
            cancel.wait(3)
            raise RuntimeError('cancelled' if cancel.is_set() else 'timeout')
        self.assertTrue(self.page._start_external_operation('test', operation, 'test'))
        self.page.request_stop()
        self.pump_until(lambda: not self.page.is_running() and not self.control.is_operation_busy())
        self.assertIn(('stop',), self.motor.actions)
        self.assertTrue(self.backend.press_button.isEnabled())

    def test_overpressure_prevents_move_and_failure_stops(self):
        self.page.set_pressure_meter_pressure(35, 'sim')
        self.page._start_external_operation('test', lambda motor, cancel:
            self.page._move_relative_checked(motor, cancel, -32, 10, 100, 3000), 'test')
        self.pump_until(lambda: not self.page.is_running() and not self.control.is_operation_busy())
        self.assertFalse(any(a[0] == 'move' for a in self.motor.actions))
        self.assertIn(('stop',), self.motor.actions)
        self.assertIn('保护上限', self.page.status_label.text())

    def test_calibration_can_reach_30n_and_move_below_35n(self):
        self.page.max_force_input.setValue(30)
        self.assertEqual(self.page.max_force_input.value(), 30)
        for force in (30, 34.99):
            with self.subTest(force=force):
                self.motor.actions.clear()
                self.page.set_pressure_meter_pressure(force, 'sim')
                self.assertTrue(self.page._start_external_operation('test', lambda motor, cancel:
                    self.page._move_relative_checked(motor, cancel, -32, 10, 100, 3000), 'test'))
                self.pump_until(lambda: not self.page.is_running() and not self.control.is_operation_busy())
                self.assertTrue(any(a[0] == 'move' for a in self.motor.actions))
                self.assertNotIn('保护上限', self.page.status_label.text())

    def test_reaching_35n_during_move_stops_motor(self):
        with patch.object(self.page, '_fresh_pressure', side_effect=[(34.99, 1), (35, 2)]):
            self.assertTrue(self.page._start_external_operation('test', lambda motor, cancel:
                self.page._move_relative_checked(motor, cancel, -32, 10, 100, 3000), 'test'))
            self.pump_until(lambda: not self.page.is_running() and not self.control.is_operation_busy())
        self.assertTrue(any(a[0] == 'move' for a in self.motor.actions))
        self.assertIn(('stop',), self.motor.actions)
        self.assertIn('35.00 N', self.page.status_label.text())

    def test_recalculate_visible_200_row_table_remains_responsive(self):
        # 必须先填满并显示表格，再重复点击；空表首次填充无法复现逐格自动列宽的卡顿。
        self.page._round_results = [
            dict(round_index=index, release_reference_position_counts=0,
                 points=[dict(force_n=point / 3, position_counts=-(700 + point * 32 + offset))
                         for point in range(70)])
            for index, offset in enumerate((0, 50, 60, 62, 64), start=1)
        ]
        self.page._mapping_metadata = dict(max_force_n=20, force_unit='N')
        self.page._mapping = mapping.build_force_position_map(self.page._round_results, 20, 8)
        self.page._rebuild_round_selector()
        self.page._populate_mapping_table()
        self.page.resize(1000, 1000)
        self.page.show()
        self.app.processEvents()
        self.assertFalse(any(entry['stable'] for entry in self.page._mapping))
        for index, checkbox in self.page._round_checkboxes:
            checkbox.setChecked(index >= 3)
        for _ in range(2):
            self.page._refresh_controls()
            self.assertTrue(self.page.recalculate_button.isEnabled())
            started = time.perf_counter()
            self.page.recalculate_button.click()
            self.app.processEvents()
            elapsed = time.perf_counter() - started
            self.assertLess(elapsed, 1.0, f'重新统计阻塞界面 {elapsed:.3f}s')
            self.assertEqual(self.page.table.rowCount(), 200)
            self.assertTrue(self.page.table.updatesEnabled())
            self.assertTrue(all(entry['stable'] and entry['sample_count'] == 3
                                for entry in self.page._mapping))
            saved = json.loads(self.page._latest_mapping_path().read_text(encoding='utf-8'))
            self.assertEqual(saved['metadata']['selected_round_indices'], [3, 4, 5])
            self.assertEqual(len(saved['mapping']), 200)
            self.assertEqual(len(saved['rounds']), 5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
