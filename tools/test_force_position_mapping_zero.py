"""标定轮间零点容差、等待退出与重复扫描的无硬件回归。"""
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
import force_position_mapping_widget as mapping


class Clock:
    def __init__(self):
        self.now = 100.0
        self.stopped = False
        self.cancel_on_wait = False

    def is_set(self):
        return self.stopped

    def wait(self, seconds):
        self.now += seconds
        self.stopped |= self.cancel_on_wait


class MappingZeroTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.time_patch = patch.object(mapping.time, 'monotonic', lambda: self.clock.now)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        self.widget = SimpleNamespace(progress_changed=SimpleNamespace(emit=Mock()), _profile=mapping.LEGACY_PROFILE)
        for name in ('_mapping_directory', '_latest_mapping_path', '_write_mapping_json'):
            setattr(self.widget, name, getattr(mapping.ForcePositionMappingWidget, name).__get__(self.widget))
        self.widget._wait_for_mapping_zero = mapping.ForcePositionMappingWidget._wait_for_mapping_zero.__get__(self.widget)

    def test_zero_band_includes_point_one_but_rejects_invalid_and_large_negative(self):
        for force in (0, .06, .099, .1, -.1):
            with self.subTest(force=force):
                self.assertTrue(mapping.pressure_is_near_mapping_zero(force))
        for force in (.100001, .13, -.100001, -2, math.nan, math.inf):
            with self.subTest(force=force):
                self.assertFalse(mapping.pressure_is_near_mapping_zero(force))

    def test_new_round_accepts_point_zero_six(self):
        self.widget._fresh_pressure = Mock(return_value=(.06, 1))
        self.assertEqual(self.widget._wait_for_mapping_zero(self.clock, '第3轮'), .06)
        self.assertEqual(self.clock.now, 100)

    def test_pressure_decay_continues_without_restarting_operation(self):
        self.widget._fresh_pressure = Mock(side_effect=[(.13,1),(.11,2),(.08,3)])
        self.assertEqual(self.widget._wait_for_mapping_zero(self.clock, '释放后'), .08)
        self.assertGreater(self.clock.now, 100)

    def test_release_confirmation_requires_post_arrival_frame(self):
        self.widget._fresh_pressure = Mock(side_effect=[(.06,7),(.06,7),(.09,8)])
        self.assertEqual(self.widget._wait_for_mapping_zero(self.clock, '释放后', after_sequence=7), .09)
        self.assertEqual(self.widget._fresh_pressure.call_count, 3)

    def test_high_pressure_has_bounded_wait(self):
        self.widget._fresh_pressure = Mock(return_value=(.13,1))
        with self.assertRaisesRegex(mapping.ForcePositionMappingError, '零点确认超时'):
            self.widget._wait_for_mapping_zero(self.clock, '释放后')
        self.assertLessEqual(self.clock.now, 110.1)

    def test_stop_interrupts_wait_promptly(self):
        self.widget._fresh_pressure = Mock(return_value=(.13,1))
        self.clock.cancel_on_wait = True
        with self.assertRaisesRegex(mapping.ForcePositionMappingError, '用户停止'):
            self.widget._wait_for_mapping_zero(self.clock, '释放后')
        self.assertLess(self.clock.now, 100.1)

    def test_invalid_or_stale_data_does_not_confirm_zero(self):
        for value, updated in ((math.nan,100),(.06,98),(None,100)):
            with self.subTest(value=value, updated=updated):
                self.widget._pressure_snapshot = lambda: (value,'test',updated,1)
                self.widget._fresh_pressure = mapping.ForcePositionMappingWidget._fresh_pressure.__get__(self.widget)
                with self.assertRaisesRegex(mapping.ForcePositionMappingError, '无有效新鲜数据'):
                    self.widget._wait_for_mapping_zero(self.clock, '释放后')

    def test_start_validation_uses_mapping_tolerance(self):
        self.widget._running = False
        self.widget._motor_control = SimpleNamespace(is_operation_busy=lambda:False, is_can_connected=lambda:True)
        self.widget._confirmed_release_position = lambda: 0
        self.widget._set_status = Mock()
        self.widget._fresh_pressure = lambda: (.06,1)
        validate = mapping.ForcePositionMappingWidget._validate_start.__get__(self.widget)
        self.assertTrue(validate(require_zero=True))
        self.widget._fresh_pressure = lambda: (.13,2)
        self.assertFalse(validate(require_zero=True))

    def test_three_rounds_finish_with_residual_point_zero_six(self):
        motor = SimpleNamespace(position=0, prepare_motion=Mock(), set_current_limit_ma=Mock())
        motor.read_position_steps = lambda: motor.position
        seq = 0
        def pressure():
            nonlocal seq
            seq += 1
            return .06, seq
        def move(motor, stop, delta, *args, **kwargs):
            motor.position += delta
            return dict(position_counts=motor.position, commanded_delta_counts=delta,
                        actual_delta_counts=delta, target_error_counts=0)
        self.widget._fresh_pressure = pressure
        self.widget._pressure_snapshot = lambda: (.06,'test',self.clock.now,seq)
        self.widget._move_relative_checked = move
        self.widget._sample_pressure_median = lambda stop: (1.0,3)
        self.widget._release_to_clearance = mapping.ForcePositionMappingWidget._release_to_clearance.__get__(self.widget)
        parameters = dict(mapping.DEFAULT_SETTINGS, max_force_n=1.0, repeat_count=3,
                          release_position_counts=0, press_step_counts=512)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            with patch.object(mapping,'mapping_directory',lambda:directory), patch.object(mapping,'latest_mapping_path',lambda:directory/'latest.json'):
                result = mapping.ForcePositionMappingWidget._run_calibration(self.widget,motor,self.clock,parameters)
            self.assertTrue(result['ok'])
            self.assertEqual(len(result['rounds']),3)
            self.assertEqual(motor.position,0)
            self.assertTrue(all(r['points'][0]['force_n']==.06 for r in result['rounds']))
            saved=json.loads((directory/'latest.json').read_text(encoding='utf-8'))
            self.assertEqual(saved['metadata']['release_zero_tolerance_n'],.1)
            self.assertEqual(len(saved['rounds']),3)


if __name__ == '__main__':
    unittest.main()
