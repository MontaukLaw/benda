"""任务恢复后的学习提示只反映事实，不恢复新任务权限或掩盖真实异常。"""
import unittest
from contextlib import ExitStack
from unittest.mock import patch

import test_zdt_force_press as native_tests
from pressure_feedback_control import LearnedPressureMap
from high_force_calibration_widget import HighForceCalibrationWidget
from qt_compat import QtWidgets


class LearningTaskNoticeTests(unittest.TestCase):
    setUp = native_tests.NativePressTests.setUp
    cleanup_widgets = native_tests.NativePressTests.cleanup_widgets

    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def task(self):
        table = LearnedPressureMap(self.control.profile.model, self.control.profile.namespace,
            self.control.profile.steps_per_turn, 0, ((.1, 12.5), (1, 35), (29.52, 760)))
        self.feedback._learned_map = table
        self.now = [20.0]
        self.motor_sample = dict(position_counts=-500, measured_speed_rpm=0, phase_current_ma=100,
            running_state=0, in_position=True, sampled_monotonic=20)
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch('time.monotonic', lambda: self.now[0]))
        stack.enter_context(patch.object(self.control, 'is_can_connected', return_value=True))
        stack.enter_context(patch.object(self.control, 'is_operation_busy', return_value=True))
        stack.enter_context(patch.object(self.control, 'latest_motor_feedback', side_effect=lambda: dict(self.motor_sample)))
        self.feedback.samples.update(dict(force_n=20, force_unit='N', accepted=True,
            sequence=1, sampled_monotonic=20))
        widget = HighForceCalibrationWidget(self.control, self.feedback, lambda: {}, None,
            output_dir=self.root/'auto')
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.timer.stop)
        widget._running = True
        self.feedback.begin_learning_task_notice(table)
        self.feedback._control_state_changed(False)
        return widget, table

    def test_recovered_task_uses_information_style_but_new_start_remains_blocked(self):
        widget, table = self.task()
        widget._progress(dict(phase='press'))
        self.assertIn('本轮启动时已核验', self.page.learning_notice_label.text())
        self.assertNotIn('暂时停用', self.page.learning_notice_label.text())
        self.assertEqual(self.feedback._learning_notice_level, 'info')
        self.assertEqual(self.page.learning_notice_label.styleSheet(), self.feedback.learning_notice_label.styleSheet())
        self.assertTrue(self.feedback.learning_recovery_pending())
        self.assertIsNone(self.feedback.learned_map_snapshot())
        self.assertIs(self.feedback._learned_map, table)
        self.assertFalse(self.feedback.start_button.isEnabled())
        self.assertIn('本轮启动时已核验', self.feedback.learning_status_label.text())

    def test_recovery_and_retry_wait_remain_warning_until_progress_continues(self):
        widget, _ = self.task()
        for phase in ('recovery', 'recovery_release', 'retry_wait'):
            widget._progress(dict(phase=phase, message='等待恢复'))
            self.assertIn('正在恢复', self.page.learning_notice_label.text())
            self.assertEqual(self.feedback._learning_notice_level, 'warning')
        widget._progress(dict(phase='retry', message='已恢复，准备重试'))
        self.assertEqual(self.feedback._learning_notice_level, 'info')

    def test_missing_pressure_motor_feedback_or_fault_never_shows_recovered(self):
        widget, _ = self.task()
        widget._progress(dict(phase='press'))
        for key, value in (('sampled_monotonic', 19), ('phase_current_ma', 5000),
                           ('running_state', 4), ('position_counts', float('nan'))):
            original = self.motor_sample[key]
            self.motor_sample[key] = value
            self.feedback._refresh()
            self.assertEqual(self.feedback._learning_notice_level, 'warning')
            self.assertNotIn('实时反馈已恢复', self.page.learning_notice_label.text())
            self.motor_sample[key] = original
        self.now[0] = 21
        self.motor_sample['sampled_monotonic'] = 21
        self.feedback._refresh()
        self.assertEqual(self.feedback._learning_notice_level, 'warning')

    def test_task_end_requires_existing_unloaded_recovery_before_new_start(self):
        widget, _ = self.task()
        widget._progress(dict(phase='press'))
        widget._finished(dict(ok=False, cancelled=True, stop_confirmed=True, release_position_confirmed=True,
                              message='用户停止', records=[]))
        self.assertIsNone(self.feedback._learning_task_notice)
        self.assertIn('等待新任务启动前核验', self.page.learning_notice_label.text())
        self.assertTrue(self.feedback.learning_recovery_pending())
        self.assertIsNone(self.feedback.learned_map_snapshot())

    def test_emergency_notice_is_not_overwritten_by_late_progress(self):
        widget, _ = self.task()
        with patch.object(self.control, 'request_external_stop'):
            widget.emergency_stop()
        widget._progress(dict(phase='press'))
        self.assertIn('紧急停止', self.page.learning_notice_label.text())
        self.assertEqual(self.feedback._learning_notice_level, 'warning')

    def test_changed_reference_cannot_display_old_table_as_usable(self):
        widget, _ = self.task()
        self.control._release = 200
        self.feedback._release_reference_changed()
        widget._progress(dict(phase='press'))
        self.assertIsNone(self.feedback._learned_map)
        self.assertNotIn('本轮启动时已核验', self.page.learning_notice_label.text())


if __name__ == '__main__':
    unittest.main()
