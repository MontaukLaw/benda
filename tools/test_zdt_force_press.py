"""新电机按力下压的端到端模拟：无硬件，所有记录和设置写入临时目录。"""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'PY_SRIPT/algo_monitor'))
sys.path.insert(0, str(ROOT/'tools'))
from test_pressure_feedback import Plant
from pressure_feedback_control import PressureFeedbackController, LearnedPressureMap
from pressure_feedback_widget import PressureFeedbackWidget
from force_press_control_widget import ForceControlError, ForcePressControlWidget
from zdt_force_press_control import ZDTForcePressControlWidget
from zdt_motor_test_widget import ZDTMotorTestWidget
from zdt_x42s_can_driver import ZDTConfig
from motor_mapping_interface import create_mapping_control
from qt_compat import QtWidgets


class NativePressTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        for name, value in (
            ('force_press_control_widget.force_press_settings_path', self.root/'config.json'),
            ('zdt_force_press_control.force_press_settings_path', self.root/'config.json'),
            ('force_press_control_widget.force_press_test_directory', self.root/'records'),
            ('zdt_force_press_control.mapping_directory', self.root/'maps'),
            ('zdt_force_press_control.force_sweep_checkpoint_path', self.root/'checkpoint.json'),
        ):
            patcher = patch(name, return_value=value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.backend = ZDTMotorTestWidget(ZDTConfig())
        self.control = create_mapping_control('zdt_x42s', None, self.backend)
        self.page = ZDTForcePressControlWidget(self.control)
        self.feedback = PressureFeedbackWidget(self.control, settings_path=self.root/'feedback.json',
                                               output_dir=self.root/'feedback_records')
        for key, value in dict(speed=10, release_speed=24, acceleration=100, max_step=256,
                               tolerance_percent=2, force_limit_n=35).items():
            self.feedback.inputs[key].setValue(value)
        self.page.set_feedback_source(self.feedback)
        self.addCleanup(self.cleanup_widgets)
        self.control._release = 0
        self.page.reload_lookup_mapping()
        self.backend.speed_input.setValue(60)
        self.backend.acceleration_input.setValue(0)
        self.backend.pulses_input.setValue(8)

    def cleanup_widgets(self):
        self.page.shutdown()
        self.feedback.shutdown()
        self.backend.shutdown()
        self.page.deleteLater()
        self.feedback.deleteLater()
        self.backend.deleteLater()
        self.app.processEvents()

    def parameters(self, force=1, travel=0):
        return dict(self.page._operation_motion_parameters(), target_force_n=force, mode='press',
                    release_position_counts=0, lookup_press_travel_counts=travel,
                    lookup_target_position_counts=-travel, lookup_created_at='test',
                    wait_for_sensor_idle_after_release=False)

    def prepare_simulation(self, plant):
        self.page._feedback_samples = plant.samples
        original_wait = plant.wait
        plant.wait = lambda seconds: original_wait(max(seconds, .000001))
        def controller(*args, **kwargs):
            kwargs['clock'] = lambda: plant.now
            return PressureFeedbackController(*args, **kwargs)
        # 原有采样流程同样取当前压力计原始值；更新两套消费者，不能制造旧缓存。
        original_read = plant.read_feedback
        def read():
            sample = original_read()
            if plant.fault != 'stale':
                self.page.set_pressure_sample(dict(force_n=plant.force(), raw_force_n=plant.force(),
                    filtered_force_n=plant.force(), force_unit='N', accepted=True,
                    sequence=round(plant.now*1000), sampled_monotonic=plant.now))
            return sample
        plant.read_feedback = read
        return patch('zdt_force_press_control.PressureFeedbackController', side_effect=controller)

    def test_arrival_matches_feedback_page_motion_and_does_not_run_hold_timer(self):
        for target in (0.1, 1, 20, 30):
            with self.subTest(target=target):
                plant = Plant()
                # 较细的硬件分辨率，可精确达到0.1 N。
                plant.force = lambda: max(0, (-plant.position-10)*.01)
                travel = round(target/.01+10)
                parameters = self.parameters(target, travel)
                with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
                    result = self.page._run_press(plant, {}, plant, parameters)
                self.assertTrue(result['ok'])
                self.assertLessEqual(abs(plant.force()-target), self.feedback._parameters(target_override=target).tolerance_n+1e-9)
                direct = Plant()
                direct.force = lambda: max(0, (-direct.position-10)*.01)
                controller = PressureFeedbackController(direct, self.control.profile, direct.samples, direct,
                    self.feedback._parameters(target_override=target), 0, clock=lambda: direct.now)
                direct_result = controller.run(acquire_only=True)
                self.assertTrue(direct_result['ok'], direct_result)
                self.assertEqual(plant.actions, direct.actions)
                self.assertEqual([{k: v for k, v in p.items() if not callable(v)} for p in plant.motion_parameters],
                                 [{k: v for k, v in p.items() if not callable(v)} for p in direct.motion_parameters])
                self.assertTrue(all(p['speed_rpm'] == (2 if target <= .2 else 10) and p['accel'] == 100 for p in plant.motion_parameters))
                self.assertNotIn('hold_remaining_s', result)
                self.assertGreater(plant.stop_count, 0)

    def test_no_table_uses_feedback_settings_and_history_does_not_change_motion(self):
        plant = Plant()
        self.feedback.inputs['max_step'].setValue(16)
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
            first = self.page._run_press(plant, {}, plant, self.parameters(5))
            self.assertTrue(all(abs(step) <= 16 for step in plant.actions))
            self.assertEqual(first['lookup_target_position_counts'], first['final_position_counts'])
            self.page._run_release(plant, {}, plant, self.parameters(5))
            plant.actions.clear()
            self.page._run_press(plant, {}, plant, self.parameters(5))
            self.assertTrue(all(abs(step) <= 16 for step in plant.actions))

    def test_loaded_descending_target_micro_adjusts_upwards(self):
        plant = Plant()
        plant.position = -135  # 5 N
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
            self.page._run_press(plant, {}, plant, self.parameters(1, 35), require_zero_start=False)
        self.assertGreater(plant.actions[0], 0)
        self.assertAlmostEqual(plant.force(), 1, delta=.02)

    def test_release_returns_to_position_in_one_command_even_at_zero_pressure(self):
        plant = Plant()
        plant.position = -700
        plant.force = lambda: 0
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
            self.page._run_release(plant, {}, plant, self.parameters())
        self.assertEqual(plant.actions, [700])
        self.assertEqual(plant.position, 0)
        self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 24)

    def test_faults_stop_without_repeating_a_position_command(self):
        for fault in ('lost_ack', 'stale', 'wrong_direction', 'stalled'):
            with self.subTest(fault=fault):
                plant = Plant()
                plant.fault = fault
                with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
                    with self.assertRaises(ForceControlError):
                        self.page._run_press(plant, {}, plant, self.parameters(5, 135))
                self.assertLessEqual(len(plant.actions), 1)
                self.assertGreater(plant.stop_count, 0)

    def test_mapping_rejects_old_invalid_or_malformed_documents(self):
        path = self.root/'maps'/self.control.profile.namespace/'force_position_map_latest.json'
        path.parent.mkdir(parents=True)
        metadata = dict(self.control.profile.metadata(), force_unit='N', mapping_mode='release_relative_travel',
                        release_reference_position_steps=0, created_at='original')
        valid = dict(metadata=metadata, mapping=[dict(force_n=1, stable=True, press_travel_steps=100, position_span_steps=1)])
        for document in ([], {'metadata': []}, dict(valid, metadata=dict(metadata, invalidated_reason='zeroed')),
                         dict(valid, metadata=dict(metadata, motor_model='pd42s1')),
                         dict(valid, metadata=dict(metadata, steps_per_turn=51200)),
                         dict(valid, metadata=dict(metadata, release_reference_position_steps=1)),
                         dict(valid, metadata=dict(metadata, release_reference_position_steps=200))):
            with self.subTest(document=document):
                path.write_text(json.dumps(document), encoding='utf8')
                self.page.reload_lookup_mapping()
                self.assertTrue(self.page._lookup_document.get('feedback_only'))
                self.assertEqual(self.page._lookup_targets[1]['press_travel_counts'], 0)
        path.write_text(json.dumps(valid), encoding='utf8')
        self.page.reload_lookup_mapping()
        self.assertEqual(self.page._lookup_targets[1]['target_position_counts'], -100)

    def test_shared_settings_are_live_and_copied_per_operation(self):
        first = self.parameters()
        self.backend.speed_input.setValue(120)
        self.backend.acceleration_input.setValue(80)
        self.backend.pulses_input.setValue(64)
        second = self.parameters()
        self.assertEqual(second, first)
        self.feedback.inputs['speed'].setValue(12)
        self.feedback.inputs['release_speed'].setValue(30)
        self.feedback.inputs['acceleration'].setValue(80)
        self.feedback.inputs['max_step'].setValue(64)
        self.feedback.inputs['tolerance_percent'].setValue(3)
        self.feedback.inputs['settle_s'].setValue(.4)
        self.feedback.inputs['stable_s'].setValue(2)
        second = self.parameters(20)
        self.assertEqual((first['press_speed_level'], first['acceleration'], first['fine_step_max']), (10, 100, 256))
        self.assertEqual((second['press_speed_level'], second['release_speed_level'], second['acceleration'], second['fine_step_max']), (12, 30, 80, 64))
        self.assertEqual(self.page._controller_settings(second), self.feedback._parameters(target_override=20))
        self.assertTrue(self.page.motion_parameter_group.isHidden())
        self.assertIn('12 RPM', self.page.native_motion_label.text())
        self.assertIn(self.control.profile.namespace, str(self.page._settings_path))
        self.assertIn(self.control.profile.namespace, str(self.page._sweep_checkpoint_path()))

    def test_stop_routes_to_new_backend_even_when_feedback_is_stale(self):
        with patch.object(self.control, 'is_can_connected', return_value=False), \
             patch.object(self.control, 'request_external_stop') as stop:
            self.page.request_stop()
        stop.assert_called_once_with()
        plant = Plant()
        plant.cancel_at = 11
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
            with self.assertRaises(ForceControlError) as error:
                self.page._run_press(plant, {}, plant, self.parameters(30))
        self.assertEqual(error.exception.reason, 'user_stop')
        self.assertGreater(plant.stop_count, 0)

    def test_learning_is_shared_only_for_current_motor_and_release_reference(self):
        profile = self.control.profile
        learned = LearnedPressureMap(profile.model, profile.namespace, profile.steps_per_turn,
                                     0, ((.2, 15), (1, 35), (5, 135), (30, 760)))
        self.page.set_learned_map_provider(lambda: learned)
        self.assertEqual(self.page._resolve_lookup_target(self.page._lookup_document, 5)['press_travel_counts'], 135)
        # 断点来源保持一致，不因某一压力用学习/寻压改变整个任务的记录身份。
        self.assertEqual(self.page._resolve_lookup_target(self.page._lookup_document, 5)['created_at'], self.page._lookup_document['metadata']['created_at'])
        self.page._native_history[(0, 1)] = 35
        self.control._release = 50
        self.page._on_release_position_changed(self.control.release_position_snapshot())
        self.assertFalse(self.page._native_history)
        self.assertEqual(self.page._resolve_lookup_target(self.page._lookup_document, 5)['press_travel_counts'], 0)

    def test_calibration_validation_and_quick_correction_use_native_acquisition(self):
        for mode in ('sweep', 'validation', 'quick_correction'):
            with self.subTest(mode=mode):
                plant = Plant()
                def capture(*args):
                    plant.wait(1)
                    return dict(sample={}, motor_feedback=plant.read_feedback(), raw_medians=[1000]*4,
                                raw_median_sum=4000, raw_median_sample_count=10, raw_median_window_seconds=1)
                parameters = dict(self.parameters(1), mode=mode, operation_id=1)
                context = {key: 1 for key in ('auto_round_index', 'auto_round_total', 'pressure_level_index',
                           'pressure_level_total', 'calibration_cycle_index', 'calibration_cycle_total',
                           'direction_step_index', 'direction_step_total', 'repeat_index', 'repeat_total', 'attempt_index')}
                context.update(round_name='模拟标定', sweep_direction='ascending')
                values = dict(signals=[100]*4, signal_sum=400, floating_count=4)
                with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now), \
                     patch.object(self.page, '_wait_for_all_floating_stable', side_effect=capture), \
                     patch.object(self.page, '_sensor_force_values', return_value=values):
                    entry, _ = self.page._run_sweep_capture_round(plant, {}, plant, parameters, context)
                self.assertAlmostEqual(entry['press_arrival_force_n'], 1, delta=.02)
                self.assertEqual(plant.position, 0)
                self.assertEqual(entry['press_arrival_position_error_counts'], 0)
                row = self.page._force_mapping_export_row(entry)
                self.assertEqual((row['motor_model'], row['speed_unit'], row['acceleration_unit']), ('zdt_x42s', 'RPM', '档'))

    def test_button_submits_shared_backend_and_freezes_motion_parameters(self):
        plant = Plant()
        submitted = []
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now), \
             patch.object(self.control, 'is_can_connected', return_value=True), \
             patch.object(self.control, 'latest_motor_feedback',
                          side_effect=lambda: dict(position_counts=plant.position, measured_speed_rpm=0,
                                                   running_state=0, phase_current_ma=200, sampled_monotonic=plant.now)), \
             patch.object(self.control, 'start_external_operation',
                          side_effect=lambda task, status: (submitted.append(task) is None, '')):
            plant.read_feedback()
            self.page.start_press(1)
            self.assertEqual(len(submitted), 1, self.page.last_status_message())
            # 已提交任务使用冻结参数；不从工作线程访问Qt控件。
            self.feedback.inputs['speed'].setValue(12)
            self.feedback.inputs['max_step'].setValue(8)
            submitted[0](plant, plant)
        self.assertTrue(all(p['speed_rpm'] == 10 for p in plant.motion_parameters))
        self.assertAlmostEqual(plant.force(), 1, delta=.02)

    def test_auto_repeat_uses_native_motion_for_every_press_and_release(self):
        plant = Plant()
        captures = []
        def capture(*args):
            captures.append(plant.force())
            plant.wait(1)
            feedback = plant.read_feedback()
            return dict(sample={}, motor_feedback=feedback, raw_medians=[1000]*4,
                        raw_median_sum=4000, raw_median_sample_count=10, raw_median_window_seconds=1)
        values = dict(signals=[100]*4, signal_sum=400, floating_count=4)
        parameters = dict(self.parameters(5), auto_rounds=2, operation_id=1, auto_test_interval_seconds=.2)
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now), \
             patch.object(self.page, '_wait_for_all_floating_stable', side_effect=capture), \
             patch.object(self.page, '_sensor_force_values', return_value=values):
            result = self.page._run_auto_test(plant, {}, plant, parameters)
        self.assertTrue(result['ok'])
        self.assertEqual(len(captures), 2)
        self.assertTrue(all(abs(f-5) <= .1 for f in captures))
        self.assertEqual(plant.position, 0)
        self.assertEqual(len([step for step in plant.actions if step > 100]), 2)
        self.assertTrue(all(p['speed_rpm'] == (24 if step > 100 else 10) and p['accel'] == 100
                            for step, p in zip(plant.actions, plant.motion_parameters)))

    def test_learned_20n_approach_matches_feedback_page(self):
        profile = self.control.profile
        table = LearnedPressureMap(profile.model, profile.namespace, profile.steps_per_turn,
                                   0, ((.2, 15), (1, 35), (5, 135), (30, 760)))
        self.feedback._learned_map = table
        plant = Plant()
        with patch.object(self.control, 'is_can_connected', return_value=True):
            parameters = self.parameters(20, 900)
        # 提交后清除页面引用不改变该任务快照；旧位置表不替代共用学习路径。
        self.feedback._learned_map = None
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
            self.page._run_press(plant, {}, plant, parameters)
        direct = Plant()
        controller = PressureFeedbackController(direct, profile, direct.samples, direct,
            self.feedback._parameters(target_override=20), 0, clock=lambda: direct.now, learned_map=table)
        self.assertTrue(controller.run(acquire_only=True)['ok'])
        self.assertEqual(plant.actions, direct.actions)
        self.assertEqual([{k: v for k, v in p.items() if not callable(v)} for p in plant.motion_parameters],
                         [{k: v for k, v in p.items() if not callable(v)} for p in direct.motion_parameters])
        self.assertGreater(abs(plant.actions[0]), 256)
        self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 30)
        self.assertTrue(all(p['speed_rpm'] == 10 for p in plant.motion_parameters[1:]))

    def test_contact_point_fast_path_matches_both_pages_for_point_one(self):
        profile = self.control.profile
        table = LearnedPressureMap(profile.model, profile.namespace, profile.steps_per_turn, 0,
            ((.05, 616), (.1, 632), (.19, 660), (1, 765)),
            contact_zero_travel=596, contact_travel=600, contact_force_n=.01)
        self.feedback._learned_map = table
        plant = Plant()
        plant.force = lambda: round(max(0, (-plant.position-596)*.0025), 2)
        with patch.object(self.control, 'is_can_connected', return_value=True):
            parameters = self.parameters(.1)
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
            result = self.page._run_press(plant, {}, plant, parameters)
        self.assertTrue(result['ok'], result)
        direct = Plant()
        direct.force = lambda: round(max(0, (-direct.position-596)*.0025), 2)
        controller = PressureFeedbackController(direct, profile, direct.samples, direct,
            self.feedback._parameters(target_override=.1), 0, clock=lambda: direct.now, learned_map=table)
        result = controller.run(acquire_only=True)
        self.assertTrue(result['ok'], result)
        self.assertTrue(result['used_contact_approach'])
        self.assertEqual(plant.actions, direct.actions)
        self.assertEqual([p['speed_rpm'] for p in plant.motion_parameters],
                         [p['speed_rpm'] for p in direct.motion_parameters])
        self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 30)
        self.assertTrue(all(p['speed_rpm'] <= 2 for p in plant.motion_parameters[1:]))

    def test_monitor_fault_suspends_shared_table_and_allows_release_only_until_recovered(self):
        profile = self.control.profile
        table = LearnedPressureMap(profile.model, profile.namespace, profile.steps_per_turn,
                                   0, ((.2, 15), (1, 35), (5, 135), (29.88, 760)))
        self.feedback._learned_map = table
        now = [10.0]
        with patch('time.monotonic', lambda: now[0]), \
                patch.object(self.control, 'is_can_connected', return_value=True), \
                patch.object(self.control, 'is_operation_busy', return_value=False), \
                patch.object(self.control, 'latest_motor_feedback', side_effect=lambda: dict(
                    position_counts=0, sampled_monotonic=now[0], measured_speed_rpm=0,
                    running_state=0, in_position=True, phase_current_ma=100)):
            self.page.operation_finished.emit(dict(ok=False, mode='press', message='停止反馈未确认'))
            self.assertIs(self.feedback._learned_map, table)
            self.assertTrue(self.feedback.learning_recovery_pending())
            self.assertIn('停止反馈未确认', self.page.learning_notice_label.text())
            self.assertIsNone(self.parameters(20)['learned_map'])
            with patch.object(ForcePressControlWidget, '_start_operation') as start:
                self.page._start_operation('press', 20)
                start.assert_not_called()
                self.page._start_operation('release', None)
                start.assert_called_once_with('release', None)
            for sequence in range(1, 4):
                now[0] += .11
                self.feedback.set_pressure_sample(dict(accepted=True, force_unit='N', force_n=.01,
                                                       sequence=sequence, sampled_monotonic=now[0]))
                self.feedback._refresh()
            self.assertFalse(self.feedback.learning_recovery_pending())
            self.assertIs(self.parameters(20)['learned_map'], table)
            self.assertTrue(self.page.learning_notice_label.isHidden())

    def test_feedback_protection_is_not_raised_for_monitor_target(self):
        self.feedback.inputs['force_limit_n'].setValue(5)
        plant = Plant()
        with self.prepare_simulation(plant), patch('time.monotonic', lambda: plant.now):
            with self.assertRaises(ForceControlError):
                self.page._run_press(plant, {}, plant, self.parameters(20))
        self.assertEqual(plant.actions, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
