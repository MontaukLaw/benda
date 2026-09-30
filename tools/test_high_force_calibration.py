"""新电机自动标定的完整多轮、取消、失效反馈和K表兼容性回归。"""
import json
import copy
import tempfile
from dataclasses import replace
from pathlib import Path
import unittest
from unittest.mock import patch
from contextlib import ExitStack

import test_pressure_feedback as feedback_tests
from pressure_feedback_control import FeedbackSettings, LearnedPressureMap
from high_force_calibration import (HighForceCalibrationRunner, calibration_targets, build_high_force_mapping,
    DEFAULT_REST_SECONDS, RestWaitControl, validate_rest_seconds)
from high_force_calibration import recover_release
from high_force_calibration_widget import HighForceCalibrationWidget
from force_signal_pressure_calibration import select_firmware_calibration_document_nodes, _segments_from_nodes
from force_signal_pressure_calibration_widget import ForceSignalPressureCalibrationWidget
from qt_compat import QtWidgets


class ClockCancel:
    def __init__(self, plant, cancelled=lambda: False):
        self.plant, self.cancelled = plant, cancelled

    def is_set(self):
        return self.cancelled()

    def wait(self, seconds):
        self.plant.now += seconds
        return self.is_set()


class HighCalibrationTests(unittest.TestCase):
    def test_integer_targets_cover_full_range_without_duplicates(self):
        self.assertEqual(calibration_targets(10), (1, 4, 7, 11, 14, 17, 20, 24, 27, 30))
        self.assertEqual(calibration_targets(20), (1, 3, 4, 6, 7, 9, 10, 12, 13, 15,
                                                  16, 18, 19, 21, 22, 24, 25, 27, 28, 30))
        for count in (10, 20):
            targets = calibration_targets(count)
            self.assertEqual(len(set(targets)), count)
            self.assertTrue(all(type(p) is int for p in targets))
        for invalid in (0, 11, 30, True, 10.0):
            with self.assertRaises(ValueError): calibration_targets(invalid)

    def test_legacy_decimal_tables_remain_usable_but_mixed_plans_are_rejected(self):
        for count in (10, 20):
            _, result, _, _ = self.run_plan(count=count, rounds=1)
            document = copy.deepcopy(result['document'])
            for i, node in enumerate(document['nodes']):
                target = round(1+29*i/(count-1), 2)
                node.update(nominal_target_force_n=target, measured_pressure_median_n=target,
                            signal_sum_median=target*4000)
            document['segments'] = _segments_from_nodes(document['nodes'])
            selected = select_firmware_calibration_document_nodes(document)
            self.assertEqual(len(selected['nodes']), count)
            self.assertEqual(selected['nodes'][1]['nominal_target_force_n'], 4.22 if count == 10 else 2.53)
            document['nodes'][1]['nominal_target_force_n'] = calibration_targets(count)[1]
            document['segments'] = _segments_from_nodes(document['nodes'])
            with self.assertRaises(ValueError): select_firmware_calibration_document_nodes(document)

    def run_plan(self, count=10, rounds=2, fault=None, cancel_phase=None, cancel_after=None, progress_hook=None,
                 rest_seconds=DEFAULT_REST_SECONDS, rest_control=None, setup=None, staged=False, historical_targets=(),
                 resume_document=None):
        plant = feedback_tests.Plant()
        if fault:
            # 持续故障现在保留任务等待恢复；由模拟用户停止结束测试，不能等旧3次上限。
            plant.cancel_at = plant.now+180
        plant.action_times, plant.checkpoint_times = [], []
        original_move = plant.move_relative_steps
        def move(steps, **kwargs):
            plant.action_times.append((plant.now, steps))
            original_move(steps, **kwargs)
        plant.move_relative_steps = move
        profile = feedback_tests.PROFILE
        learned = LearnedPressureMap(profile.model, profile.namespace, profile.steps_per_turn, 0,
                                     tuple((f, 10+f/.04) for f in (.1, .2, 1, 5, 10, 20, 30)))
        phases, checkpoints = [], []
        original_feedback = plant.read_feedback
        def motor_feedback():
            result = original_feedback()
            if fault == 'invalid_speed': result['measured_speed_rpm'] = float('nan')
            if fault == 'invalid_current': result['phase_current_ma'] = float('nan')
            return result
        plant.read_feedback = motor_feedback
        if setup: setup(plant)
        def checkpoint(rows):
            checkpoints.append(rows)
            plant.checkpoint_times.append(plant.now)
            if len(rows) == cancel_after: plant.cancelled = True
        def sensor():
            loaded = plant.force() > 0
            states = [6 if loaded else 0]*8
            if fault == 'one_not_floating' and loaded: states[5] = 2
            if fault == 'old_floating': states = [6]*8
            return dict(sequence=round(plant.now*1000), age_seconds=1 if fault == 'stale' else 0,
                        state=states, signal=[0]*4+[round(plant.force()*1000*getattr(plant, 'signal_gain', 1))]*4)
        def progress(row):
            phases.append(dict(row))
            if row.get('phase') == 'rest':
                self.assertEqual(plant.position, 0)
                self.assertEqual(plant.force(), 0)
            if progress_hook: progress_hook(plant, row)
            if row.get('phase') == cancel_phase:
                plant.cancelled = True
        settings = replace(FeedbackSettings(), max_step=256, force_limit_n=35, target_n=30, tolerance_n=.6)
        runner = HighForceCalibrationRunner(plant, profile, plant.samples, sensor, plant, settings,
            0, learned, count, rounds, 2, clock=lambda: plant.now,
            progress=progress, checkpoint=checkpoint, rest_seconds=rest_seconds, rest_control=rest_control,
            historical_targets=historical_targets,
            resume_document=resume_document,
            recovery_cancel=ClockCancel(plant, lambda: getattr(plant, 'emergency_cancelled', False)
                or fault in ('invalid_speed', 'invalid_current') and plant.now >= 15))
        # 原有用例隔离正式采集/回位/休息机制；完整三阶段另有专门的实际控制器回归。
        with ExitStack() as stack:
            if not staged:
                stack.enter_context(patch.object(runner, '_warmup', return_value=None))
                stack.enter_context(patch.object(runner, '_targeted_retest',
                    side_effect=lambda: build_high_force_mapping(runner.records, count, rounds)))
            result = runner.run()
        return plant, result, phases, checkpoints

    def test_ten_and_twenty_points_complete_multiple_rounds_with_native_controller(self):
        for count in (10, 20):
            plant, result, phases, checkpoints = self.run_plan(count)
            self.assertTrue(result['ok'], result)
            self.assertEqual(len(result['records']), count*2)
            self.assertEqual(len(checkpoints), count*2)
            self.assertEqual(plant.position, 0)
            document = result['document']
            self.assertEqual(len(document['nodes']), count)
            self.assertEqual(len(document['segments']), count-1)
            self.assertEqual(document['nodes'][0]['nominal_target_force_n'], 1)
            self.assertEqual(document['nodes'][-1]['nominal_target_force_n'], 30)
            self.assertEqual(len(select_firmware_calibration_document_nodes(document)['nodes']), count)
            for node in document['nodes']:
                self.assertAlmostEqual(node['k_n_per_signal_count'], .00025, places=8)
                self.assertEqual(node['sample_count'], 2)
            for entry in result['records']:
                self.assertEqual(entry['floating_count'], 4)
                self.assertGreaterEqual(entry['capture_seconds'], 1)
                self.assertGreaterEqual(entry['sample_count'], 3)
            self.assertEqual(sum(row.get('phase') == 'point' for row in phases), count*2)
            self.assertGreaterEqual(sum(move > 0 for move in plant.actions), count*2)

    def test_cancel_during_seek_capture_and_release_never_publishes_complete_table(self):
        for phase in ('press', 'capture', 'release', 'release_wait', 'rest'):
            with self.subTest(phase=phase):
                plant, result, _, _ = self.run_plan(cancel_phase=phase)
                self.assertFalse(result['ok'])
                self.assertTrue(result['cancelled'])
                self.assertTrue(result['stop_confirmed'])
                self.assertNotIn('document', result)
                self.assertGreater(plant.stop_count, 0)
                self.assertEqual(plant.position, 0)
                self.assertTrue(result['release_position_confirmed'])

    def test_rest_between_every_press_including_round_boundary_and_round_times(self):
        for count in (10, 20):
            plant, result, phases, _ = self.run_plan(count=count)
            self.assertTrue(result['ok'], result)
            rest_starts = [p for i, p in enumerate(phases) if p.get('phase') == 'rest'
                           and (i == 0 or phases[i-1].get('phase') != 'rest')]
            self.assertEqual(len(rest_starts), count*2-1)
            for start in rest_starts: self.assertAlmostEqual(start['rest_remaining_s'], 30)
            # 默认每个已卸载点至下一次实际下压至少30秒，首点不等待，末点不多等待。
            first_move = plant.action_times[0][0]
            self.assertLess(first_move-10, 30)
            for released_at in plant.checkpoint_times[:-1]:
                next_press = next(t for t, steps in plant.action_times if t > released_at and steps < 0)
                self.assertGreaterEqual(next_press-released_at, 30)
            self.assertEqual(plant.now, plant.checkpoint_times[-1])
            times = result['round_timings']
            self.assertEqual(len(times), 2)
            for index, timing in enumerate(times):
                start = 10 if index == 0 else plant.checkpoint_times[count-1]
                end = plant.checkpoint_times[(index+1)*count-1]
                self.assertAlmostEqual(timing['elapsed_s'], end-start)
                self.assertTrue(timing['completed'])
                self.assertEqual(timing['completed_points'], count)
            self.assertAlmostEqual(sum(t['elapsed_s'] for t in times), result['elapsed_s'])
            self.assertEqual(result['document']['metadata']['round_timings'], times)

    def test_stop_during_second_round_rest_preserves_first_round_and_partial_time(self):
        context = {}
        def stop_in_rest(plant, row):
            if row.get('phase') == 'point': context.update(row)
            if context.get('round') == 2 and row.get('phase') == 'rest' and row['rest_remaining_s'] <= 15:
                plant.cancelled = True
        _, result, _, _ = self.run_plan(progress_hook=stop_in_rest)
        self.assertFalse(result['ok'])
        self.assertTrue(result['cancelled'] and result['stop_confirmed'])
        self.assertEqual(len(result['records']), 10)
        self.assertTrue(result['round_timings'][0]['completed'])
        partial = result['round_timings'][1]
        self.assertFalse(partial['completed'])
        self.assertEqual(partial['completed_points'], 0)
        self.assertGreaterEqual(partial['elapsed_s'], 15)
        self.assertLess(partial['elapsed_s'], 16)
        self.assertNotIn('document', result)

    def test_feedback_loss_during_rest_stops_before_next_press(self):
        def lose_pressure(plant, row):
            if row.get('phase') == 'rest': plant.fault = 'stale'
            if row.get('phase') == 'retry_wait': plant.cancelled = True
        plant, result, _, _ = self.run_plan(progress_hook=lose_pressure)
        self.assertFalse(result['ok'])
        self.assertEqual(len(result['records']), 1)
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(all(t <= plant.checkpoint_times[0] for t, _ in plant.action_times))

    def test_configurable_rest_and_zero_preserve_release_checks_and_actual_timing(self):
        for seconds in (0, 7.5, 60):
            plant, result, _, _ = self.run_plan(rounds=1, rest_seconds=seconds)
            self.assertTrue(result['ok'], result)
            self.assertEqual(result['rest_seconds'], seconds)
            self.assertEqual(len(result['rest_periods']), 9)
            for period in result['rest_periods']:
                self.assertEqual(period['requested_s'], seconds)
                self.assertAlmostEqual(period['elapsed_s'], seconds, delta=.11)
                self.assertFalse(period['skipped'])
                self.assertTrue(period['completed'])
            for released_at in plant.checkpoint_times[:-1]:
                next_press = next(t for t, steps in plant.action_times if t > released_at and steps < 0)
                self.assertGreaterEqual(next_press-released_at, seconds+.19)
            self.assertEqual(result['document']['metadata']['rest_periods'], result['rest_periods'])
        for invalid in (-1, 3600.1, float('nan'), float('inf'), True, '30', None):
            with self.assertRaises(ValueError): validate_rest_seconds(invalid)

    def test_skip_only_current_rest_rejects_late_duplicate_click_and_preserves_next_rest(self):
        control = RestWaitControl()
        self.assertFalse(control.request_skip(None))
        skipped = []
        def skip(plant, row):
            if row.get('phase') != 'rest': return
            if not skipped and row['rest_remaining_s'] <= 25:
                skipped.append(row['rest_id'])
                self.assertTrue(control.request_skip(row['rest_id']))
                self.assertFalse(control.request_skip(row['rest_id']))
            elif skipped:
                self.assertFalse(control.request_skip(skipped[0]))
        plant, result, _, _ = self.run_plan(rounds=1, rest_control=control, progress_hook=skip)
        self.assertTrue(result['ok'], result)
        periods = result['rest_periods']
        self.assertTrue(periods[0]['skipped'])
        self.assertAlmostEqual(periods[0]['elapsed_s'], 5, delta=.11)
        for period in periods[1:]:
            self.assertFalse(period['skipped'])
            self.assertAlmostEqual(period['elapsed_s'], 30, delta=.11)
        first_next = next(t for t, steps in plant.action_times if t > plant.checkpoint_times[0] and steps < 0)
        self.assertLess(first_next-plant.checkpoint_times[0], 10)
        self.assertFalse(control.request_skip(skipped[0]))

    def test_skip_does_not_bypass_stop_or_invalid_unloaded_feedback(self):
        for fault in ('stale', 'loaded', 'cancel'):
            control = RestWaitControl()
            def skip_and_fault(plant, row):
                if row.get('phase') == 'retry_wait': plant.cancelled = True
                if row.get('phase') != 'rest': return
                control.request_skip(row['rest_id'])
                if fault == 'stale': plant.fault = 'stale'
                elif fault == 'loaded': plant.position = -100
                else: plant.cancelled = True
            plant, result, _, _ = self.run_plan(rest_control=control, progress_hook=skip_and_fault)
            self.assertFalse(result['ok'], result)
            self.assertEqual(len(result['records']), 1)
            self.assertNotIn('document', result)
            self.assertTrue(all(t <= plant.checkpoint_times[0] or steps > 0 for t, steps in plant.action_times))
            self.assertEqual(plant.position, 0)

    def test_transient_capture_failure_releases_then_retries_same_point(self):
        injected = []
        def glitch(plant, row):
            if row.get('phase') == 'capture' and not injected:
                injected.append(True)
                plant.fault = 'stale'
            elif row.get('phase') == 'recovery':
                plant.fault = None
        plant, result, _, _ = self.run_plan(rounds=1, progress_hook=glitch, rest_seconds=0)
        self.assertTrue(result['ok'], result)
        self.assertEqual(plant.position, 0)
        self.assertEqual(len(result['records']), 10)
        self.assertEqual(result['records'][0]['attempt'], 2)
        self.assertEqual(len(result['recoveries']), 1)
        self.assertTrue(result['recoveries'][0]['release_position_confirmed'])
        self.assertEqual(result['records'][0]['target_force_n'], 1)

    def test_persistent_capture_failure_waits_released_until_user_stops(self):
        def stop(plant, row):
            if row.get('phase') == 'retry_wait' and row['attempt'] == 4:
                self.assertEqual(plant.position, 0)
                plant.cancelled = True
        plant, result, _, _ = self.run_plan(fault='one_not_floating', rest_seconds=0, progress_hook=stop)
        self.assertFalse(result['ok'])
        self.assertEqual(len(result['point_failures']), 4)
        self.assertEqual(result['records'], [])
        self.assertEqual(plant.position, 0)
        self.assertTrue(result['release_position_confirmed'])
        self.assertTrue(result['cancelled'])
        self.assertNotIn('连续3次失败', result['message'])

    def test_motion_during_24n_seek_releases_then_retries_without_repeating_saved_points(self):
        original_window = feedback_tests.PressureFeedbackController._window
        def window(controller):
            plant = controller.motor
            if (not controller.releasing and controller.settings.target_n == 24 and plant.position < 0
                    and not getattr(plant, 'injected_seek_failure', False)):
                plant.injected_seek_failure = True
                plant.bad_stop_reads = 2
                raise feedback_tests.FeedbackControlError('采样期间电机仍在移动')
            return original_window(controller)
        def setup(plant):
            original_read = plant.read_feedback
            def feedback():
                sample = original_read()
                if getattr(plant, 'bad_stop_reads', 0):
                    plant.bad_stop_reads -= 1
                    sample.update(measured_speed_rpm=10, running_state=2)
                return sample
            plant.read_feedback = feedback
        with patch.object(feedback_tests.PressureFeedbackController, '_window', window):
            plant, result, _, _ = self.run_plan(count=20, rounds=1, rest_seconds=0, setup=setup)
        self.assertTrue(result['ok'], result)
        self.assertEqual(plant.position, 0)
        self.assertEqual(len(result['records']), 20)
        self.assertTrue(all(r['attempt'] == 1 for r in result['records'][:15]))
        self.assertEqual(result['records'][15]['target_force_n'], 24)
        self.assertEqual(result['records'][15]['attempt'], 2)
        self.assertIn('采样期间电机仍在移动', result['recoveries'][0]['reason'])
        self.assertTrue(result['recoveries'][0]['release_position_confirmed'])

    def test_release_recovery_waits_for_motor_feedback_and_does_not_require_pressure(self):
        for fault in ('lost_stop_ack', 'temporary_disconnect', 'missing_pressure', 'lost_move_ack'):
            plant = feedback_tests.Plant()
            plant.position = -610
            read, stop, move = plant.read_feedback, plant.stop, plant.move_relative_steps
            count = [0]
            def feedback():
                count[0] += 1
                if fault == 'temporary_disconnect' and count[0] < 4: raise RuntimeError('CAN短暂失联')
                sample = read()
                if count[0] < 3: sample.update(measured_speed_rpm=10, running_state=2)
                return sample
            def stop_ack():
                stop()
                if fault == 'lost_stop_ack': raise RuntimeError('停止ACK丢失')
            def move_ack(steps, **kwargs):
                move(steps, **kwargs)
                if fault == 'lost_move_ack': raise RuntimeError('位移已执行但ACK丢失')
            plant.read_feedback, plant.stop, plant.move_relative_steps = feedback, stop_ack, move_ack
            if fault == 'missing_pressure': plant.fault = 'stale'
            result = recover_release(plant, feedback_tests.PROFILE, plant.samples, FeedbackSettings(), 0,
                                     ClockCancel(plant), clock=lambda: plant.now)
            self.assertTrue(result['release_position_confirmed'], result)
            self.assertEqual(plant.position, 0)
            self.assertEqual(plant.actions, [610])
            if fault == 'missing_pressure': self.assertFalse(result['pressure_unloaded'])

    def test_recovery_can_be_emergency_stopped_without_any_blind_move(self):
        plant = feedback_tests.Plant()
        plant.position = -500
        plant.read_feedback = lambda: (_ for _ in ()).throw(RuntimeError('CAN持续断开'))
        result = recover_release(plant, feedback_tests.PROFILE, plant.samples, FeedbackSettings(), 0,
                                 ClockCancel(plant, lambda: plant.now >= 12), clock=lambda: plant.now)
        self.assertFalse(result['release_position_confirmed'])
        self.assertFalse(result['stop_confirmed'])
        self.assertEqual(plant.actions, [])

    def test_stale_frames_or_incomplete_floating_never_create_a_point(self):
        for fault in ('stale', 'one_not_floating', 'old_floating', 'invalid_speed', 'invalid_current'):
            with self.subTest(fault=fault):
                plant, result, _, _ = self.run_plan(fault=fault)
                self.assertFalse(result['ok'])
                self.assertEqual(result['records'], [])
                self.assertNotIn('document', result)
                if fault != 'one_not_floating': self.assertEqual(plant.actions, [])

    def test_stop_keeps_completed_points_and_final_checkpoint_stop_cannot_publish(self):
        for count in (1, 10):
            _, result, _, checkpoints = self.run_plan(rounds=1, cancel_after=count)
            self.assertFalse(result['ok'])
            self.assertTrue(result['cancelled'])
            self.assertNotIn('document', result)
            self.assertEqual(len(result['records']), count)
            self.assertEqual(len(checkpoints[-1]), count)

    def test_incomplete_duplicate_or_nonmonotonic_points_are_rejected(self):
        _, result, _, _ = self.run_plan()
        records = result['records']
        for variant in (records[:-1], records[:-1]+[records[0]],
                        [dict(r, signal_sum=1) for r in records]):
            with self.assertRaises(ValueError): build_high_force_mapping(variant, 10, 2)
        document = result['document']
        document['nodes'][0]['nominal_target_force_n'] = .1
        with self.assertRaises(ValueError): select_firmware_calibration_document_nodes(document)


class HighCalibrationUiTests(unittest.TestCase):
    page = feedback_tests.FeedbackSettingsUiTests.page

    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)

    def test_normal_stop_preserves_release_but_emergency_cancels_motor_worker(self):
        source = self.page()
        widget = HighForceCalibrationWidget(source.control, source, lambda: {}, None, output_dir=self.root)
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.timer.stop)
        widget._running = True
        with patch.object(source.control, 'request_external_stop') as urgent:
            widget.stop()
            self.assertTrue(widget._stop_requested.is_set())
            self.assertFalse(widget._emergency_requested.is_set())
            urgent.assert_not_called()
            self.assertTrue(widget.emergency_button.isEnabled())
            widget._progress(dict(phase='recovery', message='等待电机反馈，继续自动回位'))
            self.assertIn('继续自动回位', widget.status_label.text())
            widget.emergency_stop()
            self.assertTrue(widget._emergency_requested.is_set())
            urgent.assert_called_once()
        widget._running = False

    def test_quality_failure_never_replaces_saved_table_and_shows_errors(self):
        source = self.page()
        widget = HighForceCalibrationWidget(source.control, source, lambda: {}, None, output_dir=self.root)
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.timer.stop)
        result = dict(ok=False, message='独立验收未通过', records=[], release_position_confirmed=True,
            quality=dict(passes=[dict(validation=dict(checks=[dict(target_force_n=30, measured_n=29.4,
                predicted_n=31.7, error_n=2.3, allowed_error_n=.588, passed=False)]))]))
        with patch('high_force_calibration_widget.save_signal_pressure_mapping_document') as save:
            widget._finished(result)
            save.assert_not_called()
        self.assertIn('原K表保留', widget.stage_label.text())
        self.assertEqual(widget.quality_table.item(0, 3).text(), '独立验收未通过')

    def test_start_snapshots_settings_runs_in_shared_worker_and_publishes_k_table(self):
        source = self.page()
        source.control._release = 0
        profile = source.control.profile
        source._learned_map = LearnedPressureMap(profile.model, profile.namespace, profile.steps_per_turn,
                                                0, ((.1, 12.5), (1, 35), (30, 760)))
        source.inputs['force_limit_n'].setValue(35)
        source.inputs['max_step'].setValue(512)
        plant = feedback_tests.Plant()
        plant.now = 100
        source.samples = plant.samples
        plant.read_feedback()
        sensor = lambda: dict(sequence=round(plant.now*1000), age_seconds=0,
            state=[6 if plant.force() > 0 else 0]*8, signal=[0]*4+[round(plant.force()*1000)]*4)
        with patch('force_signal_pressure_lookup.signal_pressure_latest_mapping_path', return_value=self.root/'missing.json'):
            k_page = ForceSignalPressureCalibrationWidget()
        self.addCleanup(k_page.deleteLater)
        widget = HighForceCalibrationWidget(source.control, source, sensor, k_page, output_dir=self.root/'auto')
        self.assertEqual(widget.rest_seconds.value(), 30)
        self.assertEqual(widget.accuracy_percent.value(), 2)
        self.assertFalse(widget.skip_button.isEnabled())
        self.assertIn('24分30.0秒', widget.plan_label.text())
        widget.rest_seconds.setValue(45)
        widget.accuracy_percent.setValue(3)
        restored = HighForceCalibrationWidget(source.control, source, sensor, k_page, output_dir=self.root/'auto')
        self.addCleanup(restored.deleteLater)
        self.addCleanup(restored.shutdown)
        self.assertEqual(restored.rest_seconds.value(), 45)
        self.assertEqual(restored.accuracy_percent.value(), 3)
        widget.point_count.setCurrentIndex(1)
        self.assertIn('整数点位：1、3、4、6', widget.plan_label.text())
        self.assertNotIn('2.53', widget.plan_label.text())
        widget.point_count.setCurrentIndex(0)
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.shutdown)
        tasks = []
        def factory(*args, **kwargs):
            kwargs['clock'] = lambda: plant.now
            kwargs['recovery_cancel'] = ClockCancel(plant, kwargs['recovery_cancel'].is_set)
            args = list(args)
            args[4] = ClockCancel(plant, args[4].is_set)
            return HighForceCalibrationRunner(*args, **kwargs)
        with patch('time.monotonic', lambda: plant.now), \
                patch.object(source.control, 'is_can_connected', return_value=True), \
                patch.object(source.control, 'is_operation_busy', return_value=False), \
                patch.object(source.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
                patch('high_force_calibration_widget.HighForceCalibrationRunner', side_effect=factory), \
                patch('force_signal_pressure_calibration.signal_pressure_mapping_directory', return_value=self.root/'maps'):
            widget.round_count.setValue(1)
            widget.start()
            self.assertTrue(widget.is_running())
            self.assertEqual(len(tasks), 1)
            self.assertFalse(widget.point_count.isEnabled())
            self.assertFalse(widget.rest_seconds.isEnabled())
            self.assertFalse(widget.accuracy_percent.isEnabled())
            self.assertTrue(widget.stop_button.isEnabled())
            self.assertFalse(widget.skip_button.isEnabled())
            rest_control = widget._rest_control
            token = rest_control.begin()
            widget._progress(dict(phase='rest', rest_id=token, rest_remaining_s=45, force_n=0))
            self.assertTrue(widget.skip_button.isEnabled())
            widget.skip_button.click()
            self.assertTrue(rest_control.skip_requested(token))
            self.assertFalse(widget.skip_button.isEnabled())
            rest_control.end(token)
            next_token = rest_control.begin()
            widget.skip_rest()
            self.assertFalse(rest_control.skip_requested(next_token))
            rest_control.end(next_token)
            source.inputs['max_step'].setValue(1)
            tasks[0](plant, plant)
            self.assertFalse(widget.is_running())
            self.assertTrue(widget._result['ok'], widget._result)
            self.assertEqual(widget.table.rowCount(), 10)
            self.assertEqual(len(k_page._document['nodes']), 10)
            self.assertEqual(widget.quality_table.rowCount(), 10)
            self.assertEqual(widget.quality_table.item(0, 3).text(), '独立验收通过')
            document = json.loads((self.root/'maps/force_signal_pressure_map_latest.json').read_text(encoding='utf-8'))
            self.assertEqual(document['metadata']['calibration_profile'], 'auto_1_to_30_n')
            records = json.loads(widget._path.read_text(encoding='utf-8'))
            self.assertTrue(records['complete'])
            self.assertEqual(records['metadata']['motion_settings']['max_step'], 512)
            self.assertEqual(records['metadata']['rest_seconds'], 45)
            self.assertEqual(len(records['warmup_records']), 5)
            self.assertEqual(len(records['validation_records']), 10)
            self.assertEqual(len(records['rest_periods']), 24)
            self.assertTrue(all(r['requested_s'] == (10 if r['stage'] == 'warmup' else 45)
                                for r in records['rest_periods']))
            self.assertEqual(len(records['round_timings']), 1)
            self.assertGreater(records['round_timings'][0]['elapsed_s'], 405)
            self.assertFalse(widget.skip_button.isEnabled())
            self.assertTrue(widget.rest_seconds.isEnabled())
            self.assertEqual(widget.timing_table.rowCount(), 1)
            self.assertEqual(widget.timing_table.item(0, 2).text(), '已完成')
            self.assertIn('总耗时', widget.status_label.text())
            widget._show_round_timings([dict(round=1, elapsed_s=91.2, completed=False, completed_points=1)])
            self.assertEqual(widget.timing_table.item(0, 1).text(), '1分31.2秒')
            self.assertIn('未完成', widget.timing_table.item(0, 2).text())
            widget._progress(dict(phase='rest', rest_remaining_s=29.2, force_n=0))
            self.assertIn('剩余 30 秒', widget.status_label.text())


if __name__ == '__main__':
    unittest.main(verbosity=2)
