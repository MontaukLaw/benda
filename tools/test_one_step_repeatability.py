"""位置直达、未微调采样、闭环保持、异常释放与界面排队的离线验证。"""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import test_pressure_feedback as native
from test_high_force_calibration import ClockCancel
from one_step_learning import run_learning, default_settings, save_document, latest_path
from one_step_repeatability import run_trial, save_trial, repeatability_summary
from one_step_learning_widget import OneStepLearningWidget
from qt_compat import QtWidgets


class TrialTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        plant = native.Plant()
        cls.document = run_learning(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            default_settings(), 0, clock=lambda: plant.now)
        assert cls.document['complete']

    def trial(self, target=20, setup=None, progress=None, arrival=None, **kwargs):
        plant = native.Plant()
        plant.emergency = False
        if setup: setup(plant)
        rows = []
        def record(row):
            rows.append(row)
            if progress: progress(plant, row)
        result = run_trial(plant, native.PROFILE, plant.samples, plant,
            ClockCancel(plant, lambda: plant.emergency), replace(default_settings(), hold_s=1),
            0, self.document, target,
            mode=kwargs.pop('mode', 'position'),
            on_arrival=lambda row: arrival(plant, row) if arrival else None,
            progress=record, clock=lambda: plant.now, **kwargs)
        return plant, result, rows

    def test_exact_recorded_position_one_command_then_small_hold_moves_and_release(self):
        before = deepcopy(self.document)
        def setup(plant):
            force = plant.force
            plant.force = lambda: force()*1.04
        snapshots = []
        plant, result, _ = self.trial(setup=setup, fast_speed=80,
            arrival=lambda p, row: snapshots.append((list(p.actions), row)))
        self.assertTrue(result['ok'], result['message'])
        target_position = self.document['nodes'][20]['position_steps']
        self.assertEqual(snapshots[0][0], [target_position])
        self.assertEqual(plant.actions[0], target_position)
        self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 80)
        self.assertEqual(result['arrival']['actual_position_steps'], target_position)
        self.assertGreater(result['arrival']['error_vs_learning_n'], .7)
        self.assertLessEqual(abs(result['final_force_n']-20), .05)
        self.assertTrue(all(abs(step) <= 64 for step in plant.actions[1:-1]))
        self.assertGreater(result['correction_command_steps'], 0)
        self.assertEqual(plant.position, 0)
        self.assertEqual(self.document, before)
        frames = result['arrival']['samples']
        self.assertGreaterEqual(frames[-1]['stamp']-frames[0]['stamp'], .5-1e-9)

    def test_all_30_buttons_can_use_corresponding_learning_nodes(self):
        for target in range(1, 31):
            with self.subTest(target=target):
                plant, result, _ = self.trial(target)
                self.assertTrue(result['ok'], result['message'])
                self.assertEqual(plant.actions[0], self.document['nodes'][target]['position_steps'])
                self.assertEqual(result['arrival']['median_force_n'], self.document['nodes'][target]['measured_force_n'])

    def test_pressure_priority_all_targets_arrive_and_hold_without_position_observation(self):
        for target in range(1, 31):
            plant, result, rows = self.trial(target, mode='pressure')
            self.assertTrue(result['ok'], result['message'])
            self.assertEqual(result['arrival']['measurement_kind'], 'pressure_arrival')
            self.assertLessEqual(abs(result['arrival']['median_force_n']-target), .05)
            self.assertGreater(plant.actions[0], self.document['nodes'][target]['position_steps'])
            self.assertFalse(any(r.get('phase') == '未微调采样' for r in rows))
            self.assertEqual(plant.position, 0)

    def test_pressure_priority_reduces_dynamic_high_force_peak(self):
        def setup(p):
            force, move = p.force, p.move_relative_steps
            p.last_move, p.last_speed = -100, 0
            def motion(step, **kw):
                move(step, **kw)
                p.last_move = p.now
                p.last_speed = kw['speed_rpm'] if step < 0 else 0
            p.move_relative_steps = motion
            p.force = lambda: force()*(1+min(.15, p.last_speed*.005)*max(0, 1-(p.now-p.last_move)/.7))
        _, fixed, _ = self.trial(30, setup=setup, mode='position')
        plant, controlled, _ = self.trial(30, setup=setup, mode='pressure')
        self.assertTrue(controlled['ok'], controlled['message'])
        self.assertGreater(fixed['peak_n'], 33)
        self.assertLess(controlled['peak_n'], 31)
        self.assertLessEqual(abs(controlled['final_force_n']-30), .05)
        self.assertTrue(any(p['speed_rpm'] < 5 for p in plant.motion_parameters[1:-1]))

    def test_pressure_priority_predictive_brake_stops_stiffer_material_before_endpoint(self):
        def setup(p):
            read, stop, force = p.read_feedback, p.stop, p.force
            p.force = lambda: force()*1.4
            p.moving_to, p.rpm, p.last_tick = None, 0, p.now
            def motion(step, **kw):
                p.actions.append(step)
                p.motion_parameters.append(kw)
                p.moving_to, p.rpm, p.last_tick = p.position+step, kw['speed_rpm'], p.now
            def feedback():
                if p.moving_to is not None and p.now > p.last_tick:
                    distance = p.moving_to-p.position
                    count = max(1, round(p.rpm*3200/60*(p.now-p.last_tick)))
                    p.position += max(-count, min(count, distance))
                    p.last_tick = p.now
                    if p.position == p.moving_to: p.moving_to = None
                row = read()
                row.update(measured_speed_rpm=-p.rpm if p.moving_to is not None else 0,
                    in_position=p.moving_to is None, running_state=2 if p.moving_to is not None else 0)
                return row
            def brake():
                p.moving_to = None
                stop()
            p.move_relative_steps, p.read_feedback, p.stop = motion, feedback, brake
        plant, result, _ = self.trial(30, setup=setup, mode='pressure')
        self.assertTrue(result['ok'], result['message'])
        self.assertGreater(len(result['braking_events']), 0)
        self.assertLess(result['peak_n'], 31)
        self.assertLessEqual(abs(result['final_force_n']-30), .05)
        self.assertEqual(plant.position, 0)

    def test_pressure_priority_handles_softened_low_force_material(self):
        def setup(p):
            force = p.force
            p.force = lambda: force()*.53
        _, old, _ = self.trial(2, setup=setup, mode='position', seek_max_step=4)
        plant, result, _ = self.trial(2, setup=setup, mode='pressure')
        self.assertTrue(result['ok'], result['message'])
        self.assertLessEqual(abs(result['final_force_n']-2), .05)
        self.assertLess(result['stable_arrival_s'], old['stable_arrival_s'])
        self.assertEqual(plant.position, 0)

    def test_pressure_priority_cancel_loss_and_emergency_preserve_release_rules(self):
        for failure in ('cancel', 'stale', 'emergency'):
            def fail(p, row):
                if row.get('command_steps', 0) < 0:
                    if failure == 'cancel': p.cancelled = True
                    elif failure == 'stale': p.fault = 'stale'
                    else: p.emergency = True
            plant, result, _ = self.trial(20, mode='pressure', progress=fail)
            self.assertFalse(result['ok'])
            self.assertEqual(result['release_position_confirmed'], failure != 'emergency')
            if failure != 'emergency': self.assertEqual(plant.position, 0)

    def test_pressure_and_position_statistics_are_never_mixed(self):
        _, fixed, _ = self.trial(2, mode='position')
        _, controlled, _ = self.trial(2, mode='pressure')
        self.assertEqual(repeatability_summary([fixed, controlled], fixed)['count'], 1)
        self.assertEqual(repeatability_summary([fixed, controlled], controlled)['count'], 1)

    def test_continuous_motion_does_not_brake_at_old_pressure_threshold_and_stop_recovers(self):
        def setup(p):
            read, stop = p.read_feedback, p.stop
            p.moving_to = None
            def move(step, **kw):
                p.actions.append(step)
                p.motion_parameters.append(kw)
                p.moving_to = p.position+step
            def feedback():
                if p.moving_to is not None:
                    distance = p.moving_to-p.position
                    p.position += max(-40, min(40, distance))
                    if p.position == p.moving_to: p.moving_to = None
                row = read()
                row.update(measured_speed_rpm=30 if p.moving_to is not None else 0,
                           in_position=p.moving_to is None, running_state=2 if p.moving_to is not None else 0)
                return row
            def brake():
                p.moving_to = None
                stop()
            p.move_relative_steps, p.read_feedback, p.stop = move, feedback, brake
        plant, result, rows = self.trial(setup=setup)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(result['arrival']['actual_position_steps'], self.document['nodes'][20]['position_steps'])
        self.assertTrue(any(r.get('phase') == '位置直达' and r.get('force_n', 0) > 16 for r in rows))
        def cancel(p, row):
            if row.get('phase') == '位置直达' and row.get('command_steps') and p.position < -50:
                p.cancelled = True
        plant, result, _ = self.trial(setup=setup, progress=cancel)
        self.assertFalse(result['ok'])
        self.assertIsNone(result['arrival'])
        self.assertEqual(plant.position, 0)

    def test_starts_loaded_releases_and_presses_without_rest(self):
        plant, result, rows = self.trial(setup=lambda p: setattr(p, 'position', -60))
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(plant.actions[:2], [60, self.document['nodes'][20]['position_steps']])
        self.assertFalse(any(r.get('phase') == 'rest' for r in rows))
        self.assertEqual(result['rest_s'], 0)
        self.assertLess(result['elapsed_s'], 5)

    def test_low_force_large_shortfall_uses_seek_limit_then_returns_to_small_hold_steps(self):
        def setup(p):
            force = p.force
            p.force = lambda: force()*.53
        _, slow, _ = self.trial(2, setup=setup, seek_max_step=4)
        plant, fast, rows = self.trial(2, setup=setup, seek_max_step=64)
        self.assertTrue(fast['ok'], fast['message'])
        self.assertLess(fast['stable_arrival_s'], slow['stable_arrival_s'])
        self.assertLessEqual(abs(fast['final_force_n']-2), .05)
        self.assertTrue(any(abs(step) > 4 for step in plant.actions[1:-1]))
        hold_rows = [r for r in rows if r.get('hold_elapsed_s') is not None]
        self.assertTrue(all(abs(r.get('command_steps', 0)) <= 4 for r in hold_rows))

    def test_single_step_quantization_does_not_wait_three_seconds_for_visible_motion(self):
        def setup(p):
            move = p.move_relative_steps
            p.ignored = False
            def motion(step, **kw):
                if abs(step) == 1 and not p.ignored:
                    p.actions.append(step)
                    p.motion_parameters.append(kw)
                    p.ignored = True
                else:
                    move(step, **kw)
            p.move_relative_steps = motion
        plant, result, _ = self.trial(setup=setup)
        self.assertTrue(plant.ignored)
        self.assertTrue(result['ok'], result['message'])
        self.assertTrue(any('整数格' in e['reason'] for e in result['motion_diagnostics']))
        self.assertLess(result['elapsed_s'], 5)

    def test_observation_position_settling_restarts_window_instead_of_aborting(self):
        changed = []
        def settle(p, row):
            if row.get('phase') == '未微调采样' and not changed:
                p.position += 3
                changed.append(True)
        plant, result, _ = self.trial(progress=settle)
        self.assertTrue(result['ok'], result['message'])
        self.assertGreater(result['arrival']['observation_window_resets'], 0)
        self.assertEqual(plant.position, 0)

    def test_recorded_deceleration_overshoot_waits_for_true_stop_and_records_offset(self):
        # 复现19:37记录的终点附近反馈：仍在减速且in_position已置位，随后静止。
        def setup(p):
            move, read = p.move_relative_steps, p.read_feedback
            p.replay = []
            def motion(step, **kw):
                move(step, **kw)
                if len(p.actions) == 1:
                    target = p.position
                    p.replay = [(target+50, -20), (target+6, -11), (target-7, -3),
                                (target-7, -1), (target-12, 0), (target-12, 0), (target-11, 0)]
            def feedback():
                speed = 0
                if p.replay:
                    p.position, speed = p.replay.pop(0)
                row = read()
                row.update(measured_speed_rpm=speed, running_state=2 if speed else 0,
                           in_position=True)
                return row
            p.move_relative_steps, p.read_feedback = motion, feedback
        plant, result, _ = self.trial(setup=setup)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(result['arrival']['position_error_steps'], -11)
        self.assertEqual(result['arrival']['position_tolerance_steps'], 16)
        self.assertGreater(result['arrival']['position_arrival_s'], .4)
        self.assertGreater(result['correction_command_steps'], 0)
        self.assertEqual(plant.position, 0)

    def test_large_overshoot_or_wrong_direction_still_stops_and_releases(self):
        for backwards in (False, True):
            def setup(p):
                move = p.move_relative_steps
                def motion(step, **kw):
                    before = p.position
                    move(step, **kw)
                    if len(p.actions) == 1:
                        p.position = before+17 if backwards else p.position-17
                p.move_relative_steps = motion
            # 反向越过起点后先等待可信位置恢复；不让测试伪造向下释放。
            def recover(p, row):
                if backwards and row.get('phase') == 'recovery': p.position = 0
            plant, result, _ = self.trial(setup=setup, progress=recover)
            self.assertFalse(result['ok'])
            self.assertIsNone(result['arrival'])
            self.assertTrue(result['release_position_confirmed'])
            self.assertEqual(plant.position, 0)

    def test_ordinary_stop_during_direct_sample_and_hold_always_returns(self):
        for phase in ('位置直达', '未微调采样', '恒力保持'):
            def stop(p, row):
                if row.get('phase') == phase: p.cancelled = True
            plant, result, _ = self.trial(progress=stop)
            self.assertFalse(result['ok'], phase)
            self.assertTrue(result['cancelled'], phase)
            self.assertEqual(plant.position, 0, phase)
            self.assertTrue(result['release_position_confirmed'], phase)

    def test_emergency_after_arrival_never_auto_returns(self):
        plant, result, _ = self.trial(arrival=lambda p, _: setattr(p, 'emergency', True))
        self.assertFalse(result['ok'])
        self.assertFalse(result['release_position_confirmed'])
        self.assertLess(plant.position, 0)
        self.assertEqual(len(plant.actions), 1)

    def test_pressure_loss_after_motion_and_during_hold_still_returns(self):
        for phase in ('未微调采样', '恒力保持'):
            def lose(p, row):
                if row.get('phase') == phase: p.fault = 'stale'
            plant, result, _ = self.trial(progress=lose)
            self.assertFalse(result['ok'])
            self.assertEqual(plant.position, 0)
            self.assertTrue(result['release_position_confirmed'])

    def test_overpressure_during_direct_stops_then_releases(self):
        def setup(p):
            force = p.force
            p.force = lambda: force()*2
        plant, result, _ = self.trial(setup=setup)
        self.assertFalse(result['ok'])
        self.assertIn('保护上限', result['message'])
        self.assertIsNone(result['arrival'])
        self.assertIsNone(result['final_force_n'])
        self.assertEqual(plant.position, 0)

    def test_current_fault_no_return_until_fault_cleared(self):
        actions_at_fault = []
        def arrival(p, _):
            actions_at_fault.append(len(p.actions))
            p.current = 3500
        def recover(p, row):
            if p.current > 3000 and row.get('phase') == 'recovery' and row.get('recovery_attempt', 0) >= 2:
                self.assertEqual(len(p.actions), actions_at_fault[0])
                p.current = 200
        plant, result, _ = self.trial(arrival=arrival, progress=recover)
        self.assertFalse(result['ok'])
        self.assertEqual(plant.position, 0)

    def test_release_ack_loss_not_replayed_and_transient_link_waits_for_feedback(self):
        def setup(p):
            move, read = p.move_relative_steps, p.read_feedback
            p.fail_reads = 0
            def lost_ack(step, **kw):
                move(step, **kw)
                if step > 16:
                    p.fail_reads = 3
                    raise RuntimeError('release ACK lost')
            def delayed_read():
                if p.fail_reads:
                    p.fail_reads -= 1
                    raise RuntimeError('temporary link loss')
                return read()
            p.move_relative_steps, p.read_feedback = lost_ack, delayed_read
        plant, result, _ = self.trial(setup=setup)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(plant.position, 0)
        self.assertEqual(sum(step > 16 for step in plant.actions), 1)

    def test_invalid_table_origin_and_speed_rejected_without_movement(self):
        for origin, speed, doc in [(1, 30, self.document), (0, 401, self.document),
                                  (0, 30, dict(self.document, complete=False))]:
            p = native.Plant()
            with self.assertRaises(ValueError):
                run_trial(p, native.PROFILE, p.samples, p, ClockCancel(p), default_settings(),
                          origin, doc, 20, fast_speed=speed, clock=lambda: p.now)
            self.assertEqual(p.actions, [])

    def test_json_isolated_and_statistics_separate_conditions(self):
        _, result, _ = self.trial()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            save_document(root/'one_step_learning', self.document, native.PROFILE)
            original = latest_path(root/'one_step_learning', native.PROFILE).read_bytes()
            path = save_trial(root/'one_step_trials', result)
            self.assertEqual(json.loads(path.read_text(encoding='utf-8')), result)
            self.assertEqual(latest_path(root/'one_step_learning', native.PROFILE).read_bytes(), original)
        other = deepcopy(result)
        other['arrival']['median_force_n'] += 1
        summary = repeatability_summary([result, other, dict(result, fast_speed_rpm=60)], result)
        self.assertEqual(summary['count'], 2)
        self.assertAlmostEqual(summary['max_n']-summary['min_n'], 1)
        self.assertIsNotNone(summary['stdev_n'])


class TrialUiTests(unittest.TestCase):
    page = native.FeedbackSettingsUiTests.page

    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        TrialTests.setUpClass()

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.directory = self.root/'one_step_learning'
        self.source = self.page()
        document = deepcopy(TrialTests.document)
        document['namespace'] = self.source.control.profile.namespace
        document['profile'] = self.source.control.profile.metadata()
        save_document(self.directory, document, self.source.control.profile)
        self.plant = native.Plant()
        self.source.samples = self.plant.samples
        self.widget = OneStepLearningWidget(self.source.control, self.plant.samples, output_dir=self.directory)
        self.widget.control_mode.setCurrentIndex(self.widget.control_mode.findData('position'))
        self.addCleanup(self.widget.deleteLater)
        self.addCleanup(self.widget.shutdown)
        for method, kwargs in [('is_can_connected', dict(return_value=True)),
                               ('is_operation_busy', dict(return_value=False)),
                               ('latest_motor_feedback', dict(side_effect=self.plant.read_feedback))]:
            ctx = patch.object(self.source.control, method, **kwargs)
            ctx.start()
            self.addCleanup(ctx.stop)
        ctx = patch('time.monotonic', lambda: self.plant.now)
        ctx.start()
        self.addCleanup(ctx.stop)
        self.plant.read_feedback()
        self.widget._refresh()

    def test_history_needs_explicit_confirmation_and_zero_disables_all_30_buttons(self):
        w = self.widget
        self.assertEqual(len(w.pressure_buttons), 30)
        self.assertFalse(w.pressure_buttons[20].isEnabled())
        w.confirm_table()
        self.assertTrue(all(b.isEnabled() for b in w.pressure_buttons.values()))
        w._coordinates_invalidated()
        self.assertFalse(any(b.isEnabled() for b in w.pressure_buttons.values()))
        self.assertFalse(w.confirm_table_button.isEnabled())

    def test_worker_records_first_arrival_saves_trial_and_leaves_learning_unchanged(self):
        w, tasks, plots = self.widget, [], []
        w.confirm_table()
        original = deepcopy(w._document)
        self.assertNotIn('rest_s', w.trial_inputs)
        w.trial_inputs['hold_s'].setValue(1)
        w.show_motor_plot.connect(lambda: plots.append(True))
        def virtual(motor, profile, samples, stop, emergency, settings, origin, document, target, **kw):
            return run_trial(motor, profile, samples, ClockCancel(self.plant, stop.is_set),
                ClockCancel(self.plant, emergency.is_set), settings, origin, document, target,
                clock=lambda: self.plant.now, **kw)
        with patch.object(self.source.control, 'start_external_operation',
                          side_effect=lambda task, name: (tasks.append(task) or True, '')), \
             patch('one_step_learning_widget.run_trial', virtual):
            w.start_trial(20)
            self.assertTrue(w.is_running())
            tasks[0](self.plant, ClockCancel(self.plant))
        self.assertFalse(w.is_running())
        self.assertTrue(plots)
        self.assertEqual(w._document, original)
        self.assertEqual(w.trial_table.rowCount(), 1)
        self.assertIn('未微调压力', w.arrival_label.text())
        self.assertFalse(w.has_unsaved_result())
        self.assertEqual(len(list((self.directory.parent/'one_step_trials').glob('*.json'))), 1)

    def test_new_button_queues_release_stop_cancels_pending_and_busy_blocks(self):
        w = self.widget
        w.confirm_table()
        with patch.object(self.source.control, 'start_external_operation', return_value=(True, '')):
            w.start_trial(20)
            w.start_trial(29)
            self.assertEqual(w._pending_target, 29)
            self.assertTrue(w._stop.is_set())
            w.stop()
            self.assertIsNone(w._pending_target)
            w._running = w._trial_running = False
        with patch.object(self.source.control, 'is_operation_busy', return_value=True):
            w._refresh()
            self.assertFalse(any(b.isEnabled() for b in w.pressure_buttons.values()))

    def test_pressure_mode_worker_displays_controlled_result_and_saves_mode(self):
        w, tasks = self.widget, []
        w.control_mode.setCurrentIndex(w.control_mode.findData('pressure'))
        w.confirm_table()
        w.trial_inputs['hold_s'].setValue(1)
        def virtual(motor, profile, samples, stop, emergency, settings, origin, document, target, **kw):
            return run_trial(motor, profile, samples, ClockCancel(self.plant, stop.is_set),
                ClockCancel(self.plant, emergency.is_set), settings, origin, document, target,
                clock=lambda: self.plant.now, **kw)
        with patch.object(self.source.control, 'start_external_operation',
                          side_effect=lambda task, name: (tasks.append(task) or True, '')), \
             patch('one_step_learning_widget.run_trial', virtual):
            w.start_trial(2)
            tasks[0](self.plant, ClockCancel(self.plant))
        self.assertIn('压力优先', w.arrival_label.text())
        self.assertEqual(w.trial_table.item(0, 7).text(), '压力优先')
        self.assertFalse(w.trial_inputs['observation_s'].isEnabled())
        path = next((self.directory.parent/'one_step_trials').glob('*.json'))
        result = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(result['control_mode'], 'pressure')
        self.assertEqual(result['arrival']['measurement_kind'], 'pressure_arrival')

    def test_save_failure_keeps_result_and_blocks_next_trial_until_retry(self):
        w = self.widget
        w.confirm_table()
        result = TrialTests().trial()[1]
        with patch('one_step_learning_widget.save_trial', side_effect=PermissionError('locked')):
            w._finished(result)
        self.assertTrue(w.has_unsaved_result())
        self.assertFalse(w.pressure_buttons[20].isEnabled())
        w.save_trials()
        self.assertFalse(w.has_unsaved_result())
        self.assertTrue(w.pressure_buttons[20].isEnabled())

    def test_pending_target_waits_for_worker_idle_and_only_timer_starts_it(self):
        w, calls = self.widget, []
        w.confirm_table()
        w._pending_target = 29
        with patch.object(self.source.control, 'start_external_operation',
                          side_effect=lambda task, name: (calls.append(name) or True, '')):
            with patch.object(self.source.control, 'is_operation_busy', return_value=True):
                w._tick()
                self.assertEqual(calls, [])
            w._refresh()
            self.assertEqual(calls, [])
            self.assertTrue(w.is_running())
            w._tick()
            self.assertEqual(len(calls), 1)
            self.assertIn('29 N', calls[0])
            self.assertIsNone(w._pending_target)
            w._running = w._trial_running = False


if __name__ == '__main__':
    unittest.main(verbosity=2)
