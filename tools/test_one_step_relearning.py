"""独立重压学习的完整流程、错误回位、过冲判定和文件隔离验证。"""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

import test_pressure_feedback as native
from test_high_force_calibration import ClockCancel
import test_one_step_learning as seed_tests
import test_one_step_repeatability as trial_tests
from one_step_learning import default_settings
from one_step_relearning import (run_relearning, CycleSettings, validate_document,
    save_document, load_document, latest_path, contact_summary, ContactController,
    CyclePositionController, next_candidate)
from pressure_feedback_control import FeedbackControlError
from one_step_repeatability import run_trial
from one_step_learning_widget import OneStepLearningWidget
from high_force_calibration import recover_release


class CycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.seed, _ = seed_tests.LearningTests().learn()

    def learn(self, setup=None, on_attempt=None, progress=None, checkpoint=None, cycles=CycleSettings()):
        plant = native.Plant()
        plant.emergency = False
        if setup: setup(plant)
        rows, events, times = [], [], []
        def attempt(row):
            rows.append(row)
            times.append(plant.now)
            if on_attempt: on_attempt(plant, row)
        def record(row):
            events.append(dict(row, virtual_s=plant.now))
            if progress: progress(plant, row)
        result = run_relearning(plant, native.PROFILE, plant.samples, plant,
            ClockCancel(plant, lambda: plant.emergency), default_settings(), 0, self.seed,
            cycles=cycles, on_attempt=attempt, progress=record, checkpoint=checkpoint or (lambda doc: None),
            clock=lambda: plant.now)
        return plant, result, rows, events

    def test_complete_10_contact_and_30_independent_points(self):
        plant, doc, rows, events = self.learn()
        self.assertTrue(doc['complete'], doc['message'])
        validate_document(doc, native.PROFILE)
        self.assertEqual(len(doc['contacts']), 10)
        self.assertEqual(len(doc['points']), 30)
        self.assertEqual([n['target_force_n'] for n in doc['nodes']], list(range(31)))
        self.assertEqual(plant.position, 0)
        self.assertTrue(all(len(p['attempts']) >= 5 for p in doc['points']))
        for point in doc['points']:
            last = point['attempts'][-3:]
            self.assertTrue(all(r['ok'] for r in last))
            self.assertEqual(len({r['candidate_position_steps'] for r in last}), 1)
        # 初次直接开始；后续每次都释放、足额休息5秒，不在承压位置累积下一次。
        first_rest = next(e for e in events if e.get('phase') == 'rest')
        self.assertEqual(first_rest['stage'], 'contact')
        self.assertEqual(first_rest['attempt'], 2)
        # 此模型接触点很近，没有安全快走段；每次受力仅一条下压命令，随后一条释放。
        loaded_moves = [d for d in plant.actions if d < -CycleSettings().contact_step]
        self.assertEqual(len(loaded_moves), sum(len(p['attempts']) for p in doc['points']))
        self.assertTrue(all(r['trace'] for p in doc['points'] for r in p['attempts']))

    def test_contact_median_and_fast_boundary_are_distinct(self):
        rows = [dict(position_steps=-400-i, last_zero_position_steps=-398-i, baseline_n=0) for i in range(10)]
        contact = contact_summary(rows, 0, native.PROFILE, default_settings(), CycleSettings(fast_speed=400))
        self.assertEqual(contact['median_position_steps'], -404.5)
        self.assertGreater(contact['fast_position_steps'], -398)
        self.assertLess(contact['effective_fast_speed_rpm'], 400)
        self.assertEqual(-contact['fast_position_steps']+contact['reserve_steps'], 398)

    def test_all_ten_contacts_keep_configured_step_after_rejected_noise(self):
        def setup(plant):
            force, move = plant.force, plant.move_relative_steps
            plant.spike_until = 0
            def wrapped(delta, **kwargs):
                first_step = plant.position == 0 and delta < 0
                move(delta, **kwargs)
                if first_step:
                    plant.spike_until = plant.now+.6
            plant.move_relative_steps = wrapped
            plant.force = lambda: force()+(.01 if plant.now < plant.spike_until else 0)
        def finish(plant, row):
            if row['stage'] == 'contact' and row['attempt'] == 10 and row['ok']:
                plant.cancelled = True
        plant, doc, rows, _ = self.learn(setup=setup, on_attempt=finish,
            cycles=CycleSettings(contact_step=4))
        self.assertEqual(len(doc['contacts']), 10)
        self.assertTrue(all(row['diagnostics']['rejected_candidates'] for row in rows))
        self.assertEqual([delta for delta in plant.actions if delta < 0], [-4]*30)
        self.assertEqual(plant.position, 0)
        self.assertTrue(doc['release_position_confirmed'])

    def test_real_fast_segment_and_all_trials_start_at_release(self):
        origin_offset = 350
        seed = deepcopy(self.seed)
        for n in seed['nodes'][1:]:
            n['position_steps'] -= origin_offset
            n['travel_steps'] += origin_offset
        original = self.seed
        self.seed = seed
        starts = []
        def setup(plant):
            plant.force = lambda: max(0, (-plant.position-360)*.04)
            move = plant.move_relative_steps
            def wrapped(delta, **kwargs):
                starts.append((plant.position, delta, kwargs['speed_rpm'], plant.now))
                move(delta, **kwargs)
            plant.move_relative_steps = wrapped
        try:
            plant, doc, _, _ = self.learn(setup=setup)
        finally:
            self.seed = original
        self.assertTrue(doc['complete'], doc['message'])
        fast = [r for r in starts if r[1] == doc['contact']['fast_position_steps'] and r[0] == 0]
        self.assertEqual(len(fast), sum(len(p['attempts']) for p in doc['points']))
        self.assertTrue(all(r[2] == doc['contact']['effective_fast_speed_rpm'] for r in fast))
        self.assertTrue(all(b[3]-a[3] >= 5 for a,b in zip(fast, fast[1:])))

    def test_stop_in_contact_loaded_observation_and_rest_releases(self):
        for stage, phase in [('contact', None), ('pressure', '位置直达'),
                             ('pressure', '未微调采样'), ('pressure', 'rest')]:
            def cancel(plant, row):
                if row.get('stage') == stage and (phase is None or row.get('phase') == phase):
                    if phase == 'rest' or plant.position < 0:
                        plant.cancelled = True
            plant, doc, _, _ = self.learn(progress=cancel)
            self.assertFalse(doc['complete'])
            self.assertTrue(doc['release_position_confirmed'])
            self.assertEqual(plant.position, 0)

    def test_emergency_does_not_return(self):
        def urgent(plant, row):
            if row['stage'] == 'pressure': plant.emergency = True
        plant, doc, _, _ = self.learn(on_attempt=urgent)
        self.assertFalse(doc['complete'])
        self.assertTrue(doc['emergency_stopped'])
        self.assertLess(plant.position, 0)

    def test_stiffer_material_retries_shorter_without_within_trial_microsteps(self):
        def setup(plant): plant.force = lambda: max(0, (-plant.position-10)*.056)
        plant, doc, _, _ = self.learn(setup=setup)
        self.assertTrue(doc['complete'], doc['message'])
        attempts = doc['points'][0]['attempts']
        self.assertEqual(attempts[0]['guard_reason'], 'overshoot')
        self.assertGreater(attempts[1]['candidate_position_steps'], attempts[0]['candidate_position_steps'])
        self.assertFalse(attempts[0]['ok'])
        self.assertTrue(all(len(p['attempts']) <= 20 for p in doc['points']))

    def test_never_converging_point_stops_after_20_and_releases(self):
        def flip(plant, row):
            if row['stage'] == 'pressure':
                plant.drift = (lambda: .8) if row['attempt'] % 2 else (lambda: 0)
        plant, doc, _, _ = self.learn(on_attempt=flip)
        self.assertFalse(doc['complete'])
        self.assertEqual(len(doc['points'][0]['attempts']), 20)
        self.assertEqual(plant.position, 0)

    def test_checkpoint_failure_does_not_abandon_loaded_motor(self):
        def denied(doc): raise PermissionError('test file busy')
        plant, doc, _, _ = self.learn(checkpoint=denied)
        self.assertTrue(doc['complete'], doc['message'])
        self.assertTrue(doc['checkpoint_errors'])
        self.assertEqual(plant.position, 0)

    def test_pressure_loss_in_motion_or_capture_releases_before_retry(self):
        for phase in ('位置直达', '未微调采样'):
            faulted = []
            def progress(plant, row):
                if row.get('stage') == 'pressure' and row.get('phase') == phase and plant.position < -20 and not faulted:
                    plant.fault = 'stale'
                    faulted.append(True)
                if row.get('phase') == 'return' and faulted:
                    # 保持压力失联直到真实回到起点，随后用户停止；回位不得依赖压力。
                    plant.cancelled = True
            plant, doc, _, _ = self.learn(progress=progress)
            self.assertTrue(faulted)
            self.assertFalse(doc['complete'])
            self.assertTrue(doc['release_position_confirmed'])
            self.assertEqual(plant.position, 0)

    def test_contact_sized_stalled_return_never_counts_as_release(self):
        plant = native.Plant()
        plant.position, plant.fault = -11, 'stalled'
        recoveries = []
        def progress(row):
            if row.get('phase') == 'recovery' and row.get('recovery_attempt', 0) >= 2:
                recoveries.append(plant.position)
                plant.fault = None
        result = recover_release(plant, native.PROFILE, plant.samples, default_settings(), 0,
            ClockCancel(plant), progress=progress, clock=lambda: plant.now, return_even_within_tolerance=True)
        self.assertTrue(recoveries)
        self.assertEqual(recoveries[0], -11)
        self.assertTrue(result['release_position_confirmed'])
        self.assertEqual(plant.position, 0)

    def test_total_force_limit_releases_and_does_not_retry_dangerous_point(self):
        def progress(plant, row):
            if row.get('stage') == 'pressure' and row.get('phase') == '位置直达' and plant.position < -20:
                plant.force_override = 40
        plant, doc, _, _ = self.learn(progress=progress)
        self.assertFalse(doc['complete'])
        self.assertEqual(len(doc['points'][0]['attempts']), 1)
        self.assertEqual(plant.position, 0)
        self.assertIn('总保护压力', doc['message'])

    def test_persistent_zero_drift_is_rejected_after_release_then_same_contact_retried(self):
        def setup(plant):
            force, move = plant.force, plant.move_relative_steps
            plant.offset = 0
            def wrapped(delta, **kw):
                move(delta, **kw)
                if delta < 0: plant.offset = .01
            plant.move_relative_steps = wrapped
            plant.force = lambda: force()+plant.offset
        def finish(plant, row):
            if row['ok']: plant.cancelled = True
        plant, doc, rows, _ = self.learn(setup=setup, on_attempt=finish)
        self.assertEqual(plant.position, 0)
        self.assertFalse(rows[0]['ok'])
        self.assertTrue(rows[0]['retrying'])
        self.assertIn('底数漂移', rows[0]['message'])
        self.assertEqual([r['attempt'] for r in rows], [1, 1])
        self.assertEqual(len(doc['contacts']), 1)
        self.assertEqual(doc['contacts'][0]['baseline_n'], .01)
        self.assertLess(doc['contacts'][0]['position_steps'], rows[0]['position_steps'])

    def test_pressure_link_recovers_without_ending_or_losing_contact_number(self):
        def setup(plant): plant.fault = 'stale'
        def progress(plant, row):
            if row.get('phase') == 'contact_retry': plant.fault = None
        def finish(plant, row):
            if row['ok']: plant.cancelled = True
        plant, doc, rows, _ = self.learn(setup=setup, progress=progress, on_attempt=finish)
        self.assertEqual([r['attempt'] for r in rows], [1, 1])
        self.assertFalse(rows[0]['ok'])
        self.assertTrue(rows[1]['ok'])
        self.assertEqual(plant.position, 0)

    def test_contact_rest_motor_read_error_recovers_and_retries_same_number(self):
        injected = []
        def setup(plant):
            read = plant.read_feedback
            plant.fail_next_read = False
            def intermittent():
                if plant.fail_next_read:
                    plant.fail_next_read = False
                    raise RuntimeError('temporary motor read error')
                return read()
            plant.read_feedback = intermittent
        def progress(plant, row):
            if row.get('phase') == 'rest' and row.get('attempt') == 2 and not injected:
                injected.append(True)
                plant.fail_next_read = True
        def finish(plant, row):
            if row['attempt'] == 2 and row['ok']: plant.cancelled = True
        plant, doc, rows, _ = self.learn(setup=setup, progress=progress, on_attempt=finish)
        self.assertEqual([r['attempt'] for r in rows], [1, 2, 2])
        self.assertTrue(rows[1]['retrying'])
        self.assertTrue(rows[2]['ok'])
        self.assertEqual(len(doc['contacts']), 2)
        self.assertEqual(plant.position, 0)

    def test_unstable_baseline_waits_at_release_and_remains_cancellable(self):
        def setup(plant): plant.force = lambda: .03*(round(plant.now*20) % 2)
        def progress(plant, row):
            if row.get('phase') == 'contact_baseline_wait' and plant.now > 12:
                plant.cancelled = True
        plant, doc, rows, events = self.learn(setup=setup, progress=progress)
        self.assertTrue(doc['cancelled'])
        self.assertEqual(plant.actions, [])
        self.assertEqual(len(doc['contacts']), 0)
        self.assertTrue(any(r.get('phase') == 'contact_baseline_wait' for r in events))

    def test_peak_exceeding_tolerance_is_never_counted_even_if_it_relaxes(self):
        def setup(plant):
            force, move = plant.force, plant.move_relative_steps
            plant.last_press = -100
            def wrapped(delta, **kwargs):
                move(delta, **kwargs)
                if delta < -1: plant.last_press = plant.now
            plant.move_relative_steps = wrapped
            plant.force = lambda: force()*(1+.20*max(0, 1-(plant.now-plant.last_press)/.5))
        plant, doc, _, _ = self.learn(setup=setup)
        self.assertFalse(doc['complete'])
        self.assertEqual(plant.position, 0)
        attempts = doc['points'][0]['attempts']
        self.assertTrue(any(r['guard_reason'] == 'overshoot' for r in attempts))
        self.assertFalse(any(r['ok'] and r['peak_n'] > 1+r['tolerance_n'] for r in attempts))

    def test_files_separate_and_partial_does_not_replace_complete(self):
        _, doc, _, _ = self.learn()
        with tempfile.TemporaryDirectory() as directory:
            seed = Path(directory)/'one_step_learning_latest_zdt_x42s_1_3200.json'
            seed.write_text('seed unchanged', encoding='utf-8')
            save_document(directory, doc, native.PROFILE)
            self.assertEqual(load_document(directory, native.PROFILE), doc)
            before = latest_path(directory, native.PROFILE).read_bytes()
            partial = deepcopy(doc)
            partial['complete'] = False
            partial['points'] = partial['points'][:2]
            save_document(directory, partial, native.PROFILE)
            self.assertEqual(latest_path(directory, native.PROFILE).read_bytes(), before)
            self.assertEqual(seed.read_text(encoding='utf-8'), 'seed unchanged')
        bad = deepcopy(doc)
        bad['points'][0]['attempts'][-1]['candidate_position_steps'] -= 1
        with self.assertRaises(ValueError): validate_document(bad, native.PROFILE)
        for value in (float('nan'), .03):
            bad = deepcopy(doc)
            bad['contacts'][0]['baseline_max_n'] = value
            with self.assertRaises(ValueError): validate_document(bad, native.PROFILE)

    def test_learned_table_uses_same_trajectory_in_button_trial(self):
        _, doc, _, _ = self.learn()
        doc['nodes'][20]['medium_speed_rpm'] = 2
        for row in doc['points'][19]['attempts'][-3:]: row['medium_speed_rpm'] = 2
        plant = native.Plant()
        result = run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            default_settings(), 0, doc, 20, mode='cycles', clock=lambda: plant.now)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(result['arrival']['recorded_position_steps'], doc['nodes'][20]['position_steps'])
        self.assertEqual(result['control_mode'], 'cycles')
        self.assertEqual(result['medium_speed_rpm'], 2)
        self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 2)
        self.assertEqual(plant.position, 0)
        wrong_speed = deepcopy(doc)
        wrong_speed['points'][19]['attempts'][-1]['medium_speed_rpm'] = 3
        with self.assertRaises(ValueError): validate_document(wrong_speed, native.PROFILE)
        with self.assertRaises(ValueError):
            run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
                replace(default_settings(), acceleration=101), 0, doc, 20, mode='cycles', clock=lambda: plant.now)

    def test_contact_pressure_loss_in_search_confirmation_and_release_returns(self):
        for phase in ('搜索接触', '接触候选', 'return'):
            injected = []
            def progress(plant, row):
                if (row.get('stage') == 'contact' and phase in row.get('phase', '')
                        and plant.position < 0 and not injected):
                    injected.append(True)
                    plant.fault = 'stale'
                    # 直接废弃旧帧，保证失联实际发生在指定阶段。
                    plant.samples.read = lambda *a, **kw: (_ for _ in ()).throw(FeedbackControlError('压力断流'))
                if row.get('phase') == 'return' and injected:
                    plant.cancelled = True
            plant, doc, _, _ = self.learn(progress=progress)
            self.assertTrue(injected, phase)
            self.assertFalse(doc['complete'], phase)
            self.assertTrue(doc['release_position_confirmed'], phase)
            self.assertEqual(plant.position, 0, phase)

    def test_large_contact_step_pressure_jump_releases_without_retry(self):
        plant, doc, rows, _ = self.learn(cycles=CycleSettings(contact_step=16))
        self.assertFalse(doc['complete'])
        self.assertEqual(len(rows), 1)
        self.assertFalse(rows[0]['retrying'])
        self.assertIn('减小接触搜索步幅', rows[0]['message'])
        self.assertEqual(plant.position, 0)


class CycleUiTests(unittest.TestCase):
    page = native.FeedbackSettingsUiTests.page
    setUpClass = classmethod(trial_tests.TrialUiTests.setUpClass.__func__)
    setUp = trial_tests.TrialUiTests.setUp

    def test_worker_progress_and_separate_checkpoint_then_reload(self):
        w, tasks = self.widget, []
        w.confirm_table()
        w.cycle_inputs['contact_step'].setValue(3)
        w.cycle_inputs['contact_settle_s'].setValue(.15)
        before = (self.directory/f'one_step_learning_latest_{w.control.profile.namespace}.json').read_bytes()
        def virtual(motor, profile, samples, stop, emergency, settings, origin, seed, **kwargs):
            # 此用例验证历史重压表兼容；新连续轨迹另用动态电机模拟验证。
            self.assertTrue(kwargs['cycles'].trajectory_learning)
            kwargs['cycles'] = replace(kwargs['cycles'], trajectory_learning=False, near_contact_release=False,
                                      min_trials=5, consecutive=3, speed_step_rpm=1.0)
            return run_relearning(motor, profile, samples, ClockCancel(self.plant, stop.is_set),
                ClockCancel(self.plant, emergency.is_set), settings, origin, seed,
                clock=lambda: self.plant.now, **kwargs)
        with patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
             patch('one_step_learning_widget.cycles_learning.run_relearning', virtual):
            w.start_cycles()
            self.assertTrue(w.is_running())
            self.assertFalse(w.learn_button.isEnabled())
            self.assertFalse(w.cycle_inputs['contact_step'].isEnabled())
            tasks[0](self.plant, ClockCancel(self.plant))
        self.assertFalse(w.is_running())
        self.assertTrue(w._document['complete'], w.status_label.text())
        self.assertEqual(w.control_mode.currentData(), 'cycles')
        self.assertGreaterEqual(w.attempt_table.rowCount(), 160)
        self.assertIn('快速段终点', w.contact_label.text())
        self.assertFalse(w.has_unsaved_result(), w.path_label.text())
        self.assertEqual((self.directory/f'one_step_learning_latest_{w.control.profile.namespace}.json').read_bytes(), before)
        restored = OneStepLearningWidget(w.control, self.plant.samples, output_dir=self.directory)
        self.addCleanup(restored.deleteLater)
        self.addCleanup(restored.shutdown)
        self.assertEqual(restored.control_mode.currentData(), 'cycles')
        self.assertFalse(restored._table_confirmed)
        self.assertEqual(restored._document, w._document)
        self.assertEqual(restored._seed_document, w._seed_document)
        self.assertFalse(restored.pressure_buttons[30].isEnabled())
        self.assertEqual(restored.cycle_inputs['contact_step'].value(), 3)
        self.assertEqual(restored.cycle_inputs['contact_settle_s'].value(), .15)
        # 历史完整表没有新增字段时仍可加载，使用新搜索默认值。
        legacy = deepcopy(w._document)
        for key in ('contact_step', 'contact_settle_s'):
            legacy['cycle_settings'].pop(key)
        save_document(self.directory.parent/'one_step_cycles', legacy, w.control.profile)
        older = OneStepLearningWidget(w.control, self.plant.samples, output_dir=self.directory)
        self.addCleanup(older.deleteLater)
        self.addCleanup(older.shutdown)
        self.assertEqual(older._document, legacy)
        self.assertEqual(older.cycle_inputs['contact_step'].value(), 2)
        self.assertEqual(older.cycle_inputs['contact_settle_s'].value(), .1)

    def test_unknown_origin_busy_and_coordinate_reset_block_new_learning(self):
        w = self.widget
        self.assertFalse(w.cycle_button.isEnabled())
        w.confirm_table()
        self.assertTrue(w.cycle_button.isEnabled())
        with patch.object(w.control, 'is_operation_busy', return_value=True):
            w._refresh()
            self.assertFalse(w.cycle_button.isEnabled())
        w._coordinates_invalidated()
        self.assertFalse(w.cycle_button.isEnabled())

    def test_failed_contact_displays_reason_and_retry_without_none_position(self):
        w = self.widget
        w._show_attempt(dict(stage='contact', target_n=0, attempt=3, contact_attempt=4,
            ok=False, retrying=True, message='空载底数尚未稳定，已释放并等待重试'))
        self.assertNotIn('None', w.cycle_live_label.text())
        self.assertIn('未取得接触位置', w.cycle_live_label.text())
        self.assertIn('底数尚未稳定', w.cycle_live_label.text())
        self.assertEqual(w.attempt_table.item(0, 8).text(), '等待重试')
        w._cycle_running = True
        w._progress(dict(stage='contact', phase='contact_baseline_wait', message='在释放点等待，未下压'))
        w._progress(dict(stage='contact', phase='1 step搜索接触', message='接触第3/10次：正在小步下压'))
        self.assertNotIn('未下压', w.cycle_live_label.text())
        self.assertIn('正在小步下压', w.cycle_live_label.text())
        w._cycle_running = False


class ContactDetectionTests(unittest.TestCase):
    def run_contact(self, plant, cycles=CycleSettings()):
        controller = ContactController(plant, native.PROFILE, plant.samples, plant,
            replace(default_settings(), target_n=.15, tolerance_n=.005, acquire_timeout_s=600),
            0, cycle_settings=cycles, clock=lambda: plant.now)
        result = controller.run()
        self.assertTrue(result['ok'], result['message'])
        return controller

    def test_configured_step_reduces_long_search_time_and_wait_is_independent(self):
        times = []
        for step in (1, 2, 8):
            plant = native.Plant()
            plant.force = lambda: max(0, (-plant.position-400)*.005)
            controller = self.run_contact(plant, CycleSettings(contact_step=step))
            times.append(plant.now-10)
            self.assertEqual(plant.actions[0], -step)
            self.assertGreaterEqual(controller.contact['position_steps'], -408)
            self.assertLess(controller.contact['position_steps'], -400)
        self.assertLess(times[1], times[0]*.6)
        self.assertLess(times[2], times[0]*.2)
        plant = native.Plant()
        controller = self.run_contact(plant, CycleSettings(contact_settle_s=.35))
        self.assertEqual(controller.settings.settle_s, .35)
        self.assertEqual(default_settings().settle_s, .1)

    def test_search_never_commands_past_maximum_travel(self):
        plant = native.Plant()
        plant.force = lambda: 0
        controller = ContactController(plant, native.PROFILE, plant.samples, plant,
            replace(default_settings(), target_n=.15, tolerance_n=.005, max_travel=5), 0,
            cycle_settings=CycleSettings(contact_step=4), clock=lambda: plant.now)
        self.assertFalse(controller.run()['ok'])
        self.assertTrue(controller.contact_fatal)
        self.assertEqual(plant.actions, [-4, -1])

    def test_contact_setting_limits(self):
        for key, values in [('contact_step', (0, 17, 1.5, True)),
                            ('contact_settle_s', (.09, 2.01, float('nan'), float('inf')))]:
            for value in values:
                with self.assertRaises(ValueError):
                    replace(CycleSettings(), **{key: value}).validate(native.PROFILE)

    def test_zero_point_zero_one_quantization_does_not_abort_or_count_as_contact(self):
        plant = native.Plant()
        force = plant.force
        plant.force = lambda: force()+.01*(round(plant.now*20) % 2)
        controller = self.run_contact(plant)
        self.assertAlmostEqual(controller.contact['baseline_max_n'], .01)
        self.assertAlmostEqual(controller.contact['threshold_n'], .02)
        self.assertLessEqual(controller.contact['position_steps'], -11)

        frames = controller.contact['confirmation_samples']
        self.assertGreaterEqual(frames[-1]['stamp']-frames[0]['stamp'], .6-1e-9)

    def test_temporarily_unstable_baseline_automatically_continues_without_motion_while_waiting(self):
        plant = native.Plant()
        force, move = plant.force, plant.move_relative_steps
        first_moves = []
        plant.force = lambda: (.02*(round(plant.now*20) % 2) if plant.now < 12.5 else force())
        def wrapped(delta, **kw):
            first_moves.append(plant.now)
            move(delta, **kw)
        plant.move_relative_steps = wrapped
        self.run_contact(plant)
        self.assertGreaterEqual(first_moves[0], 13.4)

    def test_brief_zero_point_zero_one_plateau_does_not_pass_stationary_confirmation(self):
        plant = native.Plant()
        force, move = plant.force, plant.move_relative_steps
        plant.spike_until = 0
        def wrapped(delta, **kw):
            move(delta, **kw)
            if plant.position == -CycleSettings().contact_step: plant.spike_until = plant.now+.6
        plant.move_relative_steps = wrapped
        plant.force = lambda: force()+(.01 if plant.now < plant.spike_until else 0)
        controller = self.run_contact(plant)
        self.assertTrue(controller.contact_diagnostics['rejected_candidates'])
        self.assertTrue(all(delta == -2 for delta in plant.actions))
        self.assertLessEqual(controller.contact['position_steps'], -11)


class PressurePredictionRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recording = json.loads((Path(__file__).with_name('fixtures')/'one_step_1n_20260921.json').read_text(encoding='utf-8'))

    def controller(self, position=-447):
        plant = native.Plant()
        controller = CyclePositionController(plant, native.PROFILE, plant.samples, plant,
            replace(default_settings(), target_n=1), 2,
            node=dict(position_steps=position, measured_force_n=1), contact=self.recording['contact'],
            cycle_settings=CycleSettings(), clock=lambda: plant.now)
        return plant, controller

    def test_recorded_six_prediction_stops_do_not_recur_at_endpoint(self):
        false_stops = [r for r in self.recording['attempts'] if r['guard_reason'] == 'overshoot']
        self.assertEqual(len(false_stops), 6)
        for row in false_stops:
            self.assertLess(row['peak_n'], 1.05)
            self.assertGreater(row['predicted_peak_n'], 1.05)
            # 原记录未保存速度；分别注入2/5/30 RPM残余反馈，验证终点外推不依赖碰巧的速度值。
            for rpm in (2, 5, 30):
                plant, controller = self.controller(row['candidate_position_steps'])
                for r in row['last_loaded_samples']:
                    plant.now = r['stamp']+.10
                    controller._check_loaded_pressure(dict(position_counts=r['position_steps'],
                        measured_speed_rpm=-rpm, in_position=False), r['force_n'], r['stamp'], r['sequence'])
                self.assertIsNone(controller.guard_reason)

    def test_recorded_underpressure_history_can_advance_beyond_false_upper_bound(self):
        rows = self.recording['attempts'][:10]
        candidate = next_candidate(-434, 1, .71, .01, 2, self.recording['contact'], 6144, rows)
        self.assertLess(candidate, -447)
        true_overshoot = dict(candidate_position_steps=-465, guard_position_steps=-458,
            measured_force_n=None, peak_n=1.09, tolerance_n=.05, guard_reason='overshoot')
        guarded = next_candidate(-453, 1, .9, .01, 2, self.recording['contact'], 6144, rows+[true_overshoot])
        self.assertGreaterEqual(guarded, -457)

    def test_real_overshoot_at_stationary_endpoint_still_stops(self):
        plant, controller = self.controller()
        with self.assertRaises(FeedbackControlError):
            controller._check_loaded_pressure(dict(position_counts=-447, measured_speed_rpm=0, in_position=True),
                1.06, plant.now, 1)
        self.assertEqual(controller.guard_reason, 'overshoot')
        self.assertIsNone(controller.predicted_peak_n)

    def test_early_prediction_with_remaining_travel_is_separate_from_actual_overshoot(self):
        plant, controller = self.controller(-600)
        controller._previous_pressure = (.80, plant.now-.05, 1)
        with self.assertRaises(FeedbackControlError):
            controller._check_loaded_pressure(dict(position_counts=-480, measured_speed_rpm=-5, in_position=False),
                .97, plant.now, 2)
        self.assertEqual(controller.guard_reason, 'predictive_brake')
        self.assertGreater(controller.predicted_peak_n, 1.05)

    def test_relaxing_material_breaks_peak_underpressure_deadlock_by_relearning_at_lower_speed(self):
        _, seed, _ = seed_tests.LearningTests().learn()
        plant = native.Plant()
        original_force, move = plant.force, plant.move_relative_steps
        plant.last_speed, plant.last_down = 0, -100
        def motion(delta, **kw):
            move(delta, **kw)
            plant.last_speed = kw['speed_rpm'] if delta < 0 else 0
            plant.last_down = plant.now
        plant.move_relative_steps = motion
        plant.force = lambda: original_force()*(1+.035*plant.last_speed*max(0, 1-(plant.now-plant.last_down)/1.5))
        def point(row):
            if row['target_force_n'] == 1: plant.cancelled = True
        doc = run_relearning(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            default_settings(), 0, seed, on_point=point, clock=lambda: plant.now)
        self.assertEqual(plant.position, 0)
        point = doc['points'][0]
        self.assertTrue(point['converged'], doc['message'])
        self.assertTrue(point['speed_changes'])
        rpm = doc['nodes'][1]['medium_speed_rpm']
        self.assertLess(rpm, 5)
        self.assertTrue(all(r['ok'] and r['medium_speed_rpm'] == rpm for r in point['attempts'][-3:]))


if __name__ == '__main__': unittest.main()
