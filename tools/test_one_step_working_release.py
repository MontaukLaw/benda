"""接触附近工作释放点的真实路径模拟；不连接或驱动实机。"""
from copy import deepcopy
from dataclasses import replace
import unittest

import test_one_step_trajectory as trajectory_tests
from test_one_step_trajectory import (MovingPlant, native, ClockCancel, default_settings,
    run_relearning, CycleSettings, validate_document, run_trial)
from zdt_x42s_can_driver import ZDTTimeoutError


class WorkingReleaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        trajectory_tests.TrajectoryTests.setUpClass()
        cls.seed = trajectory_tests.TrajectoryTests.seed

    def learn(self, setup=None, progress=None, manual=True, fine=False):
        plant = MovingPlant()
        if setup: setup(plant)
        doc = run_relearning(plant, native.PROFILE, plant.samples, plant,
            ClockCancel(plant, lambda: plant.emergency), default_settings(), 0, deepcopy(self.seed),
            cycles=CycleSettings(trajectory_learning=True, near_contact_release=True,
                speed_step_rpm=.1 if fine else 1, medium_speed=3.5 if fine else 5,
                arrival_target_s=2 if fine else 1,
                manual_contact_position=-600 if manual else None),
            progress=lambda row: progress(plant, row) if progress else None, clock=lambda: plant.now)
        return plant, doc

    def test_fractional_learning_document_reloads_and_replays_exact_speed(self):
        plant, doc = self.learn(fine=True)
        self.assertTrue(doc['complete'], doc['message'])
        validate_document(doc, native.PROFILE)
        self.assertTrue(any(n['medium_speed_rpm'] != int(n['medium_speed_rpm']) for n in doc['nodes'][1:]))
        plant.absolute_commands.clear()
        result = run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            replace(default_settings(), hold_s=1), 0, doc, 4, mode='cycles', clock=lambda: plant.now)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(plant.absolute_commands[0]['speed'], doc['nodes'][4]['trajectory']['loaded_speed_rpm'])
        self.assertEqual(plant.position, doc['working_release_position_steps'])
        wrong = deepcopy(doc)
        wrong['cycle_settings']['speed_step_rpm'] = 1
        with self.assertRaises(ValueError): validate_document(wrong, native.PROFILE)

    def test_release_side_offset_continues_and_out_of_range_returns_original(self):
        for offset in (2, 3, 16, 17):
            def setup(plant):
                move = plant.move_relative_steps
                def release_offset(steps, **kwargs):
                    move(steps, **kwargs)
                    if steps > 0 and plant.position == -600:
                        plant.x += offset
                        plant.position += offset
                plant.move_relative_steps = release_offset
            plant, doc = self.learn(setup, fine=True)
            if offset <= 16:
                self.assertTrue(doc['complete'], doc['message'])
                validate_document(doc, native.PROFILE)
                self.assertEqual(plant.position, -600+offset)
                self.assertTrue(all(-2 <= r['position_steps']+600 <= 16
                    for check in doc['working_release_return_checks'] for r in check['samples']))
                plant.absolute_commands.clear()
                replay = run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
                    replace(default_settings(), hold_s=1), 0, doc, 1, mode='cycles', clock=lambda: plant.now)
                self.assertTrue(replay['ok'], replay['message'])
                self.assertEqual(replay['initial_positioning_s'], 0)
                self.assertEqual(doc['working_release_position_steps'], -600)
                self.assertEqual(plant.position, -600+offset)
            else:
                self.assertFalse(doc['complete'])
                self.assertFalse(doc['working_release_valid'])
                self.assertEqual(plant.position, 0)
                self.assertEqual(doc['working_release_position_error']['release_position_error_steps'], 17)

    def test_reported_minus714_is_unloaded_without_downward_correction_then_press_starts(self):
        from high_force_calibration import recover_release
        from pressure_feedback_control import PressureFeedbackController
        from one_step_working_release import confirm_unloaded, working_position_matches
        from one_step_trajectory import TrajectoryController, Trajectory
        plant = MovingPlant()
        plant.x = plant.position = -714
        plant.force = lambda: max(0, (-plant.x-717)/147)
        settings = replace(default_settings(), target_n=1, tolerance_n=.1)
        result = recover_release(plant, native.PROFILE, plant.samples, settings, -717, ClockCancel(plant),
            clock=lambda: plant.now, return_even_within_tolerance=True,
            position_tolerance_steps=2, release_side_tolerance_steps=16)
        self.assertTrue(result['release_position_confirmed'])
        self.assertEqual(result['release_position_error_steps'], 3)
        checker = PressureFeedbackController(plant, native.PROFILE, plant.samples, plant, settings, -717,
            clock=lambda: plant.now)
        evidence = []
        self.assertTrue(confirm_unloaded(checker, -717, evidence)['unloaded'])
        self.assertEqual(evidence[-1]['actual_position_steps'], -714)
        self.assertEqual(plant.actions, [])
        self.assertEqual(plant.absolute_commands, [])
        self.assertFalse(working_position_matches(-720, -717, native.PROFILE))
        controller = TrajectoryController(plant, native.PROFILE, plant.samples, plant, settings, -717,
            node=dict(position_steps=-864, measured_force_n=1),
            contact=dict(median_position_steps=-717, last_zero_boundary_steps=-701),
            cycle_settings=CycleSettings(trajectory_learning=True, near_contact_release=True, speed_step_rpm=.1),
            trajectory=Trajectory(-864, 5, -717, 5, 100, 100), clock=lambda: plant.now)
        result = controller.run()
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(plant.absolute_commands[0]['position'], -714)
        self.assertEqual(plant.absolute_commands[0]['target'], -864)

    def test_release_side_offset_with_residual_pressure_never_passes_unload(self):
        from pressure_feedback_control import PressureFeedbackController
        from one_step_working_release import confirm_unloaded
        plant = native.Plant()
        plant.position, plant.force_override = -714, .03
        checker = PressureFeedbackController(plant, native.PROFILE, plant.samples, plant,
            default_settings(), -717, clock=lambda: plant.now)
        result = confirm_unloaded(checker, -717, [])
        self.assertFalse(result['unloaded'])
        self.assertEqual(result['status'], 'persistent_residual')
        self.assertEqual(plant.actions, [])

    def test_temporary_settling_flag_recovers_and_persistent_failure_returns_original(self):
        for duration in (2.2, float('inf')):
            injected = []
            def setup(plant):
                plant.bad_until = 0
                read = plant.read_feedback
                def transient_flag():
                    sample = read()
                    if plant.now < plant.bad_until and plant.position == -600:
                        sample['in_position'] = False
                    return sample
                plant.read_feedback = transient_flag
            def progress(plant, row):
                if row.get('phase') == '工作释放点卸载复核' and not injected:
                    injected.append(True)
                    plant.bad_until = plant.now+duration
            plant, doc = self.learn(setup, progress, fine=True)
            self.assertTrue(injected)
            checks = doc['working_release_return_checks']
            self.assertTrue(any(w.get('status') == 'settling_timeout' for c in checks for w in c['windows']))
            if duration < float('inf'):
                self.assertTrue(doc['complete'], doc['message'])
                validate_document(doc, native.PROFILE)
                self.assertTrue(any(c['status'] == 'recovered' for c in checks))
            else:
                self.assertFalse(doc['complete'])
                self.assertFalse(doc['working_release_valid'])
                self.assertEqual(plant.position, 0)
                bad = next(c for c in checks if c['status'] == 'error')
                self.assertEqual(len(bad['windows']), 3)
                self.assertTrue(bad['diagnostics'])
                self.assertFalse(bad['diagnostics'][-1]['in_position'])

    def test_working_zero_keeps_absolute_coordinates_and_all_cycles_skip_air(self):
        plant, doc = self.learn()
        self.assertTrue(doc['complete'], doc['message'])
        validate_document(doc, native.PROFILE)
        self.assertEqual(doc['position_learning_version'], 4)
        self.assertEqual(doc['origin_position_steps'], 0)
        self.assertEqual(doc['working_release_position_steps'], -600)
        self.assertEqual(doc['nodes'][0]['position_steps'], -600)
        self.assertEqual(plant.position, -600)
        self.assertGreater(doc['initial_positioning_s'], 1)
        for point in doc['points']:
            self.assertTrue(point['best_pressure_candidate']['time_ok'])
            for row in point['attempts']:
                self.assertEqual(row['release_position_steps'], -600)
                self.assertEqual(len(row['commands']), 1)
                self.assertEqual(row['trajectory']['decel_position_steps'], -600)
                self.assertEqual(row['trajectory']['free_speed_rpm'], row['trajectory']['loaded_speed_rpm'])
            self.assertLessEqual(point['attempts'][-1]['position_arrival_s'], 1+1e-9)
        self.assertTrue(all(delta > 0 for delta in plant.actions))

    def test_contact_candidate_with_small_residual_backs_off_before_confirming(self):
        plant, doc = self.learn(setup=lambda p: setattr(p, 'contact', 599))
        self.assertTrue(doc['complete'], doc['message'])
        checks = doc['working_release_checks']
        self.assertFalse(checks[0]['unloaded'])
        self.assertEqual(checks[0]['position_steps'], -600)
        self.assertTrue(checks[-1]['unloaded'])
        self.assertEqual(doc['working_release_position_steps'], -592)
        self.assertEqual(plant.position, -592)

    def test_automatic_contact_also_establishes_unloaded_working_point(self):
        plant, doc = self.learn(manual=False)
        self.assertTrue(doc['complete'], doc['message'])
        self.assertEqual(len(doc['contacts']), 10)
        self.assertEqual(plant.position, doc['working_release_position_steps'])
        self.assertLess(plant.position, 0)

    def test_initial_positioning_stop_pressure_loss_and_excess_contact_return_original(self):
        for fault in ('stop', 'stale', 'pressure'):
            injected = []
            def progress(plant, row):
                if row.get('phase') == '低速确认工作释放点' and plant.position < -100 and not injected:
                    injected.append(True)
                    if fault == 'stop': plant.cancelled = True
                    elif fault == 'stale': plant.fault = 'stale'
                    else: plant.force_override = .2
            plant, doc = self.learn(progress=progress)
            self.assertTrue(injected, fault)
            self.assertFalse(doc['complete'])
            self.assertEqual(plant.position, 0)
            self.assertIsNone(doc['working_release_position_steps'])
            self.assertTrue(doc['release_position_confirmed'])

    def test_stop_and_pressure_loss_return_working_point_in_all_pressure_phases(self):
        for phase in ('连续轨迹寻压', '轨迹到位观察', 'return'):
            for fault in ('stop', 'stale'):
                injected = []
                def progress(plant, row):
                    if row.get('phase') == phase and plant.position < -602 and not injected:
                        injected.append(True)
                        if fault == 'stop': plant.cancelled = True
                        else: plant.fault = 'stale'
                plant, doc = self.learn(progress=progress, fine=True)
                self.assertTrue(injected, (phase, fault))
                self.assertFalse(doc['complete'])
                self.assertEqual(plant.position, -600)
                self.assertTrue(doc['release_position_confirmed'])

    def test_transient_motor_loss_waits_then_returns_working_point(self):
        for phase in ('连续轨迹寻压', '轨迹到位观察', 'return'):
            injected, recovered = [], []
            def setup(plant):
                read = plant.read_feedback
                def interrupted():
                    if plant.fault == 'motor_link': raise RuntimeError('temporary CAN loss')
                    return read()
                plant.read_feedback = interrupted
            def progress(plant, row):
                if row.get('phase') == phase and plant.position < -602 and not injected:
                    plant.fault = 'motor_link'
                    injected.append(True)
                if injected and row.get('phase') == 'recovery' and row.get('recovery_attempt', 0) >= 2:
                    recovered.append(True)
                    plant.fault, plant.cancelled = None, True
            plant, doc = self.learn(setup, progress, fine=True)
            self.assertTrue(injected and recovered, phase)
            self.assertEqual(plant.position, -600)
            self.assertFalse(doc['complete'])

    def test_overcurrent_blocks_return_until_feedback_recovers(self):
        injected, currents = [], []
        def setup(plant):
            move = plant.move_relative_steps
            def guarded(delta, **kwargs):
                currents.append(plant.current)
                return move(delta, **kwargs)
            plant.move_relative_steps = guarded
        def progress(plant, row):
            if row.get('phase') == '连续轨迹寻压' and plant.position < -602 and not injected:
                injected.append(True)
                plant.current = 4000
            if injected and row.get('phase') == 'recovery' and row.get('recovery_attempt', 0) >= 2:
                plant.current, plant.cancelled = 200, True
        plant, doc = self.learn(setup, progress, fine=True)
        self.assertFalse(doc['complete'])
        self.assertEqual(plant.position, -600)
        self.assertTrue(currents)
        self.assertTrue(all(v <= 3000 for v in currents))

    def test_emergency_never_returns_or_resumes(self):
        def progress(plant, row):
            if row.get('phase') == '连续轨迹寻压' and plant.position < -602:
                plant.emergency = True
        plant, doc = self.learn(progress=progress, fine=True)
        self.assertTrue(doc['emergency_stopped'])
        self.assertFalse(doc['release_position_confirmed'])
        self.assertLess(plant.position, -600)
        self.assertEqual(plant.actions, [])

    def test_residual_after_loading_invalidates_working_point_and_returns_original(self):
        def setup(plant):
            force = plant.force
            plant.residual = False
            plant.force = lambda: max(force(), .04 if plant.residual and -600 <= plant.x < -590 else 0)
        def progress(plant, row):
            if row.get('phase') == '轨迹到位观察': plant.residual = True
        plant, doc = self.learn(setup, progress)
        self.assertFalse(doc['complete'])
        self.assertFalse(doc['working_release_valid'])
        self.assertEqual(plant.position, 0)
        self.assertEqual(doc['last_release_position_steps'], 0)
        self.assertEqual(len(doc['points'][0]['attempts']), 1)
        self.assertIn('残余压力', doc['message'])

    def test_pressure_button_uses_working_release_and_detects_stale_table(self):
        plant, doc = self.learn()
        plant.absolute_commands.clear()
        result = run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            replace(default_settings(), hold_s=1), 0, doc, 1, mode='cycles', clock=lambda: plant.now)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(plant.position, -600)
        self.assertEqual(result['working_release_position_steps'], -600)
        self.assertEqual(result['initial_positioning_s'], 0)
        self.assertTrue(all(c['target'] == doc['nodes'][1]['position_steps'] for c in plant.absolute_commands))
        for key, value in [('working_release_valid', False), ('working_release_position_steps', -599),
                           ('initial_positioning_s', float('nan'))]:
            wrong = deepcopy(doc)
            wrong[key] = value
            with self.assertRaises(ValueError): validate_document(wrong, native.PROFILE)

    def test_history_button_initial_positioning_is_separate_and_release_residual_revokes_table(self):
        _, doc = self.learn()
        plant = MovingPlant()
        result = run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            replace(default_settings(), hold_s=1), 0, doc, 1, mode='cycles', clock=lambda: plant.now)
        self.assertTrue(result['ok'], result['message'])
        self.assertGreater(result['initial_positioning_s'], 1)
        self.assertEqual(plant.position, -600)
        force = plant.force
        plant.residual = False
        plant.force = lambda: max(force(), .04 if plant.residual and -600 <= plant.x < -590 else 0)
        def progress(row):
            if row.get('phase') == '轨迹到位观察': plant.residual = True
        result = run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            replace(default_settings(), hold_s=1), 0, doc, 1, mode='cycles', progress=progress, clock=lambda: plant.now)
        self.assertFalse(result['ok'])
        self.assertTrue(result['working_release_invalidated'])
        self.assertEqual(plant.position, 0)

    def test_pressure_travel_limit_does_not_shift_down_with_new_release_point(self):
        from one_step_working_release import working_settings
        initial = default_settings()
        active = working_settings(initial, 0, -600)
        self.assertEqual(-600-active.max_travel, -initial.max_travel)

    def test_after_1n_rest_transient_residual_is_archived_then_2n_continues(self):
        injected = []
        def setup(plant):
            force = plant.force
            plant.spike_until = 0
            plant.force = lambda: (.03 if plant.now < plant.spike_until else .02) if plant.position == -600 else force()
        def progress(plant, row):
            if row.get('phase') == 'rest' and row.get('target_n') == 2 and not injected:
                injected.append(True)
                plant.spike_until = plant.now+5.3
        plant, doc = self.learn(setup, progress)
        self.assertTrue(injected)
        self.assertTrue(doc['complete'], doc['message'])
        recovered = [r for r in doc['working_release_prepare_checks'] if r['status'] == 'recovered']
        self.assertTrue(recovered)
        check = recovered[0]
        self.assertEqual(check['target_n'], 2)
        self.assertEqual(check['max_force_n'], .03)
        self.assertFalse(check['windows'][0]['unloaded'])
        self.assertTrue(check['windows'][-1]['unloaded'])
        self.assertTrue(all(abs(r['force_n']) <= .02 for r in check['samples']))
        self.assertEqual(plant.position, -600)

    def test_after_1n_rest_persistent_residual_returns_original_and_preserves_1n(self):
        injected = []
        def setup(plant):
            force = plant.force
            plant.residual = False
            plant.force = lambda: .03 if plant.residual and plant.position == -600 else force()
        def progress(plant, row):
            if row.get('phase') == 'rest' and row.get('target_n') == 2 and not injected:
                injected.append(True)
                plant.residual = True
        plant, doc = self.learn(setup, progress)
        self.assertFalse(doc['complete'])
        self.assertFalse(doc['working_release_valid'])
        self.assertTrue(doc['points'][0]['converged'])
        self.assertTrue(doc['points'][0]['best_pressure_candidate']['time_ok'])
        self.assertEqual(doc['points'][1]['attempts'], [])
        check = doc['working_release_prepare_checks'][-1]
        self.assertEqual(check['status'], 'persistent_residual')
        self.assertEqual(len(check['windows']), 3)
        self.assertTrue(all(r['force_n'] == .03 for r in check['samples']))
        self.assertEqual(plant.position, 0)
        self.assertEqual(doc['last_release_position_steps'], 0)

    def test_return_transient_residual_rechecks_without_discarding_working_point(self):
        injected = []
        def setup(plant):
            force = plant.force
            plant.spike_until = 0
            plant.force = lambda: .03 if plant.position == -600 and plant.now < plant.spike_until else force()
        def progress(plant, row):
            if row.get('phase') == '工作释放点卸载复核' and plant.position == -600 and not injected:
                injected.append(True)
                plant.spike_until = plant.now+.3
        plant, doc = self.learn(setup, progress)
        self.assertTrue(doc['complete'], doc['message'])
        checks = [r for r in doc['working_release_return_checks'] if r['status'] == 'recovered']
        self.assertTrue(checks)
        self.assertEqual(checks[0]['max_force_n'], .03)
        self.assertTrue(doc['working_release_valid'])
        self.assertEqual(plant.position, -600)

    def test_actual_trajectory_start_uses_same_recheck_and_persistent_failure_never_moves_down(self):
        for persistent in (False, True):
            injected = []
            def setup(plant):
                force = plant.force
                plant.spike_until = 0
                plant.force = lambda: .03 if plant.position == -600 and plant.now < plant.spike_until else force()
            def progress(plant, row):
                if row.get('phase') == '轨迹启动前卸载复核' and not injected:
                    injected.append(True)
                    plant.spike_until = float('inf') if persistent else plant.now+.3
            plant, doc = self.learn(setup, progress)
            self.assertTrue(injected)
            row = doc['points'][0]['attempts'][0]
            self.assertEqual(row['startup_unload_checks'][0]['max_force_n'], .03)
            if persistent:
                self.assertFalse(doc['complete'])
                self.assertEqual(row['guard_reason'], 'working_release_not_unloaded')
                self.assertEqual(row['commands'], [])
                self.assertEqual(plant.position, 0)
            else:
                self.assertTrue(doc['complete'], doc['message'])
                self.assertEqual(row['startup_unload_checks'][0]['status'], 'recovered')
                self.assertEqual(plant.position, -600)

    def test_can_timeout_recovers_before_retry_in_each_phase(self):
        for phase in ('rest', '休息后工作释放点卸载复核', '轨迹启动前卸载复核',
                      '连续轨迹寻压', '轨迹到位观察', 'return', '工作释放点卸载复核'):
            with self.subTest(phase=phase):
                injected, retry_positions, retry_commands = [], [], []
                def setup(plant):
                    read = plant.read_feedback
                    plant.timeout_pending = False
                    def interrupted():
                        if plant.timeout_pending:
                            plant.timeout_pending = False
                            raise ZDTTimeoutError('motor 0x43 timeout')
                        return read()
                    plant.read_feedback = interrupted
                def progress(plant, row):
                    if row.get('phase') == phase and row.get('target_n') == 2 and not injected:
                        injected.append(True)
                        plant.timeout_pending = True
                    if row.get('phase') == 'communication_retry':
                        retry_positions.append(plant.position)
                        retry_commands.append(len(plant.absolute_commands))
                plant, doc = self.learn(setup, progress)
                self.assertTrue(injected)
                self.assertTrue(doc['complete'], doc['message'])
                validate_document(doc, native.PROFILE)
                self.assertTrue(doc['points'][0]['converged'])
                self.assertEqual(plant.position, -600)
                point = doc['points'][1]
                if phase not in ('return', '工作释放点卸载复核'):
                    self.assertEqual(retry_positions, [-600])
                    interruption, = point['communication_interruptions']
                    self.assertEqual(interruption['error_type'], 'ZDTTimeoutError')
                    self.assertTrue(interruption['retry_scheduled'])
                    self.assertTrue(interruption['release_pressure_unloaded'])
                    self.assertFalse(interruption['learning_evidence_valid'])
                    resumed = point['attempts'][interruption['attempt']-1]
                    self.assertEqual(resumed['trajectory'], interruption['trajectory'])
                    command = plant.absolute_commands[retry_commands[0]]
                    self.assertEqual(command['position'], -600)
                    self.assertEqual(command['velocity'], 0)
                    if phase in ('rest', '休息后工作释放点卸载复核', '轨迹启动前卸载复核'):
                        self.assertEqual(interruption['commands'], [])
                plans = [tuple(r['trajectory'].values()) for r in point['attempts']]
                self.assertEqual(len(plans), len(set(plans)))
                if phase in ('休息后工作释放点卸载复核', '工作释放点卸载复核'):
                    wrong = deepcopy(doc)
                    key = ('working_release_prepare_checks' if phase == '休息后工作释放点卸载复核'
                           else 'working_release_return_checks')
                    failed = next(c for c in wrong[key] if c.get('status') == 'error')
                    # 其他轮次的成功不能替代被中断这一轮的恢复证据。
                    wrong[key] = [c for c in wrong[key] if c.get('status') == 'error'
                        or any(c.get(k) != failed.get(k) for k in ('phase', 'target_n', 'attempt'))]
                    with self.assertRaises(ValueError):
                        validate_document(wrong, native.PROFILE)

    def test_lost_motion_ack_stops_and_returns_before_starting_a_new_cycle(self):
        def setup(plant):
            move = plant.move_absolute_steps
            plant.lose_ack = False
            def interrupted(position, **kwargs):
                move(position, **kwargs)
                if plant.lose_ack:
                    plant.lose_ack = False
                    plant.wait(.05)
                    plant.advance()
                    raise ZDTTimeoutError('motion acknowledged too late')
            plant.move_absolute_steps = interrupted
        injected, retry_indices = [], []
        def progress(plant, row):
            if row.get('phase') == 'trajectory_learning' and row.get('target_n') == 2 and not injected:
                injected.append(True)
                plant.lose_ack = True
            if row.get('phase') == 'communication_retry':
                retry_indices.append(len(plant.absolute_commands))
                self.assertEqual(plant.position, -600)
        plant, doc = self.learn(setup, progress)
        self.assertTrue(doc['complete'], doc['message'])
        interruption, = doc['points'][1]['communication_interruptions']
        self.assertEqual(len(interruption['commands']), 1)
        self.assertFalse(interruption['commands'][0]['acknowledged'])
        index, = retry_indices
        self.assertEqual(plant.absolute_commands[index]['position'], -600)
        self.assertGreater(plant.absolute_commands[index]['stops'], plant.absolute_commands[index-1]['stops'])

    def test_repeated_can_timeouts_have_bounded_retries_and_preserve_completed_point(self):
        def setup(plant):
            read = plant.read_feedback
            plant.timeout_pending = False
            def interrupted():
                if plant.timeout_pending:
                    plant.timeout_pending = False
                    raise ZDTTimeoutError('motor 0x43 timeout')
                return read()
            plant.read_feedback = interrupted
        def progress(plant, row):
            if row.get('phase') == '轨迹启动前卸载复核' and row.get('target_n') == 2:
                plant.timeout_pending = True
        plant, doc = self.learn(setup, progress)
        self.assertFalse(doc['complete'])
        self.assertTrue(doc['points'][0]['converged'])
        self.assertTrue(doc['points'][0]['best_pressure_candidate']['time_ok'])
        point = doc['points'][1]
        self.assertEqual(point['attempts'], [])
        interruptions = point['communication_interruptions']
        self.assertEqual(len(interruptions), 3)
        self.assertEqual([r['retry_scheduled'] for r in interruptions], [True, True, False])
        self.assertTrue(all(r['attempt'] == 1 and not r['commands'] for r in interruptions))
        self.assertEqual(plant.position, -600)
        self.assertTrue(doc['release_position_confirmed'])

    def test_can_retry_never_overrides_stop_emergency_or_missing_pressure(self):
        for fault in ('stop', 'emergency', 'stale'):
            with self.subTest(fault=fault):
                injected, retries = [], []
                def setup(plant):
                    read = plant.read_feedback
                    plant.timeout_pending = False
                    def interrupted():
                        if plant.timeout_pending:
                            plant.timeout_pending = False
                            raise ZDTTimeoutError('motor 0x43 timeout')
                        return read()
                    plant.read_feedback = interrupted
                def progress(plant, row):
                    if row.get('phase') == '连续轨迹寻压' and plant.position < -602 and not injected:
                        injected.append(True)
                        plant.timeout_pending = True
                    if injected and row.get('phase') == 'return':
                        if fault == 'stop': plant.cancelled = True
                        elif fault == 'emergency': plant.emergency = True
                        else: plant.fault = 'stale'
                    if row.get('phase') == 'communication_retry':
                        retries.append(len(plant.absolute_commands))
                plant, doc = self.learn(setup, progress)
                self.assertTrue(injected)
                self.assertFalse(doc['complete'])
                interruptions = doc['points'][0]['communication_interruptions']
                self.assertEqual(len(interruptions), 1)
                self.assertFalse(interruptions[0]['retry_scheduled'])
                self.assertEqual(retries, [len(plant.absolute_commands)])
                if fault == 'emergency':
                    self.assertLess(plant.position, -600)
                    self.assertEqual(plant.actions, [])
                else:
                    self.assertEqual(plant.position, -600)
                    self.assertTrue(doc['release_position_confirmed'])


if __name__ == '__main__': unittest.main()
