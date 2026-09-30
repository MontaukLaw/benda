"""连续轨迹的动态模拟、逐轮学习和异常回位；绝不连接实机。"""
from copy import deepcopy
from dataclasses import asdict, replace
import math
import unittest
from unittest.mock import patch

import test_pressure_feedback as native
from test_high_force_calibration import ClockCancel
import test_one_step_learning as learning_tests
import test_one_step_repeatability as trial_tests
from one_step_learning import default_settings
from one_step_relearning import CycleSettings, run_relearning, validate_document
from one_step_trajectory import Trajectory, TrajectoryController, adjust, bounded, learn_point
from one_step_repeatability import run_trial
from test_zdt_motor import driver, frames, scaled_driver
from zdt_x42s_can_driver import ZDTTimeoutError
from one_step_learning_widget import OneStepLearningWidget


class MovingPlant(native.Plant):
    """按有限加减速度运动，目标附近主动刹车；压力可注入速度相关的加载峰值。"""
    def __init__(self):
        super().__init__()
        self.x, self.velocity, self.target = 0., 0., None
        self.command_speed, self.accel = 1, 100
        self.updated = self.now
        self.absolute_commands = []
        self.emergency = False
        self.dynamic_gain = 0
        self.contact = 600

    def advance(self):
        remaining = self.now-self.updated
        while remaining > 1e-8:
            dt = min(.001, remaining)
            if self.target is not None:
                distance = abs(self.target-self.x)
                acceleration = 3200/60/((256-self.accel)*.00005)
                speed = min(self.command_speed*3200/60, math.sqrt(2*acceleration*distance))
                self.velocity += max(-acceleration*dt, min(acceleration*dt, speed-self.velocity))
                step = self.velocity*dt
                if step >= distance or distance < .02:
                    self.x, self.velocity, self.target = float(self.target), 0., None
                else:
                    self.x += math.copysign(step, self.target-self.x)
            remaining -= dt
        self.position = round(self.x)
        self.updated = self.now

    def force(self):
        static = max(0, (-self.x-self.contact)*.04)
        return static + (self.dynamic_gain*self.velocity/(3200/60) if static > 0 else 0)

    def read_feedback(self):
        self.advance()
        sample = super().read_feedback()
        sample.update(measured_speed_rpm=self.velocity/(3200/60), in_position=self.target is None)
        return sample

    def move_absolute_steps(self, position, *, speed_rpm, accel, retry_cancelled):
        self.advance()
        assert not retry_cancelled()
        self.absolute_commands.append(dict(target=position, speed=speed_rpm, acceleration=accel,
            position=self.position, velocity=self.velocity, stops=self.stop_count))
        self.target, self.command_speed, self.accel = position, speed_rpm, accel

    def move_relative_steps(self, steps, **kwargs):
        self.advance()
        super().move_relative_steps(steps, **kwargs)
        self.x, self.target, self.velocity = float(self.position), None, 0.

    def stop(self):
        self.advance()
        super().stop()
        self.target, self.velocity = None, 0.


class TrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.seed, _ = learning_tests.LearningTests().learn()
        for node in cls.seed['nodes'][1:]:
            node['position_steps'] -= 590
            node['travel_steps'] += 590

    def learn(self, setup=None, progress=None, manual=True, checkpoint=lambda doc: None, arrival_target_s=1.0,
              full_window=True):
        plant = MovingPlant()
        if setup: setup(plant)
        def record(row):
            if progress: progress(plant, row)
        doc = run_relearning(plant, native.PROFILE, plant.samples, plant,
            ClockCancel(plant, lambda: plant.emergency), default_settings(), 0, deepcopy(self.seed),
            cycles=CycleSettings(trajectory_learning=True, manual_contact_position=-600 if manual else None,
                                 arrival_target_s=arrival_target_s, require_full_observation_band=full_window),
            progress=record, checkpoint=checkpoint, clock=lambda: plant.now)
        return plant, doc

    def test_two_second_target_accepts_slower_success_and_keeps_legacy_validation_strict(self):
        _, doc = self.learn(arrival_target_s=2.0)
        self.assertTrue(doc['complete'], doc['message'])
        validate_document(doc, native.PROFILE)
        self.assertTrue(any(1 < n['position_arrival_s'] <= 2 for n in doc['nodes'][1:]))
        self.assertTrue(all(n['position_arrival_s'] <= 2+1e-9 for n in doc['nodes'][1:]))
        self.assertIn('2秒到位目标', doc['message'])
        stricter = deepcopy(doc)
        stricter['cycle_settings']['arrival_target_s'] = 1.0
        with self.assertRaises(ValueError): validate_document(stricter, native.PROFILE)

    def test_four_newtons_explores_tenths_without_acceleration_ratchet(self):
        from high_force_calibration import recover_release
        plant = MovingPlant()
        plant.x = plant.position = -717
        plant.contact = 717
        plant.force = lambda: max(0, (-plant.x-717)*4/351)
        settings = replace(default_settings(), target_n=4, tolerance_n=.1, acceleration=85)
        cycles = CycleSettings(trajectory_learning=True, near_contact_release=True,
            manual_contact_position=-717, medium_speed=3, speed_step_rpm=.1, arrival_target_s=2)
        contact = dict(median_position_steps=-717, last_zero_boundary_steps=-701,
                       baseline_min_n=0, baseline_max_n=0)
        point = dict(attempts=[], converged=False)
        def release():
            return recover_release(plant, native.PROFILE, plant.samples, settings, -717,
                ClockCancel(plant), clock=lambda: plant.now, return_even_within_tolerance=True)
        node = learn_point(plant, native.PROFILE, plant.samples, plant, settings, -717, 4, 4/351,
            -1068, contact, cycles, None, point, lambda rest: plant.wait(5), release,
            lambda: None, lambda row: None, lambda row: None, lambda row: None, lambda: plant.now)
        self.assertTrue(point['converged'])
        rows = point['attempts']
        self.assertGreater(rows[0]['position_arrival_s'], 2)
        self.assertLessEqual(node['position_arrival_s'], 2+1e-9)
        self.assertLess(node['trajectory']['loaded_speed_rpm'], 4)
        self.assertEqual([r['trajectory']['loaded_speed_rpm'] for r in rows],
                         [round(3+i*.1, 1) for i in range(len(rows))])
        self.assertTrue(all(r['trajectory']['position_steps'] == -1068 and
            r['trajectory']['acceleration'] == r['trajectory']['deceleration'] == 85 for r in rows))
        self.assertTrue(all(r['pressure_ok'] for r in rows))
        self.assertEqual(plant.position, -717)

    def test_absolute_command_packet_and_no_retry(self):
        motor, bus, device = scaled_driver()
        self.addCleanup(motor.close)
        motor._firmware = 'Emm'
        motor.move_absolute_pulses(-800, 25, 100)
        self.assertEqual([bytes(m.data) for m in bus.sent],
                         [bytes.fromhex('43 7A 6B'), bytes.fromhex('4F 71 00 01 6B'),
                          bytes.fromhex('FD 01 00 FA 64 00 00 03'), bytes.fromhex('FD 20 01 00 6B')])
        for position, speed, acc in [(-800, 0, 100), (-800, 20, 0), (-800.5, 20, 100)]:
            with self.assertRaises(ValueError): motor.move_absolute_pulses(position, speed, acc)
        missing, missing_bus, lost = scaled_driver()
        lost['motion_reply'] = False
        self.addCleanup(missing.close)
        missing._firmware = 'Emm'
        with self.assertRaises(ZDTTimeoutError): missing.move_absolute_pulses(-800, 25, 100)
        self.assertEqual(sum(m.data[0] == 0xFD for m in missing_bus.sent), 2)
        self.assertEqual(len(lost['motions']), 1)

    def test_all_points_adapt_to_one_second_without_repeating_failed_parameters(self):
        plant, doc = self.learn()
        self.assertTrue(doc['complete'], doc['message'])
        validate_document(doc, native.PROFILE)
        self.assertEqual(doc['contacts'], [])
        self.assertEqual(doc['contact']['source'], 'manual')
        self.assertEqual(plant.position, 0)
        self.assertEqual(len(doc['nodes']), 31)
        self.assertTrue(any(not r['time_ok'] for p in doc['points'] for r in p['attempts']))
        for point in doc['points']:
            attempts = point['attempts']
            self.assertTrue(attempts[-1]['ok'])
            self.assertLessEqual(attempts[-1]['position_arrival_s'], 1+1e-9)
            self.assertEqual(len({tuple(r['trajectory'].values()) for r in attempts}), len(attempts))
            for a, b in zip(attempts, attempts[1:]):
                self.assertEqual(a['next_trajectory'], b['trajectory'])
                self.assertTrue(a['adjustment_reason'])
                if not a['pressure_ok'] and a['arrival'] and not a['guard_reason']:
                    self.assertEqual({k: v for k, v in a['trajectory'].items() if k != 'position_steps'},
                                     {k: v for k, v in b['trajectory'].items() if k != 'position_steps'})
                if b['trajectory']['loaded_speed_rpm'] > a['trajectory']['loaded_speed_rpm']:
                    self.assertTrue(a['pressure_ok'])
                    self.assertEqual(a['trajectory']['position_steps'], b['trajectory']['position_steps'])
        moving_updates = [c for c in plant.absolute_commands if c['velocity'] > 0]
        self.assertTrue(moving_updates)
        for a, b in zip(plant.absolute_commands, plant.absolute_commands[1:]):
            if b['velocity'] > 0:
                self.assertEqual(a['target'], b['target'])
                self.assertEqual(a['stops'], b['stops'])

    def test_underpressure_adjusts_only_position_even_with_peak_near_limit(self):
        settings, cycles = default_settings(), CycleSettings(trajectory_learning=True)
        contact = dict(median_position_steps=-621, last_zero_boundary_steps=-605)
        plan = bounded(Trajectory(-835, 25, -300, 5, 100, 100), 0, contact, settings, cycles, native.PROFILE)
        row = dict(target_n=2, tolerance_n=.05, measured_force_n=1.88, peak_n=2.03, guard_reason=None,
            arrival=dict(min_force_n=1.86, max_force_n=1.95), candidate_position_steps=-835, trajectory=asdict(plan))
        new, reason = adjust(plan, row, [row], .02, 0, contact, settings, cycles, native.PROFILE)
        self.assertEqual(replace(new, position_steps=plan.position_steps), plan)
        self.assertLess(new.position_steps, -835)
        self.assertIn('先学习压力位置', reason)

    def test_median_mode_accepts_4n_window_dip_without_learning_deeper(self):
        from one_step_relearning import observation_pressure_ok
        values = [3.87, 3.93, 4.01]
        self.assertFalse(observation_pressure_ok(values, 4, .1, True))
        self.assertTrue(observation_pressure_ok(values, 4, .1, False))
        self.assertFalse(observation_pressure_ok([3.8, 3.85, 3.87], 4, .1, False))
        self.assertTrue(CycleSettings().require_full_observation_band)
        settings = replace(default_settings(), target_n=4, tolerance_n=.1)
        cycles = CycleSettings(trajectory_learning=True, near_contact_release=True,
            speed_step_rpm=.1, arrival_target_s=2, require_full_observation_band=False)
        contact = dict(median_position_steps=-717, last_zero_boundary_steps=-701)
        plan = Trajectory(-1059, 4.7, -987, 4.5, 100, 85)
        row = dict(target_n=4, tolerance_n=.1, measured_force_n=3.93, peak_n=4.10,
            guard_reason=None, arrival=dict(min_force_n=3.87, max_force_n=4.01),
            candidate_position_steps=-1059, trajectory=asdict(plan), pressure_ok=True, position_arrival_s=2.1)
        faster, reason = adjust(plan, row, [row], .02, -717, contact, settings, cycles, native.PROFILE)
        self.assertEqual(faster.position_steps, plan.position_steps)
        self.assertGreater(faster.loaded_speed_rpm, plan.loaded_speed_rpm)
        self.assertIn('压力已合格', reason)

    def test_separate_overshoot_limit_keeps_median_tolerance_and_legacy_rules(self):
        from types import SimpleNamespace
        from pressure_feedback_control import FeedbackControlError
        from one_step_relearning import observation_pressure_ok
        cycles = CycleSettings(overshoot_absolute_n=.20, overshoot_relative_tolerance=.01)
        for target, limit in ((1, 1.2), (4, 4.2), (20, 20.2), (30, 30.3)):
            with self.subTest(target=target):
                controller = SimpleNamespace(cycle_settings=cycles,
                    settings=replace(default_settings(), target_n=target, tolerance_n=max(.1, target*.01)))
                sample = dict(position_counts=-1059)
                TrajectoryController._check_loaded_pressure(controller, sample, limit, 0, 0)
                with self.assertRaises(FeedbackControlError):
                    TrajectoryController._check_loaded_pressure(controller, sample, limit+.01, 0, 1)
                self.assertEqual(controller.guard_reason, 'overshoot')
        self.assertAlmostEqual(CycleSettings().overshoot_limit(4, .1), 4.1)
        self.assertFalse(observation_pressure_ok([4.15]*10, 4, .1, False))

    def test_learning_accepts_median_and_peak_with_separate_limits_and_still_releases(self):
        from high_force_calibration import recover_release
        plant = MovingPlant()
        plant.x = plant.position = -717
        stopped_at = []
        def force():
            if plant.target is None and plant.x <= -1059:
                if not stopped_at: stopped_at.append(plant.now)
                return 4.15-min(.28, .27*(plant.now-stopped_at[0]))
            return max(0, (-plant.x-717)*4/342)
        plant.force = force
        settings = replace(default_settings(), target_n=4, tolerance_n=.1)
        cycles = CycleSettings(trajectory_learning=True, near_contact_release=True,
            medium_speed=5, speed_step_rpm=.1, arrival_target_s=2, require_full_observation_band=False,
            overshoot_absolute_n=.20)
        contact = dict(median_position_steps=-717, last_zero_boundary_steps=-701)
        point = dict(attempts=[], converged=False)
        def release():
            return recover_release(plant, native.PROFILE, plant.samples, settings, -717,
                ClockCancel(plant), clock=lambda: plant.now, return_even_within_tolerance=True)
        node = learn_point(plant, native.PROFILE, plant.samples, plant, settings, -717, 4, .02,
            -1059, contact, cycles, None, point, lambda rest: plant.wait(5), release,
            lambda: None, lambda row: None, lambda row: None, lambda row: None, lambda: plant.now)
        self.assertEqual(len(point['attempts']), 1)
        row = point['attempts'][0]
        self.assertLess(row['arrival']['min_force_n'], 3.9)
        self.assertGreaterEqual(node['measured_force_n'], 3.9)
        self.assertGreater(row['peak_n'], 4.1)
        self.assertLessEqual(row['peak_n'], 4.2)
        self.assertEqual(row['overshoot_limit_n'], 4.2)
        self.assertTrue(row['ok'])
        self.assertEqual(plant.position, -717)

    def test_recorded_two_step_settling_does_not_lock_arrival_window(self):
        # 09:03实测先在终点下方两步到位，随后回弹到终点；旧锚点会令观察窗永久重置。
        settled_reads = []
        def setup(plant):
            read = plant.read_feedback
            def with_settling():
                sample = read()
                if sample['in_position'] and plant.position < -600 and len(settled_reads) < 3:
                    sample['position_counts'] -= (2, 1, 1)[len(settled_reads)]
                    settled_reads.append(sample['position_counts'])
                return sample
            plant.read_feedback = with_settling
        plant, doc = self.learn(setup=setup)
        self.assertEqual(len(settled_reads), 3)
        self.assertTrue(doc['complete'], doc['message'])
        first = doc['points'][0]['attempts'][0]
        self.assertIsNotNone(first['arrival'])
        self.assertIsNone(first['guard_reason'])
        self.assertEqual(plant.position, 0)

    def test_timeout_without_observation_never_increases_speed(self):
        settings, cycles = default_settings(), CycleSettings(trajectory_learning=True)
        contact = dict(median_position_steps=-621, last_zero_boundary_steps=-605)
        plan = bounded(Trajectory(-744, 30, -151, 5, 100, 100), 0, contact, settings, cycles, native.PROFILE)
        for guard in ('position_timeout', 'settling_timeout', None):
            row = dict(target_n=1, tolerance_n=.05, measured_force_n=None, peak_n=.82,
                       last_observed_force_n=.69, guard_reason=guard, arrival=None,
                       candidate_position_steps=-744, actual_position_steps=-744, trajectory=asdict(plan))
            new, reason = adjust(plan, row, [row], .01, 0, contact, settings, cycles, native.PROFILE)
            self.assertIsNone(new, (guard, reason))

    def test_1n_pressure_position_precedes_speed_and_duplicate_exploration_keeps_speed(self):
        settings, cycles = default_settings(), CycleSettings(trajectory_learning=True)
        contact = dict(median_position_steps=-621, last_zero_boundary_steps=-605)
        plan = bounded(Trajectory(-744, 30, -151, 5, 100, 100), 0, contact, settings, cycles, native.PROFILE)
        row = dict(target_n=1, tolerance_n=.05, measured_force_n=.70, peak_n=.82,
                   guard_reason=None, arrival=dict(min_force_n=.69, max_force_n=.72),
                   candidate_position_steps=-744, trajectory=asdict(plan), pressure_ok=False, position_arrival_s=2)
        new, reason = adjust(plan, row, [row], .01, 0, contact, settings, cycles, native.PROFILE)
        self.assertLess(new.position_steps, plan.position_steps)
        self.assertEqual(replace(new, position_steps=plan.position_steps), plan)
        duplicate = dict(row, measured_force_n=None, trajectory=asdict(new))
        alternate, _ = adjust(plan, row, [duplicate, row], .01, 0, contact, settings, cycles, native.PROFILE)
        self.assertLess(new.position_steps, alternate.position_steps)
        self.assertLess(alternate.position_steps, plan.position_steps)
        self.assertEqual(replace(alternate, position_steps=plan.position_steps), plan)
        good = dict(row, measured_force_n=1, peak_n=1.03, pressure_ok=True,
                    arrival=dict(min_force_n=.99, max_force_n=1.01))
        faster, reason = adjust(plan, good, [good], .01, 0, contact, settings, cycles, native.PROFILE)
        self.assertEqual(faster.position_steps, plan.position_steps)
        self.assertGreater(faster.loaded_speed_rpm, plan.loaded_speed_rpm)
        self.assertIn('压力已合格', reason)
        unconfirmed = dict(good, pressure_ok=False)
        self.assertIsNone(adjust(plan, unconfirmed, [unconfirmed], .01, 0, contact, settings, cycles, native.PROFILE)[0])

    def test_automatic_contact_remains_available(self):
        plant, doc = self.learn(manual=False)
        self.assertEqual(len(doc['contacts']), 10, doc['message'])
        self.assertTrue(doc['complete'], doc['message'])
        self.assertEqual(plant.position, 0)

    def test_stop_and_pressure_loss_release_during_motion_observation_and_return(self):
        for phase in ('连续轨迹寻压', '轨迹到位观察', 'return'):
            for fault in ('stop', 'stale'):
                injected = []
                def progress(plant, row):
                    if row.get('phase') == phase and plant.position < -50 and not injected:
                        injected.append(True)
                        if fault == 'stop': plant.cancelled = True
                        else: plant.fault = 'stale'
                    if fault == 'stale' and injected and row.get('phase') == 'return': plant.cancelled = True
                plant, doc = self.learn(progress=progress)
                self.assertTrue(injected, (phase, fault))
                self.assertFalse(doc['complete'])
                self.assertTrue(doc['release_position_confirmed'], doc['message'])
                self.assertEqual(plant.position, 0)

    def test_emergency_during_motion_never_returns_or_resumes(self):
        def progress(plant, row):
            if row.get('phase') == '连续轨迹寻压' and plant.position < -50: plant.emergency = True
        plant, doc = self.learn(progress=progress)
        self.assertTrue(doc['emergency_stopped'])
        self.assertLess(plant.position, 0)
        self.assertEqual(plant.actions, [])

    def test_manual_coordinate_range_checked_before_motion(self):
        plant = MovingPlant()
        with self.assertRaises(ValueError):
            run_relearning(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant), default_settings(),
                0, self.seed, cycles=CycleSettings(trajectory_learning=True, manual_contact_position=10))
        self.assertEqual(plant.absolute_commands, [])

    def test_saved_trajectory_is_used_by_pressure_button(self):
        plant, doc = self.learn()
        self.assertTrue(doc['complete'], doc['message'])
        plant.absolute_commands.clear()
        result = run_trial(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            replace(default_settings(), hold_s=1), 0, doc, 2, mode='cycles', clock=lambda: plant.now)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(result['trajectory'], doc['nodes'][2]['trajectory'])
        self.assertTrue(all(c['target'] == doc['nodes'][2]['position_steps'] for c in plant.absolute_commands))

    def test_checkpoint_failure_preserves_memory_and_releases(self):
        def denied(doc): raise PermissionError('busy')
        plant, doc = self.learn(checkpoint=denied)
        self.assertTrue(doc['complete'], doc['message'])
        self.assertTrue(doc['checkpoint_errors'])
        self.assertEqual(plant.position, 0)

    def test_motor_feedback_loss_waits_for_recovery_before_release(self):
        for phase in ('连续轨迹寻压', '轨迹到位观察', 'return'):
            injected, recovered = [], []
            def setup(plant):
                read = plant.read_feedback
                def broken():
                    if plant.fault == 'motor_link': raise RuntimeError('CAN暂时断开')
                    return read()
                plant.read_feedback = broken
            def progress(plant, row):
                if row.get('phase') == phase and plant.position < -50 and not injected:
                    plant.fault = 'motor_link'
                    injected.append(True)
                if injected and row.get('phase') == 'recovery' and row.get('recovery_attempt', 0) >= 2:
                    recovered.append(True)
                    plant.fault, plant.cancelled = None, True
            plant, doc = self.learn(setup=setup, progress=progress)
            self.assertTrue(injected, phase)
            self.assertTrue(recovered, phase)
            self.assertTrue(doc['release_position_confirmed'], doc['message'])
            self.assertEqual(plant.position, 0)

    def test_dynamic_force_overshoot_is_never_accepted_and_all_parameters_are_archived(self):
        plant, doc = self.learn(setup=lambda p: setattr(p, 'dynamic_gain', .1), full_window=False)
        attempts = [r for p in doc['points'] for r in p['attempts']]
        self.assertTrue(any(r['guard_reason'] == 'overshoot' for r in attempts))
        for r in attempts:
            if r['ok']: self.assertLessEqual(r['peak_n'], r['target_n']+r['tolerance_n']+1e-9)
            self.assertTrue(r['trace'])
            self.assertTrue(r['commands'])
            self.assertTrue(r['adjustment_reason'])
        self.assertEqual(plant.position, 0)

    def test_unsafe_manual_contact_triggers_early_contact_and_release(self):
        plant, doc = self.learn(setup=lambda p: setattr(p, 'contact', 100))
        self.assertFalse(doc['complete'])
        self.assertEqual(len(doc['points'][0]['attempts']), 1)
        self.assertEqual(doc['points'][0]['attempts'][0]['guard_reason'], 'early_contact')
        self.assertEqual(plant.position, 0)

    def test_tampered_fast_trajectory_or_time_cannot_load(self):
        _, doc = self.learn()
        for corrupt in ('time', 'trajectory'):
            wrong = deepcopy(doc)
            if corrupt == 'time': wrong['points'][0]['attempts'][-1]['position_arrival_s'] = 1.1
            else: wrong['nodes'][1]['trajectory']['free_speed_rpm'] += 1
            with self.assertRaises(ValueError): validate_document(wrong, native.PROFILE)

    def test_overcurrent_waits_before_any_return_command(self):
        injected, return_currents = [], []
        def setup(plant):
            move = plant.move_relative_steps
            def guarded(delta, **kwargs):
                return_currents.append(plant.current)
                return move(delta, **kwargs)
            plant.move_relative_steps = guarded
        def progress(plant, row):
            if row.get('phase') == '连续轨迹寻压' and plant.position < -100 and not injected:
                injected.append(True)
                plant.current = 4000
            if injected and row.get('phase') == 'recovery' and row.get('recovery_attempt', 0) >= 2:
                plant.current, plant.cancelled = 200, True
        plant, doc = self.learn(setup=setup, progress=progress)
        self.assertTrue(injected)
        self.assertEqual(plant.position, 0)
        self.assertTrue(return_currents)
        self.assertTrue(all(v <= 3000 for v in return_currents))
        self.assertFalse(doc['complete'])

    def test_impossible_one_second_is_reported_instead_of_accepting_slow_pressure_success(self):
        plant = MovingPlant()
        cycles = CycleSettings(trajectory_learning=True, manual_contact_position=-600, fast_speed=1, medium_speed=1)
        doc = run_relearning(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            default_settings(), 0, self.seed, cycles=cycles, clock=lambda: plant.now)
        self.assertFalse(doc['complete'])
        rows = doc['points'][0]['attempts']
        self.assertLessEqual(len(rows), 20)
        self.assertTrue(any(r['pressure_ok'] and not r['time_ok'] for r in rows))
        self.assertFalse(any(r['ok'] for r in rows))
        self.assertEqual(len({tuple(r['trajectory'].values()) for r in rows}), len(rows))
        self.assertEqual(plant.position, 0)


class TrajectoryUiTests(unittest.TestCase):
    page = native.FeedbackSettingsUiTests.page
    setUpClass = classmethod(trial_tests.TrialUiTests.setUpClass.__func__)
    setUp = trial_tests.TrialUiTests.setUp

    def test_release_side_offset_allows_table_confirmation_restart_and_release_button(self):
        w, plant, tasks = self.widget, self.plant, []
        w._document['working_release_position_steps'] = -717
        plant.position, plant.force_override = -714, 0
        plant.now += 1
        plant.read_feedback()
        w._refresh()
        w.confirm_table()
        self.assertTrue(w._table_confirmed, w.status_label.text())
        self.assertEqual(w._working_release, -717)
        with patch.object(w.control, 'start_external_operation',
                          side_effect=lambda task, name: (tasks.append(task) or False, '测试不启动电机')):
            w.start_cycles()
            self.assertEqual(len(tasks), 1, w.status_label.text())
            w.start(release_only=True)
            self.assertEqual(len(tasks), 2, w.status_label.text())

    def test_return_zero_moves_to_absolute_zero_and_preserves_learning_with_pressure_missing(self):
        from high_force_calibration import recover_release
        w, plant, tasks = self.widget, self.plant, []
        w.confirm_table()
        document = w._document
        w._working_release = -613
        def virtual(motor, profile, samples, settings, origin, emergency, **kwargs):
            self.assertEqual(origin, 0)
            self.assertTrue(kwargs['return_even_within_tolerance'])
            return recover_release(motor, profile, samples, settings, origin,
                ClockCancel(plant, emergency.is_set), clock=lambda: plant.now, **kwargs)
        for position in (-613, -8):
            plant.position, plant.fault = position, 'stale'
            plant.now += 3
            with patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
                 patch.object(w.control, 'zero_motor_position') as reset, \
                 patch('one_step_learning_widget.recover_release', virtual):
                w.return_zero_button.click()
                self.assertTrue(w._running)
                self.assertFalse(w.return_zero_button.isEnabled())
                tasks[-1](plant, ClockCancel(plant))
                reset.assert_not_called()
            self.assertEqual(plant.position, 0)
            self.assertEqual(w._origin, 0)
            self.assertEqual(w._working_release, -613)
            self.assertIs(w._document, document)
            self.assertIn('已回到0度位置', w.status_label.text())
        self.assertEqual(plant.actions, [613, 8])

    def test_return_zero_interlocks_and_no_downward_return(self):
        w, tasks = self.widget, []
        with patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')):
            w._origin = None
            w._refresh()
            self.assertTrue(w.return_zero_button.isEnabled())
            self.assertEqual(tasks, [])
            w.confirm_table()
            with patch.object(w.control, 'is_operation_busy', return_value=True):
                w.start(return_zero=True)
                self.assertFalse(w.return_zero_button.isEnabled())
                self.assertEqual(tasks, [])
            self.plant.position = 10
            w.start(return_zero=True)
            self.assertEqual(tasks, [])
            self.assertIn('需要向下运动', w.status_label.text())
            w._coordinates_invalidated()
            self.assertFalse(w.return_zero_button.isEnabled())
            self.assertIn('坐标已失效', w.return_zero_hint.text())

    def test_return_zero_without_learning_origin_or_pressure_after_restart(self):
        from high_force_calibration import recover_release
        w, plant, tasks = self.widget, self.plant, []
        self.assertIsNone(w._origin)
        w._document = w._seed_document = None
        plant.position, plant.fault = -613, 'stale'
        plant.now += 3
        w._refresh()
        self.assertTrue(w.return_zero_button.isEnabled())
        self.assertFalse(w.capture_button.isEnabled())
        def virtual(motor, profile, samples, settings, origin, emergency, **kwargs):
            self.assertEqual(origin, 0)
            return recover_release(motor, profile, samples, settings, origin,
                ClockCancel(plant, emergency.is_set), clock=lambda: plant.now, **kwargs)
        with patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
             patch('one_step_learning_widget.recover_release', virtual), \
             patch.object(w.control, 'zero_motor_position') as reset:
            w.return_zero_button.click()
            self.assertEqual(len(tasks), 1)
            tasks[0](plant, ClockCancel(plant))
            reset.assert_not_called()
        self.assertEqual(plant.actions, [613])
        self.assertEqual(plant.position, 0)
        self.assertIsNone(w._origin)
        self.assertIsNone(w._document)

    def test_return_zero_rejects_untrusted_motor_feedback_and_displays_reason(self):
        w, plant, tasks = self.widget, self.plant, []
        cases = [dict(sampled_monotonic=plant.now-10), dict(position_counts=float('nan')),
                 dict(phase_current_ma=4000), dict(running_state=3), dict(measured_speed_rpm=5),
                 dict(position_counts=1), dict(position_counts=-w.inputs['max_travel'].value()-1)]
        for changes in cases:
            sample = dict(plant.read_feedback(), **changes)
            with self.subTest(changes=changes), \
                 patch.object(w.control, 'latest_motor_feedback', return_value=sample), \
                 patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')):
                w.start(return_zero=True)
                self.assertFalse(w.return_zero_button.isEnabled())
                self.assertEqual(tasks, [])
                self.assertEqual(w.status_label.text(), w.return_zero_hint.text())
                self.assertNotIn('无需先设定', w.return_zero_hint.text())

    def test_return_zero_stop_releases_and_emergency_prevents_motion(self):
        from high_force_calibration import recover_release
        w, plant, tasks = self.widget, self.plant, []
        w.confirm_table()
        for urgent in (False, True):
            plant.position = -613
            before = len(plant.actions)
            injected = []
            def virtual(motor, profile, samples, settings, origin, emergency, **kwargs):
                progress = kwargs.pop('progress')
                def notify(row):
                    progress(row)
                    if row['phase'] == 'recovery_release' and not injected:
                        injected.append(True)
                        w.emergency_stop() if urgent else w.stop()
                return recover_release(motor, profile, samples, settings, origin,
                    ClockCancel(plant, emergency.is_set), clock=lambda: plant.now, progress=notify, **kwargs)
            with patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
                 patch.object(w.control, 'request_external_stop'), \
                 patch('one_step_learning_widget.recover_release', virtual):
                w.return_zero_button.click()
                tasks[-1](plant, ClockCancel(plant))
            self.assertTrue(injected)
            self.assertEqual(plant.position, -613 if urgent else 0)
            if urgent: self.assertEqual(len(plant.actions), before)

    def test_new_learning_can_start_from_original_zero_after_leaving_working_point(self):
        w, tasks = self.widget, []
        w.confirm_table()
        w._working_release = -613
        self.plant.position = 0
        with patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')):
            w.start_cycles()
        self.assertEqual(len(tasks), 1, w.status_label.text())

    def test_communication_interruption_is_displayed_separately_from_parameter_failure(self):
        w = self.widget
        row = dict(stage='pressure', target_n=2, attempt=6, candidate_position_steps=-839,
            actual_position_steps=-613, measured_force_n=None, peak_n=.02, error_n=None,
            medium_speed_rpm=1, ok=False, trajectory=dict(position_steps=-839, free_speed_rpm=1,
                loaded_speed_rpm=1, decel_position_steps=-613, acceleration=100, deceleration=75),
            adjustment_reason='通信中断，不占学习轮次', message='通信中断，不占学习轮次',
            record_kind='communication_interruption', retry_scheduled=True, release_position_confirmed=True)
        w._show_attempt(row)
        self.assertEqual(w.attempt_table.item(0, 7).text(), '通信中断')
        self.assertEqual(w.attempt_table.item(0, 8).text(), '等待重试')
        w._show_attempt(dict(row, retry_scheduled=False))
        self.assertEqual(w.attempt_table.item(1, 8).text(), '已结束')

    def test_manual_checkbox_freezes_input_saves_source_and_reloads_unchecked(self):
        w, tasks = self.widget, []
        self.assertEqual(w.inputs['tolerance_n'].value(), .1)
        w.confirm_table()
        w.manual_contact_input.setValue(-10)
        w.use_manual_contact.setChecked(True)
        self.plant.move_absolute_steps = lambda pos, **kw: self.plant.move_relative_steps(pos-self.plant.position, **kw)
        def virtual(motor, profile, samples, stop, emergency, settings, origin, seed, **kwargs):
            self.assertTrue(kwargs['cycles'].trajectory_learning)
            self.assertEqual(kwargs['cycles'].arrival_target_s, 2.0)
            self.assertEqual(kwargs['cycles'].speed_step_rpm, .1)
            self.assertFalse(kwargs['cycles'].require_full_observation_band)
            self.assertEqual(kwargs['cycles'].manual_contact_position, -10)
            return run_relearning(motor, profile, samples, ClockCancel(self.plant, stop.is_set),
                ClockCancel(self.plant, emergency.is_set), settings, origin, seed, clock=lambda: self.plant.now, **kwargs)
        with patch.object(w.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
             patch('one_step_learning_widget.cycles_learning.run_relearning', virtual):
            w.start_cycles()
            self.assertFalse(w.manual_contact_input.isEnabled())
            self.assertFalse(w.use_manual_contact.isEnabled())
            tasks[0](self.plant, ClockCancel(self.plant))
        self.assertTrue(w._document['complete'], w.status_label.text())
        self.assertEqual(w._document['contacts'], [])
        self.assertTrue(w._document['cycle_settings']['near_contact_release'])
        self.assertEqual(w._document['cycle_settings']['arrival_target_s'], 2.0)
        self.assertEqual(w._document['cycle_settings']['speed_step_rpm'], .1)
        self.assertFalse(w._document['cycle_settings']['require_full_observation_band'])
        self.assertEqual(w._document['cycle_settings']['overshoot_absolute_n'], .20)
        self.assertEqual(w._document['cycle_settings']['overshoot_relative_tolerance'], .01)
        altered = deepcopy(w._document)
        final = altered['points'][3]['attempts'][-1]
        final['arrival']['samples'][0]['force_n'] = 4-altered['nodes'][4]['tolerance_n']-.01
        final['arrival']['min_force_n'] = final['arrival']['samples'][0]['force_n']
        validate_document(altered, w.control.profile)
        relaxed = deepcopy(altered)
        relaxed_final = relaxed['points'][3]['attempts'][-1]
        relaxed_final['peak_n'] = 4.15
        relaxed_final['arrival']['samples'][1]['force_n'] = 4.15
        relaxed_final['arrival']['max_force_n'] = 4.15
        validate_document(relaxed, w.control.profile)
        legacy = deepcopy(relaxed)
        legacy['cycle_settings'].pop('overshoot_absolute_n')
        with self.assertRaises(ValueError): validate_document(legacy, w.control.profile)
        relaxed_final['peak_n'] = 4.21
        with self.assertRaises(ValueError): validate_document(relaxed, w.control.profile)
        overshoot = deepcopy(altered)
        overshoot['points'][3]['attempts'][-1]['arrival']['samples'][1]['force_n'] = 4.11
        with self.assertRaises(ValueError): validate_document(overshoot, w.control.profile)
        altered['cycle_settings'].pop('require_full_observation_band')
        with self.assertRaises(ValueError): validate_document(altered, w.control.profile)
        self.assertEqual(w.cycle_inputs['medium_speed'].decimals(), 1)
        self.assertEqual(w.cycle_inputs['medium_speed'].singleStep(), .1)
        self.assertEqual(w._document['settings']['tolerance_n'], .1)
        self.assertEqual(w._document['nodes'][2]['tolerance_n'], .1)
        self.assertEqual(w._document['nodes'][30]['tolerance_n'], .3)
        self.assertEqual(w._working_release, -10)
        self.assertIn('工作释放点', w.origin_label.text())
        self.assertEqual(w.attempt_table.columnCount(), 17)
        self.assertIn('手动', w.contact_label.text())
        self.assertEqual(w.manual_contact_input.suffix(), '')
        self.assertEqual(w.manual_contact_input.prefix(), '')
        restored = OneStepLearningWidget(w.control, self.plant.samples, output_dir=self.directory)
        self.addCleanup(restored.shutdown)
        self.addCleanup(restored.deleteLater)
        self.assertFalse(restored.use_manual_contact.isChecked())
        self.assertEqual(restored.inputs['tolerance_n'].value(), .1)
        self.assertIsNone(restored._working_release)
        self.assertEqual(restored.manual_contact_input.value(), -10)
        w._coordinates_invalidated()
        self.assertFalse(w.use_manual_contact.isChecked())
        self.assertIsNone(w._working_release)

    def test_invalid_manual_input_is_rejected_before_starting_worker(self):
        w = self.widget
        w.confirm_table()
        w.manual_contact_input.setValue(5)
        w.use_manual_contact.setChecked(True)
        with patch.object(w.control, 'start_external_operation') as start:
            w.start_cycles()
            start.assert_not_called()
        self.assertIn('允许行程', w.status_label.text())


if __name__ == '__main__': unittest.main()
