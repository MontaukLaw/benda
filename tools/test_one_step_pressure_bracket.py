"""实测3 N位置振荡复现及末端降速的离线运动/恢复验证，不连接硬件。"""
from copy import deepcopy
from dataclasses import asdict, replace
import json
from pathlib import Path
import unittest

from test_one_step_trajectory import MovingPlant, native, ClockCancel
from pressure_feedback_control import FeedbackSettings
from one_step_relearning import CycleSettings
from one_step_trajectory import Trajectory, TrajectoryController, adjust, bounded, validate_plan, learn_point
from high_force_calibration import recover_release
from one_step_repeatability import CombinedCancel


class PressureBracketTests(unittest.TestCase):
    def setUp(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/one_step_3n_20260922.json').read_text(encoding='utf-8'))
        self.rows = fixture['rows']
        self.settings = replace(FeedbackSettings(**fixture['settings']), target_n=3)
        self.cycles = CycleSettings(**fixture['cycle_settings'])
        self.contact = fixture['contact']
        self.origin = -717

    def adjust(self, row, history):
        return adjust(Trajectory(**row['trajectory']), row, history, .014, self.origin,
                      self.contact, self.settings, self.cycles, native.PROFILE)

    def test_recorded_3n_overshoot_bisects_without_changing_motion_parameters(self):
        plan, reason = self.adjust(self.rows[-1], self.rows)
        self.assertEqual(plan.position_steps, -1009)
        self.assertEqual(replace(plan, position_steps=-1013), Trajectory(**self.rows[-1]['trajectory']))
        self.assertIn('夹逼', reason)
        self.assertNotIn('提前减速', reason)

    def terminal_curve(self):
        history = deepcopy(self.rows)
        plan, _ = self.adjust(history[-1], history)
        # 中点仍过冲时，下一轮只能在更小区间内试；以下是明确标注的合成分支。
        high = dict(history[-1], trajectory=asdict(plan), candidate_position_steps=plan.position_steps, peak_n=3.11)
        history.append(high)
        plan, _ = self.adjust(high, history)
        self.assertEqual(plan.position_steps, -1007)
        low = dict(history[0], trajectory=asdict(plan), candidate_position_steps=plan.position_steps)
        history.append(low)
        plan, _ = self.adjust(low, history)
        self.assertEqual(plan.position_steps, -1008)
        low = dict(low, trajectory=asdict(plan), candidate_position_steps=plan.position_steps)
        history.append(low)
        curve, reason = self.adjust(low, history)
        self.assertEqual(curve.position_steps, -1008)
        self.assertEqual(curve.free_speed_rpm, 4.5)
        self.assertEqual(curve.loaded_speed_rpm, 4.4)
        self.assertEqual((curve.acceleration, curve.deceleration), (100, 25))
        self.assertLess(curve.position_steps, curve.decel_position_steps)
        self.assertLess(curve.decel_position_steps, self.origin)
        self.assertIn('末段速度', reason)
        validate_plan(curve, self.origin, self.contact, self.settings, self.cycles, native.PROFILE)
        return curve

    def test_bracket_narrows_and_then_explores_terminal_speed(self):
        self.terminal_curve()

    def test_incompatible_and_interrupted_evidence_cannot_form_bracket(self):
        for mutate in (lambda r: r['trajectory'].update(loaded_speed_rpm=4.4),
                       lambda r: r.update(target_n=2), lambda r: r.update(tolerance_n=.05),
                       lambda r: r.update(release_position_steps=-700),
                       lambda r: r.update(learning_evidence_valid=False),
                       lambda r: r.update(guard_reason='position_timeout')):
            history = deepcopy(self.rows)
            mutate(history[0])
            _, reason = self.adjust(history[-1], history)
            self.assertNotIn('之间夹逼', reason)

    def test_conflicting_evidence_and_missing_samples_do_not_provoke_motion(self):
        history = deepcopy(self.rows)
        history[0]['trajectory']['position_steps'] = -1015
        plan, reason = self.adjust(history[-1], history)
        self.assertIsNone(plan)
        self.assertIn('证据交叉', reason)
        interrupted = dict(self.rows[-1], guard_reason=None, peak_n=2.95)
        self.assertIsNone(self.adjust(interrupted, self.rows+[interrupted])[0])

    def run_curve(self, phase=None, fault=None):
        curve = self.terminal_curve()
        plant = MovingPlant()
        plant.x = plant.position = self.origin
        plant.contact = -self.origin
        plant.force = lambda: max(0, (self.origin-plant.x)*3/(self.origin-curve.position_steps))
        emergency = ClockCancel(plant, lambda: plant.emergency)
        cancel = CombinedCancel(plant, emergency)
        injected = []
        def record(row):
            if row.get('phase') == phase and plant.position < self.origin-20 and not injected:
                injected.append(True)
                if fault == 'stop': plant.cancelled = True
                elif fault == 'stale': plant.fault = 'stale'
                elif fault == 'emergency': plant.emergency = True
        controller = TrajectoryController(plant, native.PROFILE, plant.samples, cancel, self.settings, self.origin,
            node=dict(position_steps=curve.position_steps, measured_force_n=3), contact=self.contact,
            cycle_settings=self.cycles, trajectory=curve, on_record=record, clock=lambda: plant.now)
        result = controller.run()
        release = recover_release(plant, native.PROFILE, plant.samples, self.settings, self.origin,
            emergency, progress=lambda row: record(dict(row, phase='return')),
            clock=lambda: plant.now, return_even_within_tolerance=True)
        return plant, controller, result, release, injected

    def test_terminal_curve_switches_while_moving_without_changing_endpoint(self):
        plant, controller, result, release, _ = self.run_curve()
        self.assertTrue(result['ok'], result)
        commands = plant.absolute_commands
        self.assertEqual(len(commands), 2)
        self.assertEqual([c['speed'] for c in commands], [4.5, 4.4])
        self.assertEqual(commands[0]['target'], commands[1]['target'])
        self.assertGreater(commands[1]['velocity'], 0)
        self.assertEqual(commands[0]['stops'], commands[1]['stops'])
        self.assertEqual([c['acceleration'] for c in commands], [25, 25])
        self.assertLess(commands[1]['position'], self.origin)
        self.assertLessEqual(controller.position_arrival_s, 2)
        self.assertTrue(release['release_position_confirmed'])
        self.assertEqual(plant.position, self.origin)

    def test_full_learning_converges_between_underpressure_and_overshoot(self):
        # 合成速度相关瞬态压力，核对真实学习循环和每轮回位，不声称复现真实材料。
        plant = MovingPlant()
        plant.x = plant.position = self.origin
        plant.force = lambda: max(0, (-plant.x-863)*.02)+(.06*plant.velocity/(3200/60) if plant.x < -863 else 0)
        point = dict(attempts=[], converged=False)
        releases = []
        def release():
            result = recover_release(plant, native.PROFILE, plant.samples, self.settings, self.origin,
                ClockCancel(plant), clock=lambda: plant.now, return_even_within_tolerance=True)
            releases.append(plant.position)
            return result
        node = learn_point(plant, native.PROFILE, plant.samples, plant, self.settings, self.origin, 3, .014,
            -1006, self.contact, self.cycles, dict(trajectory=self.rows[0]['trajectory']), point,
            lambda rest: plant.wait(5), release, lambda: None, lambda r: None, lambda r: None,
            lambda r: None, lambda: plant.now)
        self.assertEqual([r['candidate_position_steps'] for r in point['attempts']], [-1006, -1013, -1009])
        self.assertEqual(point['attempts'][1]['guard_reason'], 'overshoot')
        self.assertEqual(point['attempts'][1]['parameter_changes'], {'position_steps': {'before': -1013, 'after': -1009}})
        self.assertTrue(point['converged'])
        self.assertLessEqual(node['position_arrival_s'], 2)
        self.assertEqual(point['best_pressure_candidate']['trajectory'], node['trajectory'])
        self.assertEqual(releases, [self.origin]*3)

    def test_curve_stop_and_pressure_loss_release_in_all_phases(self):
        for phase in ('连续轨迹寻压', '轨迹到位观察', 'return'):
            for fault in ('stop', 'stale'):
                with self.subTest(phase=phase, fault=fault):
                    plant, _, _, release, injected = self.run_curve(phase, fault)
                    self.assertTrue(injected)
                    self.assertTrue(release['release_position_confirmed'], release)
                    self.assertEqual(plant.position, self.origin)

    def test_curve_emergency_never_returns(self):
        plant, _, _, release, injected = self.run_curve('连续轨迹寻压', 'emergency')
        self.assertTrue(injected)
        self.assertFalse(release['release_position_confirmed'])
        self.assertLess(plant.position, self.origin)
        self.assertEqual(plant.actions, [])

    def test_short_curve_falls_back_to_safe_single_speed(self):
        plan = Trajectory(-720, 4.5, -719, 4.4, 100, 25)
        safe = bounded(plan, self.origin, self.contact, self.settings, self.cycles, native.PROFILE)
        self.assertEqual(safe.free_speed_rpm, safe.loaded_speed_rpm)
        self.assertEqual(safe.decel_position_steps, self.origin)
        validate_plan(safe, self.origin, self.contact, self.settings, self.cycles, native.PROFILE)


if __name__ == '__main__':
    unittest.main()
