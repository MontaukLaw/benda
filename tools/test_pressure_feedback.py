"""闭环模拟验证：材料恢复偏移、反馈失效、运动故障及真实后台互锁。"""

from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "PY_SRIPT/algo_monitor"))
sys.path.insert(0, str(ROOT / "tools"))

from pressure_feedback_control import (
    FeedbackControlError, FeedbackSettings, PressureSamples, PressureFeedbackController, LearnedPressureMap,
    pressure_tolerance_n,
)
from motor_mapping_interface import MappingProfile, create_mapping_control


PROFILE = MappingProfile("zdt_x42s", 3200, "档", 255, "RPM", 3000)


class Plant:
    """模拟带空行程的直压材料；虚拟时间可注入回弹、通信和运动故障。"""

    def __init__(self):
        self.now = 10.0
        self.position = 0
        self.samples = PressureSamples()
        self.actions = []
        self.motion_parameters = []
        self.cancelled = False
        self.stop_count = 0
        self.current = 200
        self.fault = None
        self.drift = lambda: 0
        self.force_override = None
        self.cancel_at = math.inf
        self.last_sample_time = 0
        self.records = []

    def is_set(self):
        return self.cancelled or self.now >= self.cancel_at

    def wait(self, seconds):
        self.now += seconds
        return self.is_set()

    def force(self):
        return max(0, (-self.position-10)*.04-self.drift())

    def read_feedback(self):
        force = self.force() if self.force_override is None else self.force_override
        if self.fault != "stale" and self.now-self.last_sample_time >= .049:
            self.samples.update(dict(force_n=force, force_unit="N", accepted=True,
                                     sequence=round(self.now*1000), sampled_monotonic=self.now))
            self.last_sample_time = self.now
        return dict(position_counts=self.position, measured_speed_rpm=0,
                    phase_current_ma=self.current, running_state=0, in_position=True,
                    sampled_monotonic=self.now)

    def prepare_motion(self):
        pass

    def set_current_limit_ma(self, value):
        pass

    def move_relative_steps(self, steps, **kwargs):
        self.actions.append(steps)
        self.motion_parameters.append(dict(kwargs))
        if self.fault == "wrong_direction":
            self.position -= steps
        elif self.fault != "stalled":
            self.position += steps
        if self.fault == "lost_ack":
            raise RuntimeError("应答丢失")

    def stop(self):
        self.stop_count += 1
        if self.fault == "stop_failed":
            raise RuntimeError("停止失联")

    def run(self, release=False, release_position=0, learning=False, learned_map=None, **changes):
        parameters = dict(max_step=8, hold_s=5)
        parameters.update(changes)
        settings = replace(FeedbackSettings(), **parameters)
        self.controller = PressureFeedbackController(
            self, PROFILE, self.samples, self, settings, release_position,
            on_record=self.records.append, clock=lambda: self.now, learned_map=learned_map)
        return self.controller.run(release, learning=learning)


class FeedbackTests(unittest.TestCase):
    def test_reaches_target_and_compensates_material_drift(self):
        plant = Plant()
        plant.drift = lambda: .3 if plant.now >= 15 else 0
        result = plant.run()
        self.assertTrue(result['ok'], result)
        self.assertLessEqual(abs(plant.force()-1), .05)
        self.assertLess(plant.position, -40)
        self.assertGreater(result['hold_sample_count'], 10)
        self.assertEqual(plant.stop_count, 1)
        self.assertTrue(all(abs(step) <= 8 for step in plant.actions))

    def test_overshoot_is_corrected_with_upward_motion(self):
        plant = Plant()
        # 进入保持后压力增加，必须回退而不是继续下压。
        plant.drift = lambda: -.3 if plant.now >= 15 else 0
        result = plant.run()
        self.assertTrue(result['ok'], result)
        self.assertTrue(any(step > 0 for step in plant.actions))
        self.assertLessEqual(abs(plant.force()-1), .05)

    def test_release_only_moves_up_and_checks_new_zero_samples(self):
        plant = Plant()
        plant.position = -35
        result = plant.run(release=True)
        self.assertTrue(result['ok'], result)
        self.assertTrue(all(step > 0 for step in plant.actions))
        self.assertLessEqual(abs(plant.position), 3)
        self.assertEqual(plant.force(), 0)

    def test_release_returns_in_one_command_at_independent_speed(self):
        plant = Plant()
        plant.position = -820
        result = plant.run(release=True, release_position=5, max_step=1, speed=5, release_speed=60)
        self.assertTrue(result['ok'], result)
        self.assertEqual(plant.actions, [825])
        self.assertEqual(plant.position, 5)
        self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 60)
        # 闭环调节仍使用调节速度及最大步数，不使用释放速度。
        plant = Plant()
        self.assertTrue(plant.run(speed=5, release_speed=60)['ok'])
        self.assertTrue(all(p['speed_rpm'] == 5 for p in plant.motion_parameters))
        self.assertTrue(all(abs(step) <= 8 for step in plant.actions))

    def test_slow_release_uses_travel_timeout_instead_of_acquisition_timeout(self):
        plant = Plant()
        plant.position = -820
        original_read = plant.read_feedback
        def move(steps, **kwargs):
            plant.actions.append(steps)
            plant.motion_parameters.append(kwargs)
            plant.arrival = plant.now+12
        def read():
            if plant.now >= getattr(plant, 'arrival', math.inf):
                plant.position = 0
            sample = original_read()
            if plant.actions and plant.position < 0:
                sample.update(measured_speed_rpm=1, in_position=False)
            return sample
        plant.move_relative_steps = move
        plant.read_feedback = read
        result = plant.run(release=True, release_speed=1, acquire_timeout_s=10)
        self.assertTrue(result['ok'], result)
        self.assertEqual(plant.actions, [820])
        self.assertGreater(plant.now-plant.controller.started, 12)

    def test_release_reaches_position_despite_zero_negative_or_missing_pressure(self):
        for mode in ('zero', 'negative', 'missing', 'persistent_negative', 'persistent_missing'):
            with self.subTest(mode=mode):
                plant = Plant()
                plant.position = -820
                original_read = plant.read_feedback
                def move(steps, **kwargs):
                    plant.actions.append(steps)
                    plant.motion_parameters.append(kwargs)
                    plant.motion_started = plant.now
                def read():
                    if plant.actions:
                        plant.position = min(2, -820+round((plant.now-plant.motion_started)*400))
                    # 压力早于实际回位归零，或回弹为负值/断流；回位途中都不能刹停。
                    plant.force_override = -.25 if 'negative' in mode else 0
                    plant.fault = 'stale' if 'missing' in mode else None
                    if plant.position == 2 and not mode.startswith('persistent'):
                        plant.force_override, plant.fault = 0, None
                    sample = original_read()
                    if plant.actions and plant.position != 2:
                        sample.update(measured_speed_rpm=24, in_position=False)
                    return sample
                plant.move_relative_steps = move
                plant.read_feedback = read
                result = plant.run(release=True, release_position=2)
                self.assertEqual(plant.actions, [822])
                self.assertEqual(plant.position, 2)
                self.assertEqual(plant.stop_count, 1)
                self.assertTrue(result['stop_confirmed'])
                self.assertEqual(result['ok'], not mode.startswith('persistent'), result)
                if mode.startswith('persistent'):
                    self.assertTrue('释放后检查失败' in result['message'] or '已到释放位置' in result['message'])

    def test_release_errors_do_not_issue_retry_or_downward_commands(self):
        for case in ('above_release', 'beyond_travel', 'lost_ack', 'not_unloaded', 'stalled'):
            with self.subTest(case=case):
                plant = Plant()
                plant.position = -820
                if case == 'above_release':
                    plant.position = 17
                elif case == 'beyond_travel':
                    plant.position = -7000
                elif case == 'not_unloaded':
                    plant.force_override = 1
                else:
                    plant.fault = case
                result = plant.run(release=True)
                self.assertFalse(result['ok'], result)
                self.assertTrue(all(step > 0 for step in plant.actions))
                self.assertLessEqual(len(plant.actions), 1)
                self.assertEqual(plant.stop_count, 1)

    def test_release_remains_interruptible_during_single_long_move(self):
        plant = Plant()
        plant.position = -820
        plant.fault = 'stalled'
        plant.cancel_at = plant.now+.2
        result = plant.run(release=True)
        self.assertTrue(result.get('cancelled'), result)
        self.assertEqual(plant.actions, [820])
        self.assertEqual(plant.stop_count, 1)

    def test_force_zero_is_not_enough_to_start_away_from_release(self):
        plant = Plant()
        plant.position = -20
        plant.force_override = 0
        result = plant.run()
        self.assertFalse(result['ok'])
        self.assertEqual(plant.actions, [])

    def test_release_offset_allows_next_cycle_without_changing_reference(self):
        plant = Plant()
        original_move = plant.move_relative_steps
        def move(steps, **kwargs):
            original_move(steps, **kwargs)
            if plant.controller.releasing:
                plant.position += 8
        plant.move_relative_steps = move
        for _ in range(3):
            plant.position = -832
            released = plant.run(release=True, release_position=13)
            self.assertTrue(released['ok'], released)
            self.assertEqual(plant.position, 21)
            self.assertEqual(released['release_error_steps'], 8)
            self.assertEqual(plant.controller.release, 13)
            # 从释放点上方8步启动，1步调节也能逐渐接近，无需重新设点。
            started = plant.run(release_position=13, max_step=1)
            self.assertTrue(started['ok'], started)
            self.assertEqual(plant.controller.release, 13)

    def test_release_tolerance_requires_zero_pressure_and_still_limits_travel(self):
        for position, force, expected in ((16, 0, True), (17, 0, False),
                                           (8, .2, False), (-6145, 0, False)):
            with self.subTest(position=position, force=force):
                plant = Plant()
                plant.position = position
                plant.force_override = force
                result = plant.run(release=True)
                self.assertEqual(result['ok'], expected, result)
                self.assertEqual(plant.actions, [])

    def test_release_tolerance_does_not_relax_fine_adjustment_arrival(self):
        plant = Plant()
        original_move = plant.move_relative_steps
        def move(steps, **kwargs):
            original_move(steps, **kwargs)
            plant.position -= 8
        plant.move_relative_steps = move
        result = plant.run()
        self.assertFalse(result['ok'])
        self.assertIn('到位超时', result['message'])
        self.assertEqual(len(plant.actions), 1)

    def test_loaded_start_rejected_and_explicit_overload_release_allowed(self):
        plant = Plant()
        plant.force_override = 2
        self.assertFalse(plant.run()['ok'])
        self.assertEqual(plant.actions, [])
        plant = Plant()
        plant.position = -160
        self.assertGreater(plant.force(), 5)
        self.assertTrue(plant.run(release=True)['ok'])

    def test_release_not_blocked_by_unrelated_target_parameters(self):
        plant = Plant()
        plant.position = -35
        result = plant.run(release=True, target_n=13, force_limit_n=5)
        self.assertTrue(result['ok'], result)

    def test_stop_ack_without_stopped_feedback_is_not_success(self):
        plant = Plant()
        original = plant.read_feedback
        def read():
            sample = original()
            if plant.stop_count:
                sample['measured_speed_rpm'] = 10
            return sample
        plant.read_feedback = read
        result = plant.run()
        self.assertFalse(result['ok'])
        self.assertIn('停止命令未确认', result['message'])

    def test_stale_pressure_and_negative_reading_stop_without_moving(self):
        for mode in ('stale', 'negative'):
            plant = Plant()
            plant.read_feedback()
            if mode == 'stale':
                plant.fault = 'stale'
                plant.now += .6
            else:
                plant.force_override = -.11
                plant.now += .1
            self.assertFalse(plant.run()['ok'])
            self.assertEqual(plant.actions, [])
            self.assertEqual(plant.stop_count, 1)

    def test_loss_of_pressure_during_control_cannot_keep_moving(self):
        plant = Plant()
        def record(row):
            plant.records.append(row)
            if plant.actions:
                plant.fault = 'stale'
        settings = replace(FeedbackSettings(), max_step=8)
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
                                                settings, 0, record, clock=lambda: plant.now)
        result = controller.run()
        self.assertFalse(result['ok'])
        self.assertEqual(len(plant.actions), 1)

    def test_raw_pressure_spike_during_move_stops_before_median_filter(self):
        plant = Plant()
        original = plant.move_relative_steps
        def move(steps, **kwargs):
            original(steps, **kwargs)
            plant.force_override = 5.1
        plant.move_relative_steps = move
        result = plant.run()
        self.assertFalse(result['ok'])
        self.assertIn('保护上限', result['message'])
        self.assertEqual(len(plant.actions), 1)
        self.assertEqual(plant.stop_count, 1)

    def test_stall_wrong_direction_and_lost_ack_never_retry(self):
        for fault in ('stalled', 'wrong_direction', 'lost_ack'):
            with self.subTest(fault=fault):
                plant = Plant()
                plant.fault = fault
                result = plant.run()
                self.assertFalse(result['ok'])
                self.assertEqual(len(plant.actions), 1)
                self.assertEqual(plant.stop_count, 1)

    def test_current_force_travel_and_acquisition_limits(self):
        for fault in ('current', 'force', 'travel', 'timeout'):
            with self.subTest(fault=fault):
                plant = Plant()
                changes = {}
                if fault == 'current':
                    plant.current = 3001
                elif fault == 'force':
                    plant.force_override = 5
                elif fault == 'travel':
                    changes['max_travel'] = 16
                else:
                    plant.drift = lambda: 1000
                    changes['acquire_timeout_s'] = 10
                result = plant.run(**changes)
                self.assertFalse(result['ok'], result)
                self.assertEqual(plant.stop_count, 1)
                if fault == 'travel':
                    self.assertGreaterEqual(plant.position, -16)

    def test_cancel_and_failed_stop_are_not_reported_as_success(self):
        plant = Plant()
        plant.cancel_at = 11
        result = plant.run()
        self.assertTrue(result['cancelled'])
        self.assertFalse(result['ok'])
        plant = Plant()
        plant.fault = 'stop_failed'
        result = plant.run()
        self.assertIn('停止命令未确认', result['message'])
        self.assertFalse(result['stop_confirmed'])

    def test_pressure_timestamps_sequence_units_and_validity(self):
        samples = PressureSamples()
        valid = dict(sequence=1, sampled_monotonic=10, force_n=1, force_unit='N', accepted=True)
        samples.update(valid)
        samples.update(dict(valid, sampled_monotonic=11))
        with self.assertRaises(FeedbackControlError):
            samples.read(11)
        for delta in (dict(sequence=2, force_n=float('nan')),
                      dict(sequence=2, sampled_monotonic=12),
                      dict(sequence=2, force_unit='g'), dict(accepted=False)):
            samples = PressureSamples()
            samples.update(dict(valid, **delta))
            with self.assertRaises(FeedbackControlError):
                samples.read(10)

    def test_invalid_settings_rejected_before_commands(self):
        for change in (dict(target_n=float('nan')), dict(tolerance_n=1),
                       dict(force_limit_n=1), dict(max_step=0), dict(max_step=1.5),
                       dict(max_current_ma=3001), dict(settle_s=0),
                       dict(approach_speed=0), dict(approach_speed=31), dict(approach_speed=1.55),
                       dict(release_speed=0), dict(release_speed=401), dict(release_speed=1.55)):
            with self.subTest(change=change), self.assertRaises(FeedbackControlError):
                replace(FeedbackSettings(), **change).validate(PROFILE)

    def test_fractional_rpm_preserved_and_legacy_levels_remain_integer(self):
        from motor_mapping_interface import LEGACY_PROFILE
        settings = replace(FeedbackSettings(), speed=3.5, approach_speed=1.5, release_speed=24.5)
        settings.validate(PROFILE)
        self.assertEqual(PROFILE.speed_rpm(settings.speed), 3.5)
        self.assertEqual(PROFILE.speed_rpm(settings.release_speed), 24.5)
        with self.assertRaises(FeedbackControlError):
            settings.validate(LEGACY_PROFILE)

    def test_large_max_step_keeps_motion_within_travel_limit(self):
        replace(FeedbackSettings(), max_step=2**31-1).validate(PROFILE)
        plant = Plant()
        result = plant.run(max_step=10000, max_travel=16)
        self.assertFalse(result['ok'], result)
        self.assertIn('下一步将超过行程限制', result['message'])
        self.assertEqual(plant.actions, [])
        self.assertEqual(plant.stop_count, 1)


class AdaptiveCurvePlant(Plant):
    """按实测行程与压力量级构造分段曲线；保留运动耗时，不代表材料完整模型。"""

    POINTS = ((0, 0), (720, 0), (780, .5), (840, 1), (930, 2),
              (1170, 5), (1530, 10), (1850, 15), (2180, 20), (2450, 25), (2700, 30))

    def force(self):
        travel = -self.position
        for (x0, f0), (x1, f1) in zip(self.POINTS, self.POINTS[1:]):
            if travel <= x1:
                return round(max(0, f0+(travel-x0)*(f1-f0)/(x1-x0)), 2)
        return 30+(travel-2700)*.02

    def move_relative_steps(self, steps, **kwargs):
        super().move_relative_steps(steps, **kwargs)
        self.now += abs(steps)*60/(3200*kwargs['speed_rpm'])


class AdaptiveFeedbackTests(unittest.TestCase):
    @staticmethod
    def legacy_steps(controller, error, force):
        return max(1, min(controller.settings.max_step,
                          math.ceil(abs(error)/max(controller.settings.target_n, .5)*controller.settings.max_step)))

    def run_curve(self, target, maximum, legacy=False):
        plant = AdaptiveCurvePlant()
        settings = dict(target_n=target, tolerance_n=target*.02, max_step=maximum,
                        force_limit_n=35, speed=10, hold_s=1, acquire_timeout_s=60)
        if legacy:
            with patch.object(PressureFeedbackController, '_adjustment_steps', self.legacy_steps):
                result = plant.run(**settings)
        else:
            result = plant.run(**settings)
        return plant, result

    def test_one_newton_large_limit_no_longer_causes_repeated_overshoot(self):
        old, result = self.run_curve(1, 256, legacy=True)
        self.assertFalse(result['ok'])
        self.assertGreater(result['peak_n'], 1.5)
        for maximum in (128, 256, 1024):
            with self.subTest(maximum=maximum):
                plant, result = self.run_curve(1, maximum)
                self.assertTrue(result['ok'], result)
                self.assertLessEqual(result['peak_n'], 1.02)
                self.assertLessEqual(abs(plant.force()-1), .02)
                self.assertLess(plant.now-10, 15)

    def test_high_force_acquisition_is_faster_at_same_speed_and_step_limit(self):
        for target in (20, 25):
            for maximum in (128, 256):
                with self.subTest(target=target, maximum=maximum):
                    old, old_result = self.run_curve(target, maximum, legacy=True)
                    new, new_result = self.run_curve(target, maximum)
                    self.assertTrue(old_result['ok'], old_result)
                    self.assertTrue(new_result['ok'], new_result)
                    self.assertLess(new.now-10, (old.now-10)*.75)
                    self.assertLessEqual(new_result['peak_n'], target*1.02)
                    self.assertTrue(all(abs(step) <= maximum for step in new.actions))

    def test_in_motion_target_detection_stops_before_command_endpoint(self):
        plant = Plant()
        moving = [False]
        destination = [0]
        original_read, original_stop = plant.read_feedback, plant.stop
        def move(steps, **kwargs):
            plant.actions.append(steps)
            destination[0] = plant.position+steps
            moving[0] = True
        def read():
            if moving[0]:
                distance = destination[0]-plant.position
                plant.position += (1 if distance > 0 else -1)*min(2, abs(distance))
                moving[0] = plant.position != destination[0]
            sample = original_read()
            sample['measured_speed_rpm'] = 10 if moving[0] else 0
            sample['in_position'] = not moving[0]
            return sample
        def stop():
            moving[0] = False
            original_stop()
        plant.move_relative_steps = move
        plant.read_feedback = read
        plant.stop = stop
        result = plant.run(max_step=256)
        self.assertTrue(result['ok'], result)
        self.assertLess(abs(plant.position), abs(plant.actions[0]))
        self.assertLessEqual(result['peak_n'], 1.05)
        self.assertGreaterEqual(plant.stop_count, 2)

    def test_in_motion_stop_failure_cannot_be_reported_as_arrival(self):
        plant = Plant()
        original_move = plant.move_relative_steps
        def move(steps, **kwargs):
            original_move(steps, **kwargs)
            plant.position = -35
        plant.move_relative_steps = move
        plant.fault = 'stop_failed'
        result = plant.run(max_step=256)
        self.assertFalse(result['ok'])
        self.assertFalse(result['stop_confirmed'])
        self.assertEqual(len(plant.actions), 1)

    def test_different_material_stiffness_converges_without_fixed_mapping(self):
        for target in (1, 20):
            for factor in (.5, 2):
                with self.subTest(target=target, factor=factor):
                    plant = AdaptiveCurvePlant()
                    force = plant.force
                    plant.force = lambda: force()*factor
                    result = plant.run(target_n=target, tolerance_n=target*.02, max_step=256,
                                       force_limit_n=35, speed=10, hold_s=3)
                    self.assertTrue(result['ok'], result)
                    self.assertLessEqual(abs(plant.force()-target), target*.02)
                    self.assertLessEqual(result['peak_n'], target*1.02)

    def test_in_motion_stop_ack_requires_actual_stopped_feedback(self):
        plant = Plant()
        original_read = plant.read_feedback
        moving = [False]
        def move(steps, **kwargs):
            plant.actions.append(steps)
            plant.position = -20
            plant.force_override = 1
            moving[0] = True
        def read():
            sample = original_read()
            sample['measured_speed_rpm'] = 10 if moving[0] else 0
            return sample
        plant.read_feedback = read
        plant.move_relative_steps = move
        result = plant.run(max_step=256)
        self.assertFalse(result['ok'])
        self.assertFalse(result['stop_confirmed'])
        self.assertIn('压力到达目标后停止未确认', result['message'])
        self.assertEqual(len(plant.actions), 1)


class CenteringFeedbackTests(unittest.TestCase):
    def test_corrects_both_directions_before_leaving_user_tolerance(self):
        for drift in (.07, -.07):
            with self.subTest(drift=drift):
                plant = Plant()
                plant.force = lambda: max(0, (-plant.position-10)*.01-plant.drift())
                plant.drift = lambda: drift if (getattr(plant, 'controller', None) is not None
                    and plant.controller.hold_started is not None
                    and plant.now >= plant.controller.hold_started+1) else 0
                moves = []
                original = plant.move_relative_steps
                def move(steps, **kwargs):
                    moves.append((plant.force(), steps, plant.controller.hold_started is not None))
                    original(steps, **kwargs)
                plant.move_relative_steps = move
                result = plant.run(target_n=2, tolerance_n=.1, max_step=32, hold_s=10)
                self.assertTrue(result['ok'], result)
                corrections = [(force, step) for force, step, holding in moves
                               if holding and 1.9 < force < 2.1 and step*drift < 0]
                self.assertTrue(corrections, moves)
                self.assertTrue(all(abs(step) <= 3 for force, step in corrections))
                self.assertLessEqual(abs(plant.force()-2), .025)
                self.assertEqual(plant.controller.settings.tolerance_n, .1)

    def test_centering_reduces_mean_error_during_slow_relaxation(self):
        results = []
        for wide_band in (True, False):
            plant = Plant()
            plant.force = lambda: max(0, (-plant.position-10)*.01-.003*max(0, plant.now-20))
            settings = dict(target_n=2, tolerance_n=.1, max_step=32, hold_s=30)
            if wide_band:
                with patch.object(PressureFeedbackController, '_control_bands',
                                  lambda self: (self.settings.tolerance_n, self.settings.tolerance_n)):
                    result = plant.run(**settings)
            else:
                result = plant.run(**settings)
            self.assertTrue(result['ok'], result)
            results.append(result)
        self.assertLess(abs(results[1]['hold_mean_error_n']), abs(results[0]['hold_mean_error_n'])*.5)
        self.assertLess(results[1]['hold_rmse_n'], results[0]['hold_rmse_n']*.5)

    def test_noise_and_one_step_resolution_do_not_force_alternating_motion(self):
        for noisy in (True, False):
            with self.subTest(noisy=noisy):
                plant = Plant()
                if noisy:
                    plant.force = lambda: max(0, (-plant.position-10)*.01
                                              + (.01 if round(plant.now*20) % 2 else -.01))
                target = 2 if noisy else 1.02
                holding_moves = []
                original = plant.move_relative_steps
                def move(steps, **kwargs):
                    if plant.controller.hold_started is not None:
                        holding_moves.append((plant.now-plant.controller.hold_started, steps))
                    original(steps, **kwargs)
                plant.move_relative_steps = move
                result = plant.run(target_n=target, tolerance_n=.1 if noisy else .05,
                                   max_step=32, hold_s=20)
                self.assertTrue(result['ok'], result)
                # 首次到位按用户容差确认，保持初段可向中心收敛；后续不能追噪声来回运动。
                self.assertLessEqual(len([step for elapsed, step in holding_moves if elapsed >= 3]), 2, holding_moves)
                self.assertFalse(any(step > 0 for _, step in holding_moves) and
                                 any(step < 0 for _, step in holding_moves), holding_moves)
                self.assertLessEqual(abs(plant.force()-target), .025)


class LearningFeedbackTests(unittest.TestCase):
    def test_point_one_uses_measured_low_range_and_holds_quantized_pressure(self):
        _, table = self.learn()
        old, new = AdaptiveCurvePlant(), AdaptiveCurvePlant()
        settings = dict(target_n=.1, tolerance_n=pressure_tolerance_n(.1, 2), force_limit_n=35,
                        max_step=512, speed=10, approach_speed=30, hold_s=3, stable_s=.5)
        old_result = old.run(**settings)
        result = new.run(learned_map=table, **settings)
        self.assertTrue(old_result['ok'], old_result)
        self.assertTrue(result['ok'], result)
        self.assertTrue(result['used_learned_map'])
        self.assertGreater(abs(new.actions[0]), 500)
        self.assertEqual(new.motion_parameters[0]['speed_rpm'], 30)
        self.assertTrue(all(p['speed_rpm'] <= 2 for p in new.motion_parameters[1:]))
        self.assertTrue(all(abs(step) <= 32 for step in new.actions[2:]))
        self.assertLessEqual(result['peak_n'], .11+1e-9)
        self.assertLessEqual(abs(new.force()-.1), .01+1e-9)
        self.assertEqual(result['hold_in_band_fraction'], 1)
        self.assertLess(result['stable_arrival_s'], old_result['stable_arrival_s'])

    def test_old_learning_table_does_not_extrapolate_point_one(self):
        plant = AdaptiveCurvePlant()
        table = LearnedPressureMap(PROFILE.model, PROFILE.namespace, 3200, 0,
                                   ((.2, 744), (1, 840), (30, 2700)))
        self.assertIsNone(table.approach_travel(.1, 32))
        result = plant.run(target_n=.1, tolerance_n=.01, learned_map=table,
                           max_step=512, speed=10, approach_speed=30, hold_s=1)
        self.assertTrue(result['ok'], result)
        self.assertFalse(result['used_learned_map'])
        self.assertLessEqual(abs(plant.actions[0]), 4)
        self.assertTrue(all(abs(step) <= 32 for step in plant.actions))
        self.assertTrue(all(p['speed_rpm'] <= 2 for p in plant.motion_parameters))

    def test_low_target_and_learning_reject_unzeroed_pressure_before_moving(self):
        for learning in (False, True):
            for force in (.03, -.03, .1):
                with self.subTest(learning=learning, force=force):
                    plant = AdaptiveCurvePlant()
                    plant.force_override = force
                    result = plant.run(learning=learning, target_n=30 if learning else .1,
                                       tolerance_n=.6 if learning else .01, force_limit_n=35)
                    self.assertFalse(result['ok'], result)
                    self.assertIn('校零', result['message'])
                    self.assertEqual(plant.actions, [])
                    self.assertNotIn('learned_map', result)

    def test_low_zero_check_uses_entire_fresh_window(self):
        plant = AdaptiveCurvePlant()
        plant.force = lambda: .03 if round((plant.now-10)*20) == 2 else 0
        result = plant.run(target_n=.1, tolerance_n=.01)
        self.assertFalse(result['ok'], result)
        self.assertIn('底数超出±0.02 N', result['message'])
        self.assertEqual(plant.actions, [])

    def test_low_tolerance_includes_decimal_endpoints_without_accepting_next_digit(self):
        for force, in_band in ((.09, True), (.11, True), (.08, False), (.12, False)):
            with self.subTest(force=force):
                plant = Plant()
                plant.force_override = force
                controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
                    FeedbackSettings(target_n=.1, tolerance_n=.01), 0, clock=lambda: plant.now)
                controller._window()
                self.assertEqual(controller._window_in_band, in_band)

    def test_low_force_release_keeps_independent_speed(self):
        plant = Plant()
        plant.position = -50
        result = plant.run(release=True, target_n=.1, tolerance_n=.01, speed=10, release_speed=24)
        self.assertTrue(result['ok'], result)
        self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 24)

    def test_recorded_soft_low_force_curve_does_not_crawl_two_steps_at_a_time(self):
        # 11:58:47实测约620 step为0.05 N、666 step为0.20 N；每次动作有通信开销。
        class RecordedLowForcePlant(AdaptiveCurvePlant):
            POINTS = ((0, 0), (600, 0), (620, .05), (638, .10), (666, .20), (712, .49), (1708, 29.58))

            def move_relative_steps(self, steps, **kwargs):
                super().move_relative_steps(steps, **kwargs)
                self.now += .18

        table = LearnedPressureMap(PROFILE.model, PROFILE.namespace, 3200, 0,
            ((.04, 610), (.09, 630), (.19, 660), (.49, 712), (.98, 764), (29.58, 1708)))
        settings = dict(target_n=.2, tolerance_n=.01, force_limit_n=35, max_step=512,
                        speed=10, approach_speed=30, settle_s=.1, stable_s=.5, hold_s=1)
        old, new = RecordedLowForcePlant(), RecordedLowForcePlant()
        adaptive_steps = PressureFeedbackController._adjustment_steps

        def fixed_two_steps(controller, error, force):
            return min(adaptive_steps(controller, error, force), 2 if force > .02 else 4)

        with patch.object(PressureFeedbackController, '_adjustment_steps', fixed_two_steps):
            old_result = old.run(learned_map=table, **settings)
        result = new.run(learned_map=table, **settings)
        self.assertTrue(old_result['ok'], old_result)
        self.assertTrue(result['ok'], result)
        self.assertLess(result['stable_arrival_s'], old_result['stable_arrival_s']*.65)
        self.assertLessEqual(result['peak_n'], .21+1e-9)
        self.assertLessEqual(abs(new.force()-.2), .01+1e-9)
        self.assertTrue(any(abs(step) > 4 for step in new.actions[1:]))
        self.assertLess(len(new.actions), len(old.actions))
        self.assertTrue(all(p['speed_rpm'] <= 2 for p in new.motion_parameters[1:]))

    def test_quantized_low_response_accumulates_motion_but_not_stationary_drift(self):
        plant = Plant()
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            FeedbackSettings(target_n=.2, tolerance_n=.01), 0, clock=lambda: plant.now)
        for position, force in ((-620, .05), (-622, .05), (-624, .06), (-626, .07)):
            controller._observe_response(position, force)
        self.assertEqual(len(controller._response_slopes), 1)
        self.assertAlmostEqual(controller._response_slopes[0], .02/6)
        controller._response_slopes.clear()
        for force in (.08, .09, .10):
            controller._observe_response(-626, force)
        self.assertFalse(controller._response_slopes)
        controller._observe_response(-628, .11)
        self.assertFalse(controller._response_slopes)

    def test_low_adaptive_step_keeps_near_target_unknown_and_reversal_bounds(self):
        plant = Plant()
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            FeedbackSettings(target_n=.2, tolerance_n=.01, max_step=512), 0, clock=lambda: plant.now)
        self.assertLessEqual(controller._adjustment_steps(.15, .05), 2)
        controller._response_slopes.append(.003)
        step = controller._adjustment_steps(.15, .05)
        self.assertGreater(step, 4)
        self.assertLessEqual(step*.003, .08)
        self.assertLessEqual(controller._adjustment_steps(.02, .18), 2)
        self.assertEqual(controller._adjustment_steps(-.03, .23), 1)

    def learn(self):
        plant = AdaptiveCurvePlant()
        result = plant.run(learning=True, target_n=30, tolerance_n=.6, force_limit_n=35,
                           max_step=256, speed=10, stable_s=.5)
        self.assertTrue(result['ok'], result)
        return plant, result['learned_map']

    def test_learning_covers_low_and_high_pressure_and_releases(self):
        plant, table = self.learn()
        self.assertTrue(.01-1e-9 <= table.contact_force_n <= .02+1e-9)
        self.assertLess(table.contact_zero_travel, table.contact_travel)
        self.assertLess(table.contact_travel, table.points[0][1])
        self.assertEqual(plant.records[-1]['contact_travel'], table.contact_travel)
        self.assertEqual(len(table.points), 12)
        self.assertLessEqual(abs(table.points[0][0]-.05), .010001)
        self.assertLessEqual(abs(table.points[1][0]-.1), .010001)
        self.assertLessEqual(abs(table.points[-1][0]-30), .6)
        self.assertEqual(plant.position, 0)
        self.assertEqual(plant.force(), 0)
        self.assertGreater(plant.actions[-1], 2000)
        self.assertEqual(plant.controller.hold_errors, [])
        for target in (.1, .2, .5, 1, 2, 5, 10, 15, 20, 25, 30):
            self.assertGreater(table.approach_travel(target, 32), 0)

    def test_learned_approach_reduces_stops_then_closes_loop(self):
        _, table = self.learn()
        for target in (1, 20, 30):
            with self.subTest(target=target):
                old, new = AdaptiveCurvePlant(), AdaptiveCurvePlant()
                settings = dict(target_n=target, tolerance_n=target*.02, force_limit_n=35,
                                max_step=128, speed=10, hold_s=1)
                self.assertTrue(old.run(**settings)['ok'])
                result = new.run(learned_map=table, **settings)
                self.assertTrue(result['ok'], result)
                self.assertTrue(result['used_learned_map'])
                self.assertGreater(abs(new.actions[0]), 128)
                self.assertTrue(all(abs(step) <= 128 for step in new.actions[1:]))
                self.assertLess(len(new.actions), len(old.actions))
                self.assertLess(new.now, old.now)
                self.assertLessEqual(abs(new.force()-target), target*.02)

    def test_invalid_reference_or_smaller_travel_cannot_use_old_position(self):
        _, table = self.learn()
        for invalid in (replace(table, namespace='other'), replace(table, release_position=20),
                        replace(table, steps_per_turn=6400)):
            plant = AdaptiveCurvePlant()
            result = plant.run(learned_map=invalid, max_step=128)
            self.assertTrue(result['ok'], result)
            self.assertFalse(result['used_learned_map'])
            self.assertLessEqual(abs(plant.actions[0]), 128)
        plant = AdaptiveCurvePlant()
        result = plant.run(learned_map=table, max_step=128, max_travel=500)
        self.assertFalse(result['ok'])
        self.assertFalse(result['used_learned_map'])
        self.assertGreaterEqual(plant.position, -500)

    def test_fast_approach_stops_on_earlier_contact_in_changed_material(self):
        _, table = self.learn()
        plant = Plant()
        destination = [0]
        original_read, original_stop = plant.read_feedback, plant.stop
        def move(steps, **kwargs):
            plant.actions.append(steps)
            destination[0] = plant.position+steps
        def read():
            if plant.position > destination[0]:
                plant.position = max(destination[0], plant.position-2)
            sample = original_read()
            sample['measured_speed_rpm'] = 10 if plant.position != destination[0] else 0
            return sample
        def stop():
            destination[0] = plant.position
            original_stop()
        plant.move_relative_steps, plant.read_feedback, plant.stop = move, read, stop
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, FeedbackSettings(), 0,
                                                clock=lambda: plant.now, learned_map=table)
        controller._fast_approach()
        self.assertTrue(controller.used_learned_map)
        self.assertGreater(plant.stop_count, 0)
        self.assertLess(abs(plant.position), abs(plant.actions[0]))
        self.assertLessEqual(plant.force(), 1)

    def test_cancelled_or_failed_learning_never_returns_a_usable_table(self):
        for failure in ('cancel', 'pressure', 'release_cancel'):
            with self.subTest(failure=failure):
                plant = AdaptiveCurvePlant()
                if failure == 'cancel':
                    plant.cancel_at = 12
                elif failure == 'pressure':
                    plant.force_override = 36
                else:
                    move = plant.move_relative_steps
                    def cancel_release(steps, **kwargs):
                        move(steps, **kwargs)
                        if steps > 1000:
                            plant.cancelled = True
                    plant.move_relative_steps = cancel_release
                result = plant.run(learning=True, target_n=30, tolerance_n=.6, force_limit_n=35,
                                   max_step=256, speed=10, stable_s=.5)
                self.assertFalse(result['ok'])
                self.assertNotIn('learned_map', result)

    def test_map_rejects_nonmonotonic_data_and_never_extrapolates(self):
        with self.assertRaises(FeedbackControlError):
            LearnedPressureMap('zdt_x42s', '', 3200, 0, ((1, 800), (2, 790), (30, 2700)))
        _, table = self.learn()
        self.assertIsNone(table.travel_at(40))
        self.assertIsNone(table.travel_at(.005))


class FeedbackUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from qt_compat import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def pump_until(self, condition, timeout=12):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.app.processEvents()
            if condition():
                return
            time.sleep(.005)
        self.fail('UI timed out')

    def test_shared_worker_locks_other_page_logs_and_releases(self):
        from qt_compat import QtCore
        from test_mapping_motor_interface import MappingSimMotor
        from zdt_x42s_can_driver import ZDTConfig
        from zdt_motor_test_widget import ZDTMotorTestWidget
        from pressure_feedback_widget import PressureFeedbackWidget
        with tempfile.TemporaryDirectory() as directory:
            motor = MappingSimMotor(ZDTConfig())
            backend = ZDTMotorTestWidget(ZDTConfig(), motor_factory=lambda cfg: motor)
            control = create_mapping_control('zdt_x42s', None, backend)
            page = PressureFeedbackWidget(control, output_dir=directory,
                                          settings_path=Path(directory)/'settings.json')
            timer = QtCore.QTimer()
            seq = [0]
            def sample():
                seq[0] += 1
                page.set_pressure_sample(dict(sequence=seq[0], sampled_monotonic=time.monotonic(),
                                              force_n=max(0, -motor.position*.1), force_unit='N', accepted=True))
            timer.timeout.connect(sample)
            timer.start(20)
            try:
                self.assertFalse(page.start_button.isEnabled())
                backend.connect_can()
                self.pump_until(lambda: control.is_can_connected() and not control.is_operation_busy())
                sample()
                page._capture()
                for name, value in dict(target_n=.3, tolerance_percent=16.6667, max_step=2,
                                        stable_s=.5, hold_s=1, settle_s=.1).items():
                    page.inputs[name].setValue(value)
                page.start_control()
                self.assertTrue(page.is_running())
                self.assertAlmostEqual(page.band.getRegion()[0], .25, places=6)
                self.assertAlmostEqual(page.band.getRegion()[1], .35, places=6)
                self.assertFalse(backend.press_button.isEnabled())
                self.assertFalse(page.inputs['target_n'].isEnabled())
                self.assertFalse(control.start_external_operation(lambda *_: None, 'other')[0])
                self.pump_until(lambda: not page.is_running() and not control.is_operation_busy())
                self.assertTrue(page._result['ok'], page._result)
                self.assertEqual(motor.thread_ids, {backend._thread.ident})
                self.assertLessEqual(abs(max(0, -motor.position*.1)-.3), .05)
                self.assertIn('elapsed_s', Path(page._result['csv_path']).read_text(encoding='utf-8-sig'))
                # 控制结束后继续观察；保留本轮结果，释放也不重置曲线时间轴。
                previous_count = len(page._points)
                finished_status = page.status_label.text()
                self.pump_until(lambda: len(page._points) >= previous_count+2)
                self.assertFalse(page.is_running())
                self.assertEqual(page.status_label.text(), finished_status)
                origin = page._plot_origin
                first_point = page._points[0].copy()
                page.inputs['release_speed'].setValue(60)
                moves_before_release = len([a for a in motor.actions if a[0] == 'move'])
                page.start_release()
                self.assertEqual(page._plot_origin, origin)
                self.assertEqual(page._points[0], first_point)
                self.pump_until(lambda: not page.is_running() and not control.is_operation_busy())
                self.assertTrue(page._result['ok'], page._result)
                moves = [a[1] for a in motor.actions if a[0] == 'move']
                self.assertEqual(len(moves)-moves_before_release, 1)
                self.assertEqual(moves[-1]['speed_rpm'], 60)
                self.assertFalse(moves[-1]['ccw'])
                self.assertIn('release_speed', Path(page._result['csv_path']).read_text(encoding='utf-8-sig'))
                self.assertLessEqual(abs(motor.position), control.profile.tolerance(50))
                self.assertLessEqual(max(0, -motor.position*.1), .1)
                self.assertEqual(len(list(Path(directory).glob('*.csv'))), 2)
                # 合并按钮在带压保持时先停止，再通过原线程完成释放。
                motor.position = 0
                sample()
                page.inputs['hold_s'].setValue(30)
                page.start_control()
                self.pump_until(lambda: motor.position <= -3)
                page.release_button.click()
                self.pump_until(lambda: not page.is_running() and not page._pending_release
                                and not page._release_after_stop and not control.is_operation_busy())
                self.assertTrue(page._result['ok'], page._result)
                self.assertIn('释放完成', page._result['message'])
                self.assertTrue(page._result['stop_confirmed'])
                self.assertLessEqual(abs(motor.position), control.profile.tolerance(50))
                self.assertEqual(len(list(Path(directory).glob('*.csv'))), 4)
                self.assertEqual(page.release_button.text(), '停止并释放')
                # 运行中用户停止也要经同一工作线程退出，不能保留后台补压。
                motor.position = 0
                sample()
                page.inputs['hold_s'].setValue(30)
                page._refresh()
                page.quick_target_buttons[1].click()
                self.assertTrue(page.is_running())
                self.assertEqual(page.inputs['target_n'].value(), 1)
                self.assertFalse(any(button.isEnabled() for button in page.quick_target_buttons.values()))
                saved = json.loads(page._settings_path.read_text(encoding='utf-8'))
                self.assertEqual(saved['values']['target_n'], 1)
                page.stop()
                self.pump_until(lambda: not page.is_running() and not control.is_operation_busy())
                self.assertFalse(page._result['ok'])
                self.assertTrue(page._result.get('cancelled'), page._result)
            finally:
                timer.stop()
                page.shutdown()
                backend.shutdown()
                page.deleteLater()
                backend.deleteLater()
                self.app.processEvents()


class FeedbackSettingsUiTests(unittest.TestCase):
    pump_until = FeedbackUiTests.pump_until

    @classmethod
    def setUpClass(cls):
        from qt_compat import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def page(self, config=None, settings_path=None):
        from zdt_x42s_can_driver import ZDTConfig
        from zdt_motor_test_widget import ZDTMotorTestWidget
        from pressure_feedback_widget import PressureFeedbackWidget, feedback_settings_path
        backend = ZDTMotorTestWidget(config or ZDTConfig())
        control = create_mapping_control('zdt_x42s', None, backend)
        path = settings_path or self.root/feedback_settings_path(control.profile).name
        page = PressureFeedbackWidget(control, output_dir=self.root/'records', settings_path=path)
        def cleanup():
            page.shutdown()
            backend.shutdown()
            page.deleteLater()
            backend.deleteLater()
            self.app.processEvents()
        self.addCleanup(cleanup)
        return page

    def test_all_values_auto_save_and_restore_without_restoring_release_or_motion(self):
        page = self.page()
        values = dict(target_n=2, tolerance_percent=3.5, hold_s=6, max_step=10000, speed=7, approach_speed=25,
                      settle_s=.35, force_limit_n=8, max_travel=3200, max_current_ma=2500,
                      stable_s=1.5, acquire_timeout_s=120, acceleration=80, release_speed=60,
                      plot_window_s=45)
        self.assertEqual(set(values), set(page.inputs))
        for name, value in values.items():
            page.inputs[name].setValue(value)
        page.control._release = 13
        self.pump_until(lambda: page._settings_path.exists())
        document = json.loads(page._settings_path.read_text(encoding='utf-8'))
        self.assertEqual(document['values'], values)
        self.assertEqual(document['schema_version'], 2)
        self.assertEqual(set(document), {'schema_version', 'profile', 'values'})
        self.assertIn('已自动保存', page.settings_status_label.text())
        restored = self.page()
        self.assertEqual({k:v.value() for k,v in restored.inputs.items()}, values)
        self.assertFalse(restored.control.release_position_snapshot()['confirmed'])
        self.assertFalse(restored.is_running())
        self.assertFalse(restored.start_button.isEnabled())

    def test_percentage_tracks_target_and_protection_uses_converted_tolerance(self):
        page = self.page()
        page.inputs['tolerance_percent'].setValue(1)
        self.assertAlmostEqual(page._parameters().tolerance_n, .01)
        page.inputs['target_n'].setValue(2)
        self.assertAlmostEqual(page._parameters().tolerance_n, .02)
        page.inputs['target_n'].setValue(.1)
        self.assertAlmostEqual(page._parameters().tolerance_n, .01)
        page.inputs['target_n'].setValue(2)
        page.inputs['force_limit_n'].setValue(2.02)
        with self.assertRaises(FeedbackControlError):
            page._parameters()

    def test_quick_targets_set_value_then_start_without_overriding_protection(self):
        page = self.page()
        self.assertEqual(list(page.quick_target_buttons), [.1, .2, .5, 1, 2, 5, 10, 15, 20, 25, 30])
        self.assertFalse(any(button.isEnabled() for button in page.quick_target_buttons.values()))
        observed = []
        with patch.object(page, 'start_control', side_effect=lambda: observed.append(page.inputs['target_n'].value())):
            for target, button in page.quick_target_buttons.items():
                page.inputs['target_n'].lineEdit().setText('3')
                button.setEnabled(True)
                button.click()
        self.assertEqual(observed, [.1, .2, .5, 1, 2, 5, 10, 15, 20, 25, 30])
        page.start_target(30)
        self.assertFalse(page.is_running())
        self.assertEqual(page.inputs['force_limit_n'].value(), 5)
        self.assertIn('保护压力', page.status_label.text())
        self.assertEqual(self.page().inputs['target_n'].value(), 30)

    def test_idle_plot_monitors_channels_independently_and_gaps_stale_feedback(self):
        page = self.page()
        page.timer.stop()
        now = [100.0]
        linked = [True]
        feedback = dict(position_counts=-120, sampled_monotonic=100.0)
        def pressure(sequence, force):
            page.set_pressure_sample(dict(sequence=sequence, force_n=force, force_unit='N', accepted=True,
                                          sampled_monotonic=now[0]))
        with patch('pressure_feedback_widget.time.monotonic', side_effect=lambda: now[0]), \
                patch.object(page.control, 'is_can_connected', side_effect=lambda: linked[0]), \
                patch.object(page.control, 'latest_motor_feedback', side_effect=lambda: feedback), \
                patch.object(page.control, 'start_external_operation') as start:
            pressure(1, 1.2)
            page._refresh()
            self.assertEqual(page._points[-1]['position_steps'], -120)
            self.assertEqual(page._points[-1]['force_n'], 1.2)
            self.assertFalse(page.is_running())
            self.assertFalse(page.control.release_position_snapshot()['confirmed'])
            self.assertIn('-120 step', page.live_label.text())
            count = len(page._points)
            page._refresh()
            self.assertEqual(len(page._points), count)
            now[0] += 1
            page._refresh()
            self.assertTrue(math.isnan(page._points[-1]['force_n']))
            self.assertEqual(page._points[-1]['position_steps'], -120)
            now[0] += 2
            page._refresh()
            self.assertTrue(math.isnan(page._points[-1]['position_steps']))
            self.assertIn('位置 --', page.live_label.text())
            now[0] += .2
            feedback.update(position_counts=-90, sampled_monotonic=now[0])
            page._refresh()
            self.assertEqual(page._points[-1]['position_steps'], -90)
            self.assertTrue(math.isnan(page._points[-1]['force_n']))
            now[0] += .2
            linked[0] = False
            pressure(2, .7)
            page._refresh()
            self.assertEqual(page._points[-1]['force_n'], .7)
            self.assertTrue(math.isnan(page._points[-1]['position_steps']))
            start.assert_not_called()

    def test_finished_operation_keeps_plot_updating_without_changing_statistics(self):
        page = self.page()
        page.timer.stop()
        now = [100.0]
        feedback = dict(position_counts=-800, sampled_monotonic=100.0)
        with patch('pressure_feedback_widget.time.monotonic', side_effect=lambda: now[0]), \
                patch.object(page.control, 'is_can_connected', return_value=True), \
                patch.object(page.control, 'latest_motor_feedback', side_effect=lambda: feedback):
            for i, force in enumerate((1.0, .95, .9)):
                now[0] = 100+i*.2
                feedback['sampled_monotonic'] = now[0]
                page.set_pressure_sample(dict(sequence=i+1, force_n=force, force_unit='N', accepted=True,
                                              sampled_monotonic=now[0]))
                if i == 0:
                    page._running = True
                    page._on_finished(dict(ok=True, stop_confirmed=True, message='保持结束',
                                           hold_rmse_n=.01, hold_mean_error_n=.005, hold_in_band_fraction=1,
                                           peak_n=1.01, move_count=4))
                    summary = page.summary_label.text()
                else:
                    page._refresh()
            self.assertEqual([p['force_n'] for p in page._points], [1, .95, .9])
            self.assertEqual(page.summary_label.text(), summary)
            self.assertEqual(page.status_label.text(), '保持结束')
            self.assertFalse(page.is_running())
            count = len(page._points)
            page.shutdown()
            now[0] += 1
            page._refresh()
            self.assertEqual(len(page._points), count)

    def test_plot_shows_configurable_rolling_window_during_operation(self):
        page = self.page()
        page.timer.stop()
        self.assertEqual(page.inputs['plot_window_s'].value(), 20)
        for second in range(41):
            page._append_live_point(100+second, second/10, -second, 1, .05)
        self.assertEqual(list(page.force_curve.getData()[0]), list(range(20, 41)))
        self.assertEqual(list(page.position_curve.getData()[0]), list(range(20, 41)))
        self.assertEqual(page.plot.viewRange()[0], [20, 40])
        page._running = True
        with patch('pressure_feedback_widget.time.monotonic', return_value=140.01):
            page._refresh()
            self.assertTrue(page.inputs['plot_window_s'].isEnabled())
            self.assertFalse(page.inputs['speed'].isEnabled())
            page.inputs['plot_window_s'].setValue(5)
        page._append_live_point(141, 1, -41, 1, .05)
        self.assertEqual(list(page.force_curve.getData()[0]), list(range(36, 42)))
        self.assertEqual(page.plot.viewRange()[0], [36, 41])
        self.assertEqual(page.position_plot.viewRange()[0], [36, 41])
        page._running = False

    def test_release_button_and_start_allow_negative_or_missing_pressure(self):
        page = self.page()
        page.timer.stop()
        with patch.object(page.control, 'is_can_connected', return_value=True), \
                patch.object(page.control, 'is_operation_busy', return_value=False), \
                patch.object(page.control, 'release_position_snapshot', return_value=dict(confirmed=True, position_counts=2)), \
                patch.object(page.control, 'start_external_operation', return_value=(True, '')) as submit:
            for sequence, force in enumerate((None, -.25), 1):
                if force is not None:
                    page.samples.update(dict(force_n=force, force_unit='N', accepted=True,
                                             sampled_monotonic=time.monotonic(), sequence=sequence))
                page._refresh()
                self.assertTrue(page.release_button.isEnabled())
                self.assertFalse(page.start_button.isEnabled())
                page.release_button.click()
                self.assertTrue(page.is_running())
                self.assertTrue(page._releasing)
                page._running = False
            self.assertEqual(submit.call_count, 2)

    def test_quick_target_does_not_change_target_during_operation_or_pending_release(self):
        page = self.page()
        with patch.object(page, 'start_control') as start:
            page._running = True
            page.start_target(20)
            page._running = False
            page._pending_release = True
            page.start_target(25)
            page._pending_release = False
            with patch.object(page.control, 'is_operation_busy', return_value=True):
                page.start_target(30)
            start.assert_not_called()
        self.assertEqual(page.inputs['target_n'].value(), 1)

    def test_learning_button_runs_complete_cycle_saves_table_and_preserves_target(self):
        for offset in (0, .01, -.015):
            with self.subTest(offset=offset):
                self._check_learning_button_cycle(offset)

    def _check_learning_button_cycle(self, offset):
        page = self.page()
        plant = AdaptiveCurvePlant()
        force = plant.force
        plant.force = lambda: force()+offset
        plant.now = time.monotonic()
        page.samples = plant.samples
        plant.read_feedback()
        for name, value in dict(force_limit_n=35, tolerance_percent=2, max_step=256,
                                speed=10, stable_s=.5).items():
            page.inputs[name].setValue(value)
        tasks = []
        def submit(task, status):
            tasks.append(task)
            return True, ''
        def controller(*args, **kwargs):
            return PressureFeedbackController(*args, **kwargs, clock=lambda: plant.now)
        with patch.object(page.control, 'is_can_connected', return_value=True), \
                patch.object(page.control, 'is_operation_busy', return_value=False), \
                patch.object(page.control, 'release_position_snapshot', return_value=dict(confirmed=True, position_counts=0)), \
                patch.object(page.control, 'start_external_operation', side_effect=submit), \
                patch('pressure_feedback_widget.PressureFeedbackController', side_effect=controller):
            page._refresh()
            page.learn_button.click()
            self.assertTrue(page.is_running())
            self.assertFalse(page.learn_button.isEnabled())
            self.assertEqual(page.inputs['target_n'].value(), 1)
            self.assertEqual(page._settings.target_n, 30)
            tasks[0](plant, plant)
            self.assertFalse(page.is_running())
            self.assertTrue(page._result['ok'], page._result)
            self.assertIsNotNone(page._learned_map)
            self.assertEqual(plant.position, 0)
            document = json.loads(Path(page._result['learning_path']).read_text(encoding='utf-8'))
            self.assertEqual(len(document['points']), 12)
            self.assertEqual(document['schema_version'], 3)
            self.assertEqual(document['contact_reference_n'], offset)
            self.assertTrue(.01-1e-9 <= document['contact_force_n']-offset <= .02+1e-9)
            self.assertLess(document['contact_zero_travel'], document['contact_travel'])
            self.assertIn('接触点', page.learning_status_label.text())
            self.assertEqual(document['release_position'], 0)
            self.assertIn('1 N', page.learning_status_label.toolTip())
            self.assertIn('30 N', page.learning_status_label.toolTip())
            self.assertIn('learn', Path(page._result['csv_path']).read_text(encoding='utf-8-sig'))
            restored = self.page()
            self.assertEqual(restored._learned_map, page._learned_map)
            self.assertTrue(restored.learning_recovery_pending())
            self.assertIsNone(restored.learned_map_snapshot())
            page._release_reference_changed()
            self.assertIsNotNone(page._learned_map)
            page._coordinates_invalidated()
            self.assertIsNone(page._learned_map)

    def test_learning_requires_protection_and_invalidated_result_is_not_activated(self):
        page = self.page()
        page.start_learning()
        self.assertFalse(page.is_running())
        self.assertIn('保护压力', page.status_label.text())
        self.assertEqual(page.inputs['target_n'].value(), 1)
        table = LearnedPressureMap(page.profile.model, page.profile.namespace, page.profile.steps_per_turn,
                                   0, ((.2, 700), (1, 840), (30, 2700)))
        page._learned_map = table
        page._control_state_changed(False)
        self.assertIs(page._learned_map, table)
        self.assertTrue(page.learning_recovery_pending())
        self.assertIsNone(page.learned_map_snapshot())
        page._coordinates_invalidated()
        self.assertIsNone(page._learned_map)
        old_generation = page._learning_generation
        page._invalidate_learning('更换装夹')
        page._learning = True
        with patch.object(page.control, 'is_can_connected', return_value=True), \
                patch.object(page.control, 'release_position_snapshot', return_value=dict(confirmed=True, position_counts=0)):
            page._on_finished(dict(ok=True, stop_confirmed=True, learned_map=table,
                                   learning_generation=old_generation, message='学习完成'))
        self.assertIsNone(page._learned_map)

    def test_fault_preserves_learning_and_recovers_only_after_fresh_unloaded_feedback(self):
        page = self.page()
        page.control._release = 0
        table = LearnedPressureMap(page.profile.model, page.profile.namespace, page.profile.steps_per_turn,
                                   0, ((.2, 700), (1, 840), (29.88, 2700)))
        page._learned_map = table
        now = [10.0]
        feedback = dict(position_counts=0, measured_speed_rpm=0, phase_current_ma=100,
                        running_state=0, in_position=True, sampled_monotonic=10.0)
        with patch('time.monotonic', lambda: now[0]), \
                patch.object(page.control, 'is_can_connected', return_value=True), \
                patch.object(page.control, 'is_operation_busy', return_value=False), \
                patch.object(page.control, 'latest_motor_feedback', side_effect=lambda: dict(feedback)), \
                patch.object(page.control, 'start_external_operation') as start:
            generation = page._learning_generation
            page._on_finished(dict(ok=False, stop_confirmed=False, message='停止反馈未确认'))
            self.assertIs(page._learned_map, table)
            self.assertEqual(page._learning_generation, generation)
            self.assertIn('29.88 N', page.learning_notice_text())
            self.assertIn('停止反馈未确认', page.learning_notice_text())
            self.assertIsNone(page.learned_map_snapshot())
            page.start_control()
            start.assert_not_called()
            self.assertFalse(page.start_button.isEnabled())
            self.assertTrue(page.release_button.isEnabled())
            # 异常前的零压快照和重复读取均不能恢复。
            page.samples.update(dict(accepted=True, force_unit='N', force_n=0, sequence=1, sampled_monotonic=10))
            for _ in range(3):
                page._refresh()
            self.assertTrue(page.learning_recovery_pending())
            for sequence, stamp in enumerate((10.1, 10.21, 10.32), 2):
                now[0] = stamp
                feedback['sampled_monotonic'] = stamp
                page.samples.update(dict(accepted=True, force_unit='N', force_n=.01,
                                         sequence=sequence, sampled_monotonic=stamp))
                page._refresh()
                if sequence < 4:
                    self.assertTrue(page.learning_recovery_pending())
                    page._refresh()
                    self.assertTrue(page.learning_recovery_pending())
            self.assertFalse(page.learning_recovery_pending())
            self.assertIs(page.learned_map_snapshot(), table)
            self.assertTrue(page.learning_notice_label.isHidden())
            self.assertTrue(page.start_button.isEnabled())
            start.assert_not_called()

    def test_learning_recovery_rejects_moving_loaded_stale_or_wrong_reference(self):
        page = self.page()
        page.control._release = 0
        table = LearnedPressureMap(page.profile.model, page.profile.namespace, page.profile.steps_per_turn,
                                   0, ((.2, 700), (1, 840), (30, 2700)))
        now, sequence = [20.0], 0
        feedback = {}
        with patch('time.monotonic', lambda: now[0]), \
                patch.object(page.control, 'is_can_connected', return_value=True), \
                patch.object(page.control, 'is_operation_busy', return_value=False), \
                patch.object(page.control, 'latest_motor_feedback', side_effect=lambda: dict(feedback)):
            for fault in ('moving', 'loaded', 'stale', 'wrong_position', 'wrong_reference', 'gap', 'fault_state'):
                with self.subTest(fault=fault):
                    page.control._release = 0
                    page._learned_map = table
                    page._suspend_learning('等待确认')
                    for index in range(3):
                        now[0] += 3 if fault == 'gap' else .11
                        sequence += 1
                        feedback.update(position_counts=100 if fault == 'wrong_position' else 0,
                                        measured_speed_rpm=5 if fault == 'moving' else 0,
                                        phase_current_ma=100, in_position=True,
                                        running_state=3 if fault == 'fault_state' else 0,
                                        sampled_monotonic=now[0]-3 if fault == 'stale' else now[0])
                        if fault == 'wrong_reference':
                            page.control._release = 1
                        page.samples.update(dict(accepted=True, force_unit='N', force_n=1 if fault == 'loaded' else 0,
                                                 sequence=sequence, sampled_monotonic=now[0]))
                        page._refresh()
                    self.assertTrue(page.learning_recovery_pending())
                    self.assertIs(page._learned_map, table)
                    self.assertIsNone(page.learned_map_snapshot())

    def test_true_reference_changes_and_clear_invalidate_suspended_learning(self):
        page = self.page()
        table = LearnedPressureMap(page.profile.model, page.profile.namespace, page.profile.steps_per_turn,
                                   0, ((.2, 700), (1, 840), (30, 2700)))
        for event in ('reference', 'coordinates', 'clear', 'disconnect'):
            page.control._release = 0
            page._learned_map = table
            page._suspend_learning('临时异常')
            if event == 'reference':
                page.control._release = 1
                page._release_reference_changed()
            elif event == 'coordinates':
                page._coordinates_invalidated()
            elif event == 'disconnect':
                page.control._on_link(False)
                self.assertIs(page._learned_map, table)
                self.assertTrue(page.learning_recovery_pending())
                continue
            else:
                page._invalidate_learning('用户清除')
            self.assertIsNone(page._learned_map)
            self.assertFalse(page.learning_recovery_pending())

    def test_confirmed_user_cancel_keeps_learning_but_unconfirmed_stop_suspends(self):
        page = self.page()
        table = LearnedPressureMap(page.profile.model, page.profile.namespace, page.profile.steps_per_turn,
                                   0, ((.2, 700), (1, 840), (30, 2700)))
        page._learned_map = table
        page._on_finished(dict(ok=False, cancelled=True, stop_confirmed=True, message='用户停止'))
        self.assertIs(page._learned_map, table)
        self.assertFalse(page.learning_recovery_pending())
        page._on_finished(dict(ok=False, cancelled=True, stop_confirmed=False, message='停止未确认'))
        self.assertIs(page._learned_map, table)
        self.assertTrue(page.learning_recovery_pending())

    def test_merged_button_blocks_release_after_fault_or_unconfirmed_stop(self):
        page = self.page()
        self.assertNotEqual(page.capture_button.styleSheet(), page.start_button.styleSheet())
        self.assertNotEqual(page.start_button.styleSheet(), page.release_button.styleSheet())
        for result in (dict(ok=False, cancelled=True, stop_confirmed=False, message='停止未确认'),
                       dict(ok=False, stop_confirmed=True, message='反馈故障')):
            page._running = True
            page._release_after_stop = True
            with patch.object(page, 'start_release') as release:
                page._on_finished(result)
                release.assert_not_called()
            self.assertFalse(page._pending_release)
            self.assertIn('未自动释放', page.status_label.text())

    def test_merged_button_can_cancel_pending_release_and_stop_release(self):
        page = self.page()
        page._running = True
        with patch.object(page.control, 'request_external_stop') as stop:
            page.stop_and_release()
            self.assertTrue(page._release_after_stop)
            page.stop_and_release()
            self.assertFalse(page._release_after_stop)
            page._releasing = True
            page.stop_and_release()
            self.assertFalse(page._release_after_stop)
            self.assertEqual(stop.call_count, 3)
        page._running = False
        page._pending_release = True
        with patch.object(page, 'start_release') as release:
            page.shutdown()
            release.assert_not_called()
        self.assertFalse(page._pending_release)

    def test_legacy_tolerance_migrates_to_percentage_and_saves_new_schema(self):
        page = self.page()
        document = dict(schema_version=1, profile=page._settings_identity(),
                        values=dict(target_n=2, tolerance_n=.01, max_step=256))
        page._settings_path.write_text(json.dumps(document), encoding='utf-8')
        restored = self.page()
        self.assertEqual(restored.inputs['tolerance_percent'].value(), .5)
        self.assertAlmostEqual(restored._parameters().tolerance_n, .01)
        self.pump_until(lambda: not restored._settings_dirty)
        saved = json.loads(page._settings_path.read_text(encoding='utf-8'))
        self.assertEqual(saved['schema_version'], 2)
        self.assertNotIn('tolerance_n', saved['values'])
        self.assertEqual(saved['values']['tolerance_percent'], .5)
        self.assertEqual(self.page().inputs['tolerance_percent'].value(), .5)

    def test_pending_editor_is_flushed_on_shutdown_and_start(self):
        page = self.page()
        page.inputs['max_step'].lineEdit().setText('64')
        page.shutdown()
        restored = self.page()
        self.assertEqual(restored.inputs['max_step'].value(), 64)
        restored.inputs['speed'].lineEdit().setText('9')
        restored.start_control()
        self.assertFalse(restored.is_running())
        self.assertEqual(self.page().inputs['speed'].value(), 9)

    def test_profiles_have_separate_files_and_mismatched_file_is_not_overwritten(self):
        from zdt_x42s_can_driver import ZDTConfig
        first = self.page()
        first.inputs['max_step'].setValue(64)
        first._save_settings()
        original = first._settings_path.read_bytes()
        other_node = self.page(ZDTConfig(node_id=2))
        other_scale = self.page(ZDTConfig(steps_per_turn=6400))
        self.assertEqual(len({p._settings_path for p in (first, other_node, other_scale)}), 3)
        self.assertEqual(other_node.inputs['max_step'].value(), 16)
        mismatch = self.page(ZDTConfig(node_id=2), first._settings_path)
        self.assertIn('不匹配', mismatch.settings_status_label.text())
        mismatch.shutdown()
        self.assertEqual(first._settings_path.read_bytes(), original)

    def test_corrupt_file_preserved_and_invalid_fields_fall_back(self):
        page = self.page()
        path = page._settings_path
        path.write_text('{broken', encoding='utf-8')
        corrupt = self.page()
        self.assertIn('读取配置失败', corrupt.settings_status_label.text())
        corrupt.shutdown()
        self.assertEqual(path.read_text(encoding='utf-8'), '{broken')
        document = dict(schema_version=1, profile=page._settings_identity(),
                        values=dict(max_step=-1, release_speed='fast', speed=True,
                                    target_n=float('nan'), max_travel=10**1000, hold_s=6))
        path.write_text(json.dumps(document), encoding='utf-8')
        original = path.read_bytes()
        restored = self.page()
        self.assertEqual(restored.inputs['max_step'].value(), 16)
        self.assertEqual(restored.inputs['release_speed'].value(), 24)
        self.assertEqual(restored.inputs['speed'].value(), 5)
        self.assertEqual(restored.inputs['target_n'].value(), 1)
        self.assertEqual(restored.inputs['hold_s'].value(), 6)
        restored.shutdown()
        self.assertEqual(path.read_bytes(), original)

    def test_atomic_save_failure_preserves_previous_file_and_reports_retry(self):
        page = self.page()
        page.inputs['max_step'].setValue(32)
        page._save_settings()
        original = page._settings_path.read_bytes()
        with patch('pressure_feedback_widget.os.replace', side_effect=PermissionError('locked')):
            page.inputs['max_step'].setValue(64)
            page._save_settings()
        self.assertEqual(page._settings_path.read_bytes(), original)
        self.assertIn('保存失败', page.settings_status_label.text())
        self.assertTrue(page._settings_dirty)
        self.assertEqual(list(self.root.glob('*.tmp')), [])
        page._save_settings()
        self.assertFalse(page._settings_dirty)
        self.assertEqual(self.page().inputs['max_step'].value(), 64)

    def test_editing_combination_is_retained_but_motion_still_requires_valid_parameters(self):
        page = self.page()
        page.inputs['target_n'].setValue(13)
        page._save_settings()
        restored = self.page()
        self.assertEqual(restored.inputs['target_n'].value(), 13)
        with self.assertRaises(FeedbackControlError):
            restored._parameters()
        restored.inputs['force_limit_n'].setValue(15)
        restored._parameters()


class ContactApproachTests(unittest.TestCase):
    @staticmethod
    def table():
        return LearnedPressureMap(PROFILE.model, PROFILE.namespace, 3200, 0,
            ((.05, 616), (.1, 632), (.19, 660), (.52, 716), (1, 765), (30, 1712)),
            contact_zero_travel=596, contact_travel=600, contact_force_n=.01)

    @staticmethod
    def settings():
        return FeedbackSettings(target_n=.1, tolerance_n=.01, force_limit_n=35,
            max_step=512, speed=10, approach_speed=30, settle_s=.1, stable_s=.5, hold_s=1)

    def test_contact_splits_fast_empty_travel_from_slow_loading(self):
        # 接触位置相对学习后移8 step；保留每次命令0.18秒开销及0.01 N量化。
        class ContactPlant(AdaptiveCurvePlant):
            POINTS = ((0, 0), (604, 0), (608, .01), (624, .05), (640, .1), (668, .19), (1720, 30))
            def move_relative_steps(self, steps, **kwargs):
                super().move_relative_steps(steps, **kwargs)
                self.now += .18
        results = []
        plants = []
        for contact in (False, True):
            plant = ContactPlant()
            table = self.table() if contact else replace(self.table(), contact_zero_travel=None,
                                                         contact_travel=None, contact_force_n=None)
            controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
                self.settings(), 0, clock=lambda: plant.now, learned_map=table)
            result = controller.run(acquire_only=True)
            self.assertTrue(result['ok'], result)
            self.assertLessEqual(result['peak_n'], .11+1e-9)
            self.assertLessEqual(abs(plant.force()-.1), .01+1e-9)
            results.append(result)
            plants.append(plant)
        self.assertLess(results[1]['stable_arrival_s'], results[0]['stable_arrival_s']*.7)
        self.assertTrue(results[1]['used_contact_approach'])
        self.assertFalse(results[0]['used_contact_approach'])
        self.assertEqual(plants[1].actions[0], -(596-32))
        self.assertEqual(plants[1].motion_parameters[0]['speed_rpm'], 30)
        self.assertTrue(all(p['speed_rpm'] <= 2 for p in plants[1].motion_parameters[1:]))
        self.assertLess(len(plants[1].actions), len(plants[0].actions))

    def test_contact_rejects_missing_invalid_or_inconsistent_metadata(self):
        for change in (dict(contact_travel=None), dict(contact_zero_travel=601),
                       dict(contact_force_n=0), dict(contact_force_n=.04),
                       dict(contact_reference_n=.03), dict(contact_reference_n=float('nan')),
                       dict(contact_reference_n=.01),
                       dict(contact_travel=float('nan')), dict(contact_travel=617)):
            with self.subTest(change=change), self.assertRaises(FeedbackControlError):
                replace(self.table(), **change)

    def test_small_stable_offsets_learn_full_curve_and_keep_absolute_targets(self):
        for offset in (-.02, -.015, -.01, .01, .02):
            with self.subTest(offset=offset):
                plant = AdaptiveCurvePlant()
                force = plant.force
                plant.force = lambda: force()+offset
                result = plant.run(learning=True, target_n=30, tolerance_n=.6, force_limit_n=35,
                                   max_step=256, speed=10, stable_s=.5)
                self.assertTrue(result['ok'], result)
                table = result['learned_map']
                self.assertEqual(table.contact_reference_n, offset)
                self.assertTrue(.01-1e-9 <= table.contact_force_n-offset <= .02+1e-9)
                self.assertGreater(table.contact_travel, 720)
                self.assertGreater(table.contact_travel, table.contact_zero_travel)
                self.assertEqual(len(table.points), 12)
                self.assertLessEqual(abs(table.points[-1][0]-30), .6)
                self.assertEqual(plant.position, 0)
                plant.actions.clear()
                plant.motion_parameters.clear()
                controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
                    self.settings(), 0, clock=lambda: plant.now, learned_map=table)
                result = controller.run(acquire_only=True)
                self.assertTrue(result['ok'], result)
                self.assertTrue(result['used_contact_approach'])
                self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 30)
                self.assertLessEqual(abs(plant.force()-.1), .01+1e-9)

    def test_changed_reference_falls_back_without_rejecting_pressure_control(self):
        plant = AdaptiveCurvePlant()
        force = plant.force
        plant.force = lambda: force()+.01
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            self.settings(), 0, clock=lambda: plant.now, learned_map=self.table())
        result = controller.run(acquire_only=True)
        self.assertTrue(result['ok'], result)
        self.assertFalse(result['used_contact_approach'])
        self.assertTrue(all(p['speed_rpm'] <= 2 for p in plant.motion_parameters))

    def test_constant_small_offset_alone_is_not_a_contact_point(self):
        plant = Plant()
        plant.force_override = .01
        result = plant.run(learning=True, target_n=30, tolerance_n=.6, force_limit_n=35,
                           acquire_timeout_s=10)
        self.assertFalse(result['ok'], result)
        self.assertGreater(len(plant.actions), 0)
        self.assertIsNone(plant.controller._contact_travel)
        self.assertNotIn('learned_map', result)

    def test_learning_rejects_unstable_starting_reference_before_motion(self):
        plant = Plant()
        plant.force = lambda: .01 if round(plant.now*20) % 2 else 0
        result = plant.run(learning=True, target_n=30, tolerance_n=.6, force_limit_n=35)
        self.assertFalse(result['ok'], result)
        self.assertIn('底数不稳定', result['message'])
        self.assertEqual(plant.actions, [])

    def test_intermittent_one_cent_newton_is_not_learned_as_stable_contact(self):
        plant = Plant()
        plant.force = lambda: .01 if plant.actions and round(plant.now*20) % 4 else 0
        result = plant.run(learning=True, target_n=30, tolerance_n=.6, force_limit_n=35,
                           acquire_timeout_s=10, stable_s=.5)
        self.assertFalse(result['ok'], result)
        self.assertNotIn('learned_map', result)
        self.assertEqual(plant.controller.learning_points, [])
        self.assertIsNone(plant.controller._contact_travel)

    def test_intermediate_loaded_acquisition_does_not_reenter_fast_air_motion(self):
        plant = Plant()
        plant.position = -600
        plant.force_override = .05
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            self.settings(), 0, clock=lambda: plant.now, learned_map=self.table())
        controller._fast_approach()
        self.assertEqual(plant.actions, [])
        self.assertFalse(controller.used_contact_approach)

    def test_fast_air_command_cannot_cross_zero_boundary(self):
        plant = Plant()
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            self.settings(), 0, clock=lambda: plant.now, learned_map=self.table())
        with self.assertRaisesRegex(FeedbackControlError, '接触边界'):
            controller._move(-600, approach_force_n=.01, speed_override=30, free_travel=True)
        self.assertEqual(plant.actions, [])

    def test_contact_fast_motion_stops_on_early_contact_stale_pressure_and_cancel(self):
        for fault in ('early_contact', 'stale', 'cancel', 'stop_failed'):
            with self.subTest(fault=fault):
                plant = Plant()
                plant.force = lambda: round(max(0, (-plant.position-400)*.003), 2)
                destination, speed, last = [0.0], [0.0], [plant.now]
                read, stop = plant.read_feedback, plant.stop
                def advance():
                    distance = destination[0]-plant.position
                    step = min(abs(distance), (plant.now-last[0])*speed[0]*3200/60)
                    plant.position += math.copysign(step, distance) if distance else 0
                    last[0] = plant.now
                def move(steps, **kwargs):
                    plant.actions.append(steps)
                    plant.motion_parameters.append(kwargs)
                    destination[0], speed[0], last[0] = plant.position+steps, kwargs['speed_rpm'], plant.now
                    if fault == 'cancel':
                        plant.cancel_at = plant.now+.1
                def feedback():
                    advance()
                    if fault == 'stale' and plant.actions:
                        plant.fault = 'stale'
                    sample = read()
                    moving = abs(destination[0]-plant.position) > .01
                    sample.update(position_counts=round(plant.position), measured_speed_rpm=speed[0] if moving else 0,
                                  in_position=not moving)
                    return sample
                def brake():
                    if fault == 'stop_failed':
                        raise RuntimeError('stop failed')
                    advance()
                    destination[0] = plant.position
                    stop()
                plant.move_relative_steps, plant.read_feedback, plant.stop = move, feedback, brake
                controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
                    self.settings(), 0, clock=lambda: plant.now, learned_map=self.table())
                if fault == 'early_contact':
                    controller._contact_reference_confirmed = True
                    controller._fast_approach()
                    self.assertTrue(controller._contact_fast_aborted)
                    self.assertLess(-plant.position, -plant.actions[0])
                    self.assertEqual(len(plant.actions), 1)
                else:
                    result = controller.run(acquire_only=True)
                    self.assertFalse(result['ok'], result)
                    self.assertEqual(len(plant.actions), 1)
                    self.assertIsNone(result['stable_arrival_s'])
                    self.assertEqual(result['stop_confirmed'], fault != 'stop_failed')
                    if fault == 'stale':
                        self.assertIn('0.15秒', result['message'])


class FastAcquisitionTests(unittest.TestCase):
    def table(self):
        return LearnedPressureMap(PROFILE.model, PROFILE.namespace, PROFILE.steps_per_turn, 0,
                                  tuple((force, travel) for travel, force in AdaptiveCurvePlant.POINTS if force > 0))

    def settings(self, target=20):
        return FeedbackSettings(target_n=target, tolerance_n=target*.02, force_limit_n=35,
                                max_step=512, speed=10, approach_speed=30, settle_s=.1, stable_s=.5)

    def test_timed_curve_20n_uses_two_speeds_and_arrives_within_three_seconds(self):
        plant = AdaptiveCurvePlant()
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, self.settings(), 0,
            clock=lambda: plant.now, learned_map=self.table())
        result = controller.run(acquire_only=True)
        self.assertTrue(result['ok'], result)
        self.assertTrue(result['used_learned_map'])
        self.assertLessEqual(result['stable_arrival_s'], 3)
        self.assertGreaterEqual(result['stable_arrival_s']-result['first_in_band_s'], .5)
        self.assertEqual(len(plant.actions), 2)
        self.assertEqual([p['speed_rpm'] for p in plant.motion_parameters], [30, 10])
        self.assertLessEqual(abs(plant.force()-20), .4)
        self.assertLessEqual(result['peak_n'], 20.4)

    def test_missing_or_invalid_table_cannot_enable_fast_blind_motion(self):
        for table in (None, replace(self.table(), release_position=100)):
            plant = AdaptiveCurvePlant()
            controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, self.settings(), 0,
                clock=lambda: plant.now, learned_map=table)
            result = controller.run(acquire_only=True)
            self.assertTrue(result['ok'], result)
            self.assertFalse(result['used_learned_map'])
            self.assertLessEqual(abs(plant.actions[0]), 256)
            self.assertTrue(all(p['speed_rpm'] == 10 for p in plant.motion_parameters))

    def test_new_speed_default_remains_valid_for_legacy_motor(self):
        from motor_mapping_interface import LEGACY_PROFILE
        replace(FeedbackSettings(), release_speed=LEGACY_PROFILE.speed_max).validate(LEGACY_PROFILE)

    def test_fast_motion_monitors_early_contact_stale_pressure_and_cancel(self):
        for fault in ('early_contact', 'stale', 'cancel'):
            with self.subTest(fault=fault):
                plant = AdaptiveCurvePlant()
                destination, speed, last = [0], [0], [plant.now]
                force, read, stop = plant.force, plant.read_feedback, plant.stop
                if fault == 'early_contact':
                    plant.force = lambda: force()*2

                def advance():
                    distance = destination[0]-plant.position
                    travel = min(abs(distance), (plant.now-last[0])*speed[0]*3200/60)
                    plant.position += math.copysign(travel, distance)
                    last[0] = plant.now

                def move(steps, **kwargs):
                    plant.actions.append(steps)
                    plant.motion_parameters.append(kwargs)
                    destination[0], speed[0], last[0] = plant.position+steps, kwargs['speed_rpm'], plant.now
                    if fault == 'cancel':
                        plant.cancel_at = plant.now+.3

                def feedback():
                    advance()
                    if fault == 'stale' and plant.actions:
                        plant.fault = 'stale'
                    sample = read()
                    moving = abs(destination[0]-plant.position) > .01
                    sample.update(measured_speed_rpm=speed[0] if moving else 0,
                                  in_position=not moving, position_counts=round(plant.position))
                    return sample

                def brake():
                    advance()
                    destination[0] = plant.position
                    stop()

                plant.move_relative_steps, plant.read_feedback, plant.stop = move, feedback, brake
                plant.read_feedback()
                controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, self.settings(), 0,
                    clock=lambda: plant.now, learned_map=self.table())
                # 聚焦高速段，异常必须结束该段，不能叠加下一个下压命令。
                if fault == 'early_contact':
                    controller._fast_approach()
                    self.assertLess(-plant.position, -plant.actions[0])
                    self.assertLess(plant.force(), 20)
                    self.assertGreater(plant.stop_count, 0)
                else:
                    result = controller.run(acquire_only=True)
                    self.assertFalse(result['ok'])
                    self.assertTrue(result['stop_confirmed'])
                    self.assertEqual(len(plant.actions), 1)
                    self.assertIsNone(result['stable_arrival_s'])
                self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 30)

    def test_stationary_confirmation_does_not_repeat_settle_delay(self):
        plant = Plant()
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, self.settings(), 0,
                                                clock=lambda: plant.now)
        controller._move(-5)
        start = plant.now
        controller._window()
        after_move = plant.now-start
        start = plant.now
        previous_sequence = controller._stationary_samples[-1][2]
        controller._window()
        stationary = plant.now-start
        self.assertGreaterEqual(after_move, .3-1e-9)
        self.assertLess(stationary, .1)
        self.assertGreater(stationary, 0)
        frames = controller._stationary_samples
        self.assertNotEqual(frames[-1][2], previous_sequence)
        self.assertGreaterEqual(len(frames), 3)
        self.assertGreaterEqual(frames[-1][1]-frames[0][1], .2-1e-9)
        controller._move(-5)
        stopped = plant.now
        controller._window()
        self.assertGreaterEqual(plant.now-stopped, .3-1e-9)
        self.assertTrue(all(frame[1] >= stopped+.1-1e-9 for frame in controller._stationary_samples))

    def test_stationary_confirmation_keeps_full_stable_duration(self):
        plant = Plant()
        plant.force_override = 1
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            self.settings(1), 0, clock=lambda: plant.now)
        result = controller.run(acquire_only=True, require_zero_start=False)
        self.assertTrue(result['ok'], result)
        self.assertGreaterEqual(result['stable_arrival_s'], .5-1e-9)
        self.assertLess(result['stable_arrival_s'], .6)
        self.assertEqual(plant.actions, [])

    def test_sampling_gap_restarts_stability_and_fresh_window(self):
        plant = Plant()
        plant.force_override = 1
        read = plant.read_feedback
        first = plant.now
        def feedback():
            # 已连续合格0.3秒后中断0.3秒，但未触发0.5秒过期；不得把中断算作稳定。
            plant.fault = 'stale' if .3 <= plant.now-first < .6 else None
            return read()
        plant.read_feedback = feedback
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            self.settings(1), 0, clock=lambda: plant.now)
        result = controller.run(acquire_only=True, require_zero_start=False)
        self.assertTrue(result['ok'], result)
        self.assertGreaterEqual(result['stable_arrival_s'], 1.1-1e-9)
        self.assertTrue(all(frame[1] >= first+.6-1e-9 for frame in controller._stationary_samples))

    def test_nonlinear_learning_curve_uses_interpolated_loading_distance(self):
        # 当前点0.5 N至1 N只有末端极短一段变硬，不能把整段都当作该最大刚度。
        table = LearnedPressureMap(PROFILE.model, PROFILE.namespace, 3200, 0,
            ((.5, 700), (.98, 760), (2, 800), (30, 1700)))
        plant = Plant()
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
            self.settings(1), 0, clock=lambda: plant.now, learned_map=table)
        controller.used_learned_map = True
        controller._seed_learned_response(-700, .5)
        step = controller._adjustment_steps(.5, .5)
        self.assertGreaterEqual(step, 50)
        self.assertLess(step, table.travel_at(1)-table.travel_at(.5))
        # 超出学习范围、或位置不符合学习曲线时，仍不允许用曲线预测。
        controller._response_slopes.clear()
        controller.settings = self.settings(31)
        controller._seed_learned_response(-table.travel_at(25), 25)
        self.assertFalse(controller._response_slopes)

    def test_recorded_loading_curve_reaches_targets_with_fewer_stops(self):
        # 2026-09-20 14:20学习曲线；每个命令加入0.18秒开销，验证绝对压力与峰值。
        points = ((.04, 607), (.11, 631), (.19, 652), (.52, 709), (.98, 760),
                  (1.96, 838), (4.92, 990), (9.82, 1173), (14.78, 1331),
                  (19.81, 1457), (24.7, 1594), (29.425, 1703))
        class RecordedPlant(AdaptiveCurvePlant):
            POINTS = ((0, 0), (567, 0), (571, .01))+tuple((x, f) for f, x in points)+((1800, 35),)
            def move_relative_steps(self, steps, **kwargs):
                super().move_relative_steps(steps, **kwargs)
                self.now += .18
        table = LearnedPressureMap(PROFILE.model, PROFILE.namespace, 3200, 0, points,
                                   contact_zero_travel=567, contact_travel=571, contact_force_n=.01)
        for target, max_moves, time_limit in ((1, 3, 2.7), (2, 3, 2.8), (10, 2, 2.5),
                                             (15, 2, 2.6), (25, 2, 2.8)):
            with self.subTest(target=target):
                plant = RecordedPlant()
                controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant,
                    self.settings(target), 0, clock=lambda: plant.now, learned_map=table)
                result = controller.run(acquire_only=True)
                self.assertTrue(result['ok'], result)
                self.assertLessEqual(result['move_count'], max_moves)
                self.assertLess(result['stable_arrival_s'], time_limit)
                self.assertLessEqual(abs(plant.force()-target), target*.02+1e-9)
                self.assertLessEqual(result['peak_n'], target*1.02+1e-9)
                self.assertEqual(plant.motion_parameters[0]['speed_rpm'], 30)
                self.assertTrue(all(p['speed_rpm'] == 10 for p in plant.motion_parameters[1:]))

    def test_raw_out_of_band_samples_prevent_false_stable_arrival(self):
        plant = Plant()
        plant.force = lambda: 1.1 if round(plant.now*20) % 4 == 0 else 1.0
        settings = replace(self.settings(1), acquire_timeout_s=10)
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, settings, 0,
                                                clock=lambda: plant.now)
        result = controller.run(acquire_only=True, require_zero_start=False)
        self.assertFalse(result['ok'])
        self.assertIsNotNone(result['first_in_band_s'])
        self.assertIsNone(result['stable_arrival_s'])
        self.assertEqual(plant.actions, [])

    def test_changed_material_cannot_seed_unmatched_learning_stiffness(self):
        plant = Plant()
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, self.settings(), 0,
                                                clock=lambda: plant.now, learned_map=self.table())
        controller.used_learned_map = True
        controller._seed_learned_response(-1000, 15)
        self.assertFalse(controller._response_slopes)
        self.assertEqual(controller._adjustment_gain, .75)

    def test_arrival_metrics_survive_hold_and_are_recorded(self):
        plant = AdaptiveCurvePlant()
        settings = replace(self.settings(), hold_s=1)
        controller = PressureFeedbackController(plant, PROFILE, plant.samples, plant, settings, 0,
            clock=lambda: plant.now, learned_map=self.table(), on_record=plant.records.append)
        result = controller.run()
        self.assertTrue(result['ok'], result)
        self.assertLessEqual(result['stable_arrival_s'], 3)
        self.assertGreater(plant.now-controller.started, result['stable_arrival_s']+1)
        stable = [r['stable_arrival_s'] for r in plant.records if r['stable_arrival_s'] is not None]
        self.assertTrue(stable)
        self.assertEqual(set(stable), {result['stable_arrival_s']})


if __name__ == '__main__':
    unittest.main(verbosity=2)
