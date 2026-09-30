"""复现静止窗口速度瞬变、停止反馈延迟及最后验收点连续失败后的保留进度恢复。"""
from dataclasses import replace
import json
import unittest
from unittest.mock import patch

import test_pressure_feedback as feedback
import test_high_force_calibration as calibration
from pressure_feedback_control import PressureFeedbackController, FeedbackControlError, FeedbackSettings


class StopConfirmationTests(unittest.TestCase):
    def controller(self, plant):
        return PressureFeedbackController(plant, feedback.PROFILE, plant.samples, plant,
            replace(FeedbackSettings(), target_n=1, tolerance_n=.05), 0, clock=lambda: plant.now)

    def test_transient_speed_discards_old_window_and_resamples_after_stop(self):
        plant = feedback.Plant()
        plant.position = -35
        controller = self.controller(plant)
        controller._window()
        before = plant.now
        read = plant.read_feedback
        count = [0]
        def glitch():
            sample = read()
            count[0] += 1
            if count[0] <= 3:
                sample.update(measured_speed_rpm=10, running_state=2, in_position=False)
            return sample
        plant.read_feedback = glitch
        _, force = controller._window()
        self.assertEqual(force, 1)
        self.assertEqual(plant.stop_count, 1)
        self.assertEqual(plant.actions, [])
        self.assertGreaterEqual(plant.now-before, .4)
        self.assertTrue(all(stamp >= controller._stationary_ready for _, stamp, _ in controller._stationary_samples))
        self.assertGreaterEqual(len(controller.motion_diagnostics), 2)
        rows = controller.motion_diagnostics[0]['feedback']
        self.assertEqual(rows[-1]['measured_speed_rpm'], 10)
        self.assertIn('feedback_age_s', rows[-1])
        json.dumps(list(controller.motion_diagnostics), allow_nan=False)

    def test_final_stop_waits_for_delayed_speed_feedback(self):
        plant = feedback.Plant()
        controller = self.controller(plant)
        read = plant.read_feedback
        def delayed():
            sample = read()
            if plant.now < 10.25:
                sample.update(measured_speed_rpm=12, running_state=2, in_position=False)
            return sample
        plant.read_feedback = delayed
        controller._stop_and_confirm(monitor_operation=False)
        self.assertGreaterEqual(plant.now, 10.35-1e-8)
        self.assertEqual(plant.stop_count, 1)
        self.assertFalse(plant.actions)

    def test_duplicate_stale_faulted_and_drifting_position_never_confirm_stop(self):
        for mode in ('duplicate', 'stale', 'fault', 'current', 'position', 'moving'):
            with self.subTest(mode=mode):
                plant = feedback.Plant()
                controller = self.controller(plant)
                read = plant.read_feedback
                calls = [0]
                def invalid():
                    sample = read()
                    calls[0] += 1
                    if mode == 'duplicate': sample['sampled_monotonic'] = 10
                    if mode == 'stale': sample['sampled_monotonic'] = plant.now-1
                    if mode == 'fault': sample['running_state'] = 3
                    if mode == 'current': sample['phase_current_ma'] = 5000
                    if mode == 'position': sample['position_counts'] = -calls[0]*2
                    if mode == 'moving': sample['measured_speed_rpm'] = 10
                    return sample
                plant.read_feedback = invalid
                with self.assertRaises(FeedbackControlError):
                    controller._stop_and_confirm(monitor_operation=False)
                self.assertFalse(plant.actions)
                self.assertGreaterEqual(plant.now, 12)
                self.assertTrue(controller.motion_diagnostics)

    def test_pressure_protection_during_stop_confirmation_is_not_suppressed(self):
        plant = feedback.Plant()
        plant.force_override = 36
        controller = self.controller(plant)
        with self.assertRaisesRegex(FeedbackControlError, '保护上限'):
            controller._stop_and_confirm()
        self.assertFalse(plant.actions)


class CalibrationRetryTests(unittest.TestCase):
    run_plan = calibration.HighCalibrationTests.run_plan

    def test_final_validation_point_survives_three_failures_and_keeps_saved_points(self):
        context, failures = {}, []
        original = PressureFeedbackController._window
        def progress(plant, row):
            if row.get('phase') == 'point': context.update(row)
            if row.get('phase') == 'retry_wait':
                self.assertEqual(plant.position, 0)
                self.assertEqual(plant.force(), 0)
        def window(controller):
            if (context.get('stage') == 'validation' and context.get('target_n') == 30
                    and not controller.releasing and controller.motor.position < 0 and len(failures) < 3):
                failures.append(controller.clock())
                controller._motion_event('复现最后30 N验收点运动反馈异常')
                raise FeedbackControlError('采样期间反复出现运动反馈')
            return original(controller)
        with patch.object(PressureFeedbackController, '_window', window):
            plant, result, phases, _ = self.run_plan(rounds=3, staged=True, rest_seconds=0, progress_hook=progress)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['records']), 30)
        self.assertEqual(len(result['validation_records']), 10)
        self.assertTrue(all(r['attempt'] == 1 for r in result['validation_records'][:-1]))
        self.assertEqual(result['validation_records'][-1]['attempt'], 4)
        self.assertEqual(len(result['point_failures']), 3)
        self.assertTrue(all(r['stage'] == 'validation' and r['target_n'] == 30 for r in result['point_failures']))
        self.assertGreaterEqual(result['retry_waits'][-1]['elapsed_s'], 30)
        self.assertTrue(result['retry_waits'][-1]['completed'])
        self.assertTrue(result['motion_diagnostics'])
        self.assertEqual(plant.position, 0)
        self.assertEqual(len([p for p in phases if p.get('phase') == 'point' and p.get('stage') == 'validation']), 10)
        json.dumps(result, allow_nan=False)

    def test_transient_pressure_loss_waits_at_release_then_continues(self):
        started = []
        def glitch(plant, row):
            if row.get('phase') == 'rest' and not started:
                started.append(plant.now)
                plant.fault = 'stale'
            if row.get('phase') == 'retry_wait':
                self.assertEqual(plant.position, 0)
                if plant.now-started[0] >= 5:
                    plant.fault = None
        plant, result, _, _ = self.run_plan(rounds=1, rest_seconds=1, progress_hook=glitch)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['records']), 10)
        self.assertEqual(result['records'][0]['attempt'], 1)
        self.assertEqual(result['records'][1]['attempt'], 2)
        self.assertEqual(plant.position, 0)

    def test_stop_in_retry_wait_preserves_release_and_progress(self):
        def fault(plant, row):
            if row.get('phase') == 'rest': plant.fault = 'stale'
            if row.get('phase') == 'retry_wait': plant.cancelled = True
        plant, result, _, _ = self.run_plan(rounds=1, rest_seconds=1, progress_hook=fault)
        self.assertFalse(result['ok'])
        self.assertTrue(result['cancelled'])
        self.assertTrue(result['release_position_confirmed'])
        self.assertEqual(len(result['records']), 1)
        self.assertEqual(plant.position, 0)

    def test_emergency_during_retry_wait_never_resumes_motion(self):
        actions_at_stop = []
        def fault(plant, row):
            if row.get('phase') == 'rest': plant.fault = 'stale'
            if row.get('phase') == 'retry_wait':
                actions_at_stop[:] = plant.actions
                plant.emergency_cancelled = True
        plant, result, _, _ = self.run_plan(rounds=1, rest_seconds=1, progress_hook=fault)
        self.assertFalse(result['ok'])
        self.assertEqual(plant.actions, actions_at_stop)
        self.assertEqual(plant.position, 0)
        self.assertEqual(len(result['records']), 1)

    def test_fault_and_out_of_range_position_never_send_recovery_motion(self):
        from high_force_calibration import recover_release
        for mode in ('overcurrent', 'fault', 'unknown_position', 'outside_travel'):
            with self.subTest(mode=mode):
                plant = feedback.Plant()
                plant.position = -500
                read = plant.read_feedback
                def invalid():
                    sample = read()
                    if mode == 'overcurrent': sample['phase_current_ma'] = 5000
                    if mode == 'fault': sample['running_state'] = 4
                    if mode == 'unknown_position': sample['position_counts'] = float('nan')
                    if mode == 'outside_travel': sample['position_counts'] = -7000
                    return sample
                plant.read_feedback = invalid
                emergency = calibration.ClockCancel(plant, lambda: plant.now >= 13)
                result = recover_release(plant, feedback.PROFILE, plant.samples, FeedbackSettings(),
                    0, emergency, clock=lambda: plant.now)
                self.assertFalse(result['release_position_confirmed'])
                self.assertEqual(plant.actions, [])


if __name__ == '__main__':
    unittest.main()
