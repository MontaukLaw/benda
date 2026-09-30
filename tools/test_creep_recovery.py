"""无硬件故障注入：卸载、重试、硬超时、断点以及原始记录隔离。"""
import json
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "PY_SRIPT" / "algo_monitor"))
from force_press_control_widget import ForcePressControlWidget as Controller, ForceControlError
from creep_test_safety import CreepDeadlineEvent
from long_duration_creep_test_widget import (
    SQLiteCreepRecorder, completed_trial_indices, find_resumable_test,
    LongDurationCreepTestWidget,
    build_creep_plan, estimate_creep_plan,
)
from qt_compat import QtWidgets
from step_moto_can_driver import PD42S1CommunicationError


class Harness:
    _run_creep_test = Controller._run_creep_test
    _finish_creep_failure = Controller._finish_creep_failure

    def __init__(self, failures=()):
        self._state_lock = threading.Lock()
        self.failures = iter(failures)
        self.log = []
        self._run_creep_release = lambda *args: self.log.append("release")
        self._creep_emergency_release = lambda *args: self.log.append("unload")

    def _run_creep_trial(self, motor, tracker, stop, params, trial, total, release):
        self.log.append(("attempt", trial["trial_index"], trial["attempt_index"]))
        error = next(self.failures, None)
        if error:
            raise error
        return dict(trial)

    def _creep_progress(self, phase, trial, total, **values):
        self.log.append((phase, dict(trial), values))

    def _run_creep_wait(self, motor, tracker, stop, params, trial, total, phase, duration):
        if stop.is_set():
            raise ForceControlError("user_stop", "stop")
        self.log.append((phase, duration))

    def run(self, count=2, stop=None):
        return self._run_creep_test(None, {}, stop or threading.Event(), {
            "creep_trials": [{"planned_trial_index": i} for i in range(1, count + 1)],
            "sweep_plan": {},
        })


def timeout_error():
    return ForceControlError("motion_timeout", "target -28464 actual -28229")


class RecoveryTests(unittest.TestCase):
    def test_timeout_unloads_before_retry_and_keeps_completed_trials(self):
        h = Harness([None, timeout_error(), None])
        result = h.run()
        self.assertEqual([t["attempt_index"] for t in result["creep_completed_trials"]], [1, 2])
        unload = h.log.index("unload")
        self.assertEqual(h.log[unload + 1], ("creep_retry_recovery", 360.0))
        self.assertEqual(h.log[unload + 2], ("attempt", 2, 2))
        failed = [x for x in h.log if isinstance(x, tuple) and x[0] == "creep_trial_failed"]
        self.assertFalse(failed[0][2]["data_valid"])

    def test_three_failures_defer_only_failed_condition(self):
        h = Harness([timeout_error() for _ in range(3)])
        result = h.run()
        self.assertEqual([x["trial_index"] for x in result["creep_completed_trials"]], [2])
        self.assertEqual([x["trial_index"] for x in result["creep_skipped_trials"]], [1])
        self.assertEqual(h.log.count("unload"), 3)

    def test_persistent_failure_pauses_after_three_conditions_unloaded(self):
        h = Harness([timeout_error() for _ in range(9)])
        with self.assertRaisesRegex(ForceControlError, "连续三个"):
            h.run(4)
        self.assertEqual(h.log.count("unload"), 9)
        self.assertFalse(any(x == ("attempt", 4, 1) for x in h.log))

    def test_force_limit_unloads_without_retrying_pressure(self):
        h = Harness([ForceControlError("force_limit", "overload")])
        with self.assertRaisesRegex(ForceControlError, "overload"):
            h.run()
        self.assertEqual(h.log.count("unload"), 1)
        self.assertNotIn(("attempt", 1, 2), h.log)

    def test_finalizer_covers_unknown_error_and_user_stop(self):
        for reason in ("error", "user_stop", "motor_already_running"):
            h = Harness()
            result = {"stop_reason": reason, "message": "failure"}
            h._finish_creep_failure(None, {}, {}, result)
            self.assertTrue(result["safety_released"])
            self.assertEqual(h.log, ["unload"])

    def test_failed_release_is_not_reported_safe_or_retried_forever(self):
        h = Harness()
        h._creep_emergency_release = Mock(side_effect=RuntimeError("CAN disconnected"))
        result = {"stop_reason": "error"}
        h._finish_creep_failure(None, {}, {}, result)
        self.assertFalse(result["safety_released"])
        h._finish_creep_failure(None, {}, {}, result)
        self.assertEqual(h._creep_emergency_release.call_count, 1)

    def test_emergency_release_uses_independent_event_and_two_attempts(self):
        h = SimpleNamespace(_creep_progress=Mock(), _safe_brake=Mock(),
                            _call_with_motor_communication_retry=Mock())
        tokens = []
        def release(motor, tracker, stop, params):
            self.assertFalse(stop.is_set())
            tokens.append(stop)
            if len(tokens) == 1:
                raise timeout_error()
            return {"final_force_n": 0}
        h._run_creep_release = release
        result = Controller._creep_emergency_release(h, SimpleNamespace(clear_state=Mock()), {}, {})
        self.assertEqual(result["final_force_n"], 0)
        self.assertEqual(len(tokens), 2)

    def test_deadline_cannot_be_extended_by_communication_recovery(self):
        with patch("creep_test_safety.time.monotonic", return_value=10) as clock:
            event = CreepDeadlineEvent(threading.Event(), 30, ForceControlError)
            self.assertFalse(event.is_set())
            clock.return_value = 40
            with self.assertRaisesRegex(ForceControlError, "硬超时"):
                event.is_set()

    def test_deadline_keeps_user_stop(self):
        stop = threading.Event()
        event = CreepDeadlineEvent(stop, 30, ForceControlError)
        stop.set()
        self.assertTrue(event.is_set())
        self.assertTrue(event.wait(10))

    def test_persistent_communication_error_leaves_retry_loop(self):
        callback = Mock(side_effect=PD42S1CommunicationError("offline"))
        h = SimpleNamespace(_creep_active=True, _record_event=Mock(),
                            operation_progress=Mock(), _ocr_snapshot=lambda: {})
        stop = SimpleNamespace(is_set=lambda: False, wait=lambda seconds: False)
        with self.assertRaises(ForceControlError) as caught:
            Controller._call_with_motor_communication_retry(h, "feedback", callback, stop)
        self.assertEqual(caught.exception.reason, "motor_communication_timeout")
        self.assertEqual(callback.call_count, 3)

    def test_release_needs_fresh_zero_samples_and_position_confirmation(self):
        samples = iter([{"accepted_sequence": 0}] + [
            {"accepted_sequence": i, "accepted_force_n": 0, "accepted_age_seconds": 0.01}
            for i in range(1, 4)])
        h = SimpleNamespace(_run_release=Mock(return_value={}),
                            _ocr_snapshot=lambda: next(samples),
                            _poll_checked=Mock(return_value={"position_counts": 0, "measured_speed_rpm": 0}))
        result = Controller._run_creep_release(h, None, {}, threading.Event(),
                                              {"max_current_ma": 2800, "release_position_counts": 0})
        self.assertEqual(result["final_force_n"], 0)
        self.assertEqual(h._poll_checked.call_count, 3)

    def test_stale_raw_sample_triggers_stop_in_recorder(self):
        h = SimpleNamespace(_running=True, _recorder=Mock(), _stopping=False,
                            _sensor_sample_provider=lambda: {"sequence": 10, "has_raw": True, "age_seconds": 50, "channel_count": 8},
                            _raw_missing_since=1, phase_label=Mock(), stop_test=Mock())
        with patch("long_duration_creep_test_widget.time.monotonic", return_value=5):
            LongDurationCreepTestWidget._capture_latest_sample(h)
        h.stop_test.assert_called_once()

    def test_235_count_stopped_error_is_accepted_in_256_count_window(self):
        feedback = [{"position_counts": 3, "measured_speed_rpm": 0, "running_state": 2}] + [
            {"position_counts": -28229, "measured_speed_rpm": 0, "running_state": 2}
        ] * 4
        motor = SimpleNamespace(clear_state=Mock(), move_relative_counts=Mock(return_value={}))
        h = SimpleNamespace(_poll_checked=Mock(side_effect=feedback),
                            _set_motion_context=Mock(), _record_event=Mock(),
                            _operation_id=1, _control_mode='creep_test', _target_force_n=2,
                            _latest_sensor_sample=lambda: None, manual_observation_event=Mock(),
                            operation_progress=Mock(), _ocr_snapshot=lambda: {},
                            _communication_recovery_generation=0, _creep_active=True,
                            _call_with_motor_communication_retry=lambda action, callback, *args, **kwargs: callback())
        stop = SimpleNamespace(is_set=lambda: False, wait=lambda seconds: False)
        result = Controller._move_relative(h, motor, {}, stop, -28464, 3, 10, 2800, 30.4,
                                          "press", 28464, position_tolerance_counts=256,
                                          fixed_target_counts=-28464)
        self.assertEqual(result["position_counts"], -28229)
        self.assertEqual(motor.move_relative_counts.call_args.args[0], -28467)
        h.manual_observation_event.emit.assert_called_once()
        self.assertEqual(h.manual_observation_event.emit.call_args.args[0]['event'], 'motion_dispatched')

    def test_release_near_reference_never_commands_negative_motion(self):
        motor = SimpleNamespace(move_relative_counts=Mock())
        h = SimpleNamespace(_poll_checked=Mock(return_value={"position_counts": 10}))
        Controller._move_relative(h, motor, {}, threading.Event(), 400, 3, 10, 2800,
                                  None, "release", 400, fixed_target_counts=0)
        motor.move_relative_counts.assert_not_called()


class ResumeTests(unittest.TestCase):
    def test_sqlite_commit_recovers_noncontiguous_completed_indices(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SQLiteCreepRecorder(directory, {})
            session = recorder.start()
            recorder.enqueue_event("trial_complete", {"phase": "creep_trial_complete", "trial_index": 2, "attempt_index": 3})
            snapshot = recorder.stop()
            self.assertEqual(snapshot["completed_trial_indices"], [2])
            done = completed_trial_indices({"schema_version": 2}, session)
            self.assertEqual(done, {2})

    def test_legacy_prefix_and_failed_start_do_not_hide_original_data(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = root / "creep_1"
            session.mkdir()
            old = {"schema_version": 1, "status": "stopped", "next_trial_index": 19,
                   "trials": [{"planned_trial_index": i} for i in range(1, 73)]}
            (session / "manifest.json").write_text(json.dumps(old), encoding="utf-8")
            rejected = root / "creep_2"
            rejected.mkdir()
            (rejected / "manifest.json").write_text(json.dumps(dict(old, status="start_rejected", resumed_from=str(session))))
            path, manifest, remaining = find_resumable_test(root)
            self.assertEqual(path.parent, session)
            self.assertEqual(len(remaining), 54)

    def test_completed_child_prevents_restarting_its_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, status, parent in [("creep_1", "stopped", None), ("creep_2", "completed", str(root / "creep_1"))]:
                session = root / name
                session.mkdir()
                (session / "manifest.json").write_text(json.dumps({
                    "schema_version": 2, "status": status, "resumed_from": parent,
                    "completed_trial_indices": [], "trials": [{"planned_trial_index": 1}],
                }))
            self.assertIsNone(find_resumable_test(root))


class WidgetLifecycleTests(unittest.TestCase):
    def test_widget_retry_statistics_and_durable_finish(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(LongDurationCreepTestWidget, "_load_settings"), \
                patch.object(LongDurationCreepTestWidget, "_save_settings"), \
                patch("long_duration_creep_test_widget.creep_test_directory", return_value=Path(directory)):
            controller = SimpleNamespace(operation_progress=Mock(), operation_finished=Mock(),
                                         start_creep_test=Mock(return_value=True))
            motor = SimpleNamespace(is_can_connected=lambda: True)
            widget = LongDurationCreepTestWidget(controller, motor, lambda: {
                "has_raw": True, "channel_count": 8, "age_seconds": 0.01,
            })
            widget.set_pressure_sample({"accepted": True})
            config = widget._read_config()
            plan = build_creep_plan(config)[:2]
            widget._start_plan(config, plan, estimate_creep_plan(plan, 10))
            self.assertTrue(widget.is_running())
            self.assertFalse(widget.preset_combo.isEnabled())
            session = widget._recorder.session_dir
            widget._on_force_progress({"phase": "creep_trial_start", "trial_index": 1, "attempt_index": 1})
            widget._current_statistics["hardware_raw_sum"].add(999)
            widget._on_force_progress({"phase": "creep_trial_failed", "trial_index": 1, "attempt_index": 1})
            widget._on_force_progress({"phase": "creep_trial_start", "trial_index": 1, "attempt_index": 2})
            self.assertEqual(widget._phase_statistics, {})
            widget._on_force_progress({"phase": "creep_trial_complete", "trial_index": 1,
                                       "attempt_index": 2, "trial_result": {"trial_index": 1, "attempt_index": 2}})
            widget._on_force_finished({"mode": "creep_test", "ok": False,
                                       "safety_released": True, "message": "stopped unloaded"})
            manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["completed_trial_indices"], [1])
            self.assertEqual(manifest["next_trial_index"], 2)
            self.assertTrue(manifest["safety_released"])
            self.assertTrue(widget.preset_combo.isEnabled())
            self.assertFalse(widget.is_running())
            widget.close()


if __name__ == "__main__":
    unittest.main()
