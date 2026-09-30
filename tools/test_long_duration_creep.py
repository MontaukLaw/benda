import json
import sqlite3
import struct
import sys
import tempfile
import threading
import unittest
from pathlib import Path


MODULE_ROOT = Path(__file__).resolve().parents[1] / "PY_SRIPT" / "algo_monitor"
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from long_duration_creep_test_widget import (  # noqa: E402
    LongDurationCreepTestWidget,
    SAMPLE_COLUMNS,
    SQLiteCreepRecorder,
    build_creep_plan,
    estimate_creep_plan,
)
from data_buffer import AlgoDataBuffer  # noqa: E402
from frame_parser import AlgoFrameParser  # noqa: E402
from force_press_control_widget import ForcePressControlWidget  # noqa: E402


class CreepPlanTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            "high_forces_n": (12.0, 16.0, 20.0),
            "probe_forces_n": (2.0, 6.0, 10.0),
            "high_hold_seconds": (1.0, 10.0, 60.0),
            "recovery_seconds": (1.0, 10.0, 120.0, 360.0),
            "repeat_count": 3,
            "probe_hold_seconds": 3.0,
            "baseline_wait_seconds": 360.0,
            "inter_trial_seconds": 10.0,
            "random_seed": 20260910,
        }

    def test_full_plan_covers_each_factor_combination_once_per_repeat(self):
        plan = build_creep_plan(self.config)
        response = [item for item in plan if item["category"] == "history_response"]
        baselines = [item for item in plan if item["category"].startswith("baseline")]
        self.assertEqual(len(plan), 342)
        self.assertEqual(len(response), 324)
        self.assertEqual(len(baselines), 18)
        combinations = {
            (
                item["repeat_index"],
                item["high_force_n"],
                item["high_hold_seconds"],
                item["recovery_seconds"],
                item["probe_force_n"],
            )
            for item in response
        }
        self.assertEqual(len(combinations), len(response))
        self.assertEqual(
            [item["planned_trial_index"] for item in plan],
            list(range(1, len(plan) + 1)),
        )

    def test_estimate_accounts_for_long_recovery_and_disk(self):
        plan = build_creep_plan(self.config)
        estimate = estimate_creep_plan(plan, self.config["inter_trial_seconds"])
        self.assertEqual(estimate["trial_count"], len(plan))
        self.assertGreater(estimate["total_seconds"], 18 * 3600)
        self.assertGreater(estimate["estimated_rows"], 1_400_000)
        self.assertGreater(estimate["estimated_bytes"], 1000 * 1024 * 1024)

    def test_screening_plan_reduces_time_and_storage(self):
        screening = dict(self.config)
        screening.update(
            high_forces_n=(12.0, 20.0),
            probe_forces_n=(2.0, 6.0),
            high_hold_seconds=(1.0, 60.0),
            repeat_count=2,
        )
        plan = build_creep_plan(screening)
        estimate = estimate_creep_plan(plan, screening["inter_trial_seconds"])
        self.assertEqual(len(plan), 72)
        self.assertGreater(estimate["total_seconds"], 4 * 3600)
        self.assertLess(estimate["total_seconds"], 4.5 * 3600)
        self.assertLess(estimate["estimated_rows"], 350_000)

    def test_wait_sampling_keeps_transient_and_thins_long_tail(self):
        class Context:
            _phase_context = {"phase": "creep_recovery_wait"}

        self.assertEqual(LongDurationCreepTestWidget._capture_interval(Context(), 5), 0)
        self.assertEqual(LongDurationCreepTestWidget._capture_interval(Context(), 10), 0.1)
        self.assertEqual(LongDurationCreepTestWidget._capture_interval(Context(), 120), 0.5)
        Context._phase_context = {"phase": "creep_high_press"}
        self.assertEqual(LongDurationCreepTestWidget._capture_interval(Context(), 120), 0)


class SQLiteRecorderTests(unittest.TestCase):
    def test_raw_rows_and_metadata_are_queryable(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            recorder = SQLiteCreepRecorder(
                temporary_directory,
                {"purpose": "unit_test"},
                max_part_mib=64,
            )
            session_dir = recorder.start()
            row = {name: None for name, _sql_type in SAMPLE_COLUMNS}
            row.update(
                session_id=recorder.session_id,
                sensor_sequence=123,
                sensor_has_raw=1,
                trial_index=7,
                phase="creep_probe_hold",
                ch4_hardware_raw=45678,
                ch4_baseline=3200,
                ch4_signal=10240,
                die_temperature_celsius=25.3,
                die_temperature_age_seconds=0.2,
            )
            recorder.enqueue_sample(row)
            recorder.enqueue_event("unit_test_event", {"trial": 7})
            snapshot = recorder.stop()
            self.assertIsNone(snapshot["error"])
            self.assertEqual(snapshot["sample_count"], 1)
            self.assertEqual(snapshot["event_count"], 1)
            self.assertEqual(len(snapshot["parts"]), 1)

            database_path = session_dir / snapshot["parts"][0]
            connection = sqlite3.connect(str(database_path))
            try:
                values = connection.execute(
                    "SELECT sensor_sequence,trial_index,phase,ch4_hardware_raw,"
                    "ch4_baseline,ch4_signal,"
                    "die_temperature_celsius,die_temperature_age_seconds FROM samples"
                ).fetchone()
                metadata = dict(
                    connection.execute(
                        "SELECT key,value_json FROM session_metadata"
                    ).fetchall()
                )
                event = connection.execute(
                    "SELECT event,payload_json FROM events"
                ).fetchone()
            finally:
                connection.close()
            self.assertEqual(
                values,
                (123, 7, "creep_probe_hold", 45678, 3200, 10240, 25.3, 0.2),
            )
            self.assertEqual(json.loads(metadata["config"]), {"purpose": "unit_test"})
            self.assertEqual(event[0], "unit_test_event")
            self.assertEqual(json.loads(event[1]), {"trial": 7})


class TemperatureFrameTests(unittest.TestCase):
    def test_temperature_frame_reaches_latest_raw_sample(self):
        frame = struct.pack("<4sBBBBHh", b"DATA", 15, 0x03, 1, 0, 2, 253)
        parsed = AlgoFrameParser().feed(frame)
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0]["temperature_celsius"], 25.3)

        buffer = AlgoDataBuffer()
        buffer.append_frame(parsed[0])
        self.assertIsNone(buffer.latest_sensor_sample())
        debug = {
            "type": "debug",
            "device": "INDEX_FINGER",
            "channel_count": 8,
            "raw": (1,) * 8,
            "normalized_raw": (2,) * 8,
            "candidate_normalized_raw": (3,) * 8,
            "baseline": (4,) * 8,
            "signal": (5,) * 8,
            "fast_k_percent": (0.0,) * 8,
            "slow_k_percent": (0.0,) * 8,
            "noise": (1,) * 8,
            "state": (0,) * 8,
        }
        buffer.append_frame(debug)
        sample = buffer.latest_sensor_sample()
        self.assertEqual(sample["die_temperature_celsius"], 25.3)
        self.assertLess(sample["die_temperature_age_seconds"], 1.0)


class CreepSequenceTests(unittest.TestCase):
    def test_baseline_skips_high_pulse_and_response_runs_all_phases(self):
        class Harness:
            _run_creep_trial = ForcePressControlWidget._run_creep_trial
            def __init__(self):
                self.calls = []
                self._state_lock = threading.Lock()
                self._auto_round_index = 0
                self._auto_round_total = 0
                self._repeat_index = 0
                self._repeat_total = 0
                self._target_force_n = None

            def _creep_progress(self, phase, trial, trial_total, **values):
                self.calls.append(("progress", phase, trial["trial_index"]))

            def _run_creep_release(self, motor, tracker, stop_event, parameters):
                self.calls.append(("release",))
                return {"final_force_n": 0.0}

            @staticmethod
            def _creep_force_parameters(parameters, lookup, target_force_n):
                result = dict(parameters)
                result["target_force_n"] = target_force_n
                return result

            def _run_creep_press(self, motor, tracker, stop_event, parameters):
                target = parameters["target_force_n"]
                self.calls.append(("press", target))
                return {"final_force_n": target, "final_position_counts": -123}

            def _run_creep_wait(
                self,
                motor,
                tracker,
                stop_event,
                parameters,
                trial,
                trial_total,
                phase,
                duration_seconds,
            ):
                self.calls.append(("wait", phase, float(duration_seconds)))

        harness = Harness()
        trials = (
            {
                "category": "baseline_before",
                "repeat_index": 1,
                "high_force_n": 0.0,
                "high_hold_seconds": 0.0,
                "recovery_seconds": 10.0,
                "probe_force_n": 2.0,
                "probe_hold_seconds": 3.0,
                "high_lookup": None,
                "probe_lookup": {},
            },
            {
                "category": "history_response",
                "repeat_index": 1,
                "high_force_n": 20.0,
                "high_hold_seconds": 60.0,
                "recovery_seconds": 120.0,
                "probe_force_n": 6.0,
                "probe_hold_seconds": 3.0,
                "high_lookup": {},
                "probe_lookup": {},
            },
        )
        result = ForcePressControlWidget._run_creep_test(
            harness,
            motor=None,
            tracker={},
            stop_event=threading.Event(),
            parameters={
                "creep_trials": trials,
                "sweep_plan": {"repeat_count": 1, "inter_trial_seconds": 5.0},
                "wait_for_sensor_idle_after_release": False,
            },
        )
        presses = [call[1] for call in harness.calls if call[0] == "press"]
        self.assertEqual(presses, [2.0, 20.0, 6.0])
        self.assertEqual(sum(call[0] == "release" for call in harness.calls), 4)
        waits = [call[1:] for call in harness.calls if call[0] == "wait"]
        self.assertIn(("creep_high_hold", 60.0), waits)
        self.assertIn(("creep_recovery_wait", 120.0), waits)
        self.assertIn(("creep_inter_trial_wait", 5.0), waits)
        self.assertTrue(result["ok"])
        self.assertEqual(len(result["creep_completed_trials"]), 2)


if __name__ == "__main__":
    unittest.main()
