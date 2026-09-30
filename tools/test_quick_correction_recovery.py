"""验证快速K修正能自动清理异常遗留的MCU RAM临时表。"""

import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "PY_SRIPT" / "algo_monitor"))

from main_window import MainWindow


class QuickCorrectionRecoveryTest(unittest.TestCase):
    @staticmethod
    def make_window(discard_succeeds=True):
        calls = []
        state = {
            "target_key": (0, 0),
            "persist": False,
            "trial_kind": "quick_correction",
            "phase": "preflight_read_wait",
            "expected_key": (0, 0),
            "read_index": 0,
            "preflight": {},
            "stale_ram_cleanup_attempted": False,
        }
        window = SimpleNamespace(
            _force_calibration_transfer=state,
            _force_calibration_transfer_token=0,
            reader=SimpleNamespace(
                discard_force_calibration=lambda: discard_succeeds
            ),
            _force_calibration_snapshot=MainWindow._force_calibration_snapshot,
            _set_force_calibration_transfer_progress=lambda message: calls.append(
                ("progress", message)
            ),
            _arm_force_calibration_timeout=lambda phase: calls.append(
                ("timeout", phase)
            ),
            _finish_force_calibration_transfer=lambda success, message: calls.append(
                ("finished", success, message)
            ),
            _send_next_force_calibration_read=lambda stage: calls.append(
                ("read", stage)
            ),
        )
        return window, state, calls

    @staticmethod
    def dirty_frame():
        return {
            "region": 0,
            "force_type": 0,
            "valid": True,
            "dirty": True,
            "raw_points": tuple(range(20)),
            "calibrated_points": tuple(range(20)),
        }

    def test_dirty_ram_is_discarded_then_preflight_restarts(self):
        window, state, calls = self.make_window()
        MainWindow.on_force_calibration_parameters(window, self.dirty_frame())
        self.assertTrue(state["stale_ram_cleanup_attempted"])
        self.assertEqual(state["phase"], "cleanup_discard_ack_wait")
        self.assertFalse(any(call[0] == "finished" for call in calls))

        with patch.object(
            sys.modules["main_window"].QtCore.QTimer,
            "singleShot",
            side_effect=lambda _delay, callback: callback(),
        ):
            MainWindow.on_command_ack(window, 0)
        self.assertEqual(state["phase"], "preflight_next")
        self.assertEqual(state["read_index"], 0)
        self.assertEqual(state["preflight"], {})
        self.assertIn(("read", "preflight"), calls)

    def test_second_dirty_result_stops_instead_of_looping(self):
        window, state, calls = self.make_window()
        state["stale_ram_cleanup_attempted"] = True
        MainWindow.on_force_calibration_parameters(window, self.dirty_frame())
        finished = [call for call in calls if call[0] == "finished"]
        self.assertEqual(len(finished), 1)
        self.assertFalse(finished[0][1])
        self.assertIn("自动恢复后回读仍为dirty", finished[0][2])

    def test_cleanup_command_failure_reports_actionable_error(self):
        window, _state, calls = self.make_window(discard_succeeds=False)
        MainWindow.on_force_calibration_parameters(window, self.dirty_frame())
        finished = [call for call in calls if call[0] == "finished"]
        self.assertEqual(len(finished), 1)
        self.assertIn("自动恢复Flash原表命令发送失败", finished[0][2])


if __name__ == "__main__":
    unittest.main()
