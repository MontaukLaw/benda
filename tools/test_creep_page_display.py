"""离屏检查长时测试页面的说明、倒计时和大字体换行，不连接设备。"""
import os
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "PY_SRIPT" / "algo_monitor"))
from qt_compat import QtWidgets, QtGui
from long_duration_creep_test_widget import LongDurationCreepTestWidget, build_creep_plan


class CreepDisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        if os.name == "nt":
            for name in ("segoeui.ttf", "msyh.ttc"):
                QtGui.QFontDatabase.addApplicationFont("C:/Windows/Fonts/" + name)

    def setUp(self):
        with patch.object(LongDurationCreepTestWidget, "_load_settings"):
            self.widget = LongDurationCreepTestWidget(
                SimpleNamespace(operation_progress=Mock(), operation_finished=Mock()),
                SimpleNamespace(), lambda: {},
            )
        self.widget.setStyleSheet(
            "QWidget{background:#080d12;color:#e2e8f0;font-family:'Microsoft YaHei';font-size:14px;}"
            "QPushButton{background:#dce8f5;color:#142337;border-radius:5px;padding:6px;}"
        )
        self.widget._config = self.widget._read_config()
        self.widget._plan = build_creep_plan(self.widget._config)
        self.widget._manifest = {"completed_trials": 18}
        self.widget._runtime_started_at = time.monotonic() - 3720
        self.widget._running = True
        self.context = dict(phase="creep_recovery_wait", trial_index=19, trial_total=72,
                            planned_trial_index=19, repeat_index=1, category="history_response",
                            high_force_n=20, high_hold_seconds=60, probe_force_n=6,
                            probe_hold_seconds=3, recovery_seconds=360,
                            phase_duration_seconds=360, phase_elapsed_seconds=100)

    def tearDown(self):
        self.widget._running = False
        self.widget.close()

    def test_clear_countdown_and_no_false_stage_completion(self):
        with patch("long_duration_creep_test_widget.time.monotonic", return_value=1000) as clock:
            self.widget._on_force_progress(self.context)
            self.assertIn("第4/7步", self.widget.stage_label.text())
            self.assertIn("4分20秒", self.widget.countdown_label.text())
            self.assertIn("第 1/2 轮", self.widget.round_label.text())
            self.assertNotIn("creep_", self.widget.next_step_label.text())
            clock.return_value = 1004
            self.widget._render_runtime()
            self.assertIn("更新延迟", self.widget.countdown_label.text())
            clock.return_value = 1005
            self.widget._on_force_progress(dict(self.context, phase_elapsed_seconds=360))
            self.assertIn("等待控制器", self.widget.countdown_label.text())
            self.assertIn("空载恢复", self.widget.stage_label.text())

    def test_description_tracks_custom_plan_and_stop_clears_wait(self):
        self.assertIn("共 18 次按压", self.widget.experiment_description.text())
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('screening'))
        self.assertIn("每轮 36 项", self.widget.experiment_description.text())
        self.widget.repeat_input.setValue(1)
        self.widget.high_force_edit.setText("20")
        self.assertIn("共 1 轮，每轮 20 项，共 20 项", self.widget.experiment_description.text())
        self.widget._on_force_progress(self.context)
        self.widget._stopping = True
        self.widget._render_runtime()
        self.assertIn("正在停止", self.widget.stage_label.text())
        self.assertIn("确认卸载", self.widget.countdown_label.text())

    def test_large_text_fits_and_runtime_stays_above_scroll(self):
        self.widget._on_force_progress(self.context)
        self.widget.progress_bar.setValue(250)
        self.widget.start_button.setEnabled(False)
        self.widget.stop_button.setEnabled(True)
        for width in (800, 1050):
            self.widget.resize(width, 800)
            self.widget.show()
            for _ in range(6):
                self.app.processEvents()
            for label in (self.widget.round_label, self.widget.stage_label,
                          self.widget.countdown_label, self.widget.condition_label,
                          self.widget.next_step_label, self.widget.experiment_description):
                self.assertGreaterEqual(label.height(), label.heightForWidth(label.width()))
            self.assertLess(self.widget.runtime_panel.height(), 420)
            self.assertEqual(self.widget.layout().itemAt(0).widget(), self.widget.runtime_panel)
        if os.environ.get("CREEP_PAGE_SCREENSHOT"):
            path = Path(os.environ["CREEP_PAGE_SCREENSHOT"])
            path.parent.mkdir(parents=True, exist_ok=True)
            self.widget.grab().save(str(path))


if __name__ == "__main__":
    unittest.main()
