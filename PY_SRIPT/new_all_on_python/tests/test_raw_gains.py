"""验证增益精度、配置兼容、基线尺度切换和界面应用/保存行为。"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from PySide6 import QtWidgets
from new_all_on_python.config import AppConfig
from new_all_on_python.touch_tracker import TouchTracker
from new_all_on_python.processing_engine import ProcessingEngine
from new_all_on_python.main_window import MainWindow

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class RawGainTests(unittest.TestCase):
    def test_config_defaults_legacy_roundtrip_and_invalid_gains(self):
        first = AppConfig()
        copied = first.copy()
        copied.raw_gains[0] = 1.5
        self.assertEqual(first.raw_gains, [1.0]*20)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"config.json"
            path.write_text('{"position_alpha":0.2}', encoding="utf-8")
            self.assertEqual(AppConfig.load(path).raw_gains, [1.0]*20)
            copied.save(path)
            self.assertEqual(AppConfig.load(path), copied)
        for gains in ([1]*19, [1]*21, "1", [True]+[1]*19,
                      [float('nan')]+[1]*19, [float('inf')]+[1]*19,
                      [-1]+[1]*19, [101]+[1]*19):
            with self.subTest(gains=gains), self.assertRaises(ValueError):
                AppConfig(raw_gains=gains).validate()

    def test_gain_keeps_fractional_values_and_exceeds_wire_range(self):
        gains = [1.0]*20
        gains[0], gains[1], gains[2] = 1.234567, 100.0, 0.0
        tracker = TouchTracker(AppConfig(raw_gains=gains, baseline_update_rate=0))
        raw = [12345]*20
        for n in range(70):
            result = tracker.update(raw,n/100)
        self.assertTrue(result.baseline_ready)
        self.assertEqual(result.raw, tuple(raw))
        self.assertAlmostEqual(result.adjusted_raw[0],12345*1.234567)
        self.assertEqual(result.baseline[1],1234500.0)
        self.assertEqual(result.baseline[2],0.0)
        raw[0], raw[1], raw[2] = 13346, 13346, 65535
        result = tracker.update(raw,.7)
        self.assertAlmostEqual(result.signal[0],1001*1.234567)
        self.assertEqual(result.signal[1],100100.0)
        self.assertEqual(result.signal[2],0.0)
        with self.assertRaises(ValueError):
            tracker.update([65536]*20,.71)

    def test_gain_change_rebuilds_baseline_and_clears_old_touch(self):
        engine = ProcessingEngine(AppConfig())
        for n in range(70):
            engine.update([12000]*20,n/100)
        engine.update([17000]+[12000]*19,.7)
        self.assertTrue(engine.result.touched)
        config = engine.tracker.config.copy()
        config.raw_gains[0] = 1.5
        engine.set_config(config)
        self.assertIsNone(engine.result)
        self.assertFalse(engine.trail)
        self.assertTrue(engine.tracker.processor.calibrating)
        self.assertIsNone(engine.tracker.processor.baseline)
        for n in range(71,141):
            result = engine.update([12000]*20,n/100)
            self.assertFalse(result.touched)
        self.assertTrue(result.baseline_ready)
        self.assertEqual(result.baseline[0],18000.0)
        result=engine.update([13000]+[12000]*19,1.41)
        self.assertEqual(result.state,"TOUCH_DOWN")
        self.assertEqual((result.dx,result.dy),(0,0))

    def test_gui_drafts_apply_save_restore_and_restart(self):
        with tempfile.TemporaryDirectory() as folder, patch("new_all_on_python.main_window.CONFIG_PATH",Path(folder)/"config.json"):
            path=Path(folder)/"config.json"
            window=MainWindow(auto_connect=False)
            try:
                self.assertEqual(len(window.gain_controls),20)
                self.assertTrue(all(not c.prefix() and not c.suffix() for c in window.gain_controls))
                window.gain_controls[8].setValue(1.25)
                # 修改其他即时参数不能丢弃未应用的增益草稿。
                window._parameter_changed("position_alpha",.2)
                self.assertEqual(window.gain_controls[8].value(),1.25)
                self.assertEqual(window.engine.tracker.config.raw_gains[8],1.0)
                self.assertTrue(window.apply_raw_gains())
                self.assertEqual(AppConfig.load(path).raw_gains[8],1.25)
                self.assertEqual(window.engine.tracker.config.raw_gains[8],1.25)
                persisted=MainWindow(auto_connect=False)
                try:
                    self.assertEqual(persisted.gain_controls[8].value(),1.25)
                finally:
                    persisted.close()
                for n in range(70):
                    window.engine.update([12000]*20,n/100)
                self.assertTrue(window.engine.result.baseline_ready)
                self.assertTrue(window.apply_raw_gains())
                self.assertIsNone(window.engine.result)
                self.assertTrue(window.engine.tracker.processor.calibrating)
                window.reset_gain_inputs()
                self.assertEqual([c.value() for c in window.gain_controls],[1.0]*20)
                self.assertEqual(window.engine.tracker.config.raw_gains[8],1.25)
                window.apply_raw_gains()
                self.assertEqual(AppConfig.load(path).raw_gains,[1.0]*20)
            finally:
                window.close()
            restarted=MainWindow(auto_connect=False)
            try:
                self.assertEqual([c.value() for c in restarted.gain_controls],[1.0]*20)
            finally:
                restarted.close()

    def test_save_failure_keeps_running_config_and_baseline(self):
        # 此测试使用默认增益，不读取用户正在使用的标定配置。
        with tempfile.TemporaryDirectory() as folder, patch("new_all_on_python.main_window.CONFIG_PATH",Path(folder)/"missing.json"):
            window=MainWindow(auto_connect=False)
        try:
            for n in range(70):
                window.engine.update([12000]*20,n/100)
            previous=window.engine.result
            window.gain_controls[0].setValue(2.0)
            with patch.object(AppConfig,"save",side_effect=OSError("disk unavailable")):
                self.assertFalse(window.apply_raw_gains())
            self.assertIs(window.engine.result,previous)
            self.assertEqual(window.engine.tracker.config.raw_gains[0],1.0)
            self.assertFalse(window.engine.tracker.processor.calibrating)
        finally:
            window.close()

    def test_recording_preserves_device_raw_gain_and_scale_change(self):
        engine=ProcessingEngine(AppConfig())
        with tempfile.TemporaryDirectory() as folder:
            engine.recorder.start(engine.tracker.config,engine.revision,folder)
            try:
                engine.update([12345]*20,0)
                config=engine.tracker.config.copy()
                config.raw_gains[0]=1.234567
                engine.set_config(config)
                for n in range(1,72):
                    engine.update([12345]*20,n/100)
            finally:
                self.assertTrue(engine.recorder.finish())
            with engine.recorder.path.open(encoding="utf-8-sig", newline="") as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(rows[-1]['raw0'],'12345')
            self.assertEqual(float(rows[-1]['adjusted_raw0']),12345*1.234567)
            self.assertEqual(float(rows[-1]['raw_gain0']),1.234567)
            self.assertEqual(float(rows[-1]['baseline0']),12345*1.234567)
            events=[json.loads(x) for x in engine.recorder.path.with_suffix('.events.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual([x['kind'] for x in events if x['kind'] in ['config_changed','reset_baseline']],['config_changed','reset_baseline'])
            self.assertEqual(rows[0]['config_revision'],'0')
            self.assertEqual(rows[1]['config_revision'],'1')


if __name__ == '__main__':
    unittest.main()
