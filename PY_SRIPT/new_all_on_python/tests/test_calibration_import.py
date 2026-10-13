"""验证跨应用标定读取、原始值保护、算法输入与保存失败事务。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from PySide6 import QtWidgets
from new_all_on_python.calibration_import import read_calibration, latest_calibration_path, _calibration
from new_all_on_python.config import AppConfig
from new_all_on_python.main_window import MainWindow

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def fixture_report(target=0):
    settings = _calibration.CalibrationSettings(mass_g=50, repeats=3, target=target)
    model = _calibration.RawGainCalibration(settings)
    for ch in range(20):
        for repeat in range(3):
            baseline = [10000.0]*20; loaded = baseline.copy(); loaded[ch] += 1000+ch*100
            model.trials[ch].append(dict(channel=ch, response=1000+ch*100, capture_settings=asdict(settings),
                baseline=baseline, loaded_raw=loaded,
                unloaded=dict(raw=baseline.copy(), noise=[1.0]*20, ranges=[4]*20, timestamp=10+ch, sample_count=64),
                loaded_timestamp=12+ch, loaded_elapsed_seconds=1.6, loaded_sample_count=64,
                loaded_ranges=[4]*20, dominant_channel=ch))
            model.revision += 1
    report = model.report()
    report.update(context=None, invalid_reason='传感器连接已改变', saved_at='2026-10-11T10:00:00+08:00')
    # 缓存结果故意错误，导入只能依据原始采集重新计算。
    report['results'] = dict(ready=True, gains=[99]*20)
    return report


class CalibrationImportTests(unittest.TestCase):
    def test_recompute_full_precision_and_preserve_original(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'raw_gain_fixture.json'
            path.write_text(json.dumps(fixture_report()), encoding='utf-8')
            original = path.read_bytes(); source = read_calibration(path)
            self.assertEqual(source['gains'][0], 1.95)
            self.assertEqual(source['gains'][19], .6724)
            self.assertEqual(source['mass_g'], 50)
            self.assertEqual(source['sha256'], hashlib.sha256(original).hexdigest())
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(latest_calibration_path(folder), path)

    def test_partial_corrupt_and_out_of_range_not_silently_imported(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'raw_gain_fixture.json'
            for kind in ('partial', 'corrupt', 'nonfinite', 'range'):
                report = fixture_report(1000000 if kind=='range' else 0)
                if kind=='partial':report['trials'][19].pop()
                elif kind=='corrupt':report['trials'][0][0]['response'] += 100
                elif kind=='nonfinite':report['trials'][0][0]['loaded_raw'][0] = float('nan')
                path.write_text(json.dumps(report), encoding='utf-8')
                with self.subTest(kind=kind), self.assertRaises(ValueError): read_calibration(path)

    def test_gui_apply_algorithm_recording_and_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder); path = folder/'config.json'; record = folder/'raw_gain_fixture.json'
            record.write_text(json.dumps(fixture_report()), encoding='utf-8')
            original = record.read_bytes()
            AppConfig(raw_gains=[2.0]*20, baseline_update_rate=0).save(path)
            with patch('new_all_on_python.main_window.CONFIG_PATH', path):
                window = MainWindow(auto_connect=False); window.timer.stop()
                try:
                    for n in range(70):window.engine.update([10000]*20, n/100)
                    previous = window.engine.result
                    window.engine.recorder.start(window.config, window.engine.revision, folder/'recording')
                    self.assertTrue(window.load_calibration(record))
                    self.assertEqual(window.engine.tracker.config.raw_gains, [2.0]*20)
                    self.assertEqual(window.gain_controls[0].value(), 1.95)
                    self.assertFalse(window.apply_raw_gains())
                    self.assertIs(window.engine.result, previous)
                    window.calibration_confirm.setChecked(True)
                    with patch.object(AppConfig, 'save', side_effect=OSError('disk unavailable')):
                        self.assertFalse(window.apply_raw_gains())
                    self.assertIs(window.engine.result, previous)
                    self.assertIsNotNone(window._calibration_draft)
                    self.assertTrue(window.apply_raw_gains())
                    source = AppConfig.load(path).calibration_source
                    self.assertEqual(source['gains'][0], 1.95)
                    self.assertEqual(source['sha256'], hashlib.sha256(original).hexdigest())
                    self.assertIsNone(window.engine.result)
                    self.assertTrue(window.engine.tracker.processor.calibrating)
                    self.assertIsNone(window.engine.tracker.processor.baseline)
                    for n in range(71,142):window.engine.update([10000]*20, n/100)
                    self.assertTrue(window.engine.result.baseline_ready)
                    self.assertEqual(window.engine.result.baseline[0], 19500)
                    raw = [10000]*20; raw[0] += 1000; raw[4] += 1000
                    frame = dict(protocol='DATA', protocol_version=21, device=1, state=[2]*20)
                    result = window.engine.update(raw, 1.42, frame)
                    self.assertEqual(result.raw, tuple(raw))
                    self.assertEqual(result.signal[0], 1950)
                    self.assertAlmostEqual(result.signal[4], 1392.9)
                    expected_x = 1392.9**2/(1950**2+1392.9**2)
                    self.assertAlmostEqual(result.x_raw, expected_x)
                    self.assertNotAlmostEqual(result.x_raw, .5)
                    self.assertEqual(frame['state'], [2]*20)
                    self.assertTrue(window.engine.recorder.finish())
                    events = [json.loads(line) for line in window.engine.recorder.path.with_suffix('.events.jsonl').read_text(encoding='utf-8').splitlines()]
                    changed = next(e for e in events if e['kind']=='config_changed')
                    self.assertEqual(changed['config']['calibration_source']['sha256'], source['sha256'])
                    with window.engine.recorder.path.open(encoding='utf-8-sig', newline='') as f: rows = list(csv.DictReader(f))
                    self.assertEqual(rows[-1]['raw0'], '11000')
                    self.assertEqual(float(rows[-1]['raw_gain0']), 1.95)
                    self.assertEqual(rows[-1]['firmware_state0'], '2')
                    self.assertEqual(record.read_bytes(), original)
                    record.unlink()  # 临时夹具移除后，启动仍使用已保存增益，不重新读取外部文件。
                    restarted = MainWindow(auto_connect=False); restarted.timer.stop()
                    try:
                        self.assertEqual(restarted.config.raw_gains, source['gains'])
                        self.assertEqual(restarted.config.calibration_source['sha256'], source['sha256'])
                        self.assertEqual(restarted.gain_controls[0].value(), 1.95)
                    finally: restarted.close()
                    window.gain_controls[0].setValue(2.1)
                    self.assertTrue(window.apply_raw_gains())
                    self.assertEqual(AppConfig.load(path).calibration_source, {})
                finally:
                    window.engine.recorder.finish(); window.close()

    def test_gui_bad_record_preserves_existing_draft_and_config(self):
        with tempfile.TemporaryDirectory() as folder, patch('new_all_on_python.main_window.CONFIG_PATH', Path(folder)/'config.json'):
            path = Path(folder)/'raw_gain_fixture.json'; path.write_text(json.dumps(fixture_report()), encoding='utf-8')
            window = MainWindow(auto_connect=False); window.timer.stop()
            try:
                self.assertTrue(window.load_calibration(path)); draft = window._calibration_draft
                path.write_text('{', encoding='utf-8')
                self.assertFalse(window.load_calibration(path))
                self.assertIs(window._calibration_draft, draft)
                self.assertEqual(window.gain_controls[0].value(), 1.95)
                self.assertEqual(window.config.raw_gains, [1.0]*20)
            finally: window.close()


if __name__ == '__main__': unittest.main()
