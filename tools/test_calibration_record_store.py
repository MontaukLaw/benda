"""Windows目标占用时重试与独立备份；不能将未完整写出的文件视为保存成功。"""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import test_high_force_calibration as calibration
from calibration_record_store import CalibrationRecordStore
from force_signal_pressure_calibration import _write_json_atomic, ForceSignalPressureCalibrationError
from high_force_calibration import HighForceCalibrationRunner


class RecordStoreTests(unittest.TestCase):
    run_plan = calibration.HighCalibrationTests.run_plan

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name)/'high_force_test.json'

    def test_short_access_denial_retries_and_replaces_complete_file(self):
        self.path.write_text('{"old":true}', encoding='utf-8')
        original, calls = Path.replace, []
        def replace(source, target):
            calls.append(source)
            if len(calls) <= 2:
                raise PermissionError('Windows target busy')
            return original(source, target)
        with patch.object(Path, 'replace', replace), patch('force_signal_pressure_calibration.time.sleep'):
            _write_json_atomic(self.path, {'records': [1, 2, 3]})
        self.assertEqual(len(calls), 3)
        self.assertEqual(json.loads(self.path.read_text(encoding='utf-8'))['records'], [1, 2, 3])
        self.assertEqual(list(self.path.parent.glob('*.pending.json')), [])

    def test_persistent_target_lock_keeps_unique_complete_backups_and_then_recovers(self):
        self.path.write_text('{"old":true}', encoding='utf-8')
        store = CalibrationRecordStore(self.path)
        with patch.object(Path, 'replace', side_effect=PermissionError('target locked')), \
                patch('force_signal_pressure_calibration.time.sleep'):
            self.assertTrue(store.save({'records': list(range(25))}))
            first = store.saved_path
            self.assertTrue(store.save({'records': list(range(26))}))
            second = store.saved_path
        self.assertNotEqual(first, second)
        self.assertEqual(len(json.loads(first.read_text(encoding='utf-8'))['records']), 25)
        self.assertEqual(len(json.loads(second.read_text(encoding='utf-8'))['records']), 26)
        self.assertTrue(json.loads(self.path.read_text(encoding='utf-8'))['old'])
        self.assertEqual(store.save({'records': list(range(27))}), '')
        self.assertEqual(store.saved_path, self.path)
        self.assertEqual(len(json.loads(self.path.read_text(encoding='utf-8'))['records']), 27)

    def test_no_writable_file_does_not_claim_backup_success(self):
        store = CalibrationRecordStore(self.path)
        with patch('force_signal_pressure_calibration.tempfile.NamedTemporaryFile',
                   side_effect=PermissionError('directory denied')):
            with self.assertRaises(ForceSignalPressureCalibrationError) as caught:
                store.save({'records': [1]})
        self.assertIsNone(caught.exception.recovery_path)
        self.assertIsNone(store.saved_path)

    def test_flush_failure_rejects_incomplete_file(self):
        store = CalibrationRecordStore(self.path)
        with patch('force_signal_pressure_calibration.os.fsync', side_effect=OSError('disk full')):
            with self.assertRaises(ForceSignalPressureCalibrationError) as caught:
                store.save({'records': [1]})
        self.assertIsNone(caught.exception.recovery_path)
        self.assertEqual(list(self.path.parent.iterdir()), [])

    def test_k_table_writer_still_reports_failed_publication(self):
        with patch.object(Path, 'replace', side_effect=PermissionError('locked')), \
                patch('force_signal_pressure_calibration.time.sleep'):
            with self.assertRaises(ForceSignalPressureCalibrationError) as caught:
                _write_json_atomic(self.path, {'nodes': [1, 2]})
        self.assertFalse(self.path.exists())
        self.assertTrue(Path(caught.exception.recovery_path).exists())

    def test_target_lock_does_not_end_three_round_calibration_or_duplicate_points(self):
        store = CalibrationRecordStore(self.path)
        original = HighForceCalibrationRunner.__init__
        def init(runner, *args, **kwargs):
            original(runner, *args, **kwargs)
            checkpoint = runner.checkpoint
            def save(rows):
                checkpoint(rows)
                store.save(dict(records=rows, **runner.audit()))
            runner.checkpoint = save
        with patch.object(HighForceCalibrationRunner, '__init__', init), \
                patch.object(Path, 'replace', side_effect=PermissionError('target locked')), \
                patch('force_signal_pressure_calibration.time.sleep'):
            plant, result, _, _ = self.run_plan(rounds=3, staged=True, rest_seconds=0)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['records']), 30)
        self.assertEqual(len(result['validation_records']), 10)
        self.assertEqual(plant.position, 0)
        saved = json.loads(store.saved_path.read_text(encoding='utf-8'))
        self.assertEqual(len(saved['records']), 30)
        self.assertEqual(len(saved['validation_records']), 10)


class RecordStoreUiTests(unittest.TestCase):
    page = calibration.HighCalibrationUiTests.page
    setUp = calibration.HighCalibrationUiTests.setUp

    @classmethod
    def setUpClass(cls):
        from qt_compat import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_full_storage_failure_waits_released_then_continues_after_write_recovers(self):
        from high_force_calibration_widget import HighForceCalibrationWidget
        save, progress, recover = (CalibrationRecordStore.save,
            HighForceCalibrationWidget._progress, HighForceCalibrationRunner._recover)
        state = dict(failed=False, recovered=False, released=False)
        def blocked(store, document):
            if len(document.get('records', [])) == 1 and not state['recovered']:
                state['failed'] = True
                raise ForceSignalPressureCalibrationError('模拟目录不可写')
            return save(store, document)
        def recovered(runner, reason):
            outcome = recover(runner, reason)
            if state['failed']:
                self.assertEqual(runner.motor.position, 0)
                state['released'] = True
            return outcome
        def storage_progress(widget, row):
            if row.get('phase') == 'storage_wait':
                self.assertTrue(state['released'])
                self.assertIsNotNone(widget._pending_record_document)
                state['recovered'] = True
            return progress(widget, row)
        with patch.object(CalibrationRecordStore, 'save', blocked), \
                patch.object(HighForceCalibrationRunner, '_recover', recovered), \
                patch.object(HighForceCalibrationWidget, '_progress', storage_progress):
            calibration.HighCalibrationUiTests.test_start_snapshots_settings_runs_in_shared_worker_and_publishes_k_table(self)
        self.assertTrue(all(state.values()))


if __name__ == '__main__':
    unittest.main()
