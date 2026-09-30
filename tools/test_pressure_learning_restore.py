"""验证跨进程学习存档、旧记录迁移、释放点核验和清除后不复活。"""
from dataclasses import asdict, replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import test_pressure_feedback as feedback_tests
from pressure_feedback_control import LearnedPressureMap
from pressure_learning_store import load_learning, save_learning
from qt_compat import QtWidgets


class LearningRestoreTests(unittest.TestCase):
    page = feedback_tests.FeedbackSettingsUiTests.page

    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def table(self, page):
        return LearnedPressureMap(page.profile.model, page.profile.namespace, page.profile.steps_per_turn,
                                  -217, ((.04, 304), (.11, 330), (.19, 351), (.52, 409),
                                         (.98, 460), (1.96, 538), (5.06, 695), (9.83, 876),
                                         (14.79, 1034), (20.05, 1173), (24.71, 1297), (29.51, 1407)),
                                  266, 270, .01, 0.0)

    def saved(self):
        page = self.page()
        page._learned_map = self.table(page)
        page._save_learning()
        self.assertFalse(page._learning_storage_error)
        return page

    def test_restart_restores_only_after_three_fresh_samples_and_keeps_original_reference(self):
        first = self.saved()
        restored = self.page()
        self.assertEqual(restored._learned_map, first._learned_map)
        self.assertTrue(restored.learning_recovery_pending())
        self.assertIsNone(restored.learned_map_snapshot())
        self.assertIn('等待', restored.learning_notice_text())
        now = [restored._learning_suspended_at+1]
        feedback = dict(position_counts=-216, measured_speed_rpm=0, phase_current_ma=100,
                        running_state=0, in_position=True)
        with patch('time.monotonic', lambda: now[0]), \
                patch.object(restored.control, 'is_can_connected', return_value=True), \
                patch.object(restored.control, 'is_operation_busy', return_value=False), \
                patch.object(restored.control, 'latest_motor_feedback', side_effect=lambda: dict(feedback)), \
                patch.object(restored.control.backend, '_motion_block_reason', return_value=''), \
                patch.object(restored.control.backend, '_submit') as submit, \
                patch.object(restored.control, 'start_external_operation') as motion:
            for index in range(3):
                now[0] += .11
                feedback['sampled_monotonic'] = now[0]
                restored.samples.update(dict(accepted=True, force_unit='N', force_n=0,
                                             sequence=index+1, sampled_monotonic=now[0]))
                restored._refresh()
                if index < 2:
                    restored._refresh()
                    self.assertTrue(restored.learning_recovery_pending())
                    self.assertFalse(restored.control.release_position_snapshot()['confirmed'])
            self.assertEqual(restored.control.release_position_snapshot(), dict(position_counts=-217, confirmed=True))
            self.assertEqual(restored.learned_map_snapshot(), first._learned_map)
            self.assertTrue(restored.start_button.isEnabled())
            self.assertEqual(restored.learning_notice_text(), '')
            submit.assert_not_called()
            motion.assert_not_called()

    def test_migration_reads_latest_complete_archive_but_clear_blocks_future_migration(self):
        page = self.page()
        page.output_dir.mkdir()
        table = self.table(page)
        for name, release in (('20260919_120000', -100), ('20260920_120000', -217)):
            document = dict(schema_version=3, **asdict(replace(table, release_position=release)))
            (page.output_dir/f'feedback_{name}.learning.json').write_text(json.dumps(document), encoding='utf-8')
        restored = self.page()
        self.assertEqual(restored._learned_map, table)
        self.assertTrue(restored._learning_path.exists())
        restored._invalidate_learning('用户清除')
        reopened = self.page()
        self.assertIsNone(reopened._learned_map)
        self.assertFalse(reopened.learning_recovery_pending())
        self.assertIn('用户清除', reopened.learning_status_label.text())
        self.assertEqual(len(list(page.output_dir.glob('*.learning.json'))), 2)

    def test_corrupt_newest_archive_never_falls_back_to_older_curve(self):
        page = self.page()
        page.output_dir.mkdir()
        document = dict(schema_version=3, **asdict(self.table(page)))
        (page.output_dir/'feedback_20260919.learning.json').write_text(json.dumps(document), encoding='utf-8')
        document['points'] = document['points'][:4]
        (page.output_dir/'feedback_20260920.learning.json').write_text(json.dumps(document), encoding='utf-8')
        restored = self.page()
        self.assertIsNone(restored._learned_map)
        self.assertIn('读取失败', restored.learning_notice_text())

    def test_bad_saved_documents_are_not_loaded_or_overwritten(self):
        first = self.saved()
        original = json.loads(first._learning_path.read_text(encoding='utf-8'))
        for fault in ('nan', 'nonmonotonic', 'contact', 'namespace', 'version', 'boolean', 'truncated'):
            with self.subTest(fault=fault):
                document = json.loads(json.dumps(original))
                if fault == 'nan': document['table']['points'][0][0] = float('nan')
                if fault == 'nonmonotonic': document['table']['points'][1][1] = 1
                if fault == 'contact': document['table']['contact_travel'] = -1
                if fault == 'namespace': document['profile']['namespace'] = 'another_motor'
                if fault == 'version': document['schema_version'] = 999
                if fault == 'boolean': document['table']['release_position'] = True
                content = '{' if fault == 'truncated' else json.dumps(document)
                first._learning_path.write_text(content, encoding='utf-8')
                restored = self.page()
                self.assertIsNone(restored._learned_map)
                self.assertIn('读取失败', restored.learning_notice_text())
                self.assertEqual(first._learning_path.read_text(encoding='utf-8'), content)

    def test_disconnect_retains_table_but_coordinate_reset_and_changed_reference_persist_invalidation(self):
        for event in ('reference', 'coordinates'):
            with self.subTest(event=event):
                page = self.saved()
                page.control._on_link(False)
                self.assertIsNotNone(page._learned_map)
                self.assertIsNotNone(self.page()._learned_map)
                if event == 'coordinates':
                    page.control._invalidate_coordinates()
                else:
                    page.control._release = 0
                    page._release_reference_changed()
                self.assertIsNone(page._learned_map)
                self.assertIsNone(self.page()._learned_map)

    def test_wrong_position_loaded_stale_fault_and_changed_reference_never_restore(self):
        self.saved()
        for fault in ('position', 'loaded', 'stale', 'moving', 'not_arrived', 'driver', 'reference', 'unstable'):
            with self.subTest(fault=fault):
                page = self.page()
                now = [page._learning_suspended_at+1]
                feedback = dict(position_counts=0 if fault == 'position' else -217,
                                measured_speed_rpm=10 if fault == 'moving' else 0,
                                phase_current_ma=100, running_state=0, in_position=fault != 'not_arrived')
                if fault == 'reference': page.control._release = 0
                with patch('time.monotonic', lambda: now[0]), \
                        patch.object(page.control, 'is_can_connected', return_value=True), \
                        patch.object(page.control, 'is_operation_busy', return_value=False), \
                        patch.object(page.control, 'latest_motor_feedback', side_effect=lambda: dict(feedback)), \
                        patch.object(page.control.backend, '_motion_block_reason', return_value='电机故障' if fault == 'driver' else ''):
                    for index in range(4):
                        now[0] += .11
                        feedback['sampled_monotonic'] = now[0]-(1 if fault == 'stale' else 0)
                        force = 1 if fault == 'loaded' else ((-.01 if index%2 else .01) if fault == 'unstable' else 0)
                        page.samples.update(dict(accepted=True, force_unit='N', force_n=force,
                                                 sequence=index+1, sampled_monotonic=now[0]))
                        page._refresh()
                    self.assertTrue(page.learning_recovery_pending())
                    self.assertIsNone(page.learned_map_snapshot())
                    self.assertEqual(page.control._release, 0 if fault == 'reference' else None)

    def test_atomic_write_failure_reports_error_and_preserves_previous_file(self):
        page = self.saved()
        before = page._learning_path.read_bytes()
        with patch('pressure_learning_store.os.replace', side_effect=OSError('disk unavailable')):
            page._save_learning()
        self.assertIn('保存失败', page.learning_notice_text())
        self.assertEqual(page._learning_path.read_bytes(), before)
        self.assertFalse(list(self.root.glob('*.tmp')))


if __name__ == '__main__':
    unittest.main(verbosity=2)
