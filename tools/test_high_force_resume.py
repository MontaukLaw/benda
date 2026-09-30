"""断电/重启续跑使用实际闭环与模拟电机；不连接硬件。"""
from copy import deepcopy
from dataclasses import asdict, replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import test_high_force_calibration as native
from high_force_calibration_resume import (validate_resume, discover_resume, read_resume,
    learning_fingerprint, check_resume_context)
from high_force_calibration import HighForceCalibrationRunner
from high_force_calibration_widget import HighForceCalibrationWidget
from pressure_feedback_control import FeedbackSettings, LearnedPressureMap


PROFILE = native.feedback_tests.PROFILE


def saved_document(result, rounds=3, generation=0):
    result = deepcopy(result)
    result['complete'] = result.get('ok', False)
    result['metadata'] = dict(point_count=10, rounds=rounds, release_position=0,
        profile=PROFILE.metadata(), namespace=PROFILE.namespace,
        motion_settings=asdict(replace(FeedbackSettings(), max_step=256, force_limit_n=35,
                                      target_n=30, tolerance_n=.6)), tolerance_percent=2,
        rest_seconds=0, accuracy_percent=2, historical_targets=[], resume_generation=generation)
    return result


class ResumeTests(unittest.TestCase):
    run_plan = native.HighCalibrationTests.run_plan

    def interrupted(self, **kwargs):
        _, result, _, _ = self.run_plan(rounds=3, staged=True, rest_seconds=0, cancel_after=25, **kwargs)
        self.assertFalse(result['ok'])
        return saved_document(result)

    def test_formal_resume_preserves_25_points_and_partial_round_time(self):
        saved = self.interrupted()
        original = deepcopy(saved)
        self.assertIn('第3轮 17 N', validate_resume(saved))
        plant, result, phases, _ = self.run_plan(rounds=3, staged=True, rest_seconds=0, resume_document=saved,
                                               setup=lambda plant: setattr(plant, 'now', 100000))
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(saved, original)
        self.assertEqual(result['records'][:25], saved['records'])
        self.assertEqual(result['round_timings'][:2], saved['round_timings'][:2])
        self.assertEqual(len(result['records']), 30)
        self.assertEqual(len(result['warmup_records']), 5)
        formal = [r for r in phases if r.get('phase') == 'point' and r['stage'] == 'formal']
        self.assertEqual([r['target_n'] for r in formal], [17, 20, 24, 27, 30])
        self.assertTrue(all(r['round'] == 3 for r in formal))
        self.assertEqual(len(result['validation_records']), 10)
        self.assertAlmostEqual(result['elapsed_s']-saved['elapsed_s'], plant.now-100000)
        self.assertGreater(result['round_timings'][2]['elapsed_s'], saved['round_timings'][2]['elapsed_s'])
        self.assertEqual(plant.position, 0)
        self.assertEqual([t['stage'] for t in result['stage_timings']], ['warmup', 'formal', 'retest'])

    def test_twenty_point_plan_resumes_across_round_boundary(self):
        _, partial, _, _ = self.run_plan(count=20, rounds=2, staged=True, rest_seconds=0, cancel_after=20)
        saved = saved_document(partial, rounds=2)
        saved['metadata']['point_count'] = 20
        _, result, phases, _ = self.run_plan(count=20, rounds=2, staged=True, rest_seconds=0, resume_document=saved)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(result['records'][:20], saved['records'])
        self.assertEqual(len(result['records']), 40)
        formal = [r for r in phases if r.get('phase') == 'point' and r['stage'] == 'formal']
        self.assertEqual(len(formal), 20)
        self.assertTrue(all(r['round'] == 2 for r in formal))
        self.assertEqual(len(result['document']['nodes']), 20)

    def test_resume_again_after_stopping_during_seek_capture_or_release_returns_to_release(self):
        saved = self.interrupted()
        for phase in ('press', 'capture', 'release', 'rest'):
            plant, result, _, _ = self.run_plan(rounds=3, staged=True, rest_seconds=1,
                resume_document=saved, cancel_phase=phase)
            self.assertFalse(result['ok'])
            self.assertTrue(result['release_position_confirmed'])
            self.assertEqual(plant.position, 0)
            self.assertEqual(result['records'], saved['records'])
            second = saved_document(result, generation=1)
            _, resumed, _, _ = self.run_plan(rounds=3, staged=True, rest_seconds=0, resume_document=second)
            self.assertTrue(resumed['ok'], resumed['message'])
            self.assertEqual(len(resumed['records']), 30)

    def test_partial_warmup_continues_only_missing_repetitions(self):
        def stop(plant, row):
            if row.get('phase') == 'point' and row['stage'] == 'warmup' and row['point'] == 4:
                plant.cancelled = True
        _, partial, _, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0, progress_hook=stop)
        saved = saved_document(partial, rounds=1)
        self.assertEqual(len(saved['warmup_records']), 3)
        _, result, phases, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0, resume_document=saved)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual([r['point'] for r in phases if r.get('phase') == 'point' and r['stage'] == 'warmup'], [4, 5])
        self.assertEqual(result['warmup_records'][:3], saved['warmup_records'])

    def test_partial_retest_only_collects_missing_repeats_then_fresh_validation(self):
        def stop(plant, row):
            if row.get('phase') == 'point' and row['stage'] == 'retest' and row['round'] == 2:
                plant.cancelled = True
        _, partial, _, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0,
            historical_targets=[30], progress_hook=stop)
        saved = saved_document(partial, rounds=1)
        saved['metadata']['historical_targets'] = [30]
        self.assertEqual(len(saved['retest_records']), 1)
        _, result, phases, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0,
            historical_targets=[30], resume_document=saved)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['retest_records']), 3)
        self.assertEqual(result['retest_records'][0], saved['retest_records'][0])
        self.assertEqual([r['round'] for r in phases if r.get('phase') == 'point' and r['stage'] == 'retest'], [2, 3])
        self.assertFalse(any(r.get('phase') == 'point' and r['stage'] == 'formal' for r in phases))

    def test_interrupted_validation_is_archived_and_new_batch_is_not_mixed(self):
        def stop(plant, row):
            if row.get('phase') == 'point' and row['stage'] == 'validation' and row['point'] == 6:
                plant.cancelled = True
        _, partial, _, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0, progress_hook=stop)
        saved = saved_document(partial, rounds=1)
        self.assertEqual(len(saved['validation_records']), 5)
        _, result, phases, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0, resume_document=saved)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(result['validation_records'][:5], saved['validation_records'])
        self.assertEqual(len(result['validation_records']), 15)
        self.assertEqual({r['validation_session'] for r in result['validation_records'][5:]}, {1})
        self.assertEqual(len(result['quality']['passes'][-1]['validation']['checks']), 10)
        self.assertEqual([r['target_n'] for r in phases if r.get('phase') == 'point'], list(native.calibration_targets(10)))

    def test_later_pass_resumes_without_repeating_failed_validation_or_finished_retests(self):
        changed = []
        def interrupt(plant, row):
            if row.get('phase') != 'point': return
            if row['stage'] == 'validation': changed.append(True)
            if changed: plant.signal_gain = 1.05
            if row['stage'] == 'retest' and row.get('pass_index') == 2 and row['round'] == 2:
                plant.cancelled = True
        _, partial, _, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0, progress_hook=interrupt)
        saved = saved_document(partial, rounds=1)
        self.assertEqual(len(saved['validation_records']), 10)
        self.assertEqual(len(saved['retest_records']), 1)
        self.assertFalse(saved['quality']['passes'][0]['validation']['passed'])
        _, result, _, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0, resume_document=saved,
                                       setup=lambda plant: setattr(plant, 'signal_gain', 1.05))
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['quality']['passes']), 2)
        self.assertEqual(len(result['validation_records']), 20)
        self.assertEqual(len(result['retest_records']), 27)
        self.assertEqual(result['validation_records'][:10], saved['validation_records'])
        self.assertEqual(result['retest_records'][0], saved['retest_records'][0])

    def test_completed_quality_failure_cannot_be_resumed_as_extra_passes(self):
        def drift(plant, row):
            if row.get('phase') == 'point':
                plant.signal_gain = 1.1 if row.get('stage') == 'validation' else 1
        _, result, _, _ = self.run_plan(rounds=1, staged=True, rest_seconds=0, progress_hook=drift)
        self.assertFalse(result['ok'])
        with self.assertRaisesRegex(ValueError, '三批'):
            validate_resume(saved_document(result, rounds=1))

    def test_reject_corrupt_duplicate_or_incompatible_context_without_mutation(self):
        saved = self.interrupted()
        for mutate in (lambda d: d['records'].append(d['records'][-1]),
                       lambda d: d['records'].pop(2),
                       lambda d: d['records'][0].update(signal_sum=float('nan')),
                       lambda d: d.update(warmup_records=[])):
            bad = deepcopy(saved)
            mutate(bad)
            with self.assertRaises(ValueError): validate_resume(bad)
        learned = LearnedPressureMap(PROFILE.model, PROFILE.namespace, PROFILE.steps_per_turn,
                                     0, ((.1, 12.5), (1, 35), (30, 760)))
        current = saved['metadata']['motion_settings']
        check_resume_context(saved, PROFILE, 0, learned, current)
        saved['metadata']['learning_fingerprint'] = learning_fingerprint(learned)
        check_resume_context(saved, PROFILE, 0, learned, current)
        with self.assertRaises(ValueError): check_resume_context(saved, PROFILE, 2, learned, current)
        with self.assertRaises(ValueError):
            check_resume_context(saved, PROFILE, 0, replace(learned, points=((.1, 13), (1, 36), (30, 770))), current)
        with self.assertRaises(ValueError):
            check_resume_context(saved, PROFILE, 0, learned, dict(current, force_limit_n=34))

    def test_discovery_backup_and_completed_descendant_suppress_old_checkpoint(self):
        saved = self.interrupted()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            original = root/'high_force_20260921_160712_334961.json'
            original.write_text(json.dumps(saved), encoding='utf-8')
            found = discover_resume(root, PROFILE.namespace)
            self.assertEqual(found['path'], original)
            backup = root/('.'+original.name+'.unique.pending.json')
            newer = deepcopy(saved)
            newer['record_revision'] = 2
            backup.write_text(json.dumps(newer), encoding='utf-8')
            self.assertEqual(discover_resume(root, PROFILE.namespace)['path'], backup)
            resumed = root/'high_force_20260921_170000_000000.json'
            newer['metadata'].update(resume_root_id=original.stem, resume_generation=1)
            resumed.write_text(json.dumps(newer), encoding='utf-8')
            self.assertEqual(discover_resume(root, PROFILE.namespace)['path'], resumed)
            newer['complete'] = True
            newer['record_revision'] = 3
            resumed.write_text(json.dumps(newer), encoding='utf-8')
            self.assertIsNone(discover_resume(root, PROFILE.namespace))
            self.assertFalse(json.loads(original.read_text())['complete'])

    def test_actual_interrupted_record_next_is_round_three_17n(self):
        path = Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor/force_press_tests/high_force_calibration/high_force_20260921_160712_334961.json'
        if not path.exists(): self.skipTest('历史记录未随仓库分发')
        candidate = read_resume(path)
        self.assertIn('第3轮 17 N', candidate['summary'])
        self.assertEqual(len(candidate['document']['records']), 25)


class ResumeUiTests(unittest.TestCase):
    page = native.HighCalibrationUiTests.page
    setUp = native.HighCalibrationUiTests.setUp

    @classmethod
    def setUpClass(cls):
        cls.app = native.QtWidgets.QApplication.instance() or native.QtWidgets.QApplication([])

    def test_resume_button_restores_plan_saves_new_lineage_and_publishes_after_only_missing_points(self):
        source = self.page()
        source.control._release = 0
        profile = source.control.profile
        source._learned_map = LearnedPressureMap(profile.model, profile.namespace, profile.steps_per_turn,
                                                0, ((.1, 12.5), (1, 35), (30, 760)))
        source.inputs['force_limit_n'].setValue(35)
        fixture = ResumeTests()
        saved = fixture.interrupted()
        saved['metadata'].update(profile=profile.metadata(), namespace=profile.namespace)
        path = self.root/'high_force_20260921_160712_334961.json'
        path.write_text(json.dumps(saved), encoding='utf-8')
        before = path.read_bytes()
        plant = native.feedback_tests.Plant()
        source.samples = plant.samples
        plant.read_feedback()
        sensor = lambda: dict(sequence=round(plant.now*1000), age_seconds=0,
            state=[6 if plant.force() > 0 else 0]*8, signal=[0]*4+[round(plant.force()*1000)]*4)
        with patch('force_signal_pressure_lookup.signal_pressure_latest_mapping_path', return_value=self.root/'none.json'):
            k_page = native.ForceSignalPressureCalibrationWidget()
        self.addCleanup(k_page.deleteLater)
        widget = HighForceCalibrationWidget(source.control, source, sensor, k_page, output_dir=self.root)
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.shutdown)
        self.assertIn('第3轮 17 N', widget.resume_label.text())
        tasks = []
        def factory(*args, **kwargs):
            kwargs['clock'] = lambda: plant.now
            kwargs['recovery_cancel'] = native.ClockCancel(plant, kwargs['recovery_cancel'].is_set)
            args = list(args)
            args[4] = native.ClockCancel(plant, args[4].is_set)
            return HighForceCalibrationRunner(*args, **kwargs)
        with patch('time.monotonic', lambda: plant.now), \
                patch.object(source.control, 'is_can_connected', return_value=True), \
                patch.object(source.control, 'is_operation_busy', return_value=False), \
                patch.object(source.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
                patch('high_force_calibration_widget.HighForceCalibrationRunner', side_effect=factory), \
                patch('force_signal_pressure_calibration.signal_pressure_mapping_directory', return_value=self.root/'maps'):
            widget._refresh()
            self.assertTrue(widget.resume_button.isEnabled())
            widget.resume_button.click()
            self.assertTrue(widget.is_running(), widget.status_label.text())
            self.assertEqual(widget.round_count.value(), 3)
            self.assertEqual(widget.rest_seconds.value(), 0)
            self.assertFalse(widget.resume_button.isEnabled())
            tasks[0](plant, plant)
            self.assertTrue(widget._result['ok'], widget.status_label.text())
            self.assertEqual(path.read_bytes(), before)
            self.assertNotEqual(path, widget._path)
            final = json.loads(widget._path.read_text(encoding='utf-8'))
            self.assertEqual(final['records'][:25], saved['records'])
            self.assertEqual(len(final['records']), 30)
            self.assertEqual(final['metadata']['resume_root_id'], path.stem)
            self.assertEqual(final['metadata']['resume_generation'], 1)
            self.assertTrue(final['metadata']['resume_source_sha256'])
            self.assertEqual(len(k_page._document['nodes']), 10)
            self.assertIsNone(discover_resume(self.root, profile.namespace))

    def test_unsaved_memory_is_retained_if_storage_still_unavailable(self):
        from force_signal_pressure_calibration import ForceSignalPressureCalibrationError
        source = self.page()
        source.control._release = 0
        widget = HighForceCalibrationWidget(source.control, source, lambda: {}, None, output_dir=self.root)
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.shutdown)
        pending = ResumeTests().interrupted()
        pending['record_storage'] = dict(primary_path=str(self.root/'high_force_pending.json'))
        widget._pending_record_document = pending
        with patch.object(source.control, 'is_can_connected', return_value=True), \
                patch.object(source.control, 'is_operation_busy', return_value=False), \
                patch('high_force_calibration_widget.CalibrationRecordStore.save',
                      side_effect=ForceSignalPressureCalibrationError('磁盘仍不可写')), \
                patch.object(source.control, 'start_external_operation') as launch:
            widget._refresh()
            self.assertFalse(widget.start_button.isEnabled())
            self.assertTrue(widget.resume_button.isEnabled())
            widget.resume_button.click()
            launch.assert_not_called()
            self.assertIs(widget._pending_record_document, pending)
            self.assertEqual(len(pending['records']), 25)
            self.assertIn('磁盘仍不可写', widget.status_label.text())

    def test_complete_unsaved_memory_only_finishes_storage_without_motor_motion(self):
        source = self.page()
        source.control._release = 0
        widget = HighForceCalibrationWidget(source.control, source, lambda: {}, None, output_dir=self.root)
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.shutdown)
        _, result, _, _ = ResumeTests().run_plan(rounds=1, staged=True, rest_seconds=0)
        pending = saved_document(result, rounds=1)
        pending['record_storage'] = dict(primary_path=str(self.root/'high_force_pending.json'))
        widget._pending_record_document = pending
        with patch.object(source.control, 'is_can_connected', return_value=True), \
                patch.object(source.control, 'is_operation_busy', return_value=False), \
                patch.object(source.control, 'start_external_operation') as launch, \
                patch.object(widget, '_finished') as finished:
            widget._refresh()
            widget.resume_button.click()
            launch.assert_not_called()
            finished.assert_called_once()
            self.assertTrue(finished.call_args.args[0]['ok'])
            self.assertIsNone(widget._pending_record_document)
            self.assertTrue(json.loads(Path(pending['record_storage']['primary_path']).read_text(encoding='utf-8'))['complete'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
