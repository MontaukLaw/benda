"""独立整数位置学习、回位保护、JSON隔离与共享线程界面验证。"""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import test_pressure_feedback as native
from test_high_force_calibration import ClockCancel
from one_step_learning import (run_learning, default_settings, validate_settings, save_document,
                               load_document, latest_path, validate_document)
from one_step_learning_widget import OneStepLearningWidget
from qt_compat import QtWidgets


class LearningTests(unittest.TestCase):
    def learn(self, on_point=None, progress=None, setup=None):
        plant = native.Plant()
        plant.emergency = False
        if setup: setup(plant)
        rows = []
        def point(row):
            rows.append(row)
            if on_point: on_point(plant, row)
        result = run_learning(plant, native.PROFILE, plant.samples, plant,
            ClockCancel(plant, lambda: plant.emergency), default_settings(), 0,
            on_point=point, progress=lambda row: progress(plant, row) if progress else None,
            clock=lambda: plant.now)
        return plant, result, rows

    def test_all_integer_targets_small_steps_real_positions_and_release(self):
        plant, result, rows = self.learn()
        self.assertTrue(result['complete'], result['message'])
        self.assertEqual([r['target_force_n'] for r in rows], list(range(31)))
        self.assertEqual(result['nodes'], rows)
        self.assertEqual(plant.position, 0)
        self.assertTrue(result['pressure_unloaded'])
        self.assertTrue(all(abs(step) <= 4 for step in plant.actions[:-1]))
        self.assertGreater(plant.actions[-1], 4)
        for row in rows[1:]:
            self.assertLessEqual(abs(row['measured_force_n']-row['target_force_n']), .05+1e-9)
            self.assertAlmostEqual(row['measured_force_n'], max(0,(-row['position_steps']-10)*.04))
            self.assertEqual(row['travel_steps'], -row['position_steps'])
        self.assertIs(validate_document(result, native.PROFILE), result)

    def test_ordinary_stop_keeps_partial_rows_and_returns_to_origin(self):
        def stop(plant, row):
            if row['target_force_n'] == 5: plant.cancelled = True
        plant, result, _ = self.learn(on_point=stop)
        self.assertFalse(result['complete'])
        self.assertEqual(len(result['nodes']), 6)
        self.assertEqual(plant.position, 0)
        self.assertTrue(result['release_position_confirmed'])

    def test_nonzero_origin_preserves_absolute_positions_and_relative_travel(self):
        plant = native.Plant()
        origin = 137
        plant.position = origin
        plant.force = lambda: max(0, (origin-plant.position-10)*.04)
        result = run_learning(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            default_settings(), origin, clock=lambda: plant.now)
        self.assertTrue(result['complete'], result['message'])
        self.assertEqual(plant.position, origin)
        self.assertEqual(result['nodes'][0]['position_steps'], origin)
        self.assertTrue(all(row['position_steps']+row['travel_steps'] == origin for row in result['nodes']))

    def test_emergency_never_returns_automatically(self):
        def urgent(plant, row):
            if row['target_force_n'] == 5: plant.emergency = True
        plant, result, _ = self.learn(on_point=urgent)
        self.assertFalse(result['complete'])
        self.assertFalse(result['release_position_confirmed'])
        self.assertLess(plant.position, 0)
        self.assertTrue(all(step < 0 for step in plant.actions))

    def test_pressure_loss_while_loaded_still_releases(self):
        def fault(plant, row):
            if row['target_force_n'] == 5: plant.fault = 'stale'
        plant, result, _ = self.learn(on_point=fault)
        self.assertFalse(result['complete'])
        self.assertEqual(plant.position, 0)
        self.assertTrue(result['release_position_confirmed'])
        self.assertFalse(result['pressure_unloaded'])

    def test_stop_during_seek_or_stable_sampling_returns_before_finishing(self):
        for in_motion in (True, False):
            def progress(plant, row):
                if (row.get('target_n') == 2 and plant.position < 0
                        and bool(row.get('command_steps')) == in_motion):
                    plant.cancelled = True
            plant, result, _ = self.learn(progress=progress)
            self.assertFalse(result['complete'])
            self.assertTrue(result['cancelled'])
            self.assertEqual(plant.position, 0)
            self.assertTrue(result['release_position_confirmed'])

    def test_current_fault_waits_for_recovery_before_any_return_motion(self):
        faulted = []
        def point(plant, row):
            if row['target_force_n'] == 3:
                plant.current = 3500
                faulted.append(len(plant.actions))
        def progress(plant, row):
            if (row.get('phase') == 'recovery' and faulted and plant.current > 3000
                    and row.get('recovery_attempt', 0) >= 2):
                self.assertEqual(len(plant.actions), faulted[0])
                plant.current = 200
        plant, result, _ = self.learn(on_point=point, progress=progress)
        self.assertFalse(result['complete'])
        self.assertTrue(result['release_position_confirmed'])
        self.assertEqual(plant.position, 0)

    def test_release_lost_ack_uses_actual_position_without_repeating_move(self):
        def setup(plant):
            move = plant.move_relative_steps
            def lost_ack(steps, **kwargs):
                move(steps, **kwargs)
                if steps > 4: raise RuntimeError('release ACK lost after execution')
            plant.move_relative_steps = lost_ack
        plant, result, _ = self.learn(setup=setup)
        self.assertTrue(result['complete'], result['message'])
        self.assertEqual(plant.position, 0)
        self.assertEqual(sum(step > 4 for step in plant.actions), 1)

    def test_nonzero_start_refuses_loading_and_returns_first(self):
        plant, result, rows = self.learn(setup=lambda plant: setattr(plant, 'position', -30))
        self.assertFalse(result['complete'])
        self.assertEqual(rows, [])
        self.assertEqual(plant.position, 0)
        self.assertTrue(all(step > 0 for step in plant.actions))

    def test_each_point_has_own_deadline(self):
        plant = native.Plant()
        result = run_learning(plant, native.PROFILE, plant.samples, plant, ClockCancel(plant),
            replace(default_settings(), acquire_timeout_s=10), 0, clock=lambda: plant.now)
        self.assertTrue(result['complete'])
        self.assertGreater(result['elapsed_s'], 10)
        for bad in (replace(default_settings(), max_step=17), replace(default_settings(), tolerance_n=.6)):
            with self.assertRaises(ValueError): validate_settings(bad, native.PROFILE)

    def test_independent_json_roundtrip_and_partial_never_replaces_latest(self):
        _, document, _ = self.learn()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old = root/'pressure_feedback_config.learning.json'
            old.write_text('{"old":true}', encoding='utf-8')
            archive = save_document(root/'one_step_learning', document, native.PROFILE)
            self.assertTrue(archive.name.startswith('one_step_learning_'))
            self.assertEqual(load_document(root/'one_step_learning', native.PROFILE), document)
            original = latest_path(root/'one_step_learning', native.PROFILE).read_bytes()
            partial = dict(document, complete=False, nodes=document['nodes'][:8])
            save_document(root/'one_step_learning', partial, native.PROFILE)
            self.assertEqual(latest_path(root/'one_step_learning', native.PROFILE).read_bytes(), original)
            self.assertEqual(old.read_text(), '{"old":true}')
            for change in ({'kind':'old_learning'}, {'namespace':'other_motor'}, {'nodes':document['nodes'][:-1]}):
                with self.assertRaises(ValueError): validate_document(dict(document, **change), native.PROFILE)


class LearningUiTests(unittest.TestCase):
    page = native.FeedbackSettingsUiTests.page

    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)

    def widget(self, source):
        widget = OneStepLearningWidget(source.control, source.samples, output_dir=self.root/'one_step_learning')
        self.addCleanup(widget.deleteLater)
        self.addCleanup(widget.shutdown)
        return widget

    def test_origin_is_independent_zero_invalidates_reference_and_busy_blocks_start(self):
        source = self.page()
        plant = native.Plant()
        source.samples = plant.samples
        widget = self.widget(source)
        source.control._release = 123
        with patch('time.monotonic', lambda: plant.now), \
             patch.object(source.control, 'is_can_connected', return_value=True), \
             patch.object(source.control, 'is_operation_busy', return_value=False), \
             patch.object(source.control, 'latest_motor_feedback', side_effect=plant.read_feedback), \
             patch.object(source.control, 'zero_motor_position') as zero:
            plant.read_feedback()
            widget.capture_origin()
            self.assertEqual(widget._origin, 0)
            self.assertEqual(source.control._release, 123)
            with patch.object(source.control, 'is_operation_busy', return_value=True):
                widget._refresh()
                self.assertFalse(widget.learn_button.isEnabled())
                self.assertFalse(widget.zero_button.isEnabled())
            widget.zero_position()
            zero.assert_called_once()
            self.assertIsNone(widget._origin)
            self.assertFalse(widget.learn_button.isEnabled())

    def test_complete_worker_updates_31_rows_and_saves_separate_file(self):
        source = self.page()
        plant = native.Plant()
        source.samples = plant.samples
        widget = self.widget(source)
        tasks = []
        def virtual_run(motor, profile, samples, stop, emergency, settings, origin, **kwargs):
            return run_learning(motor, profile, samples, ClockCancel(plant, stop.is_set),
                ClockCancel(plant, emergency.is_set), settings, origin, clock=lambda: plant.now, **kwargs)
        with patch('time.monotonic', lambda: plant.now), \
             patch.object(source.control, 'is_can_connected', return_value=True), \
             patch.object(source.control, 'is_operation_busy', return_value=False), \
             patch.object(source.control, 'latest_motor_feedback', side_effect=plant.read_feedback), \
             patch.object(source.control, 'start_external_operation', side_effect=lambda task, name: (tasks.append(task) or True, '')), \
             patch('one_step_learning_widget.run_learning', virtual_run):
            plant.read_feedback()
            widget.capture_origin()
            widget.start()
            self.assertTrue(widget.is_running())
            tasks[0](plant, ClockCancel(plant))
            self.assertFalse(widget.is_running())
            self.assertTrue(widget._document['complete'], widget.status_label.text())
            self.assertEqual(widget.table.item(30, 0).text(), '30')
            self.assertEqual(float(widget.table.item(30, 1).text()), widget._document['nodes'][30]['measured_force_n'])
            self.assertTrue(widget._archive.exists())
            self.assertEqual(source._learned_map, None)
            reloaded = self.widget(source)
            self.assertIsNone(reloaded._origin)
            self.assertEqual(reloaded.table.item(30, 2).text(), widget.table.item(30, 2).text())
            self.assertFalse(reloaded.learn_button.isEnabled())


if __name__ == '__main__':
    unittest.main(verbosity=2)
