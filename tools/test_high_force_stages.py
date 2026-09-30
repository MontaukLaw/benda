"""三阶段标定：真实控制器虚拟时钟、持续漂移、独立验收及异常先释放。"""
import json
from pathlib import Path
import unittest
import test_high_force_calibration as calibration_tests
from high_force_calibration import build_high_force_mapping
from high_force_calibration_quality import (assess_mapping, historical_focus, paired_mapping,
    validate_accuracy, WARMUP_MAX)


class StageTests(unittest.TestCase):
    run_plan = calibration_tests.HighCalibrationTests.run_plan

    def test_full_pipeline_native_motor_warmup_hold_rest_and_independent_validation(self):
        for count in (10, 20):
            plant, result, phases, _ = self.run_plan(count=count, rounds=2, staged=True, rest_seconds=0)
            self.assertTrue(result['ok'], result['message'])
            self.assertEqual(plant.position, 0)
            self.assertEqual(len(result['warmup_records']), 5)
            self.assertEqual(len(result['records']), count*2)
            self.assertEqual(len(result['validation_records']), count)
            self.assertEqual(result['retest_records'], [])
            warm_rest = [r for r in result['rest_periods'] if r['stage'] == 'warmup']
            self.assertEqual(len(warm_rest), 4)
            self.assertTrue(all(r['elapsed_s'] >= 10 and not r['skipped'] for r in warm_rest))
            self.assertTrue(all(r['hold_s'] == 5 for r in result['warmup_records']))
            holding = [r for r in phases if r['phase'] == 'warmup_hold']
            self.assertTrue(holding)
            self.assertGreaterEqual(max(r['hold_elapsed_s'] for r in holding), 5)
            self.assertTrue(result['quality']['passed'])
            self.assertTrue(result['document']['metadata']['independent_validation'])
            self.assertEqual([t['stage'] for t in result['stage_timings']], ['warmup','formal','retest'])
            self.assertAlmostEqual(sum(t['elapsed_s'] for t in result['stage_timings']), result['elapsed_s'])

    def test_warmup_extends_until_recent_three_converge(self):
        def gain(plant, row):
            if row.get('phase') == 'point' and row.get('stage') == 'warmup':
                plant.signal_gain = 1+.03*min(row['point'], 5)
        _, result, _, _ = self.run_plan(staged=True, rounds=1, rest_seconds=0, progress_hook=gain)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['warmup_records']), 7)

    def test_continuous_warmup_drift_releases_and_keeps_old_table(self):
        def gain(plant, row):
            if row.get('phase') == 'point' and row.get('stage') == 'warmup':
                plant.signal_gain = 1+.03*row['point']
        plant, result, _, _ = self.run_plan(staged=True, rest_seconds=0, progress_hook=gain)
        self.assertFalse(result['ok'])
        self.assertEqual(len(result['warmup_records']), WARMUP_MAX)
        self.assertEqual(result['records'], [])
        self.assertNotIn('document', result)
        self.assertEqual(plant.position, 0)
        self.assertTrue(result['release_position_confirmed'])

    def test_historical_target_is_retested_three_times_before_validation(self):
        _, result, _, _ = self.run_plan(staged=True, rounds=1, rest_seconds=0, historical_targets=(24,30))
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual([r['target_force_n'] for r in result['retest_records']], [24]*3+[30]*3)
        self.assertEqual(result['quality']['passes'][0]['retest_targets'], [24,30])
        self.assertEqual(len(result['validation_records']), 10)

    def test_formal_outlier_gets_targeted_replacement_without_deleting_original(self):
        def gain(plant, row):
            if row.get('phase') == 'point':
                plant.signal_gain = 1.08 if row.get('stage') == 'formal' and row['round'] == 1 and row['target_n'] == 24 else 1
        _, result, _, _ = self.run_plan(staged=True, rounds=3, rest_seconds=0, progress_hook=gain)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['records']), 30)
        self.assertEqual([r['target_force_n'] for r in result['retest_records']], [24]*3)
        self.assertGreater(result['quality']['passes'][0]['initial']['max_error_n'], 1)
        self.assertEqual(len(result['validation_records']), 10)

    def test_new_drift_during_validation_cannot_self_fit_and_pass(self):
        def gain(plant, row):
            if row.get('phase') == 'point':
                plant.signal_gain = 1.1 if row.get('stage') == 'validation' else 1
        plant, result, _, _ = self.run_plan(staged=True, rounds=1, rest_seconds=0, progress_hook=gain)
        self.assertFalse(result['ok'])
        self.assertNotIn('document', result)
        self.assertEqual(len(result['quality']['passes']), 3)
        self.assertTrue(all(not p['validation']['passed'] for p in result['quality']['passes']))
        self.assertEqual(plant.position, 0)
        self.assertTrue(result['release_position_confirmed'])

    def test_changed_response_is_refit_then_validated_with_fresh_presses(self):
        changed = []
        def gain(plant, row):
            if row.get('phase') == 'point' and row.get('stage') == 'validation':
                changed.append(True)
            if changed:
                plant.signal_gain = 1.05
        _, result, _, _ = self.run_plan(staged=True, rounds=1, rest_seconds=0, progress_hook=gain)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['quality']['passes']), 2)
        self.assertFalse(result['quality']['passes'][0]['validation']['passed'])
        self.assertTrue(result['quality']['passes'][1]['validation']['passed'])
        self.assertEqual(len(result['validation_records']), 20)
        # 1 N档5%的偏差仍在±0.05 N绝对容差内，只补测其余9点。
        self.assertEqual(len(result['retest_records']), 27)

    def test_stop_in_each_stage_always_releases_without_publishing(self):
        for stage in ('warmup','formal','retest','validation'):
            def stop(plant, row):
                if row.get('phase') == 'point' and row.get('stage') == stage:
                    plant.cancelled = True
            plant, result, _, _ = self.run_plan(staged=True, rest_seconds=0,
                historical_targets=(30,), progress_hook=stop)
            self.assertFalse(result['ok'], stage)
            self.assertNotIn('document', result)
            self.assertEqual(plant.position, 0)
            self.assertTrue(result['release_position_confirmed'])

    def test_loaded_stop_in_warmup_hold_and_retest_capture_releases(self):
        for target_stage in ('warmup','retest','validation'):
            context = {}
            def stop(plant, row):
                if row.get('phase') == 'point': context.update(row)
                if context.get('stage') == target_stage and row.get('phase') in ('warmup_hold','capture'):
                    self.assertLess(plant.position, 0)
                    plant.cancelled = True
            plant, result, _, _ = self.run_plan(staged=True, rounds=1, rest_seconds=0,
                historical_targets=(30,), progress_hook=stop)
            self.assertFalse(result['ok'])
            self.assertEqual(plant.position, 0)
            self.assertTrue(result['release_position_confirmed'])

    def test_fault_in_retest_capture_recovers_and_retries_only_current_point(self):
        context, injected = {}, []
        def fault(plant, row):
            if row.get('phase') == 'point': context.update(row)
            if context.get('stage') == 'retest' and row.get('phase') == 'capture' and not injected:
                injected.append(True)
                plant.fault = 'stale'
            if row.get('phase') == 'recovery': plant.fault = None
        plant, result, _, _ = self.run_plan(staged=True, rounds=1, rest_seconds=0,
            historical_targets=(30,), progress_hook=fault)
        self.assertTrue(result['ok'], result['message'])
        self.assertEqual(len(result['records']), 10)
        self.assertEqual(len(result['retest_records']), 3)
        self.assertEqual(result['retest_records'][0]['attempt'], 2)
        self.assertEqual(plant.position, 0)

    def test_warmup_release_lost_ack_recovers_before_next_press(self):
        injected = []
        def setup(plant):
            move = plant.move_relative_steps
            def lose_ack(steps, **kwargs):
                move(steps, **kwargs)
                if steps > 0 and not injected:
                    injected.append(True)
                    raise RuntimeError('释放位移已执行但ACK丢失')
            plant.move_relative_steps = lose_ack
        plant, result, _, _ = self.run_plan(staged=True, rounds=1, rest_seconds=0, setup=setup)
        self.assertTrue(result['ok'], result['message'])
        self.assertTrue(injected)
        self.assertEqual(plant.position, 0)
        self.assertEqual(len(result['warmup_records']), 5)
        self.assertTrue(result['release_position_confirmed'])


class QualityTests(unittest.TestCase):
    def test_nodes_always_preserve_a_real_signal_pressure_pair(self):
        records = []
        from high_force_calibration import calibration_targets
        for cycle, scale in enumerate((1,1.01,1.02),1):
            for target in calibration_targets(10):
                records.append(dict(round=cycle,target_force_n=target, signal_sum=target*4000*scale,
                    final_force_n=target*(1.01 if cycle==1 else 1 if cycle==2 else .99), floating_count=4))
        document = build_high_force_mapping(records,10,3)
        for node in document['nodes']:
            actual = [(r['signal_sum'],r['final_force_n']) for r in records if r['target_force_n']==node['nominal_target_force_n']]
            self.assertIn((node['signal_sum_median'],node['measured_pressure_median_n']), actual)

    def test_nonmonotonic_pairs_are_rejected_without_reordering(self):
        groups={1:[dict(signal_sum=100,final_force_n=1,floating_count=4)],
                4:[dict(signal_sum=100,final_force_n=4,floating_count=4)]}
        with self.assertRaises(ValueError): paired_mapping(groups,{})

    def test_actual_problem_history_is_rejected_and_high_error_targets_selected(self):
        root=Path(__file__).resolve().parents[1]
        path=root/'PY_SRIPT/algo_monitor/force_press_tests/high_force_calibration/high_force_20260920_181115_057150.json'
        if not path.exists(): self.skipTest('实测历史仅保存在本地')
        data=json.loads(path.read_text(encoding='utf-8'))
        assessment=assess_mapping(data['document'],data['records'],2)
        self.assertFalse(assessment['passed'])
        self.assertGreater(assessment['max_error_n'],2.3)
        focus=historical_focus(data['document'],data['records'],tuple(range(1,31)),2)
        self.assertEqual(focus,[24,25,27,28,30])
        rebuilt=build_high_force_mapping(data['records'],20,5)
        self.assertFalse(assess_mapping(rebuilt,data['records'],2)['passed'])

    def test_accuracy_settings_reject_invalid_numbers(self):
        for value in (0, .49, 10.1, True, float('nan'), float('inf'), '2'):
            with self.assertRaises(ValueError): validate_accuracy(value)


if __name__ == '__main__':
    unittest.main(verbosity=2)
