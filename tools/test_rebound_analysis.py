"""回弹指标的时间、缺口、续测和异步界面回归，不连接硬件。"""
import json
from contextlib import closing
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import sqlite3
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from datetime import datetime, timedelta
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from rebound_analysis import analyze_rebound, first_confirmed, session_chain, hold_raw_reference, residual_percent
from long_duration_creep_test_widget import LongDurationCreepTestWidget as Widget
from qt_compat import QtWidgets
from rebound_test_history import saved_tests, saved_reports


def fixture(root, name='creep_test', parent=None, planned=1, residual=False, window=360):
    path=root/name;path.mkdir()
    m=dict(config={'protocol':'raw_rebound_v1'},status='completed',resumed_from=str(parent) if parent else None,
        output_parts=['raw_data_part_001.sqlite3'],created_at='2026-09-15T00:00:00+08:00',
        finished_at=(datetime.fromisoformat('2026-09-15T00:00:00+08:00')+timedelta(seconds=window+22)).isoformat(),dropped_count=0,recorder_error=None)
    (path/'manifest.json').write_text(json.dumps(m),encoding='utf8')
    cols=['sensor_source_monotonic','trial_index','attempt_index','sensor_sequence',
          'die_temperature_celsius','die_temperature_age_seconds']
    cols += [f'ch{c}_{v}' for c in range(4,8) for v in ('normalized_raw','baseline','state')]
    conn=sqlite3.connect(path/'raw_data_part_001.sqlite3')
    try:
        conn.execute('CREATE TABLE samples (id INTEGER PRIMARY KEY,'+','.join(k+' REAL' for k in cols)+')')
        rows=[]
        for i in range((window+22)*20):
            t=i/20
            raw=1000+(20 if t>=22 and (residual or t<30) else 0)
            rows.append([i+1,t,1,1,i,33.5,0]+[raw,1000,0 if t>=25 else 6]*4)
        conn.executemany('INSERT INTO samples VALUES ('+','.join('?' for _ in range(19))+')',rows)
        conn.execute('CREATE TABLE events (id INTEGER PRIMARY KEY,event TEXT,payload_json TEXT)')
        e=[]
        for i in range((window+22)*20):
            t=i/20;e.append(('pressure_sample',json.dumps(dict(sampled_monotonic=t,raw_force_n=1 if 11<=t<21 else 0,accepted=True))))
        e.append(('trial_complete',json.dumps(dict(trial_index=1,attempt_index=1,planned_trial_index=planned,
            repeat_index=1,high_force_n=1,high_hold_seconds=10,controller_monotonic=window+22,
            trial_result=dict(press_result=dict(motion_started_monotonic=10,arrived_monotonic=11),
                release_result=dict(release_command_monotonic=21,release_zero_confirmed_monotonic=22))))))
        conn.executemany('INSERT INTO events(event,payload_json) VALUES (?,?)',e);conn.commit()
    finally: conn.close()
    return path


class MetricsTests(unittest.TestCase):
    def test_current_raw_columns_override_legacy_values_and_are_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = fixture(Path(tmp), window=10)
            with closing(sqlite3.connect(path/'raw_data_part_001.sqlite3')) as db:
                for channel in range(4,8):
                    db.execute(f'ALTER TABLE samples ADD COLUMN ch{channel}_hardware_raw REAL')
                    db.execute(f'UPDATE samples SET ch{channel}_hardware_raw=ch{channel}_normalized_raw, ch{channel}_normalized_raw=99999')
                db.commit()
            report = analyze_rebound(path)['report']
            self.assertEqual(report['channels'][0]['pre_raw'], 1000)
            self.assertEqual(report['source_files'][0]['raw_columns']['ch4_hardware_raw'], 'ch4_hardware_raw')

    def test_paired_low_reference_excludes_large_press_and_exports_history(self):
        from rebound_test_protocol import PAIRED_PRESS_TEMPLATE
        for window in (180,0):
            with self.subTest(window=window), tempfile.TemporaryDirectory() as tmp:
                path=fixture(Path(tmp),window=window+10)
                mp=path/'manifest.json'; manifest=json.loads(mp.read_text())
                manifest['config']['template_id']=PAIRED_PRESS_TEMPLATE
                mp.write_text(json.dumps(manifest))
                with closing(sqlite3.connect(path/'raw_data_part_001.sqlite3')) as db:
                    # 20 N在11~14秒保持，15秒确认卸载；等5秒，20秒发令2 N，21~24秒保持。
                    db.execute('UPDATE samples SET '+','.join(f'''ch{c}_normalized_raw=CASE
                        WHEN sensor_source_monotonic<10 THEN 1000
                        WHEN sensor_source_monotonic<15 THEN 9000
                        WHEN sensor_source_monotonic<20 THEN 1200
                        WHEN sensor_source_monotonic<25 THEN 1700 ELSE 1100 END''' for c in range(4,8)))
                    db.execute('DELETE FROM samples WHERE sensor_source_monotonic>=?',(25+window,))
                    high=dict(press_result=dict(motion_started_monotonic=10,arrived_monotonic=11),
                              release_result=dict(release_command_monotonic=14,release_zero_confirmed_monotonic=15))
                    small=dict(press_result=dict(motion_started_monotonic=20,arrived_monotonic=21),
                               release_result=dict(release_command_monotonic=24,release_zero_confirmed_monotonic=25))
                    payload=dict(trial_index=1,attempt_index=1,planned_trial_index=1,repeat_index=1,
                        high_force_n=20,high_hold_seconds=3,controller_monotonic=25+window,
                        trial_result=dict(category='paired_rebound',high_force_n=20,probe_force_n=2,
                            probe_hold_seconds=3,recovery_seconds=5,post_observation_seconds=window,
                            paired_results=dict(high=high,probe=small)))
                    db.execute("UPDATE events SET payload_json=? WHERE event='trial_complete'",(json.dumps(payload),))
                    db.commit()
                result=analyze_rebound(path); r=result['report']; ch=r['channels'][0]
                self.assertTrue(r['paired_observation'])
                self.assertEqual(r['trials'][0]['target_n'],2)
                self.assertEqual(r['trials'][0]['hold_actual_s'],3)
                self.assertEqual(r['trials'][0]['delay_actual_s'],5)
                self.assertEqual(ch['large_pre_raw'],1000)
                self.assertEqual(ch['large_hold_raw_median'],9000)
                self.assertEqual(ch['pre_raw'],1200)
                self.assertEqual(ch['residual_before_small_counts'],200)
                self.assertEqual(ch['hold_response_counts'],500)
                self.assertEqual(ch['small_hold_from_large_reference_counts'],700)
                self.assertNotIn('recovery_sampling_gap',r['trials'][0]['quality_issues'])
                if window==0: self.assertIsNone(ch['residual_10s'])
                page=(Path(result['output_dir'])/'report.html').read_text(encoding='utf8')
                self.assertIn('5秒与60秒间隔对比',page)
                self.assertIn('小压力增量counts',page)

    def test_short_press_reuses_previous_trial_tail_and_preserves_accumulated_offset(self):
        from rebound_test_protocol import SHORT_PRESS_TEMPLATE
        self._check_continuous_press_reference(SHORT_PRESS_TEMPLATE)

    def test_repeat_short_2n_reuses_previous_tail_and_preserves_template_identity(self):
        from rebound_test_protocol import REPEAT_SHORT_2N_TEMPLATE
        self._check_continuous_press_reference(REPEAT_SHORT_2N_TEMPLATE)

    def _check_continuous_press_reference(self, template_id):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp),window=60)
            p=path/'manifest.json';m=json.loads(p.read_text())
            m['config']['template_id']=template_id
            m['finished_at']='2026-09-15T00:02:34+08:00'
            p.write_text(json.dumps(m))
            with closing(sqlite3.connect(path/'raw_data_part_001.sqlite3')) as db:
                db.execute('UPDATE samples SET '+','.join(f'ch{c}_normalized_raw=1010' for c in range(4,8))+
                           ' WHERE sensor_source_monotonic>=60')
                columns=[r[1] for r in db.execute('PRAGMA table_info(samples)')][1:]
                rows=db.execute('SELECT '+','.join(columns)+' FROM samples WHERE sensor_source_monotonic>=10').fetchall()
                new=[]
                for row in rows:
                    row=list(row);row[0]+=72;row[1]=2;row[3]+=2000
                    for c in range(4,8): row[columns.index(f'ch{c}_normalized_raw')]=1500 if row[0]<94 else 1030
                    new.append(row)
                db.executemany('INSERT INTO samples ('+','.join(columns)+') VALUES ('+','.join('?' for _ in columns)+')',new)
                payload=json.loads(db.execute("SELECT payload_json FROM events WHERE event='trial_complete'").fetchone()[0])
                payload.update(trial_index=2,planned_trial_index=2,controller_monotonic=154)
                for group in ('press_result','release_result'):
                    payload['trial_result'][group]={k:v+72 for k,v in payload['trial_result'][group].items()}
                db.execute('INSERT INTO events(event,payload_json) VALUES (?,?)',('trial_complete',json.dumps(payload)))
                db.commit()
            result=analyze_rebound(path);r=result['report'];second=r['channels'][4]
            self.assertEqual(r['trials'][1]['observation_template'],template_id)
            self.assertEqual(second['pre_raw'],1010)
            self.assertEqual(second['hold_response_counts'],490)
            self.assertEqual(second['residual_10s'],20)
            self.assertEqual(second['pre_shift_from_session_start_counts'],10)
            self.assertEqual(second['residual_10s_from_session_start_counts'],30)
            self.assertEqual(second['maximum_abs_residual_10_to_60s_counts'],20)
            self.assertNotIn('insufficient_pre_reference',r['trials'][1]['quality_issues'])
            page=(Path(result['output_dir'])/'report.html').read_text(encoding='utf8')
            self.assertIn('10～60秒最大绝对残差',page)
            self.assertNotIn('300s残差',page.split('<thead>')[-1].split('</thead>')[0])
            with closing(sqlite3.connect(path/'raw_data_part_001.sqlite3')) as db:
                db.execute('DELETE FROM samples WHERE sensor_source_monotonic BETWEEN 75 AND 79');db.commit()
            self.assertIn('pre_reference_sampling_gap',analyze_rebound(path)['report']['trials'][1]['quality_issues'])

    def test_100ms_samples_remain_valid_for_existing_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp))
            with closing(sqlite3.connect(path/'raw_data_part_001.sqlite3')) as db:
                db.execute('DELETE FROM samples WHERE id % 2 = 0')
                db.execute("DELETE FROM events WHERE event='pressure_sample' AND id % 2 = 0")
                db.commit()
            r=analyze_rebound(path)['report']
            self.assertNotIn('recovery_sampling_gap',r['trials'][0]['quality_issues'])
            self.assertNotIn('force_sampling_gap',r['trials'][0]['quality_issues'])
            self.assertTrue(r['channels'][0]['percent_raw_valid'])
            self.assertEqual(r['channels'][0]['confirmed_raw_2counts_5s'],13)
            self.assertEqual(r['channels'][0]['residual_5s'],20)

    def test_calibration_goal_and_trial_roles_are_exported_without_claiming_fitted_coefficients(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp))
            p=path/'manifest.json';m=json.loads(p.read_text())
            m['config']['calibration_goal']={'parameters_per_channel':['k','tau_seconds'],'fit_status':'not_fitted'}
            m['trials']=[{'calibration_model':'single_exponential_calibration_v1','analysis_role':'validation'}]
            p.write_text(json.dumps(m))
            result=analyze_rebound(path);out=Path(result['output_dir'])
            self.assertEqual(result['report']['trials'][0]['analysis_role'],'validation')
            self.assertEqual(result['report']['calibration_goals'][0]['fit_status'],'not_fitted')
            self.assertIn('validation',(out/'trials.csv').read_text(encoding='utf-8-sig'))
            page=(out/'report.html').read_text(encoding='utf8')
            self.assertIn('留出检验预测',page)
            self.assertIn('尚未拟合系数',page)

    def test_recalculation_preserves_raw_legacy_and_each_previous_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp))
            legacy=path/'analysis_v1';legacy.mkdir()
            (legacy/'report.html').write_text('old report',encoding='utf8')
            before={p:p.read_bytes() for p in path.rglob('*') if p.is_file()}
            first=Path(analyze_rebound(path)['output_dir'])
            first_bytes={p:p.read_bytes() for p in first.iterdir()}
            second=Path(analyze_rebound(path)['output_dir'])
            self.assertNotEqual(first,second)
            self.assertEqual([r[0] for r in saved_reports(path)],[second,first,legacy])
            for p,data in (before | first_bytes).items():
                self.assertEqual(p.read_bytes(),data)
            with self.assertRaises(FileExistsError):
                analyze_rebound(path,first)
            with patch('rebound_analysis._write_report',side_effect=OSError('disk full')):
                with self.assertRaises(OSError): analyze_rebound(path)
            self.assertEqual(len(saved_reports(path)),3)

    def test_six_hundred_second_observation_analyzes_late_return_and_actual_tail(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp),window=600,residual=True)
            conn=sqlite3.connect(path/'raw_data_part_001.sqlite3')
            try:
                conn.execute('UPDATE samples SET '+','.join(f'ch{c}_normalized_raw=1000' for c in range(4,8))+
                             ' WHERE sensor_source_monotonic>=422')
                conn.commit()
            finally: conn.close()
            result=analyze_rebound(path);output=Path(result['output_dir'])
            r=result['report'];x=r['channels'][0]
            self.assertEqual(x['confirmed_raw_2counts_5s'],405)
            self.assertEqual(x['tail_residual_counts'],0)
            self.assertEqual(x['residual_300s'],20)
            self.assertEqual(x['residual_600s'],0)
            self.assertIn('600s残差',(output/'report.html').read_text(encoding='utf8'))
            lines=(output/'recovery_1s.csv').read_text(encoding='utf-8-sig').splitlines()
            self.assertEqual(len(lines),2401)

    def test_hold_reference_uses_entire_short_hold_or_last_ten_seconds(self):
        t=np.arange(0,40,.05)
        raw=np.where(t<20,1300.,1500.)
        short=hold_raw_reference(t,raw,10,11,1000,1)
        self.assertEqual(short['hold_raw_window_s'],1)
        self.assertEqual(short['hold_raw_median'],1300)
        self.assertEqual(short['hold_response_counts'],300)
        long=hold_raw_reference(t,raw,1,31,1000,1)
        self.assertEqual(long['hold_raw_window_s'],10)
        self.assertEqual(long['hold_raw_median'],1500)
        self.assertAlmostEqual(residual_percent(-30,short['hold_raw_median'],short['percent_raw_valid']),-30/13)
        self.assertEqual(residual_percent(-30,short['hold_response_counts'],short['percent_response_valid']),-10)

    def test_percentage_rejects_missing_hold_zero_and_noise_denominators(self):
        for raw in (np.zeros(20),np.full(20,1000.),np.full(20,1002.)):
            r=hold_raw_reference(np.arange(20)/20,raw,0,1,1000,1)
            self.assertFalse(r['percent_response_valid'])
            self.assertIsNone(residual_percent(10,r['hold_response_counts'],r['percent_response_valid']))
        missing=hold_raw_reference(np.array([0.,.95]),np.array([1300.,1300.]),0,1,1000,1)
        self.assertFalse(missing['percent_raw_valid'])
        self.assertIsNone(residual_percent(10,missing['hold_raw_median'],missing['percent_raw_valid']))

    def test_confirmation_resets_on_gap_and_out_of_band(self):
        self.assertIsNone(first_confirmed([0,.2,2,2.2],[True]*4))
        self.assertIsNone(first_confirmed([0,.2,.4,.6,.8,1],[True,True,False,True,True,True]))
        self.assertAlmostEqual(first_confirmed(np.arange(0,2,.1),np.ones(20,bool)),1)

    def test_known_curve_and_force_have_expected_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp));before=(path/'manifest.json').read_bytes()
            result=analyze_rebound(path);r=result['report'];x=r['channels'][0]
            self.assertEqual(r['completed_conditions'],1)
            self.assertEqual(x['pre_raw'],1000)
            self.assertEqual(x['residual_5s'],20)
            self.assertEqual(x['residual_10s'],0)
            self.assertEqual(x['confirmed_idle_1s'],4)
            self.assertEqual(x['confirmed_raw_2counts_5s'],13)
            self.assertFalse(x['left_band_after_2counts'])
            self.assertEqual(x['tail_residual_counts'],0)
            self.assertEqual(r['trials'][0]['hold_actual_s'],10)
            self.assertEqual(r['trials'][0]['hold_median_n'],1)
            self.assertAlmostEqual(r['trials'][0]['force_integral_ns'],10,places=5)
            self.assertEqual((path/'manifest.json').read_bytes(),before)
            self.assertTrue((Path(result['output_dir'])/'report.html').is_file())
            self.assertEqual(len(r['source_files'][0]['sha256']),64)

    def test_report_and_exports_include_conditions_hold_medians_and_both_percentages(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp))
            conn=sqlite3.connect(path/'raw_data_part_001.sqlite3')
            try:
                conn.execute('UPDATE samples SET '+','.join(f'ch{c}_normalized_raw=1400' for c in range(4,8))+
                             ' WHERE sensor_source_monotonic>=11 AND sensor_source_monotonic<21')
                conn.execute('UPDATE samples SET '+','.join(f'ch{c}_normalized_raw=990' for c in range(4,8))+
                             ' WHERE sensor_source_monotonic>=30')
                conn.commit()
            finally: conn.close()
            result=analyze_rebound(path);r=result['report']['channels'][0]
            self.assertEqual(r['hold_raw_median'],1400)
            self.assertEqual(r['hold_response_counts'],400)
            self.assertAlmostEqual(r['residual_5s_percent_of_hold_raw'],20/14)
            self.assertEqual(r['residual_5s_percent_of_hold_response'],5)
            self.assertEqual(r['residual_10s_percent_of_hold_response'],-2.5)
            self.assertEqual(r['tail_residual_counts_percent_of_hold_response'],-2.5)
            content=(Path(result['output_dir'])/'report.html').read_text(encoding='utf8')
            recovery=content.split('<h2>四通道恢复</h2>')[1]
            self.assertIn('目标 1 N · 保持 10 s',recovery)
            self.assertIn('实测中位 1.00 N',recovery)
            self.assertIn('Raw: -0.71%',recovery)
            self.assertIn('增量: -2.50%',recovery)
            self.assertIn('tail_residual_counts_percent_of_hold_raw',(Path(result['output_dir'])/'channels.csv').read_text(encoding='utf-8-sig'))

    def test_not_returned_is_null_despite_idle_and_complete_window(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=analyze_rebound(fixture(Path(tmp),residual=True))['report']
            x=r['channels'][0]
            self.assertEqual(x['confirmed_idle_1s'],4)
            self.assertIsNone(x['confirmed_raw_2counts_5s'])
            self.assertIsNone(x['left_band_after_2counts'])
            self.assertEqual(x['tail_residual_counts'],20)

    def test_resume_keeps_distinct_time_axes_and_both_conditions(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=fixture(Path(tmp),'creep_a',planned=1)
            b=fixture(Path(tmp),'creep_b',a,planned=2)
            r=analyze_rebound(b)['report']
            self.assertEqual(r['completed_conditions'],2)
            self.assertEqual(len(r['sessions']),2)
            self.assertEqual([x['confirmed_raw_2counts_5s'] for x in r['channels']],[13]*8)
            m=json.loads((b/'manifest.json').read_text());m['resumed_from']=str(Path(tmp)/'absent')
            (b/'manifest.json').write_text(json.dumps(m))
            with self.assertRaisesRegex(ValueError,'缺少原会话'): session_chain(b)

    def test_running_session_is_not_analyzed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp));p=path/'manifest.json';m=json.loads(p.read_text());m['status']='running';p.write_text(json.dumps(m))
            with self.assertRaisesRegex(ValueError,'尚未结束'): analyze_rebound(path)


class AnalysisWidgetTests(unittest.TestCase):
    def test_history_selection_reload_and_running_test_do_not_open_wrong_report(self):
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as tmp,patch.object(Widget,'_load_settings'),patch.object(Widget,'_save_settings'),patch('long_duration_creep_test_widget.creep_test_directory',return_value=Path(tmp)):
            a=fixture(Path(tmp),'creep_old');b=fixture(Path(tmp),'creep_new')
            legacy=a/'analysis_v1';legacy.mkdir();(legacy/'report.html').write_text('old')
            new=Path(analyze_rebound(a)['output_dir'])
            p=b/'manifest.json';m=json.loads(p.read_text());m['status']='running';p.write_text(json.dumps(m))
            broken=Path(tmp)/'creep_broken';broken.mkdir();(broken/'manifest.json').write_text('broken')
            self.assertEqual(len(saved_tests(tmp)),2)
            controller=SimpleNamespace(operation_progress=Mock(),operation_finished=Mock())
            w=Widget(controller,SimpleNamespace(),lambda:{})
            w.refresh_test_history(a)
            self.assertEqual(w.report_history_combo.count(),2)
            self.assertEqual(w._analysis_output,new)
            w.report_history_combo.setCurrentIndex(1)
            with patch('long_duration_creep_test_widget.QtGui.QDesktopServices.openUrl') as opened:
                w._running=True
                w.open_analysis_report()
                self.assertEqual(Path(opened.call_args.args[0].toLocalFile()),legacy/'report.html')
                with patch.object(w,'_start_analysis') as analyze:
                    w.analyze_selected_test();analyze.assert_not_called()
            w._running=False
            with patch.object(w,'_start_analysis') as analyze:
                w.analyze_selected_test();analyze.assert_called_once_with(a.resolve())
            w.refresh_test_history(b)
            self.assertIsNone(w._analysis_output)
            self.assertFalse(w.open_analysis_button.isEnabled())
            self.assertFalse(w.analyze_button.isEnabled())
            w.shutdown();w.close()
            # 重新创建页面后仍能发现以前的测试及全部报告。
            w=Widget(controller,SimpleNamespace(),lambda:{})
            w.refresh_test_history(a)
            self.assertEqual(w.report_history_combo.count(),2)
            self.assertEqual(w._analysis_output,new)
            w.shutdown();w.close()

    def test_worker_computes_off_gui_thread_and_displays_result(self):
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as tmp,patch.object(Widget,'_load_settings'),patch.object(Widget,'_save_settings'):
            path=fixture(Path(tmp))
            controller=SimpleNamespace(operation_progress=Mock(),operation_finished=Mock())
            w=Widget(controller,SimpleNamespace(),lambda:{})
            caller=[]
            def analyze(p):
                caller.append(threading.current_thread().name)
                return analyze_rebound(p)
            with patch('rebound_analysis.analyze_rebound',side_effect=analyze):
                w._start_analysis(path)
                self.assertFalse(w.analyze_button.isEnabled())
                until=time.monotonic()+10
                while w._analysis_busy and time.monotonic()<until:
                    app.processEvents();time.sleep(.01)
            self.assertFalse(w._analysis_busy)
            self.assertEqual(caller,['rebound-analysis'])
            self.assertTrue(w.open_analysis_button.isEnabled())
            self.assertIn('已分析 1 个条件',w.analysis_label.text())
            w.shutdown();w.close()


if __name__=='__main__': unittest.main()
