"""固定窗口回弹计划、两小时虚拟运行、数据缺口及安全边界回归。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import json
import math
import sqlite3
import sys
import tempfile
import threading
import time
import unittest
from contextlib import closing
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'PY_SRIPT/algo_monitor'))
from rebound_test_protocol import REBOUND_PROTOCOL, build_rebound_plan, SINGLE_EXP_TEMPLATE, SHORT_PRESS_TEMPLATE
from rebound_test_protocol import PAIRED_PRESS_TEMPLATE, REPEAT_SHORT_2N_TEMPLATE
from force_press_control_widget import ForcePressControlWidget as Controller, ForceControlError
from long_duration_creep_test_widget import LongDurationCreepTestWidget as Widget, estimate_creep_plan
from qt_compat import QtWidgets, QtGui

CONFIG = dict(protocol=REBOUND_PROTOCOL, high_forces_n=[1,2,10], high_hold_seconds=[2,10,30],
              recovery_seconds=[360], repeat_count=2, baseline_wait_seconds=30, inter_trial_seconds=0)


class Clock:
    def __init__(self): self.now=100.; self.stopped=False
    def is_set(self): return self.stopped
    def wait(self, sec): self.now+=sec; return self.stopped


class VirtualController:
    _run_creep_test=Controller._run_creep_test
    _run_creep_trial=Controller._run_creep_trial
    _run_rebound_trial=Controller._run_rebound_trial
    _run_paired_rebound_trial=Controller._run_paired_rebound_trial
    _run_creep_wait=Controller._run_creep_wait
    _creep_force_parameters=staticmethod(Controller._creep_force_parameters)
    def __init__(self, clock):
        self.clock=clock; self._state_lock=threading.Lock(); self._creep_rebound_active=True
        self.presses=[]; self.releases=0; self.windows=[]; self.position=0
        self.manual_observation_event=Mock(); self.progresses=[]
        self._latest_sensor_sample=lambda: dict(has_raw=True, state=[6]*8, normalized_raw=[9000]*8, baseline=[6000]*8)
        self._ocr_snapshot=lambda: dict(accepted_force_n=.08)
    def _poll_checked(self,*args): return dict(position_counts=self.position, measured_speed_rpm=0)
    def _creep_progress(self, phase, trial, total, **kwargs):
        self.progresses.append((phase,dict(trial),kwargs))
        if kwargs.get('phase_remaining_seconds')==0: self.windows.append((phase, kwargs['phase_duration_seconds']))
    def _run_creep_press(self,motor,tracker,stop,params):
        self.presses.append(params['target_force_n']); self.position=-1000
        start=self.clock.now
        self.clock.wait(2)
        return dict(final_force_n=params['target_force_n']+.6, final_position_counts=-1000,
                    motion_started_monotonic=start, arrived_monotonic=self.clock.now)
    def _run_creep_release(self,*args):
        start=self.clock.now
        self.releases+=1; self.position=0; self.clock.wait(1)
        return dict(final_force_n=.08, release_command_monotonic=start,
                    release_zero_confirmed_monotonic=self.clock.now)


class ReboundControllerTests(unittest.TestCase):
    def run_observation(self, feedback, duration=2):
        clock=Clock(); h=VirtualController(clock)
        h._poll_checked=lambda *args: feedback(clock.now-100)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            h._run_creep_wait(None,{},clock,dict(max_current_ma=3000,release_position_counts=3),
                             dict(trial_index=3),18,'creep_recovery_wait',duration)
        return h

    def test_recorded_minus_two_rpm_with_four_counts_offset_continues(self):
        h=self.run_observation(lambda t:dict(position_counts=7,
            measured_speed_rpm=-2 if .3<t<.5 else 0))
        self.assertEqual(h.windows,[('creep_recovery_wait',2)])
        events=[call.args[0] for call in h.manual_observation_event.emit.call_args_list]
        self.assertEqual([e['status'] for e in events],['speed_only','normal'])
        self.assertEqual(events[0]['position_offset_counts'],4)

    def test_speed_alone_and_isolated_position_spikes_do_not_abort(self):
        for feedback in (
            lambda t:dict(position_counts=7,measured_speed_rpm=-2),
            lambda t:dict(position_counts=300 if .3<t<.5 or 1.3<t<1.5 else 7,measured_speed_rpm=0),
            lambda t:dict(position_counts=259,measured_speed_rpm=0),
        ):
            h=self.run_observation(feedback)
            self.assertEqual(h.windows,[('creep_recovery_wait',2)])

    def test_persistent_position_deviation_stops_even_without_speed(self):
        for position in (260,-254):
            clock=Clock(); h=VirtualController(clock)
            h._poll_checked=lambda *args:dict(position_counts=position,measured_speed_rpm=0)
            with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
                with self.assertRaises(ForceControlError) as caught:
                    h._run_creep_wait(None,{},clock,dict(max_current_ma=3000,release_position_counts=3),
                                     dict(trial_index=3),18,'creep_recovery_wait',360)
            self.assertEqual(caught.exception.reason,'creep_unexpected_motion')
            self.assertGreaterEqual(clock.now-100,.3)
            self.assertLess(clock.now-100,.6)
            self.assertIn(str(position),str(caught.exception))

    def test_repeated_feedback_timestamp_does_not_confirm_motion(self):
        h=self.run_observation(lambda t:dict(position_counts=300,measured_speed_rpm=0,sampled_monotonic=100))
        self.assertEqual(h.windows,[('creep_recovery_wait',2)])

    def test_resume_records_recovery_then_restarts_full_remaining_trials(self):
        clock=Clock(); h=VirtualController(clock)
        trials=build_rebound_plan(dict(CONFIG,random_seed=20260910))[2:]
        for trial in trials:
            trial['high_lookup']=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        params=dict(creep_trials=trials,sweep_plan=dict(CONFIG,resumed_from='previous_session'),
                    release_position_counts=0,max_current_ma=3000)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=h._run_creep_test(None,{},clock,params)
        self.assertEqual(h.windows[:3],[('creep_retry_recovery',360),('creep_pre_observation',30),('creep_high_hold',10)])
        self.assertEqual(len(h.presses),16)
        self.assertEqual(h.presses[0],2)
        self.assertEqual(sum(p=='creep_recovery_wait' and duration==360 for p,duration in h.windows),16)
        self.assertEqual([t['planned_trial_index'] for t in result['creep_completed_trials']],list(range(3,19)))
        self.assertTrue(h.progresses[0][1]['history_interrupted'])
        self.assertTrue(result['creep_completed_trials'][0]['history_interrupted'])

    def test_full_two_hour_plan_completes_despite_floating_and_raw_residual(self):
        clock=Clock(); h=VirtualController(clock)
        trials=build_rebound_plan(CONFIG)
        for trial in trials:
            trial['high_lookup']=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        params=dict(creep_trials=trials,sweep_plan=CONFIG,release_position_counts=0,max_current_ma=3000)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=h._run_creep_test(None,{},clock,params)
        self.assertTrue(result['ok'])
        self.assertEqual(len(h.presses),18)
        self.assertEqual(h.releases,19)
        self.assertEqual(sum(p=='creep_recovery_wait' and duration==360 for p,duration in h.windows),18)
        self.assertEqual(sum(p=='creep_pre_observation' and duration==30 for p,duration in h.windows),18)
        self.assertGreater(clock.now-100,7200)
        self.assertLess(clock.now-100,7500)
        self.assertTrue(all(r['fully_recovered'] is None for r in result['creep_completed_trials']))
        self.assertEqual([r['high_force_n'] for r in result['creep_completed_trials']],h.presses)
        self.assertTrue(all(r['end_sensor']['state']==[6]*8 for r in result['creep_completed_trials']))

    def test_arrival_begins_hold_without_force_stability_or_correction(self):
        h=SimpleNamespace(_poll_checked=Mock(return_value=dict(position_counts=0)),
            _ocr_snapshot=Mock(side_effect=[dict(accepted_force_n=.08,accepted_age_seconds=.01),dict(accepted_force_n=2.6)]),
            _move_relative=Mock(return_value=dict(position_counts=-1000)), _wait_for_stable_force=Mock())
        params=dict(max_current_ma=3000,lookup_target_position_counts=-1000,release_position_counts=0,
                    max_travel_counts=10000,press_speed_level=3,acceleration=10,target_force_n=2)
        r=Controller._run_rebound_press(h,None,{},Clock(),params)
        self.assertEqual(r['final_force_n'],2.6)
        h._wait_for_stable_force.assert_not_called()
        self.assertEqual(h._move_relative.call_count,1)
        self.assertEqual(h._move_relative.call_args.kwargs['zero_force_tolerance_n'],.1)

    def test_release_accepts_three_new_point_one_samples(self):
        clock=Clock(); counter=iter(range(4))
        h=SimpleNamespace(_run_release=Mock(return_value={}),
            _ocr_snapshot=lambda:dict(accepted_force_n=.1,accepted_age_seconds=.01,accepted_sequence=next(counter)),
            _poll_checked=Mock(return_value=dict(position_counts=0,measured_speed_rpm=0)))
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            r=Controller._run_creep_release(h,None,{},clock,dict(sweep_plan=CONFIG,max_current_ma=3000,release_position_counts=0))
        self.assertEqual(r['final_force_n'],.1)
        self.assertEqual(h._poll_checked.call_count,3)

    def test_actual_motion_dispatch_accepts_point_one_and_rejects_above(self):
        for force, accepted in ((.06,True),(.1,True),(.11,False),(-1,False)):
            with self.subTest(force=force):
                feedback=[dict(position_counts=0,measured_speed_rpm=0,running_state=2)]+[
                    dict(position_counts=-800,measured_speed_rpm=0,running_state=2)]*4
                motor=SimpleNamespace(clear_state=Mock(),move_relative_counts=Mock(return_value={}))
                h=SimpleNamespace(_poll_checked=Mock(side_effect=feedback),_set_motion_context=Mock(),
                    _record_event=Mock(),operation_progress=Mock(),manual_observation_event=Mock(),
                    _operation_id=1,_control_mode='creep_test',_target_force_n=2,
                    _ocr_snapshot=lambda:dict(accepted_force_n=force,accepted_age_seconds=0),
                    _latest_sensor_sample=lambda:None,_communication_recovery_generation=0,_creep_active=True,
                    _call_with_motor_communication_retry=lambda action,callback,*args,**kwargs:callback())
                def move():
                    return Controller._move_relative(h,motor,{},Clock(),-1000,3,10,3000,30.4,'press',1000,
                        require_zero_force_start=True,zero_force_tolerance_n=.1,position_tolerance_counts=256,
                        fixed_target_counts=-1000)
                if accepted:
                    self.assertEqual(move()['position_counts'],-800)
                    motor.move_relative_counts.assert_called_once()
                else:
                    with self.assertRaises(ForceControlError): move()
                    motor.move_relative_counts.assert_not_called()

    def make_poll(self, sample):
        feedback=dict(phase_current_ma=100,running_state=0,running_state_name='idle',position_counts=0,measured_speed_rpm=0)
        return SimpleNamespace(_creep_active=True,_creep_rebound_active=True,
            _latest_sensor_sample=lambda:None,_ocr_snapshot=lambda:sample,
            _read_feedback=lambda *a:feedback,
            _call_with_motor_communication_retry=lambda action,callback,*a,**kw:callback())

    def test_missing_raw_and_idle_pressure_gap_do_not_stop_observation(self):
        h=self.make_poll(dict(accepted_force_n=None,accepted_age_seconds=math.inf))
        Controller._poll_checked(h,SimpleNamespace(),{},Clock(),3000,30.4,'observe')

    def test_loaded_pressure_gap_grace_and_persistent_loss(self):
        h=self.make_poll(dict(accepted_force_n=2,accepted_age_seconds=1.5,accepted_sequence=1))
        Controller._poll_checked(h,SimpleNamespace(),{},Clock(),3000,30.4,'press')
        h._ocr_snapshot=lambda:dict(accepted_force_n=2,accepted_age_seconds=2.1)
        with self.assertRaises(ForceControlError) as caught:
            Controller._poll_checked(h,SimpleNamespace(),{},Clock(),3000,30.4,'press')
        self.assertEqual(caught.exception.reason,'pressure_meter_unavailable')

    def test_force_limit_still_stops_during_observation(self):
        import time
        h=self.make_poll(dict(accepted_force_n=31,accepted_age_seconds=0,raw_force_n=31,sampled_monotonic=time.monotonic()))
        with self.assertRaises(ForceControlError) as caught:
            Controller._poll_checked(h,SimpleNamespace(),{},Clock(),3000,30.4,'observe')
        self.assertEqual(caught.exception.reason,'force_limit')

    def test_fixed_wait_honors_user_stop(self):
        clock=Clock(); clock.stopped=True; h=VirtualController(clock)
        with self.assertRaises(ForceControlError):
            Controller._run_creep_wait(h,None,{},clock,dict(max_current_ma=3000,release_position_counts=0),
                                      dict(trial_index=1),18,'creep_recovery_wait',360)


class ReboundWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        if os.name=='nt':
            QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.load=patch.object(Widget,'_load_settings');self.load.start();self.addCleanup(self.load.stop)
        self.save=patch.object(Widget,'_save_settings');self.save.start();self.addCleanup(self.save.stop)
        self.analysis=patch.object(Widget,'_start_analysis');self.analysis.start();self.addCleanup(self.analysis.stop)
        self.sample=dict(sequence=1,has_raw=True,channel_count=8,age_seconds=0,
            normalized_raw=[1000]*8,hardware_raw=[1000]*8,baseline=[990]*8,signal=[0]*8,state=[6]*8)
        self.controller=SimpleNamespace(operation_progress=Mock(),operation_finished=Mock(),
            start_creep_test=Mock(return_value=True),request_stop=Mock())
        motor=SimpleNamespace(is_can_connected=lambda:True,latest_motor_feedback=lambda:{})
        self.widget=Widget(self.controller,motor,lambda:self.sample)
        self.addCleanup(self.close_widget)
    def close_widget(self):
        if self.widget._recorder is not None: self.widget._finish_session('stopped','test')
        self.widget.close()
    def start(self):
        self.widget.set_pressure_sample(dict(accepted=True,raw_force_n=.08))
        conf=self.widget._read_config();plan=build_rebound_plan(conf)
        with patch('long_duration_creep_test_widget.creep_test_directory',lambda:Path(self.temp.name)):
            self.widget._start_plan(conf,plan,estimate_creep_plan(plan,0))
        self.widget._capture_timer.stop();self.widget._runtime_timer.stop();self.widget._status_timer.stop()
        self.assertTrue(self.widget._running)
        return self.widget._recorder.session_dir
    def test_default_plan_and_estimate(self):
        conf=self.widget._read_config(); plan=build_rebound_plan(conf)
        self.assertEqual(conf['protocol'],REBOUND_PROTOCOL)
        self.assertEqual(len(plan),18)
        self.assertTrue(self.widget.probe_force_edit.isHidden())
        self.assertTrue(all(p['probe_force_n']==0 for p in plan))
        self.assertEqual(estimate_creep_plan(plan,0)['total_seconds'],7416)
        self.widget._config=conf
        self.widget._phase_context={'phase':'creep_recovery_wait'}
        self.assertEqual(self.widget._capture_interval(350),.1)
        self.assertEqual(estimate_creep_plan(plan,0)['estimated_rows'],74160)

    def test_paired_plan_full_five_rounds_timing_and_complete_pair_resume(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('paired_press'))
        config=self.widget._read_config(); trials=build_rebound_plan(config)
        self.assertEqual(config['template_id'], PAIRED_PRESS_TEMPLATE)
        self.assertEqual(config['record_interval_ms'], 100)
        self.assertEqual(len(trials),10)
        self.assertFalse(self.widget.probe_force_edit.isHidden())
        self.assertFalse(self.widget.inter_trial_input.isHidden())
        self.assertEqual([t['recovery_seconds'] for t in trials], [5,60]*5)
        self.assertEqual([t['repeat_index'] for t in trials], [1,1,2,2,3,3,4,4,5,5])
        self.assertEqual(estimate_creep_plan(trials,180)['total_seconds'],2175)
        self.assertEqual(estimate_creep_plan(trials,180)['estimated_rows'],21750)
        lookup=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        for trial in trials: trial.update(high_lookup=lookup,probe_lookup=lookup)
        clock=Clock();h=VirtualController(clock)
        params=dict(creep_trials=trials,sweep_plan=config,release_position_counts=0,max_current_ma=3000)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=h._run_creep_test(None,{},clock,params)
        self.assertTrue(result['ok'])
        self.assertEqual(h.presses,[20,2,20,2]*5)
        self.assertEqual(h.releases,21)
        self.assertEqual(h.windows.count(('creep_high_hold',3)),10)
        self.assertEqual(h.windows.count(('creep_probe_hold',3)),10)
        self.assertEqual(h.windows.count(('creep_inter_trial_wait',180)),9)
        self.assertEqual(h.windows.count(('creep_recovery_wait',5)),5)
        self.assertEqual(h.windows.count(('creep_recovery_wait',60)),5)
        self.assertAlmostEqual(clock.now-100,2076,places=5)
        for t in result['creep_completed_trials']:
            a=t['paired_results']['high']; b=t['paired_results']['probe']
            self.assertAlmostEqual(b['press_result']['motion_started_monotonic']-a['release_result']['release_zero_confirmed_monotonic'],t['recovery_seconds'])
            for part in (a,b):
                self.assertAlmostEqual(part['release_result']['release_command_monotonic']-part['press_result']['arrived_monotonic'],3)
        # 已完成过程保留；补测第三个过程从20 N开始，重新等待5秒后再施加2 N。
        clock=Clock();h=VirtualController(clock)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            resumed=h._run_creep_test(None,{},clock,dict(params,creep_trials=trials[2:],
                sweep_plan=dict(config,resumed_from='prior')))
        self.assertEqual(h.presses,[20,2]*8)
        self.assertEqual(h.windows[0],('creep_retry_recovery',360))
        self.assertEqual(resumed['creep_completed_trials'][0]['planned_trial_index'],3)
        self.assertTrue(resumed['creep_completed_trials'][0]['history_interrupted'])
        manifest=json.loads((self.start()/'manifest.json').read_text(encoding='utf8'))
        self.assertEqual(manifest['config']['template_id'],PAIRED_PRESS_TEMPLATE)
        self.assertEqual(len(manifest['trials']),10)

    def test_paired_template_persistence_and_invalid_edit_cannot_change_sequence_silently(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('paired_press'))
        self.load.stop();self.save.stop()
        with patch('long_duration_creep_test_widget.creep_test_settings_path',return_value=Path(self.temp.name)/'settings.json'):
            self.widget._save_settings()
            w=Widget(self.controller,SimpleNamespace(),lambda:self.sample)
            try:
                self.assertEqual(w.preset_combo.currentData(),'paired_press')
                self.assertEqual(w._read_config()['template_id'],PAIRED_PRESS_TEMPLATE)
                w.split_mib_input.setValue(256)
                self.assertEqual(len(build_rebound_plan(w._read_config())),10)
                w.recovery_edit.setText('6,60')
                self.assertFalse(w.start_button.isEnabled())
                with self.assertRaises(ValueError): build_rebound_plan(w._read_config())
                w.preset_combo.setCurrentIndex(w.preset_combo.findData('short_press'))
                self.assertEqual(len(build_rebound_plan(w._read_config())),15)
            finally:
                w.shutdown();w.close()

    def test_paired_stop_at_small_press_never_commits_half_a_process(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('paired_press'))
        config=self.widget._read_config(); trial=build_rebound_plan(config)[0]
        lookup=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        trial.update(high_lookup=lookup,probe_lookup=lookup)
        clock=Clock();h=VirtualController(clock);press=h._run_creep_press
        h._creep_emergency_release=Mock()
        def stop_on_small(motor,tracker,stop,params):
            if params['target_force_n']==2:
                clock.stopped=True
                raise ForceControlError('user_stop','用户停止')
            return press(motor,tracker,stop,params)
        h._run_creep_press=stop_on_small
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            with self.assertRaises(ForceControlError):
                h._run_creep_test(None,{},clock,dict(creep_trials=[trial],sweep_plan=config,
                    release_position_counts=0,max_current_ma=3000))
        self.assertEqual(h.presses,[20])
        h._creep_emergency_release.assert_called_once()
        self.assertFalse(any(phase=='creep_trial_complete' for phase,_,_ in h.progresses))

    def test_single_exponential_template_runs_all_nine_trials_and_preserves_fit_roles(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('single_exp'))
        config=self.widget._read_config();trials=build_rebound_plan(config)
        self.assertEqual(config['template_id'],SINGLE_EXP_TEMPLATE)
        self.assertEqual(config['calibration_goal']['parameters_per_channel'],['k','tau_seconds'])
        self.assertEqual(len(trials),9)
        self.assertEqual(estimate_creep_plan(trials,0)['total_seconds'],6228)
        self.assertEqual([t['analysis_role'] for t in trials],['fit']*6+['validation']*3)
        for repeat in range(1,4):
            block=[t for t in trials if t['repeat_index']==repeat]
            self.assertEqual(sorted(t['high_hold_seconds'] for t in block),[2,10,60])
        for trial in trials:
            trial['high_lookup']=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        clock=Clock();h=VirtualController(clock)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=h._run_creep_test(None,{},clock,dict(creep_trials=trials,sweep_plan=config,release_position_counts=0,max_current_ma=3000))
        self.assertTrue(result['ok'])
        self.assertEqual(h.presses,[2]*9)
        for hold in (2,10,60): self.assertEqual(h.windows.count(('creep_high_hold',hold)),3)
        self.assertEqual(h.windows.count(('creep_recovery_wait',600)),9)
        self.assertEqual(h.windows.count(('creep_pre_observation',60)),9)
        session=self.start()
        manifest=json.loads((session/'manifest.json').read_text(encoding='utf8'))
        self.assertEqual(manifest['config']['template_id'],SINGLE_EXP_TEMPLATE)
        self.assertEqual([t['analysis_role'] for t in manifest['trials']],['fit']*6+['validation']*3)
        # 续测使用剩余原计划：末轮的检验角色不会被重新编号成拟合。
        self.assertEqual([t['analysis_role'] for t in manifest['trials'][5:]],['fit']+['validation']*3)

    def test_short_press_runs_fifteen_presses_without_extra_wait_or_state_gate(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('short_press'))
        config=self.widget._read_config();trials=build_rebound_plan(config)
        self.assertEqual(config['template_id'],SHORT_PRESS_TEMPLATE)
        self.assertEqual(len(trials),15)
        self.assertEqual(config['record_interval_ms'],100)
        self.assertEqual([t['pre_observation_seconds'] for t in trials],[10]+[0]*14)
        self.assertEqual(estimate_creep_plan(trials,0)['total_seconds'],1075)
        for repeat in range(1,6):
            self.assertEqual(sorted(t['high_force_n'] for t in trials if t['repeat_index']==repeat),[2,10,20])
        for trial in trials:
            trial['high_lookup']=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        clock=Clock();h=VirtualController(clock)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=h._run_creep_test(None,{},clock,dict(creep_trials=trials,sweep_plan=config,release_position_counts=0,max_current_ma=3000))
        self.assertTrue(result['ok'])
        self.assertEqual(len(h.presses),15)
        self.assertEqual(h.windows.count(('creep_high_hold',3)),15)
        self.assertEqual(h.windows.count(('creep_recovery_wait',60)),15)
        self.assertEqual(sum(duration for phase,duration in h.windows if phase=='creep_pre_observation'),10)
        self.assertTrue(all(r['end_sensor']['state']==[6]*8 for r in result['creep_completed_trials']))
        manifest=json.loads((self.start()/'manifest.json').read_text(encoding='utf8'))
        self.assertEqual(manifest['config']['template_id'],SHORT_PRESS_TEMPLATE)
        self.assertEqual(manifest['trials'][1]['reference_policy'],'session_last_10s_before_press')

    def test_repeat_short_2n_completes_ten_rounds_including_final_recovery(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('repeat_short_2n'))
        config=self.widget._read_config();trials=build_rebound_plan(config)
        self.assertEqual(config['template_id'],REPEAT_SHORT_2N_TEMPLATE)
        self.assertEqual(config['record_interval_ms'],100)
        self.assertEqual([t['repeat_index'] for t in trials],list(range(1,11)))
        self.assertEqual([t['pre_observation_seconds'] for t in trials],[10]+[0]*9)
        self.assertEqual(estimate_creep_plan(trials,0)['total_seconds'],720)
        for trial in trials:
            trial['high_lookup']=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        clock=Clock();h=VirtualController(clock)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=h._run_creep_test(None,{},clock,dict(creep_trials=trials,sweep_plan=config,release_position_counts=0,max_current_ma=3000))
        self.assertTrue(result['ok'])
        self.assertEqual(h.presses,[2]*10)
        self.assertEqual(h.windows.count(('creep_high_hold',3)),10)
        self.assertEqual(h.windows.count(('creep_recovery_wait',60)),10)
        self.assertEqual(h.windows[-1],('creep_recovery_wait',60))
        self.assertEqual(sum(duration for phase,duration in h.windows if phase=='creep_pre_observation'),10)
        completed=result['creep_completed_trials']
        for index, trial in enumerate(completed):
            self.assertAlmostEqual(trial['release_result']['release_command_monotonic']-trial['press_result']['arrived_monotonic'],3)
            if index:
                self.assertAlmostEqual(trial['press_result']['motion_started_monotonic']-completed[index-1]['release_result']['release_zero_confirmed_monotonic'],60)
        self.assertAlmostEqual(clock.now-completed[-1]['release_result']['release_zero_confirmed_monotonic'],60)
        manifest=json.loads((self.start()/'manifest.json').read_text(encoding='utf8'))
        self.assertEqual(manifest['config']['template_id'],REPEAT_SHORT_2N_TEMPLATE)
        self.assertEqual(len(manifest['trials']),10)

    def test_repeat_short_2n_reload_and_custom_edit(self):
        self._check_short_press_reload('repeat_short_2n')

    def test_short_press_reload_and_custom_edit_do_not_keep_hidden_timing_rule(self):
        self._check_short_press_reload('short_press')

    def _check_short_press_reload(self, preset):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData(preset))
        self.load.stop();self.save.stop()
        with patch('long_duration_creep_test_widget.creep_test_settings_path',return_value=Path(self.temp.name)/'settings.json'):
            self.widget._save_settings()
            w=Widget(self.controller,SimpleNamespace(),lambda:self.sample)
            try:
                self.assertEqual(w.preset_combo.currentData(),preset)
                self.assertEqual(w._parameter_rows[6][0].text(),'整组开始前先记录（秒）')
                config=w._read_config()
                with self.assertRaises(ValueError): build_rebound_plan(dict(config,repeat_count=4))
                w.repeat_input.setValue(4)
                self.assertNotIn('template_id',w._read_config())
                self.assertEqual(w._parameter_rows[6][0].text(),'每次按压前先记录（秒）')
                self.assertTrue(all(t['pre_observation_seconds']==10 for t in build_rebound_plan(w._read_config())))
            finally:
                w.shutdown();w.close()

    def test_single_exponential_template_survives_settings_reload_and_edits_clear_role(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('single_exp'))
        self.load.stop();self.save.stop()
        saved=Path(self.temp.name)/'settings.json'
        with patch('long_duration_creep_test_widget.creep_test_settings_path',return_value=saved):
            self.widget._save_settings()
            w=Widget(self.controller,SimpleNamespace(),lambda:self.sample)
            try:
                self.assertEqual(w.preset_combo.currentData(),'single_exp')
                self.assertEqual(w._read_config()['template_id'],SINGLE_EXP_TEMPLATE)
                w.high_hold_edit.setText('3,10,60')
                self.assertEqual(w.preset_combo.currentData(),'custom')
                self.assertNotIn('template_id',w._read_config())
                self.assertTrue(all('analysis_role' not in t for t in build_rebound_plan(w._read_config())))
            finally:
                w.shutdown();w.close()

    def test_fixed_repeat_preset_runs_five_complete_ten_minute_observations(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('repeat_2n'))
        config=self.widget._read_config();trials=build_rebound_plan(config)
        self.assertEqual(config['protocol'],REBOUND_PROTOCOL)
        self.assertEqual(len(trials),5)
        self.assertEqual(estimate_creep_plan(trials,0)['total_seconds'],3240)
        self.assertIn('54分',self.widget.plan_label.text())
        self.assertIn('实际压力可能',self.widget.experiment_description.text())
        self.assertTrue(self.widget.probe_force_edit.isHidden())
        for trial in trials:
            self.assertEqual((trial['high_force_n'],trial['high_hold_seconds'],trial['recovery_seconds']), (2,10,600))
            trial['high_lookup']=dict(press_travel_counts=1000,target_position_counts=-1000,position_span_counts=0,created_at='test')
        clock=Clock();h=VirtualController(clock)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=h._run_creep_test(None,{},clock,dict(creep_trials=trials,sweep_plan=config,release_position_counts=0,max_current_ma=3000))
        self.assertTrue(result['ok'])
        self.assertEqual(h.presses,[2]*5)
        self.assertEqual(h.windows.count(('creep_high_hold',10)),5)
        self.assertEqual(h.windows.count(('creep_recovery_wait',600)),5)
        self.assertEqual(h.windows.count(('creep_pre_observation',30)),5)
        self.widget.repeat_input.setValue(4)
        self.assertEqual(self.widget.preset_combo.currentData(),'custom')
        self.assertEqual(self.widget._read_config()['protocol'],REBOUND_PROTOCOL)

    def test_resume_keeps_completed_data_and_restarts_third_item_in_new_session(self):
        old_session=self.start()
        for index in (1,2):
            self.widget._on_force_progress(dict(phase='creep_trial_start',trial_index=index,category='raw_rebound'))
            self.widget._on_force_progress(dict(phase='creep_trial_complete',trial_index=index,trial_result=dict(trial_index=index)))
        self.widget._on_force_progress(dict(phase='creep_trial_failed',trial_index=3,reason='creep_unexpected_motion'))
        self.widget._finish_session('stopped','test interrupted')
        old_manifest=(old_session/'manifest.json').read_bytes()
        with patch('long_duration_creep_test_widget.creep_test_directory',lambda:Path(self.temp.name)), \
                patch.object(Widget,'_confirm_start',return_value=True):
            self.widget.resume_last_test()
        self.widget._capture_timer.stop();self.widget._runtime_timer.stop();self.widget._status_timer.stop()
        self.assertTrue(self.widget._running)
        plan=self.controller.start_creep_test.call_args.args[0]
        self.assertEqual(plan['resumed_from'],str(old_session))
        self.assertEqual([t['planned_trial_index'] for t in plan['trials']],list(range(3,19)))
        self.assertEqual(plan['trials'][0]['high_force_n'],2)
        self.assertEqual(plan['trials'][0]['high_hold_seconds'],10)
        self.assertEqual(plan['trials'][0]['recovery_seconds'],360)
        self.assertNotEqual(old_session,self.widget._recorder.session_dir)
        self.assertEqual((old_session/'manifest.json').read_bytes(),old_manifest)
        self.assertEqual(self.widget._manifest['resume_context']['time_axis_policy'],'new_segment_do_not_stitch')
        self.assertEqual(self.widget._manifest['resume_context']['gap_force_history'],'unknown')

    def test_rebound_layout_and_countdown(self):
        self.widget._config=self.widget._read_config()
        self.widget._plan=build_rebound_plan(self.widget._config)
        self.widget._manifest={'completed_trials':3}
        self.widget._running=True
        self.widget._on_force_progress(dict(phase='creep_recovery_wait',category='raw_rebound',
            trial_index=4,trial_total=18,repeat_index=1,high_force_n=2,high_hold_seconds=10,
            recovery_seconds=360,phase_duration_seconds=360,phase_elapsed_seconds=50))
        self.widget.setStyleSheet("QWidget{font-family:'Microsoft YaHei';font-size:14px;background:#101923;color:#e4eff5;}")
        self.widget.resize(1050,920);self.widget.show()
        for _ in range(5): self.app.processEvents()
        self.assertIn('第5/5步',self.widget.stage_label.text())
        self.assertIn('18',self.widget.round_label.text())
        self.assertNotIn('探针',self.widget.condition_label.text())
        for label in (self.widget.stage_label,self.widget.experiment_description,self.widget.countdown_label):
            self.assertGreaterEqual(label.height(),label.heightForWidth(label.width()))
        if os.environ.get('REBOUND_PAGE_SCREENSHOT'):
            p=Path(os.environ['REBOUND_PAGE_SCREENSHOT']);p.parent.mkdir(parents=True,exist_ok=True)
            self.widget.grab().save(str(p))
        self.widget._running=False
    def test_raw_gap_continues_and_pressure_and_configuration_are_saved(self):
        session=self.start()
        self.widget._on_force_progress(dict(phase='creep_trial_start',trial_index=1,category='raw_rebound'))
        self.sample['age_seconds']=5
        self.widget._capture_latest_sample()
        self.controller.request_stop.assert_not_called()
        self.widget.set_pressure_sample(dict(accepted=True,raw_force_n=.06,sequence=4))
        self.widget.record_device_configuration('capsense_channels',dict(Nsub=300,TxDivider=100,RefCDAC=12))
        self.sample['age_seconds']=0;self.sample['sequence']=2
        self.widget._capture_latest_sample();self.widget._capture_latest_sample()
        self.widget._on_force_progress(dict(phase='creep_trial_complete',trial_index=1,trial_result=dict(trial_index=1)))
        self.widget._finish_session('completed','test')
        self.widget._start_analysis.assert_called_once_with(session)
        with closing(sqlite3.connect(next(session.glob('raw_data_part_*.sqlite3')))) as db:
            events=db.execute('SELECT event,payload_json FROM events').fetchall()
            names=[e[0] for e in events]
            self.assertIn('raw_gap_start',names);self.assertIn('raw_gap_end',names)
            self.assertIn('pressure_sample',names);self.assertIn('device_configuration',names)
            self.assertEqual(db.execute('SELECT COUNT(*) FROM samples').fetchone()[0],1)
            complete=json.loads(next(p for e,p in events if e=='trial_complete'))
            self.assertIn('raw_gap',complete['data_quality_issues'])

    def test_100ms_direct_recording_preserves_latest_values_state_events_and_pressure_validity(self):
        self.widget.preset_combo.setCurrentIndex(self.widget.preset_combo.findData('repeat_short_2n'))
        session=self.start()
        self.assertEqual(self.widget._manifest['config']['record_interval_ms'],100)
        base=time.monotonic()+1
        with patch('long_duration_creep_test_widget.time.monotonic') as now:
            for i in range(101):
                now.return_value=base+i*.01
                self.sample['sequence']=i+1
                self.sample['hardware_raw']=[2000+i+c for c in range(8)]
                self.sample['die_temperature_celsius']=25+i/100
                self.sample['die_temperature_age_seconds']=.2
                self.sample['state']=[5 if i==5 else 6]*8
                self.widget.set_pressure_sample(dict(accepted=True,raw_force_n=i/100,sequence=i+1,sampled_monotonic=now.return_value))
                self.widget._capture_latest_sample()
                self.assertEqual(self.widget._pressure_snapshot()['sequence'],i+1)
            now.return_value=base+1.01
            self.widget.set_pressure_sample(dict(accepted=False,raw_force_n=None,sequence=102))
            now.return_value=base+1.02
            self.widget.set_pressure_sample(dict(accepted=True,raw_force_n=.5,sequence=103))
        self.widget._finish_session('stopped','test')
        with closing(sqlite3.connect(next(session.glob('raw_data_part_*.sqlite3')))) as db:
            values=db.execute('SELECT ch4_hardware_raw,capture_interval_ms FROM samples ORDER BY id').fetchall()
            recorded=db.execute('SELECT die_temperature_celsius,die_temperature_age_seconds,pressure_raw_n,'+
                                ','.join(f'ch{c}_hardware_raw' for c in range(8))+' FROM samples ORDER BY id').fetchall()
            self.assertEqual(recorded,[(25+i/100,.2,i/100,*[2000+i+c for c in range(8)]) for i in range(0,101,10)])
            self.assertEqual([v[0] for v in values],list(range(2004,2105,10)))
            for raw,interval in values[1:]: self.assertAlmostEqual(interval,100,places=5)
            events=db.execute('SELECT event,payload_json FROM events').fetchall()
            states=[json.loads(p) for e,p in events if e=='sensor_state_change']
            self.assertEqual([p['states'][4] for p in states],[5,6])
            pressures=[json.loads(p) for e,p in events if e=='pressure_sample']
            self.assertEqual(len(pressures),13)
            self.assertEqual([p['sequence'] for p in pressures[:11]],list(range(1,102,10)))
            self.assertEqual([p['accepted'] for p in pressures[-2:]],[False,True])


if __name__=='__main__': unittest.main()
