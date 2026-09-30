"""运动日志降采样、关键事件和共享CSV入口回归；不连接设备。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import csv
import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from motion_log_sampling import MotionLogSampling
from force_press_control_widget import ForcePressControlWidget as Widget, ForcePressCsvRecorder


def sample(seq, states=None, age=0):
    return dict(sequence=seq,age_seconds=age,state=list(states or [6]*8),channel_count=8,
                has_raw=True,normalized_raw=[1000+seq]*8,hardware_raw=[1000+seq]*8,
                baseline=[900]*8,signal=[100]*8)


class SamplingTests(unittest.TestCase):
    def call(self,p,t,seq=None,states=None,event='sensor_sample',busy=False,age=0,feedback=None,pressure=None):
        return p.select(event,sample(int(t*1000) if seq is None else seq,states,age),
            feedback or {},pressure or {},busy,t)

    def test_three_periodic_sources_share_100ms_then_one_second_budget(self):
        p=MotionLogSampling();saved=[]
        for i in range(12001):
            for event in ('pressure_meter_sample','motor_sample','sensor_sample'):
                r=self.call(p,i/100,event=event,states=[0]*8)
                if r:saved.append((i/100,r))
        fast=[t for t,r in saved if t<60];slow=[t for t,r in saved if t>=60]
        self.assertEqual(len(fast),600)
        self.assertEqual(len(slow),60)
        self.assertTrue(all(round(b-a,4)>=1 for a,b in zip(slow,slow[1:])))

    def test_each_of_eight_channels_wakes_immediately_even_between_saved_samples(self):
        for channel in range(8):
            p=MotionLogSampling()
            for i in range(6201):self.call(p,i/100,states=[0]*8)
            states=[0]*8;states[channel]=1
            r=self.call(p,62.001,states=states)
            self.assertIn('sensor_state_changed',r['sensor_record_reason'])
            self.assertEqual(r['sensor_record_interval_ms'],100)
            self.assertEqual(json.loads(r['sensor_previous_states_json']),[0]*8)
            self.assertEqual(json.loads(r['sensor_states_json'])[channel],1)
            self.assertIsNone(self.call(p,62.05,states=states))
            self.assertIsNotNone(self.call(p,62.102,states=states))

    def test_motion_and_tests_never_downgrade_even_with_constant_floating(self):
        for state in range(1,9):
            nonidle=MotionLogSampling()
            for i in range(6201):
                r=self.call(nonidle,i/100,states=[0]*7+[state])
                if r:self.assertEqual(r['sensor_record_interval_ms'],100)
        p=MotionLogSampling()
        for i in range(6101):self.call(p,i/100,states=[0]*8)
        r=self.call(p,61.001,busy=True)
        self.assertEqual(r['sensor_record_interval_ms'],100)
        for i in range(6102,12501):
            r=self.call(p,i/100,busy=True)
            if r:self.assertEqual(r['sensor_record_interval_ms'],100)
        r=self.call(p,125.001,feedback={'position_counts':100,'measured_speed_rpm':5})
        # 普通周期记录可等到下一个100ms点，但运动不会进入慢记录。
        self.assertGreaterEqual(p.stable_since,125)

    def test_gaps_missing_states_and_reconnect_do_not_count_as_stability(self):
        p=MotionLogSampling()
        self.call(p,0)
        r=self.call(p,100)
        self.assertEqual(r['sensor_record_interval_ms'],100)
        r=self.call(p,100.01,age=5)
        self.assertIn('sensor_stream_changed',r['sensor_record_reason'])
        self.assertFalse(r['sensor_snapshot_fresh'])
        r=self.call(p,100.02,seq=1)
        self.assertEqual(r['sensor_record_interval_ms'],100)
        for i in range(10100,17200):
            r=self.call(p,i/100,states=[6]*5)
            if r:self.assertEqual(r['sensor_record_interval_ms'],100)

    def test_repeated_sensor_frame_not_written_and_events_bypass_throttle(self):
        p=MotionLogSampling()
        self.call(p,0,seq=1)
        self.assertIsNone(self.call(p,.3,seq=1))
        r=self.call(p,.301,seq=1,event='operation_failed')
        self.assertEqual(r['sensor_record_reason'],'event')
        r=self.call(p,.302,seq=1,pressure={'accepted':True})
        self.assertIn('pressure_validity_changed',r['sensor_record_reason'])

    def test_shared_widget_path_writes_direct_values_and_state_event_to_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=ForcePressCsvRecorder(directory);path=recorder.start()
            h=SimpleNamespace(_recorder=recorder,_log_sampling=MotionLogSampling(),_log_sampling_lock=threading.Lock(),
                _last_recorded_sensor_sequence=None,_running=False,
                _motor_control=SimpleNamespace(latest_motor_feedback=lambda:{},is_operation_busy=lambda:False),
                _operation_context=lambda:{},_latest_sensor_sample=lambda:None,
                _ocr_snapshot=lambda:dict(accepted=True,accepted_age_seconds=0),
                _sensor_log_values=Widget._sensor_log_values)
            h._record_sampled_event=lambda event,**values:Widget._record_sampled_event(h,event,**values)
            try:
                with patch('force_press_control_widget.time.monotonic') as clock:
                    for i in range(31):
                        clock.return_value=i/100
                        for event in ('sensor_sample','pressure_meter_sample','motor_sample'):
                            Widget._record_event(h,event,_sensor_sample=sample(i))
                    clock.return_value=.301
                    states=[6]*8;states[0]=1
                    Widget._record_event(h,'motor_sample',_sensor_sample=sample(31,states))
                    clock.return_value=.302
                    Widget._record_event(h,'operation_failed',_sensor_sample=sample(31,states),message='test failure')
            finally:recorder.stop()
            with path.open(encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
            self.assertEqual([int(r['ch4_hardware_raw']) for r in rows[:4]],[1000,1010,1020,1030])
            self.assertEqual(len(rows),6)
            self.assertEqual(rows[4]['event'],'sensor_state_change')
            self.assertEqual(json.loads(rows[4]['sensor_states_json'])[0],1)
            self.assertEqual(rows[5]['event'],'operation_failed')


if __name__=='__main__':unittest.main()
