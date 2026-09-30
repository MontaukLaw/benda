"""自动重复测试的卸载间隔、停止与原有保护；使用虚拟时钟，不连接硬件。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from force_press_control_widget import ForcePressControlWidget as Controller, ForceControlError


class Clock:
    def __init__(self): self.now=100.; self.stop_at=float('inf')
    def is_set(self): return self.now>=self.stop_at
    def wait(self, seconds): self.now+=seconds; return self.is_set()


def harness(clock):
    h=SimpleNamespace(_state_lock=threading.Lock(),operation_progress=Mock(),force_mapping_captured=Mock(),
        _record_event=Mock(),_ocr_snapshot=lambda:dict(accepted_force_n=.08,accepted_age_seconds=0),
        _sensor_force_values=lambda sample:dict(signal_sum=400,floating_count=4),
        _raw_median_log_values=lambda window:{},_signal_sum_statistics=lambda results:{},
        _raw_median_statistics=lambda results:{},_format_signal_sum_statistics=lambda stats:'',
        _format_raw_median_statistics=lambda stats:'',_latest_sensor_sample=Mock())
    h.presses=[];h.releases=[]
    def press(*args):
        h.presses.append((clock.now,args[-1]['target_force_n']));clock.wait(.5)
        return dict(final_force_n=2.3)
    def release(*args):
        assert args[-1]['wait_for_sensor_idle_after_release'] is False
        clock.wait(.1);h.releases.append(clock.now);return dict(final_force_n=.08)
    def capture(*args):
        clock.wait(1)
        return dict(sample={},raw_medians=[1000]*4,raw_median_sum=4000,
                    raw_median_sample_count=10,raw_median_window_seconds=1)
    def poll(*args):
        if clock.is_set(): raise ForceControlError('user_stop','停止')
        return {}
    h._run_press=press;h._run_release=release;h._wait_for_all_floating_stable=capture;h._poll_checked=poll
    h._wait_auto_repeat_interval=lambda *args:Controller._wait_auto_repeat_interval(h,*args)
    return h


class IntervalTests(unittest.TestCase):
    def run_plan(self, seconds, rounds=10):
        clock=Clock();h=harness(clock)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            result=Controller._run_auto_test(h,None,{},clock,dict(target_force_n=2,auto_rounds=rounds,
                auto_test_interval_seconds=seconds,max_current_ma=2800,operation_id=1,
                wait_for_sensor_idle_after_release=True))
        return clock,h,result

    def test_ten_presses_nine_intervals_no_final_wait_and_no_state_gate(self):
        clock,h,result=self.run_plan(5)
        self.assertTrue(result['ok'])
        self.assertEqual([force for _,force in h.presses],[2]*10)
        self.assertEqual(len(h.releases),10)
        for n in range(9):self.assertAlmostEqual(h.presses[n+1][0]-h.releases[n],5,places=6)
        self.assertAlmostEqual(clock.now,h.releases[-1])
        h._latest_sensor_sample.assert_not_called()
        events=[call.args[0] for call in h._record_event.call_args_list]
        self.assertEqual(events.count('auto_interval_started'),9)
        self.assertEqual(events.count('auto_interval_completed'),9)

    def test_zero_preserves_immediate_repeat_and_single_press_does_not_wait(self):
        for seconds,rounds in ((0,10),(60,1)):
            clock,h,result=self.run_plan(seconds,rounds)
            self.assertAlmostEqual(clock.now,100+rounds*1.6)
            self.assertFalse(any(c.args[0]=='auto_interval_started' for c in h._record_event.call_args_list))

    def test_fractional_interval(self):
        clock,h,result=self.run_plan(.15,3)
        self.assertAlmostEqual(h.presses[1][0]-h.releases[0],.15)

    def test_timer_begins_only_after_fresh_near_zero_pressure(self):
        clock=Clock();h=harness(clock)
        h._ocr_snapshot=lambda:dict(accepted_force_n=.3 if clock.now<100.3 else .08,accepted_age_seconds=0)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            h._wait_auto_repeat_interval(None,{},clock,dict(max_current_ma=2800),1,1,2)
        self.assertGreaterEqual(clock.now,101.3)
        self.assertLess(clock.now,101.4)

    def test_pressure_not_zero_has_bounded_wait(self):
        clock=Clock();h=harness(clock);h._ocr_snapshot=lambda:dict(accepted_force_n=.2,accepted_age_seconds=0)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            with self.assertRaises(ForceControlError) as caught:
                h._wait_auto_repeat_interval(None,{},clock,dict(max_current_ma=2800),1,1,2)
        self.assertEqual(caught.exception.reason,'auto_release_not_zero')
        self.assertLess(clock.now,110.1)

    def test_user_stop_prevents_next_press(self):
        clock=Clock();clock.stop_at=102;h=harness(clock)
        with patch('force_press_control_widget.time.monotonic',lambda:clock.now):
            with self.assertRaises(ForceControlError) as caught:
                Controller._run_auto_test(h,None,{},clock,dict(target_force_n=2,auto_rounds=10,
                    auto_test_interval_seconds=60,max_current_ma=2800,operation_id=1))
        self.assertEqual(caught.exception.reason,'user_stop')
        self.assertEqual(len(h.presses),1)
        self.assertLess(clock.now,102.1)

    def test_existing_safety_fault_propagates_without_next_press(self):
        clock=Clock();h=harness(clock)
        h._poll_checked=Mock(side_effect=ForceControlError('current_limit','过流'))
        with self.assertRaises(ForceControlError) as caught:
            Controller._run_auto_test(h,None,{},clock,dict(target_force_n=2,auto_rounds=2,
                auto_test_interval_seconds=1,max_current_ma=2800,operation_id=1))
        self.assertEqual(caught.exception.reason,'current_limit')
        self.assertEqual(len(h.presses),1)

    def test_invalid_interval_rejected_before_motion(self):
        for value in (-1,3601,float('nan'),float('inf')):
            clock=Clock();h=harness(clock)
            with self.assertRaises(ForceControlError):
                Controller._run_auto_test(h,None,{},clock,dict(target_force_n=2,auto_rounds=2,
                    auto_test_interval_seconds=value))
            self.assertEqual(h.presses,[])


if __name__=='__main__':unittest.main()
