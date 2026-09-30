"""手动回弹计时及Qt接线回归；所有电机接口均为替身，不连接硬件。"""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from manual_rebound_observation import ReboundObservation, sensor_values
from manual_rebound_widget import ManualReboundWidget
from force_press_control_widget import ForcePressControlWidget, TARGET_FORCES_N
from qt_compat import QtCore, QtWidgets, Signal


def sample(sequence, state=0, raw=1000, baseline=1000, age=0):
    return dict(sequence=sequence,has_raw=True,channel_count=8,age_seconds=age,
                raw=(raw,)*8,hardware_raw=(raw,)*8,
                baseline=(baseline,)*8,state=(state,)*8)


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.events=[]
        self.o=ReboundObservation(self.events.append)
        self.o.begin(10,2,sample(1))
        self.o.sample(10.02,sample(2,1,1100))
        self.o.pressure(10.03,2.6)
        self.o.arrived(10.2,2.5)
        self.o.release(11,sample(100,6,1100),2.4)

    def test_force_duration_and_release_reference(self):
        e=next(e for e in self.events if e['event']=='release')
        self.assertEqual(e['target_force_n'],2)
        self.assertEqual(e['release_force_n'],2.4)
        self.assertAlmostEqual(e['contact_duration_s'],.97)
        self.assertAlmostEqual(e['hold_duration_s'],.8)
        self.assertEqual(e['references'][4],1000)

    def test_idle_and_original_raw_are_independent(self):
        self.o.sample(11.1,sample(101,4,1060))
        self.o.sample(11.2,sample(102,8,1050))
        self.o.sample(11.3,sample(103,7,1040,1020))
        self.o.sample(11.4,sample(104,0,1040,1040))
        self.assertAlmostEqual(self.o.channels[4]['idle_s'],.4)
        self.assertIsNone(self.o.channels[4]['raw_s'])
        self.assertEqual(self.o.channels[4]['reference'],1000)
        self.assertEqual(self.o.phase,'released')
        for n in range(7):
            self.o.sample(11.5+n*.1,sample(105+n,0,1001,1040))
        self.assertAlmostEqual(self.o.channels[4]['raw_s'],.5)
        self.assertEqual(self.o.phase,'complete')

    def test_spike_and_duplicate_cannot_confirm_raw_return(self):
        self.o.sample(11.1,sample(101,4,1001))
        self.o.sample(11.2,sample(102,4,1020))
        self.o.sample(11.3,sample(103,4,1000))
        self.o.sample(11.9,sample(103,4,1000))
        self.assertIsNone(self.o.channels[4]['raw_s'])
        self.o.sample(11.95,sample(104,4,1000))
        self.assertTrue(self.o.gapped)
        self.assertIsNone(self.o.channels[4]['raw_s'])

    def test_disconnect_does_not_claim_idle_on_reconnect(self):
        self.o.sample(12.2,sample(101,0,1000))
        self.assertEqual(self.o.phase,'interrupted')
        self.assertIsNone(self.o.channels[4]['idle_s'])

    def test_stale_data_times_out_and_sequence_reset_interrupts(self):
        self.o.sample(11.1,sample(101,4,1060))
        self.o.sample(12.2,sample(101,0,1000,age=2))
        self.assertEqual(self.o.phase,'interrupted')
        self.setUp()
        self.o.sample(11.1,sample(1,0,1000))
        self.assertEqual(self.o.phase,'interrupted')

    def test_missing_debug_and_missing_channels(self):
        s=sample(101);s['has_raw']=False
        self.o.sample(11.1,s)
        self.assertEqual(self.o.phase,'interrupted')
        s=sample(1);s['hardware_raw']=(1,)*5
        self.assertIsNone(sensor_values(s))

    def test_per_channel_recovery_and_nonfloating_release(self):
        self.o.phase='holding'
        s=sample(101,6,1100)
        s['state']=(0,0,0,0,1,6,6,6)
        self.o.release(11.1,s,2.4)
        self.assertTrue(any(e['event']=='non_floating_release' for e in self.events))
        s=sample(102,7,1040)
        s['state']=(0,0,0,0,0,7,7,7)
        self.o.sample(11.2,s)
        self.assertAlmostEqual(self.o.channels[4]['idle_s'],.1)
        self.assertIsNone(self.o.channels[5]['idle_s'])
        self.assertEqual(next(e for e in self.events if e['event']=='idle')['channel'],4)

    def test_next_press_preserves_censored_results_and_does_not_mutate_sample(self):
        s=sample(101,0,1050,1050);original=copy.deepcopy(s)
        self.o.begin(12,4,s)
        self.assertEqual(s,original)
        previous=next(e for e in self.events if e['event']=='interrupted')
        self.assertIsNone(previous['results'][4]['raw_s'])
        self.assertEqual(self.o.trial,2)
        self.assertEqual(self.o.channels[4]['reference'],1050)


class Controller(QtCore.QObject):
    manual_observation_event=Signal(object)
    status_message=Signal(str)
    def __init__(self):
        super().__init__();self.calls=[];self.accept=True
    def manual_control_state(self):
        return dict(targets={v:v<22 for v in TARGET_FORCES_N},release=True,stop=True,lookup='测试查表',status='已拒绝')
    def _ocr_snapshot(self):
        return dict(accepted_force_n=0,accepted_age_seconds=0,accepted_at=time.monotonic())
    def start_press(self,value):
        self.calls.append(('press',value));return self.accept
    def start_release(self):
        self.calls.append(('release',));return self.accept
    def request_stop(self):
        self.calls.append(('stop',))


class WidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_commands_start_only_on_ack_release_log_and_persistence(self):
        with tempfile.TemporaryDirectory() as directory:
            c=Controller();data=sample(1)
            w=ManualReboundWidget(c,lambda:data,output_dir=directory)
            w.timer.stop()
            try:
                self.assertFalse(w.target_buttons[22].isEnabled())
                w.target_buttons[2].click()
                self.assertEqual(c.calls,[('press',2)])
                self.assertEqual(w.observation.phase,'idle')
                at=time.monotonic()
                c.manual_observation_event.emit(dict(event='motion_dispatched',at=at,mode='press',operation_id=1,sensor=data))
                self.assertEqual(w.observation.phase,'pressing')
                c.manual_observation_event.emit(dict(event='operation_complete',at=at+.1,mode='press',operation_id=1,result=dict(ok=True,final_force_n=2.6)))
                w._tick()
                w.release_button.click()
                self.assertEqual(c.calls[-1],('release',))
                c.manual_observation_event.emit(dict(event='motion_dispatched',at=at+.2,mode='release',operation_id=2,sensor=sample(2,6,1100),pressure=dict(accepted_force_n=2.5,accepted_age_seconds=0)))
                self.assertEqual(w.observation.phase,'released')
                self.assertIn('2.500 N',w.log_edit.toPlainText())
                self.assertIn('FLOATING',w.log_edit.toPlainText())
                entries=[json.loads(line) for line in w.log_path.read_text(encoding='utf-8').splitlines()]
                self.assertTrue(any(e['event']=='release' for e in entries))
                w._stop_motor()
                self.assertEqual(c.calls[-1],('stop',))
                self.assertEqual(w.observation.phase,'interrupted')
            finally:
                w.shutdown();w.close()

    def test_rejected_press_never_starts_clock(self):
        with tempfile.TemporaryDirectory() as directory:
            c=Controller();c.accept=False
            w=ManualReboundWidget(c,lambda:sample(1),output_dir=directory);w.timer.stop()
            try:
                w.target_buttons[2].click()
                self.assertEqual(w.observation.phase,'idle')
                self.assertIsNone(w._armed)
                self.assertEqual(w.status_label.text(),'已拒绝')
            finally:
                w.shutdown();w.close()

    def test_button_availability_survives_collapsed_original_parent(self):
        group=QtWidgets.QGroupBox();group.setCheckable(True);group.setChecked(False)
        content=QtWidgets.QWidget(group)
        buttons=[QtWidgets.QPushButton(content) for _ in TARGET_FORCES_N]
        for n,b in enumerate(buttons):b.setEnabled(n<14)
        release=QtWidgets.QPushButton(content);release.setEnabled(True)
        class Original:pass
        original=Original();original._target_buttons=buttons;original.release_button=release
        original.stop_button=release;original.lookup_map_label=QtWidgets.QLabel('map');original._last_status_message='ok'
        state=ForcePressControlWidget.manual_control_state(original)
        self.assertTrue(state['targets'][2]);self.assertFalse(state['targets'][30]);self.assertTrue(state['release'])
        group.close()


if __name__=='__main__':
    unittest.main(verbosity=2)
