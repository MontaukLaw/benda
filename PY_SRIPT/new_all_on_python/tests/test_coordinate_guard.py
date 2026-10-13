"""验证暂停坐标与真实触摸判定独立，以及轻触、换向、长按和异常恢复。"""
import csv
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from new_all_on_python.config import AppConfig
from new_all_on_python.touch_tracker import TouchTracker
from new_all_on_python.processing_engine import ProcessingEngine


class CoordinateGuardTests(unittest.TestCase):
    def tracker(self, **changes):
        cfg=AppConfig(baseline_update_rate=0, position_filter_enable=False,
                      roi_mode="All Sensors", touch_on_threshold=600, touch_off_threshold=350,
                      dx_deadzone=0,dy_deadzone=0)
        for k,v in changes.items():setattr(cfg,k,v)
        tracker=TouchTracker(cfg)
        tracker.processor.baseline=[12000.0]*20
        tracker.processor.calibrating=False
        return tracker

    def raw(self, **values):
        result=[12000]*20
        for key,value in values.items():result[int(key[2:])]+=value
        return result

    def fade(self, tracker, start=0):
        for n in range(10):first=tracker.update(self.raw(CH0=5000,CH1=3000),start+n*.01)
        result=tracker.update(self.raw(CH0=1200,CH1=800),start+.1)
        self.assertTrue(result.position_held)
        self.assertEqual((result.x,result.y),(first.x,first.y))
        self.assertNotEqual(result.y_raw,result.y)
        self.assertEqual((result.dx,result.dy,result.state),(0,0,"MOVING"))
        return result

    def test_decay_holds_output_but_preserves_raw_baseline_signal_and_touch(self):
        guarded=self.tracker();original=self.tracker(decay_guard_enable=False)
        samples=[self.raw(CH0=5000,CH1=3000)]*10+[self.raw(CH0=1200,CH1=800),self.raw(CH0=400,CH1=600)]+[[12000]*20]
        for i,raw in enumerate(samples):
            a,b=guarded.update(raw,i*.01),original.update(raw,i*.01)
            for field in ('raw','adjusted_raw','baseline','signal','max_index','max_signal','touched','state','x_raw','y_raw','roi_weights','active_indices'):
                self.assertEqual(getattr(a,field),getattr(b,field),field)
        self.assertEqual(a.state,"TOUCH_UP")
        self.assertFalse(a.position_held)
        self.assertEqual(a.position_confidence,"inactive")

    def test_steady_hold_and_lower_pressure_hold_preserve_contact(self):
        tracker=self.tracker()
        for n in range(300):
            r=tracker.update(self.raw(CH8=3000,CH12=3500,CH9=1700),n*.01)
            self.assertFalse(r.position_held)
            self.assertTrue(r.touched)
        point=(r.x,r.y)
        for n in range(300,600):
            r=tracker.update(self.raw(CH8=900,CH12=1050,CH9=510),n*.01)
            self.assertTrue(r.touched)
            self.assertEqual((r.x,r.y),point)

    def test_falling_signal_continuing_forward_is_not_stopped(self):
        tracker=self.tracker()
        r=tracker.update(self.raw(CH0=6000,CH4=1000),0)
        for stamp,values in [(.01,dict(CH0=5000,CH4=2000)),(.02,dict(CH0=2000,CH4=1000)),(.03,dict(CH0=1000,CH4=800))]:
            previous=r.x;r=tracker.update(self.raw(**values),stamp)
            self.assertFalse(r.position_held)
            self.assertGreater(r.x,previous)

    def test_real_reverse_and_vertical_turn_with_new_response_are_followed(self):
        tracker=self.tracker()
        for n,values in enumerate([dict(CH0=5000),dict(CH0=1000,CH4=5000),dict(CH4=300,CH5=1800),dict(CH5=300,CH1=1500),dict(CH1=300,CH0=1200)]):
            r=tracker.update(self.raw(**values),n*.01)
            self.assertFalse(r.position_held)
        self.assertLess(r.y,.1)

    def test_weak_contact_can_resume_after_confirmed_growth(self):
        tracker=self.tracker();self.fade(tracker)
        r=tracker.update(self.raw(CH0=400,CH1=400),.11)
        self.assertTrue(r.position_held)
        for stamp in [.12,.13]:
            r=tracker.update(self.raw(CH0=400,CH1=400,CH4=500),stamp)
            self.assertTrue(r.position_held)
            self.assertEqual(r.position_confidence,"recovering")
        r=tracker.update(self.raw(CH0=400,CH1=400,CH4=500),.14)
        self.assertFalse(r.position_held)
        self.assertEqual(r.state,"MOVING")
        self.assertGreater(r.x,0)

    def test_one_frame_growth_cannot_resume_and_recovery_uses_elapsed_time(self):
        tracker=self.tracker();self.fade(tracker)
        r=tracker.update(self.raw(CH0=1200,CH1=800,CH4=500),.11)
        self.assertEqual(r.position_confidence,"recovering")
        for i in range(1,20):
            r=tracker.update(self.raw(CH0=1200,CH1=800,CH4=500),.11+i*.000001)
            self.assertTrue(r.position_held)
        r=tracker.update(self.raw(CH0=1200,CH1=800),.12)
        self.assertEqual(r.position_confidence,"decay_hold")
        r=tracker.update(self.raw(CH0=1200,CH1=800,CH4=500),.13)
        self.assertTrue(r.position_held)
        r=tracker.update(self.raw(CH0=1200,CH1=800,CH4=500),.15)
        self.assertFalse(r.position_held)

    def test_release_gap_reset_toggle_and_gain_change_clear_guard(self):
        for action in ('release','gap','reset','disable','gain','baseline'):
            with self.subTest(action=action):
                tracker=self.tracker();self.fade(tracker)
                stamp=.11
                if action=='release':tracker.update([12000]*20,stamp);stamp=.12
                elif action=='gap':stamp=.40
                elif action=='reset':tracker.reset_tracker()
                elif action=='baseline':
                    tracker.reset_baseline()
                    self.assertFalse(tracker._coordinate_guard.held)
                    continue
                else:
                    cfg=tracker.config.copy()
                    if action=='disable':cfg.decay_guard_enable=False
                    else:cfg.raw_gains[0]=1.1
                    tracker.set_config(cfg)
                    if action=='gain':
                        self.assertFalse(tracker._coordinate_guard.held)
                        self.assertTrue(tracker.processor.calibrating)
                        continue
                r=tracker.update(self.raw(CH19=3000),stamp)
                self.assertFalse(r.position_held)
                if action in ('release','gap','reset'):self.assertEqual((r.state,r.dx,r.dy),("TOUCH_DOWN",0,0))

    def test_legacy_config_validation_and_csv_diagnostics(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.json';path.write_text('{"position_alpha":0.3}',encoding='utf-8')
            cfg=AppConfig.load(path);self.assertTrue(cfg.decay_guard_enable)
            cfg.decay_guard_window_ms=55;cfg.decay_guard_drop_ratio=.4;cfg.save(path)
            self.assertEqual(AppConfig.load(path),cfg)
            for key,value in [('decay_guard_window_ms',0),('decay_guard_drop_ratio',float('nan')),('decay_guard_enable',1)]:
                bad=cfg.copy();setattr(bad,key,value)
                with self.assertRaises(ValueError):bad.validate()
            engine=ProcessingEngine(self.tracker().config)
            engine.tracker.processor.baseline=[12000.0]*20;engine.tracker.processor.calibrating=False
            engine.recorder.start(engine.tracker.config,0,folder)
            frame={'protocol':'DATA','state':tuple(range(20))}
            try:
                for n in range(10):engine.update(self.raw(CH0=5000,CH1=3000),n*.01,frame)
                r=engine.update(self.raw(CH0=1200,CH1=800),.1,frame)
                self.assertTrue(r.position_held)
            finally:self.assertTrue(engine.recorder.finish())
            with engine.recorder.path.open(encoding='utf-8-sig',newline='') as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(rows[-1]['position_held'],'True')
            self.assertEqual(rows[-1]['position_confidence'],'decay_hold')
            self.assertEqual(rows[-1]['firmware_state19'],'19')
            self.assertEqual(rows[-1]['signal0'],'1200.0')
            self.assertEqual(rows[-1]['touch_state'],'MOVING')
            engine.source='Serial';engine.last_arrival=.1
            result=engine.snapshot(.5)['result']
            self.assertFalse(result.position_held)
            self.assertEqual(result.position_confidence,'inactive')

    def test_ui_toggle_persistence_and_pure_numeric_inputs(self):
        from PySide6 import QtWidgets
        from new_all_on_python.main_window import MainWindow
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as folder,patch('new_all_on_python.main_window.CONFIG_PATH',Path(folder)/'cfg.json'):
            window=MainWindow(auto_connect=False)
            try:
                self.assertTrue(window.controls['decay_guard_enable'].isChecked())
                for key in ['decay_guard_window_ms','decay_guard_drop_ratio']:
                    self.assertEqual(window.controls[key].prefix(),'');self.assertEqual(window.controls[key].suffix(),'')
                window.controls['decay_guard_enable'].setChecked(False)
                self.assertFalse(window.engine.tracker.config.decay_guard_enable)
                window.save_config()
                self.assertFalse(AppConfig.load(Path(folder)/'cfg.json').decay_guard_enable)
            finally:window.close();app.processEvents()
