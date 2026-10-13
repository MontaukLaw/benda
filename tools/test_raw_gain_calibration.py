"""验证逐次空载、20通道归一化、独立验证和采集失败边界。"""
from dataclasses import replace
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from raw_gain_calibration import CalibrationSettings, RawGainCalibration


class Stream:
    def __init__(self):
        self.model=RawGainCalibration(CalibrationSettings(repeats=3,settle_seconds=.2,window_seconds=.3))
        self.seq=0;self.t=10.

    def feed(self, raw=None, **kwargs):
        self.seq+=1;self.t+=.01
        sample=dict(sequence=self.seq,sample_timestamp=self.t,has_raw=True,channel_count=20,raw=raw or [10000]*20)
        sample.update(kwargs)
        return self.model.feed(sample)

    def capture(self, channel, response, baseline=10000, mode='calibrate'):
        self.model.start_empty(channel,self.t,mode)
        for _ in range(60):self.feed([baseline]*20)
        assert self.model.phase=='await_load', self.model.message
        self.model.start_loaded(self.t)
        raw=[baseline]*20;raw[channel]+=response
        for _ in range(60):self.feed(raw)
        assert self.model.phase=='empty', self.model.message


class CalibrationTests(unittest.TestCase):
    def complete(self):
        s=Stream()
        for ch in range(20):
            for offset in (-10,0,10):s.capture(ch,1000+ch*100+offset,10000+ch*50)
        return s

    def test_twenty_channel_median_target_and_separate_validation(self):
        s=self.complete();r=s.model.results()
        self.assertTrue(r['ready']);self.assertEqual(r['target'],1950)
        for ch,gain in enumerate(r['gains']):self.assertAlmostEqual((1000+ch*100)*gain,1950,delta=.2)
        previous=[len(t) for t in s.model.trials]
        s.capture(19,2900,baseline=23000,mode='validate')
        r=s.model.results();self.assertEqual([len(t) for t in s.model.trials],previous)
        self.assertEqual(r['channels'][19]['validation_count'],1)
        self.assertAlmostEqual(r['channels'][19]['validation_error_percent'],0,delta=.01)
        self.assertEqual(s.model.trials[0][0]['baseline'][0],10000)
        self.assertEqual(s.model.validations[19][0]['baseline'][19],23000)

    def test_new_calibration_invalidates_old_validation(self):
        s=self.complete();s.capture(0,1000,mode='validate')
        self.assertEqual(s.model.results()['channels'][0]['validation_count'],1)
        s.capture(1,1100)
        self.assertEqual(s.model.results()['channels'][0]['validation_count'],0)
        self.assertEqual(len(s.model.validations[0]),1)

    def test_cv_rejects_unrepeatable_channel(self):
        s=Stream()
        for x in (1000,2000,3000):s.capture(0,x)
        self.assertFalse(s.model.results()['channels'][0]['qualified'])
        self.assertFalse(s.model.results()['ready'])

    def test_gap_stale_and_cancel_discard_pending_only(self):
        s=Stream();s.capture(0,1000)
        s.model.start_empty(1,s.t)
        s.feed(sequence=s.seq+4)
        self.assertEqual(s.model.phase,'empty');self.assertIn('不连续',s.model.message)
        self.assertEqual(len(s.model.trials[0]),1)
        s.model.start_empty(1,s.t)
        s.model.expire(s.t+1)
        self.assertEqual(s.model.phase,'empty');self.assertIn('中断',s.model.message)
        s.model.cancel();self.assertIsNone(s.model.baseline)

    def test_duplicate_frames_do_not_complete_window(self):
        s=Stream();s.model.start_empty(0,s.t)
        for _ in range(100):s.model.feed(dict(sequence=1,sample_timestamp=10.3,has_raw=True,channel_count=20,raw=[10000]*20))
        self.assertEqual(s.model.phase,'capture_empty')

    def test_unstable_empty_and_timeout(self):
        s=Stream();s.model.start_empty(0,s.t)
        for n in range(200):s.feed([10000+n*10]*20)
        self.assertEqual(s.model.phase,'capture_empty')
        s.model.expire(s.t+22);self.assertEqual(s.model.phase,'empty')

    def test_weak_negative_and_saturated_not_stored(self):
        for response in (20,-100,55300):
            s=Stream();s.capture(0,response)
            self.assertEqual(s.model.trials[0],[])
        s=Stream();s.model.raw_limits=(11000,)*20;s.capture(0,1000)
        self.assertEqual(s.model.trials[0],[])

    def test_baseline_expires_without_loaded_confirmation(self):
        s=Stream();s.model.start_empty(0,s.t)
        for _ in range(60):s.feed()
        with self.assertRaises(ValueError):s.model.start_loaded(s.t+31)
        self.assertIsNone(s.model.baseline)

    def test_hardware_invalidation_keeps_completed_data(self):
        s=self.complete();s.model.invalidate('三参数改变')
        self.assertEqual(len(s.model.trials[0]),3)
        self.assertFalse(s.model.results()['ready'])
        with self.assertRaises(ValueError):s.model.start_empty(0,s.t)

    def test_missing_raw_and_nonfinite_values_rejected(self):
        for kwargs in ({'has_raw':False},{'raw':[float('nan')]*20},{'channel_count':8},{'raw':[True]*20}):
            s=Stream();s.model.start_empty(0,s.t);s.feed(**kwargs)
            self.assertEqual(s.model.phase,'empty')

    def test_invalid_settings(self):
        for kwargs in ({'mass_g':float('nan')},{'repeats':2},{'target':-1},{'window_seconds':0}):
            with self.assertRaises(ValueError):CalibrationSettings(**kwargs)

    def test_undo_retains_raw_record_and_invalidates_validation(self):
        s=self.complete();s.capture(0,1000,mode='validate')
        s.model.undo_last(0,'calibrate')
        self.assertEqual(len(s.model.trials[0]),2)
        self.assertFalse(s.model.results()['ready'])
        self.assertEqual(s.model.discarded[0]['trial']['response'],1010)
        self.assertEqual(s.model.report()['discarded'][0]['trial']['capture_settings']['mass_g'],100)

    def test_history_round_trip_recomputes_results_and_resets_capture(self):
        s=self.complete();s.capture(0,1000,mode='validate');s.model.undo_last(19,'calibrate')
        report=s.model.report();report['results']={'ready':True,'gains':[99]*20}
        restored=RawGainCalibration.from_report(report)
        self.assertEqual(restored.results()['channels'][0]['count'],3)
        self.assertEqual(restored.results()['channels'][19]['count'],2)
        self.assertEqual(len(restored.discarded),1)
        self.assertFalse(restored.results()['ready'])
        self.assertEqual(restored.phase,'empty');self.assertIsNone(restored.baseline)
        self.assertEqual(restored.last_sequence,0);self.assertIsNone(restored.last_timestamp)
        restored.trials[0][0]['response']=1234
        self.assertEqual(report['trials'][0][0]['response'],990)

    def test_complete_history_has_preview_gains_but_requires_environment_check(self):
        s=self.complete();s.capture(0,1000,mode='validate');report=s.model.report()
        report['invalid_reason']='传感器连接已改变'
        restored=RawGainCalibration.from_report(report);r=restored.results()
        self.assertFalse(r['ready']);self.assertEqual(r['gains'],s.model.results()['gains'])
        self.assertEqual(r['channels'][0]['validation_count'],1)
        restored.invalid_reason='';self.assertTrue(restored.results()['ready'])

    def test_history_rejects_corrupt_or_inconsistent_raw_records(self):
        original=self.complete().model.report()
        for corrupt in ('response','raw','channel','nonfinite','validation','schema'):
            report=deepcopy(original)
            if corrupt=='response':report['trials'][0][0]['response']+=100
            elif corrupt=='raw':report['trials'][0][0]['baseline']=[10000]*19
            elif corrupt=='channel':report['trials'][0][0]['channel']=1
            elif corrupt=='nonfinite':report['trials'][0][0]['loaded_raw'][0]=float('nan')
            elif corrupt=='validation':report['validations'][0]=[deepcopy(report['trials'][0][0])]
            else:report['schema_version']=2
            with self.subTest(corrupt=corrupt),self.assertRaises(ValueError):RawGainCalibration.from_report(report)


if __name__=='__main__':unittest.main()
