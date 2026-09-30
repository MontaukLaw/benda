"""验证按钮点击前历史窗口、固定CSV入口、缺失字段及写盘失败保护。"""
import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from data_recorder import SerialDataRecorder


class RecentExportTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.recorder=SerialDataRecorder(self.directory.name)
        self.now=0.0
        self.clock=patch('data_recorder.time.perf_counter',side_effect=lambda:self.now)
        self.wall=patch('data_recorder.time.time',side_effect=lambda:1700000000+self.now)
        self.clock.start();self.wall.start()
        self.addCleanup(self.clock.stop);self.addCleanup(self.wall.stop)

    def add(self, now, kind='debug'):
        self.now=now
        frame={'type':kind,'device':1,'channel_count':8,'signal':[0]*8}
        if kind=='debug':
            frame.update(raw=[now]*8,baseline=[100]*8,state=[0]*8)
        self.recorder.record(frame)

    def read(self,result):
        with Path(result['output_path']).open(encoding='utf-8-sig',newline='') as f:
            return list(csv.DictReader(f))

    def test_all_three_exact_windows_without_manual_start(self):
        for i in range(701):self.add(i)
        for minutes in (1,3,5):
            snapshot=self.recorder.snapshot_recent(minutes)
            result=self.recorder.write_recent_snapshot(snapshot)
            rows=self.read(result)
            self.assertEqual(len(rows),minutes*60+1)
            self.assertEqual(float(rows[0]['raw_0']),700-minutes*60)
            self.assertEqual(float(rows[-1]['raw_7']),700)
            self.assertEqual(float(rows[0]['elapsed_s']),0)
            self.assertFalse(result['partial_history'])
            self.assertEqual(Path(result['output_path']).name,f'recent_{minutes}min.csv')
            self.assertEqual(Path(result['output_path']).read_bytes(),Path(result['archive_path']).read_bytes())
            meta=json.loads(Path(result['output_path']).with_suffix('.meta.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['csv_sha256'],hashlib.sha256(Path(result['output_path']).read_bytes()).hexdigest())

    def test_click_snapshot_does_not_include_later_data_or_mutation(self):
        self.add(1)
        snapshot=self.recorder.snapshot_recent(1)
        self.add(2)
        rows=self.read(self.recorder.write_recent_snapshot(snapshot))
        self.assertEqual(len(rows),1)
        self.assertEqual(float(rows[0]['raw_0']),1)

    def test_partial_history_and_missing_debug_values(self):
        self.add(10)
        self.add(11,'signal')
        self.now=12
        self.recorder.record({'type':'slow_baseline','device':1,'channel_count':8,'slow_baseline':[123.25]*8})
        result=self.recorder.write_recent_snapshot(self.recorder.snapshot_recent(5))
        rows=self.read(result)
        self.assertTrue(result['partial_history'])
        self.assertEqual(result['sensor_sample_count'],2)
        self.assertEqual(rows[1]['raw_0'],'')
        self.assertEqual(rows[1]['signal_0'],'0')
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[2]['slow_baseline_7'], '123.25')
        self.assertEqual(result['frame_type_counts']['slow_baseline'], 1)

    def test_repeat_export_keeps_both_archives(self):
        self.add(1)
        a=self.recorder.write_recent_snapshot(self.recorder.snapshot_recent(1))
        self.add(2)
        b=self.recorder.write_recent_snapshot(self.recorder.snapshot_recent(1))
        self.assertNotEqual(a['archive_path'],b['archive_path'])
        self.assertEqual(len(self.read(b)),2)
        self.assertTrue(Path(a['archive_path']).exists())

    def test_failed_export_and_empty_window_preserve_previous_csv(self):
        self.add(1)
        saved=self.recorder.write_recent_snapshot(self.recorder.snapshot_recent(1))
        original=Path(saved['output_path']).read_bytes()
        self.add(2)
        with patch('data_recorder.shutil.copyfile',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.recorder.write_recent_snapshot(self.recorder.snapshot_recent(1))
        self.assertEqual(Path(saved['output_path']).read_bytes(),original)
        self.now=400
        with self.assertRaises(ValueError):self.recorder.snapshot_recent(1)
        self.assertEqual(Path(saved['output_path']).read_bytes(),original)
        self.assertFalse(list(Path(self.directory.name).glob('*.tmp')))

    def test_memory_cap_is_bounded_and_reports_shortened_history(self):
        self.recorder.HISTORY_MAX_FRAMES=10
        for i in range(101):self.add(i)
        self.assertEqual(len(self.recorder._history),10)
        snapshot=self.recorder.snapshot_recent(1)
        self.assertTrue(snapshot['metadata']['partial_history'])
        self.assertLess(snapshot['metadata']['available_seconds'],60)
        self.recorder.HISTORY_MAX_BYTES=100
        self.add(102)
        self.assertLessEqual(self.recorder._history_bytes,100)

    def test_disconnected_recent_data_can_still_be_exported(self):
        self.add(100)
        self.now=110
        self.assertTrue(self.recorder.has_recent_data)
        result=self.recorder.write_recent_snapshot(self.recorder.snapshot_recent(1))
        self.assertEqual(result['seconds_since_last_sensor'],10)
        self.now=401
        self.assertFalse(self.recorder.has_recent_data)


if __name__=='__main__':unittest.main(verbosity=2)
