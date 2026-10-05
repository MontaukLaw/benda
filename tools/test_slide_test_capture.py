"""验证试验归档边界、全量字段、校验、异常和滚动日志互不影响。"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'PY_SRIPT/algo_monitor'))
from rolling_serial_log import RollingSerialLog
from slide_test_capture import SlideTestCapture


def wait_for(predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('Timed out')
        time.sleep(.01)


class SlideCaptureTests(unittest.TestCase):
    def test_exact_boundaries_all_streams_and_checksum(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(Path(directory) / 'rolling', flush_seconds=60)
            log.submit('rx', data=b'before')
            context = dict(protocol=dict(horizontal_count=1, vertical_count=1), raw_gains=[2.] * 20)
            capture = log.start_capture(Path(directory) / 'tests', context)
            for sequence, axis in enumerate(('horizontal', 'vertical'), 99):
                log.submit('slide_test', frame=dict(event='phase_begin', phase='slide', axis=axis, trial=1))
                log.submit('rx', data=b'\x00DATA\xff')
                log.submit('frame', frame=dict(type='debug', raw=list(range(20)), baseline=[5]*20,
                                               signal=[10]*20, state=[1]*20, fast_k_percent=[20]*20))
                log.submit('frame', frame=dict(type='slow_baseline', values=[3]*20))
                log.submit('host_state_machine', frame=dict(sequence=sequence, input_val=[10]*20, baseline=[5]*20,
                                                            signal=[5]*20, state=[1]*20))
                log.submit('touchpad', frame=dict(source_sample_sequence=sequence, dx=3, dy=4))
                log.submit('tx', data=b'command')
            log.finish_capture()
            log.submit('rx', data=b'after')
            wait_for(lambda: capture.status()['done'])
            self.assertFalse(capture.status()['error'])
            with zipfile.ZipFile(capture.status()['archive']) as archive:
                raw = archive.read('records.jsonl')
                rows = [json.loads(line) for line in raw.splitlines()]
                meta = json.loads(archive.read('manifest.json'))
            self.assertEqual(meta['outcome'], 'completed')
            self.assertEqual(meta['records_sha256'], hashlib.sha256(raw).hexdigest())
            self.assertEqual(meta['counts']['sensor_samples'], 2)
            self.assertEqual(meta['counts']['frame'], 4)
            self.assertEqual(rows[0]['frame']['event'], 'start')
            self.assertEqual(rows[-1]['frame']['event'], 'finish')
            self.assertEqual([r['hex'] for r in rows if r['kind'] == 'rx'], [b'\x00DATA\xff'.hex()] * 2)
            self.assertEqual([r['sequence'] for r in rows], list(range(1, 17)))
            self.assertEqual(len(meta['phases']), 2)
            log.request_close()
            self.assertTrue(log.wait(3))
            normal = [json.loads(line) for p in log.directory.glob('*.jsonl') for line in p.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(normal[0]['hex'], b'before'.hex())
            self.assertEqual(normal[-1]['hex'], b'after'.hex())

    def test_close_preserves_partial_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(Path(directory)/'rolling')
            capture = log.start_capture(Path(directory)/'tests', dict(protocol=dict(horizontal_count=5, vertical_count=5)))
            log.submit('frame', frame=dict(type='debug', raw=[100]*20))
            log.request_close()
            self.assertTrue(log.wait(5))
            with zipfile.ZipFile(capture.status()['archive']) as archive:
                meta = json.loads(archive.read('manifest.json'))
            self.assertEqual(meta['outcome'], 'aborted')
            self.assertEqual(meta['counts']['sensor_samples'], 1)

    def test_overflow_still_finalizes_and_marks_gaps(self):
        with tempfile.TemporaryDirectory() as directory:
            entered, release = threading.Event(), threading.Event()
            original = SlideTestCapture.write
            def slow(capture, encoded, record):
                if record['kind'] == 'slide_test' and record['frame'].get('event') == 'start':
                    entered.set()
                    release.wait(3)
                original(capture, encoded, record)
            log = RollingSerialLog(Path(directory)/'rolling', queue_limit=1)
            with patch.object(SlideTestCapture, 'write', slow):
                capture = log.start_capture(Path(directory)/'tests', dict(protocol=dict(horizontal_count=1, vertical_count=1)))
                self.assertTrue(entered.wait(2))
                for _ in range(10):
                    log.submit('rx', data=b'x')
                log.finish_capture()
                release.set()
                wait_for(lambda: capture.status()['done'])
            with zipfile.ZipFile(capture.status()['archive']) as archive:
                meta = json.loads(archive.read('manifest.json'))
            self.assertEqual(meta['outcome'], 'completed_with_gaps')
            self.assertEqual(meta['dropped_records'], 9)
            log.request_close()
            self.assertTrue(log.wait(3))

    def test_zip_failure_keeps_source_data_and_normal_logger(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(Path(directory)/'rolling')
            capture = log.start_capture(Path(directory)/'tests', dict(protocol=dict(horizontal_count=1, vertical_count=1)))
            log.submit('frame', frame=dict(type='debug', raw=[100]*20))
            with patch('slide_test_capture.zipfile.ZipFile', side_effect=OSError('disk full')):
                log.finish_capture('aborted', '用户停止')
                wait_for(lambda: capture.status()['done'])
            self.assertIn('disk full', capture.status()['error'])
            self.assertIn('100', capture.path.read_text(encoding='utf-8'))
            self.assertTrue((capture.directory/'manifest.json').exists())
            log.submit('rx', data=b'logger survives')
            log.request_close()
            self.assertTrue(log.wait(3))
            self.assertFalse(log.status()['error'])

    def test_slow_archive_keeps_logging_and_close_waits(self):
        with tempfile.TemporaryDirectory() as directory:
            entered, release = threading.Event(), threading.Event()
            original = SlideTestCapture.finish
            def slow(capture, *args):
                entered.set()
                release.wait(3)
                original(capture, *args)
            log = RollingSerialLog(Path(directory)/'rolling', flush_seconds=.02)
            with patch.object(SlideTestCapture, 'finish', slow):
                capture = log.start_capture(Path(directory)/'tests', dict(protocol=dict(horizontal_count=1, vertical_count=1)))
                log.finish_capture('aborted', '用户停止')
                self.assertTrue(entered.wait(2))
                log.submit('rx', data=b'continues while archiving')
                wait_for(lambda: log.status()['written'] == 3)
                log.request_close()
                self.assertFalse(log.wait(.05))
                self.assertTrue(log.is_alive())
                release.set()
                self.assertTrue(log.wait(3))
            self.assertTrue(capture.status()['done'])

    def test_rolling_disk_failure_still_seals_partial_test(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(Path(directory)/'rolling', flush_seconds=.02, batch_bytes=1)
            log._write = lambda batch: (_ for _ in ()).throw(OSError('rolling disk full'))
            capture = log.start_capture(Path(directory)/'tests', dict(protocol=dict(horizontal_count=1, vertical_count=1)))
            wait_for(lambda: bool(log.status()['error']))
            log.submit('frame', frame=dict(type='debug', raw=[100]*20))
            log.finish_capture()
            log.request_close()
            self.assertTrue(log.wait(5))
            self.assertTrue(capture.status()['done'])
            with zipfile.ZipFile(capture.status()['archive']) as archive:
                meta = json.loads(archive.read('manifest.json'))
            self.assertEqual(meta['outcome'], 'aborted')
            self.assertIn('rolling disk full', meta['reason'])
            self.assertGreater(meta['dropped_records'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
