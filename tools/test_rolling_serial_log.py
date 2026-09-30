"""验证后台批量写入、全字段、轮转、拥塞和磁盘故障；不打开真实串口。"""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'PY_SRIPT/algo_monitor'))
from rolling_serial_log import RollingSerialLog


def rows(directory):
    return [json.loads(line) for path in sorted(Path(directory).glob('roll_*.jsonl'))
            for line in path.read_text(encoding='utf-8').splitlines()]


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('Timed out')
        time.sleep(.01)


class RollingLogTests(unittest.TestCase):
    def test_serial_reader_logs_raw_unknown_data_and_all_decoded_fields(self):
        import struct
        from serial_reader import SerialReaderThread
        from data_buffer import AlgoDataBuffer
        from data_recorder import SerialDataRecorder
        from qt_compat import QtCore
        app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])
        payload = (struct.pack('<20H', *range(100,120)) + struct.pack('<20H', *([90]*20))
                   + struct.pack('<20H', *([10]*20)) + struct.pack('<20i', *([200]*20))
                   + struct.pack('<20i', *([300]*20)) + struct.pack('<20H', *([6]*20)))
        wire = b'unknown startup bytes' + struct.pack('<4sBBBBH', b'DATA',21,0x10,1,20,len(payload)) + payload
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(Path(directory)/'rolling')
            recorder = SerialDataRecorder(Path(directory)/'snapshots')
            reader = SerialReaderThread('FAKE', AlgoDataBuffer(100), data_recorder=recorder, rolling_log=log)
            class FakeSerial:
                is_open = True
                in_waiting = len(wire)
                sent = []
                def write(self, data): self.sent.append(data); return len(data)
                def flush(self): pass
                def read(self, count):
                    reader._running = False
                    return wire
                def close(self): self.is_open = False
            serial_port = FakeSerial()
            with patch('serial.Serial', return_value=serial_port):
                reader.start(); self.assertTrue(reader.wait(3000))
            log.request_close(); self.assertTrue(log.wait(3))
            records = rows(log.directory)
            rx = b''.join(bytes.fromhex(r['hex']) for r in records if r['kind']=='rx')
            tx = b''.join(bytes.fromhex(r['hex']) for r in records if r['kind']=='tx')
            self.assertEqual(rx, wire)
            self.assertEqual(tx, b''.join(serial_port.sent))
            frames = [r['frame'] for r in records if r['kind']=='frame']
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0]['raw'][19], 119)
            self.assertEqual(frames[0]['baseline'][19], 90)
            self.assertEqual(frames[0]['signal'][19], 10)
            self.assertEqual(frames[0]['state'][19], 6)
            self.assertEqual(records[0]['kind'], 'connected')
            self.assertEqual(records[-1]['kind'], 'disconnected')

    def test_complete_frames_buffered_and_closed_off_thread(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(directory, flush_seconds=60)
            writer_threads = []
            original = log._write
            def write(batch):
                writer_threads.append(threading.current_thread().name)
                original(batch)
            log._write = write
            frame = dict(type='debug', raw=list(range(20)), baseline=[100]*20,
                         signal=[5]*20, states=[6]*20, noise=[.5]*20)
            log.submit('rx', port='COM4', data=b'\xff\x00DATA')
            log.submit('frame', port='COM4', frame=frame)
            log.submit('frame', port='COM4', frame=dict(type='slow_baseline', values=[10]*20))
            log.submit('tx', port='COM4', data=b'hello')
            centroid = dict(type='centroid', source_sample_sequence=17, sample_timestamp=1.25,
                            raw_xy=[3.5,2.75], filtered_xy=[2.7,2.1], displayed_xy=[2.7,2.1],
                            signal=[5]*20, smoothing_enabled=True,
                            filter=dict(algorithm='one_euro', min_cutoff_hz=1.5, beta=.25))
            log.submit('centroid', port='COM4', frame=centroid)
            wait_for(lambda: log.status()['pending'] == 5)
            self.assertFalse(list(Path(directory).iterdir()))
            log.request_close()
            self.assertTrue(log.wait(3))
            records = rows(directory)
            self.assertEqual(records[1]['frame'], frame)
            self.assertEqual(bytes.fromhex(records[0]['hex']), b'\xff\x00DATA')
            self.assertEqual(records[2]['frame']['type'], 'slow_baseline')
            self.assertEqual(records[4]['kind'], 'centroid')
            self.assertEqual(records[4]['frame'], centroid)
            self.assertEqual([r['sequence'] for r in records], list(range(5)))
            self.assertEqual(writer_threads, ['serial-log-writer'])
            self.assertEqual(log.status()['batches'], 1)

    def test_timed_flush_and_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(directory, flush_seconds=.03, batch_bytes=600,
                                   segment_bytes=1200, keep_files=3)
            log.submit('connected')
            wait_for(lambda: log.status()['written'] == 1)
            for _ in range(40):
                log.submit('rx', data=b'x'*100)
            log.request_close(); self.assertTrue(log.wait(3))
            paths = list(Path(directory).glob('roll_*.jsonl'))
            self.assertEqual(len(paths), 3)
            self.assertEqual(rows(directory)[-1]['sequence'], 40)
            self.assertEqual(log.status()['written'], 41)

    def test_slow_disk_does_not_block_producer_and_reports_overflow(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(directory, batch_bytes=1, queue_limit=2)
            entered, release = threading.Event(), threading.Event()
            original = log._write
            def slow_write(batch):
                entered.set(); release.wait(3); original(batch)
            log._write = slow_write
            log.submit('rx', data=b'first')
            self.assertTrue(entered.wait(2))
            try:
                start = time.monotonic()
                for _ in range(10): log.submit('rx', data=b'next')
                self.assertLess(time.monotonic()-start, .5)
                self.assertEqual(log.status()['dropped'], 8)
            finally:
                release.set(); log.request_close(); self.assertTrue(log.wait(3))
            self.assertEqual(sum(r.get('dropped_since_last', 0) for r in rows(directory)), 8)

    def test_disk_failure_retains_batch_then_recovers(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RollingSerialLog(directory, flush_seconds=.03, batch_bytes=1)
            original = log._write
            attempts = []
            def failing_once(batch):
                attempts.append(1)
                if len(attempts) == 1: raise OSError('disk full')
                original(batch)
            log._write = failing_once
            log.submit('rx', data=b'retain')
            wait_for(lambda: log.status()['written'] == 1)
            log.request_close(); self.assertTrue(log.wait(3))
            self.assertEqual(len(rows(directory)), 1)
            self.assertEqual(log.status()['error'], '')


if __name__ == '__main__':
    unittest.main(verbosity=2)
