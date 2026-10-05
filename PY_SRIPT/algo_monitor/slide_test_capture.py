"""滑动试验的独立全量归档；由滚动日志写入线程顺序调用。"""
from collections import Counter
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import threading
import time
from uuid import uuid4
import zipfile


class SlideTestCapture:
    def __init__(self, directory, context):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid4().hex[:8]
        self.directory = Path(directory) / ('slide_test_' + stamp)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.path = self.directory / 'records.jsonl'
        self.archive = self.directory.with_suffix('.zip')
        self._file = self.path.open('xb', buffering=1024 * 1024)
        self._lock = threading.Lock()
        self._status = dict(done=False, error='', archive='', directory=str(self.directory.resolve()))
        self.context = context
        self.counts = Counter()
        self.phases = []
        self.phase = None
        self.digest = hashlib.sha256()
        self.last_flush = time.monotonic()
        self.last_sensor = None
        self.max_sensor_gap = 0.
        self.last_derived = {}
        self.derived_gaps = []
        self.started_at = datetime.now().astimezone().isoformat()

    def status(self):
        with self._lock:
            return dict(self._status)

    def begin_finish(self):
        with self._lock:
            if self._status['done'] or self._status.get('saving'):
                return False
            self._status['saving'] = True
            return True

    def write(self, encoded, record):
        if self.status()['error']:
            return
        try:
            self._file.write(encoded)
            self.digest.update(encoded)
            kind = record['kind']
            self.counts[kind] += 1
            if kind == 'slide_test' and record['frame'].get('event') == 'phase_begin':
                self.phase = dict(record['frame'], counts=Counter(),
                                  timestamp=datetime.fromtimestamp(record['time_ns'] / 1e9).astimezone().isoformat())
                self.phases.append(self.phase)
            if self.phase:
                self.phase['counts'][kind] += 1
            if kind in ('host_state_machine', 'touchpad'):
                frame = record['frame']
                sequence = frame.get('sequence' if kind == 'host_state_machine' else 'source_sample_sequence')
                stamp = frame.get('sample_timestamp')
                previous = self.last_derived.get(kind)
                if previous and sequence is not None and previous[0] is not None:
                    gap = stamp - previous[1] if stamp is not None and previous[1] is not None else None
                    if sequence != previous[0] + 1 or gap is not None and (gap <= 0 or gap > .15):
                        self.derived_gaps.append(dict(kind=kind, previous_sequence=previous[0],
                            sequence=sequence, sample_gap_seconds=gap))
                self.last_derived[kind] = (sequence, stamp)
            if kind == 'frame' and record['frame'].get('type') in ('debug', 'signal'):
                self.counts['sensor_samples'] += 1
                if self.phase:
                    self.phase['counts']['sensor_samples'] += 1
                stamp = record['monotonic_ns'] / 1e9
                if self.last_sensor is not None:
                    self.max_sensor_gap = max(self.max_sensor_gap, stamp - self.last_sensor)
                self.last_sensor = stamp
            if time.monotonic() - self.last_flush >= 1:
                self._file.flush()
                self.last_flush = time.monotonic()
        except OSError as exc:
            with self._lock:
                self._status['error'] = str(exc)

    def finish(self, outcome, reason, dropped):
        try:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._file.close()
            if self.status()['error']:
                raise OSError(self.status()['error'])
            empty = [dict(axis=p['axis'], trial=p['trial']) for p in self.phases if p['phase'] == 'slide'
                     and any(not p['counts'][k] for k in ('sensor_samples', 'host_state_machine', 'touchpad'))]
            expected = self.context['protocol']['horizontal_count'] + self.context['protocol']['vertical_count']
            slide_phases = sum(p['phase'] == 'slide' for p in self.phases)
            if outcome == 'completed' and (dropped or empty or slide_phases != expected
                                          or self.max_sensor_gap > .15 or self.derived_gaps):
                outcome = 'completed_with_gaps'
            meta = dict(schema='slide_test_v1', outcome=outcome, reason=reason,
                        started_at=self.started_at, ended_at=datetime.now().astimezone().isoformat(),
                        context=self.context, counts=dict(self.counts), phases=self.phases,
                        dropped_records=dropped, empty_slide_phases=empty,
                        derived_sequence_gaps=self.derived_gaps,
                        largest_sensor_gap_seconds=self.max_sensor_gap,
                        records_sha256=self.digest.hexdigest(), records_file='records.jsonl',
                        timing_note='阶段提示的 sample_timestamp 使用 perf_counter；按采样时间对齐阶段，不能按接触次数推断实际滑动次数。')
            manifest = self.directory / 'manifest.json'
            manifest.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
            temporary = self.archive.with_suffix('.zip.tmp')
            with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
                archive.write(self.path, 'records.jsonl')
                archive.write(manifest, 'manifest.json')
            os.replace(temporary, self.archive)
            with self._lock:
                self._status.update(done=True, archive=str(self.archive.resolve()), outcome=outcome)
        except (OSError, ValueError, zipfile.BadZipFile) as exc:
            try:
                self._file.close()
            finally:
                with self._lock:
                    self._status.update(done=True, error=str(exc))
