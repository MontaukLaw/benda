"""串口全量滚动日志：生产者只入队，序列化、批量写盘和轮转在后台完成。"""
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import threading
import time
from uuid import uuid4


class RollingSerialLog:
    def __init__(self, directory=None, *, flush_seconds=5.0, batch_bytes=4*1024*1024,
                 segment_bytes=32*1024*1024, keep_files=16, queue_limit=16384,
                 queue_bytes=32*1024*1024):
        self.directory = Path(directory) if directory else Path(__file__).with_name('recordings') / 'rolling'
        self.flush_seconds = flush_seconds
        self.batch_bytes = batch_bytes
        self.segment_bytes = segment_bytes
        self.keep_files = keep_files
        self.queue_limit = queue_limit
        self.queue_bytes = queue_bytes
        self._queued_bytes = 0
        self.session = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid4().hex[:8]
        self._condition = threading.Condition()
        self._queue = deque()
        self._thread = None
        self._closing = False
        self._sequence = self._dropped = self._written = self._batches = 0
        self._pending = 0
        self._error = ''
        self._retention_error = ''
        self._last_write = None
        self._path = None
        self._segment = self._size = 0
        self._capture = None
        self._capture_drop_start = 0
        self._capture_workers = []
        self._capture_handles = []

    def start_capture(self, directory, context):
        from slide_test_capture import SlideTestCapture
        with self._condition:
            if self._closing or self._capture is not None:
                raise ValueError('日志已关闭或已有测试正在采集')
            capture = SlideTestCapture(directory, context)
            self._capture = capture
            self._capture_drop_start = self._dropped
            capture.log_drop_start = self._dropped
            self._capture_handles = [c for c in self._capture_handles if not c.status()['done']]
            self._capture_handles.append(capture)
            self.submit('slide_test', frame=dict(event='start', context=context), _control=True)
            return capture

    def finish_capture(self, outcome='completed', reason=''):
        with self._condition:
            if self._capture is None:
                return
            self.submit('slide_test', frame=dict(event='finish', outcome=outcome, reason=reason,
                        dropped_records=self._dropped - self._capture_drop_start), _control=True)
            self._capture = None

    def _finish_capture_in_background(self, capture, outcome, reason, dropped):
        if not capture.begin_finish():
            return
        # ZIP压缩单独运行，避免试验结束时阻塞持续串口日志。
        worker = threading.Thread(target=capture.finish, args=(outcome, reason, dropped),
                                  name='slide-test-archive', daemon=True)
        with self._condition:
            self._capture_workers = [w for w in self._capture_workers if w.is_alive()]
            self._capture_workers.append(worker)
            worker.start()

    def submit(self, kind, *, port=None, data=None, frame=None, message=None, _control=False):
        # 帧来自解析器，交给日志后不再修改；原始字节在生产者处转成不可变对象。
        with self._condition:
            if self._closing:
                return False
            sequence = self._sequence
            self._sequence += 1
            cost = len(data) + 1024 if data is not None else 16384 if frame is not None else 1024
            if not _control and (len(self._queue) >= self.queue_limit or self._queued_bytes + cost > self.queue_bytes):
                self._dropped += 1
                return False
            record = dict(kind=kind, sequence=sequence, time_ns=time.time_ns(),
                          monotonic_ns=time.monotonic_ns(), port=port)
            if data is not None:
                record['data'] = bytes(data)
            if frame is not None:
                record['frame'] = frame
            if message is not None:
                record['message'] = message
            if self._capture is not None:
                record['_capture'] = self._capture
            self._queue.append((record, cost))
            self._queued_bytes += cost
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name='serial-log-writer', daemon=True)
                self._thread.start()
            self._condition.notify()
            return True

    def status(self):
        with self._condition:
            return dict(session=self.session, directory=str(self.directory), path=str(self._path or ''),
                        queued=len(self._queue), pending=self._pending, dropped=self._dropped,
                        written=self._written, batches=self._batches, error=self._error,
                        retention_error=self._retention_error,
                        last_write=self._last_write, closing=self._closing)

    def request_close(self):
        with self._condition:
            self.finish_capture('aborted', '程序关闭')
            self._closing = True
            self._condition.notify_all()

    def is_alive(self):
        with self._condition:
            return (self._thread is not None and self._thread.is_alive()
                    or any(worker.is_alive() for worker in self._capture_workers))

    def wait(self, timeout=None):
        deadline = None if timeout is None else time.monotonic() + timeout
        if self._thread is not None:
            self._thread.join(timeout)
        with self._condition:
            workers = list(self._capture_workers)
        for worker in workers:
            worker.join(None if deadline is None else max(0, deadline - time.monotonic()))
        return not self.is_alive()

    @staticmethod
    def _encode(record):
        record = dict(record)
        stamp = record['time_ns'] / 1e9
        record['timestamp'] = datetime.fromtimestamp(stamp).astimezone().isoformat(timespec='microseconds')
        if 'data' in record:
            raw = record.pop('data')
            record.update(hex=raw.hex(), byte_count=len(raw))
        return (json.dumps(record, ensure_ascii=False, separators=(',', ':'))+'\n').encode('utf-8')

    def _write(self, batch):
        self.directory.mkdir(parents=True, exist_ok=True)
        if self._path is None or self._size + len(batch) > self.segment_bytes:
            self._segment += 1
            self._path = self.directory / f'roll_{self.session}_{self._segment:05d}.jsonl'
            self._size = 0
        # 失败时截断至本批开始处，重试不会留下半行或重复记录。
        with self._path.open('a+b') as target:
            target.seek(0, 2)
            start = target.tell()
            try:
                target.write(batch)
                target.flush()
            except OSError:
                target.seek(start)
                target.truncate()
                raise
        self._size += len(batch)
        retention_error = ''
        try:
            for old in sorted(self.directory.glob('roll_*.jsonl'))[:-self.keep_files]:
                if old != self._path:
                    old.unlink()
        except OSError as exc:
            # 清理失败不重写已落盘批次；告警说明保留上限暂时无法满足。
            retention_error = str(exc)
        with self._condition:
            self._retention_error = retention_error

    def _run(self):
        batch = bytearray()
        count = 0
        deadline = time.monotonic() + self.flush_seconds
        reported_drops = 0
        while True:
            with self._condition:
                if not self._queue and not self._closing:
                    self._condition.wait(max(0, deadline - time.monotonic()))
                record = None
                if self._queue and len(batch) < self.batch_bytes:
                    record, cost = self._queue.popleft()
                    self._queued_bytes -= cost
                closing = self._closing
                empty = not self._queue
                drops = self._dropped
            if record is not None:
                capture = record.pop('_capture', None)
                encoded = self._encode(record)
                batch.extend(encoded)
                if capture is not None:
                    # 使用相同编码与顺序归档；无需重复解析或阻塞串口/GUI生产者。
                    capture.write(encoded, record)
                    if record['kind'] == 'slide_test' and record['frame'].get('event') == 'finish':
                        f = record['frame']
                        self._finish_capture_in_background(capture, f['outcome'], f['reason'], f['dropped_records'])
                count += 1
            due = time.monotonic() >= deadline or len(batch) >= self.batch_bytes or (closing and empty)
            if due and drops != reported_drops:
                batch.extend(self._encode(dict(kind='log_gap', sequence=None, time_ns=time.time_ns(),
                                               monotonic_ns=time.monotonic_ns(), dropped_total=drops,
                                               dropped_since_last=drops-reported_drops)))
                reported_drops = drops
                count += 1
            with self._condition:
                self._pending = count
            if due:
                try:
                    if batch:
                        self._write(batch)
                        with self._condition:
                            self._written += count
                            self._batches += 1
                            self._last_write = datetime.now().astimezone().isoformat(timespec='seconds')
                        batch.clear()
                        count = 0
                    with self._condition:
                        self._error = ''
                        self._pending = 0
                except OSError as exc:
                    with self._condition:
                        self._error = str(exc)
                    if closing:
                        # 主日志最终写入失败时，仍独立封存已采集的试验数据，并标明未消费队列。
                        with self._condition:
                            for capture in self._capture_handles:
                                lost = sum(r.get('_capture') is capture for r, _ in self._queue)
                                self._finish_capture_in_background(capture, 'aborted',
                                    '串口日志写入失败：' + str(exc), self._dropped - capture.log_drop_start + lost)
                        return
                    # 写盘异常时保留本批、限制队列，避免占用无限内存或忙循环。
                    with self._condition:
                        self._condition.wait_for(lambda: self._closing, timeout=self.flush_seconds)
                deadline = time.monotonic() + self.flush_seconds
            if closing and empty and not batch:
                return
