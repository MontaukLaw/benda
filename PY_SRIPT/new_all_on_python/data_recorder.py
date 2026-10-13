"""有界后台 CSV 写入；完整逐帧结果和参数/重置事件一起保留。"""

import csv
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import queue
import threading
import time
from .config import RECORDING_DIR, SENSOR_POSITIONS

FIELDS = (["timestamp", "monotonic_timestamp", "source", "protocol", "protocol_version", "device", "config_revision"]
          + [f"{prefix}{i}" for prefix in ("raw", "baseline", "signal") for i in range(20)]
          + ["max_index", "max_signal", "x_raw", "y_raw", "x_filtered", "y_filtered", "dx", "dy",
             "touch_state", "touched", "baseline_ready", "position_valid", "active_sensor_count",
             "active_indices", "roi_indices", "total_signal", "below_baseline_indices",
             "roi_target_indices", "roi_transitioning"]
          + [f"roi_weight{i}" for i in range(20)]
          + [f"firmware_state{i}" for i in range(20)]
          + [f"{prefix}{i}" for prefix in ("adjusted_raw", "raw_gain") for i in range(20)]
          + ["position_held", "position_confidence"])


class CsvRecorder:
    def __init__(self):
        self.path = None
        self.error = ""
        self.active = False
        self._thread = None
        self._queue = queue.Queue(maxsize=5000)
        self._stop = threading.Event()
        self.rows_written = 0
        self.rejected_items = 0

    def start(self, config, revision, directory=RECORDING_DIR):
        if self._thread and self._thread.is_alive():
            raise RuntimeError("上一份记录正在关闭")
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        path = directory / f"touch_{stamp}.csv"
        # 在启动时验证文件可写，失败由 GUI 显示。
        csv_file = path.open("x", encoding="utf-8-sig", newline="")
        try:
            events = path.with_suffix(".events.jsonl").open("x", encoding="utf-8")
        except Exception:
            csv_file.close()
            raise
        self.path = path
        self._queue = queue.Queue(maxsize=5000)
        self._stop.clear()
        self.error = ""
        self.rows_written = self.rejected_items = 0
        self.active = True
        self._thread = threading.Thread(target=self._write, args=(csv_file, events), daemon=True,
                                        name="touch-csv-writer")
        self._thread.start()
        self.event("recording_start", config=asdict(config), config_revision=revision,
                   sensor_positions=SENSOR_POSITIONS,
                   baseline_update_rule="signed_absolute_difference_lt_freeze_threshold_v2",
                   roi_weight_rule="time_bounded_linear_membership_v1",
                   position_confidence_rule="global_decay_direction_local_growth_guard_v1",
                   raw_gain_rule="adjusted_raw=raw*raw_gain; baseline/signal use adjusted_raw; raw retains device integers",
                   negative_difference_diagnostic_note="持续0.5秒 Raw 低于 baseline 超过 max(Freeze,Noise,Stability,1)；仅诊断，不改变触摸判定",
                   timestamp_note="perf_counter高精度单调主机接收时间；设备协议没有采样时间戳/序号")

    def event(self, kind, **payload):
        if self.active:
            self._enqueue(("event", {"timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                                    "kind": kind, **payload}))

    def record(self, result, frame, source, revision):
        if not self.active:
            return
        row = {"timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
               "monotonic_timestamp": result.timestamp, "source": source,
               "protocol": frame.get("protocol", source), "protocol_version": frame.get("protocol_version", ""),
               "device": frame.get("device", ""), "config_revision": revision}
        for prefix in ("raw", "baseline", "signal"):
            row.update({f"{prefix}{i}": value for i, value in enumerate(getattr(result, prefix))})
        row.update({f"adjusted_raw{i}": value for i, value in enumerate(result.adjusted_raw)})
        row.update({f"raw_gain{i}": value for i, value in enumerate(result.raw_gains)})
        for name in ("max_index", "max_signal", "x_raw", "y_raw", "dx", "dy", "touched",
                     "baseline_ready", "position_valid", "total_signal", "position_held", "position_confidence"):
            row[name] = getattr(result, name)
        row.update(x_filtered=result.x, y_filtered=result.y, touch_state=result.state,
                   active_sensor_count=len(result.active_indices),
                   active_indices=";".join(map(str, result.active_indices)),
                   roi_indices=";".join(map(str, result.roi_indices)))
        row["below_baseline_indices"] = ";".join(map(str, result.below_baseline_indices))
        row["roi_target_indices"] = ";".join(map(str, result.roi_target_indices))
        row["roi_transitioning"] = result.roi_transitioning
        row.update({f"roi_weight{i}": weight for i, weight in enumerate(result.roi_weights)})
        if not frame.get("raw_only"):
            row.update({f"firmware_state{i}": v for i, v in enumerate(frame.get("state", ()))})
        self._enqueue(("row", row))

    def _enqueue(self, item):
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.rejected_items += 1
            self.error = "记录队列已满；已停止记录，请检查磁盘速度/空间"
            self.active = False
            self._stop.set()

    def request_stop(self):
        self.event("recording_stop")
        self.active = False
        self._stop.set()

    @property
    def closing(self):
        return bool(self._thread and self._thread.is_alive() and not self.active)

    def finish(self, timeout=2.0):
        self.request_stop()
        if self._thread:
            self._thread.join(timeout)
        return not self.closing

    def _write(self, csv_file, events):
        last_flush = time.monotonic()
        try:
            with csv_file, events:
                writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
                writer.writeheader()
                while not self._stop.is_set() or not self._queue.empty():
                    try:
                        kind, item = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        kind = None
                    if kind == "row":
                        writer.writerow(item)
                        self.rows_written += 1
                    elif kind == "event":
                        events.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + "\n")
                    if time.monotonic() - last_flush >= 1.0:
                        csv_file.flush()
                        events.flush()
                        last_flush = time.monotonic()
                events.write(json.dumps({"kind": "recording_summary", "rows_written": self.rows_written,
                                         "rejected_items": self.rejected_items, "error": self.error},
                                        ensure_ascii=False) + "\n")
        except (OSError, ValueError) as exc:
            self.error = f"记录失败：{exc}；已写 {self.rows_written} 帧"
            self.active = False
            self._stop.set()
