import csv
from collections import deque, Counter
import hashlib
import json
import os
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4


class SerialDataRecorder:
    """持续缓存五分钟完整解析帧，支持回溯导出及旧版手动记录接口。"""

    HISTORY_SECONDS = 300.0
    HISTORY_MAX_BYTES = 96 * 1024 * 1024
    HISTORY_MAX_FRAMES = 300000

    def __init__(self, output_dir=None):
        base_dir = Path(__file__).resolve().parent
        self.output_dir = Path(output_dir) if output_dir else base_dir / "recordings"
        self._lock = threading.Lock()
        self._file = None
        self._temp_path = None
        self._started_at = 0.0
        self._started_stamp = ""
        self._row_count = 0
        self._history = deque()
        self._history_bytes = 0
        self._history_sequence = 0
        self._history_available_since = None
        self._last_sensor_time = None
        self._export_lock = threading.Lock()

    @property
    def has_recent_data(self):
        with self._lock:
            return self._last_sensor_time is not None and time.perf_counter() - self._last_sensor_time <= self.HISTORY_SECONDS

    def _trim_history_locked(self, now):
        while self._history and (
            self._history[0][0] < now - self.HISTORY_SECONDS
            or self._history_bytes > self.HISTORY_MAX_BYTES
            or len(self._history) > self.HISTORY_MAX_FRAMES
        ):
            stamp, _, data = self._history.popleft()
            self._history_bytes -= len(data)
            self._history_available_since = stamp

    def snapshot_recent(self, minutes):
        """在点击时固定时间边界及不可变记录；磁盘写入交给后台线程。"""
        if minutes not in (1, 3, 5):
            raise ValueError("仅支持最近1、3或5分钟")
        with self._lock:
            end = time.perf_counter()
            exported_at = datetime.fromtimestamp(time.time()).astimezone()
            start = end - minutes * 60.0
            self._trim_history_locked(end)
            rows = tuple(row for row in self._history if start <= row[0] <= end)
            sensors = [row for row in rows if row[1] in ("debug", "signal")]
            if not sensors:
                raise ValueError(f"最近{minutes}分钟没有传感器采样，原CSV保留")
            available = max(start, self._history_available_since)
        sensor_times = [row[0] for row in sensors]
        counts = dict(Counter(row[1] for row in rows))
        return {
            "rows": rows, "start_monotonic": start,
            "metadata": {
                "minutes": minutes,
                "exported_at": exported_at.isoformat(timespec="microseconds"),
                "requested_start": datetime.fromtimestamp(exported_at.timestamp() - minutes * 60).astimezone().isoformat(timespec="microseconds"),
                "requested_end": exported_at.isoformat(timespec="microseconds"),
                "available_seconds": round(end - available, 3),
                "partial_history": available > start,
                "first_sensor_timestamp": json.loads(sensors[0][2])["timestamp"],
                "last_sensor_timestamp": json.loads(sensors[-1][2])["timestamp"],
                "sensor_span_seconds": round(sensor_times[-1] - sensor_times[0], 3),
                "seconds_since_last_sensor": round(end - sensor_times[-1], 3),
                "largest_sensor_gap_seconds": round(max((b-a for a,b in zip(sensor_times, sensor_times[1:])), default=0.0), 3),
                "row_count": len(rows), "sensor_sample_count": len(sensors), "frame_type_counts": counts,
            },
        }

    def write_recent_snapshot(self, snapshot):
        """生成时间戳副本，再原子替换固定入口；写失败时保留已有CSV。"""
        with self._export_lock:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            meta = dict(snapshot["metadata"])
            minutes = meta["minutes"]
            stamp = datetime.fromisoformat(meta["exported_at"]).strftime("%Y%m%d_%H%M%S_%f")
            token = uuid4().hex[:8]
            archive = self.output_dir / f"sensor_data_{minutes}min_{stamp}_{token}.csv"
            latest = self.output_dir / f"recent_{minutes}min.csv"
            temporary = self.output_dir / f".recent_{minutes}min_{token}.tmp"
            archive_temporary = archive.with_suffix(".tmp")
            meta_temporary = self.output_dir / f".recent_{minutes}min_{token}.json.tmp"
            try:
                field_names = ["timestamp", "elapsed_s", "sequence", "type"]
                known = set(field_names)
                for _, _, data in snapshot["rows"]:
                    for field in self._flatten(json.loads(data)):
                        if field not in known:
                            known.add(field)
                            field_names.append(field)
                with archive_temporary.open("w", encoding="utf-8-sig", newline="") as target:
                    writer = csv.DictWriter(target, fieldnames=field_names)
                    writer.writeheader()
                    for mono, _, data in snapshot["rows"]:
                        row = self._flatten(json.loads(data))
                        row["elapsed_s"] = round(mono - snapshot["start_monotonic"], 6)
                        writer.writerow(row)
                os.replace(archive_temporary, archive)
                digest = hashlib.sha256()
                with archive.open("rb") as source:
                    for block in iter(lambda: source.read(1024 * 1024), b""):
                        digest.update(block)
                meta.update(csv_file=latest.name, archive_file=archive.name, csv_sha256=digest.hexdigest())
                meta_text = json.dumps(meta, ensure_ascii=False, indent=2)
                archive.with_suffix(".meta.json").write_text(meta_text, encoding="utf-8")
                shutil.copyfile(archive, temporary)
                meta_temporary.write_text(meta_text, encoding="utf-8")
                os.replace(temporary, latest)
                os.replace(meta_temporary, latest.with_suffix(".meta.json"))
                return dict(meta, output_path=str(latest.resolve()), archive_path=str(archive.resolve()))
            finally:
                for path in (temporary, archive_temporary, meta_temporary):
                    path.unlink(missing_ok=True)

    @property
    def is_recording(self):
        with self._lock:
            return self._file is not None

    def start(self):
        with self._lock:
            if self._file is not None:
                return False

            self.output_dir.mkdir(parents=True, exist_ok=True)
            now = datetime.now()
            self._started_at = time.time()
            self._started_stamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self._temp_path = self.output_dir / f".sensor_data_{self._started_stamp}.jsonl"
            self._file = self._temp_path.open("w", encoding="utf-8", newline="\n")
            self._row_count = 0
            return True

    def record(self, frame):
        with self._lock:
            now = time.time()
            mono = time.perf_counter()
            if self._history_available_since is None:
                self._history_available_since = mono
            record = {
                "timestamp": datetime.fromtimestamp(now).astimezone().isoformat(timespec="milliseconds"),
                "sequence": self._history_sequence,
            }
            record.update(frame)
            data = json.dumps(record, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self._history.append((mono, frame["type"], data))
            self._history_bytes += len(data)
            self._history_sequence += 1
            if frame["type"] in ("debug", "signal"):
                self._last_sensor_time = mono
            self._trim_history_locked(mono)
            if self._file is not None:
                record.update(elapsed_s=round(now - self._started_at, 6), sequence=self._row_count)
                self._file.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
                self._row_count += 1

    def stop(self):
        with self._lock:
            if self._file is None:
                return None, 0

            self._file.flush()
            self._file.close()
            self._file = None
            temp_path = self._temp_path
            row_count = self._row_count
            output_path = self.output_dir / f"sensor_data_{self._started_stamp}.csv"
            self._temp_path = None

        self._convert_to_csv(temp_path, output_path)
        temp_path.unlink()
        return output_path, row_count

    @staticmethod
    def _flatten(value, prefix=""):
        flattened = {}

        if isinstance(value, dict):
            for key, item in value.items():
                child_prefix = f"{prefix}_{key}" if prefix else str(key)
                flattened.update(SerialDataRecorder._flatten(item, child_prefix))
        elif isinstance(value, (list, tuple)):
            for index, item in enumerate(value):
                child_prefix = f"{prefix}_{index}" if prefix else str(index)
                flattened.update(SerialDataRecorder._flatten(item, child_prefix))
        else:
            flattened[prefix] = value

        return flattened

    @classmethod
    def _convert_to_csv(cls, temp_path, output_path):
        field_names = []
        known_fields = set()

        with temp_path.open("r", encoding="utf-8") as source:
            for line in source:
                row = cls._flatten(json.loads(line))
                for field_name in row:
                    if field_name not in known_fields:
                        known_fields.add(field_name)
                        field_names.append(field_name)

        with temp_path.open("r", encoding="utf-8") as source, output_path.open("w", encoding="utf-8-sig", newline="") as target:
            writer = csv.DictWriter(target, fieldnames=field_names, extrasaction="ignore")
            writer.writeheader()
            for line in source:
                writer.writerow(cls._flatten(json.loads(line)))
