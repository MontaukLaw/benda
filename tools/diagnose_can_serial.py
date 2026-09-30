"""只读 CAN/串口对照采集：不发送使能、运动、清零或传感器配置命令。"""

import argparse
from collections import Counter
import ctypes
from datetime import datetime
import hashlib
import json
import multiprocessing
from pathlib import Path
import queue
import struct
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "PY_SRIPT/algo_monitor"))


def can_worker(config, stop, events, poll):
    from zdt_x42s_can_driver import ZDTX42SMotor

    motor = ZDTX42SMotor(config)
    try:
        motor.open()
        events.put({"kind": "can_open", "time": time.perf_counter()})
        while not stop.is_set():
            start = time.perf_counter()
            if poll:
                try:
                    status = motor.read_status()
                    events.put({"kind": "can_status", "start": start,
                                "time": time.perf_counter(), "position_deg": status["position_deg"],
                                "speed_rpm": status["speed_rpm"]})
                except Exception as exc:
                    events.put({"kind": "can_error", "start": start,
                                "time": time.perf_counter(), "error": repr(exc)})
            stop.wait(max(0.001, 0.1 - (time.perf_counter() - start)))
    except Exception as exc:
        events.put({"kind": "can_open_error", "time": time.perf_counter(), "error": repr(exc)})
    finally:
        try:
            motor.close()
        except Exception as exc:
            events.put({"kind": "can_close_error", "time": time.perf_counter(), "error": repr(exc)})
        events.put({"kind": "can_closed", "time": time.perf_counter()})


class SerialErrorCapture:
    """保留 pyserial 的两处 ClearCommError 调用实际读到的标志，不增加清错调用。"""

    def __init__(self):
        self.events = []
        self.original = None

    def __enter__(self):
        if sys.platform == "win32":
            from serial import win32

            self.original = win32.ClearCommError

            def observe(handle, flags, status):
                result = self.original(handle, flags, status)
                if result:
                    mask = ctypes.cast(flags, ctypes.POINTER(win32.DWORD)).contents.value
                    if mask:
                        self.events.append({"time": time.perf_counter(), "mask": mask,
                                            "names": [name for bit, name in (
                                                (1, "RXOVER"), (2, "OVERRUN"), (4, "RXPARITY"),
                                                (8, "FRAME"), (16, "BREAK"), (256, "TXFULL")) if mask & bit]})
                return result

            win32.ClearCommError = observe
        return self

    def __exit__(self, *_):
        if self.original is not None:
            from serial import win32
            win32.ClearCommError = self.original


def analyze(raw):
    from frame_parser import AlgoFrameParser

    parser = AlgoFrameParser()
    frames = parser.feed(raw)
    debug = [f for f in frames if f["type"] == "debug"]
    invalid = [f for f in debug if any(value > 8 for value in f["state"])]
    # 独立扫描原始字节，验证 DATA 帧头提前进入 debug 声明长度的现象。
    candidates = []
    offset = 0
    while True:
        offset = raw.find(b"DATA", offset)
        if offset < 0:
            break
        if offset + 10 <= len(raw):
            _, version, kind, device, channels, length = struct.unpack_from("<4sBBBBH", raw, offset)
            if 14 <= version <= 20 and kind == 0x10 and channels == 8 and length == (128 if version >= 20 else 208):
                end = offset + 10 + length
                if end <= len(raw):
                    states = struct.unpack_from("<8H", raw, end - 16)
                    next_header = raw.find(b"DATA", offset + 10, end)
                    if any(v > 8 for v in states) and 0 <= next_header <= len(raw) - 10:
                        header = raw[next_header:next_header + 10]
                        if header[4] == version and header[5] == 2 and header[6:10] == bytes((device, 8, 40, 0)):
                            candidates.append({"offset": offset, "next_header_offset": next_header,
                                               "shortfall_bytes": end - next_header,
                                               "next_header_hex": header.hex(" ")})
        offset += 4
    return {"bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest(),
            "debug_frames": len(debug), "invalid_state_frames": len(invalid),
            "signal_over_10000_frames": sum(any(v > 10000 for v in f["signal"]) for f in debug),
            "early_run_force_headers": candidates}


def drain(events, target):
    while True:
        try:
            target.append(events.get_nowait())
        except queue.Empty:
            return


def capture_phase(port, config, phase, seconds, errors):
    raw = bytearray()
    chunks = []
    events = []
    worker = stop = event_queue = None
    if phase != "off":
        if phase == "process":
            context = multiprocessing.get_context("spawn")
            stop, event_queue = context.Event(), context.Queue()
            worker = context.Process(target=can_worker, args=(config, stop, event_queue, True), daemon=True)
        else:
            stop, event_queue = threading.Event(), queue.Queue()
            worker = threading.Thread(target=can_worker, args=(config, stop, event_queue, phase != "idle"), daemon=True)
    start = time.perf_counter()
    error_start = len(errors.events)
    capture_error = None

    def receive():
        read_start = time.perf_counter()
        waiting = port.in_waiting
        data = port.read(waiting if waiting else 1)
        now = time.perf_counter()
        if data:
            chunks.append({"start": read_start, "time": now, "offset": len(raw),
                           "length": len(data), "waiting": waiting})
            raw.extend(data)
        if event_queue is not None:
            drain(event_queue, events)

    try:
        if worker:
            worker.start()
        while time.perf_counter() - start < seconds:
            receive()
    except BaseException as exc:
        capture_error = exc
    finally:
        if worker:
            stop.set()
            # 只读事务最长为两个请求的超时；留出 USB 关闭超时，不并行打开第二个 CAN。
            deadline = time.perf_counter() + 8
            while worker.is_alive() and time.perf_counter() < deadline:
                if capture_error is None:
                    try:
                        receive()
                    except BaseException as exc:
                        capture_error = exc
                else:
                    worker.join(timeout=0.05)
                    drain(event_queue, events)
            drain(event_queue, events)
            if worker.is_alive():
                capture_error = RuntimeError("CAN worker did not stop; aborting remaining phases")
        elapsed = time.perf_counter() - start
    return {"raw": raw, "chunks": chunks, "events": events, "phase": phase,
            "start": start, "elapsed_seconds": elapsed, "serial_errors": errors.events[error_start:],
            "capture_error": repr(capture_error) if capture_error else None}


def save_capture(capture, folder):
    raw, chunks, events = capture["raw"], capture["chunks"], capture["events"]
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "serial.bin").write_bytes(raw)
    (folder / "chunks.json").write_text(json.dumps(chunks), encoding="utf-8")
    (folder / "can_events.json").write_text(json.dumps(events, indent=2), encoding="utf-8")
    report = analyze(raw)
    report.update({k: capture[k] for k in ("phase", "start", "elapsed_seconds", "serial_errors", "capture_error")})
    report.update(can_status_count=sum(e["kind"] == "can_status" for e in events),
                  serial_error_counts=dict(Counter(name for e in capture["serial_errors"] for name in e["names"])),
                  can_errors=[e for e in events if "error" in e["kind"]],
                  max_waiting_bytes=max((c["waiting"] for c in chunks), default=0),
                  max_read_seconds=max((c["time"] - c["start"] for c in chunks), default=0),
                  max_reader_gap_seconds=max((b["start"] - a["time"] for a, b in zip(chunks, chunks[1:])), default=0))
    (folder / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main():
    import serial
    from motor_connection_config import load_motor_connection_settings

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--phases", nargs="+", choices=("off", "idle", "thread", "process"),
                        default=["off", "idle", "thread", "process", "off"])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error("seconds must be between 1 and 120")
    model, config = load_motor_connection_settings()
    if model != "zdt_x42s":
        parser.error("Diagnostic is restricted to the configured ZDT motor")
    folder = args.output or ROOT / "analysis_outputs" / ("can_serial_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "session.json").write_text(json.dumps({"created_at": datetime.now().astimezone().isoformat(),
        "python": sys.executable, "port": args.port, "baud": 1000000, "seconds_per_phase": args.seconds,
        "phases": args.phases, "can_config": config.__dict__, "commands": ["read system status 0x43", "read temperature 0x39"]}, indent=2), encoding="utf-8")
    captures, reports = [], []
    print("Output:", folder, flush=True)
    try:
        with SerialErrorCapture() as errors, serial.Serial(args.port, 1000000, timeout=0.02) as port:
            for i, phase in enumerate(args.phases):
                print(f"Starting {i + 1}/{len(args.phases)}: {phase}", flush=True)
                capture = capture_phase(port, config, phase, args.seconds, errors)
                captures.append(capture)
                print(f"Captured {len(capture['raw'])} bytes, {len(capture['serial_errors'])} serial errors", flush=True)
                if capture["capture_error"]:
                    raise RuntimeError(capture["capture_error"])
    finally:
        # 采集期间只在内存追加，不因分析或写盘暂停串口读取。
        for i, capture in enumerate(captures):
            report = save_capture(capture, folder / f"{i:02d}_{capture['phase']}")
            reports.append(report)
            print(json.dumps({k: report[k] for k in ("phase", "bytes", "debug_frames", "invalid_state_frames",
                  "can_status_count", "serial_error_counts", "can_errors", "max_waiting_bytes")}), flush=True)
            (folder / "summary.json").write_text(json.dumps(reports, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
