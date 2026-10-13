"""隔离加载旧解析器，避免两个应用的同名 config 模块互相覆盖。"""

import builtins
from pathlib import Path
import struct
import time
import types

PY_ROOT = Path(__file__).resolve().parents[1]


def _load_existing(path, dependencies=None):
    """原文件原样执行，仅在本模块内解析旧式绝对导入，不改 sys.path/modules。"""
    module = types.ModuleType("_touch_debug_" + path.stem)
    module.__file__ = str(path)
    namespace = vars(builtins).copy()
    normal_import = builtins.__import__

    def local_import(name, globals=None, locals=None, fromlist=(), level=0):
        if level == 0 and dependencies and name in dependencies:
            return dependencies[name]
        return normal_import(name, globals, locals, fromlist, level)

    namespace["__import__"] = local_import
    module.__dict__["__builtins__"] = namespace
    exec(compile(path.read_text(encoding="utf-8-sig"), str(path), "exec"), module.__dict__)
    return module


_config = _load_existing(PY_ROOT / "algo_monitor" / "config.py")
_low_force = _load_existing(PY_ROOT / "algo_monitor" / "low_force_calibration.py")
_algo_parser = _load_existing(PY_ROOT / "algo_monitor" / "frame_parser.py",
                              {"config": _config, "low_force_calibration": _low_force})
_raw20 = _load_existing(PY_ROOT / "raw20_serial.py")
_KNOWN_FRAME_TYPES = {value for name, value in vars(_config).items() if name.startswith("FRAME_TYPE_")}


class ProtocolDecoder:
    """给既有解析器补充帧边界诊断；Raw/Debug 数值布局只由既有解析器解码。"""

    def __init__(self, protocol="DATA"):
        if protocol not in ("DATA", "RAW20"):
            raise ValueError("未知协议")
        self.protocol = protocol
        self.parser = _algo_parser.AlgoFrameParser() if protocol == "DATA" else _raw20.FrameDecoder()
        self.buffer = bytearray()
        self.parse_errors = 0
        self.discarded_bytes = 0
        self.ignored_frames = 0
        self._partial_since = None

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        self.buffer.extend(data)
        frames = []
        magic = _config.FRAME_MAGIC if self.protocol == "DATA" else _raw20.HEADER
        while self.buffer:
            start = self.buffer.find(magic)
            if start < 0:
                discard = max(0, len(self.buffer) - len(magic) + 1)
                self.discarded_bytes += discard
                del self.buffer[:discard]
                self._partial_since = None
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
                self._partial_since = None
            if self.protocol == "DATA":
                size = 10
                if len(self.buffer) >= 10:
                    _, version, kind, device, count, length = struct.unpack_from("<4sBBBBH", self.buffer)
                    valid = version in _config.SUPPORTED_FRAME_VERSIONS and count <= 20 and length <= 1024
                    if kind in (0x01, 0x10, 0x16):
                        expected = count * (16 if version >= 20 else 26) if kind == 0x10 else count * 2
                        valid = valid and count > 0 and length == expected and (kind != 0x16 or version >= 21)
                    if not valid:
                        self._reject()
                        continue
                    size += length
            else:
                size = _raw20.FRAME_SIZE
            if len(self.buffer) < size:
                if self._partial_since is None:
                    self._partial_since = now
                elif now - self._partial_since > 0.5:
                    # 不完整帧超时后逐字节重同步；不让坏长度永久吞掉后续采样。
                    self._reject()
                    continue
                break
            packet = bytes(self.buffer[:size])
            if self.protocol == "RAW20":
                if sum(packet[4:44]) & 0xFFFF != int.from_bytes(packet[44:46], "little"):
                    self._reject()
                    continue
                decoded = self.parser.feed(packet)
                frames.extend({"type": "debug", "raw_only": True, "raw": raw,
                               "channel_count": 20, "protocol": "RAW20"} for raw in decoded)
            else:
                try:
                    decoded = self.parser.feed(packet)
                except (ValueError, struct.error, OverflowError, IndexError):
                    # 坏诊断载荷也不能中断接收；清掉旧解析器未消费的坏帧。
                    self.parser = _algo_parser.AlgoFrameParser()
                    self.parse_errors += 1
                    del self.buffer[:size]
                    self._partial_since = None
                    continue
                if not decoded:
                    if kind in _KNOWN_FRAME_TYPES:
                        self.parse_errors += 1
                    else:
                        self.ignored_frames += 1
                for frame in decoded:
                    frame["protocol"] = "DATA"
                frames.extend(decoded)
            del self.buffer[:size]
            self._partial_since = None
        return frames

    def _reject(self):
        self.parse_errors += 1
        self.discarded_bytes += 1
        del self.buffer[0]
        self._partial_since = None


def command(version, kind, payload=b""):
    return struct.pack("<4sBBBBH", b"DATA", version, kind, 0, 0, len(payload)) + payload
