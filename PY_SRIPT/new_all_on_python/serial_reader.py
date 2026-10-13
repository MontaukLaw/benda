"""8N1 接收线程；复用协议解析器，不触碰固件状态或 Flash。"""

from PySide6 import QtCore
from .protocol_bridge import ProtocolDecoder, command
from .sample_clock import now


class SerialReader(QtCore.QThread):
    status_changed = QtCore.Signal(bool, str)

    def __init__(self, port, baudrate, protocol, engine, parent=None, serial_factory=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self.protocol = protocol
        self.engine = engine
        self.serial_factory = serial_factory
        self.decoder = ProtocolDecoder(protocol)
        self.error_message = ""
        self.rejected_channels = 0
        self.signal_only_frames = 0
        self._requested_debug = False

    def stop(self):
        self.requestInterruption()

    def run(self):
        connection = None
        try:
            import serial
            factory = self.serial_factory or serial.Serial
            connection = factory(self.port, self.baudrate, timeout=0.02, write_timeout=0.2,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False, dsrdtr=False)
            self.status_changed.emit(True, f"已连接 {self.port} · {self.protocol} · {self.baudrate} baud")
            while not self.isInterruptionRequested():
                data = connection.read(min(connection.in_waiting, 65536) or 1)
                for frame in self.decoder.feed(data):
                    if frame.get("type") == "signal":
                        self.signal_only_frames += 1
                        if frame.get("channel_count") == 20 and not self._requested_debug:
                            # Signal帧没有Raw，按已验证的实际版本请求一次易失Debug模式。
                            packet = command(frame["protocol_version"], 0x82, b"\x01")
                            if connection.write(packet) != len(packet):
                                raise OSError("Debug模式命令未完整发送")
                            self._requested_debug = True
                            self.status_changed.emit(True, "收到 Signal-only，已请求 Debug，等待20通道Raw")
                        continue
                    if frame.get("type") != "debug" or "raw" not in frame:
                        continue
                    if frame.get("channel_count") != 20:
                        self.rejected_channels += 1
                        continue
                    self.engine.update(frame["raw"], now(), frame)
        except Exception as exc:
            self.error_message = str(exc)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception as exc:
                    self.error_message = self.error_message or str(exc)
            self.engine.end_source()
            message = f"串口错误：{self.error_message}" if self.error_message else "已断开"
            self.status_changed.emit(False, message)
