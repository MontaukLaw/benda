from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from pathlib import Path

from qt_compat import QtCore, Signal
from config import DFU_APPLICATION_START, DFU_APPLICATION_END, DFU_PROTECTED_START, DFU_FLASH_ROW_BYTES


SOP = 0x01
EOP = 0x17
PACKET_MIN_SIZE = 7
UART_DFU_MAX_PAYLOAD = 48

APP_RESET_MAGIC = bytes([0xAB, 0xCD])
APP_RESET_REPEAT_COUNT = 3
APP_RESET_REPEAT_DELAY_S = 0.08
APP_RESET_BOOT_DELAY_S = 0.35
APP_RESET_ENTER_RETRIES = 8
APP_RESET_ENTER_RETRY_DELAY_S = 0.35

CMD_VERIFY_APP = 0x31
CMD_SEND_DATA = 0x37
CMD_ENTER = 0x38
CMD_EXIT = 0x3B
CMD_PROGRAM_DATA = 0x49
CMD_VERIFY_DATA = 0x4A
CMD_SET_APP_META = 0x4C

COMMAND_NAMES = {
    CMD_VERIFY_APP: "VERIFY_APP",
    CMD_SEND_DATA: "SEND_DATA",
    CMD_ENTER: "ENTER",
    CMD_EXIT: "EXIT",
    CMD_PROGRAM_DATA: "PROGRAM_DATA",
    CMD_VERIFY_DATA: "VERIFY_DATA",
    CMD_SET_APP_META: "SET_APP_META",
}

STATUS_NAMES = {
    0x00: "SUCCESS",
    0x02: "VERIFY",
    0x03: "LENGTH",
    0x04: "DATA",
    0x05: "CMD",
    0x08: "CHECKSUM",
    0x0A: "ADDRESS",
    0x0F: "UNKNOWN",
    0x40: "TIMEOUT",
    0x50: "BAD_PARAM",
}


@dataclass(frozen=True)
class Cyacd2Row:
    address: int
    data: bytes


@dataclass(frozen=True)
class Cyacd2Image:
    path: Path
    product_id: int
    app_id: int
    verify_start: int
    verify_size: int
    rows: list[Cyacd2Row]

    @property
    def bytes_total(self) -> int:
        return sum(len(row.data) for row in self.rows)


class DfuError(RuntimeError):
    pass


def resolve_dfu_image_path(selected_path: str | Path) -> Path:
    """Resolve a displayed .hex path to the companion DFU .cyacd2 image."""
    path = Path(selected_path).expanduser()
    if not path.is_absolute():
        path = path.resolve()

    if path.suffix.lower() == ".cyacd2":
        candidate = path
    elif path.suffix.lower() == ".hex":
        candidate = path.with_name(path.stem + "_crc.cyacd2")
    else:
        raise ValueError("请选择 .hex 或 .cyacd2 固件文件")

    if not path.exists():
        raise FileNotFoundError(f"固件文件不存在: {path}")
    if not candidate.exists():
        raise FileNotFoundError(
            f"DFU 文件不存在: {candidate}\n"
            "请先完成固件构建，生成与 hex 同目录的 _crc.cyacd2 文件。"
        )
    return candidate


def crc32c(data: bytes) -> int:
    table = [
        0x00000000,
        0x105EC76F,
        0x20BD8EDE,
        0x30E349B1,
        0x417B1DBC,
        0x5125DAD3,
        0x61C69362,
        0x7198540D,
        0x82F63B78,
        0x92A8FC17,
        0xA24BB5A6,
        0xB21572C9,
        0xC38D26C4,
        0xD3D3E1AB,
        0xE330A81A,
        0xF36E6F75,
    ]
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        crc = (crc >> 4) ^ table[crc & 0x0F]
        crc = (crc >> 4) ^ table[crc & 0x0F]
    return (~crc) & 0xFFFFFFFF


def packet_checksum(packet_prefix: bytes, data_size: int) -> int:
    size = data_size + 4
    total = sum(packet_prefix[:size]) & 0xFFFF
    return (1 + (~total)) & 0xFFFF


def build_packet(command: int, payload: bytes = b"") -> bytes:
    header = bytes([SOP, command]) + struct.pack("<H", len(payload)) + payload
    checksum = packet_checksum(header, len(payload))
    return header + struct.pack("<H", checksum) + bytes([EOP])


def parse_packet(packet: bytes) -> tuple[int, bytes]:
    if len(packet) < PACKET_MIN_SIZE:
        raise DfuError("DFU 返回包过短")
    if packet[0] != SOP or packet[-1] != EOP:
        raise DfuError("DFU 返回包帧头或帧尾错误")

    size = struct.unpack_from("<H", packet, 2)[0]
    expected = size + PACKET_MIN_SIZE
    if len(packet) != expected:
        raise DfuError(f"DFU 返回包长度错误: 收到 {len(packet)}，期望 {expected}")

    actual = struct.unpack_from("<H", packet, 4 + size)[0]
    wanted = packet_checksum(packet[: 4 + size], size)
    if actual != wanted:
        raise DfuError(f"DFU 返回包校验错误: 0x{actual:04X} != 0x{wanted:04X}")
    return packet[1], packet[4 : 4 + size]


def validate_application_image(image: Cyacd2Image) -> None:
    """在复位或发送任何DFU命令前验证整张镜像，保护参数和尾部元数据。"""
    verify_end = image.verify_start + image.verify_size + 4
    if (
        image.app_id != 1
        or image.verify_start != DFU_APPLICATION_START
        or image.verify_size <= 0
        or verify_end > DFU_APPLICATION_END
        or DFU_APPLICATION_END > DFU_PROTECTED_START
        or verify_end % DFU_FLASH_ROW_BYTES
    ):
        raise ValueError("固件应用范围无效或超出0x1F5FF；禁止覆盖参数区及DFU保护区")
    expected_addresses = list(range(DFU_APPLICATION_START, verify_end, DFU_FLASH_ROW_BYTES))
    if [row.address for row in image.rows] != expected_addresses or any(
        len(row.data) != DFU_FLASH_ROW_BYTES for row in image.rows
    ):
        raise ValueError("固件数据行越界、重复、缺失或未对齐；未发送烧写命令")
    data = b''.join(row.data for row in image.rows)
    if crc32c(data[:-4]) != int.from_bytes(data[-4:], 'little'):
        raise ValueError("固件CRC签名不匹配；未发送烧写命令")


def parse_cyacd2(path: str | Path) -> Cyacd2Image:
    file_path = Path(path)
    lines = [
        line.strip()
        for line in file_path.read_text(encoding="ascii").splitlines()
        if line.strip()
    ]
    if len(lines) < 3:
        raise ValueError("cyacd2 文件内容过短")

    header = bytes.fromhex(lines[0])
    if len(header) < 12:
        raise ValueError("不支持的 cyacd2 文件头")

    app_id = header[0]
    product_id = int.from_bytes(header[-4:], "little")
    verify_start = None
    verify_size = None
    rows: list[Cyacd2Row] = []

    for line in lines[1:]:
        if line.startswith("@APPINFO:"):
            left, right = line[len("@APPINFO:") :].split(",", 1)
            verify_start = int(left, 0)
            verify_size = int(right, 0)
            continue
        if not line.startswith(":"):
            continue

        raw = bytes.fromhex(line[1:])
        if len(raw) <= 4:
            continue
        rows.append(
            Cyacd2Row(
                address=int.from_bytes(raw[:4], "little"),
                data=raw[4:],
            )
        )

    if not rows:
        raise ValueError("cyacd2 文件中没有可烧写的数据行")

    rows.sort(key=lambda row: row.address)
    if verify_start is None:
        verify_start = rows[0].address
    if verify_size is None:
        end = max(row.address + len(row.data) for row in rows)
        verify_size = max(0, end - verify_start - 4)

    image = Cyacd2Image(
        path=file_path,
        product_id=product_id,
        app_id=app_id,
        verify_start=verify_start,
        verify_size=verify_size,
        rows=rows,
    )
    validate_application_image(image)
    return image


def _command_name(command: int) -> str:
    return f"{COMMAND_NAMES.get(command, 'CMD')}[0x{command:02X}]"


class DfuSerialClient:
    def __init__(self, port: str, baudrate: int, timeout: float = 2.0) -> None:
        try:
            import serial
        except ImportError as exc:
            raise DfuError("pyserial 未安装") from exc

        self.port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=5.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

    def close(self) -> None:
        if self.port and self.port.is_open:
            self.port.close()

    def resync(self) -> None:
        self.port.reset_input_buffer()
        time.sleep(0.05)
        waiting = self.port.in_waiting
        if waiting:
            self.port.read(waiting)

    def request(self, command: int, payload: bytes = b"", timeout: float = 3.0) -> bytes:
        self.port.reset_input_buffer()
        self._write_packet(command, build_packet(command, payload))
        try:
            return self._read_response(timeout)
        except DfuError as exc:
            raise DfuError(f"{_command_name(command)} 失败: {exc}") from exc

    def command_no_response(self, command: int, payload: bytes = b"") -> None:
        self._write_packet(command, build_packet(command, payload))

    def write_raw(self, data: bytes) -> None:
        try:
            self.port.write(data)
            self.port.flush()
        except Exception as exc:
            raise DfuError(f"发送复位命令失败: {exc}") from exc

    def _write_packet(self, command: int, packet: bytes) -> None:
        try:
            self.port.write(packet)
            self.port.flush()
        except Exception as exc:
            raise DfuError(f"{_command_name(command)} 写入失败: {exc}") from exc

    def _read_response(self, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            byte = self.port.read(1)
            if not byte or byte[0] != SOP:
                continue

            head = self.port.read(3)
            if len(head) != 3:
                continue
            size = struct.unpack_from("<H", head, 1)[0]
            rest = self.port.read(size + 3)
            if len(rest) != size + 3:
                continue

            status, data = parse_packet(byte + head + rest)
            if status != 0:
                name = STATUS_NAMES.get(status, "ERROR")
                raise DfuError(f"设备返回 0x{status:02X} ({name})")
            return data
        raise DfuError("等待设备响应超时")


class DfuProgrammer:
    def __init__(self, client: DfuSerialClient, status, progress) -> None:
        self.client = client
        self.status = status
        self.progress = progress

    def _send_row_payload(self, data: bytes) -> None:
        for offset in range(0, len(data), UART_DFU_MAX_PAYLOAD):
            self.client.request(
                CMD_SEND_DATA,
                data[offset : offset + UART_DFU_MAX_PAYLOAD],
                timeout=5.0,
            )

    def _enter_dfu(self, image: Cyacd2Image) -> bytes:
        self.status("正在进入 DFU...")
        try:
            return self.client.request(
                CMD_ENTER,
                struct.pack("<I", image.product_id),
                timeout=5.0,
            )
        except DfuError:
            pass

        self.status("应用正在运行，发送软件复位并等待 Bootloader...")
        self.client.port.reset_input_buffer()
        for _ in range(APP_RESET_REPEAT_COUNT):
            self.client.write_raw(APP_RESET_MAGIC)
            time.sleep(APP_RESET_REPEAT_DELAY_S)
        time.sleep(APP_RESET_BOOT_DELAY_S)

        last_error = None
        for attempt in range(1, APP_RESET_ENTER_RETRIES + 1):
            self.client.port.reset_input_buffer()
            self.status(f"等待 Bootloader 响应 ({attempt}/{APP_RESET_ENTER_RETRIES})...")
            try:
                return self.client.request(
                    CMD_ENTER,
                    struct.pack("<I", image.product_id),
                    timeout=2.0,
                )
            except DfuError as exc:
                last_error = exc
                time.sleep(APP_RESET_ENTER_RETRY_DELAY_S)

        raise DfuError(f"已发送软件复位，但 Bootloader 未响应: {last_error}")

    def program(self, image: Cyacd2Image) -> None:
        # 再次验证，避免绕过文件解析或修改rows后触发设备复位/写入。
        validate_application_image(image)
        self.client.resync()
        self._enter_dfu(image)

        self.status("正在设置应用元数据...")
        metadata = bytes([image.app_id]) + struct.pack(
            "<II", image.verify_start, image.verify_size
        )
        self.client.request(CMD_SET_APP_META, metadata, timeout=10.0)

        total = len(image.rows)
        for index, row in enumerate(image.rows, start=1):
            row_crc = crc32c(row.data)
            payload = struct.pack("<II", row.address, row_crc)

            self._send_row_payload(row.data)
            self.client.request(CMD_PROGRAM_DATA, payload, timeout=10.0)

            self._send_row_payload(row.data)
            self.client.request(CMD_VERIFY_DATA, payload, timeout=5.0)

            self.progress(index, total)
            if index == 1 or index == total or index % 16 == 0:
                self.status(f"正在烧写并校验: {index}/{total}")

        self.status("正在校验完整应用...")
        response = self.client.request(
            CMD_VERIFY_APP,
            bytes([image.app_id]),
            timeout=5.0,
        )
        if not response or response[0] != 1:
            raise DfuError("应用完整性校验失败")

        self.status("校验通过，正在启动新固件...")
        self.client.command_no_response(CMD_EXIT)
        self.progress(total, total)


class DfuUpgradeThread(QtCore.QThread):
    status_changed = Signal(str)
    progress_changed = Signal(int, int)
    completed = Signal(bool, str)

    def __init__(self, port: str, baudrate: int, image: Cyacd2Image, parent=None):
        super().__init__(parent)
        self.port_name = port
        self.baudrate = baudrate
        self.image = image

    def run(self) -> None:
        client = None
        try:
            self.status_changed.emit(
                f"打开 {self.port_name}，准备升级 {self.image.path.name}..."
            )
            client = DfuSerialClient(self.port_name, self.baudrate)
            programmer = DfuProgrammer(
                client,
                status=self.status_changed.emit,
                progress=self.progress_changed.emit,
            )
            programmer.program(self.image)
        except Exception as exc:
            self.completed.emit(False, str(exc))
        else:
            self.completed.emit(True, f"升级完成: {self.image.path.name}")
        finally:
            if client is not None:
                client.close()
