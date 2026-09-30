"""接收RAW20固件串口帧，逐行打印RX0/TX0至RX4/TX3的20个原始值。"""

import argparse
import struct
import sys

HEADER = b"\xaa\x55\xaa\x55"
PAYLOAD_SIZE = 40
FRAME_SIZE = len(HEADER) + PAYLOAD_SIZE + 2
RAW_VALUES = struct.Struct("<20H")


class FrameDecoder:
    """处理半帧、粘包及噪声；校验失败逐字节搜索下一帧。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                # 保留可能属于跨次读取帧头的末尾三个字节。
                del self.buffer[: max(0, len(self.buffer) - len(HEADER) + 1)]
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            payload = self.buffer[4:44]
            checksum = int.from_bytes(self.buffer[44:46], "little")
            if sum(payload) & 0xFFFF != checksum:
                del self.buffer[0]
                continue
            frames.append(RAW_VALUES.unpack(payload))
            del self.buffer[:FRAME_SIZE]
        return frames


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="串口名称，例如 COM3")
    parser.add_argument("--baud", type=int, default=1000000, help="波特率，默认1000000")
    parser.add_argument("--list-ports", action="store_true", help="列出串口后退出")
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("波特率必须大于0")
    if not args.list_ports and not args.port:
        parser.error("请指定串口，例如 COM3，或使用 --list-ports")
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("缺少pyserial，请运行: python -m pip install pyserial", file=sys.stderr)
        return 1
    if args.list_ports:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
        return 0
    decoder = FrameDecoder()
    try:
        with serial.Serial(
            args.port, args.baud, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=0.2, xonxoff=False, rtscts=False, dsrdtr=False,
        ) as connection:
            print(
                f"已打开 {args.port}，{args.baud} 8N1；每行20点，"
                "顺序RX0/TX0..TX3，RX1/TX0..TX3，...，RX4/TX0..TX3。Ctrl+C退出。",
                file=sys.stderr,
            )
            while True:
                data = connection.read(min(connection.in_waiting, 4096) or 1)
                for values in decoder.feed(data):
                    print(" ".join(map(str, values)), flush=True)
    except KeyboardInterrupt:
        print("\n已停止接收，串口已关闭。", file=sys.stderr)
        return 0
    except (serial.SerialException, OSError) as exc:
        print(f"串口错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
