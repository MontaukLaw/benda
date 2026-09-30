"""RAW20线协议测试：分包、错位及损坏恢复，不依赖串口硬件。"""

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "PY_SRIPT"))
from raw20_serial import FRAME_SIZE, FrameDecoder, HEADER


def encode(values):
    payload = struct.pack("<20H", *values)
    return HEADER + payload + struct.pack("<H", sum(payload))


class Raw20DecoderTest(unittest.TestCase):
    def setUp(self):
        self.values = (0, 65535, 0x55AA, 0x55AA, *range(16))
        self.frame = encode(self.values)

    def test_known_wire_vector(self):
        frame = HEADER + b"\xff\xff" * 20 + b"\xd8\x27"
        self.assertEqual(len(frame), 46)
        self.assertEqual(FrameDecoder().feed(frame), [(65535,) * 20])

    def test_every_possible_split(self):
        for cut in range(FRAME_SIZE + 1):
            decoder = FrameDecoder()
            result = decoder.feed(self.frame[:cut]) + decoder.feed(self.frame[cut:])
            self.assertEqual(result, [self.values], cut)

    def test_bytewise_noise_and_consecutive_frames(self):
        decoder = FrameDecoder()
        result = []
        for byte in b"noise\xaa\x55\xaa" + self.frame * 3:
            result.extend(decoder.feed(bytes([byte])))
        self.assertEqual(result, [self.values] * 3)

    def test_corruption_insertion_and_deletion_recover(self):
        for offset in range(FRAME_SIZE):
            corrupt = bytearray(self.frame)
            corrupt[offset] ^= 1
            for damaged in (
                corrupt,
                self.frame[:offset] + self.frame[offset + 1:],
                self.frame[:offset] + b"\x7e" + self.frame[offset:],
            ):
                decoder = FrameDecoder()
                decoded = decoder.feed(damaged + self.frame * 2)
                self.assertEqual(decoded[-2:], [self.values] * 2, offset)

    def test_noise_does_not_accumulate(self):
        decoder = FrameDecoder()
        self.assertEqual(decoder.feed(b"x" * 100000), [])
        self.assertLessEqual(len(decoder.buffer), len(HEADER) - 1)
        self.assertEqual(decoder.feed(self.frame), [self.values])


if __name__ == "__main__":
    unittest.main()
