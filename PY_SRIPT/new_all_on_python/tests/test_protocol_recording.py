import csv
import json
from pathlib import Path
import struct
import tempfile
import time
import unittest

from new_all_on_python.config import AppConfig
from new_all_on_python.data_recorder import CsvRecorder
from new_all_on_python.protocol_bridge import ProtocolDecoder, command
from new_all_on_python.touch_tracker import TouchTracker


def packet(kind, payload, version=21, channels=20):
    return struct.pack("<4sBBBBH", b"DATA", version, kind, 1, channels, len(payload)) + payload


def debug_packet(version):
    payload = struct.pack("<20H", *range(12000, 12020))
    if version < 20:
        payload += struct.pack("<40i", *range(40))
    payload += struct.pack("<40H", *([11000] * 20 + [1000] * 20))
    payload += struct.pack("<40i", *([-123] * 20 + [456] * 20))
    if version < 20:
        payload += struct.pack("<20H", *([1] * 20))
    payload += struct.pack("<20H", *range(20))
    return packet(0x10, payload, version)


def raw20_packet(values):
    payload = struct.pack("<20H", *values)
    return b"\xaa\x55\xaa\x55" + payload + struct.pack("<H", sum(payload) & 0xFFFF)


class ProtocolTests(unittest.TestCase):
    def test_v21_raw_bytewise_and_multiple_frames(self):
        decoder = ProtocolDecoder()
        data = packet(0x16, struct.pack("<20H", *range(20)))
        frames = []
        for value in data:
            frames.extend(decoder.feed(bytes([value])))
        self.assertEqual(frames[0]["raw"], tuple(range(20)))
        self.assertTrue(frames[0]["raw_only"])
        self.assertEqual(len(decoder.feed(data * 8)), 8)
        self.assertEqual(decoder.parse_errors, 0)

    def test_debug_v14_to_v21_reuses_correct_layout_and_states(self):
        for version in range(14, 22):
            frame = ProtocolDecoder().feed(debug_packet(version))[0]
            self.assertEqual(frame["raw"], tuple(range(12000, 12020)))
            self.assertEqual(frame["state"], tuple(range(20)))
            self.assertEqual(frame["baseline"], (11000,) * 20)
            self.assertEqual(frame["signal"], (1000,) * 20)
            self.assertEqual(frame["fast_k_percent"], (-1.23,) * 20)

    def test_wrong_length_noise_header_and_partial_timeout_recover(self):
        good = packet(0x16, struct.pack("<20H", *range(20)))
        bad = packet(0x16, b"\x00" * 38)
        decoder = ProtocolDecoder()
        frames = decoder.feed(b"noise" + bad + good)
        self.assertEqual(len(frames), 1)
        self.assertEqual(decoder.parse_errors, 1)
        self.assertGreater(decoder.discarded_bytes, 5)
        bogus = struct.pack("<4sBBBBH", b"DATA", 21, 0x77, 1, 0, 1000)
        decoder.feed(bogus + good, now=0)
        frames = decoder.feed(b"", now=0.6)
        self.assertEqual(frames[0]["raw"], tuple(range(20)))
        self.assertEqual(decoder.parse_errors, 2)

    def test_raw20_checksum_split_and_noise_recovery(self):
        good = raw20_packet(range(20))
        decoder = ProtocolDecoder("RAW20")
        self.assertEqual(decoder.feed(good[:20]), [])
        self.assertEqual(decoder.feed(good[20:])[0]["raw"], tuple(range(20)))
        bad = bytearray(good)
        bad[-1] ^= 1
        frames = decoder.feed(b"bad" + bad + good * 2)
        self.assertEqual(len(frames), 2)
        self.assertEqual(decoder.parse_errors, 1)

    def test_signal_and_diagnostic_frames_are_not_raw_samples(self):
        decoder = ProtocolDecoder()
        frames = decoder.feed(packet(0x01, struct.pack("<20H", *([3000] * 20)))
                              + packet(0x03, struct.pack("<h", 250), channels=0))
        self.assertEqual(frames[0]["type"], "signal")
        self.assertNotIn("raw", frames[0])
        self.assertEqual(frames[1]["temperature_celsius"], 25)
        self.assertEqual(command(14, 0x82, b"\x01"), b"DATA\x0e\x82\x00\x00\x01\x00\x01")

    def test_bad_diagnostic_payload_is_counted_and_next_raw_survives(self):
        decoder = ProtocolDecoder()
        frames = decoder.feed(packet(0x03, b"\x00", channels=0)
                              + packet(0x16, struct.pack("<20H", *range(20))))
        self.assertEqual(decoder.parse_errors, 1)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["raw"], tuple(range(20)))

    def test_csv_complete_tail_and_parameter_events(self):
        tracker = TouchTracker()
        recorder = CsvRecorder()
        with tempfile.TemporaryDirectory() as folder:
            recorder.start(AppConfig(), 0, folder)
            for n in range(100):
                raw = [12000] * 20
                if n > 70:
                    raw[10] += 8000
                r = tracker.update(raw, n / 100)
                recorder.record(r, {"protocol": "DATA", "raw_only": True}, "Serial", 0)
            recorder.event("reset_tracker")
            self.assertTrue(recorder.finish())
            self.assertFalse(recorder.error)
            with recorder.path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 100)
            self.assertEqual(rows[-1]["max_index"], "10")
            self.assertEqual(rows[-1]["firmware_state0"], "")
            self.assertIn("+", rows[-1]["timestamp"])
            events = [json.loads(line) for line in recorder.path.with_suffix(".events.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual(events[0]["kind"], "recording_start")
            self.assertEqual(events[-1]["rows_written"], 100)
            self.assertIn("reset_tracker", [event["kind"] for event in events])


if __name__ == "__main__":
    unittest.main()
