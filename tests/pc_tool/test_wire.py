"""Cross-language wire/transport tests. Run after the real C telemetry fixture."""
import binascii
import io
import json
from pathlib import Path
import random
import struct
import sys
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pc_tool"))
from wire import Parser, channels, decode_status, encode, LENGTHS
from server import Hub, handler


class WireTests(unittest.TestCase):
    def test_real_c_frames_fragmentation_and_integer_precision(self):
        data = (ROOT / "build/telemetry-r86/wire-fixture.bin").read_bytes()
        reference = Parser().feed(data)
        self.assertEqual({f.kind for f in reference}, set(LENGTHS))
        for seed in range(10):
            randomizer = random.Random(seed)
            parser, frames, index = Parser(), [], 0
            while index < len(data):
                count = randomizer.randrange(1, 151)
                frames += parser.feed(data[index:index+count])
                index += count
            self.assertEqual(frames, reference)
            self.assertEqual(parser.crc_errors, 0)
        status = {}
        for frame in reference:
            if frame.kind == 16:
                status = decode_status(frame)
            values = channels(frame, status)
            self.assertTrue(all(v is None or isinstance(v, (int, float)) for _, _, v in values))
            if frame.kind == 8:
                self.assertEqual(values[0][2], 16777217)  # No float32 counter rounding.
            if frame.kind == 5:
                self.assertIsNone(values[1][2])  # Idle is not a measured zero speed.

    def test_noise_corruption_lengths_versions_and_resync(self):
        good = encode(1, struct.pack("<6fH", 1, 2, 3, 4, 5, 20, 0x3F), 7, 100)
        for index in range(len(good)):
            bad = bytearray(good)
            bad[index] ^= 0x40
            parser = Parser()
            found = parser.feed(b"garbage" + bad + good)
            self.assertEqual(len(found), 1)
            self.assertEqual(found[0].sequence, 7)
            self.assertLess(len(parser.buffer), 12)
        parser = Parser()
        for _ in range(20):
            self.assertFalse(parser.feed(bytes(8192)))
            self.assertLess(len(parser.buffer), 12)

    def test_frame_flags_define_units_without_status(self):
        for flags, target, speed in ((0x3F, "%", "rad/s"), (0x803F, "rad/s", "rad/s"), (0x403F, "%", "Hz")):
            frame = Parser().feed(encode(5, struct.pack("<6fH", 1, 2, 3, 4, 5, 6, flags)))[0]
            values = channels(frame)
            self.assertEqual(values[0][1], target)
            self.assertEqual(values[1][1], speed)
        for flags, unit in ((0x7F, "V"), (0x807F, "A")):
            frame = Parser().feed(encode(6, struct.pack("<7fH", *range(7), flags)))[0]
            self.assertEqual(channels(frame)[0][1], unit)

    def test_invalid_values_wrap_gap_reset(self):
        payload = struct.pack("<6fH", 1, float("nan"), 3, 4, 5, 6, 0x3D)
        parser = Parser()
        frames = parser.feed(encode(1, payload, 65535, 0xFFFFFFFE)+encode(1, payload, 0, 2)+encode(1, payload, 3, 20))
        self.assertEqual(parser.lost, 2)
        self.assertEqual(parser.resets, 0)
        self.assertIsNone(channels(frames[0])[1][2])
        parser.feed(encode(1, payload, 0, 0))
        self.assertEqual(parser.resets, 1)
        self.assertEqual(binascii.crc_hqx(b"123456789", 0xFFFF), 0x29B1)


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.hub = Hub()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler(self.hub))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.hub.disconnect()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def post(self, path, data, origin=None):
        headers = {"Content-Type": "application/json"}
        if origin:
            headers["Origin"] = origin
        return urlopen(Request(self.url+path, json.dumps(data).encode(), headers=headers), timeout=3)

    def test_local_ui_api_and_manual_commands(self):
        class Port:
            def __init__(self): self.sent = []
            def write(self, b): self.sent.append(b); return len(b)
            def close(self): pass
        port = Port()
        self.hub.serial, self.hub.connected = port, "TEST"
        self.assertIn(b"R88", urlopen(self.url).read())
        self.assertEqual(port.sent, [])  # Opening the UI sends nothing to a motor.
        self.post("/api/send", {"command": "M2:Speed:10"}).close()
        self.post("/api/send", {"command": "Stop"}).close()
        self.assertEqual(port.sent, [b"M2:Speed:10\n", b"Stop\n"])
        for command in ("Stop\nSpeed:10", "a"*64, "中文", ""):
            with self.assertRaises(HTTPError): self.post("/api/send", {"command": command})
        with self.assertRaises(HTTPError):
            self.post("/api/send", {"command": "Stop"}, "http://example.com")
        self.assertEqual(len(port.sent), 2)
        raw = (ROOT / "build/telemetry-r86/wire-fixture.bin").read_bytes()
        self.hub.record = io.StringIO()
        self.hub.record.name = "memory.csv"
        import csv
        self.hub.csv = csv.writer(self.hub.record)
        self.hub.accept(raw)
        response = json.load(urlopen(self.url+"/api/poll?after=0"))
        self.assertTrue(response["frames"])
        self.assertIsNotNone(response["status"].get("work"))
        self.assertIn("16777217", self.hub.record.getvalue())
        self.assertIn("status,json", self.hub.record.getvalue())


if __name__ == "__main__":
    unittest.main()
