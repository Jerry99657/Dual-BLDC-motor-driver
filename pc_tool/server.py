"""Local-only motor console. USB CDC and UART adapters both use serial COM ports."""
import argparse
from collections import deque
import csv
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import struct
import threading
import time
from urllib.parse import parse_qs, urlsplit
import webbrowser

from wire import Parser, channels, decode_status, encode

ROOT = Path(__file__).resolve().parent


class Hub:
    def __init__(self):
        self.lock = threading.RLock()
        self.connection_lock = threading.RLock()
        self.serial = None
        self.worker = None
        self.stop = threading.Event()
        self.parser = Parser()
        self.frames = deque(maxlen=2048)
        self.cursor = 0
        self.epoch = 0
        self.status = {}
        self.connected = ""
        self.error = ""
        self.received = self.last_rx = 0
        self.record = None
        self.csv = None

    def accept(self, data):
        with self.lock:
            for frame in self.parser.feed(data):
                if frame.epoch != self.epoch:
                    self.status = {}
                    self.epoch = frame.epoch
                if frame.kind == 16:
                    self.status = decode_status(frame)
                self.cursor += 1
                samples = channels(frame, self.status)
                item = {"id": self.cursor, "seq": frame.sequence, "ms": frame.ms, "kind": frame.kind,
                        "values": samples, "epoch": frame.epoch}
                self.frames.append(item)
                self.received += 1
                self.last_rx = time.monotonic()
                if self.csv:
                    stamp = datetime.now().isoformat(timespec="milliseconds")
                    if frame.kind == 16:
                        self.csv.writerow([stamp, frame.ms, frame.sequence, frame.epoch, frame.kind,
                                           "status", "json", json.dumps(self.status, ensure_ascii=False)])
                    for key, unit, value in samples:
                        self.csv.writerow([stamp, frame.ms, frame.sequence, frame.epoch, frame.kind,
                                           key, unit, "" if value is None else value])

    def disconnect(self):
        with self.connection_lock:
            self._disconnect()

    def _disconnect(self):
        self.stop.set()
        with self.lock:
            port, self.serial = self.serial, None
            self.connected = ""
        if port:
            port.close()
        if self.worker:
            self.worker.join(timeout=1)
            self.worker = None
        self.end_record()

    def connect(self, port):
        with self.connection_lock:
            self._connect(port)

    def _connect(self, port):
        import serial
        self.disconnect()
        # Prepare line states before opening; never transmit a motion command.
        link = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.25,
                             xonxoff=False, rtscts=False, dsrdtr=False)
        link.dtr = link.rts = False
        link.port = port
        link.open()
        with self.lock:
            self.serial = link
            self.parser = Parser()
            self.epoch = 0
            self.frames.clear()
            self.status = {}
            self.error = ""
            self.connected = port
            self.received = self.last_rx = 0
        self.stop.clear()
        def read():
            while not self.stop.is_set():
                try:
                    data = link.read(max(1, min(link.in_waiting, 4096)))
                    if data:
                        self.accept(data)
                except (OSError, serial.SerialException) as exc:
                    if not self.stop.is_set():
                        with self.lock:
                            self.error = str(exc)
                            self.connected = ""
                    break
        self.worker = threading.Thread(target=read, daemon=True)
        self.worker.start()

    def send(self, command):
        data = command.encode("ascii")
        if not data or len(data) > 63 or any(c in data for c in (b"\0", b"\r", b"\n")):
            raise ValueError("命令必须为 1～63 字节的单行 ASCII")
        with self.lock:
            if not self.serial or not self.connected:
                raise ValueError("尚未连接开发板")
            if self.serial.write(data + b"\n") != len(data) + 1:
                raise OSError("命令未完整写出，请检查连接")

    def begin_record(self):
        with self.lock:
            if not self.connected:
                raise ValueError("请先连接开发板")
            if self.record:
                return str(self.record.name)
            directory = ROOT / "records"
            directory.mkdir(exist_ok=True)
            path = directory / (datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".csv")
            self.record = path.open("w", encoding="utf-8-sig", newline="")
            self.csv = csv.writer(self.record)
            self.csv.writerow(["host_time", "board_ms", "sequence", "epoch", "packet_kind", "field", "unit", "value"])
            return str(path)

    def end_record(self):
        with self.lock:
            if self.record:
                self.record.close()
                self.record = self.csv = None

    def poll(self, after):
        with self.lock:
            if self.record:
                self.record.flush()
            frames = [f for f in self.frames if f["id"] > after][:256]
            return {"frames": frames, "status": self.status, "connected": self.connected, "error": self.error,
                    "received": self.received, "crc_errors": self.parser.crc_errors, "discarded": self.parser.discarded,
                    "lost": self.parser.lost, "resets": self.parser.resets,
                    "age_ms": round((time.monotonic() - self.last_rx) * 1000) if self.last_rx else None,
                    "recording": str(self.record.name) if self.record else "",
                    "overrun": bool(self.frames and after and after < self.frames[0]["id"] - 1)}


def handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, data, code=200):
            raw = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            path = urlsplit(self.path)
            try:
                if path.path == "/api/ports":
                    from serial.tools import list_ports
                    self.reply({"ports": [{"port": p.device, "description": p.description} for p in list_ports.comports()]})
                elif path.path == "/api/poll":
                    self.reply(hub.poll(int(parse_qs(path.query).get("after", [0])[0])))
                elif path.path == "/":
                    raw = (ROOT / "index.html").read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(raw)))
                    self.end_headers()
                    self.wfile.write(raw)
                else:
                    self.reply({"error": "Not found"}, 404)
            except (ValueError, OSError) as exc:
                self.reply({"error": str(exc)}, 400)

        def do_POST(self):
            origin = self.headers.get("Origin", "")
            host = self.headers.get("Host", "")
            if host not in (f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}") or (
                    origin and origin != "http://" + host) or self.headers.get("Content-Type") != "application/json":
                self.reply({"error": "Local JSON requests only"}, 403)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 1024:
                    raise ValueError("Invalid request length")
                args = json.loads(self.rfile.read(length))
                if self.path == "/api/connect":
                    hub.connect(str(args["port"]))
                elif self.path == "/api/disconnect":
                    hub.disconnect()
                elif self.path == "/api/send":
                    hub.send(str(args["command"]))
                elif self.path == "/api/record":
                    if args.get("start"):
                        self.reply({"file": hub.begin_record()})
                        return
                    hub.end_record()
                else:
                    self.reply({"error": "Not found"}, 404)
                    return
                self.reply({"ok": True})
            except (ValueError, KeyError, OSError) as exc:
                self.reply({"error": str(exc)}, 400)
    return Handler


def demo(hub):
    tick = seq = 0
    while not hub.stop.wait(0.01):
        t = tick / 1000
        values = (10 * math.sin(t), 9.5 * math.sin(t - 0.06), float("nan"), math.sin(t), 0.12 * math.sin(t), 20.0)
        if seq % 10 == 0:
            meta = bytes((0, 0, 1, 0, 3, 0, 223, 0, 0, 0, 0, 0, 64, 1, 1, 0))
            payload = meta + struct.pack("<4I16s16s16s16s", 53, 0, 0, 0, b"READY", b"DISABLED", b"DEMO", b"READY")
            hub.accept(encode(16, payload, seq, tick))
        else:
            hub.accept(encode(1, struct.pack("<6fH", *values, 0x3B), seq, tick))
        tick += 10
        seq += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765, help="Local HTTP port, not the serial COM number")
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--demo", action="store_true", help="Simulated curves only; no board commands")
    args = parser.parse_args()
    hub = Hub()
    if args.demo:
        hub.connected = "DEMO (无硬件)"
        hub.worker = threading.Thread(target=demo, args=(hub,), daemon=True)
        hub.worker.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler(hub))
    url = f"http://127.0.0.1:{server.server_port}"
    print("Motor console:", url, flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        hub.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
