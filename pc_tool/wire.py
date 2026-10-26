"""R86 binary protocol v1. No serial/GUI dependencies; all units are explicit."""
from dataclasses import dataclass
import binascii
import math
import struct

HEADER = struct.Struct("<HBBHHI")
LENGTHS = {1: 26, 2: 26, 3: 50, 4: 38, 5: 26, 6: 30, 7: 30,
           8: 48, 9: 48, 10: 44, 11: 38, 16: 96}
WORK = ("Single M1", "Single M2", "Independent", "Sync Speed", "Sync Position", "Master M1", "Master M2")
DRIVE = ("ENCODER", "HSO", "6STEP")
MODE = ("Torque", "Speed", "Position")
INPUT = ("--", "LOCAL", "USB", "UART", "FOLLOW")
HSO = ("OFF", "WAIT FLUX", "WAIT SAMPLE", "OBSERVING", "LEARNING", "TRACKING", "INPUT RANGE",
       "TIMING", "READY", "ALIGN", "RAMP", "CAPTURE", "CONTROL", "FAULT")
SIX = ("IDLE", "ALIGN", "RAMP", "SYNC", "RUN", "FAULT")
REASONS = ("OK", "OFF", "PARAM", "SAMPLE", "SEQUENCE", "RANGE", "ENCODER", "SPEED", "ANGLE",
           "FREQUENCY", "FLUX", "NUMERIC", "TIMING", "START TIMEOUT", "STANDSTILL", "SETTLE",
           "EMF LOW", "FLUX RIPPLE", "LEARN TIMEOUT", "LOCK TIMEOUT", "PHASE DRIFT")
STREAMS = ("Auto", "M1", "M2", "FOC1", "FOC2", "ENC1", "ENC2", "HSO", "Six", "Off")


def name(table, value):
    return table[value] if 0 <= value < len(table) else str(value)


def encode(kind, payload, sequence=0, ms=0):
    data = HEADER.pack(0x5AA5, 1, kind, sequence & 65535, len(payload), ms & 0xFFFFFFFF) + payload
    return data + struct.pack("<H", binascii.crc_hqx(data, 0xFFFF))


@dataclass
class Frame:
    kind: int
    sequence: int
    ms: int
    payload: bytes
    epoch: int = 0


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = self.discarded = self.lost = self.resets = 0
        self.last = None

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while len(self.buffer) >= HEADER.size:
            if self.buffer[:2] != b"\xa5\x5a":
                found = self.buffer.find(b"\xa5\x5a", 1)
                count = found if found >= 0 else len(self.buffer) - 1
                del self.buffer[:count]
                self.discarded += count
                continue
            _, version, kind, seq, length, ms = HEADER.unpack_from(self.buffer)
            if version != 1 or LENGTHS.get(kind) != length:
                del self.buffer[0]
                self.discarded += 1
                continue
            total = HEADER.size + length + 2
            if len(self.buffer) < total:
                break
            if binascii.crc_hqx(self.buffer[:total - 2], 0xFFFF) != struct.unpack_from("<H", self.buffer, total - 2)[0]:
                del self.buffer[0]
                self.crc_errors += 1
                continue
            payload = bytes(self.buffer[HEADER.size:total - 2])
            del self.buffer[:total]
            if self.last:
                old_seq, old_ms = self.last
                if ((ms - old_ms) & 0xFFFFFFFF) > 0x80000000:
                    self.resets += 1
                else:
                    gap = (seq - old_seq - 1) & 65535
                    if gap < 32768:
                        self.lost += gap
            self.last = seq, ms
            frames.append(Frame(kind, seq, ms, payload, self.resets))
        return frames


def decode_status(frame):
    meta = frame.payload[:16]
    peak, result, ls, flux = struct.unpack_from("<4I", frame.payload, 16)
    text = [part.split(b"\0", 1)[0].decode("ascii", "replace")
            for part in struct.unpack_from("<16s16s16s16s", frame.payload, 32)]
    return {"work": name(WORK, meta[0]), "drive": name(DRIVE, meta[1]),
            "mode": [name(MODE, meta[2]), name(MODE, meta[3])],
            "input": [name(INPUT, meta[4]), name(INPUT, meta[5])], "flags": list(meta[6:8]),
            "hso": name(HSO, meta[8]), "hso_reason": name(REASONS, meta[9]),
            "six": name(SIX, meta[10]), "six_reject": meta[11], "generic": bool(meta[12] & 1),
            "stream": name(STREAMS, (meta[12] >> 1) & 15), "six_throttle": not bool(meta[12] & 32),
            "bus_fresh": bool(meta[12] & 64), "hso_position_valid": bool(meta[12] & 128),
            "encoders": [bool(meta[13] & 1), bool(meta[13] & 2)],
            "control_result": struct.unpack("b", meta[14:15])[0], "hso_result": struct.unpack("b", meta[15:16])[0],
            "max_us": peak, "detect_result": result, "detect_ls_uh": ls, "detect_flux_uwb": flux,
            "axis_status": text[:2], "command_status": text[2], "detect_status": text[3]}


def channels(frame, status=None):
    """Return (stable field ID, unit, value-or-None); invalid data is never zero."""
    status = status or {}
    kind, payload = frame.kind, frame.payload
    format_flags = struct.unpack_from("<H", payload, len(payload) - 2)[0] if kind <= 7 else 0
    specs = []
    def motion(axis):
        return [(f"m{axis}.target_speed", "rad/s"), (f"m{axis}.feedback_speed", "rad/s"),
                (f"m{axis}.target_position", "rad"), (f"m{axis}.position", "rad"), (f"m{axis}.iq", "A")]
    if kind in (1, 2):
        specs = motion(kind) + [("vbus", "V")]
    elif kind == 3:
        specs = motion(1) + motion(2) + [("vbus", "V"), ("group.error", "rad/s" if format_flags & 0x8000 else "rad")]
    elif kind == 4:
        specs = [("m1.target_speed", "rad/s"), ("hso.reference_speed", "rad/s"), ("hso.control_speed", "rad/s"),
                 ("hso.observer_speed", "rad/s"), ("m1.iq", "A"), ("vbus", "V"), ("hso.angle_error", "rad"),
                 ("hso.flux", "Wb"), ("hso.relative_position", "rad")]
    elif kind == 5:
        unit = "Hz" if format_flags & 0x4000 else "rad/s"
        specs = [("six.target", unit if format_flags & 0x8000 else "%"),
                 ("six.estimated_speed", unit), ("six.reference_speed", unit), ("six.duty", "ratio"),
                 ("six.bemf", "V"), ("vbus", "V")]
    elif kind in (6, 7):
        a = kind - 5
        loop = bool(format_flags & 0x8000)
        specs = [(f"m{a}.q_command", "A" if loop else "V"), (f"m{a}.iq", "A"), (f"m{a}.id", "A"),
                 (f"m{a}.iu", "A"), (f"m{a}.iv", "A"), (f"m{a}.feedback_speed", "rad/s"), ("vbus", "V")]
    if specs:
        values = struct.unpack_from("<" + "f" * len(specs), payload)
        valid = struct.unpack_from("<H", payload, len(specs) * 4)[0]
        return [(key, unit, value if valid & (1 << i) and math.isfinite(value) else None)
                for i, ((key, unit), value) in enumerate(zip(specs, values))]
    if kind in (8, 9):
        a = kind - 7
        counters = struct.unpack_from("<8I", payload)
        values = struct.unpack_from("<7H2B", payload, 32)
        keys = ("errors", "crc", "field", "track", "spi", "frame", "zero_start_ms", "zero_end_ms",
                "age", "max_age", "zero_before", "zero_after", "zero_count", "zero_age", "raw", "status", "last_error")
        return [(f"e{a}.{key}", "count", value) for key, value in zip(keys, (*counters, *values))]
    if kind == 10:
        values = struct.unpack("<7f3I4B", payload)
        keys = ("reference_flux", "reject_angle", "reject_hz", "reject_emf", "reject_drift", "offset", "offset_limit",
                "qualified_samples", "bad_samples", "resets", "entry_reason", "capture_reason", "position_valid", "usable")
        units = ("Wb", "rad", "Hz", "ratio", "rad", "V", "V", "count", "count", "count", "code", "code", "bool", "bool")
        return [("hso." + key, unit, value if math.isfinite(value) else None) for key, unit, value in zip(keys, units, values)]
    if kind == 11:
        values = struct.unpack("<fI13H4B", payload)
        keys = ("startup_emf", "crossings", "period_pct", "ratio_pct", "ratio_tenth", "startup_ms", "sector", "quality", "gate",
                "period_0", "period_1", "period_2", "period_3", "period_4", "period_5", "state", "sampling_off", "speed_control", "reject")
        units = ("V", "count", "%", "%", "tenth", "ms", "index", "count", "ASCII", *("10us",) * 6, "code", "bool", "bool", "code")
        return [("six." + key, unit, value if math.isfinite(value) else None) for key, unit, value in zip(keys, units, values)]
    return []
