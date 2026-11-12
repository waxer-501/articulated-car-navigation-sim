"""Version-1, 32-byte, data-only IMU telemetry; no control message type.

rx_time_us is the MC02 UART frame-completion counter, NOT the IMU's internal
sample clock and NOT a Raspberry Pi timestamp. See docs/telemetry_v1.md.
"""
from dataclasses import dataclass
import math
import struct
from .imu406 import Sample, STANDARD_GRAVITY

FORMAT = struct.Struct("<BBHII9hH")
MAGIC = 0xA6
VERSION = 1
FRONT = 1
REAR = 2


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, no reflection/xorout."""
    value = 0xffff
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xffff
    return value


@dataclass(frozen=True)
class Packet:
    sensor_id: int
    sequence: int
    rx_time_us: int
    boot_id: int
    values: tuple[int, ...]  # Euler centideg, accel milli-g, gyro centideg/s

    @classmethod
    def from_sample(cls, sample: Sample, sensor_id: int, sequence: int, rx_time_us: int, boot_id: int):
        values = tuple(round(v*100) for v in sample.euler_deg)
        values += tuple(round(v*1000) for v in sample.accel_g)
        values += tuple(round(v*100) for v in sample.gyro_deg_s)
        return cls(sensor_id, sequence, rx_time_us, boot_id, values)

    def encode(self) -> bytes:
        if self.sensor_id not in (FRONT, REAR) or len(self.values) != 9:
            raise ValueError("Expected front/rear identity and nine measurements")
        if not 0 <= self.sequence <= 0xffff or not 0 <= self.rx_time_us <= 0xffffffff or not 0 <= self.boot_id <= 0xffffffff:
            raise ValueError("Sequence/time/session outside wire range")
        if any(not isinstance(v, int) or not -32768 <= v <= 32767 for v in self.values):
            raise ValueError("Measurement outside int16 range; never wrap or saturate")
        data = FORMAT.pack(MAGIC, (VERSION << 4) | self.sensor_id, self.sequence,
                           self.rx_time_us, self.boot_id, *self.values, 0)
        return data[:30]+struct.pack("<H", crc16(data[:30]))

    @classmethod
    def decode(cls, data: bytes):
        if len(data) != 32:
            raise ValueError("Telemetry must be exactly 32 bytes")
        fields = FORMAT.unpack(data)
        if fields[0] != MAGIC or fields[1] >> 4 != VERSION or fields[1] & 15 not in (FRONT, REAR):
            raise ValueError("Unknown telemetry format or sensor ID")
        if crc16(data[:30]) != fields[-1]:
            raise ValueError("Telemetry CRC mismatch")
        return cls(fields[1] & 15, fields[2], fields[3], fields[4], tuple(fields[5:14]))

    def measurements(self):
        return {"euler_deg": [v/100 for v in self.values[:3]],
                "accel_m_s2": [v*STANDARD_GRAVITY/1000 for v in self.values[3:6]],
                "gyro_rad_s": [math.radians(v/100) for v in self.values[6:]]}


class ReceiverState:
    """Per-sensor receive status. Never pair unsynchronised samples or fake time.

    Consumers receive every accepted sample separately and keep both MCU time
    and host arrival time. Age below means age since host receipt, not latency.
    """
    def __init__(self, stale_after_s=2.0):
        if not math.isfinite(stale_after_s) or stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive")
        self.stale_after_s = stale_after_s
        self.boot_id = None
        self.retired_boots = set()
        self.latest = {}
        self.counts = {"accepted": 0, "invalid": 0, "duplicates": 0, "out_of_order": 0,
                       "sequence_gaps": 0, "boot_changes": 0, "retired_boot_packets": 0}

    def accept(self, raw, host_received_s):
        if not math.isfinite(host_received_s) or host_received_s < 0:
            raise ValueError("Expected finite host monotonic receive time")
        try:
            packet = Packet.decode(raw)
        except ValueError:
            self.counts["invalid"] += 1
            return None
        if packet.boot_id in self.retired_boots:
            self.counts["retired_boot_packets"] += 1
            return None
        if self.boot_id != packet.boot_id:
            if self.boot_id is not None:
                self.retired_boots.add(self.boot_id)
                self.counts["boot_changes"] += 1
            self.boot_id = packet.boot_id
            self.latest.clear()
        old = self.latest.get(packet.sensor_id)
        gaps = 0
        discontinuity = False
        if old is not None:
            delta = (packet.sequence-old["sequence"]) & 0xffff
            if delta == 0 and packet.rx_time_us == old["mcu_uart_rx_time_us"]:
                self.counts["duplicates"] += 1
                return None
            host_gap = host_received_s-old["host_received_monotonic_s"]
            if host_gap < 0 or (host_gap <= self.stale_after_s and (delta == 0 or delta >= 0x8000)):
                self.counts["out_of_order"] += 1
                return None
            discontinuity = host_gap > self.stale_after_s
            gaps = None if discontinuity else delta-1
        row = {"sensor": "front" if packet.sensor_id == FRONT else "rear", "sensor_id": packet.sensor_id,
               "sequence": packet.sequence, "mcu_uart_rx_time_us": packet.rx_time_us,
               "boot_id": packet.boot_id, "host_received_monotonic_s": host_received_s,
               "sequence_gap": gaps, "stream_discontinuity": discontinuity,
               "axes": "sensor; mounting transform not applied",
               **packet.measurements()}
        self.latest[packet.sensor_id] = row
        self.counts["sequence_gaps"] += gaps or 0
        self.counts["accepted"] += 1
        return dict(row)

    def status(self, host_now_s):
        if not math.isfinite(host_now_s):
            raise ValueError("Invalid host monotonic time")
        sensors = {}
        for sensor_id, name in ((FRONT, "front"), (REAR, "rear")):
            row = self.latest.get(sensor_id)
            age = None if row is None else host_now_s-row["host_received_monotonic_s"]
            sensors[name] = {"present": row is not None, "age_since_receive_s": age,
                             "fresh": age is not None and 0 <= age <= self.stale_after_s}
        return {"boot_id": self.boot_id, "sensors": sensors, "counts": dict(self.counts)}
