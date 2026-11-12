"""Receive-only IMU406 V1.5 protocol parsing, based on manual pages 9-10.

The manual's '9-axis output' means Euler + accelerometer + gyroscope fields,
not nine independent sensing axes. No timestamp or sensor identity beyond
the configured address is present in this wire format. We never invent either.
"""
from dataclasses import dataclass
import math

STANDARD_GRAVITY = 9.80665


class FrameError(ValueError):
    pass


class UnsupportedOutput(FrameError):
    pass


def signed_bcd3(data: bytes, decimal_places: int) -> float:
    """Sign nibble 0/1 followed by five decimal digits; not int24 or IEEE float."""
    if len(data) != 3 or decimal_places not in (2, 3):
        raise FrameError("Expected three BCD bytes and a documented scale")
    sign = data[0] >> 4
    digits = (data[0] & 15, data[1] >> 4, data[1] & 15, data[2] >> 4, data[2] & 15)
    if sign not in (0, 1) or any(d > 9 for d in digits):
        raise FrameError("Invalid signed BCD value")
    number = 0
    for digit in digits:
        number = 10*number+digit
    return (-1 if sign else 1)*number/(10**decimal_places)


@dataclass(frozen=True)
class Frame:
    address: int
    command: int
    payload: bytes
    raw: bytes


@dataclass(frozen=True)
class Sample:
    address: int
    euler_deg: tuple[float, float, float]
    accel_g: tuple[float, float, float]
    gyro_deg_s: tuple[float, float, float]

    def si_vectors(self) -> dict:
        """Vectors in the sensor's own axes. Mount transform is NOT assumed."""
        return {"accel": [v*STANDARD_GRAVITY for v in self.accel_g],
                "gyro": [math.radians(v) for v in self.gyro_deg_s]}


def parse_frame(raw: bytes) -> Frame:
    raw = bytes(raw)
    if len(raw) < 5 or raw[0] != 0x68:
        raise FrameError("Invalid header/minimum length")
    if raw[1] < 4 or len(raw) != raw[1]+1:
        raise FrameError("Length includes length byte through checksum, excluding 0x68")
    if sum(raw[1:-1]) & 255 != raw[-1]:
        raise FrameError("Checksum mismatch")
    return Frame(raw[2], raw[3], raw[4:-1], raw)


def decode_sample(frame: Frame) -> Sample:
    if frame.command != 0x84 or len(frame.raw) != 32 or len(frame.payload) != 27:
        raise UnsupportedOutput("Requires the documented 0x84 / length 0x1F Euler+accel+gyro output")
    values = []
    for i in range(9):
        values.append(signed_bcd3(frame.payload[3*i:3*i+3], 3 if 3 <= i < 6 else 2))
    return Sample(frame.address, tuple(values[:3]), tuple(values[3:6]), tuple(values[6:]))


class StreamDecoder:
    """Bounded, fragmented/coalesced byte-stream parser; corrupt frames are discarded.

    Supports known frames up to 32 bytes. Other factory output layouts require
    explicit documentation and are not silently decoded as this layout.
    """
    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.checksum_errors = 0
        self.length_errors = 0

    def feed(self, chunk: bytes) -> list[Frame]:
        frames = []
        # Process incrementally so a very large input cannot grow the buffer.
        for value in chunk:
            self.buffer.append(value)
            while self.buffer:
                if self.buffer[0] != 0x68:
                    del self.buffer[0]
                    self.discarded_bytes += 1
                    continue
                if len(self.buffer) < 2:
                    break
                length = self.buffer[1]
                if not 4 <= length <= 31:
                    self.length_errors += 1
                    self.discarded_bytes += 1
                    del self.buffer[0]
                    continue
                if len(self.buffer) < length+1:
                    break
                candidate = bytes(self.buffer[:length+1])
                try:
                    frame = parse_frame(candidate)
                except FrameError:
                    self.checksum_errors += 1
                    self.discarded_bytes += 1
                    del self.buffer[0]
                    continue
                del self.buffer[:length+1]
                frames.append(frame)
        return frames
