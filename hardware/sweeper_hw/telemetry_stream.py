"""CRC-framed telemetry recovered from a USB serial byte stream."""
from .telemetry import MAGIC, Packet


class TelemetryStream:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.invalid_frames = 0

    def feed(self, chunk):
        frames = []
        for byte in chunk:
            self.buffer.append(byte)
            while self.buffer:
                if self.buffer[0] != MAGIC:
                    self.buffer.pop(0)
                    self.discarded_bytes += 1
                    continue
                if len(self.buffer) >= 2 and self.buffer[1] not in (0x11, 0x12):
                    self.buffer.pop(0)
                    self.discarded_bytes += 1
                    continue
                if len(self.buffer) < 32:
                    break
                raw = bytes(self.buffer[:32])
                try:
                    Packet.decode(raw)
                except ValueError:
                    self.buffer.pop(0)
                    self.discarded_bytes += 1
                    self.invalid_frames += 1
                else:
                    frames.append(raw)
                    del self.buffer[:32]
        return frames
