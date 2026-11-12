import unittest
from test_telemetry import packet
from sweeper_hw.telemetry_stream import TelemetryStream


class StreamTests(unittest.TestCase):
    def test_every_split_and_consecutive_frames(self):
        first, second = packet(1).encode(), packet(2).encode()
        for index in range(1, 32):
            parser = TelemetryStream()
            self.assertEqual(parser.feed(first[:index]), [])
            self.assertEqual(parser.feed(first[index:]+second), [first, second])

    def test_noise_bad_crc_then_recovery(self):
        good = packet().encode()
        bad = bytearray(good)
        bad[-1] ^= 1
        parser = TelemetryStream()
        self.assertEqual(parser.feed(b"USB startup text\xa6\x00"+bad+good), [good])
        self.assertGreater(parser.invalid_frames, 0)
        self.assertLess(len(parser.buffer), 32)

    def test_large_noise_is_bounded(self):
        parser = TelemetryStream()
        self.assertEqual(parser.feed(bytes([0xa6, 0x11])*10000), [])
        self.assertLess(len(parser.buffer), 32)

    def test_test_markers_do_not_become_measurements(self):
        parser = TelemetryStream()
        self.assertEqual(parser.feed(b"IMU012345000!ashining"), [])


if __name__ == "__main__":
    unittest.main()
