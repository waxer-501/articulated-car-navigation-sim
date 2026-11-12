import ctypes
import math
from pathlib import Path
import random
import subprocess
import tempfile
import unittest

from sweeper_hw.imu406 import (FrameError, UnsupportedOutput, StreamDecoder,
                              decode_sample, parse_frame, signed_bcd3)

ROOT = Path(__file__).resolve().parents[1]


def example_frame(address=0):
    # Manual page 10's nine field examples assembled into a synthetic frame.
    payload = bytes.fromhex("01 60 00 10 50 23 11 60 00 00 23 04 10 23 04 10 23 04 10 50 23 01 80 00 00 50 23")
    body = bytes([0x1f, address, 0x84])+payload
    return b"\x68"+body+bytes([sum(body) & 255])


class DecoderTests(unittest.TestCase):
    def test_manual_bcd_and_units(self):
        sample = decode_sample(parse_frame(example_frame()))
        self.assertEqual(sample.euler_deg, (160., -50.23, -160.))
        self.assertEqual(sample.accel_g, (2.304, -2.304, -2.304))
        self.assertEqual(sample.gyro_deg_s, (-50.23, 180., 50.23))
        self.assertAlmostEqual(sample.si_vectors()["accel"][0], 2.304*9.80665)
        self.assertAlmostEqual(sample.si_vectors()["gyro"][1], math.pi)

    def test_all_fragment_boundaries(self):
        raw = example_frame()
        for split in range(1, len(raw)):
            parser = StreamDecoder()
            self.assertEqual(parser.feed(raw[:split]), [])
            frames = parser.feed(raw[split:])
            self.assertEqual([f.raw for f in frames], [raw])

    def test_random_chunks_and_coalesced_frames(self):
        stream = b"".join(example_frame(i % 2) for i in range(100))
        parser, rng, frames = StreamDecoder(), random.Random(7), []
        while stream:
            length = rng.randint(1, 75)
            frames.extend(parser.feed(stream[:length]))
            stream = stream[length:]
        self.assertEqual(len(frames), 100)
        self.assertEqual([f.address for f in frames], [i % 2 for i in range(100)])

    def test_noise_corruption_and_resynchronization(self):
        raw = example_frame()
        corrupt = bytearray(raw)
        corrupt[-1] ^= 1
        parser = StreamDecoder()
        frames = parser.feed(b"noise\x68\xff"+corrupt+raw)
        self.assertEqual([f.raw for f in frames], [raw])
        self.assertGreater(parser.checksum_errors, 0)
        self.assertGreater(parser.length_errors, 0)

    def test_embedded_header_byte_is_not_a_new_frame(self):
        raw = bytearray(example_frame())
        raw[6] = 0x68  # valid BCD .68, equal to frame marker
        raw[-1] = sum(raw[1:-1]) & 255
        frames = StreamDecoder().feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(decode_sample(frames[0]).euler_deg[0], 160.68)

    def test_invalid_signed_bcd_rejected(self):
        for value in ("20 00 00", "0a 00 00", "00 f0 00", "00 00 0f"):
            with self.assertRaises(FrameError):
                signed_bcd3(bytes.fromhex(value), 2)

    def test_incomplete_frame_not_published(self):
        parser = StreamDecoder()
        self.assertEqual(parser.feed(example_frame()[:-1]), [])
        self.assertEqual(len(parser.buffer), 31)

    def test_lengths_and_checksum_not_silently_accepted(self):
        for raw in (example_frame()[:-1], example_frame()+b"\0", b"\x68\x03\0\x84\x87"):
            with self.assertRaises(FrameError):
                parse_frame(raw)

    def test_ack_does_not_become_zero_imu_sample(self):
        frame = parse_frame(bytes.fromhex("68 05 00 8C 00 91"))
        self.assertEqual(frame.command, 0x8c)
        with self.assertRaises(UnsupportedOutput):
            decode_sample(frame)

    def test_documented_address_ack_checksum_inconsistency(self):
        # Page 9 prints 0x14, but its own checksum rule gives 0x94.
        with self.assertRaises(FrameError):
            parse_frame(bytes.fromhex("68 05 00 8F 00 14"))
        self.assertEqual(parse_frame(bytes.fromhex("68 05 00 8F 00 94")).command, 0x8f)

    def test_unknown_factory_output_layout_not_guessed(self):
        body = bytes([7, 0, 0x84, 0, 0, 0])
        frame = parse_frame(b"\x68"+body+bytes([sum(body) & 255]))
        with self.assertRaises(UnsupportedOutput):
            decode_sample(frame)

    def test_offline_tool_rejects_device_file(self):
        result = subprocess.run(["/usr/bin/python3", "-m", "sweeper_hw.inspect_capture", "--file", "/dev/null"],
                                cwd=ROOT, capture_output=True)
        self.assertEqual(result.returncode, 2)


class CDecoderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="imu406-host-test-")
        library = Path(cls.temp.name)/"libimu406.so"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared",
                        str(ROOT/"firmware/common/imu406_decode.c"), "-o", str(library)], check=True)
        cls.lib = ctypes.CDLL(str(library))
        class Result(ctypes.Structure):
            _fields_ = [("address", ctypes.c_uint8), ("euler_deg", ctypes.c_float*3),
                        ("accel_m_s2", ctypes.c_float*3), ("gyro_rad_s", ctypes.c_float*3)]
        cls.Result = Result
        cls.lib.imu406_decode.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t, ctypes.POINTER(Result)]
        cls.lib.imu406_decode.restype = ctypes.c_int

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_host_c_matches_python(self):
        raw = example_frame(5)
        buffer = (ctypes.c_uint8*len(raw)).from_buffer_copy(raw)
        result = self.Result()
        self.assertEqual(self.lib.imu406_decode(buffer, len(raw), ctypes.byref(result)), 0)
        reference = decode_sample(parse_frame(raw))
        self.assertEqual(result.address, 5)
        for a, b in zip(result.euler_deg, reference.euler_deg):
            self.assertAlmostEqual(a, b, places=4)
        for a, b in zip(result.accel_m_s2, reference.si_vectors()["accel"]):
            self.assertAlmostEqual(a, b, places=4)
        for a, b in zip(result.gyro_rad_s, reference.si_vectors()["gyro"]):
            self.assertAlmostEqual(a, b, places=5)

    def test_host_c_preserves_output_on_corrupt_input(self):
        raw = bytearray(example_frame())
        raw[4] = 0x2a
        raw[-1] = sum(raw[1:-1]) & 255  # correct checksum, invalid BCD
        buffer = (ctypes.c_uint8*len(raw)).from_buffer_copy(raw)
        result = self.Result()
        result.euler_deg[0] = 123.
        before = bytes(result)
        self.assertLess(self.lib.imu406_decode(buffer, len(raw), ctypes.byref(result)), 0)
        self.assertEqual(bytes(result), before)


if __name__ == "__main__":
    unittest.main()
