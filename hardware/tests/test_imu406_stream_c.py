import ctypes
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_imu406 import example_frame

ROOT = Path(__file__).resolve().parents[1]


class Stream(ctypes.Structure):
    _fields_ = [('bytes', ctypes.c_uint8*32), ('used', ctypes.c_uint32), ('rejected', ctypes.c_uint32)]


class CStreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='imu-stream-')
        output = Path(cls.temp.name)/'stream.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                        str(ROOT/'firmware/common/imu406_stream.c'), str(ROOT/'firmware/common/imu406_decode.c'),
                        '-o', str(output)], check=True)
        cls.lib = ctypes.CDLL(str(output))
        cls.lib.imu406_stream_push.argtypes = [ctypes.POINTER(Stream), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8)]
        cls.lib.imu406_stream_push.restype = ctypes.c_int
        cls.lib.imu406_stream_discontinuity.argtypes = [ctypes.POINTER(Stream)]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def feed(self, data, stream=None):
        stream = stream if stream is not None else Stream()
        out = (ctypes.c_uint8*32)(*([0xab]*32))
        frames = []
        for byte in data:
            before = bytes(out)
            if self.lib.imu406_stream_push(ctypes.byref(stream), byte, out):
                frames.append(bytes(out))
            else:
                self.assertEqual(bytes(out), before)
            self.assertLess(stream.used, 32)
        return frames, stream

    def test_noise_and_back_to_back_frames(self):
        raw = example_frame()
        frames, _ = self.feed(b'\x00\xff\x68\x04\x00\x8c\x91'+raw*3)
        self.assertEqual(frames, [raw]*3)

    def test_deleted_byte_and_invalid_checksum_resynchronize(self):
        raw = example_frame()
        corrupt = bytearray(raw); corrupt[-1] ^= 1
        frames, stream = self.feed(raw[:13]+raw[14:]+bytes(corrupt)+raw*2)
        self.assertEqual(frames, [raw]*2)
        self.assertGreater(stream.rejected, 0)

    def test_embedded_marker_is_not_a_new_frame(self):
        raw = bytearray(example_frame()); raw[5] = 0x68; raw[-1] = sum(raw[1:-1]) & 255
        frames, _ = self.feed(bytes(raw)*2)
        self.assertEqual(frames, [bytes(raw)]*2)

    def test_bad_bcd_with_good_checksum_rejected(self):
        raw = bytearray(example_frame()); raw[5] = 0xaa; raw[-1] = sum(raw[1:-1]) & 255
        frames, _ = self.feed(bytes(raw)+example_frame())
        self.assertEqual(frames, [example_frame()])

    def test_uart_discontinuity_drops_partial_frame(self):
        raw = example_frame()
        _, stream = self.feed(raw[:16])
        self.lib.imu406_stream_discontinuity(ctypes.byref(stream))
        frames, _ = self.feed(raw[16:]+raw, stream)
        self.assertEqual(frames, [raw])

    def test_each_single_byte_deletion_recovers(self):
        raw = example_frame()
        for index in range(32):
            frames, _ = self.feed(raw[:index]+raw[index+1:]+raw*2)
            self.assertEqual(frames[-2:], [raw]*2, index)


if __name__ == '__main__':
    unittest.main()
