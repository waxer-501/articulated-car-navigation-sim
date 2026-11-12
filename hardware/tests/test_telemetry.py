import ctypes
from dataclasses import replace
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_imu406 import example_frame
from sweeper_hw.imu406 import parse_frame, decode_sample
from sweeper_hw.telemetry import Packet, ReceiverState, crc16

ROOT = Path(__file__).resolve().parents[1]


def packet(sensor_id=1, sequence=0, rx_time_us=100, boot_id=123):
    return Packet.from_sample(decode_sample(parse_frame(example_frame())), sensor_id, sequence, rx_time_us, boot_id)


class WireTests(unittest.TestCase):
    def test_crc_known_check_value(self):
        self.assertEqual(crc16(b"123456789"), 0x29b1)

    def test_exact_32_byte_round_trip_preserves_sensor_resolution(self):
        p = packet()
        self.assertEqual(len(p.encode()), 32)
        self.assertEqual(Packet.decode(p.encode()), p)
        self.assertEqual(p.values, (16000, -5023, -16000, 2304, -2304, -2304, -5023, 18000, 5023))

    def test_every_single_bit_error_rejected(self):
        original = packet().encode()
        for bit in range(256):
            damaged = bytearray(original)
            damaged[bit//8] ^= 1 << (bit % 8)
            with self.assertRaises(ValueError):
                Packet.decode(damaged)

    def test_unknown_version_or_identity_rejected(self):
        for header in (0x13, 0x21, 0x10):
            raw = bytearray(packet().encode())
            raw[1] = header
            raw[-2:] = crc16(raw[:30]).to_bytes(2, "little")
            with self.assertRaises(ValueError):
                Packet.decode(raw)

    def test_out_of_range_not_wrapped(self):
        for invalid in (replace(packet(), sequence=65536), replace(packet(), boot_id=-1),
                        replace(packet(), values=(40000,)*9)):
            with self.assertRaises(ValueError):
                invalid.encode()


class ReceiveTests(unittest.TestCase):
    def test_front_and_rear_are_independent(self):
        receiver = ReceiverState()
        a = receiver.accept(packet(1, 3).encode(), 10.)
        b = receiver.accept(packet(2, 1).encode(), 10.1)
        self.assertEqual(a["sensor"], "front")
        self.assertEqual(b["sensor"], "rear")
        self.assertTrue(all(x["fresh"] for x in receiver.status(10.2)["sensors"].values()))

    def test_gaps_duplicates_and_reordering(self):
        receiver = ReceiverState()
        receiver.accept(packet(sequence=10).encode(), 1.)
        row = receiver.accept(packet(sequence=13, rx_time_us=200).encode(), 1.1)
        self.assertEqual(row["sequence_gap"], 2)
        self.assertIsNone(receiver.accept(packet(sequence=13, rx_time_us=200).encode(), 1.2))
        self.assertIsNone(receiver.accept(packet(sequence=12, rx_time_us=150).encode(), 1.3))
        self.assertEqual(receiver.counts["duplicates"], 1)
        self.assertEqual(receiver.counts["out_of_order"], 1)

    def test_sequence_wrap_and_raw_mcu_counter_wrap(self):
        receiver = ReceiverState()
        receiver.accept(packet(sequence=65535, rx_time_us=0xfffffff0).encode(), 1.)
        row = receiver.accept(packet(sequence=0, rx_time_us=84).encode(), 1.1)
        self.assertEqual(row["sequence_gap"], 0)
        self.assertEqual(row["mcu_uart_rx_time_us"], 84)
        self.assertEqual(row["host_received_monotonic_s"], 1.1)

    def test_missing_and_stale_sensor_not_filled_with_zero(self):
        receiver = ReceiverState()
        receiver.accept(packet().encode(), 1.)
        status = receiver.status(3.1)
        self.assertFalse(status["sensors"]["front"]["fresh"])
        self.assertFalse(status["sensors"]["rear"]["present"])
        self.assertIsNone(status["sensors"]["rear"]["age_since_receive_s"])

    def test_restart_invalidates_previous_sensor_samples(self):
        receiver = ReceiverState()
        receiver.accept(packet(1, boot_id=10).encode(), 1.)
        receiver.accept(packet(2, boot_id=10).encode(), 1.1)
        receiver.accept(packet(1, boot_id=11).encode(), 2.)
        self.assertFalse(receiver.status(2.1)["sensors"]["rear"]["present"])
        self.assertIsNone(receiver.accept(packet(2, boot_id=10).encode(), 2.2))

    def test_long_gap_resynchronizes_without_fabricating_loss_count(self):
        receiver = ReceiverState()
        receiver.accept(packet(sequence=1).encode(), 1.)
        row = receiver.accept(packet(sequence=50000, rx_time_us=12340000).encode(), 600.)
        self.assertTrue(row["stream_discontinuity"])
        self.assertIsNone(row["sequence_gap"])


class FirmwareWireTests(unittest.TestCase):
    def test_c_encoder_matches_pi_decoder_byte_for_byte(self):
        with tempfile.TemporaryDirectory(prefix="imu-radio-host-") as directory:
            path = Path(directory)/"libtelemetry.so"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                            str(ROOT/"firmware/common/telemetry_v1.c"), "-o", str(path)], check=True)
            lib = ctypes.CDLL(str(path))
            lib.telemetry_v1_from_imu406.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t, ctypes.c_uint8,
                                                   ctypes.c_uint16, ctypes.c_uint32, ctypes.c_uint32,
                                                   ctypes.POINTER(ctypes.c_uint8)]
            lib.telemetry_v1_from_imu406.restype = ctypes.c_int
            raw = example_frame()
            source = (ctypes.c_uint8*32).from_buffer_copy(raw)
            result = (ctypes.c_uint8*32)()
            self.assertEqual(lib.telemetry_v1_from_imu406(source, 32, 2, 65535, 0xfffffff0, 0x12345678, result), 0)
            expected = packet(2, 65535, 0xfffffff0, 0x12345678).encode()
            self.assertEqual(bytes(result), expected)
            before = bytes(result)
            source[31] ^= 1
            self.assertLess(lib.telemetry_v1_from_imu406(source, 32, 2, 0, 0, 1, result), 0)
            self.assertEqual(bytes(result), before)


if __name__ == "__main__":
    unittest.main()
