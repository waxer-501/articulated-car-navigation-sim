import struct
import unittest
from sweeper_hw.f103_diagnostic import decode_snapshot, HEADER, SENSOR, MAGIC
from test_imu406 import example_frame


def snapshot(frames=1):
    raw = bytearray(256)
    HEADER.pack_into(raw, 0, MAGIC, 1, 256, 2, 1000, 64000000, 0, 0)
    for index in range(2):
        SENSOR.pack_into(raw, 32+80*index, 32, frames, 0, 0, 0, 999000, 0, 1,
                         example_frame() if frames else bytes(32), bytes(16))
    return raw


class DiagnosticTests(unittest.TestCase):
    def test_valid_front_and_rear_without_fabricated_clock_sync(self):
        result = decode_snapshot(snapshot())
        self.assertEqual(result['mcu_uptime_ms'], 1000)
        for state in result['sensors'].values():
            self.assertEqual(state['sample']['accel_g'], (2.304, -2.304, -2.304))
            self.assertEqual(state['mcu_uart_rx_time_us'], 999000)
            self.assertNotIn('host_monotonic_s', state)

    def test_missing_measurements_are_null(self):
        for state in decode_snapshot(snapshot(0))['sensors'].values():
            self.assertIsNone(state['sample'])
            self.assertIsNone(state['mcu_uart_rx_time_us'])

    def test_unknown_firmware_or_incomplete_size_is_rejected(self):
        for offset, value in ((0, 0), (4, 2), (8, 128)):
            raw = snapshot(); struct.pack_into('<I', raw, offset, value)
            with self.assertRaises(ValueError): decode_snapshot(raw)
        with self.assertRaises(ValueError): decode_snapshot(snapshot()[:-1])

    def test_odd_seqlock_is_rejected(self):
        raw = snapshot(); struct.pack_into('<I', raw, 12, 3)
        with self.assertRaises(ValueError): decode_snapshot(raw)

    def test_invalid_imu_frame_is_rejected_even_if_counter_nonzero(self):
        raw = snapshot(); raw[32+32+31] ^= 1
        with self.assertRaises(ValueError): decode_snapshot(raw)

    def test_uart_tail_restored_in_chronological_order(self):
        raw = snapshot()
        struct.pack_into('<I', raw, 32+24, 19)
        raw[32+64:32+80] = bytes(range(16))
        tail = decode_snapshot(raw)['sensors']['front']['latest_uart_bytes_hex']
        self.assertEqual(bytes.fromhex(tail), bytes(range(3,16))+bytes(range(3)))


if __name__ == '__main__':
    unittest.main()
