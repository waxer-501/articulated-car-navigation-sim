"""Decode a coherent, receive-only firmware diagnostic snapshot; no hardware IO."""
import struct
from .imu406 import decode_sample, parse_frame

HEADER = struct.Struct('<8I')
SENSOR = struct.Struct('<8I32s16s')
MAGIC = 0x31444d49


def decode_snapshot(raw):
    if len(raw) != 256:
        raise ValueError('Expected 256-byte diagnostic block')
    magic, abi, size, lock, ms, hz, fault, reset = HEADER.unpack_from(raw)
    if (magic, abi, size) != (MAGIC, 1, 256):
        raise ValueError('F103 capture-only v1 firmware not detected; never flash automatically')
    if lock & 1:
        raise ValueError('Firmware update in progress: retry coherent snapshot')
    result = {'mcu_uptime_ms': ms, 'nominal_core_hz': hz, 'fault': fault,
              'reset_flags': reset, 'sensors': {}}
    extension = struct.unpack_from('<16I', raw, 192)
    if extension[0] == 2:
        result['radio'] = dict(zip(('profile', 'boot_id', 'state', 'fragments_ok', 'fragments_failed',
                                    'packets_completed', 'encode_errors', 'spi_errors', 'end_ms',
                                    'packets_started', 'last_status', 'readback_config_channel_rf_feature',
                                    'readback_en_aa_aw_dynpd', 'fragment_gap_us', 'init_error', 'start_ms'), extension))
    for index, name in enumerate(('front', 'rear')):
        (rx_bytes, frames, rejected, errors, drops, us, tail_pos, max_depth,
         frame, tail) = SENSOR.unpack_from(raw, HEADER.size+index*SENSOR.size)
        used = min(tail_pos, 16)
        latest_bytes = bytes(tail[(tail_pos-used+i) % 16] for i in range(used))
        row = {'rx_bytes': rx_bytes, 'valid_frames': frames, 'rejected_candidates': rejected,
               'uart_errors': errors, 'queue_drops': drops, 'max_queue_depth': max_depth,
               'mcu_uart_rx_time_us': us if frames else None, 'latest_uart_bytes_hex': latest_bytes.hex(),
               'sample': None}
        if frames:
            sample = decode_sample(parse_frame(frame))
            row['sample'] = {'imu_address': sample.address, 'raw_hex': frame.hex(),
                             'euler_deg': sample.euler_deg, 'accel_g': sample.accel_g,
                             'gyro_deg_s': sample.gyro_deg_s, 'si': sample.si_vectors(),
                             'axes': 'sensor; mounting transform not applied'}
        result['sensors'][name] = row
    return result
