#!/usr/bin/env python3
"""Read-only F103 SWD backup/diagnostics. Does not load or flash firmware."""
import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sweeper_hw.f103_diagnostic import decode_snapshot


def load_reader():
    source = ROOT/'tools/stlink_readonly.c'
    output = ROOT/'build/host/libstlink_readonly.so'
    output.parent.mkdir(parents=True, exist_ok=True)
    library = ROOT/'.toolchain/usr/lib/x86_64-linux-gnu'
    if not output.exists() or output.stat().st_mtime_ns < source.stat().st_mtime_ns:
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                        '-I'+str(ROOT/'.toolchain/usr/include/stlink'),
                        '-I'+str(ROOT/'.toolchain/usr/include/libusb-1.0'), '-I/usr/include/libusb-1.0',
                        str(source), '-L'+str(library), '-Wl,-rpath,'+str(library),
                        '-lstlink', '-o', str(output)], check=True)
    lib = ctypes.CDLL(str(output))
    lib.capture_open.argtypes = [ctypes.c_char_p]; lib.capture_open.restype = ctypes.c_void_p
    lib.capture_close.argtypes = [ctypes.c_void_p]
    lib.capture_snapshot.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8)]
    lib.capture_flash_backup.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8)]
    lib.capture_unstable_reads.restype = ctypes.c_uint
    return lib


def capture(lib, handle, directory, seconds, hz):
    start = time.monotonic()
    last_ids, observations = {}, {}
    consistent, busy, invalid, resets = 0, 0, 0, 0
    last_ms = None
    first = latest = None
    with (directory/'diagnostics.jsonl').open('x') as stream:
        while time.monotonic()-start < seconds:
            tick = time.monotonic()
            block = (ctypes.c_uint8*256)()
            rc = lib.capture_snapshot(handle, block)
            if rc < 0:
                raise RuntimeError('SWD read failed; no reset or retry with side effects was attempted')
            if rc == 0:
                busy += 1
            else:
                raw = bytes(block)
                try:
                    row = decode_snapshot(raw)
                except ValueError as error:
                    invalid += 1
                    stream.write(json.dumps({'type': 'invalid_snapshot', 'host_monotonic_s': tick,
                                             'reason': str(error), 'raw_hex': raw.hex()})+'\n')
                    if consistent == 0:
                        raise RuntimeError(str(error)) from error
                    continue
                consistent += 1
                if last_ms is not None and ((row['mcu_uptime_ms']-last_ms) & 0xffffffff) > 0x80000000:
                    resets += 1
                    last_ids.clear()
                last_ms = row['mcu_uptime_ms']
                row.update(type='diagnostic', source='ST-Link SRAM; RF not involved',
                           host_monotonic_s=tick, capture_segment=resets, raw_snapshot_hex=raw.hex())
                stream.write(json.dumps(row, ensure_ascii=False)+'\n')
                if first is None:
                    first = row
                latest = row
                for name, state in row['sensors'].items():
                    if state['sample'] is None:
                        continue
                    identity = (state['valid_frames'], state['mcu_uart_rx_time_us'])
                    if identity != last_ids.get(name):
                        observations[name] = observations.get(name, 0)+1
                        last_ids[name] = identity
                if row['fault'] & 0x80000000:
                    raise RuntimeError(f"Firmware fault: {row['fault']:#x}")
            delay = 1/hz-(time.monotonic()-tick)
            if delay > 0:
                time.sleep(delay)
    sensor_results = {}
    for name in ('front', 'rear'):
        a = first['sensors'][name] if first else None
        b = latest['sensors'][name] if latest else None
        advance = ((b['valid_frames']-a['valid_frames']) & 0xffffffff) if a and b and not resets else None
        errors = ((b['uart_errors']-a['uart_errors']) & 0xffffffff) if a and b and not resets else None
        drops = ((b['queue_drops']-a['queue_drops']) & 0xffffffff) if a and b and not resets else None
        elapsed_ms = ((latest['mcu_uptime_ms']-first['mcu_uptime_ms']) & 0xffffffff) if first and latest and not resets else None
        sensor_results[name] = {'observed_distinct_latest_samples': observations.get(name, 0),
                                'valid_frame_counter_advance': advance, 'new_uart_errors': errors,
                                'new_queue_drops': drops,
                                'nominal_mcu_frame_rate_hz': advance*1000/elapsed_ms if advance is not None and elapsed_ms else None,
                                'passed': advance is not None and advance > 0 and errors == 0 and drops == 0 and observations.get(name, 0) > 0}
    summary = {'scope': 'stationary dual-IMU UART capture only; RF not validated',
               'elapsed_host_s': time.monotonic()-start, 'coherent_snapshots': consistent,
               'busy_snapshots_skipped': busy, 'invalid_snapshots': invalid, 'observed_restarts': resets,
               'debug_read_inconsistency_events': lib.capture_unstable_reads(),
               'sensors': sensor_results,
               'passed': all(x['passed'] for x in sensor_results.values()) and invalid == 0 and resets == 0 and latest is not None and latest['fault'] == 0,
               'limitation': 'SWD polls only the latest frame; unobserved intermediate frames are not radio packet loss. MCU time is nominal oscillator based, not IMU sample time.'}
    (directory/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary['passed'] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('backup', 'capture', 'build-reader'))
    parser.add_argument('--serial', default='33FF72064243323456261357')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--hz', type=float, default=20)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not math.isfinite(args.hz) or not 1 <= args.hz <= 100:
        parser.error('seconds must be finite/positive; hz must be in [1,100]')
    lib = load_reader()
    if args.action == 'build-reader':
        print('Read-only helper built. No USB device opened.')
        return 0
    if args.output is None:
        parser.error('--output must name a new capture directory')
    args.output.mkdir(parents=True, exist_ok=False)
    handle = lib.capture_open(args.serial.encode('ascii'))
    if not handle:
        raise RuntimeError('Cannot open chosen ST-Link / F1 low-density 32 KiB target')
    try:
        if args.action == 'backup':
            buf = (ctypes.c_uint8*32768)()
            if lib.capture_flash_backup(handle, buf) != 0:
                raise RuntimeError('Flash read failed. No protection change/erase was attempted.')
            raw = bytes(buf)
            (args.output/'original_flash.bin').write_bytes(raw)
            report = {'serial': args.serial, 'chip_id': '0x0412', 'bytes': len(raw),
                      'address': '0x08000000', 'sha256': hashlib.sha256(raw).hexdigest(),
                      'read_only': True, 'halt_or_reset_requested': False,
                      'consecutive_identical_reads_per_block': 3,
                      'debug_read_inconsistency_events': lib.capture_unstable_reads(),
                      'all_erased_ff': all(x == 255 for x in raw)}
            (args.output/'backup.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2))
            return 0
        return capture(lib, handle, args.output, args.seconds, args.hz)
    finally:
        lib.capture_close(handle)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
