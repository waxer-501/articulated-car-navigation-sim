#!/usr/bin/env python3
"""Review/flash the capture-only image. Dry-run by default; never mass erase."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from f103_swd import ROOT, load_reader
import ctypes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Explicitly overwrite first image pages and reset MCU')
    parser.add_argument('--serial', default='33FF72064243323456261357')
    parser.add_argument('--profile', choices=('capture', 'radio'), default='capture')
    args = parser.parse_args()
    name = 'f103_'+args.profile
    build = ROOT/'build'/name
    report = json.loads((build/'build.json').read_text())
    binary = build/(name+'.bin')
    expected = binary.read_bytes()
    if hashlib.sha256(expected).hexdigest() != report['binary_sha256'] or not 8 <= len(expected) <= 32768:
        raise RuntimeError('Firmware size/hash mismatch; rebuild before flashing')
    for name, digest in report['inputs'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'Firmware input changed since build: {name}')
    print(json.dumps({'action': 'FLASH AND RESET' if args.apply else 'review only; no hardware opened',
                      'serial': args.serial, 'target': '0x0412 / 32 KiB', 'binary_bytes': len(expected),
                      'image_sha256': report['binary_sha256'],
                      'pages_to_replace': (len(expected)+1023)//1024,
                      'boot_journal': report.get('boot_journal'),
                      'profile': report['profile']}, ensure_ascii=False, indent=2), flush=True)
    if not args.apply:
        return 0
    lib = load_reader()
    data = (ctypes.c_uint8*32768)()
    handle = lib.capture_open(args.serial.encode('ascii'))
    if not handle:
        raise RuntimeError('Chosen ST-Link / target mismatch; nothing written')
    destination = ROOT/'captures/f103_flash'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    destination.mkdir(parents=True, exist_ok=False)
    try:
        if lib.capture_flash_backup(handle, data) != 0:
            raise RuntimeError('Backup failed; nothing written')
        backup = bytes(data)
        (destination/'before.bin').write_bytes(backup)
    finally:
        lib.capture_close(handle)
    # st-flash 1.7.0 halts even on read. Use it only in this explicitly writable path.
    program = ROOT/'.toolchain/usr/bin/st-flash'
    import os
    environment = dict(os.environ, LD_LIBRARY_PATH=str(ROOT/'.toolchain/usr/lib/x86_64-linux-gnu'))
    with (destination/'flash.log').open('x') as stream:
        result = subprocess.run([str(program), '--serial', args.serial, '--freq=100', '--reset',
                                 'write', str(binary), '0x08000000'], env=environment,
                                stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'Flash failed: {destination}/flash.log; no automatic retry')
    handle = lib.capture_open(args.serial.encode('ascii'))
    if not handle:
        raise RuntimeError('Cannot reconnect for verification; do not reflash automatically')
    try:
        if lib.capture_flash_backup(handle, data) != 0:
            raise RuntimeError('Readback failed; do not reflash automatically')
        readback = bytes(data)
        (destination/'after.bin').write_bytes(readback)
    finally:
        lib.capture_close(handle)
    pages_end = ((len(expected)+1023)//1024)*1024
    verified = readback[:len(expected)] == expected
    preserve_end = report.get('boot_journal', {}).get('offset', 32768)
    preserved = readback[pages_end:preserve_end] == backup[pages_end:preserve_end]
    outcome = {'firmware_sha256': report['binary_sha256'], 'backup_sha256': hashlib.sha256(backup).hexdigest(),
               'readback_matches_image': verified, 'flash_beyond_image_pages_preserved': preserved,
               'preserved_offset_range': [pages_end, preserve_end],
               'boot_journal_changed': readback[preserve_end:] != backup[preserve_end:],
               'record_directory': str(destination)}
    (destination/'result.json').write_text(json.dumps(outcome, indent=2)+'\n')
    print(json.dumps(outcome, indent=2))
    if not verified or not preserved:
        raise RuntimeError('Flash verification failed; no automatic retry')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
