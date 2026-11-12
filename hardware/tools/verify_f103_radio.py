#!/usr/bin/env python3
"""Review, or explicitly flash and capture bounded real-IMU radio telemetry."""
import argparse
import ctypes
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
from f103_swd import ROOT, load_reader
sys.path.insert(0, str(ROOT))
from sweeper_hw.f103_diagnostic import decode_snapshot
from sweeper_hw.telemetry import Packet


def snapshot():
    lib = load_reader()
    handle = lib.capture_open(b'33FF72064243323456261357')
    if not handle: raise RuntimeError('Cannot open ST-Link for radio diagnostics')
    try:
        for _ in range(40):
            data = (ctypes.c_uint8*256)()
            result = lib.capture_snapshot(handle, data)
            if result < 0: raise RuntimeError('SWD diagnostic read failed')
            if result == 1:
                decoded = decode_snapshot(bytes(data))
                decoded['raw_snapshot_hex'] = bytes(data).hex()
                if 'radio' not in decoded: raise RuntimeError('Radio firmware profile not detected')
                return decoded
            time.sleep(.01)
        raise RuntimeError('No coherent radio diagnostic snapshot; no reset/reflash attempted')
    finally:
        lib.capture_close(handle)


def analyze(rows, final_diag):
    samples = [r for r in rows if r.get('kind') == 'sample']
    finals = [r for r in rows if r.get('kind') == 'final_status']
    final = finals[-1] if finals else {}
    radio = final_diag.get('radio', {})
    boot = radio.get('boot_id')
    invalid_wire = wrong_boot = 0
    groups = {'front': [], 'rear': []}
    for row in samples:
        try:
            packet = Packet.decode(bytes.fromhex(row['wire_hex']))
        except (ValueError, KeyError):
            invalid_wire += 1
            continue
        wrong_boot += packet.boot_id != boot
        groups['front' if packet.sensor_id == 1 else 'rear'].append(row)
    result = {'scope': 'real dual IMU -> F103 SPI radio -> USB RX -> this PC; stationary test',
              'raw_bytes': final.get('received_bytes'), 'accepted_packets': len(samples),
              'mcu_completed_packets': radio.get('packets_completed'),
              'wire_recheck_errors': invalid_wire, 'wrong_boot_packets': wrong_boot,
              'stream_invalid_frames': final.get('stream_invalid_frames'),
              'stream_discarded_bytes': final.get('stream_discarded_bytes'),
              'radio_final': radio, 'sensors': {},
              'limitation': 'Source sequence increments at each valid IMU frame. Gaps include deliberate latest-only downsampling, not a radio loss measurement. RF packet delivery is compared separately to MCU completed packet count. Raspberry Pi is not part of this test.'}
    for name, data in groups.items():
        duration = data[-1]['host_received_monotonic_s']-data[0]['host_received_monotonic_s'] if len(data)>1 else 0
        result['sensors'][name] = {'packets': len(data), 'active_receive_hz': (len(data)-1)/duration if duration else None,
                                    'first_sequence': data[0]['sequence'] if data else None,
                                    'last_sequence': data[-1]['sequence'] if data else None,
                                    'source_frame_gaps_including_downsampling': sum(r['sequence_gap'] or 0 for r in data),
                                    'last_measurements': {k:data[-1][k] for k in ('euler_deg','accel_m_s2','gyro_rad_s')} if data else None}
    sensor_ok = all(len(groups[name]) >= 10 and final_diag.get('sensors',{}).get(name,{}).get('uart_errors') == 0 and
                    final_diag['sensors'][name]['queue_drops'] == 0 for name in groups)
    result['passed'] = (sensor_ok and invalid_wire == 0 and wrong_boot == 0 and final.get('stream_invalid_frames') == 0 and
                        final.get('stream_discarded_bytes') == 0 and final.get('trailing_bytes') == 0 and
                        radio.get('state') == 4 and radio.get('fragments_failed') == 0 and radio.get('encode_errors') == 0 and
                        radio.get('spi_errors') == 0 and radio.get('init_error') == 0 and boot is not None and boot > 0 and
                        len(samples) == radio.get('packets_completed') and final_diag.get('fault') == 0)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--apply', action='store_true', help='Authorize firmware overwrite/reset and bounded RF test')
    p.add_argument('--output', type=Path, default=ROOT/'captures/f103_radio_20261105/first_run')
    args = p.parse_args()
    if not args.apply:
        return subprocess.run(['/usr/bin/python3', str(ROOT/'tools/flash_f103_capture.py'), '--profile', 'radio']).returncode
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    config = json.loads((ROOT/'configs/usb_radio_host.json').read_text())
    port = config['rx']['path']
    python = ROOT/'.venv/bin/python'
    child = None
    status_path = folder/'run_status.json'
    def phase(value):
        status_path.write_text(json.dumps({'phase':value,'host_monotonic_s':time.monotonic()},indent=2)+'\n')
        print(value, flush=True)
    with (folder/'receiver.log').open('x') as log:
        try:
            phase('open RX collector before firmware reset')
            child = subprocess.Popen([str(python), '-m', 'sweeper_hw.usb_receiver', '--port', port, '--baud', '9600',
                                      '--seconds', '120', '--output', str(folder/'rx.jsonl'),
                                      '--raw-output', str(folder/'serial.bin')], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            time.sleep(2)
            if child.poll() is not None: raise RuntimeError('RX collector exited during startup; inspect receiver.log')
            phase('backup, flash radio profile, reset and verify; no motion')
            with (folder/'flash_driver.log').open('x') as out:
                code = subprocess.run(['/usr/bin/python3', str(ROOT/'tools/flash_f103_capture.py'),
                                       '--profile','radio','--apply'], cwd=ROOT, stdout=out, stderr=subprocess.STDOUT).returncode
            if code: raise RuntimeError('Firmware/verification failed; no automatic retry. Inspect flash_driver.log')
            initial = snapshot()
            (folder/'initial_diagnostic.json').write_text(json.dumps(initial,ensure_ascii=False,indent=2)+'\n')
            if initial['radio']['state'] not in (1,4) or initial['radio']['init_error']:
                raise RuntimeError('SPI/radio initialization failed; inspect initial_diagnostic.json')
            phase('collect bounded telemetry window; firmware will power radio down automatically')
            while child.poll() is None: time.sleep(.2)
            if child.returncode: raise RuntimeError('RX collector failed; inspect receiver.log')
            final = snapshot()
            (folder/'final_diagnostic.json').write_text(json.dumps(final,ensure_ascii=False,indent=2)+'\n')
            rows = [json.loads(x) for x in (folder/'rx.jsonl').read_text().splitlines()]
            result = analyze(rows, final)
            (folder/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            phase('passed' if result['passed'] else 'completed; acceptance failed; inspect summary')
            print(json.dumps(result,ensure_ascii=False,indent=2))
            return 0 if result['passed'] else 2
        except BaseException as error:
            phase('failed: '+str(error))
            raise
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try: child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=10)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
