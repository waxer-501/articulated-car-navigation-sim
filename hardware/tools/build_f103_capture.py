#!/usr/bin/env python3
"""Build F103C6 capture or data-only radio firmware; never open USB or flash."""
import hashlib
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('capture', 'radio', 'radio-stream'), default='capture')
    args = parser.parse_args()
    wireless = args.profile != 'capture'
    streaming = args.profile == 'radio-stream'
    name = 'f103_'+args.profile.replace('-', '_')
    tc = ROOT / '.toolchain/usr/bin'
    cmsis = ROOT / 'vendor/stm32f1_cmsis'
    destination = ROOT / 'build' / name
    destination.mkdir(parents=True, exist_ok=True)
    for item in json.loads((cmsis / 'sources.json').read_text()):
        path = cmsis / item['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise RuntimeError(f'CMSIS source hash changed: {path}')
    sources = [ROOT/'firmware/f103_capture/main.c', ROOT/'firmware/common/imu406_stream.c',
               ROOT/'firmware/common/imu406_decode.c', cmsis/'device/startup_stm32f103x6.s']
    if wireless:
        sources += [ROOT/'firmware/common/boot_journal.c', ROOT/'firmware/common/nrf24_tx.c',
                    ROOT/'firmware/common/telemetry_v1.c', ROOT/'firmware/f103_radio/radio_app.c']
    if streaming:
        sources += [ROOT/'firmware/common/radio_stream.c']
    flags = ['-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft', '-Os', '-g3',
             '-ffunction-sections', '-fdata-sections', '-fno-common', '-fstack-usage',
             '-Wall', '-Wextra', '-Werror', '-std=c11', '-DSTM32F103x6',
             '-I'+str(cmsis/'device'), '-I'+str(cmsis/'core'), '-I'+str(ROOT/'firmware/common')]
    if wireless:
        flags += ['-DRADIO_TELEMETRY', '-I'+str(ROOT/'firmware/f103_radio')]
    if streaming:
        flags += ['-DRADIO_STREAM']
    objects = []
    for index, source in enumerate(sources):
        obj = destination/f'{index}_{source.stem}.o'
        subprocess.run([str(tc/'arm-none-eabi-gcc'), *flags, '-c', str(source), '-o', str(obj)], check=True)
        objects.append(obj)
    elf = destination/(name+'.elf')
    linker = ROOT/'firmware'/('f103_radio' if wireless else 'f103_capture')/'linker.ld'
    subprocess.run([str(tc/'arm-none-eabi-gcc'), '-mcpu=cortex-m3', '-mthumb', '-nostdlib',
                    *map(str, objects), '-T'+str(linker), '-Wl,--gc-sections,--build-id=none',
                    '-Wl,-Map='+str(destination/(name+'.map')), '-Wl,--print-memory-usage',
                    '-Wl,--start-group', '-lc', '-lgcc', '-Wl,--end-group', '-o', str(elf)], check=True)
    binary = destination/(name+'.bin')
    subprocess.run([str(tc/'arm-none-eabi-objcopy'), '-O', 'binary', str(elf), str(binary)], check=True)
    subprocess.run([str(tc/'arm-none-eabi-size'), str(elf)], check=True)
    inputs = sources + [linker] + list((ROOT/'firmware/common').glob('*.h'))
    if wireless: inputs += list((ROOT/'firmware/f103_radio').glob('*.h'))
    inputs += [ROOT/'tools/build_f103_capture.py']
    if streaming: inputs += [ROOT/'configs/radio_spi_stream.json']
    report = {'target': 'STM32F103C6T6A', 'flash_limit_bytes': 32768, 'sram_limit_bytes': 10240,
              'profile': ('dual IMU RX; no IMU commands; real IMU telemetry RF TX, nominal seconds 5..65 after boot' if wireless else
                          'receive-only; UART1/2; no IMU commands; radio CE low; no RF transmission'),
              'profile_id': args.profile,
              'compiler_flags': flags,
              'diagnostic_address': '0x20000000', 'diagnostic_bytes': 256,
              'binary_bytes': binary.stat().st_size, 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'inputs': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
              'cmsis_sources': json.loads((cmsis/'sources.json').read_text()), 'flashed': False}
    if streaming:
        report['profile'] = 'dual IMU RX -> direct SPI radio receiver; 32-byte whole frames; continuous; follows source rate; no IMU commands'
    if wireless:
        report['boot_journal'] = {'offset': 30720, 'bytes': 2048, 'purpose': 'per-device boot ID; app writes one 8-byte record each boot'}
        report['radio_test'] = {'channel': 60, 'air_bps': 250000, 'address_spi_hex': '3443101001',
                                'fragment_bytes': 16, 'fragment_gap_us': 300000, 'start_ms': 5000, 'end_ms': 65000}
        if streaming:
            report.pop('radio_test')
            report['radio_stream'] = {'channel': 60, 'air_bps': 250000, 'address_spi_hex': '3443101001',
                                      'payload_bytes': 32, 'dynamic_payload': False, 'hardware_crc_bytes': 1,
                                      'auto_ack': True, 'retransmit_delay_us': 750, 'retransmit_count': 3,
                                      'fragment_gap_us': 0, 'continuous': True, 'imu_commands': False,
                                      'source_rate_hz_per_sensor_expected': 25, 'rate_policy': 'each valid frame; no artificial downsampling',
                                      'queue_depth_per_sensor': 4, 'maximum_queue_age_us': 100000,
                                      'tx_completion_deadline_us': 100000, 'power_up_settle_us': 2000,
                                      'receiver_config': 'configs/radio_spi_stream.json', 'physically_verified': False}
    (destination/'build.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print('Built only; no hardware was opened:', binary)


if __name__ == '__main__':
    main()
