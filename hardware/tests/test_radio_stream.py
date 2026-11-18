"""C streaming scheduler + real TX state machine with fake SPI; no USB/MCU IO."""
import ctypes as C
import json
import struct
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock
from test_imu406 import example_frame
from test_radio_firmware import FakeSPI, IO
from test_f103_diagnostic import snapshot
from sweeper_hw.telemetry import Packet, ReceiverState
from sweeper_hw.f103_diagnostic import decode_snapshot
from sweeper_hw.monitor_state import MonitorState
from sweeper_hw.monitor_view import MonitorView

ROOT = Path(__file__).resolve().parents[1]


class StreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='radio-stream-fake-')
        directory = Path(cls.temp.name)
        harness = directory/'size.c'
        harness.write_text('#include "radio_stream.h"\n#include <stddef.h>\n'
                           'size_t stream_size(void) { return sizeof(radio_stream_t); }\n')
        out = directory/'stream.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                        '-I'+str(ROOT/'firmware/common'), str(harness),
                        str(ROOT/'firmware/common/radio_stream.c'),
                        str(ROOT/'firmware/common/nrf24_tx.c'),
                        str(ROOT/'firmware/common/telemetry_v1.c'), '-o', str(out)], check=True)
        cls.lib = C.CDLL(str(out))
        cls.lib.stream_size.restype = C.c_size_t
        cls.lib.radio_stream_init.argtypes = [C.c_void_p, C.POINTER(IO), C.c_uint32, C.c_uint32]
        cls.lib.radio_stream_frame.argtypes = [C.c_void_p, C.c_uint, C.POINTER(C.c_uint8), C.c_uint32, C.c_uint32]
        cls.lib.radio_stream_poll.argtypes = [C.c_void_p, C.c_uint32]
        cls.lib.radio_stream_diagnostic.argtypes = [C.c_void_p, C.POINTER(C.c_uint32)]
        cls.lib.radio_stream_busy.argtypes = [C.c_void_p]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setup_stream(self, now=0, boot=7, **kwargs):
        self.fake = FakeSPI(**kwargs)
        self.storage = C.create_string_buffer(self.lib.stream_size())
        self.started = None
        self.now = now
        return self.lib.radio_stream_init(self.storage, C.byref(self.fake.io), boot, now)

    def frame(self, sensor, seq, now=None, raw=None):
        raw = example_frame() if raw is None else raw
        data = (C.c_uint8*32).from_buffer_copy(raw)
        return self.lib.radio_stream_frame(self.storage, sensor, data,
                                          self.now if now is None else now, seq)

    def diagnostic(self):
        words = (C.c_uint32*16)()
        self.lib.radio_stream_diagnostic(self.storage, words)
        return list(words)

    def pump(self, now, ack=True):
        self.now = now
        if ack and self.started is not None and now-self.started >= 1500:
            self.fake.status |= 0x20
        count = len(self.fake.payloads)
        self.lib.radio_stream_poll(self.storage, now & 0xffffffff)
        if len(self.fake.payloads) != count:
            self.started = now
        elif not self.lib.radio_stream_busy(self.storage):
            self.started = None

    def drain(self, start, end, ack=True):
        for now in range(start, end, 50):
            self.pump(now, ack)

    def test_fixed_32_byte_profile_and_settle_without_early_rf(self):
        self.assertEqual(self.setup_stream(), 0)
        self.assertEqual(self.fake.regs[4][0], 0x23)
        self.assertEqual(self.fake.regs[0x1c][0], 0)
        self.assertEqual(self.fake.regs[0x1d][0], 0)
        self.assertEqual(self.fake.regs[0x11][0], 32)
        self.assertEqual(self.fake.regs[0x10], bytes.fromhex('3443101001'))
        self.frame(0, 1, 0)
        self.pump(1999)
        self.assertEqual(self.fake.ce_highs, 0)
        self.pump(2000)
        self.assertEqual(len(self.fake.payloads[0]), 32)
        self.assertEqual(Packet.decode(self.fake.payloads[0]).sequence, 1)

    def test_pi_receiver_register_contract_matches_the_c_transmitter(self):
        self.setup_stream()
        config = json.loads((ROOT/'configs/radio_spi_stream.json').read_text())
        registers = config['receiver_registers_hex']
        addresses = {'CONFIG':0,'EN_AA':1,'EN_RXADDR':2,'SETUP_AW':3,'RF_CH':5,
                     'RF_SETUP':6,'DYNPD':0x1c,'FEATURE':0x1d,'RX_PW_P0':0x11}
        for name, address in addresses.items():
            value = self.fake.regs[address][0]
            if name == 'CONFIG':value |= 1  # RX mode differs from TX mode only.
            self.assertEqual(value, int(registers[name],16),name)
        self.assertEqual(self.fake.regs[0x0a].hex(),registers['RX_ADDR_P0_spi'])
        self.assertEqual(list(self.fake.regs[0x10]),config['air']['address_bytes_in_spi_order'])

    def simulate_rate(self, hz, seconds):
        self.assertEqual(self.setup_stream(), 0)
        count = hz*seconds
        period = 1000000//hz
        state = ReceiverState(.5)
        for index in range(count):
            t = index*period+4000
            for sensor in range(2):
                self.assertEqual(self.frame(sensor, index+1, t), 0)
            self.drain(t, t+period)
        diag = self.diagnostic()
        self.assertEqual(diag[3], 2*count, diag)
        self.assertEqual(diag[4], 0)
        self.assertEqual(diag[8:10], [0, 0])
        self.assertEqual(diag[11], 0)
        self.assertEqual(len(self.fake.payloads), 2*count)
        for index, wire in enumerate(self.fake.payloads):
            packet = Packet.decode(wire)
            self.assertEqual(packet.sensor_id, index % 2+1)
            self.assertEqual(packet.sequence, index//2+1)
            self.assertEqual(packet.rx_time_us, (index//2)*period+4000)
            self.assertIsNotNone(state.accept(wire, .004+index*period/2e6))
        self.assertEqual(state.counts['sequence_gaps'], 0)

    def test_every_sample_at_dual_25_hz_with_real_telemetry_crc_and_no_duplicates(self):
        self.simulate_rate(25, 3)

    def test_scheduler_follows_higher_source_rate_without_configuring_imu(self):
        self.simulate_rate(100, 3)

    def test_does_not_stop_at_old_65_second_window(self):
        self.simulate_rate(25, 67)
        self.assertEqual(self.diagnostic()[2], 1)

    def test_bounded_queues_keep_newest_and_report_overflow_fairly(self):
        self.setup_stream()
        for seq in range(1, 8):self.frame(0, seq, 4000)
        for seq in range(1, 7):self.frame(1, seq, 4000)
        self.drain(4000, 30000)
        diag = self.diagnostic()
        self.assertEqual(diag[8], 5)
        self.assertEqual(diag[12], 8)
        packets = [Packet.decode(wire) for wire in self.fake.payloads]
        self.assertEqual([p.sensor_id for p in packets], [1, 2]*4)
        self.assertEqual([p.sequence for p in packets if p.sensor_id==1], [4,5,6,7])
        self.assertEqual([p.sequence for p in packets if p.sensor_id==2], [3,4,5,6])

    def test_stale_queued_samples_are_counted_not_transmitted(self):
        self.setup_stream()
        self.frame(0, 1, 0);self.frame(1, 1, 0)
        self.pump(100001)
        self.assertEqual(self.fake.payloads, [])
        self.assertEqual(self.diagnostic()[9], 2)

    def test_invalid_frame_and_missing_or_unidentified_radio_do_not_emit(self):
        for kwargs in ({'boot':0}, {'missing':True}):
            self.assertLess(self.setup_stream(**kwargs), 0)
            self.assertLess(self.frame(0,1), 0)
            self.drain(4000,5000)
            self.assertEqual(self.fake.ce_highs, 0)
        self.setup_stream()
        self.assertLess(self.frame(0,1,raw=bytes(32)), 0)
        self.assertEqual(self.diagnostic()[6],1)
        self.assertEqual(self.fake.payloads, [])

    def test_failed_packet_releases_channel_for_fresh_other_imu(self):
        self.setup_stream();self.frame(0,1,4000);self.frame(1,1,4000)
        self.pump(4000,False);self.pump(4050,False)
        self.fake.status |= 0x10
        self.pump(4100,False)
        self.drain(4150,7000)
        diag=self.diagnostic()
        self.assertEqual(diag[3:6],[1,1,2])
        self.assertEqual(Packet.decode(self.fake.payloads[1]).sensor_id,2)

    def test_stuck_radio_uses_generous_100ms_deadline_not_one_second_backlog(self):
        self.setup_stream();self.frame(0,1,4000)
        self.pump(4000,False);self.pump(4050,False)
        self.pump(104049,False)
        self.assertEqual(self.diagnostic()[4],0)
        self.frame(1,1,104050)
        self.pump(104050,False)
        self.assertEqual(self.diagnostic()[4],1)
        self.assertEqual(Packet.decode(self.fake.payloads[1]).sensor_id,2)

    def test_timestamp_wrap_and_long_uptime_do_not_rearm_startup_wait(self):
        self.setup_stream(now=0xfffff000)
        self.frame(0,65535,0xfffff800)
        self.drain(0xfffff800,0x100001000)
        self.assertEqual(self.diagnostic()[3],1)
        self.frame(0,65536,0x80002000)
        self.drain(0x180002000,0x180003000)
        self.assertEqual(self.diagnostic()[3],2)
        self.assertEqual(Packet.decode(self.fake.payloads[-1]).sequence,0)
        self.assertEqual(Packet.decode(self.fake.payloads[-1]).rx_time_us,0x80002000)

    def test_continuous_diagnostic_is_not_a_usb_completed_window_claim(self):
        self.setup_stream();self.frame(0,1,4000);self.drain(4000,7000)
        raw=snapshot();struct.pack_into('<16I',raw,192,*self.diagnostic())
        data=decode_snapshot(raw)
        self.assertTrue(data['radio']['continuous'])
        self.assertEqual(data['radio']['packets_completed'],1)
        monitor=MonitorState();monitor.connection('usb',True,now=1);monitor.diagnostic(data,1)
        comparison=monitor.snapshot(1)['comparison']
        self.assertFalse(comparison['complete_window_observed'])
        self.assertNotIn('matched',comparison)
        self.assertTrue(comparison['continuous'])

    def test_monitor_renders_stream_profile_without_legacy_fragment_fields(self):
        self.setup_stream();self.frame(0,1,4000);self.drain(4000,7000)
        raw=snapshot();struct.pack_into('<16I',raw,192,*self.diagnostic())
        monitor=MonitorState();monitor.connection('swd',True)
        monitor.diagnostic(decode_snapshot(raw),time.monotonic())
        view=MonitorView.__new__(MonitorView)
        view.state=monitor;view.root=Mock();view.card=Mock();view.source=Mock()
        view.source.get.return_value='STM32 旁路诊断'
        view.mode_label=Mock();view.banner=Mock();view.banner_label=Mock();view.footer=Mock()
        view.sensor_text={n:Mock() for n in ('front','rear')}
        view.raw_text={n:Mock() for n in ('front','rear')}
        view.plots={(n,key):Mock() for n in ('front','rear') for key in ('euler_deg','accel_m_s2','gyro_deg_s')}
        view.offline=False;view.hardware=None;view.folder=Path('unused');view.frame=0
        view.update()
        self.assertTrue(any(call.args[1]=='连续整包发送' for call in view.card.call_args_list))
        self.assertIn('不是树莓派实收',view.banner.set.call_args.args[0])
        view.root.after.assert_called_once()


if __name__ == '__main__':unittest.main()
