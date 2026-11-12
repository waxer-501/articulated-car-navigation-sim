import json
from pathlib import Path
import tempfile
import unittest
from test_telemetry import packet
from test_f103_diagnostic import snapshot
from sweeper_hw.f103_diagnostic import decode_snapshot
from sweeper_hw.monitor_state import MonitorState
from sweeper_hw.monitor import load_replay


def diagnostic(boot=2,ms=1000,frames=25,state=1,packets=0):
    d=decode_snapshot(snapshot(frames))
    d['mcu_uptime_ms']=ms
    d['radio']={'boot_id':boot,'state':state,'packets_completed':packets,'end_ms':65000}
    return d


class MonitorTests(unittest.TestCase):
    def test_fragmented_wire_is_not_a_zero_sample(self):
        s=MonitorState();raw=packet().encode()
        self.assertEqual(s.usb_bytes(raw[:16],1),[])
        self.assertEqual(s.snapshot(1)['radio_latest'],{})
        self.assertEqual(len(s.usb_bytes(raw[16:],1.3)),1)
        self.assertEqual(s.snapshot(1.3)['usb']['bytes'],32)

    def test_stale_and_crc_error_do_not_invent_updates(self):
        s=MonitorState();raw=packet().encode();s.usb_bytes(raw,1)
        wrong=bytearray(raw);wrong[-1]^=1
        self.assertEqual(s.usb_bytes(wrong,5),[])
        snap=s.snapshot(5)
        self.assertEqual(snap['crc_errors'],1)
        self.assertFalse(snap['rx_status']['sensors']['front']['fresh'])
        self.assertEqual(snap['radio_latest']['front']['host_received_monotonic_s'],1)

    def test_boot_change_clears_old_other_sensor(self):
        s=MonitorState()
        for sensor in (1,2):s.usb_bytes(packet(sensor,boot_id=1).encode(),1)
        s.usb_bytes(packet(2,boot_id=2).encode(),2)
        self.assertNotIn('front',s.snapshot(2)['radio_latest'])
        self.assertIn('rear',s.snapshot(2)['radio_latest'])

    def test_source_rate_uses_mcu_counter_not_host_time(self):
        s=MonitorState();s.diagnostic(diagnostic(ms=1000,frames=25),10)
        s.diagnostic(diagnostic(ms=1200,frames=30),100)
        self.assertEqual(s.snapshot(100)['source_hz']['front'],25)

    def test_starting_after_rf_ends_cannot_claim_packet_loss(self):
        s=MonitorState();s.connection('usb',True,now=1)
        s.diagnostic(diagnostic(ms=90000,state=4,packets=100),2)
        result=s.snapshot(2)['comparison']
        self.assertFalse(result['complete_window_observed'])
        self.assertNotIn('matched',result)

    def test_full_boot_window_is_compared_only_after_finish(self):
        s=MonitorState();s.connection('usb',True,now=.1)
        s.diagnostic(diagnostic(ms=2000),1)
        s.usb_bytes(packet(1,boot_id=2).encode(),2)
        self.assertNotIn('matched',s.snapshot(2)['comparison'])
        s.diagnostic(diagnostic(ms=68000,state=4,packets=1),70)
        self.assertTrue(s.snapshot(70)['comparison']['matched'])
        s.connection('usb',False)
        self.assertFalse(s.snapshot(70)['comparison']['complete_window_observed'])

    def test_continuous_receiver_across_restart_covers_new_boot(self):
        s=MonitorState();s.connection('usb',True,now=1)
        s.diagnostic(diagnostic(boot=2,ms=90000,state=4),2)
        s.diagnostic(diagnostic(boot=3,ms=6000),12)
        self.assertTrue(s.snapshot(12)['comparison']['complete_window_observed'])

    def test_source_diagnostics_never_increment_radio_receive_count(self):
        s=MonitorState();s.diagnostic(diagnostic(frames=2500,packets=100),10)
        snap=s.snapshot(10)
        self.assertEqual(snap['rx_status']['counts']['accepted'],0)
        self.assertEqual(snap['comparison']['received'],0)
        self.assertIn('front',snap['source_latest'])

    def test_replay_validates_wire_and_never_updates_live_receiver(self):
        s=MonitorState()
        raw=packet().encode()
        row={'kind':'sample','wire_hex':raw.hex(),'boot_id':123,'sensor_id':1,'sensor':'front',
             'host_received_monotonic_s':1,'euler_deg':[999,999,999]}
        with tempfile.TemporaryDirectory() as directory:
            f=Path(directory)/'rx.jsonl';f.write_text(json.dumps(row)+'\n')
            data=load_replay(f)
            self.assertEqual(data[0]['euler_deg'][0],160)
            self.assertEqual(s.snapshot(2)['rx_status']['counts']['accepted'],0)
            row['sensor']='rear';f.write_text(json.dumps(row)+'\n')
            with self.assertRaises(ValueError):load_replay(f)


if __name__=='__main__':unittest.main()
