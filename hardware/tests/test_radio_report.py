"""Acceptance report must distinguish source downsampling from RF delivery loss."""
from pathlib import Path
import sys
import unittest
from copy import deepcopy
from test_telemetry import packet
from sweeper_hw.telemetry import ReceiverState

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from verify_f103_radio import analyze


def capture():
    state = ReceiverState(2.)
    rows = []
    for index in range(10):
        for sensor in (1,2):
            raw = packet(sensor_id=sensor, sequence=index*30, boot_id=123, rx_time_us=1000+index*1200000).encode()
            row = state.accept(raw, 1+index*1.2+sensor*.6)
            rows.append(dict(kind='sample', wire_hex=raw.hex(), **row))
    rows.append(dict(kind='final_status', received_bytes=640, stream_invalid_frames=0,
                     stream_discarded_bytes=0, trailing_bytes=0))
    diag = dict(fault=0, radio=dict(boot_id=123, state=4, fragments_failed=0, packets_completed=20,
                                   encode_errors=0, spi_errors=0, init_error=0),
                sensors={s:dict(uart_errors=0,queue_drops=0) for s in ('front','rear')})
    return rows, diag


class RadioReportTests(unittest.TestCase):
    def test_deliberate_source_gaps_do_not_fail_complete_radio_delivery(self):
        rows, diag = capture()
        result = analyze(rows, diag)
        self.assertTrue(result['passed'])
        self.assertGreater(result['sensors']['front']['source_frame_gaps_including_downsampling'],0)

    def test_unreceived_completed_packet_fails(self):
        rows, diag = capture(); diag['radio']['packets_completed'] += 1
        self.assertFalse(analyze(rows,diag)['passed'])

    def test_crc_boot_and_uart_faults_cannot_be_hidden_by_good_packet_counts(self):
        rows, diag = capture()
        for field in ('wire','boot','uart','init'):
            r, d = deepcopy(rows), deepcopy(diag)
            if field == 'wire': r[0]['wire_hex'] = '00'*32
            elif field == 'boot': d['radio']['boot_id'] = 124
            elif field == 'uart': d['sensors']['rear']['uart_errors'] = 1
            else: d['radio']['init_error'] = 1
            self.assertFalse(analyze(r,d)['passed'],field)

    def test_running_test_is_not_reported_as_finished(self):
        rows, diag = capture(); diag['radio']['state'] = 1
        self.assertFalse(analyze(rows,diag)['passed'])


if __name__ == '__main__': unittest.main()
