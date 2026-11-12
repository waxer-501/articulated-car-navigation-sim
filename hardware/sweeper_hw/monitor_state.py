"""Thread-safe monitor data; all input is measured RX or explicit SWD diagnostics."""
from collections import deque
import copy
import math
import threading
import time
from .telemetry import ReceiverState
from .telemetry_stream import TelemetryStream


def display_sample(row):
    return {**row, 'gyro_deg_s': [math.degrees(x) for x in row['gyro_rad_s']]}


class MonitorState:
    def __init__(self):
        self.lock = threading.RLock()
        self.receiver = ReceiverState(2.5)
        self.decoder = TelemetryStream()
        self.usb = {'connected': False, 'error': None, 'bytes': 0}
        self.swd = {'connected': False, 'error': None, 'busy_reads': 0}
        self.mcu = None
        self.mcu_seen = None
        self.previous_mcu = None
        self.source_hz = {'front': None, 'rear': None}
        self.source_latest = {}
        self.radio_latest = {}
        self.history = {s:{n:deque(maxlen=1800) for n in ('front','rear')} for s in ('radio','mcu')}
        self.per_boot = {}
        self.covered_boots = set()

    def connection(self, kind, connected, error=None, now=None):
        with self.lock:
            if kind=='usb' and connected and not self.usb['connected']:
                self.usb['connected_since']=time.monotonic() if now is None else now
            getattr(self, kind).update(connected=connected, error=error)
            if kind == 'usb' and not connected:
                self.covered_boots.clear()  # A receiver interruption invalidates complete-window claims.

    def usb_bytes(self, raw, now):
        accepted = []
        with self.lock:
            self.usb['bytes'] += len(raw)
            for wire in self.decoder.feed(raw):
                previous_boot = self.receiver.boot_id
                row = self.receiver.accept(wire, now)
                if row is None: continue
                if previous_boot != self.receiver.boot_id:self.radio_latest.clear()
                row = display_sample(dict(row, wire_hex=wire.hex()))
                name = row['sensor']
                self.radio_latest[name] = row
                self.history['radio'][name].append((now,row))
                self.per_boot[row['boot_id']] = self.per_boot.get(row['boot_id'],0)+1
                while len(self.per_boot)>32: self.per_boot.pop(next(iter(self.per_boot)))
                accepted.append(copy.deepcopy(row))
        return accepted

    def diagnostic(self, data, now):
        with self.lock:
            old = self.mcu
            old_seen=self.mcu_seen
            if old and old.get('radio',{}).get('boot_id') == data.get('radio',{}).get('boot_id'):
                dt = (data['mcu_uptime_ms']-old['mcu_uptime_ms']) & 0xffffffff
                if 0 < dt < 10000:
                    for n in ('front','rear'):
                        count = (data['sensors'][n]['valid_frames']-old['sensors'][n]['valid_frames']) & 0xffffffff
                        self.source_hz[n] = count*1000/dt if count < 10000 else None
            else:
                self.source_hz = {'front':None,'rear':None}
                self.source_latest.clear()
            self.mcu, self.mcu_seen = copy.deepcopy(data), now
            radio = data.get('radio',{})
            observed_restart=(old is not None and old.get('radio',{}).get('boot_id')!=radio.get('boot_id') and
                              old_seen is not None and self.usb.get('connected_since',now)<=old_seen)
            if self.usb['connected'] and radio.get('boot_id') and (data['mcu_uptime_ms'] < 4000 or observed_restart):
                self.covered_boots.add(radio['boot_id'])
            for name, item in data['sensors'].items():
                sample = item['sample']
                if sample is None: continue
                previous = self.source_latest.get(name)
                row = {'sensor':name, 'sequence':item['valid_frames'] & 65535,
                       'mcu_uart_rx_time_us':item['mcu_uart_rx_time_us'], 'boot_id':radio.get('boot_id'),
                       'euler_deg':list(sample['euler_deg']), 'gyro_deg_s':list(sample['gyro_deg_s']),
                       'accel_m_s2':list(sample['si']['accel']), 'wire_hex':sample['raw_hex'],
                       'host_received_monotonic_s':now, 'valid_frames':item['valid_frames']}
                identity = (row['boot_id'],row['valid_frames'],row['mcu_uart_rx_time_us'])
                old_id = None if previous is None else (previous['boot_id'],previous['valid_frames'],previous['mcu_uart_rx_time_us'])
                if identity != old_id:
                    self.source_latest[name] = row
                    self.history['mcu'][name].append((now,row))

    def snapshot(self, now):
        with self.lock:
            histories = {s:{n:list(v) for n,v in values.items()} for s,values in self.history.items()}
            rate = {}
            for name, items in histories['radio'].items():
                recent = [t for t,_ in items if now-t <= 12]
                rate[name] = (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else 0.
            radio = (self.mcu or {}).get('radio',{})
            boot = radio.get('boot_id')
            comparison = {'complete_window_observed':boot in self.covered_boots,
                          'sent':radio.get('packets_completed'), 'received':self.per_boot.get(boot,0),
                          'reason':'本窗口没有覆盖本次启动的完整发送过程，不能据此计算无线丢包。'}
            if boot in self.covered_boots:
                comparison['reason'] = '发送中，等待自动结束后核对。'
                if radio.get('state') == 4 and self.mcu and now-(self.mcu_seen or now)<3:
                    since_end = self.mcu['mcu_uptime_ms']-radio.get('end_ms',0)
                    if since_end > 2500:
                        comparison['matched'] = radio['packets_completed'] == self.per_boot.get(boot,0)
                        comparison['reason'] = ('完整包计数一致。' if comparison['matched'] else '完整包计数不同，请查看原始记录。')
            return copy.deepcopy({'usb':self.usb,'swd':self.swd,'mcu':self.mcu,'mcu_seen':self.mcu_seen,
                'source_hz':self.source_hz,'radio_hz':rate,'source_latest':self.source_latest,
                'radio_latest':self.radio_latest,'history':histories,'comparison':comparison,
                'rx_status':self.receiver.status(now),'crc_errors':self.decoder.invalid_frames,
                'discarded_bytes':self.decoder.discarded_bytes,'trailing_bytes':len(self.decoder.buffer)})
