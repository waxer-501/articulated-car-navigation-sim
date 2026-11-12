"""Desktop monitor for the independent articulated car. No target writes/reset/flash."""
import argparse
import ctypes
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

from .monitor_state import MonitorState, display_sample
from .f103_diagnostic import decode_snapshot
from .telemetry import Packet

ROOT = Path(__file__).resolve().parents[1]
LOCAL_ARCHIVE = ROOT/'captures/f103_radio_20261105/power_cycle_recovery/rx.jsonl'
ARCHIVE = LOCAL_ARCHIVE if LOCAL_ARCHIVE.exists() else ROOT/'validation/2026-11-05/f103_radio_20261105/power_cycle_recovery/rx.jsonl'


class Recorder:
    def __init__(self, folder):
        self.folder = folder
        folder.mkdir(parents=True,exist_ok=False)
        self.lock = threading.Lock()
        self.events = (folder/'events.jsonl').open('x',encoding='utf-8')
        self.raw = (folder/'radio.bin').open('xb')
        self.csv = (folder/'radio.csv').open('x',encoding='utf-8-sig')
        self.csv.write('sensor,boot_id,sequence,mcu_uart_rx_us,host_monotonic_s,roll_deg,pitch_deg,yaw_deg,ax_m_s2,ay_m_s2,az_m_s2,gx_deg_s,gy_deg_s,gz_deg_s\n')

    def event(self, kind, row):
        with self.lock:
            self.events.write(json.dumps({'kind':kind,**row},ensure_ascii=False)+'\n');self.events.flush()
            if kind == 'radio_sample':
                values=[row['sensor'],row['boot_id'],row['sequence'],row['mcu_uart_rx_time_us'],row['host_received_monotonic_s'],
                        *row['euler_deg'],*row['accel_m_s2'],*row['gyro_deg_s']]
                self.csv.write(','.join(map(str,values))+'\n');self.csv.flush()

    def bytes(self, data):
        with self.lock:self.raw.write(data);self.raw.flush()

    def close(self):
        with self.lock:
            self.events.close();self.raw.close();self.csv.close()


class Hardware:
    def __init__(self, state, record, port):
        self.state,self.record,self.port=state,record,port
        self.stop = threading.Event()
        self.swd_enabled = threading.Event();self.swd_enabled.set()
        self.usb_enabled = threading.Event();self.usb_enabled.set()
        self.threads=[threading.Thread(target=self.serial_loop,daemon=True),threading.Thread(target=self.swd_loop,daemon=True)]

    def start(self):
        for t in self.threads:t.start()

    def serial_loop(self):
        import serial
        while not self.stop.is_set():
            if not self.usb_enabled.is_set():self.state.connection('usb',False,'接收已暂停');self.stop.wait(.2);continue
            port=serial.Serial(port=None,baudrate=9600,timeout=.1,exclusive=True,xonxoff=False,rtscts=False,dsrdtr=False)
            port.dtr=port.rts=False;port.port=self.port
            try:
                port.open();self.state.connection('usb',True)
                self.record.event('usb_connected',{'host_monotonic_s':time.monotonic(),'port':self.port})
                while not self.stop.is_set() and self.usb_enabled.is_set():
                    data=port.read(min(port.in_waiting or 1,4096))
                    if not data:continue
                    now=time.monotonic();self.record.bytes(data)
                    for row in self.state.usb_bytes(data,now):self.record.event('radio_sample',row)
            except (serial.SerialException,OSError) as e:
                self.state.connection('usb',False,str(e));self.stop.wait(2)
            finally:port.close();self.state.connection('usb',False)

    def swd_loop(self):
        from tools.f103_swd import load_reader
        lib=None;handle=None
        try:
            while not self.stop.is_set():
                if not self.swd_enabled.is_set():
                    if handle:lib.capture_close(handle);handle=None
                    self.state.connection('swd',False,'STM32 诊断已暂停');self.stop.wait(.2);continue
                try:
                    if lib is None:lib=load_reader()
                    if handle is None:
                        handle=lib.capture_open(b'33FF72064243323456261357')
                        if not handle:raise RuntimeError('ST-Link 未连接、被其他工具占用或目标未识别')
                    data=(ctypes.c_uint8*256)();rc=lib.capture_snapshot(handle,data)
                    if rc<0:raise RuntimeError('ST-Link 读取失败')
                    if rc==0:
                        with self.state.lock:self.state.swd['busy_reads']+=1
                    else:
                        decoded=decode_snapshot(bytes(data));now=time.monotonic()
                        self.state.diagnostic(decoded,now);self.state.connection('swd',True)
                        self.record.event('mcu_diagnostic',{'host_monotonic_s':now,'raw_snapshot_hex':bytes(data).hex(),**decoded})
                    self.stop.wait(.4)
                except (RuntimeError,OSError,ValueError) as e:
                    self.state.connection('swd',False,str(e))
                    if handle:lib.capture_close(handle);handle=None
                    self.stop.wait(2)
        finally:
            if handle:lib.capture_close(handle)

    def close(self):
        self.stop.set()
        for t in self.threads:t.join(timeout=5)
        return not any(t.is_alive() for t in self.threads)


def load_replay(path):
    result=[]
    for line in path.read_text().splitlines():
        row=json.loads(line)
        if row.get('kind')!='sample':continue
        packet=Packet.decode(bytes.fromhex(row['wire_hex']))
        name='front' if packet.sensor_id==1 else 'rear'
        if packet.boot_id!=row['boot_id'] or packet.sensor_id!=row['sensor_id'] or name!=row['sensor']:raise ValueError('回放身份不一致')
        stamp=row['host_received_monotonic_s']
        if not math.isfinite(stamp) or (result and stamp<result[-1]['host_received_monotonic_s']):raise ValueError('回放时间顺序无效')
        result.append(display_sample({**row,**packet.measurements(),'sequence':packet.sequence,'mcu_uart_rx_time_us':packet.rx_time_us}))
    if not result:raise ValueError('回放文件没有有效无线数据')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default=None)
    p.add_argument('--offline',action='store_true',help='Never open serial or ST-Link; explicit recorded playback only')
    p.add_argument('--replay',type=Path,default=ARCHIVE)
    p.add_argument('--output',type=Path)
    p.add_argument('--screenshot',type=Path,help='Offline rendering check; capture only this application window')
    args=p.parse_args()
    if args.screenshot and not args.offline:p.error('--screenshot requires --offline')
    if args.screenshot:
        from PIL import ImageGrab  # Rendering QA only; normal monitor needs no Pillow.
    import tkinter as tk
    from .monitor_view import MonitorView
    state=MonitorState();record=None;hardware=None
    folder=args.output or ROOT/'captures/monitor'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    if not args.offline:
        port=args.port or json.loads((ROOT/'configs/usb_radio_host.json').read_text())['rx']['path']
        record=Recorder(folder);record.event('session',{'created_utc':datetime.now(timezone.utc).isoformat(),'mode':'live; read-only','port':port})
        hardware=Hardware(state,record,port)
    root=tk.Tk()
    try:archive=load_replay(args.replay);archive_error=None
    except (OSError,ValueError,KeyError) as e:archive=[];archive_error=str(e)
    view=MonitorView(root,state,hardware,folder,archive,args.offline,archive_error)
    root.after(100,root.lift)
    if hardware:hardware.start()
    if args.screenshot:
        root.attributes('-topmost',True)
        def screenshot():
            try:
                root.update_idletasks()
                x,y=root.winfo_rootx(),root.winfo_rooty();w,h=root.winfo_width(),root.winfo_height()
                args.screenshot.parent.mkdir(parents=True,exist_ok=True)
                ImageGrab.grab(bbox=(x,y,x+w,y+h),xdisplay=root.winfo_screen()).save(args.screenshot)
            finally:root.destroy()
        root.after(8000,screenshot)
    try:root.mainloop()
    finally:
        if hardware:
            complete=hardware.close()
            if complete:record.close()
    if args.screenshot and not args.screenshot.exists():raise RuntimeError('Rendering screenshot was not created')
    return 0


if __name__=='__main__':raise SystemExit(main())
