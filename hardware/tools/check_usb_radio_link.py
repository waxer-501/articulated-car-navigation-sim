#!/usr/bin/env python3
"""Short, bounded TX-only -> RX-only USB radio test; no configuration commands."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import secrets
import time
from zoneinfo import ZoneInfo


def open_port(path, baud):
    import serial
    from serial.tools import list_ports
    resolved = str(Path(path).resolve(strict=True))
    info = next((p for p in list_ports.comports() if p.device == resolved), None)
    if info is None or (info.vid, info.pid) != (0x1a86, 0x7523):
        raise RuntimeError(f"Not a verified CH340 endpoint: {path}")
    port = serial.Serial(port=None, baudrate=baud, bytesize=8, parity="N", stopbits=1,
                         timeout=.05, write_timeout=.5, exclusive=True,
                         xonxoff=False, rtscts=False, dsrdtr=False)
    port.dtr = False
    port.rts = False
    port.port = resolved
    port.open()
    return port


def read_for(port, seconds, stop_marker=None):
    data = bytearray()
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        data.extend(port.read(min(port.in_waiting or 1, 4096)))
        if stop_marker and stop_marker in data:
            # Allow serial tail/padding to arrive before classifying transparency.
            time.sleep(.05)
            data.extend(port.read(min(port.in_waiting, 4096)))
            break
        if len(data) >= 65536:
            raise RuntimeError("Unexpected continuous RX traffic; stopped bounded test")
    return bytes(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/usb_radio_host.json"))
    parser.add_argument("--baud", type=int, required=True)
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--describe", action="store_true", help="Show planned operation without opening devices")
    args = parser.parse_args()
    if args.baud not in (9600, 19200, 38400, 57600, 115200) or not 1 <= args.count <= 10:
        parser.error("Use a supported UART baud and 1..10 test markers")
    cfg = json.loads(args.config.read_text())
    if cfg["tx"]["path"] == cfg["rx"]["path"]:
        parser.error("TX and RX must be separate devices")
    result = {"time": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(), "baud": args.baud,
              "tx": cfg["tx"]["path"], "rx": cfg["rx"]["path"],
              "direction": "TX -> RX only", "configuration_commands_sent": False,
              "vehicle_connected_by_this_program": False, "results": []}
    if args.describe:
        print(json.dumps({**result, "planned_markers": args.count, "marker_bytes": 13,
                          "expected_duration_s": .5+args.count*2.1}, ensure_ascii=False, indent=2))
        return
    rx = tx = None
    try:
        rx = open_port(result["rx"], args.baud)
        tx = open_port(result["tx"], args.baud)
        result["baseline_rx_hex"] = read_for(rx, .5).hex(" ")
        nonce = secrets.token_hex(3).upper()
        for index in range(args.count):
            marker = f"IMU{nonce}{index:03d}!".encode("ascii")
            written = tx.write(marker)
            received = read_for(rx, 2., marker)
            row = {"sent_ascii": marker.decode(), "written": written, "received_hex": received.hex(" "),
                   "marker_received": marker in received, "exact": received == marker}
            result["results"].append(row)
            print(json.dumps(row), flush=True)
            time.sleep(.1)
        result["passed"] = all(r["marker_received"] for r in result["results"])
        result["exact_transparency"] = all(r["exact"] for r in result["results"])
    except Exception as exc:
        result["passed"] = False
        result["error"] = str(exc)
    finally:
        if tx is not None:
            tx.close()
        if rx is not None:
            rx.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"passed": result["passed"], "output": str(args.output),
                      "error": result.get("error")}, ensure_ascii=False), flush=True)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
