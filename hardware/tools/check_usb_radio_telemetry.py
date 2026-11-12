"""Bounded synthetic telemetry test across the identified TX/RX USB boards."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo
from .check_usb_radio_link import open_port, read_for
from sweeper_hw.telemetry import Packet
from sweeper_hw.telemetry_stream import TelemetryStream


def main():
    args_parser = argparse.ArgumentParser(description=__doc__)
    args_parser.add_argument("--config", type=Path, default=Path("configs/usb_radio_host.json"))
    args_parser.add_argument("--output", type=Path, required=True)
    args_parser.add_argument("--fragment-interval", type=float, default=.3)
    args = args_parser.parse_args()
    if not .05 <= args.fragment_interval <= 1.:
        args_parser.error("Fragment interval must be between 0.05 and 1 second")
    cfg = json.loads(args.config.read_text())
    if not cfg.get("baud_rate_verified") or cfg["baud_rate"] != 9600:
        args_parser.error("Run the short ASCII test and record the verified baud first")
    frames = [Packet(1+i % 2, i//2, 1000000+i*50000, 0x54455354,
                     (0, -1, 104, 166, -1234, 1000, 256, -256, 327)).encode() for i in range(20)]
    result = {"time": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
              "synthetic": True, "real_imu_data": False, "direction": "TX -> RX only",
              "baud": 9600, "fragment_bytes": 16, "fragment_interval_s": args.fragment_interval,
              "sent_frames": len(frames), "sent_bytes": sum(map(len, frames))}
    rx = tx = None
    received = bytearray()
    started = time.monotonic()
    try:
        rx = open_port(cfg["rx"]["path"], 9600)
        tx = open_port(cfg["tx"]["path"], 9600)
        result["baseline_rx_hex"] = read_for(rx, .3).hex(" ")
        for frame in frames:
            for offset in (0, 16):
                fragment = frame[offset:offset+16]
                if tx.write(fragment) != len(fragment):
                    raise RuntimeError("Incomplete USB write")
                received.extend(read_for(rx, args.fragment_interval))
        received.extend(read_for(rx, .5))
        stream = TelemetryStream()
        parsed = stream.feed(received)
        result.update(received_bytes=len(received), valid_frames=len(parsed),
                      exact_stream=bytes(received) == b"".join(frames),
                      exact_frame_order=parsed == frames, invalid_frames=stream.invalid_frames,
                      discarded_bytes=stream.discarded_bytes, received_hex=received.hex(" "),
                      received_sensor_ids=[Packet.decode(frame).sensor_id for frame in parsed])
        result["passed"] = result["exact_stream"] and result["exact_frame_order"]
    except Exception as exc:
        result.update(passed=False, error=str(exc))
    finally:
        if tx is not None:
            tx.close()
        if rx is not None:
            rx.close()
        result["wall_seconds"] = time.monotonic()-started
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({k: v for k, v in result.items() if k != "received_hex"}, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
