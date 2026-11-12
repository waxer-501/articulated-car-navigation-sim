"""RX USB serial collector. Has no serial write or vehicle-control code path."""
import argparse
import json
from pathlib import Path
import time
from contextlib import nullcontext
from .telemetry import ReceiverState
from .telemetry_stream import TelemetryStream


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Explicit RX endpoint; never auto-selects a serial port")
    parser.add_argument("--baud", type=int, required=True, help="Verified USB-board UART baud; no assumed default")
    parser.add_argument("--seconds", type=float, default=30.)
    parser.add_argument("--stale-after", type=float, default=2.)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-output", type=Path, help="Optional raw serial bytes; must not already exist")
    args = parser.parse_args()
    if not 0 < args.seconds <= 86400 or args.baud not in (9600, 19200, 38400, 57600, 115200, 230400):
        parser.error("Provide a positive duration <= one day and a supported verified UART rate")
    import serial
    receiver = ReceiverState(args.stale_after)
    stream = TelemetryStream()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Fail rather than overwrite an earlier capture.
    if args.raw_output:
        args.raw_output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as output, (args.raw_output.open('xb') if args.raw_output else nullcontext()) as raw_output:
        port = serial.Serial(port=None, baudrate=args.baud, timeout=.05, exclusive=True,
                             xonxoff=False, rtscts=False, dsrdtr=False)
        port.dtr, port.rts = False, False
        port.port = args.port
        started = time.monotonic()
        last_status = started
        total_bytes = 0
        try:
            port.open()
            while time.monotonic()-started < args.seconds:
                data = port.read(min(port.in_waiting or 1, 4096))
                received_at = time.monotonic()
                total_bytes += len(data)
                if raw_output is not None: raw_output.write(data)
                for raw in stream.feed(data):
                    row = receiver.accept(raw, received_at)
                    if row is not None:
                        output.write(json.dumps({"kind": "sample", "wire_hex": raw.hex(), **row}, ensure_ascii=False)+"\n")
                if received_at-last_status >= 1.:
                    status = {"kind": "status", "host_monotonic_s": received_at,
                              "received_bytes": total_bytes, "stream_invalid_frames": stream.invalid_frames,
                              "stream_discarded_bytes": stream.discarded_bytes,
                              **receiver.status(received_at)}
                    output.write(json.dumps(status, ensure_ascii=False)+"\n")
                    output.flush()
                    print(json.dumps(status, ensure_ascii=False), flush=True)
                    last_status = received_at
        except KeyboardInterrupt:
            pass
        finally:
            port.close()
            final = {"kind": "final_status", "received_bytes": total_bytes,
                     "stream_invalid_frames": stream.invalid_frames, "stream_discarded_bytes": stream.discarded_bytes,
                     "trailing_bytes": len(stream.buffer), **receiver.status(time.monotonic())}
            output.write(json.dumps(final, ensure_ascii=False)+"\n")
            print(json.dumps(final, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
