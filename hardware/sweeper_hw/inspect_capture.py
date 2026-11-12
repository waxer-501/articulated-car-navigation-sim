"""Decode a saved regular file or explicit hex; never opens a serial device."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from .imu406 import StreamDecoder, decode_sample, FrameError


def main():
    parser = argparse.ArgumentParser(description="离线检查 IMU406 数据；不打开设备、不发送配置")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--file", type=Path, help="已保存的原始二进制文件（不是串口设备）")
    source.add_argument("--hex", help="离线十六进制字节，例如 68 05 00 8C 00 91")
    args = parser.parse_args()
    decoder = StreamDecoder()
    samples, other, invalid = 0, 0, 0
    def consume(data):
        nonlocal samples, other, invalid
        for frame in decoder.feed(data):
            if frame.command != 0x84:
                other += 1
                print(json.dumps({"kind": "non_sample_frame", "command": frame.command, "raw_hex": frame.raw.hex(" ")}, ensure_ascii=False))
                continue
            try:
                sample = decode_sample(frame)
            except FrameError as exc:
                invalid += 1
                print(json.dumps({"kind": "rejected_sample", "error": str(exc), "raw_hex": frame.raw.hex(" ")}, ensure_ascii=False))
                continue
            samples += 1
            print(json.dumps({"kind": "imu406_sensor_axes_unaligned", **asdict(sample),
                              "si": sample.si_vectors(), "timestamp": None, "raw_hex": frame.raw.hex(" ")}, ensure_ascii=False))
    if args.file:
        if not args.file.is_file():
            parser.error("只接受已保存的普通文件；不接受 /dev 下的字符设备")
        with args.file.open("rb") as stream:
            while chunk := stream.read(4096):
                consume(chunk)
    else:
        try:
            consume(bytes.fromhex(args.hex))
        except ValueError as exc:
            parser.error(str(exc))
    print(json.dumps({"samples": samples, "other_frames": other, "invalid_samples": invalid,
                      "checksum_errors": decoder.checksum_errors, "length_errors": decoder.length_errors,
                      "discarded_bytes": decoder.discarded_bytes, "trailing_bytes": len(decoder.buffer)}, ensure_ascii=False), file=sys.stderr)


if __name__ == "__main__":
    main()
