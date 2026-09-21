#!/usr/bin/env python3
"""Read-only bench probe for this board, Dynamixel Protocol 2.0.

Install Tools/requirements.txt into a project-local venv first.
Example: python Tools/imu_probe.py --port COM5 --seconds 30 --csv Build/imu.csv
Use a USB single-wire half-duplex bus adapter; the log UART is a different port.
No torque, position, Flash, baud, or other write instructions are sent.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import struct
import time

HEADER = b"\xff\xff\xfd\x00"


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x8005 if crc & 0x8000 else 0)) & 0xffff
    return crc


def packet(device_id: int, instruction: int, parameters: bytes = b"") -> bytes:
    body = (bytes([instruction]) + parameters).replace(b"\xff\xff\xfd", b"\xff\xff\xfd\xfd")
    frame = HEADER + bytes([device_id]) + struct.pack("<H", len(body) + 2) + body
    return frame + struct.pack("<H", crc16(frame))


class Bus:
    def __init__(self, serial_port, device_id: int = 200):
        self.port = serial_port
        self.device_id = device_id

    def request(self, instruction: int, parameters: bytes = b"", broadcast=False) -> bytes:
        self.port.reset_input_buffer()
        self.port.write(packet(254 if broadcast else self.device_id, instruction, parameters))
        self.port.flush()
        deadline = time.monotonic() + 0.15
        pending = bytearray()
        while time.monotonic() < deadline:
            pending.extend(self.port.read(max(1, self.port.in_waiting)))
            while True:
                start = pending.find(HEADER)
                if start < 0:
                    pending[:] = pending[-3:]
                    break
                if start:
                    del pending[:start]
                if len(pending) < 7:
                    break
                size = 7 + int.from_bytes(pending[5:7], "little")
                if size < 11 or size > 512:
                    del pending[0]
                    continue
                if len(pending) < size:
                    break
                frame = bytes(pending[:size])
                del pending[:size]
                if crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                    continue
                body = frame[7:-2].replace(b"\xff\xff\xfd\xfd", b"\xff\xff\xfd")
                # Ignore local adapter echo and other devices' status packets.
                if frame[4] != self.device_id or len(body) < 2 or body[0] != 0x55:
                    continue
                if body[1]:
                    raise RuntimeError(f"device returned protocol error 0x{body[1]:02x}")
                return body[2:]
        raise TimeoutError("no valid ID 200 status packet; check bus adapter, baud, wiring and power")

    def read(self, address: int, count: int, sync=False) -> bytes:
        params = struct.pack("<HH", address, count)
        if sync:
            data = self.request(0x82, params + bytes([self.device_id]), broadcast=True)
        else:
            data = self.request(0x02, params)
        if len(data) != count:
            raise RuntimeError(f"expected {count} data bytes, got {len(data)}")
        return data


def decode(data: bytes) -> dict:
    if len(data) != 116:  # register 124..239
        raise ValueError("diagnostic block must be 116 bytes")
    gyro_raw = struct.unpack_from("<3h", data, 0)
    xyz = struct.unpack_from("<3e", data, 6)
    norm2 = sum(x * x for x in xyz)
    valid = all(math.isfinite(x) for x in xyz) and norm2 <= 1.02
    w = math.sqrt(max(0.0, 1.0 - norm2)) if valid else float("nan")
    if valid and norm2 > 1.0:
        xyz = tuple(x / math.sqrt(norm2) for x in xyz)
    names = ("sample_count", "gyro_count", "accel_count", "uptime_ms", "gyro_age_ms", "quat_age_ms",
             "spi_errors", "fifo_overflows", "invalid_quaternions", "recovery_count", "rx_overflows",
             "uart_errors", "crc_errors", "malformed_packets", "receive_timeouts", "rx_packets",
             "tx_packets", "cancelled_replies", "reset_flags", "clock_hz", "tx_timeouts", "log_dropped")
    result = dict(zip(names, struct.unpack_from("<22I", data, 24)))
    result.update(ready=bool(data[19] & 1) and valid, who_am_i=data[20], error=data[21],
                  clock_status=data[22], watchdog=bool(data[23]),
                  gyro_x_dps=gyro_raw[0] * .0175, gyro_y_dps=gyro_raw[1] * .0175,
                  gyro_z_dps=gyro_raw[2] * .0175, q_x=xyz[0], q_y=xyz[1], q_z=xyz[2], q_w=w)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="bus adapter COM port, e.g. COM5")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--hz", type=float, default=50)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--sync", action="store_true", help="exercise Sync Read with ID 200 first/alone")
    args = parser.parse_args()
    if not (0 < args.hz <= 100 and args.seconds > 0):
        parser.error("require 0 < hz <= 100 and seconds > 0")
    try:
        import serial
    except ImportError:
        parser.error("pyserial is missing: python -m pip install -r Tools/requirements.txt")
    file = None
    good = faults = not_ready = repeated = 0
    previous = None
    try:
        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            file = args.csv.open("w", newline="", encoding="utf-8-sig")
        writer = None
        with serial.Serial(args.port, 1000000, timeout=.01, write_timeout=.2) as port:
            bus = Bus(port)
            ping = bus.request(1)
            if len(ping) != 3 or int.from_bytes(ping[:2], "little") != 0x4D44:
                raise RuntimeError(f"unexpected model/fw response: {ping.hex()}; this tool targets the custom IMU board")
            print(f"ID=200 model=0x4D44 firmware={ping[2]} baud=1000000; no write commands")
            start = next_sample = next_report = time.monotonic()
            while time.monotonic() - start < args.seconds:
                try:
                    row = decode(bus.read(124, 116, args.sync))
                    row = {"host_seconds": time.monotonic() - start, **row}
                    good += 1
                    not_ready += not row["ready"]
                    repeated += previous == row["sample_count"]
                    previous = row["sample_count"]
                    if file:
                        if writer is None:
                            writer = csv.DictWriter(file, fieldnames=list(row))
                            writer.writeheader()
                        writer.writerow(row)
                    if time.monotonic() >= next_report:
                        print(json.dumps(row, ensure_ascii=False))
                        next_report = time.monotonic() + 1
                except (TimeoutError, RuntimeError, ValueError) as error:
                    faults += 1
                    print(f"communication error: {error}")
                next_sample += 1 / args.hz
                time.sleep(max(0, next_sample - time.monotonic()))
                if next_sample < time.monotonic() - 1:
                    next_sample = time.monotonic()
        print(f"reads={good} errors={faults} not_ready={not_ready} repeated_sample_count={repeated}")
        return 0 if good and not faults and not not_ready else 1
    finally:
        if file:
            file.close()


if __name__ == "__main__":
    raise SystemExit(main())
