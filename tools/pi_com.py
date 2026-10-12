#!/usr/bin/env python3
"""Raspberry Pi USB client for the ESP32 conveyor controller."""

import argparse
import sys
import threading

import serial


def read_forever(port: serial.Serial) -> None:
    while port.is_open:
        try:
            line = port.readline()
        except serial.SerialException as exc:
            print(f"Serial error: {exc}", file=sys.stderr)
            return
        if line:
            print(line.decode("utf-8", errors="replace").rstrip())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="e.g. /dev/ttyUSB0 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--send", help="send one controller command and listen")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        reader = threading.Thread(target=read_forever, args=(port,), daemon=True)
        reader.start()

        if args.send is not None:
            port.write((args.send.rstrip("\r\n") + "\n").encode())
            reader.join()
            return 0

        print("Commands: STATUS, SYNC, START, STOP, RUN 0/1. Ctrl-C to quit.")
        print("When ESP32 prints CAMERA, enter 0 (defect) or 1 (pass).")
        print("When ESP32 prints IR2 BLOCKED while running, enter 0 (sour) or 1 (sweet).")
        print("IR2 CLEAR is a sensor status only; do not send another result.")
        try:
            for line in sys.stdin:
                port.write(line.rstrip("\r\n").encode() + b"\n")
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
