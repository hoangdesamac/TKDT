"""Check ESP32 UART or return a fixed classification for each SCAN request."""

import argparse
import sys
import time


def check_connection(uart):
    deadline = time.monotonic() + 8
    next_ping = 0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_ping:
            uart.write(b"P")
            next_ping = time.monotonic() + 1
        message = uart.readline().decode("ascii", errors="replace").strip()
        if message:
            print(f"ESP32 -> Pi: {message}", flush=True)
        if message == "PONG":
            print("UART OK: both transmit and receive work.")
            return True
    print("No PONG. Check power, wiring, UART settings and port.", file=sys.stderr)
    return False


def serve_scans(uart, result):
    print(f"Test mode: every SCAN returns {result}; no camera is used.", flush=True)
    while True:
        message = uart.readline().decode("ascii", errors="replace").strip()
        if not message:
            continue
        print(f"ESP32 -> Pi: {message}", flush=True)
        scan_id = parse_scan_request(message)
        if scan_id is not None:
            send_classification(uart, scan_id, result)


def parse_scan_request(message):
    fields = message.split(",")
    if len(fields) != 2 or fields[0] != "SCAN":
        return None
    value = fields[1]
    if not value or len(value) > 10 or not value.isascii() or not value.isdecimal():
        return None
    scan_id = int(value)
    return scan_id if 0 < scan_id <= 0xFFFFFFFF else None


def send_classification(uart, scan_id, result):
    """Send the camera's classification using the ID from its SCAN request."""
    if not 0 < scan_id <= 0xFFFFFFFF or str(result) not in ("1", "2", "3"):
        raise ValueError("Expected a nonzero 32-bit scan ID and class 1, 2 or 3")
    response = f"RESULT,{scan_id},{result}\n"
    uart.write(response.encode("ascii"))
    print(f"Pi -> ESP32: {response.strip()}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--mode", choices=("ping", "scan"), default="ping")
    parser.add_argument("--result", choices=("1", "2", "3"), default="3")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("Install pySerial: sudo apt install python3-serial", file=sys.stderr)
        return 1

    try:
        with serial.Serial(
            args.port, baudrate=115200, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=0.2, write_timeout=1,
        ) as uart:
            if not check_connection(uart):
                return 1
            if args.mode == "scan":
                serve_scans(uart, args.result)
    except serial.SerialException as error:
        print(f"UART error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nUART program stopped. This does not stop the ESP32 conveyor.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
