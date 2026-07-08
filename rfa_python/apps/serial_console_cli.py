"""Tiny serial console for checking unknown instrument command strings."""

from __future__ import annotations

import argparse
import time

try:
    import serial  # type: ignore
except ImportError:
    serial = None


def main() -> None:
    p = argparse.ArgumentParser(description="Send one ASCII command to a serial instrument")
    p.add_argument("port")
    p.add_argument("command")
    p.add_argument("--baudrate", type=int, default=9600)
    p.add_argument("--timeout", type=float, default=1.0)
    p.add_argument("--terminator", default="\\n", help="use \\n, \\r, or \\r\\n")
    p.add_argument("--read-lines", type=int, default=1)
    args = p.parse_args()
    if serial is None:
        raise SystemExit("pyserial is not installed")
    term = args.terminator.encode("utf-8").decode("unicode_escape")
    with serial.Serial(args.port, baudrate=args.baudrate, timeout=args.timeout) as ser:
        ser.reset_input_buffer()
        ser.write((args.command + term).encode("ascii"))
        ser.flush()
        time.sleep(0.05)
        for _ in range(args.read_lines):
            print(ser.readline().decode("ascii", errors="replace").strip())


if __name__ == "__main__":
    main()
