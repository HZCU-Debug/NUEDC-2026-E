"""Simple full-duplex UART test for a PC USB-TTL adapter and Raspberry Pi.

Wiring for the Raspberry Pi 40-pin header:
  USB-TTL TX -> Pi pin 10 / GPIO15 / RX
  USB-TTL RX -> Pi pin 8  / GPIO14 / TX
  USB-TTL GND -> Pi GND

Use a 3.3 V TTL adapter. Do not connect its VCC/5 V pin to the Raspberry Pi.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError as error:
    raise SystemExit(
        "pyserial is not installed. Run: python -m pip install pyserial"
    ) from error


def printable(data: bytes) -> str:
    return data.decode("utf-8", errors="replace").rstrip("\r\n")


def open_port(name: str, baud: int, timeout: float) -> serial.Serial:
    try:
        return serial.Serial(
            port=name,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=1.0,
        )
    except serial.SerialException as error:
        raise RuntimeError(f"cannot open {name}: {error}") from error


def list_serial_ports() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1
    for port in ports:
        print(f"{port.device}\t{port.description}\t{port.hwid}")
    return 0


def run_echo(args: argparse.Namespace) -> int:
    with open_port(args.port, args.baud, timeout=0.2) as port:
        port.reset_input_buffer()
        print(f"Echo receiver ready: {port.name} {args.baud} 8N1")
        print("Waiting for newline-terminated messages; press Ctrl+C to stop.")
        while True:
            request = port.readline()
            if not request:
                continue
            print(
                f"RX {len(request):3d} bytes: {printable(request)!r} "
                f"hex={request.hex(' ')}"
            )
            reply = b"PI5_ACK " + request.rstrip(b"\r\n") + b"\n"
            port.write(reply)
            port.flush()
            print(f"TX {len(reply):3d} bytes: {printable(reply)!r}")


def run_sender(args: argparse.Namespace) -> int:
    failures = 0
    with open_port(args.port, args.baud, timeout=0.1) as port:
        port.reset_input_buffer()
        print(f"Sender ready: {port.name} {args.baud} 8N1")
        for sequence in range(1, args.count + 1):
            payload = (
                f"{args.message} seq={sequence} "
                f"unix_ms={time.time_ns() // 1_000_000}\n"
            ).encode("utf-8")
            port.write(payload)
            port.flush()
            print(
                f"TX {len(payload):3d} bytes: {printable(payload)!r} "
                f"hex={payload.hex(' ')}"
            )

            deadline = time.monotonic() + args.reply_timeout
            reply = b""
            while time.monotonic() < deadline:
                reply = port.readline()
                if reply:
                    break
            expected = b"PI5_ACK " + payload.rstrip(b"\r\n") + b"\n"
            if reply == expected:
                print(f"RX {len(reply):3d} bytes: {printable(reply)!r}  [PASS]")
            elif reply:
                failures += 1
                print(
                    f"RX {len(reply):3d} bytes: {printable(reply)!r} "
                    f"hex={reply.hex(' ')}  [UNEXPECTED]"
                )
            else:
                failures += 1
                print("RX timeout: no reply from Raspberry Pi  [FAIL]")

            if sequence < args.count:
                time.sleep(args.interval)

    passed = args.count - failures
    print(f"Result: {passed}/{args.count} round trips passed")
    return 0 if failures == 0 else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PC <-> Raspberry Pi UART test")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="list serial ports on this computer")

    echo = subparsers.add_parser("echo", help="receive and echo on Raspberry Pi")
    echo.add_argument("--port", default="/dev/ttyAMA0")
    echo.add_argument("--baud", type=int, default=115200)

    sender = subparsers.add_parser("send", help="send test messages from PC")
    sender.add_argument("--port", required=True, help="for example COM5")
    sender.add_argument("--baud", type=int, default=115200)
    sender.add_argument("--message", default="PC_TO_PI5_TEST")
    sender.add_argument("--count", type=int, default=5)
    sender.add_argument("--interval", type=float, default=0.5)
    sender.add_argument("--reply-timeout", type=float, default=1.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "list":
            return list_serial_ports()
        if args.command == "echo":
            return run_echo(args)
        return run_sender(args)
    except KeyboardInterrupt:
        print("\nStopped")
        return 130
    except (RuntimeError, serial.SerialException) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
