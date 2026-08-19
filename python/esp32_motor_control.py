"""Control the MF4015V2 through the XIAO ESP32-S3 serial interface."""

from __future__ import annotations

import argparse
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is required: python -m pip install pyserial")
    raise SystemExit(1)


MIN_SPEED_DPS = 0
MAX_SPEED_DPS = 360


def normalize_command(text: str) -> str:
    parts = text.strip().lower().split()
    if parts in (["r"], ["s"]):
        return parts[0]

    if len(parts) == 2 and parts[0] == "m":
        try:
            speed = int(parts[1])
        except ValueError as error:
            raise ValueError("Speed must be an integer from 0 to 360.") from error

        if not MIN_SPEED_DPS <= speed <= MAX_SPEED_DPS:
            raise ValueError("Speed must be an integer from 0 to 360.")
        return f"m {speed}"

    raise ValueError("Use r, m <0..360>, or s.")


def print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        print(f"{port.device}: {port.description}")


def read_serial(port: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            line = port.readline()
        except serial.SerialException as error:
            if not stop_event.is_set():
                print(f"\nSerial read failed: {error}")
            return

        if line:
            print(f"ESP32> {line.decode('utf-8', errors='replace').rstrip()}")


def send_command(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()
    print(f"PC   > {command}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send r, m <0..360>, and s commands to the ESP32 motor controller."
    )
    parser.add_argument("--port", default="COM7", help="ESP32 serial port (default: COM7)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--list-ports", action="store_true", help="List serial ports and exit")
    parser.add_argument("--command", help="Send one command and exit, for example: --command r")
    parser.add_argument(
        "--wait",
        type=float,
        default=1.5,
        help="Seconds to read replies in one-command mode (default: 1.5)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_ports:
        print_ports()
        return 0

    one_command = None
    if args.command is not None:
        try:
            one_command = normalize_command(args.command)
        except ValueError as error:
            print(f"Invalid command: {error}")
            return 2

    try:
        port = serial.Serial(
            args.port,
            args.baud,
            timeout=0.1,
            write_timeout=1.0,
        )
    except serial.SerialException as error:
        print(f"Cannot open {args.port}: {error}")
        return 1

    stop_event = threading.Event()
    reader = threading.Thread(target=read_serial, args=(port, stop_event), daemon=True)
    reader.start()

    try:
        time.sleep(1.2)
        print(f"Connected to {args.port} at {args.baud} baud.")

        if one_command is not None:
            send_command(port, one_command)
            time.sleep(max(0.0, args.wait))
            return 0

        print("Commands: r | m <0..360> | s | q")
        while True:
            try:
                text = input("motor> ").strip()
            except EOFError:
                break

            if text.lower() in {"q", "quit", "exit"}:
                break
            if not text:
                continue

            try:
                command = normalize_command(text)
            except ValueError as error:
                print(f"Invalid command: {error}")
                continue

            send_command(port, command)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            send_command(port, "s")
            time.sleep(0.3)
        except serial.SerialException:
            pass
        stop_event.set()
        reader.join(timeout=0.5)
        port.close()
        print("Serial port closed.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
