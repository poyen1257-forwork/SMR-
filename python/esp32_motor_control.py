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
MIN_ANGLE_DEG = 0
MAX_ANGLE_DEG = 360


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

    if len(parts) == 3 and parts[0] == "n":
        try:
            angle = int(parts[1])
            speed = int(parts[2])
        except ValueError as error:
            raise ValueError("Angle and speed must be integers.") from error

        if not MIN_ANGLE_DEG <= angle <= MAX_ANGLE_DEG:
            raise ValueError("Angle must be an integer from 0 to 360 degrees.")
        if not MIN_SPEED_DPS <= speed <= MAX_SPEED_DPS:
            raise ValueError("Speed must be an integer from 0 to 360 dps.")
        if angle > 0 and speed == 0:
            raise ValueError("Speed must be greater than 0 when angle is not 0.")
        return f"n {angle} {speed}"

    raise ValueError("Use r, m <0..360>, n <0..360> <0..360>, or s.")


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


def execute_command(port: serial.Serial, command: str) -> None:
    parts = command.split()
    if parts[0] != "n":
        send_command(port, command)
        return

    angle = int(parts[1])
    speed = int(parts[2])
    if angle == 0:
        send_command(port, "m 0")
        return

    duration = angle / speed
    print(
        f"Timed move: {angle} degrees at {speed} dps "
        f"for {duration:.3f} seconds."
    )
    print("Press Ctrl+C for emergency stop.")
    send_command(port, f"m {speed}")
    time.sleep(duration)
    send_command(port, "m 0")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send r, m <0..360>, n <angle> <speed>, and s commands to the "
            "ESP32 motor controller."
        )
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
        port = serial.Serial()
        port.port = args.port
        port.baudrate = args.baud
        port.timeout = 0.1
        port.write_timeout = 1.0
        # Avoid toggling the XIAO ESP32-S3 reset/boot control lines.
        port.dtr = False
        port.rts = False
        port.open()
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
            execute_command(port, one_command)
            time.sleep(max(0.0, args.wait))
            return 0

        print("Commands: r | m <0..360> | n <0..360> <0..360> | s | q")
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

            execute_command(port, command)
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
