"""Use MTi-630R yaw changes as guarded relative motor position commands."""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial")
    raise SystemExit(1)


def wrap_degrees(angle: float) -> float:
    """Keep an angle difference in the range -180 to +180 degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive MTi yaw and command matching relative motor positions."
    )
    parser.add_argument("--listen-host", default="0.0.0.0", help="local UDP address (default: all)")
    parser.add_argument("--listen-port", type=int, default=5005, help="local UDP port (default: 5005)")
    parser.add_argument("--transport", choices=("tcp", "udp"), default="tcp", help="yaw transport (default: tcp)")
    parser.add_argument("--sensor-ip", default="10.196.173.57", help="expected MTi PC IP address")
    parser.add_argument("--sensor-port", type=int, default=5006, help="remote TCP port (default: 5006)")
    parser.add_argument("--esp-port", default="COM7", help="ESP32 serial port (default: COM7)")
    parser.add_argument("--baud", type=int, default=115200, help="ESP32 serial baud rate")
    parser.add_argument("--deadband", type=float, default=0.05, help="yaw degrees before moving (default: 0.05)")
    parser.add_argument(
        "--gain",
        type=float,
        default=1.2,
        help="multiply measured yaw speed to help the motor catch up (default: 1.2)",
    )
    parser.add_argument("--min-speed", type=int, default=10, help="minimum moving speed in dps (default: 10)")
    parser.add_argument("--max-speed", type=int, default=720, help="maximum motor speed in dps (default: 720)")
    parser.add_argument("--watchdog", type=float, default=0.5, help="stop after this many seconds without yaw")
    parser.add_argument(
        "--command-period",
        type=float,
        default=0.01,
        help="minimum seconds between changing motor commands (default: 0.01)",
    )
    parser.add_argument(
        "--speed-step",
        type=int,
        default=1,
        help="round speed to this many dps to suppress sensor noise (default: 1)",
    )
    parser.add_argument(
        "--latency-sample-every",
        type=int,
        default=10,
        help="record one ESP32 timing acknowledgement every N position commands (default: 10)",
    )
    parser.add_argument(
        "--latency-csv",
        type=Path,
        default=Path("logs/yaw_motor_latency.csv"),
        help="CSV file for timestamped latency samples",
    )
    parser.add_argument(
        "--imu-clock-offset-ms",
        type=float,
        default=0.0,
        help="bridge clock minus IMU-PC clock in ms; 0 assumes synchronized clocks",
    )
    parser.add_argument("--duration", type=float, default=0, help="seconds to run; 0 runs until Ctrl+C")
    parser.add_argument(
        "--arm",
        action="store_true",
        help="allow serial commands to the motor; without this flag the script only prints decisions",
    )
    args = parser.parse_args()
    if not 1 <= args.listen_port <= 65535 or not 1 <= args.sensor_port <= 65535:
        parser.error("--listen-port and --sensor-port must be between 1 and 65535")
    if args.deadband < 0 or args.gain <= 0:
        parser.error("--deadband must be nonnegative and --gain must be greater than 0")
    if not 1 <= args.max_speed <= 720:
        parser.error("--max-speed must be between 1 and 720 dps")
    if not 1 <= args.min_speed <= args.max_speed:
        parser.error("--min-speed must be between 1 and --max-speed")
    if args.watchdog <= 0 or args.command_period <= 0 or args.duration < 0:
        parser.error("--watchdog and --command-period must be greater than 0; --duration cannot be negative")
    if not 1 <= args.speed_step <= args.max_speed:
        parser.error("--speed-step must be between 1 and --max-speed")
    if args.latency_sample_every < 1:
        parser.error("--latency-sample-every must be at least 1")
    return args


def open_motor(args: argparse.Namespace):
    if not args.arm:
        return None
    port = serial.Serial()
    port.port = args.esp_port
    port.baudrate = args.baud
    port.timeout = 0
    port.write_timeout = 1.0
    # Do not use DTR/RTS, which can reset or boot the XIAO ESP32-S3.
    port.dtr = False
    port.rts = False
    port.open()
    return port


def send_motor(port, command: str, echo: bool = True) -> None:
    if port is not None:
        port.write((command + "\n").encode("ascii"))
        port.flush()
    if echo:
        print(f"MOTOR> {command}")


def read_yaw(packet: bytes) -> tuple[float, int] | None:
    try:
        value = json.loads(packet.decode("utf-8"))
        yaw = float(value["yaw"])
        imu_tx_ns = int(value.get("imu_tx_ns", 0))
    except (UnicodeDecodeError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return None
    return (yaw, imu_tx_ns) if math.isfinite(yaw) else None


def record_esp32_timing(port, pending, writer, buffer: bytes, imu_clock_offset_ns: int) -> bytes:
    """Read compact ESP32 timing replies without blocking control updates."""
    if port is None or port.in_waiting == 0:
        return buffer

    buffer += port.read(port.in_waiting)
    while b"\n" in buffer:
        raw_line, buffer = buffer.split(b"\n", 1)
        line = raw_line.decode("ascii", errors="replace").strip()
        if not line.startswith("LAT,"):
            if line:
                print(f"ESP32> {line}")
            continue

        fields = line.split(",")
        if len(fields) not in (5, 6):
            print(f"ESP32> invalid timing reply: {line}")
            continue
        try:
            sequence = int(fields[1])
            imu_tx_ns = int(fields[2])
            bridge_tx_ns = int(fields[3])
            esp_rx_us = int(fields[4])
            esp_can_tx_us = int(fields[5]) if len(fields) == 6 else 0
        except ValueError:
            print(f"ESP32> invalid timing values: {line}")
            continue

        received_ns = time.time_ns()
        sample = pending.pop(sequence, None)
        bridge_rx_ns = sample["bridge_rx_ns"] if sample else 0
        if writer and imu_tx_ns and bridge_rx_ns:
            writer.writerow(
                {
                    "sequence": sequence,
                    "imu_tx_ns": imu_tx_ns,
                    "bridge_rx_ns": bridge_rx_ns,
                    "bridge_tx_ns": bridge_tx_ns,
                    "esp_rx_us": esp_rx_us,
                    "esp_can_tx_us": esp_can_tx_us,
                    "esp_ack_rx_ns": received_ns,
                    "imu_to_bridge_ms": (
                        bridge_rx_ns - (imu_tx_ns + imu_clock_offset_ns)
                    ) / 1_000_000,
                    "bridge_to_esp_ack_ms": (received_ns - bridge_tx_ns) / 1_000_000,
                    "imu_to_esp_ack_ms": (
                        received_ns - (imu_tx_ns + imu_clock_offset_ns)
                    ) / 1_000_000,
                    "esp_serial_to_can_tx_ms": (
                        (esp_can_tx_us - esp_rx_us) / 1_000
                        if esp_can_tx_us and esp_rx_us
                        else ""
                    ),
                }
            )
            print(
                f"latency seq={sequence} imu->bridge="
                f"{(bridge_rx_ns - (imu_tx_ns + imu_clock_offset_ns)) / 1_000_000:.2f} ms "
                f"bridge->ESP ack={(received_ns - bridge_tx_ns) / 1_000_000:.2f} ms "
                f"ESP serial->CAN="
                f"{((esp_can_tx_us - esp_rx_us) / 1_000) if esp_can_tx_us else 0:.2f} ms"
            )
    return buffer


def main() -> int:
    args = parse_args()
    receiver = None
    tcp_buffer = b""
    try:
        if args.transport == "udp":
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind((args.listen_host, args.listen_port))
        else:
            receiver = socket.create_connection((args.sensor_ip, args.sensor_port), timeout=5)
        receiver.settimeout(0.1)
    except OSError as error:
        print(f"Cannot open {args.transport.upper()} yaw connection: {error}", file=sys.stderr)
        return 1

    try:
        motor = open_motor(args)
    except serial.SerialException as error:
        receiver.close()
        print(f"Cannot open {args.esp_port}: {error}", file=sys.stderr)
        return 1

    last_motion_yaw = None
    last_motion_yaw_at = None
    last_yaw_time = None
    last_command_at = 0.0
    position_commands = 0
    last_tracking_report_at = 0.0
    timing_buffer = b""
    pending_timing: dict[int, dict[str, int]] = {}
    command_sequence = 0
    imu_clock_offset_ns = round(args.imu_clock_offset_ms * 1_000_000)
    latency_file = None
    latency_writer = None
    started = time.monotonic()
    if args.transport == "udp":
        print(f"Listening for MTi yaw on UDP {args.listen_host}:{args.listen_port}")
        print(f"Expected sensor PC: {args.sensor_ip}")
    else:
        print(f"Connected to MTi TCP stream at {args.sensor_ip}:{args.sensor_port}")
    print("Mode: ARMED" if args.arm else "Mode: DRY RUN (use --arm to move the motor)")
    print("Yaw changes accumulate until the deadband is reached, then become relative position commands.")
    print(f"Control update period: {args.command_period:.3f} s")
    print(
        f"Timing samples: every {args.latency_sample_every} position commands -> "
        f"{args.latency_csv}"
    )
    print(f"IMU clock correction: {args.imu_clock_offset_ms:.3f} ms")

    try:
        args.latency_csv.parent.mkdir(parents=True, exist_ok=True)
        latency_file = args.latency_csv.open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        latency_writer = csv.DictWriter(
            latency_file,
            fieldnames=(
                "sequence",
                "imu_tx_ns",
                "bridge_rx_ns",
                "bridge_tx_ns",
                "esp_rx_us",
                "esp_can_tx_us",
                "esp_ack_rx_ns",
                "imu_to_bridge_ms",
                "bridge_to_esp_ack_ms",
                "imu_to_esp_ack_ms",
                "esp_serial_to_can_tx_ms",
            ),
        )
        latency_writer.writeheader()
    except OSError as error:
        print(f"Cannot open latency CSV {args.latency_csv}: {error}", file=sys.stderr)
        if motor is not None:
            motor.close()
        receiver.close()
        return 1

    try:
        while args.duration == 0 or time.monotonic() - started < args.duration:
            now = time.monotonic()
            timing_buffer = record_esp32_timing(
                motor, pending_timing, latency_writer, timing_buffer,
                imu_clock_offset_ns
            )
            try:
                if args.transport == "udp":
                    packet, sender = receiver.recvfrom(4096)
                else:
                    while b"\n" not in tcp_buffer:
                        received = receiver.recv(4096)
                        if not received:
                            raise ConnectionError("remote MTi stream closed")
                        tcp_buffer += received
                    packet, tcp_buffer = tcp_buffer.split(b"\n", 1)
                    sender = (args.sensor_ip, args.sensor_port)
            except socket.timeout:
                timing_buffer = record_esp32_timing(
                    motor, pending_timing, latency_writer, timing_buffer,
                    imu_clock_offset_ns
                )
                if last_yaw_time and now - last_yaw_time > args.watchdog:
                    print("Yaw watchdog expired; the last finite position target remains active.")
                    last_yaw_time = None
                continue
            except ConnectionError as error:
                print(f"Yaw stream ended: {error}", file=sys.stderr)
                break

            if args.transport == "udp" and args.sensor_ip and sender[0] != args.sensor_ip:
                print(f"Ignored UDP packet from unexpected sender {sender[0]}")
                continue
            yaw_packet = read_yaw(packet)
            if yaw_packet is None:
                continue
            yaw, imu_tx_ns = yaw_packet
            bridge_rx_ns = time.time_ns()
            last_yaw_time = now

            if last_motion_yaw is None:
                last_motion_yaw = yaw
                last_motion_yaw_at = now
                print(f"Yaw baseline set to {last_motion_yaw:.2f} degrees. Motor remains stopped.")
                send_motor(motor, "s")
                continue

            delta = wrap_degrees(yaw - last_motion_yaw)
            if abs(delta) >= args.deadband and now - last_command_at >= args.command_period:
                elapsed = max(now - last_motion_yaw_at, args.command_period)
                yaw_speed_dps = abs(delta) / elapsed
                speed = round(yaw_speed_dps * args.gain / args.speed_step) * args.speed_step
                speed = min(args.max_speed, max(args.min_speed, speed))
                command_sequence += 1
                timing_sample = (
                    motor is not None
                    and command_sequence % args.latency_sample_every == 0
                )
                bridge_tx_ns = time.time_ns()
                if timing_sample:
                    command = (
                        f"n {delta:.2f} {speed} {command_sequence} "
                        f"{imu_tx_ns} {bridge_tx_ns}"
                    )
                    pending_timing[command_sequence] = {
                        "bridge_rx_ns": bridge_rx_ns,
                        "bridge_tx_ns": bridge_tx_ns,
                    }
                else:
                    command = f"n {delta:.2f} {speed}"
                send_motor(motor, command, echo=False)
                position_commands += 1
                if now - last_tracking_report_at >= 0.5:
                    print(
                        f"tracking: yaw={yaw:7.2f} delta={delta:7.2f} "
                        f"yaw_speed={yaw_speed_dps:6.1f} dps command={command} "
                        f"sent={position_commands}"
                    )
                    position_commands = 0
                    last_tracking_report_at = now
                last_motion_yaw = yaw
                last_motion_yaw_at = now
                last_command_at = now
            timing_buffer = record_esp32_timing(
                motor, pending_timing, latency_writer, timing_buffer,
                imu_clock_offset_ns
            )
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            if motor is not None:
                send_motor(motor, "s")
                motor.close()
        finally:
            if latency_file is not None:
                latency_file.close()
            receiver.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
