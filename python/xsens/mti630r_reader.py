"""Continuously read an Xsens MTi-630R with the official XDA Python API."""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import sys
import time
from collections import deque
from pathlib import Path
from threading import Lock

import xsensdeviceapi as xda


class PacketCallback(xda.XsCallback):
    """Keep a small thread-safe queue of the newest XDA packets."""

    def __init__(self, max_packets: int = 20) -> None:
        super().__init__()
        self._packets: deque = deque(maxlen=max_packets)
        self._lock = Lock()

    def onLiveDataAvailable(self, _device, packet) -> None:
        with self._lock:
            self._packets.append(xda.XsDataPacket(packet))

    def get_packet(self):
        with self._lock:
            if not self._packets:
                return None
            return self._packets.popleft()


def find_mti():
    """Return the first MTi/MTi-G port found by the official scanner."""
    ports = xda.XsScanner_scanPorts()
    for index in range(ports.size()):
        port = ports[index]
        device_id = port.deviceId()
        if device_id.isMti() or device_id.isMtig():
            return port
    raise RuntimeError("No MTi device found. Check its power, cable, and COM port.")


def configure_device(device, rate_hz: int, yaw_only: bool) -> None:
    """Request either full 9-axis data or a lightweight yaw tracking stream."""
    if not device.gotoConfig():
        raise RuntimeError("Could not put the MTi into configuration mode.")

    outputs = xda.XsOutputConfigurationArray()
    outputs.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
    outputs.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))
    outputs.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, rate_hz))
    if not yaw_only:
        outputs.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, rate_hz))
        outputs.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, rate_hz))
        outputs.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, rate_hz))

    if not device.setOutputConfiguration(outputs):
        raise RuntimeError("The MTi rejected the requested output configuration.")
    if not device.gotoMeasurement():
        raise RuntimeError("Could not put the MTi into measurement mode.")


def packet_values(packet):
    """Convert one XDA packet into simple Python values."""
    try:
        euler = packet.orientationEuler()
    except RuntimeError:
        return None

    values = {
        "time_s": time.time(),
        "acc_x": float("nan"),
        "acc_y": float("nan"),
        "acc_z": float("nan"),
        "gyro_x": float("nan"),
        "gyro_y": float("nan"),
        "gyro_z": float("nan"),
        "mag_x": float("nan"),
        "mag_y": float("nan"),
        "mag_z": float("nan"),
        "roll": float("nan"),
        "pitch": float("nan"),
        "yaw": float("nan"),
    }
    if packet.containsCalibratedData():
        acc = packet.calibratedAcceleration()
        gyro = packet.calibratedGyroscopeData()
        mag = packet.calibratedMagneticField()
        values.update(
            acc_x=acc[0],
            acc_y=acc[1],
            acc_z=acc[2],
            gyro_x=gyro[0],
            gyro_y=gyro[1],
            gyro_z=gyro[2],
            mag_x=mag[0],
            mag_y=mag[1],
            mag_z=mag[2],
        )
    values.update(roll=euler.x(), pitch=euler.y(), yaw=euler.z())
    return values


def parse_args():
    parser = argparse.ArgumentParser(description="Read Xsens MTi-630R 9-axis data")
    parser.add_argument("--rate", type=int, default=100, help="sensor rate in Hz (default: 100)")
    parser.add_argument(
        "--yaw-only",
        action="store_true",
        help="request only orientation data to maximize the yaw stream rate",
    )
    parser.add_argument(
        "--print-rate", type=float, default=10.0, help="terminal update rate in Hz (default: 10)"
    )
    parser.add_argument(
        "--stream-rate",
        type=float,
        default=100.0,
        help="TCP/UDP yaw stream rate in Hz (default: 100)",
    )
    parser.add_argument("--duration", type=float, default=0, help="seconds to run; 0 runs until Ctrl+C")
    parser.add_argument("--csv", type=Path, help="optional CSV output file")
    parser.add_argument("--udp-host", help="optional receiver IP address for JSON UDP packets")
    parser.add_argument("--udp-port", type=int, default=5005, help="UDP receiver port (default: 5005)")
    parser.add_argument("--tcp-port", type=int, help="optional TCP port that streams newline-delimited JSON")
    args = parser.parse_args()
    if not 1 <= args.rate <= 400:
        parser.error("--rate must be between 1 and 400 Hz")
    if args.print_rate <= 0 or args.stream_rate <= 0:
        parser.error("--print-rate and --stream-rate must be greater than 0")
    if args.duration < 0:
        parser.error("--duration cannot be negative")
    if not 1 <= args.udp_port <= 65535:
        parser.error("--udp-port must be between 1 and 65535")
    if args.tcp_port is not None and not 1 <= args.tcp_port <= 65535:
        parser.error("--tcp-port must be between 1 and 65535")
    return args


def main() -> int:
    args = parse_args()
    control = xda.XsControl_construct()
    if control is None:
        print("Failed to create the XDA control object.", file=sys.stderr)
        return 1

    port = None
    device = None
    callback = None
    csv_file = None
    writer = None
    udp_socket = None
    tcp_server = None
    tcp_client = None

    try:
        print("Scanning for an Xsens MTi...")
        port = find_mti()
        print(
            f"Found {port.deviceId().toXsString()} on "
            f"{port.portName()} at {port.baudrate()} baud"
        )

        if not control.openPort(port.portName(), port.baudrate()):
            raise RuntimeError(f"Could not open {port.portName()}. Close MT Manager and retry.")

        device = control.device(port.deviceId())
        if device is None:
            raise RuntimeError("XDA opened the port but could not create the device object.")

        print(f"Device: {device.productCode()}")
        callback = PacketCallback()
        device.addCallbackHandler(callback)
        configure_device(device, args.rate, args.yaw_only)
        print("Output: yaw-only orientation" if args.yaw_only else "Output: full 9-axis + orientation")

        fieldnames = [
            "time_s",
            "acc_x",
            "acc_y",
            "acc_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "mag_x",
            "mag_y",
            "mag_z",
            "roll",
            "pitch",
            "yaw",
        ]
        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            csv_file = args.csv.open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            print(f"CSV: {args.csv.resolve()}")
        if args.udp_host:
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"UDP: {args.udp_host}:{args.udp_port}")
        if args.tcp_port:
            tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            tcp_server.bind(("0.0.0.0", args.tcp_port))
            tcp_server.listen(1)
            tcp_server.setblocking(False)
            print(f"TCP: listening on 0.0.0.0:{args.tcp_port}")

        print("Reading data. Press Ctrl+C to stop.")
        print("Units: acceleration=m/s^2, gyro=rad/s, magnetic field=a.u., angles=degrees")
        started = time.monotonic()
        next_print = started
        next_stream = started
        print_interval = 1.0 / args.print_rate
        stream_interval = 1.0 / args.stream_rate

        while args.duration == 0 or time.monotonic() - started < args.duration:
            packet = callback.get_packet()
            if packet is None:
                time.sleep(0.001)
                continue

            values = packet_values(packet)
            if values is None:
                continue
            if writer:
                writer.writerow(values)
            if tcp_server and tcp_client is None:
                try:
                    tcp_client, address = tcp_server.accept()
                    print(f"TCP client connected: {address[0]}:{address[1]}")
                except BlockingIOError:
                    pass
            now = time.monotonic()
            if args.yaw_only or now >= next_stream:
                # Send only the newest orientation at the control period. This
                # keeps full 9-axis streams bounded, while yaw-only tracking
                # forwards every sensor callback at the configured sensor rate.
                if not args.yaw_only:
                    next_stream = now + stream_interval
                # Timestamp at the moment this process emits the yaw packet.
                # It is deliberately separate from time_s, which is retained
                # for human-readable logs and CSV compatibility.
                stream_values = {
                    "time_s": values["time_s"],
                    "imu_tx_ns": time.time_ns(),
                    "roll": values["roll"],
                    "pitch": values["pitch"],
                    "yaw": values["yaw"],
                }
                stream_payload = json.dumps(
                    stream_values, allow_nan=False, separators=(",", ":")
                ).encode("utf-8")
                if udp_socket and math.isfinite(values["yaw"]):
                    udp_socket.sendto(stream_payload, (args.udp_host, args.udp_port))
                if tcp_client and math.isfinite(values["yaw"]):
                    try:
                        tcp_client.sendall(stream_payload + b"\n")
                    except OSError:
                        tcp_client.close()
                        tcp_client = None

            if now >= next_print:
                print(
                    "ACC[{acc_x:8.3f} {acc_y:8.3f} {acc_z:8.3f}]  "
                    "GYRO[{gyro_x:8.4f} {gyro_y:8.4f} {gyro_z:8.4f}]  "
                    "MAG[{mag_x:7.3f} {mag_y:7.3f} {mag_z:7.3f}]  "
                    "RPY[{roll:7.2f} {pitch:7.2f} {yaw:7.2f}]".format(**values),
                    flush=True,
                )
                next_print = now + print_interval

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except RuntimeError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    finally:
        if csv_file:
            csv_file.close()
        if udp_socket:
            udp_socket.close()
        if tcp_client:
            tcp_client.close()
        if tcp_server:
            tcp_server.close()
        if device is not None and callback is not None:
            device.removeCallbackHandler(callback)
        if port is not None:
            control.closePort(port.portName())
        control.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
