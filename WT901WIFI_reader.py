import argparse
import socket
import sys
import time
from dataclasses import dataclass, field


FRAME_HEADER = 0x55
FRAME_SIZE = 11


def int16_le(low_byte, high_byte):
    value = low_byte | (high_byte << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def checksum_ok(frame):
    return (sum(frame[:10]) & 0xFF) == frame[10]


@dataclass
class ImuData:
    acc_g: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gyro_dps: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    angle_deg: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mag_raw: list = field(default_factory=lambda: [0, 0, 0])
    temperature_c: float = 0.0
    valid_frames: int = 0
    checksum_errors: int = 0
    total_bytes: int = 0
    updated: bool = False


class WitMotionParser:
    def __init__(self):
        self.frame = bytearray()
        self.data = ImuData()

    def feed(self, payload):
        parsed_frames = []
        for byte in payload:
            self.data.total_bytes += 1
            parsed = self._feed_byte(byte)
            if parsed:
                parsed_frames.append(parsed)
        return parsed_frames

    def _feed_byte(self, byte):
        if len(self.frame) == 0 and byte != FRAME_HEADER:
            return None

        self.frame.append(byte)

        if len(self.frame) == 2 and not (0x50 <= self.frame[1] <= 0x5F):
            self.frame.clear()
            if byte == FRAME_HEADER:
                self.frame.append(byte)
            return None

        if len(self.frame) < FRAME_SIZE:
            return None

        frame = bytes(self.frame)
        self.frame.clear()

        if not checksum_ok(frame):
            self.data.checksum_errors += 1
            return None

        return self._parse_frame(frame)

    def _parse_frame(self, frame):
        frame_type = frame[1]
        x = int16_le(frame[2], frame[3])
        y = int16_le(frame[4], frame[5])
        z = int16_le(frame[6], frame[7])
        t = int16_le(frame[8], frame[9])

        self.data.valid_frames += 1
        self.data.updated = True

        if frame_type == 0x51:
            self.data.acc_g = [x / 32768 * 16, y / 32768 * 16, z / 32768 * 16]
            self.data.temperature_c = t / 100
            return "ACC", self.data.acc_g, self.data.temperature_c, frame

        if frame_type == 0x52:
            self.data.gyro_dps = [x / 32768 * 2000, y / 32768 * 2000, z / 32768 * 2000]
            self.data.temperature_c = t / 100
            return "GYRO", self.data.gyro_dps, self.data.temperature_c, frame

        if frame_type == 0x53:
            self.data.angle_deg = [x / 32768 * 180, y / 32768 * 180, z / 32768 * 180]
            self.data.temperature_c = t / 100
            return "ANGLE", self.data.angle_deg, self.data.temperature_c, frame

        if frame_type == 0x54:
            self.data.mag_raw = [x, y, z]
            return "MAG", self.data.mag_raw, None, frame

        return f"TYPE_0x{frame_type:02X}", [x, y, z], t / 100, frame


def format_values(label, values, temperature):
    if label == "MAG":
        return f"{label:<5} x={values[0]:>6.0f}, y={values[1]:>6.0f}, z={values[2]:>6.0f}"

    unit = {
        "ACC": "g",
        "GYRO": "deg/s",
        "ANGLE": "deg",
    }.get(label, "raw")

    temp_text = "" if temperature is None else f", T={temperature:.2f} C"
    return (
        f"{label:<5} x={values[0]:>9.3f}, y={values[1]:>9.3f}, "
        f"z={values[2]:>9.3f} {unit}{temp_text}"
    )


def print_status(parser, prefix=""):
    data = parser.data
    print(
        f"{prefix}valid_frames={data.valid_frames}, bytes={data.total_bytes}, "
        f"checksum_errors={data.checksum_errors}"
    )
    print(
        "LATEST "
        f"ACC[g]=({data.acc_g[0]:.3f}, {data.acc_g[1]:.3f}, {data.acc_g[2]:.3f}) | "
        f"GYRO[dps]=({data.gyro_dps[0]:.3f}, {data.gyro_dps[1]:.3f}, {data.gyro_dps[2]:.3f}) | "
        f"ANGLE[deg]=({data.angle_deg[0]:.3f}, {data.angle_deg[1]:.3f}, {data.angle_deg[2]:.3f}) | "
        f"MAG=({data.mag_raw[0]}, {data.mag_raw[1]}, {data.mag_raw[2]})"
    )


def import_serial_modules():
    try:
        import serial
        from serial.tools import list_ports
    except ModuleNotFoundError:
        print("pyserial is not installed for this Python.")
        print("Install it with:")
        print(f"  {sys.executable} -m pip install pyserial")
        raise SystemExit(1)

    return serial, list_ports


def list_serial_ports():
    _, list_ports = import_serial_modules()
    ports = list(list_ports.comports())

    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")


def run_serial(args):
    serial, _ = import_serial_modules()

    if args.list:
        list_serial_ports()
        return

    if not args.port:
        list_serial_ports()
        print("\nPlease run again with --port COMx, for example:")
        print(f"  {sys.executable} {__file__} serial --port COM3")
        return

    parser = WitMotionParser()
    last_status = time.time()

    print(f"Opening serial {args.port} at {args.baud} baud...")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print("Reading WT901 data. Press Ctrl+C to stop.")
        while True:
            payload = ser.read(args.read_size)
            parsed_frames = parser.feed(payload)

            for label, values, temperature, frame in parsed_frames:
                if args.raw:
                    raw_text = " ".join(f"{byte:02X}" for byte in frame)
                    print(f"RAW {raw_text}")
                if args.print_each:
                    print(format_values(label, values, temperature))

            now = time.time()
            if now - last_status >= args.status_interval:
                print_status(parser)
                last_status = now


def run_udp(args):
    parser = WitMotionParser()
    udp_packets = 0
    last_status = time.time()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(1)

    print(f"Listening UDP on {args.host}:{args.port}")
    print("Set WT901WIFI server IP to this PC IP and server port to the same UDP port.")
    print("Reading WT901 data. Press Ctrl+C to stop.")

    while True:
        try:
            payload, remote = sock.recvfrom(args.buffer_size)
        except socket.timeout:
            payload = b""
            remote = None

        if payload:
            udp_packets += 1
            parsed_frames = parser.feed(payload)

            if args.raw_packet:
                preview = " ".join(f"{byte:02X}" for byte in payload[: args.raw_limit])
                if len(payload) > args.raw_limit:
                    preview += " ..."
                print(f"UDP #{udp_packets} from {remote[0]}:{remote[1]}, size={len(payload)}, raw={preview}")

            for label, values, temperature, frame in parsed_frames:
                if args.raw:
                    raw_text = " ".join(f"{byte:02X}" for byte in frame)
                    print(f"RAW {raw_text}")
                if args.print_each:
                    print(format_values(label, values, temperature))

        now = time.time()
        if now - last_status >= args.status_interval:
            print_status(parser, prefix=f"udp_packets={udp_packets}, ")
            last_status = now


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Read WitMotion WT901WIFI data through USB serial or Wi-Fi UDP."
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)

    serial_parser = subparsers.add_parser("serial", help="Read through USB serial COM port")
    serial_parser.add_argument("--list", action="store_true", help="List COM ports and exit")
    serial_parser.add_argument("--port", help="COM port, for example COM3")
    serial_parser.add_argument("--baud", type=int, default=9600, help="Baud rate, default: 9600")
    serial_parser.add_argument("--read-size", type=int, default=256, help="Serial read buffer size")
    serial_parser.add_argument("--status-interval", type=float, default=2.0, help="Status print interval seconds")
    serial_parser.add_argument("--raw", action="store_true", help="Print every valid raw 11-byte frame")
    serial_parser.add_argument("--print-each", action="store_true", help="Print every parsed frame")
    serial_parser.set_defaults(func=run_serial)

    udp_parser = subparsers.add_parser("udp", help="Read through WT901WIFI UDP stream")
    udp_parser.add_argument("--host", default="0.0.0.0", help="Local bind host, default: 0.0.0.0")
    udp_parser.add_argument("--port", type=int, default=1399, help="Local UDP port, default: 1399")
    udp_parser.add_argument("--buffer-size", type=int, default=2048, help="UDP receive buffer size")
    udp_parser.add_argument("--status-interval", type=float, default=2.0, help="Status print interval seconds")
    udp_parser.add_argument("--raw", action="store_true", help="Print every valid raw 11-byte frame")
    udp_parser.add_argument("--raw-packet", action="store_true", help="Print received UDP packet preview")
    udp_parser.add_argument("--raw-limit", type=int, default=64, help="Max UDP raw preview bytes")
    udp_parser.add_argument("--print-each", action="store_true", help="Print every parsed frame")
    udp_parser.set_defaults(func=run_udp)

    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    try:
        args.func(args)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
