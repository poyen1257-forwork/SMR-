import argparse
import math
import sys
import time
from dataclasses import dataclass, field


FRAME_HEADER = 0x55
FRAME_SIZE = 11
READ_VERSION_COMMAND = bytes.fromhex("FF AA 27 2E 00")


def int16_le(low_byte, high_byte):
    value = low_byte | (high_byte << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def checksum_ok(frame):
    return (sum(frame[:10]) & 0xFF) == frame[10]


def calc_gravity_roll_pitch(acc_g):
    ax, ay, az = acc_g
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def normalize_angle(degrees):
    while degrees > 180:
        degrees -= 360
    while degrees <= -180:
        degrees += 360
    return degrees


def calc_gravity_mag_pose(acc_g, mag_raw, fallback_yaw):
    roll, pitch = calc_gravity_roll_pitch(acc_g)
    mx, my, mz = mag_raw
    if abs(mx) + abs(my) + abs(mz) < 1e-6:
        return [roll, pitch, fallback_yaw]

    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)

    mag_x = mx * cp + mz * sp
    mag_y = mx * sr * sp + my * cr - mz * sr * cp
    yaw = math.degrees(math.atan2(-mag_y, mag_x))
    return [roll, pitch, normalize_angle(yaw)]


def import_serial_modules():
    try:
        import serial
        from serial.tools import list_ports
    except ModuleNotFoundError:
        print("pyserial is not installed.")
        print(f"Install it with: {sys.executable} -m pip install pyserial")
        raise SystemExit(1)

    return serial, list_ports


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


class WitParser:
    def __init__(self):
        self.frame = bytearray()
        self.data = ImuData()

    def feed(self, payload):
        parsed = []
        for byte in payload:
            self.data.total_bytes += 1
            item = self.feed_byte(byte)
            if item:
                parsed.append(item)
        return parsed

    def feed_byte(self, byte):
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

        return self.parse_frame(frame)

    def parse_frame(self, frame):
        frame_type = frame[1]
        x = int16_le(frame[2], frame[3])
        y = int16_le(frame[4], frame[5])
        z = int16_le(frame[6], frame[7])
        t = int16_le(frame[8], frame[9])

        self.data.valid_frames += 1

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


class BwtLineParser:
    def __init__(self):
        self.buffer = bytearray()
        self.total_lines = 0
        self.valid_lines = 0

    def feed(self, payload):
        parsed = []
        self.buffer.extend(payload)

        while True:
            line_end = self.buffer.find(b"\r\n")
            if line_end < 0:
                self._trim_noise()
                break

            line = bytes(self.buffer[:line_end])
            del self.buffer[: line_end + 2]
            self.total_lines += 1

            item = self.parse_line(line)
            if item:
                self.valid_lines += 1
                parsed.append(item)

        return parsed

    def _trim_noise(self):
        header_index = self.buffer.find(b"WT")
        if header_index > 0:
            del self.buffer[:header_index]
        if len(self.buffer) > 512:
            del self.buffer[:-64]

    def parse_line(self, line):
        if not line.startswith(b"WT") or len(line) < 52:
            return None

        device_id = line[:12].decode("ascii", errors="replace")
        payload = line[12:52]

        yy, month, day, hour, minute, second = payload[:6]
        millisecond = payload[6] | (payload[7] << 8)
        values = [int16_le(payload[i], payload[i + 1]) for i in range(8, 40, 2)]

        acc_g = [raw / 32768 * 16 for raw in values[0:3]]
        gyro_dps = [raw / 32768 * 2000 for raw in values[3:6]]
        mag_ut = [raw / 10 for raw in values[6:9]]
        angle_f09_deg = [raw / 32768 * 180 for raw in values[9:12]]
        device_angle_deg = angle_f09_deg
        mag_raw = mag_ut
        gravity_roll, gravity_pitch = calc_gravity_roll_pitch(acc_g)
        gravity_angle_deg = [gravity_roll, gravity_pitch, device_angle_deg[2]]
        gravity_mag_angle_deg = calc_gravity_mag_pose(acc_g, mag_raw, device_angle_deg[2])
        temperature_c = values[12] / 100

        return {
            "device_id": device_id,
            "time": f"20{yy:02d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d}.{millisecond:03d}",
            "values": values,
            "acc_g": acc_g,
            "gyro_dps": gyro_dps,
            "angle_deg": device_angle_deg,
            "device_angle_deg": device_angle_deg,
            "gravity_angle_deg": gravity_angle_deg,
            "gravity_mag_angle_deg": gravity_mag_angle_deg,
            "mag_ut": mag_ut,
            "angle_f09_deg": angle_f09_deg,
            "mag_raw": mag_raw,
            "temperature_c": temperature_c,
            "extra": values[13:16],
            "raw_line": line,
        }


def list_ports():
    _, serial_ports = import_serial_modules()
    ports = list(serial_ports.comports())
    if not ports:
        print("No COM ports found.")
        return []

    print("Available COM ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    return ports


def format_frame(label, values, temperature):
    if label == "MAG":
        return f"{label:<5} x={values[0]:>6.0f}, y={values[1]:>6.0f}, z={values[2]:>6.0f}"

    unit = {"ACC": "g", "GYRO": "deg/s", "ANGLE": "deg"}.get(label, "raw")
    temp = "" if temperature is None else f", T={temperature:.2f} C"
    return f"{label:<5} x={values[0]:>9.3f}, y={values[1]:>9.3f}, z={values[2]:>9.3f} {unit}{temp}"


def print_status(parser):
    data = parser.data
    print(
        f"Status: bytes={data.total_bytes}, valid_frames={data.valid_frames}, "
        f"checksum_errors={data.checksum_errors}"
    )
    print(
        f"LATEST ACC[g]=({data.acc_g[0]:.3f}, {data.acc_g[1]:.3f}, {data.acc_g[2]:.3f}) | "
        f"GYRO[dps]=({data.gyro_dps[0]:.3f}, {data.gyro_dps[1]:.3f}, {data.gyro_dps[2]:.3f}) | "
        f"ANGLE[deg]=({data.angle_deg[0]:.3f}, {data.angle_deg[1]:.3f}, {data.angle_deg[2]:.3f}) | "
        f"MAG=({data.mag_raw[0]}, {data.mag_raw[1]}, {data.mag_raw[2]})"
    )


def print_bwt_frame(frame):
    display_time = frame["time"].split(" ", 1)[1]
    angle = frame["device_angle_deg"] if frame["angle_source"] == "device" else frame["gravity_angle_deg"]
    angle_label = "感測器原始角度" if frame["angle_source"] == "device" else "重力計算角度"

    print(f"時間：{display_time}")
    print(
        "三軸加速度："
        f"X={frame['acc_g'][0]:.3f} g, "
        f"Y={frame['acc_g'][1]:.3f} g, "
        f"Z={frame['acc_g'][2]:.3f} g"
    )
    print(
        "三軸角速度："
        f"X={frame['gyro_dps'][0]:.3f} deg/s, "
        f"Y={frame['gyro_dps'][1]:.3f} deg/s, "
        f"Z={frame['gyro_dps'][2]:.3f} deg/s"
    )
    if frame["show_compare_angle"]:
        print(
            "三軸角度（感測器原始角度）："
            f"Roll={frame['device_angle_deg'][0]:.3f} deg, "
            f"Pitch={frame['device_angle_deg'][1]:.3f} deg, "
            f"Yaw={frame['device_angle_deg'][2]:.3f} deg"
        )
        print(
            "三軸角度（重力計算角度）："
            f"Roll={frame['gravity_angle_deg'][0]:.3f} deg, "
            f"Pitch={frame['gravity_angle_deg'][1]:.3f} deg, "
            f"Yaw={frame['gravity_angle_deg'][2]:.3f} deg"
        )
    else:
        print(
            f"三軸角度（{angle_label}）："
            f"Roll={angle[0]:.3f} deg, "
            f"Pitch={angle[1]:.3f} deg, "
            f"Yaw={angle[2]:.3f} deg"
        )
    print()


def print_bwt_debug_fields(frame):
    values = frame["values"]
    display_time = frame["time"].split(" ", 1)[1]
    print(f"時間：{display_time}")
    print("原始欄位：")
    for index, value in enumerate(values):
        print(f"  F{index:02d} = {value}")
    print(
        "磁場 F06~F08："
        f"X={frame['mag_ut'][0]:.3f} uT, "
        f"Y={frame['mag_ut'][1]:.3f} uT, "
        f"Z={frame['mag_ut'][2]:.3f} uT"
    )
    print(
        "WitMotion 官方角度 F09~F11："
        f"Roll={frame['angle_f09_deg'][0]:.3f} deg, "
        f"Pitch={frame['angle_f09_deg'][1]:.3f} deg, "
        f"Yaw={frame['angle_f09_deg'][2]:.3f} deg"
    )
    print(
        "目前視覺化預設使用：WitMotion 官方角度 F09~F11"
    )
    print(
        "目前姿態（重力+磁場計算）："
        f"Roll={frame['gravity_mag_angle_deg'][0]:.3f} deg, "
        f"Pitch={frame['gravity_mag_angle_deg'][1]:.3f} deg, "
        f"Yaw={frame['gravity_mag_angle_deg'][2]:.3f} deg"
    )
    print(
        "目前顯示角度（F09~F11）："
        f"Roll={frame['angle_deg'][0]:.3f} deg, "
        f"Pitch={frame['angle_deg'][1]:.3f} deg, "
        f"Yaw={frame['angle_deg'][2]:.3f} deg"
    )
    print()


def run_read(args):
    serial, _ = import_serial_modules()
    wit_parser = WitParser()
    bwt_parser = BwtLineParser()
    latest_bwt_frame = None
    start_time = time.time()
    last_display = 0.0

    print(f"開啟序列埠：{args.port}，鮑率：{args.baud}")
    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        print("開始讀取 BWT901CL 資料，按 Ctrl+C 停止。")
        print()
        while True:
            now = time.time()
            if args.duration is not None and now - start_time >= args.duration:
                return

            payload = ser.read(args.read_size)
            if args.raw_bytes and payload:
                print("BYTES " + " ".join(f"{byte:02X}" for byte in payload))

            for frame in bwt_parser.feed(payload):
                if args.raw_frame:
                    print("RAW-LINE " + " ".join(f"{byte:02X}" for byte in frame["raw_line"]))
                frame["angle_source"] = args.angle_source
                frame["show_compare_angle"] = args.compare_angle
                latest_bwt_frame = frame

            for label, values, temperature, frame in wit_parser.feed(payload):
                if args.raw_frame:
                    print("RAW " + " ".join(f"{byte:02X}" for byte in frame))
                print(format_frame(label, values, temperature))

            now = time.time()
            if latest_bwt_frame and now - last_display >= args.display_interval:
                if args.debug_fields:
                    print_bwt_debug_fields(latest_bwt_frame)
                else:
                    print_bwt_frame(latest_bwt_frame)
                last_display = now


def run_diagnose(args):
    serial, _ = import_serial_modules()
    list_ports()

    bauds = args.baud or [9600, 19200, 38400, 57600, 115200, 230400]
    print("\nScanning read-version response:")
    for baud in bauds:
        try:
            with serial.Serial(args.port, baud, timeout=0.4) as ser:
                ser.reset_input_buffer()
                ser.write(READ_VERSION_COMMAND)
                ser.flush()
                time.sleep(0.4)
                data = ser.read(200)
            print(f"  baud={baud:<6} response_bytes={len(data):<3} raw={data.hex(' ').upper()}")
        except Exception as exc:
            print(f"  baud={baud:<6} error={exc}")

    print("\nScanning passive stream at preferred baud:")
    wit_parser = WitParser()
    bwt_parser = BwtLineParser()
    start = time.time()
    with serial.Serial(args.port, args.preferred_baud, timeout=1) as ser:
        while time.time() - start < args.duration:
            payload = ser.read(256)
            bwt_parser.feed(payload)
            wit_parser.feed(payload)
    print_status(wit_parser)
    print(f"BWT lines: total={bwt_parser.total_lines}, valid={bwt_parser.valid_lines}")


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Read or diagnose WitMotion BWT901CL data.")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    list_parser = subparsers.add_parser("list", help="List COM ports")
    list_parser.set_defaults(func=lambda args: list_ports())

    read_parser = subparsers.add_parser("read", help="Read continuous BWT901CL stream")
    read_parser.add_argument("--port", required=True, help="COM port, for example COM4")
    read_parser.add_argument("--baud", type=int, default=9600, help="Baud rate, default: 9600")
    read_parser.add_argument("--duration", type=float, help="Stop after this many seconds")
    read_parser.add_argument("--display-interval", type=float, default=0.1, help="Display interval seconds, default: 0.1")
    read_parser.add_argument("--read-size", type=int, default=256)
    read_parser.add_argument("--raw-bytes", action="store_true")
    read_parser.add_argument("--raw-frame", action="store_true")
    read_parser.add_argument("--debug-fields", action="store_true", help="Show all raw fields in the WT data line")
    read_parser.add_argument(
        "--angle-source",
        choices=["gravity", "device"],
        default="device",
        help="Angle display source: gravity uses acceleration roll/pitch, device uses original angle fields",
    )
    read_parser.add_argument("--compare-angle", action="store_true", help="Show both device angle and gravity angle")
    read_parser.set_defaults(func=run_read)

    diag_parser = subparsers.add_parser("diagnose", help="Scan baud rates and passive stream")
    diag_parser.add_argument("--port", required=True, help="COM port, for example COM4")
    diag_parser.add_argument("--baud", type=int, action="append", help="Baud to scan; can be repeated")
    diag_parser.add_argument("--preferred-baud", type=int, default=9600)
    diag_parser.add_argument("--duration", type=float, default=5.0)
    diag_parser.set_defaults(func=run_diagnose)

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
