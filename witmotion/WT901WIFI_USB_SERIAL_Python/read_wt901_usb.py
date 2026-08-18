import argparse
import time

import serial
from serial.tools import list_ports


def int16_le(low_byte, high_byte):
    value = low_byte | (high_byte << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def checksum_ok(frame):
    return (sum(frame[:10]) & 0xFF) == frame[10]


def parse_frame(frame):
    frame_type = frame[1]
    x = int16_le(frame[2], frame[3])
    y = int16_le(frame[4], frame[5])
    z = int16_le(frame[6], frame[7])
    t = int16_le(frame[8], frame[9])

    if frame_type == 0x51:
        return "ACC[g]", (x / 32768 * 16, y / 32768 * 16, z / 32768 * 16), t / 100
    if frame_type == 0x52:
        return "GYRO[dps]", (x / 32768 * 2000, y / 32768 * 2000, z / 32768 * 2000), t / 100
    if frame_type == 0x53:
        return "ANGLE[deg]", (x / 32768 * 180, y / 32768 * 180, z / 32768 * 180), t / 100
    if frame_type == 0x54:
        return "MAG[raw]", (x, y, z), None
    return f"TYPE 0x{frame_type:02X}", (x, y, z), t / 100


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")


def main():
    parser = argparse.ArgumentParser(description="Read WitMotion WT901WIFI data through USB serial.")
    parser.add_argument("--port", help="COM port, for example COM3")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate, default: 9600")
    parser.add_argument("--raw", action="store_true", help="Print every valid raw 11-byte frame")
    parser.add_argument("--raw-bytes", action="store_true", help="Print received bytes before frame parsing")
    parser.add_argument("--duration", type=float, help="Stop after this many seconds")
    parser.add_argument("--status-interval", type=float, default=5.0, help="Status print interval seconds")
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return

    if not args.port:
        list_serial_ports()
        print("\nRun again with --port COMx, for example:")
        print("  python read_wt901_usb.py --port COM3")
        return

    valid_frames = 0
    checksum_errors = 0
    total_bytes = 0
    frame = bytearray()
    start_time = time.time()
    last_status = time.time()

    print(f"Opening {args.port} at {args.baud} baud...")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print("Reading. Press Ctrl+C to stop.")

        while True:
            if args.duration is not None and time.time() - start_time >= args.duration:
                print(
                    f"Done: bytes={total_bytes}, valid_frames={valid_frames}, "
                    f"checksum_errors={checksum_errors}"
                )
                return

            data = ser.read(256)
            total_bytes += len(data)

            if args.raw_bytes and data:
                raw_bytes = " ".join(f"{byte:02X}" for byte in data)
                print(f"BYTES {raw_bytes}")

            for byte in data:
                if len(frame) == 0 and byte != 0x55:
                    continue

                frame.append(byte)

                if len(frame) == 2 and not (0x50 <= frame[1] <= 0x5F):
                    frame.clear()
                    if byte == 0x55:
                        frame.append(byte)
                    continue

                if len(frame) < 11:
                    continue

                if checksum_ok(frame):
                    valid_frames += 1
                    label, values, temperature = parse_frame(frame)
                    raw_text = " ".join(f"{b:02X}" for b in frame)

                    if args.raw:
                        print(f"RAW {raw_text}")

                    if temperature is None:
                        print(f"{label}: {values[0]:.0f}, {values[1]:.0f}, {values[2]:.0f}")
                    else:
                        print(
                            f"{label}: {values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f} | "
                            f"T={temperature:.2f} C"
                        )
                else:
                    checksum_errors += 1

                frame.clear()

            now = time.time()
            if now - last_status >= args.status_interval:
                print(
                    f"Status: bytes={total_bytes}, valid_frames={valid_frames}, "
                    f"checksum_errors={checksum_errors}"
                )
                last_status = now


if __name__ == "__main__":
    main()
