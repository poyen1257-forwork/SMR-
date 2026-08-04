import argparse
import sys
import time

import serial

from BWT901CL_visualizer import BwtLineParser


RATE_CODES = {
    0.1: 0x00,
    0.2: 0x01,
    0.5: 0x02,
    1: 0x03,
    2: 0x04,
    5: 0x05,
    10: 0x06,
    20: 0x07,
    50: 0x08,
    100: 0x09,
    125: 0x0A,
    200: 0x0B,
}


def send_command(ser, hex_command):
    ser.write(bytes.fromhex(hex_command))
    ser.flush()
    time.sleep(0.2)
    return ser.read(512)


def clean_reply(payload):
    text = payload.decode("ascii", errors="ignore")
    return " ".join(text.replace("\r", " ").replace("\n", " ").split())


def configure_sensor(ser, rate_hz, bandwidth_256=True, algorithm="axis6"):
    rate_code = RATE_CODES.get(rate_hz)
    if rate_code is None:
        supported = ", ".join(str(rate) for rate in RATE_CODES)
        raise ValueError(f"Unsupported rate {rate_hz}. Supported: {supported}")

    replies = []
    replies.append(("unlock", send_command(ser, "FF AA 69 88 B5")))
    replies.append(("horizontal", send_command(ser, "FF AA 23 00 00")))
    if algorithm == "axis6":
        replies.append(("axis6", send_command(ser, "FF AA 24 01 00")))
    elif algorithm == "axis9":
        replies.append(("axis9", send_command(ser, "FF AA 24 00 00")))
    if bandwidth_256:
        replies.append(("bandwidth256", send_command(ser, "FF AA 1F 00 00")))
    replies.append(("rate", send_command(ser, f"FF AA 03 {rate_code:02X} 00")))
    replies.append(("save", send_command(ser, "FF AA 00 00 00")))
    return replies


def measure_rate(ser, seconds):
    parser = BwtLineParser()
    ser.reset_input_buffer()
    start = time.time()
    count = 0
    first_sensor_time = None
    last_sensor_time = None

    while time.time() - start < seconds:
        payload = ser.read(512)
        for frame in parser.feed(payload):
            count += 1
            if first_sensor_time is None:
                first_sensor_time = frame["time"]
            last_sensor_time = frame["time"]

    elapsed = time.time() - start
    return count, elapsed, first_sensor_time, last_sensor_time


def main():
    arg_parser = argparse.ArgumentParser(description="Set and measure BWT901CL COM data rate.")
    arg_parser.add_argument("--port", default="COM4")
    arg_parser.add_argument("--baud", type=int, default=9600)
    arg_parser.add_argument("--rate", type=float, default=100)
    arg_parser.add_argument("--algorithm", choices=["axis6", "axis9", "none"], default="axis6")
    arg_parser.add_argument("--seconds", type=float, default=5)
    args = arg_parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        ser.reset_input_buffer()
        replies = configure_sensor(ser, args.rate, algorithm=args.algorithm)
        for name, reply in replies:
            print(f"{name}: {clean_reply(reply)}")

        time.sleep(1)
        count, elapsed, first_sensor_time, last_sensor_time = measure_rate(ser, args.seconds)

    actual_hz = count / elapsed if elapsed else 0
    print(f"requested_rate_hz={args.rate}")
    print(f"frames={count}")
    print(f"elapsed_s={elapsed:.3f}")
    print(f"actual_hz={actual_hz:.2f}")
    print(f"first_sensor_time={first_sensor_time}")
    print(f"last_sensor_time={last_sensor_time}")

    if actual_hz < args.rate * 0.8:
        print("result=FAILED_TO_REACH_REQUESTED_RATE")
        sys.exit(2)

    print("result=OK")


if __name__ == "__main__":
    main()
