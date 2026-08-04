import argparse
import socket
import sys
import time

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


def command_sequence(rate_hz):
    rate_code = RATE_CODES.get(rate_hz)
    if rate_code is None:
        supported = ", ".join(str(rate) for rate in RATE_CODES)
        raise ValueError(f"Unsupported rate {rate_hz}. Supported: {supported}")

    return [
        ("unlock", bytes.fromhex("FF AA 69 88 B5")),
        ("horizontal", bytes.fromhex("FF AA 23 00 00")),
        ("axis9", bytes.fromhex("FF AA 24 00 00")),
        ("bandwidth256", bytes.fromhex("FF AA 1F 00 00")),
        ("rate", bytes([0xFF, 0xAA, 0x03, rate_code, 0x00])),
        ("save", bytes.fromhex("FF AA 00 00 00")),
    ]


def measure_udp(bind_host, port, seconds, configure_rate):
    parser = BwtLineParser()
    remote = None
    frame_count = 0
    first_time = None
    last_time = None
    command_sent = False

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((bind_host, port))
        sock.settimeout(0.2)
        print(f"UDP 監聽中：{bind_host}:{port}")
        print("等待感測器 Wi-Fi 封包...")

        start = time.time()
        while time.time() - start < seconds:
            try:
                payload, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue

            remote = addr
            if configure_rate is not None and not command_sent:
                print(f"收到感測器封包：{remote[0]}:{remote[1]}")
                print("開始送出 Wi-Fi 設定指令...")
                for name, command in command_sequence(configure_rate):
                    sock.sendto(command, remote)
                    print(f"已送出 {name}: {command.hex(' ').upper()}")
                    time.sleep(0.15)
                command_sent = True
                parser = BwtLineParser()
                frame_count = 0
                first_time = None
                last_time = None
                start = time.time()
                continue

            for frame in parser.feed(payload):
                frame_count += 1
                if first_time is None:
                    first_time = frame["time"]
                last_time = frame["time"]

    elapsed = time.time() - start
    return remote, frame_count, elapsed, first_time, last_time, command_sent


def measure_tcp(bind_host, port, seconds, configure_rate):
    parser = BwtLineParser()
    frame_count = 0
    first_time = None
    last_time = None
    command_sent = False
    remote = None

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((bind_host, port))
        server.listen(1)
        server.settimeout(seconds)
        print(f"TCP Server 監聽中：{bind_host}:{port}")
        print("等待感測器連線...")

        client, remote = server.accept()
        with client:
            print(f"感測器已連線：{remote[0]}:{remote[1]}")
            client.settimeout(0.2)

            if configure_rate is not None:
                for name, command in command_sequence(configure_rate):
                    client.sendall(command)
                    print(f"已送出 {name}: {command.hex(' ').upper()}")
                    time.sleep(0.15)
                command_sent = True

            start = time.time()
            while time.time() - start < seconds:
                try:
                    payload = client.recv(4096)
                except socket.timeout:
                    continue
                if not payload:
                    break

                for frame in parser.feed(payload):
                    frame_count += 1
                    if first_time is None:
                        first_time = frame["time"]
                    last_time = frame["time"]

    elapsed = time.time() - start
    return remote, frame_count, elapsed, first_time, last_time, command_sent


def main():
    arg_parser = argparse.ArgumentParser(
        description="用 Wi-Fi 設定 BWT901CL/WT901 輸出頻率，並量測實際收到的資料 Hz。"
    )
    arg_parser.add_argument("--mode", choices=["udp", "tcp"], default="udp")
    arg_parser.add_argument("--bind-host", default="0.0.0.0")
    arg_parser.add_argument("--port", type=int, default=1399)
    arg_parser.add_argument("--rate", type=float, default=100)
    arg_parser.add_argument("--seconds", type=float, default=8)
    arg_parser.add_argument("--measure-only", action="store_true")
    args = arg_parser.parse_args()

    configure_rate = None if args.measure_only else args.rate

    try:
        if args.mode == "udp":
            remote, count, elapsed, first_time, last_time, command_sent = measure_udp(
                args.bind_host, args.port, args.seconds, configure_rate
            )
        else:
            remote, count, elapsed, first_time, last_time, command_sent = measure_tcp(
                args.bind_host, args.port, args.seconds, configure_rate
            )
    except TimeoutError:
        print("等待感測器逾時。請確認電腦已連到感測器 Wi-Fi，且連線模式/Port 正確。")
        sys.exit(3)
    except OSError as exc:
        print(f"網路錯誤：{exc}")
        sys.exit(3)

    actual_hz = count / elapsed if elapsed else 0
    print()
    print(f"感測器位址={remote[0] + ':' + str(remote[1]) if remote else 'None'}")
    print(f"已送出設定指令={command_sent}")
    print(f"要求輸出頻率_Hz={configure_rate}")
    print(f"收到資料筆數={count}")
    print(f"量測秒數={elapsed:.3f}")
    print(f"實際資料頻率_Hz={actual_hz:.2f}")
    print(f"第一筆感測器時間={first_time}")
    print(f"最後一筆感測器時間={last_time}")

    if count == 0:
        print("結果=沒有收到感測器資料")
        sys.exit(3)

    if configure_rate is not None and actual_hz < configure_rate * 0.8:
        print("結果=未達要求頻率")
        sys.exit(2)

    print("結果=OK")


if __name__ == "__main__":
    main()
