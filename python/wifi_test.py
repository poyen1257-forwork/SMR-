"""Send supervised single-motor test commands to the ESP32 over Wi-Fi."""

from __future__ import annotations

import math
import os
import sys
import threading
import time

from faro_yaw_follow import WifiMotorPort, read_motor_position, send_position_step


# Use the IP printed by the ESP32 on COM7; the environment variable can override it.
ESP32_HOST = os.environ.get("SMR_ESP32_HOST", "10.127.162.235")
ESP32_PORT = 5007

# Test limits; check the motor's available travel before moving.
MAX_STEP_DEG = 180.0
MIN_STEP_SPEED_DPS = 10
MAX_STEP_SPEED_DPS = 180
MAX_RUN_SPEED_DPS = 360
KEEPALIVE_PERIOD_S = 2.0

USAGE = (
    "Usage: python python/wifi_test.py [r|p|s]\n"
    "       python python/wifi_test.py n <degrees> <speed_dps>\n"
    "       python python/wifi_test.py m <speed_dps> (runs until Ctrl+C)\n"
    "No arguments: enter commands on one connection; use s to stop, Ctrl+C exits.\n"
    "Examples: n 2 30, n -2 30, m 30, s"
)


def parse_command(args: list[str]) -> tuple[str, float | int | None, float | int | None]:
    if not args:
        raise ValueError(USAGE)
    command = args[0].lower()
    if command in {"r", "p", "s"} and len(args) == 1:
        return command, None, None
    if command == "n" and len(args) == 3:
        angle = float(args[1])
        speed = int(args[2])
        if not math.isfinite(angle) or not 0 < abs(angle) <= MAX_STEP_DEG:
            raise ValueError(f"n angle must be within +/-{MAX_STEP_DEG:g} degrees, excluding zero")
        if not MIN_STEP_SPEED_DPS <= speed <= MAX_STEP_SPEED_DPS:
            raise ValueError(f"n speed must be {MIN_STEP_SPEED_DPS}..{MAX_STEP_SPEED_DPS} dps")
        return command, angle, speed
    if command == "m" and len(args) == 2:
        speed = int(args[1])
        if speed == 0 or abs(speed) > MAX_RUN_SPEED_DPS:
            raise ValueError(f"m speed must be within +/-{MAX_RUN_SPEED_DPS} dps, excluding zero")
        return command, speed, None
    raise ValueError(USAGE)


class SpeedKeepalive:
    def __init__(self, port: WifiMotorPort) -> None:
        self.port = port
        self.error: OSError | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _run(self) -> None:
        while not self._stop.wait(KEEPALIVE_PERIOD_S):
            try:
                self.port.write(b"k\n")
            except OSError as error:
                self.error = error
                return

    def start(self) -> None:
        self.error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None


def wait_for_reply(port: WifiMotorPort, prefix: str, timeout_s: float = 3.0) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        print(f"ESP32> {line}")
        if line.startswith(prefix):
            return line
        if line.startswith(("TX failed:", "TX queue failed", "RX timeout", "Speed command failed",
                            "Stop command was not acknowledged", "CAN is not ready", "BUSY",
                            "MOTOR_STOP_PENDING")):
            raise RuntimeError(line)
    raise TimeoutError(f"No complete {prefix} reply from the motor")


def send_stop(port: WifiMotorPort, timeout_s: float = 6.0) -> None:
    port.write(b"s\n")
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        print(f"ESP32> {line}")
        if line.startswith("RX id=0x141 dlc=8 data=81"):
            return
        if line.startswith(("TX failed:", "TX queue failed", "RX timeout",
                            "Speed command failed", "Stop command was not acknowledged")):
            last_error = line
        if line.startswith(("MOTOR_STOP_PENDING", "CAN is not ready", "BUSY")):
            raise RuntimeError(line)
    detail = f"; last error: {last_error}" if last_error else ""
    raise TimeoutError(f"Motor stop was not confirmed within {timeout_s:g} seconds{detail}")


def run_command(port: WifiMotorPort, command: str,
                first: float | int | None, second: float | int | None) -> None:
    motion_attempted = False
    try:
        if command == "r":
            port.write(b"r\n")
            wait_for_reply(port, "MF5015V2 temp=")
        elif command == "p":
            position = read_motor_position(port)
            if position is None:
                raise RuntimeError("Motor position read failed")
            print(f"Motor angle: {position:+.2f} deg")
        elif command == "s":
            send_stop(port)
        elif command == "n":
            angle, speed = float(first), int(second)
            before = read_motor_position(port)
            if before is None:
                raise RuntimeError("Cannot read position; no motion command sent")
            motion_attempted = True
            if send_position_step(port, angle, speed, 1, 0) is not True:
                raise RuntimeError("CAN acceptance is unconfirmed; do not retry this angle")
            position_tolerance = min(0.5, max(0.005, abs(angle) * 0.1))
            deadline = time.monotonic() + max(1.5, abs(angle) / speed + 3.0)
            after = before
            while time.monotonic() < deadline:
                time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                after = read_motor_position(port)
                if after is None:
                    raise RuntimeError("Motor position feedback failed during the move")
                measured_change = after - before
                if angle * measured_change < -0.2 * abs(angle):
                    raise RuntimeError(
                        f"Motor moved in the wrong direction: {measured_change:+.2f} deg"
                    )
                if abs(measured_change - angle) <= position_tolerance:
                    break
            measured_change = after - before
            print(f"Measured change: {measured_change:+.2f} deg (requested {angle:+.2f} deg)")
            if abs(measured_change - angle) > position_tolerance:
                raise RuntimeError(
                    f"Position target not reached before timeout; measured {measured_change:+.2f} deg"
                )
        else:
            speed = int(first)
            motion_attempted = True
            port.write(f"m {speed}\n".encode("ascii"))
            wait_for_reply(port, "RX id=0x141 dlc=8 data=A2")
            motion_attempted = False
    finally:
        if motion_attempted:
            try:
                send_stop(port)
            except (OSError, ValueError, RuntimeError) as error:
                raise RuntimeError(
                    f"STOP NOT CONFIRMED: {error}; cut motor power if it is still moving."
                ) from error


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if args:
        try:
            one_command = parse_command(args)
        except ValueError as error:
            print(error, file=sys.stderr)
            return 2
    else:
        one_command = None

    port: WifiMotorPort | None = None
    try:
        port = WifiMotorPort(ESP32_HOST, ESP32_PORT)
        keepalive = SpeedKeepalive(port)
        speed_running = False
        print(f"Connected to ESP32 at {ESP32_HOST}:{ESP32_PORT}")
        try:
            if one_command is not None:
                run_command(port, *one_command)
                if one_command[0] == "m":
                    speed_running = True
                    keepalive.start()
                    print("Motor running; press Ctrl+C to stop.")
                    while True:
                        if keepalive.error is not None:
                            raise RuntimeError(f"Wi-Fi keepalive failed: {keepalive.error}")
                        time.sleep(0.1)
                return 0

            print(USAGE)
            while True:
                try:
                    line = input("motor> ")
                except EOFError:
                    break
                if not line.strip():
                    continue
                try:
                    command = parse_command(line.split())
                except ValueError as error:
                    print(error, file=sys.stderr)
                    continue
                if speed_running and command[0] == "n":
                    print("Use s before an angle move.", file=sys.stderr)
                    continue
                keepalive.stop()
                run_command(port, *command)
                if command[0] == "m":
                    speed_running = True
                elif command[0] == "s":
                    speed_running = False
                if speed_running:
                    keepalive.start()
            return 0
        finally:
            keepalive.stop()
            if speed_running:
                try:
                    send_stop(port)
                except (OSError, ValueError, RuntimeError) as error:
                    raise RuntimeError(
                        f"STOP NOT CONFIRMED: {error}; cut motor power if it is still moving."
                    ) from error
    except KeyboardInterrupt:
        print("\nStopped; closing the Wi-Fi connection.")
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Test failed: {error}", file=sys.stderr)
        return 1
    finally:
        if port is not None:
            port.close()


if __name__ == "__main__":
    raise SystemExit(main())
