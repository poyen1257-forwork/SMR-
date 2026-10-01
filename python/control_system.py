"""Operate the remote MTi reader and local yaw-to-motor bridge together."""

from __future__ import annotations

import argparse
import base64
import getpass
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import paramiko
except ImportError:
    print("paramiko is required: python -m pip install paramiko")
    raise SystemExit(1)


class YawMotorSystem:
    def __init__(self, args: argparse.Namespace, password: str) -> None:
        self.args = args
        self.password = password
        self.bridge: subprocess.Popen[str] | None = None
        self.bridge_log: object | None = None
        self.bridge_log_thread: threading.Thread | None = None

    def remote_powershell(self, script: str) -> str:
        encoded = base64.b64encode(script.encode("utf-16le")).decode("ascii")
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                self.args.remote_host,
                username=self.args.remote_user,
                password=self.password,
                timeout=10,
            )
            _, stdout, stderr = client.exec_command(
                f"powershell -NoProfile -EncodedCommand {encoded}", timeout=20
            )
            output = stdout.read().decode("utf-8", errors="replace")
            error = stderr.read().decode("utf-8", errors="replace")
            if error.strip() and not error.lstrip().startswith("#< CLIXML"):
                print("Remote PowerShell returned an error. Check the remote reader log.")
            return output
        finally:
            client.close()

    def stop_remote_reader(self) -> None:
        output = self.remote_powershell(
            """
$readers = Get-CimInstance Win32_Process | Where-Object {
    $_.CommandLine -like '*mti630r_reader.py*'
}
$readers | ForEach-Object {
    Stop-Process -Id $_.ProcessId -ErrorAction SilentlyContinue
}
Write-Output ('STOPPED_READERS=' + @($readers).Count)
"""
        )
        for line in output.splitlines():
            if line.startswith("STOPPED_READERS="):
                print(f"Remote IMU reader stopped: {line.split('=', 1)[1]} process(es).")

    def start_remote_reader(self) -> bool:
        self.stop_remote_reader()
        root = self.args.remote_root.replace("'", "''")
        remote_python = self.args.remote_python.replace("'", "''")
        output = self.remote_powershell(
            f"""
$root = '{root}'
$python = '{remote_python}'
$reader = Join-Path $root 'mti630r_reader.py'
$stdout = Join-Path $root 'mti_stream_system.log'
$stderr = Join-Path $root 'mti_stream_system_error.log'
if (-not (Test-Path -LiteralPath $reader)) {{
    Write-Output 'READER_FILE=NOT_FOUND'
    exit
}}
$commandLine = 'cmd.exe /c ""' + $python + '" "' + $reader + '" --rate 100 --stream-rate 100 --tcp-port {self.args.sensor_port} --print-rate 1 > "' + $stdout + '" 2> "' + $stderr + '""'
$result = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{{CommandLine=$commandLine; CurrentDirectory=$root}}
Write-Output ('CREATE_RETURN=' + $result.ReturnValue)
"""
        )
        if "CREATE_RETURN=0" not in output:
            print("Could not start remote IMU reader.")
            return False

        for _ in range(20):
            time.sleep(0.25)
            status = self.remote_powershell(
                f"""
$listener = Get-NetTCPConnection -LocalPort {self.args.sensor_port} -State Listen -ErrorAction SilentlyContinue
if ($listener) {{ Write-Output 'TCP=LISTENING' }}
"""
            )
            if "TCP=LISTENING" in status:
                print(f"Remote IMU reader is listening on TCP {self.args.sensor_port}.")
                return True

        print("Remote reader did not open its TCP port.")
        self.stop_remote_reader()
        return False

    def forward_bridge_output(self) -> None:
        bridge = self.bridge
        if bridge is None or bridge.stdout is None:
            return
        for line in bridge.stdout:
            if self.bridge_log is not None:
                self.bridge_log.write(line)
            print(f"BRIDGE> {line.rstrip()}")

    def close_bridge_log(self) -> None:
        if self.bridge_log_thread is not None:
            self.bridge_log_thread.join(timeout=1)
            self.bridge_log_thread = None
        if self.bridge_log is not None:
            self.bridge_log.close()
            self.bridge_log = None

    def start_bridge(self) -> bool:
        if self.bridge is not None and self.bridge.poll() is None:
            print("Local yaw bridge is already running.")
            return True

        Path("logs").mkdir(exist_ok=True)
        self.bridge_log = Path("logs/xsens2esp32.log").open(
            "w", encoding="utf-8", buffering=1
        )
        command = [
            sys.executable,
            "-u",
            "python/xsens2esp32.py",
            "--sensor-ip",
            self.args.remote_host,
            "--sensor-port",
            str(self.args.sensor_port),
            "--esp-port",
            self.args.esp_port,
            "--deadband",
            str(self.args.deadband),
            "--max-speed",
            str(self.args.max_speed),
            "--latency-sample-every",
            str(self.args.latency_sample_every),
        ]
        if self.args.arm:
            command.append("--arm")

        flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        self.bridge = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            creationflags=flags,
        )
        self.bridge_log_thread = threading.Thread(
            target=self.forward_bridge_output,
            name="yaw-motor-bridge-log",
            daemon=True,
        )
        self.bridge_log_thread.start()
        time.sleep(1)
        if self.bridge.poll() is not None:
            print("Local yaw bridge exited. Read logs/xsens2esp32.log")
            self.close_bridge_log()
            self.bridge = None
            return False

        mode = "ARMED" if self.args.arm else "DRY RUN"
        print(f"Local yaw bridge started ({mode}).")
        print("Bridge log: logs/xsens2esp32.log")
        return True

    def send_stop_to_esp(self) -> None:
        try:
            import serial

            port = serial.Serial()
            port.port = self.args.esp_port
            port.baudrate = 115200
            port.timeout = 0
            port.write_timeout = 1.0
            port.dtr = False
            port.rts = False
            port.open()
            port.write(b"s\n")
            port.flush()
            port.close()
        except Exception as error:
            print(f"Could not send final ESP32 stop command: {error}")

    def stop_bridge(self) -> None:
        if self.bridge is None:
            return
        if self.bridge.poll() is None:
            print("Stopping local yaw bridge and requesting motor stop...")
            try:
                self.bridge.send_signal(signal.CTRL_BREAK_EVENT)
                self.bridge.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.bridge.terminate()
                try:
                    self.bridge.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.bridge.kill()
                    self.bridge.wait(timeout=2)
        self.bridge = None
        self.send_stop_to_esp()
        self.close_bridge_log()

    def start(self) -> None:
        try:
            if self.start_remote_reader():
                self.start_bridge()
        except (OSError, paramiko.SSHException) as error:
            print(
                f"Cannot reach remote IMU computer {self.args.remote_host}: {error}. "
                "Check its power, network, IP address, and SSH service."
            )

    def end(self) -> None:
        self.stop_bridge()
        try:
            self.stop_remote_reader()
        except (OSError, paramiko.SSHException) as error:
            print(f"Could not stop remote IMU reader: {error}")
        print("System ended.")

    def status(self) -> None:
        local = "running" if self.bridge and self.bridge.poll() is None else "stopped"
        print(f"Local yaw bridge: {local}")
        try:
            output = self.remote_powershell(
                f"""
$listener = Get-NetTCPConnection -LocalPort {self.args.sensor_port} -State Listen -ErrorAction SilentlyContinue
if ($listener) {{ Write-Output 'REMOTE_READER=RUNNING' }} else {{ Write-Output 'REMOTE_READER=STOPPED' }}
"""
            )
            print(output.strip())
        except (OSError, paramiko.SSHException) as error:
            print(f"Remote IMU reader: unreachable ({error})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive yaw motor system controller.")
    parser.add_argument("--remote-host", default="10.196.173.57")
    parser.add_argument("--remote-user", default="pmc64")
    parser.add_argument("--remote-root", default=r"C:\Users\PMC64\Desktop\xsens_mti630r")
    parser.add_argument(
        "--remote-python",
        default=r"C:\Users\PMC64\AppData\Local\Programs\Python\Python39\python.exe",
    )
    parser.add_argument("--sensor-port", type=int, default=5006)
    parser.add_argument("--esp-port", default="COM7")
    parser.add_argument("--deadband", type=float, default=0.05)
    parser.add_argument("--max-speed", type=int, default=720)
    parser.add_argument("--latency-sample-every", type=int, default=10)
    parser.add_argument("--arm", action="store_true", help="allow the bridge to move the motor")
    args = parser.parse_args()
    if not 0.01 <= args.deadband <= 10:
        parser.error("--deadband must be from 0.01 to 10 degrees")
    if not 1 <= args.max_speed <= 720:
        parser.error("--max-speed must be from 1 to 720 dps")
    if not 1 <= args.sensor_port <= 65535 or args.latency_sample_every < 1:
        parser.error("invalid port or latency sample interval")
    return args


def main() -> int:
    args = parse_args()
    password = os.environ.get("IMU_SSH_PASSWORD") or getpass.getpass(
        f"SSH password for {args.remote_user}@{args.remote_host}: "
    )
    system = YawMotorSystem(args, password)
    print("Commands: start, status, end, quit")
    try:
        while True:
            command = input("system> ").strip().lower()
            if command == "start":
                system.start()
            elif command == "status":
                system.status()
            elif command == "end":
                system.end()
            elif command in {"quit", "exit"}:
                system.end()
                return 0
            elif command:
                print("Commands: start, status, end, quit")
    except KeyboardInterrupt:
        print()
        system.end()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
