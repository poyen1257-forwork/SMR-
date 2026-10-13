"""結合 Tracker 位置與 Xsens 姿態，控制單顆馬達持續朝向固定原點。

資料流程：遠端 Xsens reader -> SSH/TCP -> 本程式 <- TCP 5010 <- Tracker 模擬器。
本程式再透過 USB Serial 傳送 p / n / s 指令給 mf5015v2_sample 韌體，
由 ESP32 使用 CAN 控制 MF5015V2；連線位置及運動參數集中在下方常數。

Xsens 固定在馬達底座。啟動時假設馬達已朝向 Tracker，讀取編碼器建立基準；
之後累積「Tracker 方位角變化 - 底座 yaw 變化」作為馬達追蹤目標。
每 0.2 秒回讀位置並計算剩餘角度，以 n 傳送有速度限制的相對角度增量；
CAN 傳送確認不代表轉動已完成，位置修正以實際編碼器回讀為準。
啟動讀取收到完整失敗回覆時會重試；交易未結束而逾時仍會退出。
模擬器斷線時保留程式等待重新連線。
TCP 5011 回傳 IMU yaw 與馬達世界方向，供模擬器繪圖；--arm 使用編碼器，
未加 --arm 的 dry-run 使用估算位置，並以 estimate 標記，不能視為實測。

Tracker 輸入是本專案定義的逐行 JSON（NDJSON），不是 FARO SDK 原始封包；
欄位為 distance_mm、azimuth_deg、zenith_deg、timestamp_ns、valid。
實際 Tracker 需由轉接程式提供相同欄位，並確認座標軸、方向與單位一致。

從專案根目錄啟動：python python/faro_yaw_follow.py --arm
SSH 密碼由互動輸入或 IMU_SSH_PASSWORD 環境變數提供，不寫入程式碼。
"""

from __future__ import annotations

import argparse
import base64
import getpass
import json
import math
import os
import queue
import selectors
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial")
    raise SystemExit(1)

try:
    import paramiko
except ImportError:
    print("paramiko is required: python -m pip install paramiko")
    raise SystemExit(1)


# Remote Xsens reader and local FARO adapter settings.
FARO_MODE = "azimuth"
SENSOR_IP = "10.127.162.57"
SENSOR_PORT = 5008
EXISTING_READER_PORT = 5006
REMOTE_USER = "pmc64"
REMOTE_ROOT = r"C:\Users\PMC64\Desktop\xsens_mti630r"
REMOTE_PYTHON = r"C:\Users\PMC64\AppData\Local\Programs\Python\Python39\python.exe"
ESP32_PORT = "COM7"
SERIAL_BAUD = 115200
FARO_BIND_HOST = "127.0.0.1"
FARO_BIND_PORT = 5010
YAW_TELEMETRY_HOST = "127.0.0.1"
YAW_TELEMETRY_PORT = 5011

# Motor command settings; keep runtime-specific values here, not in CLI args.
SPEED_DPS = 180
COMMAND_PERIOD_S = 0.2
MAX_MISSES = 5
MAX_STEP_DEG = 30.0
ANGLE_DEADBAND_DEG = 0.5
POSITION_CORRECTION_PERIOD_S = 0.2
STARTUP_POSITION_RETRY_S = 1.0
MOTOR_STALL_TIMEOUT_S = 1.0
ENCODER_PROGRESS_THRESHOLD_DEG = 0.1
MAX_POSITION_TRACKING_ERROR_DEG = 45.0
MAX_WRONG_WAY_SAMPLES = 3

# Sample timing limits; armed operation cannot exceed these bounds.
MAX_SAMPLE_AGE_MS = 250.0
MAX_ARRIVAL_SKEW_MS = 20.0
ARMED_MAX_SAMPLE_AGE_MS = 250.0
ARMED_MAX_ARRIVAL_SKEW_MS = 20.0


class YawTelemetryPublisher:
    """Send the newest paired Xsens yaw and motor heading to the simulator."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self._latest: queue.Queue[dict[str, float | int | str]] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="yaw-ui-telemetry", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def update(
        self,
        yaw_deg: float,
        timestamp_ns: int,
        motor_heading_deg: float | None = None,
        motor_heading_source: str = "encoder",
    ) -> None:
        sample: dict[str, float | int | str] = {
            "yaw_deg": yaw_deg,
            "timestamp_ns": timestamp_ns,
            "local_monotonic_s": time.monotonic(),
        }
        if motor_heading_deg is not None:
            sample["motor_heading_deg"] = motor_heading_deg
            sample["motor_heading_source"] = motor_heading_source
        try:
            self._latest.put_nowait(sample)
        except queue.Full:
            try:
                self._latest.get_nowait()
            except queue.Empty:
                pass
            try:
                self._latest.put_nowait(sample)
            except queue.Full:
                pass

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.5)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=0.5) as channel:
                    channel.settimeout(0.5)
                    while not self._stop.is_set():
                        try:
                            sample = self._latest.get(timeout=0.25)
                        except queue.Empty:
                            continue
                        payload = json.dumps(
                            {"type": "imu_yaw", **sample}, separators=(",", ":")
                        ).encode("utf-8") + b"\n"
                        channel.sendall(payload)
            except OSError:
                self._stop.wait(0.25)


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def motor_alignment_delta(target_deg: float, current_deg: float) -> float:
    return wrap_degrees(target_deg - current_deg)


def accumulate_heading_target(
    target_position_deg: float,
    previous_heading_deg: float,
    current_heading_deg: float,
) -> tuple[float, float]:
    """Unwrap heading changes into a continuous multi-turn motor target."""
    delta = wrap_degrees(current_heading_deg - previous_heading_deg)
    return target_position_deg + delta, current_heading_deg


def bounded_position_correction(
    target_position_deg: float, actual_position_deg: float, max_step_deg: float
) -> float:
    error = target_position_deg - actual_position_deg
    # Match the firmware's signed 0.01-degree increment, including rounding.
    limit_cdeg = math.floor(max_step_deg * 100.0)
    magnitude_cdeg = min(limit_cdeg, math.floor(abs(error) * 100.0 + 0.5))
    return math.copysign(magnitude_cdeg / 100.0, error)


def encoder_moved_opposite_command(
    previous_position_deg: float,
    actual_position_deg: float,
    command_target_deg: float,
    tolerance_deg: float,
) -> bool:
    # Use the error BEFORE the movement: crossing the target is not reversal.
    expected_delta = command_target_deg - previous_position_deg
    actual_delta = actual_position_deg - previous_position_deg
    return (
        abs(expected_delta) > tolerance_deg
        and abs(actual_delta) >= ENCODER_PROGRESS_THRESHOLD_DEG
        and expected_delta * actual_delta < 0.0
    )


def motor_world_heading_from_position(
    startup_azimuth_deg: float,
    startup_yaw_deg: float,
    current_yaw_deg: float,
    startup_motor_position_deg: float,
    actual_motor_position_deg: float,
) -> float:
    """Estimate the motor's world heading from startup alignment and encoder feedback."""
    startup_tracker_heading = wrap_degrees(startup_azimuth_deg + 180.0)
    base_yaw_change = wrap_degrees(current_yaw_deg - startup_yaw_deg)
    motor_joint_change = actual_motor_position_deg - startup_motor_position_deg
    return wrap_degrees(startup_tracker_heading + base_yaw_change + motor_joint_change)


def received_samples_are_pairable(
    now: float,
    imu_received_at: float,
    faro_received_at: float,
    max_age_ms: float,
    max_arrival_skew_ms: float,
) -> bool:
    """Check local receipt freshness; cross-host source-clock offset is not a gate."""
    max_age_s = max_age_ms / 1000.0
    max_arrival_skew_s = max_arrival_skew_ms / 1000.0
    return (
        now - imu_received_at <= max_age_s
        and now - faro_received_at <= max_age_s
        and abs(imu_received_at - faro_received_at) <= max_arrival_skew_s
    )


def desired_motor_angle_from_azimuth(
    faro_azimuth_deg: float,
    xsens_yaw_deg: float,
) -> float:
    """計算朝向原點的關節角度：Tracker 方位角 + 180 度 - 底座 yaw。

    輸入與回傳單位都是度；假設兩者座標軸一致、正角為從 +X 逆時針，
    且 Xsens 固定在馬達底座。實際控制使用此角度相對啟動基準的變化量。
    """
    bearing_to_origin = wrap_degrees(faro_azimuth_deg + 180.0)
    yaw_in_faro_frame = wrap_degrees(xsens_yaw_deg)
    return wrap_degrees(bearing_to_origin - yaw_in_faro_frame)


def parse_imu_line(line: bytes) -> dict[str, float] | None:
    try:
        packet = json.loads(line.decode("utf-8"))
        if packet.get("stream_version") != 1:
            return None
        yaw = float(packet["yaw"])
        source_time_ns = int(packet["host_read_ns"])
        quaternion = tuple(float(packet[key]) for key in ("q_w", "q_x", "q_y", "q_z"))
        norm = math.sqrt(sum(value * value for value in quaternion))
        if source_time_ns <= 0 or not math.isfinite(yaw) or not 0.9 <= norm <= 1.1:
            return None
        return {"yaw_deg": yaw, "source_time_ns": source_time_ns}
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError, OverflowError):
        return None


def parse_faro_line(line: bytes, mode: str = FARO_MODE) -> dict[str, float | int | None] | None:
    """解析距離／方位角／天頂角 JSON；無效資料回傳 None。距離單位為毫米。"""
    try:
        if mode != "azimuth":
            return None
        packet = json.loads(line.decode("utf-8"))
        if packet.get("valid", True) is not True:
            return None
        source_time_ns = int(packet["timestamp_ns"])
        if source_time_ns <= 0:
            return None
        azimuth_deg = float(packet["azimuth_deg"])
        distance_mm = float(packet["distance_mm"])
        zenith_deg = float(packet["zenith_deg"])
        values = (azimuth_deg, distance_mm, zenith_deg)
        if (not all(math.isfinite(value) for value in values)
                or distance_mm <= 0
                or not 0.0 <= zenith_deg <= 180.0):
            return None
        return {
            "azimuth_deg": azimuth_deg,
            "distance_mm": distance_mm,
            "zenith_deg": zenith_deg,
            "source_time_ns": source_time_ns,
        }
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError, OverflowError):
        return None


def drain_latest_sample(
    channel: socket.socket,
    buffer: bytearray,
    stream: str,
    faro_mode: str,
) -> tuple[dict[str, float | int | None] | None, float | None]:
    """Drain queued TCP data and retain only the newest valid complete sample."""
    latest = None
    received_at = None
    while True:
        try:
            data = channel.recv(65536)
        except BlockingIOError:
            break
        if not data:
            raise ConnectionError(f"{stream} stream disconnected.")
        chunk_received_at = time.monotonic()
        buffer.extend(data)
        if len(buffer) > 1_000_000:
            raise ValueError(f"{stream} line buffer exceeded 1 MB.")
        while b"\n" in buffer:
            raw, _, remainder = buffer.partition(b"\n")
            buffer[:] = remainder
            if stream == "imu":
                sample = parse_imu_line(raw)
            else:
                sample = parse_faro_line(raw, faro_mode)
            if sample is not None:
                latest = sample
                received_at = chunk_received_at
    return latest, received_at


def remote_powershell(client: paramiko.SSHClient, script: str) -> tuple[str, str, int]:
    encoded = base64.b64encode(script.encode("utf-16le")).decode("ascii")
    _, stdout, stderr = client.exec_command(
        f"powershell -NoProfile -EncodedCommand {encoded}", timeout=20
    )
    output = stdout.read().decode("utf-8", errors="replace")
    error = stderr.read().decode("utf-8", errors="replace")
    status = stdout.channel.recv_exit_status()
    return output, error, status


def start_remote_reader(args: argparse.Namespace) -> tuple[paramiko.SSHClient, int, str]:
    """透過 SSH 部署並啟動 Xsens reader，回傳 SSH 連線、程序 PID 及檔名。"""
    password = os.environ.get("IMU_SSH_PASSWORD") or getpass.getpass(
        f"SSH password for {args.remote_user}@{args.sensor_ip}: "
    )
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.sensor_ip, username=args.remote_user, password=password, timeout=10)
    except BaseException:
        client.close()
        raise

    remote_root = args.remote_root.replace("'", "''")
    remote_python = args.remote_python.replace("'", "''")
    reader_name = f"mti630r_faro_reader_{os.getpid()}_{time.time_ns()}.py"
    reader_source = Path(__file__).resolve().parent / "xsens" / "mti630r_reader.py"
    reader_path = f"{args.remote_root}\\{reader_name}"
    reader_pid: int | None = None

    try:
        if not reader_source.is_file():
            raise RuntimeError(f"Xsens reader source not found: {reader_source}")

        preflight = f"""
$root = '{remote_root}'
$python = '{remote_python}'
if (-not (Test-Path -LiteralPath $root)) {{ Write-Output 'ROOT=NOT_FOUND'; exit 2 }}
if (-not (Test-Path -LiteralPath $python)) {{ Write-Output 'PYTHON=NOT_FOUND'; exit 3 }}
$allProcesses = @(Get-CimInstance Win32_Process)
$readers = @($allProcesses | Where-Object {{
    $_.Name -eq 'python.exe' -and $_.CommandLine -match 'mti630r_.*reader\\.py'
}})
$targetListeners = @(Get-NetTCPConnection -LocalPort {args.sensor_port} -State Listen -ErrorAction SilentlyContinue)
$readerIds = @($readers | ForEach-Object {{ [int]$_.ProcessId }})
$foreignListeners = @($targetListeners | Where-Object {{ $readerIds -notcontains [int]$_.OwningProcess }})
if ($foreignListeners.Count -gt 0) {{
    $ownerIds = ($foreignListeners | ForEach-Object {{ $_.OwningProcess }} | Sort-Object -Unique) -join ','
    Write-Output ('TARGET_PORT=IN_USE_BY_OTHER_PROCESS; PID=' + $ownerIds)
    exit 6
}}
foreach ($reader in $readers) {{
    Stop-Process -Id $reader.ProcessId -ErrorAction SilentlyContinue
}}
if ($readers.Count -gt 0) {{ Start-Sleep -Milliseconds 500 }}
$remainingReaders = @(Get-CimInstance Win32_Process | Where-Object {{
    $_.Name -eq 'python.exe' -and $_.CommandLine -match 'mti630r_.*reader\\.py'
}})
$remainingListeners = @(Get-NetTCPConnection -LocalPort {args.sensor_port} -State Listen -ErrorAction SilentlyContinue)
if ($remainingReaders.Count -gt 0) {{ Write-Output 'READER_CLEANUP_FAILED=1'; exit 7 }}
if ($remainingListeners.Count -gt 0) {{
    $ownerIds = ($remainingListeners | ForEach-Object {{ $_.OwningProcess }} | Sort-Object -Unique) -join ','
    Write-Output ('TARGET_PORT=STILL_IN_USE; PID=' + $ownerIds)
    exit 8
}}
if ($readers.Count -gt 0) {{ Write-Output ('CLEANED_OLD_READERS=' + $readers.Count) }}
Write-Output 'READY=1'
"""
        output, error, status = remote_powershell(client, preflight)
        if status != 0 or "READY=1" not in output:
            if "TARGET_PORT=IN_USE_BY_OTHER_PROCESS" in output:
                raise RuntimeError(
                    f"Remote TCP {args.sensor_port} is owned by a non-Xsens process "
                    f"(PID shown below); it was not stopped. {output.strip()}"
                )
            if "TARGET_PORT=STILL_IN_USE" in output:
                raise RuntimeError(f"Remote TCP {args.sensor_port} remains occupied after reader cleanup: {output.strip()}")
            if "READER_CLEANUP_FAILED=1" in output:
                raise RuntimeError("A previous Xsens reader could not be stopped; check its remote process permissions.")
            raise RuntimeError(f"Remote reader preflight failed: {output.strip()} {error.strip()}")
        cleaned_line = next((line for line in output.splitlines() if line.startswith("CLEANED_OLD_READERS=")), None)
        if cleaned_line:
            print(f"Cleaned up {cleaned_line.split('=', 1)[1]} stale remote Xsens reader process(es).")

        with client.open_sftp() as sftp:
            sftp.put(str(reader_source), reader_path.replace("\\", "/"))

        log_prefix = reader_name.removesuffix(".py")
        launch = f"""
$root = '{remote_root}'
$python = '{remote_python}'
$reader = Join-Path $root '{reader_name}'
$stdout = Join-Path $root '{log_prefix}.log'
$stderr = Join-Path $root '{log_prefix}_error.log'
$commandLine = 'cmd.exe /c ""' + $python + '" -u "' + $reader + '" --rate 400 --tcp-host 127.0.0.1 --tcp-port {args.sensor_port} --tcp-client-timeout 30 --print-rate 1 --stream-rate 400 --exit-on-disconnect > "' + $stdout + '" 2> "' + $stderr + '""'
$result = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{{CommandLine=$commandLine; CurrentDirectory=$root}}
Write-Output ('CREATE_RETURN=' + $result.ReturnValue)
Write-Output ('LAUNCH_PID=' + $result.ProcessId)
"""
        output, error, status = remote_powershell(client, launch)
        if status != 0 or "CREATE_RETURN=0" not in output:
            raise RuntimeError(f"Could not start remote Xsens reader: {output.strip()} {error.strip()}")

        for _ in range(60):
            time.sleep(0.5)
            check, _, _ = remote_powershell(
                client,
                f"""
$listener = Get-NetTCPConnection -LocalPort {args.sensor_port} -State Listen -ErrorAction SilentlyContinue | Select-Object -First 1
$reader = Get-CimInstance Win32_Process | Where-Object {{
    $_.Name -eq 'python.exe' -and $_.CommandLine -like '*{reader_name}*'
}} | Select-Object -First 1
if ($reader) {{ Write-Output ('READER_PID=' + $reader.ProcessId) }}
if ($listener) {{ Write-Output ('LISTENER_PID=' + $listener.OwningProcess) }}
""",
            )
            pid_line = next((line for line in check.splitlines() if line.startswith("READER_PID=")), None)
            if pid_line is not None:
                reader_pid = int(pid_line.split("=", 1)[1])
            listener_line = next((line for line in check.splitlines() if line.startswith("LISTENER_PID=")), None)
            if listener_line is not None:
                listener_pid = int(listener_line.split("=", 1)[1])
                if reader_pid is not None and listener_pid != reader_pid:
                    raise RuntimeError(
                        f"Remote TCP {args.sensor_port} listener belongs to PID {listener_pid}, "
                        f"not this Xsens reader (PID {reader_pid}); refusing the stream."
                    )
            if listener_line is not None and reader_pid is not None:
                print(f"Started remote Xsens reader on {args.sensor_ip}:{args.sensor_port} (PID {reader_pid}).")
                print("SSH TCP forwarding is active; source clocks need not match. Local sample freshness and arrival skew are still checked.")
                return client, reader_pid, reader_name

        logs, _, _ = remote_powershell(
            client,
            f"""
$root = '{remote_root}'
$out = Join-Path $root '{log_prefix}.log'
$err = Join-Path $root '{log_prefix}_error.log'
if (Test-Path -LiteralPath $out) {{ Get-Content -LiteralPath $out -Tail 20 }}
if (Test-Path -LiteralPath $err) {{ Get-Content -LiteralPath $err -Tail 20 }}
""",
        )
        raise RuntimeError(f"Remote reader did not listen on TCP {args.sensor_port}. Startup log:\n{logs.strip()}")
    except BaseException:
        try:
            stop_remote_reader(client, reader_pid, reader_name, args.remote_root)
        finally:
            client.close()
        raise


def stop_remote_reader(
    client: paramiko.SSHClient, reader_pid: int | None, reader_name: str, remote_root: str
) -> None:
    pid_filter = f"$_.ProcessId -eq {reader_pid} -and " if reader_pid is not None else ""
    remote_root = remote_root.replace("'", "''")
    script = f"""
$readers = @(Get-CimInstance Win32_Process | Where-Object {{
    $_.Name -eq 'python.exe' -and {pid_filter}$_.CommandLine -like '*{reader_name}*'
}})
if ($readers.Count -gt 0) {{
    $targetIds = @($readers | ForEach-Object {{ $_.ProcessId }})
    foreach ($reader in $readers) {{
        Stop-Process -Id $reader.ProcessId -ErrorAction SilentlyContinue
    }}
    Start-Sleep -Milliseconds 300
    $remaining = @(Get-CimInstance Win32_Process | Where-Object {{
        $_.Name -eq 'python.exe' -and $targetIds -contains $_.ProcessId -and $_.CommandLine -like '*{reader_name}*'
    }})
}} else {{ $remaining = @() }}
$readerPath = Join-Path '{remote_root}' '{reader_name}'
if (Test-Path -LiteralPath $readerPath) {{
    Remove-Item -LiteralPath $readerPath -ErrorAction SilentlyContinue
}}
if ($readers.Count -gt 0 -and $remaining.Count -eq 0) {{
    Write-Output 'STOPPED=1'
}} elseif ($remaining.Count -gt 0) {{
    Write-Output ('STOP_FAILED=1; REMAINING_PID=' + $remaining[0].ProcessId)
}} else {{ Write-Output 'STOPPED=0' }}
"""
    try:
        output, error, status = remote_powershell(client, script)
        if status == 0 and "STOPPED=1" in output:
            print(f"Stopped only this session's remote reader (PID {reader_pid}).")
        elif "STOPPED=0" in output:
            print("Remote reader process already exited; no other process was stopped.")
        elif "STOP_FAILED=1" in output:
            print(f"Remote reader cleanup failed; process is still running: {output.strip()}")
        elif error.strip():
            print(f"Could not stop this session's remote reader: {error.strip()}")
        elif status != 0:
            print(f"Remote reader cleanup returned exit status {status}: {output.strip()}")
    except (OSError, paramiko.SSHException) as error:
        print(f"Could not stop this session's remote reader (PID {reader_pid}): {error}")


def _forward_reader_stream(
    channel: paramiko.Channel,
    forward_socket: socket.socket,
    stop_event: threading.Event,
) -> None:
    try:
        channel.settimeout(0.5)
        forward_socket.settimeout(2.0)
        while not stop_event.is_set():
            try:
                data = channel.recv(65536)
            except socket.timeout:
                continue
            if not data:
                break
            forward_socket.sendall(data)
    except (OSError, paramiko.SSHException):
        pass
    finally:
        try:
            forward_socket.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        forward_socket.close()
        channel.close()


@dataclass
class RemoteSensorSession:
    sensor_socket: socket.socket
    ssh_client: paramiko.SSHClient
    reader_pid: int
    reader_name: str
    remote_root: str
    channel: paramiko.Channel
    forward_socket: socket.socket
    stop_event: threading.Event
    forward_thread: threading.Thread

    def close(self) -> None:
        self.stop_event.set()
        self.sensor_socket.close()
        self.channel.close()
        self.forward_thread.join(timeout=2.5)
        if self.forward_thread.is_alive():
            self.forward_socket.close()
            self.forward_thread.join(timeout=0.5)
        try:
            stop_remote_reader(
                self.ssh_client, self.reader_pid, self.reader_name, self.remote_root
            )
        finally:
            self.ssh_client.close()


def open_remote_sensor_connection(args: argparse.Namespace) -> RemoteSensorSession:
    client, reader_pid, reader_name = start_remote_reader(args)
    channel = None
    sensor_socket = forward_socket = None
    stop_event = threading.Event()
    try:
        transport = client.get_transport()
        if transport is None or not transport.is_active():
            raise RuntimeError("SSH transport closed before opening the Xsens TCP channel.")
        channel = transport.open_channel(
            "direct-tcpip",
            dest_addr=("127.0.0.1", args.sensor_port),
            src_addr=("127.0.0.1", 0),
            timeout=10,
        )
        if channel is None:
            raise RuntimeError("SSH server refused TCP forwarding to the remote Xsens reader.")
        sensor_socket, forward_socket = socket.socketpair()
        sensor_socket.setblocking(False)
        forward_thread = threading.Thread(
            target=_forward_reader_stream,
            args=(channel, forward_socket, stop_event),
            name="faro-xsens-ssh-forward",
            daemon=True,
        )
        forward_thread.start()
        print(f"Xsens stream tunneled over SSH from {args.sensor_ip}:{args.sensor_port}.")
        return RemoteSensorSession(
            sensor_socket=sensor_socket,
            ssh_client=client,
            reader_pid=reader_pid,
            reader_name=reader_name,
            remote_root=args.remote_root,
            channel=channel,
            forward_socket=forward_socket,
            stop_event=stop_event,
            forward_thread=forward_thread,
        )
    except BaseException:
        stop_event.set()
        for resource in (sensor_socket, forward_socket, channel):
            if resource is not None:
                resource.close()
        try:
            stop_remote_reader(client, reader_pid, reader_name, args.remote_root)
        finally:
            client.close()
        raise


def open_faro_listener(host: str) -> socket.socket:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, FARO_BIND_PORT))
    listener.listen(1)
    listener.setblocking(False)
    return listener


def create_follow_selector(
    faro_listener: socket.socket, sensor: socket.socket
) -> selectors.BaseSelector:
    selector = selectors.DefaultSelector()
    try:
        selector.register(faro_listener, selectors.EVENT_READ, "faro_listener")
        selector.register(sensor, selectors.EVENT_READ, "imu")
    except BaseException:
        selector.close()
        raise
    return selector


def parse_motor_position_response(response: str) -> float | None:
    prefix = "POS_CDEG,"
    if not response.startswith(prefix):
        return None
    try:
        return int(response[len(prefix):]) / 100.0
    except ValueError:
        return None


def read_motor_position(port: serial.Serial, timeout_s: float = 2.5) -> float | None:
    """送出 p，將韌體回傳的 POS_CDEG 轉為多圈角度；完成的讀取失敗回傳 None。

    必須收完本筆交易的結束回覆，避免下一次誤用舊位置；交易未結束則拋出超時。
    """
    print("Reading actual multi-turn motor angle (command p / CAN 0x92).")
    port.write(b"p\n")
    port.flush()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        response = raw.decode("utf-8", errors="replace").rstrip()
        print(f"ESP32> {response}")
        position = parse_motor_position_response(response)
        if position is not None:
            return position
        # Intermediate CAN diagnostics precede this final failure line.
        # Consume the whole p response before permitting another transaction.
        if response == "Motor position read failed":
            return None
    raise TimeoutError("Motor position response did not finish; serial transaction is unsynchronized.")


def is_position_command_failure(response: str) -> bool:
    return any(
        marker in response
        for marker in (
            "Position TX failed",
            "Position TX queue failed",
            "Increment TX failed",
            "Increment TX queue failed",
            "Increment command failed",
        )
    )


def send_position_step(
    port: serial.Serial | None,
    delta_deg: float,
    speed_dps: int,
    sequence: int,
    imu_timestamp_ns: int,
) -> bool | None:
    """送出 n 增量指令；True 表示傳送已確認，False 表示失敗，None 表示結果不明。

    指令依序包含角度、速度、序號、IMU 時間及主機時間；不等待馬達轉到定位。
    dry-run（port 為 None）只列印指令並回傳 True，不代表收到硬體傳送確認。
    """
    command = f"n {delta_deg:.3f} {speed_dps} {sequence} {imu_timestamp_ns} {time.time_ns()}"
    if port is None:
        print(f"MOTOR> {command} (dry run)")
        return True
    port.write((command + "\n").encode("ascii"))
    port.flush()
    print(f"MOTOR> {command}")
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        response = raw.decode("utf-8", errors="replace").rstrip()
        print(f"ESP32> {response}")
        fields = response.split(",", 2)
        if len(fields) > 1 and fields[0] == "LAT" and fields[1] == str(sequence):
            return True
        if is_position_command_failure(response):
            return False
    return None


def send_soft_stop(port: serial.Serial | None) -> None:
    if port is None:
        return
    try:
        port.write(b"s\n")
        port.flush()
        print("MOTOR> s (soft stop)")
    except serial.SerialException as error:
        print(f"Could not send soft stop: {error}")


def run(args: argparse.Namespace) -> int:
    """管理感測器、Tracker TCP 與馬達 Serial，執行配對資料及編碼器回授控制。"""
    try:
        faro_listener = open_faro_listener(args.faro_bind_host)
    except OSError as error:
        print(f"Cannot listen for FARO adapter on {args.faro_bind_host}:{FARO_BIND_PORT}: {error}")
        return 1

    print(f"Waiting for FARO JSON adapter on {args.faro_bind_host}:{FARO_BIND_PORT}.")
    print("Expected each line: {\"distance_mm\":...,\"azimuth_deg\":...,\"zenith_deg\":...,\"timestamp_ns\":...}")
    sensor_session: RemoteSensorSession | None = None
    try:
        sensor_session = open_remote_sensor_connection(args)
        sensor = sensor_session.sensor_socket
    except (OSError, RuntimeError, paramiko.SSHException, ValueError) as error:
        faro_listener.close()
        print(f"Cannot start/connect to the remote Xsens reader through SSH: {error}")
        return 1

    motor_port: serial.Serial | None = None
    startup_motor_position_deg: float | None = 0.0
    if args.arm:
        try:
            motor_port = serial.Serial()
            motor_port.port = args.esp_port
            motor_port.baudrate = SERIAL_BAUD
            motor_port.timeout = 0.02
            motor_port.write_timeout = 1.0
            motor_port.dtr = False
            motor_port.rts = False
            motor_port.open()
            motor_port.reset_input_buffer()
            startup_motor_position_deg = read_motor_position(motor_port)
        except (OSError, serial.SerialException, RuntimeError) as error:
            if motor_port is not None and motor_port.is_open:
                motor_port.close()
            sensor_session.close()
            faro_listener.close()
            print(error)
            return 1

    try:
        selector = create_follow_selector(faro_listener, sensor)
    except (OSError, ValueError) as error:
        if motor_port is not None and motor_port.is_open:
            motor_port.close()
        sensor_session.close()
        faro_listener.close()
        print(f"Cannot initialize FARO/IMU socket selector: {error}")
        return 1
    buffers: dict[socket.socket, bytearray] = {sensor: bytearray()}
    yaw_publisher = YawTelemetryPublisher(YAW_TELEMETRY_HOST, YAW_TELEMETRY_PORT)
    yaw_publisher.start()
    faro_socket: socket.socket | None = None
    latest_imu: dict[str, float] | None = None
    latest_faro: dict[str, float | int] | None = None
    imu_received_at = faro_received_at = 0.0
    target_motor_position: float | None = None
    last_desired_heading: float | None = None
    startup_azimuth_deg: float | None = None
    startup_yaw_deg: float | None = None
    latest_motor_position_deg = startup_motor_position_deg
    next_startup_position_read_at = time.monotonic() + STARTUP_POSITION_RETRY_S
    next_position_feedback_at: float | None = None
    commanded_target: float | None = None
    last_encoder_progress_at: float | None = None
    wrong_way_samples = 0
    commanded_direction = 0
    direction_grace_samples = 0
    last_pair: tuple[int, int] | None = None
    last_good_pair_at: float | None = None
    last_report_at = 0.0
    stale_wait_reported = False
    misses = sequence = 0
    may_need_stop = False
    exit_code = 0

    print("Mode: ARMED" if args.arm else "Mode: DRY RUN (default; no motor commands).")
    print("Mode: FARO distance/Azimuth/Zenith; tracker origin is assumed.")
    print("Startup assumption: current motor direction is already aligned toward the tracker.")
    if args.arm:
        if startup_motor_position_deg is None:
            print("Initial motor angle unavailable; keeping the process running and retrying p every second.")
        else:
            print(f"Startup motor multi-turn position={startup_motor_position_deg:+.2f} deg.")
    print("Xsens and FARO heading frames are assumed aligned.")

    def process_pair(now: float) -> None:
        nonlocal target_motor_position, last_desired_heading
        nonlocal startup_azimuth_deg, startup_yaw_deg, next_position_feedback_at
        nonlocal latest_motor_position_deg, commanded_target, wrong_way_samples
        nonlocal commanded_direction, direction_grace_samples
        nonlocal last_encoder_progress_at, last_pair
        nonlocal last_report_at, last_good_pair_at, misses, sequence, may_need_stop, exit_code
        nonlocal stale_wait_reported
        if latest_imu is None or latest_faro is None:
            return
        if not received_samples_are_pairable(
            now,
            imu_received_at,
            faro_received_at,
            args.max_age_ms,
            args.max_arrival_skew_ms,
        ):
            return
        source_skew_ms = abs(
            int(latest_imu["source_time_ns"]) - int(latest_faro["source_time_ns"])
        ) / 1_000_000.0
        pair = (int(latest_imu["source_time_ns"]), int(latest_faro["source_time_ns"]))
        if pair == last_pair:
            return
        last_pair = pair
        last_good_pair_at = now
        if stale_wait_reported:
            print("Fresh FARO/Xsens samples resumed; follow control is active again.")
            stale_wait_reported = False
        desired = desired_motor_angle_from_azimuth(
            float(latest_faro["azimuth_deg"]),
            latest_imu["yaw_deg"],
        )
        if target_motor_position is None:
            target_motor_position = startup_motor_position_deg
            commanded_target = startup_motor_position_deg
            last_desired_heading = desired
            startup_azimuth_deg = float(latest_faro["azimuth_deg"])
            startup_yaw_deg = float(latest_imu["yaw_deg"])
            if motor_port is not None:
                next_position_feedback_at = now + POSITION_CORRECTION_PERIOD_S
            print(
                f"Startup heading baseline={desired:+.2f} deg; motor position baseline="
                f"{startup_motor_position_deg:+.2f} deg; no initial movement."
            )
        else:
            target_motor_position, last_desired_heading = accumulate_heading_target(
                float(target_motor_position), float(last_desired_heading), desired
            )

        if motor_port is None:
            # Dry run follows the same position target, but cannot claim encoder feedback.
            feedback_position = latest_motor_position_deg
            encoder_delta = 0.0
            do_feedback = now >= (next_position_feedback_at or 0.0)
        else:
            do_feedback = (
                next_position_feedback_at is not None and now >= next_position_feedback_at
            )
            feedback_position = latest_motor_position_deg

        if do_feedback:
            previous_feedback_position = latest_motor_position_deg
            if motor_port is not None:
                # A p transaction can take 1.6 s on the ESP32 failure path.
                # Finish that transaction before issuing another p or n.
                actual_position = read_motor_position(motor_port)
                next_position_feedback_at = time.monotonic() + POSITION_CORRECTION_PERIOD_S
                if actual_position is None:
                    misses += 1
                    print(f"Position feedback failed ({misses}/{args.max_misses}); no move sent.")
                    if misses >= args.max_misses:
                        print("Maximum consecutive position feedback failures reached; stopping.")
                        exit_code = 1
                    return
                misses = 0
                encoder_delta = actual_position - latest_motor_position_deg
                if abs(encoder_delta) >= ENCODER_PROGRESS_THRESHOLD_DEG:
                    last_encoder_progress_at = time.monotonic()
                latest_motor_position_deg = actual_position
                feedback_position = actual_position
            else:
                next_position_feedback_at = now + POSITION_CORRECTION_PERIOD_S

            if not received_samples_are_pairable(
                time.monotonic(),
                imu_received_at,
                faro_received_at,
                args.max_age_ms,
                args.max_arrival_skew_ms,
            ):
                print("Position feedback took too long; skipping correction on stale sensor data.")
            else:
                encoder_error = float(commanded_target) - feedback_position
                correction_limit = min(args.max_step_deg, args.speed_dps * POSITION_CORRECTION_PERIOD_S)
                correction = bounded_position_correction(
                    float(target_motor_position), feedback_position, correction_limit
                )
                send_correction = correction != 0.0 and abs(correction) >= args.angle_deadband
                next_direction = 1 if correction > 0.0 else -1
                reversing = send_correction and commanded_direction not in (0, next_direction)
                if abs(encoder_error) > MAX_POSITION_TRACKING_ERROR_DEG:
                    print(
                        f"Encoder tracking error {encoder_error:+.2f} deg exceeds "
                        f"{MAX_POSITION_TRACKING_ERROR_DEG:.1f} deg; stopping."
                    )
                    exit_code = 1
                elif direction_grace_samples > 0 or reversing:
                    # Do not count old-direction inertia as a new-direction fault.
                    direction_grace_samples = max(0, direction_grace_samples - 1)
                    wrong_way_samples = 0
                elif encoder_moved_opposite_command(
                    previous_feedback_position, feedback_position,
                    float(commanded_target), max(args.angle_deadband, 0.2),
                ):
                    wrong_way_samples += 1
                    print(
                        f"Opposite encoder motion ({wrong_way_samples}/{MAX_WRONG_WAY_SAMPLES}): "
                        f"previous={previous_feedback_position:+.2f} deg "
                        f"actual={feedback_position:+.2f} deg "
                        f"command target={commanded_target:+.2f} deg."
                    )
                    if wrong_way_samples >= MAX_WRONG_WAY_SAMPLES:
                        print("Encoder is moving away from the commanded position; stopping.")
                        exit_code = 1
                else:
                    wrong_way_samples = 0

                if abs(encoder_error) <= max(args.angle_deadband, 0.2):
                    last_encoder_progress_at = None
                elif last_encoder_progress_at is None:
                    last_encoder_progress_at = time.monotonic()
                elif time.monotonic() - last_encoder_progress_at > MOTOR_STALL_TIMEOUT_S:
                    print(
                        f"Encoder made no progress for {MOTOR_STALL_TIMEOUT_S:.1f}s while "
                        f"following commanded target={commanded_target:+.2f} deg "
                        f"(actual={feedback_position:+.2f} deg); stopping."
                    )
                    exit_code = 1

                if exit_code != 0:
                    return
                if not send_correction:
                    if abs(encoder_error) <= max(args.angle_deadband, 0.2):
                        misses = 0
                        print(
                            f"Position check OK: target={target_motor_position:+.2f} deg "
                            f"actual={feedback_position:+.2f} deg error="
                            f"{float(target_motor_position) - feedback_position:+.2f} deg."
                        )
                    else:
                        print(
                            f"Motor tracking: setpoint={commanded_target:+.2f} deg "
                            f"actual={feedback_position:+.2f} deg; waiting for motion, "
                            f"target error={float(target_motor_position) - feedback_position:+.2f} deg."
                        )
                else:
                    sequence += 1
                    may_need_stop = motor_port is not None
                    print(
                        f"Position command: target={target_motor_position:+.2f} deg "
                        f"commanded={commanded_target:+.2f} deg next={correction:+.2f} deg "
                        f"encoder={feedback_position:+.2f} deg "
                        f"step={correction:+.2f} deg at {args.speed_dps} dps."
                    )
                    command_result = send_position_step(
                        motor_port, correction, args.speed_dps, sequence,
                        int(latest_imu["source_time_ns"]),
                    )
                    if command_result is True:
                        # TX confirmation does not mean the motor reached this point.
                        commanded_target = feedback_position + correction
                        if reversing:
                            direction_grace_samples = 1
                            wrong_way_samples = 0
                        commanded_direction = next_direction
                        if motor_port is None:
                            latest_motor_position_deg += correction
                            feedback_position = latest_motor_position_deg
                        if last_encoder_progress_at is None:
                            last_encoder_progress_at = time.monotonic()
                        misses = 0
                    elif command_result is None:
                        print("Correction command outcome is ambiguous; not retrying it.")
                        exit_code = 1
                    else:
                        print(
                            "Correction TX was not confirmed; treating the relative move as ambiguous "
                            "and stopping without retry."
                        )
                        exit_code = 1

        motor_heading = None
        if startup_azimuth_deg is not None and startup_yaw_deg is not None:
            motor_heading = motor_world_heading_from_position(
                startup_azimuth_deg,
                startup_yaw_deg,
                float(latest_imu["yaw_deg"]),
                startup_motor_position_deg,
                latest_motor_position_deg,
            )
        yaw_publisher.update(
            float(latest_imu["yaw_deg"]),
            int(latest_imu["source_time_ns"]),
            motor_heading,
            motor_heading_source="encoder" if motor_port is not None else "estimate",
        )

        pending = float(target_motor_position) - latest_motor_position_deg

        if now - last_report_at >= 0.5:
            tracker_values = (
                f"azimuth={float(latest_faro['azimuth_deg']):+.3f} deg "
                f"distance={latest_faro['distance_mm']} mm zenith={latest_faro['zenith_deg']} deg"
            )
            print(f"FARO {tracker_values} yaw={latest_imu['yaw_deg']:+.2f} "
                  f"target={desired:+.2f} motor_heading={motor_heading!s} "
                  f"pending={pending:+.2f} deg source_skew={source_skew_ms:.1f} ms")
            last_report_at = now

    try:
        while exit_code == 0:
            events = selector.select(timeout=0.05)
            for key, _ in events:
                channel = key.fileobj
                if key.data == "faro_listener":
                    connection, address = faro_listener.accept()
                    if faro_socket is not None:
                        connection.close()
                        print("Rejected an extra FARO adapter connection.")
                        continue
                    faro_socket = connection
                    faro_socket.setblocking(False)
                    buffers[faro_socket] = bytearray()
                    selector.register(faro_socket, selectors.EVENT_READ, "faro")
                    print(f"FARO adapter connected from {address[0]}:{address[1]}.")
                    continue

                try:
                    sample, sample_received_at = drain_latest_sample(
                        channel, buffers[channel], key.data, args.faro_mode
                    )
                except ConnectionError:
                    if key.data != "faro":
                        raise
                    selector.unregister(channel)
                    buffers.pop(channel, None)
                    channel.close()
                    faro_socket = None
                    latest_faro = None
                    faro_received_at = 0.0
                    print("FARO adapter disconnected; follow process remains active and waits for reconnect.")
                    continue
                if sample is None:
                    continue
                if key.data == "imu":
                    latest_imu = sample
                    imu_received_at = sample_received_at
                else:
                    latest_faro = sample
                    faro_received_at = sample_received_at

            now = time.monotonic()
            if startup_motor_position_deg is None:
                if now >= next_startup_position_read_at:
                    position = read_motor_position(motor_port)
                    next_startup_position_read_at = time.monotonic() + STARTUP_POSITION_RETRY_S
                    if position is not None:
                        startup_motor_position_deg = position
                        latest_motor_position_deg = position
                        print(f"Startup motor multi-turn position={position:+.2f} deg; ready for fresh sensor data.")
                    else:
                        print("Still waiting for initial motor angle; retrying without exiting.")
                # Keep draining sensors while waiting; establish the heading
                # baseline with a fresh pair after the motor angle is known.
                continue
            process_pair(now)
            if (last_good_pair_at is not None
                    and time.monotonic() - last_good_pair_at
                    > (args.max_age_ms + args.max_arrival_skew_ms) / 1000.0):
                if not stale_wait_reported:
                    print(
                        "FARO/Xsens samples are stale; new follow commands are paused. "
                        "The process remains active and waits for fresh data."
                    )
                    stale_wait_reported = True
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except (OSError, ValueError, ConnectionError, serial.SerialException) as error:
        print(f"FARO follow stopped: {error}")
        exit_code = 1
    finally:
        yaw_publisher.close()
        if may_need_stop:
            send_soft_stop(motor_port)
        if motor_port is not None and motor_port.is_open:
            motor_port.close()
        selector.close()
        if faro_socket is not None:
            faro_socket.close()
        faro_listener.close()
        if sensor_session is not None:
            sensor_session.close()
    return exit_code


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Use FARO distance/Azimuth/Zenith and Xsens yaw to keep one MF5015V2 facing the origin."
    )
    parser.set_defaults(
        faro_mode=FARO_MODE,
        esp_port=ESP32_PORT,
        sensor_ip=SENSOR_IP,
        sensor_port=SENSOR_PORT,
        remote_user=REMOTE_USER,
        remote_root=REMOTE_ROOT,
        remote_python=REMOTE_PYTHON,
        faro_bind_host=FARO_BIND_HOST,
        speed_dps=SPEED_DPS,
        command_period=COMMAND_PERIOD_S,
        max_misses=MAX_MISSES,
        max_age_ms=MAX_SAMPLE_AGE_MS,
        max_arrival_skew_ms=MAX_ARRIVAL_SKEW_MS,
        max_step_deg=MAX_STEP_DEG,
        angle_deadband=ANGLE_DEADBAND_DEG,
    )
    parser.add_argument("--arm", action="store_true",
                        help="enable motor commands; dry-run is the default")
    args = parser.parse_args()
    if args.faro_mode != "azimuth":
        parser.error("This controller accepts only FARO distance/Azimuth/Zenith measurements.")
    numeric_values = [args.command_period, args.max_step_deg,
                      args.angle_deadband, args.max_age_ms, args.max_arrival_skew_ms]
    for value in numeric_values:
        if not math.isfinite(value):
            parser.error("numeric settings must be finite")
    if not 1 <= args.speed_dps <= 720:
        parser.error("SPEED_DPS must be between 1 and 720")
    if not 1 <= args.sensor_port <= 65535:
        parser.error("SENSOR_PORT must be between 1 and 65535")
    if not args.sensor_ip.strip() or not args.remote_user.strip():
        parser.error("SENSOR_IP and REMOTE_USER cannot be empty")
    if args.command_period <= 0 or args.max_step_deg <= 0:
        parser.error("command period and maximum step must be positive")
    if (args.angle_deadband < 0 or args.max_age_ms <= 0
            or args.max_arrival_skew_ms <= 0):
        parser.error("deadband cannot be negative; sample age and arrival skew must be positive")
    if args.max_misses < 1:
        parser.error("MAX_MISSES must be at least 1")
    if args.arm and args.max_age_ms > ARMED_MAX_SAMPLE_AGE_MS:
        parser.error(
            f"armed mode cannot use MAX_SAMPLE_AGE_MS above {ARMED_MAX_SAMPLE_AGE_MS:g}"
        )
    if args.arm and args.max_arrival_skew_ms > ARMED_MAX_ARRIVAL_SKEW_MS:
        parser.error(
            f"armed mode cannot use MAX_ARRIVAL_SKEW_MS above {ARMED_MAX_ARRIVAL_SKEW_MS:g}"
        )
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
