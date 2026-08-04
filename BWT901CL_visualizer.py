import argparse
import math
import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk


def int16_le(low_byte, high_byte):
    value = low_byte | (high_byte << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def normalize_angle(degrees):
    while degrees > 180:
        degrees -= 360
    while degrees <= -180:
        degrees += 360
    return degrees


def calc_gravity_roll_pitch(acc_g):
    ax, ay, az = acc_g
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def calc_gravity_mag_pose(frame):
    ax, ay, az = frame["acc_g"]
    mx, my, mz = frame["mag_raw"]
    roll, pitch = calc_gravity_roll_pitch((ax, ay, az))

    if abs(mx) + abs(my) + abs(mz) < 1e-6:
        return [roll, pitch, frame["angle_deg"][2]]

    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)

    mag_x = mx * cp + mz * sp
    mag_y = mx * sr * sp + my * cr - mz * sr * cp
    yaw = math.degrees(math.atan2(-mag_y, mag_x))
    return [roll, pitch, normalize_angle(yaw)]


class BwtLineParser:
    def __init__(self, angle_fields="f09"):
        self.buffer = bytearray()
        self.angle_fields = angle_fields

    def feed(self, payload):
        frames = []
        self.buffer.extend(payload)

        while True:
            line_end = self.buffer.find(b"\r\n")
            if line_end < 0:
                self._trim_noise()
                break

            line = bytes(self.buffer[:line_end])
            del self.buffer[: line_end + 2]

            frame = self._parse_line(line)
            if frame:
                frames.append(frame)

        return frames

    def _trim_noise(self):
        header_index = self.buffer.find(b"WT")
        if header_index > 0:
            del self.buffer[:header_index]
        if len(self.buffer) > 512:
            del self.buffer[:-64]

    def _parse_line(self, line):
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
        angle_f09 = [raw / 32768 * 180 for raw in values[9:12]]
        if self.angle_fields == "f06":
            angle_deg = [raw / 32768 * 180 for raw in values[6:9]]
            mag_raw = [raw / 10 for raw in values[9:12]]
        else:
            angle_deg = angle_f09
            mag_raw = mag_ut
        temperature_c = values[12] / 100

        return {
            "device_id": device_id,
            "time": f"{hour:02d}:{minute:02d}:{second:02d}.{millisecond:03d}",
            "acc_g": acc_g,
            "gyro_dps": gyro_dps,
            "mag_raw": mag_raw,
            "angle_deg": angle_deg,
            "mag_ut": mag_ut,
            "angle_f09_deg": angle_f09,
            "temperature_c": temperature_c,
            "raw_line": line,
        }


class SerialReader(threading.Thread):
    def __init__(
        self,
        port,
        baud,
        output_queue,
        stop_event,
        sensor_rate_hz=None,
        angle_fields="f09",
        algorithm="axis6",
    ):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.sensor_rate_hz = int(sensor_rate_hz) if sensor_rate_hz is not None else None
        self.algorithm = algorithm
        self.parser = BwtLineParser(angle_fields=angle_fields)

    def run(self):
        try:
            import serial
        except ModuleNotFoundError:
            self.output_queue.put(("error", "找不到 pyserial，請先執行：python -m pip install pyserial"))
            return

        try:
            with serial.Serial(self.port, self.baud, timeout=0.02) as ser:
                self.output_queue.put(("status", f"已連線 {self.port} / {self.baud} baud"))
                self._configure_sensor(ser)
                while not self.stop_event.is_set():
                    payload = ser.read(256)
                    for frame in self.parser.feed(payload):
                        self.output_queue.put(("frame", frame))
        except Exception as exc:
            self.output_queue.put(("error", f"序列埠錯誤：{exc}"))

    def _configure_sensor(self, ser):
        rate_codes = {
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
        rate_code = rate_codes.get(self.sensor_rate_hz) if self.sensor_rate_hz is not None else None
        if self.sensor_rate_hz is not None and rate_code is None:
            self.output_queue.put(("status", f"不支援的感測器輸出率：{self.sensor_rate_hz}Hz"))
            rate_code = None

        try:
            ser.reset_input_buffer()

            replies = []
            commands = [
                bytes.fromhex("FF AA 69 88 B5"),  # unlock
                bytes.fromhex("FF AA 23 00 00"),  # horizontal installation
            ]
            if self.algorithm == "axis6":
                commands.append(bytes.fromhex("FF AA 24 01 00"))  # relative yaw
            elif self.algorithm == "axis9":
                commands.append(bytes.fromhex("FF AA 24 00 00"))  # magnetic absolute yaw
            commands.append(bytes.fromhex("FF AA 1F 00 00"))  # 256Hz bandwidth
            if rate_code is not None:
                commands.append(bytes([0xFF, 0xAA, 0x03, rate_code, 0x00]))  # return rate
            commands.append(bytes.fromhex("FF AA 00 00 00"))  # save

            for command in commands:
                ser.write(command)
                ser.flush()
                time.sleep(0.12)
                reply = ser.read(256)
                if reply:
                    replies.append(reply)

            reply_text = b"".join(replies).decode("ascii", errors="ignore").strip()
            reply_text = reply_text.replace("\r", " ").replace("\n", " ").strip()
            if reply_text:
                self.output_queue.put(
                    ("status", f"已送出設定：algorithm={self.algorithm}, rate={self.sensor_rate_hz}Hz，回覆：{reply_text}")
                )
            else:
                self.output_queue.put(("status", "已送出設定，但沒有讀到 SET_OK"))
        except Exception as exc:
            self.output_queue.put(("status", f"感測器設定失敗：{exc}"))


class UdpReader(threading.Thread):
    def __init__(self, bind_host, port, output_queue, stop_event, angle_fields="f09"):
        super().__init__(daemon=True)
        self.bind_host = bind_host
        self.port = port
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.parser = BwtLineParser(angle_fields=angle_fields)

    def run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((self.bind_host, self.port))
                sock.settimeout(0.2)
                self.output_queue.put(("status", f"UDP 監聽中 {self.bind_host}:{self.port}"))
                while not self.stop_event.is_set():
                    try:
                        payload, remote = sock.recvfrom(4096)
                    except socket.timeout:
                        continue

                    for frame in self.parser.feed(payload):
                        frame["remote"] = f"{remote[0]}:{remote[1]}"
                        self.output_queue.put(("frame", frame))
        except Exception as exc:
            self.output_queue.put(("error", f"UDP 錯誤：{exc}"))


class TcpServerReader(threading.Thread):
    def __init__(self, bind_host, port, output_queue, stop_event, angle_fields="f09"):
        super().__init__(daemon=True)
        self.bind_host = bind_host
        self.port = port
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.parser = BwtLineParser(angle_fields=angle_fields)

    def run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.bind_host, self.port))
                server.listen(1)
                server.settimeout(0.2)
                self.output_queue.put(("status", f"TCP server 監聽中 {self.bind_host}:{self.port}"))

                while not self.stop_event.is_set():
                    try:
                        client, remote = server.accept()
                    except socket.timeout:
                        continue

                    with client:
                        client.settimeout(0.2)
                        self.output_queue.put(("status", f"TCP 已連線 {remote[0]}:{remote[1]}"))
                        while not self.stop_event.is_set():
                            try:
                                payload = client.recv(4096)
                            except socket.timeout:
                                continue
                            if not payload:
                                break

                            for frame in self.parser.feed(payload):
                                frame["remote"] = f"{remote[0]}:{remote[1]}"
                                self.output_queue.put(("frame", frame))
        except Exception as exc:
            self.output_queue.put(("error", f"TCP 錯誤：{exc}"))


class AppFileReader(threading.Thread):
    def __init__(self, file_path, output_queue, stop_event, hz):
        super().__init__(daemon=True)
        self.file_path = file_path
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.interval = max(0.001, 1 / hz)

    def run(self):
        try:
            with open(self.file_path, "r", encoding="utf-8-sig", errors="replace") as file:
                lines = [line.rstrip("\n") for line in file if line.strip()]
        except Exception as exc:
            self.output_queue.put(("error", f"檔案讀取錯誤：{exc}"))
            return

        rows = [line.split("\t") for line in lines[1:]]
        frames = []
        for row in rows:
            if len(row) < 16:
                continue
            try:
                frames.append(
                    {
                        "device_id": row[1],
                        "time": self._format_sensor_time(row[2]),
                        "acc_g": [float(row[3]), float(row[4]), float(row[5])],
                        "gyro_dps": [float(row[6]), float(row[7]), float(row[8])],
                        "angle_deg": [float(row[9]), float(row[10]), float(row[11])],
                        "mag_raw": [float(row[12]), float(row[13]), float(row[14])],
                        "temperature_c": float(row[15]),
                    }
                )
            except ValueError:
                continue

        if not frames:
            self.output_queue.put(("error", "檔案內沒有可解析的官方 APP 資料列"))
            return

        self.output_queue.put(("status", f"檔案回放：{len(frames)} 筆"))
        index = 0
        while not self.stop_event.is_set():
            self.output_queue.put(("frame", frames[index]))
            index = (index + 1) % len(frames)
            time.sleep(self.interval)

    def _format_sensor_time(self, text):
        parts = text.split()
        if len(parts) < 2:
            return text
        time_parts = parts[-1].split(":")
        if len(time_parts) != 4:
            return parts[-1]
        hour, minute, second, millisecond = time_parts
        return f"{int(hour):02d}:{int(minute):02d}:{int(second):02d}.{int(millisecond):03d}"


def rotate_point(point, roll_deg, pitch_deg, yaw_deg):
    x, y, z = point
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # Rotate by X/Roll, then Y/Pitch, then Z/Yaw.
    y, z = y * cr - z * sr, y * sr + z * cr
    x, z = x * cp + z * sp, -x * sp + z * cp
    x, y = x * cy - y * sy, x * sy + y * cy
    return x, y, z


def matrix_from_pose(roll_deg, pitch_deg, yaw_deg):
    x_axis = rotate_point((1, 0, 0), roll_deg, pitch_deg, yaw_deg)
    y_axis = rotate_point((0, 1, 0), roll_deg, pitch_deg, yaw_deg)
    z_axis = rotate_point((0, 0, 1), roll_deg, pitch_deg, yaw_deg)
    return [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]


def transpose_matrix(matrix):
    return [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]


def multiply_matrices(left, right):
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def transform_point(matrix, point):
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


def display_basis(point):
    x, y, z = point
    return -y, x, z


def transform_for_display(point, display_matrix):
    return display_basis(transform_point(display_matrix, point))


def project_point(point, width, height, scale):
    x, y, z = point
    distance = 5.0
    factor = distance / (distance - z)
    screen_x = width / 2 + x * scale * factor
    screen_y = height / 2 - y * scale * factor
    return screen_x, screen_y


class VisualizerApp:
    def __init__(
        self,
        root,
        port,
        baud,
        demo=False,
        hz=20,
        file_path=None,
        source="serial",
        bind_host="0.0.0.0",
        wifi_port=1399,
        sensor_rate_hz=100,
        angle_fields="f09",
        pose_source="device",
        algorithm="axis6",
        display_mode="current",
        stationary_lock=False,
        stationary_gyro_threshold=1.0,
    ):
        self.root = root
        self.root.title("BWT901CL 姿態視覺化")
        self.root.geometry("1120x720")
        self.root.minsize(860, 560)

        self.port = port
        self.baud = baud
        self.demo = demo
        self.file_path = file_path
        self.hz = hz
        self.source = source
        self.bind_host = bind_host
        self.wifi_port = wifi_port
        self.sensor_rate_hz = sensor_rate_hz
        self.angle_fields = angle_fields
        self.pose_source = pose_source
        self.algorithm = algorithm
        self.display_mode = display_mode
        self.stationary_lock = stationary_lock
        self.stationary_gyro_threshold = stationary_gyro_threshold
        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = None
        self.latest_frame = None
        self.last_frame_time = None
        self.reference_pose = None
        self.display_reference_pose = None
        self.display_reference_matrix = None
        self.locked_display_matrix = None
        self.received_frame_count = 0
        self.rate_window_start = time.time()
        self.measured_data_hz = 0.0
        self.base_status = "等待資料..."
        self.frame_interval_ms = max(1, round(1000 / hz))
        self.demo_start = time.time()

        self._build_ui()
        self._start_reader()
        self._schedule_update()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=(16, 12))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)

        ttk.Label(header, text="BWT901CL 姿態視覺化", font=("Microsoft JhengHei UI", 16, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        self.status_var = tk.StringVar(value="等待資料...")
        ttk.Label(header, textvariable=self.status_var).grid(row=0, column=1, sticky="e")

        content = ttk.Frame(self.root, padding=(16, 0, 16, 16))
        content.grid(row=1, column=0, sticky="nsew")
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=0)
        content.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(content, background="#10151f", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        panel = ttk.LabelFrame(content, text="即時資料", padding=(12, 10))
        panel.grid(row=0, column=1, sticky="ns", padx=(14, 0))
        panel.columnconfigure(0, weight=1)

        self.data_text_var = tk.StringVar(value=self._format_data_text())
        self.data_label = ttk.Label(
            panel,
            textvariable=self.data_text_var,
            justify="left",
            anchor="nw",
            font=("Consolas", 11),
            wraplength=340,
        )
        self.data_label.grid(row=0, column=0, sticky="new")

    def _start_reader(self):
        if self.demo:
            self.status_var.set("示範模式")
            return

        if self.file_path:
            self.reader = AppFileReader(self.file_path, self.data_queue, self.stop_event, hz=self.hz)
        elif self.source == "udp":
            self.reader = UdpReader(self.bind_host, self.wifi_port, self.data_queue, self.stop_event, self.angle_fields)
        elif self.source == "tcp":
            self.reader = TcpServerReader(self.bind_host, self.wifi_port, self.data_queue, self.stop_event, self.angle_fields)
        else:
            self.reader = SerialReader(
                self.port,
                self.baud,
                self.data_queue,
                self.stop_event,
                sensor_rate_hz=self.sensor_rate_hz,
                angle_fields=self.angle_fields,
                algorithm=self.algorithm,
            )
        self.reader.start()

    def _schedule_update(self):
        self._poll_queue()
        if self.demo:
            self._update_demo_frame()
        self._draw()
        self.root.after(self.frame_interval_ms, self._schedule_update)

    def _poll_queue(self):
        while True:
            try:
                message_type, payload = self.data_queue.get_nowait()
            except queue.Empty:
                break

            if message_type == "frame":
                self.latest_frame = payload
                self.last_frame_time = time.time()
                self.received_frame_count += 1
            elif message_type == "status":
                self.base_status = payload
                self.status_var.set(payload)
            elif message_type == "error":
                self.base_status = payload
                self.status_var.set(payload)

    def _update_demo_frame(self):
        elapsed = time.time() - self.demo_start
        self.latest_frame = {
            "time": time.strftime("%H:%M:%S") + f".{int((elapsed % 1) * 1000):03d}",
            "acc_g": [0.0, 0.0, 1.0],
            "gyro_dps": [0.0, 0.0, 30.0],
            "angle_deg": [
                35 * math.sin(elapsed * 0.8),
                25 * math.sin(elapsed * 0.6),
                (elapsed * 35) % 360 - 180,
            ],
        }

    def _draw(self):
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)

        if not self.latest_frame:
            self._draw_empty(width, height)
            return

        pose = self._select_pose(self.latest_frame)
        display_matrix = self._display_matrix(self.latest_frame, pose)
        self._update_text_values(self.latest_frame, pose)

        self._draw_axis_view(width, height, display_matrix)
        self._update_measured_rate()

    def _draw_empty(self, width, height):
        self.canvas.create_text(
            width / 2,
            height / 2,
            text="等待 BWT901CL 資料...",
            fill="#dce6f2",
            font=("Microsoft JhengHei UI", 18, "bold"),
        )

    def _relative_pose(self, pose):
        if self.reference_pose is None:
            self.reference_pose = pose[:]
        return [normalize_angle(pose[index] - self.reference_pose[index]) for index in range(3)]

    def _is_stationary(self, frame):
        gx, gy, gz = frame["gyro_dps"]
        return max(abs(gx), abs(gy), abs(gz)) < self.stationary_gyro_threshold

    def _display_matrix(self, frame, pose):
        current_matrix = matrix_from_pose(*pose)
        if self.display_mode == "initial-relative":
            if self.display_reference_matrix is None:
                self.display_reference_pose = pose[:]
                self.display_reference_matrix = current_matrix
            display_matrix = multiply_matrices(transpose_matrix(self.display_reference_matrix), current_matrix)
        else:
            display_matrix = current_matrix

        if not self.stationary_lock or self.demo:
            self.locked_display_matrix = display_matrix
            return display_matrix
        if self.locked_display_matrix is None:
            self.locked_display_matrix = display_matrix
        elif not self._is_stationary(frame):
            self.locked_display_matrix = display_matrix
        return self.locked_display_matrix

    def _select_pose(self, frame):
        if self.pose_source == "device":
            return frame["angle_deg"]
        if self.pose_source == "relative-device":
            return self._relative_pose(frame["angle_deg"])
        if self.pose_source == "gravity-mag":
            return calc_gravity_mag_pose(frame)
        return calc_gravity_mag_pose(frame)

    def _pose_label(self):
        if self.pose_source == "device":
            return "感測器原始角度"
        if self.pose_source == "relative-device":
            return "目前姿態：相對啟動角度"
        return "診斷：重力+磁場計算"

    def _update_text_values(self, frame, pose):
        roll, pitch, yaw = pose
        ax, ay, az = frame["acc_g"]
        gx, gy, gz = frame["gyro_dps"]

        self.data_text_var.set(
            self._format_data_text(
                time_text=frame["time"],
                acc=(ax, ay, az),
                gyro=(gx, gy, gz),
                angle=(roll, pitch, yaw),
                angle_label=self._pose_label(),
            )
        )

        if not self.demo and self.last_frame_time and time.time() - self.last_frame_time > 2:
            self.status_var.set("超過 2 秒未收到新資料")

    def _update_measured_rate(self):
        now = time.time()
        elapsed = now - self.rate_window_start
        if elapsed < 1.0:
            return

        self.measured_data_hz = self.received_frame_count / elapsed
        self.received_frame_count = 0
        self.rate_window_start = now
        self.status_var.set(f"{self.base_status} | 實際資料 {self.measured_data_hz:.1f}Hz / 畫面 {1000 / self.frame_interval_ms:.1f}Hz")

    def _format_data_text(self, time_text="--", acc=None, gyro=None, angle=None, angle_label="目前姿態：重力+磁場"):
        if acc is None:
            acc = (None, None, None)
        if gyro is None:
            gyro = (None, None, None)
        if angle is None:
            angle = (None, None, None)

        return (
            f"時間：{time_text}\n\n"
            "三軸加速度\n"
            f"  X {self._format_value(acc[0], 'g')}\n"
            f"  Y {self._format_value(acc[1], 'g')}\n"
            f"  Z {self._format_value(acc[2], 'g')}\n\n"
            "三軸角速度\n"
            f"  X {self._format_value(gyro[0], 'deg/s')}\n"
            f"  Y {self._format_value(gyro[1], 'deg/s')}\n"
            f"  Z {self._format_value(gyro[2], 'deg/s')}\n\n"
            f"三軸角度（{angle_label}）\n"
            f"  Roll  {self._format_value(angle[0], 'deg')}\n"
            f"  Pitch {self._format_value(angle[1], 'deg')}\n"
            f"  Yaw   {self._format_value(angle[2], 'deg')}"
        )

    def _format_value(self, value, unit):
        if value is None:
            return f"{'--':>10} {unit}"
        return f"{value:10.3f} {unit}"

    def _draw_grid(self, width, height):
        center_y = height * 0.58
        for offset in range(-5, 6):
            y = center_y + offset * 28
            self.canvas.create_line(80, y, width - 80, y, fill="#1e2b3f", width=1)
        self.canvas.create_text(
            18,
            18,
            text="畫面：現在姿態  零角度時 X向上  Y向左  Z向外",
            fill="#dce6f2",
            anchor="nw",
            font=("Microsoft JhengHei UI", 11),
        )

    def _draw_axis_view(self, width, height, display_matrix):
        self._draw_grid(width, height)
        self._draw_wire_box(width, height, display_matrix)
        self._draw_axis_labels(width, height, display_matrix)

    def _draw_wire_box(self, width, height, display_matrix):
        vertices = [
            (-0.75, -0.45, -0.25),
            (0.75, -0.45, -0.25),
            (0.75, 0.45, -0.25),
            (-0.75, 0.45, -0.25),
            (-0.75, -0.45, 0.25),
            (0.75, -0.45, 0.25),
            (0.75, 0.45, 0.25),
            (-0.75, 0.45, 0.25),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        scale = min(width, height) * 0.24
        rotated = [transform_for_display(point, display_matrix) for point in vertices]
        projected = [project_point(point, width, height, scale) for point in rotated]

        for start, end in edges:
            self.canvas.create_line(*projected[start], *projected[end], fill="#9fb4cc", width=1)

    def _draw_cube(self, width, height, display_matrix):
        vertices = [
            (-1, -0.65, -0.35),
            (1, -0.65, -0.35),
            (1, 0.65, -0.35),
            (-1, 0.65, -0.35),
            (-1, -0.65, 0.35),
            (1, -0.65, 0.35),
            (1, 0.65, 0.35),
            (-1, 0.65, 0.35),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        scale = min(width, height) * 0.22
        rotated = [transform_for_display(point, display_matrix) for point in vertices]
        projected = [project_point(point, width, height, scale) for point in rotated]

        faces = [
            (0, 1, 2, 3, "#284160"),
            (4, 5, 6, 7, "#3c6795"),
            (0, 1, 5, 4, "#315277"),
            (2, 3, 7, 6, "#223850"),
            (1, 2, 6, 5, "#376087"),
            (0, 3, 7, 4, "#1f344b"),
        ]
        face_depths = [(sum(rotated[i][2] for i in face[:4]) / 4, face) for face in faces]
        for _, face in sorted(face_depths):
            points = []
            for index in face[:4]:
                points.extend(projected[index])
            self.canvas.create_polygon(points, fill=face[4], outline="#94b7da", width=1)

        for start, end in edges:
            self.canvas.create_line(*projected[start], *projected[end], fill="#dce6f2", width=2)

    def _draw_axis_labels(self, width, height, display_matrix):
        axis_length = 1.9
        axes = [
            ("X", (axis_length, 0, 0), "#ff5f57"),
            ("Y", (0, axis_length, 0), "#55d17a"),
            ("Z", (0, 0, axis_length), "#4fa3ff"),
        ]
        scale = min(width, height) * 0.22
        origin = project_point(transform_for_display((0, 0, 0), display_matrix), width, height, scale)

        for label, endpoint, color in axes:
            rotated = transform_for_display(endpoint, display_matrix)
            projected = project_point(rotated, width, height, scale)
            distance = math.hypot(projected[0] - origin[0], projected[1] - origin[1])
            if distance < 8:
                radius = 8
                self.canvas.create_oval(
                    origin[0] - radius,
                    origin[1] - radius,
                    origin[0] + radius,
                    origin[1] + radius,
                    outline=color,
                    width=4,
                )
                projected = (origin[0] + 22, origin[1] - 22)
            else:
                self.canvas.create_line(*origin, *projected, fill=color, width=5, arrow=tk.LAST)
            self.canvas.create_text(
                projected[0],
                projected[1],
                text=label,
                fill=color,
                font=("Microsoft JhengHei UI", 16, "bold"),
                anchor="center",
            )

    def _on_close(self):
        self.stop_event.set()
        self.root.destroy()


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Visualize BWT901CL Roll/Pitch/Yaw from serial, Wi-Fi, or APP file data.")
    parser.add_argument("--source", choices=["serial", "udp", "tcp"], default="serial", help="Data source, default: serial")
    parser.add_argument("--port", default="COM4", help="COM port, default: COM4")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate, default: 9600")
    parser.add_argument("--bind-host", default="0.0.0.0", help="Wi-Fi local bind host, default: 0.0.0.0")
    parser.add_argument("--wifi-port", type=int, default=1399, help="Wi-Fi UDP/TCP port, default: 1399")
    parser.add_argument("--hz", type=float, default=60, help="Visual/data refresh rate, default: 60Hz")
    parser.add_argument("--sensor-rate-hz", type=float, default=100, help="Set sensor output rate on serial startup, default: 100Hz")
    parser.add_argument("--angle-fields", choices=["f09", "f06"], default="f09", help="Angle fields in WT text data, default: f09")
    parser.add_argument("--algorithm", choices=["axis6", "axis9", "none"], default="axis6", help="Sensor algorithm for serial startup, default: axis6")
    parser.add_argument("--display-mode", choices=["current", "initial-relative"], default="current", help="3D display mode, default: current")
    parser.add_argument("--stationary-lock", action="store_true", help="Hold display while gyro is near zero")
    parser.add_argument("--no-stationary-lock", action="store_true", help="Disable display hold while gyro is near zero")
    parser.add_argument("--stationary-gyro-threshold", type=float, default=1.0, help="Gyro threshold for stationary display lock, default: 1.0 deg/s")
    parser.add_argument(
        "--pose-source",
        choices=["relative-device", "gravity-mag", "device"],
        default="device",
        help="Pose source, default: device",
    )
    parser.add_argument("--file", help="Replay official WitMotion APP exported txt file")
    parser.add_argument("--demo", action="store_true", help="Run without serial data")
    return parser


def main():
    args = build_arg_parser().parse_args()
    root = tk.Tk()
    VisualizerApp(
        root,
        args.port,
        args.baud,
        demo=args.demo,
        hz=args.hz,
        file_path=args.file,
        source=args.source,
        bind_host=args.bind_host,
        wifi_port=args.wifi_port,
        sensor_rate_hz=args.sensor_rate_hz,
        angle_fields=args.angle_fields,
        pose_source=args.pose_source,
        algorithm=args.algorithm,
        display_mode=args.display_mode,
        stationary_lock=args.stationary_lock and not args.no_stationary_lock,
        stationary_gyro_threshold=args.stationary_gyro_threshold,
    )
    root.mainloop()


if __name__ == "__main__":
    main()
