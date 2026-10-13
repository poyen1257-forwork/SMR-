"""以滑鼠模擬 XY 平面上的 IMU／SMR 移動，提供 Tracker 距離與角度資料。

此程式使用 Tkinter 圖形介面；Tracker 應設於原點 (0, 0)，橘點代表可拖曳的
IMU／SMR 位置。位置輸入與計算使用毫米，畫面距離以公尺顯示並保留毫米精度。
由相對 XY 計算距離及方位角，平面模擬的 Z 為 0、天頂角固定為 90 度。

按「連線並開始傳送」後，將資料經 TCP 5010 傳給 faro_yaw_follow.py，
預設傳送頻率為 100 Hz，並將傳送紀錄寫入 logs/tracker_simulator.csv。
資料採本專案的逐行 JSON（NDJSON）格式，不是 FARO SDK 原始輸出；
實際 Tracker 的 SDK 轉接程式需提供同樣的欄位與單位。
TCP 5011 接收控制端回傳的 IMU yaw 及馬達方向，並以方向箭頭顯示。

從專案根目錄啟動：python python/tracker_simulator.py
控制端另執行 python python/faro_yaw_follow.py --arm，模擬器本身不直接控制馬達。
"""

from __future__ import annotations

import csv
import json
import math
import queue
import socket
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5010
YAW_TELEMETRY_HOST = "127.0.0.1"
YAW_TELEMETRY_PORT = 5011
DEFAULT_RATE_HZ = 100.0
LOG_PATH = Path(__file__).resolve().parents[1] / "logs" / "tracker_simulator.csv"
LOG_FIELDS = (
    "timestamp_ns", "tracker_x_mm", "tracker_y_mm", "imu_x_mm", "imu_y_mm",
    "x_mm", "y_mm", "distance_mm", "azimuth_deg", "zenith_deg", "valid",
)
TRACKER_OUTPUT_FIELDS = (
    "distance_mm", "azimuth_deg", "zenith_deg", "valid", "timestamp_ns",
)


def make_measurement(
    tracker_x_mm: float,
    tracker_y_mm: float,
    imu_x_mm: float,
    imu_y_mm: float,
    timestamp_ns: int | None = None,
) -> dict[str, float | int | bool | None]:
    """由 Tracker 與 IMU 的 XY 毫米座標，計算距離、角度及有效狀態。"""
    coordinates = (tracker_x_mm, tracker_y_mm, imu_x_mm, imu_y_mm)
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError("coordinates must be finite")

    x_mm = imu_x_mm - tracker_x_mm
    y_mm = imu_y_mm - tracker_y_mm
    distance_mm = math.hypot(x_mm, y_mm)
    valid = distance_mm >= 1.0
    azimuth = math.degrees(math.atan2(y_mm, x_mm)) if valid else None
    stamp = time.time_ns() if timestamp_ns is None else int(timestamp_ns)
    if stamp <= 0:
        raise ValueError("timestamp_ns must be positive")

    return {
        "x_mm": round(x_mm, 3),
        "y_mm": round(y_mm, 3),
        "z_mm": 0.0,
        "distance_mm": round(distance_mm, 3),
        "azimuth_deg": round(azimuth, 6) if azimuth is not None else None,
        "zenith_deg": 90.0,
        "timestamp_ns": stamp,
        "valid": valid,
    }


def format_distance_m(distance_mm: float) -> str:
    """將毫米距離顯示為公尺，保留小數三位，也就是 1 毫米顯示精度。"""
    return f"{distance_mm / 1000.0:.3f} m"


def encode_measurement_line(measurement: dict[str, object]) -> bytes:
    """挑出距離、角度、有效狀態與時間戳記，編碼成一行 UTF-8 JSON。"""
    payload = {key: measurement[key] for key in TRACKER_OUTPUT_FIELDS}
    return json.dumps(
        payload, allow_nan=False, separators=(",", ":")
    ).encode("utf-8") + b"\n"


def parse_yaw_telemetry_line(line: bytes) -> dict[str, float | str] | None:
    """解析控制端的 IMU yaw 與馬達方向回報，供 UI 更新方向箭頭。"""
    try:
        packet = json.loads(line.decode("utf-8"))
        yaw = float(packet["yaw_deg"])
        if packet.get("type") != "imu_yaw" or not math.isfinite(yaw):
            return None
        sample: dict[str, float | str] = {"yaw_deg": yaw}
        local_monotonic = float(packet.get("local_monotonic_s", time.monotonic()))
        if not math.isfinite(local_monotonic):
            return None
        sample["local_monotonic_s"] = local_monotonic
        if "motor_heading_deg" in packet:
            motor_heading = float(packet["motor_heading_deg"])
            if not math.isfinite(motor_heading):
                return None
            source = packet.get("motor_heading_source", "encoder")
            if source not in ("encoder", "estimate"):
                return None
            sample["motor_heading_deg"] = motor_heading
            sample["motor_heading_source"] = source
        return sample
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None


def heading_arrow_endpoint(
    x: float, y: float, yaw_deg: float, length_px: float
) -> tuple[float, float]:
    """Map +X/CCW yaw to a canvas endpoint where screen Y increases downward."""
    radians = math.radians(yaw_deg)
    return x + length_px * math.cos(radians), y - length_px * math.sin(radians)


class TrackerSimulator:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Tracker XY 模擬器")
        self.root.geometry("1160x760")
        self.root.minsize(900, 620)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.tracker_x_var = tk.StringVar(value="0")
        self.tracker_y_var = tk.StringVar(value="0")
        self.start_imu_x_var = tk.StringVar(value="300")
        self.start_imu_y_var = tk.StringVar(value="0")
        self.scale_var = tk.StringVar(value="100")
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        self.rate_var = tk.StringVar(value=str(DEFAULT_RATE_HZ))
        self.status_var = tk.StringVar(value="未連線；拖曳 IMU 標記可改變位置")
        self.position_var = tk.StringVar(value="")
        self.distance_var = tk.StringVar(value="")
        self.azimuth_var = tk.StringVar(value="")
        self.yaw_var = tk.StringVar(value="IMU / 馬達方向：等待 Xsens")

        self.position_lock = threading.Lock()
        self.connection_lock = threading.Lock()
        self.tracker_position = (0.0, 0.0)
        self.initial_imu_position = (300.0, 0.0)
        self.imu_position = (300.0, 0.0)
        self.client_socket: socket.socket | None = None
        self.sender_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.ui_messages: queue.Queue[tuple[str, str]] = queue.Queue()
        self.yaw_messages: queue.Queue[dict[str, float | str]] = queue.Queue(maxsize=1)
        self.imu_yaw_deg: float | None = None
        self.motor_heading_deg: float | None = None
        self.motor_heading_source = "encoder"
        self.motor_heading_received_at: float | None = None
        self.yaw_server_socket: socket.socket | None = None
        self.yaw_server_stop = threading.Event()
        self.yaw_receiver_thread: threading.Thread | None = None
        self.dragging_imu = False
        self.canvas_origin = (0.0, 0.0)
        self.imu_canvas_point = (0.0, 0.0)

        self._build_ui()
        self.apply_settings()
        self._start_yaw_receiver()
        self.root.after(100, self._poll_sender_messages)

    def _build_ui(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        controls = ttk.Frame(outer, width=310, padding=(0, 0, 14, 0))
        controls.pack(side="left", fill="y")
        controls.pack_propagate(False)

        ttk.Label(controls, text="Tracker / IMU", font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(controls, text="座標單位：mm；正 Y 朝上", foreground="#53636a").pack(anchor="w", pady=(2, 12))

        self._section(controls, "位置設定")
        self._entry_row(controls, "Tracker X", self.tracker_x_var)
        self._entry_row(controls, "Tracker Y", self.tracker_y_var)
        self._entry_row(controls, "IMU 初始 X", self.start_imu_x_var)
        self._entry_row(controls, "IMU 初始 Y", self.start_imu_y_var)
        self._entry_row(controls, "比例 mm/100px", self.scale_var)
        row = ttk.Frame(controls)
        row.pack(fill="x", pady=(8, 12))
        ttk.Button(row, text="套用並重設 IMU", command=self.apply_settings).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="回到初始點", command=self.reset_imu).pack(side="left", padx=(6, 0))

        self._section(controls, "TCP/IP 輸出")
        self._entry_row(controls, "yaw_follow 主機", self.host_var)
        self._entry_row(controls, "TCP Port", self.port_var)
        self._entry_row(controls, "傳送頻率 Hz", self.rate_var)
        self.connection_button = ttk.Button(controls, text="連線並開始傳送", command=self.toggle_connection)
        self.connection_button.pack(fill="x", pady=(8, 6))
        ttk.Label(controls, textvariable=self.status_var, wraplength=285).pack(anchor="w", pady=(0, 10))

        self._section(controls, "即時量測")
        ttk.Label(controls, textvariable=self.position_var, font=("Consolas", 10)).pack(anchor="w", pady=2)
        ttk.Label(controls, textvariable=self.distance_var, font=("Segoe UI", 13, "bold"), foreground="#075e59").pack(anchor="w", pady=2)
        ttk.Label(controls, textvariable=self.azimuth_var, font=("Segoe UI", 10)).pack(anchor="w", pady=2)
        ttk.Label(controls, textvariable=self.yaw_var, font=("Segoe UI", 10, "bold"), foreground="#1d4ed8").pack(anchor="w", pady=2)
        ttk.Separator(controls).pack(fill="x", pady=12)
        ttk.Label(controls, text="量測記錄", font=("Segoe UI", 10, "bold")).pack(anchor="w")
        ttk.Label(controls, text=str(LOG_PATH), wraplength=285, foreground="#53636a").pack(anchor="w", pady=(3, 8))
        ttk.Label(
            controls,
            text="輸出距離、Azimuth、Zenith 球面量測的專案 JSON；不是 FARO SDK 原生封包。",
            wraplength=285,
            foreground="#53636a",
        ).pack(anchor="w")

        view = ttk.Frame(outer)
        view.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(view, background="#f4f7f6", highlightthickness=1, highlightbackground="#ccd6d4")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._begin_drag)
        self.canvas.bind("<B1-Motion>", self._drag_imu)
        self.canvas.bind("<ButtonRelease-1>", self._end_drag)

    def _start_yaw_receiver(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind((YAW_TELEMETRY_HOST, YAW_TELEMETRY_PORT))
            listener.listen(1)
            listener.settimeout(0.25)
        except OSError as error:
            listener.close()
            self.yaw_var.set(f"IMU 方向：無法監聽 {YAW_TELEMETRY_PORT} ({error})")
            return

        self.yaw_server_socket = listener
        self.yaw_receiver_thread = threading.Thread(
            target=self._yaw_receiver_loop,
            name="tracker-yaw-receiver",
            daemon=True,
        )
        self.yaw_receiver_thread.start()

    def _yaw_receiver_loop(self) -> None:
        listener = self.yaw_server_socket
        if listener is None:
            return
        while not self.yaw_server_stop.is_set():
            try:
                connection, _address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with connection:
                connection.settimeout(0.25)
                buffer = bytearray()
                while not self.yaw_server_stop.is_set():
                    try:
                        data = connection.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not data:
                        break
                    buffer.extend(data)
                    if len(buffer) > 65536:
                        buffer.clear()
                        continue
                    while b"\n" in buffer:
                        line, _, remainder = buffer.partition(b"\n")
                        buffer[:] = remainder
                        sample = parse_yaw_telemetry_line(line)
                        if sample is None:
                            continue
                        try:
                            self.yaw_messages.put_nowait(sample)
                        except queue.Full:
                            try:
                                self.yaw_messages.get_nowait()
                            except queue.Empty:
                                pass
                            try:
                                self.yaw_messages.put_nowait(sample)
                            except queue.Full:
                                pass

    @staticmethod
    def _section(parent: ttk.Frame, title: str) -> None:
        ttk.Label(parent, text=title, font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(10, 4))

    @staticmethod
    def _entry_row(parent: ttk.Frame, label: str, variable: tk.StringVar) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, width=17).pack(side="left")
        ttk.Entry(row, textvariable=variable, width=13).pack(side="right", fill="x", expand=True)

    def apply_settings(self) -> None:
        try:
            tracker = (float(self.tracker_x_var.get()), float(self.tracker_y_var.get()))
            imu = (float(self.start_imu_x_var.get()), float(self.start_imu_y_var.get()))
            scale = float(self.scale_var.get())
            if not all(math.isfinite(value) for value in (*tracker, *imu, scale)) or scale <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("輸入錯誤", "座標必須是有限數值，比例必須大於 0。")
            return

        with self.position_lock:
            self.tracker_position = tracker
            self.initial_imu_position = imu
            self.imu_position = imu
        self.redraw()

    def reset_imu(self) -> None:
        with self.position_lock:
            self.imu_position = self.initial_imu_position
        self.redraw()

    def _snapshot(self) -> tuple[tuple[float, float], tuple[float, float]]:
        with self.position_lock:
            return self.tracker_position, self.imu_position

    def _mm_per_100px(self) -> float:
        try:
            value = float(self.scale_var.get())
            return value if math.isfinite(value) and value > 0 else 100.0
        except ValueError:
            return 100.0

    @staticmethod
    def _grid_step(scale: float) -> float:
        candidates = (10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000)
        pixels = [step * 100.0 / scale for step in candidates]
        eligible = [(abs(pixel - 80.0), step) for pixel, step in zip(pixels, candidates) if 45 <= pixel <= 130]
        return float(min(eligible)[1] if eligible else candidates[-1])

    def redraw(self) -> None:
        if not hasattr(self, "canvas"):
            return
        canvas = self.canvas
        width = max(canvas.winfo_width(), 1)
        height = max(canvas.winfo_height(), 1)
        center_x, center_y = width / 2.0, height / 2.0
        self.canvas_origin = (center_x, center_y)
        tracker, imu = self._snapshot()
        scale = self._mm_per_100px()
        pixels_per_mm = 100.0 / scale
        imu_x = center_x + (imu[0] - tracker[0]) * pixels_per_mm
        imu_y = center_y - (imu[1] - tracker[1]) * pixels_per_mm
        self.imu_canvas_point = (imu_x, imu_y)

        canvas.delete("all")
        grid_step = self._grid_step(scale)
        grid_pixels = grid_step * pixels_per_mm
        if grid_pixels > 0:
            count_x = int(width / grid_pixels) + 2
            count_y = int(height / grid_pixels) + 2
            for index in range(-count_x, count_x + 1):
                x = center_x + index * grid_pixels
                canvas.create_line(x, 0, x, height, fill="#dce5e3", width=1)
            for index in range(-count_y, count_y + 1):
                y = center_y + index * grid_pixels
                canvas.create_line(0, y, width, y, fill="#dce5e3", width=1)

        canvas.create_line(0, center_y, width, center_y, fill="#657579", width=2)
        canvas.create_line(center_x, 0, center_x, height, fill="#657579", width=2)
        canvas.create_text(12, 12, anchor="nw", text="XY 平面（以 Tracker 為相對座標原點）", fill="#243438", font=("Segoe UI", 11, "bold"))
        canvas.create_text(width - 12, 12, anchor="ne", text=f"比例 {scale:g} mm / 100 px", fill="#53636a", font=("Segoe UI", 9))

        for index in range(-count_x, count_x + 1):
            x = center_x + index * grid_pixels
            if 0 <= x <= width and index:
                canvas.create_text(x + 3, center_y + 3, anchor="nw", text=f"{index * grid_step:g}", fill="#879496", font=("Segoe UI", 8))
        for index in range(-count_y, count_y + 1):
            y = center_y - index * grid_pixels
            if 0 <= y <= height and index:
                canvas.create_text(center_x + 4, y + 2, anchor="nw", text=f"{index * grid_step:g}", fill="#879496", font=("Segoe UI", 8))

        canvas.create_line(center_x, center_y, imu_x, imu_y, fill="#497680", width=2)
        canvas.create_oval(center_x - 11, center_y - 11, center_x + 11, center_y + 11, fill="#0f766e", outline="white", width=2)
        canvas.create_line(center_x - 17, center_y, center_x + 17, center_y, fill="#0f766e", width=2)
        canvas.create_line(center_x, center_y - 17, center_x, center_y + 17, fill="#0f766e", width=2)
        canvas.create_text(center_x + 15, center_y + 13, anchor="nw", text=f"Tracker ({tracker[0]:g}, {tracker[1]:g})", fill="#075e59", font=("Segoe UI", 9, "bold"))

        imu_color = "#b42318" if math.hypot(imu[0] - tracker[0], imu[1] - tracker[1]) < 1.0 else "#d97706"
        canvas.create_oval(imu_x - 13, imu_y - 13, imu_x + 13, imu_y + 13, fill=imu_color, outline="white", width=2)
        if self.imu_yaw_deg is None:
            marker_label = "IMU (拖曳)"
        else:
            radians = math.radians(self.imu_yaw_deg)
            arrow_start = (imu_x + 11 * math.cos(radians), imu_y - 11 * math.sin(radians))
            arrow_end = heading_arrow_endpoint(imu_x, imu_y, self.imu_yaw_deg, 48.0)
            canvas.create_line(
                *arrow_start, *arrow_end, fill="#1d4ed8", width=3,
                arrow=tk.LAST, arrowshape=(12, 14, 5),
            )
            marker_label = f"IMU {self.imu_yaw_deg:+.1f}°"
        motor_heading_fresh = (
            self.motor_heading_deg is not None
            and self.motor_heading_received_at is not None
            and time.monotonic() - self.motor_heading_received_at <= 0.5
        )
        if motor_heading_fresh:
            motor_start_angle = math.radians(self.motor_heading_deg)
            motor_start = (
                imu_x + 11 * math.cos(motor_start_angle),
                imu_y - 11 * math.sin(motor_start_angle),
            )
            motor_end = heading_arrow_endpoint(imu_x, imu_y, self.motor_heading_deg, 64.0)
            canvas.create_line(
                *motor_start, *motor_end, fill="#15803d", width=3,
                arrow=tk.LAST, arrowshape=(12, 14, 5),
            )
            source_label = "編碼器推算" if self.motor_heading_source == "encoder" else "估算"
            marker_label += f" / 馬達 {self.motor_heading_deg:+.1f}° ({source_label})"
        elif self.motor_heading_deg is not None:
            marker_label += " / 馬達方向過期"
        else:
            marker_label += " / 馬達方向待回讀"
        canvas.create_text(imu_x + 16, imu_y - 18, anchor="nw", text=marker_label, fill="#7c3f08", font=("Segoe UI", 10, "bold"))

        measurement = make_measurement(*tracker, *imu)
        midpoint = ((center_x + imu_x) / 2.0, (center_y + imu_y) / 2.0)
        canvas.create_text(midpoint[0] + 5, midpoint[1] - 16, anchor="sw", text=format_distance_m(measurement["distance_mm"]), fill="#243438", font=("Segoe UI", 10, "bold"))
        self.position_var.set(f"相對 X/Y  {measurement['x_mm']:+.1f}, {measurement['y_mm']:+.1f} mm")
        self.distance_var.set(f"距離  {format_distance_m(measurement['distance_mm'])}")
        azimuth = measurement["azimuth_deg"]
        self.azimuth_var.set("方位  未定義（兩點重合）" if azimuth is None else f"方位  {azimuth:+.3f}°（+X 逆時針）")
        if self.imu_yaw_deg is not None:
            motor_heading_fresh = (
                self.motor_heading_deg is not None
                and self.motor_heading_received_at is not None
                and time.monotonic() - self.motor_heading_received_at <= 0.5
            )
            motor_text = (
                "方向過期"
                if self.motor_heading_deg is not None and not motor_heading_fresh
                else "等待回讀"
                if self.motor_heading_deg is None
                else f"{self.motor_heading_deg:+.2f}° ({'編碼器推算' if self.motor_heading_source == 'encoder' else '估算'})"
            )
            self.yaw_var.set(
                f"IMU 方向  {self.imu_yaw_deg:+.2f}°（藍）  |  馬達世界方向  {motor_text}（綠）"
            )

    def _begin_drag(self, event: tk.Event) -> None:
        x, y = self.imu_canvas_point
        if math.hypot(event.x - x, event.y - y) <= 24:
            self.dragging_imu = True
            self.canvas.grab_set()

    def _drag_imu(self, event: tk.Event) -> None:
        if not self.dragging_imu:
            return
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        px = min(max(event.x, 14), width - 14)
        py = min(max(event.y, 14), height - 14)
        center_x, center_y = self.canvas_origin
        scale = self._mm_per_100px()
        tracker, _ = self._snapshot()
        position = (
            tracker[0] + (px - center_x) * scale / 100.0,
            tracker[1] - (py - center_y) * scale / 100.0,
        )
        with self.position_lock:
            self.imu_position = position
        self.redraw()

    def _end_drag(self, _event: tk.Event) -> None:
        if self.dragging_imu:
            self.dragging_imu = False
            try:
                self.canvas.grab_release()
            except tk.TclError:
                pass

    def toggle_connection(self) -> None:
        """建立或中斷 Tracker 資料 TCP 連線；傳送工作交給背景執行緒。"""
        with self.connection_lock:
            existing = self.client_socket
        if existing is not None:
            self.disconnect()
            return

        try:
            host = self.host_var.get().strip()
            port = int(self.port_var.get())
            rate = float(self.rate_var.get())
            if not host or not 1 <= port <= 65535 or not math.isfinite(rate) or not 1 <= rate <= 100:
                raise ValueError
            client = socket.create_connection((host, port), timeout=1.5)
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except (OSError, ValueError):
            messagebox.showerror("TCP 連線失敗", "請確認主機、Port，以及 faro_yaw_follow.py 是否已啟動並監聽 TCP 5010。")
            return

        self.stop_event.clear()
        with self.connection_lock:
            self.client_socket = client
        self.connection_button.configure(text="中斷連線")
        self.status_var.set(f"已連線 {host}:{port}，以 {rate:g} Hz 傳送並記錄 CSV")
        self.sender_thread = threading.Thread(
            target=self._sender_loop,
            args=(client, rate),
            name="tracker-tcp-sender",
            daemon=True,
        )
        self.sender_thread.start()

    def _sender_loop(self, client: socket.socket, rate_hz: float) -> None:
        """依指定頻率擷取目前座標、傳送量測值，同步追加 CSV 紀錄。"""
        try:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            new_log = not LOG_PATH.exists() or LOG_PATH.stat().st_size == 0
            with LOG_PATH.open("a", newline="", encoding="utf-8") as log_file:
                writer = csv.DictWriter(log_file, fieldnames=LOG_FIELDS)
                if new_log:
                    writer.writeheader()
                    log_file.flush()
                while not self.stop_event.wait(1.0 / rate_hz):
                    tracker, imu = self._snapshot()
                    measurement = make_measurement(*tracker, *imu)
                    client.sendall(encode_measurement_line(measurement))
                    writer.writerow({
                        "timestamp_ns": measurement["timestamp_ns"],
                        "tracker_x_mm": tracker[0],
                        "tracker_y_mm": tracker[1],
                        "imu_x_mm": imu[0],
                        "imu_y_mm": imu[1],
                        **{key: measurement[key] for key in (
                            "x_mm", "y_mm", "distance_mm", "azimuth_deg", "zenith_deg", "valid"
                        )},
                    })
                    log_file.flush()
                    self.ui_messages.put(("sent", f"傳送中：距離 {format_distance_m(measurement['distance_mm'])}"))
        except (OSError, ValueError) as error:
            if not self.stop_event.is_set():
                self.ui_messages.put(("error", f"TCP 傳輸中斷：{error}"))
        finally:
            try:
                client.close()
            except OSError:
                pass
            with self.connection_lock:
                if self.client_socket is client:
                    self.client_socket = None
            if not self.stop_event.is_set():
                self.ui_messages.put(("disconnected", "連線已中斷；可重新連線"))

    def disconnect(self) -> None:
        self.stop_event.set()
        with self.connection_lock:
            client, self.client_socket = self.client_socket, None
        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                client.close()
            except OSError:
                pass
        self.connection_button.configure(text="連線並開始傳送")
        self.status_var.set("已中斷連線")

    def _poll_sender_messages(self) -> None:
        now = time.monotonic()
        motor_heading_was_fresh = (
            self.motor_heading_deg is not None
            and self.motor_heading_received_at is not None
            and now - self.motor_heading_received_at <= 0.5
        )
        yaw_changed = False
        while True:
            try:
                sample = self.yaw_messages.get_nowait()
                self.imu_yaw_deg = float(sample["yaw_deg"])
                motor_heading = sample.get("motor_heading_deg")
                self.motor_heading_deg = (
                    float(motor_heading) if isinstance(motor_heading, (float, int)) else None
                )
                self.motor_heading_source = str(sample.get("motor_heading_source", "encoder"))
                try:
                    self.motor_heading_received_at = float(sample["local_monotonic_s"])
                except (KeyError, TypeError, ValueError):
                    self.motor_heading_received_at = now
                yaw_changed = True
            except queue.Empty:
                break
        while True:
            try:
                kind, message = self.ui_messages.get_nowait()
            except queue.Empty:
                break
            self.status_var.set(message)
            if kind in ("error", "disconnected"):
                self.connection_button.configure(text="連線並開始傳送")
        motor_heading_is_fresh = (
            self.motor_heading_deg is not None
            and self.motor_heading_received_at is not None
            and time.monotonic() - self.motor_heading_received_at <= 0.5
        )
        if yaw_changed or motor_heading_was_fresh != motor_heading_is_fresh:
            self.redraw()
        self.root.after(100, self._poll_sender_messages)

    def close(self) -> None:
        self.yaw_server_stop.set()
        if self.yaw_server_socket is not None:
            self.yaw_server_socket.close()
        if self.yaw_receiver_thread is not None:
            self.yaw_receiver_thread.join(timeout=0.5)
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    TrackerSimulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
