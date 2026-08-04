import argparse
import tkinter as tk

from BWT901CL_visualizer import VisualizerApp


def main():
    parser = argparse.ArgumentParser(description="BWT901CL COM 視覺化接收程式")
    parser.add_argument("--port", default="COM4", help="COM port，預設 COM4")
    parser.add_argument("--baud", type=int, default=9600, help="鮑率，預設 9600")
    parser.add_argument("--hz", type=float, default=60, help="畫面更新頻率，預設 60Hz")
    parser.add_argument("--sensor-rate-hz", type=float, default=100, help="開啟 COM 後嘗試設定感測器輸出率，預設 100Hz")
    parser.add_argument("--algorithm", choices=["axis6", "axis9", "none"], default="axis6", help="感測器演算法，預設 axis6")
    parser.add_argument("--display-mode", choices=["current", "initial-relative"], default="current", help="3D顯示模式，預設 current")
    parser.add_argument("--stationary-lock", action="store_true", help="開啟靜止鎖定")
    parser.add_argument("--no-stationary-lock", action="store_true", help="關閉靜止鎖定，直接顯示原始漂移")
    parser.add_argument("--stationary-gyro-threshold", type=float, default=1.0, help="靜止判斷角速度門檻，預設 1.0 deg/s")
    parser.add_argument("--angle-fields", choices=["f09", "f06"], default="f09", help="角度欄位來源，預設 f09")
    parser.add_argument(
        "--pose-source",
        choices=["relative-device", "gravity-mag", "device"],
        default="device",
        help="姿態來源，預設 device",
    )
    parser.add_argument("--demo", action="store_true", help="不接硬體，使用模擬資料")
    parser.add_argument("--file", help="播放 WitMotion 官方 APP 匯出的 txt 檔")
    args = parser.parse_args()

    root = tk.Tk()
    VisualizerApp(
        root,
        port=args.port,
        baud=args.baud,
        demo=args.demo,
        hz=args.hz,
        file_path=args.file,
        source="serial",
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
