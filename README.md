# SMR Learning Workspace

這個 repository 用來整理 SMR 學習與實作程式，包含 WitMotion 姿態感測器、MF5015-V2 馬達控制、ESP32 韌體與相關筆記。

## Repository 分類

| 目錄 | 內容 |
| --- | --- |
| `motors/mf5015v2/` | MF5015-V2 馬達 CAN 控制程式與入門筆記 |
| `sensors/witmotion/` | WT901 / BWT901CL 感測器讀取、Wi-Fi 測試、視覺化工具 |
| `firmware/esp32/` | ESP32 Arduino 韌體 |
| `docs/` | 共用文件、資料格式、硬體設定筆記 |

## 快速開始

### WT901WIFI Wi-Fi 資料測試

先查電腦 Wi-Fi IP：

```powershell
ipconfig
```

找到 Wi-Fi adapter 底下的 `IPv4 Address`，例如：

```text
10.5.98.50
```

到 WitMotion 官方設定軟體，把 WT901WIFI 設成：

```text
Network mode: Station Mode
Protocol: UDP
Server mode: Specify user server
Wi-Fi SSID: 你的 2.4GHz Wi-Fi 名稱
Wi-Fi password: 你的 Wi-Fi 密碼
User server IP: 你的電腦 IPv4，例如 10.5.98.50
User server port: 1399
```

設定完成後儲存，必要時重開 WT901WIFI。

在電腦端接收 UDP 原始資料：

```powershell
python sensors/witmotion/WT901WIFI_reader.py udp --port 1399 --raw-packet
```

如果要印出解析後的 ACC / GYRO / ANGLE：

```powershell
python sensors/witmotion/WT901WIFI_reader.py udp --port 1399 --raw-packet --print-each
```

確認資料正常後，開啟 Wi-Fi 版視覺化：

```powershell
python sensors/witmotion/BWT901CL_visualizer_WIFI.py --mode udp --wifi-port 1399
```

更多細節請看：

- [WitMotion sensors guide](sensors/witmotion/README.md)
- [WitMotion record format](docs/WITMOTION_RECORD_FORMAT.md)
- [WT901WIFI ESP32 notes](docs/README_WT901WIFI_ESP32.md)

### MF5015-V2 馬達

```powershell
python motors/mf5015v2/mf5015v2_can.py --help
python motors/mf5015v2/mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 status
```

教學文件：

- [MF5015-V2 beginner guide](motors/mf5015v2/MF5015V2_BEGINNER_GUIDE.md)

## 開發流程

```powershell
git status
git add .
git commit -m "Describe your change"
git push
```

每次新增功能時，建議先放到對應分類目錄，再更新 README。
