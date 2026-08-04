# WitMotion Sensors

這個資料夾放 WT901 / BWT901CL / WT901WIFI 姿態感測器相關 Python 程式。

## 檔案

| 檔案 | 用途 |
| --- | --- |
| `BWT901CL_reader.py` | 讀取與診斷 BWT901CL 資料 |
| `BWT901CL_visualizer.py` | BWT901CL 主要視覺化程式 |
| `BWT901CL_visualizer_COM.py` | 使用 COM port 開啟視覺化 |
| `BWT901CL_visualizer_WIFI.py` | 使用 Wi-Fi UDP/TCP 開啟視覺化 |
| `BWT901CL_wifi_config.py` | BWT901CL Wi-Fi 設定工具 |
| `BWT901CL_rate_test.py` | 感測器輸出率設定與測試 |
| `WT901WIFI_reader.py` | WT901WIFI USB serial / UDP 讀取工具 |
| `WT901WIFI_USB_SERIAL_Python/` | WT901WIFI USB serial 範例 |

## WT901WIFI Wi-Fi 版操作流程

### 1. 確認電腦 Wi-Fi IP

在 PowerShell 執行：

```powershell
ipconfig
```

找到 Wi-Fi adapter 底下的 `IPv4 Address`，例如：

```text
10.5.98.50
```

後面 WitMotion 設定軟體的 `User server IP` 就填這個 IP。

### 2. 設定 WT901WIFI

用 WitMotion 官方設定軟體設定感測器：

```text
Network mode: Station Mode
Protocol: UDP
Server mode: Specify user server
Wi-Fi SSID: 你的 2.4GHz Wi-Fi 名稱
Wi-Fi password: 你的 Wi-Fi 密碼
User server IP: 你的電腦 IPv4，例如 10.5.98.50
User server port: 1399
```

注意：

- WT901WIFI 通常只支援 2.4GHz Wi-Fi。
- 電腦與 WT901WIFI 要在同一個 Wi-Fi 網段。
- 設定儲存後，如果沒有馬上送資料，請重開 WT901WIFI 電源。

### 3. 先測 UDP 是否收到資料

從 repository 根目錄執行：

```powershell
python sensors/witmotion/WT901WIFI_reader.py udp --port 1399 --raw-packet
```

成功時會看到類似：

```text
Listening UDP on 0.0.0.0:1399
UDP #1 from 10.5.98.xx:xxxxx, size=...
```

看到 `UDP #...` 代表 Wi-Fi 封包已經進到電腦。

### 4. 印出解析後資料

```powershell
python sensors/witmotion/WT901WIFI_reader.py udp --port 1399 --raw-packet --print-each
```

會看到類似：

```text
ACC ...
GYRO ...
ANGLE ...
```

常見資料類型：

| Frame | 內容 | 單位 |
| --- | --- | --- |
| `0x55 0x51` | 加速度 ACC | g |
| `0x55 0x52` | 角速度 GYRO | deg/s |
| `0x55 0x53` | 歐拉角 ANGLE | degree |
| `0x55 0x54` | 磁場 MAG | raw |

### 5. 開啟 Wi-Fi 視覺化

確認 UDP 資料正常後執行：

```powershell
python sensors/witmotion/BWT901CL_visualizer_WIFI.py --mode udp --wifi-port 1399
```

如果想使用原始漂移顯示，不啟用靜止鎖定：

```powershell
python sensors/witmotion/BWT901CL_visualizer_WIFI.py --mode udp --wifi-port 1399 --no-stationary-lock
```

### 6. 常見問題

收不到 `UDP #...`：

- 檢查 `User server IP` 是否填電腦 Wi-Fi IPv4。
- 檢查 port 是否都是 `1399`。
- 確認 WT901WIFI 與電腦在同一個 2.4GHz Wi-Fi。
- 關閉或允許 Windows 防火牆中的 Python UDP 存取。
- 感測器設定後重開電源。

有 UDP 但沒有 ACC / GYRO / ANGLE：

- 先加 `--raw-packet` 看原始封包。
- 確認感測器輸出格式是 WitMotion 標準 11-byte frame。
- 檢查 baud/data output 設定是否被改成非標準格式。

## COM 版常用指令

```powershell
python sensors/witmotion/BWT901CL_reader.py list
python sensors/witmotion/BWT901CL_reader.py read --port COM4
python sensors/witmotion/BWT901CL_visualizer_COM.py --port COM4
python sensors/witmotion/WT901WIFI_reader.py serial --list
```

如果已經進到 `sensors/witmotion/` 目錄，可以省略前面的路徑：

```powershell
python BWT901CL_reader.py list
```
