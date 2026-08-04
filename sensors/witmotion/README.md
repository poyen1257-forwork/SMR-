# WitMotion Sensors

這個資料夾放 WT901 / BWT901CL 姿態感測器相關 Python 程式。

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

## 常用指令

```powershell
python BWT901CL_reader.py list
python BWT901CL_reader.py read --port COM4
python BWT901CL_visualizer_COM.py --port COM4
python BWT901CL_visualizer_WIFI.py --mode udp --wifi-port 1399
python WT901WIFI_reader.py serial --list
```

如果從 repository 根目錄執行，請加上路徑：

```powershell
python sensors/witmotion/BWT901CL_reader.py list
```
