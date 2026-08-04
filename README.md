# SMR Learning Workspace

這個 repository 用來整理 SMR 學習與實作程式，包含 WitMotion 姿態感測器、MF5015-V2 馬達控制、ESP32 韌體與相關筆記。

## Repository 分類

| 目錄 | 內容 |
| --- | --- |
| `motors/mf5015v2/` | MF5015-V2 馬達 CAN 控制程式與入門筆記 |
| `sensors/witmotion/` | WT901 / BWT901CL 感測器讀取、視覺化、Wi-Fi/COM 工具 |
| `firmware/esp32/` | ESP32 Arduino 韌體 |
| `docs/` | 共用文件、資料格式、硬體設定筆記 |

## 快速開始

### MF5015-V2 馬達

```powershell
python motors/mf5015v2/mf5015v2_can.py --help
python motors/mf5015v2/mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 status
```

教學文件：

- [MF5015-V2 beginner guide](motors/mf5015v2/MF5015V2_BEGINNER_GUIDE.md)

### WitMotion 感測器

```powershell
python sensors/witmotion/BWT901CL_reader.py list
python sensors/witmotion/BWT901CL_reader.py read --port COM4
python sensors/witmotion/BWT901CL_visualizer_COM.py --port COM4
```

相關文件：

- [WitMotion record format](docs/WITMOTION_RECORD_FORMAT.md)
- [WT901WIFI ESP32 notes](docs/README_WT901WIFI_ESP32.md)

## 開發流程

```powershell
git status
git add .
git commit -m "Describe your change"
git push
```

每次新增功能時，建議先放到對應分類目錄，再更新 README。
