# MF5015-V2 Motor Control

這個資料夾放 MF5015-V2 馬達控制相關內容。

## 檔案

| 檔案 | 用途 |
| --- | --- |
| `mf5015v2_can.py` | 使用 `python-can` 控制 MF5015-V2 CAN 版馬達 |
| `MF5015V2_BEGINNER_GUIDE.md` | 馬達電壓、電流、CAN/RS485、封包格式入門筆記 |

## 常用指令

```powershell
python mf5015v2_can.py --help
python mf5015v2_can.py --interface slcan --channel COM3 --bitrate 1000000 --motor-id 1 status
python mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 speed --dps 30 --duration 1
python mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 stop
```

第一次測試請先使用可限流電源，先讀 `status`，確認通訊成功後再下運動命令。
