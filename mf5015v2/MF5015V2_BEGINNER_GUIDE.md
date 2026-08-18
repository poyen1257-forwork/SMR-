# MF5015-V2 馬達入門筆記

這份筆記用資工角度整理：你可以先把 MF5015-V2 想成「內建驅動器、編碼器、控制韌體的伺服馬達節點」。Python 程式不是直接切換三相線圈，而是透過 CAN 或 RS485 傳封包給馬達內部驅動器。

## 1. 先確認你買的是 CAN 還是 RS485

MF5015-V2 常見有兩種通訊版本：

- CAN 版：接 `CAN_H`、`CAN_L`，Python 端通常用 USB-CAN 轉接器與 `python-can`。
- RS485 版：接 `A/B` 差動線，Python 端通常用 USB-RS485 轉接器與 `pyserial`。

兩者不能只靠軟體互換。CAN 版要用 CAN transceiver，RS485 版要用 RS485 transceiver。

## 2. 重要規格摘要

不同商店資料會有 10T / 35T、CAN / RS485 版本差異，接線前以你手上型號標籤和賣家手冊為準。

| 項目 | 常見規格 |
| --- | --- |
| 額定電壓 | 16 V |
| 驅動輸入電壓 | 7.4-32 V |
| 通訊 | CAN 或 RS485 |
| CAN baudrate | 常見 1 Mbps，部分資料列出 100K/125K/250K/500K/1M |
| RS485 baudrate | 常見 115200 bps，也可能支援更高 |
| 編碼器 | 18-bit magnetic encoder |
| 控制模式 | torque loop、speed loop、position loop |
| 馬達重量 | 約 175 g |

10T 常見資料：額定扭矩約 0.38 N.m、額定電流約 4.9 A、最高轉速約 2040 RPM。
35T 常見資料：額定扭矩約 0.32 N.m、額定電流約 1.22 A、最高轉速約 580 RPM。

## 3. 你需要準備的硬體

CAN 版本：

- MF5015-V2 CAN 版馬達
- DC 電源供應器，建議可限流，第一次測試先設低一點
- USB-CAN 轉接器
- 兩條 CAN 訊號線：`CAN_H`、`CAN_L`
- 電源線：`V+`、`GND`
- 120 ohm 終端電阻：CAN bus 兩端各一顆；短線單馬達測試常見是轉接器端或馬達端已有一顆，但要確認

RS485 版本：

- USB-RS485 轉接器
- `A/B` 差動訊號線
- 電源線：`V+`、`GND`

## 4. 資工版電機概念速成

電壓 V：像「電位差」。MF5015-V2 的馬達驅動輸入常見是 7.4-32 V，但額定工作點常見 16 V。不要把 5 V Arduino 腳位或 USB 直接當馬達電源。

電流 A：像「實際流過去的量」。馬達卡住、快速加速、負載變大時電流會上升。第一次測試一定用可限流電源，程式也先用小電流。

功率 W：約等於 `V * A`。例如 16 V、5 A 就是 80 W 等級，已經不是普通 GPIO 可以碰的世界。

扭矩 N.m：旋轉的力。0.38 N.m 表示在 1 m 力臂上約 0.38 N 的力；力臂越短可承受的力越大。

轉速 RPM / dps：RPM 是每分鐘幾圈；dps 是每秒幾度。換算：`1 RPM = 6 dps`。

編碼器：馬達內部的位置感測器。18-bit 表示一圈理論上可分成 `2^18 = 262144` 個刻度。

## 5. CAN 通訊怎麼看

CAN bus 不是 UART 那種 TX/RX 一對一，而是多節點匯流排：

- 訊號線是 `CAN_H` 和 `CAN_L`，用差動電壓抗雜訊。
- 每個封包有 arbitration ID，像 topic 或地址。
- MF/RMD 類協定常用 standard frame、8 bytes data。
- 常見單馬達命令 ID 是 `0x140 + motor_id`。
- 常見回覆 ID 是 `0x240 + motor_id`，有些相容版本可能沿用送出 ID 回覆，所以範例程式兩者都接受。

CAN frame 概念：

```text
arbitration_id = 0x141         # motor_id = 1
data = A1 00 00 00 32 00 00 00
       ^^          ^^^^^
       command     little-endian 參數
```

資料欄位通常是 little-endian，這跟 x86 記憶體很像：低位元組先傳。

## 6. 常用命令

以下是 MF/RMD 類 CAN 協定常見命令，實際仍以你的版本手冊為準。

| 命令 | byte | 用途 |
| --- | --- | --- |
| Motor off | `0x80` | 關閉馬達輸出 |
| Motor stop | `0x81` | 停止運動 |
| Motor running | `0x88` | 恢復運行狀態 |
| Read status 1 | `0x9A` | 溫度、電壓、錯誤旗標等 |
| Clear error | `0x9B` | 清除錯誤旗標 |
| Read status 2 | `0x9C` | 溫度、電流、速度、角度/編碼器 |
| Read status 3 | `0x9D` | 三相電流 |
| Torque closed-loop | `0xA1` | 電流/扭矩控制 |
| Speed closed-loop | `0xA2` | 速度控制 |
| Multi-turn position | `0xA3` | 多圈位置控制 |
| Multi-turn position + speed | `0xA4` | 位置控制並限制速度 |

## 7. Python 程式使用方式

安裝套件：

```powershell
python -m pip install python-can
```

先查狀態：

```powershell
python mf5015v2_can.py --interface slcan --channel COM3 --bitrate 1000000 --motor-id 1 status
```

如果你用的是 SocketCAN，例如 Linux/Raspberry Pi：

```bash
python mf5015v2_can.py --interface socketcan --channel can0 --bitrate 1000000 --motor-id 1 status
```

保守測試速度，每秒 30 度，跑 1 秒後自動 stop：

```powershell
python mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 speed --dps 30 --duration 1
```

保守測試電流，0.1 A，0.2 秒後自動 stop：

```powershell
python mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 torque --amps 0.1 --duration 0.2
```

移到多圈角度 90 度，最大速度 30 dps：

```powershell
python mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 position --degrees 90 --max-speed-dps 30
```

停止：

```powershell
python mf5015v2_can.py --interface slcan --channel COM3 --motor-id 1 stop
```

## 8. 第一次上電檢查清單

- 馬達固定好，輸出端不要接重負載。
- 電源正負不要接反。
- 電源供應器先限流，例如 0.5-1 A 起步。
- CAN_H 對 CAN_H，CAN_L 對 CAN_L。
- CAN baudrate 先試 1 Mbps。
- motor_id 先試 1。
- 先跑 `status`，能讀到資料再下運動命令。
- 第一次只下很小的速度或電流。

## 9. 常見錯誤

收不到回覆：

- CAN_H/CAN_L 接反。
- baudrate 不一致。
- motor_id 不是 1。
- USB-CAN 轉接器的 python-can interface 名稱不對。
- 沒有供應馬達主電源。
- CAN bus 沒有適當終端電阻。

馬達會抖或叫：

- 負載太大。
- PID 或控制參數不適合。
- 速度/電流命令太激進。
- 電源限流太低造成驅動器重啟。

電腦能通訊但馬達不動：

- 可能處於 stop/off/error 狀態，可先 `clear-error` 再 `run`。
- 速度或電流命令太小。
- 驅動器保護中，例如低電壓或過溫。

## 10. 資料來源

- MYACTUATOR downloads pages list RMD-L/RMD-X manuals and Motor Motion Protocol documents.
- SMC Powers MF5015-V2 product pages describe MF V2 CAN/RS485 driver versions and 18-bit encoder upgrade.
- AIFITLAB / LKMTECH MF5015-V2 product information lists voltage, current, CAN/RS485, encoder, and control-mode specifications.
