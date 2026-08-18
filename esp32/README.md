# ESP32 Firmware

這個資料夾放 ESP32 / Arduino 相關韌體。

## 檔案

| 檔案 | 用途 |
| --- | --- |
| `ESP32_WCMCU230_CAN_TEST/` | XIAO ESP32-S3 + WCMCU230 與 MF4015-V2 / MF5015-V2 CAN 通訊測試 |
| `ESP32_WCMCU230_LOOPBACK_TEST/` | ESP32 TWAI/CAN 自我回送測試，不需連馬達 |
| `esp32can_basic/` | 使用第三方 ESP32CAN 函式庫的舊版基礎範例 |

## 使用方式

## ESP32 + WCMCU230 CAN 測試

如果只是要先確認 ESP32 TWAI 程式能執行，先開啟：

```text
esp32/ESP32_WCMCU230_LOOPBACK_TEST/ESP32_WCMCU230_LOOPBACK_TEST.ino
```

預設腳位：

```text
XIAO D6 / GPIO43 -> WCMCU230 CTX / TXD
XIAO D7 / GPIO44 -> WCMCU230 CRX / RXD
XIAO 3V3        -> WCMCU230 3V3
XIAO GND        -> WCMCU230 GND
```

Serial Monitor 設 `115200` baud。看到 `LOOPBACK OK` 代表 ESP32 TWAI/CAN driver 與程式環境正常。這個測試不需要接馬達。

接線預設：

```text
ESP32 3V3   -> WCMCU230 VCC
ESP32 GND   -> WCMCU230 GND -> 馬達 GND
ESP32 D6 / GPIO43 -> WCMCU230 CTX / TXD
ESP32 D7 / GPIO44 -> WCMCU230 CRX / RXD
WCMCU230 CAN_H -> 馬達 CAN_H
WCMCU230 CAN_L -> 馬達 CAN_L
16V 電源 + -> 馬達 V+
16V 電源 - -> 馬達 GND
```

用 Arduino IDE 開啟：

```text
esp32/ESP32_WCMCU230_CAN_TEST/ESP32_WCMCU230_CAN_TEST.ino
```

Arduino IDE 的 `USB CDC On Boot` 必須設為 `Enabled`，Serial Monitor 設為 `115200` baud。

程式啟動後不會自行送出 CAN 指令：

| 指令 | 功能 |
| --- | --- |
| `r` | 讀取馬達狀態；收到有效回覆後才會解鎖低速測試 |
| `m` | 以 30 dps 低速轉動 0.5 秒，接著自動停止 |
| `s` | 立即送出停止指令 |
