# ESP32 Firmware

這個資料夾放 ESP32 / Arduino 相關韌體。

## 檔案

| 檔案 | 用途 |
| --- | --- |
| `mf5015v2_sample/` | XIAO ESP32-S3 + WCMCU230 與 MF4015-V2 / MF5015-V2 CAN 通訊測試 |
| `wcmcu_test/` | ESP32 TWAI/CAN 自我回送測試，不需連馬達 |
| `can_basic/` | 參考 ESP32 CAN demo 改寫的最小 TWAI 通訊範例，只讀取馬達狀態 |

## 使用方式

## ESP32 + WCMCU230 CAN 測試

### 最小通訊測試

先開啟：

```text
esp32/can_basic/can_basic.ino
```

這份程式使用 ESP32 Arduino Core 內建的 `driver/twai.h`，不需要另外安裝
`ESP32CAN` 函式庫。Serial Monitor 設為 `115200`，輸入 `r` 後只會送出
`CAN ID 0x141`、命令 `0x9A` 與 `0x9C` 的讀取狀態封包，不會命令馬達轉動。

參考程式：

- [nhatuan84/esp32-can-protocol-demo](https://github.com/nhatuan84/esp32-can-protocol-demo)

如果只是要先確認 ESP32 TWAI 程式能執行，先開啟：

```text
esp32/wcmcu_test/wcmcu_test.ino
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
esp32/mf5015v2_sample/mf5015v2_sample.ino
```

Arduino IDE 的 `USB CDC On Boot` 必須設為 `Enabled`，Serial Monitor 設為 `115200` baud。

程式啟動後不會自行送出 CAN 指令：

| 指令 | 功能 |
| --- | --- |
| `r` | 讀取馬達狀態；收到有效回覆後才會解鎖低速測試 |
| `m` | 以 30 dps 低速轉動 0.5 秒，接著自動停止 |
| `s` | 立即送出停止指令 |
