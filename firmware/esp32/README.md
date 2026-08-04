# ESP32 Firmware

這個資料夾放 ESP32 / Arduino 相關韌體。

## 檔案

| 目錄 | 用途 |
| --- | --- |
| `WT901WIFI_ESP32_UDP/` | ESP32 UDP 接收 / 轉發 WT901WIFI 資料的 Arduino sketch |
| `ESP32_WCMCU230_CAN_TEST/` | ESP32 + WCMCU230/SN65HVD230 CAN 測試 MF5015-V2 的 Arduino sketch |
| `ESP32_WCMCU230_LOOPBACK_TEST/` | ESP32 TWAI/CAN 自我回送測試，不需連馬達 |

## 使用方式

用 Arduino IDE 或相容工具開啟：

```text
firmware/esp32/WT901WIFI_ESP32_UDP/WT901WIFI_ESP32_UDP.ino
```

詳細設定請看：

- [WT901WIFI ESP32 notes](../../docs/README_WT901WIFI_ESP32.md)

## ESP32 + WCMCU230 CAN 測試

如果只是要先確認 ESP32 與 CAN 程式能跑，先開這個：

```text
firmware/esp32/ESP32_WCMCU230_LOOPBACK_TEST/ESP32_WCMCU230_LOOPBACK_TEST.ino
```

預設腳位：

```text
XIAO D4 / GPIO5 -> WCMCU230 CTX / TXD
XIAO D5 / GPIO6 -> WCMCU230 CRX / RXD
XIAO 3V3        -> WCMCU230 3V3
XIAO GND        -> WCMCU230 GND
```

Serial Monitor 設 `115200` baud。看到 `LOOPBACK OK` 代表 ESP32 TWAI/CAN driver 與程式環境正常。這個測試不需要接馬達。

接線預設：

```text
ESP32 3V3   -> WCMCU230 VCC
ESP32 GND   -> WCMCU230 GND -> 馬達 GND
ESP32 GPIO5 -> WCMCU230 CTX / TXD
ESP32 GPIO4 -> WCMCU230 CRX / RXD
WCMCU230 CAN_H -> 馬達 CAN_H
WCMCU230 CAN_L -> 馬達 CAN_L
16V 電源 + -> 馬達 V+
16V 電源 - -> 馬達 GND
```

用 Arduino IDE 開啟：

```text
firmware/esp32/ESP32_WCMCU230_CAN_TEST/ESP32_WCMCU230_CAN_TEST.ino
```

Serial Monitor 設 `115200` baud。程式預設每秒讀一次馬達狀態，不會讓馬達轉。輸入 `h` 可看指令，輸入 `m` 才會做 30 dps、1 秒的低速測試。
