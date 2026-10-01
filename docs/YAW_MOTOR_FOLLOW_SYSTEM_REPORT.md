# 姿態角跟隨馬達系統技術報告

## 1. 專案目的

本系統以 Xsens MTi-630R 量測物件的航向角（yaw），控制 MF4015V2／MF5015V2 CAN 馬達做相同角度的相對轉動。系統採用位置控制：感測器轉動多少角度，馬達就接收相同的相對位置目標；馬達到達目標後不會維持在連續速度模式。

本報告以感測器、控制程式與馬達控制器部署在同一系統的架構說明。Python 負責感測器資料與控制決策；ESP32 負責即時 CAN 封包處理並與馬達驅動器通訊。

## 2. 系統架構

```text
Xsens MTi-630R
  |  官方 XDA API，100 Hz yaw 資料
  v
Python 感測器讀取程式
  |  本機 TCP，換行分隔 JSON，port 5006
  v
Python yaw-馬達橋接程式
  |  USB Serial，115200 bps
  v
XIAO ESP32-S3
  |  UART 邏輯：D6/GPIO43 -> CTX，D7/GPIO44 <- CRX
  v
WCMCU230 / SN65HVD230 CAN 收發器
  |  CAN 2.0A，1 Mbps
  v
MF4015V2 / MF5015V2 整合式馬達驅動器
```

馬達電源與 ESP32 USB 電源相互獨立；CAN 收發器與馬達訊號地必須共地。CAN_H 對接 CAN_H、CAN_L 對接 CAN_L，CAN 匯流排兩端各需一個 120 ohm 終端電阻。

## 3. 硬體與通訊參數

| 項目 | 目前設定 |
| --- | --- |
| 姿態感測器 | Xsens MTi-630R |
| 控制板 | Seeed Studio XIAO ESP32-S3 |
| CAN 收發器 | WCMCU230 / SN65HVD230 |
| 馬達 | MF4015V2；控制協定與本系統使用的 MF5015V2 指令相容 |
| CAN 鮑率 | 1 Mbps |
| CAN 格式 | 標準 11-bit 資料框，8-byte payload |
| 馬達命令 ID | Driver ID 1 時為 `0x141` |
| ESP32 CAN TX | D6 / GPIO43 接 WCMCU230 CTX/TXD |
| ESP32 CAN RX | D7 / GPIO44 接 WCMCU230 CRX/RXD |
| ESP32 控制序列埠 | USB Serial，115200 bps |
| yaw 量測與傳輸頻率 | 100 Hz |
| 馬達目標更新週期 | 10 ms，目標 100 Hz |
| 位置控制速度上限 | 1 至 720 dps |

## 4. 軟體模組

### 4.1 Xsens 感測器讀取程式

原始碼：`python/xsens/mti630r_reader.py`

程式使用 Xsens 官方 `xsensdeviceapi` XDA Python API，自動掃描 MTi 裝置、設定輸出資料，並將四元數換算為 roll、pitch、yaw（單位：degree）。

馬達跟隨時使用 `--yaw-only` 模式，只向感測器要求姿態資料，不傳送加速度、角速度與磁場資料。這可降低 115200 bps 感測器串列通訊負載，使 yaw 控制資料維持約 100 Hz。

控制資料以換行分隔 JSON 傳送：

```json
{"time_s":1730000000.0,"roll":0.15,"pitch":-0.08,"yaw":43.27}
```

若要做感測器驗證或資料分析，程式仍保留完整 9 軸資料輸出與 CSV 紀錄功能。

### 4.2 yaw-馬達橋接程式

原始碼：`python/xsens2esp32.py`

橋接程式接收 yaw JSON 資料，經 USB Serial 對 ESP32 傳送文字命令。第一筆 yaw 只作為初始參考點，不會使馬達轉動。後續控制流程如下：

1. 將 yaw 差值包回 `-180` 至 `+180` degree 範圍，處理航向角跨越正負 180 degree 的情況。
2. 累積角度變化，超過 1 degree deadband 才下達新目標，避免感測器雜訊造成頻繁動作。
3. 以 `yaw 角度變化 / 經過時間` 計算感測器角速度。
4. 角速度乘上可調增益，預設為 `1.2`，再限制在 `10..720 dps`。
5. 向 ESP32 傳送 `n <相對角度> <最高速度>` 位置命令。

橋接程式最快每 10 ms 傳送一次命令。它每 0.5 秒輸出一次摘要，而不是逐筆列印 100 Hz 指令，避免終端輸出成為延遲來源。

程式必須帶 `--arm` 才會開啟 ESP32 序列埠並實際傳送馬達命令；未帶此選項時是 dry-run 診斷模式。

### 4.3 ESP32 CAN 韌體

原始碼：`esp32/mf5015v2_sample/mf5015v2_sample.ino`

韌體負責兩件事：

1. 解析 Python 或序列監控器輸入的命令。
2. 將命令編碼為 MF 馬達驅動器所需的 CAN 封包。

韌體將 D6/D7 設定為 TWAI/CAN，使用 1 Mbps 與正常自動重送模式。ESP32 開機只初始化序列埠與 CAN，不會自動下達馬達動作指令。

在 100 Hz 的位置跟隨模式中，ESP32 將位置 CAN 封包放入傳送佇列後立即處理下一筆序列命令，不等待每一筆馬達回覆。這是刻意設計：若每 10 ms 都等待 RX 回覆，單一逾時就可能阻塞序列處理並造成位置目標過期。手動狀態讀取與初始化命令仍採完整 TX/RX 驗證。

## 5. 馬達命令與 CAN 協定

### ESP32 序列命令

| 命令 | 功能 |
| --- | --- |
| `r` | 讀取馬達狀態 1 |
| `m <速度>` | 連續速度模式，範圍 `-720..720 dps` |
| `n <角度> <速度>` | 相對位置移動；角度單位為 degree，速度為最高 dps |
| `s` | 停止馬達；速度模式使用緩停，位置模式會送停止命令 |

正常 yaw 跟隨只使用 `n` 命令。

### 主要 CAN 封包

| 功能 | CAN byte 0 | payload 說明 |
| --- | --- | --- |
| 讀取多圈位置 | `0x92` | 回覆 bytes 4-7：signed 32-bit、little-endian、`0.01 degree/LSB` |
| 讀取狀態 1 | `0x9A` | 驅動器溫度、匯流排電壓與故障旗標 |
| 速度控制 | `0xA2` | bytes 4-7：signed 32-bit、little-endian、`0.01 dps/LSB` |
| 帶速度限制的位置控制 | `0xA4` | bytes 2-3：最高速度 dps；bytes 4-7：signed 多圈絕對目標、`0.01 degree/LSB` |
| 停止 | `0x81` | 停止命令 |

ESP32 收到相對 `n` 命令時，第一次會先以 `0x92` 讀取馬達當前多圈角度，建立內部絕對目標；之後把每筆相對角度累加，並以 `0xA4` 傳送新的絕對目標。

此作法可避免軟體啟動時強迫馬達回到任意的絕對零點。

## 6. 控制行為

```text
感測器 yaw 改變 +12.50 degree
        |
        v
Python 傳送：n 12.50 <依 yaw 速度計算的最高速度>
        |
        v
ESP32 目標 = 前一個馬達目標 + 12.50 degree
        |
        v
CAN 0xA4 命令馬達移動至新的多圈位置
```

當感測器停止轉動時，橋接程式不再建立新的位置目標。馬達完成最後一個有限位置目標後，依驅動器設定維持或停止，不會留在連續速度模式。

## 7. 安全與可靠性設計

| 項目 | 實作方式 |
| --- | --- |
| 開機不自動動作 | ESP32 開機只初始化 CAN 與序列埠 |
| 軟體解鎖 | Python 必須使用 `--arm` 才能實際控制馬達 |
| 雜訊抑制 | yaw 需超過 1 degree deadband 才建立新目標 |
| 速度限制 | 韌體與 Python 皆限制位置與速度命令於 720 dps 內 |
| CAN 重送 | TWAI 使用正常模式，未使用 single-shot 模式 |
| 高頻回應 | 位置封包不等待 RX；狀態查詢仍驗證 RX |
| 輸出頻率 | yaw-only 模式移除控制不需要的 9 軸欄位 |
| 診斷資訊 | ESP32 回報 CAN 佇列與匯流排錯誤；Python 回報連線與序列埠錯誤 |

## 8. 目前驗證結果

| 驗證項目 | 結果 |
| --- | --- |
| ESP32 韌體編譯 | 通過，ESP32 Arduino Core 3.3.11 |
| ESP32 韌體燒錄 | 通過，Flash hash 驗證完成 |
| Xsens yaw-only reader | 已使用官方 XDA 讀取 roll/pitch/yaw |
| TCP yaw 資料頻率 | 2 秒量測約 99 Hz |
| Python bridge dry-run | 可連接 yaw 資料、建立初始參考角，未解鎖時不會控制馬達 |
| ESP32 與 WCMCU230 CAN loopback | 已驗證基本 CAN 硬體路徑 |

## 9. 建議啟動流程

先啟動 Xsens yaw-only reader：

```powershell
python python/xsens/mti630r_reader.py --rate 100 --stream-rate 100 --yaw-only --tcp-port 5006
```

先以 dry-run 啟動橋接程式：

```powershell
python python/xsens2esp32.py --sensor-ip 127.0.0.1 --sensor-port 5006 --esp-port COM7
```

確認 yaw 與命令摘要合理後，再解鎖馬達：

```powershell
python python/xsens2esp32.py --sensor-ip 127.0.0.1 --sensor-port 5006 --esp-port COM7 --arm
```

## 10. 後續工程項目

目前架構是高頻率的位置命令跟隨系統。馬達驅動器利用自身編碼器閉迴路完成位置控制，但 Python 尚未建立「Xsens yaw 與馬達實際角度」的外層誤差回授。

建議後續工作：

1. 量測正反向重複轉動時的 yaw 與馬達角度誤差。
2. 確認實際機構安裝方向是否需要反轉馬達正負方向。
3. 依實際負載與電源能力調整 `--gain`、deadband 與 720 dps 上限。
4. 加入長時間測試期間的馬達狀態與故障紀錄。
5. 若系統需要絕對機械零點，加入受控的 homing 或機構對位流程。
