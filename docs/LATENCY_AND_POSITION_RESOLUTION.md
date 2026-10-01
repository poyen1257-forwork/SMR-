# 延遲量測與最小定位角度

## 延遲資料鏈

系統以低負擔的抽樣方式記錄時間：控制仍維持 100 Hz，預設每 10 個位置命令記錄一次延遲樣本。

| 時間欄位 | 產生位置 | 意義 |
| --- | --- | --- |
| `imu_tx_ns` | `mti630r_reader.py` | IMU yaw JSON 封包送出前的 Unix 時間戳記，單位為 ns。 |
| `bridge_rx_ns` | `xsens2esp32.py` | Python bridge 完成解析該 JSON 封包的時間。 |
| `bridge_tx_ns` | `xsens2esp32.py` | Python 將位置命令寫入 ESP32 USB Serial 前的時間。 |
| `esp_rx_us` | ESP32 韌體 | ESP32 完成接收並解析擴充位置命令時的開機後時間，單位為 us。 |
| `esp_ack_rx_ns` | `xsens2esp32.py` | Python 收到 ESP32 `LAT` 確認回覆的時間。 |

執行 bridge 後，延遲結果會寫入 `logs/yaw_motor_latency.csv`：

```powershell
python python/xsens2esp32.py --sensor-ip 127.0.0.1 --sensor-port 5006 --esp-port COM7 --arm
```

CSV 的主要欄位：

| 欄位 | 用途 |
| --- | --- |
| `imu_to_bridge_ms` | IMU JSON 送出到 bridge 解析完成的延遲。 |
| `bridge_to_esp_ack_ms` | bridge 寫入 USB Serial 到收到 ESP32 確認的往返時間，包含 ESP32 解析與 USB 回傳。 |
| `imu_to_esp_ack_ms` | 從 IMU JSON 送出到收到 ESP32 確認的端到端時間。 |

`esp_rx_us` 是 ESP32 自己的開機後計時，沒有和電腦的 Unix 時鐘同步，因此不能直接與 `*_ns` 相減取得單向延遲。它用來分析 ESP32 內部接收時序；CSV 中的端到端值則是可直接比較的實測值。

## 最小定位角度

MF4015V2 與 MF5015V2 使用相同的 CAN 多圈位置格式。`0xA4` 命令的目標位置位於 `DATA[4..7]`，為 signed 32-bit little-endian，單位 `0.01 度/LSB`；`0x92` 回覆的實際多圈角度位於 `DATA[1..7]`，以 little-endian 有號 56-bit 數值承載，單位同為 `0.01 度/LSB`。解碼 56-bit 欄位時需正確符號延伸；不要把命令與回覆的欄位位置混用。

因此：

- 通訊命令的最小可表示增量為 **0.01 度**。
- 絕對值小於 **0.005 度**的輸入在韌體四捨五入後會成為 0，不能形成位移命令；其他輸入會量化為最接近的 `0.01 度`。
- 兩款馬達的 18-bit encoder 理論角度量化約為 `360 / 2^18 = 0.00137 度`，但這是回授解析度，不是保證可以移動的最小機械角度。

實際最小可重複運動角度會受減速比、摩擦、負載、馬達控制器增益與供電狀態影響。公開可取得的資料沒有提供 MF4015V2 或 MF5015V2 的「保證最小可動角度」規格，因此不可宣稱兩者在 0.01 度都一定會實際移動。

建議在無負載、低速度下，分別對兩顆馬達測試 `1`、`0.5`、`0.2`、`0.1`、`0.05`、`0.02`、`0.01` 度，每個角度重複 20 次，再以 `0x92` 多圈位置回讀及外部 IMU/治具確認可重複性。以目前追蹤系統的 `1 度` deadband 來說，已高於命令解析度，適合先進行穩定性與延遲測試。
