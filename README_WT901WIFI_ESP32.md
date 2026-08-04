# WT901WIFI 9-axis MCU data practice

This practice uses an ESP32 as the MCU and receives WitMotion WT901WIFI data over UDP. The example is intentionally simple: confirm that UDP data is received, show raw bytes, and print parsed 9-axis values when valid WitMotion frames are found.

## Hardware and network

1. Use a 2.4GHz Wi-Fi network. WT901WIFI does not use 5GHz Wi-Fi.
2. Power the WT901WIFI with Type-C.
3. Connect the computer and ESP32 to the same Wi-Fi first.
4. Upload `WT901WIFI_ESP32_UDP/WT901WIFI_ESP32_UDP.ino` to the ESP32.
5. Open Serial Monitor at `115200` baud and note the `ESP32 IP`.

## WT901WIFI setting

Use WitMotion PC software to configure the sensor:

1. Set network mode to `Station Mode`.
2. Select `UDP protocol`.
3. Choose `Specify user server`.
4. Wi-Fi SSID/password: same 2.4GHz Wi-Fi used by ESP32.
5. User server IP: the ESP32 IP printed in Serial Monitor.
6. User server port: `1399` unless you changed `LISTEN_PORT` in the sketch.

After saving the setting, restart the sensor if it does not immediately stream data.

## Arduino sketch settings

Edit these lines before uploading:

```cpp
const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const uint16_t LISTEN_PORT = 1399;
```

Set this to `false` after you confirm data is arriving:

```cpp
const bool PRINT_RAW_UDP = true;
```

## How to confirm it is working

Serial Monitor should show lines like:

```text
ESP32 IP: 192.168.1.23
Listening UDP port: 1399
UDP #1 from 192.168.1.55:50000, size=44, raw=55 51 ...
VALID frames 4 | UDP packets 1 | bytes 44 | checksum errors 0
ACC[g] ...
```

Meaning:

- `UDP #...`: ESP32 has received network data from WT901WIFI.
- `raw=55 51 ...`: raw WitMotion bytes are arriving.
- `VALID frames`: checksum passed and the frame was parsed.
- `checksum errors`: should usually stay low or `0`.

## Parsed data

The sketch parses the common WitMotion 11-byte standard frames:

- `0x55 0x51`: 3-axis acceleration, unit `g`
- `0x55 0x52`: 3-axis angular velocity, unit `deg/s`
- `0x55 0x53`: 3-axis Euler angle, unit `degree`
- `0x55 0x54`: 3-axis magnetic field raw value

If Serial Monitor shows only `packets 0`, check:

- ESP32 and WT901WIFI are on the same 2.4GHz Wi-Fi.
- WT901WIFI is in Station Mode, not AP Mode.
- The configured user server IP is the ESP32 IP.
- The UDP port matches `LISTEN_PORT`.
- Firewall/router isolation is not blocking Wi-Fi clients.
