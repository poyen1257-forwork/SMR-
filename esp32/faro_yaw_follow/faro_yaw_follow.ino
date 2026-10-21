#include <Arduino.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include "esp_timer.h"
#include "driver/twai.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Enable USB CDC On Boot before using D6/D7 for CAN."
#endif

// Wi-Fi 熱點與 TCP 通訊；密碼只填本機，勿推送至公開 GitHub。
static const char WIFI_SSID[] = "YOUR_HOTSPOT_SSID";       // ESP32 要連線的熱點名稱；公開前請勿填入真實憑證。
static const char WIFI_PASSWORD[] = "YOUR_HOTSPOT_PASSWORD";   // 上述熱點的密碼；公開前請勿填入真實憑證。
static const uint16_t MOTOR_TCP_PORT = 5007;      // Python 連線控制馬達的 TCP 埠。
static const IPAddress WIFI_LOCAL_IP(10, 127, 162, 235); // ESP32 固定 IP。
static const IPAddress WIFI_GATEWAY(10, 127, 162, 164);  // 目前熱點閘道。
static const IPAddress WIFI_SUBNET(255, 255, 255, 0);   // 目前熱點子網路遮罩。
static const IPAddress WIFI_DNS(10, 127, 162, 164);     // 目前熱點 DNS。

// CAN 接線與馬達位址（XIAO D6/GPIO43 -> CTX、D7/GPIO44 <- CRX）。
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_43;  // ESP32 輸出 CAN TX 到 WCMCU230 CTX。
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_44;  // ESP32 從 WCMCU230 CRX 接收 CAN RX。
static const uint32_t MOTOR_CAN_ID = 0x141;        // 馬達使用的標準 CAN 識別碼。

// 馬達 CAN 指令碼；只定義傳輸資料的第一個位元組。
static const uint8_t READ_STATE_1 = 0x9A;          // 讀取馬達溫度、電壓及錯誤狀態。
static const uint8_t READ_MULTI_TURN_ANGLE = 0x92; // 讀取多圈角度。
static const uint8_t SPEED_CONTROL = 0xA2;         // 設定持續轉動速度。
static const uint8_t INCREMENT_ANGLE_SPEED = 0xA8; // 依增量角度轉動。
static const uint8_t MOTOR_STOP = 0x81;             // 停止馬達。

// 馬達運動限制，速度單位為每秒度數（dps）。
static const int MIN_SPEED_DPS = -720;             // 持續轉動的最大反向速度。
static const int MAX_SPEED_DPS = 720;              // 持續轉動的最大正向速度。
static const int MIN_POSITION_SPEED_DPS = 1;       // 增量角度指令允許的最低速度。
static const int MAX_POSITION_SPEED_DPS = 720;     // 增量角度指令允許的最高速度。
static const int SOFT_STOP_STEP_DPS = 30;          // 緩停時每步降低的速度。

// CAN 回覆與失聯停止保護，時間單位為毫秒。
static const uint32_t CAN_ACK_TIMEOUT_MS = 50;     // 等待匯流排確認的最長時間。
static const uint32_t CAN_REPLY_TIMEOUT_MS = 100;  // 等待馬達回覆的最長時間。
static const uint32_t MOTOR_COMMAND_LEASE_MS = 6000; // 運動指令過期後要求停止的時間。
static const uint32_t STOP_RETRY_MS = 500;         // 停止指令失敗後的重試間隔。
static const uint32_t WIFI_OUTPUT_STALL_MS = 1000; // TCP 持續不可寫時斷線並要求停機。

// CAN 與馬達運動的執行狀態；以下不是使用者要填的設定值。
static bool can_ready = false;                      // CAN 驅動是否已成功啟動。
static bool soft_stop_active = false;               // 是否正在逐步降低轉速。
static uint32_t next_soft_stop_step_at = 0;         // 下次緩停降速的時間點。
static int commanded_speed_dps = 0;                 // 最近設定的持續轉速。

// TCP 指令接收與 Wi-Fi 連線狀態。
static char command_buffer[96] = {};                // 暫存一行收到的 TCP 指令。
static size_t command_length = 0;                   // 目前暫存的指令字元數。
static WiFiServer motor_server(MOTOR_TCP_PORT);     // 接受 Python 連線的 TCP 伺服器。
static WiFiClient motor_client;                     // 目前連線的 Python 控制端。
static bool controller_connected = false;           // 是否已有控制端連線。
static bool server_started = false;                 // TCP 伺服器是否已啟動。

// 連線失效與停止保護的執行狀態。
static bool motor_motion_requested = false;        // 已送運動指令，失聯時需要求停機。
static bool stop_pending = false;                   // 是否仍需補送停止命令。
static uint32_t last_motor_command_at = 0;         // 上次收到控制指令的時間點。
static uint32_t next_stop_retry_at = 0;            // 下次補送停止命令的時間點。

static uint8_t wifi_output_queue[2048] = {};
static size_t wifi_output_begin = 0;
static size_t wifi_output_end = 0;
static uint32_t wifi_output_progress_at = 0;

void clear_wifi_output() {
  wifi_output_begin = 0;
  wifi_output_end = 0;
}

void fail_motor_client() {
  controller_connected = false;
  motor_client.stop();
  clear_wifi_output();
  command_length = 0;
  if (motor_motion_requested) {
    stop_pending = true;
    next_stop_retry_at = millis();
  }
}

void flush_wifi_output() {
  if (!controller_connected || wifi_output_begin == wifi_output_end) {
    return;
  }
  int socket_fd = motor_client.fd();
  if (socket_fd < 0) {
    fail_motor_client();
    return;
  }
  int sent = send(socket_fd, wifi_output_queue + wifi_output_begin,
                  wifi_output_end - wifi_output_begin, MSG_DONTWAIT);
  if (sent > 0) {
    wifi_output_begin += (size_t)sent;
    wifi_output_progress_at = millis();
    if (wifi_output_begin == wifi_output_end) {
      clear_wifi_output();
    }
  } else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) ||
             millis() - wifi_output_progress_at > WIFI_OUTPUT_STALL_MS) {
    fail_motor_client();
  }
}

bool queue_wifi_output(const uint8_t *buffer, size_t length) {
  if (length > sizeof(wifi_output_queue) - (wifi_output_end - wifi_output_begin)) {
    return false;
  }
  if (length > sizeof(wifi_output_queue) - wifi_output_end) {
    memmove(wifi_output_queue, wifi_output_queue + wifi_output_begin,
            wifi_output_end - wifi_output_begin);
    wifi_output_end -= wifi_output_begin;
    wifi_output_begin = 0;
  }
  if (wifi_output_begin == wifi_output_end) {
    wifi_output_progress_at = millis();
  }
  memcpy(wifi_output_queue + wifi_output_end, buffer, length);
  wifi_output_end += length;
  return true;
}

void write_serial_if_ready(const uint8_t *buffer, size_t length) {
  if (!Serial) {
    return;
  }
  int room = Serial.availableForWrite();
  if (room >= 0 && (size_t)room >= length) {
    Serial.write(buffer, length);
  }
}

class SerialOutput : public Print {
 public:
  using Print::write;
  size_t write(uint8_t value) override { return write(&value, 1); }
  size_t write(const uint8_t *buffer, size_t length) override {
    write_serial_if_ready(buffer, length);
    return length;
  }
};

static SerialOutput serial_output;

class MotorOutput : public Print {
 public:
  using Print::write;

  size_t write(uint8_t value) override { return write(&value, 1); }

  size_t write(const uint8_t *buffer, size_t length) override {
    if (controller_connected && !queue_wifi_output(buffer, length)) {
      fail_motor_client();
    }
    write_serial_if_ready(buffer, length);
    return length;
  }
};

static MotorOutput output;

void print_frame(Print &destination, const char *label,
                 const twai_message_t &frame) {
  char line[80];
  int length = snprintf(line, sizeof(line), "%s id=0x%03lX dlc=%u data=",
                        label, (unsigned long)frame.identifier,
                        frame.data_length_code);
  if (length < 0 || (size_t)length >= sizeof(line)) {
    return;
  }
  for (uint8_t i = 0; i < frame.data_length_code; i++) {
    int added = snprintf(line + length, sizeof(line) - length,
                         i + 1 == frame.data_length_code ? "%02X" : "%02X ",
                         frame.data[i]);
    if (added < 0 || (size_t)added >= sizeof(line) - length) {
      return;
    }
    length += added;
  }
  line[length++] = '\n';
  destination.write((const uint8_t *)line, length);
}

void print_status() {
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) == ESP_OK) {
    output.printf("TWAI state=%d tx_error=%lu rx_error=%lu bus_error=%lu tx_queue=%lu\n",
                  status.state,
                  status.tx_error_counter,
                  status.rx_error_counter,
                  status.bus_error_count,
                  status.msgs_to_tx);
  }
}

bool start_can() {
  pinMode((uint8_t)CAN_TX_PIN, INPUT_PULLUP);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
  delay(20);

  if (digitalRead((uint8_t)CAN_TX_PIN) == LOW ||
      digitalRead((uint8_t)CAN_RX_PIN) == LOW) {
    output.println("CAN start failed: D6 or D7 is LOW");
    return false;
  }

  digitalWrite((uint8_t)CAN_TX_PIN, HIGH);
  pinMode((uint8_t)CAN_TX_PIN, OUTPUT);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
  gpio_set_drive_capability(CAN_TX_PIN, GPIO_DRIVE_CAP_0);
  output.println("CAN pin level check: PASS");

  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  general.tx_queue_len = 1;
  general.rx_queue_len = 10;

  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
    output.println("twai_driver_install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    output.println("twai_start failed");
    twai_driver_uninstall();
    return false;
  }

  uint32_t alerts = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                    TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF;
  twai_reconfigure_alerts(alerts, nullptr);
  output.println("CAN started: 1 Mbps, standard ID 0x141");
  return true;
}

bool can_send_frame(const uint8_t data[8], int64_t *can_tx_us = nullptr) {
  if (!can_ready) {
    output.println("CAN is not ready");
    return false;
  }

  // Each command is sent once; a repeated relative move could rotate twice.
  twai_message_t stale = {};
  while (twai_receive(&stale, 0) == ESP_OK) {
  }

  twai_message_t tx = {};
  tx.identifier = MOTOR_CAN_ID;
  tx.data_length_code = 8;
  tx.ss = 1;
  memcpy(tx.data, data, 8);

  uint32_t old_alerts = 0;
  twai_read_alerts(&old_alerts, 0);
  if (Serial) {
    print_frame(serial_output, "TX", tx);
  }

  if (twai_transmit(&tx, 0) != ESP_OK) {
    output.println(data[0] == INCREMENT_ANGLE_SPEED
                       ? "Increment TX queue failed" : "TX queue failed");
    print_status();
    return false;
  }

  uint32_t started = millis();
  while (millis() - started < CAN_ACK_TIMEOUT_MS) {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(2)) != ESP_OK) {
      continue;
    }
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      if (can_tx_us != nullptr) {
        *can_tx_us = esp_timer_get_time();
      }
      output.println("TX ACK: OK");
      return true;
    }
    if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) {
      break;
    }
  }

  output.println(data[0] == INCREMENT_ANGLE_SPEED
                     ? "Increment TX failed: no CAN ACK" : "TX failed: no CAN ACK");
  print_status();
  return false;
}

bool can_exchange(const uint8_t data[8], uint8_t expected_command,
                  twai_message_t *reply, int64_t *can_tx_us = nullptr) {
  if (!can_send_frame(data, can_tx_us)) {
    return false;
  }

  uint32_t started = millis();
  while (millis() - started < CAN_REPLY_TIMEOUT_MS) {
    twai_message_t rx = {};
    if (twai_receive(&rx, pdMS_TO_TICKS(2)) != ESP_OK) {
      continue;
    }
    bool from_motor = rx.identifier == MOTOR_CAN_ID ||
                      rx.identifier == MOTOR_CAN_ID + 0x100;
    if (!rx.extd && !rx.rtr && from_motor &&
        rx.data_length_code == 8 && rx.data[0] == expected_command) {
      print_frame(output, "RX", rx);
      if (reply != nullptr) {
        *reply = rx;
      }
      return true;
    }
  }

  output.println("RX timeout");
  return false;
}

void read_motor_state() {
  const uint8_t request[8] = {READ_STATE_1, 0, 0, 0, 0, 0, 0, 0};
  twai_message_t reply = {};
  if (!can_exchange(request, READ_STATE_1, &reply)) {
    return;
  }

  int8_t temperature = (int8_t)reply.data[1];
  uint16_t voltage_raw =
      (uint16_t)reply.data[2] | ((uint16_t)reply.data[3] << 8);
  output.printf("MF5015V2 temp=%dC voltage=%.2fV error=0x%02X\n",
                temperature,
                voltage_raw * 0.01f,
                reply.data[7]);
}

bool read_motor_position(int64_t *position_cdeg) {
  const uint8_t request[8] = {READ_MULTI_TURN_ANGLE, 0, 0, 0, 0, 0, 0, 0};
  twai_message_t reply = {};
  if (!can_exchange(request, READ_MULTI_TURN_ANGLE, &reply)) {
    return false;
  }

  uint64_t raw = 0;
  for (uint8_t i = 1; i < 8; ++i) {
    raw |= (uint64_t)reply.data[i] << ((i - 1) * 8);
  }
  int64_t signed_angle = (int64_t)raw;
  if (raw & (1ULL << 55)) {
    signed_angle -= (1LL << 56);
  }
  *position_cdeg = signed_angle;
  output.printf("POS_CDEG,%lld\n", (long long)signed_angle);
  return true;
}

bool run_motor(int speed_dps) {
  // MF5015V2 speed unit is 0.01 dps/LSB.
  int32_t speed_control = (int32_t)speed_dps * 100;
  const uint8_t speed_command[8] = {
      SPEED_CONTROL, 0, 0, 0,
      (uint8_t)(speed_control & 0xFF),
      (uint8_t)((speed_control >> 8) & 0xFF),
      (uint8_t)((speed_control >> 16) & 0xFF),
      (uint8_t)((speed_control >> 24) & 0xFF)};
  twai_message_t reply = {};

  if (speed_dps != 0) {
    motor_motion_requested = true;
  }
  if (!can_exchange(speed_command, SPEED_CONTROL, &reply)) {
    output.println("Speed command failed");
    return false;
  }
  commanded_speed_dps = speed_dps;
  motor_motion_requested = speed_dps != 0;
  return true;
}

bool move_motor_increment(float delta_degrees, int max_speed_dps,
                          bool report_latency = false, uint32_t sequence = 0,
                          uint64_t imu_tx_ns = 0, uint64_t bridge_tx_ns = 0,
                          int64_t esp_rx_us = 0) {
  if (!isfinite(delta_degrees) || max_speed_dps < MIN_POSITION_SPEED_DPS ||
      max_speed_dps > MAX_POSITION_SPEED_DPS) {
    output.println("Increment command: angle must be finite; speed must be 1..720 dps.");
    return false;
  }

  double scaled_delta = round((double)delta_degrees * 100.0);
  if (scaled_delta < INT32_MIN || scaled_delta > INT32_MAX) {
    output.println("Increment command cancelled: angle is outside protocol range.");
    return false;
  }
  int32_t delta_cdeg = (int32_t)scaled_delta;
  if (delta_cdeg == 0) {
    output.println("Increment command ignored: magnitude is below 0.005 deg.");
    return true;
  }

  uint16_t speed_limit = (uint16_t)max_speed_dps;
  uint32_t raw_delta = (uint32_t)delta_cdeg;
  const uint8_t increment_command[8] = {
      INCREMENT_ANGLE_SPEED,
      0,
      (uint8_t)(speed_limit & 0xFF),
      (uint8_t)((speed_limit >> 8) & 0xFF),
      (uint8_t)(raw_delta & 0xFF),
      (uint8_t)((raw_delta >> 8) & 0xFF),
      (uint8_t)((raw_delta >> 16) & 0xFF),
      (uint8_t)((raw_delta >> 24) & 0xFF)};
  int64_t esp_can_tx_us = 0;
  motor_motion_requested = true;
  if (!can_exchange(increment_command, INCREMENT_ANGLE_SPEED, nullptr,
                    report_latency ? &esp_can_tx_us : nullptr)) {
    output.println("Increment command failed");
    return false;
  }

  soft_stop_active = false;
  commanded_speed_dps = 0;
  if (report_latency) {
    output.printf("LAT,%lu,%llu,%llu,%lld,%lld\n", (unsigned long)sequence,
                  (unsigned long long)imu_tx_ns,
                  (unsigned long long)bridge_tx_ns, (long long)esp_rx_us,
                  (long long)esp_can_tx_us);
  }
  return true;
}

bool stop_motor_immediately() {
  const uint8_t stop_command[8] = {MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0};
  soft_stop_active = false;
  output.println("Stop motor");
  if (!can_exchange(stop_command, MOTOR_STOP, nullptr)) {
    output.println("Stop command was not acknowledged");
    stop_pending = true;
    next_stop_retry_at = millis() + STOP_RETRY_MS;
    return false;
  }
  stop_pending = false;
  commanded_speed_dps = 0;
  motor_motion_requested = false;
  return true;
}

void begin_soft_stop() {
  if (commanded_speed_dps == 0) {
    stop_motor_immediately();
    return;
  }

  soft_stop_active = true;
  next_soft_stop_step_at = millis() + 100;
  output.printf("Soft stop: %d dps, reduce by %d dps every 100 ms\n",
                commanded_speed_dps,
                SOFT_STOP_STEP_DPS);
}

void update_soft_stop() {
  if (!soft_stop_active ||
      (int32_t)(millis() - next_soft_stop_step_at) < 0) {
    return;
  }

  int next_speed = commanded_speed_dps;
  if (next_speed > 0) {
    next_speed -= SOFT_STOP_STEP_DPS;
    if (next_speed < 0) {
      next_speed = 0;
    }
  } else {
    next_speed += SOFT_STOP_STEP_DPS;
    if (next_speed > 0) {
      next_speed = 0;
    }
  }

  if (next_speed == 0) {
    stop_motor_immediately();
    return;
  }

  if (!run_motor(next_speed)) {
    stop_motor_immediately();
    return;
  }

  next_soft_stop_step_at = millis() + 100;
}

bool valid_speed(int speed_dps) {
  if (speed_dps < MIN_SPEED_DPS || speed_dps > MAX_SPEED_DPS) {
    output.println("Speed must be an integer from -720 to 720 dps.");
    return false;
  }
  return true;
}

void handle_command(char *line) {
  char command = 0;
  char extra = 0;
  if (sscanf(line, " %c", &command) != 1) {
    return;
  }

  if (command == 'r' && sscanf(line, " %c %c", &command, &extra) == 1) {
    read_motor_state();
    return;
  }
  if (command == 'p' && sscanf(line, " %c %c", &command, &extra) == 1) {
    int64_t position_cdeg = 0;
    if (!read_motor_position(&position_cdeg)) {
      output.println("Motor position read failed");
    }
    return;
  }
  if (command == 's' && sscanf(line, " %c %c", &command, &extra) == 1) {
    if (stop_pending) {
      stop_motor_immediately();
    } else {
      begin_soft_stop();
    }
    return;
  }
  if (stop_pending) {
    output.println("MOTOR_STOP_PENDING");
    return;
  }
  if (command == 'k' && sscanf(line, " %c %c", &command, &extra) == 1) {
    return;
  }
  if (command == 'm') {
    int speed_dps = 0;
    if (sscanf(line, " %c %d %c", &command, &speed_dps, &extra) == 2 &&
        valid_speed(speed_dps)) {
      soft_stop_active = false;
      run_motor(speed_dps);
      return;
    }
  }
  if (command == 'n') {
    float delta_degrees = 0.0f;
    int max_speed_dps = 0;
    uint32_t sequence = 0;
    uint64_t imu_tx_ns = 0;
    uint64_t bridge_tx_ns = 0;
    int fields = sscanf(line, " %c %f %d %lu %llu %llu %c", &command,
                        &delta_degrees, &max_speed_dps, &sequence, &imu_tx_ns,
                        &bridge_tx_ns, &extra);
    if (fields == 3) {
      if (!move_motor_increment(delta_degrees, max_speed_dps)) {
        output.println("Increment command failed");
      }
      return;
    }
    if (fields == 6) {
      const int64_t esp_rx_us = esp_timer_get_time();
      if (!move_motor_increment(delta_degrees, max_speed_dps, true, sequence,
                                imu_tx_ns, bridge_tx_ns, esp_rx_us)) {
        output.println("Increment command failed");
      }
      return;
    }
  }
  output.println("Invalid command. Use r, m -720..720, n <deg> <1..720 dps>, or s.");
}

void connect_wifi() {
  if (WIFI_SSID[0] == '\0' || strcmp(WIFI_SSID, "YOUR_HOTSPOT_SSID") == 0 ||
      strcmp(WIFI_PASSWORD, "YOUR_HOTSPOT_PASSWORD") == 0) {
    output.println("Set WIFI_SSID and WIFI_PASSWORD at the top of this sketch.");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    serial_output.printf("Wi-Fi disconnected: reason=%u\n",
                         info.wifi_sta_disconnected.reason);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  if (!WiFi.config(WIFI_LOCAL_IP, WIFI_GATEWAY, WIFI_SUBNET, WIFI_DNS)) {
    output.println("Static Wi-Fi IP configuration failed; Wi-Fi was not started.");
    return;
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  output.println("Connecting to Wi-Fi...");
}

void disconnect_motor_client() {
  const bool stop_motion = controller_connected && motor_motion_requested;
  controller_connected = false;
  motor_client.stop();
  clear_wifi_output();
  command_length = 0;
  if (stop_motion) {
    stop_pending = true;
    output.println("Wi-Fi controller disconnected; stopping motor.");
    if (stop_motor_immediately()) {
      stop_pending = false;
    } else {
      next_stop_retry_at = millis() + STOP_RETRY_MS;
    }
  }
}

void retry_pending_stop() {
  if (!stop_pending || (int32_t)(millis() - next_stop_retry_at) < 0) {
    return;
  }
  next_stop_retry_at = millis() + STOP_RETRY_MS;
  if (stop_motor_immediately()) {
    stop_pending = false;
    output.println("Pending motor stop confirmed.");
  }
}

bool motor_client_closed() {
  int socket_fd = motor_client.fd();
  if (socket_fd < 0) {
    return true;
  }
  uint8_t next_byte = 0;
  // NetworkClient::connected() in ESP32 core 3.3.11 treats recv()==0 as connected.
  return recv(socket_fd, &next_byte, 1, MSG_PEEK | MSG_DONTWAIT) == 0;
}

void reject_motor_client(WiFiClient &client, const char *message) {
  int socket_fd = client.fd();
  if (socket_fd >= 0) {
    send(socket_fd, message, strlen(message), MSG_DONTWAIT);
  }
  client.stop();
}

void service_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    if (controller_connected) {
      disconnect_motor_client();
    }
    if (server_started) {
      motor_server.end();
      server_started = false;
    }
    return;
  }
  if (!server_started) {
    motor_server.begin();
    motor_server.setNoDelay(true);
    server_started = true;
    output.printf("Wi-Fi motor server: %s:%u\n",
                  WiFi.localIP().toString().c_str(), MOTOR_TCP_PORT);
  }

  if (controller_connected &&
      (!motor_client.connected() || motor_client_closed())) {
    disconnect_motor_client();
  }
  if (controller_connected && motor_motion_requested &&
      millis() - last_motor_command_at > MOTOR_COMMAND_LEASE_MS) {
    output.println("Motor command lease expired; stopping motor.");
    disconnect_motor_client();
  }
  flush_wifi_output();
  WiFiClient incoming = motor_server.accept();
  if (incoming) {
    if (motor_client && motor_client.connected()) {
      reject_motor_client(incoming, "BUSY\r\n");
    } else {
      motor_client = incoming;
      motor_client.setNoDelay(true);
      controller_connected = true;
      last_motor_command_at = millis();
      command_length = 0;
      clear_wifi_output();
    }
  }
  if (!motor_client || !motor_client.connected()) {
    return;
  }

  while (motor_client.available()) {
    char incoming_byte = (char)motor_client.read();
    if (incoming_byte == '\r') {
      continue;
    }
    if (incoming_byte == '\n') {
      if (command_length == 0) {
        continue;
      }
      command_buffer[command_length] = '\0';
      last_motor_command_at = millis();
      handle_command(command_buffer);
      command_length = 0;
      flush_wifi_output();
      if (!controller_connected) {
        return;
      }
      continue;
    }
    if (command_length < sizeof(command_buffer) - 1) {
      command_buffer[command_length++] = incoming_byte;
    } else {
      command_length = 0;
      output.println("Command is too long.");
      disconnect_motor_client();
      return;
    }
  }
  flush_wifi_output();
}

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_MODE
  // USB logging must not hold up CAN commands or Wi-Fi replies when COM7 is unread.
  Serial.setTxTimeoutMs(1);
#endif
  delay(1000);
  output.println();
  output.println("MF5015V2 CAN/Wi-Fi (D6 TX, D7 RX)");
  output.println("TCP commands: r, p, m <dps>, n <degrees> <speed>, s");
  output.println("TCP control has no application authentication; use a private hotspot only.");
  can_ready = start_can();
  connect_wifi();
}

void loop() {
  update_soft_stop();
  service_wifi();
  retry_pending_stop();
}
