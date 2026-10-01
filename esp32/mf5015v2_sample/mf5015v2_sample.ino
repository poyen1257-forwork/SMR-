#include <Arduino.h>
#include <limits.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/twai.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Enable USB CDC On Boot before using D6/D7 for CAN."
#endif

// XIAO ESP32-S3 -> WCMCU230
// D6 / GPIO43 -> CTX
// D7 / GPIO44 <- CRX
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_43;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_44;
static const uint32_t MOTOR_CAN_ID = 0x141;
static const uint8_t READ_MULTI_TURN_ANGLE = 0x92;
static const uint8_t READ_STATE_1 = 0x9A;
static const uint8_t SPEED_CONTROL = 0xA2;
static const uint8_t POSITION_CONTROL_SPEED = 0xA4;
static const uint8_t MOTOR_STOP = 0x81;

static const int MIN_SPEED_DPS = -720;
static const int MAX_SPEED_DPS = 720;
static const int MIN_POSITION_SPEED_DPS = 1;
static const int MAX_POSITION_SPEED_DPS = 720;
static const int SOFT_STOP_STEP_DPS = 30;

static bool can_ready = false;
static bool soft_stop_active = false;
static uint32_t next_soft_stop_step_at = 0;
static int commanded_speed_dps = 0;
static bool position_target_initialized = false;
static int32_t position_target_cdeg = 0;
static uint32_t position_commands_sent = 0;
static uint32_t last_position_report_at = 0;
static char command_buffer[96] = {};
static size_t command_length = 0;
static uint32_t last_command_byte_at = 0;

void print_frame(const char *label, const twai_message_t &frame) {
  Serial.printf("%s id=0x%03lX dlc=%u data=",
                label,
                (unsigned long)frame.identifier,
                frame.data_length_code);
  for (uint8_t i = 0; i < frame.data_length_code; i++) {
    if (frame.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame.data[i], HEX);
    Serial.print(i + 1 == frame.data_length_code ? '\n' : ' ');
  }
}

void print_status() {
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) == ESP_OK) {
    Serial.printf("TWAI state=%d tx_error=%lu rx_error=%lu bus_error=%lu\n",
                  status.state,
                  status.tx_error_counter,
                  status.rx_error_counter,
                  status.bus_error_count);
  }
}

bool start_can() {
  pinMode((uint8_t)CAN_TX_PIN, INPUT_PULLUP);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
  delay(20);

  if (digitalRead((uint8_t)CAN_TX_PIN) == LOW ||
      digitalRead((uint8_t)CAN_RX_PIN) == LOW) {
    Serial.println("CAN start failed: D6 or D7 is LOW");
    return false;
  }

  digitalWrite((uint8_t)CAN_TX_PIN, HIGH);
  pinMode((uint8_t)CAN_TX_PIN, OUTPUT);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
  gpio_set_drive_capability(CAN_TX_PIN, GPIO_DRIVE_CAP_0);
  Serial.println("CAN pin level check: PASS");

  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  general.tx_queue_len = 1;
  general.rx_queue_len = 10;

  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
    Serial.println("twai_driver_install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("twai_start failed");
    twai_driver_uninstall();
    return false;
  }

  uint32_t alerts = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                    TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF;
  twai_reconfigure_alerts(alerts, nullptr);
  Serial.println("CAN started: 1 Mbps, standard ID 0x141");
  return true;
}

bool can_exchange(const uint8_t data[8], uint8_t expected_command,
                  twai_message_t *reply) {
  if (!can_ready) {
    Serial.println("CAN is not ready");
    return false;
  }

  // Remove stale frames so this request is paired with its own response.
  twai_message_t stale = {};
  while (twai_receive(&stale, 0) == ESP_OK) {
  }

  twai_message_t tx = {};
  tx.identifier = MOTOR_CAN_ID;
  tx.data_length_code = 8;
  // Normal mode retries a frame when an ACK is briefly missed. Do not use
  // single-shot mode for a live motor control bus.
  tx.ss = 0;
  memcpy(tx.data, data, 8);

  uint32_t old_alerts = 0;
  twai_read_alerts(&old_alerts, 0);
  print_frame("TX", tx);

  if (twai_transmit(&tx, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial.println("TX queue failed");
    print_status();
    return false;
  }

  bool tx_ok = false;
  uint32_t started = millis();
  while (millis() - started < 500) {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(20)) != ESP_OK) {
      continue;
    }
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      tx_ok = true;
      break;
    }
    if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) {
      break;
    }
  }

  if (!tx_ok) {
    Serial.println("TX failed: no CAN ACK");
    print_status();
    return false;
  }

  Serial.println("TX ACK: OK");
  started = millis();
  while (millis() - started < 1000) {
    twai_message_t rx = {};
    if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) {
      continue;
    }
    print_frame("RX", rx);
    bool from_motor = rx.identifier == MOTOR_CAN_ID ||
                      rx.identifier == MOTOR_CAN_ID + 0x100;
    if (!rx.extd && !rx.rtr && from_motor &&
        rx.data_length_code == 8 && rx.data[0] == expected_command) {
      if (reply != nullptr) {
        *reply = rx;
      }
      return true;
    }
  }

  Serial.println("RX timeout");
  return false;
}

bool can_send_position_target(const uint8_t data[8], bool wait_for_tx_success,
                              int64_t *can_tx_us) {
  if (!can_ready) {
    Serial.println("CAN is not ready");
    return false;
  }

  // Position tracking can send a new target every 10 ms. Never wait for the
  // response here; serial input must remain available for the next target.
  twai_message_t stale = {};
  while (twai_receive(&stale, 0) == ESP_OK) {
  }

  twai_message_t tx = {};
  tx.identifier = MOTOR_CAN_ID;
  tx.data_length_code = 8;
  memcpy(tx.data, data, 8);

  uint32_t old_alerts = 0;
  twai_read_alerts(&old_alerts, 0);
  if (twai_transmit(&tx, 0) != ESP_OK) {
    Serial.println("Position TX queue failed");
    print_status();
    return false;
  }

  if (!wait_for_tx_success) {
    return true;
  }

  uint32_t started = millis();
  while (millis() - started < 30) {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(5)) != ESP_OK) {
      continue;
    }
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      if (can_tx_us != nullptr) {
        *can_tx_us = esp_timer_get_time();
      }
      return true;
    }
    if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) {
      break;
    }
  }

  Serial.println("Position TX failed: no CAN ACK");
  print_status();
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
  Serial.printf("MF4015V2 temp=%dC voltage=%.2fV error=0x%02X\n",
                temperature,
                voltage_raw * 0.01f,
                reply.data[7]);
}

bool run_motor(int speed_dps) {
  // MF4015V2 speed unit is 0.01 dps/LSB.
  int32_t speed_control = (int32_t)speed_dps * 100;
  const uint8_t speed_command[8] = {
      SPEED_CONTROL, 0, 0, 0,
      (uint8_t)(speed_control & 0xFF),
      (uint8_t)((speed_control >> 8) & 0xFF),
      (uint8_t)((speed_control >> 16) & 0xFF),
      (uint8_t)((speed_control >> 24) & 0xFF)};
  twai_message_t reply = {};

  Serial.printf("Start speed control: %d dps\n", speed_dps);
  if (!can_exchange(speed_command, SPEED_CONTROL, &reply)) {
    Serial.println("Speed command failed");
    return false;
  }
  commanded_speed_dps = speed_dps;
  return true;
}

bool read_motor_position(int32_t *position_cdeg) {
  const uint8_t request[8] = {READ_MULTI_TURN_ANGLE, 0, 0, 0, 0, 0, 0, 0};
  twai_message_t reply = {};
  if (!can_exchange(request, READ_MULTI_TURN_ANGLE, &reply)) {
    return false;
  }

  uint32_t raw = (uint32_t)reply.data[4] |
                 ((uint32_t)reply.data[5] << 8) |
                 ((uint32_t)reply.data[6] << 16) |
                 ((uint32_t)reply.data[7] << 24);
  *position_cdeg = (int32_t)raw;
  Serial.printf("Motor multi-turn position: %.2f deg\n", *position_cdeg * 0.01f);
  return true;
}

bool move_motor_relative(float delta_degrees, int max_speed_dps,
                         bool report_latency = false, uint32_t sequence = 0,
                         uint64_t imu_tx_ns = 0, uint64_t bridge_tx_ns = 0,
                         int64_t esp_rx_us = 0) {
  if (!isfinite(delta_degrees) || max_speed_dps < MIN_POSITION_SPEED_DPS ||
      max_speed_dps > MAX_POSITION_SPEED_DPS) {
    Serial.println("Position command: angle must be finite; speed must be 1..720 dps.");
    return false;
  }

  int32_t delta_cdeg = (int32_t)lroundf(delta_degrees * 100.0f);
  if (delta_cdeg == 0) {
    Serial.println("Position command ignored: magnitude is below 0.005 deg.");
    return true;
  }

  if (!position_target_initialized) {
    if (!read_motor_position(&position_target_cdeg)) {
      Serial.println("Position command cancelled: cannot read motor position.");
      return false;
    }
    position_target_initialized = true;
  }

  int64_t target = (int64_t)position_target_cdeg + delta_cdeg;
  if (target < INT32_MIN || target > INT32_MAX) {
    Serial.println("Position command cancelled: target angle is outside protocol range.");
    return false;
  }

  int32_t target_cdeg = (int32_t)target;
  const uint8_t position_command[8] = {
      POSITION_CONTROL_SPEED,
      0,
      (uint8_t)(max_speed_dps & 0xFF),
      (uint8_t)((max_speed_dps >> 8) & 0xFF),
      (uint8_t)(target_cdeg & 0xFF),
      (uint8_t)((target_cdeg >> 8) & 0xFF),
      (uint8_t)((target_cdeg >> 16) & 0xFF),
      (uint8_t)((target_cdeg >> 24) & 0xFF)};
  int64_t esp_can_tx_us = 0;
  if (!can_send_position_target(position_command, report_latency,
                                &esp_can_tx_us)) {
    Serial.println("Position command failed");
    return false;
  }

  soft_stop_active = false;
  commanded_speed_dps = 0;
  position_target_cdeg = target_cdeg;
  position_commands_sent++;
  if (report_latency) {
    Serial.printf("LAT,%lu,%llu,%llu,%lld,%lld\n", (unsigned long)sequence,
                  (unsigned long long)imu_tx_ns,
                  (unsigned long long)bridge_tx_ns, (long long)esp_rx_us,
                  (long long)esp_can_tx_us);
  }
  if (millis() - last_position_report_at >= 500) {
    Serial.printf("Position tracking: target=%.2f deg, speed=%d dps, sent=%lu\n",
                  target_cdeg * 0.01f, max_speed_dps,
                  (unsigned long)position_commands_sent);
    position_commands_sent = 0;
    last_position_report_at = millis();
  }
  return true;
}

bool stop_motor_immediately() {
  const uint8_t stop_command[8] = {MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0};
  soft_stop_active = false;
  Serial.println("Stop motor");
  if (!can_exchange(stop_command, MOTOR_STOP, nullptr)) {
    Serial.println("Stop command was not acknowledged");
    return false;
  }
  commanded_speed_dps = 0;
  return true;
}

void begin_soft_stop() {
  if (commanded_speed_dps == 0) {
    stop_motor_immediately();
    return;
  }

  soft_stop_active = true;
  next_soft_stop_step_at = millis() + 100;
  Serial.printf("Soft stop: %d dps, reduce by %d dps every 100 ms\n",
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

  if (!run_motor(next_speed)) {
    next_soft_stop_step_at = millis() + 100;
    return;
  }

  if (next_speed == 0) {
    stop_motor_immediately();
    return;
  }

  next_soft_stop_step_at = millis() + 100;
}

bool valid_speed(int speed_dps) {
  if (speed_dps < MIN_SPEED_DPS || speed_dps > MAX_SPEED_DPS) {
    Serial.println("Speed must be an integer from -720 to 720 dps.");
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
  if (command == 's' && sscanf(line, " %c %c", &command, &extra) == 1) {
    begin_soft_stop();
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
      move_motor_relative(delta_degrees, max_speed_dps);
      return;
    }
    if (fields == 6) {
      const int64_t esp_rx_us = esp_timer_get_time();
      move_motor_relative(delta_degrees, max_speed_dps, true, sequence,
                          imu_tx_ns, bridge_tx_ns, esp_rx_us);
      return;
    }
  }
  Serial.println("Invalid command. Use r, m -720..720, n <deg> <1..720 dps>, or s.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("MF5015V2 CAN test (D6 TX, D7 RX)");
  Serial.println("r = read motor state");
  Serial.println("m <-720..720> = run continuously; negative reverses direction");
  Serial.println("n <degrees> <speed> = move relative angle using position control");
  Serial.println("s = soft stop (reduce speed by 30 dps every 100 ms)");
  can_ready = start_can();
}

void loop() {
  update_soft_stop();

  while (Serial.available()) {
    char incoming = (char)Serial.read();
    if (incoming == '\r' || incoming == '\n') {
      if (command_length > 0) {
        command_buffer[command_length] = '\0';
        handle_command(command_buffer);
        command_length = 0;
      }
      continue;
    }

    if (command_length < sizeof(command_buffer) - 1) {
      command_buffer[command_length++] = incoming;
      last_command_byte_at = millis();
    } else {
      command_length = 0;
      Serial.println("Command is too long.");
    }
  }

  // VS Code serial monitors may send text without CR/LF.
  if (command_length > 0 && millis() - last_command_byte_at >= 100) {
    command_buffer[command_length] = '\0';
    handle_command(command_buffer);
    command_length = 0;
  }
}
