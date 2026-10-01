#include <Arduino.h>
#include <limits.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/twai.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Enable USB CDC On Boot before using XIAO D5-D8 for CAN."
#endif

struct MotorChannel {
  const char *name;
  gpio_num_t tx_pin;
  gpio_num_t rx_pin;
  bool position_target_initialized;
  int64_t position_target_cdeg;
  float commanded_speed_dps;
  bool soft_stop_active;
  uint32_t next_soft_stop_step_at;
  bool motor_restored;
};

static MotorChannel motors[] = {
    {"MF5015V2", (gpio_num_t)D5, (gpio_num_t)D6, false, 0, 0.0f, false, 0, false},
    {"MF4015V2", (gpio_num_t)D7, (gpio_num_t)D8, false, 0, 0.0f, false, 0, false},
};
static const size_t MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);
static const uint32_t MOTOR_CAN_ID = 0x141;
static const uint8_t READ_MULTI_TURN_ANGLE = 0x92;
static const uint8_t READ_STATE_1 = 0x9A;
static const uint8_t SPEED_CONTROL = 0xA2;
static const uint8_t POSITION_CONTROL_SPEED = 0xA4;
static const uint8_t MOTOR_RUNNING = 0x88;
static const uint8_t MOTOR_STOP = 0x81;

static const int MIN_SPEED_DPS = -720;
static const int MAX_SPEED_DPS = 720;
static const int MIN_POSITION_SPEED_DPS = 30;
static const int MAX_POSITION_SPEED_DPS = 720;
static const int SOFT_STOP_STEP_DPS = 30;

static bool can_ready = false;
static int active_channel = -1;
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

void release_can_pins(size_t channel) {
  pinMode((uint8_t)motors[channel].tx_pin, INPUT_PULLUP);
  pinMode((uint8_t)motors[channel].rx_pin, INPUT_PULLUP);
}

void stop_can() {
  if (can_ready) {
    twai_stop();
    twai_driver_uninstall();
  }
  can_ready = false;
  if (active_channel >= 0) {
    release_can_pins((size_t)active_channel);
  }
  active_channel = -1;
}

bool start_can(size_t channel) {
  if (channel >= MOTOR_COUNT) {
    return false;
  }

  const gpio_num_t tx_pin = motors[channel].tx_pin;
  const gpio_num_t rx_pin = motors[channel].rx_pin;
  pinMode((uint8_t)tx_pin, INPUT_PULLUP);
  pinMode((uint8_t)rx_pin, INPUT_PULLUP);
  delay(20);

  if (digitalRead((uint8_t)tx_pin) == LOW ||
      digitalRead((uint8_t)rx_pin) == LOW) {
    Serial.printf("%s CAN start failed: TX/RX pin reads LOW\n",
                  motors[channel].name);
    return false;
  }

  digitalWrite((uint8_t)tx_pin, HIGH);
  pinMode((uint8_t)tx_pin, OUTPUT);
  pinMode((uint8_t)rx_pin, INPUT_PULLUP);
  gpio_set_drive_capability(tx_pin, GPIO_DRIVE_CAP_0);

  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
  general.tx_queue_len = 1;
  general.rx_queue_len = 10;

  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
    Serial.printf("%s twai_driver_install failed\n", motors[channel].name);
    release_can_pins(channel);
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.printf("%s twai_start failed\n", motors[channel].name);
    twai_driver_uninstall();
    release_can_pins(channel);
    return false;
  }

  uint32_t alerts = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                    TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF;
  if (twai_reconfigure_alerts(alerts, nullptr) != ESP_OK) {
    Serial.printf("%s alert setup failed\n", motors[channel].name);
    twai_stop();
    twai_driver_uninstall();
    release_can_pins(channel);
    return false;
  }

  delay(10);

  can_ready = true;
  active_channel = (int)channel;
  Serial.printf("CAN active: %s, 1 Mbps, standard ID 0x141\n",
                motors[channel].name);
  return true;
}

bool select_can_channel(size_t channel) {
  if (channel >= MOTOR_COUNT) {
    return false;
  }
  if (can_ready && active_channel == (int)channel) {
    return true;
  }
  stop_can();
  return start_can(channel);
}

bool can_exchange(size_t channel, const uint8_t data[8],
                  uint8_t expected_command, twai_message_t *reply) {
  if (!select_can_channel(channel)) {
    Serial.printf("Cannot activate CAN for %s\n", motors[channel].name);
    return false;
  }

  twai_message_t stale = {};
  while (twai_receive(&stale, 0) == ESP_OK) {
  }

  twai_message_t tx = {};
  tx.identifier = MOTOR_CAN_ID;
  tx.data_length_code = 8;
  tx.ss = 0;
  memcpy(tx.data, data, 8);

  uint32_t ignored_alerts = 0;
  twai_read_alerts(&ignored_alerts, 0);
  Serial.printf("%s ", motors[channel].name);
  print_frame("TX", tx);

  if (twai_transmit(&tx, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial.printf("%s TX queue failed\n", motors[channel].name);
    print_status();
    stop_can();
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
    Serial.printf("%s TX failed: no CAN ACK\n", motors[channel].name);
    print_status();
    stop_can();
    return false;
  }

  Serial.printf("%s TX ACK: OK\n", motors[channel].name);
  started = millis();
  while (millis() - started < 1000) {
    twai_message_t rx = {};
    if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) {
      continue;
    }
    Serial.printf("%s ", motors[channel].name);
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

  Serial.printf("%s RX timeout\n", motors[channel].name);
  return false;
}

bool can_send_position_target(size_t channel, const uint8_t data[8],
                              bool wait_for_tx_success, int64_t *can_tx_us) {
  if (!select_can_channel(channel)) {
    Serial.printf("Cannot activate CAN for %s\n", motors[channel].name);
    return false;
  }

  twai_message_t stale = {};
  while (twai_receive(&stale, 0) == ESP_OK) {
  }

  twai_message_t tx = {};
  tx.identifier = MOTOR_CAN_ID;
  tx.data_length_code = 8;
  memcpy(tx.data, data, 8);
  uint32_t ignored_alerts = 0;
  twai_read_alerts(&ignored_alerts, 0);

  if (twai_transmit(&tx, 0) != ESP_OK) {
    Serial.printf("%s position TX queue failed\n", motors[channel].name);
    print_status();
    stop_can();
    return false;
  }
  if (!wait_for_tx_success) {
    return true;
  }

  uint32_t started = millis();
  bool tx_ok = false;
  while (millis() - started < 30) {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(5)) != ESP_OK) {
      continue;
    }
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      tx_ok = true;
      if (can_tx_us != nullptr) {
        *can_tx_us = esp_timer_get_time();
      }
      break;
    }
    if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) {
      break;
    }
  }

  if (!tx_ok) {
    Serial.printf("%s position TX failed: no CAN ACK\n", motors[channel].name);
    print_status();
    stop_can();
    return false;
  }
  if (!wait_for_tx_success) {
    return true;
  }

  Serial.printf("%s position TX ACK: OK; waiting for motor 0xA4 reply\n",
                motors[channel].name);
  started = millis();
  while (millis() - started < 500) {
    twai_message_t rx = {};
    if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) {
      continue;
    }
    Serial.printf("%s ", motors[channel].name);
    print_frame("RX", rx);
    bool from_motor = rx.identifier == MOTOR_CAN_ID ||
                      rx.identifier == MOTOR_CAN_ID + 0x100;
    if (!rx.extd && !rx.rtr && from_motor &&
        rx.data_length_code == 8 &&
        rx.data[0] == POSITION_CONTROL_SPEED) {
      return true;
    }
  }

  Serial.printf("%s position command got CAN ACK but no 0xA4 motor reply\n",
                motors[channel].name);
  return false;
}

bool read_motor_state(size_t channel) {
  const uint8_t request[8] = {READ_STATE_1, 0, 0, 0, 0, 0, 0, 0};
  twai_message_t reply = {};
  if (!can_exchange(channel, request, READ_STATE_1, &reply)) {
    return false;
  }

  int8_t temperature = (int8_t)reply.data[1];
  uint16_t voltage_raw =
      (uint16_t)reply.data[2] | ((uint16_t)reply.data[3] << 8);
  Serial.printf("%s temp=%dC voltage=%.2fV error=0x%02X\n",
                motors[channel].name, temperature, voltage_raw * 0.01f,
                reply.data[7]);
  return true;
}

bool send_stop_to_channel(size_t channel) {
  const uint8_t stop_command[8] = {MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0};
  return can_exchange(channel, stop_command, MOTOR_STOP, nullptr);
}

void read_all_motor_states() {
  for (size_t channel = 0; channel < MOTOR_COUNT; ++channel) {
    read_motor_state(channel);
  }
}

bool run_motor(size_t channel, float speed_dps) {
  if (channel >= MOTOR_COUNT || !isfinite(speed_dps) ||
      speed_dps < MIN_SPEED_DPS || speed_dps > MAX_SPEED_DPS) {
    Serial.println("Speed must be finite and between -720 and 720 dps.");
    return false;
  }

  int32_t speed_control = (int32_t)lroundf(speed_dps * 100.0f);
  const uint8_t command[8] = {
      SPEED_CONTROL, 0, 0, 0,
      (uint8_t)(speed_control & 0xFF),
      (uint8_t)((speed_control >> 8) & 0xFF),
      (uint8_t)((speed_control >> 16) & 0xFF),
      (uint8_t)((speed_control >> 24) & 0xFF)};
  Serial.printf("Set %s to %.2f dps\n", motors[channel].name, speed_dps);
  if (!can_exchange(channel, command, SPEED_CONTROL, nullptr)) {
    Serial.printf("Speed command failed for %s.\n", motors[channel].name);
    return false;
  }

  motors[channel].commanded_speed_dps = speed_dps;
  motors[channel].position_target_initialized = false;
  return true;
}

bool read_motor_position(size_t channel, int64_t *position_cdeg) {
  const uint8_t request[8] = {READ_MULTI_TURN_ANGLE, 0, 0, 0, 0, 0, 0, 0};
  twai_message_t reply = {};
  if (!can_exchange(channel, request, READ_MULTI_TURN_ANGLE, &reply)) {
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
  Serial.printf("%s position=%.2f deg\n", motors[channel].name,
                *position_cdeg * 0.01f);
  return true;
}

bool move_motor_relative(size_t channel, float delta_degrees,
                         bool report_latency = false, uint32_t sequence = 0,
                         uint64_t imu_tx_ns = 0, uint64_t bridge_tx_ns = 0,
                         int64_t esp_rx_us = 0) {
  if (channel >= MOTOR_COUNT || !isfinite(delta_degrees) ||
      fabsf(delta_degrees) > 360000.0f) {
    Serial.println("Position command: angle must be finite and within +/-360000 deg.");
    return false;
  }

  int64_t delta_cdeg = (int64_t)llround((double)delta_degrees * 100.0);
  if (delta_cdeg == 0) {
    Serial.println("Position command ignored: magnitude is below 0.005 deg.");
    return true;
  }

  if (!motors[channel].motor_restored) {
    const uint8_t running_command[8] = {MOTOR_RUNNING, 0, 0, 0, 0, 0, 0, 0};
    if (!can_exchange(channel, running_command, MOTOR_RUNNING, nullptr)) {
      Serial.printf("Position command cancelled: cannot restore %s.\n",
                    motors[channel].name);
      return false;
    }
    motors[channel].motor_restored = true;
  }

  if (!motors[channel].position_target_initialized) {
    if (!read_motor_position(channel, &motors[channel].position_target_cdeg)) {
      Serial.printf("Position command cancelled: cannot read %s position.\n",
                    motors[channel].name);
      return false;
    }
    motors[channel].position_target_initialized = true;
  }

  int64_t target = motors[channel].position_target_cdeg + delta_cdeg;
  if (target < INT32_MIN || target > INT32_MAX) {
    Serial.printf("Position command cancelled: %s target outside protocol range.\n",
                  motors[channel].name);
    return false;
  }

  const int max_speed_dps = min(
      MAX_POSITION_SPEED_DPS,
      max(MIN_POSITION_SPEED_DPS, (int)ceilf(fabsf(delta_degrees))));
  const int32_t target_cdeg = (int32_t)target;
  int64_t esp_can_tx_us = 0;
  const uint8_t command[8] = {
      POSITION_CONTROL_SPEED,
      0,
      (uint8_t)(max_speed_dps & 0xFF),
      (uint8_t)((max_speed_dps >> 8) & 0xFF),
      (uint8_t)(target_cdeg & 0xFF),
      (uint8_t)((target_cdeg >> 8) & 0xFF),
      (uint8_t)((target_cdeg >> 16) & 0xFF),
      (uint8_t)((target_cdeg >> 24) & 0xFF)};
  if (!can_send_position_target(channel, command, true, &esp_can_tx_us)) {
    return false;
  }

  motors[channel].position_target_cdeg = target_cdeg;
  motors[channel].commanded_speed_dps = 0.0f;
  motors[channel].soft_stop_active = false;
  position_commands_sent++;
  if (report_latency) {
    Serial.printf("LAT,%lu,%llu,%llu,%lld,%lld\n", (unsigned long)sequence,
                  (unsigned long long)imu_tx_ns,
                  (unsigned long long)bridge_tx_ns, (long long)esp_rx_us,
                  (long long)esp_can_tx_us);
  }
  if (millis() - last_position_report_at >= 500) {
    Serial.printf("%s position target: delta=%.2f deg speed=%d dps sent=%lu\n",
                  motors[channel].name, delta_cdeg * 0.01f, max_speed_dps,
                  (unsigned long)position_commands_sent);
    position_commands_sent = 0;
    last_position_report_at = millis();
  }
  return true;
}

bool stop_motor_immediately(size_t channel) {
  if (channel >= MOTOR_COUNT) {
    return false;
  }
  Serial.printf("Stop %s\n", motors[channel].name);
  bool ok = send_stop_to_channel(channel);
  if (ok) {
    motors[channel].commanded_speed_dps = 0.0f;
    motors[channel].soft_stop_active = false;
    motors[channel].motor_restored = false;
  }
  return ok;
}

void begin_soft_stop(size_t channel) {
  if (channel >= MOTOR_COUNT) {
    return;
  }
  if (motors[channel].commanded_speed_dps == 0.0f) {
    stop_motor_immediately(channel);
    return;
  }
  motors[channel].soft_stop_active = true;
  motors[channel].next_soft_stop_step_at = millis() + 100;
  Serial.printf("Soft stop %s: %.2f dps, reduce by %d every 100 ms\n",
                motors[channel].name, motors[channel].commanded_speed_dps,
                SOFT_STOP_STEP_DPS);
}

void update_soft_stop() {
  for (size_t channel = 0; channel < MOTOR_COUNT; ++channel) {
    MotorChannel &motor = motors[channel];
    if (!motor.soft_stop_active ||
        (int32_t)(millis() - motor.next_soft_stop_step_at) < 0) {
      continue;
    }

    float next_speed = motor.commanded_speed_dps;
    if (next_speed > 0.0f) {
      next_speed = fmaxf(0.0f, next_speed - SOFT_STOP_STEP_DPS);
    } else {
      next_speed = fminf(0.0f, next_speed + SOFT_STOP_STEP_DPS);
    }
    if (!run_motor(channel, next_speed)) {
      Serial.printf("Soft-stop step failed for %s; sending immediate stop.\n",
                    motor.name);
      if (!stop_motor_immediately(channel)) {
        motor.soft_stop_active = true;
        motor.next_soft_stop_step_at = millis() + 100;
        Serial.printf("Stop not acknowledged for %s; soft-stop will retry.\n",
                      motor.name);
      }
      continue;
    }
    if (next_speed == 0.0f) {
      stop_motor_immediately(channel);
      continue;
    }
    motor.next_soft_stop_step_at = millis() + 100;
  }
}

int selected_motor(const char *token) {
  if (strcmp(token, "m1") == 0) {
    return 0;
  }
  if (strcmp(token, "m2") == 0) {
    return 1;
  }
  return -1;
}

void handle_command(char *line) {
  char motor_token[4] = {};
  char operation = 0;
  if (sscanf(line, " %3s %c", motor_token, &operation) != 2) {
    return;
  }

  int channel = selected_motor(motor_token);
  if (channel < 0) {
    Serial.println("Select a motor with m1 or m2.");
    return;
  }

  char extra = 0;
  if (operation == 'r' && sscanf(line, " %3s %c %c", motor_token,
                                 &operation, &extra) == 2) {
    if (read_motor_state((size_t)channel)) {
    int64_t position_cdeg = 0;
      read_motor_position((size_t)channel, &position_cdeg);
    }
    return;
  }
  if (operation == 's' && sscanf(line, " %3s %c %c", motor_token,
                                 &operation, &extra) == 2) {
    begin_soft_stop((size_t)channel);
    return;
  }

  if (operation == 'm') {
    float speed_dps = 0.0f;
    if (sscanf(line, " %3s %c %f %c", motor_token, &operation,
               &speed_dps, &extra) == 3 && isfinite(speed_dps) &&
        speed_dps >= MIN_SPEED_DPS && speed_dps <= MAX_SPEED_DPS) {
      motors[channel].soft_stop_active = false;
      run_motor((size_t)channel, speed_dps);
      return;
    }
  } else if (operation == 'n') {
    float delta_degrees = 0.0f;
    uint32_t sequence = 0;
    uint64_t imu_tx_ns = 0;
    uint64_t bridge_tx_ns = 0;
    int fields = sscanf(line, " %3s %c %f %lu %llu %llu %c", motor_token,
                        &operation, &delta_degrees, &sequence, &imu_tx_ns,
                        &bridge_tx_ns, &extra);
    if (fields == 3) {
      move_motor_relative((size_t)channel, delta_degrees);
      return;
    }
    if (fields == 6) {
      const int64_t esp_rx_us = esp_timer_get_time();
      move_motor_relative((size_t)channel, delta_degrees, true, sequence,
                          imu_tx_ns, bridge_tx_ns, esp_rx_us);
      return;
    }
  }
  Serial.println("Use: m1|m2 m <speed>, m1|m2 n <degrees>, m1|m2 s, or m1|m2 r.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Dual motor CAN test (one TWAI, switched channels)");
  Serial.println("MF5015V2: D5 TX, D6 RX");
  Serial.println("MF4015V2: D7 TX, D8 RX");
  Serial.println("Commands: m1/m2 selects one motor; no command is broadcast to both.");
  Serial.println("m1 m <dps> / m2 m <dps> = continuous speed, range -720..720");
  Serial.println("m1 n <degrees> / m2 n <degrees> = relative angle, nominally 1 second");
  Serial.println("m1 s / m2 s = reduce selected motor by 30 dps every 100 ms");
  Serial.println("m1 r / m2 r = read selected motor state");
  can_ready = start_can(0);
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

  if (command_length > 0 && millis() - last_command_byte_at >= 100) {
    command_buffer[command_length] = '\0';
    handle_command(command_buffer);
    command_length = 0;
  }
}
