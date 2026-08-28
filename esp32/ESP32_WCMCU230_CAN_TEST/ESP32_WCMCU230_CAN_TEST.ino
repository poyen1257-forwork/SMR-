#include <Arduino.h>
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
static const uint8_t READ_STATE_1 = 0x9A;
static const uint8_t SPEED_CONTROL = 0xA2;
static const uint8_t MOTOR_STOP = 0x81;

static const int MIN_SPEED_DPS = 0;
static const int MAX_SPEED_DPS = 360;

static bool can_ready = false;
static bool soft_stop_active = false;
static uint32_t next_soft_stop_step_at = 0;
static int commanded_speed_dps = 0;
static char command_buffer[32] = {};
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
  tx.ss = 1;
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
    if (!rx.extd && !rx.rtr && rx.identifier == MOTOR_CAN_ID &&
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
  if (commanded_speed_dps <= 0) {
    stop_motor_immediately();
    return;
  }

  soft_stop_active = true;
  next_soft_stop_step_at = millis();
  Serial.printf("Soft stop: %d dps, decrease 10 dps every 100 ms\n",
                commanded_speed_dps);
}

void update_soft_stop() {
  if (!soft_stop_active ||
      (int32_t)(millis() - next_soft_stop_step_at) < 0) {
    return;
  }

  int next_speed = commanded_speed_dps - 10;
  if (next_speed < 0) {
    next_speed = 0;
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
    Serial.println("Speed must be an integer from 0 to 360 dps.");
    return false;
  }
  return true;
}

void handle_command(char *line) {
  char command = 0;
  int value = 0;
  char extra = 0;
  int fields = sscanf(line, " %c %d %c", &command, &value, &extra);

  if (fields == 1 && command == 'r') {
    read_motor_state();
    return;
  }
  if (fields == 1 && command == 's') {
    begin_soft_stop();
    return;
  }
  if (fields == 2 && command == 'm' && valid_speed(value)) {
    soft_stop_active = false;
    run_motor(value);
    return;
  }
  Serial.println("Invalid command. Use r, m 0..360, or s.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("MF4015V2 CAN test");
  Serial.println("r = read motor state");
  Serial.println("m <0..360> = run continuously at the given dps");
  Serial.println("s = soft stop (minus 10 dps every 100 ms)");
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
