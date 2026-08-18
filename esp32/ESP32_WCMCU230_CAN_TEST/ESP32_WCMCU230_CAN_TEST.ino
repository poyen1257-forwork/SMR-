#include <Arduino.h>
#include "driver/twai.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "D6/D7 are UART0 pins. Enable USB CDC On Boot before using them for CAN."
#endif

// XIAO ESP32-S3 + WCMCU230
// D6/GPIO43 -> WCMCU230 CTX/TXD
// D7/GPIO44 <- WCMCU230 CRX/RXD
// WCMCU230 CANH -> motor CANH
// WCMCU230 CANL -> motor CANL

static const gpio_num_t CAN_TX_PIN = GPIO_NUM_43;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_44;
static_assert(D6 == 43, "Wrong board: XIAO ESP32-S3 D6 must be GPIO43");
static_assert(D7 == 44, "Wrong board: XIAO ESP32-S3 D7 must be GPIO44");

static const uint8_t MOTOR_ID = 1;
static const uint32_t MOTOR_CAN_ID = 0x140 + MOTOR_ID;  // 0x141

static const uint8_t CMD_STOP = 0x81;
static const uint8_t CMD_READ_STATE_1 = 0x9A;
static const uint8_t CMD_READ_STATE_2 = 0x9C;
static const uint8_t CMD_SPEED = 0xA2;

static const uint32_t CAN_TX_TIMEOUT_MS = 300;
static const uint32_t CAN_REPLY_TIMEOUT_MS = 700;
static const uint32_t COMMUNICATION_VALID_MS = 3000;

static bool can_ready = false;
static uint32_t last_valid_reply_ms = 0;

void release_can_pins_safely() {
  pinMode((uint8_t)CAN_TX_PIN, INPUT_PULLUP);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
}

bool prepare_can_pins_safely() {
  // Check the external levels while both pins are weak inputs. This prevents
  // output contention if CTX/CRX were accidentally swapped.
  release_can_pins_safely();
  delay(20);

  int tx_external = digitalRead((uint8_t)CAN_TX_PIN);
  int rx_external = digitalRead((uint8_t)CAN_RX_PIN);
  Serial.printf("GPIO input check: TX=%s RX=%s\n",
                tx_external == HIGH ? "HIGH" : "LOW",
                rx_external == HIGH ? "HIGH" : "LOW");

  if (tx_external != HIGH || rx_external != HIGH) {
    Serial.println("SAFETY LOCK: CTX/CRX logic level is LOW before TWAI starts");
    Serial.println("Check D6/D7 wiring, shorts, and WCMCU230 power");
    return false;
  }

  // Preload HIGH before OUTPUT. SN65HVD230 TXD HIGH is CAN recessive/idle.
  digitalWrite((uint8_t)CAN_TX_PIN, HIGH);
  pinMode((uint8_t)CAN_TX_PIN, OUTPUT);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
  gpio_set_drive_capability(CAN_TX_PIN, GPIO_DRIVE_CAP_0);
  delay(20);

  if (digitalRead((uint8_t)CAN_TX_PIN) != HIGH) {
    Serial.println("SAFETY LOCK: D6/GPIO43 cannot hold HIGH");
    release_can_pins_safely();
    return false;
  }

  return true;
}

void print_bytes(const uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    Serial.print(i + 1 == len ? '\n' : ' ');
  }
}

int16_t read_i16_le(const uint8_t *data, uint8_t index) {
  return (int16_t)((uint16_t)data[index] | ((uint16_t)data[index + 1] << 8));
}

uint16_t read_u16_le(const uint8_t *data, uint8_t index) {
  return (uint16_t)data[index] | ((uint16_t)data[index + 1] << 8);
}

bool is_motor_reply(uint32_t identifier) {
  return identifier == MOTOR_CAN_ID || identifier == (0x240 + MOTOR_ID);
}

void print_twai_status() {
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    return;
  }

  Serial.printf(
      "TWAI state=%d tx_queue=%lu rx_queue=%lu tx_error=%lu rx_error=%lu "
      "tx_failed=%lu rx_missed=%lu bus_error=%lu\n",
      status.state,
      status.msgs_to_tx,
      status.msgs_to_rx,
      status.tx_error_counter,
      status.rx_error_counter,
      status.tx_failed_count,
      status.rx_missed_count,
      status.bus_error_count);
}

void print_motor_data(const twai_message_t &rx) {
  if (!is_motor_reply(rx.identifier) || rx.data_length_code != 8) {
    return;
  }

  if (rx.data[0] == CMD_READ_STATE_1) {
    int8_t temp_c = (int8_t)rx.data[1];
    uint16_t voltage_raw = read_u16_le(rx.data, 2);
    Serial.printf("STATE1 temp=%dC voltage=%.2fV state=0x%02X error=0x%02X\n",
                  temp_c,
                  voltage_raw * 0.01f,
                  rx.data[6],
                  rx.data[7]);
  }

  if (rx.data[0] == CMD_READ_STATE_2 || rx.data[0] == CMD_SPEED) {
    int8_t temp_c = (int8_t)rx.data[1];
    int16_t current_raw = read_i16_le(rx.data, 2);
    int16_t speed_dps = read_i16_le(rx.data, 4);
    uint16_t encoder = read_u16_le(rx.data, 6);
    Serial.printf("STATE2 temp=%dC current_raw=%d speed=%d_dps encoder=%u\n",
                  temp_c,
                  current_raw,
                  speed_dps,
                  encoder);
  }
}

bool receive_can(uint8_t expected_command, uint32_t duration_ms) {
  bool got_reply = false;
  uint32_t start = millis();

  while (millis() - start < duration_ms) {
    twai_message_t rx = {};
    esp_err_t result = twai_receive(&rx, pdMS_TO_TICKS(20));
    if (result != ESP_OK) {
      continue;
    }

    Serial.printf("RX id=0x%03lX dlc=%u data=",
                  (unsigned long)rx.identifier,
                  rx.data_length_code);
    print_bytes(rx.data, rx.data_length_code);
    print_motor_data(rx);

    if (!rx.extd && !rx.rtr && is_motor_reply(rx.identifier) &&
        rx.data_length_code == 8 && rx.data[0] == expected_command) {
      got_reply = true;
      break;
    }
  }

  return got_reply;
}

// Big function 1: communication.
// Send one CAN frame, then receive CAN frames for timeout_ms.
bool can_exchange(const uint8_t tx_data[8], uint32_t timeout_ms) {
  if (!can_ready) {
    Serial.println("CAN safety lock is active; frame not sent");
    return false;
  }

  twai_status_info_t initial_status = {};
  if (twai_get_status_info(&initial_status) != ESP_OK ||
      initial_status.state != TWAI_STATE_RUNNING) {
    Serial.println("CAN safety lock: TWAI is not running");
    return false;
  }

  twai_clear_receive_queue();

  uint32_t old_alerts = 0;
  twai_read_alerts(&old_alerts, 0);

  twai_message_t tx = {};
  tx.identifier = MOTOR_CAN_ID;
  tx.extd = 0;
  tx.rtr = 0;
  tx.ss = 1;  // One attempt only; run_command() performs the controlled retry.
  tx.data_length_code = 8;
  memcpy(tx.data, tx_data, 8);

  Serial.printf("TX id=0x%03lX data=", (unsigned long)tx.identifier);
  print_bytes(tx.data, tx.data_length_code);

  esp_err_t result = twai_transmit(&tx, pdMS_TO_TICKS(150));
  if (result != ESP_OK) {
    Serial.printf("TX failed: %d\n", result);
    twai_clear_transmit_queue();
    return false;
  }

  bool tx_success = false;
  uint32_t tx_start = millis();
  while (millis() - tx_start < CAN_TX_TIMEOUT_MS) {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(20)) != ESP_OK) {
      continue;
    }

    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      tx_success = true;
      break;
    }

    if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) {
      break;
    }
  }

  if (!tx_success) {
    Serial.println("TX was not ACKed by the CAN bus");
    twai_clear_transmit_queue();
    print_twai_status();
    return false;
  }

  Serial.println("TX ACK: OK");

  bool got_reply = receive_can(tx_data[0], timeout_ms);
  if (got_reply &&
      (tx_data[0] == CMD_READ_STATE_1 || tx_data[0] == CMD_READ_STATE_2)) {
    last_valid_reply_ms = millis();
  }
  if (!got_reply) {
    Serial.printf("RX timeout waiting for command 0x%02X\n", tx_data[0]);
    print_twai_status();
  }

  return got_reply;
}

void make_read_frame(uint8_t command, uint8_t out[8]) {
  memset(out, 0, 8);
  out[0] = command;
}

void make_speed_frame(float dps, uint8_t out[8]) {
  memset(out, 0, 8);
  int32_t raw = (int32_t)lroundf(dps * 100.0f);  // 0.01 dps/LSB
  out[0] = CMD_SPEED;
  out[4] = (uint8_t)(raw & 0xFF);
  out[5] = (uint8_t)((raw >> 8) & 0xFF);
  out[6] = (uint8_t)((raw >> 16) & 0xFF);
  out[7] = (uint8_t)((raw >> 24) & 0xFF);
}

void make_stop_frame(uint8_t out[8]) {
  memset(out, 0, 8);
  out[0] = CMD_STOP;
}

// Big function 2: command.
// Convert Serial command r/s/m into CAN data for can_exchange().
void run_command(char command) {
  if (command == '\r' || command == '\n' || command == ' ') {
    return;
  }

  uint8_t frame[8] = {};

  if (command == 'r') {
    Serial.println("Command r: read motor state");
    make_read_frame(CMD_READ_STATE_1, frame);
    if (!can_exchange(frame, CAN_REPLY_TIMEOUT_MS)) {
      delay(50);
      can_exchange(frame, CAN_REPLY_TIMEOUT_MS);
    }
    make_read_frame(CMD_READ_STATE_2, frame);
    if (!can_exchange(frame, CAN_REPLY_TIMEOUT_MS)) {
      delay(50);
      can_exchange(frame, CAN_REPLY_TIMEOUT_MS);
    }
    return;
  }

  if (command == 's') {
    Serial.println("Command s: stop motor");
    make_stop_frame(frame);
    can_exchange(frame, 1000);
    return;
  }

  if (command == 'm') {
    if (last_valid_reply_ms == 0 ||
        millis() - last_valid_reply_ms > COMMUNICATION_VALID_MS) {
      Serial.println("SAFETY LOCK: run r and receive a recent valid reply first");
      return;
    }

    Serial.println("Command m: low speed 30 dps");
    make_speed_frame(30.0f, frame);
    can_exchange(frame, 500);
    delay(500);
    make_stop_frame(frame);
    can_exchange(frame, 500);
    return;
  }

  Serial.println("Unknown command");
}

bool setup_can() {
  if (!prepare_can_pins_safely()) {
    return false;
  }

  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 1;
  general_config.rx_queue_len = 20;

  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(&general_config, &timing_config, &filter_config);
  if (result != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", result);
    release_can_pins_safely();
    return false;
  }

  result = twai_start();
  if (result != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", result);
    twai_driver_uninstall();
    release_can_pins_safely();
    return false;
  }

  gpio_set_drive_capability(CAN_TX_PIN, GPIO_DRIVE_CAP_0);
  delay(20);
  if (digitalRead((uint8_t)CAN_TX_PIN) != HIGH) {
    Serial.println("SAFETY LOCK: TWAI TX idle level is LOW");
    twai_stop();
    twai_driver_uninstall();
    release_can_pins_safely();
    return false;
  }

  uint32_t alerts = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                    TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;
  result = twai_reconfigure_alerts(alerts, nullptr);
  if (result != ESP_OK) {
    Serial.printf("twai_reconfigure_alerts failed: %d\n", result);
    twai_stop();
    twai_driver_uninstall();
    release_can_pins_safely();
    return false;
  }

  Serial.println("TWAI/CAN started at 1 Mbps");
  print_twai_status();
  return true;
}

void print_help() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  r = read motor state");
  Serial.println("  m = low speed 30 dps");
  Serial.println("  s = stop motor");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("MF4015V2 CAN test");
  Serial.println("CAN: 1 Mbps, ID=1, CAN ID=0x141");
  Serial.println("Pins: D6/GPIO43->CTX, D7/GPIO44<-CRX");
  can_ready = setup_can();
  if (!can_ready) {
    Serial.println("CAN disabled. No frame can be transmitted");
  }
  print_help();
}

void loop() {
  twai_message_t rx = {};
  if (twai_receive(&rx, 0) == ESP_OK) {
    Serial.printf("RX id=0x%03lX dlc=%u data=",
                  (unsigned long)rx.identifier,
                  rx.data_length_code);
    print_bytes(rx.data, rx.data_length_code);
    print_motor_data(rx);
  }

  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();
  while (Serial.available()) {
    Serial.read();
  }

  run_command(command);
}
