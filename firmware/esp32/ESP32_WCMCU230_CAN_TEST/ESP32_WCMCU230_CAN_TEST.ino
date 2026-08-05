#include <Arduino.h>
#include "driver/twai.h"

// Seeed Studio XIAO ESP32-S3:
// D4 = GPIO5, D5 = GPIO6.
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_5;  // XIAO D4 -> WCMCU230 CTX/TXD
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_6;  // XIAO D5 -> WCMCU230 CRX/RXD

// MF5015-V2 / MF-RMD common CAN settings.
static const uint8_t MOTOR_ID = 1;
static const uint32_t MOTOR_TX_ID = 0x140 + MOTOR_ID;
static const uint32_t MOTOR_REPLY_ID_1 = 0x240 + MOTOR_ID;
static const uint32_t MOTOR_REPLY_ID_2 = 0x140 + MOTOR_ID;

static const uint8_t CMD_MOTOR_OFF = 0x80;
static const uint8_t CMD_MOTOR_STOP = 0x81;
static const uint8_t CMD_MOTOR_RUNNING = 0x88;
static const uint8_t CMD_READ_STATUS_2 = 0x9C;
static const uint8_t CMD_SPEED_CLOSED_LOOP = 0xA2;

static bool auto_status = false;
static uint32_t last_status_ms = 0;

int16_t read_int16_le(const uint8_t *data, int offset) {
  return (int16_t)((uint16_t)data[offset] | ((uint16_t)data[offset + 1] << 8));
}

void print_frame(const twai_message_t &msg) {
  Serial.printf("RX id=0x%03lX dlc=%d data=", (unsigned long)msg.identifier, msg.data_length_code);
  for (int i = 0; i < msg.data_length_code; i++) {
    if (msg.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(msg.data[i], HEX);
    Serial.print(i + 1 == msg.data_length_code ? '\n' : ' ');
  }
}

void print_twai_status() {
  twai_status_info_t status = {};
  esp_err_t result = twai_get_status_info(&status);
  if (result != ESP_OK) {
    Serial.printf("twai_get_status_info failed: %d\n", result);
    return;
  }

  Serial.printf(
      "TWAI status: state=%d tx_queue=%lu rx_queue=%lu tx_err=%lu rx_err=%lu "
      "tx_failed=%lu bus_error=%lu\n",
      (int)status.state,
      (unsigned long)status.msgs_to_tx,
      (unsigned long)status.msgs_to_rx,
      (unsigned long)status.tx_error_counter,
      (unsigned long)status.rx_error_counter,
      (unsigned long)status.tx_failed_count,
      (unsigned long)status.bus_error_count);
}

void print_status_2(const twai_message_t &msg) {
  if (msg.data_length_code != 8 || msg.data[0] != CMD_READ_STATUS_2) {
    return;
  }

  int8_t temperature_c = (int8_t)msg.data[1];
  int16_t iq_raw = read_int16_le(msg.data, 2);
  int16_t speed_dps = read_int16_le(msg.data, 4);
  int16_t encoder_raw = read_int16_le(msg.data, 6);

  Serial.println("MF5015 status:");
  Serial.printf("  temperature: %d C\n", temperature_c);
  Serial.printf("  iq_raw:      %d\n", iq_raw);
  Serial.printf("  iq_approx:   %.2f A using 0.01A/LSB protocol scale\n", iq_raw * 0.01f);
  Serial.printf("  speed:       %d dps\n", speed_dps);
  Serial.printf("  encoder:     %d\n", encoder_raw);
}

bool send_frame(uint8_t command, const uint8_t payload[7]) {
  twai_message_t msg = {};
  msg.identifier = MOTOR_TX_ID;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;
  msg.data[0] = command;

  for (int i = 0; i < 7; i++) {
    msg.data[i + 1] = payload[i];
  }

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  if (result != ESP_OK) {
    Serial.printf("TX failed: %d\n", result);
    if (result == ESP_ERR_TIMEOUT) {
      Serial.println("  ESP_ERR_TIMEOUT: CAN frame was not transmitted.");
      Serial.println("  Most common cause: no CAN ACK from another powered node.");
      Serial.println("  Check motor power, CANH/CANL wiring, common GND, baudrate, and termination.");
    }
    print_twai_status();
    return false;
  }

  Serial.printf("TX id=0x%03lX cmd=0x%02X\n", (unsigned long)msg.identifier, command);
  return true;
}

bool send_simple_command(uint8_t command) {
  uint8_t payload[7] = {0, 0, 0, 0, 0, 0, 0};
  return send_frame(command, payload);
}

bool send_speed_dps(float dps) {
  int32_t raw = (int32_t)lroundf(dps * 100.0f);  // 0.01 dps/LSB
  uint8_t payload[7] = {0, 0, 0, 0, 0, 0, 0};

  payload[3] = (uint8_t)(raw & 0xFF);
  payload[4] = (uint8_t)((raw >> 8) & 0xFF);
  payload[5] = (uint8_t)((raw >> 16) & 0xFF);
  payload[6] = (uint8_t)((raw >> 24) & 0xFF);

  return send_frame(CMD_SPEED_CLOSED_LOOP, payload);
}

void receive_frames(uint32_t duration_ms) {
  uint32_t start = millis();
  while (millis() - start < duration_ms) {
    twai_message_t rx_msg = {};
    esp_err_t result = twai_receive(&rx_msg, pdMS_TO_TICKS(20));
    if (result != ESP_OK) {
      continue;
    }

    print_frame(rx_msg);

    if (rx_msg.identifier == MOTOR_REPLY_ID_1 || rx_msg.identifier == MOTOR_REPLY_ID_2) {
      print_status_2(rx_msg);
    }
  }
}

void print_help() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h = help");
  Serial.println("  s = read MF5015 status 2 once");
  Serial.println("  a = toggle auto status read every 1 second");
  Serial.println("  r = motor running command");
  Serial.println("  x = motor stop command");
  Serial.println("  o = motor off command");
  Serial.println("  m = low speed test: 30 dps for 1 second, then stop");
  Serial.println();
  Serial.println("Default behavior: idle. It does not move the motor and does not spam CAN.");
  Serial.println("First test: type s and confirm RX/MF5015 status before using m.");
  Serial.println();
}

void handle_serial_command() {
  if (!Serial.available()) {
    return;
  }

  char c = Serial.read();
  while (Serial.available()) {
    Serial.read();
  }

  if (c == 'h' || c == '?') {
    print_help();
  } else if (c == 's') {
    send_simple_command(CMD_READ_STATUS_2);
  } else if (c == 'a') {
    auto_status = !auto_status;
    Serial.printf("auto status: %s\n", auto_status ? "ON" : "OFF");
  } else if (c == 'r') {
    send_simple_command(CMD_MOTOR_RUNNING);
  } else if (c == 'x') {
    send_simple_command(CMD_MOTOR_STOP);
  } else if (c == 'o') {
    send_simple_command(CMD_MOTOR_OFF);
  } else if (c == 'm') {
    Serial.println("Low speed test: 30 dps for 1 second.");
    send_speed_dps(30.0f);
    receive_frames(1000);
    send_simple_command(CMD_MOTOR_STOP);
  } else {
    Serial.println("Unknown command. Type h for help.");
  }
}

void setup_can() {
  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 10;
  general_config.rx_queue_len = 20;

  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(&general_config, &timing_config, &filter_config);
  if (result != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", result);
    while (true) {
      delay(1000);
    }
  }

  result = twai_start();
  if (result != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", result);
    while (true) {
      delay(1000);
    }
  }

  Serial.println("TWAI/CAN started at 1 Mbps.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 + WCMCU230/SN65HVD230 CAN test for MF5015-V2");
  Serial.println("Wiring:");
  Serial.println("  ESP32 3V3  -> WCMCU230 VCC");
  Serial.println("  ESP32 GND  -> WCMCU230 GND -> motor GND");
  Serial.println("  ESP32 GPIO5 -> WCMCU230 CTX/TXD");
  Serial.println("  ESP32 GPIO6 -> WCMCU230 CRX/RXD");
  Serial.println("  WCMCU230 CAN_H -> motor CAN_H");
  Serial.println("  WCMCU230 CAN_L -> motor CAN_L");
  Serial.println("  External 16V + -> motor V+");
  Serial.println("  External 16V - -> motor GND");

  setup_can();
  print_help();
}

void loop() {
  handle_serial_command();
  receive_frames(10);

  if (auto_status && millis() - last_status_ms >= 1000) {
    last_status_ms = millis();
    send_simple_command(CMD_READ_STATUS_2);
  }
}
