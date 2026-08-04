#include <Arduino.h>
#include "driver/twai.h"

// Seeed Studio XIAO ESP32-S3:
// D4 = GPIO5, D5 = GPIO6.
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_5;  // XIAO D4 -> WCMCU230 CTX/TXD
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_6;  // XIAO D5 -> WCMCU230 CRX/RXD

static const uint32_t TEST_CAN_ID = 0x123;
static uint32_t tx_count = 0;
static uint32_t rx_count = 0;

void print_frame(const char *prefix, const twai_message_t &msg) {
  Serial.printf("%s id=0x%03lX dlc=%d data=", prefix, (unsigned long)msg.identifier, msg.data_length_code);
  for (int i = 0; i < msg.data_length_code; i++) {
    if (msg.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(msg.data[i], HEX);
    Serial.print(i + 1 == msg.data_length_code ? '\n' : ' ');
  }
}

void setup_twai_loopback() {
  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
  general_config.tx_queue_len = 10;
  general_config.rx_queue_len = 20;
  general_config.self_test = true;

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

  Serial.println("TWAI self-test loopback started at 1 Mbps.");
}

bool send_test_frame() {
  twai_message_t msg = {};
  msg.identifier = TEST_CAN_ID;
  msg.extd = 0;
  msg.rtr = 0;
  msg.self = 1;
  msg.data_length_code = 8;

  msg.data[0] = 0xA5;
  msg.data[1] = 0x5A;
  msg.data[2] = (uint8_t)(tx_count & 0xFF);
  msg.data[3] = (uint8_t)((tx_count >> 8) & 0xFF);
  msg.data[4] = (uint8_t)((tx_count >> 16) & 0xFF);
  msg.data[5] = (uint8_t)((tx_count >> 24) & 0xFF);
  msg.data[6] = CAN_TX_PIN;
  msg.data[7] = CAN_RX_PIN;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  if (result != ESP_OK) {
    Serial.printf("TX failed: %d\n", result);
    return false;
  }

  tx_count++;
  print_frame("TX", msg);
  return true;
}

void receive_for(uint32_t duration_ms) {
  uint32_t start = millis();
  while (millis() - start < duration_ms) {
    twai_message_t msg = {};
    esp_err_t result = twai_receive(&msg, pdMS_TO_TICKS(20));
    if (result != ESP_OK) {
      continue;
    }

    rx_count++;
    print_frame("RX", msg);

    bool ok = msg.identifier == TEST_CAN_ID &&
              msg.data_length_code == 8 &&
              msg.data[0] == 0xA5 &&
              msg.data[1] == 0x5A;
    Serial.println(ok ? "LOOPBACK OK" : "LOOPBACK DATA MISMATCH");
  }
}

void print_help() {
  Serial.println();
  Serial.println("ESP32 + WCMCU230 loopback test");
  Serial.println("Board: Seeed Studio XIAO ESP32-S3");
  Serial.println("Default pins:");
  Serial.println("  XIAO D4 / GPIO5 -> WCMCU230 CTX/TXD");
  Serial.println("  XIAO D5 / GPIO6 -> WCMCU230 CRX/RXD");
  Serial.println("  XIAO 3V3        -> WCMCU230 3V3");
  Serial.println("  XIAO GND        -> WCMCU230 GND");
  Serial.println();
  Serial.println("This sketch uses TWAI self-test loopback.");
  Serial.println("No motor is required.");
  Serial.println("Type t in Serial Monitor to send one test frame.");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  print_help();
  setup_twai_loopback();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) {
      Serial.read();
    }
    if (c == 't') {
      send_test_frame();
      receive_for(300);
    } else if (c == 'h' || c == '?') {
      print_help();
    }
  }

  static uint32_t last_auto_ms = 0;
  if (millis() - last_auto_ms >= 1000) {
    last_auto_ms = millis();
    send_test_frame();
    receive_for(100);
    Serial.printf("summary: tx=%lu rx=%lu\n", (unsigned long)tx_count, (unsigned long)rx_count);
  }
}
