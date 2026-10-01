#include <Arduino.h>
#include "driver/twai.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Enable USB CDC On Boot before using D6/D7 for CAN."
#endif

// XIAO ESP32-S3 + WCMCU230
// D6 / GPIO43 -> WCMCU230 CTX / TXD
// D7 / GPIO44 <- WCMCU230 CRX / RXD
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_43;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_44;
static_assert(D6 == 43, "Select the XIAO ESP32-S3 board");
static_assert(D7 == 44, "Select the XIAO ESP32-S3 board");

static const uint32_t MOTOR_CAN_ID = 0x141;  // Motor ID 1
static const uint8_t CMD_READ_STATE_1 = 0x9A;

static bool can_ready = false;
static uint32_t current_bitrate = 0;

void print_twai_status() {
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    Serial.println("Cannot read TWAI status");
    return;
  }

  Serial.printf(
      "TWAI state=%d tx_queue=%lu rx_queue=%lu tx_error=%lu rx_error=%lu "
      "tx_failed=%lu bus_error=%lu\n",
      status.state,
      status.msgs_to_tx,
      status.msgs_to_rx,
      status.tx_error_counter,
      status.rx_error_counter,
      status.tx_failed_count,
      status.bus_error_count);
}

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

void release_can_pins() {
  pinMode((uint8_t)CAN_TX_PIN, INPUT_PULLUP);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
}

bool start_can(uint32_t bitrate) {
  // Check both logic lines before TWAI changes TX into an output.
  release_can_pins();
  delay(20);

  if (digitalRead((uint8_t)CAN_TX_PIN) == LOW ||
      digitalRead((uint8_t)CAN_RX_PIN) == LOW) {
    Serial.println("CAN not started: D6 or D7 is LOW");
    Serial.println("Check CTX/CRX wiring and WCMCU230 3.3V/GND");
    return false;
  }

  // SN65HVD230 TXD HIGH means CAN recessive (idle).
  digitalWrite((uint8_t)CAN_TX_PIN, HIGH);
  pinMode((uint8_t)CAN_TX_PIN, OUTPUT);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
  gpio_set_drive_capability(CAN_TX_PIN, GPIO_DRIVE_CAP_0);

  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  general.tx_queue_len = 1;
  general.rx_queue_len = 10;

  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  switch (bitrate) {
    case 100000:
      timing = TWAI_TIMING_CONFIG_100KBITS();
      break;
    case 125000:
      timing = TWAI_TIMING_CONFIG_125KBITS();
      break;
    case 250000:
      timing = TWAI_TIMING_CONFIG_250KBITS();
      break;
    case 500000:
      timing = TWAI_TIMING_CONFIG_500KBITS();
      break;
    case 1000000:
      timing = TWAI_TIMING_CONFIG_1MBITS();
      break;
    default:
      Serial.println("Unsupported CAN bitrate");
      release_can_pins();
      return false;
  }
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(&general, &timing, &filter);
  if (result != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", result);
    release_can_pins();
    return false;
  }

  result = twai_start();
  if (result != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", result);
    twai_driver_uninstall();
    release_can_pins();
    return false;
  }

  uint32_t enabled_alerts =
      TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF |
      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS;
  result = twai_reconfigure_alerts(enabled_alerts, nullptr);
  if (result != ESP_OK) {
    Serial.printf("twai_reconfigure_alerts failed: %d\n", result);
    twai_stop();
    twai_driver_uninstall();
    release_can_pins();
    return false;
  }

  current_bitrate = bitrate;
  Serial.printf("CAN started: %lu bps, D6=TX, D7=RX\n",
                (unsigned long)bitrate);
  print_twai_status();
  return true;
}

void stop_can() {
  if (can_ready) {
    twai_stop();
    twai_driver_uninstall();
  }
  can_ready = false;
  current_bitrate = 0;
  release_can_pins();
  delay(100);
}

bool send_motor_status_request() {
  if (!can_ready) {
    Serial.println("CAN is not ready");
    return false;
  }

  twai_message_t tx = {};
  tx.identifier = MOTOR_CAN_ID;
  tx.data_length_code = 8;
  tx.ss = 1;  // One attempt; do not continuously retry an unacknowledged frame.
  tx.data[0] = CMD_READ_STATE_1;

  uint32_t old_alerts = 0;
  twai_read_alerts(&old_alerts, 0);

  print_frame("TX", tx);
  esp_err_t result = twai_transmit(&tx, pdMS_TO_TICKS(100));
  if (result != ESP_OK) {
    Serial.printf("TX queue failed: %d\n", result);
    print_twai_status();
    return false;
  }

  uint32_t alerts = 0;
  result = twai_read_alerts(&alerts, pdMS_TO_TICKS(300));
  if (result != ESP_OK) {
    Serial.printf("TX result timeout/error: %d\n", result);
    print_twai_status();
    return false;
  }

  if (alerts & TWAI_ALERT_BUS_OFF) {
    Serial.println("TX failed: CAN controller entered BUS-OFF");
    can_ready = false;
    print_twai_status();
    return false;
  }

  if (alerts & TWAI_ALERT_TX_FAILED) {
    Serial.println("TX failed: frame was not ACKed or CAN bus has a physical error");
    print_twai_status();
    return false;
  }

  if (!(alerts & TWAI_ALERT_TX_SUCCESS)) {
    Serial.printf("TX failed: unexpected TWAI alerts=0x%08lX\n",
                  (unsigned long)alerts);
    print_twai_status();
    return false;
  }

  Serial.println("TX ACK: OK");
  return true;
}

bool receive_can_frame() {
  if (!can_ready) {
    return false;
  }

  twai_message_t rx = {};
  if (twai_receive(&rx, pdMS_TO_TICKS(20)) == ESP_OK) {
    print_frame("RX", rx);
    return true;
  }
  return false;
}

void scan_can_bitrates() {
  const uint32_t bitrates[] = {1000000, 500000, 250000, 125000, 100000};

  Serial.println("Scanning supported CAN bitrates with read-only command 0x9A");
  for (uint32_t bitrate : bitrates) {
    stop_can();
    Serial.printf("--- Try %lu bps ---\n", (unsigned long)bitrate);
    can_ready = start_can(bitrate);
    if (!can_ready || !send_motor_status_request()) {
      continue;
    }

    uint32_t started = millis();
    while (millis() - started < 1000) {
      if (receive_can_frame()) {
        Serial.printf("RX FOUND at %lu bps\n", (unsigned long)bitrate);
        return;
      }
    }
    Serial.println("Frame was ACKed, but no motor reply was received");
  }

  Serial.println("No ACK/RX at any supported bitrate");
}

void test_transceiver_echo() {
  stop_can();

  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLUP);
  delay(2);
  int rx_with_pullup = digitalRead((uint8_t)CAN_RX_PIN);
  pinMode((uint8_t)CAN_RX_PIN, INPUT_PULLDOWN);
  delay(2);
  int rx_with_pulldown = digitalRead((uint8_t)CAN_RX_PIN);
  pinMode((uint8_t)CAN_RX_PIN, INPUT);

  digitalWrite((uint8_t)CAN_TX_PIN, HIGH);
  pinMode((uint8_t)CAN_TX_PIN, OUTPUT);
  gpio_set_drive_capability(CAN_TX_PIN, GPIO_DRIVE_CAP_0);
  delay(2);

  int tx_idle_level = digitalRead((uint8_t)CAN_TX_PIN);
  int idle_level = digitalRead((uint8_t)CAN_RX_PIN);
  digitalWrite((uint8_t)CAN_TX_PIN, LOW);
  delayMicroseconds(20);
  int tx_dominant_level = digitalRead((uint8_t)CAN_TX_PIN);
  int dominant_level = digitalRead((uint8_t)CAN_RX_PIN);
  digitalWrite((uint8_t)CAN_TX_PIN, HIGH);
  delay(2);
  int tx_recovered_level = digitalRead((uint8_t)CAN_TX_PIN);
  int recovered_level = digitalRead((uint8_t)CAN_RX_PIN);

  Serial.printf("D7 connection check: pullup=%d pulldown=%d\n",
                rx_with_pullup,
                rx_with_pulldown);
  Serial.printf("ESP32 D6: idle=%d dominant=%d recovered=%d\n",
                tx_idle_level,
                tx_dominant_level,
                tx_recovered_level);
  Serial.printf("WCMCU230 D7: idle=%d dominant=%d recovered=%d\n",
                idle_level,
                dominant_level,
                recovered_level);
  if (idle_level == HIGH && dominant_level == LOW &&
      recovered_level == HIGH) {
    Serial.println("WCMCU230 TX->CAN->RX path: PASS");
  } else {
    Serial.println("WCMCU230 TX->CAN->RX path: FAIL");
  }

  release_can_pins();
  can_ready = start_can(1000000);
}

void print_help() {
  Serial.println();
  Serial.println("MF4015V2 basic CAN communication test");
  Serial.println("  r = send read-only motor status command 0x9A");
  Serial.println("  b = scan 1M/500k/250k/125k/100k using read-only 0x9A");
  Serial.println("  e = test WCMCU230 TX-to-RX electrical echo");
  Serial.println("  h = show help");
  Serial.println("No CAN frame is sent automatically.");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  print_help();
  can_ready = start_can(1000000);
}

void loop() {
  receive_can_frame();

  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();
  while (Serial.available()) {
    Serial.read();
  }

  if (command == 'r') {
    send_motor_status_request();
  } else if (command == 'b') {
    scan_can_bitrates();
  } else if (command == 'e') {
    test_transceiver_echo();
  } else if (command == 'h' || command == '?') {
    print_help();
  }
}
