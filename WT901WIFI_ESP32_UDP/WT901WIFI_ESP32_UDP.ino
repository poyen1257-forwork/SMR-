#include <WiFi.h>
#include <WiFiUdp.h>

// WT901WIFI must be configured to Station mode and "Specify user server".
// Set the sensor's server IP to this ESP32's IP and the server port below.
const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const uint16_t LISTEN_PORT = 1399;
const bool PRINT_RAW_UDP = true;
const uint8_t RAW_PREVIEW_BYTES = 64;

WiFiUDP udp;

struct WitMotionData {
  float accG[3] = {0.0f, 0.0f, 0.0f};
  float gyroDps[3] = {0.0f, 0.0f, 0.0f};
  float angleDeg[3] = {0.0f, 0.0f, 0.0f};
  float mag[3] = {0.0f, 0.0f, 0.0f};
  float temperatureC = 0.0f;
  uint32_t packetCount = 0;
  bool accUpdated = false;
  bool gyroUpdated = false;
  bool angleUpdated = false;
  bool magUpdated = false;
};

WitMotionData imu;

uint8_t frame[11];
uint8_t frameIndex = 0;
uint32_t lastPrintMs = 0;
uint32_t udpPacketCount = 0;
uint32_t udpByteCount = 0;
uint32_t validFrameCount = 0;
uint32_t checksumErrorCount = 0;

int16_t readInt16LE(const uint8_t lowByte, const uint8_t highByte) {
  return (int16_t)((uint16_t)lowByte | ((uint16_t)highByte << 8));
}

bool checksumOk(const uint8_t *packet) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 10; i++) {
    sum += packet[i];
  }
  return sum == packet[10];
}

void parseWitFrame(const uint8_t *packet) {
  const uint8_t type = packet[1];
  const int16_t x = readInt16LE(packet[2], packet[3]);
  const int16_t y = readInt16LE(packet[4], packet[5]);
  const int16_t z = readInt16LE(packet[6], packet[7]);
  const int16_t t = readInt16LE(packet[8], packet[9]);

  imu.packetCount++;
  validFrameCount++;

  switch (type) {
    case 0x51:  // Acceleration, unit: g
      imu.accG[0] = x / 32768.0f * 16.0f;
      imu.accG[1] = y / 32768.0f * 16.0f;
      imu.accG[2] = z / 32768.0f * 16.0f;
      imu.temperatureC = t / 100.0f;
      imu.accUpdated = true;
      break;

    case 0x52:  // Angular velocity, unit: deg/s
      imu.gyroDps[0] = x / 32768.0f * 2000.0f;
      imu.gyroDps[1] = y / 32768.0f * 2000.0f;
      imu.gyroDps[2] = z / 32768.0f * 2000.0f;
      imu.temperatureC = t / 100.0f;
      imu.gyroUpdated = true;
      break;

    case 0x53:  // Euler angle, unit: degree
      imu.angleDeg[0] = x / 32768.0f * 180.0f;
      imu.angleDeg[1] = y / 32768.0f * 180.0f;
      imu.angleDeg[2] = z / 32768.0f * 180.0f;
      imu.temperatureC = t / 100.0f;
      imu.angleUpdated = true;
      break;

    case 0x54:  // Magnetometer raw value
      imu.mag[0] = (float)x;
      imu.mag[1] = (float)y;
      imu.mag[2] = (float)z;
      imu.magUpdated = true;
      break;

    default:
      break;
  }
}

void feedWitByte(const uint8_t byteIn) {
  if (frameIndex == 0 && byteIn != 0x55) {
    return;
  }

  frame[frameIndex++] = byteIn;

  if (frameIndex == 2 && (frame[1] < 0x50 || frame[1] > 0x5F)) {
    frameIndex = (byteIn == 0x55) ? 1 : 0;
    return;
  }

  if (frameIndex < sizeof(frame)) {
    return;
  }

  if (checksumOk(frame)) {
    parseWitFrame(frame);
  } else {
    checksumErrorCount++;
  }

  frameIndex = 0;
}

void printImuData() {
  Serial.printf(
      "VALID frames %lu | UDP packets %lu | bytes %lu | checksum errors %lu\n",
      (unsigned long)validFrameCount,
      (unsigned long)udpPacketCount,
      (unsigned long)udpByteCount,
      (unsigned long)checksumErrorCount);

  Serial.printf(
      "ACC[g] %.3f, %.3f, %.3f | GYRO[dps] %.2f, %.2f, %.2f | ANG[deg] %.2f, %.2f, %.2f | MAG %.0f, %.0f, %.0f | T %.2f C\n",
      imu.accG[0], imu.accG[1], imu.accG[2],
      imu.gyroDps[0], imu.gyroDps[1], imu.gyroDps[2],
      imu.angleDeg[0], imu.angleDeg[1], imu.angleDeg[2],
      imu.mag[0], imu.mag[1], imu.mag[2],
      imu.temperatureC);

  imu.accUpdated = false;
  imu.gyroUpdated = false;
  imu.angleUpdated = false;
  imu.magUpdated = false;
}

void printUdpPreview(const int packetSize) {
  Serial.printf("UDP #%lu from %s:%u, size=%d, raw=",
                (unsigned long)udpPacketCount,
                udp.remoteIP().toString().c_str(),
                udp.remotePort(),
                packetSize);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  if (udp.begin(LISTEN_PORT)) {
    Serial.print("Listening UDP port: ");
    Serial.println(LISTEN_PORT);
  } else {
    Serial.println("UDP listen failed");
  }
}

void loop() {
  const int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    udpPacketCount++;
    uint8_t rawPrinted = 0;

    if (PRINT_RAW_UDP) {
      printUdpPreview(packetSize);
    }

    while (udp.available()) {
      const uint8_t byteIn = (uint8_t)udp.read();
      udpByteCount++;

      if (PRINT_RAW_UDP && rawPrinted < RAW_PREVIEW_BYTES) {
        if (byteIn < 0x10) {
          Serial.print("0");
        }
        Serial.print(byteIn, HEX);
        Serial.print(" ");
        rawPrinted++;
      }

      feedWitByte(byteIn);
    }

    if (PRINT_RAW_UDP) {
      if (packetSize > RAW_PREVIEW_BYTES) {
        Serial.print("...");
      }
      Serial.println();
    }
  }

  const bool hasFreshData = imu.accUpdated || imu.gyroUpdated || imu.angleUpdated || imu.magUpdated;
  if (hasFreshData && millis() - lastPrintMs >= 100) {
    lastPrintMs = millis();
    printImuData();
  }
}
