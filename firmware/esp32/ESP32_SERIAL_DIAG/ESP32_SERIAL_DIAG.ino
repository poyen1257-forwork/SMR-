#include <Arduino.h>

static uint32_t count = 0;

void setup() {
  Serial.begin(115200);
  delay(3000);
}

void loop() {
  Serial.printf("serial diag ok, count=%lu, millis=%lu\n",
                (unsigned long)count,
                (unsigned long)millis());
  count++;
  delay(1000);
}
