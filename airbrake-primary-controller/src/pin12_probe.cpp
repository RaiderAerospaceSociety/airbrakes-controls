#ifdef PIN12_PROBE
#include <Arduino.h>

#ifndef PROBE_PIN
#define PROBE_PIN 12
#endif

// PROBE_MODE:
// 0 = force LOW (0 V)
// 1 = force HIGH (~3.3 V)
// 2 = blink at ~1 Hz (multimeter sees average, scope sees square wave)
#ifndef PROBE_MODE
#define PROBE_MODE 2
#endif

#ifndef PROBE_BLINK_MS
#define PROBE_BLINK_MS 500
#endif

void setup() {
  pinMode(PROBE_PIN, OUTPUT);
  Serial.begin(115200);
  // Wait a bit for USB CDC to come up
  delay(200);
  Serial.println("PIN12_PROBE active");
#if PROBE_MODE == 0
  digitalWrite(PROBE_PIN, LOW);
  Serial.printf("Pin %d forced LOW\n", PROBE_PIN);
#elif PROBE_MODE == 1
  digitalWrite(PROBE_PIN, HIGH);
  Serial.printf("Pin %d forced HIGH\n", PROBE_PIN);
#endif
}

void loop() {
#if PROBE_MODE == 2
  digitalWrite(PROBE_PIN, HIGH);
  delay(PROBE_BLINK_MS);
  digitalWrite(PROBE_PIN, LOW);
  delay(PROBE_BLINK_MS);
#else
  delay(1000);
#endif
}

#endif // PIN12_PROBE

