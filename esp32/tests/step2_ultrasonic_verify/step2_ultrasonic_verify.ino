/*
 * ARK-5 Step 2 — Dual HC-SR04 verify
 * FRONT: GPIO18/19 | LEFT: GPIO5/15 (D5/D15) | RIGHT: GPIO16/4
 * VCC = 5V from L298N (NOT ESP32 3.3V)
 */
#include <Arduino.h>

#define PIN_US_F_TRIG  18
#define PIN_US_F_ECHO  19
#define PIN_US_L_TRIG  5
#define PIN_US_L_ECHO  15
#define PIN_US_R_TRIG  16
#define PIN_US_R_ECHO  4
#define US_MAX_CM 400

static int readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 30000);
  if (us <= 0) return -1;
  int cm = (int)(us * 0.0343f / 2.0f);
  return (cm < 2 || cm > US_MAX_CM) ? -1 : cm;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_US_F_TRIG, OUTPUT);
  pinMode(PIN_US_L_TRIG, OUTPUT);
  pinMode(PIN_US_R_TRIG, OUTPUT);
  pinMode(PIN_US_F_ECHO, INPUT);
  pinMode(PIN_US_L_ECHO, INPUT);
  pinMode(PIN_US_R_ECHO, INPUT);
  Serial.println("\n=== ARK-5 Dual HC-SR04 Test ===");
  Serial.println("FRONT=18/19  LEFT=5/15 (D5/D15)  RIGHT=16/4");
  Serial.println("VCC = 5V from L298N\n");
}

void loop() {
  int f = readCm(PIN_US_F_TRIG, PIN_US_F_ECHO);
  delay(5);
  int l = readCm(PIN_US_L_TRIG, PIN_US_L_ECHO);
  delay(5);
  int r = readCm(PIN_US_R_TRIG, PIN_US_R_ECHO);
  Serial.printf("FRONT:%4d cm | LEFT:%4d cm | RIGHT:%4d cm\n", f, l, r);
  delay(400);
}
