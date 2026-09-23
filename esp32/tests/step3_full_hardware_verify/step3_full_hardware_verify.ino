/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║   ARK-5  Step 3 — Full Hardware Verify (Serial Monitor Only)    ║
 * ║   Tests: MPU6050 · HC-SR04 x3 · L298N Motors                   ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  WIRING QUICK-REF:                                              ║
 * ║  MPU6050 : SDA=21  SCL=22   (3.3V, GND)                        ║
 * ║  US FRONT: TRIG=18 ECHO=19  (5V from L298N)                    ║
 * ║  US LEFT : TRIG=5  ECHO=15  (5V from L298N)                    ║
 * ║  US RIGHT: TRIG=16 ECHO=4   (5V from L298N)                    ║
 * ║  MOTOR L : ENA=25  IN1=26  IN2=27  (L298N)                     ║
 * ║  MOTOR R : ENB=33  IN3=32  IN4=14  (L298N)                     ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  HOW TO USE:                                                    ║
 * ║  1. Flash this to ESP32                                         ║
 * ║  2. Open Serial Monitor @ 115200 baud                           ║
 * ║  3. Commands:                                                   ║
 * ║     'r' → Run all tests once (MPU + US + Motors)               ║
 * ║     'm' → Motor test only                                       ║
 * ║     auto → Sensor readings every 500ms continuously             ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Data structure for MPU6050
struct ImuData { float ax, ay, az, gx, gy, gz; };

// ── LEDC PWM Compatibility for ESP32 Core 2.x and Core 3.x ────────────
static inline void pwmSetup(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t res) {
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, freq, res);
#else
  ledcSetup(channel, freq, res);
  ledcAttachPin(pin, channel);
#endif
}

static inline void pwmWrite(uint8_t pin, uint8_t channel, uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pin, val);
#else
  ledcWrite(channel, val);
#endif
}

// ── Pin Definitions ──────────────────────────────────────────────────
#define PIN_SDA       21
#define PIN_SCL       22
#define MPU_ADDR      0x68

#define PIN_US_F_TRIG 18
#define PIN_US_F_ECHO 19
#define PIN_US_L_TRIG 5
#define PIN_US_L_ECHO 15
#define PIN_US_R_TRIG 16
#define PIN_US_R_ECHO 4
#define US_MAX_CM     400

#define PIN_ENA       25
#define PIN_IN1       26
#define PIN_IN2       27
#define PIN_ENB       33
#define PIN_IN3       32
#define PIN_IN4       14
#define TEST_PWM      150

#define PWM_CH_L      0
#define PWM_CH_R      1
#define PWM_FREQ      1000
#define PWM_RES       8

// ── Global State ─────────────────────────────────────────────────────
bool          mpuOK    = false;
float         yawDeg   = 0.0f;
float         yawRad   = 0.0f;
unsigned long lastMs   = 0;

// ══════════════════════════════════════════════════════════════════════
//  HELPERS
// ══════════════════════════════════════════════════════════════════════
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);
  delay(100);
  Wire.beginTransmission(MPU_ADDR);
  return (Wire.endTransmission() == 0);
}

ImuData mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);
  int16_t axr=(Wire.read()<<8)|Wire.read();
  int16_t ayr=(Wire.read()<<8)|Wire.read();
  int16_t azr=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  int16_t gxr=(Wire.read()<<8)|Wire.read();
  int16_t gyr=(Wire.read()<<8)|Wire.read();
  int16_t gzr=(Wire.read()<<8)|Wire.read();
  return {axr/16384.f,ayr/16384.f,azr/16384.f,gxr/131.f,gyr/131.f,gzr/131.f};
}

int readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 30000UL);
  if (us <= 0) return -1;
  int cm = (int)(us * 0.0343f / 2.0f);
  return (cm < 2 || cm > US_MAX_CM) ? -1 : cm;
}

void motorsStop() {
  pwmWrite(PIN_ENA, PWM_CH_L, 0); pwmWrite(PIN_ENB, PWM_CH_R, 0);
  digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,LOW);
  digitalWrite(PIN_IN3,LOW); digitalWrite(PIN_IN4,LOW);
}
void motorsForward(uint8_t spd) {
  digitalWrite(PIN_IN1,HIGH); digitalWrite(PIN_IN2,LOW);
  digitalWrite(PIN_IN3,HIGH); digitalWrite(PIN_IN4,LOW);
  pwmWrite(PIN_ENA, PWM_CH_L, spd); pwmWrite(PIN_ENB, PWM_CH_R, spd);
}
void motorsBackward(uint8_t spd) {
  digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,HIGH);
  digitalWrite(PIN_IN3,LOW); digitalWrite(PIN_IN4,HIGH);
  pwmWrite(PIN_ENA, PWM_CH_L, spd); pwmWrite(PIN_ENB, PWM_CH_R, spd);
}

// ══════════════════════════════════════════════════════════════════════
//  TEST ROUTINES
// ══════════════════════════════════════════════════════════════════════
void printDiv() { Serial.println(F("----------------------------------------------")); }

void testMPU() {
  printDiv();
  Serial.println(F("[ MPU6050 TEST ]"));
  if (!mpuOK) {
    Serial.println(F("  FAIL - MPU6050 not found at 0x68"));
    Serial.println(F("  Check: SDA=21, SCL=22, 3.3V, GND"));
    return;
  }
  ImuData d = mpuRead();
  float tg = sqrtf(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
  Serial.printf("  OK  - ax=%.2f ay=%.2f az=%.2f g  |g|=%.2f\n",d.ax,d.ay,d.az,tg);
  Serial.printf("        gx=%.1f gy=%.1f gz=%.1f deg/s\n",d.gx,d.gy,d.gz);
  Serial.printf("        Yaw = %.1f deg\n", yawDeg);
  if (tg<0.5f||tg>1.5f) Serial.println(F("  WARNING: |accel| abnormal - sensor fault?"));
}

void testUltrasonics() {
  printDiv();
  Serial.println(F("[ ULTRASONIC TEST (HC-SR04 x3) ]"));
  int f=readCm(PIN_US_F_TRIG,PIN_US_F_ECHO); delay(5);
  int l=readCm(PIN_US_L_TRIG,PIN_US_L_ECHO); delay(5);
  int r=readCm(PIN_US_R_TRIG,PIN_US_R_ECHO);
  Serial.print(F("  FRONT (18/19): "));
  (f<0)?Serial.println(F("FAIL - No echo (5V wiring check)")) :Serial.printf("%d cm OK\n",f);
  Serial.print(F("  LEFT  ( 5/15): "));
  (l<0)?Serial.println(F("FAIL - No echo (5V wiring check)")) :Serial.printf("%d cm OK\n",l);
  Serial.print(F("  RIGHT (16/ 4): "));
  (r<0)?Serial.println(F("FAIL - No echo (5V wiring check)")) :Serial.printf("%d cm OK\n",r);
}

void testMotors() {
  printDiv();
  Serial.println(F("[ MOTOR TEST (L298N) ]"));
  Serial.println(F("  WARNING: Motors chalenge - robot ko flat rakhein!"));
  Serial.println(F("  -- FORWARD 2s --"));
  motorsForward(TEST_PWM); delay(2000);
  Serial.println(F("  -- STOP 1s --"));
  motorsStop(); delay(1000);
  Serial.println(F("  -- BACKWARD 2s --"));
  motorsBackward(TEST_PWM); delay(2000);
  motorsStop();
  Serial.println(F("  Motor test done - dono wheels ghoomni chahiye thin"));
  Serial.println(F("  Agar koi wheel nahi ghomi: IN pins ya ENA/ENB check karein"));
}

void runAllTests() {
  Serial.println(F("\n=============================================="));
  Serial.println(F("       ARK-5  FULL HARDWARE TEST"));
  Serial.println(F("=============================================="));
  testMPU();
  testUltrasonics();
  testMotors();
  printDiv();
  Serial.println(F("All tests done! FAIL waley components fix karein."));
  printDiv();
}

// ══════════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println(F("\n=============================================="));
  Serial.println(F("  ARK-5  Step 3 - Full Hardware Verify"));
  Serial.println(F("  Commands: 'r'=all tests  'm'=motors only"));
  Serial.println(F("  Auto: sensor readings har 500ms"));
  Serial.println(F("==============================================\n"));

  mpuOK = mpuInit();
  Serial.print(F("MPU6050    ... "));
  Serial.println(mpuOK ? F("OK (found at 0x68)") : F("FAIL (SDA=21 SCL=22 check)"));

  pinMode(PIN_US_F_TRIG,OUTPUT); pinMode(PIN_US_F_ECHO,INPUT);
  pinMode(PIN_US_L_TRIG,OUTPUT); pinMode(PIN_US_L_ECHO,INPUT);
  pinMode(PIN_US_R_TRIG,OUTPUT); pinMode(PIN_US_R_ECHO,INPUT);
  Serial.println(F("Ultrasonic ... OK (pins set)"));

  pwmSetup(PIN_ENA, PWM_CH_L, PWM_FREQ, PWM_RES);
  pwmSetup(PIN_ENB, PWM_CH_R, PWM_FREQ, PWM_RES);
  pinMode(PIN_IN1,OUTPUT); pinMode(PIN_IN2,OUTPUT);
  pinMode(PIN_IN3,OUTPUT); pinMode(PIN_IN4,OUTPUT);
  motorsStop();
  Serial.println(F("Motors     ... OK (stopped)\n"));

  Serial.println(F("TIME(ms)  | F-cm  L-cm  R-cm | ax    ay    az   | Yaw(deg)"));
  Serial.println(F("----------|-------------------|------------------|---------"));
}

// ══════════════════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════════════════
void loop() {
  if (Serial.available()) {
    char cmd=(char)Serial.read();
    while(Serial.available()) Serial.read();
    if (cmd=='r'||cmd=='R') { runAllTests(); return; }
    if (cmd=='m'||cmd=='M') { testMotors();  return; }
  }

  unsigned long now = millis();
  if (now - lastMs < 500) return;
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  float ax=0,ay=0,az=0;
  if (mpuOK) {
    ImuData d=mpuRead();
    ax=d.ax; ay=d.ay; az=d.az;
    float gyroYaw = yawRad + d.gz * DEG_TO_RAD * dt;
    float accelYaw = atan2f(-d.ay, d.ax);
    yawRad = 0.98f*gyroYaw + 0.02f*accelYaw;
    yawDeg = yawRad * RAD_TO_DEG;
  }

  int f=readCm(PIN_US_F_TRIG,PIN_US_F_ECHO); delay(5);
  int l=readCm(PIN_US_L_TRIG,PIN_US_L_ECHO); delay(5);
  int r=readCm(PIN_US_R_TRIG,PIN_US_R_ECHO);

  char fs[6],ls[6],rs[6];
  if (f < 0) strcpy(fs, " --- "); else sprintf(fs, "%4d ", f);
  if (l < 0) strcpy(ls, " --- "); else sprintf(ls, "%4d ", l);
  if (r < 0) strcpy(rs, " --- "); else sprintf(rs, "%4d ", r);

  Serial.printf("%9lu | %s %s %s| %5.2f %5.2f %5.2f | %7.1f\n",
                now, fs, ls, rs, ax, ay, az, yawDeg);
}
